#include "ui/ps2_app.h"
#include "ui/ps2_gl_gs_backend.h"
#include "ui/theme_settings.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl2.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <cstdlib>
#include <string>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#include <commdlg.h>
#endif

namespace ps2::ui {

namespace {

constexpr std::size_t kAudioChannels = 2u;
constexpr std::size_t kStutterHistoryFrames =
    static_cast<std::size_t>(Spu2::kSampleRate) * 400u / 1000u;
constexpr std::size_t kStutterQueueTargetFrames =
    static_cast<std::size_t>(Spu2::kSampleRate) * 80u / 1000u;

} // namespace

bool Ps2App::init() {
    SDL_SetMainReady();
    if (SDL_Init(
            SDL_INIT_VIDEO |
            SDL_INIT_GAMECONTROLLER |
            SDL_INIT_AUDIO) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    struct GlContextAttempt {
        int major;
        int minor;
        int profile;
        const char* glsl;
        bool use_opengl2;
    };

    const GlContextAttempt attempts[] = {
        {4, 3, SDL_GL_CONTEXT_PROFILE_CORE, "#version 430", false},
        {3, 3, SDL_GL_CONTEXT_PROFILE_CORE, "#version 330", false},
        {3, 2, SDL_GL_CONTEXT_PROFILE_CORE, "#version 150", false},
        {2, 1, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY, "#version 120", true},
    };

    for (const auto& attempt : attempts) {
        SDL_GL_ResetAttributes();
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, attempt.major);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, attempt.minor);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, attempt.profile);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

        window_ = SDL_CreateWindow(
            "VibeStation - PS2 Experimental",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            1280,
            800,
            SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);

        if (!window_) {
            continue;
        }

        gl_context_ = SDL_GL_CreateContext(window_);
        if (!gl_context_) {
            SDL_DestroyWindow(window_);
            window_ = nullptr;
            continue;
        }

        SDL_GL_MakeCurrent(window_, gl_context_);
        SDL_GL_SetSwapInterval(1);
        imgui_glsl_version_ = attempt.glsl;
        use_imgui_opengl2_backend_ = attempt.use_opengl2;
        gl_major_ = attempt.major;
        gl_minor_ = attempt.minor;
        break;
    }

    if (!window_ || !gl_context_) {
        std::fprintf(stderr, "Unable to create a compatible OpenGL context.\n");
        shutdown();
        return false;
    }

    SDL_AudioSpec desired_audio{};
    desired_audio.freq = static_cast<int>(Spu2::kSampleRate);
    desired_audio.format = AUDIO_S16SYS;
    desired_audio.channels = 2;
    desired_audio.samples = 1024;
    desired_audio.callback = nullptr;

    SDL_AudioSpec obtained_audio{};
    audio_device_ = SDL_OpenAudioDevice(
        nullptr,
        0,
        &desired_audio,
        &obtained_audio,
        0);
    if (audio_device_ != 0 &&
        (obtained_audio.freq != desired_audio.freq ||
         obtained_audio.format != desired_audio.format ||
         obtained_audio.channels != desired_audio.channels)) {
        SDL_CloseAudioDevice(audio_device_);
        audio_device_ = 0;
    }
    if (audio_device_ != 0) {
        SDL_PauseAudioDevice(audio_device_, 0);
        reset_audio_stutter();
        audio_stutter_thread_stop_.store(false, std::memory_order_release);
        audio_stutter_thread_ =
            std::thread(&Ps2App::audio_stutter_thread_main, this);
    } else {
        std::fprintf(
            stderr,
            "PS2 audio output unavailable: %s\n",
            SDL_GetError());
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = "vibestation_ps2_imgui.ini";

    ImGui::StyleColorsDark();
    ui_theme::ensure_theme_settings_initialized();
    ui_theme::apply_theme_style(ImGui::GetStyle());
    ui_theme::register_theme_settings_handler();

    if (!ImGui_ImplSDL2_InitForOpenGL(window_, gl_context_)) {
        std::fprintf(stderr, "ImGui SDL backend initialization failed.\n");
        shutdown();
        return false;
    }

    if (use_imgui_opengl2_backend_) {
        if (!ImGui_ImplOpenGL2_Init()) {
            std::fprintf(stderr, "ImGui OpenGL2 backend initialization failed.\n");
            shutdown();
            return false;
        }
    } else {
        if (!ImGui_ImplOpenGL3_Init(imgui_glsl_version_)) {
            std::fprintf(stderr, "ImGui OpenGL3 backend initialization failed.\n");
            shutdown();
            return false;
        }
    }

    if (gl_major_ > 4 ||
        (gl_major_ == 4 && gl_minor_ >= 3)) {
        SDL_GL_SetAttribute(
            SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 1);
        SDL_GLContext gpu_context =
            SDL_GL_CreateContext(window_);
        SDL_GL_SetAttribute(
            SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 0);
        SDL_GL_MakeCurrent(window_, gl_context_);

        if (gpu_context != nullptr) {
            gpu_gs_backend_ =
                std::make_unique<Ps2GlGsBackend>(
                    window_, gpu_context);
            if (gpu_gs_backend_->available()) {
                system_.gs_core().set_gpu_backend(
                    gpu_gs_backend_.get());
            } else {
                gpu_gs_backend_.reset();
            }
        }
    }

    system_.gs_core().set_async_rasterization(true);
    reset_core();
    status_message_ = "PS2 experimental core ready";
    return true;
}

int Ps2App::run() {
    bool quit = false;
    int result = 0;

    while (!quit) {
        process_events(quit);
        update_pad_input();
        update_emulation();
        update_audio();

        if (!visible_capture_path_.empty() &&
            (!emulation_running_ || system_.halted())) {
            std::fprintf(
                stderr,
                "UI capture stopped before reaching a visible BIOS frame: %s\n",
                status_message_.c_str());
            result = 4;
            break;
        }

        if (use_imgui_opengl2_backend_) {
            ImGui_ImplOpenGL2_NewFrame();
        } else {
            ImGui_ImplOpenGL3_NewFrame();
        }
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        update_display_texture();
        render_ui();

        ImGui::Render();

        int display_width = 0;
        int display_height = 0;
        SDL_GL_GetDrawableSize(window_, &display_width, &display_height);
        glViewport(0, 0, display_width, display_height);

        const ImVec4 background = ui_theme::g_theme_settings.background;
        glClearColor(background.x, background.y, background.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        if (use_imgui_opengl2_backend_) {
            ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
        } else {
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }

        if (!visible_capture_path_.empty() &&
            system_.gs_display().has_visible_pixels() &&
            system_.ee().state().instructions_executed >=
                visible_capture_minimum_ee_) {
            if (!write_window_ppm(
                    visible_capture_path_, display_width, display_height)) {
                result = 3;
            }
            visible_capture_path_.clear();
            quit = true;
        }

        SDL_GL_SwapWindow(window_);
    }

    return result;
}

bool Ps2App::launch_bios(const std::string& path) {
    return load_bios_from_path(path) && start_bios();
}

void Ps2App::set_ee_jit_enabled(bool enabled) {
    system_.ee().set_jit_enabled(enabled);
}

void Ps2App::capture_visible_window(
    const std::string& path,
    unsigned long long minimum_ee_instructions) {
    visible_capture_path_ = path;
    visible_capture_minimum_ee_ = minimum_ee_instructions;
}

bool Ps2App::write_window_ppm(
    const std::string& path,
    int width,
    int height) {
    if (width <= 0 || height <= 0) {
        std::fprintf(stderr, "UI capture failed: invalid drawable size.\n");
        return false;
    }

    std::vector<std::uint8_t> rgb(
        static_cast<std::size_t>(width) *
        static_cast<std::size_t>(height) * 3u);

    while (glGetError() != GL_NO_ERROR) {
    }
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
    if (glGetError() != GL_NO_ERROR) {
        std::fprintf(stderr, "UI capture failed: glReadPixels error.\n");
        return false;
    }

    std::FILE* output = std::fopen(path.c_str(), "wb");
    if (!output) {
        std::fprintf(stderr, "UI capture failed: cannot open %s.\n", path.c_str());
        return false;
    }

    std::fprintf(output, "P6\n%d %d\n255\n", width, height);
    const std::size_t row_bytes = static_cast<std::size_t>(width) * 3u;
    bool ok = true;
    for (int y = height - 1; y >= 0; --y) {
        const std::uint8_t* row =
            rgb.data() + static_cast<std::size_t>(y) * row_bytes;
        if (std::fwrite(row, 1, row_bytes, output) != row_bytes) {
            ok = false;
            break;
        }
    }
    if (std::fclose(output) != 0) {
        ok = false;
    }

    if (ok) {
        std::fprintf(
            stdout,
            "UI_VISIBLE_CAPTURE=%s WIDTH=%d HEIGHT=%d EE=%llu\n",
            path.c_str(),
            width,
            height,
            static_cast<unsigned long long>(
                system_.ee().state().instructions_executed));
    } else {
        std::fprintf(stderr, "UI capture failed while writing %s.\n", path.c_str());
    }
    return ok;
}

void Ps2App::shutdown() {
    audio_stutter_thread_stop_.store(true, std::memory_order_release);
    if (audio_stutter_thread_.joinable()) {
        audio_stutter_thread_.join();
    }
    if (audio_device_ != 0) {
        SDL_ClearQueuedAudio(audio_device_);
        SDL_CloseAudioDevice(audio_device_);
        audio_device_ = 0;
    }
    if (controller_ != nullptr) {
        SDL_GameControllerClose(controller_);
        controller_ = nullptr;
    }

    system_.gs_core().set_gpu_backend(nullptr);
    gpu_gs_backend_.reset();

    if (display_texture_ != 0 && gl_context_ != nullptr) {
        glDeleteTextures(1, &display_texture_);
        display_texture_ = 0;
        display_texture_width_ = 0;
        display_texture_height_ = 0;
        display_texture_generation_ = ~0ull;
    }

    if (ImGui::GetCurrentContext() != nullptr) {
        if (use_imgui_opengl2_backend_) {
            ImGui_ImplOpenGL2_Shutdown();
        } else {
            ImGui_ImplOpenGL3_Shutdown();
        }
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();
    }

    if (gl_context_) {
        SDL_GL_DeleteContext(gl_context_);
        gl_context_ = nullptr;
    }

    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }

    SDL_Quit();
}


void Ps2App::update_display_texture() {
    const auto& display = system_.gs_display();
    if (!display.valid() || display.rgba8().empty()) {
        return;
    }
    if (display_texture_generation_ == display.generation()) {
        return;
    }

    if (display_texture_ == 0) {
        glGenTextures(1, &display_texture_);
        glBindTexture(GL_TEXTURE_2D, display_texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    } else {
        glBindTexture(GL_TEXTURE_2D, display_texture_);
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    if (display_texture_width_ != display.width() ||
        display_texture_height_ != display.height()) {
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGBA,
            static_cast<GLsizei>(display.width()),
            static_cast<GLsizei>(display.height()),
            0,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            display.rgba8().data());
        display_texture_width_ = display.width();
        display_texture_height_ = display.height();
    } else {
        glTexSubImage2D(
            GL_TEXTURE_2D,
            0,
            0,
            0,
            static_cast<GLsizei>(display.width()),
            static_cast<GLsizei>(display.height()),
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            display.rgba8().data());
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    display_texture_generation_ = display.generation();
}

void Ps2App::update_pad_input() {
    if (controller_ != nullptr &&
        SDL_GameControllerGetAttached(controller_) == SDL_FALSE) {
        SDL_GameControllerClose(controller_);
        controller_ = nullptr;
    }

    if (controller_ == nullptr) {
        const int joystick_count = SDL_NumJoysticks();
        for (int i = 0; i < joystick_count; ++i) {
            if (SDL_IsGameController(i) == SDL_TRUE) {
                controller_ = SDL_GameControllerOpen(i);
                if (controller_ != nullptr) break;
            }
        }
    }

    Sio2Pad::State state{};
    const Uint8* keys = SDL_GetKeyboardState(nullptr);

    const auto set_button =
        [&](Sio2Pad::Button button, bool pressed) {
            if (!pressed) return;
            state.buttons &= static_cast<u16>(
                ~(1u << static_cast<u8>(button)));
        };

    // Compact keyboard fallback: arrows=d-pad, Z/X/A/S=face buttons,
    // Enter/Backspace=Start/Select, Q/E=L1/R1, W/R=L2/R2.
    set_button(Sio2Pad::Button::Up, keys[SDL_SCANCODE_UP] != 0);
    set_button(Sio2Pad::Button::Down, keys[SDL_SCANCODE_DOWN] != 0);
    set_button(Sio2Pad::Button::Left, keys[SDL_SCANCODE_LEFT] != 0);
    set_button(Sio2Pad::Button::Right, keys[SDL_SCANCODE_RIGHT] != 0);
    set_button(Sio2Pad::Button::Cross, keys[SDL_SCANCODE_Z] != 0);
    set_button(Sio2Pad::Button::Circle, keys[SDL_SCANCODE_X] != 0);
    set_button(Sio2Pad::Button::Square, keys[SDL_SCANCODE_A] != 0);
    set_button(Sio2Pad::Button::Triangle, keys[SDL_SCANCODE_S] != 0);
    set_button(Sio2Pad::Button::Start, keys[SDL_SCANCODE_RETURN] != 0);
    set_button(Sio2Pad::Button::Select, keys[SDL_SCANCODE_BACKSPACE] != 0);
    set_button(Sio2Pad::Button::L1, keys[SDL_SCANCODE_Q] != 0);
    set_button(Sio2Pad::Button::R1, keys[SDL_SCANCODE_E] != 0);
    set_button(Sio2Pad::Button::L2, keys[SDL_SCANCODE_W] != 0);
    set_button(Sio2Pad::Button::R2, keys[SDL_SCANCODE_R] != 0);

    if (controller_ != nullptr) {
        const auto button = [&](SDL_GameControllerButton id) {
            return SDL_GameControllerGetButton(controller_, id) != 0;
        };
        set_button(Sio2Pad::Button::Cross,
            button(SDL_CONTROLLER_BUTTON_A));
        set_button(Sio2Pad::Button::Circle,
            button(SDL_CONTROLLER_BUTTON_B));
        set_button(Sio2Pad::Button::Square,
            button(SDL_CONTROLLER_BUTTON_X));
        set_button(Sio2Pad::Button::Triangle,
            button(SDL_CONTROLLER_BUTTON_Y));
        set_button(Sio2Pad::Button::Select,
            button(SDL_CONTROLLER_BUTTON_BACK));
        set_button(Sio2Pad::Button::Start,
            button(SDL_CONTROLLER_BUTTON_START));
        set_button(Sio2Pad::Button::L3,
            button(SDL_CONTROLLER_BUTTON_LEFTSTICK));
        set_button(Sio2Pad::Button::R3,
            button(SDL_CONTROLLER_BUTTON_RIGHTSTICK));
        set_button(Sio2Pad::Button::L1,
            button(SDL_CONTROLLER_BUTTON_LEFTSHOULDER));
        set_button(Sio2Pad::Button::R1,
            button(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER));
        set_button(Sio2Pad::Button::Up,
            button(SDL_CONTROLLER_BUTTON_DPAD_UP));
        set_button(Sio2Pad::Button::Down,
            button(SDL_CONTROLLER_BUTTON_DPAD_DOWN));
        set_button(Sio2Pad::Button::Left,
            button(SDL_CONTROLLER_BUTTON_DPAD_LEFT));
        set_button(Sio2Pad::Button::Right,
            button(SDL_CONTROLLER_BUTTON_DPAD_RIGHT));

        const auto axis_to_u8 = [&](SDL_GameControllerAxis id) {
            int value = SDL_GameControllerGetAxis(controller_, id);
            if (value > -4096 && value < 4096) value = 0;
            const int scaled =
                ((value + 32768) * 255) / 65535;
            return static_cast<u8>(
                std::clamp(scaled, 0, 255));
        };

        state.lx = axis_to_u8(SDL_CONTROLLER_AXIS_LEFTX);
        state.ly = axis_to_u8(SDL_CONTROLLER_AXIS_LEFTY);
        state.rx = axis_to_u8(SDL_CONTROLLER_AXIS_RIGHTX);
        state.ry = axis_to_u8(SDL_CONTROLLER_AXIS_RIGHTY);

        set_button(
            Sio2Pad::Button::L2,
            SDL_GameControllerGetAxis(
                controller_, SDL_CONTROLLER_AXIS_TRIGGERLEFT) > 8192);
        set_button(
            Sio2Pad::Button::R2,
            SDL_GameControllerGetAxis(
                controller_, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) > 8192);
    }

    system_.pad().set_state(state);
}

void Ps2App::reset_audio_stutter() {
    {
        std::lock_guard<std::mutex> lock(audio_history_mutex_);
        // The tape is always exactly 400 ms long. Before real SPU2 samples
        // have filled it, the unused portion is intentional silence.
        audio_history_.assign(
            kStutterHistoryFrames * kAudioChannels, 0);
        audio_history_write_frame_ = 0u;
        audio_history_play_frame_ = 0u;
    }
    lag_stutter_active_.store(false, std::memory_order_release);
}

void Ps2App::remember_audio_history(
    const s16* samples,
    std::size_t frames) {
    if (samples == nullptr || frames == 0u) return;

    std::lock_guard<std::mutex> lock(audio_history_mutex_);
    const std::size_t capacity_frames = kStutterHistoryFrames;
    const std::size_t capacity_samples =
        capacity_frames * kAudioChannels;
    if (audio_history_.size() != capacity_samples) {
        audio_history_.assign(capacity_samples, 0);
        audio_history_write_frame_ = 0u;
        audio_history_play_frame_ = 0u;
    }

    // Literal FIFO rolling tape: producer writes at the oldest slot and then
    // advances. Once all 400 ms slots contain real audio, each new frame
    // overwrites exactly the oldest frame. The newest frame is never discarded.
    for (std::size_t frame = 0u; frame < frames; ++frame) {
        const std::size_t dst =
            audio_history_write_frame_ * kAudioChannels;
        const std::size_t src = frame * kAudioChannels;
        audio_history_[dst + 0u] = samples[src + 0u];
        audio_history_[dst + 1u] = samples[src + 1u];
        audio_history_write_frame_ =
            (audio_history_write_frame_ + 1u) % capacity_frames;
    }
}

void Ps2App::audio_stutter_thread_main() {
    std::vector<s16> refill(
        kStutterQueueTargetFrames * kAudioChannels);

    while (!audio_stutter_thread_stop_.load(
        std::memory_order_acquire)) {
        if (audio_device_ == 0u ||
            !lag_stutter_enabled_.load(std::memory_order_acquire)) {
            lag_stutter_active_.store(false, std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        const std::size_t queued_frames =
            SDL_GetQueuedAudioSize(audio_device_) /
            (kAudioChannels * sizeof(s16));
        if (queued_frames >= kStutterQueueTargetFrames) {
            lag_stutter_active_.store(true, std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        const std::size_t needed_frames =
            kStutterQueueTargetFrames - queued_frames;
        {
            std::lock_guard<std::mutex> lock(audio_history_mutex_);
            if (audio_history_.size() !=
                kStutterHistoryFrames * kAudioChannels) {
                audio_history_.assign(
                    kStutterHistoryFrames * kAudioChannels, 0);
                audio_history_write_frame_ = 0u;
                audio_history_play_frame_ = 0u;
            }

            // Consumer independently walks the fixed 400 ms tape forever.
            // If emulation takes 200 ms to produce its next chunk, this host
            // thread continues reading the existing tape instead of allowing
            // SDL to underrun. Early in boot, unfilled tape slots are silence,
            // so the loop length is still 400 ms rather than a tiny buzzing
            // fragment that grows unpredictably.
            for (std::size_t frame = 0u;
                 frame < needed_frames;
                 ++frame) {
                const std::size_t src =
                    audio_history_play_frame_ * kAudioChannels;
                const std::size_t dst = frame * kAudioChannels;
                refill[dst + 0u] = audio_history_[src + 0u];
                refill[dst + 1u] = audio_history_[src + 1u];
                audio_history_play_frame_ =
                    (audio_history_play_frame_ + 1u) %
                    kStutterHistoryFrames;
            }
        }

        if (SDL_QueueAudio(
                audio_device_,
                refill.data(),
                static_cast<Uint32>(
                    needed_frames *
                    kAudioChannels *
                    sizeof(s16))) != 0) {
            SDL_ClearQueuedAudio(audio_device_);
            lag_stutter_active_.store(false, std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
            continue;
        }

        lag_stutter_active_.store(true, std::memory_order_release);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    lag_stutter_active_.store(false, std::memory_order_release);
}

void Ps2App::update_audio() {
    const std::size_t available = system_.spu2().queued_frames();
    auto pcm = available != 0u
        ? system_.spu2().take_samples(available)
        : std::vector<s16>{};

    if (audio_device_ == 0u || pcm.empty()) return;

    const std::size_t frames = pcm.size() / kAudioChannels;

    if (lag_stutter_enabled_.load(std::memory_order_acquire)) {
        // In rolling-stutter mode fresh SPU2 data never goes directly to SDL.
        // It only updates the 400 ms tape. The independent host feeder thread
        // is the sole playback source, so slow EE/GS frames cannot cause
        // sporadic queue starvation between update_audio() calls.
        remember_audio_history(pcm.data(), frames);
        return;
    }

    // Normal non-stutter path: retain the old low-latency queue behavior.
    constexpr Uint32 kLatencyResetBytes =
        Spu2::kSampleRate * 2u * sizeof(s16) / 8u;
    if (SDL_GetQueuedAudioSize(audio_device_) > kLatencyResetBytes) {
        SDL_ClearQueuedAudio(audio_device_);
    }

    constexpr std::size_t kMaxSubmitFrames = 4096u;
    const std::size_t submit_frames =
        std::min(frames, kMaxSubmitFrames);
    const std::size_t first_frame = frames - submit_frames;
    const s16* data =
        pcm.data() + first_frame * kAudioChannels;

    if (SDL_QueueAudio(
            audio_device_,
            data,
            static_cast<Uint32>(
                submit_frames *
                kAudioChannels *
                sizeof(s16))) != 0) {
        SDL_ClearQueuedAudio(audio_device_);
    }
}

void Ps2App::process_events(bool& quit) {
    SDL_Event event{};
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);

        if (event.type == SDL_QUIT) {
            quit = true;
            continue;
        }

        if (event.type == SDL_WINDOWEVENT &&
            event.window.event == SDL_WINDOWEVENT_CLOSE &&
            event.window.windowID == SDL_GetWindowID(window_)) {
            quit = true;
            continue;
        }

        if (event.type == SDL_KEYDOWN && event.key.repeat == 0) {
            const bool ctrl = (event.key.keysym.mod & KMOD_CTRL) != 0;
            if (ctrl && event.key.keysym.sym == SDLK_b) {
                const std::string path = open_bios_dialog();
                if (!path.empty()) {
                    load_bios_from_path(path);
                }
            } else if (event.key.keysym.sym == SDLK_F5) {
                if (system_.bios().loaded()) {
                    start_bios();
                } else {
                    reset_core();
                }
            } else if (event.key.keysym.sym == SDLK_F6) {
                emulation_running_ = false;
                status_message_ = "PS2 execution paused";
            } else if (event.key.keysym.sym == SDLK_F7) {
                if (!emulation_running_) {
                    step_iop_once();
                }
            } else if (event.key.keysym.sym == SDLK_F8) {
                if (!emulation_running_) {
                    step_ee_once();
                }
            } else if (event.key.keysym.sym == SDLK_F9) {
                show_ee_debug_ = !show_ee_debug_;
            } else if (event.key.keysym.sym == SDLK_F10) {
                show_iop_debug_ = !show_iop_debug_;
            } else if (event.key.keysym.sym == SDLK_F11) {
                show_gs_debug_ = !show_gs_debug_;
            }
        }
    }
}

void Ps2App::render_ui() {
    menu_bar();

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoBackground;

    ImGui::Begin("PS2DockSpace", nullptr, flags);
    ImGui::PopStyleVar(3);
    panel_main();
    ImGui::End();

    if (show_system_) {
        panel_system();
    }
    if (show_ee_debug_) {
        panel_ee_debug();
    }
    if (show_iop_debug_) {
        panel_iop_debug();
    }
    if (show_gs_debug_) {
        panel_gs_debug();
    }
    if (show_scheduler_) {
        panel_scheduler();
    }
    if (show_settings_) {
        panel_settings();
    }
    if (show_about_) {
        panel_about();
    }
}

void Ps2App::menu_bar() {
    if (!ImGui::BeginMainMenuBar()) {
        return;
    }

    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Load BIOS...", "Ctrl+B")) {
            const std::string path = open_bios_dialog();
            if (!path.empty()) {
                load_bios_from_path(path);
            }
        }
        ImGui::MenuItem("Load ELF...", nullptr, false, false);
        ImGui::Separator();
        if (ImGui::MenuItem("Exit", "Alt+F4")) {
            SDL_Event event{};
            event.type = SDL_QUIT;
            SDL_PushEvent(&event);
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Emulation")) {
        if (ImGui::MenuItem("Start BIOS", "F5", false,
                            system_.bios().loaded())) {
            start_bios();
        }
        if (ImGui::MenuItem("Reset Core")) {
            reset_core();
        }
        ImGui::Separator();

        const bool can_execute =
            system_.bios_started() && !system_.halted();

        if (ImGui::MenuItem(
                "Run", nullptr, false,
                can_execute && !emulation_running_)) {
            emulation_running_ = true;
            status_message_ = "PS2 execution running";
        }
        if (ImGui::MenuItem(
                "Pause", "F6", false, emulation_running_)) {
            emulation_running_ = false;
            status_message_ = "PS2 execution paused";
        }
        if (ImGui::MenuItem(
                "Step IOP Instruction", "F7", false,
                can_execute && !emulation_running_ &&
                !system_.iop_halted())) {
            step_iop_once();
        }
        if (ImGui::MenuItem(
                "Step EE Instruction", "F8", false,
                can_execute && !emulation_running_)) {
            step_ee_once();
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
        ImGui::MenuItem("System", nullptr, &show_system_);
        ImGui::MenuItem("EE Debug", "F9", &show_ee_debug_);
        ImGui::MenuItem("IOP Debug", "F10", &show_iop_debug_);
        ImGui::MenuItem("GS Debug", "F11", &show_gs_debug_);
        ImGui::MenuItem("Scheduler", nullptr, &show_scheduler_);
        ImGui::Separator();
        ImGui::MenuItem("Settings", "Ctrl+,", &show_settings_);
        ImGui::MenuItem("About", nullptr, &show_about_);
        ImGui::EndMenu();
    }

    const float status_width =
        ImGui::CalcTextSize(status_message_.c_str()).x + 20.0f;
    const float desired_x = ImGui::GetWindowWidth() - status_width;
    if (desired_x > ImGui::GetCursorPosX() + 20.0f) {
        ImGui::SameLine(desired_x);
        ImGui::TextColored(
            ImVec4(0.55f, 0.45f, 0.90f, 1.0f),
            "%s",
            status_message_.c_str());
    }

    ImGui::EndMainMenuBar();
}

void Ps2App::panel_main() {
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const ImVec2 start = ImGui::GetCursorPos();
    const float center_x = start.x + (available.x * 0.5f);
    const float center_y = start.y + (available.y * 0.5f);

    const auto& display = system_.gs_display();
    const auto show_boot_progress = [&]() {
        if (!emulation_running_ || display.has_visible_pixels()) {
            return;
        }
        ImGui::SetCursorPos(ImVec2(start.x + 16.0f, start.y + 16.0f));
        ImGui::TextColored(
            ImVec4(0.70f, 0.80f, 1.0f, 1.0f),
            "Booting BIOS - waiting for first GS image");
        ImGui::Text(
            "EE instructions: %.1f million",
            static_cast<double>(system_.ee().state().instructions_executed) /
                1'000'000.0);
        if (ee_instructions_per_second_ > 0.0) {
            ImGui::Text(
                "Host throughput: %.1f million EE instructions/s",
                ee_instructions_per_second_ / 1'000'000.0);
        }
        ImGui::TextDisabled("This is not the PS2 hardware clock.");
    };
    if (display.valid() && display_texture_ != 0 &&
        display.width() != 0 && display.height() != 0) {
        const float aspect =
            static_cast<float>(display.width()) /
            static_cast<float>(display.height());
        ImVec2 image_size = available;
        if (image_size.y > 0.0f && image_size.x / image_size.y > aspect) {
            image_size.x = image_size.y * aspect;
        } else if (aspect > 0.0f) {
            image_size.y = image_size.x / aspect;
        }
        image_size.x = std::max(1.0f, image_size.x);
        image_size.y = std::max(1.0f, image_size.y);

        ImGui::SetCursorPos(ImVec2(
            start.x + (available.x - image_size.x) * 0.5f,
            start.y + (available.y - image_size.y) * 0.5f));
        ImGui::Image(
            (ImTextureID)(intptr_t)display_texture_,
            image_size,
            ImVec2(0.0f, 0.0f),
            ImVec2(1.0f, 1.0f));

        ImGui::SetCursorPos(ImVec2(start.x + 10.0f, start.y + 10.0f));
        ImGui::TextDisabled(
            "GS circuit %u  %ux%u  PSM 0x%02X",
            display.circuit(),
            display.width(),
            display.height(),
            display.psm());
        show_boot_progress();
        return;
    }

    const ImVec4 title_color =
        ui_theme::current_startup_title_color(ui_theme::g_theme_settings);
    const ImVec4 text_color =
        ui_theme::current_startup_text_color(ui_theme::g_theme_settings);
    const ImVec4 secondary =
        ui_theme::theme_lerp(
            text_color, ui_theme::g_theme_settings.background, 0.18f);

    const char* title = "VibeStation";
    ImGui::PushStyleColor(ImGuiCol_Text, title_color);
    ImGui::SetWindowFontScale(2.0f);
    const ImVec2 title_size = ImGui::CalcTextSize(title);
    ImGui::SetCursorPos(
        ImVec2(center_x - title_size.x * 0.5f, center_y - 130.0f));
    ImGui::TextUnformatted(title);
    ImGui::SetWindowFontScale(1.0f);
    ImGui::PopStyleColor();

    const char* subtitle = "PlayStation 2 Experimental Core";
    const ImVec2 subtitle_size = ImGui::CalcTextSize(subtitle);
    ImGui::SetCursorPos(
        ImVec2(center_x - subtitle_size.x * 0.5f, center_y - 77.0f));
    ImGui::TextColored(text_color, "%s", subtitle);

    const char* phase = "BIOS bootstrap: VIF1 + VU1 + GS display";
    const ImVec2 phase_size = ImGui::CalcTextSize(phase);
    ImGui::SetCursorPos(
        ImVec2(center_x - phase_size.x * 0.5f, center_y - 49.0f));
    ImGui::TextColored(secondary, "%s", phase);

    ImGui::SetCursorPos(ImVec2(center_x - 205.0f, center_y + 1.0f));
    if (!system_.bios().loaded()) {
        if (ImGui::Button("Load BIOS", ImVec2(125.0f, 0.0f))) {
            const std::string path = open_bios_dialog();
            if (!path.empty()) {
                load_bios_from_path(path);
            }
        }
    } else {
        if (ImGui::Button(
                system_.bios_started() ? "Restart BIOS" : "Start BIOS",
                ImVec2(125.0f, 0.0f))) {
            start_bios();
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("EE Debug", ImVec2(125.0f, 0.0f))) {
        show_ee_debug_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("IOP Debug", ImVec2(125.0f, 0.0f))) {
        show_iop_debug_ = true;
    }

    ImGui::SetCursorPos(ImVec2(center_x - 280.0f, center_y + 57.0f));
    ImGui::BeginChild("PS2CoreSummary", ImVec2(560.0f, 220.0f), true);
    ImGui::TextColored(title_color, "Experimental core status");
    ImGui::Separator();

    const auto& state = system_.ee().state();
    const auto& iop_state = system_.iop().state();

    ImGui::Text("BIOS");
    ImGui::SameLine(190.0f);
    if (system_.bios().loaded()) {
        if (system_.bios().romver().empty()) {
            ImGui::Text("Loaded");
        } else {
            ImGui::Text(
                "Loaded (ROMVER %s)", system_.bios().romver().c_str());
        }
    } else {
        ImGui::TextDisabled("not loaded");
    }

    ImGui::Text("EE RAM");
    ImGui::SameLine(190.0f);
    ImGui::Text("%zu MiB", EeRam::kSize / (1024u * 1024u));

    ImGui::Text("EE PC");
    ImGui::SameLine(190.0f);
    ImGui::Text("0x%08X", state.pc);

    ImGui::Text("Reset opcode");
    ImGui::SameLine(190.0f);
    if (system_.bios_started()) {
        ImGui::Text("0x%08X", system_.reset_instruction());
    } else {
        ImGui::TextDisabled("not fetched");
    }

    ImGui::Text("BIOS execution");
    ImGui::SameLine(190.0f);
    if (!system_.bios_started()) {
        ImGui::TextDisabled("not started");
    } else if (system_.halted()) {
        ImGui::TextColored(
            ImVec4(0.90f, 0.45f, 0.45f, 1.0f), "halted");
    } else if (emulation_running_) {
        ImGui::TextColored(
            ImVec4(0.45f, 0.85f, 0.45f, 1.0f), "running");
    } else {
        ImGui::Text("paused");
    }

    ImGui::Text("EE backend");
    ImGui::SameLine(190.0f);
    ImGui::TextUnformatted(
        system_.ee().jit_enabled() ? "experimental x64 JIT" : "interpreter");

    ImGui::Text("EE instructions");
    ImGui::SameLine(190.0f);
    ImGui::Text(
        "%llu",
        static_cast<unsigned long long>(state.instructions_executed));

    ImGui::Text("IOP PC");
    ImGui::SameLine(190.0f);
    ImGui::Text("0x%08X", iop_state.pc);

    ImGui::Text("IOP instructions");
    ImGui::SameLine(190.0f);
    ImGui::Text(
        "%llu",
        static_cast<unsigned long long>(iop_state.instructions_executed));


    ImGui::Text("GIF qwords");
    ImGui::SameLine(190.0f);
    ImGui::Text(
        "%llu",
        static_cast<unsigned long long>(
            system_.gs_core().submitted_gif_qwords()));

    ImGui::Text("GS primitives");
    ImGui::SameLine(190.0f);
    ImGui::Text(
        "%llu",
        static_cast<unsigned long long>(
            system_.gs_core().submitted_primitives()));

    const auto& vu_stats = system_.vu1().stats();
    ImGui::Text("VU1 / XGKICK");
    ImGui::SameLine(190.0f);
    ImGui::Text(
        "%llu instr / %llu kicks",
        static_cast<unsigned long long>(vu_stats.instructions),
        static_cast<unsigned long long>(vu_stats.xgkicks));

    ImGui::Text("IOP state");
    ImGui::SameLine(190.0f);
    if (system_.iop_halted()) {
        ImGui::TextColored(
            ImVec4(0.90f, 0.65f, 0.30f, 1.0f),
            "halted (EE/GS continuing)");
    } else {
        ImGui::Text("running");
    }
    ImGui::EndChild();
    show_boot_progress();
}

void Ps2App::panel_system() {
    ImGui::SetNextWindowSize(
        ImVec2(650.0f, 560.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("PS2 System", &show_system_)) {
        ImGui::End();
        return;
    }

    const auto& state = system_.ee().state();
    const auto& iop_state = system_.iop().state();

    ImGui::Text("VibeStation PS2 Lab");
    ImGui::Separator();

    ImGui::Text(
        "BIOS: %s", system_.bios().loaded() ? "loaded" : "not loaded");
    if (system_.bios().loaded()) {
        ImGui::TextWrapped("Path: %s", system_.bios().path().c_str());
        ImGui::Text(
            "Size: %zu MiB", Bios::kSize / (1024u * 1024u));
        ImGui::Text(
            "ROMVER: %s",
            system_.bios().romver().empty()
                ? "(not detected)"
                : system_.bios().romver().c_str());
        ImGui::Text(
            "ROM mapping: 0x%08X / 0x%08X / 0x%08X",
            Bios::kPhysicalBase, Bios::kCachedBase, Bios::kUncachedBase);
    }

    ImGui::Spacing();
    ImGui::InputText(
        "BIOS path", bios_path_input_.data(), bios_path_input_.size());

    if (ImGui::Button("Browse...")) {
        const std::string path = open_bios_dialog();
        if (!path.empty()) {
            load_bios_from_path(path);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Load Path")) {
        load_bios_from_path(bios_path_input_.data());
    }
    ImGui::SameLine();

    if (!system_.bios().loaded()) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button(
            system_.bios_started() ? "Restart BIOS" : "Start BIOS")) {
        start_bios();
    }
    if (!system_.bios().loaded()) {
        ImGui::EndDisabled();
    }

    ImGui::Separator();
    ImGui::Text("EE PC: 0x%08X", state.pc);
    ImGui::Text("EE next PC: 0x%08X", state.next_pc);
    ImGui::Text("IOP PC: 0x%08X", iop_state.pc);
    ImGui::Text("IOP next PC: 0x%08X", iop_state.next_pc);
    ImGui::Text(
        "Scheduler tick: %llu",
        static_cast<unsigned long long>(system_.scheduler().now()));
    ImGui::Text(
        "EE instructions: %llu",
        static_cast<unsigned long long>(state.instructions_executed));
    ImGui::Text(
        "IOP instructions: %llu",
        static_cast<unsigned long long>(iop_state.instructions_executed));
    ImGui::Text(
        "Execution state: %s",
        system_.halted()
            ? "halted"
            : (emulation_running_ ? "running" : "paused"));

    if (system_.bios_started()) {
        ImGui::Text(
            "EE reset instruction: 0x%08X", system_.reset_instruction());
        ImGui::Text(
            "IOP reset instruction: 0x%08X",
            system_.iop_reset_instruction());
        if (system_.halted()) {
            ImGui::TextWrapped(
                "Halt: %s",
                system_.halt_reason().c_str());
        }
        if (system_.iop_halted()) {
            ImGui::TextColored(
                ImVec4(0.90f, 0.65f, 0.30f, 1.0f),
                "IOP halted; EE/GS bootstrap is still running");
            ImGui::TextWrapped(
                "IOP: %s",
                system_.iop().halt_reason().c_str());
        }
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Subsystem readiness");
    ImGui::BulletText("BIOS ROM mapping: available");
    ImGui::BulletText("EE reset startup: available");
    ImGui::BulletText("EE interpreter/COP0 subset: running");
    ImGui::BulletText("EE scratchpad: available");
    ImGui::BulletText("Early EE SIO/SBUS/RDRAM/DMAC registers: available");
    ImGui::BulletText("IOP RAM: 2 MiB shared with EE");
    ImGui::BulletText("IOP R3000A interpreter/COP0: running");
    ImGui::BulletText("IOP INTC I_STAT/I_MASK/I_CTRL + IRQ2: available");
    ImGui::BulletText("EE/IOP clock interleave: 8:1 startup model");
    ImGui::BulletText("IOP hardware register window: partial");
    ImGui::BulletText("SIF/SBUS bridge: partial");
    ImGui::BulletText("CDVD bootstrap registers/SCMDs: partial");
    ImGui::BulletText("GS privileged registers: partial");
    ImGui::BulletText("Scheduler: advancing with EE execution");
    ImGui::BulletText("IOP timers/INTC/DMAC + full CDVD/SPU2: pending");

    ImGui::End();
}

void Ps2App::panel_ee_debug() {
    ImGui::SetNextWindowSize(
        ImVec2(760.0f, 650.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("EE Debug", &show_ee_debug_)) {
        ImGui::End();
        return;
    }

    const auto& state = system_.ee().state();

    ImGui::Text("PC: 0x%08X", state.pc);
    ImGui::SameLine();
    ImGui::Text("Next PC: 0x%08X", state.next_pc);
    ImGui::Text(
        "HI: 0x%016llX   LO: 0x%016llX",
        static_cast<unsigned long long>(state.hi),
        static_cast<unsigned long long>(state.lo));

    u32 current_instruction = 0;
    const bool can_fetch_current =
        system_.bios_started() &&
        system_.bus().read32(state.pc, current_instruction);

    ImGui::Text(
        "Instructions: %llu",
        static_cast<unsigned long long>(state.instructions_executed));
    ImGui::Text(
        "Last: PC 0x%08X  opcode 0x%08X",
        state.last_pc,
        state.last_instruction);
    if (can_fetch_current) {
        ImGui::Text("Current opcode: 0x%08X", current_instruction);
    }
    ImGui::Text(
        "COP0 PRId: 0x%08X  Status: 0x%08X  Count: 0x%08X",
        state.cop0[15], state.cop0[12], state.cop0[9]);

    if (system_.ee().halted()) {
        ImGui::TextColored(
            ImVec4(0.90f, 0.45f, 0.45f, 1.0f),
            "HALTED");
        ImGui::TextWrapped("%s", system_.ee().halt_reason().c_str());
    } else if (system_.bios_started()) {
        if (emulation_running_) {
            if (ImGui::Button("Pause EE (F6)")) {
                emulation_running_ = false;
                status_message_ = "EE execution paused";
            }
        } else if (ImGui::Button("Step EE (F8)")) {
            step_ee_once();
        }
    }

    ImGui::Separator();

    const ImGuiTableFlags flags =
        ImGuiTableFlags_Borders |
        ImGuiTableFlags_RowBg |
        ImGuiTableFlags_ScrollY |
        ImGuiTableFlags_SizingStretchProp;

    if (ImGui::BeginTable(
            "EERegisters", 3, flags, ImVec2(0.0f, 490.0f))) {
        ImGui::TableSetupColumn(
            "Register", ImGuiTableColumnFlags_WidthFixed, 90.0f);
        ImGui::TableSetupColumn("High 64");
        ImGui::TableSetupColumn("Low 64");
        ImGui::TableHeadersRow();

        for (int i = 0; i < 32; ++i) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("r%d", i);

            ImGui::TableSetColumnIndex(1);
            ImGui::Text(
                "0x%016llX",
                static_cast<unsigned long long>(state.gpr[i].hi));

            ImGui::TableSetColumnIndex(2);
            ImGui::Text(
                "0x%016llX",
                static_cast<unsigned long long>(state.gpr[i].lo));
        }

        ImGui::EndTable();
    }

    ImGui::End();
}


void Ps2App::panel_iop_debug() {
    ImGui::SetNextWindowSize(
        ImVec2(680.0f, 620.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("IOP Debug", &show_iop_debug_)) {
        ImGui::End();
        return;
    }

    const auto& state = system_.iop().state();

    ImGui::Text("PC: 0x%08X", state.pc);
    ImGui::SameLine();
    ImGui::Text("Next PC: 0x%08X", state.next_pc);
    ImGui::Text(
        "HI: 0x%08X   LO: 0x%08X",
        state.hi,
        state.lo);
    ImGui::Text(
        "Instructions: %llu",
        static_cast<unsigned long long>(state.instructions_executed));

    u32 current_instruction = 0;
    const bool can_fetch_current =
        system_.bios_started() &&
        system_.iop_bus().read32(state.pc, current_instruction);

    ImGui::Text(
        "Last: PC 0x%08X  opcode 0x%08X",
        state.last_pc,
        state.last_instruction);
    if (can_fetch_current) {
        ImGui::Text("Current opcode: 0x%08X", current_instruction);
    }
    ImGui::Text(
        "COP0 PRId: 0x%08X  Status: 0x%08X  Cause: 0x%08X  EPC: 0x%08X",
        state.cop0[15],
        state.cop0[12],
        state.cop0[13],
        state.cop0[14]);

    if (system_.iop().halted()) {
        ImGui::TextColored(
            ImVec4(0.90f, 0.45f, 0.45f, 1.0f),
            "HALTED");
        ImGui::TextWrapped("%s", system_.iop().halt_reason().c_str());
    } else if (system_.bios_started() && !emulation_running_) {
        if (ImGui::Button("Step IOP (F7)")) {
            step_iop_once();
        }
    }

    ImGui::Separator();

    const ImGuiTableFlags flags =
        ImGuiTableFlags_Borders |
        ImGuiTableFlags_RowBg |
        ImGuiTableFlags_ScrollY |
        ImGuiTableFlags_SizingStretchProp;

    if (ImGui::BeginTable(
            "IOPRegisters", 2, flags, ImVec2(0.0f, 445.0f))) {
        ImGui::TableSetupColumn(
            "Register", ImGuiTableColumnFlags_WidthFixed, 90.0f);
        ImGui::TableSetupColumn("Value");
        ImGui::TableHeadersRow();

        for (int i = 0; i < 32; ++i) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("r%d", i);

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("0x%08X", state.gpr[i]);
        }

        ImGui::EndTable();
    }

    ImGui::End();
}


void Ps2App::panel_gs_debug() {
    ImGui::SetNextWindowSize(
        ImVec2(610.0f, 560.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("GS Debug", &show_gs_debug_)) {
        ImGui::End();
        return;
    }

    const auto& gs = system_.gs_core();
    const auto& stats = gs.stats();

    ImGui::Text("GIF packet active: %s", gs.packet_active() ? "yes" : "no");
    ImGui::Text("Current PRIM: %u", gs.current_prim());
    ImGui::Text(
        "Host->local: %s   PSM: 0x%02X   pixels left: %u",
        gs.transfer_active() ? "active" : "idle",
        gs.transfer_psm(),
        gs.transfer_pixels_remaining());
    ImGui::Separator();

    if (ImGui::BeginTable("GSStats", 2,
                          ImGuiTableFlags_Borders |
                          ImGuiTableFlags_RowBg |
                          ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Counter");
        ImGui::TableSetupColumn("Value");
        ImGui::TableHeadersRow();

        const auto row = [](const char* name, unsigned long long value) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(name);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%llu", value);
        };

        row("GIF tags", static_cast<unsigned long long>(stats.gif_tags));
        row("GIF qwords", static_cast<unsigned long long>(stats.gif_qwords));
        row("EOP packets", static_cast<unsigned long long>(stats.eop_packets));
        row("Register writes", static_cast<unsigned long long>(stats.register_writes));
        row("Packed writes", static_cast<unsigned long long>(stats.packed_writes));
        row("REGLIST writes", static_cast<unsigned long long>(stats.reglist_writes));
        row("IMAGE qwords", static_cast<unsigned long long>(stats.image_qwords));
        row("IMAGE bytes", static_cast<unsigned long long>(stats.image_bytes));
        row("Host->local transfers", static_cast<unsigned long long>(stats.host_to_local_transfers));
        row("Host->local pixels", static_cast<unsigned long long>(stats.host_to_local_pixels));
        row("Unsupported transfers", static_cast<unsigned long long>(stats.unsupported_transfers));
        row("Unsupported packed", static_cast<unsigned long long>(stats.unsupported_packed));
        row("Vertex kicks", static_cast<unsigned long long>(stats.vertices));
        row("Primitive kicks", static_cast<unsigned long long>(stats.primitives));
        row("Raster draws", static_cast<unsigned long long>(stats.raster_draws));
        row("Raster pixels", static_cast<unsigned long long>(stats.raster_pixels));
        row("Textured raster draws", static_cast<unsigned long long>(stats.textured_raster_draws));
        row("Texture samples", static_cast<unsigned long long>(stats.texture_samples));
        row("Skipped raster draws", static_cast<unsigned long long>(stats.skipped_raster_draws));
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Key GS registers");

    struct RegisterRow {
        const char* name;
        u32 address;
    };
    static constexpr RegisterRow regs[] = {
        {"PRIM", 0x00},
        {"RGBAQ", 0x01},
        {"XYZ2", 0x05},
        {"SCISSOR_1", 0x40},
        {"TEST_1", 0x47},
        {"FRAME_1", 0x4C},
        {"FRAME_2", 0x4D},
        {"BITBLTBUF", 0x50},
        {"TRXPOS", 0x51},
        {"TRXREG", 0x52},
        {"TRXDIR", 0x53},
    };

    if (ImGui::BeginTable("GSRegisters", 3,
                          ImGuiTableFlags_Borders |
                          ImGuiTableFlags_RowBg |
                          ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Register");
        ImGui::TableSetupColumn("Addr", ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn("Value");
        ImGui::TableHeadersRow();

        for (const auto& reg : regs) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(reg.name);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("0x%02X", reg.address);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text(
                "0x%016llX",
                static_cast<unsigned long long>(gs.register_value(reg.address)));
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

void Ps2App::panel_scheduler() {
    ImGui::SetNextWindowSize(
        ImVec2(430.0f, 230.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("PS2 Scheduler", &show_scheduler_)) {
        ImGui::End();
        return;
    }

    ImGui::Text(
        "Current tick: %llu",
        static_cast<unsigned long long>(system_.scheduler().now()));
    ImGui::Text(
        "Queue empty: %s", system_.scheduler().empty() ? "yes" : "no");

    ImGui::Separator();
    ImGui::TextWrapped(
        "The scheduler is already part of the PS2 core so asynchronous "
        "hardware can be added without falling back to scanline-sized "
        "catch-up loops.");

    ImGui::Spacing();
    ImGui::TextDisabled(
        "Event inspection will expand when EE timers, DMAC, GIF, VIF and "
        "GS begin scheduling real work.");

    ImGui::End();
}

void Ps2App::panel_settings() {
    ImGui::SetNextWindowSize(
        ImVec2(470.0f, 270.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Settings", &show_settings_)) {
        ImGui::End();
        return;
    }

    ImGui::Text("Appearance");
    ImGui::Separator();

    const int preset_count = ui_theme::theme_preset_count();
    const int selected = std::clamp(
        ui_theme::g_selected_theme_preset_index,
        0,
        std::max(0, preset_count - 1));

    const char* preview =
        preset_count > 0
            ? ui_theme::theme_preset_by_index(selected).label
            : "None";

    if (ImGui::BeginCombo("Theme Preset", preview)) {
        for (int i = 0; i < preset_count; ++i) {
            const bool is_selected =
                i == ui_theme::g_selected_theme_preset_index;
            if (ImGui::Selectable(
                    ui_theme::theme_preset_by_index(i).label,
                    is_selected)) {
                ui_theme::g_selected_theme_preset_index = i;
                ui_theme::apply_theme_preset_by_index(i);
                ui_theme::apply_theme_style(ImGui::GetStyle());
                ui_theme::mark_theme_settings_dirty();
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("PS2 devices");
    ImGui::Text(
        "Controller: %s",
        controller_ != nullptr
            ? SDL_GameControllerName(controller_)
            : "keyboard fallback");
    ImGui::Text(
        "Audio: %s",
        audio_device_ != 0
            ? "48 kHz stereo active"
            : "unavailable");
    bool lag_stutter_enabled =
        lag_stutter_enabled_.load(std::memory_order_acquire);
    if (ImGui::Checkbox(
            "Lag Stutter Effect",
            &lag_stutter_enabled)) {
        lag_stutter_enabled_.store(
            lag_stutter_enabled,
            std::memory_order_release);
        SDL_ClearQueuedAudio(audio_device_);
        if (lag_stutter_enabled) {
            std::lock_guard<std::mutex> lock(audio_history_mutex_);
            // The write position is the oldest slot on a full rolling tape,
            // so entering here starts with the least-recent audio first.
            audio_history_play_frame_ = audio_history_write_frame_;
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled(
        lag_stutter_active_.load(std::memory_order_acquire)
            ? "(rolling 400 ms)"
            : "(Source-style)");
    ImGui::TextDisabled(
        "Plays one fixed 400 ms rolling tape on a host feeder thread; "
        "new SPU2 frames overwrite the oldest tape frames first.");
    ImGui::TextDisabled(
        "Keyboard: arrows D-pad, Z/X/A/S face, Enter/Backspace Start/Select, "
        "Q/E L1/R1, W/R L2/R2.");

    ImGui::Spacing();
    ImGui::TextDisabled(
        "PS2 UI settings are stored separately in "
        "vibestation_ps2_imgui.ini.");
    ImGui::TextDisabled(
        "The normal PS1 VibeStation UI configuration is not modified.");

    ImGui::End();
}

void Ps2App::panel_about() {
    ImGui::SetNextWindowSize(
        ImVec2(470.0f, 280.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("About VibeStation PS2 Lab", &show_about_)) {
        ImGui::End();
        return;
    }

    ImGui::SetWindowFontScale(1.35f);
    ImGui::TextColored(
        ui_theme::current_startup_title_color(
            ui_theme::g_theme_settings),
        "VibeStation");
    ImGui::SetWindowFontScale(1.0f);

    ImGui::Text("PlayStation 2 Experimental Core");
    ImGui::Separator();
    ImGui::TextWrapped(
        "This build is an isolated PS2 research core. It mirrors the "
        "VibeStation UI style without linking the PS1 System, GPU, SPU, "
        "renderer, or runtime classes.");
    ImGui::Spacing();
    ImGui::TextWrapped(
        "BIOS images are not distributed with VibeStation. Load a BIOS "
        "dumped from hardware you own.");
    ImGui::Spacing();
    ImGui::TextDisabled(
        "Current milestone: execute the retail BIOS with both the EE and "
        "IOP alive, share the 2 MiB IOP RAM window, and advance the two "
        "processors with the PS2 startup 8:1 clock relationship.");

    ImGui::End();
}

std::string Ps2App::open_bios_dialog() {
#ifdef _WIN32
    std::array<char, 1024> path{};

    OPENFILENAMEA dialog{};
    dialog.lStructSize = sizeof(dialog);
    dialog.lpstrFile = path.data();
    dialog.nMaxFile = static_cast<DWORD>(path.size());
    dialog.lpstrFilter =
        "PS2 BIOS Images (*.bin)\0*.bin\0All Files (*.*)\0*.*\0";
    dialog.nFilterIndex = 1;
    dialog.lpstrTitle = "Select PlayStation 2 BIOS";
    dialog.Flags =
        OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;

    if (GetOpenFileNameA(&dialog)) {
        return path.data();
    }
    return {};
#else
    status_message_ =
        "Native BIOS picker is currently Windows-only; paste a path in "
        "View > System.";
    show_system_ = true;
    return {};
#endif
}

bool Ps2App::load_bios_from_path(const std::string& path) {
    if (path.empty()) {
        status_message_ = "BIOS path is empty.";
        return false;
    }

    std::string error;
    if (!system_.load_bios(path, error)) {
        status_message_ = "BIOS load failed: " + error;
        return false;
    }

    emulation_running_ = false;

    std::snprintf(
        bios_path_input_.data(),
        bios_path_input_.size(),
        "%s",
        path.c_str());

    const std::string file_name =
        std::filesystem::path(path).filename().string();

    status_message_ = "BIOS loaded: " + file_name;
    if (!system_.bios().romver().empty()) {
        status_message_ +=
            " (ROMVER " + system_.bios().romver() + ")";
    }
    return true;
}

bool Ps2App::start_bios() {
    std::string error;
    if (!system_.boot_bios(error)) {
        status_message_ = "BIOS startup failed: " + error;
        return false;
    }

    emulation_running_ = true;
    reset_audio_stutter();
    if (audio_device_ != 0) SDL_ClearQueuedAudio(audio_device_);
    speed_sample_time_ = std::chrono::steady_clock::now();
    speed_sample_instructions_ = system_.ee().state().instructions_executed;
    ee_instructions_per_second_ = 0.0;

    char message[160]{};
    std::snprintf(
        message,
        sizeof(message),
        "EE+IOP BIOS execution started at 0x%08X (reset opcode 0x%08X)",
        Bios::kResetVector,
        system_.reset_instruction());
    status_message_ = message;
    return true;
}

bool Ps2App::step_ee_once() {
    if (!system_.bios_started() || system_.halted()) {
        return false;
    }

    std::string error;
    if (!system_.step_ee(error)) {
        emulation_running_ = false;
        status_message_ = "Execution halted: " + error;
        return false;
    }

    char message[128]{};
    std::snprintf(
        message,
        sizeof(message),
        "EE step -> PC 0x%08X (%llu instructions)",
        system_.ee().state().pc,
        static_cast<unsigned long long>(
            system_.ee().state().instructions_executed));
    status_message_ = message;
    return true;
}


bool Ps2App::step_iop_once() {
    if (!system_.bios_started() || system_.halted()) {
        return false;
    }

    std::string error;
    if (!system_.step_iop(error)) {
        emulation_running_ = false;
        status_message_ = "IOP halted: " + error;
        return false;
    }

    char message[128]{};
    std::snprintf(
        message,
        sizeof(message),
        "IOP step -> PC 0x%08X (%llu instructions)",
        system_.iop().state().pc,
        static_cast<unsigned long long>(
            system_.iop().state().instructions_executed));
    status_message_ = message;
    return true;
}

void Ps2App::update_emulation() {
    if (!emulation_running_ ||
        !system_.bios_started() ||
        system_.halted()) {
        if (bootstrap_swap_interval_disabled_ &&
            SDL_GL_SetSwapInterval(1) == 0) {
            bootstrap_swap_interval_disabled_ = false;
        }
        return;
    }

    // During blank-screen bootstrap, run longer slices and avoid waiting for
    // VSync on frames that cannot yet show BIOS pixels. Restore normal frame
    // pacing as soon as the composed display becomes visible.
    constexpr u64 kNormalChunkInstructions = 8192;
    constexpr u64 kBootstrapChunkInstructions = 1'000'000;
    constexpr u64 kNormalMaxInstructionsPerFrame = 500000;
    constexpr u64 kBootstrapMaxInstructionsPerFrame = 250'000'000;
    constexpr auto kNormalCpuTimeSlice = std::chrono::milliseconds(14);
    constexpr auto kBootstrapCpuTimeSlice =
        std::chrono::seconds(6);

    // PCRTC can become valid while it still scans an untouched black buffer.
    // Keep the larger bootstrap slice until the composed display actually
    // contains visible RGB data; validity alone is not a first-frame signal.
    const bool bootstrap_turbo =
        !system_.gs_display().has_visible_pixels();
    if (bootstrap_turbo != bootstrap_swap_interval_disabled_ &&
        SDL_GL_SetSwapInterval(bootstrap_turbo ? 0 : 1) == 0) {
        bootstrap_swap_interval_disabled_ = bootstrap_turbo;
    }
    const u64 max_instructions =
        bootstrap_turbo
            ? kBootstrapMaxInstructionsPerFrame
            : kNormalMaxInstructionsPerFrame;
    const auto cpu_time_slice =
        bootstrap_turbo
            ? kBootstrapCpuTimeSlice
            : kNormalCpuTimeSlice;

    const auto deadline =
        std::chrono::steady_clock::now() + cpu_time_slice;
    u64 executed = 0;
    std::string error;

    while (executed < max_instructions &&
           !system_.halted()) {
        const u64 budget =
            std::min<u64>(
                bootstrap_turbo ? kBootstrapChunkInstructions :
                    kNormalChunkInstructions,
                max_instructions - executed);
        const u64 ran = system_.run_ee(budget, error);
        executed += ran;

        if (bootstrap_turbo) {
            system_.refresh_display();
            if (system_.gs_display().has_visible_pixels()) break;
        }

        if (ran == 0 || !error.empty() ||
            std::chrono::steady_clock::now() >= deadline) {
            break;
        }
    }

    // Blank-screen bootstrap samples the PCRTC after each large chunk above.
    // Once pixels are visible, VBlank already updates the display. Repeating
    // a full VRAM scan every host UI frame needlessly drains the GS worker.

    const auto sample_time = std::chrono::steady_clock::now();
    const auto sample_seconds =
        std::chrono::duration<double>(sample_time - speed_sample_time_).count();
    if (sample_seconds >= 0.5) {
        const u64 instructions = system_.ee().state().instructions_executed;
        ee_instructions_per_second_ =
            static_cast<double>(instructions - speed_sample_instructions_) /
            sample_seconds;
        speed_sample_instructions_ = instructions;
        speed_sample_time_ = sample_time;
    }

    if (system_.halted()) {
        emulation_running_ = false;
        status_message_ = "Execution halted: " + system_.halt_reason();
    } else if (!error.empty()) {
        emulation_running_ = false;
        status_message_ = "Execution stopped: " + error;
    } else if (system_.iop_halted()) {
        status_message_ =
            "IOP halted; EE/GS continuing for BIOS bootstrap";
    }
}

void Ps2App::reset_core() {
    emulation_running_ = false;
    ee_instructions_per_second_ = 0.0;
    reset_audio_stutter();
    if (audio_device_ != 0) SDL_ClearQueuedAudio(audio_device_);
    system_.reset(0);
    status_message_ =
        system_.bios().loaded()
            ? "PS2 core reset; BIOS remains loaded"
            : "PS2 core reset";
}

} // namespace ps2::ui
