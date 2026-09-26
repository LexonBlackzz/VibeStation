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

Ps2App::~Ps2App() = default;

namespace {

constexpr std::size_t kAudioChannels = 2u;
constexpr std::size_t kStutterHistoryFrames =
    static_cast<std::size_t>(Spu2::kSampleRate) * 400u / 1000u;
constexpr std::size_t kStutterQueueTargetFrames =
    static_cast<std::size_t>(Spu2::kSampleRate) * 80u / 1000u;

const char* ee_major_opcode_name(u32 opcode) {
    switch (opcode & 63u) {
    case 0x00u: return "SPECIAL";
    case 0x01u: return "REGIMM";
    case 0x02u: return "J";
    case 0x03u: return "JAL";
    case 0x04u: return "BEQ";
    case 0x05u: return "BNE";
    case 0x06u: return "BLEZ";
    case 0x07u: return "BGTZ";
    case 0x08u: return "ADDI";
    case 0x09u: return "ADDIU";
    case 0x0Au: return "SLTI";
    case 0x0Bu: return "SLTIU";
    case 0x0Cu: return "ANDI";
    case 0x0Du: return "ORI";
    case 0x0Eu: return "XORI";
    case 0x0Fu: return "LUI";
    case 0x10u: return "COP0";
    case 0x11u: return "COP1";
    case 0x12u: return "COP2";
    case 0x14u: return "BEQL";
    case 0x15u: return "BNEL";
    case 0x16u: return "BLEZL";
    case 0x17u: return "BGTZL";
    case 0x18u: return "DADDI";
    case 0x19u: return "DADDIU";
    case 0x1Au: return "LDL";
    case 0x1Bu: return "LDR";
    case 0x1Cu: return "MMI";
    case 0x1Eu: return "LQ";
    case 0x1Fu: return "SQ";
    case 0x20u: return "LB";
    case 0x21u: return "LH";
    case 0x22u: return "LWL";
    case 0x23u: return "LW";
    case 0x24u: return "LBU";
    case 0x25u: return "LHU";
    case 0x26u: return "LWR";
    case 0x27u: return "LWU";
    case 0x28u: return "SB";
    case 0x29u: return "SH";
    case 0x2Au: return "SWL";
    case 0x2Bu: return "SW";
    case 0x2Cu: return "SDL";
    case 0x2Du: return "SDR";
    case 0x2Eu: return "SWR";
    case 0x2Fu: return "CACHE";
    case 0x30u: return "LL";
    case 0x31u: return "LWC1";
    case 0x33u: return "PREF";
    case 0x34u: return "LLD";
    case 0x36u: return "LQC2";
    case 0x37u: return "LD";
    case 0x38u: return "SC";
    case 0x39u: return "SWC1";
    case 0x3Cu: return "SCD";
    case 0x3Eu: return "SQC2";
    case 0x3Fu: return "SD";
    default: return "OP";
    }
}

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
    if (show_profiler_) {
        panel_profiler();
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
        ImGui::MenuItem("Profiler", nullptr, &show_profiler_);
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
        if (emulation_running_ &&
            guest_fields_per_second_ > 0.0) {
            ImGui::Text(
                "%.1f FPS  |  %.0f%% speed  |  %.1f MIPS",
                guest_frames_per_second_,
                emulation_speed_percent_,
                ee_instructions_per_second_ / 1'000'000.0);
        }
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

    ImGui::Separator();
    ImGui::TextUnformatted("Rolling profiler");
    ImGui::Text(
        "Native EE: %.1f MIPS   %.1f%% of retired EE",
        profile_native_mips_,
        profile_native_coverage_percent_);
    ImGui::Text(
        "Native blocks: %.0f/s   %.1f instr/block",
        profile_native_blocks_per_second_,
        profile_average_native_block_);
    ImGui::Text(
        "Native exits: %.0f guard/s   %.0f code-store/s   %.1f cache flush/s",
        profile_guard_bailouts_per_second_,
        profile_code_store_exits_per_second_,
        profile_cache_flushes_per_second_);
    ImGui::Text(
        "IOP: %.1f MIPS   VU1: %.1f MIPS",
        profile_iop_mips_,
        profile_vu1_mips_);
    ImGui::Text(
        "Host run thread: quiet EE %.1f%%   IOP catch-up %.1f%%   slow path %.1f%%   other %.1f%%",
        profile_host_ee_percent_,
        profile_host_iop_percent_,
        profile_host_slow_percent_,
        profile_host_other_percent_);
    ImGui::TextDisabled(
        "EE coverage = native JIT-retired instructions / all retired EE instructions.");

    std::array<std::pair<double, u32>, 64> fallback_rates{};
    for (u32 opcode = 0; opcode < 64u; ++opcode) {
        fallback_rates[opcode] = {
            profile_fallbacks_per_second_[opcode], opcode};
    }
    std::sort(
        fallback_rates.begin(),
        fallback_rates.end(),
        [](const auto& a, const auto& b) {
            return a.first > b.first;
        });
    bool any_fallback = false;
    for (std::size_t i = 0; i < 6u; ++i) {
        if (fallback_rates[i].first <= 0.0) break;
        if (!any_fallback) {
            ImGui::TextUnformatted("Top native fallback opcodes:");
            any_fallback = true;
        }
        ImGui::BulletText(
            "%s (0x%02X): %.0f exits/s",
            ee_major_opcode_name(fallback_rates[i].second),
            fallback_rates[i].second,
            fallback_rates[i].first);
    }
    if (!any_fallback && system_.ee().jit_enabled()) {
        ImGui::TextDisabled("Top native fallback opcodes: none in this sample");
    }

    ImGui::Separator();
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
    ImGui::BulletText("IOP R3000A interpreter + native x64 tier: running");
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


void Ps2App::panel_profiler() {
    ImGui::SetNextWindowSize(
        ImVec2(690.0f, 500.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("PS2 Performance Profiler", &show_profiler_)) {
        ImGui::End();
        return;
    }

    ImGui::Text(
        "Guest: %.1f FPS   %.1f%% speed   EE %.1f MIPS",
        guest_frames_per_second_,
        emulation_speed_percent_,
        ee_instructions_per_second_ / 1'000'000.0);
    ImGui::Text(
        "Backend: %s",
        system_.ee().jit_enabled()
            ? "experimental x64 JIT"
            : "interpreter");
    ImGui::TextDisabled(
        system_.profile_timing_enabled()
            ? "Profiler host timing ON: diagnostic mode; benchmark numbers are perturbed."
            : "Benchmark mode: host timing OFF; cheap execution counters remain active.");
    ImGui::Separator();

    ImGui::Text(
        "Native EE: %.1f MIPS   %.1f%% coverage",
        profile_native_mips_,
        profile_native_coverage_percent_);
    ImGui::Text(
        "Native blocks: %.0f/s   average %.1f instr/block",
        profile_native_blocks_per_second_,
        profile_average_native_block_);
    ImGui::Text(
        "Native exits: %.0f guard/s   %.0f code-store/s   %.1f cache flush/s",
        profile_guard_bailouts_per_second_,
        profile_code_store_exits_per_second_,
        profile_cache_flushes_per_second_);

    const auto percentile_floor =
        [](const std::array<u64, 16>& histogram, double fraction) -> u64 {
            u64 total = 0u;
            for (const u64 count : histogram) total += count;
            if (total == 0u) return 0u;
            const u64 target = std::max<u64>(
                1u, static_cast<u64>(
                    static_cast<double>(total) * fraction + 0.999999));
            u64 cumulative = 0u;
            for (u32 bucket = 0u; bucket < histogram.size(); ++bucket) {
                cumulative += histogram[bucket];
                if (cumulative >= target) {
                    return u64{1} << bucket;
                }
            }
            return u64{1} << (histogram.size() - 1u);
        };

    const auto& ee_jit = system_.ee().jit();
    const u64 ee_entry_attempts =
        ee_jit.native_entry_attempt_count();
    const u64 ee_entry_successes =
        ee_jit.native_entry_success_count();
    const double ee_entry_success_percent =
        ee_entry_attempts != 0u
            ? static_cast<double>(ee_entry_successes) * 100.0 /
                static_cast<double>(ee_entry_attempts)
            : 0.0;
    const double ee_average_residency =
        ee_entry_successes != 0u
            ? static_cast<double>(
                  ee_jit.native_residency_instruction_count()) /
                static_cast<double>(ee_entry_successes)
            : 0.0;
    const auto& ee_residency =
        ee_jit.native_residency_histogram();

    ImGui::Spacing();
    ImGui::TextUnformatted("EE native-entry diagnosis");
    ImGui::Text(
        "JIT entries: %llu attempts   %llu success (%.1f%%)   %llu compile failures",
        static_cast<unsigned long long>(ee_entry_attempts),
        static_cast<unsigned long long>(ee_entry_successes),
        ee_entry_success_percent,
        static_cast<unsigned long long>(
            ee_jit.block_compile_failure_count()));
    ImGui::Text(
        "Direct-RAM system entry: %llu attempts   %llu success   %llu failed",
        static_cast<unsigned long long>(
            system_.direct_native_entry_attempts()),
        static_cast<unsigned long long>(
            system_.direct_native_entry_successes()),
        static_cast<unsigned long long>(
            system_.direct_native_entry_failures()));
    ImGui::Text(
        "Residency: avg %.1f instr   p50~%llu   p95~%llu   max %llu",
        ee_average_residency,
        static_cast<unsigned long long>(
            percentile_floor(ee_residency, 0.50)),
        static_cast<unsigned long long>(
            percentile_floor(ee_residency, 0.95)),
        static_cast<unsigned long long>(
            ee_jit.native_residency_max()));
    ImGui::TextDisabled(
        "p50/p95 are low-overhead log2 histogram estimates; max is exact.");

    const auto& ee_rejects = system_.quiet_ee_rejects();
    static constexpr std::array<const char*, 9> kEeRejectNames = {
        "budget<2", "SIF completion", "VU running", "GS IRQ",
        "DMA/SIF service", "pending EE IRQ", "video edge",
        "scheduler due", "deadline<2"};
    if (ImGui::TreeNode("System refusals before JIT entry")) {
        for (u32 i = 0u; i < ee_rejects.size(); ++i) {
            ImGui::BulletText(
                "%s: %llu",
                kEeRejectNames[i],
                static_cast<unsigned long long>(ee_rejects[i]));
        }
        ImGui::TreePop();
    }

    std::array<std::pair<u64, u32>, 64> compile_stops{};
    const auto& compile_stop_counts = ee_jit.compile_stop_opcodes();
    const auto& delay_stop_counts = ee_jit.delay_slot_stop_opcodes();
    for (u32 opcode = 0u; opcode < 64u; ++opcode) {
        compile_stops[opcode] = {compile_stop_counts[opcode], opcode};
    }
    std::sort(
        compile_stops.begin(),
        compile_stops.end(),
        [](const auto& a, const auto& b) {
            return a.first > b.first;
        });
    if (ImGui::TreeNode("Actual JIT compile stoppers")) {
        bool any = false;
        for (std::size_t i = 0u; i < 10u; ++i) {
            const auto [count, opcode] = compile_stops[i];
            if (count == 0u) break;
            any = true;
            ImGui::BulletText(
                "%s (0x%02X): %llu stops (%llu in delay slots)",
                ee_major_opcode_name(opcode),
                opcode,
                static_cast<unsigned long long>(count),
                static_cast<unsigned long long>(
                    delay_stop_counts[opcode]));
        }
        if (!any) {
            ImGui::TextDisabled("No compile stoppers recorded yet.");
        }
        ImGui::TreePop();
    }

    ImGui::Text(
        "IOP: %.1f MIPS   native %.1f MIPS (%.1f%% raw / %.1f%% active)",
        profile_iop_mips_,
        profile_iop_native_mips_,
        profile_iop_native_coverage_percent_,
        profile_iop_active_native_coverage_percent_);
    ImGui::Text(
        "IOP execution split: %.1f MIPS native   %.1f idle-skip   %.1f slow path",
        profile_iop_native_mips_,
        profile_iop_idle_mips_,
        profile_iop_slow_mips_);
    ImGui::Text(
        "IOP native: %.0f blocks/s   %.1f instr/block   %.0f chains/s",
        profile_iop_native_blocks_per_second_,
        profile_iop_average_native_block_,
        profile_iop_native_chains_per_second_);
    ImGui::Text(
        "IOP native exits: %.0f guard/s   %.0f code-store/s   VU1 %.1f MIPS",
        profile_iop_guard_exits_per_second_,
        profile_iop_code_store_exits_per_second_,
        profile_vu1_mips_);

    const u64 iop_entries = system_.iop().jit_entry_attempts();
    const u64 iop_successes = system_.iop().jit_entry_successes();
    const double iop_entry_success_percent =
        iop_entries != 0u
            ? static_cast<double>(iop_successes) * 100.0 /
                static_cast<double>(iop_entries)
            : 0.0;
    const double iop_average_residency =
        iop_successes != 0u
            ? static_cast<double>(
                  system_.iop().jit_residency_instructions()) /
                static_cast<double>(iop_successes)
            : 0.0;
    const auto iop_residency =
        system_.iop().jit_residency_histogram();
    ImGui::Text(
        "IOP JIT entries: %llu run calls   %llu attempts   %llu success (%.1f%%)   %llu compile failures",
        static_cast<unsigned long long>(system_.iop().jit_run_calls()),
        static_cast<unsigned long long>(iop_entries),
        static_cast<unsigned long long>(iop_successes),
        iop_entry_success_percent,
        static_cast<unsigned long long>(
            system_.iop().jit_compile_failures()));
    ImGui::Text(
        "IOP load-delay entries rescued: %llu",
        static_cast<unsigned long long>(
            system_.iop().jit_load_delay_entry_retires()));
    ImGui::Text(
        "IOP residency: avg %.1f instr   p50~%llu   p95~%llu   max %llu",
        iop_average_residency,
        static_cast<unsigned long long>(
            percentile_floor(iop_residency, 0.50)),
        static_cast<unsigned long long>(
            percentile_floor(iop_residency, 0.95)),
        static_cast<unsigned long long>(
            system_.iop().jit_residency_max()));
    if (ImGui::TreeNode("IOP entry refusals")) {
        static constexpr std::array<const char*, 5> kIopRejectNames = {
            "zero budget", "halted", "pending load",
            "delay slot", "pending IRQ"};
        for (u32 i = 0u; i < kIopRejectNames.size(); ++i) {
            ImGui::BulletText(
                "%s: %llu",
                kIopRejectNames[i],
                static_cast<unsigned long long>(
                    system_.iop().jit_entry_reject(i)));
        }
        ImGui::TreePop();
    }

    const auto iop_stop_counts =
        system_.iop().jit_compile_stop_opcodes();
    const auto iop_delay_stop_counts =
        system_.iop().jit_delay_slot_stop_opcodes();
    std::array<std::pair<u64, u32>, 64> iop_compile_stops{};
    for (u32 opcode = 0u; opcode < 64u; ++opcode) {
        iop_compile_stops[opcode] = {
            iop_stop_counts[opcode], opcode};
    }
    std::sort(
        iop_compile_stops.begin(),
        iop_compile_stops.end(),
        [](const auto& a, const auto& b) {
            return a.first > b.first;
        });
    if (ImGui::TreeNode("Actual IOP JIT compile stoppers")) {
        bool any = false;
        for (std::size_t i = 0u; i < 10u; ++i) {
            const auto [count, opcode] = iop_compile_stops[i];
            if (count == 0u) break;
            any = true;
            ImGui::BulletText(
                "%s (0x%02X): %llu stops (%llu in delay slots)",
                ee_major_opcode_name(opcode),
                opcode,
                static_cast<unsigned long long>(count),
                static_cast<unsigned long long>(
                    iop_delay_stop_counts[opcode]));
        }
        if (!any) {
            ImGui::TextDisabled("No IOP compile stoppers recorded yet.");
        }
        ImGui::TreePop();
    }

    ImGui::Spacing();
    bool host_timing_enabled =
        system_.profile_timing_enabled();
    if (ImGui::Checkbox(
            "Enable host-time sampling (slower)",
            &host_timing_enabled)) {
        system_.set_profile_timing_enabled(
            host_timing_enabled);
    }
    ImGui::TextUnformatted("Host emulation-thread time");
    if (!host_timing_enabled) {
        ImGui::TextDisabled(
            "Disabled for performance; native/IOP counters above remain active.");
    }
    if (ImGui::BeginTable(
            "ProfilerHostTime", 2,
            ImGuiTableFlags_Borders |
            ImGuiTableFlags_RowBg |
            ImGuiTableFlags_SizingStretchProp)) {
        const auto row = [](const char* label, double value) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.1f%%", value);
        };
        row("Quiet/native EE execution", profile_host_ee_percent_);
        row("Batched IOP catch-up", profile_host_iop_percent_);
        row("Sampled full slow path", profile_host_slow_percent_);
        row("Scheduler/DMA/GS/other", profile_host_other_percent_);
        ImGui::EndTable();
    }
    ImGui::TextDisabled(
        host_timing_enabled
            ? "Slow-path time is sampled 1/256; timing itself still perturbs emulation."
            : "Turn host-time sampling on only when diagnosing where host time goes.");

    std::array<std::pair<double, u32>, 64> fallback_rates{};
    for (u32 opcode = 0; opcode < 64u; ++opcode) {
        fallback_rates[opcode] = {
            profile_fallbacks_per_second_[opcode], opcode};
    }
    std::sort(
        fallback_rates.begin(),
        fallback_rates.end(),
        [](const auto& a, const auto& b) {
            return a.first > b.first;
        });

    ImGui::Spacing();
    ImGui::TextUnformatted("Top native fallback opcode families");
    bool any_fallback = false;
    for (std::size_t i = 0; i < 8u; ++i) {
        if (fallback_rates[i].first <= 0.0) break;
        any_fallback = true;
        ImGui::BulletText(
            "%s (0x%02X): %.0f exits/s",
            ee_major_opcode_name(fallback_rates[i].second),
            fallback_rates[i].second,
            fallback_rates[i].first);
    }
    if (!any_fallback) {
        ImGui::TextDisabled(
            system_.ee().jit_enabled()
                ? "No native fallback exits in this sample."
                : "Enable the EE JIT to collect native fallback data.");
    }

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

    if (gpu_gs_backend_ != nullptr) {
        bool enabled = gpu_gs_enabled_;
        if (ImGui::Checkbox(
                "GPU GS (OpenGL compute)",
                &enabled)) {
            gpu_gs_enabled_ = enabled;
            system_.gs_core().set_gpu_backend(
                enabled
                    ? static_cast<GsGpuBackend*>(
                        gpu_gs_backend_.get())
                    : nullptr);
        }
        ImGui::SameLine();
        ImGui::TextDisabled(
            system_.gs_core().gpu_backend_active()
                ? "(active)"
                : "(software fallback)");
    } else {
        ImGui::TextDisabled(
            "GPU GS: unavailable (OpenGL 4.3 compute required)");
    }

    const float ui_fps = ImGui::GetIO().Framerate;
    const float ui_ms =
        ui_fps > 0.0f ? 1000.0f / ui_fps : 0.0f;

    ImGui::Separator();
    ImGui::Text(
        "Guest: %.1f FPS   %.1f%% speed   %.1f fields/s",
        guest_frames_per_second_,
        emulation_speed_percent_,
        guest_fields_per_second_);
    ImGui::Text(
        "Host UI: %.1f FPS   %.2f ms/frame   EE: %.1f MIPS",
        static_cast<double>(ui_fps),
        static_cast<double>(ui_ms),
        ee_instructions_per_second_ / 1'000'000.0);
    ImGui::TextDisabled(
        "100%% = 59.94 fields/s = 29.97 interlaced frames/s.");
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
        row("GPU sprite draws", static_cast<unsigned long long>(stats.gpu_sprite_draws));
        row("GPU sprite pixels", static_cast<unsigned long long>(stats.gpu_sprite_pixels));
        row("GPU->CPU VRAM syncs", static_cast<unsigned long long>(stats.gpu_syncs_to_cpu));
        row("CPU parallel sprite draws", static_cast<unsigned long long>(stats.parallel_sprite_draws));
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
    speed_sample_fields_ = system_.video_fields_started();
    ee_instructions_per_second_ = 0.0;
    guest_fields_per_second_ = 0.0;
    guest_frames_per_second_ = 0.0;
    emulation_speed_percent_ = 0.0;

    const auto& jit = system_.ee().jit();
    profile_sample_native_instructions_ = jit.block_instruction_count();
    profile_sample_native_blocks_ = jit.block_executed_count();
    profile_sample_guard_bailouts_ = jit.block_guard_bailout_count();
    profile_sample_code_store_exits_ = jit.block_code_store_exit_count();
    profile_sample_cache_flushes_ = jit.cache_flush_count();
    profile_sample_run_ns_ = system_.profile_run_ns();
    profile_sample_ee_ns_ = system_.profile_ee_ns();
    profile_sample_iop_ns_ = system_.profile_iop_ns();
    profile_sample_slow_path_ns_ = system_.profile_slow_path_ns();
    profile_sample_iop_instructions_ =
        system_.iop().state().instructions_executed;
    profile_sample_iop_idle_pairs_ =
        system_.skipped_iop_idle_pairs();
    profile_sample_iop_native_instructions_ =
        system_.iop().jit_native_instructions();
    profile_sample_iop_native_blocks_ =
        system_.iop().jit_native_blocks();
    profile_sample_iop_native_chains_ =
        system_.iop().jit_native_chains();
    profile_sample_iop_guard_exits_ =
        system_.iop().jit_guard_exits();
    profile_sample_iop_code_store_exits_ =
        system_.iop().jit_code_store_exits();
    profile_sample_vu1_instructions_ =
        system_.vu1().stats().instructions;
    profile_sample_fallback_opcodes_ =
        system_.native_fallback_opcodes();
    profile_fallbacks_per_second_.fill(0.0);

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
        constexpr double kNtscFieldsPerSecond = 59.94;

        const u64 instructions =
            system_.ee().state().instructions_executed;
        const u64 fields =
            system_.video_fields_started();

        const u64 ee_delta =
            instructions - speed_sample_instructions_;
        ee_instructions_per_second_ =
            static_cast<double>(ee_delta) /
            sample_seconds;

        const auto& jit = system_.ee().jit();
        const u64 native_instructions =
            jit.block_instruction_count();
        const u64 native_blocks =
            jit.block_executed_count();
        const u64 guard_bailouts =
            jit.block_guard_bailout_count();
        const u64 code_store_exits =
            jit.block_code_store_exit_count();
        const u64 cache_flushes =
            jit.cache_flush_count();

        const u64 native_delta =
            native_instructions - profile_sample_native_instructions_;
        const u64 native_block_delta =
            native_blocks - profile_sample_native_blocks_;
        const u64 guard_delta =
            guard_bailouts - profile_sample_guard_bailouts_;
        const u64 code_store_delta =
            code_store_exits - profile_sample_code_store_exits_;
        const u64 cache_flush_delta =
            cache_flushes - profile_sample_cache_flushes_;

        profile_native_mips_ =
            static_cast<double>(native_delta) /
            sample_seconds / 1'000'000.0;
        profile_native_coverage_percent_ =
            ee_delta != 0u
                ? static_cast<double>(native_delta) * 100.0 /
                    static_cast<double>(ee_delta)
                : 0.0;
        profile_native_blocks_per_second_ =
            static_cast<double>(native_block_delta) /
            sample_seconds;
        profile_average_native_block_ =
            native_block_delta != 0u
                ? static_cast<double>(native_delta) /
                    static_cast<double>(native_block_delta)
                : 0.0;
        profile_guard_bailouts_per_second_ =
            static_cast<double>(guard_delta) / sample_seconds;
        profile_code_store_exits_per_second_ =
            static_cast<double>(code_store_delta) / sample_seconds;
        profile_cache_flushes_per_second_ =
            static_cast<double>(cache_flush_delta) / sample_seconds;

        const u64 iop_instructions =
            system_.iop().state().instructions_executed;
        const u64 iop_delta =
            iop_instructions - profile_sample_iop_instructions_;
        profile_iop_mips_ =
            static_cast<double>(iop_delta) /
            sample_seconds / 1'000'000.0;

        const u64 iop_native_instructions =
            system_.iop().jit_native_instructions();
        const u64 iop_native_blocks =
            system_.iop().jit_native_blocks();
        const u64 iop_native_chains =
            system_.iop().jit_native_chains();
        const u64 iop_guard_exits =
            system_.iop().jit_guard_exits();
        const u64 iop_code_store_exits =
            system_.iop().jit_code_store_exits();
        const u64 iop_native_delta =
            iop_native_instructions -
            profile_sample_iop_native_instructions_;
        const u64 iop_native_block_delta =
            iop_native_blocks -
            profile_sample_iop_native_blocks_;
        profile_iop_native_mips_ =
            static_cast<double>(iop_native_delta) /
            sample_seconds / 1'000'000.0;
        const u64 iop_idle_pairs =
            system_.skipped_iop_idle_pairs();
        const u64 iop_idle_instructions_delta =
            (iop_idle_pairs - profile_sample_iop_idle_pairs_) * 2u;
        const u64 iop_active_delta =
            iop_delta > iop_idle_instructions_delta
                ? iop_delta - iop_idle_instructions_delta
                : 0u;
        const u64 iop_slow_delta =
            iop_active_delta > iop_native_delta
                ? iop_active_delta - iop_native_delta
                : 0u;
        profile_iop_idle_mips_ =
            static_cast<double>(iop_idle_instructions_delta) /
            sample_seconds / 1'000'000.0;
        profile_iop_slow_mips_ =
            static_cast<double>(iop_slow_delta) /
            sample_seconds / 1'000'000.0;
        // Raw coverage is retained for continuity with older profiler
        // screenshots. Active coverage excludes the explicit OSDSYS J/NOP
        // idle accelerator because those instructions never touched the
        // interpreter and should not be counted as failed JIT work.
        profile_iop_native_coverage_percent_ =
            iop_delta != 0u
                ? static_cast<double>(iop_native_delta) * 100.0 /
                    static_cast<double>(iop_delta)
                : 0.0;
        profile_iop_active_native_coverage_percent_ =
            iop_active_delta != 0u
                ? static_cast<double>(iop_native_delta) * 100.0 /
                    static_cast<double>(iop_active_delta)
                : 0.0;
        profile_iop_native_blocks_per_second_ =
            static_cast<double>(iop_native_block_delta) /
            sample_seconds;
        profile_iop_average_native_block_ =
            iop_native_block_delta != 0u
                ? static_cast<double>(iop_native_delta) /
                    static_cast<double>(iop_native_block_delta)
                : 0.0;
        profile_iop_native_chains_per_second_ =
            static_cast<double>(
                iop_native_chains -
                profile_sample_iop_native_chains_) /
            sample_seconds;
        profile_iop_guard_exits_per_second_ =
            static_cast<double>(
                iop_guard_exits -
                profile_sample_iop_guard_exits_) /
            sample_seconds;
        profile_iop_code_store_exits_per_second_ =
            static_cast<double>(
                iop_code_store_exits -
                profile_sample_iop_code_store_exits_) /
            sample_seconds;

        const u64 vu1_instructions =
            system_.vu1().stats().instructions;
        profile_vu1_mips_ =
            static_cast<double>(
                vu1_instructions - profile_sample_vu1_instructions_) /
            sample_seconds / 1'000'000.0;

        const u64 run_ns = system_.profile_run_ns();
        const u64 ee_ns = system_.profile_ee_ns();
        const u64 iop_ns = system_.profile_iop_ns();
        const u64 slow_path_ns = system_.profile_slow_path_ns();
        const u64 run_delta = run_ns - profile_sample_run_ns_;
        const u64 ee_ns_delta = ee_ns - profile_sample_ee_ns_;
        const u64 iop_ns_delta = iop_ns - profile_sample_iop_ns_;
        const u64 slow_path_ns_delta =
            slow_path_ns - profile_sample_slow_path_ns_;
        const u64 accounted_ns =
            ee_ns_delta + iop_ns_delta + slow_path_ns_delta;
        const u64 other_ns =
            run_delta > accounted_ns
                ? run_delta - accounted_ns
                : 0u;
        if (run_delta != 0u) {
            const double scale =
                100.0 / static_cast<double>(run_delta);
            profile_host_ee_percent_ =
                static_cast<double>(ee_ns_delta) * scale;
            profile_host_iop_percent_ =
                static_cast<double>(iop_ns_delta) * scale;
            profile_host_slow_percent_ =
                static_cast<double>(slow_path_ns_delta) * scale;
            profile_host_other_percent_ =
                static_cast<double>(other_ns) * scale;
        } else {
            profile_host_ee_percent_ = 0.0;
            profile_host_iop_percent_ = 0.0;
            profile_host_slow_percent_ = 0.0;
            profile_host_other_percent_ = 0.0;
        }

        const auto& fallback_opcodes =
            system_.native_fallback_opcodes();
        for (u32 opcode = 0; opcode < 64u; ++opcode) {
            profile_fallbacks_per_second_[opcode] =
                static_cast<double>(
                    fallback_opcodes[opcode] -
                    profile_sample_fallback_opcodes_[opcode]) /
                sample_seconds;
            profile_sample_fallback_opcodes_[opcode] =
                fallback_opcodes[opcode];
        }

        profile_sample_native_instructions_ = native_instructions;
        profile_sample_native_blocks_ = native_blocks;
        profile_sample_guard_bailouts_ = guard_bailouts;
        profile_sample_code_store_exits_ = code_store_exits;
        profile_sample_cache_flushes_ = cache_flushes;
        profile_sample_run_ns_ = run_ns;
        profile_sample_ee_ns_ = ee_ns;
        profile_sample_iop_ns_ = iop_ns;
        profile_sample_slow_path_ns_ = slow_path_ns;
        profile_sample_iop_instructions_ = iop_instructions;
        profile_sample_iop_idle_pairs_ =
            system_.skipped_iop_idle_pairs();
        profile_sample_iop_native_instructions_ =
            iop_native_instructions;
        profile_sample_iop_native_blocks_ = iop_native_blocks;
        profile_sample_iop_native_chains_ = iop_native_chains;
        profile_sample_iop_guard_exits_ = iop_guard_exits;
        profile_sample_iop_code_store_exits_ =
            iop_code_store_exits;
        profile_sample_vu1_instructions_ = vu1_instructions;

        guest_fields_per_second_ =
            static_cast<double>(
                fields - speed_sample_fields_) /
            sample_seconds;
        guest_frames_per_second_ =
            guest_fields_per_second_ * 0.5;
        emulation_speed_percent_ =
            (guest_fields_per_second_ /
             kNtscFieldsPerSecond) * 100.0;

        speed_sample_instructions_ = instructions;
        speed_sample_fields_ = fields;
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
    profile_native_mips_ = 0.0;
    profile_iop_mips_ = 0.0;
    profile_iop_native_mips_ = 0.0;
    profile_iop_native_coverage_percent_ = 0.0;
    profile_iop_native_blocks_per_second_ = 0.0;
    profile_iop_average_native_block_ = 0.0;
    profile_iop_native_chains_per_second_ = 0.0;
    profile_iop_guard_exits_per_second_ = 0.0;
    profile_iop_code_store_exits_per_second_ = 0.0;
    profile_vu1_mips_ = 0.0;
    profile_native_coverage_percent_ = 0.0;
    profile_native_blocks_per_second_ = 0.0;
    profile_average_native_block_ = 0.0;
    profile_guard_bailouts_per_second_ = 0.0;
    profile_code_store_exits_per_second_ = 0.0;
    profile_cache_flushes_per_second_ = 0.0;
    profile_host_ee_percent_ = 0.0;
    profile_host_iop_percent_ = 0.0;
    profile_host_slow_percent_ = 0.0;
    profile_host_other_percent_ = 0.0;
    profile_fallbacks_per_second_.fill(0.0);
    reset_audio_stutter();
    if (audio_device_ != 0) SDL_ClearQueuedAudio(audio_device_);
    system_.reset(0);
    status_message_ =
        system_.bios().loaded()
            ? "PS2 core reset; BIOS remains loaded"
            : "PS2 core reset";
}

} // namespace ps2::ui
