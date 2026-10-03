#include "ui/ps2_app.h"
#include "ui/ps2_gl_gs_backend.h"
#include "ui/theme_settings.h"
#include "ui/vs2/vs2_frontend.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl2.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#include <commdlg.h>
#include <shobjidl.h>
#endif

namespace ps2::ui {

namespace {
constexpr const char* kFrontendTitle = "VibeStation 2";
constexpr const char* kDeveloperTitle = "VibeStation - PS2 Experimental";
} // namespace

Ps2App::Ps2App() = default;
Ps2App::~Ps2App() = default;

namespace {

constexpr std::size_t kAudioChannels = 2u;
constexpr std::size_t kStutterHistoryFrames =
    static_cast<std::size_t>(Spu2::kSampleRate) * 400u / 1000u;
constexpr std::size_t kStutterQueueTargetFrames =
    static_cast<std::size_t>(Spu2::kSampleRate) * 80u / 1000u;

void append_be32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::uint8_t>(value >> shift));
    }
}

std::uint32_t png_crc(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

void append_png_chunk(std::vector<std::uint8_t>& out, const char* type,
                      const std::vector<std::uint8_t>& data) {
    append_be32(out, static_cast<std::uint32_t>(data.size()));
    const std::size_t start = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), data.begin(), data.end());
    append_be32(out, png_crc(out.data() + start, out.size() - start));
}

// Uncompressed PNG (stored DEFLATE blocks), the same format VibeStation 1's
// snapshots use. RGBA8 with red in the low byte, alpha forced opaque.
std::vector<std::uint8_t> encode_png(const std::vector<std::uint32_t>& rgba, int width,
                                     int height) {
    std::vector<std::uint8_t> raw;
    raw.reserve(static_cast<std::size_t>(height) * (static_cast<std::size_t>(width) * 4u + 1u));
    for (int y = 0; y < height; ++y) {
        raw.push_back(0); // filter: none
        for (int x = 0; x < width; ++x) {
            const std::uint32_t px = rgba[static_cast<std::size_t>(y) * width + x];
            raw.push_back(static_cast<std::uint8_t>(px));
            raw.push_back(static_cast<std::uint8_t>(px >> 8));
            raw.push_back(static_cast<std::uint8_t>(px >> 16));
            raw.push_back(255);
        }
    }

    std::vector<std::uint8_t> idat = {0x78, 0x01};
    for (std::size_t offset = 0; offset < raw.size();) {
        const std::size_t len = std::min<std::size_t>(raw.size() - offset, 65535u);
        const bool last = offset + len == raw.size();
        idat.push_back(last ? 1 : 0);
        idat.push_back(static_cast<std::uint8_t>(len));
        idat.push_back(static_cast<std::uint8_t>(len >> 8));
        idat.push_back(static_cast<std::uint8_t>(~len));
        idat.push_back(static_cast<std::uint8_t>(~len >> 8));
        idat.insert(idat.end(), raw.begin() + offset, raw.begin() + offset + len);
        offset += len;
    }
    std::uint32_t a = 1, b = 0;
    for (const std::uint8_t byte : raw) {
        a = (a + byte) % 65521u;
        b = (b + a) % 65521u;
    }
    append_be32(idat, (b << 16) | a);

    std::vector<std::uint8_t> png = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::vector<std::uint8_t> ihdr;
    append_be32(ihdr, static_cast<std::uint32_t>(width));
    append_be32(ihdr, static_cast<std::uint32_t>(height));
    ihdr.insert(ihdr.end(), {8, 6, 0, 0, 0}); // 8-bit RGBA
    append_png_chunk(png, "IHDR", ihdr);
    append_png_chunk(png, "IDAT", idat);
    append_png_chunk(png, "IEND", {});
    return png;
}

} // namespace

bool Ps2App::init(const HostedWindow* host) {
    hosted_ = host != nullptr;
    if (hosted_) {
        // Running inside VibeStation's shared window, next to VibeStation 1.
        window_ = host->window;
        gl_context_ = host->gl_context;
        gl_major_ = host->gl_major;
        gl_minor_ = host->gl_minor;
        imgui_glsl_version_ = host->glsl;
        use_imgui_opengl2_backend_ = host->opengl2;
        SDL_GL_MakeCurrent(window_, gl_context_);
    } else if (!create_own_window()) {
        return false;
    }
    return init_after_window();
}

bool Ps2App::create_own_window() {
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
            kFrontendTitle,
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            1280,
            800,
            SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI |
                SDL_WINDOW_HIDDEN);

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
        // Black from the first moment, not a blank window while the rest of
        // the app (fonts, textures, audio) sets up.
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        SDL_GL_SwapWindow(window_);
        SDL_ShowWindow(window_);
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
    return true;
}

bool Ps2App::init_after_window() {
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
    // Own context, made current explicitly: when hosted, VibeStation 1's
    // context already exists and ImGui would not switch to the new one.
    imgui_context_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(imgui_context_);

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
            // Off by default: the parallel CPU rasterizer outruns the VRAM
            // readbacks the GPU path needs (measured 88% vs 64% speed on the
            // BIOS animation). Enable it in the GS debug panel.
            if (!gpu_gs_backend_->available()) {
                gpu_gs_backend_.reset();
            }
        }
    }

    system_.gs_core().set_async_rasterization(true);
    reset_core();
    status_message_ = "PS2 experimental core ready";

    // Last, so saved settings apply to the fully set up core and GPU backend.
    frontend_ = std::make_unique<vs2::Frontend>(*this);
    frontend_->init();
    SDL_SetWindowTitle(window_, developer_view_ ? kDeveloperTitle : kFrontendTitle);
    return true;
}

int Ps2App::run() {
    begin_run();
    while (frame()) {
    }
    stop_emulation_thread();
    return run_result_;
}

void Ps2App::begin_run() {
    if (emu_thread_.joinable()) return;
    emu_stop_.store(false, std::memory_order_release);
    emu_thread_ = std::thread(&Ps2App::emulation_thread_main, this);
    frame_started_ = std::chrono::steady_clock::now();
}

bool Ps2App::frame() {
    ImGui::SetCurrentContext(imgui_context_);
    bool quit = false;
    int& result = run_result_;
    auto& frame_started = frame_started_;
    {
        // Hold the core only while reading or mutating emulator state; the
        // GL submission and vsync wait below run while the core executes.
        ui_waiting_.store(true, std::memory_order_release);
        std::unique_lock<std::mutex> core_lock(core_mutex_);
        ui_waiting_.store(false, std::memory_order_release);

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
            return false;
        }

        if (use_imgui_opengl2_backend_) {
            ImGui_ImplOpenGL2_NewFrame();
        } else {
            ImGui_ImplOpenGL3_NewFrame();
        }
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        update_display_texture();
        update_ps1_texture();
        render_ui();

        ImGui::Render();

        const bool capture_now =
            !visible_capture_path_.empty() &&
            system_.gs_display().has_visible_pixels() &&
            system_.ee().state().instructions_executed >=
                visible_capture_minimum_ee_;
        if (benchmark_fields_ != 0 &&
            system_.gs_display().has_visible_pixels()) {
            const u64 field = system_.video_fields_started();
            const auto now = std::chrono::steady_clock::now();
            if (!benchmark_started_) {
                benchmark_started_ = true;
                benchmark_start_field_ = field;
                benchmark_start_time_ = now;
                benchmark_ui_frames_ = 0;
            } else {
                ++benchmark_ui_frames_;
                if (field - benchmark_start_field_ >= benchmark_fields_) {
                    const double seconds =
                        std::chrono::duration<double>(
                            now - benchmark_start_time_).count();
                    const u64 fields = field - benchmark_start_field_;
                    std::fprintf(
                        stdout,
                        "UI_BENCHMARK_FIELDS=%llu UI_BENCHMARK_SECONDS=%.3f "
                        "UI_BENCHMARK_FIELD_RATE=%.3f "
                        "UI_BENCHMARK_UI_FPS=%.1f GPU_GS=%d\n",
                        static_cast<unsigned long long>(fields),
                        seconds,
                        static_cast<double>(fields) / seconds,
                        static_cast<double>(benchmark_ui_frames_) / seconds,
                        system_.gs_core().gpu_backend_active() ? 1 : 0);
                    std::fflush(stdout);
                    quit = true;
                }
            }
        }
        core_lock.unlock();

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

        if (capture_now) {
            if (!write_window_ppm(
                    visible_capture_path_, display_width, display_height)) {
                result = 3;
            }
            visible_capture_path_.clear();
            quit = true;
        }

        SDL_GL_SwapWindow(window_);

        // Frame cap. Vsync is requested, but some drivers ignore it for
        // windowed (or unfocused) apps and the loop would redraw flat out.
        // Sleep off whatever is left of the display's frame time; with
        // working vsync nothing is left and this does nothing. The core lock
        // is already released, so emulation is unaffected.
        {
            using clock = std::chrono::steady_clock;
            SDL_DisplayMode mode{};
            const int display = SDL_GetWindowDisplayIndex(window_);
            const int refresh =
                display >= 0 && SDL_GetCurrentDisplayMode(display, &mode) == 0 && mode.refresh_rate > 0
                    ? mode.refresh_rate
                    : 60;
            const auto frame_time = std::chrono::duration_cast<clock::duration>(
                std::chrono::duration<double>(1.0 / refresh));
            const auto deadline = frame_started + frame_time;
            const auto now = clock::now();
            if (now < deadline) {
                // Coarse sleep, then yield for the last stretch: Windows
                // sleeps are only accurate to about a millisecond.
                const auto coarse = deadline - now - std::chrono::milliseconds(2);
                if (coarse > clock::duration::zero()) std::this_thread::sleep_for(coarse);
                while (clock::now() < deadline) std::this_thread::yield();
            }
            // Keep a steady cadence; after a late frame, start over from now.
            const auto after = clock::now();
            frame_started = after < deadline + frame_time ? deadline : after;
        }
    }
    return !quit;
}

void Ps2App::stop_emulation_thread() {
    emu_stop_.store(true, std::memory_order_release);
    if (emu_thread_.joinable()) emu_thread_.join();
}

bool Ps2App::launch_bios(const std::string& path) {
    return load_bios_from_path(path) && (ps1_->active() || start_bios());
}

void Ps2App::set_ee_jit_enabled(bool enabled) {
    system_.ee().set_jit_enabled(enabled);
}

void Ps2App::set_ee_dynarec_enabled(bool enabled) {
    system_.ee().set_dynarec_enabled(enabled);
}

void Ps2App::set_gpu_gs_enabled(bool enabled) {
    gpu_gs_enabled_ = enabled;
    system_.gs_core().set_gpu_backend(
        enabled && gpu_gs_backend_ != nullptr
            ? static_cast<GsGpuBackend*>(gpu_gs_backend_.get())
            : nullptr);
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
    stop_emulation_thread();
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

    stop_ps1();
    // Before the GL context goes: the frontend owns textures and threads.
    if (frontend_) {
        frontend_->shutdown();
        frontend_.reset();
    }
    system_.gs_core().set_gpu_backend(nullptr);
    gpu_gs_backend_.reset();

    if (ps1_texture_ != 0 && gl_context_ != nullptr) {
        glDeleteTextures(1, &ps1_texture_);
        ps1_texture_ = 0;
    }

    if (display_texture_ != 0 && gl_context_ != nullptr) {
        glDeleteTextures(1, &display_texture_);
        display_texture_ = 0;
        display_texture_width_ = 0;
        display_texture_height_ = 0;
        display_texture_generation_ = ~0ull;
    }

    if (imgui_context_ != nullptr) {
        ImGui::SetCurrentContext(imgui_context_);
        if (use_imgui_opengl2_backend_) {
            ImGui_ImplOpenGL2_Shutdown();
        } else {
            ImGui_ImplOpenGL3_Shutdown();
        }
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext(imgui_context_);
        imgui_context_ = nullptr;
    }

    // When hosted, the window, the GL context and SDL belong to the host.
    if (hosted_) {
        window_ = nullptr;
        gl_context_ = nullptr;
        return;
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

// ---------------------------------------------------------------------------
// Running inside VibeStation next to VibeStation 1

void Ps2App::on_activated() {
    ImGui::SetCurrentContext(imgui_context_);
    SDL_GL_SetSwapInterval(1);
    SDL_SetWindowTitle(window_, developer_view_ ? kDeveloperTitle : kFrontendTitle);
    ImGui::GetIO().ClearInputKeys();
    begin_run();
    developer_view_ = false;
    if (frontend_) frontend_->restart_boot();
}

void Ps2App::on_deactivated() {
    std::lock_guard<std::mutex> lock(core_mutex_);
    pause_session();
    // Nothing may keep sounding while VibeStation 1 has the window: the
    // stutter tape would otherwise loop its last 400 ms forever.
    audio_live_.store(false, std::memory_order_release);
    reset_audio_stutter();
    if (audio_device_ != 0) SDL_ClearQueuedAudio(audio_device_);
    if (frontend_) frontend_->on_hidden();
}

bool Ps2App::take_vs1_switch_request() {
    const bool requested = vs1_switch_requested_;
    vs1_switch_requested_ = false;
    return requested;
}

bool Ps2App::can_switch_to_vs1() const { return hosted_; }
void Ps2App::switch_to_vs1() { vs1_switch_requested_ = true; }

void Ps2App::hand_ps1_disc_to_vs1(const std::string& path) { ps1_handoff_path_ = path; }

std::string Ps2App::take_ps1_handoff() { return std::exchange(ps1_handoff_path_, {}); }

void Ps2App::begin_vs1_switch() {
    if (frontend_) frontend_->leave_to_vs1();
    else vs1_switch_requested_ = true;
}


void Ps2App::update_display_texture() {
    const auto& display = system_.gs_display();
    // Scanout runs on the GS worker; keep the frame stable while uploading.
    const auto display_lock = display.lock();
    if (!display.valid() || display.rgba8().empty()) {
        return;
    }
    if (display_texture_ != 0 && display_filter_applied_ != display_filter_) {
        const GLint filter = display_filter_ == 0 ? GL_NEAREST : GL_LINEAR;
        glBindTexture(GL_TEXTURE_2D, display_texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        glBindTexture(GL_TEXTURE_2D, 0);
        display_filter_applied_ = display_filter_;
    }
    if (display_texture_generation_ == display.generation()) {
        return;
    }

    if (display_texture_ == 0) {
        glGenTextures(1, &display_texture_);
        glBindTexture(GL_TEXTURE_2D, display_texture_);
        const GLint filter = display_filter_ == 0 ? GL_NEAREST : GL_LINEAR;
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        display_filter_applied_ = display_filter_;
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
    if (frontend_) {
        frontend_->on_game_frame(display.rgba8().data(), static_cast<int>(display.width()),
                                 static_cast<int>(display.height()));
    }
}

void Ps2App::update_ps1_texture() {
    if (!ps1_->active()) return;
    int width = 0;
    int height = 0;
    if (!ps1_->take_frame(ps1_frame_, width, height) || width <= 0 ||
        height <= 0 ||
        ps1_frame_.size() < static_cast<std::size_t>(width) * height) {
        return;
    }
    if (ps1_texture_ == 0) {
        glGenTextures(1, &ps1_texture_);
        glBindTexture(GL_TEXTURE_2D, ps1_texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    } else {
        glBindTexture(GL_TEXTURE_2D, ps1_texture_);
    }
    const GLint filter = display_filter_ == 0 ? GL_NEAREST : GL_LINEAR;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    if (width != ps1_texture_width_ || height != ps1_texture_height_) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, ps1_frame_.data());
        ps1_texture_width_ = width;
        ps1_texture_height_ = height;
    } else {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA,
                        GL_UNSIGNED_BYTE, ps1_frame_.data());
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    ps1_width_ = width;
    ps1_height_ = height;
    if (frontend_) frontend_->on_game_frame(ps1_frame_.data(), width, height);
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
    // While a frontend menu is up, the keys drive the menu, not the game.
    if (!developer_view_ && frontend_ && !frontend_->wants_game_input()) {
        system_.pad().set_state(state);
        ps1_->set_buttons(0xFFFFu);
        return;
    }
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
    // The PS1 pad word has the two button bytes the other way round.
    ps1_->set_buttons(static_cast<std::uint16_t>(
        (state.buttons << 8) | (state.buttons >> 8)));
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
            !audio_live_.load(std::memory_order_acquire) ||
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
    const bool live = emulation_running_ && !system_.halted();
    if (audio_live_.exchange(live, std::memory_order_acq_rel) && !live &&
        audio_device_ != 0u) {
        // Emulation just stopped (pause, halt, reset): drain what is queued
        // and wipe the tape so nothing keeps looping the last 400 ms.
        reset_audio_stutter();
        SDL_ClearQueuedAudio(audio_device_);
    }
    if (!live) return;

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

        if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
            event.key.keysym.sym == SDLK_F12) {
            set_developer_view(!developer_view_);
            continue;
        }
        // The frontend reads its own keys through ImGui.
        if (!developer_view_ && frontend_) continue;

        if (event.type == SDL_KEYDOWN && event.key.repeat == 0) {
            const bool ctrl = (event.key.keysym.mod & KMOD_CTRL) != 0;
            if (ctrl && event.key.keysym.sym == SDLK_o) {
                const std::string path = open_disc_dialog();
                if (!path.empty()) {
                    load_disc_from_path(path);
                }
            } else if (ctrl && event.key.keysym.sym == SDLK_b) {
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
    if (!developer_view_ && frontend_) {
        frontend_->frame();
        return;
    }
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
        if (ImGui::MenuItem("Open Disc...", "Ctrl+O")) {
            const std::string path = open_disc_dialog();
            if (!path.empty()) {
                load_disc_from_path(path);
            }
        }
        if (ImGui::MenuItem("Eject Disc", nullptr, false,
                            system_.cdvd().has_disc() || ps1_->active())) {
            stop_ps1();
            system_.eject_disc();
            reset_core();
            if (system_.bios().loaded()) start_bios();
            status_message_ = "Disc ejected";
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

    if (ImGui::BeginMenu("PS1 Speed", ps1_->active())) {
        static constexpr const char* kLabels[] = {
            "1x (real time)", "2x", "4x", "Unlimited"};
        static constexpr double kSpeeds[] = {1.0, 2.0, 4.0, 0.0};
        for (int i = 0; i < 4; ++i) {
            if (ImGui::MenuItem(kLabels[i], nullptr, ps1_speed_index_ == i)) {
                ps1_speed_index_ = i;
                ps1_->set_speed(kSpeeds[i]);
            }
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
        ImGui::Separator();
        if (ImGui::MenuItem("VibeStation 2 Frontend", "F12", false, frontend_ != nullptr)) {
            set_developer_view(false);
        }
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

    if (ps1_->active() && ps1_texture_ != 0 && ps1_width_ > 0) {
        // PS1 output is 4:3 whatever VRAM region is scanned out.
        const float aspect = 4.0f / 3.0f;
        ImVec2 image_size = available;
        if (image_size.y > 0.0f && image_size.x / image_size.y > aspect) {
            image_size.x = image_size.y * aspect;
        } else {
            image_size.y = image_size.x / aspect;
        }
        image_size.x = std::max(1.0f, std::floor(image_size.x + 0.5f));
        image_size.y = std::max(1.0f, std::floor(image_size.y + 0.5f));
        ImGui::SetCursorPos(ImVec2(
            std::floor(start.x + (available.x - image_size.x) * 0.5f),
            std::floor(start.y + (available.y - image_size.y) * 0.5f)));
        ImGui::Image((ImTextureID)(intptr_t)ps1_texture_, image_size);
        ImGui::SetCursorPos(ImVec2(start.x + 10.0f, start.y + 10.0f));
        ImGui::TextDisabled("PS1 core  %dx%d", ps1_width_, ps1_height_);
        return;
    }

    const auto& display = system_.gs_display();
    if (display.valid() && display_texture_ != 0 &&
        display.width() != 0 && display.height() != 0) {
        // The console outputs 4:3 whatever the scanned-out raster is
        // (640x224 field, 640x448 frame, 640x512 PAL, ...).
        const float aspect = 4.0f / 3.0f;
        ImVec2 image_size = available;
        if (image_size.y > 0.0f && image_size.x / image_size.y > aspect) {
            image_size.x = image_size.y * aspect;
        } else if (aspect > 0.0f) {
            image_size.y = image_size.x / aspect;
        }
        // Whole-pixel size and position: a fractional rectangle makes the
        // sampler straddle host pixels differently across the image.
        image_size.x = std::max(1.0f, std::floor(image_size.x + 0.5f));
        image_size.y = std::max(1.0f, std::floor(image_size.y + 0.5f));

        ImGui::SetCursorPos(ImVec2(
            std::floor(start.x + (available.x - image_size.x) * 0.5f),
            std::floor(start.y + (available.y - image_size.y) * 0.5f)));
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
        return;
    }
    // Running but nothing on screen yet: a black 4:3 picture, speed readout
    // still visible.
    if (emulation_running_) {
        const float aspect = 4.0f / 3.0f;
        ImVec2 image_size = available;
        if (image_size.y > 0.0f && image_size.x / image_size.y > aspect) {
            image_size.x = image_size.y * aspect;
        } else {
            image_size.y = image_size.x / aspect;
        }
        image_size.x = std::max(1.0f, std::floor(image_size.x + 0.5f));
        image_size.y = std::max(1.0f, std::floor(image_size.y + 0.5f));
        ImGui::SetCursorPos(ImVec2(
            std::floor(start.x + (available.x - image_size.x) * 0.5f),
            std::floor(start.y + (available.y - image_size.y) * 0.5f)));
        const ImVec2 corner = ImGui::GetCursorScreenPos();
        ImGui::Dummy(image_size);
        ImGui::GetWindowDrawList()->AddRectFilled(
            corner,
            ImVec2(corner.x + image_size.x, corner.y + image_size.y),
            IM_COL32(0, 0, 0, 255));
        if (guest_fields_per_second_ > 0.0) {
            ImGui::SetCursorPos(ImVec2(start.x + 10.0f, start.y + 28.0f));
            ImGui::Text(
                "%.1f FPS  |  %.0f%% speed  |  %.1f MIPS",
                guest_frames_per_second_,
                emulation_speed_percent_,
                ee_instructions_per_second_ / 1'000'000.0);
        }
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
    bool limit_speed = limit_speed_.load(std::memory_order_acquire);
    if (ImGui::Checkbox("Limit speed to 100%", &limit_speed)) {
        limit_speed_.store(limit_speed, std::memory_order_release);
    }
    ImGui::TextDisabled(
        "Off runs the core flat out, useful for measuring raw speed.");

    const char* const filters[] = {"Nearest", "Bilinear"};
    ImGui::Combo(
        "Display filter", &display_filter_, filters, 2);
    ImGui::TextDisabled(
        "How the 640x448 picture is scaled to the window.");

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

std::string Ps2App::open_bios_dialog(const char* title) {
#ifdef _WIN32
    std::array<char, 1024> path{};

    OPENFILENAMEA dialog{};
    dialog.lStructSize = sizeof(dialog);
    dialog.lpstrFile = path.data();
    dialog.nMaxFile = static_cast<DWORD>(path.size());
    dialog.lpstrFilter =
        "BIOS Images (*.bin)\0*.bin\0All Files (*.*)\0*.*\0";
    dialog.nFilterIndex = 1;
    dialog.lpstrTitle = title;
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

std::string Ps2App::open_disc_dialog() {
#ifdef _WIN32
    std::array<char, 1024> path{};

    OPENFILENAMEA dialog{};
    dialog.lStructSize = sizeof(dialog);
    dialog.lpstrFile = path.data();
    dialog.nMaxFile = static_cast<DWORD>(path.size());
    dialog.lpstrFilter =
        "Disc Images (*.iso;*.bin;*.img;*.cue)\0*.iso;*.bin;*.img;*.cue\0"
        "All Files (*.*)\0*.*\0";
    dialog.nFilterIndex = 1;
    dialog.lpstrTitle = "Open PlayStation / PlayStation 2 disc image";
    dialog.Flags =
        OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;

    if (GetOpenFileNameA(&dialog)) {
        return path.data();
    }
    return {};
#else
    status_message_ = "Native disc picker is currently Windows-only.";
    return {};
#endif
}

bool Ps2App::load_disc_from_path(const std::string& path) {
    stop_ps1();
    std::string error;
    if (!system_.load_disc(path, error)) {
        status_message_ = "Disc load failed: " + error;
        return false;
    }
    if (system_.cdvd().disc().type() == DiscType::Ps1Cd) {
        return start_ps1(path);
    }
    status_message_ =
        "Disc loaded: " + std::filesystem::path(path).filename().string();
    if (!system_.bios().loaded()) {
        status_message_ += " (load a BIOS to boot it)";
        return true;
    }
    // The BIOS only looks at the drive during startup, so restart it.
    reset_core();
    return start_bios();
}

bool Ps2App::start_ps1(const std::string& disc_path) {
    if (ps1_bios_path_.empty()) {
        ps1_bios_path_ = open_bios_dialog(
            "Select a PlayStation 1 BIOS (e.g. SCPH1001.BIN)");
        if (ps1_bios_path_.empty()) {
            status_message_ = "PS1 disc needs a PS1 BIOS (--ps1-bios <path>)";
            return false;
        }
    }
    emulation_running_ = false;
    std::string error;
    if (!ps1_->start(ps1_bios_path_, disc_path, error)) {
        status_message_ = error;
        ps1_bios_path_.clear();
        return false;
    }
    ps1_->set_speed(1.0);
    ps1_speed_index_ = 0;
    status_message_ = "PS1 disc running on the PS1 core: " +
        std::filesystem::path(disc_path).filename().string();
    return true;
}

void Ps2App::stop_ps1() {
    ps1_->stop();
    ps1_width_ = ps1_height_ = 0;
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
    // The mechacon clock runs in GMT+9; the BIOS applies the user's zone.
    system_.cdvd().set_clock(static_cast<u64>(
        std::time(nullptr) - 946684800 + 9 * 3600));
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

void Ps2App::emulation_thread_main() {
    using clock = std::chrono::steady_clock;
    // The EE retires one instruction per cycle here, so 294.912 MHz of
    // retired instructions is real time. Skipped BIOS loops count too.
    constexpr double kEeHz = 294'912'000.0;
    // ~1 ms of host time per slice so the UI can take the core between them.
    constexpr u64 kChunkInstructions = 100'000;
    constexpr double kMaxLeadSeconds = 0.002;
    // Never chase a debt for longer than about one frame, so a slow stretch
    // is not followed by a burst above 100%.
    constexpr double kMaxLagSeconds = 0.016;

    clock::time_point base_time{};
    u64 base_instructions = 0;
    std::string error;

    while (!emu_stop_.load(std::memory_order_acquire)) {
        if (ui_waiting_.load(std::memory_order_acquire)) {
            std::this_thread::yield();
            continue;
        }

        std::unique_lock<std::mutex> lock(core_mutex_);
        if (!emulation_running_ ||
            !system_.bios_started() ||
            system_.halted()) {
            base_time = {};
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        const u64 total = system_.ee().state().instructions_executed;
        const auto now = clock::now();
        if (base_time == clock::time_point{} ||
            !limit_speed_.load(std::memory_order_acquire) ||
            turbo_.load(std::memory_order_acquire)) {
            base_time = now;
            base_instructions = total;
        } else {
            const double emulated =
                static_cast<double>(total - base_instructions) / kEeHz;
            const double wall =
                std::chrono::duration<double>(now - base_time).count();
            if (emulated > wall + kMaxLeadSeconds) {
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            // Too slow to keep up: restart the clock rather than chase a debt.
            if (wall > emulated + kMaxLagSeconds) {
                base_time = now;
                base_instructions = total;
            }
        }

        const u64 ran = system_.run_ee(kChunkInstructions, error);

        if (!error.empty()) {
            emulation_running_ = false;
            status_message_ = "Execution stopped: " + error;
            error.clear();
        } else if (ran == 0) {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void Ps2App::update_emulation() {
    if (!emulation_running_ ||
        !system_.bios_started() ||
        system_.halted()) {
        if (system_.halted() && emulation_running_) {
            emulation_running_ = false;
            status_message_ = "Execution halted: " + system_.halt_reason();
        }
        return;
    }

    const auto sample_time = std::chrono::steady_clock::now();
    const auto sample_seconds =
        std::chrono::duration<double>(sample_time - speed_sample_time_).count();
    if (sample_seconds >= 0.5) {
        constexpr double kNtscFieldsPerSecond = 59.94;

        const u64 instructions =
            system_.ee().state().instructions_executed;
        const u64 fields =
            system_.video_fields_started();

        ee_instructions_per_second_ =
            static_cast<double>(
                instructions - speed_sample_instructions_) /
            sample_seconds;
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

    if (system_.iop_halted()) {
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

// ---------------------------------------------------------------------------
// VibeStation 2 frontend host

void Ps2App::set_developer_view(bool enabled) {
    if (!frontend_) enabled = true;
    developer_view_ = enabled;
    if (window_ != nullptr) {
        SDL_SetWindowTitle(window_, enabled ? kDeveloperTitle : kFrontendTitle);
    }
    if (!frontend_) return;
    if (enabled) frontend_->on_hidden();
    else frontend_->on_shown();
}

bool Ps2App::bios_loaded() const { return system_.bios().loaded(); }
std::string Ps2App::bios_romver() const { return system_.bios().romver(); }
std::string Ps2App::bios_path() const { return bios_path_input_.data(); }
bool Ps2App::load_bios(const std::string& path) { return load_bios_from_path(path); }
std::string Ps2App::pick_bios_file() { return open_bios_dialog(); }

std::string Ps2App::pick_folder(const char* title) {
#ifdef _WIN32
    std::string result;
    const HRESULT com = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    IFileOpenDialog* dialog = nullptr;
    if (SUCCEEDED(CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER,
                                   IID_PPV_ARGS(&dialog)))) {
        DWORD options = 0;
        dialog->GetOptions(&options);
        dialog->SetOptions(options | FOS_PICKFOLDERS | FOS_FORCEFILESYSTEM);
        const std::wstring wide_title(title, title + std::strlen(title));
        dialog->SetTitle(wide_title.c_str());
        IShellItem* item = nullptr;
        if (SUCCEEDED(dialog->Show(nullptr)) && SUCCEEDED(dialog->GetResult(&item))) {
            PWSTR path = nullptr;
            if (SUCCEEDED(item->GetDisplayName(SIGDN_FILESYSPATH, &path))) {
                result = std::filesystem::path(path).string();
                CoTaskMemFree(path);
            }
            item->Release();
        }
        dialog->Release();
    }
    if (SUCCEEDED(com)) CoUninitialize();
    return result;
#else
    (void)title;
    status_message_ = "The folder picker is currently Windows-only.";
    return {};
#endif
}

bool Ps2App::start_bios_session() {
    // Start Emulation boots into the BIOS menu, so take out any disc.
    stop_ps1();
    if (system_.cdvd().has_disc()) system_.eject_disc();
    reset_core();
    return start_bios();
}

bool Ps2App::boot_disc(const std::string& path) { return load_disc_from_path(path); }

bool Ps2App::session_active() const {
    return ps1_->active() || (system_.bios_started() && !system_.halted());
}

bool Ps2App::session_running() const {
    return ps1_->active() || (emulation_running_ && session_active());
}

void Ps2App::pause_session() {
    // The PS1 core has no pause; it keeps running behind the menu.
    if (emulation_running_) {
        emulation_running_ = false;
        status_message_ = "Paused";
    }
}

void Ps2App::resume_session() {
    if (!system_.bios_started() || system_.halted()) return;
    emulation_running_ = true;
    speed_sample_time_ = std::chrono::steady_clock::now();
    speed_sample_instructions_ = system_.ee().state().instructions_executed;
    speed_sample_fields_ = system_.video_fields_started();
    status_message_ = "Running";
}

void Ps2App::stop_session() {
    stop_ps1();
    reset_core();
    status_message_ = "Emulation stopped";
}

vs2::GameView Ps2App::game_view() const {
    vs2::GameView view{};
    if (ps1_->active()) {
        if (ps1_texture_ != 0 && ps1_width_ > 0) {
            view = {ps1_texture_, ps1_width_, ps1_height_, true};
        }
        return view;
    }
    if (display_texture_ != 0 && display_texture_width_ != 0) {
        view = {display_texture_, static_cast<int>(display_texture_width_),
                static_cast<int>(display_texture_height_), false};
    }
    return view;
}

double Ps2App::speed_percent() const { return emulation_speed_percent_; }
double Ps2App::frames_per_second() const { return guest_frames_per_second_; }
std::string Ps2App::status_message() const { return status_message_; }

void Ps2App::set_turbo(bool active) { turbo_.store(active, std::memory_order_release); }

bool Ps2App::save_snapshot() {
    std::vector<std::uint32_t> frame;
    int width = 0;
    int height = 0;
    if (ps1_->active()) {
        frame = ps1_frame_;
        width = ps1_width_;
        height = ps1_height_;
    } else {
        const auto& display = system_.gs_display();
        const auto display_lock = display.lock();
        if (display.valid()) {
            frame = display.rgba8();
            width = static_cast<int>(display.width());
            height = static_cast<int>(display.height());
        }
    }
    if (width <= 0 || height <= 0 ||
        frame.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
        status_message_ = "No picture to save yet.";
        return false;
    }

    std::error_code ec;
    const std::filesystem::path dir = std::filesystem::current_path(ec) / "snapshots";
    std::filesystem::create_directories(dir, ec);
    const std::time_t now = std::time(nullptr);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char stamp[32] = {};
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);
    std::filesystem::path path = dir / ("ps2_snapshot_" + std::string(stamp) + ".png");
    for (int n = 1; std::filesystem::exists(path, ec); ++n) {
        path = dir / ("ps2_snapshot_" + std::string(stamp) + "_" + std::to_string(n) + ".png");
    }

    const std::vector<std::uint8_t> png = encode_png(frame, width, height);
    FILE* file = nullptr;
#ifdef _WIN32
    _wfopen_s(&file, path.c_str(), L"wb");
#else
    file = std::fopen(path.c_str(), "wb");
#endif
    const bool written = file != nullptr &&
        std::fwrite(png.data(), 1, png.size(), file) == png.size();
    if (file != nullptr) std::fclose(file);
    status_message_ = written ? "Snapshot saved: " + path.filename().string()
                              : "Couldn't save the snapshot.";
    return written;
}

vs2::EeCore Ps2App::ee_core() const {
    if (system_.ee().dynarec_enabled()) return vs2::EeCore::Dynarec;
    if (system_.ee().jit_enabled()) return vs2::EeCore::Jit;
    return vs2::EeCore::Interpreter;
}

void Ps2App::set_ee_core(vs2::EeCore core) {
    // Each setter switches the other backend off when it enables its own.
    system_.ee().set_jit_enabled(core == vs2::EeCore::Jit);
    system_.ee().set_dynarec_enabled(core == vs2::EeCore::Dynarec);
}

bool Ps2App::gpu_gs_available() const { return gpu_gs_backend_ != nullptr; }
bool Ps2App::gpu_gs_enabled() const { return gpu_gs_enabled_ && gpu_gs_backend_ != nullptr; }
bool Ps2App::limit_speed() const { return limit_speed_.load(std::memory_order_acquire); }
void Ps2App::set_limit_speed(bool enabled) { limit_speed_.store(enabled, std::memory_order_release); }
int Ps2App::display_filter() const { return display_filter_; }
void Ps2App::set_display_filter(int filter) { display_filter_ = filter == 0 ? 0 : 1; }
bool Ps2App::lag_stutter() const { return lag_stutter_enabled_.load(std::memory_order_acquire); }

void Ps2App::set_lag_stutter(bool enabled) {
    // Same switch-over as the developer Settings panel.
    lag_stutter_enabled_.store(enabled, std::memory_order_release);
    if (audio_device_ != 0) SDL_ClearQueuedAudio(audio_device_);
    if (enabled) {
        std::lock_guard<std::mutex> lock(audio_history_mutex_);
        audio_history_play_frame_ = audio_history_write_frame_;
    }
}

void Ps2App::open_developer_view() { set_developer_view(true); }

void Ps2App::request_quit() {
    SDL_Event event{};
    event.type = SDL_QUIT;
    SDL_PushEvent(&event);
}

} // namespace ps2::ui
