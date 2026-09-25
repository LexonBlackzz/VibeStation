#include "app.h"
#include "platform/disc_path_utils.h"
#include "platform/memory_card_utils.h"
#include "ui/input_bindings.h"
#include "ui/output_resolution_utils.h"
#include "ui/screenshot_utils.h"
#include "ui/theme_settings.h"
#include "vibestation_version.h"
#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_opengl2.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <nlohmann/json.hpp>
#include <sstream>
#include <ctime>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#include <commdlg.h>
#include <shlobj.h>
#include <shobjidl.h>
#endif

namespace {
    constexpr const char* kAppConfigFileName = "vibestation_config.ini";
    float smooth_ui_value(float current, float target, float delta_seconds,
        float response_seconds = 0.18f) {
        if (current < 0.0f || delta_seconds <= 0.0f) {
            return target;
        }
        const float safe_response = std::max(response_seconds, 0.001f);
        const float alpha = 1.0f - std::exp(-delta_seconds / safe_response);
        return current + ((target - current) * alpha);
    }

    std::string trim_copy(const std::string& input) {
        const size_t begin = input.find_first_not_of(" \t\r\n");
        if (begin == std::string::npos) {
            return {};
        }
        const size_t end = input.find_last_not_of(" \t\r\n");
        return input.substr(begin, end - begin + 1);
    }

    int log_level_to_config_value(LogLevel level) {
        switch (level) {
        case LogLevel::Debug:
            return 0;
        case LogLevel::Info:
            return 1;
        case LogLevel::Warn:
            return 2;
        case LogLevel::Error:
            return 3;
        }
        return 1;
    }

    LogLevel parse_log_level_config(const std::string& value, LogLevel fallback) {
        if (value == "0" || value == "debug" || value == "DEBUG") {
            return LogLevel::Debug;
        }
        if (value == "1" || value == "info" || value == "INFO") {
            return LogLevel::Info;
        }
        if (value == "2" || value == "warn" || value == "warning" ||
            value == "WARN" || value == "WARNING") {
            return LogLevel::Warn;
        }
        if (value == "3" || value == "error" || value == "ERROR") {
            return LogLevel::Error;
        }
        return fallback;
    }

    int normalize_turbo_speed_percent(int percent) {
        if (percent <= 0) {
            return 0;
        }
        return (percent >= 400) ? 400 : 200;
    }

    double turbo_speed_multiplier_from_percent(int percent) {
        const int normalized = normalize_turbo_speed_percent(percent);
        if (normalized == 0) {
            return 0.0;
        }
        return static_cast<double>(normalized) / 100.0;
    }

    int normalize_slowdown_speed_percent(int percent) {
        return std::max(10, std::min(100, percent));
    }

    double slowdown_speed_multiplier_from_percent(int percent) {
        return static_cast<double>(normalize_slowdown_speed_percent(percent)) / 100.0;
    }

    constexpr double kSpuDiagnosticSpeedMultiplier = 0.83;
    constexpr double kSpuDiagnosticReverbMixMultiplier = 4.00;

}

void App::set_input_recorder_config(const InputRecorder::Config& config) {
    input_recorder_config_ = config;
    input_movie_cli_pending_ =
        config.recording_enabled() || config.playback_enabled();
    if (!config.record_path.empty()) {
        std::snprintf(input_movie_record_path_, sizeof(input_movie_record_path_),
            "%s", config.record_path.c_str());
    }
    if (!config.playback_path.empty()) {
        std::snprintf(input_movie_playback_path_, sizeof(input_movie_playback_path_),
            "%s", config.playback_path.c_str());
    }
    input_movie_stop_at_eof_ =
        config.end_behavior == InputRecorder::PlaybackEndBehavior::Stop;
    input_movie_loop_ =
        config.end_behavior == InputRecorder::PlaybackEndBehavior::Loop;
    input_recorder_.set_config(config);
}

bool App::init() {
    printf("[App::init] Initializing SDL...\n");
    fflush(stdout);
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER | SDL_INIT_AUDIO) !=
        0) {
        LOG_ERROR("SDL_Init failed: %s", SDL_GetError());
        printf("[App::init] SDL_Init FAILED: %s\n", SDL_GetError());
        fflush(stdout);
        return false;
    }
    printf("[App::init] SDL OK\n");
    fflush(stdout);

    if (!input_) {
        input_ = std::make_unique<InputManager>();
    }
    load_persistent_config();
    if (!discord_presence_) {
        discord_presence_ = std::make_unique<DiscordPresence>();
    }
    sync_discord_presence_config();

    struct GlContextAttempt {
        int major;
        int minor;
        int profile;
        const char* imgui_glsl;
        const char* label;
        bool use_imgui_opengl2_backend;
    };

    const GlContextAttempt attempts[] = {
        {3, 3, SDL_GL_CONTEXT_PROFILE_CORE, "#version 330", "OpenGL 3.3 Core",
         false},
        {3, 2, SDL_GL_CONTEXT_PROFILE_CORE, "#version 150", "OpenGL 3.2 Core",
         false},
        {2, 1, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY, "#version 120",
         "OpenGL 2.1 Compatibility", true},
    };

    bool context_ready = false;
    for (const GlContextAttempt& attempt : attempts) {
        SDL_GL_ResetAttributes();
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, attempt.major);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, attempt.minor);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, attempt.profile);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

        printf("[App::init] Creating window/context (%s)...\n", attempt.label);
        fflush(stdout);
        window_ = SDL_CreateWindow(
            "VibeStation - PS1 Emulator", SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED, 1280, 800,
            SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
        if (!window_) {
            LOG_WARN("SDL_CreateWindow failed for %s: %s", attempt.label,
                SDL_GetError());
            printf("[App::init] SDL_CreateWindow failed for %s: %s\n", attempt.label,
                SDL_GetError());
            fflush(stdout);
            continue;
        }

        gl_context_ = SDL_GL_CreateContext(window_);
        if (!gl_context_) {
            LOG_WARN("SDL_GL_CreateContext failed for %s: %s", attempt.label,
                SDL_GetError());
            printf("[App::init] GL Context failed for %s: %s\n", attempt.label,
                SDL_GetError());
            fflush(stdout);
            SDL_DestroyWindow(window_);
            window_ = nullptr;
            continue;
        }

        SDL_GL_MakeCurrent(window_, gl_context_);
        SDL_GL_SetSwapInterval(config_vsync_ ? 1 : 0);
        imgui_glsl_version_ = attempt.imgui_glsl;
        use_imgui_opengl2_backend_ = attempt.use_imgui_opengl2_backend;
        context_ready = true;

        const GLubyte* gl_version = glGetString(GL_VERSION);
        printf("[App::init] GL Context OK (%s, driver: %s)\n", attempt.label,
            gl_version ? reinterpret_cast<const char*>(gl_version) : "Unknown");
        fflush(stdout);
        break;
    }

    if (!context_ready) {
        printf("[App::init] GL Context FAILED: No compatible OpenGL context found.\n");
        fflush(stdout);
        return false;
    }

    // Init Dear ImGui
    printf("[App::init] Initializing ImGui...\n");
    fflush(stdout);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = "imgui.ini";

    // Bake the Definitive UI font atlas at the sizes we actually render.
    // This avoids scaling ImGui's tiny default font up to large text sizes.
    initialize_definitive_ui_fonts();

    // Style â€” Dark with custom colors
    ImGui::StyleColorsDark();
    ui_theme::ensure_theme_settings_initialized();
    ui_theme::apply_theme_style(ImGui::GetStyle());
    ui_theme::register_theme_settings_handler();
    if (io.IniFilename != nullptr && io.IniFilename[0] != '\0') {
        ImGui::LoadIniSettingsFromDisk(io.IniFilename);
    }

    // Custom color palette â€” deep purple/blue
    printf("[App::init] ImGui styled\n");
    fflush(stdout);

    if (!ImGui_ImplSDL2_InitForOpenGL(window_, gl_context_)) {
        printf("[App::init] ImGui SDL backend FAILED: %s\n", SDL_GetError());
        fflush(stdout);
        return false;
    }
    if (use_imgui_opengl2_backend_) {
        if (!ImGui_ImplOpenGL2_Init()) {
            printf("[App::init] ImGui OpenGL2 backend FAILED\n");
            fflush(stdout);
            return false;
        }
    }
    else {
        if (!ImGui_ImplOpenGL3_Init(imgui_glsl_version_)) {
            printf("[App::init] ImGui OpenGL3 backend FAILED (GLSL=%s)\n",
                imgui_glsl_version_);
            fflush(stdout);
            return false;
        }
    }
    printf("[App::init] ImGui backends OK (%s)\n",
        use_imgui_opengl2_backend_ ? "OpenGL2" : "OpenGL3");
    fflush(stdout);

    printf("[App::init] Window + UI ready. Deferring emulator runtime init.\n");
    fflush(stdout);
    LOG_INFO("VibeStation initialized successfully!");
    return true;
}

bool App::init_runtime() {
    if (runtime_ready_) {
        return true;
    }

    printf("[App::init_runtime] Initializing emulator runtime...\n");
    fflush(stdout);

    system_ = std::make_unique<System>();
    system_->set_input_recorder(&input_recorder_);
    renderer_ = std::make_unique<Renderer>();
    if (!input_) {
        input_ = std::make_unique<InputManager>();
    }

    try_autoload_bios_from_config();

    if (!renderer_->init(window_)) {
        LOG_ERROR("Renderer init failed");
        printf("[App::init_runtime] Renderer FAILED\n");
        fflush(stdout);
        renderer_.reset();
        input_.reset();
        system_.reset();
        return false;
    }
    renderer_->set_bilinear_filtering(g_bilinear_filtering);

    if (!emu_runner_.start(system_.get())) {
        LOG_ERROR("EmuRunner failed to start");
        printf("[App::init_runtime] EmuRunner FAILED\n");
        fflush(stdout);
        renderer_.reset();
        input_.reset();
        system_.reset();
        return false;
    }

    if (!frame_presentation_worker_.start(&emu_runner_)) {
        LOG_ERROR("FramePresentationWorker failed to start");
        printf("[App::init_runtime] Presentation worker FAILED\n");
        fflush(stdout);
        emu_runner_.stop();
        renderer_.reset();
        input_.reset();
        system_.reset();
        return false;
    }

    emu_runner_.set_speed(1.0);
    apply_speed_override();
    apply_memory_card_settings(false);

    runtime_ready_ = true;
    printf("[App::init_runtime] Runtime ready\n");
    fflush(stdout);
    return true;
}

bool App::launch_disc_from_cli(const std::string& bios_path,
    const std::string& disc_path, bool direct_boot) {
    if (!init_runtime()) {
        status_message_ = "CLI launch failed: runtime initialization failed.";
        return false;
    }

    if (!bios_path.empty()) {
        if (!system_->load_bios(bios_path)) {
            status_message_ = "CLI launch failed: BIOS load failed.";
            LOG_ERROR("CLI launch: failed to load BIOS: %s", bios_path.c_str());
            return false;
        }
        bios_path_ = bios_path;
    }

    std::string bin_path;
    std::string cue_path;
    std::string error;
    if (!resolve_disc_paths(disc_path, bin_path, cue_path, error)) {
        status_message_ = "CLI launch failed: " + error;
        LOG_ERROR("CLI launch: %s", error.c_str());
        return false;
    }

    const bool previous_direct_boot = config_direct_disc_boot_;
    config_direct_disc_boot_ = direct_boot;
    if (!load_disc_from_ui(bin_path, cue_path)) {
        config_direct_disc_boot_ = previous_direct_boot;
        LOG_ERROR("CLI launch: failed to select disc: %s", disc_path.c_str());
        return false;
    }

    if (!boot_disc_from_ui()) {
        config_direct_disc_boot_ = previous_direct_boot;
        LOG_ERROR("CLI launch: failed to boot disc: %s", disc_path.c_str());
        return false;
    }

    status_message_ = "CLI launch: " +
        std::filesystem::path(disc_path).filename().string();
    return true;
}

double App::current_speed_override() const {
    if (turbo_hold_active_ || gameplay_toolbar_turbo_active_) {
        return turbo_speed_multiplier_from_percent(config_turbo_speed_percent_);
    }
    if (slowdown_hold_active_) {
        return slowdown_speed_multiplier_from_percent(config_slowdown_speed_percent_);
    }
    if (config_spu_diagnostic_mode_) {
        return kSpuDiagnosticSpeedMultiplier;
    }
    return 1.0;
}

double App::current_effective_speed_multiplier() const {
    if (!has_started_emulation_ || system_ == nullptr) {
        return 0.0;
    }

    const double target_fps = system_->target_fps();
    const double core_frame_ms = std::max(0.0, runtime_snapshot_.core_frame_ms);
    if (target_fps <= 0.0 || core_frame_ms <= 0.0) {
        return 0.0;
    }

    const double baseline_frame_budget_ms = 1000.0 / target_fps;
    if (baseline_frame_budget_ms <= 0.0) {
        return 0.0;
    }
    return baseline_frame_budget_ms / core_frame_ms;
}

double App::current_emulation_slowdown_percent() const {
    if (!has_started_emulation_ || system_ == nullptr) {
        return 0.0;
    }

    const double target_fps = system_->target_fps();
    const double raw_speed = emu_runner_.speed();
    if (target_fps <= 0.0 || raw_speed <= 0.0) {
        return 0.0;
    }
    const double speed = std::max(0.25, raw_speed);

    const double requested_frame_budget_ms = 1000.0 / (target_fps * speed);
    const double core_frame_ms = std::max(0.0, runtime_snapshot_.core_frame_ms);
    if (requested_frame_budget_ms <= 0.0 || core_frame_ms <= requested_frame_budget_ms) {
        return 0.0;
    }

    const double sustained_speed_ratio = requested_frame_budget_ms / core_frame_ms;
    return std::clamp((1.0 - sustained_speed_ratio) * 100.0, 0.0, 100.0);
}

void App::apply_speed_override() {
    emu_runner_.set_speed(current_speed_override());
}

void App::run() {
    if (!init_runtime()) {
        return;
    }

    // Thread roles:
    //   UI/OpenGL presentation = normal priority (this thread)
    //   PS1 emulation          = high priority (EmuRunner)
    //   frame scaling/ambient  = low priority (FramePresentationWorker)
    //   host audio playback    = SDL's dedicated audio callback thread
    SDL_SetThreadPriority(SDL_THREAD_PRIORITY_NORMAL);

    bool quit = false;
    last_fps_time_ = SDL_GetTicks();
    const u64 perf_freq = SDL_GetPerformanceFrequency();
    const double target_frame_sec = 1.0 / 60.0;

    while (!quit) {
        const u64 loop_start_counter = SDL_GetPerformanceCounter();
        process_events(quit);
        update();

        // Emulation, frame post-processing, and UI rendering now run on
        // separate threads. The emulation worker publishes raw frames; this
        // main/UI thread only moves them into the presentation mailbox.
        FrameSnapshot frame;
        if (emu_runner_.consume_latest_frame(frame)) {
            game_frame_count_++;

            int output_width = 320;
            int output_height = 240;
            output_resolution_dimensions(
                g_output_resolution_mode,
                output_width,
                output_height);

            const bool turbo_resolution_clamp =
                (turbo_hold_active_ ||
                    gameplay_toolbar_turbo_active_) &&
                g_output_resolution_mode !=
                    OutputResolutionMode::R320x240 &&
                (frame.width > 320 ||
                    frame.height > 240);

            if (turbo_resolution_clamp) {
                output_width = 320;
                output_height = 240;
            }

            frame_presentation_worker_.submit(
                std::move(frame),
                output_width,
                output_height);
        }

        // OpenGL texture upload remains on the window/context thread, but all
        // CPU scaling and Ambilight analysis is already complete here.
        PreparedUiFrame prepared_frame;
        if (frame_presentation_worker_.consume_latest(
                prepared_frame)) {
            renderer_->upload_frame(
                prepared_frame.rgba,
                prepared_frame.width,
                prepared_frame.height);

            latest_frame_width_ =
                prepared_frame.width;
            latest_frame_height_ =
                prepared_frame.height;

            latest_frame_rgba_.swap(
                prepared_frame.rgba);

            frame_presentation_worker_.recycle_consumed(
                std::move(prepared_frame));
        }
        runtime_snapshot_ = emu_runner_.runtime_snapshot();
        u32 now_ms = SDL_GetTicks();
        if (has_started_emulation_ && system_ != nullptr) {
            const double target_fps = system_->target_fps();
            const u64 completed_frames = emu_runner_.completed_frame_count();
            if (target_fps > 0.0) {
                if (last_emulation_speed_sample_ms_ == 0 ||
                    completed_frames < last_emulation_speed_sample_frame_) {
                    last_emulation_speed_sample_ms_ = now_ms;
                    last_emulation_speed_sample_frame_ = completed_frames;
                }
                else {
                    const u32 elapsed_ms = now_ms - last_emulation_speed_sample_ms_;
                    if (elapsed_ms >= 250) {
                        const u64 frame_delta =
                            completed_frames - last_emulation_speed_sample_frame_;
                        measured_emulation_speed_multiplier_ =
                            ((static_cast<double>(frame_delta) * 1000.0) /
                                static_cast<double>(elapsed_ms)) / target_fps;
                        last_emulation_speed_sample_ms_ = now_ms;
                        last_emulation_speed_sample_frame_ = completed_frames;
                    }
                }
            }
            else {
                measured_emulation_speed_multiplier_ = 0.0;
                last_emulation_speed_sample_ms_ = now_ms;
                last_emulation_speed_sample_frame_ = completed_frames;
            }
        }
        else {
            measured_emulation_speed_multiplier_ = 0.0;
            last_emulation_speed_sample_ms_ = 0;
            last_emulation_speed_sample_frame_ = 0;
        }
        push_performance_history_sample();
        update_discord_presence();
        if (discord_presence_) {
            discord_presence_->tick();
        }
        emu_runner_.set_vram_debug_capture_enabled(show_vram_);

        if (show_vram_ ||
            (!emu_runner_.is_running() &&
                (now_ms - last_vram_update_ms_) >= 1000)) {
            update_vram_debug_texture();
            last_vram_update_ms_ = now_ms;
        }

        if (!emu_runner_.is_running() && emu_runner_.playback_stopped_at_eof()) {
            fprintf(stdout, "[App] Playback finished, requesting exit.\n");
            fflush(stdout);
            SDL_Event quit_event;
            quit_event.type = SDL_QUIT;
            SDL_PushEvent(&quit_event);
        }

        // Start ImGui frame
        if (use_imgui_opengl2_backend_) {
            ImGui_ImplOpenGL2_NewFrame();
        }
        else {
            ImGui_ImplOpenGL3_NewFrame();
        }
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        render_ui();

        // Render
        ImGui::Render();

        int w, h;
        SDL_GetWindowSize(window_, &w, &h);
        glViewport(0, 0, w, h);
        const ImVec4& clear_color = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        glClearColor(clear_color.x, clear_color.y, clear_color.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        const auto render_start = std::chrono::high_resolution_clock::now();
        if (use_imgui_opengl2_backend_) {
            ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
        }
        else {
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }
        const auto swap_start = std::chrono::high_resolution_clock::now();
        SDL_GL_SwapWindow(window_);
        const auto frame_end = std::chrono::high_resolution_clock::now();
        render_ms_ =
            std::chrono::duration<double, std::milli>(swap_start - render_start)
            .count();
        swap_ms_ =
            std::chrono::duration<double, std::milli>(frame_end - swap_start)
            .count();
        present_ms_ = render_ms_ + swap_ms_;

        // FPS counter
        video_frame_count_++;
        u32 now = SDL_GetTicks();
        if (now - last_fps_time_ >= 1000) {
            video_fps_ =
                static_cast<float>(video_frame_count_) * 1000.0f / (now - last_fps_time_);
            game_fps_ =
                static_cast<float>(game_frame_count_) * 1000.0f / (now - last_fps_time_);
            video_frame_count_ = 0;
            game_frame_count_ = 0;
            last_fps_time_ = now;

            char title[128];
            snprintf(title, sizeof(title), "VibeStation - PS1 Emulator | %.1f Game FPS | %.1f Video FPS",
                game_fps_, video_fps_);
            SDL_SetWindowTitle(window_, title);
        }

        if (!config_vsync_) {
            const u64 loop_end_counter = SDL_GetPerformanceCounter();
            const double elapsed_sec =
                static_cast<double>(loop_end_counter - loop_start_counter) /
                static_cast<double>(perf_freq);
            if (elapsed_sec < target_frame_sec) {
                const double remain_sec = target_frame_sec - elapsed_sec;
                if (remain_sec > 0.002) {
                    const u32 delay_ms =
                        static_cast<u32>((remain_sec - 0.001) * 1000.0);
                    if (delay_ms > 0) {
                        SDL_Delay(delay_ms);
                    }
                }
                while (true) {
                    const u64 now_counter = SDL_GetPerformanceCounter();
                    const double total_sec =
                        static_cast<double>(now_counter - loop_start_counter) /
                        static_cast<double>(perf_freq);
                    if (total_sec >= target_frame_sec) {
                        break;
                    }
                    SDL_Delay(0);
                }
            }
        }
    }
}

void App::process_events(bool& quit) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);

        if (event.type == SDL_QUIT) {
            quit = true;
        }
        if (event.type == SDL_WINDOWEVENT &&
            event.window.event == SDL_WINDOWEVENT_FOCUS_LOST) {
            if (turbo_hold_active_ || gameplay_toolbar_turbo_active_) {
                turbo_hold_active_ = false;
                gameplay_toolbar_turbo_active_ = false;
                apply_speed_override();
            }
            if (slowdown_hold_active_) {
                slowdown_hold_active_ = false;
                apply_speed_override();
            }
            gameplay_toolbar_rewind_active_ = false;
            if (emu_runner_.is_rewind_active()) {
                emu_runner_.set_rewind_active(false);
            }
        }
        if (pending_bind_index_ >= 0 && event.type == SDL_KEYDOWN &&
            !event.key.repeat) {
            const int bind_index = pending_bind_index_;
            const SDL_Scancode scancode = event.key.keysym.scancode;
            pending_bind_index_ = -1;
            if (scancode == SDL_SCANCODE_ESCAPE) {
                status_message_ = "Keyboard rebinding canceled";
            }
            else {
                input_->set_key_binding(scancode, kKeyboardBindEntries[bind_index].button);
                save_persistent_config();
                status_message_ = std::string("Bound ") +
                    kKeyboardBindEntries[bind_index].label + " to " +
                    SDL_GetScancodeName(scancode);
            }
            continue;
        }
        if (event.type == SDL_KEYDOWN && !event.key.repeat &&
            event.key.keysym.sym == SDLK_BACKSPACE) {
            const u16 mods = static_cast<u16>(event.key.keysym.mod);
            const bool ctrl = (mods & KMOD_CTRL) != 0;
            const bool alt = (mods & KMOD_ALT) != 0;
            const bool gui = (mods & KMOD_GUI) != 0;
            if (!ctrl && !alt && !gui) {
                turbo_hold_active_ = true;
                apply_speed_override();
                continue;
            }
        }
        if (event.type == SDL_KEYUP && event.key.keysym.sym == SDLK_BACKSPACE) {
            if (turbo_hold_active_) {
                turbo_hold_active_ = false;
                apply_speed_override();
            }
            continue;
        }
        if (event.type == SDL_KEYDOWN && !event.key.repeat &&
            event.key.keysym.sym == SDLK_RSHIFT) {
            slowdown_hold_active_ = true;
            apply_speed_override();
            continue;
        }
        if (event.type == SDL_KEYUP && event.key.keysym.sym == SDLK_RSHIFT) {
            if (slowdown_hold_active_) {
                slowdown_hold_active_ = false;
                apply_speed_override();
            }
            continue;
        }
        if (event.type == SDL_KEYDOWN && !event.key.repeat &&
            event.key.keysym.sym == SDLK_RCTRL) {
            if (config_rewind_enabled_ && emu_runner_.is_running()) {
                emu_runner_.set_rewind_active(true);
            }
            continue;
        }
        if (event.type == SDL_KEYUP && event.key.keysym.sym == SDLK_RCTRL) {
            if (emu_runner_.is_rewind_active()) {
                emu_runner_.set_rewind_active(false);
            }
            continue;
        }
        if (event.type == SDL_KEYDOWN && !event.key.repeat) {
            const SDL_Keycode key = event.key.keysym.sym;
            const u16 mods = static_cast<u16>(event.key.keysym.mod);
            const bool ctrl = (mods & KMOD_CTRL) != 0;
            const bool alt = (mods & KMOD_ALT) != 0;
            const bool gui = (mods & KMOD_GUI) != 0;
            const bool no_mod = !ctrl && !alt && !gui;

            if (ctrl && key == SDLK_b) {
                std::string path = open_file_dialog(
                    "BIOS Files (*.bin)\0*.bin\0All Files\0*.*\0", "Select PS1 BIOS");
                if (!path.empty()) {
                    emu_runner_.pause_and_wait_idle();
                    disable_ram_reaper_mode();
                    disable_gpu_reaper_mode();
                    disable_sound_reaper_mode();
                    if (system_->load_bios(path)) {
                        bios_path_ = path;
                        save_persistent_config();
                        has_started_emulation_ = false;
                        set_grim_reaper_mode(false);
                        status_message_ = "BIOS loaded: " + system_->bios().get_info();
                    }
                    else {
                        status_message_ = "Failed to load BIOS!";
                    }
                }
            }
            else if (ctrl && key == SDLK_o) {
                std::string path = open_file_dialog(
                    "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
                    "Select PS1 Game");
                if (!path.empty()) {
                    std::string bin;
                    std::string cue;
                    std::string error;
                    if (!resolve_disc_paths(path, bin, cue, error)) {
                        status_message_ = error;
                    }
                    else {
                        load_disc_from_ui(bin, cue);
                    }
                }
            }
            else if (ctrl && key == SDLK_COMMA) {
                if (!has_started_emulation_ &&
                    !definitive_detailed_settings_) {
                    if (show_settings_) {
                        close_definitive_settings();
                    }
                    else {
                        open_definitive_settings();
                    }
                }
                else {
                    show_settings_ = !show_settings_;
                }
            }
            else if (ctrl && key == SDLK_F5) {
                if (system_->bios_loaded() && !emu_runner_.is_running() &&
                    (system_->disc_loaded() || !game_bin_path_.empty())) {
                    boot_disc_from_ui();
                }
            }
            else if (ctrl && key == SDLK_e) {
                if (system_->bios_loaded() && !emu_runner_.is_running() &&
                    (system_->disc_loaded() || !game_bin_path_.empty())) {
                    unload_disc_from_ui();
                }
            }
            else if (no_mod && key == SDLK_F5) {
                if (system_->bios_loaded() && !emu_runner_.is_running()) {
                    if (has_started_emulation_) {
                        emu_runner_.set_running(true);
                        status_message_ = "Emulation resumed";
                    }
                    else {
                        start_bios_from_ui();
                    }
                }
            }
            else if (no_mod && key == SDLK_F6) {
                if (emu_runner_.is_running()) {
                    emu_runner_.pause_and_wait_idle();
                    status_message_ = "Emulation paused";
                }
            }
            else if (no_mod && key == SDLK_F7) {
                if (system_->bios_loaded() && has_started_emulation_) {
                    emu_runner_.pause_and_wait_idle();
                    disable_ram_reaper_mode();
                    disable_gpu_reaper_mode();
                    disable_sound_reaper_mode();
                    has_started_emulation_ = false;
                    status_message_ = "Emulation stopped";
                }
            }
            else if (no_mod && key == SDLK_F8) {
                save_snapshot_png();
            }
            else if (no_mod && key == SDLK_F9) {
                show_debug_cpu_ = !show_debug_cpu_;
            }
            else if (no_mod && key == SDLK_F10) {
                show_vram_ = !show_vram_;
            }
            else if (no_mod && key == SDLK_F11) {
                show_perf_ = !show_perf_;
            }
            else if (no_mod && key == SDLK_F12) {
                show_perf_profiler_ = !show_perf_profiler_;
                g_profile_detailed_timing = show_perf_profiler_;
            }
        }

        const ImGuiIO& io = ImGui::GetIO();
        if (should_route_keyboard_to_emu(event, io)) {
            input_->process_event(event);
        }
        // Focus-loss must always reach input, otherwise a routed KEYDOWN can
        // remain latched when its KEYUP is consumed by the UI.
        if (event.type == SDL_WINDOWEVENT &&
            event.window.event == SDL_WINDOWEVENT_FOCUS_LOST) {
            input_->process_event(event);
        }
        // Always process gamepad events
        if (event.type == SDL_CONTROLLERDEVICEADDED ||
            event.type == SDL_CONTROLLERDEVICEREMOVED ||
            event.type == SDL_CONTROLLERBUTTONDOWN ||
            event.type == SDL_CONTROLLERBUTTONUP) {
            input_->process_event(event);
        }
    }
}

void App::update() {
    input_->update();
    if (system_) {
        const double reverb_mix =
            config_spu_diagnostic_mode_ ? kSpuDiagnosticReverbMixMultiplier : 1.0;
        system_->set_spu_reverb_mix_multiplier(reverb_mix);
        system_->set_spu_force_reverb(config_spu_diagnostic_mode_);
    }
    g_spu_force_audio_queue = slowdown_hold_active_ && !g_spu_enable_audio_queue;
    sync_ram_reaper_config();
    sync_gpu_reaper_config();
    sync_sound_reaper_config();

    // Push controller state into lock-free mailbox consumed by the emu thread.
    const u16 buttons = input_->controller().button_state();
    const u8 lx = input_->controller().lx();
    const u8 ly = input_->controller().ly();
    const u8 rx = input_->controller().rx();
    const u8 ry = input_->controller().ry();
    emu_runner_.set_input_state(buttons, lx, ly, rx, ry);

    // Keep input visible for paused-step workflows.
    if (!emu_runner_.is_running()) {
        system_->sio().set_button_state(buttons);
        system_->sio().set_analog_state(lx, ly, rx, ry);
    }

    last_button_state_ = buttons;
    emu_input_focused_ = emu_runner_.is_running() &&
        ((SDL_GetWindowFlags(window_) & SDL_WINDOW_INPUT_FOCUS) !=
            0);
    if (has_started_emulation_) {
        static constexpr u32 kUnderrunNoticeSamplePeriodMs = 1000;
        static constexpr u64 kUnderrunNoticeThreshold = 3;
        const u64 underruns = runtime_snapshot_.audio_queue.underrun_count;
        const u32 now_ms = SDL_GetTicks();
        if (underrun_notice_last_tick_ms_ == 0) {
            underrun_notice_last_tick_ms_ = now_ms;
            underrun_notice_last_events_ = underruns;
        }
        else if ((now_ms - underrun_notice_last_tick_ms_) >=
            kUnderrunNoticeSamplePeriodMs) {
            const u64 underrun_delta = underruns >= underrun_notice_last_events_
                ? underruns - underrun_notice_last_events_
                : underruns;
            const u32 bucket_value = static_cast<u32>(std::min<u64>(
                underrun_delta, std::numeric_limits<u32>::max()));
            if (underrun_notice_bucket_count_ < underrun_notice_buckets_.size()) {
                ++underrun_notice_bucket_count_;
            }
            else {
                underrun_notice_bucket_sum_ -=
                    underrun_notice_buckets_[underrun_notice_bucket_index_];
            }
            underrun_notice_buckets_[underrun_notice_bucket_index_] = bucket_value;
            underrun_notice_bucket_sum_ += bucket_value;
            underrun_notice_bucket_index_ =
                (underrun_notice_bucket_index_ + 1u) % underrun_notice_buckets_.size();
            show_fast_mode_notice_ =
                (underrun_notice_bucket_count_ >= underrun_notice_buckets_.size()) &&
                (underrun_notice_bucket_sum_ >= kUnderrunNoticeThreshold);
            underrun_notice_last_tick_ms_ = now_ms;
            underrun_notice_last_events_ = underruns;
        }

    }
    else {
        show_fast_mode_notice_ = false;
        underrun_notice_buckets_.fill(0);
        underrun_notice_bucket_index_ = 0;
        underrun_notice_bucket_count_ = 0;
        underrun_notice_bucket_sum_ = 0;
        underrun_notice_last_tick_ms_ = 0;
        underrun_notice_last_events_ = 0;
    }

    if (system_ != nullptr) {
        const u32 now_ms = SDL_GetTicks();
        const float delta_seconds =
            (last_audio_metrics_smooth_tick_ms_ > 0 && now_ms >= last_audio_metrics_smooth_tick_ms_)
            ? static_cast<float>(now_ms - last_audio_metrics_smooth_tick_ms_) / 1000.0f
            : 0.0f;
        last_audio_metrics_smooth_tick_ms_ = now_ms;

        const auto& rb = system_->spu().ring_buffer();
        const size_t avail = rb.available_samples();
        const size_t cap = rb.capacity_samples();
        const float fill_pct = cap > 0
            ? (static_cast<float>(avail) / static_cast<float>(cap)) * 100.0f
            : 0.0f;
        const float available_ms = static_cast<float>(
            (static_cast<double>(avail) /
                (static_cast<double>(AudioRingBuffer::DEFAULT_SAMPLE_RATE) *
                    static_cast<double>(AudioRingBuffer::DEFAULT_CHANNELS))) * 1000.0);
        const float queue_kb =
            static_cast<float>(
                runtime_snapshot_.audio_queue.queue_stereo_frames * 2u *
                sizeof(s16)) / 1024.0f;

        smoothed_audio_buffer_fill_pct_ =
            smooth_ui_value(smoothed_audio_buffer_fill_pct_, fill_pct, delta_seconds);
        smoothed_audio_buffer_available_ms_ =
            smooth_ui_value(smoothed_audio_buffer_available_ms_, available_ms, delta_seconds);
        smoothed_audio_queue_kb_ =
            smooth_ui_value(smoothed_audio_queue_kb_, queue_kb, delta_seconds);

        const bool playback_enabled =
            has_started_emulation_ && emu_runner_.is_running();
        bool slowdown_stutter_hint = false;
        if (playback_enabled && g_spu_enable_slowdown_stutter) {
            const double target_fps = system_->target_fps();
            const double frame_budget_ms =
                (target_fps > 0.0) ? (1000.0 / target_fps) : 0.0;
            const double core_ms = runtime_snapshot_.core_frame_ms;
            if (frame_budget_ms > 0.0 && core_ms > 0.0) {
                const double activate_ms = frame_budget_ms * 1.25;
                const double release_ms = frame_budget_ms * 1.10;
                slowdown_stutter_hint = slowdown_stutter_hint_active_
                    ? (core_ms >= release_ms)
                    : (core_ms >= activate_ms);
            }
        }
        slowdown_stutter_hint_active_ = slowdown_stutter_hint;
        system_->spu().set_lag_stutter_hint(slowdown_stutter_hint);
        system_->set_spu_host_playback_enabled(playback_enabled);
    }
    else {
        smoothed_audio_buffer_fill_pct_ = -1.0f;
        smoothed_audio_buffer_available_ms_ = -1.0f;
        smoothed_audio_queue_kb_ = -1.0f;
        slowdown_stutter_hint_active_ = false;
        last_audio_metrics_smooth_tick_ms_ = 0;
    }

    // ── Audio heartbeat: pull samples from the SPU ring buffer into SDL. ──
    // If the emulator is running smoothly, this pulls pristine audio.
    // If the SPU thread drops frames / the emulator hitches, the ring
    // buffer's read path automatically engages the Source Engine-style
    // stutter loop (repeating the last ~400 ms of audio).
    if (system_) {
        system_->spu().pump_audio_to_device();
    }
}

void App::render_ui() {
    // The definitive launcher owns the full viewport while idle. During
    // emulation, the gameplay screen uses its own floating toolbar instead of
    // the legacy ImGui main menu bar so the game image stays visually clean.

    // Main dockspace
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoBackground;
    if (has_started_emulation_) {
        flags |= ImGuiWindowFlags_NoNavFocus;
    }

    ImGui::Begin("DockSpace", nullptr, flags);
    ImGui::PopStyleVar(3);

    if (has_started_emulation_) {
        panel_emulator_screen();
    }
    else {
        panel_definitive_home();
    }
    ImGui::End();

    // Grim Reaper is a fixed right-side overlay so gameplay/launcher remains
    // visible and running underneath it.
    if (definitive_grim_reaper_active_) {
        panel_definitive_grim_reaper();
    }

    // Optional panels
    if (show_logging_) {
        show_settings_ = true;
        show_logging_ = false;
    }
    if (show_settings_) {
        if (!has_started_emulation_ && !definitive_detailed_settings_) {
            panel_definitive_settings();
        }
        else {
            panel_settings();
        }
    }
    // The legacy Grim Reaper window remains compiled for migration/debugging,
    // but normal navigation now uses the full-screen definitive section.
    if (show_about_)
        panel_about();
    if (show_debug_cpu_)
        panel_debug_cpu();
    if (show_vram_)
        panel_vram();
    if (show_perf_profiler_)
        panel_performance();
    if (show_sound_status_)
        panel_sound_status();
    if (show_bindings_config_)
        panel_bindings_config();
    if (show_fmv_diagnostics_)
        panel_fmv_diagnostics();
    if (show_corruption_presets_)
        panel_corruption_presets();
}

void App::menu_bar() {
    if (ImGui::BeginMainMenuBar()) {
        const bool emu_running = emu_runner_.is_running();
        const bool bios_loaded = system_->bios_loaded();
        const bool disc_loaded = system_->disc_loaded();

        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Load BIOS...", "Ctrl+B")) {
                std::string path = open_file_dialog(
                    "BIOS Files (*.bin)\0*.bin\0All Files\0*.*\0", "Select PS1 BIOS");
                if (!path.empty()) {
                    emu_runner_.pause_and_wait_idle();
                    disable_ram_reaper_mode();
                    disable_gpu_reaper_mode();
                    disable_sound_reaper_mode();
                    if (system_->load_bios(path)) {
                        bios_path_ = path;
                        save_persistent_config();
                        has_started_emulation_ = false;
                        set_grim_reaper_mode(false);
                        status_message_ = "BIOS loaded: " + system_->bios().get_info();
                    }
                    else {
                        status_message_ = "Failed to load BIOS!";
                    }
                }
            }
            if (ImGui::MenuItem("Load Game...", "Ctrl+O")) {
                std::string path = open_file_dialog(
                    "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
                    "Select PS1 Game");
                if (!path.empty()) {
                    std::string bin;
                    std::string cue;
                    std::string error;
                    if (!resolve_disc_paths(path, bin, cue, error)) {
                        status_message_ = error;
                        ImGui::EndMenu();
                        ImGui::EndMainMenuBar();
                        return;
                    }
                    if (!load_disc_from_ui(bin, cue)) {
                        ImGui::EndMenu();
                        ImGui::EndMainMenuBar();
                        return;
                    }
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit", "Alt+F4")) {
                SDL_Event quit_event;
                quit_event.type = SDL_QUIT;
                SDL_PushEvent(&quit_event);
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Emulation")) {
            const char* bios_menu_label =
                has_started_emulation_ ? "Resume" : "Start BIOS";
            if (ImGui::MenuItem("Boot Disc", "Ctrl+F5", false,
                bios_loaded && !emu_running &&
                (disc_loaded || !game_bin_path_.empty()))) {
                if (!boot_disc_from_ui()) {
                    ImGui::EndMenu();
                    ImGui::EndMainMenuBar();
                    return;
                }
            }
            if (ImGui::MenuItem("Eject Disc", "Ctrl+E", false,
                bios_loaded && !emu_running &&
                (disc_loaded || !game_bin_path_.empty()))) {
                unload_disc_from_ui();
            }
            if (ImGui::MenuItem("Direct Disc Boot (Skip BIOS Intro)", nullptr,
                config_direct_disc_boot_)) {
                config_direct_disc_boot_ = !config_direct_disc_boot_;
                save_persistent_config();
            }
            if (ImGui::MenuItem(bios_menu_label, "F5", false,
                bios_loaded && !emu_running)) {
                if (has_started_emulation_) {
                    emu_runner_.set_running(true);
                    status_message_ = "Emulation resumed";
                }
                else {
                    if (!start_bios_from_ui()) {
                        ImGui::EndMenu();
                        ImGui::EndMainMenuBar();
                        return;
                    }
                }
            }
            if (ImGui::MenuItem("Pause", "F6", false, emu_running)) {
                emu_runner_.pause_and_wait_idle();
                status_message_ = "Emulation paused";
            }
            if (ImGui::MenuItem("Stop", "F7", false,
                bios_loaded && has_started_emulation_)) {
                emu_runner_.pause_and_wait_idle();
                disable_ram_reaper_mode();
                disable_gpu_reaper_mode();
                disable_sound_reaper_mode();
                has_started_emulation_ = false;
                status_message_ = "Emulation stopped";
            }
            if (ImGui::MenuItem("Take Snapshot", "F8", false,
                !latest_frame_rgba_.empty())) {
                save_snapshot_png();
            }
            if (ImGui::MenuItem("Restart BIOS", nullptr, false, bios_loaded)) {
                emu_runner_.pause_and_wait_idle();
                disable_ram_reaper_mode();
                disable_gpu_reaper_mode();
                disable_sound_reaper_mode();
                set_grim_reaper_mode(false);
                if (!bios_path_.empty() && !system_->load_bios(bios_path_)) {
                    status_message_ = "Failed to reload original BIOS";
                }
                else {
                    system_->reset();
                    apply_memory_card_settings(false);
                    has_started_emulation_ = true;
                    emu_runner_.set_running(true);
                    status_message_ = "BIOS emulation restarted";
                }
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Settings", "Ctrl+,", &show_settings_);
            ImGui::MenuItem("CPU Debug", "F9", &show_debug_cpu_);
            ImGui::MenuItem("Show VRAM", "F10", &show_vram_);
            ImGui::MenuItem("Performance Overlay", "F11", &show_perf_);
            if (ImGui::MenuItem("Performance Profiler", "F12",
                show_perf_profiler_)) {
                show_perf_profiler_ = !show_perf_profiler_;
                g_profile_detailed_timing = show_perf_profiler_;
            }
            ImGui::MenuItem("Voice Levels", nullptr, &show_sound_status_);
            ImGui::MenuItem("Logging", nullptr, &show_logging_);
            ImGui::MenuItem("About", nullptr, &show_about_);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Grim Reaper")) {
            if (ImGui::MenuItem("Open Grim Reaper")) {
                open_definitive_grim_reaper();
            }
            ImGui::EndMenu();
        }

        // Status bar on the right
        const char* disc_text = disc_loaded ? "Disc: Loaded" : "Disc: None";
        const float status_width = ImGui::CalcTextSize(status_message_.c_str()).x + 16.0f;
        const float disc_width = ImGui::CalcTextSize(disc_text).x + 32.0f;

        ImGui::SameLine(ImGui::GetWindowWidth() - status_width - disc_width - 64.0f);
        ImGui::TextColored(ImVec4(0.5f, 0.4f, 0.8f, 1.0f), "%s",
            status_message_.c_str());

        ImGui::SameLine(ImGui::GetWindowWidth() - disc_width - 72.0f);
        ImGui::TextColored(disc_loaded ? ImVec4(0.4f, 0.8f, 0.4f, 1.0f)
            : ImVec4(0.85f, 0.45f, 0.45f, 1.0f),
            "%s", disc_text);

        ImGui::SameLine(ImGui::GetWindowWidth() - 96.0f);
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "%.0f/%.0f FPS",
            game_fps_, video_fps_);

        ImGui::EndMainMenuBar();
    }
}

bool App::should_route_keyboard_to_emu(const SDL_Event& event,
    const ImGuiIO& io) const {
    const bool keyboard_event =
        (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP);
    if (!keyboard_event) {
        return false;
    }
    if (!emu_runner_.is_running()) {
        return false;
    }
    if ((SDL_GetWindowFlags(window_) & SDL_WINDOW_INPUT_FOCUS) == 0) {
        return false;
    }
    // Always release keys even while UI captures keyboard, to avoid stuck
    // controller bits when focus changes between emulator and widgets.
    if (event.type == SDL_KEYUP) {
        return true;
    }
    if (io.WantTextInput) {
        return false;
    }
    return true;
}



// â”€â”€ File Dialog â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

bool App::resolve_disc_paths(const std::string& selected_path,
    std::string& bin_path, std::string& cue_path,
    std::string& error) const {
    bin_path.clear();
    cue_path.clear();
    error.clear();

    std::string normalized_path = trim_copy(selected_path);
    if (normalized_path.size() >= 2u &&
        normalized_path.front() == '"' &&
        normalized_path.back() == '"') {
        normalized_path =
            normalized_path.substr(1u, normalized_path.size() - 2u);
        normalized_path = trim_copy(normalized_path);
    }

    const std::filesystem::path selected(normalized_path);
    std::string ext = selected.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
        });

    if (ext == ".cue") {
        cue_path = selected.string();
        // Multi-track cues often point to "Track 1/2/3..." filenames that do
        // not match the cue basename, so parse the first FILE entry first.
        bin_path = resolve_first_bin_from_cue(selected);
        if (bin_path.empty()) {
            std::filesystem::path sibling_bin = selected;
            sibling_bin.replace_extension(".bin");
            if (std::filesystem::exists(sibling_bin)) {
                bin_path = sibling_bin.string();
            }
            else {
                std::filesystem::path sibling_bin_upper = selected;
                sibling_bin_upper.replace_extension(".BIN");
                if (std::filesystem::exists(sibling_bin_upper)) {
                    bin_path = sibling_bin_upper.string();
                }
            }
        }
        return true;
    }

    if (ext == ".bin") {
        bin_path = selected.string();
        std::filesystem::path sibling_cue = selected;
        sibling_cue.replace_extension(".cue");
        if (std::filesystem::exists(sibling_cue)) {
            cue_path = sibling_cue.string();
        }
        else {
            std::filesystem::path sibling_cue_upper = selected;
            sibling_cue_upper.replace_extension(".CUE");
            if (std::filesystem::exists(sibling_cue_upper)) {
                cue_path = sibling_cue_upper.string();
            }
        }

        // CUE is optional. If missing, CDROM loader falls back to
        // TRACK 01 MODE2/2352 with INDEX 01 00:00:00.
        return true;
    }

    error = "Unsupported disc image. Select a .cue or .bin file.";
    return false;
}

std::array<std::string, App::kMemoryCardSlotCount> App::resolve_memory_card_paths() const {
    std::array<std::string, kMemoryCardSlotCount> paths{};
    const std::filesystem::path card_dir =
        std::filesystem::current_path() / kMemoryCardDirName;

    std::string game_stem;
    if (!game_cue_path_.empty()) {
        game_stem = std::filesystem::path(game_cue_path_).stem().string();
    }
    else if (!game_bin_path_.empty()) {
        game_stem = std::filesystem::path(game_bin_path_).stem().string();
    }
    else if (system_ != nullptr && !system_->cdrom().resolved_disc_path().empty()) {
        game_stem = std::filesystem::path(system_->cdrom().resolved_disc_path()).stem().string();
    }
    if (!game_stem.empty()) {
        game_stem = sanitize_memory_card_stem(game_stem);
    }

    for (int slot = 0; slot < kMemoryCardSlotCount; ++slot) {
        const int mode = std::max(0, std::min(2, config_memory_card_mode_[slot]));
        if (mode == 2) {
            paths[slot].clear();
            continue;
        }

        std::string file_stem;
        if (mode == 1 && !game_stem.empty()) {
            file_stem = game_stem + "_slot" + std::to_string(slot + 1);
        }
        else {
            file_stem = "generic_slot" + std::to_string(slot + 1);
        }

        paths[slot] = (card_dir / (file_stem + ".mcd")).string();
    }

    return paths;
}

void App::apply_memory_card_settings(bool save_config) {
    if (save_config) {
        save_persistent_config();
    }

    memory_card_target_paths_ = resolve_memory_card_paths();
    if (!system_) {
        return;
    }

    if (has_started_emulation_ && emu_runner_.is_running()) {
        emu_runner_.request_memory_card_paths(memory_card_target_paths_);
        return;
    }

    if (has_started_emulation_) {
        emu_runner_.pause_and_wait_idle();
    }

    for (u32 slot = 0; slot < memory_card_target_paths_.size(); ++slot) {
        if (!system_->set_memory_card_slot(slot, memory_card_target_paths_[slot])) {
            status_message_ = "Failed to mount memory card slot " +
                std::to_string(static_cast<unsigned>(slot + 1));
        }
    }
}

bool App::load_disc_from_ui(const std::string& bin_path,
    const std::string& cue_path) {
    const std::string disc_label =
        cue_path.empty()
        ? std::filesystem::path(bin_path).filename().string()
        : std::filesystem::path(cue_path).filename().string();

    const bool hot_insert = has_started_emulation_;
    if (hot_insert) {
        game_bin_path_ = bin_path;
        game_cue_path_ = cue_path;
        apply_memory_card_settings(false);
        emu_runner_.request_live_disc_insert(bin_path, cue_path);
        status_message_ = "Disc inserted: " + disc_label + " (live)";
        return true;
    }

    game_bin_path_ = bin_path;
    game_cue_path_ = cue_path;
    apply_memory_card_settings(false);
    status_message_ = "Disc selected: " + disc_label + " (Emulation > Boot Disc)";
    return true;
}

bool App::save_snapshot_png() {
    if (latest_frame_rgba_.empty() || latest_frame_width_ <= 0 ||
        latest_frame_height_ <= 0) {
        status_message_ = "Snapshot unavailable: no frame captured yet.";
        return false;
    }

    std::error_code ec;
    const std::filesystem::path snapshot_dir =
        std::filesystem::current_path() / "snapshots";
    std::filesystem::create_directories(snapshot_dir, ec);
    if (ec) {
        status_message_ = "Snapshot failed: couldn't create snapshots directory.";
        return false;
    }

    const std::time_t now_time = std::time(nullptr);
    std::tm now_tm{};
#ifdef _WIN32
    localtime_s(&now_tm, &now_time);
#else
    localtime_r(&now_time, &now_tm);
#endif
    char stamp[32] = {};
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &now_tm);

    std::filesystem::path output_path =
        snapshot_dir / ("snapshot_" + std::string(stamp) + ".png");
    for (int suffix = 1; std::filesystem::exists(output_path); ++suffix) {
        output_path = snapshot_dir /
            ("snapshot_" + std::string(stamp) + "_" + std::to_string(suffix) + ".png");
    }

    std::vector<u8> png_bytes;
    if (!encode_rgba_png(latest_frame_rgba_, latest_frame_width_, latest_frame_height_,
        png_bytes)) {
        status_message_ = "Snapshot failed: PNG encode error.";
        return false;
    }

    std::ofstream out(output_path, std::ios::binary);
    if (!out.is_open()) {
        status_message_ = "Snapshot failed: couldn't open output file.";
        return false;
    }
    out.write(reinterpret_cast<const char*>(png_bytes.data()),
        static_cast<std::streamsize>(png_bytes.size()));
    if (!out.good()) {
        status_message_ = "Snapshot failed: write error.";
        return false;
    }

    status_message_ = "Snapshot saved: " + output_path.string();
    return true;
}

bool App::start_bios_from_ui() {
    if (!system_->bios_loaded()) {
        status_message_ = "Load a BIOS first.";
        return false;
    }

    emu_runner_.pause_and_wait_idle();
    disable_ram_reaper_mode();
    disable_gpu_reaper_mode();
    disable_sound_reaper_mode();
    system_->reset();
    apply_memory_card_settings(false);
    if (!start_configured_input_movie()) {
        return false;
    }
    emu_runner_.configure_rewind(config_rewind_enabled_,
        config_rewind_buffer_seconds_, static_cast<int>(system_->target_fps()));
    gameplay_toolbar_visibility_ = 0.0f;
    gameplay_toolbar_reveal_hold_ = 0.0f;
    gameplay_toolbar_turbo_active_ = false;
    gameplay_toolbar_rewind_active_ = false;
    has_started_emulation_ = true;
    emu_runner_.set_running(true);
    status_message_ = "Emulation started (BIOS)";
    return true;
}

bool App::boot_disc_from_ui() {
    emu_runner_.pause_and_wait_idle();
    disable_ram_reaper_mode();
    disable_gpu_reaper_mode();
    disable_sound_reaper_mode();

    if (!system_->bios_loaded()) {
        status_message_ = "Load a BIOS before booting a disc.";
        return false;
    }

    if (!game_bin_path_.empty() || !game_cue_path_.empty()) {
        // Always (re)attach the currently selected game so changing selection
        // after a prior boot takes effect.
        if (!system_->load_game(game_bin_path_, game_cue_path_)) {
            status_message_ = "Failed to attach selected disc image.";
            return false;
        }
    }

    if (!system_->disc_loaded()) {
        status_message_ = "No disc loaded. Use File > Load Game.";
        return false;
    }

    if (!system_->boot_disc(config_direct_disc_boot_)) {
        status_message_ = "Boot Disc failed. Check BIOS/disc image.";
        return false;
    }

    apply_memory_card_settings(false);
    if (!start_configured_input_movie()) {
        return false;
    }
    emu_runner_.configure_rewind(config_rewind_enabled_,
        config_rewind_buffer_seconds_, static_cast<int>(system_->target_fps()));
    gameplay_toolbar_visibility_ = 0.0f;
    gameplay_toolbar_reveal_hold_ = 0.0f;
    gameplay_toolbar_turbo_active_ = false;
    gameplay_toolbar_rewind_active_ = false;
    has_started_emulation_ = true;
    emu_runner_.set_running(true);
    status_message_ = config_direct_disc_boot_
        ? "Direct booting disc (BIOS intro skipped)..."
        : "Booting disc from BIOS...";
    return true;
}

bool App::unload_disc_from_ui() {
    if (!system_->bios_loaded()) {
        status_message_ = "Load a BIOS first.";
        return false;
    }

    emu_runner_.pause_and_wait_idle();
    disable_ram_reaper_mode();
    disable_gpu_reaper_mode();
    disable_sound_reaper_mode();

    // If emulation was already running, also send a live eject to the
    // emulation thread in case it picked up the disc via hot-insert.
    emu_runner_.request_live_disc_eject();

    system_->unload_disc();
    game_bin_path_.clear();
    game_cue_path_.clear();
    system_->reset();
    has_started_emulation_ = true;
    emu_runner_.set_running(true);
    status_message_ = "Disc ejected — BIOS menu";
    return true;
}

bool App::launch_bios_only_from_cli(const std::string& bios_path) {
    if (!init_runtime()) {
        status_message_ = "CLI BIOS-only: runtime init failed.";
        return false;
    }

    if (!bios_path.empty()) {
        if (!system_->load_bios(bios_path)) {
            status_message_ = "CLI BIOS-only: BIOS load failed.";
            LOG_ERROR("CLI BIOS-only: failed to load BIOS: %s", bios_path.c_str());
            return false;
        }
        bios_path_ = bios_path;
    }

    emu_runner_.pause_and_wait_idle();
    disable_ram_reaper_mode();
    disable_gpu_reaper_mode();
    disable_sound_reaper_mode();
    system_->reset();
    if (!start_configured_input_movie()) {
        return false;
    }
    has_started_emulation_ = true;
    emu_runner_.set_running(true);
    status_message_ = "BIOS-only mode (no disc)";
    return true;
}

bool App::start_configured_input_movie() {
    if (!input_movie_cli_pending_) {
        return true;
    }

    const std::string disc_path =
        !game_cue_path_.empty() ? game_cue_path_ : game_bin_path_;
    input_movie_cli_pending_ = false;
    if (!input_recorder_.init(disc_path, VIBESTATION_FULL_VERSION_STRING)) {
        const InputRecorder::Status movie_status = input_recorder_.status();
        status_message_ = "Input movie failed: " + movie_status.status_message;
        LOG_ERROR("%s", status_message_.c_str());
        return false;
    }

    const InputRecorder::Status movie_status = input_recorder_.status();
    if (movie_status.mode == InputRecorder::Mode::Recording) {
        status_message_ = "Input recording started: " + movie_status.record_path;
    }
    else if (movie_status.mode == InputRecorder::Mode::Playing) {
        status_message_ = "Input playback started: " + movie_status.playback_path;
    }
    return true;
}

InputRecorder::PlaybackEndBehavior App::input_movie_end_behavior() const {
    if (input_movie_loop_) {
        return InputRecorder::PlaybackEndBehavior::Loop;
    }
    if (input_movie_stop_at_eof_) {
        return InputRecorder::PlaybackEndBehavior::Stop;
    }
    return InputRecorder::PlaybackEndBehavior::HoldNeutral;
}

bool App::start_input_recording_from_ui() {
    if (input_movie_record_path_[0] == '\0') {
        status_message_ = "Input movie: enter a recording filename or path.";
        return false;
    }

    InputRecorder::Config config{};
    config.record_path = input_movie_record_path_;
    config.end_behavior = input_movie_end_behavior();
    input_recorder_.set_config(config);
    const std::string disc_path =
        !game_cue_path_.empty() ? game_cue_path_ : game_bin_path_;
    if (!input_recorder_.init(disc_path, VIBESTATION_FULL_VERSION_STRING)) {
        status_message_ = "Input movie: " + input_recorder_.status().status_message;
        return false;
    }

    input_recorder_config_ = input_recorder_.config();
    input_movie_cli_pending_ = false;
    status_message_ = "Input recording started: " +
        input_recorder_.status().record_path;
    return true;
}

bool App::start_input_playback_from_ui() {
    if (input_movie_playback_path_[0] == '\0') {
        status_message_ = "Input movie: enter a playback filename or path.";
        return false;
    }

    InputRecorder::Config config{};
    config.playback_path = input_movie_playback_path_;
    config.end_behavior = input_movie_end_behavior();
    input_recorder_.set_config(config);
    const std::string disc_path =
        !game_cue_path_.empty() ? game_cue_path_ : game_bin_path_;
    if (!input_recorder_.init(disc_path, VIBESTATION_FULL_VERSION_STRING)) {
        status_message_ = "Input movie: " + input_recorder_.status().status_message;
        return false;
    }

    input_recorder_config_ = input_recorder_.config();
    input_movie_cli_pending_ = false;
    status_message_ = "Input playback started: " +
        input_recorder_.status().playback_path;
    return true;
}

std::string App::open_file_dialog(const char* filter, const char* title) {
#ifdef _WIN32
    OPENFILENAMEA ofn = {};
    char filename[MAX_PATH] = "";
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrTitle = title;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;

    if (GetOpenFileNameA(&ofn)) {
        return std::string(filename);
    }
#endif
    return "";
}

std::string App::open_folder_dialog(const char* title) {
#ifdef _WIN32
    BROWSEINFOA bi = {};
    bi.hwndOwner = nullptr;
    bi.lpszTitle = title;
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;

    LPITEMIDLIST pidl = SHBrowseForFolderA(&bi);
    if (pidl == nullptr) {
        return "";
    }

    char folder_path[MAX_PATH] = "";
    if (!SHGetPathFromIDListA(pidl, folder_path)) {
        CoTaskMemFree(pidl);
        return "";
    }
    CoTaskMemFree(pidl);
    return std::string(folder_path);
#else
    (void)title;
    return "";
#endif
}

void App::refresh_game_library() {
    game_library_.clear();
    game_library_last_scan_ms_ = SDL_GetTicks();
    game_library_dirty_ = false;
    rom_directory_valid_ = false;

    if (rom_directory_.empty()) {
        return;
    }

    std::error_code ec;
    if (!std::filesystem::exists(rom_directory_, ec) ||
        !std::filesystem::is_directory(rom_directory_, ec)) {
        return;
    }
    rom_directory_valid_ = true;

    std::vector<std::filesystem::path> cue_paths;
    std::filesystem::recursive_directory_iterator it(
        rom_directory_, std::filesystem::directory_options::skip_permission_denied, ec);
    std::filesystem::recursive_directory_iterator end;
    if (ec) {
        return;
    }

    for (; it != end; it.increment(ec)) {
        if (ec) {
            ec.clear();
            continue;
        }
        if (!it->is_regular_file(ec)) {
            continue;
        }

        std::filesystem::path file = it->path();
        std::string ext = file.extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
            });

        if (ext == ".cue") {
            cue_paths.push_back(file);
        }
    }

    for (const auto& cue : cue_paths) {
        std::string bin;
        std::string cue_path;
        std::string error;
        if (!resolve_disc_paths(cue.string(), bin, cue_path, error)) {
            continue;
        }
        GameLibraryEntry entry{};
        entry.title = cue.stem().string();
        entry.bin_path = bin;
        entry.cue_path = cue_path;
        game_library_.push_back(std::move(entry));
    }

    std::sort(game_library_.begin(), game_library_.end(),
        [](const GameLibraryEntry& a, const GameLibraryEntry& b) {
            if (a.title == b.title) {
                return a.cue_path < b.cue_path;
            }
            return a.title < b.title;
        });
}


void App::load_persistent_config() {
    config_ = Config::load(kAppConfigFileName);

    // Apply config values to globals for backward compatibility
    config_.apply_to_globals();

    // App-specific members not in Config
    bios_path_ = config_.bios_path;
    rom_directory_ = config_.rom_directory;
    config_vsync_ = config_.vsync;
    config_low_spec_mode_ = config_.low_spec_mode;
    config_direct_disc_boot_ = config_.direct_disc_boot;
    config_turbo_speed_percent_ = config_.turbo_speed_percent;
    config_slowdown_speed_percent_ = config_.slowdown_speed_percent;
    config_spu_diagnostic_mode_ = config_.spu_diagnostic_mode;
    config_discord_rich_presence_ = config_.discord_rich_presence;
    config_rewind_enabled_ = config_.rewind_enabled;
    config_rewind_buffer_seconds_ = config_.rewind_buffer_seconds;
    config_memory_card_mode_[0] = config_.memory_card_slot_mode[0];
    config_memory_card_mode_[1] = config_.memory_card_slot_mode[1];
    std::snprintf(log_path_, sizeof(log_path_), "%s", config_.log_file_path.c_str());

    if (!rom_directory_.empty()) {
        game_library_dirty_ = true;
    }

    // Key bindings are stored as JSON integer keys (e.g. "bind_start": 40)
    {
        std::ifstream in(kAppConfigFileName);
        nlohmann::json j;
        if (in.is_open()) {
            try { in >> j; } catch (...) {}
        }
        for (const auto& entry : kKeyboardBindEntries) {
            if (j.contains(entry.config_key) && j[entry.config_key].is_number()) {
                const SDL_Scancode scancode =
                    static_cast<SDL_Scancode>(j[entry.config_key].get<int>());
                if (scancode == SDL_SCANCODE_UNKNOWN) {
                    input_->clear_key_binding(entry.button);
                } else {
                    input_->set_key_binding(scancode, entry.button);
                }
            }
        }
    }
}

void App::save_persistent_config() const {
    // Sync config from App members (loaded from config file at startup)
    Config out = config_;
    out.bios_path = bios_path_;
    out.rom_directory = rom_directory_;
    out.vsync = config_vsync_;
    out.low_spec_mode = config_low_spec_mode_;
    out.direct_disc_boot = config_direct_disc_boot_;
    out.turbo_speed_percent = config_turbo_speed_percent_;
    out.slowdown_speed_percent = config_slowdown_speed_percent_;
    out.spu_diagnostic_mode = config_spu_diagnostic_mode_;
    out.discord_rich_presence = config_discord_rich_presence_;
    out.rewind_enabled = config_rewind_enabled_;
    out.rewind_buffer_seconds = config_rewind_buffer_seconds_;
    out.memory_card_slot_mode[0] = config_memory_card_mode_[0];
    out.memory_card_slot_mode[1] = config_memory_card_mode_[1];
    out.log_file_path = log_path_;

    // Sync from globals that UI panels write to directly
    out.gpu_fast_mode = g_gpu_fast_mode;
    out.gpu_extreme_fast_mode = g_gpu_extreme_fast_mode;
    out.bilinear_filtering = g_bilinear_filtering;
    out.deinterlace_mode = g_deinterlace_mode;
    out.output_resolution_mode = g_output_resolution_mode;
    out.log_level = g_log_level;
    out.log_timestamps = g_log_timestamp;
    out.log_collapse_repeats = g_log_dedupe;
    out.log_fmv_diagnostics = g_log_fmv_diagnostics;
    out.log_repeat_flush = g_log_dedupe_flush;
    out.log_category_mask = g_log_category_mask;
    out.cpu_deep_diagnostics = g_cpu_deep_diagnostics;
    out.detailed_profiling = g_profile_detailed_timing;
    out.experimental_bios_size_mode = g_experimental_bios_size_mode;
    out.unsafe_ps2_bios_mode = g_unsafe_ps2_bios_mode;
    out.experimental_unhandled_special_returns_zero = g_experimental_unhandled_special_returns_zero;
    out.experimental_dma_command_sanitizer = g_experimental_dma_command_sanitizer;
    out.cpu_execution_mode = g_cpu_execution_mode;
    out.spu.target_latency_ms = g_spu_audio_target_latency_ms;
    out.spu.soft_latency_ms = g_spu_audio_soft_latency_ms;
    out.spu.max_latency_ms = g_spu_audio_max_latency_ms;
    out.spu.output_buffer_seconds = g_spu_output_buffer_seconds;
    out.spu.xa_buffer_seconds = g_spu_xa_buffer_seconds;
    out.spu.enable_audio_queue = g_spu_enable_audio_queue;
    out.spu.enable_smooth_trim = g_spu_enable_smooth_trim;
    out.spu.enable_lag_stutter = g_spu_enable_lag_stutter;
    out.spu.enable_slowdown_stutter = g_spu_enable_slowdown_stutter;
    out.spu.show_audio_stats = g_spu_show_audio_stats;
    out.spu.audio_stats_log = g_spu_audio_stats_log;
    out.spu.advanced_sound_status = g_spu_advanced_sound_status;
    out.trace.dma = g_trace_dma;
    out.trace.cdrom = g_trace_cdrom;
    out.trace.cpu = g_trace_cpu;
    out.trace.bus = g_trace_bus;
    out.trace.ram = g_trace_ram;
    out.trace.gpu = g_trace_gpu;
    out.trace.spu = g_trace_spu;
    out.trace.irq = g_trace_irq;
    out.trace.timer = g_trace_timer;
    out.trace.sio = g_trace_sio;
    out.trace.burst_cpu = g_trace_burst_cpu;
    out.trace.stride_cpu = g_trace_stride_cpu;
    out.trace.burst_bus = g_trace_burst_bus;
    out.trace.stride_bus = g_trace_stride_bus;
    out.trace.burst_ram = g_trace_burst_ram;
    out.trace.stride_ram = g_trace_stride_ram;
    out.trace.burst_dma = g_trace_burst_dma;
    out.trace.stride_dma = g_trace_stride_dma;
    out.trace.burst_cdrom = g_trace_burst_cdrom;
    out.trace.stride_cdrom = g_trace_stride_cdrom;
    out.trace.burst_gpu = g_trace_burst_gpu;
    out.trace.stride_gpu = g_trace_stride_gpu;
    out.trace.burst_spu = g_trace_burst_spu;
    out.trace.stride_spu = g_trace_stride_spu;
    out.trace.burst_irq = g_trace_burst_irq;
    out.trace.stride_irq = g_trace_stride_irq;
    out.trace.burst_timer = g_trace_burst_timer;
    out.trace.stride_timer = g_trace_stride_timer;
    out.trace.burst_sio = g_trace_burst_sio;
    out.trace.stride_sio = g_trace_stride_sio;
    out.mdec_debug.disable_dma1_reorder = g_mdec_debug_disable_dma1_reorder;
    out.mdec_debug.disable_chroma = g_mdec_debug_disable_chroma;
    out.mdec_debug.disable_luma = g_mdec_debug_disable_luma;
    out.mdec_debug.force_solid_output = g_mdec_debug_force_solid_output;
    out.mdec_debug.swap_input_halfwords = g_mdec_debug_swap_input_halfwords;
    out.mdec_debug.compare_macroblocks = g_mdec_debug_compare_macroblocks;
    out.mdec_debug.upload_probe = g_mdec_debug_upload_probe;
    out.mdec_debug.color_block_mask = g_mdec_debug_color_block_mask;

    out.save(kAppConfigFileName);

    {
        std::ifstream in(kAppConfigFileName);
        nlohmann::json j;
        if (in.is_open()) {
            try { in >> j; } catch (...) {}
        }
        for (const auto& entry : kKeyboardBindEntries) {
            j[entry.config_key] = static_cast<int>(input_->key_for_button(entry.button));
        }
        std::ofstream out_stream(kAppConfigFileName, std::ios::out | std::ios::trunc);
        if (out_stream.is_open()) {
            out_stream << j.dump(2) << "\n";
        }
    }
}

void App::try_autoload_bios_from_config() {
    if (!system_ || system_->bios_loaded()) {
        return;
    }

    if (bios_path_.empty()) {
        return;
    }

    if (!std::filesystem::exists(bios_path_)) {
        status_message_ = "Saved BIOS path not found. Load BIOS manually.";
        return;
    }

    if (system_->load_bios(bios_path_)) {
        has_started_emulation_ = false;
        status_message_ = "Auto-loaded BIOS: " + system_->bios().get_info();
    }
    else {
        status_message_ = "Failed to auto-load saved BIOS. Load BIOS manually.";
    }
}
void App::shutdown() {
    save_persistent_config();
    if (ImGui::GetCurrentContext() != nullptr) {
        ImGuiIO& io = ImGui::GetIO();
        if (io.IniFilename != nullptr && io.IniFilename[0] != '\0') {
            ImGui::SaveIniSettingsToDisk(io.IniFilename);
        }
    }
    // Stop presentation first: it may still recycle raw buffers back to
    // EmuRunner. The emulation thread remains alive until that worker exits.
    frame_presentation_worker_.stop();
    emu_runner_.stop();
    input_recorder_.shutdown();
    discord_presence_.reset();
    if (renderer_) {
        renderer_->shutdown();
    }
    renderer_.reset();
    input_.reset();
    if (system_) {
        system_->shutdown();
    }
    system_.reset();
    runtime_ready_ = false;

    release_definitive_ui_assets();

    if (vram_debug_texture_ != 0) {
        glDeleteTextures(1, &vram_debug_texture_);
        vram_debug_texture_ = 0;
    }

    if (use_imgui_opengl2_backend_) {
        ImGui_ImplOpenGL2_Shutdown();
    }
    else {
        ImGui_ImplOpenGL3_Shutdown();
    }
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context_);
    SDL_DestroyWindow(window_);
    if (g_log_file) {
        log_flush_repeats();
        std::fclose(g_log_file);
        g_log_file = nullptr;
    }
    SDL_Quit();
}
