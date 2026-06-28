#include "app.h"
#include "platform/disc_path_utils.h"
#include "platform/memory_card_utils.h"
#include "ui/input_bindings.h"
#include "ui/output_resolution_utils.h"
#include "ui/screenshot_utils.h"
#include "ui/theme_settings.h"
#include "ui/panels/cpu_backend_panel.h"
#include "ui/panels/grim_reaper_panel.h"
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
#include <random>
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
    constexpr float kEmulatorScreenBottomOverscanPixels = 3.0f;
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

    void seed_mt19937(std::mt19937& rng, u64 seed) {
        const u32 lo = static_cast<u32>(seed & 0xFFFFFFFFull);
        const u32 hi = static_cast<u32>((seed >> 32) & 0xFFFFFFFFull);
        std::seed_seq seq{ lo, hi, 0x9E3779B9u, 0x243F6A88u };
        rng.seed(seq);
    }

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
    if (turbo_hold_active_) {
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

    bool quit = false;
    last_fps_time_ = SDL_GetTicks();
    const u64 perf_freq = SDL_GetPerformanceFrequency();
    const double target_frame_sec = 1.0 / 60.0;

    while (!quit) {
        const u64 loop_start_counter = SDL_GetPerformanceCounter();
        process_events(quit);
        update();

        FrameSnapshot frame;
        if (emu_runner_.consume_latest_frame(frame)) {
            game_frame_count_++;
            const bool turbo_resolution_clamp =
                turbo_hold_active_ &&
                g_output_resolution_mode != OutputResolutionMode::R320x240 &&
                (frame.width > 320 || frame.height > 240);
            int output_width = 320;
            int output_height = 240;
            output_resolution_dimensions(g_output_resolution_mode, output_width, output_height);
            if (turbo_resolution_clamp) {
                resample_rgba_nearest(frame.rgba, frame.width, frame.height,
                    turbo_frame_rgba_, 320, 240);
                renderer_->upload_frame(turbo_frame_rgba_, 320, 240);
                latest_frame_width_ = 320;
                latest_frame_height_ = 240;
                latest_frame_rgba_ = turbo_frame_rgba_;
            }
            else if (frame.width != output_width || frame.height != output_height) {
                resample_rgba_nearest(frame.rgba, frame.width, frame.height,
                    scaled_frame_rgba_, output_width, output_height);
                renderer_->upload_frame(scaled_frame_rgba_, output_width, output_height);
                latest_frame_width_ = output_width;
                latest_frame_height_ = output_height;
                latest_frame_rgba_ = scaled_frame_rgba_;
            }
            else {
                renderer_->upload_frame(frame.rgba, frame.width, frame.height);
                latest_frame_width_ = frame.width;
                latest_frame_height_ = frame.height;
                latest_frame_rgba_ = std::move(frame.rgba);
            }
            emu_runner_.recycle_consumed_frame(std::move(frame));
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
            if (turbo_hold_active_) {
                turbo_hold_active_ = false;
                apply_speed_override();
            }
            if (slowdown_hold_active_) {
                slowdown_hold_active_ = false;
                apply_speed_override();
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
                show_settings_ = !show_settings_;
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
        }

        const ImGuiIO& io = ImGui::GetIO();
        if (should_route_keyboard_to_emu(event, io)) {
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
    menu_bar();

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
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoBackground;

    ImGui::Begin("DockSpace", nullptr, flags);
    ImGui::PopStyleVar(3);

    panel_emulator_screen();
    ImGui::End();

    // Optional panels
    if (show_logging_) {
        show_settings_ = true;
        show_logging_ = false;
    }
    if (show_settings_)
        panel_settings();
    if (show_grim_reaper_)
        panel_grim_reaper();
    if (show_about_)
        panel_about();
    if (show_debug_cpu_)
        panel_debug_cpu();
    if (show_vram_)
        panel_vram();
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
            ImGui::MenuItem("Voice Levels", nullptr, &show_sound_status_);
            ImGui::MenuItem("Logging", nullptr, &show_logging_);
            ImGui::MenuItem("About", nullptr, &show_about_);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Grim Reaper")) {
            ImGui::MenuItem("Open Panel", nullptr, &show_grim_reaper_);
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

void App::panel_emulator_screen() {
    if (!has_started_emulation_) {
        const bool bios_loaded = system_->bios_loaded();
        const bool disc_loaded = system_->disc_loaded();

        // Show a centered welcome message
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        const char* logo_text = "VibeStation";
        const ImVec4 startup_title_color =
            ui_theme::current_startup_title_color(ui_theme::g_theme_settings);
        const ImVec4 startup_text_color =
            ui_theme::current_startup_text_color(ui_theme::g_theme_settings);
        const ImVec4 startup_text_secondary =
            ui_theme::theme_lerp(startup_text_color, ui_theme::g_theme_settings.background, 0.18f);
        ImGui::PushStyleColor(ImGuiCol_Text, startup_title_color);
        ImGui::SetWindowFontScale(2.0f);
        const ImVec2 logo_size = ImGui::CalcTextSize(logo_text);
        ImGui::SetCursorPos(ImVec2(center.x - (logo_size.x * 0.5f), center.y - 80));
        ImGui::Text("%s", logo_text);
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();

        ImGui::SetCursorPos(ImVec2(center.x - 180, center.y - 20));
        ImGui::TextColored(startup_text_color,
            "Load a BIOS (File > Load BIOS) to get started.");

        ImGui::SetCursorPos(ImVec2(center.x - 180, center.y + 5));
        ImGui::TextColored(startup_text_secondary,
            "Then load a game and use Emulation > Boot Disc.");

        const char* bios_button_label = bios_loaded ? "Change BIOS" : "Load BIOS";
        const ImVec2 button_size(120.0f, 0.0f);

        ImGui::SetCursorPos(ImVec2(center.x - 200, center.y + 42));
        if (ImGui::Button(bios_button_label, button_size)) {
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

        ImGui::SameLine();
        if (!bios_loaded) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Load Game", button_size)) {
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
        if (!bios_loaded) {
            ImGui::EndDisabled();
        }

        ImGui::SameLine();
        if (!bios_loaded) {
            ImGui::BeginDisabled();
        }
        const bool has_selected_game = !game_bin_path_.empty() || system_->disc_loaded();
        const char* start_button_label =
            has_selected_game ? "Boot Selected Game" : "Start Emulation";
        const ImVec2 start_button_size(150.0f, 0.0f);
        if (ImGui::Button(start_button_label, start_button_size)) {
            if (has_selected_game) {
                boot_disc_from_ui();
            }
            else {
                start_bios_from_ui();
            }
        }
        if (!bios_loaded) {
            ImGui::EndDisabled();
        }

        // Eject Disc button (visible when a game is selected and emulation is idle)
        if (bios_loaded && has_selected_game && !emu_runner_.is_running()) {
            ImGui::SameLine();
            const ImVec2 eject_button_size(100.0f, 0.0f);
            if (ImGui::Button("Eject Disc", eject_button_size)) {
                unload_disc_from_ui();
            }
        }

        const bool rom_dir_valid = rom_directory_valid_;
        if (game_library_dirty_ ||
            (rom_dir_valid &&
                (SDL_GetTicks() - game_library_last_scan_ms_ > 15000u))) {
            refresh_game_library();
        }

        ImGui::SetCursorPos(ImVec2(center.x - 300, center.y + 90));
        ImGui::BeginChild("IdleGameLibrary", ImVec2(600, 230), true);
        ImGui::TextColored(ImVec4(0.75f, 0.72f, 0.95f, 1.0f), "Game Library");
        if (rom_dir_valid) {
            ImGui::TextDisabled("ROM Directory: %s", rom_directory_.c_str());
        }
        else {
            ImGui::TextDisabled("ROM Directory: not set");
        }
        if (ImGui::Button("Set ROM Directory", ImVec2(150.0f, 0.0f))) {
            const std::string selected = open_folder_dialog("Select ROM Directory");
            if (!selected.empty()) {
                rom_directory_ = selected;
                game_library_dirty_ = true;
                save_persistent_config();
                refresh_game_library();
                status_message_ = "ROM directory set: " + rom_directory_;
            }
        }
        ImGui::SameLine();
        if (!rom_dir_valid) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Refresh", ImVec2(90.0f, 0.0f))) {
            game_library_dirty_ = true;
            refresh_game_library();
        }
        if (!rom_dir_valid) {
            ImGui::EndDisabled();
        }
        ImGui::Separator();

        if (!rom_dir_valid) {
            ImGui::TextColored(ImVec4(0.88f, 0.45f, 0.45f, 1.0f),
                "No ROM directory configured.");
            ImGui::TextWrapped("Set a ROM directory to scan and list games here.");
        }
        else if (game_library_.empty()) {
            ImGui::TextColored(ImVec4(0.85f, 0.75f, 0.45f, 1.0f),
                "No playable disc images found.");
        }
        else {
            for (size_t i = 0; i < game_library_.size(); ++i) {
                const auto& entry = game_library_[i];
                std::string label = entry.title + "##game_" + std::to_string(i);
                if (ImGui::Selectable(label.c_str(), false)) {
                    load_disc_from_ui(entry.bin_path, entry.cue_path);
                }
            }
        }
        ImGui::EndChild();

        if (!rom_dir_valid) {
            const char* warning =
                "No ROM directory in vibestation_config.ini. Set one to enable the game list.";
            const ImVec2 text_size = ImGui::CalcTextSize(warning);
            const float warning_y = ImGui::GetWindowHeight() - 60.0f;
            ImGui::SetCursorPos(ImVec2((ImGui::GetWindowWidth() - text_size.x) * 0.5f, warning_y));
            ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.45f, 1.0f), "%s", warning);

            const ImVec2 button_size(170.0f, 0.0f);
            ImGui::SetCursorPos(
                ImVec2((ImGui::GetWindowWidth() - button_size.x) * 0.5f, warning_y + 22.0f));
            if (ImGui::Button("Set ROM Directory##warning", button_size)) {
                const std::string selected = open_folder_dialog("Select ROM Directory");
                if (!selected.empty()) {
                    rom_directory_ = selected;
                    game_library_dirty_ = true;
                    save_persistent_config();
                    refresh_game_library();
                    status_message_ = "ROM directory set: " + rom_directory_;
                }
            }
        }
    }
    else {
        if (latest_frame_rgba_.empty()) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Snapshot (F8)", ImVec2(130.0f, 0.0f))) {
            save_snapshot_png();
        }
        if (latest_frame_rgba_.empty()) {
            ImGui::EndDisabled();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%dx%d", std::max(0, latest_frame_width_),
            std::max(0, latest_frame_height_));
        ImGui::Spacing();

        if (show_fast_mode_notice_) {
            ImGui::TextColored(ImVec4(0.95f, 0.3f, 0.3f, 1.0f),
                "%s",
                g_gpu_fast_mode
                ? "Increase in underruns with Fast Mode, disable unnecessary logging!"
                : "Increase in underruns, enable Fast Mode!");
            ImGui::Spacing();
        }
        ImVec2 avail = ImGui::GetContentRegionAvail();
        // Safe presentation baseline for BIOS/logo recovery: fixed 4:3 letterbox.
        const float display_aspect = 4.0f / 3.0f;
        const float dst_aspect =
            (avail.y > 0.0f) ? (avail.x / avail.y) : display_aspect;
        ImVec2 draw_size = avail;
        if (dst_aspect > display_aspect) {
            draw_size.x = avail.y * display_aspect;
        }
        else {
            draw_size.y = (display_aspect > 0.0f) ? (avail.x / display_aspect) : avail.y;
        }
        const float x_pad = (avail.x - draw_size.x) * 0.5f;
        const float y_pad = (avail.y - draw_size.y) * 0.5f;
        ImVec2 cursor = ImGui::GetCursorPos();
        ImGui::SetCursorPos(ImVec2(cursor.x + x_pad, cursor.y + y_pad));
        const ImVec2 image_pos = ImGui::GetCursorScreenPos();
        const float overscan_v =
            (latest_frame_height_ > 0)
            ? std::min(0.02f,
                kEmulatorScreenBottomOverscanPixels /
                static_cast<float>(std::max(1, latest_frame_height_)))
            : 0.0f;
        ImGui::Image((ImTextureID)(intptr_t)renderer_->get_texture_id(), draw_size,
            ImVec2(0.0f, 0.0f), ImVec2(1.0f, std::max(0.0f, 1.0f - overscan_v)));
        draw_performance_overlay(image_pos, draw_size);
    }
}

void App::draw_spu_diagnostic_mode_controls() {
    if (!config_spu_diagnostic_mode_) {
        if (ImGui::Button("Slowed + Reverb Mode")) {
            config_spu_diagnostic_mode_ = true;
            apply_speed_override();
            save_persistent_config();
        }
    }
    else {
        if (ImGui::Button("Disable Slowed + Reverb Mode")) {
            config_spu_diagnostic_mode_ = false;
            apply_speed_override();
            save_persistent_config();
        }
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(config_spu_diagnostic_mode_ ? "Active" : "Inactive");
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
        "Enables per-sample SPU diagnostics. Voice level meters stay live.");
}

void App::draw_sound_status_content() {
    const auto& diag = runtime_snapshot_.spu_audio;
    const auto& queue_stats = runtime_snapshot_.audio_queue;
    const auto avg_count = [](u64 accum, u64 samples) -> float {
        const u64 denom = std::max<u64>(samples, 1);
        return static_cast<float>(accum) / static_cast<float>(denom);
        };
    const float avg_logical =
        avg_count(diag.logical_voice_accum, diag.logical_voice_samples);
    const float avg_env = avg_count(diag.env_voice_accum, diag.env_voice_samples);
    const float avg_audible =
        avg_count(diag.audible_voice_accum, diag.audible_voice_samples);
    const float queue_kb_raw =
        static_cast<float>(queue_stats.queue_stereo_frames * 2u * sizeof(s16)) /
        1024.0f;
    const float queue_kb =
        (smoothed_audio_queue_kb_ >= 0.0f) ? smoothed_audio_queue_kb_ : queue_kb_raw;
    const float queue_peak_kb =
        static_cast<float>(queue_stats.max_observed_queue_stereo_frames * 2u *
            sizeof(s16)) /
        1024.0f;

    ImGui::Text("Realtime SPU voice monitor (updates every emulated frame).");
    if (ImGui::Button("Reset Sound Stats")) {
        const bool was_running = emu_runner_.is_running();
        if (was_running) {
            emu_runner_.pause_and_wait_idle();
        }
        system_->reset_spu_audio_diag();
        runtime_snapshot_.spu_audio = system_->spu_audio_diag();
        runtime_snapshot_.audio_queue = system_->spu().audio_queue_stats(false);
        status_message_ = "SPU sound stats reset";
        if (was_running) {
            emu_runner_.set_running(true);
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Use after BIOS boot to isolate gameplay peaks.");
    ImGui::Separator();
    ImGui::Text("Running: %s", runtime_snapshot_.running ? "Yes" : "No");
    ImGui::Text("Reverb: %s", diag.reverb_enabled ? "Enabled" : "Disabled");
    ImGui::Text("Generated/Queued Frames: %llu / %llu",
        static_cast<unsigned long long>(diag.generated_frames),
        static_cast<unsigned long long>(diag.queued_frames));
    ImGui::Text("Dropped Stereo Frames: %llu (Overruns: %llu, Underruns: %llu)",
        static_cast<unsigned long long>(queue_stats.dropped_stereo_frames),
        static_cast<unsigned long long>(queue_stats.overrun_count),
        static_cast<unsigned long long>(queue_stats.underrun_count));
    if (g_spu_show_audio_stats) {
        const double queue_latency_ms =
            queue_stats.obtained_callback_sample_rate == 0u
            ? 0.0
            : (static_cast<double>(queue_stats.queue_stereo_frames) * 1000.0) /
                static_cast<double>(queue_stats.obtained_callback_sample_rate);
        ImGui::Text("Audio Queue: %.1f KB (peak %.1f KB), %.1f ms",
            queue_kb, queue_peak_kb, queue_latency_ms);
        ImGui::Text("Produced / Pushed / Callback Consumed: %llu / %llu / %llu stereo frames",
            static_cast<unsigned long long>(queue_stats.produced_stereo_frames),
            static_cast<unsigned long long>(queue_stats.pushed_stereo_frames),
            static_cast<unsigned long long>(queue_stats.callback_consumed_stereo_frames));
    }
    if (!g_spu_advanced_sound_status) {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.85f, 0.75f, 0.35f, 1.0f),
            "Advanced sound status logging is disabled.");
        ImGui::TextDisabled("Enable it in Settings > Audio for voice peaks, ENDX, and detailed SPU counters.");
        return;
    }
    ImGui::Text("Voices Logical (phase!=off): avg %.2f / peak %u",
        avg_logical, static_cast<unsigned>(diag.logical_voice_peak));
    ImGui::Text("Voices Env>0: avg %.2f / peak %u",
        avg_env, static_cast<unsigned>(diag.env_voice_peak));
    ImGui::Text("Voices Audible (nonzero out): avg %.2f / peak %u",
        avg_audible, static_cast<unsigned>(diag.audible_voice_peak));
    ImGui::Text("Logical peak ctx: sample %llu | KON/KOFF %llu/%llu | ENDX 0x%06X",
        static_cast<unsigned long long>(diag.logical_voice_peak_sample),
        static_cast<unsigned long long>(diag.logical_voice_peak_key_on_events),
        static_cast<unsigned long long>(diag.logical_voice_peak_key_off_events),
        static_cast<unsigned>(diag.logical_voice_peak_endx_mask & 0x00FFFFFFu));
    ImGui::Text("Env>0 peak ctx: sample %llu | KON/KOFF %llu/%llu | ENDX 0x%06X",
        static_cast<unsigned long long>(diag.env_voice_peak_sample),
        static_cast<unsigned long long>(diag.env_voice_peak_key_on_events),
        static_cast<unsigned long long>(diag.env_voice_peak_key_off_events),
        static_cast<unsigned>(diag.env_voice_peak_endx_mask & 0x00FFFFFFu));
    ImGui::Text("Audible peak ctx: sample %llu | KON/KOFF %llu/%llu | ENDX 0x%06X",
        static_cast<unsigned long long>(diag.audible_voice_peak_sample),
        static_cast<unsigned long long>(diag.audible_voice_peak_key_on_events),
        static_cast<unsigned long long>(diag.audible_voice_peak_key_off_events),
        static_cast<unsigned>(diag.audible_voice_peak_endx_mask & 0x00FFFFFFu));
    ImGui::Text("KEY ON/OFF Events: %llu / %llu",
        static_cast<unsigned long long>(diag.key_on_events),
        static_cast<unsigned long long>(diag.key_off_events));
    ImGui::Text("Voice offs (END/RELEASE): %llu / %llu",
        static_cast<unsigned long long>(diag.off_due_to_end_flag),
        static_cast<unsigned long long>(diag.release_to_off_events));
    ImGui::Text("Ignored while SPU disabled (KON/KOFF): %llu / %llu",
        static_cast<unsigned long long>(diag.keyon_ignored_while_disabled),
        static_cast<unsigned long long>(diag.keyoff_ignored_while_disabled));
    ImGui::Text("SPUCNT disable events / forced-off voices: %llu / %llu",
        static_cast<unsigned long long>(diag.spucnt_enable_clear_events),
        static_cast<unsigned long long>(diag.spu_disable_forced_off_voices));
    ImGui::Text("KON writes L/H: %llu / %llu | bits collected: %llu | multiwrite windows: %llu",
        static_cast<unsigned long long>(diag.kon_write_events_low),
        static_cast<unsigned long long>(diag.kon_write_events_high),
        static_cast<unsigned long long>(diag.kon_bits_collected),
        static_cast<unsigned long long>(diag.kon_multiwrite_same_sample_events));
    ImGui::Text("KOFF writes L/H: %llu / %llu | bits collected: %llu | multiwrite windows: %llu",
        static_cast<unsigned long long>(diag.koff_write_events_low),
        static_cast<unsigned long long>(diag.koff_write_events_high),
        static_cast<unsigned long long>(diag.koff_bits_collected),
        static_cast<unsigned long long>(diag.koff_multiwrite_same_sample_events));
    ImGui::Text("ENDX Mask: 0x%06X",
        static_cast<unsigned>(runtime_snapshot_.spu_endx_mask & 0x00FFFFFFu));

    ImGui::Spacing();
    ImGui::Text("Voice Levels");
    ImGuiTableFlags table_flags =
        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
        ImGuiTableFlags_SizingStretchSame;
    const float voice_row_height =
        (ImGui::GetTextLineHeight() * 2.0f) + ImGui::GetFrameHeight() +
        (ImGui::GetStyle().ItemSpacing.y * 3.0f);
    if (ImGui::BeginTable("SPUSoundStatusVoices", 4, table_flags)) {
        ImGui::TableSetupColumn("##voice_col_0", ImGuiTableColumnFlags_WidthStretch,
            1.0f);
        ImGui::TableSetupColumn("##voice_col_1", ImGuiTableColumnFlags_WidthStretch,
            1.0f);
        ImGui::TableSetupColumn("##voice_col_2", ImGuiTableColumnFlags_WidthStretch,
            1.0f);
        ImGui::TableSetupColumn("##voice_col_3", ImGuiTableColumnFlags_WidthStretch,
            1.0f);
        for (size_t voice = 0; voice < runtime_snapshot_.spu_voice_level_l.size();
            ++voice) {
            if ((voice % 4u) == 0u) {
                ImGui::TableNextRow(ImGuiTableRowFlags_None, voice_row_height);
            }
            ImGui::TableSetColumnIndex(static_cast<int>(voice % 4u));

            const s16 level_l = runtime_snapshot_.spu_voice_level_l[voice];
            const s16 level_r = runtime_snapshot_.spu_voice_level_r[voice];
            const int abs_l = (level_l < 0) ? -static_cast<int>(level_l)
                : static_cast<int>(level_l);
            const int abs_r = (level_r < 0) ? -static_cast<int>(level_r)
                : static_cast<int>(level_r);
            const int peak = std::max(abs_l, abs_r);
            const float meter = std::min(1.0f, static_cast<float>(peak) / 32767.0f);
            const bool active = runtime_snapshot_.spu_voice_active[voice];

            ImGui::TextColored(active ? ImVec4(0.4f, 0.9f, 0.4f, 1.0f)
                : ImVec4(0.55f, 0.55f, 0.55f, 1.0f),
                "V%02u", static_cast<unsigned>(voice));
            ImGui::SameLine();
            ImGui::TextUnformatted(active ? "ON" : "OFF");
            ImGui::ProgressBar(meter, ImVec2(-1.0f, 0.0f));
            ImGui::Text("L:%6d  R:%6d", static_cast<int>(level_l),
                static_cast<int>(level_r));
        }
        ImGui::EndTable();
    }
}

void App::panel_sound_status() {
    ImGui::SetNextWindowSize(ImVec2(720, 500), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Voice Levels", &show_sound_status_)) {
        ImGui::Text("Voice Levels");
        ImGuiTableFlags table_flags =
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_SizingStretchSame;
        const float voice_row_height =
            (ImGui::GetTextLineHeight() * 2.0f) + ImGui::GetFrameHeight() +
            (ImGui::GetStyle().ItemSpacing.y * 3.0f);
        if (ImGui::BeginTable("SPUVoiceLevelsOnly", 4, table_flags)) {
            ImGui::TableSetupColumn("##voice_col_0", ImGuiTableColumnFlags_WidthStretch,
                1.0f);
            ImGui::TableSetupColumn("##voice_col_1", ImGuiTableColumnFlags_WidthStretch,
                1.0f);
            ImGui::TableSetupColumn("##voice_col_2", ImGuiTableColumnFlags_WidthStretch,
                1.0f);
            ImGui::TableSetupColumn("##voice_col_3", ImGuiTableColumnFlags_WidthStretch,
                1.0f);
            for (size_t voice = 0; voice < runtime_snapshot_.spu_voice_level_l.size();
                ++voice) {
                if ((voice % 4u) == 0u) {
                    ImGui::TableNextRow(ImGuiTableRowFlags_None, voice_row_height);
                }
                ImGui::TableSetColumnIndex(static_cast<int>(voice % 4u));

                const s16 level_l = runtime_snapshot_.spu_voice_level_l[voice];
                const s16 level_r = runtime_snapshot_.spu_voice_level_r[voice];
                const int abs_l = (level_l < 0) ? -static_cast<int>(level_l)
                    : static_cast<int>(level_l);
                const int abs_r = (level_r < 0) ? -static_cast<int>(level_r)
                    : static_cast<int>(level_r);
                const int peak = std::max(abs_l, abs_r);
                const float meter = std::min(1.0f, static_cast<float>(peak) / 32767.0f);
                const bool active = runtime_snapshot_.spu_voice_active[voice];

                ImGui::TextColored(active ? ImVec4(0.4f, 0.9f, 0.4f, 1.0f)
                    : ImVec4(0.55f, 0.55f, 0.55f, 1.0f),
                    "V%02u", static_cast<unsigned>(voice));
                ImGui::SameLine();
                ImGui::TextUnformatted(active ? "ON" : "OFF");
                ImGui::ProgressBar(meter, ImVec2(-1.0f, 0.0f));
                ImGui::Text("L:%6d  R:%6d", static_cast<int>(level_l),
                    static_cast<int>(level_r));
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void App::panel_performance() {
    ImGui::SetNextWindowSize(ImVec2(400, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Performance Profiler", &show_perf_)) {
        if (!has_started_emulation_) {
            ImGui::Text("Emulation not running.");
            ImGui::End();
            return;
        }

        const auto& stats = runtime_snapshot_.profiling;
        ImGui::Text("Frame Time Breakdown:");
        ImGui::Separator();

        auto row = [](const char* label, double ms, ImVec4 color) {
            ImGui::Text("%-10s:", label);
            ImGui::SameLine(100);
            ImGui::TextColored(color, "%.3f ms", ms);
            };

        if (g_profile_detailed_timing) {
            row("CPU*", stats.cpu_ms, ImVec4(0.4f, 0.8f, 0.4f, 1.0f));
            row("GPU", stats.gpu_ms, ImVec4(0.8f, 0.4f, 0.4f, 1.0f));
            row("SPU", stats.spu_ms, ImVec4(0.4f, 0.4f, 0.8f, 1.0f));
            row("DMA", stats.dma_ms, ImVec4(0.8f, 0.8f, 0.4f, 1.0f));
            row("Timers", stats.timers_ms, ImVec4(0.4f, 0.8f, 0.8f, 1.0f));
            row("CDROM", stats.cdrom_ms, ImVec4(0.8f, 0.4f, 0.8f, 1.0f));
            ImGui::TextDisabled("*CPU excludes time already attributed to GPU.");
            ImGui::TextDisabled(
                "GPU here is CPU-side emulated GPU command time, not host GPU timestamp/present time.");

            draw_performance_gpu_dip_diagnostics();
        }
        else {
            ImGui::TextDisabled("Detailed subsystem timings are disabled.");
        }
        ImGui::Separator();
        row("Core", runtime_snapshot_.core_frame_ms, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        row("Render", render_ms_, ImVec4(0.9f, 0.9f, 0.7f, 1.0f));
        row("Swap", swap_ms_, ImVec4(0.7f, 0.9f, 0.9f, 1.0f));
        row("Present", present_ms_, ImVec4(0.7f, 0.9f, 0.9f, 1.0f));
        const CpuBackendStats& backend = runtime_snapshot_.cpu_backend_stats;
        ImGui::Separator();
        draw_cpu_backend_mode_summary(backend, runtime_snapshot_.cpu_backend);
        ImGui::Text("Decoded blocks: %u  Cache: %llu / %llu  Invalidations: %llu",
            backend.block_count,
            static_cast<unsigned long long>(backend.cache_hits),
            static_cast<unsigned long long>(backend.cache_misses),
            static_cast<unsigned long long>(backend.invalidations));
        ImGui::Text("Invalidation queries: %llu  no-code exits: %llu",
            static_cast<unsigned long long>(backend.invalidation_queries),
            static_cast<unsigned long long>(
                backend.invalidation_fast_no_code_page_exits));
        ImGui::Text("Invalidation blocks: examined %llu  invalidated %llu",
            static_cast<unsigned long long>(backend.invalidation_blocks_examined),
            static_cast<unsigned long long>(
                backend.invalidation_blocks_invalidated));
        ImGui::Text("Decoded/native/fallback instructions: %llu / %llu / %llu",
            static_cast<unsigned long long>(backend.decoded_instructions),
            static_cast<unsigned long long>(backend.native_instructions),
            static_cast<unsigned long long>(backend.fallback_instructions));
        if (backend.forced_interpreter_instructions != 0 ||
            backend.forced_interpreter_last_reason !=
                CpuForcedInterpreterReason::None) {
            ImGui::Text("Forced interpreter: %s  slices %llu  instr %llu",
                cpu_forced_interpreter_reason_name(
                    backend.forced_interpreter_last_reason),
                static_cast<unsigned long long>(
                    backend.forced_interpreter_slices),
                static_cast<unsigned long long>(
                    backend.forced_interpreter_instructions));
        }
        ImGui::Text("Native: available %u  attempts %llu  successes %llu  compiled %llu",
            backend.native_available ? 1u : 0u,
            static_cast<unsigned long long>(backend.native_compile_attempts),
            static_cast<unsigned long long>(backend.native_compile_successes),
            static_cast<unsigned long long>(backend.native_blocks_compiled));
        ImGui::Text("Native entries: %llu  cycles %llu  code %zu bytes",
            static_cast<unsigned long long>(backend.native_block_entries),
            static_cast<unsigned long long>(backend.native_cycles),
            backend.native_code_bytes);
        ImGui::Text("Native fallback: rejected %llu  compile fail %llu  decoded %llu",
            static_cast<unsigned long long>(
                backend.native_rejected_unsafe_blocks),
            static_cast<unsigned long long>(backend.native_compile_failures),
            static_cast<unsigned long long>(
                backend.native_to_decoded_fallbacks));
        ImGui::Text("Native gating: cold %llu  short %llu",
            static_cast<unsigned long long>(backend.native_hot_threshold_skips),
            static_cast<unsigned long long>(backend.native_short_block_skips));
        ImGui::Text("Native rejects: branch %llu  mem %llu  cop0 %llu  cop2 %llu",
            static_cast<unsigned long long>(backend.native_reject_branch),
            static_cast<unsigned long long>(backend.native_reject_memory),
            static_cast<unsigned long long>(backend.native_reject_cop0),
            static_cast<unsigned long long>(backend.native_reject_cop2));
        ImGui::Text("Native rejects: exc/unk %llu  state %llu  budget %llu  icache %llu",
            static_cast<unsigned long long>(
                backend.native_reject_exception_unknown),
            static_cast<unsigned long long>(backend.native_reject_unsafe_state),
            static_cast<unsigned long long>(backend.native_reject_budget),
            static_cast<unsigned long long>(backend.native_reject_icache));
        ImGui::Text("Native state rejects: pc %llu  branch-delay %llu  load-delay %llu",
            static_cast<unsigned long long>(backend.native_reject_pc_state),
            static_cast<unsigned long long>(
                backend.native_reject_branch_delay_state),
            static_cast<unsigned long long>(
                backend.native_reject_load_delay_state));
        ImGui::Text("Native state rejects: irq %llu  invalidated %llu  other %llu",
            static_cast<unsigned long long>(backend.native_reject_irq_state),
            static_cast<unsigned long long>(
                backend.native_reject_invalidated_state),
            static_cast<unsigned long long>(backend.native_reject_other_state));
        ImGui::Text("Native rejects: mmio %llu  unaligned %llu",
            static_cast<unsigned long long>(backend.native_reject_mmio),
            static_cast<unsigned long long>(backend.native_reject_unaligned));
        ImGui::Text("Block entries: decoded %llu  native %llu  avg %.2f / %.2f instr",
            static_cast<unsigned long long>(backend.decoded_block_entries),
            static_cast<unsigned long long>(backend.native_block_entries),
            backend.decoded_block_entries == 0
                ? 0.0
                : static_cast<double>(backend.decoded_instructions) /
                      static_cast<double>(backend.decoded_block_entries),
            backend.native_block_entries == 0
                ? 0.0
                : static_cast<double>(backend.native_instructions) /
                      static_cast<double>(backend.native_block_entries));
        ImGui::Text("Memory helpers: %llu  MMIO: %llu  Exceptions: %llu",
            static_cast<unsigned long long>(backend.memory_helper_calls),
            static_cast<unsigned long long>(backend.mmio_accesses),
            static_cast<unsigned long long>(backend.exceptions));
        ImGui::Text("Native memory helpers: %llu  fast L/S: %llu / %llu  exception exits: %llu",
            static_cast<unsigned long long>(
                backend.native_memory_helper_calls),
            static_cast<unsigned long long>(
                backend.native_memory_fastpath_loads),
            static_cast<unsigned long long>(
                backend.native_memory_fastpath_stores),
            static_cast<unsigned long long>(
                backend.native_memory_exception_exits));
        ImGui::Text("Helper load-delay: entries %llu  passes %llu  fallbacks %llu",
            static_cast<unsigned long long>(
                backend.native_helper_load_delay_entries),
            static_cast<unsigned long long>(
                backend.native_helper_load_delay_passes),
            static_cast<unsigned long long>(
                backend.native_helper_load_delay_fallbacks));
        if (config_vsync_ && swap_ms_ > 8.0) {
            ImGui::TextDisabled(
                "Swap includes VSync/compositor wait. Disable VSync to profile CPU cost.");
        }

        float budget_ms = 1000.0f / 60.0f;
        float usage = static_cast<float>(runtime_snapshot_.core_frame_ms / budget_ms);
        ImGui::Spacing();
        ImGui::Text("Core Frame Budget Usage (%.1f%%):", usage * 100.0f);
        ImGui::ProgressBar(usage, ImVec2(-1.0f, 0.0f));
    }
    ImGui::End();
}

void App::panel_about() {
    ImGui::SetNextWindowSize(ImVec2(400, 200), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("About VibeStation", &show_about_,
        ImGuiWindowFlags_NoResize)) {
        ImGui::TextColored(ImVec4(0.6f, 0.4f, 1.0f, 1.0f), "VibeStation v0.5.2");
        ImGui::Separator();
        ImGui::Text("A PlayStation 1 emulator");
        ImGui::Spacing();
        ImGui::Text("CPU: MIPS R3000A %s",
            cpu_execution_mode_name(effective_cpu_execution_mode()));
        ImGui::Text("GPU: Software rasterizer");
        ImGui::Text("GTE: Fixed-point geometry engine");
        ImGui::Text("SPU: Gaussian + reverb core (stage 2)");
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.6f, 1.0f),
            "Built with SDL2 + Dear ImGui + OpenGL 3.3");
    }
    ImGui::End();
}

void App::panel_debug_cpu() {
    ImGui::SetNextWindowSize(ImVec2(450, 500), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("CPU Debug", &show_debug_cpu_)) {
        const bool running = emu_runner_.is_running();
        const CpuBackendStats backend_stats =
            running ? runtime_snapshot_.cpu_backend_stats
                    : system_->cpu().cpu_backend_stats();
        const CpuExecutionMode requested_backend =
            running ? runtime_snapshot_.cpu_backend
                    : effective_cpu_execution_mode();
        draw_cpu_backend_mode_summary(backend_stats, requested_backend);
        ImGui::Text("Blocks: %u  Decoded entries: %llu  Native entries: %llu",
            backend_stats.block_count,
            static_cast<unsigned long long>(backend_stats.decoded_block_entries),
            static_cast<unsigned long long>(backend_stats.native_block_entries));
        ImGui::Text("Decoded/native/fallback instr: %llu / %llu / %llu",
            static_cast<unsigned long long>(backend_stats.decoded_instructions),
            static_cast<unsigned long long>(backend_stats.native_instructions),
            static_cast<unsigned long long>(backend_stats.fallback_instructions));
        if (backend_stats.forced_interpreter_instructions != 0 ||
            backend_stats.forced_interpreter_last_reason !=
                CpuForcedInterpreterReason::None) {
            ImGui::Text("Forced interpreter: %s  slices %llu  instr %llu",
                cpu_forced_interpreter_reason_name(
                    backend_stats.forced_interpreter_last_reason),
                static_cast<unsigned long long>(
                    backend_stats.forced_interpreter_slices),
                static_cast<unsigned long long>(
                    backend_stats.forced_interpreter_instructions));
        }
        ImGui::Text("Native: available %u  attempts %llu  successes %llu  compiled %llu",
            backend_stats.native_available ? 1u : 0u,
            static_cast<unsigned long long>(
                backend_stats.native_compile_attempts),
            static_cast<unsigned long long>(
                backend_stats.native_compile_successes),
            static_cast<unsigned long long>(
                backend_stats.native_blocks_compiled));
        ImGui::Text("Native entries: %llu  cycles %llu  code %zu bytes",
            static_cast<unsigned long long>(
                backend_stats.native_block_entries),
            static_cast<unsigned long long>(backend_stats.native_cycles),
            backend_stats.native_code_bytes);
        ImGui::Text("Native fallback: rejected %llu  compile fail %llu  decoded %llu",
            static_cast<unsigned long long>(
                backend_stats.native_rejected_unsafe_blocks),
            static_cast<unsigned long long>(
                backend_stats.native_compile_failures),
            static_cast<unsigned long long>(
                backend_stats.native_to_decoded_fallbacks));
        ImGui::Text("Native gating: cold %llu  short %llu",
            static_cast<unsigned long long>(
                backend_stats.native_hot_threshold_skips),
            static_cast<unsigned long long>(
                backend_stats.native_short_block_skips));
        ImGui::Text("Native rejects: branch %llu  mem %llu  cop0 %llu  cop2 %llu",
            static_cast<unsigned long long>(backend_stats.native_reject_branch),
            static_cast<unsigned long long>(backend_stats.native_reject_memory),
            static_cast<unsigned long long>(backend_stats.native_reject_cop0),
            static_cast<unsigned long long>(backend_stats.native_reject_cop2));
        ImGui::Text("Native rejects: exc/unk %llu  state %llu  budget %llu  icache %llu",
            static_cast<unsigned long long>(
                backend_stats.native_reject_exception_unknown),
            static_cast<unsigned long long>(
                backend_stats.native_reject_unsafe_state),
            static_cast<unsigned long long>(backend_stats.native_reject_budget),
            static_cast<unsigned long long>(backend_stats.native_reject_icache));
        ImGui::Text("Native state rejects: pc %llu  branch-delay %llu  load-delay %llu",
            static_cast<unsigned long long>(
                backend_stats.native_reject_pc_state),
            static_cast<unsigned long long>(
                backend_stats.native_reject_branch_delay_state),
            static_cast<unsigned long long>(
                backend_stats.native_reject_load_delay_state));
        ImGui::Text("Native state rejects: irq %llu  invalidated %llu  other %llu",
            static_cast<unsigned long long>(
                backend_stats.native_reject_irq_state),
            static_cast<unsigned long long>(
                backend_stats.native_reject_invalidated_state),
            static_cast<unsigned long long>(
                backend_stats.native_reject_other_state));
        ImGui::Text("Native rejects: mmio %llu  unaligned %llu",
            static_cast<unsigned long long>(backend_stats.native_reject_mmio),
            static_cast<unsigned long long>(
                backend_stats.native_reject_unaligned));
        ImGui::Text("Native memory: helpers %llu  fast L/S %llu / %llu  exception exits %llu",
            static_cast<unsigned long long>(
                backend_stats.native_memory_helper_calls),
            static_cast<unsigned long long>(
                backend_stats.native_memory_fastpath_loads),
            static_cast<unsigned long long>(
                backend_stats.native_memory_fastpath_stores),
            static_cast<unsigned long long>(
                backend_stats.native_memory_exception_exits));
        ImGui::Text("Helper load-delay: entries %llu  passes %llu  fallbacks %llu",
            static_cast<unsigned long long>(
                backend_stats.native_helper_load_delay_entries),
            static_cast<unsigned long long>(
                backend_stats.native_helper_load_delay_passes),
            static_cast<unsigned long long>(
                backend_stats.native_helper_load_delay_fallbacks));
        ImGui::Text("Invalidation: queries %llu  no-code %llu  examined %llu  hit %llu",
            static_cast<unsigned long long>(backend_stats.invalidation_queries),
            static_cast<unsigned long long>(
                backend_stats.invalidation_fast_no_code_page_exits),
            static_cast<unsigned long long>(
                backend_stats.invalidation_blocks_examined),
            static_cast<unsigned long long>(
                backend_stats.invalidation_blocks_invalidated));
        ImGui::Separator();
        if (running) {
            ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.4f, 1.0f),
                "Pause emulation to inspect registers.");
        }
        else {
            ImGui::Text("PC: 0x%08X", system_->cpu().pc());
            ImGui::Text("Cycles: %llu",
                (unsigned long long)system_->cpu().cycle_count());
            ImGui::Separator();

            if (ImGui::BeginTable("Registers", 4,
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                const char* reg_names[] = {
                    "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0", "t1", "t2",
                    "t3",   "t4", "t5", "t6", "t7", "s0", "s1", "s2", "s3", "s4", "s5",
                    "s6",   "s7", "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra" };
                for (int i = 0; i < 32; i++) {
                    ImGui::TableNextColumn();
                    ImGui::TextColored(ImVec4(0.6f, 0.5f, 0.9f, 1.0f), "$%s",
                        reg_names[i]);
                    ImGui::SameLine(55);
                    ImGui::Text("0x%08X", system_->cpu().reg(i));
                }
                ImGui::EndTable();
            }
        }

        ImGui::Spacing();
        if (ImGui::Button("Step") && !running) {
            system_->step();
        }
        ImGui::SameLine();
        if (ImGui::Button(running ? "Pause" : "Run")) {
            if (running) {
                emu_runner_.pause_and_wait_idle();
            }
            else {
                apply_memory_card_settings(false);
                has_started_emulation_ = true;
                emu_runner_.set_running(true);
            }
        }
    }
    ImGui::End();
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
    if (!input_recorder_.init(disc_path, "VibeStation v0.5.2")) {
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
    if (!input_recorder_.init(disc_path, "VibeStation v0.5.2")) {
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
    if (!input_recorder_.init(disc_path, "VibeStation v0.5.2")) {
        status_message_ = "Input movie: " + input_recorder_.status().status_message;
        return false;
    }

    input_recorder_config_ = input_recorder_.config();
    input_movie_cli_pending_ = false;
    status_message_ = "Input playback started: " +
        input_recorder_.status().playback_path;
    return true;
}

void App::sync_ram_reaper_config() {
    if (!system_) {
        return;
    }
    System::RamReaperConfig cfg{};
    cfg.enabled = ram_reaper_enabled_;
    cfg.range_start = ram_reaper_range_start_;
    cfg.range_end = ram_reaper_range_end_;
    cfg.writes_per_frame = ram_reaper_writes_per_frame_;
    cfg.intensity_percent = ram_reaper_intensity_percent_;
    cfg.affect_main_ram = ram_reaper_affect_main_ram_;
    cfg.affect_vram = ram_reaper_affect_vram_;
    cfg.affect_spu_ram = ram_reaper_affect_spu_ram_;
    cfg.use_custom_seed = ram_reaper_use_custom_seed_;
    cfg.seed = ram_reaper_seed_;
    system_->set_ram_reaper_config(cfg);
    ram_reaper_active_seed_ = system_->ram_reaper_last_seed();
    ram_reaper_total_mutations_ = system_->ram_reaper_total_mutations();
}

void App::disable_ram_reaper_mode() {
    ram_reaper_enabled_ = false;
    if (system_) {
        system_->disable_ram_reaper();
    }
}

void App::sync_gpu_reaper_config() {
    if (!system_) {
        return;
    }
    System::GpuReaperConfig cfg{};
    cfg.enabled = gpu_reaper_enabled_;
    cfg.writes_per_frame = gpu_reaper_writes_per_frame_;
    cfg.intensity_percent = gpu_reaper_intensity_percent_;
    cfg.affect_geometry = gpu_reaper_affect_geometry_;
    cfg.affect_texture_state = gpu_reaper_affect_texture_state_;
    cfg.affect_display_state = gpu_reaper_affect_display_state_;
    cfg.use_custom_seed = gpu_reaper_use_custom_seed_;
    cfg.seed = gpu_reaper_seed_;
    system_->set_gpu_reaper_config(cfg);
    gpu_reaper_active_seed_ = system_->gpu_reaper_last_seed();
    gpu_reaper_total_mutations_ = system_->gpu_reaper_total_mutations();
}

void App::disable_gpu_reaper_mode() {
    gpu_reaper_enabled_ = false;
    if (system_) {
        system_->disable_gpu_reaper();
    }
}

void App::sync_sound_reaper_config() {
    if (!system_) {
        return;
    }
    System::SoundReaperConfig cfg{};
    cfg.enabled = sound_reaper_enabled_;
    cfg.writes_per_frame = sound_reaper_writes_per_frame_;
    cfg.intensity_percent = sound_reaper_intensity_percent_;
    cfg.affect_pitch = sound_reaper_affect_pitch_;
    cfg.affect_envelope = sound_reaper_affect_envelope_;
    cfg.affect_reverb = sound_reaper_affect_reverb_;
    cfg.affect_mixer = sound_reaper_affect_mixer_;
    cfg.use_custom_seed = sound_reaper_use_custom_seed_;
    cfg.seed = sound_reaper_seed_;
    system_->set_sound_reaper_config(cfg);
    sound_reaper_active_seed_ = system_->sound_reaper_last_seed();
    sound_reaper_total_mutations_ = system_->sound_reaper_total_mutations();
}

void App::disable_sound_reaper_mode() {
    sound_reaper_enabled_ = false;
    if (system_) {
        system_->disable_sound_reaper();
    }
}


void App::set_grim_reaper_mode(bool enabled) {
    if (enabled) {
        grim_reaper_mode_active_ = true;
        if (grim_reaper_keep_console_logs_) {
            return;
        }
        if (!grim_reaper_logs_suppressed_) {
            grim_reaper_saved_log_mask_ = g_log_category_mask;
            grim_reaper_saved_log_level_ = g_log_level;
            grim_reaper_logs_suppressed_ = true;
        }
        g_log_category_mask = 0;
        g_log_level = LogLevel::Error;
        return;
    }

    grim_reaper_mode_active_ = false;
    if (grim_reaper_logs_suppressed_) {
        g_log_category_mask = grim_reaper_saved_log_mask_;
        g_log_level = grim_reaper_saved_log_level_;
        grim_reaper_logs_suppressed_ = false;
    }
}
bool App::reap_and_reboot_bios() {
    if (!system_ || bios_path_.empty()) {
        status_message_ = "Load a BIOS first.";
        return false;
    }

    std::ifstream in(bios_path_, std::ios::binary);
    if (!in.is_open()) {
        status_message_ = "Failed to open source BIOS file.";
        return false;
    }

    in.seekg(0, std::ios::end);
    const std::streamoff size_off = in.tellg();
    if (size_off <= 0) {
        status_message_ = "Source BIOS is empty or unreadable.";
        return false;
    }
    const size_t bios_size = static_cast<size_t>(size_off);
    in.seekg(0, std::ios::beg);

    std::vector<u8> bios_data(bios_size, 0);
    in.read(reinterpret_cast<char*>(bios_data.data()),
        static_cast<std::streamsize>(bios_data.size()));
    if (!in) {
        status_message_ = "Failed reading source BIOS.";
        return false;
    }

    grim_reaper_area_index_ =
        std::max(0, std::min(kGrimReaperRangeCount - 1, grim_reaper_area_index_));

    auto parse_hex = [](const char* text, size_t& out) -> bool {
        if (!text) {
            return false;
        }
        while (*text != '\0' && std::isspace(static_cast<unsigned char>(*text))) {
            ++text;
        }
        if (*text == '\0') {
            return false;
        }
        char* end_ptr = nullptr;
        const unsigned long long value = std::strtoull(text, &end_ptr, 16);
        if (end_ptr == text) {
            return false;
        }
        while (*end_ptr != '\0' && std::isspace(static_cast<unsigned char>(*end_ptr))) {
            ++end_ptr;
        }
        if (*end_ptr != '\0') {
            return false;
        }
        out = static_cast<size_t>(value);
        return true;
        };

    size_t start = 0;
    size_t end = bios_data.size() - 1u;
    std::string range_slug;

    if (grim_reaper_area_index_ == (kGrimReaperRangeCount - 1)) {
        size_t custom_start = 0;
        if (!parse_hex(grim_reaper_custom_start_hex_, custom_start)) {
            status_message_ = "Invalid custom start hex.";
            return false;
        }

        size_t custom_end = 0;
        bool has_custom_end = parse_hex(grim_reaper_custom_end_hex_, custom_end);
        if (!has_custom_end || custom_end == 0) {
            custom_end = bios_data.size() - 1u;
        }

        start = std::min(custom_start, bios_data.size() - 1u);
        end = std::min(custom_end, bios_data.size() - 1u);
        range_slug = "custom";
    }
    else {
        const GrimReaperRange range = kGrimReaperRanges[grim_reaper_area_index_];
        if (bios_data.size() <= static_cast<size_t>(range.start)) {
            status_message_ = "Selected corruption range is outside BIOS size.";
            return false;
        }
        start = static_cast<size_t>(range.start);
        end = std::min(static_cast<size_t>(range.end), bios_data.size() - 1u);
        range_slug = range.slug;
    }

    if (end < start) {
        status_message_ = "Selected corruption range is invalid.";
        return false;
    }

    const size_t span = end - start + 1u;
    const float pct_max = (grim_reaper_area_index_ == 0) ? 0.1f : 100.0f;
    const float pct = std::max(0.001f, std::min(pct_max, grim_reaper_random_percent_));
    size_t mutations =
        static_cast<size_t>((pct / 100.0f) * static_cast<float>(span));
    if (mutations == 0) {
        mutations = 1;
    }

    const u64 seed =
        grim_use_custom_seed_
        ? grim_seed_
        : ((static_cast<u64>(std::random_device{}()) << 32) ^
            static_cast<u64>(std::random_device{}()));
    grim_last_used_seed_ = seed;
    grim_seed_ = seed;
    std::mt19937 rng;
    seed_mt19937(rng, seed);

    for (size_t i = 0; i < mutations; ++i) {
        const size_t idx = start + (static_cast<size_t>(rng()) % span);
        bios_data[idx] = static_cast<u8>(rng() & 0xFFu);
    }
    const std::filesystem::path src_path = std::filesystem::path(bios_path_);
    const std::filesystem::path out_path =
        src_path.parent_path() /
        (src_path.stem().string() + "_grim_" + range_slug + src_path.extension().string());

    std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        status_message_ = "Failed to create corrupted BIOS copy.";
        return false;
    }
    out.write(reinterpret_cast<const char*>(bios_data.data()),
        static_cast<std::streamsize>(bios_data.size()));
    if (!out) {
        status_message_ = "Failed writing corrupted BIOS copy.";
        return false;
    }

    emu_runner_.pause_and_wait_idle();
    disable_ram_reaper_mode();
    disable_gpu_reaper_mode();
    disable_sound_reaper_mode();
    set_grim_reaper_mode(true);
    if (!system_->load_bios(out_path.string())) {
        set_grim_reaper_mode(false);
        status_message_ = "Failed to load corrupted BIOS copy.";
        return false;
    }

    grim_reaper_last_mutations_ = static_cast<u32>(mutations);
    grim_reaper_last_output_path_ = out_path.string();

    has_started_emulation_ = false;
    system_->reset();
    apply_memory_card_settings(false);
    has_started_emulation_ = true;
    emu_runner_.set_running(true);

    status_message_ = "Reaped BIOS copy loaded (seed " + std::to_string(seed) + "): " +
        out_path.filename().string();
    return true;
}

bool App::reap_and_reboot_bios_batch() {
    if (!system_ || bios_path_.empty()) {
        status_message_ = "Load a BIOS first.";
        return false;
    }

    const bool do_intro = grim_batch_intro_enabled_;
    const bool do_charset = grim_batch_charset_enabled_;
    const bool do_end = grim_batch_end_enabled_;
    if (!do_intro && !do_charset && !do_end) {
        status_message_ = "Select at least one batch corruption range.";
        return false;
    }

    std::ifstream in(bios_path_, std::ios::binary);
    if (!in.is_open()) {
        status_message_ = "Failed to open source BIOS file.";
        return false;
    }

    in.seekg(0, std::ios::end);
    const std::streamoff size_off = in.tellg();
    if (size_off <= 0) {
        status_message_ = "Source BIOS is empty or unreadable.";
        return false;
    }
    const size_t bios_size = static_cast<size_t>(size_off);
    in.seekg(0, std::ios::beg);

    std::vector<u8> bios_data(bios_size, 0);
    in.read(reinterpret_cast<char*>(bios_data.data()),
        static_cast<std::streamsize>(bios_data.size()));
    if (!in) {
        status_message_ = "Failed reading source BIOS.";
        return false;
    }

    size_t total_mutations = 0;
    auto next_random_seed = []() -> u64 {
        return ((static_cast<u64>(std::random_device{}()) << 32) ^
            static_cast<u64>(std::random_device{}()));
        };
    auto apply_range = [&](const GrimReaperRange& range, float percent,
        float max_percent, u64& seed_slot) -> bool {
            if (bios_data.size() <= static_cast<size_t>(range.start)) {
                return false;
            }

            const size_t start = static_cast<size_t>(range.start);
            const size_t end = std::min(static_cast<size_t>(range.end), bios_data.size() - 1u);
            if (end < start) {
                return false;
            }

            const float pct = std::max(0.001f, std::min(max_percent, percent));
            const size_t span = end - start + 1u;
            size_t mutations =
                static_cast<size_t>((pct / 100.0f) * static_cast<float>(span));
            if (mutations == 0) {
                mutations = 1;
            }

            const u64 seed = grim_batch_use_custom_seeds_ ? seed_slot : next_random_seed();
            seed_slot = seed;
            grim_last_used_seed_ = seed;
            std::mt19937 rng;
            seed_mt19937(rng, seed);
            for (size_t i = 0; i < mutations; ++i) {
                const size_t idx = start + (static_cast<size_t>(rng()) % span);
                bios_data[idx] = static_cast<u8>(rng() & 0xFFu);
            }

            total_mutations += mutations;
            return true;
        };

    std::string slug = "batch";
    if (do_intro) {
        if (!apply_range(kGrimReaperRanges[0], grim_batch_intro_percent_, 0.1f,
            grim_batch_intro_seed_)) {
            status_message_ = "Intro range is outside BIOS size.";
            return false;
        }
        slug += "_intro";
    }
    if (do_charset) {
        if (!apply_range(kGrimReaperRanges[1], grim_batch_charset_percent_, 100.0f,
            grim_batch_charset_seed_)) {
            status_message_ = "Character Sets range is outside BIOS size.";
            return false;
        }
        slug += "_charset";
    }
    if (do_end) {
        if (!apply_range(kGrimReaperRanges[2], grim_batch_end_percent_, 100.0f,
            grim_batch_end_seed_)) {
            status_message_ = "End range is outside BIOS size.";
            return false;
        }
        slug += "_end";
    }

    const std::filesystem::path src_path = std::filesystem::path(bios_path_);
    const std::filesystem::path out_path =
        src_path.parent_path() /
        (src_path.stem().string() + "_grim_" + slug + src_path.extension().string());

    std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        status_message_ = "Failed to create corrupted BIOS copy.";
        return false;
    }
    out.write(reinterpret_cast<const char*>(bios_data.data()),
        static_cast<std::streamsize>(bios_data.size()));
    if (!out) {
        status_message_ = "Failed writing corrupted BIOS copy.";
        return false;
    }

    emu_runner_.pause_and_wait_idle();
    disable_ram_reaper_mode();
    disable_gpu_reaper_mode();
    disable_sound_reaper_mode();
    set_grim_reaper_mode(true);
    if (!system_->load_bios(out_path.string())) {
        set_grim_reaper_mode(false);
        status_message_ = "Failed to load corrupted BIOS copy.";
        return false;
    }

    grim_reaper_last_mutations_ = static_cast<u32>(total_mutations);
    grim_reaper_last_output_path_ = out_path.string();

    has_started_emulation_ = false;
    system_->reset();
    apply_memory_card_settings(false);
    has_started_emulation_ = true;
    emu_runner_.set_running(true);

    status_message_ = "Batch reaped BIOS loaded (last seed " +
        std::to_string(grim_last_used_seed_) + "): " +
        out_path.filename().string();
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
    std::ifstream in(kAppConfigFileName);
    if (!in.is_open()) {
        return;
    }

    auto trim = [](std::string& s) {
        const size_t begin = s.find_first_not_of(" \t\r\n");
        if (begin == std::string::npos) {
            s.clear();
            return;
        }
        const size_t end = s.find_last_not_of(" \t\r\n");
        s = s.substr(begin, end - begin + 1);
        };
    auto parse_bool = [](const std::string& v, bool fallback) {
        if (v == "1" || v == "true" || v == "TRUE" || v == "on" || v == "ON") {
            return true;
        }
        if (v == "0" || v == "false" || v == "FALSE" || v == "off" || v == "OFF") {
            return false;
        }
        return fallback;
        };
    auto parse_cpu_mode = [](std::string v, CpuExecutionMode fallback) {
        std::transform(v.begin(), v.end(), v.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        v.erase(std::remove(v.begin(), v.end(), '-'), v.end());
        v.erase(std::remove(v.begin(), v.end(), '_'), v.end());
        if (v == "1" || v == "decoded" || v == "decodedblock" ||
            v == "blockinterpreter" || v == "blockinterp" || v == "block") {
            return CpuExecutionMode::DecodedBlockInterpreter;
        }
        if (v == "2" || v == "x64jit" || v == "recompiler" ||
            v == "jit" || v == "dynarec") {
            return CpuExecutionMode::X64Jit;
        }
        if (v == "0" || v == "interpreter" || v == "interp") {
            return CpuExecutionMode::Interpreter;
        }
        return fallback;
        };

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const size_t eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }

        std::string key = line.substr(0, eq);
        std::string value = line.substr(eq + 1);
        trim(key);
        trim(value);

        if (key == "bios_path") {
            bios_path_ = value;
        }
        else if (key == "rom_directory") {
            rom_directory_ = value;
            game_library_dirty_ = true;
        }
        else if (key == "vsync") {
            config_vsync_ = parse_bool(value, config_vsync_);
        }
        else if (key == "low_spec_mode") {
            config_low_spec_mode_ = parse_bool(value, config_low_spec_mode_);
            g_low_spec_mode = config_low_spec_mode_;
        }
        else if (key == "direct_disc_boot") {
            config_direct_disc_boot_ =
                parse_bool(value, config_direct_disc_boot_);
        }
        else if (key == "cpu_execution_mode") {
            g_cpu_execution_mode = parse_cpu_mode(value, g_cpu_execution_mode);
        }
        else if (key == "cpu_x64_jit_hot_block_threshold") {
            const unsigned long parsed =
                std::strtoul(value.c_str(), nullptr, 10);
            g_cpu_x64_jit_hot_block_threshold =
                static_cast<u32>(std::min(1000000ul, parsed));
        }
        else if (key == "cpu_x64_jit_min_block_instructions") {
            const unsigned long parsed =
                std::strtoul(value.c_str(), nullptr, 10);
            g_cpu_x64_jit_min_block_instructions =
                static_cast<u32>(std::max(1ul, std::min(64ul, parsed)));
        }
        else if (key == "cpu_x64_jit_branch_tail_enabled") {
            g_cpu_x64_jit_branch_tail_enabled =
                parse_bool(value, g_cpu_x64_jit_branch_tail_enabled);
        }
        else if (key == "cpu_x64_jit_all_native_enabled") {
            g_cpu_x64_jit_all_native_enabled =
                parse_bool(value, g_cpu_x64_jit_all_native_enabled);
        }
        else if (key == "cpu_x64_jit_native_memory_enabled") {
            g_cpu_x64_jit_native_memory_enabled =
                parse_bool(value, g_cpu_x64_jit_native_memory_enabled);
        }
        else if (key == "cpu_x64_jit_native_alu_enabled") {
            g_cpu_x64_jit_native_alu_enabled =
                parse_bool(value, g_cpu_x64_jit_native_alu_enabled);
        }
        else if (key == "memory_card_slot1_mode") {
            const int parsed = static_cast<int>(std::strtol(value.c_str(), nullptr, 10));
            config_memory_card_mode_[0] = std::max(0, std::min(2, parsed));
        }
        else if (key == "memory_card_slot2_mode") {
            const int parsed = static_cast<int>(std::strtol(value.c_str(), nullptr, 10));
            config_memory_card_mode_[1] = std::max(0, std::min(2, parsed));
        }
        else if (key == "turbo_speed_percent") {
            const int parsed = static_cast<int>(std::strtol(value.c_str(), nullptr, 10));
            config_turbo_speed_percent_ = normalize_turbo_speed_percent(parsed);
        }
        else if (key == "slowdown_speed_percent") {
            const int parsed = static_cast<int>(std::strtol(value.c_str(), nullptr, 10));
            config_slowdown_speed_percent_ = normalize_slowdown_speed_percent(parsed);
        }
        else if (key == "spu_diagnostic_mode") {
            config_spu_diagnostic_mode_ = parse_bool(value, config_spu_diagnostic_mode_);
        }
        else if (key == "discord_rich_presence") {
            config_discord_rich_presence_ =
                parse_bool(value, config_discord_rich_presence_);
        }
        else if (key == "gpu_fast_mode") {
            g_gpu_fast_mode = parse_bool(value, g_gpu_fast_mode);
        }
        else if (key == "gpu_extreme_fast_mode") {
            g_gpu_extreme_fast_mode =
                parse_bool(value, g_gpu_extreme_fast_mode);
        }
        else if (key == "mdec_debug_disable_dma1_reorder") {
            g_mdec_debug_disable_dma1_reorder =
                parse_bool(value, g_mdec_debug_disable_dma1_reorder);
        }
        else if (key == "mdec_debug_disable_chroma") {
            g_mdec_debug_disable_chroma =
                parse_bool(value, g_mdec_debug_disable_chroma);
        }
        else if (key == "mdec_debug_disable_luma") {
            g_mdec_debug_disable_luma =
                parse_bool(value, g_mdec_debug_disable_luma);
        }
        else if (key == "mdec_debug_force_solid_output") {
            g_mdec_debug_force_solid_output =
                parse_bool(value, g_mdec_debug_force_solid_output);
        }
        else if (key == "mdec_debug_swap_input_halfwords") {
            g_mdec_debug_swap_input_halfwords =
                parse_bool(value, g_mdec_debug_swap_input_halfwords);
        }
        else if (key == "mdec_debug_compare_macroblocks") {
            g_mdec_debug_compare_macroblocks =
                parse_bool(value, g_mdec_debug_compare_macroblocks);
        }
        else if (key == "mdec_debug_upload_probe") {
            g_mdec_debug_upload_probe =
                parse_bool(value, g_mdec_debug_upload_probe);
        }
        else if (key == "mdec_debug_color_block_mask") {
            const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 10);
            g_mdec_debug_color_block_mask =
                static_cast<u8>(std::max(0ul, std::min(15ul, parsed)));
        }
        else if (key.rfind("bind_", 0) == 0) {
            const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 10);
            const SDL_Scancode scancode = static_cast<SDL_Scancode>(parsed);
            for (const auto& entry : kKeyboardBindEntries) {
                if (key == entry.config_key) {
                    if (scancode == SDL_SCANCODE_UNKNOWN) {
                        input_->clear_key_binding(entry.button);
                    }
                    else {
                        input_->set_key_binding(scancode, entry.button);
                    }
                    break;
                }
            }
        }
        else if (key == "spu_output_buffer_seconds") {
            const float parsed = std::strtof(value.c_str(), nullptr);
            const float clamped = std::max(0.05f, std::min(8.0f, parsed));
            g_spu_output_buffer_seconds = clamped;
        }
        else if (key == "spu_audio_target_latency_ms") {
            const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 10);
            g_spu_audio_target_latency_ms = static_cast<u32>(
                std::clamp(parsed, 10ul, 500ul));
        }
        else if (key == "spu_audio_max_latency_ms") {
            const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 10);
            g_spu_audio_max_latency_ms = static_cast<u32>(
                std::clamp(parsed, 10ul, 1000ul));
        }
        else if (key == "spu_audio_soft_latency_ms") {
            const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 10);
            g_spu_audio_soft_latency_ms = static_cast<u32>(
                std::clamp(parsed, 10ul, 750ul));
        }
        else if (key == "spu_xa_buffer_seconds") {
            const float parsed = std::strtof(value.c_str(), nullptr);
            const float clamped = std::max(0.0f, std::min(5.0f, parsed));
            g_spu_xa_buffer_seconds = clamped;
        }
        // Backward compatibility with older config keys in milliseconds.
        else if (key == "spu_output_latency_ms") {
            const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 10);
            g_spu_audio_target_latency_ms = static_cast<u32>(
                std::clamp(parsed, 10ul, 500ul));
        }
        else if (key == "spu_xa_latency_ms") {
            const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 10);
            const u32 clamped = static_cast<u32>(std::min(2000ul, parsed));
            g_spu_xa_buffer_seconds = static_cast<float>(clamped) / 1000.0f;
        }
        else if (key == "spu_enable_audio_queue") {
            g_spu_enable_audio_queue = parse_bool(value, g_spu_enable_audio_queue);
        }
        else if (key == "spu_enable_smooth_trim") {
            g_spu_enable_smooth_trim =
                parse_bool(value, g_spu_enable_smooth_trim);
        }
        else if (key == "spu_show_audio_stats") {
            g_spu_show_audio_stats = parse_bool(value, g_spu_show_audio_stats);
        }
        else if (key == "spu_audio_stats_log") {
            g_spu_audio_stats_log = parse_bool(value, g_spu_audio_stats_log);
        }
        else if (key == "spu_enable_lag_stutter") {
            g_spu_enable_lag_stutter =
                parse_bool(value, g_spu_enable_lag_stutter);
        }
        else if (key == "spu_enable_slowdown_stutter") {
            g_spu_enable_slowdown_stutter =
                parse_bool(value, g_spu_enable_slowdown_stutter);
        }
        else if (key == "advanced_sound_status_logging") {
            g_spu_advanced_sound_status =
                parse_bool(value, g_spu_advanced_sound_status);
        }
        else if (key == "log_level") {
            g_log_level = parse_log_level_config(value, g_log_level);
        }
        else if (key == "log_timestamps") {
            g_log_timestamp = parse_bool(value, g_log_timestamp);
        }
        else if (key == "log_collapse_repeats") {
            g_log_dedupe = parse_bool(value, g_log_dedupe);
        }
        else if (key == "log_fmv_diagnostics") {
            g_log_fmv_diagnostics =
                parse_bool(value, g_log_fmv_diagnostics);
        }
        else if (key == "log_repeat_flush") {
            const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 10);
            g_log_dedupe_flush = static_cast<u32>(std::max(1ul, parsed));
        }
        else if (key == "log_category_mask") {
            const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 0);
            g_log_category_mask = static_cast<u32>(parsed);
        }
        else if (key == "log_file_path") {
            std::snprintf(log_path_, sizeof(log_path_), "%s", value.c_str());
        }
        else if (key == "trace_dma") {
            g_trace_dma = parse_bool(value, g_trace_dma);
        }
        else if (key == "trace_cdrom") {
            g_trace_cdrom = parse_bool(value, g_trace_cdrom);
        }
        else if (key == "trace_cpu") {
            g_trace_cpu = parse_bool(value, g_trace_cpu);
        }
        else if (key == "trace_bus") {
            g_trace_bus = parse_bool(value, g_trace_bus);
        }
        else if (key == "trace_ram") {
            g_trace_ram = parse_bool(value, g_trace_ram);
        }
        else if (key == "trace_gpu") {
            g_trace_gpu = parse_bool(value, g_trace_gpu);
        }
        else if (key == "trace_spu") {
            g_trace_spu = parse_bool(value, g_trace_spu);
        }
        else if (key == "trace_irq") {
            g_trace_irq = parse_bool(value, g_trace_irq);
        }
        else if (key == "trace_timer") {
            g_trace_timer = parse_bool(value, g_trace_timer);
        }
        else if (key == "trace_sio") {
            g_trace_sio = parse_bool(value, g_trace_sio);
        }
        else if (key == "cpu_deep_diagnostics") {
            g_cpu_deep_diagnostics =
                parse_bool(value, g_cpu_deep_diagnostics);
        }
        else if (key == "trace_burst_cpu") {
            g_trace_burst_cpu = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_stride_cpu") {
            g_trace_stride_cpu = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_burst_bus") {
            g_trace_burst_bus = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_stride_bus") {
            g_trace_stride_bus = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_burst_ram") {
            g_trace_burst_ram = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_stride_ram") {
            g_trace_stride_ram = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_burst_dma") {
            g_trace_burst_dma = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_stride_dma") {
            g_trace_stride_dma = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_burst_cdrom") {
            g_trace_burst_cdrom = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_stride_cdrom") {
            g_trace_stride_cdrom = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_burst_gpu") {
            g_trace_burst_gpu = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_stride_gpu") {
            g_trace_stride_gpu = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_burst_spu") {
            g_trace_burst_spu = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_stride_spu") {
            g_trace_stride_spu = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_burst_irq") {
            g_trace_burst_irq = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_stride_irq") {
            g_trace_stride_irq = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_burst_timer") {
            g_trace_burst_timer = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_stride_timer") {
            g_trace_stride_timer = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_burst_sio") {
            g_trace_burst_sio = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "trace_stride_sio") {
            g_trace_stride_sio = static_cast<u32>(std::max(1ul, std::strtoul(value.c_str(), nullptr, 10)));
        }
        else if (key == "detailed_profiling") {
            g_profile_detailed_timing = parse_bool(value, g_profile_detailed_timing);
        }
        else if (key == "experimental_bios_size_mode") {
            g_experimental_bios_size_mode = parse_bool(value, g_experimental_bios_size_mode);
        }
        else if (key == "unsafe_ps2_bios_mode") {
            g_unsafe_ps2_bios_mode = parse_bool(value, g_unsafe_ps2_bios_mode);
        }
        else if (key == "experimental_unhandled_special_returns_zero") {
            g_experimental_unhandled_special_returns_zero =
                parse_bool(value, g_experimental_unhandled_special_returns_zero);
        }
        else if (key == "experimental_dma_command_sanitizer") {
            g_experimental_dma_command_sanitizer =
                parse_bool(value, g_experimental_dma_command_sanitizer);
        }
        else if (key == "deinterlace_mode") {
            const unsigned long mode = std::strtoul(value.c_str(), nullptr, 10);
            const int idx = static_cast<int>(std::max(0ul, std::min(2ul, mode)));
            g_deinterlace_mode = static_cast<DeinterlaceMode>(idx);
        }
        else if (key == "bilinear_filtering") {
            g_bilinear_filtering = parse_bool(value, g_bilinear_filtering);
        }
        else if (key == "output_resolution_mode") {
            const unsigned long mode = std::strtoul(value.c_str(), nullptr, 10);
            const int idx = static_cast<int>(std::max(0ul, std::min(2ul, mode)));
            g_output_resolution_mode = static_cast<OutputResolutionMode>(idx);
        }
    }

    g_spu_output_buffer_seconds =
        std::max(0.05f, std::min(8.0f, g_spu_output_buffer_seconds));
    g_spu_audio_target_latency_ms =
        std::clamp(g_spu_audio_target_latency_ms, 10u, 500u);
    g_spu_audio_soft_latency_ms = std::clamp(g_spu_audio_soft_latency_ms,
        g_spu_audio_target_latency_ms, 750u);
    g_spu_audio_max_latency_ms = std::clamp(g_spu_audio_max_latency_ms,
        g_spu_audio_soft_latency_ms, 1000u);
    g_spu_xa_buffer_seconds =
        std::max(0.0f, std::min(5.0f, g_spu_xa_buffer_seconds));
    if (!g_gpu_fast_mode) {
        g_gpu_extreme_fast_mode = false;
    }
    memory_card_target_paths_ = resolve_memory_card_paths();

    if (g_unsafe_ps2_bios_mode) {
        g_experimental_bios_size_mode = true;
    }
}

void App::save_persistent_config() const {
    std::ofstream out(kAppConfigFileName, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return;
    }

    out << "bios_path=" << bios_path_ << "\n";
    out << "rom_directory=" << rom_directory_ << "\n";
    out << "vsync=" << (config_vsync_ ? 1 : 0) << "\n";
    out << "low_spec_mode=" << (config_low_spec_mode_ ? 1 : 0) << "\n";
    out << "direct_disc_boot=" << (config_direct_disc_boot_ ? 1 : 0) << "\n";
    out << "cpu_execution_mode="
        << cpu_execution_mode_to_config_value(g_cpu_execution_mode) << "\n";
    out << "cpu_x64_jit_hot_block_threshold="
        << g_cpu_x64_jit_hot_block_threshold << "\n";
    out << "cpu_x64_jit_min_block_instructions="
        << g_cpu_x64_jit_min_block_instructions << "\n";
    out << "cpu_x64_jit_branch_tail_enabled="
        << (g_cpu_x64_jit_branch_tail_enabled ? 1 : 0) << "\n";
    out << "cpu_x64_jit_all_native_enabled="
        << (g_cpu_x64_jit_all_native_enabled ? 1 : 0) << "\n";
    out << "cpu_x64_jit_native_memory_enabled="
        << (g_cpu_x64_jit_native_memory_enabled ? 1 : 0) << "\n";
    out << "cpu_x64_jit_native_alu_enabled="
        << (g_cpu_x64_jit_native_alu_enabled ? 1 : 0) << "\n";
    out << "memory_card_slot1_mode="
        << std::max(0, std::min(2, config_memory_card_mode_[0])) << "\n";
    out << "memory_card_slot2_mode="
        << std::max(0, std::min(2, config_memory_card_mode_[1])) << "\n";
    out << "turbo_speed_percent=" << config_turbo_speed_percent_ << "\n";
    out << "slowdown_speed_percent=" << config_slowdown_speed_percent_ << "\n";
    out << "spu_diagnostic_mode=" << (config_spu_diagnostic_mode_ ? 1 : 0) << "\n";
    out << "discord_rich_presence=" << (config_discord_rich_presence_ ? 1 : 0) << "\n";
    out << "gpu_fast_mode=" << (g_gpu_fast_mode ? 1 : 0) << "\n";
    out << "gpu_extreme_fast_mode="
        << ((g_gpu_fast_mode && g_gpu_extreme_fast_mode) ? 1 : 0) << "\n";
    out << "mdec_debug_disable_dma1_reorder="
        << (g_mdec_debug_disable_dma1_reorder ? 1 : 0) << "\n";
    out << "mdec_debug_disable_chroma=" << (g_mdec_debug_disable_chroma ? 1 : 0) << "\n";
    out << "mdec_debug_disable_luma=" << (g_mdec_debug_disable_luma ? 1 : 0) << "\n";
    out << "mdec_debug_force_solid_output="
        << (g_mdec_debug_force_solid_output ? 1 : 0) << "\n";
    out << "mdec_debug_swap_input_halfwords="
        << (g_mdec_debug_swap_input_halfwords ? 1 : 0) << "\n";
    out << "mdec_debug_compare_macroblocks="
        << (g_mdec_debug_compare_macroblocks ? 1 : 0) << "\n";
    out << "mdec_debug_upload_probe="
        << (g_mdec_debug_upload_probe ? 1 : 0) << "\n";
    out << "mdec_debug_color_block_mask="
        << static_cast<unsigned>(g_mdec_debug_color_block_mask & 0x0Fu) << "\n";
    for (const auto& entry : kKeyboardBindEntries) {
        out << entry.config_key << "="
            << static_cast<int>(input_->key_for_button(entry.button)) << "\n";
    }
    out << std::fixed << std::setprecision(3);
    out << "spu_xa_buffer_seconds=" << g_spu_xa_buffer_seconds << "\n";
    out.unsetf(std::ios::floatfield);
    out << "spu_audio_target_latency_ms=" << g_spu_audio_target_latency_ms << "\n";
    out << "spu_audio_soft_latency_ms=" << g_spu_audio_soft_latency_ms << "\n";
    out << "spu_audio_max_latency_ms=" << g_spu_audio_max_latency_ms << "\n";
    out << "spu_enable_audio_queue=" << (g_spu_enable_audio_queue ? 1 : 0) << "\n";
    out << "spu_enable_smooth_trim=" << (g_spu_enable_smooth_trim ? 1 : 0) << "\n";
    out << "spu_show_audio_stats=" << (g_spu_show_audio_stats ? 1 : 0) << "\n";
    out << "spu_audio_stats_log=" << (g_spu_audio_stats_log ? 1 : 0) << "\n";
    out << "spu_enable_lag_stutter=" << (g_spu_enable_lag_stutter ? 1 : 0) << "\n";
    out << "spu_enable_slowdown_stutter="
        << (g_spu_enable_slowdown_stutter ? 1 : 0) << "\n";
    out << "advanced_sound_status_logging=" << (g_spu_advanced_sound_status ? 1 : 0) << "\n";
    out << "log_level=" << log_level_to_config_value(g_log_level) << "\n";
    out << "log_timestamps=" << (g_log_timestamp ? 1 : 0) << "\n";
    out << "log_collapse_repeats=" << (g_log_dedupe ? 1 : 0) << "\n";
    out << "log_fmv_diagnostics=" << (g_log_fmv_diagnostics ? 1 : 0) << "\n";
    out << "log_repeat_flush=" << static_cast<unsigned>(g_log_dedupe_flush) << "\n";
    out << "log_category_mask=" << g_log_category_mask << "\n";
    out << "log_file_path=" << log_path_ << "\n";
    out << "trace_dma=" << (g_trace_dma ? 1 : 0) << "\n";
    out << "trace_cdrom=" << (g_trace_cdrom ? 1 : 0) << "\n";
    out << "trace_cpu=" << (g_trace_cpu ? 1 : 0) << "\n";
    out << "trace_bus=" << (g_trace_bus ? 1 : 0) << "\n";
    out << "trace_ram=" << (g_trace_ram ? 1 : 0) << "\n";
    out << "trace_gpu=" << (g_trace_gpu ? 1 : 0) << "\n";
    out << "trace_spu=" << (g_trace_spu ? 1 : 0) << "\n";
    out << "trace_irq=" << (g_trace_irq ? 1 : 0) << "\n";
    out << "trace_timer=" << (g_trace_timer ? 1 : 0) << "\n";
    out << "trace_sio=" << (g_trace_sio ? 1 : 0) << "\n";
    out << "cpu_deep_diagnostics=" << (g_cpu_deep_diagnostics ? 1 : 0) << "\n";
    out << "trace_burst_cpu=" << g_trace_burst_cpu << "\n";
    out << "trace_stride_cpu=" << g_trace_stride_cpu << "\n";
    out << "trace_burst_bus=" << g_trace_burst_bus << "\n";
    out << "trace_stride_bus=" << g_trace_stride_bus << "\n";
    out << "trace_burst_ram=" << g_trace_burst_ram << "\n";
    out << "trace_stride_ram=" << g_trace_stride_ram << "\n";
    out << "trace_burst_dma=" << g_trace_burst_dma << "\n";
    out << "trace_stride_dma=" << g_trace_stride_dma << "\n";
    out << "trace_burst_cdrom=" << g_trace_burst_cdrom << "\n";
    out << "trace_stride_cdrom=" << g_trace_stride_cdrom << "\n";
    out << "trace_burst_gpu=" << g_trace_burst_gpu << "\n";
    out << "trace_stride_gpu=" << g_trace_stride_gpu << "\n";
    out << "trace_burst_spu=" << g_trace_burst_spu << "\n";
    out << "trace_stride_spu=" << g_trace_stride_spu << "\n";
    out << "trace_burst_irq=" << g_trace_burst_irq << "\n";
    out << "trace_stride_irq=" << g_trace_stride_irq << "\n";
    out << "trace_burst_timer=" << g_trace_burst_timer << "\n";
    out << "trace_stride_timer=" << g_trace_stride_timer << "\n";
    out << "trace_burst_sio=" << g_trace_burst_sio << "\n";
    out << "trace_stride_sio=" << g_trace_stride_sio << "\n";
    out << "detailed_profiling=" << (g_profile_detailed_timing ? 1 : 0) << "\n";
    out << "experimental_bios_size_mode=" << (g_experimental_bios_size_mode ? 1 : 0) << "\n";
    out << "unsafe_ps2_bios_mode=" << (g_unsafe_ps2_bios_mode ? 1 : 0) << "\n";
    out << "experimental_unhandled_special_returns_zero=" <<
        (g_experimental_unhandled_special_returns_zero ? 1 : 0) << "\n";
    out << "experimental_dma_command_sanitizer=" <<
        (g_experimental_dma_command_sanitizer ? 1 : 0) << "\n";
    out << "deinterlace_mode=" << static_cast<int>(g_deinterlace_mode) << "\n";
    out << "bilinear_filtering=" << (g_bilinear_filtering ? 1 : 0) << "\n";
    out << "output_resolution_mode=" << static_cast<int>(g_output_resolution_mode) << "\n";
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
