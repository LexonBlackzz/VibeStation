#include "ui/ps2_app.h"
#include "ui/theme_settings.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl2.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#include <algorithm>
#include <cstdio>

namespace ps2::ui {

bool Ps2App::init() {
    SDL_SetMainReady();
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
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
        break;
    }

    if (!window_ || !gl_context_) {
        std::fprintf(stderr, "Unable to create a compatible OpenGL context.\n");
        shutdown();
        return false;
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

    reset_core();
    status_message_ = "PS2 experimental core ready";
    return true;
}

void Ps2App::run() {
    bool quit = false;

    while (!quit) {
        process_events(quit);

        if (use_imgui_opengl2_backend_) {
            ImGui_ImplOpenGL2_NewFrame();
        } else {
            ImGui_ImplOpenGL3_NewFrame();
        }
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

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

        SDL_GL_SwapWindow(window_);
    }
}

void Ps2App::shutdown() {
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
            if (event.key.keysym.sym == SDLK_F5) {
                reset_core();
            } else if (event.key.keysym.sym == SDLK_F9) {
                show_ee_debug_ = !show_ee_debug_;
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
        ImGui::MenuItem("Load BIOS...", nullptr, false, false);
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
        if (ImGui::MenuItem("Reset Core", "F5")) {
            reset_core();
        }
        ImGui::MenuItem("Run", nullptr, false, false);
        ImGui::MenuItem("Pause", nullptr, false, false);
        ImGui::MenuItem("Step EE Instruction", nullptr, false, false);
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
        ImGui::MenuItem("System", nullptr, &show_system_);
        ImGui::MenuItem("EE Debug", "F9", &show_ee_debug_);
        ImGui::MenuItem("Scheduler", nullptr, &show_scheduler_);
        ImGui::Separator();
        ImGui::MenuItem("Settings", "Ctrl+,", &show_settings_);
        ImGui::MenuItem("About", nullptr, &show_about_);
        ImGui::EndMenu();
    }

    const float status_width = ImGui::CalcTextSize(status_message_.c_str()).x + 20.0f;
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

    const ImVec4 title_color =
        ui_theme::current_startup_title_color(ui_theme::g_theme_settings);
    const ImVec4 text_color =
        ui_theme::current_startup_text_color(ui_theme::g_theme_settings);
    const ImVec4 secondary =
        ui_theme::theme_lerp(text_color, ui_theme::g_theme_settings.background, 0.18f);

    const char* title = "VibeStation";
    ImGui::PushStyleColor(ImGuiCol_Text, title_color);
    ImGui::SetWindowFontScale(2.0f);
    const ImVec2 title_size = ImGui::CalcTextSize(title);
    ImGui::SetCursorPos(ImVec2(center_x - title_size.x * 0.5f, center_y - 115.0f));
    ImGui::TextUnformatted(title);
    ImGui::SetWindowFontScale(1.0f);
    ImGui::PopStyleColor();

    const char* subtitle = "PlayStation 2 Experimental Core";
    const ImVec2 subtitle_size = ImGui::CalcTextSize(subtitle);
    ImGui::SetCursorPos(ImVec2(center_x - subtitle_size.x * 0.5f, center_y - 62.0f));
    ImGui::TextColored(text_color, "%s", subtitle);

    const char* phase = "Phase 0: core scaffold + UI shell";
    const ImVec2 phase_size = ImGui::CalcTextSize(phase);
    ImGui::SetCursorPos(ImVec2(center_x - phase_size.x * 0.5f, center_y - 34.0f));
    ImGui::TextColored(secondary, "%s", phase);

    ImGui::SetCursorPos(ImVec2(center_x - 205.0f, center_y + 16.0f));
    if (ImGui::Button("Reset Core", ImVec2(125.0f, 0.0f))) {
        reset_core();
    }
    ImGui::SameLine();
    if (ImGui::Button("EE Debug", ImVec2(125.0f, 0.0f))) {
        show_ee_debug_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("System", ImVec2(125.0f, 0.0f))) {
        show_system_ = true;
    }

    ImGui::SetCursorPos(ImVec2(center_x - 260.0f, center_y + 72.0f));
    ImGui::BeginChild("PS2CoreSummary", ImVec2(520.0f, 150.0f), true);
    ImGui::TextColored(title_color, "Experimental core status");
    ImGui::Separator();

    const auto& state = system_.ee().state();
    ImGui::Text("EE RAM");
    ImGui::SameLine(180.0f);
    ImGui::Text("%zu MiB", EeRam::kSize / (1024u * 1024u));

    ImGui::Text("EE PC");
    ImGui::SameLine(180.0f);
    ImGui::Text("0x%08X", state.pc);

    ImGui::Text("Scheduler tick");
    ImGui::SameLine(180.0f);
    ImGui::Text("%llu",
        static_cast<unsigned long long>(system_.scheduler().now()));

    ImGui::Text("R5900 execution");
    ImGui::SameLine(180.0f);
    ImGui::TextDisabled("not implemented yet");

    ImGui::Text("GS / IOP / SPU2");
    ImGui::SameLine(180.0f);
    ImGui::TextDisabled("not implemented yet");
    ImGui::EndChild();
}

void Ps2App::panel_system() {
    ImGui::SetNextWindowSize(ImVec2(430.0f, 280.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("PS2 System", &show_system_)) {
        ImGui::End();
        return;
    }

    const auto& state = system_.ee().state();

    ImGui::Text("VibeStation PS2 Lab");
    ImGui::Separator();
    ImGui::Text("EE RAM: %zu MiB", EeRam::kSize / (1024u * 1024u));
    ImGui::Text("EE PC: 0x%08X", state.pc);
    ImGui::Text("EE next PC: 0x%08X", state.next_pc);
    ImGui::Text("Scheduler tick: %llu",
        static_cast<unsigned long long>(system_.scheduler().now()));

    ImGui::Spacing();
    ImGui::TextDisabled("Subsystem readiness");
    ImGui::BulletText("EE state: initialized");
    ImGui::BulletText("EE RAM/bus: available");
    ImGui::BulletText("Scheduler: available");
    ImGui::BulletText("R5900 interpreter: pending");
    ImGui::BulletText("ELF loader: pending");
    ImGui::BulletText("GS / IOP / SPU2: pending");

    ImGui::Spacing();
    if (ImGui::Button("Reset Core")) {
        reset_core();
    }

    ImGui::End();
}

void Ps2App::panel_ee_debug() {
    ImGui::SetNextWindowSize(ImVec2(760.0f, 620.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("EE Debug", &show_ee_debug_)) {
        ImGui::End();
        return;
    }

    const auto& state = system_.ee().state();

    ImGui::Text("PC: 0x%08X", state.pc);
    ImGui::SameLine();
    ImGui::Text("Next PC: 0x%08X", state.next_pc);
    ImGui::Text("HI: 0x%016llX   LO: 0x%016llX",
        static_cast<unsigned long long>(state.hi),
        static_cast<unsigned long long>(state.lo));

    ImGui::Separator();

    const ImGuiTableFlags flags =
        ImGuiTableFlags_Borders |
        ImGuiTableFlags_RowBg |
        ImGuiTableFlags_ScrollY |
        ImGuiTableFlags_SizingStretchProp;

    if (ImGui::BeginTable("EERegisters", 3, flags, ImVec2(0.0f, 470.0f))) {
        ImGui::TableSetupColumn("Register", ImGuiTableColumnFlags_WidthFixed, 90.0f);
        ImGui::TableSetupColumn("High 64");
        ImGui::TableSetupColumn("Low 64");
        ImGui::TableHeadersRow();

        for (int i = 0; i < 32; ++i) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("r%d", i);

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("0x%016llX",
                static_cast<unsigned long long>(state.gpr[i].hi));

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("0x%016llX",
                static_cast<unsigned long long>(state.gpr[i].lo));
        }

        ImGui::EndTable();
    }

    ImGui::End();
}

void Ps2App::panel_scheduler() {
    ImGui::SetNextWindowSize(ImVec2(430.0f, 230.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("PS2 Scheduler", &show_scheduler_)) {
        ImGui::End();
        return;
    }

    ImGui::Text("Current tick: %llu",
        static_cast<unsigned long long>(system_.scheduler().now()));
    ImGui::Text("Queue empty: %s", system_.scheduler().empty() ? "yes" : "no");

    ImGui::Separator();
    ImGui::TextWrapped(
        "The scheduler is already part of the PS2 core so asynchronous hardware "
        "can be added without falling back to scanline-sized catch-up loops.");

    ImGui::Spacing();
    ImGui::TextDisabled(
        "Event inspection will expand when EE timers, DMAC, GIF, VIF and GS "
        "begin scheduling real work.");

    ImGui::End();
}

void Ps2App::panel_settings() {
    ImGui::SetNextWindowSize(ImVec2(470.0f, 270.0f), ImGuiCond_FirstUseEver);
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
        preset_count > 0 ? ui_theme::theme_preset_by_index(selected).label : "None";

    if (ImGui::BeginCombo("Theme Preset", preview)) {
        for (int i = 0; i < preset_count; ++i) {
            const bool is_selected = i == ui_theme::g_selected_theme_preset_index;
            if (ImGui::Selectable(ui_theme::theme_preset_by_index(i).label, is_selected)) {
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
    ImGui::TextDisabled(
        "PS2 UI settings are stored separately in vibestation_ps2_imgui.ini.");
    ImGui::TextDisabled(
        "The normal PS1 VibeStation UI configuration is not modified.");

    ImGui::End();
}

void Ps2App::panel_about() {
    ImGui::SetNextWindowSize(ImVec2(460.0f, 250.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("About VibeStation PS2 Lab", &show_about_)) {
        ImGui::End();
        return;
    }

    ImGui::SetWindowFontScale(1.35f);
    ImGui::TextColored(
        ui_theme::current_startup_title_color(ui_theme::g_theme_settings),
        "VibeStation");
    ImGui::SetWindowFontScale(1.0f);

    ImGui::Text("PlayStation 2 Experimental Core");
    ImGui::Separator();
    ImGui::TextWrapped(
        "This build is an isolated PS2 research core. It intentionally mirrors "
        "the VibeStation UI style without linking the PS1 System, GPU, SPU, "
        "renderer, or runtime classes.");
    ImGui::Spacing();
    ImGui::TextDisabled("Current milestone: scaffold, EE RAM/bus, scheduler, UI shell.");

    ImGui::End();
}

void Ps2App::reset_core() {
    system_.reset(0);
    status_message_ = "PS2 core reset";
}

} // namespace ps2::ui
