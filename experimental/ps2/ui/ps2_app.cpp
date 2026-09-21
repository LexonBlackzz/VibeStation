#include "ui/ps2_app.h"
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
#include <string>

#ifdef _WIN32
#include <Windows.h>
#include <commdlg.h>
#endif

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
        update_emulation();

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

        SDL_GL_SwapWindow(window_);
    }
}

void Ps2App::shutdown() {
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
                can_execute && !emulation_running_)) {
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

    const char* phase = "Phase 8: FST textured GS software output";
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


    const auto& gs_stats = system_.gs_core().stats();
    ImGui::Text("GIF qwords");
    ImGui::SameLine(190.0f);
    ImGui::Text(
        "%llu",
        static_cast<unsigned long long>(gs_stats.gif_qwords));

    ImGui::Text("GS primitives");
    ImGui::SameLine(190.0f);
    ImGui::Text(
        "%llu",
        static_cast<unsigned long long>(gs_stats.primitives));

    ImGui::Text("Next subsystem");
    ImGui::SameLine(190.0f);
    ImGui::TextDisabled("Paletted textures + TEXA + blending/depth");
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
        return;
    }

    // Keep the desktop UI responsive while giving the interpreter enough
    // runway to get through the retail BIOS boot sequence.  The old fixed
    // 20k-instruction slice made one emulated NTSC field take hundreds of
    // host frames before the display could even be sampled.
    constexpr u64 kChunkInstructions = 8192;
    constexpr u64 kMaxInstructionsPerFrame = 500000;
    constexpr auto kCpuTimeSlice = std::chrono::milliseconds(8);

    const auto deadline = std::chrono::steady_clock::now() + kCpuTimeSlice;
    u64 executed = 0;
    std::string error;

    while (executed < kMaxInstructionsPerFrame &&
           !system_.halted()) {
        const u64 budget =
            std::min<u64>(
                kChunkInstructions,
                kMaxInstructionsPerFrame - executed);
        const u64 ran = system_.run_ee(budget, error);
        executed += ran;

        if (ran == 0 || !error.empty() ||
            std::chrono::steady_clock::now() >= deadline) {
            break;
        }
    }

    // PCRTC register writes can become usable well before the simplified
    // video timer reaches its next VBlank.  Sample once per host frame so the
    // first BIOS pixels appear as soon as VRAM and DISPLAY/DISPFB are valid.
    system_.refresh_display();

    if (system_.halted()) {
        emulation_running_ = false;
        status_message_ = "Execution halted: " + system_.halt_reason();
    } else if (!error.empty()) {
        emulation_running_ = false;
        status_message_ = "Execution stopped: " + error;
    }
}

void Ps2App::reset_core() {
    emulation_running_ = false;
    system_.reset(0);
    status_message_ =
        system_.bios().loaded()
            ? "PS2 core reset; BIOS remains loaded"
            : "PS2 core reset";
}

} // namespace ps2::ui
