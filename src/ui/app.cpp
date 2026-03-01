#include "app.h"
#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#include <commdlg.h>
#include <shobjidl.h>
#endif

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

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  printf("[App::init] Creating window...\n");
  fflush(stdout);
  window_ = SDL_CreateWindow(
      "VibeStation - PS1 Emulator", SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED, 1280, 800,
      SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  if (!window_) {
    LOG_ERROR("SDL_CreateWindow failed: %s", SDL_GetError());
    printf("[App::init] SDL_CreateWindow FAILED: %s\n", SDL_GetError());
    fflush(stdout);
    return false;
  }
  printf("[App::init] Window OK\n");
  fflush(stdout);

  printf("[App::init] Creating GL context...\n");
  fflush(stdout);
  gl_context_ = SDL_GL_CreateContext(window_);
  if (!gl_context_) {
    printf("[App::init] GL Context FAILED: %s\n", SDL_GetError());
    fflush(stdout);
    return false;
  }
  SDL_GL_MakeCurrent(window_, gl_context_);
  SDL_GL_SetSwapInterval(0); // Uncapped render loop; emulator speed is time-driven.
  printf("[App::init] GL Context OK\n");
  fflush(stdout);

  // Init Dear ImGui
  printf("[App::init] Initializing ImGui...\n");
  fflush(stdout);
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io; // Reserved for future config flags

  // Style — Dark with custom colors
  ImGui::StyleColorsDark();
  ImGuiStyle &style = ImGui::GetStyle();
  style.WindowRounding = 6.0f;
  style.FrameRounding = 4.0f;
  style.GrabRounding = 4.0f;
  style.TabRounding = 4.0f;
  style.WindowBorderSize = 1.0f;
  style.FrameBorderSize = 0.0f;
  style.ScrollbarRounding = 6.0f;
  style.WindowPadding = ImVec2(10, 10);

  // Custom color palette — deep purple/blue
  auto &colors = style.Colors;
  colors[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.06f, 0.12f, 0.95f);
  colors[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.08f, 0.18f, 1.00f);
  colors[ImGuiCol_TitleBgActive] = ImVec4(0.16f, 0.10f, 0.30f, 1.00f);
  colors[ImGuiCol_MenuBarBg] = ImVec4(0.10f, 0.08f, 0.15f, 1.00f);
  colors[ImGuiCol_Tab] = ImVec4(0.14f, 0.10f, 0.25f, 1.00f);
  colors[ImGuiCol_TabHovered] = ImVec4(0.30f, 0.20f, 0.55f, 1.00f);
  colors[ImGuiCol_TabActive] = ImVec4(0.24f, 0.15f, 0.45f, 1.00f);
  colors[ImGuiCol_Header] = ImVec4(0.20f, 0.14f, 0.36f, 1.00f);
  colors[ImGuiCol_HeaderHovered] = ImVec4(0.30f, 0.20f, 0.50f, 1.00f);
  colors[ImGuiCol_HeaderActive] = ImVec4(0.35f, 0.25f, 0.60f, 1.00f);
  colors[ImGuiCol_Button] = ImVec4(0.20f, 0.14f, 0.36f, 1.00f);
  colors[ImGuiCol_ButtonHovered] = ImVec4(0.34f, 0.22f, 0.58f, 1.00f);
  colors[ImGuiCol_ButtonActive] = ImVec4(0.40f, 0.28f, 0.65f, 1.00f);
  colors[ImGuiCol_FrameBg] = ImVec4(0.12f, 0.09f, 0.20f, 1.00f);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.18f, 0.12f, 0.30f, 1.00f);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.24f, 0.16f, 0.40f, 1.00f);
  colors[ImGuiCol_CheckMark] = ImVec4(0.60f, 0.40f, 1.00f, 1.00f);
  colors[ImGuiCol_SliderGrab] = ImVec4(0.50f, 0.35f, 0.85f, 1.00f);
  colors[ImGuiCol_SliderGrabActive] = ImVec4(0.60f, 0.40f, 1.00f, 1.00f);
  colors[ImGuiCol_Separator] = ImVec4(0.20f, 0.15f, 0.35f, 1.00f);
  colors[ImGuiCol_Text] = ImVec4(0.90f, 0.88f, 0.95f, 1.00f);
  printf("[App::init] ImGui styled\n");
  fflush(stdout);

  ImGui_ImplSDL2_InitForOpenGL(window_, gl_context_);
  ImGui_ImplOpenGL3_Init("#version 330");
  printf("[App::init] ImGui backends OK\n");
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
  renderer_ = std::make_unique<Renderer>();
  input_ = std::make_unique<InputManager>();

  if (!renderer_->init(window_)) {
    LOG_ERROR("Renderer init failed");
    printf("[App::init_runtime] Renderer FAILED\n");
    fflush(stdout);
    renderer_.reset();
    input_.reset();
    system_.reset();
    return false;
  }

  runtime_ready_ = true;
  printf("[App::init_runtime] Runtime ready\n");
  fflush(stdout);
  return true;
}

void App::run() {
  if (!init_runtime()) {
    return;
  }

  bool quit = false;
  last_fps_time_ = SDL_GetTicks();
  perf_last_counter_ = SDL_GetPerformanceCounter();
  const u64 perf_freq = SDL_GetPerformanceFrequency();
  const double target_frame_sec = 1.0 / 60.0;
  emu_frame_accum_sec_ = 0.0;

  while (!quit) {
    const u64 loop_start_counter = SDL_GetPerformanceCounter();
    const double delta_sec =
        static_cast<double>(loop_start_counter - perf_last_counter_) /
        static_cast<double>(perf_freq);
    perf_last_counter_ = loop_start_counter;

    emu_frame_accum_sec_ += std::min(delta_sec, 0.25);

    process_events(quit);
    update();

    if (system_->is_running()) {
      const double frame_time = 1.0 / system_->target_fps();
      int steps = 0;
      while (emu_frame_accum_sec_ >= frame_time && steps < 4) {
        system_->run_frame();
        emu_frame_accum_sec_ -= frame_time;
        ++steps;
      }
    } else {
      emu_frame_accum_sec_ = 0.0;
    }

    update_vram_debug_texture();

    // Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    render_ui();

    // Render
    ImGui::Render();

    int w, h;
    SDL_GetWindowSize(window_, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.05f, 0.03f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Render emulator output if running
    if (system_->is_running()) {
      renderer_->render(system_->gpu());
    }

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window_);

    // FPS counter
    frame_count_++;
    u32 now = SDL_GetTicks();
    if (now - last_fps_time_ >= 1000) {
      fps_ =
          static_cast<float>(frame_count_) * 1000.0f / (now - last_fps_time_);
      frame_count_ = 0;
      last_fps_time_ = now;

      char title[128];
      snprintf(title, sizeof(title), "VibeStation - PS1 Emulator | %.1f FPS",
               fps_);
      SDL_SetWindowTitle(window_, title);
    }

    const u64 loop_end_counter = SDL_GetPerformanceCounter();
    const double loop_elapsed_sec =
        static_cast<double>(loop_end_counter - loop_start_counter) /
        static_cast<double>(perf_freq);
    if (loop_elapsed_sec < target_frame_sec) {
      const double remaining_sec = target_frame_sec - loop_elapsed_sec;
      if (remaining_sec > 0.002) {
        const u32 delay_ms =
            static_cast<u32>((remaining_sec - 0.001) * 1000.0);
        if (delay_ms > 0) {
          SDL_Delay(delay_ms);
        }
      }
      while (true) {
        const u64 now_counter = SDL_GetPerformanceCounter();
        const double elapsed =
            static_cast<double>(now_counter - loop_start_counter) /
            static_cast<double>(perf_freq);
        if (elapsed >= target_frame_sec) {
          break;
        }
      }
    }
  }
}

void App::process_events(bool &quit) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);

    if (event.type == SDL_QUIT) {
      quit = true;
    }
    if (event.type == SDL_KEYDOWN && !event.key.repeat &&
        event.key.keysym.sym == SDLK_F10) {
      show_vram_ = !show_vram_;
    }

    const ImGuiIO &io = ImGui::GetIO();
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

  // Push controller state to the system
  const u16 buttons = input_->controller().button_state();
  system_->sio().set_button_state(buttons);
  system_->sio().set_analog_state(
      input_->controller().lx(), input_->controller().ly(),
      input_->controller().rx(), input_->controller().ry());
  last_button_state_ = buttons;
  emu_input_focused_ =
      system_ && system_->is_running() &&
      ((SDL_GetWindowFlags(window_) & SDL_WINDOW_INPUT_FOCUS) != 0);

}

void App::render_ui() {
  menu_bar();

  // Main dockspace
  ImGuiViewport *viewport = ImGui::GetMainViewport();
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
  if (show_about_)
    panel_about();
  if (show_debug_cpu_)
    panel_debug_cpu();
  if (show_vram_)
    panel_vram();
}

void App::menu_bar() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Load BIOS...", "Ctrl+B")) {
        std::string path = open_file_dialog(
            "BIOS Files (*.bin)\0*.bin\0All Files\0*.*\0", "Select PS1 BIOS");
        if (!path.empty()) {
          if (system_->load_bios(path)) {
            bios_path_ = path;
            has_started_emulation_ = false;
            system_->set_running(false);
            status_message_ = "BIOS loaded: " + system_->bios().get_info();
          } else {
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
          if (system_->load_game(bin, cue)) {
            game_bin_path_ = bin;
            game_cue_path_ = cue;
            has_started_emulation_ = false;
            system_->set_running(false);
            status_message_ = "Disc loaded: " +
                              std::filesystem::path(cue).filename().string() +
                              " (Emulation > Boot Disc)";
            if (!system_->cdrom().track_map_valid()) {
              status_message_ += " [track map warning]";
            }
          } else {
            status_message_ = "Failed to load game image";
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
      if (ImGui::MenuItem("Boot Disc", "Ctrl+F5", false,
                          system_->bios_loaded() && !system_->is_running() &&
                              (system_->disc_loaded() || !game_cue_path_.empty()))) {
        if (!boot_disc_from_ui()) {
          ImGui::EndMenu();
          ImGui::EndMainMenuBar();
          return;
        }
      }
      if (ImGui::MenuItem("Start / Resume", "F5", false,
                          system_->bios_loaded() && !system_->is_running())) {
        if (has_started_emulation_) {
          system_->set_running(true);
          status_message_ = "Emulation resumed";
        } else if (system_->disc_loaded() || !game_cue_path_.empty()) {
          if (!boot_disc_from_ui()) {
            ImGui::EndMenu();
            ImGui::EndMainMenuBar();
            return;
          }
        } else {
          system_->reset();
          system_->set_running(true);
          has_started_emulation_ = true;
          status_message_ = "Emulation started (BIOS)";
        }
      }
      if (ImGui::MenuItem("Pause", "F6", false, system_->is_running())) {
        system_->set_running(false);
        status_message_ = "Emulation paused";
      }
      if (ImGui::MenuItem("Reset", "F7", false, system_->bios_loaded())) {
        system_->reset();
        has_started_emulation_ = false;
        status_message_ = "System reset";
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
      ImGui::MenuItem("Settings", "Ctrl+,", &show_settings_);
      ImGui::MenuItem("CPU Debug", "F9", &show_debug_cpu_);
      ImGui::MenuItem("Show VRAM", "F10", &show_vram_);
      ImGui::MenuItem("Logging", nullptr, &show_logging_);
      ImGui::MenuItem("About", nullptr, &show_about_);
      ImGui::EndMenu();
    }

    // Status bar on the right
    const bool disc_loaded = system_->disc_loaded();
    const char *disc_text = disc_loaded ? "Disc: Loaded" : "Disc: None";
    const float status_width = ImGui::CalcTextSize(status_message_.c_str()).x + 16.0f;
    const float disc_width = ImGui::CalcTextSize(disc_text).x + 16.0f;

    ImGui::SameLine(ImGui::GetWindowWidth() - status_width - disc_width - 84.0f);
    ImGui::TextColored(ImVec4(0.5f, 0.4f, 0.8f, 1.0f), "%s",
                       status_message_.c_str());

    ImGui::SameLine(ImGui::GetWindowWidth() - disc_width - 72.0f);
    ImGui::TextColored(disc_loaded ? ImVec4(0.4f, 0.8f, 0.4f, 1.0f)
                                   : ImVec4(0.85f, 0.45f, 0.45f, 1.0f),
                       "%s", disc_text);

    ImGui::SameLine(ImGui::GetWindowWidth() - 72.0f);
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "%.0f FPS", fps_);

    ImGui::EndMainMenuBar();
  }
}

bool App::should_route_keyboard_to_emu(const SDL_Event &event,
                                       const ImGuiIO &io) const {
  const bool keyboard_event =
      (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP);
  if (!keyboard_event) {
    return false;
  }
  if (!system_ || !system_->is_running()) {
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
  if (!system_->is_running()) {
    // Show a centered welcome message
    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetCursorPos(ImVec2(center.x - 200, center.y - 60));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.4f, 0.8f, 1.0f));
    ImGui::SetWindowFontScale(2.0f);
    ImGui::Text("VibeStation");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::PopStyleColor();

    ImGui::SetCursorPos(ImVec2(center.x - 180, center.y));
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.7f, 1.0f),
                       "Load a BIOS (File > Load BIOS) to get started.");

    ImGui::SetCursorPos(ImVec2(center.x - 180, center.y + 25));
    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.6f, 1.0f),
                       "Then load a game and use Emulation > Boot Disc.");
  } else {
    ImVec2 avail = ImGui::GetContentRegionAvail();
    // Safe presentation baseline for BIOS/logo recovery: fixed 4:3 letterbox.
    const float display_aspect = 4.0f / 3.0f;
    const float dst_aspect =
        (avail.y > 0.0f) ? (avail.x / avail.y) : display_aspect;
    ImVec2 draw_size = avail;
    if (dst_aspect > display_aspect) {
      draw_size.x = avail.y * display_aspect;
    } else {
      draw_size.y = (display_aspect > 0.0f) ? (avail.x / display_aspect) : avail.y;
    }
    const float x_pad = (avail.x - draw_size.x) * 0.5f;
    const float y_pad = (avail.y - draw_size.y) * 0.5f;
    ImVec2 cursor = ImGui::GetCursorPos();
    ImGui::SetCursorPos(ImVec2(cursor.x + x_pad, cursor.y + y_pad));
    ImGui::Image((ImTextureID)(intptr_t)renderer_->get_texture_id(), draw_size,
                 ImVec2(0, 0), ImVec2(1, 1));
  }
}

void App::panel_settings() {
  ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
  if (ImGui::Begin("Settings", &show_settings_)) {
    if (ImGui::BeginTabBar("SettingsTabs")) {
      if (ImGui::BeginTabItem("Input")) {
        ImGui::Text("Keyboard Bindings");
        ImGui::Separator();

        const char *button_names[] = {"Cross (Z)",     "Circle (X)",
                                      "Square (A)",    "Triangle (S)",
                                      "L1 (Q)",        "R1 (W)",
                                      "L2 (E)",        "R2 (R)",
                                      "Start (Enter)", "Select (Backspace)",
                                      "D-Pad Up",      "D-Pad Down",
                                      "D-Pad Left",    "D-Pad Right"};
        ImGui::TextWrapped("Default: Arrows=D-Pad, Z/X/A/S=Face, "
                           "Q/W/E/R=Shoulders, Enter=Start, Backspace=Select");
        ImGui::Spacing();
        ImGui::Text("Input Focus: %s", emu_input_focused_ ? "Active" : "Inactive");
        ImGui::Text("Buttons (active-low): 0x%04X", last_button_state_);
        ImGui::Text("SIO TX/RX: 0x%02X / 0x%02X",
                    static_cast<unsigned>(system_->boot_diag().last_sio_tx),
                    static_cast<unsigned>(system_->boot_diag().last_sio_rx));
        ImGui::Text("SIO tx42/full-poll: %s / %s",
                    system_->boot_diag().saw_tx_cmd42 ? "Yes" : "No",
                    system_->boot_diag().saw_full_pad_poll ? "Yes" : "No");
        ImGui::Spacing();

        if (ImGui::Button("Reset to Defaults")) {
          input_->set_default_bindings();
        }

        ImGui::Spacing();
        ImGui::Text("Gamepad: %s", input_->gamepad_name().c_str());
        if (input_->has_gamepad()) {
          ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f),
                             "Connected — auto-mapped");
        } else {
          ImGui::TextColored(
              ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
              "No gamepad detected. Connect one and it will auto-bind.");
        }

        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Video")) {
        ImGui::Text("Resolution: %dx%d", system_->gpu().display_mode().width(),
                    system_->gpu().display_mode().height());
        ImGui::Text("Internal Upscaling: 1x (native)");
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.3f, 1.0f),
                           "PGXP: Not yet implemented");
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Audio")) {
        ImGui::Text("SPU emulation: Gaussian + reverb core (stage 2)");
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.3f, 1.0f),
                           "XA/CDDA baseline is live; advanced modulation is still in progress.");
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("System")) {
        ImGui::Text("BIOS: %s", bios_path_.empty()
                                    ? "Not loaded"
                                    : system_->bios().get_info().c_str());
        ImGui::Text("CPU Clock: 33.8688 MHz");
        ImGui::Checkbox("Experimental BIOS size mode",
                        &g_experimental_bios_size_mode);
        ImGui::Checkbox("Unsafe PS2 BIOS mode", &g_unsafe_ps2_bios_mode);
        if (g_unsafe_ps2_bios_mode) {
          g_experimental_bios_size_mode = true;
        }
        ImGui::TextColored(
            ImVec4(0.7f, 0.7f, 0.3f, 1.0f),
            "Experimental mode accepts KB-aligned BIOS files.");
        ImGui::TextColored(
            ImVec4(0.8f, 0.5f, 0.3f, 1.0f),
            "Unsafe PS2 mode maps full BIOS size and is expected to be unstable.");
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Logging")) {
        const char *levels[] = {"Debug", "Info", "Warn", "Error"};
        int level_index = 1;
        switch (g_log_level) {
        case LogLevel::Debug:
          level_index = 0;
          break;
        case LogLevel::Info:
          level_index = 1;
          break;
        case LogLevel::Warn:
          level_index = 2;
          break;
        case LogLevel::Error:
          level_index = 3;
          break;
        }
        if (ImGui::Combo("Level", &level_index, levels, IM_ARRAYSIZE(levels))) {
          g_log_level = static_cast<LogLevel>(level_index);
        }

        ImGui::Checkbox("Timestamps", &g_log_timestamp);
        ImGui::Checkbox("Collapse Repeats", &g_log_dedupe);
        int dedupe_flush = static_cast<int>(g_log_dedupe_flush);
        if (ImGui::InputInt("Repeat Flush", &dedupe_flush)) {
          g_log_dedupe_flush = static_cast<u32>(std::max(1, dedupe_flush));
        }
        ImGui::Separator();
        ImGui::Text("Categories");

        auto category_checkbox = [](const char *label, LogCategory cat) {
          bool enabled = log_category_enabled(cat);
          if (ImGui::Checkbox(label, &enabled)) {
            if (enabled) {
              g_log_category_mask |= log_category_bit(cat);
            } else {
              g_log_category_mask &= ~log_category_bit(cat);
            }
          }
        };
        category_checkbox("General", LogCategory::General);
        ImGui::SameLine();
        category_checkbox("App", LogCategory::App);
        ImGui::SameLine();
        category_checkbox("BIOS", LogCategory::Bios);
        category_checkbox("CPU", LogCategory::Cpu);
        ImGui::SameLine();
        category_checkbox("BUS", LogCategory::Bus);
        ImGui::SameLine();
        category_checkbox("RAM", LogCategory::Ram);
        ImGui::SameLine();
        category_checkbox("IRQ", LogCategory::Irq);
        category_checkbox("DMA", LogCategory::Dma);
        ImGui::SameLine();
        category_checkbox("CDROM", LogCategory::Cdrom);
        ImGui::SameLine();
        category_checkbox("GPU", LogCategory::Gpu);
        category_checkbox("SPU", LogCategory::Spu);
        ImGui::SameLine();
        category_checkbox("SIO", LogCategory::Sio);
        ImGui::SameLine();
        category_checkbox("Timer", LogCategory::Timer);
        category_checkbox("Input", LogCategory::Input);

        if (ImGui::Button("Enable All Categories")) {
          g_log_category_mask = 0xFFFFFFFFu;
        }
        ImGui::SameLine();
        if (ImGui::Button("Disable All Categories")) {
          g_log_category_mask = 0;
        }

        ImGui::Separator();
        ImGui::Text("Trace Channels");
        ImGui::Checkbox("DMA Trace", &g_trace_dma);
        ImGui::SameLine();
        ImGui::Checkbox("CDROM Trace", &g_trace_cdrom);
        ImGui::Checkbox("CPU Trace", &g_trace_cpu);
        ImGui::SameLine();
        ImGui::Checkbox("BUS Trace", &g_trace_bus);
        ImGui::SameLine();
        ImGui::Checkbox("RAM Trace", &g_trace_ram);
        ImGui::Checkbox("GPU Trace", &g_trace_gpu);
        ImGui::SameLine();
        ImGui::Checkbox("SPU Trace", &g_trace_spu);
        ImGui::Checkbox("IRQ Trace", &g_trace_irq);
        ImGui::SameLine();
        ImGui::Checkbox("Timer Trace", &g_trace_timer);
        ImGui::Checkbox("SIO Trace", &g_trace_sio);

        if (ImGui::Button("Enable All Traces")) {
          g_trace_dma = true;
          g_trace_cdrom = true;
          g_trace_cpu = true;
          g_trace_bus = true;
          g_trace_ram = true;
          g_trace_gpu = true;
          g_trace_spu = true;
          g_trace_irq = true;
          g_trace_timer = true;
          g_trace_sio = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Disable All Traces")) {
          g_trace_dma = false;
          g_trace_cdrom = false;
          g_trace_cpu = false;
          g_trace_bus = false;
          g_trace_ram = false;
          g_trace_gpu = false;
          g_trace_spu = false;
          g_trace_irq = false;
          g_trace_timer = false;
          g_trace_sio = false;
        }

        ImGui::Separator();
        ImGui::Text("Trace Sampling (burst/stride)");
        auto sample_pair = [](const char *label, u32 &burst, u32 &stride) {
          int b = static_cast<int>(burst);
          int s = static_cast<int>(stride);
          std::string burst_label = std::string(label) + " Burst";
          std::string stride_label = std::string(label) + " Stride";
          if (ImGui::InputInt(burst_label.c_str(), &b)) {
            burst = static_cast<u32>(std::max(1, b));
          }
          if (ImGui::InputInt(stride_label.c_str(), &s)) {
            stride = static_cast<u32>(std::max(1, s));
          }
        };
        sample_pair("CPU", g_trace_burst_cpu, g_trace_stride_cpu);
        sample_pair("BUS", g_trace_burst_bus, g_trace_stride_bus);
        sample_pair("RAM", g_trace_burst_ram, g_trace_stride_ram);
        sample_pair("SPU", g_trace_burst_spu, g_trace_stride_spu);
        sample_pair("GPU", g_trace_burst_gpu, g_trace_stride_gpu);
        sample_pair("DMA", g_trace_burst_dma, g_trace_stride_dma);
        sample_pair("CDROM", g_trace_burst_cdrom, g_trace_stride_cdrom);
        sample_pair("IRQ", g_trace_burst_irq, g_trace_stride_irq);
        sample_pair("Timer", g_trace_burst_timer, g_trace_stride_timer);
        sample_pair("SIO", g_trace_burst_sio, g_trace_stride_sio);
        if (ImGui::Button("Reset Trace Sampling Defaults")) {
          g_trace_burst_cpu = 128;
          g_trace_stride_cpu = 32768;
          g_trace_burst_bus = 256;
          g_trace_stride_bus = 16384;
          g_trace_burst_ram = 32;
          g_trace_stride_ram = 131072;
          g_trace_burst_dma = 64;
          g_trace_stride_dma = 2048;
          g_trace_burst_cdrom = 128;
          g_trace_stride_cdrom = 256;
          g_trace_burst_gpu = 512;
          g_trace_stride_gpu = 2048;
          g_trace_burst_spu = 128;
          g_trace_stride_spu = 4096;
          g_trace_burst_irq = 128;
          g_trace_stride_irq = 2048;
          g_trace_burst_timer = 64;
          g_trace_stride_timer = 2048;
          g_trace_burst_sio = 64;
          g_trace_stride_sio = 2048;
        }

        ImGui::Separator();
        ImGui::InputText("Log File", log_path_, IM_ARRAYSIZE(log_path_));
        if (g_log_file == nullptr) {
          if (ImGui::Button("Start File Logging")) {
            g_log_file = std::fopen(log_path_, "w");
            status_message_ =
                (g_log_file != nullptr) ? "File logging enabled" : "Failed to open log file";
          }
        } else {
          if (ImGui::Button("Stop File Logging")) {
            log_flush_repeats();
            std::fclose(g_log_file);
            g_log_file = nullptr;
            status_message_ = "File logging disabled";
          }
        }

        if (ImGui::Button("Boot Debug Preset")) {
          g_log_level = LogLevel::Debug;
          g_log_category_mask = 0xFFFFFFFFu;
          g_trace_dma = true;
          g_trace_cdrom = true;
          g_trace_cpu = true;
          g_trace_bus = true;
          g_trace_ram = true;
          g_trace_gpu = true;
          g_trace_spu = true;
          g_trace_irq = true;
          g_trace_timer = true;
          g_trace_sio = true;
          g_trace_burst_cpu = 128;
          g_trace_stride_cpu = 32768;
          g_trace_burst_bus = 256;
          g_trace_stride_bus = 16384;
          g_trace_burst_ram = 32;
          g_trace_stride_ram = 131072;
          g_trace_burst_dma = 64;
          g_trace_stride_dma = 2048;
          g_trace_burst_cdrom = 128;
          g_trace_stride_cdrom = 256;
          g_trace_burst_gpu = 512;
          g_trace_stride_gpu = 2048;
          g_trace_burst_spu = 128;
          g_trace_stride_spu = 4096;
          g_trace_burst_irq = 128;
          g_trace_stride_irq = 2048;
          g_trace_burst_timer = 64;
          g_trace_stride_timer = 2048;
          g_trace_burst_sio = 64;
          g_trace_stride_sio = 2048;
          status_message_ = "Logging preset applied";
        }

        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
  }
  ImGui::End();
}

void App::panel_about() {
  ImGui::SetNextWindowSize(ImVec2(400, 200), ImGuiCond_FirstUseEver);
  if (ImGui::Begin("About VibeStation", &show_about_,
                   ImGuiWindowFlags_NoResize)) {
    ImGui::TextColored(ImVec4(0.6f, 0.4f, 1.0f, 1.0f), "VibeStation v0.1.0");
    ImGui::Separator();
    ImGui::Text("A PlayStation 1 emulator");
    ImGui::Spacing();
    ImGui::Text("CPU: MIPS R3000A interpreter");
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
    ImGui::Text("PC: 0x%08X", system_->cpu().pc());
    ImGui::Text("Cycles: %llu",
                (unsigned long long)system_->cpu().cycle_count());
    ImGui::Separator();

    if (ImGui::BeginTable("Registers", 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      const char *reg_names[] = {
          "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0", "t1", "t2",
          "t3",   "t4", "t5", "t6", "t7", "s0", "s1", "s2", "s3", "s4", "s5",
          "s6",   "s7", "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"};
      for (int i = 0; i < 32; i++) {
        ImGui::TableNextColumn();
        ImGui::TextColored(ImVec4(0.6f, 0.5f, 0.9f, 1.0f), "$%s", reg_names[i]);
        ImGui::SameLine(55);
        ImGui::Text("0x%08X", system_->cpu().reg(i));
      }
      ImGui::EndTable();
    }

    ImGui::Spacing();
    if (ImGui::Button("Step") && !system_->is_running()) {
      system_->step();
    }
    ImGui::SameLine();
    if (ImGui::Button(system_->is_running() ? "Pause" : "Run")) {
      system_->set_running(!system_->is_running());
    }
  }
  ImGui::End();
}

void App::update_vram_debug_texture() {
  if (!system_) {
    return;
  }

  if (vram_debug_texture_ == 0) {
    glGenTextures(1, &vram_debug_texture_);
    glBindTexture(GL_TEXTURE_2D, vram_debug_texture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }

  const u16 *vram = system_->gpu().vram();
  std::vector<u32> rgba(psx::VRAM_WIDTH * psx::VRAM_HEIGHT);
  for (size_t i = 0; i < rgba.size(); ++i) {
    const u16 p = vram[i];
    const u8 r = static_cast<u8>((p & 0x1F) << 3);
    const u8 g = static_cast<u8>(((p >> 5) & 0x1F) << 3);
    const u8 b = static_cast<u8>(((p >> 10) & 0x1F) << 3);
    rgba[i] = r | (static_cast<u32>(g) << 8) | (static_cast<u32>(b) << 16) |
              0xFF000000;
  }

  glBindTexture(GL_TEXTURE_2D, vram_debug_texture_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, psx::VRAM_WIDTH, psx::VRAM_HEIGHT, 0,
               GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
}

void App::panel_vram() {
  ImGui::SetNextWindowSize(ImVec2(980, 620), ImGuiCond_FirstUseEver);
  if (ImGui::Begin("VRAM Debug", &show_vram_)) {
    ImGui::Text("Raw VRAM 1024x512 (15-bit)");
    ImGui::Separator();

    ImVec2 avail = ImGui::GetContentRegionAvail();
    const float tex_w = static_cast<float>(psx::VRAM_WIDTH);
    const float tex_h = static_cast<float>(psx::VRAM_HEIGHT);
    const float scale = std::min(avail.x / tex_w, avail.y / tex_h);
    ImVec2 draw_size(tex_w * scale, tex_h * scale);

    if (vram_debug_texture_ != 0) {
      ImGui::Image((ImTextureID)(intptr_t)vram_debug_texture_, draw_size,
                   ImVec2(0, 0), ImVec2(1, 1));
    }
  }
  ImGui::End();
}

// ── File Dialog ────────────────────────────────────────────────────

bool App::resolve_disc_paths(const std::string &selected_path,
                             std::string &bin_path, std::string &cue_path,
                             std::string &error) const {
  bin_path.clear();
  cue_path.clear();
  error.clear();

  const std::filesystem::path selected(selected_path);
  std::string ext = selected.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (ext == ".cue") {
    cue_path = selected.string();
    std::filesystem::path sibling_bin = selected;
    sibling_bin.replace_extension(".bin");
    if (std::filesystem::exists(sibling_bin)) {
      bin_path = sibling_bin.string();
    } else {
      std::filesystem::path sibling_bin_upper = selected;
      sibling_bin_upper.replace_extension(".BIN");
      if (std::filesystem::exists(sibling_bin_upper)) {
        bin_path = sibling_bin_upper.string();
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
    } else {
      std::filesystem::path sibling_cue_upper = selected;
      sibling_cue_upper.replace_extension(".CUE");
      if (std::filesystem::exists(sibling_cue_upper)) {
        cue_path = sibling_cue_upper.string();
      }
    }

    if (cue_path.empty()) {
      error =
          "Selected BIN requires matching CUE file in the same directory.";
      return false;
    }
    return true;
  }

  error = "Unsupported disc image. Select a .cue or .bin file.";
  return false;
}

bool App::boot_disc_from_ui() {
  if (!system_->bios_loaded()) {
    status_message_ = "Load a BIOS before booting a disc.";
    return false;
  }

  if (!system_->disc_loaded() && !game_cue_path_.empty()) {
    if (!system_->load_game(game_bin_path_, game_cue_path_)) {
      status_message_ = "Failed to reattach loaded disc image.";
      return false;
    }
  }

  if (!system_->disc_loaded()) {
    status_message_ = "No disc loaded. Use File > Load Game.";
    return false;
  }

  if (!system_->boot_disc()) {
    status_message_ = "Boot Disc failed. Check BIOS/disc image.";
    return false;
  }

  has_started_emulation_ = true;
  status_message_ = "Booting disc from BIOS...";
  return true;
}

std::string App::open_file_dialog(const char *filter, const char *title) {
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

void App::shutdown() {
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

  ImGui_ImplOpenGL3_Shutdown();
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




