#include "app.h"
#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#include <commdlg.h>
#include <shobjidl.h>
#endif

namespace {
constexpr const char *kAppConfigFileName = "vibestation_config.ini";
struct GrimReaperRange {
  const char *label;
  const char *slug;
  u32 start;
  u32 end;
};

constexpr GrimReaperRange kGrimReaperRanges[] = {
    {"Intro/Bootmenu (0x18000-0x63FFF)", "intro", 0x18000u, 0x63FFFu},
    {"Character Sets (0x64000-0x7FF31)", "charset", 0x64000u, 0x7FF31u},
    {"End (0x7FF32-0x7FFFF)", "end", 0x7FF32u, 0x7FFFFu},
    {"Custom (Hex Range)", "custom", 0x00000u, 0x00000u},
};

constexpr int kGrimReaperRangeCount =
    static_cast<int>(sizeof(kGrimReaperRanges) / sizeof(kGrimReaperRanges[0]));

struct KeyboardBindEntry {
  const char *label;
  const char *config_key;
  PsxButton button;
};

constexpr KeyboardBindEntry kKeyboardBindEntries[] = {
    {"Cross", "bind_cross", PsxButton::Cross},
    {"Circle", "bind_circle", PsxButton::Circle},
    {"Square", "bind_square", PsxButton::Square},
    {"Triangle", "bind_triangle", PsxButton::Triangle},
    {"L1", "bind_l1", PsxButton::L1},
    {"R1", "bind_r1", PsxButton::R1},
    {"L2", "bind_l2", PsxButton::L2},
    {"R2", "bind_r2", PsxButton::R2},
    {"Start", "bind_start", PsxButton::Start},
    {"Select", "bind_select", PsxButton::Select},
    {"D-Pad Up", "bind_up", PsxButton::Up},
    {"D-Pad Down", "bind_down", PsxButton::Down},
    {"D-Pad Left", "bind_left", PsxButton::Left},
    {"D-Pad Right", "bind_right", PsxButton::Right},
};

constexpr int kKeyboardBindEntryCount =
    static_cast<int>(sizeof(kKeyboardBindEntries) / sizeof(kKeyboardBindEntries[0]));
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
  SDL_GL_SetSwapInterval(config_vsync_ ? 1 : 0);
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
  const u64 perf_freq = SDL_GetPerformanceFrequency();
  const double target_frame_sec = 1.0 / 60.0;

  while (!quit) {
    const u64 loop_start_counter = SDL_GetPerformanceCounter();
    process_events(quit);
    update();
    runtime_snapshot_ = emu_runner_.runtime_snapshot();

    FrameSnapshot frame;
    if (emu_runner_.consume_latest_frame(frame)) {
      renderer_->upload_frame(frame.rgba, frame.width, frame.height);
      runtime_snapshot_ = emu_runner_.runtime_snapshot();
    }

    u32 now_ms = SDL_GetTicks();
    if (show_vram_ ||
        (!emu_runner_.is_running() &&
            (now_ms - last_vram_update_ms_) >= 1000)) {
        update_vram_debug_texture();
        last_vram_update_ms_ = now_ms;
    }

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

    const auto present_start = std::chrono::high_resolution_clock::now();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window_);
    const auto present_end = std::chrono::high_resolution_clock::now();
    present_ms_ =
        std::chrono::duration<double, std::milli>(present_end - present_start)
            .count();

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

void App::process_events(bool &quit) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);

    if (event.type == SDL_QUIT) {
      quit = true;
    }
    if (pending_bind_index_ >= 0 && event.type == SDL_KEYDOWN &&
        !event.key.repeat) {
      const int bind_index = pending_bind_index_;
      const SDL_Scancode scancode = event.key.keysym.scancode;
      pending_bind_index_ = -1;
      if (scancode == SDL_SCANCODE_ESCAPE) {
        status_message_ = "Keyboard rebinding canceled";
      } else {
        input_->set_key_binding(scancode, kKeyboardBindEntries[bind_index].button);
        save_persistent_config();
        status_message_ = std::string("Bound ") +
                          kKeyboardBindEntries[bind_index].label + " to " +
                          SDL_GetScancodeName(scancode);
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
          if (system_->load_bios(path)) {
            bios_path_ = path;
            save_persistent_config();
            has_started_emulation_ = false;
            set_grim_reaper_mode(false);
          status_message_ = "BIOS loaded: " + system_->bios().get_info();
          } else {
            status_message_ = "Failed to load BIOS!";
          }
        }
      } else if (ctrl && key == SDLK_o) {
        std::string path = open_file_dialog(
            "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
            "Select PS1 Game");
        if (!path.empty()) {
          std::string bin;
          std::string cue;
          std::string error;
          if (!resolve_disc_paths(path, bin, cue, error)) {
            status_message_ = error;
          } else {
            emu_runner_.pause_and_wait_idle();
            if (system_->load_game(bin, cue)) {
              game_bin_path_ = bin;
              game_cue_path_ = cue;
              has_started_emulation_ = false;
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
      } else if (ctrl && key == SDLK_COMMA) {
        show_settings_ = !show_settings_;
      } else if (ctrl && key == SDLK_F5) {
        if (system_->bios_loaded() && !emu_runner_.is_running() &&
            (system_->disc_loaded() || !game_cue_path_.empty())) {
          boot_disc_from_ui();
        }
      } else if (no_mod && key == SDLK_F5) {
        if (system_->bios_loaded() && !emu_runner_.is_running()) {
          if (has_started_emulation_) {
            emu_runner_.set_running(true);
            status_message_ = "Emulation resumed";
          } else if (system_->disc_loaded() || !game_cue_path_.empty()) {
            boot_disc_from_ui();
          } else {
            emu_runner_.pause_and_wait_idle();
            system_->reset();
            has_started_emulation_ = true;
            emu_runner_.set_running(true);
            status_message_ = "Emulation started (BIOS)";
          }
        }
      } else if (no_mod && key == SDLK_F6) {
        if (emu_runner_.is_running()) {
          emu_runner_.pause_and_wait_idle();
          status_message_ = "Emulation paused";
        }
      } else if (no_mod && key == SDLK_F7) {
        if (system_->bios_loaded() && has_started_emulation_) {
          emu_runner_.pause_and_wait_idle();
          has_started_emulation_ = false;
          status_message_ = "Emulation stopped";
        }
      } else if (no_mod && key == SDLK_F9) {
        show_debug_cpu_ = !show_debug_cpu_;
      } else if (no_mod && key == SDLK_F10) {
        show_vram_ = !show_vram_;
      } else if (no_mod && key == SDLK_F11) {
        show_perf_ = !show_perf_;
      }
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
    const u64 underruns = runtime_snapshot_.spu_audio.underrun_events;
    const u32 now_ms = SDL_GetTicks();
    if (underrun_notice_last_tick_ms_ == 0) {
      underrun_notice_last_tick_ms_ = now_ms;
      underrun_notice_last_events_ = underruns;
    } else if ((now_ms - underrun_notice_last_tick_ms_) >=
               kUnderrunNoticeSamplePeriodMs) {
      const u32 bucket_value =
          static_cast<u32>(std::min<u64>(underruns - underrun_notice_last_events_,
                                         std::numeric_limits<u32>::max()));
      if (underrun_notice_bucket_count_ < underrun_notice_buckets_.size()) {
        ++underrun_notice_bucket_count_;
      } else {
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
  } else {
    show_fast_mode_notice_ = false;
    underrun_notice_buckets_.fill(0);
    underrun_notice_bucket_index_ = 0;
    underrun_notice_bucket_count_ = 0;
    underrun_notice_bucket_sum_ = 0;
    underrun_notice_last_tick_ms_ = 0;
    underrun_notice_last_events_ = 0;
  }
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
  if (show_grim_reaper_)
    panel_grim_reaper();
  if (show_about_)
    panel_about();
  if (show_debug_cpu_)
    panel_debug_cpu();
  if (show_vram_)
    panel_vram();
  if (show_perf_)
    panel_performance();
  if (show_sound_status_)
    panel_sound_status();
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
          if (system_->load_bios(path)) {
            bios_path_ = path;
            save_persistent_config();
            has_started_emulation_ = false;
            set_grim_reaper_mode(false);
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
          emu_runner_.pause_and_wait_idle();
          if (system_->load_game(bin, cue)) {
            game_bin_path_ = bin;
            game_cue_path_ = cue;
            has_started_emulation_ = false;
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
                          bios_loaded && !emu_running &&
                              (disc_loaded || !game_cue_path_.empty()))) {
        if (!boot_disc_from_ui()) {
          ImGui::EndMenu();
          ImGui::EndMainMenuBar();
          return;
        }
      }
      if (ImGui::MenuItem("Start / Resume", "F5", false,
                          bios_loaded && !emu_running)) {
        if (has_started_emulation_) {
          emu_runner_.set_running(true);
          status_message_ = "Emulation resumed";
        } else if (disc_loaded || !game_cue_path_.empty()) {
          if (!boot_disc_from_ui()) {
            ImGui::EndMenu();
            ImGui::EndMainMenuBar();
            return;
          }
        } else {
          emu_runner_.pause_and_wait_idle();
          system_->reset();
          has_started_emulation_ = true;
          emu_runner_.set_running(true);
          status_message_ = "Emulation started (BIOS)";
        }
      }
      if (ImGui::MenuItem("Pause", "F6", false, emu_running)) {
        emu_runner_.pause_and_wait_idle();
        status_message_ = "Emulation paused";
      }
      if (ImGui::MenuItem("Stop", "F7", false,
                          bios_loaded && has_started_emulation_)) {
        emu_runner_.pause_and_wait_idle();
        has_started_emulation_ = false;
        status_message_ = "Emulation stopped";
      }
      if (ImGui::MenuItem("Restart BIOS", nullptr, false, bios_loaded)) {
        emu_runner_.pause_and_wait_idle();
        set_grim_reaper_mode(false);
        if (!bios_path_.empty() && !system_->load_bios(bios_path_)) {
          status_message_ = "Failed to reload original BIOS";
        } else {
          system_->reset();
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
      ImGui::MenuItem("Performance", "F11", &show_perf_);
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
    ImGui::SetCursorPos(ImVec2(center.x - 200, center.y - 80));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.4f, 0.8f, 1.0f));
    ImGui::SetWindowFontScale(2.0f);
    ImGui::Text("VibeStation");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::PopStyleColor();

    ImGui::SetCursorPos(ImVec2(center.x - 180, center.y - 20));
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.7f, 1.0f),
                       "Load a BIOS (File > Load BIOS) to get started.");

    ImGui::SetCursorPos(ImVec2(center.x - 180, center.y + 5));
    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.6f, 1.0f),
                       "Then load a game and use Emulation > Boot Disc.");

    const char *bios_button_label = bios_loaded ? "Change BIOS" : "Load BIOS";
    const ImVec2 button_size(120.0f, 0.0f);

    ImGui::SetCursorPos(ImVec2(center.x - 200, center.y + 42));
    if (ImGui::Button(bios_button_label, button_size)) {
      std::string path = open_file_dialog(
          "BIOS Files (*.bin)\0*.bin\0All Files\0*.*\0", "Select PS1 BIOS");
      if (!path.empty()) {
        emu_runner_.pause_and_wait_idle();
        if (system_->load_bios(path)) {
          bios_path_ = path;
          save_persistent_config();
          has_started_emulation_ = false;
          set_grim_reaper_mode(false);
          status_message_ = "BIOS loaded: " + system_->bios().get_info();
        } else {
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
        } else {
          emu_runner_.pause_and_wait_idle();
          if (system_->load_game(bin, cue)) {
            game_bin_path_ = bin;
            game_cue_path_ = cue;
            has_started_emulation_ = false;
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
    }
    if (!bios_loaded) {
      ImGui::EndDisabled();
    }

    ImGui::SameLine();
    if (!bios_loaded) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button("Start Emulation", button_size)) {
      if (disc_loaded || !game_cue_path_.empty()) {
        boot_disc_from_ui();
      } else {
        emu_runner_.pause_and_wait_idle();
        system_->reset();
        has_started_emulation_ = true;
        emu_runner_.set_running(true);
        status_message_ = "Emulation started (BIOS)";
      }
    }
    if (!bios_loaded) {
      ImGui::EndDisabled();
    }
  } else {
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
        if (pending_bind_index_ >= 0) {
          ImGui::TextColored(ImVec4(0.95f, 0.8f, 0.3f, 1.0f),
                             "Press a key for %s (Esc to cancel)",
                             kKeyboardBindEntries[pending_bind_index_].label);
        }
        ImGui::Text("Input Focus: %s", emu_input_focused_ ? "Active" : "Inactive");
        ImGui::Text("Buttons (active-low): 0x%04X", last_button_state_);
        const auto &diag = runtime_snapshot_.boot_diag;
        ImGui::Text("SIO TX/RX: 0x%02X / 0x%02X",
                    static_cast<unsigned>(diag.last_sio_tx),
                    static_cast<unsigned>(diag.last_sio_rx));
        ImGui::Text("SIO tx42/full-poll: %s / %s",
                    diag.saw_tx_cmd42 ? "Yes" : "No",
                    diag.saw_full_pad_poll ? "Yes" : "No");
        ImGui::Spacing();

        if (ImGui::Button("Reset to Defaults")) {
          input_->set_default_bindings();
          pending_bind_index_ = -1;
          save_persistent_config();
        }

        ImGui::Spacing();
        for (int i = 0; i < kKeyboardBindEntryCount; ++i) {
          const SDL_Scancode scancode =
              input_->key_for_button(kKeyboardBindEntries[i].button);
          const char *key_name =
              (scancode != SDL_SCANCODE_UNKNOWN) ? SDL_GetScancodeName(scancode)
                                                 : "Unbound";
          ImGui::Text("%s", kKeyboardBindEntries[i].label);
          ImGui::SameLine(140.0f);
          std::string assign_label =
              std::string((key_name && key_name[0] != '\0') ? key_name : "Unbound") +
              "##bind_" + kKeyboardBindEntries[i].config_key;
          if (ImGui::Button(assign_label.c_str(), ImVec2(120.0f, 0.0f))) {
            pending_bind_index_ = i;
          }
          ImGui::SameLine();
          std::string clear_label =
              std::string("Clear##") + kKeyboardBindEntries[i].config_key;
          if (ImGui::Button(clear_label.c_str())) {
            input_->clear_key_binding(kKeyboardBindEntries[i].button);
            if (pending_bind_index_ == i) {
              pending_bind_index_ = -1;
            }
            save_persistent_config();
          }
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
        const int frame_w = std::max(1, renderer_->last_frame_width());
        const int frame_h = std::max(1, renderer_->last_frame_height());
        ImGui::Text("Resolution: %dx%d", frame_w, frame_h);
        ImGui::Text("Display Area: %ux%u",
                    static_cast<unsigned>(runtime_snapshot_.boot_diag.display_width),
                    static_cast<unsigned>(runtime_snapshot_.boot_diag.display_height));
        ImGui::Checkbox("Fast Mode", &g_gpu_fast_mode);
        ImGui::TextColored(
            ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
            "Uses optimized GPU paths for lower CPU usage at the cost of possible artifacting.");
        const char *deinterlace_modes[] = {"Weave (Stable)", "Bob (Field)",
                                           "Blend (Soft)"};
        int deinterlace_index = static_cast<int>(g_deinterlace_mode);
        if (ImGui::Combo("Deinterlace", &deinterlace_index, deinterlace_modes,
                         IM_ARRAYSIZE(deinterlace_modes))) {
          deinterlace_index = std::max(0, std::min(2, deinterlace_index));
          g_deinterlace_mode =
              static_cast<DeinterlaceMode>(deinterlace_index);
        }

        const char *resolution_modes[] = {"320x240", "640x480", "1024x768"};
        int resolution_index = static_cast<int>(g_output_resolution_mode);
        if (ImGui::Combo("Output Resolution", &resolution_index,
                         resolution_modes, IM_ARRAYSIZE(resolution_modes))) {
          resolution_index = std::max(0, std::min(2, resolution_index));
          g_output_resolution_mode =
              static_cast<OutputResolutionMode>(resolution_index);
        }
        ImGui::Text("Internal Upscaling: 1x (native)");
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Audio")) {
        ImGui::Text("SPU emulation: Gaussian + reverb core (stage 2)");
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.3f, 1.0f),
                           "XA/CDDA baseline is live; advanced modulation is still in progress.");

        int desired_samples = static_cast<int>(g_spu_desired_samples);
        if (ImGui::InputInt("Desired Samples", &desired_samples, 256, 1024)) {
          desired_samples = std::max(64, std::min(65535, desired_samples));
          g_spu_desired_samples = static_cast<u32>(desired_samples);
        }
        ImGui::TextDisabled("Applied on next audio device init.");
        ImGui::Checkbox("Advanced Sound Status Logging", &g_spu_advanced_sound_status);
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                           "Enables per-sample SPU diagnostics and per-frame voice snapshots.");

        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Sound Status")) {
        draw_sound_status_content();
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("System")) {
        ImGui::Text("BIOS: %s", bios_path_.empty()
                                    ? "Not loaded"
                                    : system_->bios().get_info().c_str());
        ImGui::Text("CPU Clock: 33.8688 MHz");
        ImGui::Separator();
        ImGui::Text("Performance");
        ImGui::Text("Emulation pacing: fixed 60 Hz");
        if (ImGui::Checkbox("VSync Playback", &config_vsync_)) {
          SDL_GL_SetSwapInterval(config_vsync_ ? 1 : 0);
        }
        ImGui::Checkbox("Detailed Profiling", &g_profile_detailed_timing);
        if (ImGui::Checkbox("Low-spec Mode", &config_low_spec_mode_)) {
            g_low_spec_mode = config_low_spec_mode_;
        }
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                           "Reduces audio complexity and internal precision.");

        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Experimental")) {
        ImGui::TextColored(ImVec4(0.9f, 0.7f, 0.3f, 1.0f),
                           "Disabled by default. Enable only for targeted testing.");
        ImGui::Separator();

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

        ImGui::Separator();
        ImGui::Checkbox("Unhandled SPECIAL fallback (rd <- 0)",
                        &g_experimental_unhandled_special_returns_zero);
        ImGui::TextColored(
            ImVec4(0.8f, 0.5f, 0.3f, 1.0f),
            "When enabled, unknown SPECIAL funct values write 0 to rd instead of raising Reserved Instruction.");

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

void App::draw_sound_status_content() {
  const auto &diag = runtime_snapshot_.spu_audio;
  const auto avg_count = [](u64 accum, u64 samples) -> float {
    const u64 denom = std::max<u64>(samples, 1);
    return static_cast<float>(accum) / static_cast<float>(denom);
  };
  const float avg_logical =
      avg_count(diag.logical_voice_accum, diag.logical_voice_samples);
  const float avg_env = avg_count(diag.env_voice_accum, diag.env_voice_samples);
  const float avg_audible =
      avg_count(diag.audible_voice_accum, diag.audible_voice_samples);
  const float queue_kb =
      static_cast<float>(diag.queue_last_bytes) / 1024.0f;
  const float queue_peak_kb =
      static_cast<float>(diag.queue_peak_bytes) / 1024.0f;

  ImGui::Text("Realtime SPU voice monitor (updates every emulated frame).");
  if (ImGui::Button("Reset Sound Stats")) {
    const bool was_running = emu_runner_.is_running();
    if (was_running) {
      emu_runner_.pause_and_wait_idle();
    }
    system_->reset_spu_audio_diag();
    runtime_snapshot_.spu_audio = system_->spu_audio_diag();
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
  ImGui::Text("Dropped Frames: %llu (Overruns: %llu, Underruns: %llu)",
              static_cast<unsigned long long>(diag.dropped_frames),
              static_cast<unsigned long long>(diag.overrun_events),
              static_cast<unsigned long long>(diag.underrun_events));
  ImGui::Text("Audio Queue: %.1f KB (peak %.1f KB)", queue_kb, queue_peak_kb);
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

    const auto &stats = runtime_snapshot_.profiling;
    ImGui::Text("Frame Time Breakdown:");
    ImGui::Separator();

    auto row = [](const char *label, double ms, ImVec4 color) {
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
    } else {
      ImGui::TextDisabled("Detailed subsystem timings are disabled.");
    }
    ImGui::Separator();
    row("Core", runtime_snapshot_.core_frame_ms, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    row("Present", present_ms_, ImVec4(0.7f, 0.9f, 0.9f, 1.0f));

    float budget_ms = 1000.0f / 60.0f;
    float usage = static_cast<float>(runtime_snapshot_.core_frame_ms / budget_ms);
    ImGui::Spacing();
    ImGui::Text("Core Frame Budget Usage (%.1f%%):", usage * 100.0f);
    ImGui::ProgressBar(usage, ImVec2(-1.0f, 0.0f));
  }
  ImGui::End();
}

void App::panel_grim_reaper() {
  ImGui::SetNextWindowSize(ImVec2(540, 330), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Grim Reaper", &show_grim_reaper_)) {
    ImGui::End();
    return;
  }

  ImGui::TextColored(ImVec4(0.9f, 0.4f, 0.4f, 1.0f),
                     "Experimental BIOS corruption. Original BIOS is never modified.");

  grim_reaper_area_index_ =
      std::max(0, std::min(kGrimReaperRangeCount - 1, grim_reaper_area_index_));
  const char *grim_area_labels[kGrimReaperRangeCount] = {};
  for (int i = 0; i < kGrimReaperRangeCount; ++i) {
    grim_area_labels[i] = kGrimReaperRanges[i].label;
  }
  ImGui::Combo("Target Area", &grim_reaper_area_index_, grim_area_labels,
               kGrimReaperRangeCount);

  const bool grim_intro_mode = (grim_reaper_area_index_ == 0);
  const float grim_slider_max = grim_intro_mode ? 0.1f : 100.0f;
  grim_reaper_random_percent_ =
      std::max(0.001f, std::min(grim_slider_max, grim_reaper_random_percent_));
  ImGui::SliderFloat("Random Strike (%)", &grim_reaper_random_percent_, 0.001f,
                     grim_slider_max, "%.3f%%");
  if (grim_intro_mode) {
    const float p = grim_reaper_random_percent_;
    if (p >= 0.1f - 1e-6f) {
      ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f),
                         "Absolute Death - Boot is very likely to fail.");
    } else if (p >= 0.05f) {
      ImGui::TextColored(
          ImVec4(0.95f, 0.2f, 0.2f, 1.0f),
          "Danger - Very unstable, may not work, extreme corruption.");
    } else if (p >= 0.02f) {
      ImGui::TextColored(
          ImVec4(1.0f, 0.55f, 0.1f, 1.0f),
          "Warning - Unstable, heavy corruptions.");
    } else if (p >= 0.01f) {
      ImGui::TextColored(
          ImVec4(0.2f, 0.9f, 0.3f, 1.0f),
          "Recommended - Stable corruptions.");
    } else {
      ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                         "Very mild corruption range.");
    }
  }
  if (grim_reaper_area_index_ == (kGrimReaperRangeCount - 1)) {
    ImGui::InputText("Custom Start (hex)", grim_reaper_custom_start_hex_,
                     IM_ARRAYSIZE(grim_reaper_custom_start_hex_));
    ImGui::InputText("Custom End (hex)", grim_reaper_custom_end_hex_,
                     IM_ARRAYSIZE(grim_reaper_custom_end_hex_));
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                       "Set end to 0 for end-of-file.");
  }

  ImGui::Separator();
  ImGui::Text("Corruption Seed");
  ImGui::Checkbox("Use Custom Seed", &grim_use_custom_seed_);
  if (grim_use_custom_seed_) {
    ImGui::InputScalar("Seed", ImGuiDataType_U32, &grim_seed_);
  }
  if (ImGui::Checkbox(
          "Do not suppress console logs when corrupting (may freeze emulator!)",
          &grim_reaper_keep_console_logs_)) {
    if (grim_reaper_mode_active_) {
      if (grim_reaper_keep_console_logs_ && grim_reaper_logs_suppressed_) {
        g_log_category_mask = grim_reaper_saved_log_mask_;
        g_log_level = grim_reaper_saved_log_level_;
        grim_reaper_logs_suppressed_ = false;
      } else if (!grim_reaper_keep_console_logs_ &&
                 !grim_reaper_logs_suppressed_) {
        grim_reaper_saved_log_mask_ = g_log_category_mask;
        grim_reaper_saved_log_level_ = g_log_level;
        grim_reaper_logs_suppressed_ = true;
        g_log_category_mask = 0;
        g_log_level = LogLevel::Error;
      }
    }
  }
  ImGui::Text("Last Used Seed: %u", static_cast<unsigned>(grim_last_used_seed_));
  if (!system_->bios_loaded() || bios_path_.empty()) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("Reap && Reboot BIOS")) {
    reap_and_reboot_bios();
  }
  ImGui::SameLine();
  if (ImGui::Button("Boot Input Seed")) {
    const bool prev_use_custom_seed = grim_use_custom_seed_;
    grim_use_custom_seed_ = true;
    reap_and_reboot_bios();
    grim_use_custom_seed_ = prev_use_custom_seed;
  }
  if (!system_->bios_loaded() || bios_path_.empty()) {
    ImGui::EndDisabled();
    ImGui::TextColored(ImVec4(0.8f, 0.6f, 0.3f, 1.0f), "Load a BIOS first.");
  }

  ImGui::SameLine();
  if (!has_started_emulation_) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("Stop")) {
    emu_runner_.pause_and_wait_idle();
    has_started_emulation_ = false;
    status_message_ = "Emulation stopped";
  }
  if (!has_started_emulation_) {
    ImGui::EndDisabled();
  }

  ImGui::SameLine();
  if (grim_reaper_last_output_path_.empty()) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("Restart Corrupted BIOS")) {
    emu_runner_.pause_and_wait_idle();
    set_grim_reaper_mode(true);
    if (!system_->load_bios(grim_reaper_last_output_path_)) {
      set_grim_reaper_mode(false);
      status_message_ = "Failed to load last corrupted BIOS copy.";
    } else {
      has_started_emulation_ = false;
      system_->reset();
      has_started_emulation_ = true;
      emu_runner_.set_running(true);
      status_message_ = "Corrupted BIOS emulation restarted";
    }
  }
  if (grim_reaper_last_output_path_.empty()) {
    ImGui::EndDisabled();
  }



  ImGui::Separator();
  ImGui::Text("Batch Corruption");
  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                     "Select multiple ranges and apply random strike per range.");

  ImGui::Checkbox("Intro/Bootmenu", &grim_batch_intro_enabled_);
  if (grim_batch_intro_enabled_) {
    grim_batch_intro_percent_ =
        std::max(0.001f, std::min(0.1f, grim_batch_intro_percent_));
    ImGui::SliderFloat("Intro Strike (%)", &grim_batch_intro_percent_, 0.001f,
                       0.1f, "%.3f%%");

    const float p = grim_batch_intro_percent_;
    if (p >= 0.1f - 1e-6f) {
      ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f),
                         "Absolute Death - Boot is very likely to fail.");
    } else if (p >= 0.05f) {
      ImGui::TextColored(
          ImVec4(0.95f, 0.2f, 0.2f, 1.0f),
          "Danger - Very unstable, may not work, extreme corruption.");
    } else if (p >= 0.02f) {
      ImGui::TextColored(
          ImVec4(1.0f, 0.55f, 0.1f, 1.0f),
          "Warning - Unstable, heavy corruptions.");
    } else if (p >= 0.01f) {
      ImGui::TextColored(
          ImVec4(0.2f, 0.9f, 0.3f, 1.0f),
          "Recommended - Stable corruptions.");
    }
  }

  ImGui::Checkbox("Character Sets", &grim_batch_charset_enabled_);
  if (grim_batch_charset_enabled_) {
    grim_batch_charset_percent_ =
        std::max(0.001f, std::min(100.0f, grim_batch_charset_percent_));
    ImGui::SliderFloat("Charset Strike (%)", &grim_batch_charset_percent_, 0.001f,
                       100.0f, "%.3f%%");
  }

  ImGui::Checkbox("End", &grim_batch_end_enabled_);
  if (grim_batch_end_enabled_) {
    grim_batch_end_percent_ =
        std::max(0.001f, std::min(100.0f, grim_batch_end_percent_));
    ImGui::SliderFloat("End Strike (%)", &grim_batch_end_percent_, 0.001f,
                       100.0f, "%.3f%%");
  }

  const bool batch_has_any_selection =
      grim_batch_intro_enabled_ || grim_batch_charset_enabled_ || grim_batch_end_enabled_;
  if (!system_->bios_loaded() || bios_path_.empty() || !batch_has_any_selection) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("Batch Corrupt && Start")) {
    reap_and_reboot_bios_batch();
  }
  if (!system_->bios_loaded() || bios_path_.empty() || !batch_has_any_selection) {
    ImGui::EndDisabled();
  }

  if (!grim_reaper_last_output_path_.empty()) {
    ImGui::Text("Last Corrupted BIOS: %s", grim_reaper_last_output_path_.c_str());
    ImGui::Text("Last Mutations: %u",
                static_cast<unsigned>(grim_reaper_last_mutations_));
  }
  ImGui::TextColored(grim_reaper_mode_active_ ? ImVec4(0.9f, 0.6f, 0.3f, 1.0f)
                                               : ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                     "Console logging: %s",
                     grim_reaper_logs_suppressed_ ? "Suppressed" : "Normal");

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
    const bool running = emu_runner_.is_running();
    if (running) {
      ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.4f, 1.0f),
                         "Pause emulation to inspect registers.");
    } else {
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
      } else {
        has_started_emulation_ = true;
        emu_runner_.set_running(true);
      }
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
  emu_runner_.pause_and_wait_idle();

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
  emu_runner_.set_running(true);
  status_message_ = "Booting disc from BIOS...";
  return true;
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
  in.read(reinterpret_cast<char *>(bios_data.data()),
          static_cast<std::streamsize>(bios_data.size()));
  if (!in) {
    status_message_ = "Failed reading source BIOS.";
    return false;
  }

  grim_reaper_area_index_ =
      std::max(0, std::min(kGrimReaperRangeCount - 1, grim_reaper_area_index_));

  auto parse_hex = [](const char *text, size_t &out) -> bool {
    if (!text) {
      return false;
    }
    while (*text != '\0' && std::isspace(static_cast<unsigned char>(*text))) {
      ++text;
    }
    if (*text == '\0') {
      return false;
    }
    char *end_ptr = nullptr;
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
  } else {
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

  const u32 seed = grim_use_custom_seed_
                       ? grim_seed_
                       : static_cast<u32>(std::random_device{}());
  grim_last_used_seed_ = seed;
  grim_seed_ = seed;
  std::mt19937 rng(seed);

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
  out.write(reinterpret_cast<const char *>(bios_data.data()),
            static_cast<std::streamsize>(bios_data.size()));
  if (!out) {
    status_message_ = "Failed writing corrupted BIOS copy.";
    return false;
  }

  emu_runner_.pause_and_wait_idle();
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
  in.read(reinterpret_cast<char *>(bios_data.data()),
          static_cast<std::streamsize>(bios_data.size()));
  if (!in) {
    status_message_ = "Failed reading source BIOS.";
    return false;
  }

  const u32 seed = grim_use_custom_seed_
                       ? grim_seed_
                       : static_cast<u32>(std::random_device{}());
  grim_last_used_seed_ = seed;
  grim_seed_ = seed;
  std::mt19937 rng(seed);

  size_t total_mutations = 0;
  auto apply_range = [&](const GrimReaperRange &range, float percent,
                         float max_percent) -> bool {
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

    for (size_t i = 0; i < mutations; ++i) {
      const size_t idx = start + (static_cast<size_t>(rng()) % span);
      bios_data[idx] = static_cast<u8>(rng() & 0xFFu);
    }

    total_mutations += mutations;
    return true;
  };

  std::string slug = "batch";
  if (do_intro) {
    if (!apply_range(kGrimReaperRanges[0], grim_batch_intro_percent_, 0.1f)) {
      status_message_ = "Intro range is outside BIOS size.";
      return false;
    }
    slug += "_intro";
  }
  if (do_charset) {
    if (!apply_range(kGrimReaperRanges[1], grim_batch_charset_percent_, 100.0f)) {
      status_message_ = "Character Sets range is outside BIOS size.";
      return false;
    }
    slug += "_charset";
  }
  if (do_end) {
    if (!apply_range(kGrimReaperRanges[2], grim_batch_end_percent_, 100.0f)) {
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
  out.write(reinterpret_cast<const char *>(bios_data.data()),
            static_cast<std::streamsize>(bios_data.size()));
  if (!out) {
    status_message_ = "Failed writing corrupted BIOS copy.";
    return false;
  }

  emu_runner_.pause_and_wait_idle();
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
  has_started_emulation_ = true;
  emu_runner_.set_running(true);

  status_message_ = "Batch reaped BIOS loaded (seed " + std::to_string(seed) + "): " +
                    out_path.filename().string();
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


void App::load_persistent_config() {
  std::ifstream in(kAppConfigFileName);
  if (!in.is_open()) {
    return;
  }

  auto trim = [](std::string &s) {
    const size_t begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      s.clear();
      return;
    }
    const size_t end = s.find_last_not_of(" \t\r\n");
    s = s.substr(begin, end - begin + 1);
  };
  auto parse_bool = [](const std::string &v, bool fallback) {
    if (v == "1" || v == "true" || v == "TRUE" || v == "on" || v == "ON") {
      return true;
    }
    if (v == "0" || v == "false" || v == "FALSE" || v == "off" || v == "OFF") {
      return false;
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
    } else if (key == "vsync") {
      config_vsync_ = parse_bool(value, config_vsync_);
    } else if (key == "low_spec_mode") {
      config_low_spec_mode_ = parse_bool(value, config_low_spec_mode_);
      g_low_spec_mode = config_low_spec_mode_;
    } else if (key == "gpu_fast_mode") {
      g_gpu_fast_mode = parse_bool(value, g_gpu_fast_mode);
    } else if (key.rfind("bind_", 0) == 0) {
      const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 10);
      const SDL_Scancode scancode = static_cast<SDL_Scancode>(parsed);
      for (const auto &entry : kKeyboardBindEntries) {
        if (key == entry.config_key) {
          if (scancode == SDL_SCANCODE_UNKNOWN) {
            input_->clear_key_binding(entry.button);
          } else {
            input_->set_key_binding(scancode, entry.button);
          }
          break;
        }
      }
    } else if (key == "spu_desired_samples") {
      const unsigned long parsed = std::strtoul(value.c_str(), nullptr, 10);
      const u32 clamped = static_cast<u32>(std::max(64ul, std::min(65535ul, parsed)));
      g_spu_desired_samples = clamped;
    } else if (key == "advanced_sound_status_logging") {
      g_spu_advanced_sound_status =
          parse_bool(value, g_spu_advanced_sound_status);
    } else if (key == "detailed_profiling") {
      g_profile_detailed_timing = parse_bool(value, g_profile_detailed_timing);
    } else if (key == "experimental_bios_size_mode") {
      g_experimental_bios_size_mode = parse_bool(value, g_experimental_bios_size_mode);
    } else if (key == "unsafe_ps2_bios_mode") {
      g_unsafe_ps2_bios_mode = parse_bool(value, g_unsafe_ps2_bios_mode);
    } else if (key == "experimental_unhandled_special_returns_zero") {
      g_experimental_unhandled_special_returns_zero =
          parse_bool(value, g_experimental_unhandled_special_returns_zero);
    } else if (key == "deinterlace_mode") {
      const unsigned long mode = std::strtoul(value.c_str(), nullptr, 10);
      const int idx = static_cast<int>(std::max(0ul, std::min(2ul, mode)));
      g_deinterlace_mode = static_cast<DeinterlaceMode>(idx);
    } else if (key == "output_resolution_mode") {
      const unsigned long mode = std::strtoul(value.c_str(), nullptr, 10);
      const int idx = static_cast<int>(std::max(0ul, std::min(2ul, mode)));
      g_output_resolution_mode = static_cast<OutputResolutionMode>(idx);
    }
  }

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
  out << "vsync=" << (config_vsync_ ? 1 : 0) << "\n";
  out << "low_spec_mode=" << (config_low_spec_mode_ ? 1 : 0) << "\n";
  out << "gpu_fast_mode=" << (g_gpu_fast_mode ? 1 : 0) << "\n";
  for (const auto &entry : kKeyboardBindEntries) {
    out << entry.config_key << "="
        << static_cast<int>(input_->key_for_button(entry.button)) << "\n";
  }
  out << "spu_desired_samples=" << static_cast<unsigned>(g_spu_desired_samples) << "\n";
  out << "advanced_sound_status_logging=" << (g_spu_advanced_sound_status ? 1 : 0) << "\n";
  out << "detailed_profiling=" << (g_profile_detailed_timing ? 1 : 0) << "\n";
  out << "experimental_bios_size_mode=" << (g_experimental_bios_size_mode ? 1 : 0) << "\n";
  out << "unsafe_ps2_bios_mode=" << (g_unsafe_ps2_bios_mode ? 1 : 0) << "\n";
  out << "experimental_unhandled_special_returns_zero=" <<
      (g_experimental_unhandled_special_returns_zero ? 1 : 0) << "\n";
  out << "deinterlace_mode=" << static_cast<int>(g_deinterlace_mode) << "\n";
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
  } else {
    status_message_ = "Failed to auto-load saved BIOS. Load BIOS manually.";
  }
}
void App::shutdown() {
  save_persistent_config();
  emu_runner_.stop();
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



