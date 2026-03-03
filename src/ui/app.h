#pragma once
#include "emu_runner.h"
#include "../core/renderer.h"
#include "../core/system.h"
#include "../core/types.h"
#include "../input/input_manager.h"
#include <memory>
#include <string>


struct SDL_Window;
union SDL_Event;
struct ImGuiIO;
typedef void *SDL_GLContext;

// ── ImGui Application Shell ────────────────────────────────────────
// Top-level application that owns the System, Renderer, and UI.

class App {
public:
  bool init();
  void run();
  void shutdown();

private:
  SDL_Window *window_ = nullptr;
  SDL_GLContext gl_context_ = nullptr;

  std::unique_ptr<System> system_;
  std::unique_ptr<Renderer> renderer_;
  std::unique_ptr<InputManager> input_;
  EmuRunner emu_runner_;
  bool runtime_ready_ = false;

  // UI State
  bool show_demo_window_ = false;
  bool show_settings_ = false;
  bool show_about_ = false;
  bool show_debug_cpu_ = false;
  bool show_vram_ = false;
  bool show_perf_ = false;
  bool show_logging_ = false;
  std::string bios_path_;
  std::string game_bin_path_;
  std::string game_cue_path_;
  std::string status_message_ = "Welcome to VibeStation!";
  char log_path_[260] = "vibestation_runtime.log";
  bool has_started_emulation_ = false;
  bool emu_input_focused_ = false;
  u16 last_button_state_ = 0xFFFF;

  // Frame timing
  float fps_ = 0.0f;
  u32 frame_count_ = 0;
  u32 last_fps_time_ = 0;
  u32 last_vram_update_ms_ = 0;
  double present_ms_ = 0.0;
  EmuRunner::RuntimeSnapshot runtime_snapshot_{};
  unsigned int vram_debug_texture_ = 0;

  // Configurable performance options
  bool config_vsync_ = true;
  bool config_low_spec_mode_ = false;

  void process_events(bool &quit);
  bool should_route_keyboard_to_emu(const SDL_Event &event,
                                    const ImGuiIO &io) const;
  void update();
  void render_ui();

  // UI panels
  void menu_bar();
  void panel_emulator_screen();
  void panel_settings();
  void panel_about();
  void panel_debug_cpu();
  void panel_vram();
  void panel_performance();
  void update_vram_debug_texture();

  // File dialog helpers
  std::string open_file_dialog(const char *filter, const char *title);
  bool resolve_disc_paths(const std::string &selected_path, std::string &bin_path,
                          std::string &cue_path, std::string &error) const;
  bool boot_disc_from_ui();

  // Deferred heavy initialization to avoid large stack allocations on startup.
  bool init_runtime();
};
