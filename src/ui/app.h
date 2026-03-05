#pragma once
#include "emu_runner.h"
#include "../core/renderer.h"
#include "../core/system.h"
#include "../core/types.h"
#include "../input/input_manager.h"
#include <memory>
#include <string>
#include <vector>

struct SDL_Window;
union SDL_Event;
struct ImGuiIO;
typedef void *SDL_GLContext;

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
  bool show_sound_status_ = false;
  bool show_grim_reaper_ = false;
  std::string bios_path_;
  std::string game_bin_path_;
  std::string game_cue_path_;
  std::string status_message_ = "Welcome to VibeStation!";
  char log_path_[260] = "vibestation_runtime.log";
  bool has_started_emulation_ = false;
  bool emu_input_focused_ = false;
  u16 last_button_state_ = 0xFFFF;
  bool show_fast_mode_notice_ = false;
  std::array<u32, 5> underrun_notice_buckets_ = {};
  u32 underrun_notice_bucket_index_ = 0;
  u32 underrun_notice_bucket_count_ = 0;
  u32 underrun_notice_bucket_sum_ = 0;
  u32 underrun_notice_last_tick_ms_ = 0;
  u64 underrun_notice_last_events_ = 0;
  int pending_bind_index_ = -1;

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

  // Grim Reaper (experimental BIOS corruption sandbox)
  int grim_reaper_area_index_ = 0;
  float grim_reaper_random_percent_ = 0.15f;
  char grim_reaper_custom_start_hex_[32] = "18000";
  char grim_reaper_custom_end_hex_[32] = "0";
  u32 grim_reaper_last_mutations_ = 0;
  std::string grim_reaper_last_output_path_;
  bool grim_reaper_mode_active_ = false;
  bool grim_reaper_keep_console_logs_ = false;
  bool grim_reaper_logs_suppressed_ = false;
  u32 grim_reaper_saved_log_mask_ = 0xFFFFFFFFu;
  LogLevel grim_reaper_saved_log_level_ = LogLevel::Info;
  bool grim_batch_intro_enabled_ = false;
  bool grim_batch_charset_enabled_ = false;
  bool grim_batch_end_enabled_ = false;
  float grim_batch_intro_percent_ = 0.02f;
  float grim_batch_charset_percent_ = 89.0f;
  float grim_batch_end_percent_ = 100.0f;
  bool grim_use_custom_seed_ = false;
  u32 grim_seed_ = 1u;
  u32 grim_last_used_seed_ = 0u;

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
  void panel_sound_status();
  void panel_grim_reaper();
  void draw_sound_status_content();
  void update_vram_debug_texture();

  // File dialog helpers
  std::string open_file_dialog(const char *filter, const char *title);
  bool resolve_disc_paths(const std::string &selected_path, std::string &bin_path,
                          std::string &cue_path, std::string &error) const;
  bool boot_disc_from_ui();
  bool reap_and_reboot_bios();
  bool reap_and_reboot_bios_batch();
  void set_grim_reaper_mode(bool enabled);

  // Deferred heavy initialization to avoid large stack allocations on startup.
  bool init_runtime();
  void load_persistent_config();
  void save_persistent_config() const;
  void try_autoload_bios_from_config();
};
