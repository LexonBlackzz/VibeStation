#pragma once
#include "emu_runner.h"
#include "../core/renderer.h"
#include "../core/system.h"
#include "../core/types.h"
#include "../input/input_manager.h"
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

struct SDL_Window;
union SDL_Event;
struct ImGuiIO;
struct ImVec2;
typedef void* SDL_GLContext;

class App {
public:
	bool init();
	void run();
	void shutdown();

private:
	SDL_Window* window_ = nullptr;
	SDL_GLContext gl_context_ = nullptr;
	const char* imgui_glsl_version_ = "#version 330";
	bool use_imgui_opengl2_backend_ = false;

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
	bool show_corruption_presets_ = false;
	std::string bios_path_;
	std::string rom_directory_;
	std::string game_bin_path_;
	std::string game_cue_path_;
	std::string status_message_ = "Welcome to VibeStation!";
	char log_path_[260] = "vibestation_runtime.log";
	char log_search_[128] = "";
	u64 selected_log_seq_ = 0;
	bool log_auto_scroll_ = true;
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
	static constexpr int kPerfHistorySamples = 240;
	float fps_ = 0.0f;
	u32 frame_count_ = 0;
	u32 last_fps_time_ = 0;
	u32 last_vram_update_ms_ = 0;
	double present_ms_ = 0.0;
	double render_ms_ = 0.0;
	double swap_ms_ = 0.0;
	std::array<float, kPerfHistorySamples> perf_cpu_ms_history_ = {};
	std::array<float, kPerfHistorySamples> perf_gpu_ms_history_ = {};
	std::array<float, kPerfHistorySamples> perf_core_ms_history_ = {};
	int perf_history_write_index_ = 0;
	int perf_history_count_ = 0;
	EmuRunner::RuntimeSnapshot runtime_snapshot_{};
	std::vector<u32> latest_frame_rgba_{};
	int latest_frame_width_ = 0;
	int latest_frame_height_ = 0;
	unsigned int vram_debug_texture_ = 0;

	// Configurable performance options
	bool config_vsync_ = true;
	bool config_low_spec_mode_ = false;
	bool config_direct_disc_boot_ = false;
	int config_turbo_speed_percent_ = 200;
	int config_slowdown_speed_percent_ = 50;
	bool config_spu_diagnostic_mode_ = false;
	static constexpr int kMemoryCardSlotCount = 2;
	std::array<int, kMemoryCardSlotCount> config_memory_card_mode_ = { 0, 0 };
	std::array<std::string, kMemoryCardSlotCount> memory_card_target_paths_{};
	bool turbo_hold_active_ = false;
	bool slowdown_hold_active_ = false;

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
	bool grim_batch_use_custom_seeds_ = false;
	u64 grim_batch_intro_seed_ = 1u;
	u64 grim_batch_charset_seed_ = 1u;
	u64 grim_batch_end_seed_ = 1u;
	bool grim_use_custom_seed_ = false;
	u64 grim_seed_ = 1u;
	u64 grim_last_used_seed_ = 0u;
	bool ram_reaper_enabled_ = false;
	u32 ram_reaper_writes_per_frame_ = 64u;
	float ram_reaper_intensity_percent_ = 35.0f;
	bool ram_reaper_affect_main_ram_ = true;
	bool ram_reaper_affect_vram_ = true;
	bool ram_reaper_affect_spu_ram_ = true;
	u32 ram_reaper_range_start_ = 0u;
	u32 ram_reaper_range_end_ = psx::RAM_SIZE - 1u;
	bool ram_reaper_use_custom_seed_ = false;
	u64 ram_reaper_seed_ = 1u;
	u64 ram_reaper_active_seed_ = 0u;
	u64 ram_reaper_total_mutations_ = 0;
	bool gpu_reaper_enabled_ = false;
	u32 gpu_reaper_writes_per_frame_ = 48u;
	float gpu_reaper_intensity_percent_ = 25.0f;
	bool gpu_reaper_affect_geometry_ = true;
	bool gpu_reaper_affect_texture_state_ = true;
	bool gpu_reaper_affect_display_state_ = false;
	bool gpu_reaper_use_custom_seed_ = false;
	u64 gpu_reaper_seed_ = 1u;
	u64 gpu_reaper_active_seed_ = 0u;
	u64 gpu_reaper_total_mutations_ = 0;
	bool sound_reaper_enabled_ = false;
	u32 sound_reaper_writes_per_frame_ = 32u;
	float sound_reaper_intensity_percent_ = 20.0f;
	bool sound_reaper_affect_pitch_ = true;
	bool sound_reaper_affect_envelope_ = true;
	bool sound_reaper_affect_reverb_ = true;
	bool sound_reaper_affect_mixer_ = true;
	bool sound_reaper_use_custom_seed_ = false;
	u64 sound_reaper_seed_ = 1u;
	u64 sound_reaper_active_seed_ = 0u;
	u64 sound_reaper_total_mutations_ = 0;
	int sound_ram_voice_index_ = 0;
	char grim_preset_name_[64] = "grim_preset";
	char batch_preset_name_[64] = "grim_batch";
	char ram_preset_name_[64] = "ram_reaper";
	char gpu_preset_name_[64] = "gpu_reaper";
	char sound_preset_name_[64] = "sound_reaper";
	int selected_corruption_preset_index_ = -1;
	bool game_library_dirty_ = true;
	u32 game_library_last_scan_ms_ = 0;
	bool rom_directory_valid_ = false;

	struct GameLibraryEntry {
		std::string title;
		std::string bin_path;
		std::string cue_path;
	};
	std::vector<GameLibraryEntry> game_library_;

	struct CorruptionPresetListEntry {
		std::string file_name;
		std::string display_name;
		std::string preset_type;
		std::filesystem::path path;
	};
	std::vector<CorruptionPresetListEntry> corruption_presets_;

	void process_events(bool& quit);
	bool should_route_keyboard_to_emu(const SDL_Event& event,
		const ImGuiIO& io) const;
	void update();
	void render_ui();
	void push_performance_history_sample();
	void draw_performance_overlay(const ImVec2& image_pos, const ImVec2& image_size);

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
	void panel_corruption_presets();
	void draw_sound_status_content();
	void update_vram_debug_texture();

	// File dialog helpers
	std::string open_file_dialog(const char* filter, const char* title);
	std::string open_folder_dialog(const char* title);
	bool resolve_disc_paths(const std::string& selected_path, std::string& bin_path,
		std::string& cue_path, std::string& error) const;
	void refresh_game_library();
	bool load_disc_from_ui(const std::string& bin_path, const std::string& cue_path);
	bool start_bios_from_ui();
	bool boot_disc_from_ui();
	bool save_snapshot_png();
	bool reap_and_reboot_bios();
	bool reap_and_reboot_bios_batch();
	std::array<std::string, kMemoryCardSlotCount> resolve_memory_card_paths() const;
	void apply_memory_card_settings(bool save_config);
	bool save_current_grim_preset(bool batch_mode);
	bool save_current_ram_preset();
	bool save_current_gpu_preset();
	bool save_current_sound_preset();
	bool load_corruption_preset(const std::filesystem::path& path);
	void refresh_corruption_preset_list();
	void set_grim_reaper_mode(bool enabled);
	void sync_ram_reaper_config();
	void disable_ram_reaper_mode();
	void sync_gpu_reaper_config();
	void disable_gpu_reaper_mode();
	void sync_sound_reaper_config();
	void disable_sound_reaper_mode();

	// Deferred heavy initialization to avoid large stack allocations on startup.
	bool init_runtime();
	void load_persistent_config();
	void save_persistent_config() const;
	void try_autoload_bios_from_config();
	double current_speed_override() const;
	void apply_speed_override();
};
