#pragma once

#include "core/ps2_system.h"
#include "ui/ps2_gl_gs_backend.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct SDL_Window;
struct _SDL_GameController;
typedef struct _SDL_GameController SDL_GameController;
typedef void* SDL_GLContext;

namespace ps2::ui {

class Ps2App {
public:
    ~Ps2App();
    bool init();
    int run();
    void shutdown();
    bool launch_bios(const std::string& path);
    void set_ee_jit_enabled(bool enabled);
    void capture_visible_window(
        const std::string& path,
        unsigned long long minimum_ee_instructions = 0);

private:
    void process_events(bool& quit);
    void update_pad_input();
    void update_audio();
    void reset_audio_stutter();
    void remember_audio_history(const s16* samples, std::size_t frames);
    void audio_stutter_thread_main();
    void render_ui();
    void update_display_texture();
    void menu_bar();
    void panel_main();
    void panel_system();
    void panel_ee_debug();
    void panel_iop_debug();
    void panel_gs_debug();
    void panel_scheduler();
    void panel_settings();
    void panel_about();

    std::string open_bios_dialog();
    bool load_bios_from_path(const std::string& path);
    bool start_bios();
    bool step_ee_once();
    bool step_iop_once();
    void update_emulation();
    void reset_core();
    bool write_window_ppm(const std::string& path, int width, int height);

    SDL_Window* window_ = nullptr;
    SDL_GLContext gl_context_ = nullptr;
    int gl_major_ = 0;
    int gl_minor_ = 0;
    std::unique_ptr<Ps2GlGsBackend> gpu_gs_backend_{};
    SDL_GameController* controller_ = nullptr;
    unsigned int audio_device_ = 0;
    std::atomic<bool> lag_stutter_enabled_{true};
    std::atomic<bool> lag_stutter_active_{false};
    std::vector<s16> audio_history_{};
    std::size_t audio_history_write_frame_ = 0;
    std::size_t audio_history_play_frame_ = 0;
    std::mutex audio_history_mutex_{};
    std::atomic<bool> audio_stutter_thread_stop_{false};
    std::thread audio_stutter_thread_{};
    const char* imgui_glsl_version_ = "#version 330";
    bool use_imgui_opengl2_backend_ = false;
    bool gpu_gs_enabled_ = true;
    unsigned int display_texture_ = 0;
    u32 display_texture_width_ = 0;
    u32 display_texture_height_ = 0;
    u64 display_texture_generation_ = ~0ull;

    Ps2System system_{};

    bool show_system_ = false;
    bool show_ee_debug_ = false;
    bool show_iop_debug_ = false;
    bool show_gs_debug_ = false;
    bool show_scheduler_ = false;
    bool show_settings_ = false;
    bool show_about_ = false;
    bool emulation_running_ = false;
    bool bootstrap_swap_interval_disabled_ = false;
    std::chrono::steady_clock::time_point speed_sample_time_{};
    u64 speed_sample_instructions_ = 0;
    u64 speed_sample_fields_ = 0;
    double ee_instructions_per_second_ = 0.0;
    double guest_fields_per_second_ = 0.0;
    double guest_frames_per_second_ = 0.0;
    double emulation_speed_percent_ = 0.0;

    u64 profile_sample_native_instructions_ = 0;
    u64 profile_sample_native_blocks_ = 0;
    u64 profile_sample_guard_bailouts_ = 0;
    u64 profile_sample_code_store_exits_ = 0;
    u64 profile_sample_cache_flushes_ = 0;
    u64 profile_sample_run_ns_ = 0;
    u64 profile_sample_ee_ns_ = 0;
    u64 profile_sample_iop_ns_ = 0;
    u64 profile_sample_vu_ns_ = 0;
    u64 profile_sample_iop_instructions_ = 0;
    u64 profile_sample_vu1_instructions_ = 0;
    std::array<u64, 64> profile_sample_fallback_opcodes_{};

    double profile_native_mips_ = 0.0;
    double profile_iop_mips_ = 0.0;
    double profile_vu1_mips_ = 0.0;
    double profile_native_coverage_percent_ = 0.0;
    double profile_native_blocks_per_second_ = 0.0;
    double profile_average_native_block_ = 0.0;
    double profile_guard_bailouts_per_second_ = 0.0;
    double profile_code_store_exits_per_second_ = 0.0;
    double profile_cache_flushes_per_second_ = 0.0;
    double profile_host_ee_percent_ = 0.0;
    double profile_host_iop_percent_ = 0.0;
    double profile_host_vu_percent_ = 0.0;
    double profile_host_other_percent_ = 0.0;
    std::array<double, 64> profile_fallbacks_per_second_{};

    std::string visible_capture_path_{};
    unsigned long long visible_capture_minimum_ee_ = 0;

    std::array<char, 1024> bios_path_input_{};
    std::string status_message_ = "PS2 experimental core ready";
};

} // namespace ps2::ui
