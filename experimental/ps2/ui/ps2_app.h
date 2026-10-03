#pragma once

#include "core/ps2_system.h"
#include "ps1/ps1_mode.h"
#include "ui/ps2_gl_gs_backend.h"
#include "ui/vs2/vs2_host.h"

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
struct ImGuiContext;

namespace ps2::ui {

namespace vs2 {
class Frontend;
}

// A window owned by someone else (VibeStation, when the PS2 app runs inside
// it next to VibeStation 1).
struct HostedWindow {
    SDL_Window* window = nullptr;
    SDL_GLContext gl_context = nullptr;
    int gl_major = 0;
    int gl_minor = 0;
    const char* glsl = "#version 330";
    bool opengl2 = false;
};

class Ps2App : public vs2::Host {
public:
    Ps2App();
    ~Ps2App() override;
    // Without a host, creates its own window (the standalone PS2 lab).
    bool init(const HostedWindow* host = nullptr);
    int run();
    // run() split up for a host: begin_run() once, then frame() until it
    // returns false (quit).
    void begin_run();
    bool frame();
    // The host switched to or away from this app.
    void on_activated();
    void on_deactivated();
    // True once, after the user chose "VibeStation 1" in the menu.
    bool take_vs1_switch_request();
    // Starts the switch as if "VibeStation 1" had been chosen (--switch-test).
    void begin_vs1_switch();
    void shutdown();
    bool launch_bios(const std::string& path);
    bool load_disc_from_path(const std::string& path);
    void set_ps1_bios(const std::string& path) { ps1_bios_path_ = path; }
    void set_ee_jit_enabled(bool enabled);
    void set_ee_dynarec_enabled(bool enabled);
    void set_gpu_gs_enabled(bool enabled) override;
    // Opens the PS2 lab interface instead of the VibeStation 2 frontend.
    void set_developer_view(bool enabled);

    // vs2::Host
    [[nodiscard]] bool bios_loaded() const override;
    [[nodiscard]] std::string bios_romver() const override;
    [[nodiscard]] std::string bios_path() const override;
    bool load_bios(const std::string& path) override;
    std::string pick_bios_file() override;
    std::string pick_folder(const char* title) override;
    bool start_bios_session() override;
    bool boot_disc(const std::string& path) override;
    [[nodiscard]] bool session_active() const override;
    [[nodiscard]] bool session_running() const override;
    void pause_session() override;
    void resume_session() override;
    [[nodiscard]] vs2::GameView game_view() const override;
    [[nodiscard]] double speed_percent() const override;
    [[nodiscard]] double frames_per_second() const override;
    [[nodiscard]] std::string status_message() const override;
    void set_turbo(bool active) override;
    bool save_snapshot() override;
    [[nodiscard]] vs2::EeCore ee_core() const override;
    void set_ee_core(vs2::EeCore core) override;
    [[nodiscard]] bool gpu_gs_available() const override;
    [[nodiscard]] bool gpu_gs_enabled() const override;
    [[nodiscard]] bool limit_speed() const override;
    void set_limit_speed(bool enabled) override;
    [[nodiscard]] int display_filter() const override;
    void set_display_filter(int filter) override;
    [[nodiscard]] bool lag_stutter() const override;
    void set_lag_stutter(bool enabled) override;
    void open_developer_view() override;
    void request_quit() override;
    [[nodiscard]] bool can_switch_to_vs1() const override;
    void switch_to_vs1() override;
    void capture_visible_window(
        const std::string& path,
        unsigned long long minimum_ee_instructions = 0);
    // Runs the full UI until `fields` guest fields have elapsed after the
    // first visible BIOS frame, prints the measured rate, then exits.
    void benchmark_visible_fields(u64 fields) { benchmark_fields_ = fields; }

private:
    bool create_own_window();
    bool init_after_window();
    void process_events(bool& quit);
    void update_pad_input();
    void update_audio();
    void reset_audio_stutter();
    void remember_audio_history(const s16* samples, std::size_t frames);
    void audio_stutter_thread_main();
    void render_ui();
    void update_display_texture();
    void update_ps1_texture();
    void menu_bar();
    void panel_main();
    void panel_system();
    void panel_ee_debug();
    void panel_iop_debug();
    void panel_gs_debug();
    void panel_scheduler();
    void panel_settings();
    void panel_about();

    std::string open_bios_dialog(const char* title = "Select PlayStation 2 BIOS");
    bool start_ps1(const std::string& disc_path);
    void stop_ps1();
    std::string open_disc_dialog();
    bool load_bios_from_path(const std::string& path);
    bool start_bios();
    bool step_ee_once();
    bool step_iop_once();
    void update_emulation();
    void emulation_thread_main();
    void stop_emulation_thread();
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
    // False while paused, halted or stopped: the 400 ms tape must not replay.
    std::atomic<bool> audio_live_{false};
    std::vector<s16> audio_history_{};
    std::size_t audio_history_write_frame_ = 0;
    std::size_t audio_history_play_frame_ = 0;
    std::mutex audio_history_mutex_{};
    std::atomic<bool> audio_stutter_thread_stop_{false};
    std::thread audio_stutter_thread_{};
    const char* imgui_glsl_version_ = "#version 330";
    bool use_imgui_opengl2_backend_ = false;
    bool gpu_gs_enabled_ = false;
    unsigned int display_texture_ = 0;
    u32 display_texture_width_ = 0;
    u32 display_texture_height_ = 0;
    u64 display_texture_generation_ = ~0ull;
    // Host-side scaling of the 640x448 output: 0 = nearest, 1 = bilinear.
    int display_filter_ = 1;
    int display_filter_applied_ = -1;

    // The core runs on emu_thread_. core_mutex_ guards system_ and every UI
    // member derived from it: the UI frame holds it except while waiting for
    // vsync, and the emulation thread holds it per short slice. ui_waiting_
    // makes the emulation thread back off so the UI is never starved.
    std::mutex core_mutex_{};
    std::atomic<bool> ui_waiting_{false};
    std::atomic<bool> emu_stop_{false};
    std::atomic<bool> limit_speed_{true};
    // Held fast-forward: ignores limit_speed_ without changing the setting.
    std::atomic<bool> turbo_{false};
    std::thread emu_thread_{};

    Ps2System system_{};

    // PS1 discs run on the standalone PS1 core while PS2 emulation is idle.
    std::unique_ptr<Ps1Mode> ps1_ = std::make_unique<Ps1Mode>();
    std::string ps1_bios_path_{};
    std::vector<u32> ps1_frame_{};
    unsigned int ps1_texture_ = 0;
    int ps1_width_ = 0;
    int ps1_height_ = 0;
    int ps1_texture_width_ = 0;
    int ps1_texture_height_ = 0;
    int ps1_speed_index_ = 0;

    bool show_system_ = false;
    bool show_ee_debug_ = false;
    bool show_iop_debug_ = false;
    bool show_gs_debug_ = false;
    bool show_scheduler_ = false;
    bool show_settings_ = false;
    bool show_about_ = false;
    bool emulation_running_ = false;
    std::chrono::steady_clock::time_point speed_sample_time_{};
    u64 speed_sample_instructions_ = 0;
    u64 speed_sample_fields_ = 0;
    double ee_instructions_per_second_ = 0.0;
    double guest_fields_per_second_ = 0.0;
    double guest_frames_per_second_ = 0.0;
    double emulation_speed_percent_ = 0.0;
    std::string visible_capture_path_{};
    unsigned long long visible_capture_minimum_ee_ = 0;
    u64 benchmark_fields_ = 0;
    bool benchmark_started_ = false;
    u64 benchmark_start_field_ = 0;
    u64 benchmark_ui_frames_ = 0;
    std::chrono::steady_clock::time_point benchmark_start_time_{};

    std::array<char, 1024> bios_path_input_{};
    std::string status_message_ = "PS2 experimental core ready";

    // VibeStation 2 frontend; the lab panels above are the developer view.
    std::unique_ptr<vs2::Frontend> frontend_{};
    bool developer_view_ = false;

    // Hosted inside VibeStation (shared window, own ImGui context).
    bool hosted_ = false;
    ImGuiContext* imgui_context_ = nullptr;
    bool vs1_switch_requested_ = false;
    // frame() loop state (formerly locals of run()).
    int run_result_ = 0;
    std::chrono::steady_clock::time_point frame_started_{};
};

} // namespace ps2::ui
