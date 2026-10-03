#pragma once

#include <string>

namespace ps2::ui::vs2 {

enum class EeCore {
    Interpreter,
    Jit,
    Dynarec,
};

// What the frontend shows while a game runs: the newest emulator frame.
struct GameView {
    unsigned int texture = 0;
    int width = 0;
    int height = 0;
    bool ps1 = false;
};

// Everything the VibeStation 2 frontend needs from the emulator app. The
// frontend never touches the cores directly, so this is also the seam a
// shared PS1/PS2 frontend can be built on later.
//
// All calls happen on the UI thread while the app holds its core lock.
class Host {
public:
    virtual ~Host() = default;

    // BIOS
    [[nodiscard]] virtual bool bios_loaded() const = 0;
    // ROMVER string of the loaded BIOS (empty when unknown).
    [[nodiscard]] virtual std::string bios_romver() const = 0;
    [[nodiscard]] virtual std::string bios_path() const = 0;
    virtual bool load_bios(const std::string& path) = 0;
    // Native file picker; empty when cancelled.
    virtual std::string pick_bios_file() = 0;
    virtual std::string pick_folder(const char* title) = 0;

    // Session
    virtual bool start_bios_session() = 0;
    virtual bool boot_disc(const std::string& path) = 0;
    [[nodiscard]] virtual bool session_active() const = 0;
    [[nodiscard]] virtual bool session_running() const = 0;
    virtual void pause_session() = 0;
    virtual void resume_session() = 0;
    [[nodiscard]] virtual GameView game_view() const = 0;
    [[nodiscard]] virtual double speed_percent() const = 0;
    [[nodiscard]] virtual double frames_per_second() const = 0;
    [[nodiscard]] virtual std::string status_message() const = 0;
    // Runs flat out while held (the toolbar's fast-forward button).
    virtual void set_turbo(bool active) = 0;
    // Saves the current frame as a PNG under snapshots/; the result goes to
    // status_message().
    virtual bool save_snapshot() = 0;

    // Settings
    [[nodiscard]] virtual EeCore ee_core() const = 0;
    virtual void set_ee_core(EeCore core) = 0;
    [[nodiscard]] virtual bool gpu_gs_available() const = 0;
    [[nodiscard]] virtual bool gpu_gs_enabled() const = 0;
    virtual void set_gpu_gs_enabled(bool enabled) = 0;
    [[nodiscard]] virtual bool limit_speed() const = 0;
    virtual void set_limit_speed(bool enabled) = 0;
    [[nodiscard]] virtual int display_filter() const = 0;
    virtual void set_display_filter(int filter) = 0;
    [[nodiscard]] virtual bool lag_stutter() const = 0;
    virtual void set_lag_stutter(bool enabled) = 0;

    // App
    virtual void open_developer_view() = 0;
    virtual void request_quit() = 0;
    // Running inside VibeStation next to VibeStation 1: offer the way back.
    [[nodiscard]] virtual bool can_switch_to_vs1() const = 0;
    virtual void switch_to_vs1() = 0;
};

} // namespace ps2::ui::vs2
