#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ps2::ui {

// Runs a PS1 disc on the standalone VibeStation PS1 core (src/core) instead of
// the PS2 BIOS's PS1 compatibility mode. Everything PS1-specific stays behind
// this header so the PS2 code never sees the PS1 core's global names.
class Ps1Mode {
public:
    Ps1Mode();
    ~Ps1Mode();

    // `disc_path` is a .cue or .bin. Starts emulating immediately.
    bool start(
        const std::string& bios_path,
        const std::string& disc_path,
        std::string& error);
    void stop();
    [[nodiscard]] bool active() const;

    // Digital pad, PS1 bit layout (0 = pressed), same as the PS2 pad's.
    void set_buttons(std::uint16_t buttons);
    // 0 = unlimited, otherwise a multiple of real speed (0.25-4).
    void set_speed(double speed);
    // Newest unseen frame; false when nothing new has been produced.
    bool take_frame(std::vector<std::uint32_t>& rgba, int& width, int& height);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ps2::ui
