#include "ps1/ps1_mode.h"

#include "core/system.h"
#include "platform/disc_path_utils.h"
#include "ui/emu_runner.h"

#include <filesystem>

namespace ps2::ui {

struct Ps1Mode::Impl {
    std::unique_ptr<System> system;
    EmuRunner runner;
    std::uint64_t last_frame = 0;
    bool running = false;
};

Ps1Mode::Ps1Mode() : impl_(std::make_unique<Impl>()) {}
Ps1Mode::~Ps1Mode() { stop(); }

bool Ps1Mode::start(
    const std::string& bios_path,
    const std::string& disc_path,
    std::string& error) {
    stop();

    std::filesystem::path cue = disc_path;
    std::string bin = disc_path;
    if (cue.extension() == ".cue" || cue.extension() == ".CUE") {
        bin = resolve_first_bin_from_cue(cue);
        if (bin.empty()) {
            error = "No data track found in " + disc_path;
            return false;
        }
    } else {
        std::filesystem::path sibling = cue;
        sibling.replace_extension(".cue");
        cue = std::filesystem::exists(sibling) ? sibling : std::filesystem::path();
    }

    impl_->system = std::make_unique<System>();
    System& sys = *impl_->system;
    if (!sys.load_bios(bios_path)) {
        error = "PS1 BIOS load failed: " + bios_path;
        impl_->system.reset();
        return false;
    }
    if (!sys.load_game(bin, cue.string())) {
        error = "PS1 disc load failed: " + disc_path;
        impl_->system.reset();
        return false;
    }
    if (!sys.boot_disc()) {
        error = "PS1 boot failed (check BIOS/disc)";
        impl_->system.reset();
        return false;
    }
    impl_->last_frame = 0;
    if (!impl_->runner.start(&sys)) {
        error = "PS1 runner failed to start";
        impl_->system.reset();
        return false;
    }
    impl_->runner.set_running(true);
    impl_->running = true;
    return true;
}

void Ps1Mode::stop() {
    if (!impl_->running) return;
    impl_->runner.set_running(false);
    impl_->runner.stop();
    impl_->system->shutdown();
    impl_->system.reset();
    impl_->running = false;
}

bool Ps1Mode::active() const { return impl_->running; }

void Ps1Mode::set_buttons(std::uint16_t buttons) {
    if (impl_->running) impl_->runner.set_input_state(buttons, 128, 128, 128, 128);
}

void Ps1Mode::set_speed(double speed) {
    if (impl_->running) impl_->runner.set_speed(speed);
}

bool Ps1Mode::take_frame(
    std::vector<std::uint32_t>& rgba, int& width, int& height) {
    if (!impl_->running) return false;
    FrameSnapshot frame;
    if (!impl_->runner.consume_latest_frame(frame) ||
        frame.frame_id == impl_->last_frame) {
        return false;
    }
    impl_->last_frame = frame.frame_id;
    width = frame.width;
    height = frame.height;
    rgba.assign(frame.rgba.begin(), frame.rgba.end());
    return true;
}

} // namespace ps2::ui
