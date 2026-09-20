#include "core/ps2_system.h"

namespace ps2 {

Ps2System::Ps2System()
    : bus_(ram_, scratchpad_, hw_, bios_),
      ee_(bus_) {
    reset();
}

void Ps2System::reset(u32 entry_point) {
    ram_.reset();
    scratchpad_.reset();
    hw_.reset();
    scheduler_.reset();
    ee_.reset(entry_point);
    bios_started_ = false;
    reset_instruction_ = 0;
}

bool Ps2System::load_bios(const std::string& path, std::string& error) {
    if (!bios_.load_file(path, error)) {
        return false;
    }
    reset();
    return true;
}

bool Ps2System::boot_bios(std::string& error) {
    error.clear();
    if (!bios_.loaded()) {
        error = "No PS2 BIOS is loaded.";
        return false;
    }

    reset(Bios::kResetVector);
    if (!bus_.read32(ee_.state().pc, reset_instruction_)) {
        error = "BIOS loaded, but the EE reset vector could not be fetched.";
        reset();
        return false;
    }

    bios_started_ = true;
    return true;
}

bool Ps2System::step_ee(std::string& error) {
    if (!bios_started_) {
        error = "BIOS has not been started.";
        return false;
    }
    return ee_.step(error);
}

u64 Ps2System::run_ee(
    u64 instruction_budget,
    std::string& error) {
    if (!bios_started_) {
        error = "BIOS has not been started.";
        return 0;
    }
    return ee_.run(instruction_budget, error);
}

} // namespace ps2
