#include "core/ps2_system.h"

namespace ps2 {

Ps2System::Ps2System()
    : iop_bus_(iop_ram_, iop_hw_, hw_, cdvd_, bios_),
      bus_(ram_, scratchpad_, hw_, iop_hw_, iop_ram_, gs_, bios_),
      ee_(bus_),
      iop_(iop_bus_) {
    reset();
}

void Ps2System::reset(u32 entry_point) {
    ram_.reset();
    scratchpad_.reset();
    hw_.reset();
    iop_hw_.reset();
    cdvd_.reset();
    iop_ram_.reset();
    iop_bus_.reset();
    gs_.reset();
    scheduler_.reset();

    ee_.reset(entry_point);
    iop_.reset(Bios::kResetVector);

    bios_started_ = false;
    reset_instruction_ = 0;
    iop_reset_instruction_ = 0;
    ee_iop_phase_ = 0;
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

    if (!iop_bus_.read32(iop_.state().pc, iop_reset_instruction_)) {
        error = "BIOS loaded, but the IOP reset vector could not be fetched.";
        reset();
        return false;
    }

    bios_started_ = true;
    return true;
}

bool Ps2System::advance_iop_for_ee_step(std::string& error) {
    ++ee_iop_phase_;
    if (ee_iop_phase_ < 8) {
        return true;
    }

    ee_iop_phase_ = 0;
    if (!iop_.step(error)) {
        error = "IOP halted: " + error;
        return false;
    }
    return true;
}

bool Ps2System::step_ee(std::string& error) {
    error.clear();

    if (!bios_started_) {
        error = "BIOS has not been started.";
        return false;
    }
    if (halted()) {
        error = halt_reason();
        return false;
    }

    if (!ee_.step(error)) {
        error = "EE halted: " + error;
        return false;
    }

    scheduler_.run_until(scheduler_.now() + 1, {});
    return advance_iop_for_ee_step(error);
}

bool Ps2System::step_iop(std::string& error) {
    error.clear();

    if (!bios_started_) {
        error = "BIOS has not been started.";
        return false;
    }
    if (iop_.halted()) {
        error = iop_.halt_reason();
        return false;
    }

    return iop_.step(error);
}

u64 Ps2System::run_ee(
    u64 instruction_budget,
    std::string& error) {
    error.clear();

    if (!bios_started_) {
        error = "BIOS has not been started.";
        return 0;
    }

    u64 executed = 0;
    while (executed < instruction_budget && !halted()) {
        const u64 before = ee_.state().instructions_executed;
        if (!step_ee(error)) {
            if (ee_.state().instructions_executed != before) {
                ++executed;
            }
            break;
        }
        ++executed;
    }

    return executed;
}

std::string Ps2System::halt_reason() const {
    if (ee_.halted()) {
        return "EE: " + ee_.halt_reason();
    }
    if (iop_.halted()) {
        return "IOP: " + iop_.halt_reason();
    }
    return {};
}

} // namespace ps2
