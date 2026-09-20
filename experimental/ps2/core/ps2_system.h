#pragma once

#include "core/bios/bios.h"
#include "core/cdvd/cdvd_hw.h"
#include "core/ee/ee_cpu.h"
#include "core/gs/gs_privileged.h"
#include "core/hw/ee_hw.h"
#include "core/hw/iop_hw_window.h"
#include "core/iop/iop_bus.h"
#include "core/iop/iop_cpu.h"
#include "core/iop/iop_ram.h"
#include "core/memory/ee_bus.h"
#include "core/memory/ee_ram.h"
#include "core/memory/ee_scratchpad.h"
#include "core/scheduler/scheduler.h"

#include <string>

namespace ps2 {

class Ps2System {
public:
    Ps2System();

    void reset(u32 entry_point = 0);
    bool load_bios(const std::string& path, std::string& error);
    bool boot_bios(std::string& error);
    bool step_ee(std::string& error);
    bool step_iop(std::string& error);
    u64 run_ee(u64 instruction_budget, std::string& error);

    [[nodiscard]] Bios& bios() { return bios_; }
    [[nodiscard]] const Bios& bios() const { return bios_; }

    [[nodiscard]] EeRam& ram() { return ram_; }
    [[nodiscard]] const EeRam& ram() const { return ram_; }

    [[nodiscard]] IopRam& iop_ram() { return iop_ram_; }
    [[nodiscard]] const IopRam& iop_ram() const { return iop_ram_; }

    [[nodiscard]] EeBus& bus() { return bus_; }
    [[nodiscard]] const EeBus& bus() const { return bus_; }

    [[nodiscard]] IopBus& iop_bus() { return iop_bus_; }
    [[nodiscard]] const IopBus& iop_bus() const { return iop_bus_; }

    [[nodiscard]] Scheduler& scheduler() { return scheduler_; }
    [[nodiscard]] const Scheduler& scheduler() const { return scheduler_; }

    [[nodiscard]] EeCpu& ee() { return ee_; }
    [[nodiscard]] const EeCpu& ee() const { return ee_; }

    [[nodiscard]] IopCpu& iop() { return iop_; }
    [[nodiscard]] const IopCpu& iop() const { return iop_; }

    [[nodiscard]] bool bios_started() const { return bios_started_; }
    [[nodiscard]] bool halted() const {
        return ee_.halted() || iop_.halted();
    }
    [[nodiscard]] std::string halt_reason() const;

    [[nodiscard]] u32 reset_instruction() const {
        return reset_instruction_;
    }
    [[nodiscard]] u32 iop_reset_instruction() const {
        return iop_reset_instruction_;
    }

private:
    bool advance_iop_for_ee_step(std::string& error);

    Bios bios_{};
    CdvdHw cdvd_{};
    EeRam ram_{};
    EeScratchpad scratchpad_{};
    EeHw hw_{};
    IopHwWindow iop_hw_{};
    IopRam iop_ram_{};
    GsPrivileged gs_{};
    IopBus iop_bus_;
    EeBus bus_;
    Scheduler scheduler_{};
    EeCpu ee_;
    IopCpu iop_;

    bool bios_started_ = false;
    u32 reset_instruction_ = 0;
    u32 iop_reset_instruction_ = 0;
    u32 ee_iop_phase_ = 0;
};

} // namespace ps2
