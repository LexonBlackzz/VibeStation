#pragma once

#include "core/bios/bios.h"
#include "core/ee/ee_cpu.h"
#include "core/gs/gs_privileged.h"
#include "core/hw/ee_hw.h"
#include "core/hw/iop_hw_window.h"
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
    u64 run_ee(u64 instruction_budget, std::string& error);

    [[nodiscard]] Bios& bios() { return bios_; }
    [[nodiscard]] const Bios& bios() const { return bios_; }

    [[nodiscard]] EeRam& ram() { return ram_; }
    [[nodiscard]] const EeRam& ram() const { return ram_; }

    [[nodiscard]] EeBus& bus() { return bus_; }
    [[nodiscard]] const EeBus& bus() const { return bus_; }

    [[nodiscard]] Scheduler& scheduler() { return scheduler_; }
    [[nodiscard]] const Scheduler& scheduler() const { return scheduler_; }

    [[nodiscard]] EeCpu& ee() { return ee_; }
    [[nodiscard]] const EeCpu& ee() const { return ee_; }

    [[nodiscard]] bool bios_started() const { return bios_started_; }
    [[nodiscard]] u32 reset_instruction() const { return reset_instruction_; }

private:
    Bios bios_{};
    EeRam ram_{};
    EeScratchpad scratchpad_{};
    EeHw hw_{};
    IopHwWindow iop_hw_{};
    GsPrivileged gs_{};
    EeBus bus_;
    Scheduler scheduler_{};
    EeCpu ee_;
    bool bios_started_ = false;
    u32 reset_instruction_ = 0;
};

} // namespace ps2
