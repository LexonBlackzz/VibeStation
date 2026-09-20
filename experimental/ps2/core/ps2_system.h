#pragma once

#include "core/ee/ee_cpu.h"
#include "core/memory/ee_bus.h"
#include "core/memory/ee_ram.h"
#include "core/scheduler/scheduler.h"

namespace ps2 {

class Ps2System {
public:
    Ps2System();

    void reset(u32 entry_point = 0);

    [[nodiscard]] EeRam& ram() { return ram_; }
    [[nodiscard]] const EeRam& ram() const { return ram_; }

    [[nodiscard]] EeBus& bus() { return bus_; }
    [[nodiscard]] const EeBus& bus() const { return bus_; }

    [[nodiscard]] Scheduler& scheduler() { return scheduler_; }
    [[nodiscard]] const Scheduler& scheduler() const { return scheduler_; }

    [[nodiscard]] EeCpu& ee() { return ee_; }
    [[nodiscard]] const EeCpu& ee() const { return ee_; }

private:
    EeRam ram_{};
    EeBus bus_;
    Scheduler scheduler_{};
    EeCpu ee_;
};

} // namespace ps2
