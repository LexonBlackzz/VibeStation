#pragma once

#include "common/types.h"

#include <array>

namespace ps2 {

class EeBus;

struct EeGpr {
    u64 lo = 0;
    u64 hi = 0;
};

struct EeCpuState {
    std::array<EeGpr, 32> gpr{};
    u64 hi = 0;
    u64 lo = 0;
    u32 pc = 0;
    u32 next_pc = 4;
};

class EeCpu {
public:
    explicit EeCpu(EeBus& bus) : bus_(bus) {}

    void reset(u32 entry_point = 0);

    [[nodiscard]] const EeCpuState& state() const { return state_; }
    [[nodiscard]] EeCpuState& state() { return state_; }

private:
    EeBus& bus_;
    EeCpuState state_{};
};

} // namespace ps2
