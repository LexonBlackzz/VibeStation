#pragma once

#include "common/types.h"

namespace ps2 {

class EeHw;
class IopIntc;

class VideoTiming {
public:
    enum class Phase {
        Render,
        VBlank,
    };

    // Early bootstrap uses the console's NTSC timing until GS mode registers
    // provide a better source.  These are EE-cycle approximations for a
    // 59.94 Hz field cadence (29.97 Hz interlaced frame cadence).
    static constexpr u64 kNtscFieldCycles = 4'920'120u;
    static constexpr u64 kNtscVBlankCycles = 421'725u;
    static constexpr u64 kNtscRenderCycles =
        kNtscFieldCycles - kNtscVBlankCycles;

    void reset();
    void tick(u64 ee_cycles, EeHw& ee_hw, IopIntc& iop_intc);

    [[nodiscard]] Phase phase() const { return phase_; }
    [[nodiscard]] u64 cycles_to_transition() const { return cycles_to_transition_; }
    [[nodiscard]] u64 fields_started() const { return fields_started_; }

private:
    Phase phase_ = Phase::Render;
    u64 cycles_to_transition_ = kNtscRenderCycles;
    u64 fields_started_ = 0;
};

} // namespace ps2
