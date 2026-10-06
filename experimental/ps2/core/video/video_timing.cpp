#include "core/video/video_timing.h"

#include "core/hw/ee_hw.h"
#include "core/iop/iop_intc.h"

namespace ps2 {

void VideoTiming::reset() {
    phase_ = Phase::Render;
    cycles_to_transition_ = kNtscRenderCycles;
    fields_started_ = 0;
}

void VideoTiming::tick(u64 ee_cycles, EeHw& ee_hw, IopIntc& iop_intc) {
    while (ee_cycles != 0) {
        if (ee_cycles < cycles_to_transition_) {
            cycles_to_transition_ -= ee_cycles;
            return;
        }

        ee_cycles -= cycles_to_transition_;

        if (phase_ == Phase::Render) {
            // EE INTC VBLANK start = 2.  The IOP receives its matching
            // VBLANK interrupt on I_STAT bit 0.
            ee_hw.raise_intc(2);
            iop_intc.raise(0);
            phase_ = Phase::VBlank;
            cycles_to_transition_ = kNtscVBlankCycles;
            ++fields_started_;
        } else {
            // EE INTC VBLANK end = 3.  The IOP exposes this as EVBLANK on
            // I_STAT bit 11.
            ee_hw.raise_intc(3);
            iop_intc.raise(11);
            phase_ = Phase::Render;
            cycles_to_transition_ = kNtscRenderCycles;
        }
    }
}

} // namespace ps2
