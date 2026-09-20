#include "core/ee/ee_cpu.h"

namespace ps2 {

void EeCpu::reset(u32 entry_point) {
    state_ = {};
    state_.pc = entry_point;
    state_.next_pc = entry_point + 4;
}

} // namespace ps2
