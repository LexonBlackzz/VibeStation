#include "core/ps2_system.h"

namespace ps2 {

Ps2System::Ps2System()
    : bus_(ram_),
      ee_(bus_) {
    reset();
}

void Ps2System::reset(u32 entry_point) {
    ram_.reset();
    scheduler_.reset();
    ee_.reset(entry_point);
}

} // namespace ps2
