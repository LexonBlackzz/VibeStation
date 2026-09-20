#include "core/hw/ee_hw.h"

namespace ps2 {

void EeHw::reset() {
    cycles_ = 0;
    timer0_epoch_ = 0;
    timer0_count_base_ = 0;
    timer0_mode_ = 0;
    memory_ctrl_f500_ = 0;
}

void EeHw::tick(u64 cycles) {
    cycles_ += cycles;
}

bool EeHw::read32(u32 address, u32& value) const {
    switch (address) {
    case 0x10000000u: {
        const u64 elapsed =
            cycles_ >= timer0_epoch_ ? cycles_ - timer0_epoch_ : 0;
        const u32 delta =
            (timer0_mode_ & 0x80u) != 0
                ? static_cast<u32>(elapsed / 16u)
                : 0u;
        value = (timer0_count_base_ + delta) & 0xFFFFu;
        return true;
    }
    case 0x10000010u:
        value = timer0_mode_;
        return true;
    case 0x1000F500u:
        value = memory_ctrl_f500_;
        return true;
    default:
        return false;
    }
}

bool EeHw::write32(u32 address, u32 value) {
    switch (address) {
    case 0x10000000u:
        timer0_count_base_ = value & 0xFFFFu;
        timer0_epoch_ = cycles_;
        return true;
    case 0x10000010u:
        timer0_mode_ = value;
        timer0_count_base_ = 0;
        timer0_epoch_ = cycles_;
        return true;
    case 0x1000F500u:
        memory_ctrl_f500_ = value;
        return true;
    default:
        return false;
    }
}

} // namespace ps2
