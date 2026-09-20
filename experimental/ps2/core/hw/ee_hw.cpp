#include "core/hw/ee_hw.h"

namespace ps2 {
namespace {

constexpr u32 kMchRicm = 0x1000F430u;
constexpr u32 kMchDrd = 0x1000F440u;
constexpr u32 kDmacEnabler = 0x1000F520u;
constexpr u32 kDmacEnablew = 0x1000F590u;
constexpr u32 kSbusF240 = 0x1000F240u;
constexpr u32 kSbusF260 = 0x1000F260u;

} // namespace

void EeHw::reset() {
    cycles_ = 0;
    timer0_epoch_ = 0;
    timer0_count_base_ = 0;
    timer0_mode_ = 0;
    regs_.fill(0);
    mch_ricm_ = 0;
    rdram_sdevid_ = 0;

    // Reset values used by the retail BIOS during early hardware probing.
    generic_write32(kDmacEnabler, 0x1201u);
    generic_write32(kDmacEnablew, 0x1201u);
    generic_write32(kSbusF260, 0x1D000060u);
}

void EeHw::tick(u64 cycles) {
    cycles_ += cycles;
}

bool EeHw::in_reg_window(u32 address, std::size_t width) const {
    if (address < kRegBase) {
        return false;
    }
    const std::size_t offset = static_cast<std::size_t>(address - kRegBase);
    return offset <= kRegSize && width <= (kRegSize - offset);
}

u32 EeHw::generic_read32(u32 address) const {
    const u32 offset = address - kRegBase;
    return static_cast<u32>(regs_[offset]) |
           (static_cast<u32>(regs_[offset + 1]) << 8) |
           (static_cast<u32>(regs_[offset + 2]) << 16) |
           (static_cast<u32>(regs_[offset + 3]) << 24);
}

void EeHw::generic_write32(u32 address, u32 value) {
    const u32 offset = address - kRegBase;
    for (u32 i = 0; i < 4; ++i) {
        regs_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
}

bool EeHw::read8(u32 address, u8& value) const {
    if (!in_reg_window(address, 1)) {
        return false;
    }

    if (address == 0x1000F410u) {
        value = 0;
        return true;
    }

    value = regs_[address - kRegBase];
    return true;
}

bool EeHw::read16(u32 address, u16& value) const {
    if (!in_reg_window(address, 2)) {
        return false;
    }

    const u32 offset = address - kRegBase;
    value = static_cast<u16>(regs_[offset]) |
            (static_cast<u16>(regs_[offset + 1]) << 8);
    return true;
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
    default:
        break;
    }

    if (!in_reg_window(address, 4)) {
        return false;
    }

    if (address == 0x1000F410u) {
        value = 0;
        return true;
    }

    if (address == kMchRicm) {
        value = mch_ricm_;
        return true;
    }

    if (address == kMchDrd) {
        if (((mch_ricm_ >> 6) & 0xFu) == 0) {
            switch ((mch_ricm_ >> 16) & 0xFFFu) {
            case 0x21: // INIT
                if (rdram_sdevid_ < 2) {
                    ++rdram_sdevid_;
                    value = 0x1Fu;
                    return true;
                }
                value = 0;
                return true;
            case 0x23: // CNFGA
                value = 0x0D0Du;
                return true;
            case 0x24: // CNFGB
                value = 0x0090u;
                return true;
            case 0x40: // DEVID
                value = mch_ricm_ & 0x1Fu;
                return true;
            default:
                break;
            }
        }

        value = 0;
        return true;
    }

    if (address == kSbusF240) {
        value = generic_read32(address) | 0xF0000102u;
        return true;
    }

    value = generic_read32(address);
    return true;
}

bool EeHw::read64(u32 address, u64& value) const {
    u32 lo = 0;
    u32 hi = 0;
    if (!read32(address, lo) || !read32(address + 4, hi)) {
        return false;
    }

    value = static_cast<u64>(lo) | (static_cast<u64>(hi) << 32);
    return true;
}

bool EeHw::write8(u32 address, u8 value) {
    if (!in_reg_window(address, 1)) {
        return false;
    }
    regs_[address - kRegBase] = value;
    return true;
}

bool EeHw::write16(u32 address, u16 value) {
    if (!in_reg_window(address, 2)) {
        return false;
    }

    const u32 offset = address - kRegBase;
    regs_[offset] = static_cast<u8>(value);
    regs_[offset + 1] = static_cast<u8>(value >> 8);
    return true;
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
    default:
        break;
    }

    if (!in_reg_window(address, 4)) {
        return false;
    }

    if (address == kMchRicm) {
        if ((((value >> 16) & 0xFFFu) == 0x21u) &&
            (((value >> 6) & 0xFu) == 1u) &&
            ((generic_read32(kMchDrd) & 0x80u) == 0)) {
            rdram_sdevid_ = 0;
        }

        mch_ricm_ = value & ~0x80000000u;
        generic_write32(address, mch_ricm_);
        return true;
    }

    if (address == kDmacEnablew) {
        generic_write32(kDmacEnablew, value);
        generic_write32(kDmacEnabler, value);
        return true;
    }

    if (address == kSbusF240) {
        u32 old_value = generic_read32(address);
        if ((value & 0x100u) != 0) {
            old_value |= 0x100u;
        } else {
            old_value &= ~0x100u;
        }
        generic_write32(address, old_value);
        return true;
    }

    generic_write32(address, value);
    return true;
}

bool EeHw::write64(u32 address, u64 value) {
    return write32(address, static_cast<u32>(value)) &&
           write32(address + 4, static_cast<u32>(value >> 32));
}

} // namespace ps2
