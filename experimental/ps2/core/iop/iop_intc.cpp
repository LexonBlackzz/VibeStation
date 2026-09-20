#include "core/iop/iop_intc.h"

#include <initializer_list>

namespace ps2 {

void IopIntc::reset() {
    istat_ = 0;
    imask_ = 0;
    ictrl_ = 0;
}

bool IopIntc::decode(
    u32 physical,
    u32& base,
    u32& byte_offset) {
    for (const u32 candidate : {kIStat, kIMask, kICtrl}) {
        if (physical >= candidate && physical < candidate + 4u) {
            base = candidate;
            byte_offset = physical - candidate;
            return true;
        }
    }
    return false;
}

u32 IopIntc::read_register(u32 base) const {
    switch (base) {
    case kIStat:
        return istat_;
    case kIMask:
        return imask_;
    case kICtrl: {
        const u32 value = ictrl_;
        // PCSX2 and retail software treat I_CTRL as read-to-clear.
        ictrl_ = 0;
        return value;
    }
    default:
        return 0;
    }
}

bool IopIntc::read8(u32 physical, u8& value) const {
    u32 base = 0;
    u32 byte_offset = 0;
    if (!decode(physical, base, byte_offset)) {
        return false;
    }

    const u32 reg = read_register(base);
    value = static_cast<u8>(reg >> (byte_offset * 8));
    return true;
}

bool IopIntc::read16(u32 physical, u16& value) const {
    u32 base = 0;
    u32 byte_offset = 0;
    if (!decode(physical, base, byte_offset) ||
        byte_offset > 2u) {
        return false;
    }

    const u32 reg = read_register(base);
    value = static_cast<u16>(reg >> (byte_offset * 8));
    return true;
}

bool IopIntc::read32(u32 physical, u32& value) const {
    u32 base = 0;
    u32 byte_offset = 0;
    if (!decode(physical, base, byte_offset) ||
        byte_offset != 0) {
        return false;
    }

    value = read_register(base);
    return true;
}

void IopIntc::write_partial(
    u32 base,
    u32 byte_offset,
    u32 width,
    u32 value) {
    const u32 bits = width * 8u;
    const u32 lane_mask =
        bits == 32u
            ? 0xFFFFFFFFu
            : ((1u << bits) - 1u) << (byte_offset * 8u);
    const u32 shifted =
        (value << (byte_offset * 8u)) & lane_mask;

    switch (base) {
    case kIStat:
        // I_STAT acknowledges by ANDing the written value into the
        // selected lanes. Zeros clear pending sources; ones keep them.
        istat_ &=
            (~lane_mask) | shifted;
        break;
    case kIMask:
        imask_ =
            (imask_ & ~lane_mask) | shifted;
        break;
    case kICtrl:
        ictrl_ =
            (ictrl_ & ~lane_mask) | shifted;
        break;
    default:
        break;
    }
}

bool IopIntc::write8(u32 physical, u8 value) {
    u32 base = 0;
    u32 byte_offset = 0;
    if (!decode(physical, base, byte_offset)) {
        return false;
    }

    write_partial(base, byte_offset, 1, value);
    return true;
}

bool IopIntc::write16(u32 physical, u16 value) {
    u32 base = 0;
    u32 byte_offset = 0;
    if (!decode(physical, base, byte_offset) ||
        byte_offset > 2u) {
        return false;
    }

    write_partial(base, byte_offset, 2, value);
    return true;
}

bool IopIntc::write32(u32 physical, u32 value) {
    u32 base = 0;
    u32 byte_offset = 0;
    if (!decode(physical, base, byte_offset) ||
        byte_offset != 0) {
        return false;
    }

    write_partial(base, 0, 4, value);
    return true;
}

void IopIntc::raise(u32 irq) {
    if (irq < 32u) {
        istat_ |= 1u << irq;
    }
}

} // namespace ps2
