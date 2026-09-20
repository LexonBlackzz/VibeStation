#include "core/hw/iop_hw_window.h"

namespace ps2 {

void IopHwWindow::reset() {
    data_.fill(0);
}

bool IopHwWindow::contains(u32 address, std::size_t width) const {
    if (address < kBase) {
        return false;
    }

    const std::size_t offset = static_cast<std::size_t>(address - kBase);
    return offset <= kSize && width <= (kSize - offset);
}

bool IopHwWindow::read8(u32 address, u8& value) const {
    if (!contains(address, 1)) {
        return false;
    }

    // Current PCSX2 behavior for these Page-3 IOP registers.
    if (address == 0x1F803100u) {
        value = 0;
        return true;
    }
    if (address == 0x1F803204u) {
        value = 0x7Cu;
        return true;
    }

    value = data_[address - kBase];
    return true;
}

bool IopHwWindow::read16(u32 address, u16& value) const {
    if (!contains(address, 2)) {
        return false;
    }

    u8 lo = 0;
    u8 hi = 0;
    read8(address, lo);
    read8(address + 1, hi);
    value = static_cast<u16>(lo) | (static_cast<u16>(hi) << 8);
    return true;
}

bool IopHwWindow::read32(u32 address, u32& value) const {
    if (!contains(address, 4)) {
        return false;
    }

    value = 0;
    for (u32 i = 0; i < 4; ++i) {
        u8 byte = 0;
        read8(address + i, byte);
        value |= static_cast<u32>(byte) << (i * 8);
    }
    return true;
}

bool IopHwWindow::read64(u32 address, u64& value) const {
    if (!contains(address, 8)) {
        return false;
    }

    value = 0;
    for (u32 i = 0; i < 8; ++i) {
        u8 byte = 0;
        read8(address + i, byte);
        value |= static_cast<u64>(byte) << (i * 8);
    }
    return true;
}

bool IopHwWindow::write8(u32 address, u8 value) {
    if (!contains(address, 1)) {
        return false;
    }
    data_[address - kBase] = value;
    return true;
}

bool IopHwWindow::write16(u32 address, u16 value) {
    if (!contains(address, 2)) {
        return false;
    }
    const u32 offset = address - kBase;
    for (u32 i = 0; i < 2; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
    return true;
}

bool IopHwWindow::write32(u32 address, u32 value) {
    if (!contains(address, 4)) {
        return false;
    }
    const u32 offset = address - kBase;
    for (u32 i = 0; i < 4; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
    return true;
}

bool IopHwWindow::write64(u32 address, u64 value) {
    if (!contains(address, 8)) {
        return false;
    }
    const u32 offset = address - kBase;
    for (u32 i = 0; i < 8; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
    return true;
}

} // namespace ps2
