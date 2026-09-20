#include "core/gs/gs_privileged.h"

namespace ps2 {

void GsPrivileged::reset() {
    data_.fill(0);
}

bool GsPrivileged::contains(u32 address, std::size_t width) const {
    if (address < kBase) {
        return false;
    }
    const std::size_t offset = static_cast<std::size_t>(address - kBase);
    return offset <= kSize && width <= (kSize - offset);
}

bool GsPrivileged::read8(u32 address, u8& value) const {
    if (!contains(address, 1)) return false;
    value = data_[address - kBase];
    return true;
}

bool GsPrivileged::read16(u32 address, u16& value) const {
    if (!contains(address, 2)) return false;
    const u32 offset = address - kBase;
    value = static_cast<u16>(data_[offset]) |
            (static_cast<u16>(data_[offset + 1]) << 8);
    return true;
}

bool GsPrivileged::read32(u32 address, u32& value) const {
    if (!contains(address, 4)) return false;
    const u32 offset = address - kBase;
    value = static_cast<u32>(data_[offset]) |
            (static_cast<u32>(data_[offset + 1]) << 8) |
            (static_cast<u32>(data_[offset + 2]) << 16) |
            (static_cast<u32>(data_[offset + 3]) << 24);
    return true;
}

bool GsPrivileged::read64(u32 address, u64& value) const {
    if (!contains(address, 8)) return false;
    const u32 offset = address - kBase;
    value = 0;
    for (u32 i = 0; i < 8; ++i) {
        value |= static_cast<u64>(data_[offset + i]) << (i * 8);
    }
    return true;
}

bool GsPrivileged::write8(u32 address, u8 value) {
    if (!contains(address, 1)) return false;
    data_[address - kBase] = value;
    return true;
}

bool GsPrivileged::write16(u32 address, u16 value) {
    if (!contains(address, 2)) return false;
    const u32 offset = address - kBase;
    for (u32 i = 0; i < 2; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
    return true;
}

bool GsPrivileged::write32(u32 address, u32 value) {
    if (!contains(address, 4)) return false;
    const u32 offset = address - kBase;
    for (u32 i = 0; i < 4; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
    return true;
}

bool GsPrivileged::write64(u32 address, u64 value) {
    if (!contains(address, 8)) return false;
    const u32 offset = address - kBase;
    for (u32 i = 0; i < 8; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
    return true;
}

} // namespace ps2
