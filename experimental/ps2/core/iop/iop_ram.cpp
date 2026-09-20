#include "core/iop/iop_ram.h"

#include <algorithm>

namespace ps2 {

IopRam::IopRam()
    : data_(kSize, 0) {}

void IopRam::reset() {
    std::fill(data_.begin(), data_.end(), 0);
}

bool IopRam::contains(u32 offset, std::size_t width) const {
    const auto start = static_cast<std::size_t>(offset);
    return start <= kSize && width <= (kSize - start);
}

bool IopRam::read8(u32 offset, u8& value) const {
    if (!contains(offset, 1)) return false;
    value = data_[offset];
    return true;
}

bool IopRam::read16(u32 offset, u16& value) const {
    if (!contains(offset, 2)) return false;
    value = static_cast<u16>(data_[offset]) |
            (static_cast<u16>(data_[offset + 1]) << 8);
    return true;
}

bool IopRam::read32(u32 offset, u32& value) const {
    if (!contains(offset, 4)) return false;
    value = static_cast<u32>(data_[offset]) |
            (static_cast<u32>(data_[offset + 1]) << 8) |
            (static_cast<u32>(data_[offset + 2]) << 16) |
            (static_cast<u32>(data_[offset + 3]) << 24);
    return true;
}

bool IopRam::read64(u32 offset, u64& value) const {
    if (!contains(offset, 8)) return false;
    value = 0;
    for (u32 i = 0; i < 8; ++i) {
        value |= static_cast<u64>(data_[offset + i]) << (i * 8);
    }
    return true;
}

bool IopRam::write8(u32 offset, u8 value) {
    if (!contains(offset, 1)) return false;
    data_[offset] = value;
    return true;
}

bool IopRam::write16(u32 offset, u16 value) {
    if (!contains(offset, 2)) return false;
    data_[offset] = static_cast<u8>(value);
    data_[offset + 1] = static_cast<u8>(value >> 8);
    return true;
}

bool IopRam::write32(u32 offset, u32 value) {
    if (!contains(offset, 4)) return false;
    for (u32 i = 0; i < 4; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
    return true;
}

bool IopRam::write64(u32 offset, u64 value) {
    if (!contains(offset, 8)) return false;
    for (u32 i = 0; i < 8; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
    return true;
}

} // namespace ps2
