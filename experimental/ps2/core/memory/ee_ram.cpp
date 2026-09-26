#include "core/memory/ee_ram.h"

#include <algorithm>
#include <bit>
#include <cstring>

namespace ps2 {

EeRam::EeRam()
    : data_(kSize, 0) {}

void EeRam::reset() {
    std::fill(data_.begin(), data_.end(), u8{0});
    page_generation_.fill(0);
}

namespace {

u32 code_region_mask_for_span(
    u32 page_offset,
    std::size_t width) {
    if (width == 0u) return 0u;
    const u32 first_region =
        page_offset >> EeRam::kCodeRegionShift;
    const u32 last_byte = static_cast<u32>(
        std::min<std::size_t>(
            EeRam::kPageSize - 1u,
            static_cast<std::size_t>(page_offset) + width - 1u));
    const u32 last_region =
        last_byte >> EeRam::kCodeRegionShift;
    u32 mask = 0u;
    for (u32 region = first_region;
         region <= last_region;
         ++region) {
        mask |= 1u << region;
    }
    return mask << EeRam::kCodeMaskShift;
}

void bump_generation_if_tracked(
    u32& metadata,
    u32 page_offset,
    std::size_t width) {
    const u32 touched =
        code_region_mask_for_span(page_offset, width);
    if ((metadata & touched) == 0u) return;
    const u32 next =
        ((metadata & EeRam::kGenerationMask) + 1u) &
        EeRam::kGenerationMask;
    metadata =
        (metadata & EeRam::kCodeMask) | next;
}

} // namespace

void EeRam::track_code_range(
    u32 offset,
    std::size_t width) {
    if (width == 0u || offset >= kSize) return;
    std::size_t remaining =
        std::min<std::size_t>(width, kSize - offset);
    u32 cursor = offset;
    while (remaining != 0u) {
        const u32 page = cursor / kPageSize;
        const u32 page_offset = cursor & (kPageSize - 1u);
        const std::size_t chunk =
            std::min<std::size_t>(
                remaining, kPageSize - page_offset);
        page_generation_[page] |=
            code_region_mask_for_span(page_offset, chunk);
        cursor += static_cast<u32>(chunk);
        remaining -= chunk;
    }
}

void EeRam::mark_jit_written(
    u32* page_metadata,
    u32 offset,
    std::size_t width) {
    if (page_metadata == nullptr ||
        width == 0u ||
        offset >= kSize) {
        return;
    }
    std::size_t remaining =
        std::min<std::size_t>(width, kSize - offset);
    u32 cursor = offset;
    while (remaining != 0u) {
        const u32 page = cursor / kPageSize;
        const u32 page_offset = cursor & (kPageSize - 1u);
        const std::size_t chunk =
            std::min<std::size_t>(
                remaining, kPageSize - page_offset);
        bump_generation_if_tracked(
            page_metadata[page], page_offset, chunk);
        cursor += static_cast<u32>(chunk);
        remaining -= chunk;
    }
}

void EeRam::mark_written(u32 offset, std::size_t width) {
    mark_jit_written(page_generation_.data(), offset, width);
}

bool EeRam::contains(u32 offset, std::size_t width) const {
    const auto start = static_cast<std::size_t>(offset);
    return start <= kSize && width <= (kSize - start);
}

bool EeRam::read8(u32 offset, u8& value) const {
    if (!contains(offset, 1)) {
        return false;
    }
    value = data_[offset];
    return true;
}

bool EeRam::read16(u32 offset, u16& value) const {
    if (!contains(offset, 2)) {
        return false;
    }
    if constexpr (std::endian::native == std::endian::little) {
        std::memcpy(&value, data_.data() + offset, sizeof(value));
    } else {
        value = static_cast<u16>(data_[offset]) |
                (static_cast<u16>(data_[offset + 1]) << 8);
    }
    return true;
}

bool EeRam::read32(u32 offset, u32& value) const {
    if (!contains(offset, 4)) {
        return false;
    }
    if constexpr (std::endian::native == std::endian::little) {
        std::memcpy(&value, data_.data() + offset, sizeof(value));
    } else {
        value = static_cast<u32>(data_[offset]) |
                (static_cast<u32>(data_[offset + 1]) << 8) |
                (static_cast<u32>(data_[offset + 2]) << 16) |
                (static_cast<u32>(data_[offset + 3]) << 24);
    }
    return true;
}

bool EeRam::read64(u32 offset, u64& value) const {
    if (!contains(offset, 8)) {
        return false;
    }

    if constexpr (std::endian::native == std::endian::little) {
        std::memcpy(&value, data_.data() + offset, sizeof(value));
    } else {
        value = 0;
        for (u32 i = 0; i < 8; ++i) {
            value |= static_cast<u64>(data_[offset + i]) << (i * 8);
        }
    }
    return true;
}

bool EeRam::write8(u32 offset, u8 value) {
    if (!contains(offset, 1)) {
        return false;
    }
    data_[offset] = value;
    mark_written(offset, 1u);
    return true;
}

bool EeRam::write16(u32 offset, u16 value) {
    if (!contains(offset, 2)) {
        return false;
    }
    if constexpr (std::endian::native == std::endian::little) {
        std::memcpy(data_.data() + offset, &value, sizeof(value));
    } else {
        for (u32 i = 0; i < 2; ++i) {
            data_[offset + i] = static_cast<u8>(value >> (i * 8));
        }
    }
    mark_written(offset, 2u);
    return true;
}

bool EeRam::write32(u32 offset, u32 value) {
    if (!contains(offset, 4)) {
        return false;
    }
    if constexpr (std::endian::native == std::endian::little) {
        std::memcpy(data_.data() + offset, &value, sizeof(value));
    } else {
        for (u32 i = 0; i < 4; ++i) {
            data_[offset + i] = static_cast<u8>(value >> (i * 8));
        }
    }
    mark_written(offset, 4u);
    return true;
}

bool EeRam::write64(u32 offset, u64 value) {
    if (!contains(offset, 8)) {
        return false;
    }
    if constexpr (std::endian::native == std::endian::little) {
        std::memcpy(data_.data() + offset, &value, sizeof(value));
    } else {
        for (u32 i = 0; i < 8; ++i) {
            data_[offset + i] = static_cast<u8>(value >> (i * 8));
        }
    }
    mark_written(offset, 8u);
    return true;
}

bool EeRam::fill_zero(u32 offset, std::size_t length) {
    if (!contains(offset, length)) return false;
    std::memset(data_.data() + offset, 0, length);
    mark_written(offset, length);
    return true;
}

bool EeRam::nibble_swap(u32 offset, std::size_t length,
                        u8& last_original) {
    if (length == 0 || !contains(offset, length)) return false;
    last_original = data_[offset + length - 1];
    for (std::size_t i = 0; i < length; ++i) {
        const u8 byte = data_[offset + i];
        data_[offset + i] = static_cast<u8>((byte >> 4) | (byte << 4));
    }
    mark_written(offset, length);
    return true;
}

bool EeRam::copy_forward(u32 destination, u32 source,
                         std::size_t length, u8& last_value) {
    if (length == 0u || !contains(destination, length) ||
        !contains(source, length)) return false;
    for (std::size_t i = 0; i < length; ++i) {
        last_value = data_[source + i];
        data_[destination + i] = last_value;
    }
    mark_written(destination, length);
    return true;
}

bool EeRam::matches_words(u32 offset,
                          std::span<const u32> words) const {
    const std::size_t bytes = words.size_bytes();
    if (!contains(offset, bytes)) return false;
    if constexpr (std::endian::native == std::endian::little) {
        return std::memcmp(data_.data() + offset, words.data(), bytes) == 0;
    } else {
        for (std::size_t i = 0; i < words.size(); ++i) {
            u32 actual = 0;
            if (!read32(offset + static_cast<u32>(4u * i), actual) ||
                actual != words[i]) return false;
        }
        return true;
    }
}

} // namespace ps2
