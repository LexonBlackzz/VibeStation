#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>

namespace ps2 {

class EeScratchpad {
public:
    static constexpr u32 kBase = 0x70000000u;
    static constexpr std::size_t kSize = 16u * 1024u;

    void reset();
    [[nodiscard]] bool contains(u32 address, std::size_t width) const;

    [[nodiscard]] bool read8(u32 address, u8& value) const;
    [[nodiscard]] bool read16(u32 address, u16& value) const;
    [[nodiscard]] bool read32(u32 address, u32& value) const;
    [[nodiscard]] bool read64(u32 address, u64& value) const;

    [[nodiscard]] bool write8(u32 address, u8 value);
    [[nodiscard]] bool write16(u32 address, u16 value);
    [[nodiscard]] bool write32(u32 address, u32 value);
    [[nodiscard]] bool write64(u32 address, u64 value);

private:
    std::array<u8, kSize> data_{};
};

} // namespace ps2
