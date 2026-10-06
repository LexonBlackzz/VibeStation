#pragma once

#include "common/types.h"

#include <cstddef>
#include <vector>

namespace ps2 {

class IopRam {
public:
    static constexpr std::size_t kSize = 2u * 1024u * 1024u;
    static constexpr u32 kEePhysicalBase = 0x1C000000u;

    IopRam();

    void reset();

    [[nodiscard]] bool read8(u32 offset, u8& value) const;
    [[nodiscard]] bool read16(u32 offset, u16& value) const;
    [[nodiscard]] bool read32(u32 offset, u32& value) const;
    [[nodiscard]] bool read64(u32 offset, u64& value) const;

    [[nodiscard]] bool write8(u32 offset, u8 value);
    [[nodiscard]] bool write16(u32 offset, u16 value);
    [[nodiscard]] bool write32(u32 offset, u32 value);
    [[nodiscard]] bool write64(u32 offset, u64 value);

private:
    [[nodiscard]] bool contains(u32 offset, std::size_t width) const;

    std::vector<u8> data_;
};

} // namespace ps2
