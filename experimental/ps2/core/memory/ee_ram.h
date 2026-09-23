#pragma once

#include "common/types.h"

#include <cstddef>
#include <span>
#include <vector>

namespace ps2 {

class EeRam {
public:
    static constexpr std::size_t kSize = 32u * 1024u * 1024u;

    EeRam();

    void reset();

    [[nodiscard]] bool read8(u32 offset, u8& value) const;
    [[nodiscard]] bool read16(u32 offset, u16& value) const;
    [[nodiscard]] bool read32(u32 offset, u32& value) const;
    [[nodiscard]] bool read64(u32 offset, u64& value) const;

    [[nodiscard]] bool write8(u32 offset, u8 value);
    [[nodiscard]] bool write16(u32 offset, u16 value);
    [[nodiscard]] bool write32(u32 offset, u32 value);
    [[nodiscard]] bool write64(u32 offset, u64 value);
    [[nodiscard]] bool fill_zero(u32 offset, std::size_t length);
    [[nodiscard]] bool nibble_swap(u32 offset, std::size_t length,
                                   u8& last_original);
    [[nodiscard]] bool copy_forward(u32 destination, u32 source,
                                    std::size_t length, u8& last_value);
    [[nodiscard]] bool matches_words(u32 offset,
                                     std::span<const u32> words) const;

    [[nodiscard]] constexpr std::size_t size() const { return kSize; }

private:
    [[nodiscard]] bool contains(u32 offset, std::size_t width) const;

    std::vector<u8> data_;
};

} // namespace ps2
