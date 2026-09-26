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

    static constexpr u32 kPageSize = 4096u;
    static constexpr u32 kPageCount =
        static_cast<u32>(kSize) / kPageSize;
    [[nodiscard]] u8* data() { return data_.data(); }
    [[nodiscard]] const u8* data() const { return data_.data(); }
    [[nodiscard]] u32 page_generation(u32 offset) const {
        return page_generations_[(offset & (kSize - 1u)) / kPageSize];
    }
    [[nodiscard]] u32* page_generation_data() {
        return page_generations_.data();
    }
    void track_code_page(u32 offset) {
        tracked_code_pages_[(offset & (kSize - 1u)) / kPageSize] = 1u;
    }

private:
    void note_write(u32 offset, std::size_t width);
    [[nodiscard]] bool contains(u32 offset, std::size_t width) const;

    std::vector<u8> data_;
    std::vector<u32> page_generations_;
    std::vector<u8> tracked_code_pages_;
};

} // namespace ps2
