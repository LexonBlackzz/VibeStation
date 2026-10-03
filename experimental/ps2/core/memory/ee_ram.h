#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>
#include <span>
#include <vector>

namespace ps2 {

class EeRam {
public:
    static constexpr std::size_t kSize = 32u * 1024u * 1024u;
    static constexpr u32 kPageSize = 4096u;
    static constexpr u32 kPageCount =
        static_cast<u32>(kSize / kPageSize);

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
    [[nodiscard]] u8* data() { return data_.data(); }
    [[nodiscard]] const u8* data() const { return data_.data(); }
    [[nodiscard]] u32* page_generation_data() {
        return page_generation_.data();
    }
    [[nodiscard]] const u32* page_generation_data() const {
        return page_generation_.data();
    }
    [[nodiscard]] u8* code_page_tracked_data() {
        return code_page_tracked_.data();
    }
    [[nodiscard]] const u8* code_page_tracked_data() const {
        return code_page_tracked_.data();
    }
    [[nodiscard]] u32 page_generation(u32 offset) const {
        return page_generation_[offset / kPageSize];
    }
    void track_code_page(u32 offset) {
        code_page_tracked_[offset / kPageSize] = 1u;
    }

private:
    [[nodiscard]] bool contains(u32 offset, std::size_t width) const;
    void mark_written(u32 offset, std::size_t width);

    std::vector<u8> data_;
    std::array<u32, kPageCount> page_generation_{};
    std::array<u8, kPageCount> code_page_tracked_{};
};

} // namespace ps2
