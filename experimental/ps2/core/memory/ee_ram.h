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

    // JIT coherency metadata packs a 24-bit generation counter and an
    // 8-region translated-code mask into one 32-bit word per 4 KiB page.
    // 512-byte regions are deliberately conservative, but much finer than
    // invalidating on every write anywhere in a translated 4 KiB page.
    static constexpr u32 kCodeRegionShift = 9u;
    static constexpr u32 kCodeRegionSize = 1u << kCodeRegionShift;
    static constexpr u32 kCodeRegionCount = kPageSize / kCodeRegionSize;
    static constexpr u32 kGenerationMask = 0x00FFFFFFu;
    static constexpr u32 kCodeMaskShift = 24u;
    static constexpr u32 kCodeMask = 0xFF000000u;

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
    [[nodiscard]] u32 page_generation(u32 offset) const {
        return page_generation_[offset / kPageSize] & kGenerationMask;
    }
    void track_code_range(u32 offset, std::size_t width);
    // Compatibility/debug helper: explicitly mark the whole 4 KiB page.
    // Production JIT paths use track_code_range() for finer invalidation.
    void track_code_page(u32 offset) {
        track_code_range(
            offset & ~(kPageSize - 1u),
            kPageSize);
    }
    static void track_jit_code(
        u32* page_metadata,
        u32 offset,
        std::size_t width);

    // Native direct stores bypass EeRam::write*. Keep their coherency update
    // identical to mark_written() without forcing a C++ call from generated
    // code. The JIT consumes page_generation_data() as packed metadata.
    static void mark_jit_written(
        u32* page_metadata,
        u32 offset,
        std::size_t width);

    [[nodiscard]] static u32 generation_from_metadata(u32 metadata) {
        return metadata & kGenerationMask;
    }

private:
    [[nodiscard]] bool contains(u32 offset, std::size_t width) const;
    void mark_written(u32 offset, std::size_t width);

    std::vector<u8> data_;
    std::array<u32, kPageCount> page_generation_{};
};

} // namespace ps2
