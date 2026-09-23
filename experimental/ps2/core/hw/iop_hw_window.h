#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>

namespace ps2 {

class IopIntc;

class IopHwWindow {
public:
    static constexpr u32 kBase = 0x1F800000u;
    static constexpr std::size_t kSize = 64u * 1024u;

    void reset();
    void tick(u64 cycles, IopIntc& intc);
    [[nodiscard]] bool contains(u32 address, std::size_t width) const;

    [[nodiscard]] bool read8(u32 address, u8& value) const;
    [[nodiscard]] bool read16(u32 address, u16& value) const;
    [[nodiscard]] bool read32(u32 address, u32& value) const;
    [[nodiscard]] bool read64(u32 address, u64& value) const;
    [[nodiscard]] u16 sif_dma_ready_mask() const {
        // CHCR.START is bit 24. Align IOP DMA9/10 with EE SIF0/1 bits.
        return static_cast<u16>(
            ((data_[0x152Bu] & 1u) << 5) |
            ((data_[0x153Bu] & 1u) << 6));
    }

    [[nodiscard]] bool write8(u32 address, u8 value);
    [[nodiscard]] bool write16(u32 address, u16 value);
    [[nodiscard]] bool write32(u32 address, u32 value);
    [[nodiscard]] bool write64(u32 address, u64 value);

private:
    [[nodiscard]] static bool decode_timer(
        u32 address,
        u32& index,
        u32& reg,
        u32& byte_offset);
    [[nodiscard]] u32 timer_value(u32 index, u32 reg) const;
    void write_timer(u32 index, u32 reg, u32 value);
    void write_timer_partial(
        u32 index,
        u32 reg,
        u32 byte_offset,
        u32 width,
        u32 value);
    [[nodiscard]] u32 timer_rate(u32 index) const;
    void fire_timer_irq(IopIntc& intc, u32 index, bool overflow);

    std::array<u8, kSize> data_{};
    std::array<u64, 6> timer_phase_{};
    std::array<u32, 6> timer_count_{};
    std::array<u32, 6> timer_mode_{};
    std::array<u32, 6> timer_target_{};
    std::array<u32, 6> timer_rate_cache_{};
};

} // namespace ps2
