#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>

namespace ps2 {

class EeHw {
public:
    void reset();
    void tick(u64 cycles);

    [[nodiscard]] bool read8(u32 physical, u8& value) const;
    [[nodiscard]] bool read16(u32 physical, u16& value) const;
    [[nodiscard]] bool read32(u32 physical, u32& value) const;
    [[nodiscard]] bool read64(u32 physical, u64& value) const;

    [[nodiscard]] bool write8(u32 physical, u8 value);
    [[nodiscard]] bool write16(u32 physical, u16 value);
    [[nodiscard]] bool write32(u32 physical, u32 value);
    [[nodiscard]] bool write64(u32 physical, u64 value);

    [[nodiscard]] u64 cycles() const { return cycles_; }

private:
    static constexpr u32 kRegBase = 0x1000F000u;
    static constexpr std::size_t kRegSize = 0x600u;

    [[nodiscard]] bool in_reg_window(u32 address, std::size_t width) const;
    [[nodiscard]] u32 generic_read32(u32 address) const;
    void generic_write32(u32 address, u32 value);

    u64 cycles_ = 0;
    u64 timer0_epoch_ = 0;
    u32 timer0_count_base_ = 0;
    u32 timer0_mode_ = 0;

    std::array<u8, kRegSize> regs_{};
    u32 mch_ricm_ = 0;
    mutable u32 rdram_sdevid_ = 0;
};

} // namespace ps2
