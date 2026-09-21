#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>

namespace ps2 {

class EeHw {
public:
    void reset();
    void tick(u64 cycles);
    void raise_intc(u32 irq);
    void raise_dmac(u32 channel);
    void update_vif1_stat(u32 set_bits, u32 clear_bits);
    [[nodiscard]] u32 vif1_stat() const;
    [[nodiscard]] bool intc_pending() const;
    [[nodiscard]] bool dmac_pending() const;

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
    std::array<u64, 4> timer_phase_{};
    std::array<u32, 4> timer_count_base_{};
    std::array<u32, 4> timer_mode_{};
    std::array<u32, 4> timer_comp_{};
    std::array<u32, 4> timer_hold_{};

    std::array<u8, kRegSize> regs_{};

    static constexpr u32 kDmacBase = 0x10008000u;
    static constexpr std::size_t kDmacSize = 0x6200u;
    std::array<u8, kDmacSize> dmac_regs_{};

    u32 ipu_cmd_ = 0;
    u32 ipu_ctrl_ = 0;
    u32 ipu_bp_ = 0;
    u32 ipu_top_ = 0;
    std::array<u64, 2> ipu_in_fifo_{};
    std::array<u64, 2> ipu_out_fifo_{};

    std::array<u8, 0x400> vif0_regs_{};
    std::array<u8, 0x400> vif1_regs_{};
    std::array<u64, 2> vif0_fifo_{};
    std::array<u64, 2> vif1_fifo_{};

    u32 gif_ctrl_ = 0;
    u32 gif_mode_ = 0;
    u32 gif_stat_ = 0;
    std::array<u64, 2> gif_fifo_{};

    std::array<u16, 0x100> dve_bus_{};
    std::array<u16, 0x100> dve_regs_{};
    u16 dve_current_reg_ = 0;
    bool dve_command_executing_ = false;
    bool dve_error_detected_ = false;

    u32 mch_ricm_ = 0;
    mutable u32 rdram_sdevid_ = 0;
};

} // namespace ps2
