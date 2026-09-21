#pragma once

#include "common/types.h"

#include <array>

namespace ps2 {

class Bios;
class CdvdHw;
class EeHw;
class IopHwWindow;
class IopIntc;
class IopRam;

class IopBus {
public:
    IopBus(
        IopRam& ram,
        IopHwWindow& hw,
        EeHw& ee_hw,
        IopIntc& intc,
        CdvdHw& cdvd,
        const Bios& bios);

    void reset();
    void tick(u64 cycles);

    [[nodiscard]] bool read8(u32 address, u8& value) const;
    [[nodiscard]] bool read16(u32 address, u16& value) const;
    [[nodiscard]] bool read32(u32 address, u32& value) const;

    [[nodiscard]] bool write8(u32 address, u8 value);
    [[nodiscard]] bool write16(u32 address, u16 value);
    [[nodiscard]] bool write32(u32 address, u32 value);

    [[nodiscard]] static u32 to_physical(u32 address);
    [[nodiscard]] static bool is_ram_address(u32 address) {
        return to_physical(address) < 0x00800000u;
    }

    [[nodiscard]] bool interrupt_pending() const;
    void raise_dma_irq(u32 channel);

private:
    struct RootCounter {
        u64 count = 0;
        u32 mode = 0;
        u64 target = 0;
        u64 phase = 0;
    };

    [[nodiscard]] static bool decode_root_counter(
        u32 physical,
        u32& index,
        u32& reg);
    [[nodiscard]] u64 root_counter_rate(u32 index) const;
    [[nodiscard]] bool read_root_counter(u32 physical, u32 width, u32& value) const;
    [[nodiscard]] bool write_root_counter(u32 physical, u32 width, u32 value);

    [[nodiscard]] bool write_dma_icr(u32 physical, u32 value);
    [[nodiscard]] bool read_ohci(u32 physical, u32 width, u32& value) const;
    [[nodiscard]] bool write_ohci(u32 physical, u32 width, u32 value);
    void reset_ohci(bool hard);
    [[nodiscard]] bool read_sif32(u32 physical, u32& value) const;
    [[nodiscard]] bool write_sif32(u32 physical, u32 value);

    IopRam& ram_;
    IopHwWindow& hw_;
    EeHw& ee_hw_;
    IopIntc& intc_;
    CdvdHw& cdvd_;
    const Bios& bios_;
    std::array<u8, 0x100> cache_control_{};
    std::array<u8, 0x800> spu2_regs_{};
    std::array<u32, 0x40> ohci_regs_{};
    u64 ohci_frame_phase_ = 0;
    std::array<RootCounter, 6> root_counters_{};
};

} // namespace ps2
