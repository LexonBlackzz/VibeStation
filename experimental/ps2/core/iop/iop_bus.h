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

private:
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
};

} // namespace ps2
