#pragma once

#include "common/types.h"

namespace ps2 {

class Bios;
class EeHw;
class EeRam;
class EeScratchpad;
class GsPrivileged;
class IopHwWindow;

class EeBus {
public:
    EeBus(
        EeRam& ram,
        EeScratchpad& scratchpad,
        EeHw& hw,
        IopHwWindow& iop_hw,
        GsPrivileged& gs,
        const Bios& bios);

    [[nodiscard]] bool read8(u32 address, u8& value) const;
    [[nodiscard]] bool read16(u32 address, u16& value) const;
    [[nodiscard]] bool read32(u32 address, u32& value) const;
    [[nodiscard]] bool read64(u32 address, u64& value) const;

    [[nodiscard]] bool write8(u32 address, u8 value);
    [[nodiscard]] bool write16(u32 address, u16 value);
    [[nodiscard]] bool write32(u32 address, u32 value);
    [[nodiscard]] bool write64(u32 address, u64 value);

    void tick(u64 cycles);

    [[nodiscard]] static u32 to_physical(u32 address);
    [[nodiscard]] static bool is_iop_ram_physical(u32 address) {
        return address >= 0x1C000000u && address < 0x1C200000u;
    }

private:
    EeRam& ram_;
    EeScratchpad& scratchpad_;
    EeHw& hw_;
    IopHwWindow& iop_hw_;
    GsPrivileged& gs_;
    const Bios& bios_;
};

} // namespace ps2
