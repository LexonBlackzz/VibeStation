#pragma once

#include "common/types.h"

#include <array>

namespace ps2 {

class Bios;
class EeHw;
class EeRam;
class EeScratchpad;
class GsPrivileged;
class IopHwWindow;
class IopRam;

class EeBus {
public:
    EeBus(
        EeRam& ram,
        EeScratchpad& scratchpad,
        EeHw& hw,
        IopHwWindow& iop_hw,
        IopRam& iop_ram,
        GsPrivileged& gs,
        const Bios& bios);

    void reset();

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
    [[nodiscard]] static u32 iop_ram_offset(u32 physical) {
        return physical - 0x1C000000u;
    }

    EeRam& ram_;
    EeScratchpad& scratchpad_;
    EeHw& hw_;
    IopHwWindow& iop_hw_;
    IopRam& iop_ram_;
    GsPrivileged& gs_;
    const Bios& bios_;

    std::array<u8, 0x1000> vu0_micro_{};
    std::array<u8, 0x1000> vu0_data_{};
    std::array<u8, 0x4000> vu1_micro_{};
    std::array<u8, 0x4000> vu1_data_{};
};

} // namespace ps2
