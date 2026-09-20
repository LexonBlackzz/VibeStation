#pragma once

#include "common/types.h"

namespace ps2 {

class Bios;
class EeRam;

class EeBus {
public:
    EeBus(EeRam& ram, const Bios& bios) : ram_(ram), bios_(bios) {}

    [[nodiscard]] bool read8(u32 address, u8& value) const;
    [[nodiscard]] bool read16(u32 address, u16& value) const;
    [[nodiscard]] bool read32(u32 address, u32& value) const;
    [[nodiscard]] bool read64(u32 address, u64& value) const;

    [[nodiscard]] bool write8(u32 address, u8 value);
    [[nodiscard]] bool write16(u32 address, u16 value);
    [[nodiscard]] bool write32(u32 address, u32 value);
    [[nodiscard]] bool write64(u32 address, u64 value);

private:
    [[nodiscard]] static u32 to_physical(u32 address);

    EeRam& ram_;
    const Bios& bios_;
};

} // namespace ps2
