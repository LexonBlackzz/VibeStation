#include "core/memory/ee_bus.h"

#include "core/bios/bios.h"
#include "core/gs/gs_privileged.h"
#include "core/hw/ee_hw.h"
#include "core/hw/iop_hw_window.h"
#include "core/iop/iop_ram.h"
#include "core/memory/ee_ram.h"
#include "core/memory/ee_scratchpad.h"

namespace ps2 {

EeBus::EeBus(
    EeRam& ram,
    EeScratchpad& scratchpad,
    EeHw& hw,
    IopHwWindow& iop_hw,
    IopRam& iop_ram,
    GsPrivileged& gs,
    const Bios& bios)
    : ram_(ram),
      scratchpad_(scratchpad),
      hw_(hw),
      iop_hw_(iop_hw),
      iop_ram_(iop_ram),
      gs_(gs),
      bios_(bios) {}

u32 EeBus::to_physical(u32 address) {
    if (address >= 0x80000000u && address < 0xC0000000u) {
        return address & 0x1FFFFFFFu;
    }
    return address;
}

bool EeBus::read8(u32 address, u8& value) const {
    if (scratchpad_.contains(address, 1)) {
        return scratchpad_.read8(address, value);
    }

    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.read8(physical, value);
    }
    if (is_iop_ram_physical(physical)) {
        return iop_ram_.read8(iop_ram_offset(physical), value);
    }
    if (hw_.read8(physical, value)) {
        return true;
    }
    if (iop_hw_.read8(physical, value)) {
        return true;
    }
    if (gs_.read8(physical, value)) {
        return true;
    }
    return bios_.read8_physical(physical, value);
}

bool EeBus::read16(u32 address, u16& value) const {
    if (scratchpad_.contains(address, 2)) {
        return scratchpad_.read16(address, value);
    }

    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.read16(physical, value);
    }
    if (is_iop_ram_physical(physical)) {
        return iop_ram_.read16(iop_ram_offset(physical), value);
    }
    if (hw_.read16(physical, value)) {
        return true;
    }
    if (iop_hw_.read16(physical, value)) {
        return true;
    }
    if (gs_.read16(physical, value)) {
        return true;
    }
    return bios_.read16_physical(physical, value);
}

bool EeBus::read32(u32 address, u32& value) const {
    if (scratchpad_.contains(address, 4)) {
        return scratchpad_.read32(address, value);
    }

    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.read32(physical, value);
    }
    if (is_iop_ram_physical(physical)) {
        return iop_ram_.read32(iop_ram_offset(physical), value);
    }
    if (hw_.read32(physical, value)) {
        return true;
    }
    if (iop_hw_.read32(physical, value)) {
        return true;
    }
    if (gs_.read32(physical, value)) {
        return true;
    }
    return bios_.read32_physical(physical, value);
}

bool EeBus::read64(u32 address, u64& value) const {
    if (scratchpad_.contains(address, 8)) {
        return scratchpad_.read64(address, value);
    }

    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.read64(physical, value);
    }
    if (is_iop_ram_physical(physical)) {
        return iop_ram_.read64(iop_ram_offset(physical), value);
    }
    if (hw_.read64(physical, value)) {
        return true;
    }
    if (iop_hw_.read64(physical, value)) {
        return true;
    }
    if (gs_.read64(physical, value)) {
        return true;
    }
    return bios_.read64_physical(physical, value);
}

bool EeBus::write8(u32 address, u8 value) {
    if (scratchpad_.contains(address, 1)) {
        return scratchpad_.write8(address, value);
    }

    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.write8(physical, value);
    }
    if (is_iop_ram_physical(physical)) {
        return iop_ram_.write8(iop_ram_offset(physical), value);
    }
    if (hw_.write8(physical, value)) {
        return true;
    }
    if (iop_hw_.write8(physical, value)) {
        return true;
    }
    return gs_.write8(physical, value);
}

bool EeBus::write16(u32 address, u16 value) {
    if (scratchpad_.contains(address, 2)) {
        return scratchpad_.write16(address, value);
    }

    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.write16(physical, value);
    }
    if (is_iop_ram_physical(physical)) {
        return iop_ram_.write16(iop_ram_offset(physical), value);
    }
    if (hw_.write16(physical, value)) {
        return true;
    }
    if (iop_hw_.write16(physical, value)) {
        return true;
    }
    return gs_.write16(physical, value);
}

bool EeBus::write32(u32 address, u32 value) {
    if (scratchpad_.contains(address, 4)) {
        return scratchpad_.write32(address, value);
    }

    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.write32(physical, value);
    }
    if (is_iop_ram_physical(physical)) {
        return iop_ram_.write32(iop_ram_offset(physical), value);
    }
    if (hw_.write32(physical, value)) {
        return true;
    }
    if (iop_hw_.write32(physical, value)) {
        return true;
    }
    return gs_.write32(physical, value);
}

bool EeBus::write64(u32 address, u64 value) {
    if (scratchpad_.contains(address, 8)) {
        return scratchpad_.write64(address, value);
    }

    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.write64(physical, value);
    }
    if (is_iop_ram_physical(physical)) {
        return iop_ram_.write64(iop_ram_offset(physical), value);
    }
    if (hw_.write64(physical, value)) {
        return true;
    }
    if (iop_hw_.write64(physical, value)) {
        return true;
    }
    return gs_.write64(physical, value);
}

void EeBus::tick(u64 cycles) {
    hw_.tick(cycles);
}

} // namespace ps2
