#include "core/memory/ee_bus.h"

#include "core/bios/bios.h"
#include "core/hw/ee_hw.h"
#include "core/memory/ee_ram.h"
#include "core/memory/ee_scratchpad.h"

namespace ps2 {

EeBus::EeBus(
    EeRam& ram,
    EeScratchpad& scratchpad,
    EeHw& hw,
    const Bios& bios)
    : ram_(ram),
      scratchpad_(scratchpad),
      hw_(hw),
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
    if (hw_.read32(physical, value)) {
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
    return bios_.read64_physical(physical, value);
}

bool EeBus::write8(u32 address, u8 value) {
    if (scratchpad_.contains(address, 1)) {
        return scratchpad_.write8(address, value);
    }
    const u32 physical = to_physical(address);
    return physical < EeRam::kSize
        ? ram_.write8(physical, value)
        : false;
}

bool EeBus::write16(u32 address, u16 value) {
    if (scratchpad_.contains(address, 2)) {
        return scratchpad_.write16(address, value);
    }
    const u32 physical = to_physical(address);
    return physical < EeRam::kSize
        ? ram_.write16(physical, value)
        : false;
}

bool EeBus::write32(u32 address, u32 value) {
    if (scratchpad_.contains(address, 4)) {
        return scratchpad_.write32(address, value);
    }
    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.write32(physical, value);
    }
    return hw_.write32(physical, value);
}

bool EeBus::write64(u32 address, u64 value) {
    if (scratchpad_.contains(address, 8)) {
        return scratchpad_.write64(address, value);
    }
    const u32 physical = to_physical(address);
    return physical < EeRam::kSize
        ? ram_.write64(physical, value)
        : false;
}

void EeBus::tick(u64 cycles) {
    hw_.tick(cycles);
}

} // namespace ps2
