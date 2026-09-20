#include "core/memory/ee_bus.h"

#include "core/bios/bios.h"
#include "core/memory/ee_ram.h"

namespace ps2 {

u32 EeBus::to_physical(u32 address) {
    if (address >= 0x80000000u && address < 0xC0000000u) {
        return address & 0x1FFFFFFFu;
    }
    return address;
}

bool EeBus::read8(u32 address, u8& value) const {
    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.read8(physical, value);
    }
    return bios_.read8_physical(physical, value);
}

bool EeBus::read16(u32 address, u16& value) const {
    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.read16(physical, value);
    }
    return bios_.read16_physical(physical, value);
}

bool EeBus::read32(u32 address, u32& value) const {
    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.read32(physical, value);
    }
    return bios_.read32_physical(physical, value);
}

bool EeBus::read64(u32 address, u64& value) const {
    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.read64(physical, value);
    }
    return bios_.read64_physical(physical, value);
}

bool EeBus::write8(u32 address, u8 value) {
    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.write8(physical, value);
    }
    return false;
}

bool EeBus::write16(u32 address, u16 value) {
    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.write16(physical, value);
    }
    return false;
}

bool EeBus::write32(u32 address, u32 value) {
    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.write32(physical, value);
    }
    return false;
}

bool EeBus::write64(u32 address, u64 value) {
    const u32 physical = to_physical(address);
    if (physical < EeRam::kSize) {
        return ram_.write64(physical, value);
    }
    return false;
}

} // namespace ps2
