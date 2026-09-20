#include "core/memory/ee_bus.h"

#include "core/memory/ee_ram.h"

namespace ps2 {

u32 EeBus::to_physical(u32 address) {
    if (address >= 0x80000000u && address < 0xC0000000u) {
        return address & 0x1FFFFFFFu;
    }
    return address;
}

bool EeBus::read8(u32 address, u8& value) const {
    return ram_.read8(to_physical(address), value);
}

bool EeBus::read16(u32 address, u16& value) const {
    return ram_.read16(to_physical(address), value);
}

bool EeBus::read32(u32 address, u32& value) const {
    return ram_.read32(to_physical(address), value);
}

bool EeBus::read64(u32 address, u64& value) const {
    return ram_.read64(to_physical(address), value);
}

bool EeBus::write8(u32 address, u8 value) {
    return ram_.write8(to_physical(address), value);
}

bool EeBus::write16(u32 address, u16 value) {
    return ram_.write16(to_physical(address), value);
}

bool EeBus::write32(u32 address, u32 value) {
    return ram_.write32(to_physical(address), value);
}

bool EeBus::write64(u32 address, u64 value) {
    return ram_.write64(to_physical(address), value);
}

} // namespace ps2
