#include "core/iop/iop_bus.h"

#include "core/bios/bios.h"
#include "core/cdvd/cdvd_hw.h"
#include "core/hw/ee_hw.h"
#include "core/hw/iop_hw_window.h"
#include "core/iop/iop_intc.h"
#include "core/iop/iop_ram.h"

namespace ps2 {
namespace {

constexpr u32 kRamMirrorEnd = 0x00800000u;
constexpr u32 kSifBase = 0x1D000000u;
constexpr u32 kSpu2Base = 0x1F900000u;
constexpr u32 kSpu2Size = 0x800u;
constexpr u32 kCacheControlBase = 0xFFFE0100u;
constexpr u32 kCacheControlEnd = 0xFFFE0200u;

} // namespace

IopBus::IopBus(
    IopRam& ram,
    IopHwWindow& hw,
    EeHw& ee_hw,
    IopIntc& intc,
    CdvdHw& cdvd,
    const Bios& bios)
    : ram_(ram),
      hw_(hw),
      ee_hw_(ee_hw),
      intc_(intc),
      cdvd_(cdvd),
      bios_(bios) {}

void IopBus::reset() {
    cache_control_.fill(0);
    spu2_regs_.fill(0);
}

u32 IopBus::to_physical(u32 address) {
    if (address >= 0x80000000u && address < 0xC0000000u) {
        return address & 0x1FFFFFFFu;
    }
    return address;
}

bool IopBus::interrupt_pending() const {
    return intc_.pending();
}

bool IopBus::read_sif32(u32 physical, u32& value) const {
    if (physical < kSifBase || physical >= kSifBase + 0x100u) {
        return false;
    }

    const u32 reg = physical & 0xF0u;
    switch (reg) {
    case 0x00:
        return ee_hw_.read32(0x1000F200u, value);
    case 0x10:
        return ee_hw_.read32(0x1000F210u, value);
    case 0x20:
        return ee_hw_.read32(0x1000F220u, value);
    case 0x30:
        return ee_hw_.read32(0x1000F230u, value);
    case 0x40:
        if (!ee_hw_.read32(0x1000F240u, value)) return false;
        value |= 0xF0000002u;
        return true;
    case 0x60:
        value = 0;
        return true;
    default:
        value = 0;
        return true;
    }
}

bool IopBus::write_sif32(u32 physical, u32 value) {
    if (physical < kSifBase || physical >= kSifBase + 0x100u) {
        return false;
    }

    const u32 reg = physical & 0xF0u;
    u32 current = 0;
    switch (reg) {
    case 0x00:
        return true;
    case 0x10:
        return ee_hw_.write32(0x1000F210u, value);
    case 0x20:
        if (!ee_hw_.read32(0x1000F220u, current)) return false;
        return ee_hw_.write32(0x1000F220u, current & ~value);
    case 0x30:
        if (!ee_hw_.read32(0x1000F230u, current)) return false;
        return ee_hw_.write32(0x1000F230u, current | value);
    case 0x40: {
        if (!ee_hw_.read32(0x1000F240u, current)) return false;
        const u32 temp = value & 0xF0u;
        if ((value & 0xA0u) != 0) {
            current &= ~0xF000u;
            current |= 0x2000u;
        }
        current ^= temp;
        return ee_hw_.write32(0x1000F240u, current);
    }
    case 0x60:
        return ee_hw_.write32(0x1000F260u, 0);
    default:
        return true;
    }
}

bool IopBus::read8(u32 address, u8& value) const {
    if (address >= kCacheControlBase && address < kCacheControlEnd) {
        value = cache_control_[address - kCacheControlBase];
        return true;
    }
    const u32 physical = to_physical(address);
    if (physical >= kSpu2Base && physical < kSpu2Base + kSpu2Size) {
        value = spu2_regs_[physical - kSpu2Base];
        return true;
    }
    if (physical < kRamMirrorEnd) return ram_.read8(physical & static_cast<u32>(IopRam::kSize - 1), value);
    if (intc_.read8(physical, value)) return true;
    if (cdvd_.read8(physical, value)) return true;
    if (hw_.read8(physical, value)) return true;
    if (physical >= kSifBase && physical < kSifBase + 0x100u) {
        u32 word = 0;
        if (!read_sif32(physical & ~3u, word)) return false;
        value = static_cast<u8>(word >> ((physical & 3u) * 8));
        return true;
    }
    return bios_.read8_physical(physical, value);
}

bool IopBus::read16(u32 address, u16& value) const {
    const u32 physical = to_physical(address);
    if (physical >= kSpu2Base && physical + 2u <= kSpu2Base + kSpu2Size) {
        const u32 offset = physical - kSpu2Base;
        value = static_cast<u16>(spu2_regs_[offset]) |
                (static_cast<u16>(spu2_regs_[offset + 1u]) << 8);
        return true;
    }
    if (intc_.read16(physical, value)) return true;
    u8 lo=0, hi=0;
    if (!read8(address, lo) || !read8(address+1, hi)) return false;
    value = static_cast<u16>(lo) | (static_cast<u16>(hi)<<8);
    return true;
}

bool IopBus::read32(u32 address, u32& value) const {
    if (address >= kCacheControlBase && address + 4 <= kCacheControlEnd) {
        const u32 offset = address - kCacheControlBase;
        value = static_cast<u32>(cache_control_[offset]) |
                (static_cast<u32>(cache_control_[offset+1])<<8) |
                (static_cast<u32>(cache_control_[offset+2])<<16) |
                (static_cast<u32>(cache_control_[offset+3])<<24);
        return true;
    }
    const u32 physical = to_physical(address);
    if (physical >= kSpu2Base && physical + 4u <= kSpu2Base + kSpu2Size) {
        const u32 offset = physical - kSpu2Base;
        value = static_cast<u32>(spu2_regs_[offset]) |
                (static_cast<u32>(spu2_regs_[offset + 1u]) << 8) |
                (static_cast<u32>(spu2_regs_[offset + 2u]) << 16) |
                (static_cast<u32>(spu2_regs_[offset + 3u]) << 24);
        return true;
    }
    if (intc_.read32(physical, value)) return true;
    if (physical >= kSifBase && physical < kSifBase + 0x100u) return read_sif32(physical, value);
    if (physical < kRamMirrorEnd) {
        value = 0;
        for (u32 i=0;i<4;++i) {
            u8 byte=0;
            const u32 offset=(physical+i)&static_cast<u32>(IopRam::kSize-1);
            if (!ram_.read8(offset, byte)) return false;
            value |= static_cast<u32>(byte) << (i*8);
        }
        return true;
    }
    if (cdvd_.read32(physical, value)) return true;
    if (hw_.read32(physical, value)) return true;
    return bios_.read32_physical(physical, value);
}

bool IopBus::write8(u32 address, u8 value) {
    if (address >= kCacheControlBase && address < kCacheControlEnd) {
        cache_control_[address-kCacheControlBase]=value; return true;
    }
    const u32 physical=to_physical(address);
    if (physical >= kSpu2Base && physical < kSpu2Base + kSpu2Size) {
        spu2_regs_[physical - kSpu2Base] = value;
        return true;
    }
    if (physical < kRamMirrorEnd) return ram_.write8(physical & static_cast<u32>(IopRam::kSize-1), value);
    if (intc_.write8(physical,value)) return true;
    if (cdvd_.write8(physical,value)) return true;
    if (hw_.write8(physical,value)) return true;
    if (physical >= kSifBase && physical < kSifBase+0x100u) {
        u32 word=0; if(!read_sif32(physical&~3u,word)) return false;
        const u32 shift=(physical&3u)*8;
        word=(word&~(0xFFu<<shift))|(static_cast<u32>(value)<<shift);
        return write_sif32(physical&~3u,word);
    }
    return false;
}

bool IopBus::write16(u32 address, u16 value) {
    const u32 physical=to_physical(address);
    if (physical >= kSpu2Base && physical + 2u <= kSpu2Base + kSpu2Size) {
        const u32 offset = physical - kSpu2Base;
        spu2_regs_[offset] = static_cast<u8>(value);
        spu2_regs_[offset + 1u] = static_cast<u8>(value >> 8);
        return true;
    }
    if (physical == 0x1F801450u) {
        if (!hw_.write16(physical, value)) return false;
        if ((value & 0x2u) != 0) ee_hw_.raise_intc(1);
        return true;
    }
    if (intc_.write16(physical,value)) return true;
    return write8(address,static_cast<u8>(value)) && write8(address+1,static_cast<u8>(value>>8));
}

bool IopBus::write32(u32 address, u32 value) {
    if (address >= kCacheControlBase && address + 4 <= kCacheControlEnd) {
        const u32 offset=address-kCacheControlBase;
        for(u32 i=0;i<4;++i) cache_control_[offset+i]=static_cast<u8>(value>>(i*8));
        return true;
    }
    const u32 physical=to_physical(address);
    if (physical >= kSpu2Base && physical + 4u <= kSpu2Base + kSpu2Size) {
        const u32 offset = physical - kSpu2Base;
        for (u32 i = 0; i < 4u; ++i) {
            spu2_regs_[offset + i] = static_cast<u8>(value >> (i * 8));
        }
        return true;
    }
    if (physical == 0x1F801450u) {
        if (!hw_.write32(physical, value)) return false;
        if ((value & 0x2u) != 0) ee_hw_.raise_intc(1);
        return true;
    }
    if (intc_.write32(physical,value)) return true;
    if (physical >= kSifBase && physical < kSifBase+0x100u) return write_sif32(physical,value);
    if (physical < kRamMirrorEnd) {
        for(u32 i=0;i<4;++i){
            const u32 offset=(physical+i)&static_cast<u32>(IopRam::kSize-1);
            if(!ram_.write8(offset,static_cast<u8>(value>>(i*8)))) return false;
        }
        return true;
    }
    if (cdvd_.write32(physical,value)) return true;
    if (hw_.write32(physical,value)) return true;
    return false;
}

} // namespace ps2
