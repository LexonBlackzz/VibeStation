#include "core/memory/ee_bus.h"

#include "core/bios/bios.h"
#include "core/gs/gs_core.h"
#include "core/gs/gs_privileged.h"
#include "core/hw/ee_hw.h"
#include "core/hw/iop_hw_window.h"
#include "core/iop/iop_ram.h"
#include "core/memory/ee_ram.h"
#include "core/memory/ee_scratchpad.h"

namespace ps2 {

EeBus::EeBus(EeRam& ram, EeScratchpad& scratchpad, EeHw& hw,
             IopHwWindow& iop_hw, IopRam& iop_ram, GsPrivileged& gs,
             GsCore& gs_core, const Bios& bios)
    : ram_(ram), scratchpad_(scratchpad), hw_(hw), iop_hw_(iop_hw),
      iop_ram_(iop_ram), gs_(gs), gs_core_(gs_core), bios_(bios) {}

void EeBus::reset() {
    vu0_micro_.fill(0);
    vu0_data_.fill(0);
    vu1_micro_.fill(0);
    vu1_data_.fill(0);
}

u32 EeBus::to_physical(u32 address) {
    // The EE exposes main RAM through uncached (0x2...) and uncached
    // accelerated (0x3...) aliases in addition to the normal physical,
    // KSEG0, and KSEG1 views. Retail kernel interrupt code uses the 0x2
    // alias as soon as the scheduler starts.
    if (address >= 0x20000000u && address < 0x22000000u) {
        return address - 0x20000000u;
    }
    if (address >= 0x30000000u && address < 0x32000000u) {
        return address - 0x30000000u;
    }
    if (address >= 0x80000000u && address < 0xC0000000u) return address & 0x1FFFFFFFu;
    return address;
}

bool EeBus::read8(u32 address, u8& value) const {
    if (scratchpad_.contains(address,1)) return scratchpad_.read8(address,value);
    const u32 physical=to_physical(address);
    if (physical >= 0x11000000u && physical < 0x11004000u) { value = vu0_micro_[(physical - 0x11000000u) & 0xFFFu]; return true; }
    if (physical >= 0x11004000u && physical < 0x11008000u) { value = vu0_data_[(physical - 0x11004000u) & 0xFFFu]; return true; }
    if (physical >= 0x11008000u && physical < 0x1100C000u) { value = vu1_micro_[physical - 0x11008000u]; return true; }
    if (physical >= 0x1100C000u && physical < 0x11010000u) { value = vu1_data_[physical - 0x1100C000u]; return true; }
    if(physical<EeRam::kSize) return ram_.read8(physical,value);
    if (is_iop_ram_physical(physical)) return iop_ram_.read8(iop_ram_offset(physical),value);
    if(hw_.read8(physical,value)) return true;
    if(iop_hw_.read8(physical,value)) return true;
    if(gs_.read8(physical,value)) return true;
    return bios_.read8_physical(physical,value);
}
bool EeBus::read16(u32 address,u16& value) const {
    if(scratchpad_.contains(address,2)) return scratchpad_.read16(address,value);
    const u32 physical=to_physical(address);
    if (physical >= 0x11000000u && physical + 2u <= 0x11010000u) {
        value = 0;
        for (u32 i = 0; i < 2u; ++i) { u8 b = 0; if (!read8(physical + i, b)) return false; value |= static_cast<u16>(b) << (i * 8); }
        return true;
    }
    if(physical<EeRam::kSize) return ram_.read16(physical,value);
    if(is_iop_ram_physical(physical)) return iop_ram_.read16(iop_ram_offset(physical),value);
    if(hw_.read16(physical,value)) return true;
    if(iop_hw_.read16(physical,value)) return true;
    if(gs_.read16(physical,value)) return true;
    return bios_.read16_physical(physical,value);
}
bool EeBus::read32(u32 address,u32& value) const {
    if(scratchpad_.contains(address,4)) return scratchpad_.read32(address,value);
    const u32 physical=to_physical(address);
    if (physical >= 0x11000000u && physical + 4u <= 0x11010000u) {
        value = 0;
        for (u32 i = 0; i < 4u; ++i) { u8 b = 0; if (!read8(physical + i, b)) return false; value |= static_cast<u32>(b) << (i * 8); }
        return true;
    }
    if(physical<EeRam::kSize) return ram_.read32(physical,value);
    if(is_iop_ram_physical(physical)) return iop_ram_.read32(iop_ram_offset(physical),value);
    if(hw_.read32(physical,value)) return true;
    if(iop_hw_.read32(physical,value)) return true;
    if(gs_.read32(physical,value)) return true;
    return bios_.read32_physical(physical,value);
}
bool EeBus::read64(u32 address,u64& value) const {
    if(scratchpad_.contains(address,8)) return scratchpad_.read64(address,value);
    const u32 physical=to_physical(address);
    if (physical >= 0x11000000u && physical + 8u <= 0x11010000u) {
        value = 0;
        for (u32 i = 0; i < 8u; ++i) { u8 b = 0; if (!read8(physical + i, b)) return false; value |= static_cast<u64>(b) << (i * 8); }
        return true;
    }
    if(physical<EeRam::kSize) return ram_.read64(physical,value);
    if(is_iop_ram_physical(physical)) return iop_ram_.read64(iop_ram_offset(physical),value);
    if(hw_.read64(physical,value)) return true;
    if(iop_hw_.read64(physical,value)) return true;
    if(gs_.read64(physical,value)) return true;
    return bios_.read64_physical(physical,value);
}
bool EeBus::write8(u32 address,u8 value){
    if(scratchpad_.contains(address,1)) return scratchpad_.write8(address,value);
    const u32 physical=to_physical(address);
    if (physical >= 0x11000000u && physical < 0x11004000u) { vu0_micro_[(physical - 0x11000000u) & 0xFFFu] = value; return true; }
    if (physical >= 0x11004000u && physical < 0x11008000u) { vu0_data_[(physical - 0x11004000u) & 0xFFFu] = value; return true; }
    if (physical >= 0x11008000u && physical < 0x1100C000u) { vu1_micro_[physical - 0x11008000u] = value; return true; }
    if (physical >= 0x1100C000u && physical < 0x11010000u) { vu1_data_[physical - 0x1100C000u] = value; return true; }
    if(physical<EeRam::kSize) return ram_.write8(physical,value);
    if(is_iop_ram_physical(physical)) return iop_ram_.write8(iop_ram_offset(physical),value);
    if(hw_.write8(physical,value)) return true;
    if(iop_hw_.write8(physical,value)) return true;
    return gs_.write8(physical,value);
}
bool EeBus::write16(u32 address,u16 value){
    if(scratchpad_.contains(address,2)) return scratchpad_.write16(address,value);
    const u32 physical=to_physical(address);
    if (physical >= 0x11000000u && physical + 2u <= 0x11010000u) {
        for (u32 i = 0; i < 2u; ++i) if (!write8(physical + i, static_cast<u8>(value >> (i * 8)))) return false;
        return true;
    }
    if(physical<EeRam::kSize) return ram_.write16(physical,value);
    if(is_iop_ram_physical(physical)) return iop_ram_.write16(iop_ram_offset(physical),value);
    if(hw_.write16(physical,value)) return true;
    if(iop_hw_.write16(physical,value)) return true;
    return gs_.write16(physical,value);
}
bool EeBus::write32(u32 address,u32 value){
    if(scratchpad_.contains(address,4)) return scratchpad_.write32(address,value);
    const u32 physical=to_physical(address);
    if (physical >= GsCore::kGifFifoBase && physical < GsCore::kGifFifoBase + 0x10u)
        return gs_core_.write_gif_fifo32(physical, value);
    if (physical >= 0x11000000u && physical + 4u <= 0x11010000u) {
        for (u32 i = 0; i < 4u; ++i) if (!write8(physical + i, static_cast<u8>(value >> (i * 8)))) return false;
        return true;
    }
    if(physical<EeRam::kSize) return ram_.write32(physical,value);
    if(is_iop_ram_physical(physical)) return iop_ram_.write32(iop_ram_offset(physical),value);
    if(hw_.write32(physical,value)) return true;
    if(iop_hw_.write32(physical,value)) return true;
    return gs_.write32(physical,value);
}
bool EeBus::write64(u32 address,u64 value){
    if(scratchpad_.contains(address,8)) return scratchpad_.write64(address,value);
    const u32 physical=to_physical(address);
    if (physical >= GsCore::kGifFifoBase && physical < GsCore::kGifFifoBase + 0x10u)
        return gs_core_.write_gif_fifo64(physical, value);
    if (physical >= 0x11000000u && physical + 8u <= 0x11010000u) {
        for (u32 i = 0; i < 8u; ++i) if (!write8(physical + i, static_cast<u8>(value >> (i * 8)))) return false;
        return true;
    }
    if(physical<EeRam::kSize) return ram_.write64(physical,value);
    if(is_iop_ram_physical(physical)) return iop_ram_.write64(iop_ram_offset(physical),value);
    if(hw_.write64(physical,value)) return true;
    if(iop_hw_.write64(physical,value)) return true;
    return gs_.write64(physical,value);
}
void EeBus::tick(u64 cycles){ hw_.tick(cycles); }
void EeBus::raise_intc(u32 irq){ hw_.raise_intc(irq); }
void EeBus::raise_dmac(u32 channel){ hw_.raise_dmac(channel); }
void EeBus::update_vif1_stat(u32 set_bits,u32 clear_bits){ hw_.update_vif1_stat(set_bits,clear_bits); }
u32 EeBus::vif1_stat() const { return hw_.vif1_stat(); }
bool EeBus::intc_pending() const { return hw_.intc_pending(); }
bool EeBus::dmac_pending() const { return hw_.dmac_pending(); }

} // namespace ps2
