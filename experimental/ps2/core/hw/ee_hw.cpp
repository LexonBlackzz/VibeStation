#include "core/hw/ee_hw.h"

namespace ps2 {
namespace {

constexpr u32 kIntcStat = 0x1000F000u;
constexpr u32 kIntcMask = 0x1000F010u;
constexpr u32 kMchRicm = 0x1000F430u;
constexpr u32 kMchDrd = 0x1000F440u;
constexpr u32 kDmacEnabler = 0x1000F520u;
constexpr u32 kDmacEnablew = 0x1000F590u;
constexpr u32 kSbusF240 = 0x1000F240u;
constexpr u32 kSbusF260 = 0x1000F260u;

bool decode_timer(u32 address, u32& index, u32& reg) {
    for (u32 i = 0; i < 4; ++i) {
        const u32 base = 0x10000000u + (i * 0x800u);
        if (address >= base && address < base + 0x40u) {
            const u32 off = address - base;
            if (off == 0x00u || off == 0x10u || off == 0x20u || off == 0x30u) {
                index = i; reg = off; return true;
            }
        }
    }
    return false;
}

} // namespace

void EeHw::reset() {
    cycles_ = 0;
    timer_phase_.fill(0);
    timer_count_base_.fill(0);
    timer_mode_.fill(0);
    timer_comp_.fill(0);
    timer_hold_.fill(0);
    regs_.fill(0);
    dmac_regs_.fill(0);
    ipu_cmd_ = ipu_ctrl_ = ipu_bp_ = ipu_top_ = 0;
    ipu_in_fifo_.fill(0);
    ipu_out_fifo_.fill(0);
    vif0_regs_.fill(0);
    vif1_regs_.fill(0);
    vif0_fifo_.fill(0);
    vif1_fifo_.fill(0);
    gif_ctrl_ = 0;
    gif_mode_ = 0;
    gif_stat_ = 0;
    gif_fifo_.fill(0);
    dve_bus_.fill(0);
    dve_regs_.fill(0);
    dve_current_reg_ = 0;
    dve_command_executing_ = false;
    dve_error_detected_ = false;
    mch_ricm_ = 0;
    rdram_sdevid_ = 0;

    // Reset values used by the retail BIOS during early hardware probing.
    generic_write32(kDmacEnabler, 0x1201u);
    generic_write32(kDmacEnablew, 0x1201u);
    generic_write32(kSbusF260, 0x1D000060u);
}

void EeHw::tick(u64 cycles) {
    cycles_ += cycles;

    for (u32 i = 0; i < 4u; ++i) {
        u32& mode = timer_mode_[i];

        // CUE=0 pauses the counter. Gate timing is intentionally permissive
        // until H/V gate edges are supplied by VideoTiming; BIOS bootstrap
        // primarily relies on free-running timers.
        if ((mode & (1u << 7)) == 0) {
            continue;
        }

        u64 rate = 2;
        switch (mode & 0x3u) {
        case 0: rate = 2; break;       // BUSCLK (EE clock / 2)
        case 1: rate = 32; break;      // BUSCLK / 16
        case 2: rate = 512; break;     // BUSCLK / 256
        case 3: rate = 18876; break;   // Bootstrap HBLANK divisor used by BIOS calibration
        }

        timer_phase_[i] += cycles;
        while (timer_phase_[i] >= rate) {
            timer_phase_[i] -= rate;

            const u32 previous = timer_count_base_[i] & 0xFFFFu;
            u32 next = (previous + 1u) & 0xFFFFu;

            if (next == (timer_comp_[i] & 0xFFFFu)) {
                const bool flag_was_clear = (mode & (1u << 10)) == 0;
                mode |= 1u << 10;
                if (flag_was_clear && (mode & (1u << 8)) != 0) {
                    raise_intc(9u + i);
                }
                if ((mode & (1u << 6)) != 0) {
                    next = 0;
                }
            }

            if (previous == 0xFFFFu) {
                const bool flag_was_clear = (mode & (1u << 11)) == 0;
                mode |= 1u << 11;
                if (flag_was_clear && (mode & (1u << 9)) != 0) {
                    raise_intc(9u + i);
                }
            }

            timer_count_base_[i] = next;
        }
    }
}

u32 EeHw::vif1_stat() const {
    return static_cast<u32>(vif1_regs_[0]) |
           (static_cast<u32>(vif1_regs_[1]) << 8) |
           (static_cast<u32>(vif1_regs_[2]) << 16) |
           (static_cast<u32>(vif1_regs_[3]) << 24);
}

void EeHw::update_vif1_stat(u32 set_bits, u32 clear_bits) {
    u32 value = vif1_stat();
    value &= ~clear_bits;
    value |= set_bits;
    for (u32 i = 0; i < 4u; ++i) {
        vif1_regs_[i] = static_cast<u8>(value >> (i * 8));
    }
}

void EeHw::raise_intc(u32 irq) {
    if (irq < 16u) {
        generic_write32(kIntcStat, generic_read32(kIntcStat) | (1u << irq));
    }
}

bool EeHw::intc_pending() const {
    return (generic_read32(kIntcStat) & generic_read32(kIntcMask) & 0xFFFFu) != 0;
}

void EeHw::raise_dmac(u32 channel) {
    if (channel >= 10u) return;
    const u32 offset = 0x1000E010u - kDmacBase;
    u32 stat =
        static_cast<u32>(dmac_regs_[offset]) |
        (static_cast<u32>(dmac_regs_[offset + 1]) << 8) |
        (static_cast<u32>(dmac_regs_[offset + 2]) << 16) |
        (static_cast<u32>(dmac_regs_[offset + 3]) << 24);
    stat |= 1u << channel;
    for (u32 i = 0; i < 4; ++i) {
        dmac_regs_[offset + i] = static_cast<u8>(stat >> (i * 8));
    }
}

bool EeHw::dmac_pending() const {
    const u32 offset = 0x1000E010u - kDmacBase;
    const u32 stat =
        static_cast<u32>(dmac_regs_[offset]) |
        (static_cast<u32>(dmac_regs_[offset + 1]) << 8) |
        (static_cast<u32>(dmac_regs_[offset + 2]) << 16) |
        (static_cast<u32>(dmac_regs_[offset + 3]) << 24);
    const u32 causes = stat & 0x03FFu;
    const u32 masks = (stat >> 16) & 0x03FFu;
    return (causes & masks) != 0;
}

bool EeHw::in_reg_window(u32 address, std::size_t width) const {
    if (address < kRegBase) {
        return false;
    }
    const std::size_t offset = static_cast<std::size_t>(address - kRegBase);
    return offset <= kRegSize && width <= (kRegSize - offset);
}

u32 EeHw::generic_read32(u32 address) const {
    const u32 offset = address - kRegBase;
    return static_cast<u32>(regs_[offset]) |
           (static_cast<u32>(regs_[offset + 1]) << 8) |
           (static_cast<u32>(regs_[offset + 2]) << 16) |
           (static_cast<u32>(regs_[offset + 3]) << 24);
}

void EeHw::generic_write32(u32 address, u32 value) {
    const u32 offset = address - kRegBase;
    for (u32 i = 0; i < 4; ++i) {
        regs_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
}

bool EeHw::read8(u32 address, u8& value) const {
    if (!in_reg_window(address, 1)) {
        return false;
    }

    if (address == 0x1000F410u) {
        value = 0;
        return true;
    }

    value = regs_[address - kRegBase];
    return true;
}

bool EeHw::read16(u32 address, u16& value) const {
    if (address >= 0x1A000000u && address < 0x1A000020u) {
        const u32 offset = address & 0x1Fu;
        if (offset == 0x06u) {
            u16 result = dve_bus_[0x06] & 2u;
            if (dve_error_detected_) result |= 1u;
            if (dve_bus_[0x06] < 3u && dve_command_executing_) {
                ++const_cast<EeHw*>(this)->dve_bus_[0x06];
            } else {
                const_cast<EeHw*>(this)->dve_command_executing_ = false;
            }
            value = result;
            return true;
        }
        value = dve_bus_[offset];
        return true;
    }
    if (!in_reg_window(address, 2)) {
        return false;
    }

    const u32 offset = address - kRegBase;
    value = static_cast<u16>(regs_[offset]) |
            (static_cast<u16>(regs_[offset + 1]) << 8);
    return true;
}

bool EeHw::read32(u32 address, u32& value) const {
    switch (address) {
    case 0x10002000u: value = ipu_cmd_; return true;
    case 0x10002010u: value = ipu_ctrl_; return true;
    case 0x10002020u: value = ipu_bp_; return true;
    case 0x10002030u: value = ipu_top_; return true;
    default: break;
    }
    auto read_vif = [&](u32 base, const auto& regs) -> bool {
        if (address < base || address + 4u > base + 0x400u) return false;
        const u32 o = address - base;
        value = static_cast<u32>(regs[o]) | (static_cast<u32>(regs[o+1]) << 8) |
                (static_cast<u32>(regs[o+2]) << 16) | (static_cast<u32>(regs[o+3]) << 24);
        return true;
    };
    if (read_vif(0x10003800u, vif0_regs_)) return true;
    if (read_vif(0x10003C00u, vif1_regs_)) return true;
    if (address >= kDmacBase && address + 4u <= kDmacBase + kDmacSize) {
        const u32 o = address - kDmacBase;
        value = static_cast<u32>(dmac_regs_[o]) |
                (static_cast<u32>(dmac_regs_[o + 1]) << 8) |
                (static_cast<u32>(dmac_regs_[o + 2]) << 16) |
                (static_cast<u32>(dmac_regs_[o + 3]) << 24);
        return true;
    }
    switch (address) {
    case 0x10003000u:
        value = gif_ctrl_;
        return true;
    case 0x10003010u:
        value = gif_mode_;
        return true;
    case 0x10003020u:
        value = gif_stat_;
        return true;
    default:
        break;
    }

    u32 timer_index = 0;
    u32 timer_reg = 0;
    if (decode_timer(address, timer_index, timer_reg)) {
        switch (timer_reg) {
        case 0x00u:
            value = timer_count_base_[timer_index] & 0xFFFFu;
            return true;
        case 0x10u: value = timer_mode_[timer_index]; return true;
        case 0x20u: value = timer_comp_[timer_index]; return true;
        case 0x30u: value = timer_hold_[timer_index]; return true;
        default: break;
        }
    }

    if (!in_reg_window(address, 4)) {
        return false;
    }

    if (address == 0x1000F410u) {
        value = 0;
        return true;
    }

    if (address == kMchRicm) {
        value = mch_ricm_;
        return true;
    }

    if (address == kMchDrd) {
        if (((mch_ricm_ >> 6) & 0xFu) == 0) {
            switch ((mch_ricm_ >> 16) & 0xFFFu) {
            case 0x21: // INIT
                if (rdram_sdevid_ < 2) {
                    ++rdram_sdevid_;
                    value = 0x1Fu;
                    return true;
                }
                value = 0;
                return true;
            case 0x23: // CNFGA
                value = 0x0D0Du;
                return true;
            case 0x24: // CNFGB
                value = 0x0090u;
                return true;
            case 0x40: // DEVID
                value = mch_ricm_ & 0x1Fu;
                return true;
            default:
                break;
            }
        }

        value = 0;
        return true;
    }

    if (address == kSbusF240) {
        value = generic_read32(address) | 0xF0000102u;
        return true;
    }

    value = generic_read32(address);
    return true;
}

bool EeHw::read64(u32 address, u64& value) const {
    if (address >= 0x10007000u && address < 0x10007010u) { value = ipu_out_fifo_[(address - 0x10007000u) >> 3]; return true; }
    if (address >= 0x10007010u && address < 0x10007020u) { value = ipu_in_fifo_[(address - 0x10007010u) >> 3]; return true; }
    u32 lo = 0;
    u32 hi = 0;
    if (!read32(address, lo) || !read32(address + 4, hi)) {
        return false;
    }

    value = static_cast<u64>(lo) | (static_cast<u64>(hi) << 32);
    return true;
}

bool EeHw::write8(u32 address, u8 value) {
    if (!in_reg_window(address, 1)) {
        return false;
    }
    regs_[address - kRegBase] = value;
    return true;
}

bool EeHw::write16(u32 address, u16 value) {
    if (address >= 0x1A000000u && address < 0x1A000020u) {
        const u32 offset = address & 0xFFu;
        if (offset == 0x06u) {
            dve_bus_[0x06] &= static_cast<u16>(~3u);
        } else {
            dve_bus_[offset] = value;
        }

        if (offset == 0x00u) {
            if (dve_bus_[0x02] == 0x4Fu || dve_bus_[0x02] == 0x41u) {
                dve_error_detected_ = true;
            } else if ((dve_bus_[0x00] & 0x80u) != 0) {
                if (dve_bus_[0x02] == 0x43u) {
                    int size = static_cast<int>(dve_bus_[0x00] & 0x0Fu);
                    dve_current_reg_ = dve_bus_[0x10];
                    --size;
                    for (int i = 0; i < size; ++i) {
                        dve_regs_[dve_current_reg_ & 0xFFu] = dve_bus_[0x12 + i];
                    }
                    dve_command_executing_ = true;
                    dve_error_detected_ = false;
                } else if (dve_bus_[0x02] == 0x42u) {
                    const int size = static_cast<int>(dve_bus_[0x00] & 0x0Fu);
                    for (int i = 0; i < size; ++i) {
                        dve_bus_[0x10 + i] = dve_regs_[dve_current_reg_ & 0xFFu];
                    }
                    dve_command_executing_ = true;
                    dve_error_detected_ = false;
                }
            }
        } else if (offset == 0x0Au) {
            dve_error_detected_ = (value == 0);
        }
        return true;
    }
    if (!in_reg_window(address, 2)) {
        return false;
    }

    const u32 offset = address - kRegBase;
    regs_[offset] = static_cast<u8>(value);
    regs_[offset + 1] = static_cast<u8>(value >> 8);
    return true;
}

bool EeHw::write32(u32 address, u32 value) {
    switch (address) {
    case 0x10002000u:
        ipu_cmd_ = value & 0x7FFFFFFFu;
        // BCLR/SETTH are immediate in the hardware-facing bootstrap model.
        ipu_ctrl_ &= ~0x80000000u;
        return true;
    case 0x10002010u:
        ipu_ctrl_ = value;
        if ((value & 0x40000000u) != 0) {
            ipu_cmd_ = 0; ipu_ctrl_ = 0; ipu_bp_ = 0; ipu_top_ = 0;
            ipu_in_fifo_.fill(0); ipu_out_fifo_.fill(0);
        }
        return true;
    case 0x10002020u: ipu_bp_ = value; return true;
    case 0x10002030u: ipu_top_ = value; return true;
    default: break;
    }
    auto write_vif = [&](u32 base, auto& regs, bool vif1) -> bool {
        if (address < base || address + 4u > base + 0x400u) return false;
        const u32 o = address - base;
        if (o == 0x10u) { // FBRST
            if ((value & 0x1u) != 0) {
                std::array<u8, 16> rowcol{};
                for (u32 i = 0; i < 16; ++i) rowcol[i] = regs[0x100u + i];
                regs.fill(0);
                for (u32 i = 0; i < 16; ++i) regs[0x100u + i] = rowcol[i];
            }
            u32 stat = static_cast<u32>(regs[0]) | (static_cast<u32>(regs[1]) << 8) |
                       (static_cast<u32>(regs[2]) << 16) | (static_cast<u32>(regs[3]) << 24);
            if ((value & 0x2u) != 0) stat |= 1u << 9;
            if ((value & 0x4u) != 0) stat |= 1u << 8;
            if ((value & 0x8u) != 0) stat &= ~((1u<<8)|(1u<<9)|(1u<<10)|(1u<<11)|(1u<<12)|(1u<<13));
            for (u32 i=0;i<4;++i) regs[i]=static_cast<u8>(stat>>(i*8));
            return true;
        }
        if (o == 0x00u && vif1) {
            const u32 old = static_cast<u32>(regs[0]) | (static_cast<u32>(regs[1]) << 8) |
                            (static_cast<u32>(regs[2]) << 16) | (static_cast<u32>(regs[3]) << 24);
            value = (old & ~(1u << 23)) | (value & (1u << 23));
        }
        for (u32 i=0;i<4;++i) regs[o+i]=static_cast<u8>(value>>(i*8));
        return true;
    };
    if (write_vif(0x10003800u, vif0_regs_, false)) return true;
    if (write_vif(0x10003C00u, vif1_regs_, true)) return true;
    if (address >= kDmacBase && address + 4u <= kDmacBase + kDmacSize) {
        const u32 local = address - kDmacBase;
        auto read_dmac = [&](u32 a) {
            const u32 o = a - kDmacBase;
            return static_cast<u32>(dmac_regs_[o]) |
                   (static_cast<u32>(dmac_regs_[o + 1]) << 8) |
                   (static_cast<u32>(dmac_regs_[o + 2]) << 16) |
                   (static_cast<u32>(dmac_regs_[o + 3]) << 24);
        };
        auto store_dmac = [&](u32 a, u32 v) {
            const u32 o = a - kDmacBase;
            for (u32 i = 0; i < 4; ++i) dmac_regs_[o + i] = static_cast<u8>(v >> (i * 8));
        };

        if (address == 0x1000E010u || address == 0x1000E100u) {
            const u32 old = read_dmac(0x1000E010u);
            const u32 next = (old & ~(value & 0xFFFFu)) ^ (value & 0xFFFF0000u);
            store_dmac(0x1000E010u, next);
            return true;
        }

        const u32 lane = address & 0xFFu;
        if (lane == 0x20u && address < 0x1000E000u) value &= 0xFFFFu; // QWC
        if ((address == 0x1000D010u || address == 0x1000D410u)) value &= 0x7FFFFFFFu;
        if (lane == 0x80u && address < 0x1000E000u) value &= 0x3FF0u; // SADR
        (void)local;
        store_dmac(address, value);
        return true;
    }
    switch (address) {
    case 0x10003000u: // GIF_CTRL
        gif_ctrl_ = value & 0x9u;
        if ((gif_ctrl_ & 0x1u) != 0) {
            gif_fifo_.fill(0);
            gif_stat_ = 0;
        }
        gif_stat_ = (gif_stat_ & ~(1u << 3)) | (gif_ctrl_ & (1u << 3));
        return true;
    case 0x10003010u: // GIF_MODE
        gif_mode_ = value;
        gif_stat_ = (gif_stat_ & ~0x5u) | (gif_mode_ & 0x5u);
        return true;
    default:
        break;
    }

    u32 timer_index = 0;
    u32 timer_reg = 0;
    if (decode_timer(address, timer_index, timer_reg)) {
        switch (timer_reg) {
        case 0x00u:
            timer_count_base_[timer_index] = value & 0xFFFFu;
            timer_phase_[timer_index] = 0;
            return true;
        case 0x10u: {
            // MODE bits 10/11 are sticky event flags. Writing a one clears
            // the corresponding flag; bits 0..9 replace the timer settings.
            u32 flags = timer_mode_[timer_index] & 0xC00u;
            flags &= ~(value & 0xC00u);
            timer_mode_[timer_index] = (value & 0x3FFu) | flags;
            timer_phase_[timer_index] = 0;
            return true;
        }
        case 0x20u:
            timer_comp_[timer_index] = value & 0xFFFFu;
            return true;
        case 0x30u:
            timer_hold_[timer_index] = value & 0xFFFFu;
            return true;
        default: break;
        }
    }

    if (!in_reg_window(address, 4)) {
        return false;
    }

    if (address == kIntcStat) {
        generic_write32(kIntcStat, generic_read32(kIntcStat) & ~value);
        return true;
    }

    if (address == kIntcMask) {
        generic_write32(kIntcMask, generic_read32(kIntcMask) ^ (value & 0xFFFFu));
        return true;
    }

    if (address == kMchRicm) {
        if ((((value >> 16) & 0xFFFu) == 0x21u) &&
            (((value >> 6) & 0xFu) == 1u) &&
            ((generic_read32(kMchDrd) & 0x80u) == 0)) {
            rdram_sdevid_ = 0;
        }

        mch_ricm_ = value & ~0x80000000u;
        generic_write32(address, mch_ricm_);
        return true;
    }

    if (address == kDmacEnablew) {
        generic_write32(kDmacEnablew, value);
        generic_write32(kDmacEnabler, value);
        return true;
    }

    if (address == kSbusF240) {
        u32 old_value = generic_read32(address);
        if ((value & 0x100u) != 0) {
            old_value |= 0x100u;
        } else {
            old_value &= ~0x100u;
        }
        generic_write32(address, old_value);
        return true;
    }

    generic_write32(address, value);
    return true;
}

bool EeHw::write64(u32 address, u64 value) {
    if (address >= 0x10007000u && address < 0x10007010u) { ipu_out_fifo_[(address - 0x10007000u) >> 3] = value; return true; }
    if (address >= 0x10007010u && address < 0x10007020u) { ipu_in_fifo_[(address - 0x10007010u) >> 3] = value; return true; }
    if (address >= 0x10004000u && address < 0x10004010u) { vif0_fifo_[(address - 0x10004000u) >> 3] = value; return true; }
    if (address >= 0x10005000u && address < 0x10005010u) { vif1_fifo_[(address - 0x10005000u) >> 3] = value; return true; }
    if (address >= 0x10006000u && address < 0x10006010u) {
        gif_fifo_[(address - 0x10006000u) >> 3] = value;
        return true;
    }
    return write32(address, static_cast<u32>(value)) &&
           write32(address + 4, static_cast<u32>(value >> 32));
}

} // namespace ps2
