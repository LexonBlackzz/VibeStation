#include "core/iop/iop_bus.h"

#include "core/bios/bios.h"
#include "core/cdvd/cdvd_hw.h"
#include "core/hw/ee_hw.h"
#include "core/hw/iop_hw_window.h"
#include "core/iop/iop_intc.h"
#include "core/iop/iop_ram.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u32 kRamMirrorEnd = 0x00800000u;
constexpr u32 kExtensionRomBase = 0x1E000000u;
constexpr u32 kExtensionRomEnd = 0x1E800000u;
constexpr u32 kSifBase = 0x1D000000u;
constexpr u32 kDmaIcr = 0x1F8010F4u;
constexpr u32 kDmaIcr2 = 0x1F801574u;
constexpr u32 kDma4Madr = 0x1F8010C0u;
constexpr u32 kDma4Bcr = 0x1F8010C4u;
constexpr u32 kDma4Chcr = 0x1F8010C8u;
constexpr u32 kDma7Madr = 0x1F801500u;
constexpr u32 kDma7Bcr = 0x1F801504u;
constexpr u32 kDma7Chcr = 0x1F801508u;
constexpr u32 kDma6Madr = 0x1F8010E0u;
constexpr u32 kDma6Bcr = 0x1F8010E4u;
constexpr u32 kDma6Chcr = 0x1F8010E8u;
constexpr u32 kDma11Madr = 0x1F801540u;
constexpr u32 kDma11Bcr = 0x1F801544u;
constexpr u32 kDma11Chcr = 0x1F801548u;
constexpr u32 kDma12Madr = 0x1F801550u;
constexpr u32 kDma12Bcr = 0x1F801554u;
constexpr u32 kDma12Chcr = 0x1F801558u;
constexpr u32 kDmaStart = 1u << 24;
constexpr u32 kSpu2Base = 0x1F900000u;
constexpr u32 kSpu2Statx0 = 0x344u;
constexpr u32 kSpu2Statx1 = 0x744u;
constexpr u32 kSpu2Size = 0x10000u;
constexpr u32 kSio2CmdBase = 0x1F808200u;
constexpr u32 kSio2CmdEnd = 0x1F808240u;
constexpr u32 kSio2Tx = 0x1F808260u;
constexpr u32 kSio2Rx = 0x1F808264u;
constexpr u32 kSio2Ctrl = 0x1F808268u;
constexpr u32 kSio2CmdStat = 0x1F80826Cu;
constexpr u32 kSio2PortStat = 0x1F808270u;
constexpr u32 kSio2FifoStat = 0x1F808274u;
constexpr u32 kSio2TxPos = 0x1F808278u;
constexpr u32 kSio2RxPos = 0x1F80827Cu;
constexpr u32 kSio2Intr = 0x1F808280u;
constexpr u32 kSio2Start = 1u;
constexpr u32 kOhciBase = 0x1F801600u;
constexpr u32 kOhciSize = 0x100u;
constexpr u32 kOhciFrameCycles = 36864u;
constexpr u32 kOhciIntrMie = 1u << 31;
constexpr u32 kOhciRhNps = 1u << 9;
constexpr u32 kOhciPortPps = 1u << 8;
constexpr u32 kFirewireBase = 0x1F808400u;
constexpr u32 kFirewireSize = 0x150u;
constexpr u32 kDev9Base = 0x10000000u;
constexpr u32 kDev9Size = 0x00010000u;
constexpr u32 kCacheControlBase = 0xFFFE0100u;
constexpr u32 kCacheControlEnd = 0xFFFE0200u;

bool is_extension_rom(u32 physical, u32 width) {
    return physical >= kExtensionRomBase &&
           physical <= kExtensionRomEnd - width;
}

bool is_iop_peripheral_open_bus(u32 physical) {
    // The R3000A peripheral/expansion area contains many optional devices.
    // Uninstalled slots read as zero in the compatibility path instead of
    // raising a fatal CPU-side bus error. Keep ROM0 (0x1FC00000+) excluded.
    return physical >= 0x1F000000u &&
           physical < 0x1FC00000u;
}

u32 canonical_cdvd_address(u32 physical) {
    if ((physical & 0xFFFF0000u) == 0x1F400000u) {
        return CdvdHw::kBase | (physical & 0xFFu);
    }
    return physical;
}

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
    sio2_.reset();
    spu2_.reset();
    spu2_dma4_irq_cycles_ = 0;
    reset_ohci(true);
    firewire_regs_.fill(0);
    firewire_regs_[(0x10u) >> 2] = 0x8u; // SCLK ready.
    root_counters_.fill({});
    root_counter_rate_cache_.fill(1u);
    root_counter_debug_ = {};
    for (u32 i = 0; i < root_counters_.size(); ++i) {
        root_counters_[i].mode = 1u << 10; // IRQ output starts enabled.
        root_counters_[i].target =
            i < 3u ? 0x10000ull : 0x100000000ull;
    }
}

void IopBus::reset_ohci(bool hard) {
    const u32 ownership =
        hard ? 0u : (ohci_regs_[1] & (1u << 8));
    ohci_regs_.fill(0);
    ohci_regs_[0] = 0x10u; // OHCI 1.0 revision.
    ohci_regs_[1] = hard ? 0u : (ownership | (3u << 6)); // USB suspend after HCR.
    ohci_regs_[4] = kOhciIntrMie;
    ohci_regs_[13] = 0x27782EDFu; // FSMPS/FI reset values used by OHCI drivers.
    ohci_regs_[17] = 0x628u;      // Low-speed threshold.
    ohci_regs_[18] = kOhciRhNps | 2u; // Two-port root hub, no power switching.
    ohci_regs_[21] = kOhciPortPps;
    ohci_regs_[22] = kOhciPortPps;
    ohci_frame_phase_ = 0;
}

bool IopBus::read_ohci(
    u32 physical,
    u32 width,
    u32& value) const {
    if (physical < kOhciBase ||
        physical + width > kOhciBase + kOhciSize ||
        width == 0u || width > 4u) {
        return false;
    }

    const u32 offset = physical - kOhciBase;
    const u32 aligned = offset & ~3u;
    const u32 byte_offset = offset & 3u;
    if (byte_offset + width > 4u) return false;
    const u32 index = aligned >> 2;

    u32 raw = 0;
    if (index == 5u) {
        // InterruptEnable and InterruptDisable expose the same mask.
        raw = ohci_regs_[4];
    } else if (index == 14u) {
        // A lightweight frame countdown is enough for BIOS/USBD timing probes.
        const u32 fi = ohci_regs_[13] & 0x3FFFu;
        const u64 elapsed =
            ohci_frame_phase_ % kOhciFrameCycles;
        raw = static_cast<u32>(
            (static_cast<u64>(fi) *
             (kOhciFrameCycles - elapsed)) /
            kOhciFrameCycles);
    } else if (index == 21u || index == 22u) {
        // No devices connected, but the root ports are powered.
        raw = kOhciPortPps;
    } else if (index < ohci_regs_.size()) {
        raw = ohci_regs_[index];
    } else {
        raw = 0xFFFFFFFFu;
    }

    const u32 bits = width * 8u;
    const u32 mask =
        bits == 32u ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    value = (raw >> (byte_offset * 8u)) & mask;
    return true;
}

bool IopBus::write_ohci(
    u32 physical,
    u32 width,
    u32 value) {
    if (physical < kOhciBase ||
        physical + width > kOhciBase + kOhciSize ||
        width == 0u || width > 4u) {
        return false;
    }

    const u32 offset = physical - kOhciBase;
    const u32 aligned = offset & ~3u;
    const u32 byte_offset = offset & 3u;
    if (byte_offset + width > 4u) return false;
    const u32 index = aligned >> 2;
    if (index >= ohci_regs_.size()) return true;

    u32 current = 0;
    (void)read_ohci(kOhciBase + aligned, 4u, current);
    const u32 bits = width * 8u;
    const u32 lane_mask =
        bits == 32u
            ? 0xFFFFFFFFu
            : ((1u << bits) - 1u) << (byte_offset * 8u);
    const u32 merged =
        (current & ~lane_mask) |
        ((value << (byte_offset * 8u)) & lane_mask);

    switch (index) {
    case 0: // HcRevision is read-only.
    case 7: // HcPeriodCurrentED
    case 12: // HcDoneHead
    case 14: // HcFmRemaining
    case 15: // HcFmNumber
        return true;
    case 1: // HcControl
        ohci_regs_[1] = merged;
        return true;
    case 2: // HcCommandStatus
        if ((merged & 1u) != 0) {
            // Host-controller reset is self-clearing. A generic backing store
            // would leave HCR stuck and trap firmware in its reset wait loop.
            reset_ohci(false);
            return true;
        }
        // No devices/lists are being scheduled in the bootstrap model, so
        // CLF/BLF/OCR complete immediately instead of remaining busy forever.
        ohci_regs_[2] = 0;
        return true;
    case 3: // HcInterruptStatus: write-one-to-clear.
        ohci_regs_[3] &= ~merged;
        return true;
    case 4: // HcInterruptEnable
        ohci_regs_[4] |= merged;
        return true;
    case 5: // HcInterruptDisable
        ohci_regs_[4] &= ~merged;
        return true;
    case 6: // HcHCCA
        ohci_regs_[6] = merged & 0xFFFFFF00u;
        return true;
    case 8: // HcControlHeadED
    case 9: // HcControlCurrentED
    case 10: // HcBulkHeadED
    case 11: // HcBulkCurrentED
        ohci_regs_[index] = merged & 0xFFFFFFF0u;
        return true;
    case 13: // HcFmInterval
        ohci_regs_[13] = merged;
        return true;
    case 16: // HcPeriodicStart
    case 17: // HcLSThreshold
        ohci_regs_[index] = merged & 0xFFFFu;
        return true;
    case 18: // HcRhDescriptorA: bootstrap model has fixed capabilities.
    case 19: // HcRhDescriptorB
        return true;
    case 20: // HcRhStatus: no attached devices/change events to latch.
        ohci_regs_[20] = 0;
        return true;
    case 21:
    case 22:
        // Root-port reset/power requests complete immediately with no device.
        ohci_regs_[index] = kOhciPortPps;
        return true;
    default:
        ohci_regs_[index] = merged;
        return true;
    }
}

bool IopBus::read_firewire(
    u32 physical,
    u32 width,
    u32& value) const {
    if (physical < kFirewireBase ||
        physical + width > kFirewireBase + kFirewireSize ||
        width == 0u || width > 4u) {
        return false;
    }

    const u32 offset = physical - kFirewireBase;
    const u32 aligned = offset & ~3u;
    const u32 byte_offset = offset & 3u;
    if (byte_offset + width > 4u) return false;
    const u32 index = aligned >> 2;

    u32 raw = 0;
    if (aligned == 0x00u) {
        raw = 0xFFC00001u; // BIOS-visible node ID.
    } else if (aligned == 0x7Cu) {
        raw = 0x10000001u; // Link/node comparison probe value.
    } else if (index < firewire_regs_.size()) {
        raw = firewire_regs_[index];
    }

    const u32 bits = width * 8u;
    const u32 mask =
        bits == 32u ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    value = (raw >> (byte_offset * 8u)) & mask;
    return true;
}

bool IopBus::write_firewire(
    u32 physical,
    u32 width,
    u32 value) {
    if (physical < kFirewireBase ||
        physical + width > kFirewireBase + kFirewireSize ||
        width == 0u || width > 4u) {
        return false;
    }

    const u32 offset = physical - kFirewireBase;
    const u32 aligned = offset & ~3u;
    const u32 byte_offset = offset & 3u;
    if (byte_offset + width > 4u) return false;
    const u32 index = aligned >> 2;
    if (index >= firewire_regs_.size()) return true;

    u32 current = 0;
    (void)read_firewire(kFirewireBase + aligned, 4u, current);
    const u32 bits = width * 8u;
    const u32 lane_mask =
        bits == 32u
            ? 0xFFFFFFFFu
            : ((1u << bits) - 1u) << (byte_offset * 8u);
    const u32 merged =
        (current & ~lane_mask) |
        ((value << (byte_offset * 8u)) & lane_mask);

    switch (aligned) {
    case 0x00u: // Node ID is read-only.
    case 0x7Cu:
        return true;
    case 0x08u: // Control 0: Bus ID reset is self-clearing.
        firewire_regs_[index] = merged & ~0x00800000u;
        return true;
    case 0x10u: // Control 2: expose SCLK ready after initialization.
        firewire_regs_[index] = 0x8u;
        return true;
    case 0x14u: { // PHY access.
        u32 result = merged;
        if ((result & 0x40000000u) != 0) {
            // PHY write completes immediately in the no-device bootstrap model.
            result &= ~0x4000FFFFu;
        } else if ((result & 0x80000000u) != 0) {
            const u32 reg = (result >> 24) & 0xFu;
            result &= ~0x80000000u;
            result = (result & ~0x00000FFFu) | (reg << 8);
        }
        firewire_regs_[index] = result;
        return true;
    }
    case 0x20u: // Interrupt status 0
    case 0x28u: // Interrupt status 1
    case 0x30u: // Interrupt status 2
        firewire_regs_[index] &= ~merged;
        return true;
    case 0x24u: // Interrupt masks are direct writes.
    case 0x2Cu:
    case 0x34u:
    case 0xB8u: // DMA control/status 0
    case 0x138u: // DMA control/status 1
        firewire_regs_[index] = merged;
        return true;
    default:
        firewire_regs_[index] = merged;
        return true;
    }
}

bool IopBus::decode_root_counter(
    u32 physical,
    u32& index,
    u32& reg) {
    static constexpr u32 bases[6] = {
        0x1F801100u, 0x1F801110u, 0x1F801120u,
        0x1F801480u, 0x1F801490u, 0x1F8014A0u,
    };
    for (u32 i = 0; i < 6u; ++i) {
        if (physical >= bases[i] && physical < bases[i] + 0x0Cu) {
            index = i;
            reg = (physical - bases[i]) & ~3u;
            return true;
        }
    }
    return false;
}

u64 IopBus::root_counter_rate(u32 index) const {
    const u32 mode = root_counters_[index].mode;
    if (index == 0u) return (mode & (1u << 8)) != 0 ? 3u : 1u;
    if (index == 1u) return (mode & (1u << 8)) != 0 ? 2344u : 1u;
    if (index == 2u) return (mode & (1u << 9)) != 0 ? 8u : 1u;
    if (index == 3u) return (mode & (1u << 8)) != 0 ? 2344u : 1u;

    switch ((mode >> 13) & 0x3u) {
    case 1: return 8u;
    case 2: return 16u;
    case 3: return 256u;
    default: return 1u;
    }
}

bool IopBus::read_root_counter(
    u32 physical,
    u32 width,
    u32& value) const {
    u32 index = 0;
    u32 reg = 0;
    if (!decode_root_counter(physical, index, reg)) return false;

    static constexpr u32 bases[6] = {
        0x1F801100u, 0x1F801110u, 0x1F801120u,
        0x1F801480u, 0x1F801490u, 0x1F8014A0u,
    };
    const u32 byte_offset = physical - (bases[index] + reg);
    if (byte_offset + width > 4u) return false;

    u32 raw = 0;
    if (reg == 0u) {
        raw = static_cast<u32>(root_counters_[index].count);
    } else if (reg == 4u) {
        raw = root_counters_[index].mode;
    } else if (reg == 8u) {
        raw = static_cast<u32>(root_counters_[index].target);
    } else {
        return false;
    }

    if (index < 3u) raw &= 0xFFFFu;
    const u32 bits = width * 8u;
    const u32 mask =
        bits == 32u ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    value = (raw >> (byte_offset * 8u)) & mask;
    return true;
}

bool IopBus::write_root_counter(
    u32 physical,
    u32 width,
    u32 value) {
    u32 index = 0;
    u32 reg = 0;
    if (!decode_root_counter(physical, index, reg)) return false;

    static constexpr u32 bases[6] = {
        0x1F801100u, 0x1F801110u, 0x1F801120u,
        0x1F801480u, 0x1F801490u, 0x1F8014A0u,
    };
    const u32 byte_offset = physical - (bases[index] + reg);
    if (byte_offset + width > 4u) return false;

    u32 current = 0;
    if (reg == 0u) {
        current = static_cast<u32>(root_counters_[index].count);
    } else if (reg == 4u) {
        current = root_counters_[index].mode;
    } else if (reg == 8u) {
        current = static_cast<u32>(root_counters_[index].target);
    } else {
        return false;
    }

    const u32 bits = width * 8u;
    const u32 lane_mask =
        bits == 32u
            ? 0xFFFFFFFFu
            : ((1u << bits) - 1u) << (byte_offset * 8u);
    const u32 merged =
        (current & ~lane_mask) |
        ((value << (byte_offset * 8u)) & lane_mask);

    RootCounter& counter = root_counters_[index];
    const u64 counter_mask =
        index < 3u ? 0xFFFFull : 0xFFFFFFFFull;

    if (reg == 0u) {
        ++root_counter_debug_.count_writes[index];
        counter.count = merged & counter_mask;
        counter.phase = 0;
        return true;
    }

    if (reg == 4u) {
        ++root_counter_debug_.mode_writes[index];
        root_counter_debug_.last_mode_write[index] =
            merged & 0xFFFFu;

        const u32 old_mode = counter.mode;
        const bool old_flag_had_irq =
            (((old_mode & (1u << 11)) != 0u) &&
             ((old_mode & (1u << 4)) != 0u)) ||
            (((old_mode & (1u << 12)) != 0u) &&
             ((old_mode & (1u << 5)) != 0u));

        // Preserve hardware-owned status/IRQ state. If the old target/overflow
        // flag was raised while its IRQ source was disabled, TIMRMAN's mode
        // write rearms the timer and clears those stale event flags.
        u32 flags = old_mode & 0x1C00u;
        if (!old_flag_had_irq) {
            flags &= ~0x1800u;
        }
        counter.mode =
            (merged & 0x63FFu) | flags | (1u << 10);
        root_counter_rate_cache_[index] =
            static_cast<u32>(root_counter_rate(index));
        counter.count = 0;
        counter.phase = 0;
        counter.target_deferred = false;
        return true;
    }

    ++root_counter_debug_.target_writes[index];
    root_counter_debug_.last_target_write[index] =
        static_cast<u32>(merged & counter_mask);
    if (root_counter_debug_.first_nonzero_target[index] == 0u &&
        (merged & counter_mask) != 0u) {
        root_counter_debug_.first_nonzero_target[index] =
            static_cast<u32>(merged & counter_mask);
    }
    counter.target = merged & counter_mask;
    counter.target_deferred = counter.target <= counter.count;
    return true;
}

void IopBus::tick(u64 cycles) {
    spu2_.tick(cycles);
    ohci_frame_phase_ += cycles;
    while (ohci_frame_phase_ >= kOhciFrameCycles) {
        ohci_frame_phase_ -= kOhciFrameCycles;
        ohci_regs_[15] =
            (ohci_regs_[15] + 1u) & 0xFFFFu;
    }

    static constexpr u32 irq_sources[6] = {
        4u, 5u, 6u, 14u, 15u, 16u,
    };

    for (u32 i = 0; i < root_counters_.size(); ++i) {
        RootCounter& counter = root_counters_[i];
        const u64 rate = root_counter_rate_cache_[i];
        u64 increments = 0;
        if (cycles == 1u) {
            ++counter.phase;
            if (counter.phase >= rate) {
                counter.phase -= rate;
                increments = 1;
            }
        } else {
            counter.phase += cycles;
            increments = counter.phase / rate;
            counter.phase %= rate;
        }
        if (increments == 0) continue;

        const u64 maximum =
            i < 3u ? 0xFFFFull : 0xFFFFFFFFull;

        for (u64 step = 0; step < increments; ++step) {
            ++counter.count;

            if (!counter.target_deferred &&
                counter.target <= maximum &&
                counter.count >= counter.target) {
                ++root_counter_debug_.target_events[i];
                const bool first = (counter.mode & (1u << 11)) == 0;
                const bool repeat = (counter.mode & (1u << 6)) != 0;
                counter.mode |= 1u << 11;
                // The reached-target flag is sticky, but repeat mode can still
                // pulse the interrupt on each real target crossing.
                if ((first || repeat) &&
                    (counter.mode & (1u << 4)) != 0) {
                    ++root_counter_debug_.irq_events[i];
                    intc_.raise(irq_sources[i]);
                }
                if ((counter.mode & (1u << 3)) != 0) {
                    counter.count =
                        counter.target == 0 ? 0 : counter.count - counter.target;
                } else {
                    // Hardware keeps the compare register intact. Internally
                    // defer it until counter wrap, rather than replacing the
                    // guest-visible target with an impossible sentinel value.
                    counter.target_deferred = true;
                }
            }

            if (counter.count > maximum) {
                ++root_counter_debug_.overflow_events[i];
                const bool first = (counter.mode & (1u << 12)) == 0;
                const bool repeat = (counter.mode & (1u << 6)) != 0;
                counter.mode |= 1u << 12;
                if ((first || repeat) &&
                    (counter.mode & (1u << 5)) != 0) {
                    ++root_counter_debug_.irq_events[i];
                    intc_.raise(irq_sources[i]);
                }
                counter.count &= maximum;
                counter.target_deferred = false;
            }
        }
    }

    if (spu2_dma4_irq_cycles_ != 0) {
        if (cycles >= spu2_dma4_irq_cycles_) {
            spu2_dma4_irq_cycles_ = 0;
            intc_.raise(9u);
        } else {
            spu2_dma4_irq_cycles_ -= cycles;
        }
    }
}

u32 IopBus::to_physical(u32 address) {
    if (address >= 0x80000000u && address < 0xC0000000u) {
        return address & 0x1FFFFFFFu;
    }
    return address;
}

u8* IopBus::jit_ram_data() {
    return ram_.data();
}

u32* IopBus::jit_page_generations() {
    return ram_.page_generation_data();
}

u32 IopBus::jit_page_generation(u32 address) const {
    return ram_.page_generation(
        to_physical(address) &
        static_cast<u32>(IopRam::kSize - 1u));
}

void IopBus::jit_track_code_page(u32 address) {
    ram_.track_code_page(
        to_physical(address) &
        static_cast<u32>(IopRam::kSize - 1u));
}

bool IopBus::interrupt_pending() const {
    return intc_.pending();
}

bool IopBus::write_dma_icr(u32 physical, u32 value) {
    if (physical != kDmaIcr && physical != kDmaIcr2) {
        return false;
    }

    u32 current = 0;
    if (!hw_.read32(physical, current)) return false;

    // Lower 24 bits are normal control fields. Bits 24..30 are channel
    // completion flags and acknowledge by writing one. Bit 31 reflects the
    // aggregate interrupt condition.
    u32 next =
        (current & 0xFF000000u) |
        (value & 0x00FFFFFFu);
    next &= ~(value & 0x7F000000u);

    const u32 enables = (next >> 16) & 0x7Fu;
    const u32 flags = (next >> 24) & 0x7Fu;
    const bool force = (next & (1u << 15)) != 0;
    const bool master = (next & (1u << 23)) != 0;
    if (force || (master && (enables & flags) != 0)) {
        next |= 0x80000000u;
    } else {
        next &= ~0x80000000u;
    }

    if (!hw_.write32(physical, next)) return false;
    if (force || (enables & flags) != 0) {
        intc_.raise(3);
    }
    return true;
}

void IopBus::raise_dma_irq(u32 channel) {
    u32 address = 0;
    u32 index = 0;
    bool sif_always_routes = false;

    if (channel <= 6u) {
        address = kDmaIcr;
        index = channel;
    } else if (channel <= 12u) {
        address = kDmaIcr2;
        index = channel - 7u;
        // IOP DMA9/10 are SIF0/SIF1. Retail firmware expects their DMA
        // completion to reach INTC source 3 even during early DICR2 setup.
        sif_always_routes = channel == 9u || channel == 10u;
    } else {
        return;
    }

    u32 current = 0;
    if (!hw_.read32(address, current)) return;

    current |= 1u << (24u + index);

    const bool enabled =
        (current & (1u << (16u + index))) != 0;
    const bool master =
        (current & (1u << 23)) != 0;
    const bool force =
        (current & (1u << 15)) != 0;

    if (force || (master && enabled)) {
        current |= 0x80000000u;
    } else {
        current &= ~0x80000000u;
    }

    (void)hw_.write32(address, current);
    if (sif_always_routes || force || enabled) {
        intc_.raise(3);
    }
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

u16 IopBus::sif_dma_ready_mask() const {
    return hw_.sif_dma_ready_mask();
}

bool IopBus::can_tick_event_free(u64 cycles) const {
    if (spu2_dma4_irq_cycles_ != 0u &&
        spu2_dma4_irq_cycles_ <= cycles) return false;
    for (u32 i = 0; i < root_counters_.size(); ++i) {
        const RootCounter& counter = root_counters_[i];
        const u64 rate = root_counter_rate_cache_[i];
        const u64 increments = (counter.phase + cycles) / rate;
        if (increments == 0u) continue;
        const u64 maximum = i < 3u ? 0xFFFFull : 0xFFFFFFFFull;
        if (counter.count + increments >= counter.target ||
            counter.count + increments > maximum) return false;
    }
    return true;
}

bool IopBus::tick_event_free(u64 cycles) {
    if (!can_tick_event_free(cycles)) return false;

    spu2_.tick(cycles);
    const u64 frame_total = ohci_frame_phase_ + cycles;
    ohci_regs_[15] = (ohci_regs_[15] +
        static_cast<u32>(frame_total / kOhciFrameCycles)) & 0xFFFFu;
    ohci_frame_phase_ = frame_total % kOhciFrameCycles;
    for (u32 i = 0; i < root_counters_.size(); ++i) {
        RootCounter& counter = root_counters_[i];
        const u64 rate = root_counter_rate_cache_[i];
        const u64 phase_total = counter.phase + cycles;
        counter.count += phase_total / rate;
        counter.phase = phase_total % rate;
    }
    if (spu2_dma4_irq_cycles_ != 0u) {
        spu2_dma4_irq_cycles_ -= cycles;
    }
    return true;
}

bool IopBus::read8(u32 address, u8& value) const {
    if (address >= kCacheControlBase && address < kCacheControlEnd) {
        value = cache_control_[address - kCacheControlBase];
        return true;
    }
    const u32 physical = to_physical(address);
    if (physical < kRamMirrorEnd)
        return ram_.read8(physical & static_cast<u32>(IopRam::kSize - 1u), value);
    if (physical >= kDev9Base &&
        physical < kDev9Base + kDev9Size) {
        // No expansion-bay adapter is attached. Real IOP mappings still
        // expose the DEV9 aperture and return zero for absent hardware.
        value = 0;
        return true;
    }
    u32 ohci_value = 0;
    if (read_ohci(physical, 1u, ohci_value)) {
        value = static_cast<u8>(ohci_value);
        return true;
    }
    u32 fw_value = 0;
    if (read_firewire(physical, 1u, fw_value)) {
        value = static_cast<u8>(fw_value);
        return true;
    }
    u32 timer_value = 0;
    if (read_root_counter(physical, 1u, timer_value)) {
        value = static_cast<u8>(timer_value);
        return true;
    }
    if (physical >= kSpu2Base && physical < kSpu2Base + kSpu2Size) {
        return spu2_.read8(physical - kSpu2Base, value);
    }
    if (is_extension_rom(physical, 1u)) {
        value = 0;
        return true;
    }
    if (physical == kSio2Rx) {
        value = sio2_.read_byte();
        return true;
    }
    if (intc_.read8(physical, value)) return true;
    if (cdvd_.read8(canonical_cdvd_address(physical), value)) return true;
    if (hw_.read8(physical, value)) return true;
    if (physical >= kSifBase && physical < kSifBase + 0x100u) {
        u32 word = 0;
        if (!read_sif32(physical & ~3u, word)) return false;
        value = static_cast<u8>(word >> ((physical & 3u) * 8));
        return true;
    }
    if (bios_.read8_physical(physical, value)) return true;
    if (is_iop_peripheral_open_bus(physical)) {
        value = 0;
        return true;
    }
    return false;
}

bool IopBus::read16(u32 address, u16& value) const {
    const u32 physical = to_physical(address);
    if (physical <= kRamMirrorEnd - 2u) {
        const u32 offset = physical & static_cast<u32>(IopRam::kSize - 1u);
        if (offset <= IopRam::kSize - 2u)
            return ram_.read16(offset, value);
    }
    if (physical >= kDev9Base &&
        physical + 2u <= kDev9Base + kDev9Size) {
        value = 0;
        return true;
    }
    u32 ohci_value = 0;
    if (read_ohci(physical, 2u, ohci_value)) {
        value = static_cast<u16>(ohci_value);
        return true;
    }
    u32 fw_value = 0;
    if (read_firewire(physical, 2u, fw_value)) {
        value = static_cast<u16>(fw_value);
        return true;
    }
    u32 timer_value = 0;
    if (read_root_counter(physical, 2u, timer_value)) {
        value = static_cast<u16>(timer_value);
        return true;
    }
    if (physical >= kSpu2Base && physical + 2u <= kSpu2Base + kSpu2Size) {
        return spu2_.read16(physical - kSpu2Base, value);
    }
    if (intc_.read16(physical, value)) return true;
    if (is_extension_rom(physical, 2u)) {
        value = 0;
        return true;
    }
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
    if (physical <= kRamMirrorEnd - 4u) {
        const u32 offset = physical & static_cast<u32>(IopRam::kSize - 1u);
        if (offset <= IopRam::kSize - 4u)
            return ram_.read32(offset, value);
    }
    if (physical >= kDev9Base &&
        physical + 4u <= kDev9Base + kDev9Size) {
        value = 0;
        return true;
    }
    u32 ohci_value = 0;
    if (read_ohci(physical, 4u, ohci_value)) {
        value = ohci_value;
        return true;
    }
    u32 fw_value = 0;
    if (read_firewire(physical, 4u, fw_value)) {
        value = fw_value;
        return true;
    }
    u32 timer_value = 0;
    if (read_root_counter(physical, 4u, timer_value)) {
        value = timer_value;
        return true;
    }
    if (physical >= kSpu2Base && physical + 4u <= kSpu2Base + kSpu2Size) {
        return spu2_.read32(physical - kSpu2Base, value);
    }
    if (physical >= kSio2CmdBase && physical < kSio2CmdEnd &&
        (physical & 3u) == 0u) {
        value = sio2_.command((physical - kSio2CmdBase) >> 2);
        return true;
    }
    if (physical == kSio2Rx) {
        value = static_cast<u32>(sio2_.read_byte()) |
                (static_cast<u32>(sio2_.read_byte()) << 8) |
                (static_cast<u32>(sio2_.read_byte()) << 16) |
                (static_cast<u32>(sio2_.read_byte()) << 24);
        return true;
    }
    if (physical == kSio2CmdStat) { value = sio2_.cmd_stat(); return true; }
    if (physical == kSio2PortStat) { value = sio2_.port_stat(); return true; }
    if (physical == kSio2FifoStat) { value = sio2_.fifo_stat(); return true; }
    if (physical == kSio2TxPos) { value = sio2_.tx_pos(); return true; }
    if (physical == kSio2RxPos) { value = sio2_.rx_pos(); return true; }
    if (physical == kSio2Intr) { value = sio2_.intr(); return true; }
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
    if (is_extension_rom(physical, 4u)) {
        value = 0;
        return true;
    }
    if (cdvd_.read32(canonical_cdvd_address(physical), value)) return true;
    if (hw_.read32(physical, value)) return true;
    if (bios_.read32_physical(physical, value)) return true;
    if (is_iop_peripheral_open_bus(physical)) {
        value = 0;
        return true;
    }
    return false;
}

bool IopBus::write8(u32 address, u8 value) {
    if (address >= kCacheControlBase && address < kCacheControlEnd) {
        cache_control_[address-kCacheControlBase]=value; return true;
    }
    const u32 physical=to_physical(address);
    if (physical < kRamMirrorEnd)
        return ram_.write8(physical & static_cast<u32>(IopRam::kSize - 1u), value);
    if (physical >= kDev9Base &&
        physical < kDev9Base + kDev9Size) {
        return true;
    }
    if (physical >= kOhciBase &&
        physical < kOhciBase + kOhciSize) {
        return write_ohci(physical, 1u, value);
    }
    if (physical >= kFirewireBase &&
        physical < kFirewireBase + kFirewireSize) {
        return write_firewire(physical, 1u, value);
    }
    u32 timer_index = 0;
    u32 timer_reg = 0;
    if (decode_root_counter(physical, timer_index, timer_reg)) {
        return write_root_counter(physical, 1u, value);
    }
    if (physical >= kSpu2Base && physical < kSpu2Base + kSpu2Size) {
        return spu2_.write8(physical - kSpu2Base, value);
    }
    if (physical == kSio2Tx) {
        sio2_.write_byte(value);
        return true;
    }
    if (is_extension_rom(physical, 1u)) return true;
    if (intc_.write8(physical,value)) return true;
    if (cdvd_.write8(canonical_cdvd_address(physical),value)) return true;
    if (hw_.write8(physical,value)) return true;
    if (physical >= kSifBase && physical < kSifBase+0x100u) {
        u32 word=0; if(!read_sif32(physical&~3u,word)) return false;
        const u32 shift=(physical&3u)*8;
        word=(word&~(0xFFu<<shift))|(static_cast<u32>(value)<<shift);
        return write_sif32(physical&~3u,word);
    }
    if (is_iop_peripheral_open_bus(physical)) return true;
    return false;
}

bool IopBus::write16(u32 address, u16 value) {
    const u32 physical=to_physical(address);
    if (physical <= kRamMirrorEnd - 2u) {
        const u32 offset = physical & static_cast<u32>(IopRam::kSize - 1u);
        if (offset <= IopRam::kSize - 2u)
            return ram_.write16(offset, value);
    }
    if (physical >= kDev9Base &&
        physical + 2u <= kDev9Base + kDev9Size) {
        return true;
    }
    if (physical >= kOhciBase &&
        physical < kOhciBase + kOhciSize) {
        return write_ohci(physical, 2u, value);
    }
    if (physical >= kFirewireBase &&
        physical < kFirewireBase + kFirewireSize) {
        return write_firewire(physical, 2u, value);
    }
    u32 timer_index = 0;
    u32 timer_reg = 0;
    if (decode_root_counter(physical, timer_index, timer_reg)) {
        return write_root_counter(physical, 2u, value);
    }
    if (physical >= kSpu2Base && physical + 2u <= kSpu2Base + kSpu2Size) {
        return spu2_.write16(physical - kSpu2Base, value);
    }
    if (physical == 0x1F801450u) {
        if (!hw_.write16(physical, value)) return false;
        if ((value & 0x2u) != 0) ee_hw_.raise_intc(1);
        return true;
    }
    if (intc_.write16(physical,value)) return true;
    if (is_extension_rom(physical, 2u)) return true;
    return write8(address,static_cast<u8>(value)) && write8(address+1,static_cast<u8>(value>>8));
}

bool IopBus::write32(u32 address, u32 value) {
    if (address >= kCacheControlBase && address + 4 <= kCacheControlEnd) {
        const u32 offset=address-kCacheControlBase;
        for(u32 i=0;i<4;++i) cache_control_[offset+i]=static_cast<u8>(value>>(i*8));
        return true;
    }
    const u32 physical=to_physical(address);
    if (physical <= kRamMirrorEnd - 4u) {
        const u32 offset = physical & static_cast<u32>(IopRam::kSize - 1u);
        if (offset <= IopRam::kSize - 4u)
            return ram_.write32(offset, value);
    }
    if (physical >= kDev9Base &&
        physical + 4u <= kDev9Base + kDev9Size) {
        return true;
    }
    if (physical >= kOhciBase &&
        physical < kOhciBase + kOhciSize) {
        return write_ohci(physical, 4u, value);
    }
    if (physical >= kFirewireBase &&
        physical < kFirewireBase + kFirewireSize) {
        return write_firewire(physical, 4u, value);
    }
    u32 timer_index = 0;
    u32 timer_reg = 0;
    if (decode_root_counter(physical, timer_index, timer_reg)) {
        return write_root_counter(physical, 4u, value);
    }
    if (physical == kDmaIcr || physical == kDmaIcr2) {
        return write_dma_icr(physical, value);
    }
    if (physical == kDma6Chcr) {
        u32 stored = value;
        if ((value & kDmaStart) != 0) {
            u32 madr = 0;
            u32 bcr = 0;
            if (!hw_.read32(kDma6Madr, madr) ||
                !hw_.read32(kDma6Bcr, bcr)) {
                return false;
            }

            // IOP DMA6 is the PS1-compatible ordering-table clear channel.
            // The BIOS-visible transfer walks backward through IOP RAM,
            // linking each word to the previous address and terminating with
            // 0x00FFFFFF. PCSX2 accepts this canonical CHCR value as the
            // hardware OTC operation as well.
            if ((value & 0x11000003u) == 0x11000002u) {
                const u32 words =
                    std::min<u32>(
                        bcr,
                        static_cast<u32>(IopRam::kSize / 4u));
                u32 address =
                    madr & static_cast<u32>(IopRam::kSize - 1u);
                for (u32 i = 0; i < words; ++i) {
                    const u32 link =
                        i + 1u == words
                            ? 0x00FFFFFFu
                            : ((address - 4u) & 0x00FFFFFFu);
                    if (!ram_.write32(address, link)) return false;
                    address =
                        (address - 4u) &
                        static_cast<u32>(IopRam::kSize - 1u);
                }
                (void)hw_.write32(kDma6Madr, address);
                (void)hw_.write32(kDma6Bcr, 0u);
            }

            stored &= ~kDmaStart;
        }

        if (!hw_.write32(kDma6Chcr, stored)) return false;
        if ((value & kDmaStart) != 0) {
            raise_dma_irq(6u);
        }
        return true;
    }

    if (physical >= kSio2CmdBase && physical < kSio2CmdEnd &&
        (physical & 3u) == 0u) {
        sio2_.set_command(
            (physical - kSio2CmdBase) >> 2,
            value);
        return hw_.write32(physical, value);
    }
    if (physical == kSio2Tx) {
        sio2_.write_byte(static_cast<u8>(value));
        sio2_.write_byte(static_cast<u8>(value >> 8));
        sio2_.write_byte(static_cast<u8>(value >> 16));
        sio2_.write_byte(static_cast<u8>(value >> 24));
        return true;
    }
    if (physical == kSio2Ctrl) {
        if (!hw_.write32(physical, value)) return false;
        if ((value & kSio2Start) != 0u) {
            sio2_.start_transfer();
            (void)hw_.write32(kSio2CmdStat, sio2_.cmd_stat());
            (void)hw_.write32(kSio2Intr, sio2_.intr());
            intc_.raise(17u);
        }
        return true;
    }
    if (physical == kSio2Intr) {
        sio2_.acknowledge_intr(value);
        return hw_.write32(kSio2Intr, sio2_.intr());
    }

    if (physical == kDma4Chcr || physical == kDma7Chcr) {
        const u32 core = physical == kDma4Chcr ? 0u : 1u;
        const u32 channel = core == 0u ? 4u : 7u;
        const u32 madr_reg = core == 0u ? kDma4Madr : kDma7Madr;
        const u32 bcr_reg = core == 0u ? kDma4Bcr : kDma7Bcr;

        u32 stored = value;
        if ((value & kDmaStart) != 0u) {
            u32 madr = 0;
            u32 bcr = 0;
            if (!hw_.read32(madr_reg, madr) ||
                !hw_.read32(bcr_reg, bcr)) {
                return false;
            }
            const u32 words =
                (bcr & 0xFFFFu) * (bcr >> 16);
            const u32 halfwords = words * 2u;

            const bool ok = (value & 1u) != 0u
                ? spu2_.dma_write(core, ram_, madr, halfwords)
                : spu2_.dma_read(core, ram_, madr, halfwords);
            if (!ok) return false;

            (void)hw_.write32(
                madr_reg,
                madr + words * 4u);

            u16 stat = 0;
            const u32 stat_offset =
                core == 0u ? kSpu2Statx0 : kSpu2Statx1;
            (void)spu2_.read16(stat_offset, stat);
            stat = static_cast<u16>((stat | 0x0080u) & ~0x0400u);
            (void)spu2_.write16(stat_offset, stat);
            (void)spu2_.write16(
                core == 0u ? 0x1B0u : 0x5B0u,
                0u);

            stored &= ~kDmaStart;
            raise_dma_irq(channel);
            if (core == 0u) {
                spu2_dma4_irq_cycles_ =
                    words == 0u ? 48u :
                    static_cast<u64>(words) * 48u;
            }
        }
        return hw_.write32(physical, stored);
    }

    if (physical == kDma11Chcr || physical == kDma12Chcr) {
        const bool tx = physical == kDma11Chcr;
        const u32 channel = tx ? 11u : 12u;
        const u32 madr_reg = tx ? kDma11Madr : kDma12Madr;
        const u32 bcr_reg = tx ? kDma11Bcr : kDma12Bcr;

        u32 stored = value;
        if ((value & kDmaStart) != 0u) {
            u32 madr = 0;
            u32 bcr = 0;
            if (!hw_.read32(madr_reg, madr) ||
                !hw_.read32(bcr_reg, bcr)) {
                return false;
            }

            const u32 block_words = bcr & 0xFFFFu;
            const u32 blocks = bcr >> 16;
            const u32 words = block_words * blocks;
            const u32 bytes = words * 4u;
            sio2_.set_dma_block_size(
                static_cast<std::size_t>(block_words) * 4u);

            for (u32 i = 0; i < bytes; ++i) {
                const u32 ram_addr =
                    (madr + i) &
                    static_cast<u32>(IopRam::kSize - 1u);
                if (tx) {
                    u8 byte = 0;
                    if (!ram_.read8(ram_addr, byte)) return false;
                    sio2_.write_byte(byte);
                } else {
                    if (!ram_.write8(ram_addr, sio2_.read_byte()))
                        return false;
                }
            }

            sio2_.clear_dma_block_size();
            (void)hw_.write32(madr_reg, madr + bytes);
            stored &= ~kDmaStart;
            raise_dma_irq(channel);
        }
        return hw_.write32(physical, stored);
    }
    if (physical >= kSpu2Base && physical + 4u <= kSpu2Base + kSpu2Size) {
        return spu2_.write32(physical - kSpu2Base, value);
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
    if (is_extension_rom(physical, 4u)) return true;
    if (cdvd_.write32(canonical_cdvd_address(physical),value)) return true;
    if (hw_.write32(physical,value)) return true;
    if (is_iop_peripheral_open_bus(physical)) return true;
    return false;
}

} // namespace ps2
