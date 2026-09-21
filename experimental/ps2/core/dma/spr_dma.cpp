#include "core/dma/spr_dma.h"

#include "core/memory/ee_bus.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u32 kDmacCtrl = 0x1000E000u;
constexpr u32 kSprFromBase = 0x1000D000u;
constexpr u32 kSprToBase = 0x1000D400u;
constexpr u32 kChcr = 0x00u;
constexpr u32 kMadr = 0x10u;
constexpr u32 kQwc = 0x20u;
constexpr u32 kTadr = 0x30u;
constexpr u32 kSadr = 0x80u;
constexpr u32 kStr = 1u << 8;
constexpr u32 kTte = 1u << 6;
constexpr u32 kTie = 1u << 7;
constexpr u32 kModeNormal = 0u;
constexpr u32 kModeChain = 1u;
constexpr u32 kModeInterleave = 2u;
constexpr u32 kScratchpadBase = 0x70000000u;
constexpr u32 kScratchpadMask = 0x3FFFu;

u32 bus_address(u32 value) {
    if ((value & 0x80000000u) != 0)
        return kScratchpadBase | (value & kScratchpadMask);
    return value & 0x7FFFFFF0u;
}

u32 scratchpad_address(u32 sadr) {
    return kScratchpadBase | (sadr & kScratchpadMask);
}

} // namespace

void SprDma::reset() {
    from_ = {};
    to_ = {};
}

bool SprDma::transfer_from_spr(
    EeBus& bus, u32& madr, u32& qwc, u32& sadr,
    std::string& error) {
    const u32 chunk = std::min(qwc, 0x400u);
    for (u32 q = 0; q < chunk; ++q) {
        u64 lo = 0;
        u64 hi = 0;
        const u32 src = scratchpad_address(sadr + q * 16u);
        const u32 dst = bus_address(madr + q * 16u);
        if (!bus.read64(src, lo) || !bus.read64(src + 8u, hi)) {
            error = "SPR-from scratchpad read fault";
            return false;
        }
        if (!bus.write64(dst, lo) || !bus.write64(dst + 8u, hi)) {
            error = "SPR-from destination write fault";
            return false;
        }
    }
    madr += chunk * 16u;
    sadr = (sadr + chunk * 16u) & kScratchpadMask;
    qwc -= chunk;
    return true;
}

bool SprDma::transfer_to_spr(
    EeBus& bus, u32& madr, u32& qwc, u32& sadr,
    std::string& error) {
    const u32 chunk = std::min(qwc, 0x400u);
    for (u32 q = 0; q < chunk; ++q) {
        u64 lo = 0;
        u64 hi = 0;
        const u32 src = bus_address(madr + q * 16u);
        const u32 dst = scratchpad_address(sadr + q * 16u);
        if (!bus.read64(src, lo) || !bus.read64(src + 8u, hi)) {
            error = "SPR-to source read fault";
            return false;
        }
        if (!bus.write64(dst, lo) || !bus.write64(dst + 8u, hi)) {
            error = "SPR-to scratchpad write fault";
            return false;
        }
    }
    madr += chunk * 16u;
    sadr = (sadr + chunk * 16u) & kScratchpadMask;
    qwc -= chunk;
    return true;
}

bool SprDma::service_from_spr(EeBus& bus, std::string& error) {
    u32 chcr = 0;
    if (!bus.read32(kSprFromBase + kChcr, chcr)) {
        error = "SPR-from CHCR read failed";
        return false;
    }
    if ((chcr & kStr) == 0) return true;

    u32 madr = 0;
    u32 qwc = 0;
    u32 sadr = 0;
    if (!bus.read32(kSprFromBase + kMadr, madr) ||
        !bus.read32(kSprFromBase + kQwc, qwc) ||
        !bus.read32(kSprFromBase + kSadr, sadr)) {
        error = "SPR-from channel state read failed";
        return false;
    }
    qwc &= 0xFFFFu;
    sadr &= kScratchpadMask;

    const u32 mode = (chcr >> 2) & 0x3u;
    if (mode == kModeNormal || mode == kModeInterleave) {
        if (qwc != 0 &&
            !transfer_from_spr(bus, madr, qwc, sadr, error))
            return false;
        if (qwc == 0) from_.end_after_qwc = true;
    } else if (mode == kModeChain) {
        if (qwc == 0 && !from_.end_after_qwc) {
            u64 tag_lo = 0;
            u64 tag_hi = 0;
            const u32 tag = scratchpad_address(sadr);
            if (!bus.read64(tag, tag_lo) || !bus.read64(tag + 8u, tag_hi)) {
                error = "SPR-from destination tag read fault";
                return false;
            }
            (void)tag_hi;
            const u32 tag0 = static_cast<u32>(tag_lo);
            const u32 tag1 = static_cast<u32>(tag_lo >> 32);
            qwc = tag0 & 0xFFFFu;
            const u32 id = (tag0 >> 28) & 0x7u;
            const bool irq = (tag0 & 0x80000000u) != 0;
            sadr = (sadr + 16u) & kScratchpadMask;
            madr = bus_address(tag1);
            chcr =
                (chcr & 0x0000FFFFu) |
                (tag0 & 0xFFFF0000u);
            from_.end_after_qwc =
                id == 7u ||
                (id != 0u && id != 1u) ||
                (irq && (chcr & kTie) != 0);
        }
        if (qwc != 0 &&
            !transfer_from_spr(bus, madr, qwc, sadr, error))
            return false;
    } else {
        return true;
    }

    if (!bus.write32(kSprFromBase + kMadr, madr) ||
        !bus.write32(kSprFromBase + kQwc, qwc) ||
        !bus.write32(kSprFromBase + kSadr, sadr) ||
        !bus.write32(kSprFromBase + kChcr, chcr)) {
        error = "SPR-from progress write failed";
        return false;
    }

    if (qwc == 0 && from_.end_after_qwc) {
        from_.end_after_qwc = false;
        if (!bus.write32(kSprFromBase + kChcr, chcr & ~kStr)) {
            error = "SPR-from completion write failed";
            return false;
        }
        bus.raise_dmac(8u);
    }
    return true;
}

bool SprDma::service_to_spr(EeBus& bus, std::string& error) {
    u32 chcr = 0;
    if (!bus.read32(kSprToBase + kChcr, chcr)) {
        error = "SPR-to CHCR read failed";
        return false;
    }
    if ((chcr & kStr) == 0) return true;

    u32 madr = 0;
    u32 qwc = 0;
    u32 tadr = 0;
    u32 sadr = 0;
    if (!bus.read32(kSprToBase + kMadr, madr) ||
        !bus.read32(kSprToBase + kQwc, qwc) ||
        !bus.read32(kSprToBase + kTadr, tadr) ||
        !bus.read32(kSprToBase + kSadr, sadr)) {
        error = "SPR-to channel state read failed";
        return false;
    }
    qwc &= 0xFFFFu;
    sadr &= kScratchpadMask;

    const u32 mode = (chcr >> 2) & 0x3u;
    if (mode == kModeNormal || mode == kModeInterleave) {
        if (qwc != 0 &&
            !transfer_to_spr(bus, madr, qwc, sadr, error))
            return false;
        if (qwc == 0) to_.end_after_qwc = true;
    } else if (mode == kModeChain) {
        if (qwc == 0 && !to_.end_after_qwc) {
            u64 tag_lo = 0;
            u64 tag_hi = 0;
            const u32 tag = bus_address(tadr);
            if (!bus.read64(tag, tag_lo) || !bus.read64(tag + 8u, tag_hi)) {
                error = "SPR-to source tag read fault";
                return false;
            }
            const u32 tag0 = static_cast<u32>(tag_lo);
            const u32 tag1 = static_cast<u32>(tag_lo >> 32);
            qwc = tag0 & 0xFFFFu;
            const u32 id = (tag0 >> 28) & 0x7u;
            const bool irq = (tag0 & 0x80000000u) != 0;
            const u32 address = bus_address(tag1);
            chcr =
                (chcr & 0x0000FFFFu) |
                (tag0 & 0xFFFF0000u);
            to_.end_after_qwc =
                id == 0u || id == 7u ||
                (irq && (chcr & kTie) != 0);

            switch (id) {
            case 0: // REFE
                madr = address;
                tadr = tag + 16u;
                break;
            case 1: // CNT
                madr = tag + 16u;
                tadr = madr + qwc * 16u;
                break;
            case 2: // NEXT
                madr = tag + 16u;
                tadr = address;
                break;
            case 3: // REF
            case 4: // REFS
                madr = address;
                tadr = tag + 16u;
                break;
            case 5: // CALL
            case 6: // RET
                // Nested SPR chains are outside the BIOS bootstrap path.
                madr = tag + 16u;
                tadr = address;
                to_.end_after_qwc = true;
                break;
            case 7: // END
                madr = tag + 16u;
                tadr = madr + qwc * 16u;
                break;
            }

            if ((chcr & kTte) != 0) {
                const u32 dst = scratchpad_address(sadr);
                if (!bus.write64(dst, tag_lo) || !bus.write64(dst + 8u, tag_hi)) {
                    error = "SPR-to TTE tag write fault";
                    return false;
                }
                sadr = (sadr + 16u) & kScratchpadMask;
            }
        }
        if (qwc != 0 &&
            !transfer_to_spr(bus, madr, qwc, sadr, error))
            return false;
    } else {
        return true;
    }

    if (!bus.write32(kSprToBase + kMadr, madr) ||
        !bus.write32(kSprToBase + kQwc, qwc) ||
        !bus.write32(kSprToBase + kTadr, tadr) ||
        !bus.write32(kSprToBase + kSadr, sadr) ||
        !bus.write32(kSprToBase + kChcr, chcr)) {
        error = "SPR-to progress write failed";
        return false;
    }

    if (qwc == 0 && to_.end_after_qwc) {
        to_.end_after_qwc = false;
        if (!bus.write32(kSprToBase + kChcr, chcr & ~kStr)) {
            error = "SPR-to completion write failed";
            return false;
        }
        bus.raise_dmac(9u);
    }
    return true;
}

bool SprDma::service(EeBus& bus, std::string& error) {
    error.clear();
    u32 ctrl = 0;
    if (!bus.read32(kDmacCtrl, ctrl)) {
        error = "SPR DMAC CTRL read failed";
        return false;
    }
    if ((ctrl & 1u) == 0) return true;
    if (!service_from_spr(bus, error)) return false;
    return service_to_spr(bus, error);
}

} // namespace ps2
