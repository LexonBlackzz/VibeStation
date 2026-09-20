#include "core/dma/gif_dma.h"

#include "core/gs/gs_core.h"
#include "core/memory/ee_bus.h"

namespace ps2 {
namespace {

constexpr u32 kGifChcr = 0x1000A000u;
constexpr u32 kGifMadr = 0x1000A010u;
constexpr u32 kGifQwc = 0x1000A020u;
constexpr u32 kGifTadr = 0x1000A030u;
constexpr u32 kGifAsr0 = 0x1000A040u;
constexpr u32 kGifAsr1 = 0x1000A050u;
constexpr u32 kDmacCtrl = 0x1000E000u;

constexpr u32 kChcrTte = 1u << 6;
constexpr u32 kChcrTie = 1u << 7;
constexpr u32 kChcrStr = 1u << 8;

constexpr u32 kModeNormal = 0;
constexpr u32 kModeChain = 1;

u32 apply_spr(u32 address, bool spr) {
    if (!spr) return address & 0x7FFFFFF0u;
    return 0x70000000u | (address & 0x3FF0u);
}

} // namespace

void GifDma::reset() {
    end_after_qwc_ = false;
}

bool GifDma::complete(EeBus& bus, u32 chcr) {
    end_after_qwc_ = false;
    chcr &= ~kChcrStr;
    if (!bus.write32(kGifChcr, chcr)) return false;
    if (!bus.write32(kGifQwc, 0)) return false;
    bus.raise_dmac(2);
    return true;
}

bool GifDma::service(EeBus& bus, GsCore& gs, std::string& error) {
    error.clear();

    u32 ctrl = 0;
    u32 chcr = 0;
    if (!bus.read32(kDmacCtrl, ctrl) || !bus.read32(kGifChcr, chcr)) {
        error = "GIF DMA register read failed";
        return false;
    }

    if ((ctrl & 1u) == 0 || (chcr & kChcrStr) == 0) {
        return true;
    }

    const u32 mode = (chcr >> 2) & 0x3u;
    if (mode != kModeNormal && mode != kModeChain) {
        // Interleave is not valid for GIF path 3. Leave the channel armed so
        // software can observe its own programming error instead of completing
        // a transfer that the hardware would not perform.
        return true;
    }

    u32 qwc = 0;
    u32 madr = 0;
    u32 tadr = 0;
    if (!bus.read32(kGifQwc, qwc) ||
        !bus.read32(kGifMadr, madr) ||
        !bus.read32(kGifTadr, tadr)) {
        error = "GIF DMA channel state read failed";
        return false;
    }
    qwc &= 0xFFFFu;

    if (mode == kModeNormal && qwc == 0) {
        if (!complete(bus, chcr)) {
            error = "GIF DMA completion write failed";
            return false;
        }
        return true;
    }

    // Source-chain tags with QWC=0 are legal. Walk a few in one service call
    // so an empty CNT/NEXT sequence cannot stall the whole machine.
    for (u32 guard = 0; mode == kModeChain && qwc == 0 && guard < 8; ++guard) {
        if (end_after_qwc_) {
            if (!complete(bus, chcr)) {
                error = "GIF DMA chain completion write failed";
                return false;
            }
            return true;
        }

        u64 tag_lo = 0;
        u64 tag_hi = 0;
        const u32 tag_address = tadr & 0x7FFFFFF0u;
        if (!bus.read64(tag_address, tag_lo) ||
            !bus.read64(tag_address + 8u, tag_hi)) {
            error = "GIF DMA tag fetch fault";
            return false;
        }

        const u32 tag0 = static_cast<u32>(tag_lo);
        const u32 tag1 = static_cast<u32>(tag_lo >> 32);
        qwc = tag0 & 0xFFFFu;
        const u32 id = (tag0 >> 28) & 0x7u;
        const bool irq = (tag0 & 0x80000000u) != 0;
        const bool spr = (tag1 & 0x80000000u) != 0;
        const u32 address = apply_spr(tag1, spr);

        chcr = (chcr & 0x0000FFFFu) | (tag0 & 0xFFFF0000u);
        end_after_qwc_ = irq && ((chcr & kChcrTie) != 0);

        switch (id) {
        case 0: // REFE
            madr = address;
            tadr = tag_address + 16u;
            end_after_qwc_ = true;
            break;
        case 1: // CNT
            madr = tag_address + 16u;
            tadr = madr + qwc * 16u;
            break;
        case 2: // NEXT
            madr = tag_address + 16u;
            tadr = address;
            break;
        case 3: // REF
        case 4: // REFS
            madr = address;
            tadr = tag_address + 16u;
            break;
        case 5: { // CALL
            madr = tag_address + 16u;
            const u32 return_address = madr + qwc * 16u;
            u32 asp = (chcr >> 4) & 0x3u;
            if (asp == 0) {
                if (!bus.write32(kGifAsr0, return_address)) {
                    error = "GIF DMA ASR0 write failed";
                    return false;
                }
                asp = 1;
            } else if (asp == 1) {
                if (!bus.write32(kGifAsr1, return_address)) {
                    error = "GIF DMA ASR1 write failed";
                    return false;
                }
                asp = 2;
            } else {
                end_after_qwc_ = true;
            }
            chcr = (chcr & ~(0x3u << 4)) | (asp << 4);
            tadr = address;
            break;
        }
        case 6: { // RET
            madr = tag_address + 16u;
            u32 asp = (chcr >> 4) & 0x3u;
            if (asp == 2) {
                if (!bus.read32(kGifAsr1, tadr)) {
                    error = "GIF DMA ASR1 read failed";
                    return false;
                }
                if (!bus.write32(kGifAsr1, 0)) {
                    error = "GIF DMA ASR1 clear failed";
                    return false;
                }
                asp = 1;
            } else if (asp == 1) {
                if (!bus.read32(kGifAsr0, tadr)) {
                    error = "GIF DMA ASR0 read failed";
                    return false;
                }
                if (!bus.write32(kGifAsr0, 0)) {
                    error = "GIF DMA ASR0 clear failed";
                    return false;
                }
                asp = 0;
            } else {
                end_after_qwc_ = true;
            }
            chcr = (chcr & ~(0x3u << 4)) | (asp << 4);
            break;
        }
        case 7: // END
            madr = tag_address + 16u;
            end_after_qwc_ = true;
            break;
        }

        if (!bus.write32(kGifChcr, chcr) ||
            !bus.write32(kGifMadr, madr) ||
            !bus.write32(kGifQwc, qwc) ||
            !bus.write32(kGifTadr, tadr)) {
            error = "GIF DMA tag state write failed";
            return false;
        }

        // TTE forwards the upper 64 bits of the DMA tag onto path 3. Model it
        // as one padded qword; this is sufficient for bootstrap chain tags.
        if ((chcr & kChcrTte) != 0) {
            gs.write_gif_qword(tag_hi, 0);
        }

        if (qwc == 0 && end_after_qwc_) {
            if (!complete(bus, chcr)) {
                error = "GIF DMA empty chain completion failed";
                return false;
            }
            return true;
        }
    }

    if (qwc == 0) {
        return true;
    }

    u64 lo = 0;
    u64 hi = 0;
    if (!bus.read64(madr, lo) || !bus.read64(madr + 8u, hi)) {
        error = "GIF DMA payload fetch fault";
        return false;
    }

    gs.write_gif_qword(lo, hi);
    madr += 16u;
    --qwc;

    if (!bus.write32(kGifMadr, madr) || !bus.write32(kGifQwc, qwc)) {
        error = "GIF DMA progress write failed";
        return false;
    }

    if (qwc == 0 && (mode == kModeNormal || end_after_qwc_)) {
        if (!complete(bus, chcr)) {
            error = "GIF DMA completion failed";
            return false;
        }
    }

    return true;
}

} // namespace ps2
