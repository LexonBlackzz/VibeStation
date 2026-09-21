#include "core/dma/sif_dma.h"

#include "core/iop/iop_bus.h"
#include "core/iop/iop_intc.h"
#include "core/memory/ee_bus.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u32 kDmacCtrl = 0x1000E000u;

constexpr u32 kEeSif0 = 0x1000C000u;
constexpr u32 kEeSif1 = 0x1000C400u;
constexpr u32 kEeChcr = 0x00u;
constexpr u32 kEeMadr = 0x10u;
constexpr u32 kEeQwc  = 0x20u;
constexpr u32 kEeTadr = 0x30u;
constexpr u32 kEeAsr0 = 0x40u;
constexpr u32 kEeAsr1 = 0x50u;

constexpr u32 kEeStr = 1u << 8;
constexpr u32 kEeTte = 1u << 6;
constexpr u32 kEeTie = 1u << 7;

constexpr u32 kIopDma9Madr = 0x1F801520u;
constexpr u32 kIopDma9Bcr  = 0x1F801524u;
constexpr u32 kIopDma9Chcr = 0x1F801528u;
constexpr u32 kIopDma9Tadr = 0x1F80152Cu;

constexpr u32 kIopDma10Madr = 0x1F801530u;
constexpr u32 kIopDma10Bcr  = 0x1F801534u;
constexpr u32 kIopDma10Chcr = 0x1F801538u;
constexpr u32 kIopDma10Tadr = 0x1F80153Cu;

constexpr u32 kIopDmaStart = 0x01000000u;

constexpr u32 kModeNormal = 0u;
constexpr u32 kModeChain = 1u;

u32 apply_spr(u32 address, bool spr) {
    if (!spr) return address & 0x7FFFFFF0u;
    return 0x70000000u | (address & 0x3FF0u);
}

bool tag_ends(u32 tag_word) {
    const bool irq = (tag_word & 0x80000000u) != 0;
    const u32 id = (tag_word >> 28) & 0x7u;
    return irq || (id & 0x4u) != 0;
}

} // namespace

void SifDma::reset() {}

void SifDma::append_qword(
    std::vector<u32>& words,
    u64 lo,
    u64 hi) {
    words.push_back(static_cast<u32>(lo));
    words.push_back(static_cast<u32>(lo >> 32));
    words.push_back(static_cast<u32>(hi));
    words.push_back(static_cast<u32>(hi >> 32));
}

bool SifDma::collect_ee_source_chain(
    EeBus& bus,
    u32 channel_base,
    std::vector<u32>& words,
    u32& final_chcr,
    std::string& error) {
    error.clear();
    words.clear();

    u32 chcr = 0;
    u32 madr = 0;
    u32 qwc = 0;
    u32 tadr = 0;
    if (!bus.read32(channel_base + kEeChcr, chcr) ||
        !bus.read32(channel_base + kEeMadr, madr) ||
        !bus.read32(channel_base + kEeQwc, qwc) ||
        !bus.read32(channel_base + kEeTadr, tadr)) {
        error = "SIF EE DMA state read failed";
        return false;
    }

    const u32 mode = (chcr >> 2) & 0x3u;
    qwc &= 0xFFFFu;

    auto append_payload = [&](u32 address, u32 count) -> bool {
        for (u32 i = 0; i < count; ++i) {
            u64 lo = 0;
            u64 hi = 0;
            const u32 at = address + i * 16u;
            if (!bus.read64(at, lo) ||
                !bus.read64(at + 8u, hi)) {
                error = "SIF EE source payload read fault";
                return false;
            }
            append_qword(words, lo, hi);
            if (words.size() > (1u << 20)) {
                error = "SIF EE source chain exceeded bootstrap limit";
                return false;
            }
        }
        return true;
    };

    if (mode == kModeNormal) {
        if (!append_payload(madr & 0x7FFFFFF0u, qwc)) {
            return false;
        }
        madr = (madr + qwc * 16u) & 0x7FFFFFF0u;
        qwc = 0;
    } else if (mode == kModeChain) {
        bool end = false;

        for (u32 guard = 0; guard < 4096u && !end; ++guard) {
            if (qwc != 0) {
                if (!append_payload(madr, qwc)) return false;
                madr = (madr + qwc * 16u) & 0x7FFFFFF0u;
                qwc = 0;
            }

            u64 tag_lo = 0;
            u64 tag_hi = 0;
            const u32 tag_address = tadr & 0x7FFFFFF0u;
            if (!bus.read64(tag_address, tag_lo) ||
                !bus.read64(tag_address + 8u, tag_hi)) {
                error = "SIF EE source tag read fault";
                return false;
            }

            const u32 tag0 = static_cast<u32>(tag_lo);
            const u32 tag1 = static_cast<u32>(tag_lo >> 32);
            qwc = tag0 & 0xFFFFu;
            const u32 id = (tag0 >> 28) & 0x7u;
            const bool irq = (tag0 & 0x80000000u) != 0;
            const bool spr = (tag1 & 0x80000000u) != 0;
            const u32 address = apply_spr(tag1, spr);

            chcr =
                (chcr & 0x0000FFFFu) |
                (tag0 & 0xFFFF0000u);
            if (irq && (chcr & kEeTie) != 0) end = true;

            switch (id) {
            case 0: // REFE
                madr = address;
                tadr = tag_address + 16u;
                end = true;
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
                const u32 return_address =
                    madr + qwc * 16u;
                u32 asp = (chcr >> 4) & 0x3u;
                if (asp == 0u) {
                    if (!bus.write32(
                            channel_base + kEeAsr0,
                            return_address)) {
                        error = "SIF EE ASR0 write failed";
                        return false;
                    }
                    asp = 1u;
                } else if (asp == 1u) {
                    if (!bus.write32(
                            channel_base + kEeAsr1,
                            return_address)) {
                        error = "SIF EE ASR1 write failed";
                        return false;
                    }
                    asp = 2u;
                } else {
                    end = true;
                }
                chcr =
                    (chcr & ~(0x3u << 4)) |
                    (asp << 4);
                tadr = address;
                break;
            }
            case 6: { // RET
                madr = tag_address + 16u;
                u32 asp = (chcr >> 4) & 0x3u;
                if (asp == 2u) {
                    if (!bus.read32(
                            channel_base + kEeAsr1,
                            tadr)) {
                        error = "SIF EE ASR1 read failed";
                        return false;
                    }
                    asp = 1u;
                } else if (asp == 1u) {
                    if (!bus.read32(
                            channel_base + kEeAsr0,
                            tadr)) {
                        error = "SIF EE ASR0 read failed";
                        return false;
                    }
                    asp = 0u;
                } else {
                    end = true;
                }
                chcr =
                    (chcr & ~(0x3u << 4)) |
                    (asp << 4);
                break;
            }
            case 7: // END
                madr = tag_address + 16u;
                end = true;
                break;
            }

            if ((chcr & kEeTte) != 0) {
                words.push_back(
                    static_cast<u32>(tag_hi));
                words.push_back(
                    static_cast<u32>(tag_hi >> 32));
            }

            if (qwc != 0) {
                if (!append_payload(madr, qwc)) return false;
                madr = (madr + qwc * 16u) & 0x7FFFFFF0u;
                qwc = 0;
            }
        }
    } else {
        error = "unsupported SIF EE DMA mode";
        return false;
    }

    if (!bus.write32(channel_base + kEeMadr, madr) ||
        !bus.write32(channel_base + kEeQwc, qwc) ||
        !bus.write32(channel_base + kEeTadr, tadr) ||
        !bus.write32(channel_base + kEeChcr, chcr)) {
        error = "SIF EE DMA state write failed";
        return false;
    }

    final_chcr = chcr;
    return true;
}

bool SifDma::service_sif1(
    EeBus& ee_bus,
    IopBus& iop_bus,
    IopIntc& iop_intc,
    std::string& error) {
    u32 ee_chcr = 0;
    u32 iop_chcr = 0;
    if (!ee_bus.read32(kEeSif1 + kEeChcr, ee_chcr) ||
        !iop_bus.read32(kIopDma10Chcr, iop_chcr)) {
        error = "SIF1 channel state read failed";
        return false;
    }
    if ((ee_chcr & kEeStr) == 0 ||
        (iop_chcr & kIopDmaStart) == 0) {
        return true;
    }

    std::vector<u32> stream;
    u32 final_chcr = ee_chcr;
    if (!collect_ee_source_chain(
            ee_bus,
            kEeSif1,
            stream,
            final_chcr,
            error)) {
        return false;
    }

    std::size_t pos = 0;
    bool saw_end = false;
    while (pos + 4u <= stream.size()) {
        const u32 data = stream[pos + 0u];
        const u32 words = stream[pos + 1u] & 0x000FFFFCu;
        const u32 destination = data & 0x00FFFFFFu;
        pos += 4u;

        if (pos + words > stream.size()) {
            error = "SIF1 destination packet exceeds EE source stream";
            return false;
        }

        for (u32 i = 0; i < words; ++i) {
            if (!iop_bus.write32(
                    destination + i * 4u,
                    stream[pos + i])) {
                error = "SIF1 IOP RAM write fault";
                return false;
            }
        }

        if (!iop_bus.write32(
                kIopDma10Madr,
                destination + words * 4u)) {
            error = "SIF1 IOP MADR update failed";
            return false;
        }
        pos += words;

        saw_end = tag_ends(data);
        if (saw_end) break;
    }

    if (!stream.empty() && pos == 0u) {
        error = "SIF1 stream did not contain a destination tag";
        return false;
    }

    if (!ee_bus.write32(
            kEeSif1 + kEeChcr,
            final_chcr & ~kEeStr) ||
        !iop_bus.write32(kIopDma10Bcr, 0u) ||
        !iop_bus.write32(kIopDma10Tadr, 0u) ||
        !iop_bus.write32(
            kIopDma10Chcr,
            iop_chcr & ~kIopDmaStart)) {
        error = "SIF1 completion state write failed";
        return false;
    }

    ee_bus.raise_dmac(6);
    iop_bus.raise_dma_irq(10);
    (void)iop_intc;
    (void)saw_end;
    return true;
}

bool SifDma::service_sif0(
    EeBus& ee_bus,
    IopBus& iop_bus,
    IopIntc& iop_intc,
    std::string& error) {
    u32 ee_chcr = 0;
    u32 iop_chcr = 0;
    if (!ee_bus.read32(kEeSif0 + kEeChcr, ee_chcr) ||
        !iop_bus.read32(kIopDma9Chcr, iop_chcr)) {
        error = "SIF0 channel state read failed";
        return false;
    }
    if ((ee_chcr & kEeStr) == 0 ||
        (iop_chcr & kIopDmaStart) == 0) {
        return true;
    }

    u32 tadr = 0;
    if (!iop_bus.read32(kIopDma9Tadr, tadr)) {
        error = "SIF0 IOP TADR read failed";
        return false;
    }

    bool iop_end = false;
    bool ee_end = false;
    u32 last_iop_madr = 0;
    u32 last_ee_madr = 0;

    for (u32 guard = 0;
         guard < 4096u && !(iop_end && ee_end);
         ++guard) {
        u32 data = 0;
        u32 words = 0;
        u32 ee_tag0 = 0;
        u32 ee_tag1 = 0;
        if (!iop_bus.read32(tadr + 0u, data) ||
            !iop_bus.read32(tadr + 4u, words) ||
            !iop_bus.read32(tadr + 8u, ee_tag0) ||
            !iop_bus.read32(tadr + 12u, ee_tag1)) {
            error = "SIF0 IOP source tag read fault";
            return false;
        }

        const u32 source = data & 0x00FFFFFFu;
        const u32 source_words = words & 0x000FFFFFu;
        const u32 destination =
            apply_spr(
                ee_tag1,
                (ee_tag1 & 0x80000000u) != 0);
        const u32 qwc = ee_tag0 & 0xFFFFu;
        const u32 destination_words = qwc * 4u;
        const u32 copy_words =
            std::min(source_words, destination_words);

        for (u32 i = 0; i < destination_words; ++i) {
            u32 value = 0;
            if (i < copy_words &&
                !iop_bus.read32(source + i * 4u, value)) {
                error = "SIF0 IOP payload read fault";
                return false;
            }
            if (!ee_bus.write32(
                    destination + i * 4u,
                    value)) {
                error = "SIF0 EE destination write fault";
                return false;
            }
        }

        last_iop_madr = source + source_words * 4u;
        last_ee_madr = destination + destination_words * 4u;
        tadr += 16u;

        iop_end = tag_ends(data);
        const bool ee_irq =
            (ee_tag0 & 0x80000000u) != 0;
        const u32 ee_id = (ee_tag0 >> 28) & 0x7u;
        ee_end =
            ee_id == 7u ||
            (ee_irq && (ee_chcr & kEeTie) != 0);

        ee_chcr =
            (ee_chcr & 0x0000FFFFu) |
            (ee_tag0 & 0xFFFF0000u);

        // The IOP and EE chains terminate independently. In particular, the
        // IOP commonly ends a packet and interrupts its producer while the EE
        // destination chain remains armed for the next packet.
        if (iop_end || ee_end) break;
    }

    if (!ee_bus.write32(kEeSif0 + kEeMadr, last_ee_madr) ||
        !ee_bus.write32(kEeSif0 + kEeQwc, 0u) ||
        !ee_bus.write32(
            kEeSif0 + kEeChcr,
            ee_end ? (ee_chcr & ~kEeStr) : ee_chcr) ||
        !iop_bus.write32(kIopDma9Madr, last_iop_madr) ||
        !iop_bus.write32(kIopDma9Tadr, tadr) ||
        !iop_bus.write32(
            kIopDma9Chcr,
            iop_end ? (iop_chcr & ~kIopDmaStart) : iop_chcr) ||
        (iop_end && !iop_bus.write32(kIopDma9Bcr, 0u))) {
        error = "SIF0 completion state write failed";
        return false;
    }

    if (ee_end) ee_bus.raise_dmac(5);
    if (iop_end) iop_bus.raise_dma_irq(9);
    (void)iop_intc;
    return true;
}

bool SifDma::service(
    EeBus& ee_bus,
    IopBus& iop_bus,
    IopIntc& iop_intc,
    std::string& error) {
    error.clear();

    u32 ctrl = 0;
    if (!ee_bus.read32(kDmacCtrl, ctrl)) {
        error = "SIF DMAC CTRL read failed";
        return false;
    }
    if ((ctrl & 1u) == 0) return true;

    if (!service_sif0(
            ee_bus,
            iop_bus,
            iop_intc,
            error)) {
        return false;
    }
    if (!service_sif1(
            ee_bus,
            iop_bus,
            iop_intc,
            error)) {
        return false;
    }
    return true;
}

} // namespace ps2
