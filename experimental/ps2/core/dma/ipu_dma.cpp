#include "core/dma/ipu_dma.h"

#include "core/memory/ee_bus.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u32 kDmacCtrl = 0x1000E000u;
constexpr u32 kFromIpuBase = 0x1000B000u;
constexpr u32 kToIpuBase = 0x1000B400u;
constexpr u32 kChcr = 0x00u;
constexpr u32 kMadr = 0x10u;
constexpr u32 kQwc = 0x20u;
constexpr u32 kTadr = 0x30u;
constexpr u32 kStr = 1u << 8;
constexpr u32 kTie = 1u << 7;
constexpr u32 kModeNormal = 0u;
constexpr u32 kModeChain = 1u;

constexpr u32 kIpuCtrl = 0x10002010u;
constexpr u32 kIpuOut = 0x10007000u;
constexpr u32 kIpuIn = 0x10007010u;

u32 apply_spr(u32 address, bool spr) {
    if (!spr) return address & 0x7FFFFFF0u;
    return 0x70000000u | (address & 0x3FF0u);
}

bool write_input_qword(
    EeBus& bus,
    u32 address,
    std::string& error) {
    u64 lo = 0;
    u64 hi = 0;
    if (!bus.read64(address, lo) ||
        !bus.read64(address + 8u, hi)) {
        error = "IPU input DMA source read fault";
        return false;
    }
    if (!bus.write64(kIpuIn, lo) ||
        !bus.write64(kIpuIn + 8u, hi)) {
        error = "IPU input FIFO write fault";
        return false;
    }
    return true;
}

} // namespace

void IpuDma::reset() {
    end_to_ipu_ = false;
}

bool IpuDma::service_from_ipu(
    EeBus& bus,
    std::string& error) {
    u32 chcr = 0;
    if (!bus.read32(kFromIpuBase + kChcr, chcr)) {
        error = "IPU0 CHCR read failed";
        return false;
    }
    if ((chcr & kStr) == 0) return true;

    u32 qwc = 0;
    u32 madr = 0;
    if (!bus.read32(kFromIpuBase + kQwc, qwc) ||
        !bus.read32(kFromIpuBase + kMadr, madr)) {
        error = "IPU0 DMA state read failed";
        return false;
    }
    qwc &= 0xFFFFu;

    // Hardware ends a zero-QWC FROM_IPU transfer when there is no output
    // available. This is used by firmware probes and should not hang STR.
    if (qwc == 0) {
        if (!bus.write32(kFromIpuBase + kChcr, chcr & ~kStr)) {
            error = "IPU0 zero-QWC completion failed";
            return false;
        }
        bus.raise_dmac(3u);
        return true;
    }

    // FROM_IPU is normal-mode only. Never fabricate decoded output: if the
    // modeled output FIFO is empty, retain STR and let software keep polling.
    if (((chcr >> 2) & 0x3u) != kModeNormal) return true;

    u32 ctrl = 0;
    if (!bus.read32(kIpuCtrl, ctrl)) {
        error = "IPU CTRL read failed";
        return false;
    }
    u32 ofc = (ctrl >> 4) & 0xFu;
    if (ofc == 0) return true;

    const u32 transfer = std::min(qwc, ofc);
    for (u32 i = 0; i < transfer; ++i) {
        u64 lo = 0;
        u64 hi = 0;
        if (!bus.read64(kIpuOut, lo) ||
            !bus.read64(kIpuOut + 8u, hi)) {
            error = "IPU output FIFO read fault";
            return false;
        }
        const u32 dst = apply_spr(
            madr + i * 16u,
            (madr & 0x80000000u) != 0);
        if (!bus.write64(dst, lo) ||
            !bus.write64(dst + 8u, hi)) {
            error = "IPU0 destination write fault";
            return false;
        }
    }

    madr += transfer * 16u;
    qwc -= transfer;
    ofc -= transfer;
    ctrl = (ctrl & ~(0xFu << 4)) | ((ofc & 0xFu) << 4);

    if (!bus.write32(kFromIpuBase + kMadr, madr) ||
        !bus.write32(kFromIpuBase + kQwc, qwc) ||
        !bus.write32(kIpuCtrl, ctrl)) {
        error = "IPU0 DMA progress write failed";
        return false;
    }

    if (qwc == 0) {
        if (!bus.write32(kFromIpuBase + kChcr, chcr & ~kStr)) {
            error = "IPU0 completion write failed";
            return false;
        }
        bus.raise_dmac(3u);
    }
    return true;
}

bool IpuDma::service_to_ipu(
    EeBus& bus,
    std::string& error) {
    u32 chcr = 0;
    if (!bus.read32(kToIpuBase + kChcr, chcr)) {
        error = "IPU1 CHCR read failed";
        return false;
    }
    if ((chcr & kStr) == 0) return true;

    u32 qwc = 0;
    u32 madr = 0;
    u32 tadr = 0;
    if (!bus.read32(kToIpuBase + kQwc, qwc) ||
        !bus.read32(kToIpuBase + kMadr, madr) ||
        !bus.read32(kToIpuBase + kTadr, tadr)) {
        error = "IPU1 DMA state read failed";
        return false;
    }
    qwc &= 0xFFFFu;

    const u32 mode = (chcr >> 2) & 0x3u;
    if (mode != kModeNormal && mode != kModeChain) return true;

    if (mode == kModeChain && qwc == 0 && !end_to_ipu_) {
        u64 tag_lo = 0;
        u64 tag_hi = 0;
        const u32 tag_address = tadr & 0x7FFFFFF0u;
        if (!bus.read64(tag_address, tag_lo) ||
            !bus.read64(tag_address + 8u, tag_hi)) {
            error = "IPU1 DMA tag fetch fault";
            return false;
        }
        (void)tag_hi;

        const u32 tag0 = static_cast<u32>(tag_lo);
        const u32 tag1 = static_cast<u32>(tag_lo >> 32);
        const u32 id = (tag0 >> 28) & 0x7u;
        const bool irq = (tag0 & 0x80000000u) != 0;
        const bool spr = (tag1 & 0x80000000u) != 0;
        const u32 address = apply_spr(tag1, spr);
        qwc = tag0 & 0xFFFFu;

        chcr =
            (chcr & 0x0000FFFFu) |
            (tag0 & 0xFFFF0000u);
        end_to_ipu_ =
            id == 0u || id == 7u ||
            (irq && (chcr & kTie) != 0);

        switch (id) {
        case 0: // REFE
            madr = address;
            tadr = tag_address + 16u;
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
        case 5: // CALL
        case 6: // RET
            // Bootstrap IPU traffic does not require nested source chains.
            // Finish this payload rather than fabricating an ASR stack.
            madr = tag_address + 16u;
            tadr = address;
            end_to_ipu_ = true;
            break;
        case 7: // END
            madr = tag_address + 16u;
            tadr = madr + qwc * 16u;
            break;
        }

        if (!bus.write32(kToIpuBase + kChcr, chcr) ||
            !bus.write32(kToIpuBase + kMadr, madr) ||
            !bus.write32(kToIpuBase + kQwc, qwc) ||
            !bus.write32(kToIpuBase + kTadr, tadr)) {
            error = "IPU1 tag state write failed";
            return false;
        }
    }

    if (qwc != 0) {
        const u32 chunk = std::min(qwc, 8u);
        for (u32 i = 0; i < chunk; ++i) {
            const u32 src = apply_spr(
                madr + i * 16u,
                (madr & 0x80000000u) != 0);
            if (!write_input_qword(bus, src, error)) return false;
        }
        madr += chunk * 16u;
        qwc -= chunk;
        if (!bus.write32(kToIpuBase + kMadr, madr) ||
            !bus.write32(kToIpuBase + kQwc, qwc)) {
            error = "IPU1 DMA progress write failed";
            return false;
        }
    }

    if (qwc == 0 &&
        (mode == kModeNormal || end_to_ipu_)) {
        end_to_ipu_ = false;
        if (!bus.write32(kToIpuBase + kChcr, chcr & ~kStr)) {
            error = "IPU1 completion write failed";
            return false;
        }
        bus.raise_dmac(4u);
    }
    return true;
}

bool IpuDma::service(EeBus& bus, std::string& error) {
    error.clear();

    u32 ctrl = 0;
    if (!bus.read32(kDmacCtrl, ctrl)) {
        error = "IPU DMAC CTRL read failed";
        return false;
    }
    if ((ctrl & 1u) == 0) return true;

    if (!service_from_ipu(bus, error)) return false;
    return service_to_ipu(bus, error);
}

} // namespace ps2
