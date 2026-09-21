#include "core/dma/vif1_dma.h"

#include "core/gs/gs_core.h"
#include "core/gs/gs_privileged.h"
#include "core/memory/ee_bus.h"

namespace ps2 {
namespace {

constexpr u32 kDmacCtrl = 0x1000E000u;
constexpr u32 kVif1Chcr = 0x10009000u;
constexpr u32 kVif1Madr = 0x10009010u;
constexpr u32 kVif1Qwc = 0x10009020u;
constexpr u32 kVif1Stat = 0x10003C00u;

constexpr u32 kChcrDir = 1u << 0;
constexpr u32 kChcrModeMask = 3u << 2;
constexpr u32 kChcrStr = 1u << 8;
constexpr u32 kVif1Fdr = 1u << 23;

} // namespace

bool Vif1Dma::complete(EeBus& bus, u32 chcr) {
    if (!bus.write32(kVif1Qwc, 0u)) return false;
    if (!bus.write32(kVif1Chcr, chcr & ~kChcrStr)) return false;
    bus.raise_dmac(1);
    return true;
}

bool Vif1Dma::service(
    EeBus& bus,
    GsCore& gs,
    const GsPrivileged& privileged,
    std::string& error) {
    error.clear();

    u32 chcr = 0;
    if (!bus.read32(kVif1Chcr, chcr)) {
        error = "failed to read VIF1 CHCR";
        return false;
    }

    if ((chcr & kChcrStr) == 0) return true;

    u32 ctrl = 0;
    if (!bus.read32(kDmacCtrl, ctrl)) {
        error = "failed to read DMAC CTRL";
        return false;
    }
    if ((ctrl & 1u) == 0) return true;

    // This service owns the GS -> EE reverse path only. Memory -> VIF1 is
    // handled by the VIF command path and must not be consumed here.
    if ((chcr & kChcrDir) != 0) return true;

    // Local-to-host readback uses normal mode. Leave other channel modes
    // armed until their dedicated VIF1 DMA implementation handles them.
    if ((chcr & kChcrModeMask) != 0) return true;

    u32 vif1_stat = 0;
    if (!bus.read32(kVif1Stat, vif1_stat)) {
        error = "failed to read VIF1 STAT";
        return false;
    }
    if ((vif1_stat & kVif1Fdr) == 0 || privileged.busdir() == 0) {
        return true;
    }

    u32 qwc = 0;
    if (!bus.read32(kVif1Qwc, qwc)) {
        error = "failed to read VIF1 QWC";
        return false;
    }
    if (qwc == 0) {
        if (!complete(bus, chcr)) {
            error = "failed to complete zero-length VIF1 DMA";
            return false;
        }
        return true;
    }

    u64 lo = 0;
    u64 hi = 0;
    if (!gs.read_local_to_host_qword(lo, hi)) {
        // The GS has not produced readback data yet. Real hardware stalls the
        // channel here rather than completing it with invented data.
        return true;
    }

    u32 madr = 0;
    if (!bus.read32(kVif1Madr, madr)) {
        error = "failed to read VIF1 MADR";
        return false;
    }
    madr &= 0x7FFFFFF0u;

    if (!bus.write64(madr, lo) || !bus.write64(madr + 8u, hi)) {
        error = "VIF1 reverse DMA RAM write fault";
        return false;
    }

    madr = (madr + 16u) & 0x7FFFFFF0u;
    --qwc;
    if (!bus.write32(kVif1Madr, madr) ||
        !bus.write32(kVif1Qwc, qwc)) {
        error = "failed to update VIF1 DMA registers";
        return false;
    }

    if (qwc == 0 && !complete(bus, chcr)) {
        error = "failed to complete VIF1 reverse DMA";
        return false;
    }

    return true;
}

} // namespace ps2
