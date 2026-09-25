#pragma once

#include "common/types.h"
#include "core/gs/gs_rasterizer.h"

namespace ps2 {

class GsVram;

// Optional accelerator interface. GsCore remains fully functional without a
// backend; unsupported states and synchronization barriers fall back to the
// software renderer.
class GsGpuBackend {
public:
    virtual ~GsGpuBackend() = default;

    [[nodiscard]] virtual bool available() const = 0;
    [[nodiscard]] virtual const char* name() const = 0;

    // Queue one dependency-safe sprite. The backend may execute it
    // asynchronously; CPU VRAM is intentionally stale until synchronize_to_cpu.
    virtual bool submit_sprite(
        const GsVram& vram,
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        s32 top,
        s32 bottom,
        u64 area) = 0;

    // Complete queued GPU work and merge GPU-authoritative bytes back into
    // CPU VRAM. Returns true when CPU VRAM was modified.
    virtual bool synchronize_to_cpu(GsVram& vram) = 0;

    // Called after a core reset. The next accelerated draw will upload the
    // new CPU-authoritative VRAM image.
    virtual void invalidate_cpu_source() = 0;
};

} // namespace ps2
