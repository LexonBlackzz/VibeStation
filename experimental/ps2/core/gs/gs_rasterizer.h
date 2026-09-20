#pragma once

#include "common/types.h"

namespace ps2 {

class GsVram;

struct GsRasterVertex {
    s32 x = 0; // 12.4 fixed-point after XYOFFSET subtraction.
    s32 y = 0;
    u32 z = 0;
    u32 rgba = 0;
};

struct GsRasterContext {
    u32 fbp = 0; // GS block pointer (256-byte units).
    u32 fbw = 0; // GS 64-pixel width units.
    u32 psm = 0;
    u32 fbmask = 0;
    s32 scax0 = 0;
    s32 scax1 = 0;
    s32 scay0 = 0;
    s32 scay1 = 0;
};

class GsRasterizer {
public:
    [[nodiscard]] static bool supported_target(const GsRasterContext& ctx);

    static u64 draw_sprite(
        GsVram& vram,
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b);

    static u64 draw_triangle(
        GsVram& vram,
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        const GsRasterVertex& c);

private:
    static bool write_color(
        GsVram& vram,
        const GsRasterContext& ctx,
        s32 x,
        s32 y,
        u32 rgba);
};

} // namespace ps2
