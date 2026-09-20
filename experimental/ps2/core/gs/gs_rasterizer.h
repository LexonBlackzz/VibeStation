#pragma once

#include "common/types.h"

namespace ps2 {

class GsVram;

struct GsRasterVertex {
    s32 x = 0; // 12.4 fixed-point after XYOFFSET subtraction.
    s32 y = 0;
    u32 z = 0;
    u32 rgba = 0;
    s32 u = 0; // 10.4 fixed-point for FST/UV mode.
    s32 v = 0;
    float s = 0.0f; // STQ mode values.
    float t = 0.0f;
    float q = 1.0f;
};

struct GsTextureState {
    bool enabled = false;
    u32 bp = 0; // GS block pointer (256-byte units).
    u32 bw = 0; // GS 64-pixel width units.
    u32 psm = 0;
    u32 width = 0;
    u32 height = 0;
    u32 wms = 0;
    u32 wmt = 0;
    u32 minu = 0;
    u32 maxu = 0;
    u32 minv = 0;
    u32 maxv = 0;
    bool tcc = false;
    u32 tfx = 0;
    bool fst = true;

    u32 cbp = 0;
    u32 cpsm = 0;
    bool csm2 = false;
    u32 csa = 0;
    u32 clut_bw = 0;
    u32 clut_u = 0;
    u32 clut_v = 0;

    u32 ta0 = 0;
    u32 ta1 = 0;
    bool aem = false;
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

    bool gouraud = false;
    bool alpha_blend = false;

    bool ate = false;
    u32 atst = 1;
    u32 aref = 0;
    u32 afail = 0;
    bool date = false;
    bool datm = false;

    bool zte = false;
    u32 ztst = 1;
    u32 zbp = 0;
    u32 zpsm = 48;
    bool zmask = true;

    u32 alpha_a = 0;
    u32 alpha_b = 1;
    u32 alpha_c = 0;
    u32 alpha_d = 1;
    u32 alpha_fix = 0;
    bool pabe = false;
    bool fba = false;
    bool color_clamp = true;

    GsTextureState texture{};
};

class GsRasterizer {
public:
    [[nodiscard]] static bool supported_target(const GsRasterContext& ctx);
    [[nodiscard]] static bool supported_texture(const GsTextureState& texture);

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
    static bool draw_pixel(
        GsVram& vram,
        const GsRasterContext& ctx,
        s32 x,
        s32 y,
        u32 z,
        u32 rgba);
    [[nodiscard]] static u32 shade_pixel(
        const GsVram& vram,
        const GsTextureState& texture,
        s32 u,
        s32 v,
        u32 vertex_rgba);
};

} // namespace ps2
