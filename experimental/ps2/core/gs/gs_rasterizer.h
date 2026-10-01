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
    u32 fog = 0xFFu;
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
    // TEX1 MMAG/MMIN resolved to a single filter choice for this draw.
    bool linear = false;

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
    u64* nonzero_samples = nullptr; // Optional trace counter.
    u64* alpha_samples = nullptr; // Optional trace counter.
    u32* first_sample_x = nullptr;
    u32* first_sample_y = nullptr;
    u32* first_sample_rgba = nullptr;
    u64* nonzero_shaded = nullptr; // Optional trace counter.
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

    bool fog_enabled = false;
    u32 fog_color = 0;
    u32 scanmask = 0;
    bool dither = false;
    u64 dimx = 0;

    GsTextureState texture{};
    u64* nonzero_colors = nullptr; // Optional trace counter.
    u64* nonzero_inputs = nullptr; // Optional trace counter.
    u64* nonzero_input_alpha = nullptr; // Optional trace counter.
    u32* first_input_rgba = nullptr; // Optional trace sample.
    u32* first_alpha_input_rgba = nullptr; // Optional trace sample.
};

// VRAM footprint of one primitive, used to decide whether a run of draws can
// be rasterized in parallel by screen-row bands without changing the result.
struct GsBandFootprint {
    s32 top = 0;    // Clipped rows touched: [top, bottom).
    s32 bottom = 0;
    u64 area = 0;   // Clipped bounding-box pixels (work estimate).
    bool has_frame = false;
    bool has_depth = false;
    bool has_texture = false;
    u64 frame_begin = 0, frame_end = 0, frame_key = 0;
    u64 depth_begin = 0, depth_end = 0, depth_key = 0;
    u64 texture_begin = 0, texture_end = 0;
};

class GsRasterizer {
public:
    [[nodiscard]] static bool supported_target(const GsRasterContext& ctx);
    [[nodiscard]] static bool supported_texture(const GsTextureState& texture);

    static u64 draw_point(
        GsVram& vram,
        const GsRasterContext& ctx,
        const GsRasterVertex& vertex);

    static u64 draw_line(
        GsVram& vram,
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b);

    static u64 draw_sprite(
        GsVram& vram,
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b);

    // Persistent-worker support. The plan only succeeds for large sprite
    // states whose framebuffer writes cannot alias their texture/depth reads.
    static bool parallel_sprite_plan(
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        s32& top,
        s32& bottom,
        u64& area);
    static u64 draw_sprite_rows(
        GsVram& vram,
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        s32 row_begin,
        s32 row_end);

    static u64 draw_triangle(
        GsVram& vram,
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        const GsRasterVertex& c);

    // draw_triangle() restricted to rows [row_begin, row_end). Pixels are
    // independent, so bands drawn by different threads give the exact result
    // of one draw_triangle() call.
    static u64 draw_triangle_rows(
        GsVram& vram,
        const GsRasterContext& ctx,
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        const GsRasterVertex& c,
        s32 row_begin,
        s32 row_end);

    // Fills the footprint for a triangle (primitive 3-5) or sprite (6).
    // Returns false when the draw must not be banded (unsupported format,
    // tracing hooks, CLUT reads, or it samples VRAM it also writes).
    static bool band_plan(
        const GsRasterContext& ctx,
        u32 primitive,
        const GsRasterVertex& a,
        const GsRasterVertex& b,
        const GsRasterVertex& c,
        GsBandFootprint& fp);

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
    [[nodiscard]] static u32 apply_fog(
        u32 rgba,
        u32 fog_color,
        u32 fog);
    [[nodiscard]] static u32 apply_dither(
        u32 rgba,
        const GsRasterContext& ctx,
        s32 x,
        s32 y);
};

} // namespace ps2
