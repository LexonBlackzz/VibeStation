#include "core/gs/gs_rasterizer.h"

#include "core/gs/gs_vram.h"

#include <algorithm>

namespace ps2 {
namespace {

s32 floor_div16(s32 value) {
    if (value >= 0) return value >> 4;
    return -static_cast<s32>((static_cast<u32>(-value) + 15u) >> 4);
}

s32 ceil_div16(s32 value) {
    if (value >= 0) return (value + 15) >> 4;
    return -static_cast<s32>(static_cast<u32>(-value) >> 4);
}

s64 edge(
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    s32 px,
    s32 py) {
    return static_cast<s64>(px - a.x) * static_cast<s64>(b.y - a.y) -
           static_cast<s64>(py - a.y) * static_cast<s64>(b.x - a.x);
}

u16 rgba32_to_16(u32 c) {
    const u32 rb = c & 0x00F800F8u;
    const u32 ga = c & 0x8000F800u;
    return static_cast<u16>(
        (ga >> 16) | (rb >> 9) | (ga >> 6) | (rb >> 3));
}

} // namespace

bool GsRasterizer::supported_target(const GsRasterContext& ctx) {
    if (ctx.fbw == 0 || !GsVram::supported_color_psm(ctx.psm)) return false;
    if ((ctx.psm == 2u || ctx.psm == 10u) && ctx.fbmask != 0) return false;
    return true;
}

bool GsRasterizer::write_color(
    GsVram& vram,
    const GsRasterContext& ctx,
    s32 x,
    s32 y,
    u32 rgba) {
    if (x < ctx.scax0 || x > ctx.scax1 || y < ctx.scay0 || y > ctx.scay1)
        return false;

    const u32 ux = static_cast<u32>(x);
    const u32 uy = static_cast<u32>(y);

    if (ctx.psm == 0u) {
        const u32 old = vram.read_pixel(0, ux, uy, ctx.fbp, ctx.fbw);
        const u32 value = (old & ctx.fbmask) | (rgba & ~ctx.fbmask);
        return vram.write_pixel(0, ux, uy, ctx.fbp, ctx.fbw, value);
    }

    if (ctx.psm == 1u) {
        const u32 old = vram.read_pixel(1, ux, uy, ctx.fbp, ctx.fbw);
        const u32 mask = ctx.fbmask & 0x00FFFFFFu;
        const u32 value = (old & mask) | (rgba & ~mask & 0x00FFFFFFu);
        return vram.write_pixel(1, ux, uy, ctx.fbp, ctx.fbw, value);
    }

    return vram.write_pixel(
        ctx.psm, ux, uy, ctx.fbp, ctx.fbw, rgba32_to_16(rgba));
}

u64 GsRasterizer::draw_sprite(
    GsVram& vram,
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b) {
    if (!supported_target(ctx)) return 0;

    const s32 min_x_fp = std::min(a.x, b.x);
    const s32 max_x_fp = std::max(a.x, b.x);
    const s32 min_y_fp = std::min(a.y, b.y);
    const s32 max_y_fp = std::max(a.y, b.y);

    s32 left = ceil_div16(min_x_fp);
    s32 right = ceil_div16(max_x_fp);
    s32 top = ceil_div16(min_y_fp);
    s32 bottom = ceil_div16(max_y_fp);

    left = std::max(left, ctx.scax0);
    right = std::min(right, ctx.scax1 + 1);
    top = std::max(top, ctx.scay0);
    bottom = std::min(bottom, ctx.scay1 + 1);

    if (left >= right || top >= bottom) return 0;

    u64 pixels = 0;
    for (s32 y = top; y < bottom; ++y) {
        for (s32 x = left; x < right; ++x) {
            if (write_color(vram, ctx, x, y, b.rgba)) ++pixels;
        }
    }
    return pixels;
}

u64 GsRasterizer::draw_triangle(
    GsVram& vram,
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    const GsRasterVertex& c) {
    if (!supported_target(ctx)) return 0;

    const s64 area = edge(a, b, c.x, c.y);
    if (area == 0) return 0;

    const s32 min_x_fp = std::min({a.x, b.x, c.x});
    const s32 max_x_fp = std::max({a.x, b.x, c.x});
    const s32 min_y_fp = std::min({a.y, b.y, c.y});
    const s32 max_y_fp = std::max({a.y, b.y, c.y});

    s32 left = std::max(floor_div16(min_x_fp), ctx.scax0);
    s32 right = std::min(ceil_div16(max_x_fp), ctx.scax1 + 1);
    s32 top = std::max(floor_div16(min_y_fp), ctx.scay0);
    s32 bottom = std::min(ceil_div16(max_y_fp), ctx.scay1 + 1);

    u64 pixels = 0;
    for (s32 y = top; y < bottom; ++y) {
        for (s32 x = left; x < right; ++x) {
            const s32 px = x * 16 + 8;
            const s32 py = y * 16 + 8;
            const s64 e0 = edge(a, b, px, py);
            const s64 e1 = edge(b, c, px, py);
            const s64 e2 = edge(c, a, px, py);
            const bool inside =
                area > 0 ? (e0 >= 0 && e1 >= 0 && e2 >= 0)
                         : (e0 <= 0 && e1 <= 0 && e2 <= 0);
            if (inside && write_color(vram, ctx, x, y, c.rgba)) ++pixels;
        }
    }
    return pixels;
}

} // namespace ps2
