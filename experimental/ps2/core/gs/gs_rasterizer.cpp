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

u32 channel(u32 color, u32 shift) {
    return (color >> shift) & 0xFFu;
}

u32 modulate_channel(u32 texture, u32 vertex) {
    return std::min(255u, (texture * vertex) >> 7);
}

s32 wrap_coordinate(
    s32 coordinate,
    u32 size,
    u32 mode,
    u32 min_value,
    u32 max_value) {
    if (size == 0) return 0;
    const s32 maximum = static_cast<s32>(size - 1u);

    switch (mode & 3u) {
    case 0: // REPEAT
        return coordinate & maximum;
    case 1: // CLAMP
        return std::clamp(coordinate, 0, maximum);
    case 2: { // REGION_CLAMP
        const s32 lo = static_cast<s32>(std::min(min_value, size - 1u));
        const s32 hi = static_cast<s32>(std::min(max_value, size - 1u));
        return std::clamp(coordinate, std::min(lo, hi), std::max(lo, hi));
    }
    case 3: // REGION_REPEAT: (coord & mask) | fix
        return (coordinate & static_cast<s32>(min_value & (size - 1u))) |
               static_cast<s32>(max_value);
    default:
        return coordinate;
    }
}

} // namespace

bool GsRasterizer::supported_target(const GsRasterContext& ctx) {
    if (ctx.fbw == 0 || !GsVram::supported_color_psm(ctx.psm)) return false;
    if ((ctx.psm == 2u || ctx.psm == 10u) && ctx.fbmask != 0) return false;
    return true;
}

bool GsRasterizer::supported_texture(const GsTextureState& texture) {
    if (!texture.enabled) return true;
    if (texture.bw == 0 || texture.width == 0 || texture.height == 0)
        return false;
    if (texture.width > 1024u || texture.height > 1024u)
        return false;
    if (texture.tfx > 1u) // HIGHLIGHT/HIGHLIGHT2 not modeled yet.
        return false;

    // PSMCT32 has native alpha. PSMCT24 is safe only when texture alpha is
    // ignored. PSMCT16/16S require TEXA semantics before their alpha can be
    // modeled correctly.
    if (texture.psm == 0u) return true;
    if (texture.psm == 1u && !texture.tcc) return true;
    return false;
}

u32 GsRasterizer::shade_pixel(
    const GsVram& vram,
    const GsTextureState& texture,
    s32 u,
    s32 v,
    u32 vertex_rgba) {
    if (!texture.enabled) return vertex_rgba;

    const s32 texel_u = wrap_coordinate(
        u >> 4, texture.width, texture.wms, texture.minu, texture.maxu);
    const s32 texel_v = wrap_coordinate(
        v >> 4, texture.height, texture.wmt, texture.minv, texture.maxv);

    const u32 raw = vram.read_pixel(
        texture.psm,
        static_cast<u32>(texel_u),
        static_cast<u32>(texel_v),
        texture.bp,
        texture.bw);
    const u32 texture_rgba =
        texture.psm == 1u ? ((raw & 0x00FFFFFFu) | 0x80000000u) : raw;

    if (texture.tfx == 1u) { // DECAL
        const u32 alpha = texture.tcc
            ? (texture_rgba & 0xFF000000u)
            : (vertex_rgba & 0xFF000000u);
        return (texture_rgba & 0x00FFFFFFu) | alpha;
    }

    // MODULATE uses GS 1.7 fixed-point color math: component*component >> 7.
    u32 out = 0;
    out |= modulate_channel(channel(texture_rgba, 0), channel(vertex_rgba, 0));
    out |= modulate_channel(channel(texture_rgba, 8), channel(vertex_rgba, 8)) << 8;
    out |= modulate_channel(channel(texture_rgba, 16), channel(vertex_rgba, 16)) << 16;
    const u32 alpha = texture.tcc
        ? modulate_channel(channel(texture_rgba, 24), channel(vertex_rgba, 24))
        : channel(vertex_rgba, 24);
    out |= alpha << 24;
    return out;
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
    if (!supported_target(ctx) || !supported_texture(ctx.texture)) return 0;

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

    const s64 dx = static_cast<s64>(b.x) - a.x;
    const s64 dy = static_cast<s64>(b.y) - a.y;
    u64 pixels = 0;
    for (s32 y = top; y < bottom; ++y) {
        for (s32 x = left; x < right; ++x) {
            const s32 px = x * 16 + 8;
            const s32 py = y * 16 + 8;
            const s32 u = dx != 0
                ? static_cast<s32>(
                    static_cast<s64>(a.u) +
                    (static_cast<s64>(b.u - a.u) * (px - a.x)) / dx)
                : a.u;
            const s32 v = dy != 0
                ? static_cast<s32>(
                    static_cast<s64>(a.v) +
                    (static_cast<s64>(b.v - a.v) * (py - a.y)) / dy)
                : a.v;
            const u32 rgba = shade_pixel(vram, ctx.texture, u, v, b.rgba);
            if (write_color(vram, ctx, x, y, rgba)) ++pixels;
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
    if (!supported_target(ctx) || !supported_texture(ctx.texture)) return 0;

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
            const s64 w0 = edge(b, c, px, py);
            const s64 w1 = edge(c, a, px, py);
            const s64 w2 = edge(a, b, px, py);
            const bool inside =
                area > 0 ? (w0 >= 0 && w1 >= 0 && w2 >= 0)
                         : (w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (!inside) continue;

            const s32 u = static_cast<s32>(
                (w0 * a.u + w1 * b.u + w2 * c.u) / area);
            const s32 v = static_cast<s32>(
                (w0 * a.v + w1 * b.v + w2 * c.v) / area);
            const u32 rgba = shade_pixel(vram, ctx.texture, u, v, c.rgba);
            if (write_color(vram, ctx, x, y, rgba)) ++pixels;
        }
    }
    return pixels;
}

} // namespace ps2
