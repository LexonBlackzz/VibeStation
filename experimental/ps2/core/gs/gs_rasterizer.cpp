#include "core/gs/gs_rasterizer.h"

#include "core/gs/gs_vram.h"

#include <algorithm>
#include <limits>

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

u32 rgba16_to_32(u16 c) {
    return ((static_cast<u32>(c) & 0x8000u) << 16) |
           ((static_cast<u32>(c) & 0x7C00u) << 9) |
           ((static_cast<u32>(c) & 0x03E0u) << 6) |
           ((static_cast<u32>(c) & 0x001Fu) << 3);
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

bool alpha_test_pass(u32 atst, u32 alpha, u32 reference) {
    switch (atst & 7u) {
    case 0: return false;                    // NEVER
    case 1: return true;                     // ALWAYS
    case 2: return alpha < reference;        // LESS
    case 3: return alpha <= reference;       // LEQUAL
    case 4: return alpha == reference;       // EQUAL
    case 5: return alpha >= reference;       // GEQUAL
    case 6: return alpha > reference;        // GREATER
    case 7: return alpha != reference;       // NOTEQUAL
    default: return true;
    }
}

u32 depth_value_for_psm(u32 psm, u32 z) {
    if (psm == 49u) return z & 0x00FFFFFFu;
    if (psm == 50u || psm == 58u) return z & 0x0000FFFFu;
    return z;
}

bool depth_test_pass(u32 ztst, u32 source, u32 destination) {
    switch (ztst & 3u) {
    case 0: return false;                    // NEVER
    case 1: return true;                     // ALWAYS
    case 2: return source >= destination;    // GEQUAL
    case 3: return source > destination;     // GREATER
    default: return true;
    }
}

u32 read_frame_rgba(
    const GsVram& vram,
    const GsRasterContext& ctx,
    u32 x,
    u32 y) {
    const u32 raw = vram.read_pixel(ctx.psm, x, y, ctx.fbp, ctx.fbw);
    if (ctx.psm == 0u) return raw;
    if (ctx.psm == 1u) return (raw & 0x00FFFFFFu) | 0x80000000u;
    return rgba16_to_32(static_cast<u16>(raw));
}

u32 select_blend_color(u32 selector, u32 source, u32 destination) {
    if ((selector & 3u) == 0u) return source;
    if ((selector & 3u) == 1u) return destination;
    return 0;
}

u32 blend_color(u32 source, u32 destination, const GsRasterContext& ctx) {
    const u32 factor =
        (ctx.alpha_c & 3u) == 0u ? channel(source, 24) :
        (ctx.alpha_c & 3u) == 1u ? channel(destination, 24) :
        (ctx.alpha_c & 3u) == 2u ? ctx.alpha_fix :
        0u;

    u32 output = source & 0xFF000000u;
    for (u32 shift : {0u, 8u, 16u}) {
        const s32 cs = static_cast<s32>(channel(source, shift));
        const s32 cd = static_cast<s32>(channel(destination, shift));
        const s32 a = static_cast<s32>(
            select_blend_color(ctx.alpha_a, static_cast<u32>(cs), static_cast<u32>(cd)));
        const s32 b = static_cast<s32>(
            select_blend_color(ctx.alpha_b, static_cast<u32>(cs), static_cast<u32>(cd)));
        const s32 d = static_cast<s32>(
            select_blend_color(ctx.alpha_d, static_cast<u32>(cs), static_cast<u32>(cd)));

        s32 value = ((a - b) * static_cast<s32>(factor)) / 128 + d;
        if (ctx.color_clamp) {
            value = std::clamp(value, 0, 255);
        } else {
            value &= 0xFF;
        }
        output |= static_cast<u32>(value) << shift;
    }
    return output;
}

u32 interpolate_channel(
    s64 w0,
    s64 w1,
    s64 w2,
    s64 area,
    u32 a,
    u32 b,
    u32 c,
    u32 shift) {
    const s64 numerator =
        w0 * static_cast<s64>(channel(a, shift)) +
        w1 * static_cast<s64>(channel(b, shift)) +
        w2 * static_cast<s64>(channel(c, shift));
    const s64 value = numerator / area;
    return static_cast<u32>(std::clamp<s64>(value, 0, 255));
}

u32 interpolate_rgba(
    s64 w0,
    s64 w1,
    s64 w2,
    s64 area,
    u32 a,
    u32 b,
    u32 c) {
    u32 out = 0;
    for (u32 shift : {0u, 8u, 16u, 24u}) {
        out |= interpolate_channel(w0, w1, w2, area, a, b, c, shift) << shift;
    }
    return out;
}

u32 interpolate_z(
    s64 w0,
    s64 w1,
    s64 w2,
    s64 area,
    u32 a,
    u32 b,
    u32 c) {
    const long double numerator =
        static_cast<long double>(w0) * static_cast<long double>(a) +
        static_cast<long double>(w1) * static_cast<long double>(b) +
        static_cast<long double>(w2) * static_cast<long double>(c);
    long double value = numerator / static_cast<long double>(area);
    value = std::clamp(
        value,
        static_cast<long double>(0),
        static_cast<long double>(std::numeric_limits<u32>::max()));
    return static_cast<u32>(value);
}

} // namespace

bool GsRasterizer::supported_target(const GsRasterContext& ctx) {
    if (ctx.fbw == 0 || !GsVram::supported_color_psm(ctx.psm)) return false;
    if ((ctx.psm == 2u || ctx.psm == 10u) && ctx.fbmask != 0) return false;
    if (ctx.zte && !GsVram::supported_depth_psm(ctx.zpsm)) return false;
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

bool GsRasterizer::draw_pixel(
    GsVram& vram,
    const GsRasterContext& ctx,
    s32 x,
    s32 y,
    u32 z,
    u32 rgba) {
    if (x < ctx.scax0 || x > ctx.scax1 || y < ctx.scay0 || y > ctx.scay1)
        return false;

    const u32 ux = static_cast<u32>(x);
    const u32 uy = static_cast<u32>(y);
    const u32 destination = read_frame_rgba(vram, ctx, ux, uy);

    bool write_frame = true;
    bool write_depth = ctx.zte && !ctx.zmask;
    bool rgb_only = false;

    if (ctx.ate &&
        !alpha_test_pass(ctx.atst, channel(rgba, 24), ctx.aref)) {
        switch (ctx.afail & 3u) {
        case 0: // KEEP
            write_frame = false;
            write_depth = false;
            break;
        case 1: // FB_ONLY
            write_depth = false;
            break;
        case 2: // ZB_ONLY
            write_frame = false;
            break;
        case 3: // RGB_ONLY
            write_depth = false;
            rgb_only = true;
            break;
        }
    }

    if (!write_frame && !write_depth) return false;

    // DATE has no effect on a 24-bit framebuffer. Otherwise DATM selects the
    // destination-alpha MSB that is allowed to pass.
    if (ctx.date && ctx.psm != 1u) {
        const bool destination_alpha =
            ctx.psm == 0u
                ? ((destination >> 31) & 1u) != 0
                : ((vram.read_pixel(ctx.psm, ux, uy, ctx.fbp, ctx.fbw) >> 15) & 1u) != 0;
        if (destination_alpha != ctx.datm) return false;
    }

    if (ctx.zte) {
        const u32 source_z = depth_value_for_psm(ctx.zpsm, z);
        const u32 destination_z = vram.read_depth(
            ctx.zpsm, ux, uy, ctx.zbp, ctx.fbw);
        if (!depth_test_pass(ctx.ztst, source_z, destination_z)) {
            return false;
        }
    }

    if (write_frame) {
        const bool blend_enabled =
            ctx.alpha_blend &&
            (!ctx.pabe || ((rgba & 0x80000000u) != 0));
        u32 output = blend_enabled ? blend_color(rgba, destination, ctx) : rgba;

        if (ctx.fba && !rgb_only) output |= 0x80000000u;

        if (ctx.psm == 0u) {
            u32 mask = ctx.fbmask;
            if (rgb_only) mask |= 0xFF000000u;
            const u32 old = vram.read_pixel(0, ux, uy, ctx.fbp, ctx.fbw);
            output = (old & mask) | (output & ~mask);
            if (!vram.write_pixel(0, ux, uy, ctx.fbp, ctx.fbw, output))
                return false;
        } else if (ctx.psm == 1u) {
            const u32 old = vram.read_pixel(1, ux, uy, ctx.fbp, ctx.fbw);
            const u32 mask = ctx.fbmask & 0x00FFFFFFu;
            output = (old & mask) | (output & ~mask & 0x00FFFFFFu);
            if (!vram.write_pixel(1, ux, uy, ctx.fbp, ctx.fbw, output))
                return false;
        } else {
            u16 packed = rgba32_to_16(output);
            if (rgb_only) {
                const u16 old = static_cast<u16>(
                    vram.read_pixel(ctx.psm, ux, uy, ctx.fbp, ctx.fbw));
                packed = static_cast<u16>((packed & 0x7FFFu) | (old & 0x8000u));
            }
            if (!vram.write_pixel(ctx.psm, ux, uy, ctx.fbp, ctx.fbw, packed))
                return false;
        }
    }

    if (write_depth) {
        const u32 source_z = depth_value_for_psm(ctx.zpsm, z);
        if (!vram.write_depth(ctx.zpsm, ux, uy, ctx.zbp, ctx.fbw, source_z))
            return false;
    }

    return true;
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
            if (draw_pixel(vram, ctx, x, y, b.z, rgba)) ++pixels;
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
            const u32 vertex_rgba = ctx.gouraud
                ? interpolate_rgba(
                    w0, w1, w2, area, a.rgba, b.rgba, c.rgba)
                : c.rgba;
            const u32 rgba = shade_pixel(
                vram, ctx.texture, u, v, vertex_rgba);
            const u32 z = interpolate_z(
                w0, w1, w2, area, a.z, b.z, c.z);
            if (draw_pixel(vram, ctx, x, y, z, rgba)) ++pixels;
        }
    }
    return pixels;
}

} // namespace ps2
