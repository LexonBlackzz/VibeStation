#include "core/gs/gs_rasterizer.h"

#include "core/gs/gs_vram.h"

#include <algorithm>
#include <array>
#include <cmath>
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


s32 dither_value(u64 dimx, s32 x, s32 y) {
    const u32 index =
        (static_cast<u32>(y) & 3u) * 4u +
        (static_cast<u32>(x) & 3u);
    const u32 raw = static_cast<u32>((dimx >> (index * 4u)) & 0x7u);
    return (raw & 0x4u) != 0
        ? static_cast<s32>(raw) - 8
        : static_cast<s32>(raw);
}

u32 interpolate_scalar(
    s64 w0,
    s64 w1,
    s64 w2,
    s64 area,
    u32 a,
    u32 b,
    u32 c) {
    const s64 numerator =
        w0 * static_cast<s64>(a) +
        w1 * static_cast<s64>(b) +
        w2 * static_cast<s64>(c);
    return static_cast<u32>(
        std::clamp<s64>(numerator / area, 0, 255));
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

s32 stq_to_fixed(float coordinate, float q, u32 size) {
    if (!std::isfinite(coordinate) || !std::isfinite(q) ||
        std::fabs(q) < 1.0e-20f || size == 0) {
        return 0;
    }

    const long double fixed =
        (static_cast<long double>(coordinate) / static_cast<long double>(q)) *
        static_cast<long double>(size) * 16.0L;
    const long double lo =
        static_cast<long double>(std::numeric_limits<s32>::min());
    const long double hi =
        static_cast<long double>(std::numeric_limits<s32>::max());
    return static_cast<s32>(std::clamp(fixed, lo, hi));
}

s32 st_to_fixed_scaled(
    float coordinate,
    long double scale) {
    if (!std::isfinite(coordinate)) return 0;
    const long double fixed =
        static_cast<long double>(coordinate) * scale;
    const long double lo =
        static_cast<long double>(std::numeric_limits<s32>::min());
    const long double hi =
        static_cast<long double>(std::numeric_limits<s32>::max());
    return static_cast<s32>(std::clamp(fixed, lo, hi));
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
    if (ctx.zte && !GsVram::supported_depth_psm(ctx.zpsm)) return false;
    return true;
}

bool GsRasterizer::supported_texture(const GsTextureState& texture) {
    if (!texture.enabled) return true;
    if (texture.bw == 0 || texture.width == 0 || texture.height == 0)
        return false;
    if (texture.width > 1024u || texture.height > 1024u)
        return false;
    if (texture.tfx > 3u)
        return false;
    if (!GsVram::supported_texture_psm(texture.psm))
        return false;

    const bool indexed =
        texture.psm == 19u || texture.psm == 20u ||
        texture.psm == 27u || texture.psm == 36u ||
        texture.psm == 44u;
    if (indexed) {
        if (texture.cpsm != 0u && texture.cpsm != 1u &&
            texture.cpsm != 2u && texture.cpsm != 10u)
            return false;
        if (texture.csm2 && texture.clut_bw == 0)
            return false;
    }
    return true;
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

    const u32 x = static_cast<u32>(texel_u);
    const u32 y = static_cast<u32>(texel_v);
    const bool indexed =
        texture.psm == 19u || texture.psm == 20u ||
        texture.psm == 27u || texture.psm == 36u ||
        texture.psm == 44u;

    u32 texture_rgba = 0;
    if (indexed) {
        const u32 index = vram.read_index(
            texture.psm, x, y, texture.bp, texture.bw);
        texture_rgba = vram.read_clut_color(
            texture.psm,
            index,
            texture.cbp,
            texture.cpsm,
            texture.csm2,
            texture.csa,
            texture.clut_bw,
            texture.clut_u,
            texture.clut_v,
            texture.ta0,
            texture.ta1,
            texture.aem);
    } else {
        const u32 raw = vram.read_pixel(
            texture.psm, x, y, texture.bp, texture.bw);
        if (texture.psm == 0u) {
            texture_rgba = raw;
        } else if (texture.psm == 1u) {
            const u32 rgb = raw & 0x00FFFFFFu;
            const u32 alpha =
                (texture.aem && rgb == 0) ? 0u : (texture.ta0 & 0xFFu);
            texture_rgba = rgb | (alpha << 24);
        } else {
            const u16 color = static_cast<u16>(raw);
            const u32 alpha =
                (color & 0x8000u) != 0 ? (texture.ta1 & 0xFFu) :
                (texture.aem && (color & 0x7FFFu) == 0) ? 0u :
                (texture.ta0 & 0xFFu);
            texture_rgba =
                (alpha << 24) |
                ((static_cast<u32>(color) & 0x7C00u) << 9) |
                ((static_cast<u32>(color) & 0x03E0u) << 6) |
                ((static_cast<u32>(color) & 0x001Fu) << 3);
        }
    }

    if (texture.nonzero_samples != nullptr &&
        (texture_rgba & 0x00FFFFFFu) != 0u) {
        ++*texture.nonzero_samples;
        if (texture.first_sample_x != nullptr &&
            *texture.first_sample_x == 0xFFFFFFFFu) {
            *texture.first_sample_x = x;
            *texture.first_sample_y = y;
            *texture.first_sample_rgba = texture_rgba;
        }
    }
    if (texture.alpha_samples != nullptr &&
        (texture_rgba & 0xFF000000u) != 0u) {
        ++*texture.alpha_samples;
    }
    auto record_shaded = [&](u32 result) {
        if (texture.nonzero_shaded != nullptr &&
            (result & 0x00FFFFFFu) != 0u) {
            ++*texture.nonzero_shaded;
        }
        return result;
    };

    if (texture.tfx == 1u) { // DECAL
        const u32 alpha = texture.tcc
            ? (texture_rgba & 0xFF000000u)
            : (vertex_rgba & 0xFF000000u);
        return record_shaded((texture_rgba & 0x00FFFFFFu) | alpha);
    }

    const u32 vertex_alpha = channel(vertex_rgba, 24);

    if (texture.tfx == 2u || texture.tfx == 3u) {
        // HIGHLIGHT/HIGHLIGHT2:
        //   RGB = clamp((Ct * Cv) / 128 + Av)
        // HIGHLIGHT alpha adds Av to At when TCC is enabled, while
        // HIGHLIGHT2 preserves At. With TCC disabled both use Av.
        u32 out = 0;
        for (u32 shift : {0u, 8u, 16u}) {
            const u32 modulated =
                modulate_channel(
                    channel(texture_rgba, shift),
                    channel(vertex_rgba, shift));
            const u32 highlighted =
                std::min(255u, modulated + vertex_alpha);
            out |= highlighted << shift;
        }

        u32 alpha = vertex_alpha;
        if (texture.tcc) {
            const u32 texture_alpha = channel(texture_rgba, 24);
            alpha = texture.tfx == 2u
                ? std::min(255u, texture_alpha + vertex_alpha)
                : texture_alpha;
        }
        out |= alpha << 24;
        return record_shaded(out);
    }

    // MODULATE uses GS 1.7 fixed-point color math: component*component >> 7.
    u32 out = 0;
    out |= modulate_channel(channel(texture_rgba, 0), channel(vertex_rgba, 0));
    out |= modulate_channel(channel(texture_rgba, 8), channel(vertex_rgba, 8)) << 8;
    out |= modulate_channel(channel(texture_rgba, 16), channel(vertex_rgba, 16)) << 16;
    const u32 alpha = texture.tcc
        ? modulate_channel(channel(texture_rgba, 24), vertex_alpha)
        : vertex_alpha;
    out |= alpha << 24;
    return record_shaded(out);
}

u32 GsRasterizer::apply_fog(u32 rgba, u32 fog_color, u32 fog) {
    fog &= 0xFFu;
    u32 out = rgba & 0xFF000000u;
    for (u32 shift : {0u, 8u, 16u}) {
        const u32 source = channel(rgba, shift);
        const u32 target = channel(fog_color, shift);
        const u32 value =
            (source * fog + target * (256u - fog)) >> 8;
        out |= (value & 0xFFu) << shift;
    }
    return out;
}

u32 GsRasterizer::apply_dither(
    u32 rgba,
    const GsRasterContext& ctx,
    s32 x,
    s32 y) {
    if (!ctx.dither || (ctx.psm != 2u && ctx.psm != 10u)) return rgba;

    const s32 d = dither_value(ctx.dimx, x, y);
    u32 out = rgba & 0xFF000000u;
    for (u32 shift : {0u, 8u, 16u}) {
        s32 value = static_cast<s32>(channel(rgba, shift)) + d;
        if (ctx.color_clamp) value = std::clamp(value, 0, 255);
        else value &= 0xFF;
        out |= static_cast<u32>(value) << shift;
    }
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

    // SCANMSK 2 prohibits even lines; 3 prohibits odd lines. Value 1 is
    // reserved and is treated as normal rendering.
    if ((ctx.scanmask == 2u && (y & 1) == 0) ||
        (ctx.scanmask == 3u && (y & 1) != 0)) {
        return false;
    }

    const u32 ux = static_cast<u32>(x);
    const u32 uy = static_cast<u32>(y);
    if (ctx.nonzero_inputs != nullptr &&
        (rgba & 0x00FFFFFFu) != 0u) {
        ++*ctx.nonzero_inputs;
        if (ctx.nonzero_input_alpha != nullptr &&
            (rgba & 0xFF000000u) != 0u) {
            ++*ctx.nonzero_input_alpha;
            if (ctx.first_alpha_input_rgba != nullptr &&
                *ctx.first_alpha_input_rgba == 0u) {
                *ctx.first_alpha_input_rgba = rgba;
            }
        }
        if (ctx.first_input_rgba != nullptr &&
            *ctx.first_input_rgba == 0u) {
            *ctx.first_input_rgba = rgba;
        }
    }

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

    u32 frame_address = 0;
    bool frame_address_valid = false;
    auto load_frame_address = [&]() -> u32 {
        if (!frame_address_valid) {
            frame_address = GsVram::pixel_address_bytes(
                ctx.psm, ux, uy, ctx.fbp, ctx.fbw);
            frame_address_valid = true;
        }
        return frame_address;
    };

    u32 destination_raw = 0;
    bool destination_raw_valid = false;
    auto load_destination_raw = [&]() -> u32 {
        if (!destination_raw_valid) {
            destination_raw = vram.read_pixel_at_address(
                ctx.psm, load_frame_address());
            destination_raw_valid = true;
        }
        return destination_raw;
    };
    auto load_destination_rgba = [&]() -> u32 {
        const u32 raw = load_destination_raw();
        if (ctx.psm == 0u) return raw;
        if (ctx.psm == 1u) {
            return (raw & 0x00FFFFFFu) | 0x80000000u;
        }
        return rgba16_to_32(static_cast<u16>(raw));
    };

    // DATE has no effect on a 24-bit framebuffer. Otherwise DATM selects the
    // destination-alpha MSB that is allowed to pass.
    if (ctx.date && ctx.psm != 1u) {
        const u32 raw = load_destination_raw();
        const bool destination_alpha =
            ctx.psm == 0u
                ? ((raw >> 31) & 1u) != 0
                : ((raw >> 15) & 1u) != 0;
        if (destination_alpha != ctx.datm) return false;
    }

    u32 depth_address = 0;
    u32 source_z = 0;
    if (ctx.zte) {
        const u32 ztst = ctx.ztst & 3u;
        if (ztst == 0u) {
            return false; // NEVER
        }

        source_z = depth_value_for_psm(ctx.zpsm, z);
        if (write_depth || ztst >= 2u) {
            depth_address = GsVram::depth_address_bytes(
                ctx.zpsm, ux, uy, ctx.zbp, ctx.fbw);
        }

        // ZTST=ALWAYS does not depend on destination Z. Avoid the VRAM read
        // entirely; keep the address only when a depth write is required.
        if (ztst >= 2u) {
            const u32 destination_z =
                vram.read_depth_at_address(
                    ctx.zpsm, depth_address);
            if (!depth_test_pass(
                    ztst, source_z, destination_z)) {
                return false;
            }
        }
    }

    if (write_frame) {
        const bool blend_enabled =
            ctx.alpha_blend &&
            (!ctx.pabe || ((rgba & 0x80000000u) != 0));
        u32 output = blend_enabled
            ? blend_color(rgba, load_destination_rgba(), ctx)
            : rgba;

        if (ctx.fba && !rgb_only) output |= 0x80000000u;

        u32 written_color = 0;
        if (ctx.psm == 0u) {
            u32 mask = ctx.fbmask;
            if (rgb_only) mask |= 0xFF000000u;
            if (mask != 0u) {
                const u32 old = load_destination_raw();
                output = (old & mask) | (output & ~mask);
            }
            written_color = output & 0x00FFFFFFu;
            if (!vram.write_pixel_at_address_untracked(
                    0u, load_frame_address(), output)) {
                return false;
            }
        } else if (ctx.psm == 1u) {
            const u32 mask = ctx.fbmask & 0x00FFFFFFu;
            if (mask != 0u) {
                const u32 old =
                    load_destination_raw() & 0x00FFFFFFu;
                output = (old & mask) |
                         (output & ~mask & 0x00FFFFFFu);
            } else {
                output &= 0x00FFFFFFu;
            }
            written_color = output & 0x00FFFFFFu;
            if (!vram.write_pixel_at_address_untracked(
                    1u, load_frame_address(), output)) {
                return false;
            }
        } else {
            output = apply_dither(output, ctx, x, y);
            u16 packed = rgba32_to_16(output);

            // FRAME.FBMSK is expressed in 32-bit RGBA channel bit positions
            // even for PSMCT16/16S. Pack those mask bits to RGB5A1 exactly
            // like the GS software reference path.
            const u32 rb = ctx.fbmask & 0x00F800F8u;
            const u32 ga = ctx.fbmask & 0x8000F800u;
            u16 mask = static_cast<u16>(
                (ga >> 16) | (rb >> 9) | (ga >> 6) | (rb >> 3));
            if (rgb_only) mask = static_cast<u16>(mask | 0x8000u);

            if (mask != 0u) {
                const u16 old =
                    static_cast<u16>(load_destination_raw());
                packed = static_cast<u16>(
                    (old & mask) | (packed & static_cast<u16>(~mask)));
            }
            written_color = packed & 0x7FFFu;
            if (!vram.write_pixel_at_address_untracked(
                    ctx.psm, load_frame_address(), packed)) {
                return false;
            }
        }
        if (ctx.nonzero_colors != nullptr && written_color != 0u) {
            ++*ctx.nonzero_colors;
        }
    }

    if (write_depth) {
        if (!vram.write_depth_at_address_untracked(
                ctx.zpsm, depth_address, source_z)) {
            return false;
        }
    }

    return true;
}

u64 GsRasterizer::draw_point(
    GsVram& vram,
    const GsRasterContext& ctx,
    const GsRasterVertex& vertex) {
    if (!supported_target(ctx) || !supported_texture(ctx.texture)) return 0;

    const s32 x = static_cast<s32>(
        std::floor(static_cast<double>(vertex.x) / 16.0 + 0.5));
    const s32 y = static_cast<s32>(
        std::floor(static_cast<double>(vertex.y) / 16.0 + 0.5));

    s32 u = vertex.u;
    s32 v = vertex.v;
    if (ctx.texture.enabled && !ctx.texture.fst) {
        u = stq_to_fixed(vertex.s, vertex.q, ctx.texture.width);
        v = stq_to_fixed(vertex.t, vertex.q, ctx.texture.height);
    }

    u32 rgba = shade_pixel(
        vram, ctx.texture, u, v, vertex.rgba);
    if (ctx.fog_enabled) rgba = apply_fog(rgba, ctx.fog_color, vertex.fog);
    return draw_pixel(vram, ctx, x, y, vertex.z, rgba) ? 1u : 0u;
}

u64 GsRasterizer::draw_line(
    GsVram& vram,
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b) {
    if (!supported_target(ctx) || !supported_texture(ctx.texture)) return 0;

    const double x0 = static_cast<double>(a.x) / 16.0;
    const double y0 = static_cast<double>(a.y) / 16.0;
    const double x1 = static_cast<double>(b.x) / 16.0;
    const double y1 = static_cast<double>(b.y) / 16.0;
    const double dx = x1 - x0;
    const double dy = y1 - y0;

    if (dx == 0.0 && dy == 0.0) {
        return draw_point(vram, ctx, b);
    }

    const bool step_x = std::abs(dx) >= std::abs(dy);
    const bool pos_x = dx >= 0.0;
    const bool pos_y = dy >= 0.0;
    const s32 dxi = pos_x ? 1 : -1;
    const s32 dyi = pos_y ? 1 : -1;

    double rx0 = std::floor(x0 + 0.5);
    double ry0 = std::floor(y0 + 0.5);
    double rx1 = std::floor(x1 + 0.5);
    double ry1 = std::floor(y1 + 0.5);

    // Match the GS diamond-exit endpoint rule used by the reference software
    // renderer. This matters for connected line strips: adjacent segments do
    // not double-draw their shared endpoint.
    const auto exits_diamond = [&](double ex, double ey) {
        const double distance = std::abs(ex) + std::abs(ey);
        if (distance < 0.5) return false;
        if (step_x) {
            const bool direction_ok = pos_x ? ex > 0.0 : ex < 0.0;
            return direction_ok &&
                   (distance > 0.5 || ey >= 0.0);
        }

        const bool direction_ok = pos_y ? ey > 0.0 : ey < 0.0;
        return direction_ok &&
               (distance > 0.5 || ex >= 0.0);
    };

    const bool draw_first = !exits_diamond(x0 - rx0, y0 - ry0);
    const bool draw_last = exits_diamond(x1 - rx1, y1 - ry1);

    if (!draw_first) {
        rx0 += step_x ? dxi : 0;
        ry0 += step_x ? 0 : dyi;
    }
    if (!draw_last) {
        rx1 -= step_x ? dxi : 0;
        ry1 -= step_x ? 0 : dyi;
    }

    const s32 start_major =
        static_cast<s32>(step_x ? rx0 : ry0);
    const s32 end_major =
        static_cast<s32>(step_x ? rx1 : ry1);
    const s32 major_step = step_x ? dxi : dyi;

    if ((major_step > 0 && start_major > end_major) ||
        (major_step < 0 && start_major < end_major)) {
        return 0;
    }

    auto interpolate_u32 = [](u32 lhs, u32 rhs, long double t) {
        const long double value =
            static_cast<long double>(lhs) +
            (static_cast<long double>(rhs) -
             static_cast<long double>(lhs)) * t;
        return static_cast<u32>(std::clamp(
            value,
            static_cast<long double>(0),
            static_cast<long double>(std::numeric_limits<u32>::max())));
    };
    auto interpolate_s32 = [](s32 lhs, s32 rhs, long double t) {
        const long double value =
            static_cast<long double>(lhs) +
            (static_cast<long double>(rhs) -
             static_cast<long double>(lhs)) * t;
        return static_cast<s32>(std::clamp(
            value,
            static_cast<long double>(std::numeric_limits<s32>::min()),
            static_cast<long double>(std::numeric_limits<s32>::max())));
    };
    auto interpolate_rgba_line = [](u32 lhs, u32 rhs, long double t) {
        u32 out = 0;
        for (u32 shift : {0u, 8u, 16u, 24u}) {
            const long double value =
                static_cast<long double>(channel(lhs, shift)) +
                (static_cast<long double>(channel(rhs, shift)) -
                 static_cast<long double>(channel(lhs, shift))) * t;
            const u32 component = static_cast<u32>(
                std::clamp(value, 0.0L, 255.0L));
            out |= component << shift;
        }
        return out;
    };

    u64 pixels = 0;
    for (s32 major = start_major;; major += major_step) {
        long double t = 0.0L;
        if (step_x) {
            t = dx != 0.0
                ? (static_cast<long double>(major) -
                   static_cast<long double>(x0)) /
                  static_cast<long double>(dx)
                : 0.0L;
        } else {
            t = dy != 0.0
                ? (static_cast<long double>(major) -
                   static_cast<long double>(y0)) /
                  static_cast<long double>(dy)
                : 0.0L;
        }
        t = std::clamp(t, 0.0L, 1.0L);

        const double dependent =
            step_x ? y0 + dy * static_cast<double>(t)
                   : x0 + dx * static_cast<double>(t);
        const s32 x = step_x
            ? major
            : static_cast<s32>(std::floor(dependent + 0.5));
        const s32 y = step_x
            ? static_cast<s32>(std::floor(dependent + 0.5))
            : major;

        s32 u = 0;
        s32 v = 0;
        if (ctx.texture.enabled && ctx.texture.fst) {
            u = interpolate_s32(a.u, b.u, t);
            v = interpolate_s32(a.v, b.v, t);
        } else if (ctx.texture.enabled) {
            const float s = static_cast<float>(
                static_cast<long double>(a.s) +
                (static_cast<long double>(b.s) -
                 static_cast<long double>(a.s)) * t);
            const float tex_t = static_cast<float>(
                static_cast<long double>(a.t) +
                (static_cast<long double>(b.t) -
                 static_cast<long double>(a.t)) * t);
            const float q = static_cast<float>(
                static_cast<long double>(a.q) +
                (static_cast<long double>(b.q) -
                 static_cast<long double>(a.q)) * t);
            u = stq_to_fixed(s, q, ctx.texture.width);
            v = stq_to_fixed(tex_t, q, ctx.texture.height);
        }

        const u32 vertex_rgba = ctx.gouraud
            ? interpolate_rgba_line(a.rgba, b.rgba, t)
            : b.rgba;
        u32 rgba = shade_pixel(
            vram, ctx.texture, u, v, vertex_rgba);
        if (ctx.fog_enabled) {
            const u32 fog = static_cast<u32>(std::clamp<long double>(
                static_cast<long double>(a.fog) +
                (static_cast<long double>(b.fog) -
                 static_cast<long double>(a.fog)) * t,
                0.0L, 255.0L));
            rgba = apply_fog(rgba, ctx.fog_color, fog);
        }
        const u32 z = interpolate_u32(a.z, b.z, t);
        if (draw_pixel(vram, ctx, x, y, z, rgba)) {
            ++pixels;
        }

        if (major == end_major) break;
    }

    return pixels;
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

    // FST sprite coordinates are separable: U depends only on X and V only
    // on Y. BIOS/OSDSYS draws many textured sprites, so computing both
    // 64-bit divisions for every pixel wastes most of the raster time.
    // Precompute each axis once while preserving the exact integer formula.
    std::array<s32, 2048> cached_u;
    std::array<s32, 2048> cached_v;
    const bool cached_fst = ctx.texture.enabled && ctx.texture.fst;
    if (cached_fst) {
        for (s32 x = left; x < right; ++x) {
            const s32 px = x * 16 + 8;
            cached_u[static_cast<std::size_t>(x - left)] =
                dx != 0
                    ? static_cast<s32>(
                        static_cast<s64>(a.u) +
                        (static_cast<s64>(b.u - a.u) * (px - a.x)) / dx)
                    : a.u;
        }
        for (s32 y = top; y < bottom; ++y) {
            const s32 py = y * 16 + 8;
            cached_v[static_cast<std::size_t>(y - top)] =
                dy != 0
                    ? static_cast<s32>(
                        static_cast<s64>(a.v) +
                        (static_cast<s64>(b.v - a.v) * (py - a.y)) / dy)
                    : a.v;
        }
    }

    u64 pixels = 0;
    for (s32 y = top; y < bottom; ++y) {
        const s32 py = y * 16 + 8;
        const s32 cached_row_v = cached_fst
            ? cached_v[static_cast<std::size_t>(y - top)]
            : 0;
        for (s32 x = left; x < right; ++x) {
            const s32 px = x * 16 + 8;
            s32 u = 0;
            s32 v = 0;
            if (ctx.texture.enabled) {
                if (ctx.texture.fst) {
                    u = cached_u[static_cast<std::size_t>(x - left)];
                    v = cached_row_v;
                } else {
                    const long double fx = dx != 0
                        ? static_cast<long double>(px - a.x) / static_cast<long double>(dx)
                        : 0.0L;
                    const long double fy = dy != 0
                        ? static_cast<long double>(py - a.y) / static_cast<long double>(dy)
                        : 0.0L;
                    const float s = static_cast<float>(
                        static_cast<long double>(a.s) +
                        static_cast<long double>(b.s - a.s) * fx);
                    const float t = static_cast<float>(
                        static_cast<long double>(a.t) +
                        static_cast<long double>(b.t - a.t) * fy);
                    const long double fq = (fx + fy) * 0.5L;
                    const float q = static_cast<float>(
                        static_cast<long double>(a.q) +
                        static_cast<long double>(b.q - a.q) * fq);
                    u = stq_to_fixed(s, q, ctx.texture.width);
                    v = stq_to_fixed(t, q, ctx.texture.height);
                }
            }
            u32 rgba = shade_pixel(vram, ctx.texture, u, v, b.rgba);
            if (ctx.fog_enabled) {
                const long double fx = dx != 0
                    ? std::clamp(
                        static_cast<long double>(px - a.x) /
                        static_cast<long double>(dx), 0.0L, 1.0L)
                    : 0.0L;
                const long double fy = dy != 0
                    ? std::clamp(
                        static_cast<long double>(py - a.y) /
                        static_cast<long double>(dy), 0.0L, 1.0L)
                    : 0.0L;
                const long double t_fog = (fx + fy) * 0.5L;
                const u32 fog = static_cast<u32>(std::clamp<long double>(
                    static_cast<long double>(a.fog) +
                    (static_cast<long double>(b.fog) -
                     static_cast<long double>(a.fog)) * t_fog,
                    0.0L, 255.0L));
                rgba = apply_fog(rgba, ctx.fog_color, fog);
            }
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
    const bool constant_q = a.q == b.q && b.q == c.q;
    const bool constant_rgba = a.rgba == b.rgba && b.rgba == c.rgba;
    const bool constant_z = a.z == b.z && b.z == c.z;
    const bool positive_area = area > 0;
    const long double inv_area =
        1.0L / static_cast<long double>(area);
    const bool scaled_constant_q =
        ctx.texture.enabled &&
        !ctx.texture.fst &&
        constant_q &&
        std::isfinite(a.q) &&
        std::fabs(a.q) >= 1.0e-20f;
    const long double constant_u_scale =
        scaled_constant_q
            ? (static_cast<long double>(ctx.texture.width) *
               16.0L / static_cast<long double>(a.q))
            : 0.0L;
    const long double constant_v_scale =
        scaled_constant_q
            ? (static_cast<long double>(ctx.texture.height) *
               16.0L / static_cast<long double>(a.q))
            : 0.0L;

    // Edge functions are affine in screen space.  Evaluate them once at the
    // top-left pixel centre, then advance by their exact 16.4 fixed-point
    // deltas instead of recomputing three 64-bit cross products per pixel.
    const s32 start_px = left * 16 + 8;
    const s32 start_py = top * 16 + 8;
    s64 row_w0 = edge(b, c, start_px, start_py);
    s64 row_w1 = edge(c, a, start_px, start_py);
    s64 row_w2 = edge(a, b, start_px, start_py);

    const s64 w0_dx = 16ll * static_cast<s64>(c.y - b.y);
    const s64 w1_dx = 16ll * static_cast<s64>(a.y - c.y);
    const s64 w2_dx = 16ll * static_cast<s64>(b.y - a.y);
    const s64 w0_dy = -16ll * static_cast<s64>(c.x - b.x);
    const s64 w1_dy = -16ll * static_cast<s64>(a.x - c.x);
    const s64 w2_dy = -16ll * static_cast<s64>(b.x - a.x);

    u64 pixels = 0;
    for (s32 y = top; y < bottom; ++y) {
        s64 w0 = row_w0;
        s64 w1 = row_w1;
        s64 w2 = row_w2;
        for (s32 x = left; x < right; ++x) {
            const bool inside =
                positive_area ? (w0 >= 0 && w1 >= 0 && w2 >= 0)
                              : (w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (inside) {
                s32 u = 0;
                s32 v = 0;
                if (ctx.texture.enabled) {
                    if (ctx.texture.fst) {
                        u = static_cast<s32>(
                            (w0 * a.u + w1 * b.u + w2 * c.u) / area);
                        v = static_cast<s32>(
                            (w0 * a.v + w1 * b.v + w2 * c.v) / area);
                    } else {
                        const float s = static_cast<float>(
                            (static_cast<long double>(w0) * a.s +
                             static_cast<long double>(w1) * b.s +
                             static_cast<long double>(w2) * c.s) * inv_area);
                        const float t = static_cast<float>(
                            (static_cast<long double>(w0) * a.t +
                             static_cast<long double>(w1) * b.t +
                             static_cast<long double>(w2) * c.t) * inv_area);
                        if (scaled_constant_q) {
                            u = st_to_fixed_scaled(
                                s, constant_u_scale);
                            v = st_to_fixed_scaled(
                                t, constant_v_scale);
                        } else {
                            const float q =
                                constant_q
                                    ? a.q
                                    : static_cast<float>(
                                        (static_cast<long double>(w0) * a.q +
                                         static_cast<long double>(w1) * b.q +
                                         static_cast<long double>(w2) * c.q) *
                                        inv_area);
                            u = stq_to_fixed(
                                s, q, ctx.texture.width);
                            v = stq_to_fixed(
                                t, q, ctx.texture.height);
                        }
                    }
                }
                const u32 vertex_rgba = ctx.gouraud && !constant_rgba
                    ? interpolate_rgba(
                        w0, w1, w2, area, a.rgba, b.rgba, c.rgba)
                    : c.rgba;
                u32 rgba = shade_pixel(
                    vram, ctx.texture, u, v, vertex_rgba);
                if (ctx.fog_enabled) {
                    const u32 fog = interpolate_scalar(
                        w0, w1, w2, area, a.fog, b.fog, c.fog);
                    rgba = apply_fog(rgba, ctx.fog_color, fog);
                }
                const u32 z = !ctx.zte ? 0u : constant_z ? a.z
                    : interpolate_z(w0, w1, w2, area, a.z, b.z, c.z);
                if (draw_pixel(vram, ctx, x, y, z, rgba)) ++pixels;
            }

            w0 += w0_dx;
            w1 += w1_dx;
            w2 += w2_dx;
        }
        row_w0 += w0_dy;
        row_w1 += w1_dy;
        row_w2 += w2_dy;
    }
    return pixels;
}

} // namespace ps2
