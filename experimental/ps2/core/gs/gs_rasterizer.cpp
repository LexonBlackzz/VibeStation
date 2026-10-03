#include "core/gs/gs_rasterizer.h"

#include "core/gs/gs_vram.h"

#include <algorithm>
#include <array>
#include <bit>
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

// Where, inside its pixel, a sprite's UV/ST is evaluated (12.4 fixed point:
// 0 = top-left corner, 8 = centre). The BIOS builds its 1:1 text sprites with
// a +0.5 texel UV offset, which only selects the intended texel when the
// attribute is taken at the corner.
constexpr s32 kSpriteUvOffset = 0;

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
    // Retail OSDSYS spends almost all of its PSMCT16 raster time in:
    //   A=Cs, B=ZERO, C=FIX, D=Cd, COLCLAMP=1
    // which reduces exactly to Cd + (Cs * FIX) / 128.  Keep this ahead of
    // the generic selector/divide path while preserving the GS integer
    // blend equation exactly.
    if (ctx.color_clamp &&
        (ctx.alpha_a & 3u) == 0u &&
        (ctx.alpha_b & 3u) == 2u &&
        (ctx.alpha_c & 3u) == 2u &&
        (ctx.alpha_d & 3u) == 1u) {
        const u32 fix = ctx.alpha_fix & 0xFFu;
        u32 output = source & 0xFF000000u;
        for (u32 shift : {0u, 8u, 16u}) {
            const u32 cs = channel(source, shift);
            const u32 cd = channel(destination, shift);
            const u32 value = cd + ((cs * fix) >> 7u);
            output |= std::min(255u, value) << shift;
        }
        return output;
    }

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

s64 floor_div(s64 numerator, s64 divisor) {
    s64 quotient = numerator / divisor;
    if ((numerator % divisor) != 0 && numerator < 0) --quotient;
    return quotient;
}

// Interpolates Gouraud color along a triangle row without dividing per pixel.
// Per channel the reference value is
//   clamp(trunc((w0*Ca + w1*Cb + w2*Cc) / area), 0, 255).
// Each channel numerator changes by a constant per pixel, so the floor
// quotient and remainder by |area| can be advanced incrementally. Because the
// result is clamped to [0, 255], floor and C++ truncation give the same value:
// they differ only for negative non-integral quotients, which both clamp to 0.
class GouraudStepper {
public:
    GouraudStepper(s64 area, u32 a, u32 b, u32 c,
                   s64 w0_dx, s64 w1_dx, s64 w2_dx)
        : magnitude_(area < 0 ? -area : area), sign_(area < 0 ? -1 : 1) {
        for (u32 i = 0; i < 4u; ++i) {
            const u32 shift = i * 8u;
            ca_[i] = static_cast<s64>(channel(a, shift));
            cb_[i] = static_cast<s64>(channel(b, shift));
            cc_[i] = static_cast<s64>(channel(c, shift));
            const s64 delta =
                sign_ * (w0_dx * ca_[i] + w1_dx * cb_[i] + w2_dx * cc_[i]);
            dq_[i] = floor_div(delta, magnitude_);
            dr_[i] = delta - dq_[i] * magnitude_;
        }
    }

    void begin_row(s64 w0, s64 w1, s64 w2) {
        for (u32 i = 0; i < 4u; ++i) {
            const s64 numerator =
                sign_ * (w0 * ca_[i] + w1 * cb_[i] + w2 * cc_[i]);
            q_[i] = floor_div(numerator, magnitude_);
            r_[i] = numerator - q_[i] * magnitude_;
        }
    }

    void step() {
        for (u32 i = 0; i < 4u; ++i) {
            q_[i] += dq_[i];
            r_[i] += dr_[i];
            if (r_[i] >= magnitude_) {
                r_[i] -= magnitude_;
                ++q_[i];
            }
        }
    }

    [[nodiscard]] u32 rgba() const {
        u32 out = 0;
        for (u32 i = 0; i < 4u; ++i) {
            out |= static_cast<u32>(std::clamp<s64>(q_[i], 0, 255))
                   << (i * 8u);
        }
        return out;
    }

private:
    s64 magnitude_;
    s64 sign_;
    s64 ca_[4]{}, cb_[4]{}, cc_[4]{};
    s64 q_[4]{}, r_[4]{};
    s64 dq_[4]{}, dr_[4]{};
};

// Bit-level equivalents of std::isfinite/std::fabs for float. MSVC does not
// always inline the <cmath> versions, and these run per STQ pixel.
bool float_is_finite(float value) {
    return (std::bit_cast<u32>(value) & 0x7F800000u) != 0x7F800000u;
}
float float_abs(float value) {
    return std::bit_cast<float>(std::bit_cast<u32>(value) & 0x7FFFFFFFu);
}

inline s32 stq_to_fixed(float coordinate, float q, u32 size) {
    if (!float_is_finite(coordinate) || !float_is_finite(q) ||
        float_abs(q) < 1.0e-20f || size == 0) {
        return 0;
    }

    const double fixed =
        (static_cast<double>(coordinate) / static_cast<double>(q)) *
        static_cast<double>(size) * 16.0;
    const double lo =
        static_cast<double>(std::numeric_limits<s32>::min());
    const double hi =
        static_cast<double>(std::numeric_limits<s32>::max());
    return static_cast<s32>(std::clamp(fixed, lo, hi));
}

inline s32 st_to_fixed_scaled(
    float coordinate,
    double scale) {
    if (!float_is_finite(coordinate)) return 0;
    const double fixed =
        static_cast<double>(coordinate) * scale;
    const double lo =
        static_cast<double>(std::numeric_limits<s32>::min());
    const double hi =
        static_cast<double>(std::numeric_limits<s32>::max());
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
    const double numerator =
        static_cast<double>(w0) * static_cast<double>(a) +
        static_cast<double>(w1) * static_cast<double>(b) +
        static_cast<double>(w2) * static_cast<double>(c);
    double value = numerator / static_cast<double>(area);
    value = std::clamp(
        value,
        0.0,
        static_cast<double>(std::numeric_limits<u32>::max()));
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

namespace {

// Raw 32/24/16-bit texel (formats 0, 1, 2, 10) to RGBA8888 with TEXA alpha.
u32 texel_from_raw(const GsTextureState& texture, u32 raw) {
    if (texture.psm == 0u) return raw;
    if (texture.psm == 1u) {
        const u32 rgb = raw & 0x00FFFFFFu;
        const u32 alpha =
            (texture.aem && rgb == 0) ? 0u : (texture.ta0 & 0xFFu);
        return rgb | (alpha << 24);
    }
    const u32 color = raw & 0xFFFFu;
    const u32 alpha =
        (color & 0x8000u) != 0 ? (texture.ta1 & 0xFFu) :
        (texture.aem && (color & 0x7FFFu) == 0) ? 0u :
        (texture.ta0 & 0xFFu);
    return (alpha << 24) |
           ((color & 0x7C00u) << 9) |
           ((color & 0x03E0u) << 6) |
           ((color & 0x001Fu) << 3);
}

// One texel as RGBA8888 after CLUT lookup and TEXA alpha expansion.
u32 texel_rgba(const GsVram& vram, const GsTextureState& texture,
               u32 x, u32 y) {
    const bool indexed =
        texture.psm == 19u || texture.psm == 20u ||
        texture.psm == 27u || texture.psm == 36u ||
        texture.psm == 44u;
    if (indexed) {
        const u32 index = vram.read_index(
            texture.psm, x, y, texture.bp, texture.bw);
        return vram.read_clut_color(
            texture.psm, index, texture.cbp, texture.cpsm, texture.csm2,
            texture.csa, texture.clut_bw, texture.clut_u, texture.clut_v,
            texture.ta0, texture.ta1, texture.aem);
    }
    if (texture.psm == 2u) {
        const u16 color = vram.read_psmct16(
            x, y, texture.bp, texture.bw);
        const u32 alpha =
            (color & 0x8000u) != 0 ? (texture.ta1 & 0xFFu) :
            (texture.aem && (color & 0x7FFFu) == 0) ? 0u :
            (texture.ta0 & 0xFFu);
        return (alpha << 24) |
               ((static_cast<u32>(color) & 0x7C00u) << 9) |
               ((static_cast<u32>(color) & 0x03E0u) << 6) |
               ((static_cast<u32>(color) & 0x001Fu) << 3);
    }
    const u32 raw = vram.read_pixel(
        texture.psm, x, y, texture.bp, texture.bw);
    if (texture.psm == 0u) return raw;
    if (texture.psm == 1u) {
        const u32 rgb = raw & 0x00FFFFFFu;
        const u32 alpha =
            (texture.aem && rgb == 0) ? 0u : (texture.ta0 & 0xFFu);
        return rgb | (alpha << 24);
    }
    const u16 color = static_cast<u16>(raw);
    const u32 alpha =
        (color & 0x8000u) != 0 ? (texture.ta1 & 0xFFu) :
        (texture.aem && (color & 0x7FFFu) == 0) ? 0u :
        (texture.ta0 & 0xFFu);
    return (alpha << 24) |
           ((static_cast<u32>(color) & 0x7C00u) << 9) |
           ((static_cast<u32>(color) & 0x03E0u) << 6) |
           ((static_cast<u32>(color) & 0x001Fu) << 3);
}

// GS bilinear filter: the sample point lies half a texel from the texel
// centres, and the blend weights have 4 bits of precision. When the point
// falls exactly on a texel centre (a 1:1 copy) only one texel is read.
u32 sample_bilinear(const GsVram& vram, const GsTextureState& texture,
                    s32 u, s32 v) {
    const s32 uu = u - 8;
    const s32 vv = v - 8;
    const u32 fx = static_cast<u32>(uu) & 15u;
    const u32 fy = static_cast<u32>(vv) & 15u;
    const s32 xi = uu >> 4;
    const s32 yi = vv >> 4;
    const u32 x0 = static_cast<u32>(wrap_coordinate(
        xi, texture.width, texture.wms, texture.minu, texture.maxu));
    const u32 y0 = static_cast<u32>(wrap_coordinate(
        yi, texture.height, texture.wmt, texture.minv, texture.maxv));
    const bool plain =
        texture.psm == 0u || texture.psm == 1u ||
        texture.psm == 2u || texture.psm == 10u;
    if (fx == 0u && fy == 0u) {
        if (!plain) return texel_rgba(vram, texture, x0, y0);
        u32 raw;
        vram.read_pixel_quad(
            texture.psm, texture.bp, texture.bw,
            x0, y0, x0, y0, false, false, &raw);
        return texel_from_raw(texture, raw);
    }
    const u32 x1 = static_cast<u32>(wrap_coordinate(
        xi + 1, texture.width, texture.wms, texture.minu, texture.maxu));
    const u32 y1 = static_cast<u32>(wrap_coordinate(
        yi + 1, texture.height, texture.wmt, texture.minv, texture.maxv));
    u32 c00;
    u32 c10;
    u32 c01;
    u32 c11;
    if (plain) {
        u32 raw[4] = {0, 0, 0, 0};
        vram.read_pixel_quad(
            texture.psm, texture.bp, texture.bw,
            x0, y0, x1, y1, fx != 0u, fy != 0u, raw);
        c00 = texel_from_raw(texture, raw[0]);
        c10 = fx != 0u ? texel_from_raw(texture, raw[1]) : c00;
        c01 = fy != 0u ? texel_from_raw(texture, raw[2]) : c00;
        c11 = (fx != 0u && fy != 0u)
            ? texel_from_raw(texture, raw[3])
            : (fx != 0u ? c10 : c01);
    } else {
        c00 = texel_rgba(vram, texture, x0, y0);
        c10 = fx != 0u ? texel_rgba(vram, texture, x1, y0) : c00;
        c01 = fy != 0u ? texel_rgba(vram, texture, x0, y1) : c00;
        c11 = (fx != 0u && fy != 0u)
            ? texel_rgba(vram, texture, x1, y1)
            : (fx != 0u ? c10 : c01);
    }
    // Same arithmetic as per-channel (a*(16-fx)+b*fx)*(16-fy)+... >> 8, with
    // two channels per word: every 16-bit lane stays below 65536.
    constexpr u32 kLanes = 0x00FF00FFu;
    const u32 wx0 = 16u - fx;
    const u32 wy0 = 16u - fy;
    const u32 top_rb = (c00 & kLanes) * wx0 + (c10 & kLanes) * fx;
    const u32 bottom_rb = (c01 & kLanes) * wx0 + (c11 & kLanes) * fx;
    const u32 top_ag = ((c00 >> 8) & kLanes) * wx0 +
                       ((c10 >> 8) & kLanes) * fx;
    const u32 bottom_ag = ((c01 >> 8) & kLanes) * wx0 +
                          ((c11 >> 8) & kLanes) * fx;
    const u32 rb = ((top_rb * wy0 + bottom_rb * fy) >> 8) & kLanes;
    const u32 ag = ((top_ag * wy0 + bottom_ag * fy) >> 8) & kLanes;
    return rb | (ag << 8);
}

// TFX stage on an already sampled texel (no tracing hooks).
u32 apply_texture_function(
    const GsTextureState& texture, u32 texture_rgba, u32 vertex_rgba) {
    if (texture.tfx == 1u) { // DECAL
        const u32 alpha = texture.tcc
            ? (texture_rgba & 0xFF000000u)
            : (vertex_rgba & 0xFF000000u);
        return (texture_rgba & 0x00FFFFFFu) | alpha;
    }
    if (texture.tfx == 0u && vertex_rgba == 0x80808080u) {
        // MODULATE by 1.0 is the identity on colour (t * 128 >> 7).
        return texture.tcc
            ? texture_rgba
            : ((texture_rgba & 0x00FFFFFFu) | 0x80000000u);
    }
    const u32 vertex_alpha = channel(vertex_rgba, 24);
    if (texture.tfx == 2u || texture.tfx == 3u) { // HIGHLIGHT(2)
        u32 out = 0;
        for (u32 shift : {0u, 8u, 16u}) {
            const u32 modulated = modulate_channel(
                channel(texture_rgba, shift), channel(vertex_rgba, shift));
            out |= std::min(255u, modulated + vertex_alpha) << shift;
        }
        u32 alpha = vertex_alpha;
        if (texture.tcc) {
            const u32 texture_alpha = channel(texture_rgba, 24);
            alpha = texture.tfx == 2u
                ? std::min(255u, texture_alpha + vertex_alpha)
                : texture_alpha;
        }
        return out | (alpha << 24);
    }
    u32 out = 0;
    out |= modulate_channel(channel(texture_rgba, 0), channel(vertex_rgba, 0));
    out |= modulate_channel(channel(texture_rgba, 8), channel(vertex_rgba, 8)) << 8;
    out |= modulate_channel(channel(texture_rgba, 16), channel(vertex_rgba, 16)) << 16;
    const u32 alpha = texture.tcc
        ? modulate_channel(channel(texture_rgba, 24), vertex_alpha)
        : vertex_alpha;
    return out | (alpha << 24);
}

} // namespace

u32 GsRasterizer::shade_pixel(
    const GsVram& vram,
    const GsTextureState& texture,
    s32 u,
    s32 v,
    u32 vertex_rgba) {
    if (!texture.enabled) return vertex_rgba;
    if (texture.linear) {
        return apply_texture_function(
            texture, sample_bilinear(vram, texture, u, v), vertex_rgba);
    }

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

    // Fast common path: PSMCT16 + MODULATE with tracing disabled.
    // A 5-bit channel expands as c5<<3, so the GS modulation
    // ((c5<<3) * Cv) >> 7 is exactly (c5 * Cv) >> 4.
    if (texture.psm == 2u &&
        texture.tfx == 0u &&
        texture.nonzero_samples == nullptr &&
        texture.alpha_samples == nullptr &&
        texture.first_sample_x == nullptr &&
        texture.first_sample_y == nullptr &&
        texture.first_sample_rgba == nullptr &&
        texture.nonzero_shaded == nullptr) {
        const u16 color = vram.read_psmct16(
            x, y, texture.bp, texture.bw);
        const u32 r5 = color & 0x1Fu;
        const u32 g5 = (color >> 5u) & 0x1Fu;
        const u32 b5 = (color >> 10u) & 0x1Fu;
        const u32 vr = channel(vertex_rgba, 0u);
        const u32 vg = channel(vertex_rgba, 8u);
        const u32 vb = channel(vertex_rgba, 16u);
        const u32 va = channel(vertex_rgba, 24u);
        u32 out = 0u;
        out |= std::min(255u, (r5 * vr) >> 4u);
        out |= std::min(255u, (g5 * vg) >> 4u) << 8u;
        out |= std::min(255u, (b5 * vb) >> 4u) << 16u;
        u32 alpha =
            (color & 0x8000u) != 0u ? (texture.ta1 & 0xFFu) :
            (texture.aem && (color & 0x7FFFu) == 0u) ? 0u :
            (texture.ta0 & 0xFFu);
        if (texture.tcc) {
            alpha = modulate_channel(alpha, va);
        } else {
            alpha = va;
        }
        out |= alpha << 24u;
        return out;
    }

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
        if (texture.psm == 2u) {
            const u16 color = vram.read_psmct16(
                x, y, texture.bp, texture.bw);
            const u32 alpha =
                (color & 0x8000u) != 0 ? (texture.ta1 & 0xFFu) :
                (texture.aem && (color & 0x7FFFu) == 0) ? 0u :
                (texture.ta0 & 0xFFu);
            texture_rgba =
                (alpha << 24) |
                ((static_cast<u32>(color) & 0x7C00u) << 9) |
                ((static_cast<u32>(color) & 0x03E0u) << 6) |
                ((static_cast<u32>(color) & 0x001Fu) << 3);
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

bool simple_frame_write(const GsRasterContext& ctx) {
    const bool depth_noop =
        !ctx.zte || (((ctx.ztst & 3u) == 1u) && ctx.zmask);
    const bool dither_noop =
        !ctx.dither || (ctx.psm != 2u && ctx.psm != 10u);
    return (ctx.scanmask & 2u) == 0u &&
           !ctx.ate &&
           !ctx.date &&
           depth_noop &&
           !ctx.alpha_blend &&
           !ctx.fba &&
           ctx.fbmask == 0u &&
           dither_noop &&
           ctx.nonzero_colors == nullptr &&
           ctx.nonzero_inputs == nullptr &&
           ctx.nonzero_input_alpha == nullptr &&
           ctx.first_input_rgba == nullptr &&
           ctx.first_alpha_input_rgba == nullptr;
}

bool hot_osdsys_psm16_pixel_state(const GsRasterContext& ctx) {
    return ctx.texture.enabled &&
           ctx.texture.psm == 2u &&
           ctx.psm == 0u &&
           ctx.zte &&
           ((ctx.ztst & 3u) == 1u || (ctx.ztst & 3u) == 2u) &&
           (ctx.zpsm == 48u || ctx.zpsm == 49u) &&
           ctx.zmask &&
           ctx.alpha_blend &&
           !ctx.pabe &&
           (ctx.alpha_a & 3u) == 0u &&
           (ctx.alpha_b & 3u) == 2u &&
           (ctx.alpha_c & 3u) == 2u &&
           (ctx.alpha_d & 3u) == 1u &&
           ctx.color_clamp &&
           !ctx.ate &&
           !ctx.date &&
           !ctx.fba &&
           ctx.fbmask == 0u &&
           (ctx.scanmask & 2u) == 0u &&
           ctx.nonzero_colors == nullptr &&
           ctx.nonzero_inputs == nullptr &&
           ctx.nonzero_input_alpha == nullptr &&
           ctx.first_input_rgba == nullptr &&
           ctx.first_alpha_input_rgba == nullptr;
}

bool draw_hot_osdsys_psm16_pixel(
    GsVram& vram,
    const GsRasterContext& ctx,
    s32 x,
    s32 y,
    u32 z,
    u32 source) {
    const u32 ux = static_cast<u32>(x);
    const u32 uy = static_cast<u32>(y);

    u32 frame_address = 0u;
    u32 depth_address = 0u;
    GsVram::color_depth32_addresses(
        ux, uy, ctx.fbp, ctx.zbp, ctx.fbw,
        frame_address, depth_address);
    if ((ctx.ztst & 3u) == 2u) { // GEQUAL; ALWAYS needs no depth read
        const u32 source_z = depth_value_for_psm(ctx.zpsm, z);
        if (source_z < vram.read_depth_at_address(ctx.zpsm, depth_address))
            return false;
    }

    const u32 destination =
        vram.read_pixel_at_address(0u, frame_address);

    u32 output = source & 0xFF000000u;
    const u32 sr = source & 0xFFu;
    const u32 sg = (source >> 8u) & 0xFFu;
    const u32 sb = (source >> 16u) & 0xFFu;
    const u32 dr = destination & 0xFFu;
    const u32 dg = (destination >> 8u) & 0xFFu;
    const u32 db = (destination >> 16u) & 0xFFu;
    const u32 fix = ctx.alpha_fix & 0xFFu;
    output |= std::min(255u, dr + ((sr * fix) >> 7u));
    output |= std::min(255u, dg + ((sg * fix) >> 7u)) << 8u;
    output |= std::min(255u, db + ((sb * fix) >> 7u)) << 16u;

    return vram.write_pixel_at_address_untracked(
        0u, frame_address, output);
}

bool draw_simple_frame_pixel(
    GsVram& vram,
    const GsRasterContext& ctx,
    s32 x,
    s32 y,
    u32 rgba) {
    const u32 address = GsVram::pixel_address_bytes(
        ctx.psm,
        static_cast<u32>(x),
        static_cast<u32>(y),
        ctx.fbp,
        ctx.fbw);
    if (ctx.psm == 0u) {
        return vram.write_pixel_at_address_untracked(0u, address, rgba);
    }
    if (ctx.psm == 1u) {
        return vram.write_pixel_at_address_untracked(
            1u, address, rgba & 0x00FFFFFFu);
    }
    return vram.write_pixel_at_address_untracked(
        ctx.psm, address, rgba32_to_16(rgba));
}

// Opaque writes with ZTST=ALWAYS and depth writes enabled: the pixel is
// stored unconditionally together with its depth. This is the shape of the
// BIOS menu's full-screen copy/blit sprites, which otherwise fall into the
// fully general draw_pixel().
bool simple_zwrite_state(const GsRasterContext& ctx) {
    return ctx.zte &&
           (ctx.ztst & 3u) == 1u &&
           !ctx.zmask &&
           (ctx.zpsm == 48u || ctx.zpsm == 49u) &&
           (ctx.psm == 0u || ctx.psm == 1u) &&
           (ctx.scanmask & 2u) == 0u &&
           !ctx.ate &&
           !ctx.date &&
           !ctx.alpha_blend &&
           !ctx.fba &&
           ctx.fbmask == 0u &&
           ctx.nonzero_colors == nullptr &&
           ctx.nonzero_inputs == nullptr &&
           ctx.nonzero_input_alpha == nullptr &&
           ctx.first_input_rgba == nullptr &&
           ctx.first_alpha_input_rgba == nullptr;
}

bool draw_simple_zwrite_pixel(
    GsVram& vram,
    const GsRasterContext& ctx,
    s32 x,
    s32 y,
    u32 z,
    u32 rgba) {
    u32 frame_address = 0u;
    u32 depth_address = 0u;
    GsVram::color_depth32_addresses(
        static_cast<u32>(x), static_cast<u32>(y), ctx.fbp, ctx.zbp,
        ctx.fbw, frame_address, depth_address);
    vram.write_pixel_at_address_untracked(
        ctx.psm, frame_address,
        ctx.psm == 0u ? rgba : (rgba & 0x00FFFFFFu));
    return vram.write_depth_at_address_untracked(
        ctx.zpsm, depth_address, depth_value_for_psm(ctx.zpsm, z));
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

bool parallel_sprite_vram_safe(
    const GsRasterContext& ctx,
    s32 left,
    s32 right,
    s32 top,
    s32 bottom,
    const GsRasterVertex& va,
    const GsRasterVertex& vb,
    bool require_fst = true) {
    if (left < 0 || top < 0 ||
        right <= left || bottom <= top ||
        ctx.fbw == 0u ||
        static_cast<u32>(right) > ctx.fbw * 64u ||
        ctx.nonzero_colors != nullptr ||
        ctx.nonzero_inputs != nullptr ||
        ctx.nonzero_input_alpha != nullptr ||
        ctx.first_input_rgba != nullptr ||
        ctx.first_alpha_input_rgba != nullptr) {
        return false;
    }

    if (ctx.texture.enabled) {
        // The texture range below covers the whole texture, so it bounds
        // STQ sampling as well; sprites keep their original FST-only rule.
        if ((require_fst && !ctx.texture.fst) ||
            (ctx.texture.psm != 0u &&
             ctx.texture.psm != 1u &&
             ctx.texture.psm != 2u &&
             ctx.texture.psm != 10u) ||
            ctx.texture.nonzero_samples != nullptr ||
            ctx.texture.alpha_samples != nullptr ||
            ctx.texture.first_sample_x != nullptr ||
            ctx.texture.first_sample_y != nullptr ||
            ctx.texture.first_sample_rgba != nullptr ||
            ctx.texture.nonzero_shaded != nullptr) {
            return false;
        }
    }

    auto page_height = [](u32 psm) -> u32 {
        if (psm == 0u || psm == 1u ||
            psm == 48u || psm == 49u) {
            return 32u;
        }
        if (psm == 2u || psm == 10u ||
            psm == 50u || psm == 58u) {
            return 64u;
        }
        return 0u;
    };

    const auto screen_range = [&](
        u32 bp,
        u32 bw,
        u32 psm,
        u64& begin,
        u64& finish) -> bool {
        const u32 ph = page_height(psm);
        if (ph == 0u || bw == 0u) return false;

        const u32 min_page_x =
            static_cast<u32>(left) >> 6u;
        const u32 max_page_x =
            static_cast<u32>(right - 1) >> 6u;
        const u32 min_page_y =
            static_cast<u32>(top) / ph;
        const u32 max_page_y =
            static_cast<u32>(bottom - 1) / ph;
        if (max_page_x >= bw) return false;

        const u64 base = static_cast<u64>(bp) * 256u;
        const u64 first_page =
            static_cast<u64>(min_page_y) * bw +
            min_page_x;
        const u64 last_page =
            static_cast<u64>(max_page_y) * bw +
            max_page_x;
        begin = base + first_page * 8192u;
        finish = base + (last_page + 1u) * 8192u;
        return finish <= GsVram::kSize;
    };

    const auto texture_range = [&](
        u64& begin,
        u64& finish) -> bool {
        const u32 ph = page_height(ctx.texture.psm);
        if (ph == 0u ||
            ctx.texture.bw == 0u ||
            ctx.texture.width == 0u ||
            ctx.texture.height == 0u) {
            return false;
        }
        // An FST sprite only samples between its first and last clipped
        // pixel centres (same integer interpolation as draw_sprite_rows).
        // When REPEAT/CLAMP cannot remap those coordinates (they lie inside
        // the texture), bound the pages by that extent instead of the whole
        // texture; a 1024-wide texture in a 640-wide buffer would otherwise
        // always be rejected. One 10.4 unit of slack covers truncation.
        u32 used_width = ctx.texture.width;
        u32 used_height = ctx.texture.height;
        // Only an FST sprite (va/vb are its corners) has this linear extent.
        if (require_fst &&
            (ctx.texture.wms & 3u) < 2u &&
            (ctx.texture.wmt & 3u) < 2u) {
            const auto interpolate = [](s32 a_t, s32 b_t, s32 a_c,
                                        s32 b_c, s32 c) -> s32 {
                const s64 d = static_cast<s64>(b_c) - a_c;
                return d != 0
                    ? static_cast<s32>(
                          static_cast<s64>(a_t) +
                          (static_cast<s64>(b_t - a_t) * (c - a_c)) / d)
                    : a_t;
            };
            const s32 u_first = interpolate(
                va.u, vb.u, va.x, vb.x, left * 16 + kSpriteUvOffset);
            const s32 u_last = interpolate(
                va.u, vb.u, va.x, vb.x, (right - 1) * 16 + kSpriteUvOffset);
            const s32 v_first = interpolate(
                va.v, vb.v, va.y, vb.y, top * 16 + kSpriteUvOffset);
            const s32 v_last = interpolate(
                va.v, vb.v, va.y, vb.y, (bottom - 1) * 16 + kSpriteUvOffset);
            // Bilinear also reads the neighbouring texel on each side.
            const s32 slack = ctx.texture.linear ? 16 : 1;
            const s32 u_lo = (std::min(u_first, u_last) - slack) >> 4;
            const s32 u_hi = (std::max(u_first, u_last) + slack) >> 4;
            const s32 v_lo = (std::min(v_first, v_last) - slack) >> 4;
            const s32 v_hi = (std::max(v_first, v_last) + slack) >> 4;
            if (u_lo >= 0 && v_lo >= 0 &&
                u_hi < static_cast<s32>(ctx.texture.width) &&
                v_hi < static_cast<s32>(ctx.texture.height)) {
                used_width = static_cast<u32>(u_hi) + 1u;
                used_height = static_cast<u32>(v_hi) + 1u;
            }
        }
        const u32 pages_x = (used_width + 63u) >> 6u;
        const u32 pages_y = (used_height + ph - 1u) / ph;
        // Pages are addressed linearly (page_y * bw + page_x), so this hull
        // also bounds texels that run past the buffer width into the next
        // page row; no need to reject pages_x > bw.
        if (pages_x == 0u || pages_y == 0u) {
            return false;
        }

        const u64 base =
            static_cast<u64>(ctx.texture.bp) * 256u;
        const u64 last_page =
            static_cast<u64>(pages_y - 1u) *
                ctx.texture.bw +
            (pages_x - 1u);
        begin = base;
        finish = base + (last_page + 1u) * 8192u;
        return finish <= GsVram::kSize;
    };

    const auto disjoint = [](
        u64 a0, u64 a1, u64 b0, u64 b1) {
        return a1 <= b0 || b1 <= a0;
    };

    u64 frame_begin = 0u;
    u64 frame_end = 0u;
    if (!screen_range(
            ctx.fbp,
            ctx.fbw,
            ctx.psm,
            frame_begin,
            frame_end)) {
        return false;
    }

    u64 depth_begin = 0u;
    u64 depth_end = 0u;
    if (ctx.zte) {
        if (!screen_range(
                ctx.zbp,
                ctx.fbw,
                ctx.zpsm,
                depth_begin,
                depth_end)) {
            return false;
        }
        // Frame writes must never perturb another worker's depth test.
        if (!disjoint(
                frame_begin, frame_end,
                depth_begin, depth_end)) {
            return false;
        }
    }

    if (!ctx.texture.enabled) return true;

    u64 source_begin = 0u;
    u64 source_end = 0u;
    if (!texture_range(source_begin, source_end)) {
        return false;
    }

    // Texture feedback makes pixel order observable inside the primitive.
    if (!disjoint(
            source_begin, source_end,
            frame_begin, frame_end)) {
        return false;
    }

    // A depth-writing primitive can also feed back through a texture.
    if (ctx.zte && !ctx.zmask &&
        !disjoint(
            source_begin, source_end,
            depth_begin, depth_end)) {
        return false;
    }

    return true;
}

bool GsRasterizer::parallel_sprite_plan(
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    s32& top,
    s32& bottom,
    u64& area) {
    if (!supported_target(ctx) ||
        !supported_texture(ctx.texture)) {
        return false;
    }

    s32 left = ceil_div16(std::min(a.x, b.x));
    s32 right = ceil_div16(std::max(a.x, b.x));
    top = ceil_div16(std::min(a.y, b.y));
    bottom = ceil_div16(std::max(a.y, b.y));

    left = std::max(left, ctx.scax0);
    right = std::min(right, ctx.scax1 + 1);
    top = std::max(top, ctx.scay0);
    bottom = std::min(bottom, ctx.scay1 + 1);
    if (left >= right || top >= bottom) return false;

    area =
        static_cast<u64>(right - left) *
        static_cast<u64>(bottom - top);
    // Below this the helper wake-up costs more than it saves.
    if (area < 16384u || bottom - top < 16) {
        return false;
    }

    return parallel_sprite_vram_safe(
        ctx, left, right, top, bottom, a, b);
}

u64 GsRasterizer::draw_sprite(
    GsVram& vram,
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b) {
    return draw_sprite_rows(
        vram, ctx, a, b, ctx.scay0, ctx.scay1 + 1);
}

u64 GsRasterizer::draw_sprite_rows(
    GsVram& vram,
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    s32 row_begin,
    s32 row_end) {
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
    top = std::max(top, row_begin);
    bottom = std::min(bottom, row_end);

    if (left >= right || top >= bottom) return 0;

    const s64 dx = static_cast<s64>(b.x) - a.x;
    const s64 dy = static_cast<s64>(b.y) - a.y;

    std::array<s32, 2048> cached_u;
    std::array<s32, 2048> cached_v;
    const bool cached_fst = ctx.texture.enabled && ctx.texture.fst;
    if (cached_fst) {
        for (s32 x = left; x < right; ++x) {
            const s32 px = x * 16 + kSpriteUvOffset;
            cached_u[static_cast<std::size_t>(x - left)] =
                dx != 0
                    ? static_cast<s32>(
                        static_cast<s64>(a.u) +
                        (static_cast<s64>(b.u - a.u) * (px - a.x)) / dx)
                    : a.u;
        }
        for (s32 y = top; y < bottom; ++y) {
            const s32 py = y * 16 + kSpriteUvOffset;
            cached_v[static_cast<std::size_t>(y - top)] =
                dy != 0
                    ? static_cast<s32>(
                        static_cast<s64>(a.v) +
                        (static_cast<s64>(b.v - a.v) * (py - a.y)) / dy)
                    : a.v;
        }
    }

    const bool hot_psm16_pixels =
        hot_osdsys_psm16_pixel_state(ctx);
    const bool simple_pixels =
        !hot_psm16_pixels && simple_frame_write(ctx);
    const bool zwrite_pixels =
        !hot_psm16_pixels && !simple_pixels && simple_zwrite_state(ctx);
    u64 pixels = 0;
    for (s32 y = top; y < bottom; ++y) {
        const s32 py = y * 16 + kSpriteUvOffset;
        const s32 cached_row_v = cached_fst
            ? cached_v[static_cast<std::size_t>(y - top)]
            : 0;
        for (s32 x = left; x < right; ++x) {
            const s32 px = x * 16 + kSpriteUvOffset;
            s32 u = 0;
            s32 v = 0;
            if (ctx.texture.enabled) {
                if (ctx.texture.fst) {
                    u = cached_u[static_cast<std::size_t>(x - left)];
                    v = cached_row_v;
                } else {
                    const long double fx = dx != 0
                        ? static_cast<long double>(px - a.x) /
                          static_cast<long double>(dx)
                        : 0.0L;
                    const long double fy = dy != 0
                        ? static_cast<long double>(py - a.y) /
                          static_cast<long double>(dy)
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
            u32 rgba = shade_pixel(
                vram, ctx.texture, u, v, b.rgba);
            if (ctx.fog_enabled) {
                const long double fx = dx != 0
                    ? std::clamp(
                        static_cast<long double>(px - a.x) /
                        static_cast<long double>(dx),
                        0.0L, 1.0L)
                    : 0.0L;
                const long double fy = dy != 0
                    ? std::clamp(
                        static_cast<long double>(py - a.y) /
                        static_cast<long double>(dy),
                        0.0L, 1.0L)
                    : 0.0L;
                const long double t_fog = (fx + fy) * 0.5L;
                const u32 fog = static_cast<u32>(
                    std::clamp<long double>(
                        static_cast<long double>(a.fog) +
                        (static_cast<long double>(b.fog) -
                         static_cast<long double>(a.fog)) * t_fog,
                        0.0L, 255.0L));
                rgba = apply_fog(rgba, ctx.fog_color, fog);
            }
            const bool wrote = hot_psm16_pixels
                ? draw_hot_osdsys_psm16_pixel(
                    vram, ctx, x, y, b.z, rgba)
                : simple_pixels
                    ? draw_simple_frame_pixel(
                        vram, ctx, x, y, rgba)
                    : zwrite_pixels
                        ? draw_simple_zwrite_pixel(
                            vram, ctx, x, y, b.z, rgba)
                        : draw_pixel(
                            vram, ctx, x, y, b.z, rgba);
            if (wrote) ++pixels;
        }
    }
    return pixels;
}

bool GsRasterizer::triangle_row_span(
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    const GsRasterVertex& c,
    s32& top,
    s32& bottom) {
    top = std::max(floor_div16(std::min({a.y, b.y, c.y})), ctx.scay0);
    bottom = std::min(
        ceil_div16(std::max({a.y, b.y, c.y})), ctx.scay1 + 1);
    return top < bottom;
}

bool GsRasterizer::band_parallel_safe(const GsRasterContext& ctx) {
    return supported_target(ctx) &&
           supported_texture(ctx.texture) &&
           parallel_sprite_vram_safe(
               ctx,
               ctx.scax0,
               ctx.scax1 + 1,
               ctx.scay0,
               ctx.scay1 + 1,
               GsRasterVertex{},
               GsRasterVertex{},
               false);
}

bool GsRasterizer::band_plan(
    const GsRasterContext& ctx,
    u32 primitive,
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    const GsRasterVertex& c,
    GsBandFootprint& fp) {
    fp = GsBandFootprint{};
    if (!supported_target(ctx) ||
        !supported_texture(ctx.texture) ||
        ctx.fbw == 0u ||
        ctx.nonzero_colors != nullptr ||
        ctx.nonzero_inputs != nullptr ||
        ctx.nonzero_input_alpha != nullptr ||
        ctx.first_input_rgba != nullptr ||
        ctx.first_alpha_input_rgba != nullptr) {
        return false;
    }
    if (ctx.texture.enabled) {
        // Indexed formats also read a CLUT that this footprint ignores.
        if ((ctx.texture.psm != 0u && ctx.texture.psm != 1u &&
             ctx.texture.psm != 2u && ctx.texture.psm != 10u) ||
            ctx.texture.nonzero_samples != nullptr ||
            ctx.texture.alpha_samples != nullptr ||
            ctx.texture.first_sample_x != nullptr ||
            ctx.texture.first_sample_y != nullptr ||
            ctx.texture.first_sample_rgba != nullptr ||
            ctx.texture.nonzero_shaded != nullptr) {
            return false;
        }
    }

    // Same clipped bounds as draw_triangle() / draw_sprite_rows().
    s32 left;
    s32 right;
    s32 top;
    s32 bottom;
    if (primitive == 6u) {
        left = std::max(ceil_div16(std::min(a.x, b.x)), ctx.scax0);
        right = std::min(ceil_div16(std::max(a.x, b.x)), ctx.scax1 + 1);
        top = std::max(ceil_div16(std::min(a.y, b.y)), ctx.scay0);
        bottom = std::min(ceil_div16(std::max(a.y, b.y)), ctx.scay1 + 1);
    } else {
        left = std::max(
            floor_div16(std::min({a.x, b.x, c.x})), ctx.scax0);
        right = std::min(
            ceil_div16(std::max({a.x, b.x, c.x})), ctx.scax1 + 1);
        top = std::max(
            floor_div16(std::min({a.y, b.y, c.y})), ctx.scay0);
        bottom = std::min(
            ceil_div16(std::max({a.y, b.y, c.y})), ctx.scay1 + 1);
    }
    if (left >= right || top >= bottom) {
        return true; // Draws nothing; no hazards.
    }
    if (left < 0 || top < 0) return false;
    fp.top = top;
    fp.bottom = bottom;
    fp.area = static_cast<u64>(right - left) *
              static_cast<u64>(bottom - top);

    auto page_height = [](u32 psm) -> u32 {
        if (psm == 0u || psm == 1u || psm == 48u || psm == 49u) return 32u;
        if (psm == 2u || psm == 10u || psm == 50u || psm == 58u) return 64u;
        return 0u;
    };
    auto screen_range = [&](u32 bp, u32 psm, u64& begin, u64& finish) {
        const u32 ph = page_height(psm);
        if (ph == 0u) return false;
        const u32 min_page_x = static_cast<u32>(left) >> 6u;
        const u32 max_page_x = static_cast<u32>(right - 1) >> 6u;
        const u32 min_page_y = static_cast<u32>(top) / ph;
        const u32 max_page_y = static_cast<u32>(bottom - 1) / ph;
        if (max_page_x >= ctx.fbw) return false;
        const u64 base = static_cast<u64>(bp) * 256u;
        begin = base +
            (static_cast<u64>(min_page_y) * ctx.fbw + min_page_x) * 8192u;
        finish = base +
            (static_cast<u64>(max_page_y) * ctx.fbw + max_page_x + 1u) *
                8192u;
        return finish <= GsVram::kSize;
    };

    if (!screen_range(ctx.fbp, ctx.psm, fp.frame_begin, fp.frame_end)) {
        return false;
    }
    fp.has_frame = true;
    fp.frame_key = (static_cast<u64>(ctx.psm) << 48u) |
                   (static_cast<u64>(ctx.fbp) << 16u) | ctx.fbw;
    if (ctx.zte) {
        if (!screen_range(ctx.zbp, ctx.zpsm, fp.depth_begin, fp.depth_end)) {
            return false;
        }
        fp.has_depth = true;
        fp.depth_key = (1ull << 62u) |
                       (static_cast<u64>(ctx.zpsm) << 48u) |
                       (static_cast<u64>(ctx.zbp) << 16u) | ctx.fbw;
        if (fp.frame_begin < fp.depth_end && fp.depth_begin < fp.frame_end) {
            return false;
        }
    }

    if (ctx.texture.enabled) {
        const u32 ph = page_height(ctx.texture.psm);
        if (ph == 0u || ctx.texture.bw == 0u) return false;
        const u64 base = static_cast<u64>(ctx.texture.bp) * 256u;
        const u32 pages_x = (ctx.texture.width + 63u) >> 6u;
        const u32 pages_y = (ctx.texture.height + ph - 1u) / ph;
        fp.has_texture = true;
        fp.texture_begin = base;
        if (pages_x == 0u || pages_y == 0u) {
            fp.texture_end = GsVram::kSize;
        } else {
            // Linear page index bound; also valid when pages_x > bw.
            fp.texture_end = std::min<u64>(
                GsVram::kSize,
                base + (static_cast<u64>(pages_y - 1u) * ctx.texture.bw +
                        pages_x) * 8192u);
        }
        // A primitive that samples what it writes depends on pixel order.
        if ((fp.texture_begin < fp.frame_end &&
             fp.frame_begin < fp.texture_end) ||
            (fp.has_depth &&
             fp.texture_begin < fp.depth_end &&
             fp.depth_begin < fp.texture_end)) {
            return false;
        }
    }
    return true;
}

u64 GsRasterizer::draw_triangle(
    GsVram& vram,
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    const GsRasterVertex& c) {
    return draw_triangle_rows(
        vram, ctx, a, b, c, ctx.scay0, ctx.scay1 + 1);
}

u64 GsRasterizer::draw_triangle_rows(
    GsVram& vram,
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    const GsRasterVertex& c,
    s32 row_begin,
    s32 row_end) {
    if (!supported_target(ctx) || !supported_texture(ctx.texture)) return 0;

    const s64 area = edge(a, b, c.x, c.y);
    if (area == 0) return 0;

    const s32 min_x_fp = std::min({a.x, b.x, c.x});
    const s32 max_x_fp = std::max({a.x, b.x, c.x});
    const s32 min_y_fp = std::min({a.y, b.y, c.y});
    const s32 max_y_fp = std::max({a.y, b.y, c.y});

    s32 left = std::max(floor_div16(min_x_fp), ctx.scax0);
    s32 right = std::min(ceil_div16(max_x_fp), ctx.scax1 + 1);
    // Edge values are evaluated exactly at the first row of this band, so a
    // band produces the same pixels as the corresponding full-draw rows.
    s32 top = std::max({floor_div16(min_y_fp), ctx.scay0, row_begin});
    s32 bottom = std::min({ceil_div16(max_y_fp), ctx.scay1 + 1, row_end});
    const bool constant_q = a.q == b.q && b.q == c.q;
    const bool constant_rgba = a.rgba == b.rgba && b.rgba == c.rgba;
    const bool constant_z = a.z == b.z && b.z == c.z;
    const bool positive_area = area > 0;
    const double inv_area =
        1.0 / static_cast<double>(area);
    const bool scaled_constant_q =
        ctx.texture.enabled &&
        !ctx.texture.fst &&
        constant_q &&
        std::isfinite(a.q) &&
        std::fabs(a.q) >= 1.0e-20f;
    const double constant_u_scale =
        scaled_constant_q
            ? (static_cast<double>(ctx.texture.width) *
               16.0 / static_cast<double>(a.q))
            : 0.0;
    const double constant_v_scale =
        scaled_constant_q
            ? (static_cast<double>(ctx.texture.height) *
               16.0 / static_cast<double>(a.q))
            : 0.0;

    // Edge functions are affine in screen space.  Evaluate them once at the
    // first pixel, then advance by their exact 16.4 fixed-point deltas
    // instead of recomputing three 64-bit cross products per pixel.
    // Like the GS (and PCSX2's software renderer) a pixel is sampled at its
    // integer position, with no half-pixel offset: vertex coordinates are
    // already relative to pixel positions, and sprites follow the same rule.
    const s32 start_px = left * 16;
    const s32 start_py = top * 16;
    s64 row_w0 = edge(b, c, start_px, start_py);
    s64 row_w1 = edge(c, a, start_px, start_py);
    s64 row_w2 = edge(a, b, start_px, start_py);

    const s64 w0_dx = 16ll * static_cast<s64>(c.y - b.y);
    const s64 w1_dx = 16ll * static_cast<s64>(a.y - c.y);
    const s64 w2_dx = 16ll * static_cast<s64>(b.y - a.y);
    const s64 w0_dy = -16ll * static_cast<s64>(c.x - b.x);
    const s64 w1_dy = -16ll * static_cast<s64>(a.x - c.x);
    const s64 w2_dy = -16ll * static_cast<s64>(b.x - a.x);

    // Samples can land exactly on an edge, so shared edges need a tie-break:
    // an edge owns its boundary pixels only if it is a left edge (inward
    // normal points +x) or a top edge (horizontal, inward normal points +y).
    const s64 inward = positive_area ? 1 : -1;
    const auto owns_edge = [&](s64 gx, s64 gy) {
        gx *= inward;
        gy *= inward;
        return gx > 0 || (gx == 0 && gy > 0);
    };
    const s64 tie0 = owns_edge(w0_dx, w0_dy) ? 0 : 1;
    const s64 tie1 = owns_edge(w1_dx, w1_dy) ? 0 : 1;
    const s64 tie2 = owns_edge(w2_dx, w2_dy) ? 0 : 1;

    // Constant-Q ST interpolation is affine. Keep exact integer edge
    // stepping, but advance the double-precision S/T numerators across a row
    // instead of rebuilding six weighted products for every covered pixel.
    // Row starts are recomputed from integer edge values, bounding floating
    // accumulation to one scanline.
    const double s_num_dx = scaled_constant_q
        ? static_cast<double>(w0_dx) * a.s +
          static_cast<double>(w1_dx) * b.s +
          static_cast<double>(w2_dx) * c.s
        : 0.0;
    const double t_num_dx = scaled_constant_q
        ? static_cast<double>(w0_dx) * a.t +
          static_cast<double>(w1_dx) * b.t +
          static_cast<double>(w2_dx) * c.t
        : 0.0;

    const bool hot_psm16_pixels =
        hot_osdsys_psm16_pixel_state(ctx);
    const bool simple_pixels =
        !hot_psm16_pixels && simple_frame_write(ctx);

    const bool zwrite_pixels =
        !hot_psm16_pixels && !simple_pixels && simple_zwrite_state(ctx);
    // Hoisted so VRAM stores in the pixel loop cannot force reloads.
    const bool texture_enabled = ctx.texture.enabled;
    const bool texture_fst = ctx.texture.fst;
    const bool fog_enabled = ctx.fog_enabled;
    // Z only matters for a real depth comparison or a depth write.
    const bool need_z =
        ctx.zte && ((ctx.ztst & 3u) >= 2u || !ctx.zmask);
    const bool gouraud_step = ctx.gouraud && !constant_rgba;
    GouraudStepper gouraud(
        area, a.rgba, b.rgba, c.rgba, w0_dx, w1_dx, w2_dx);
    // Edge i covers a pixel when its oriented value is at least tie_i.
    const s64 edge_ties[3] = {tie0, tie1, tie2};
    u64 pixels = 0;
    for (s32 y = top; y < bottom; ++y) {
        s64 w0 = row_w0;
        s64 w1 = row_w1;
        s64 w2 = row_w2;
        double s_num = scaled_constant_q
            ? static_cast<double>(row_w0) * a.s +
              static_cast<double>(row_w1) * b.s +
              static_cast<double>(row_w2) * c.s
            : 0.0;
        double t_num = scaled_constant_q
            ? static_cast<double>(row_w0) * a.t +
              static_cast<double>(row_w1) * b.t +
              static_cast<double>(row_w2) * c.t
            : 0.0;

        // A triangle row covers one contiguous run of pixels. Jump straight
        // to its first pixel using the exact integer edge functions, then
        // stop at the first uncovered pixel after it.
        s32 x = left;
        {
            const s64 row_edges[3] = {w0, w1, w2};
            const s64 edge_steps[3] = {w0_dx, w1_dx, w2_dx};
            s64 skip = 0;
            bool row_empty = false;
            for (u32 i = 0; i < 3u; ++i) {
                const s64 value =
                    (positive_area ? row_edges[i] : -row_edges[i]) -
                    edge_ties[i];
                const s64 step =
                    positive_area ? edge_steps[i] : -edge_steps[i];
                if (value >= 0) continue;
                if (step <= 0) {
                    row_empty = true;
                    break;
                }
                skip = std::max(skip, (-value + step - 1) / step);
            }
            if (row_empty || skip >= static_cast<s64>(right - left)) {
                x = right;
            } else if (skip != 0) {
                x += static_cast<s32>(skip);
                w0 += skip * w0_dx;
                w1 += skip * w1_dx;
                w2 += skip * w2_dx;
                // The S/T numerators are stepped by repeated addition;
                // advance them the same way so rounding is unchanged.
                if (scaled_constant_q) {
                    for (s64 i = 0; i < skip; ++i) {
                        s_num += s_num_dx;
                        t_num += t_num_dx;
                    }
                }
            }
        }
        if (gouraud_step && x < right) gouraud.begin_row(w0, w1, w2);
        for (; x < right; ++x) {
            const bool inside =
                positive_area
                    ? (w0 >= tie0 && w1 >= tie1 && w2 >= tie2)
                    : (w0 <= -tie0 && w1 <= -tie1 && w2 <= -tie2);
            if (!inside) break;
            {
                s32 u = 0;
                s32 v = 0;
                if (texture_enabled) {
                    if (texture_fst) {
                        u = static_cast<s32>(
                            (w0 * a.u + w1 * b.u + w2 * c.u) / area);
                        v = static_cast<s32>(
                            (w0 * a.v + w1 * b.v + w2 * c.v) / area);
                    } else {
                        const float s = scaled_constant_q
                            ? static_cast<float>(s_num * inv_area)
                            : static_cast<float>(
                                (static_cast<double>(w0) * a.s +
                                 static_cast<double>(w1) * b.s +
                                 static_cast<double>(w2) * c.s) * inv_area);
                        const float t = scaled_constant_q
                            ? static_cast<float>(t_num * inv_area)
                            : static_cast<float>(
                                (static_cast<double>(w0) * a.t +
                                 static_cast<double>(w1) * b.t +
                                 static_cast<double>(w2) * c.t) * inv_area);
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
                                        (static_cast<double>(w0) * a.q +
                                         static_cast<double>(w1) * b.q +
                                         static_cast<double>(w2) * c.q) *
                                        inv_area);
                            u = stq_to_fixed(
                                s, q, ctx.texture.width);
                            v = stq_to_fixed(
                                t, q, ctx.texture.height);
                        }
                    }
                }
                const u32 vertex_rgba =
                    gouraud_step ? gouraud.rgba() : c.rgba;
                u32 rgba = shade_pixel(
                    vram, ctx.texture, u, v, vertex_rgba);
                if (fog_enabled) {
                    const u32 fog = interpolate_scalar(
                        w0, w1, w2, area, a.fog, b.fog, c.fog);
                    rgba = apply_fog(rgba, ctx.fog_color, fog);
                }
                const u32 z = !need_z ? 0u : constant_z ? a.z
                    : interpolate_z(w0, w1, w2, area, a.z, b.z, c.z);
                const bool wrote = hot_psm16_pixels
                    ? draw_hot_osdsys_psm16_pixel(
                        vram, ctx, x, y, z, rgba)
                    : simple_pixels
                        ? draw_simple_frame_pixel(vram, ctx, x, y, rgba)
                        : zwrite_pixels
                            ? draw_simple_zwrite_pixel(
                                vram, ctx, x, y, z, rgba)
                            : draw_pixel(vram, ctx, x, y, z, rgba);
                if (wrote) ++pixels;
            }

            w0 += w0_dx;
            w1 += w1_dx;
            w2 += w2_dx;
            if (gouraud_step) gouraud.step();
            if (scaled_constant_q) {
                s_num += s_num_dx;
                t_num += t_num_dx;
            }
        }
        row_w0 += w0_dy;
        row_w1 += w1_dy;
        row_w2 += w2_dy;
    }
    return pixels;
}

} // namespace ps2
