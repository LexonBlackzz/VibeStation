#include "gpu.h"
#include "system.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace {
    const std::array<u32, 32768>& rgb555_to_rgba_lut() {
        static const std::array<u32, 32768> lut = [] {
            std::array<u32, 32768> values{};
            for (u32 pixel = 0; pixel < values.size(); ++pixel) {
                const u32 r5 = pixel & 0x1Fu;
                const u32 g5 = (pixel >> 5) & 0x1Fu;
                const u32 b5 = (pixel >> 10) & 0x1Fu;
                values[pixel] =
                    ((r5 << 3) | (r5 >> 2)) |
                    (((g5 << 3) | (g5 >> 2)) << 8) |
                    (((b5 << 3) | (b5 >> 2)) << 16) | 0xFF000000u;
            }
            return values;
        }();
        return lut;
    }
    inline bool fmv_diagnostics_enabled() {
        return g_log_fmv_diagnostics;
    }

    inline bool gpu_command_diagnostics_enabled() {
        return g_log_fmv_diagnostics || g_mdec_debug_upload_probe;
    }

    System::GpuProfileBucket gpu_profile_bucket_for_opcode(u8 op) {
        if ((op >= 0x20u && op <= 0x23u) ||
            (op >= 0x28u && op <= 0x2Bu)) {
            return System::GpuProfileBucket::Flat;
        }
        if ((op >= 0x30u && op <= 0x33u) ||
            (op >= 0x38u && op <= 0x3Bu)) {
            return System::GpuProfileBucket::Gouraud;
        }
        if ((op >= 0x24u && op <= 0x27u) ||
            (op >= 0x2Cu && op <= 0x2Fu)) {
            return System::GpuProfileBucket::Textured;
        }
        if ((op >= 0x34u && op <= 0x37u) ||
            (op >= 0x3Cu && op <= 0x3Fu)) {
            return System::GpuProfileBucket::GouraudTextured;
        }
        if (op == 0x02u || (op >= 0x60u && op <= 0x7Fu)) {
            return System::GpuProfileBucket::Rect;
        }
        if (op >= 0x40u && op <= 0x5Fu) {
            return System::GpuProfileBucket::Line;
        }
        if (op >= 0x80u && op <= 0xDFu) {
            return System::GpuProfileBucket::Transfer;
        }
        return System::GpuProfileBucket::Other;
    }

    int clamp_display_dimension(int value, int fallback, int max_value) {
        const int candidate = (value > 0) ? value : fallback;
        return std::max(1, std::min(candidate, max_value));
    }

    void suppress_isolated_bottom_noise_row(std::vector<u32>* rgba,
        int width, int height) {
        if (rgba == nullptr || width <= 0 || height < 3) {
            return;
        }

        auto count_non_black = [&](int row) {
            const size_t row_base =
                static_cast<size_t>(row) * static_cast<size_t>(width);
            int count = 0;
            for (int x = 0; x < width; ++x) {
                const u32 pixel = (*rgba)[row_base + static_cast<size_t>(x)];
                if ((pixel & 0x00FFFFFFu) != 0u) {
                    ++count;
                }
            }
            return count;
            };

        const int row_last = height - 1;
        const int row_prev = height - 2;
        const int row_prev2 = height - 3;
        const int last_non_black = count_non_black(row_last);
        const int prev_non_black = count_non_black(row_prev);
        const int prev2_non_black = count_non_black(row_prev2);

        // Some BIOS/menu scenes leave a sparse junk row at the very bottom while
        // the rows above are fully black. Suppress only that isolated case.
        if (last_non_black <= 0 || last_non_black > (width / 3) ||
            prev_non_black > std::max(1, width / 64) ||
            prev2_non_black > std::max(1, width / 64)) {
            return;
        }

        const size_t row_base =
            static_cast<size_t>(row_last) * static_cast<size_t>(width);
        std::fill_n(rgba->begin() + static_cast<std::ptrdiff_t>(row_base),
            width, 0xFF000000u);
    }

    inline s16 sign_extend_11(u32 value) {
        return static_cast<s16>(static_cast<s16>((value & 0x7FFu) << 5) >> 5);
    }

    int horizontal_divisor(u8 hres_mode) {
        static const int kDivisors[] = { 10, 8, 5, 4, 7 };
        if (hres_mode < 5) {
            return kDivisors[hres_mode];
        }
        return 10;
    }

    constexpr int kNtscTicksPerLine = 3413;
    constexpr int kNtscTotalLines = 263;
    constexpr int kPalTicksPerLine = 3406;
    constexpr int kPalTotalLines = 314;
    constexpr int kNtscHorizontalActiveStart = 488;
    constexpr int kNtscHorizontalActiveEnd = 3288;
    constexpr int kNtscVerticalActiveStart = 16;
    constexpr int kNtscVerticalActiveEnd = 256;
    constexpr int kPalHorizontalActiveStart = 488;
    constexpr int kPalHorizontalActiveEnd = 3300;
    constexpr int kPalVerticalActiveStart = 20;
    constexpr int kPalVerticalActiveEnd = 308;

    struct CrtcRect {
        int mode_width = 0;
        int mode_height = 0;
        int width = 0;
        int height = 0;
        int vram_left = 0;
        int vram_top = 0;
        int vram_width = 0;
        int vram_height = 0;
        int skip_x = 0;
        int divisor = 1;
    };

    CrtcRect calculate_crtc_rect(const DisplayMode& display) {
        CrtcRect rect{};
        rect.mode_width = display.width();
        rect.mode_height = display.height();
        rect.divisor = std::max(1, horizontal_divisor(display.hres));

        const int horizontal_total =
            display.is_pal ? kPalTicksPerLine : kNtscTicksPerLine;
        const int vertical_total =
            display.is_pal ? kPalTotalLines : kNtscTotalLines;
        const int active_h_start =
            display.is_pal ? kPalHorizontalActiveStart : kNtscHorizontalActiveStart;
        const int active_h_end =
            display.is_pal ? kPalHorizontalActiveEnd : kNtscHorizontalActiveEnd;
        const int active_v_start =
            display.is_pal ? kPalVerticalActiveStart : kNtscVerticalActiveStart;
        const int active_v_end =
            display.is_pal ? kPalVerticalActiveEnd : kNtscVerticalActiveEnd;

        const int horizontal_start =
            (std::min<int>(display.x1, horizontal_total) / rect.divisor) *
            rect.divisor;
        const int horizontal_end =
            (std::min<int>(display.x2, horizontal_total) / rect.divisor) *
            rect.divisor;
        const int visible_h_start =
            std::clamp(horizontal_start, active_h_start, active_h_end);
        const int visible_h_end =
            std::clamp(horizontal_end, visible_h_start, active_h_end);
        const int horizontal_ticks =
            std::max(0, horizontal_end - horizontal_start);
        const int horizontal_pixels = horizontal_ticks / rect.divisor;
        const int visible_pixels =
            std::max(0, (visible_h_end - visible_h_start) / rect.divisor);

        rect.width = (visible_pixels > 0)
            ? clamp_display_dimension(visible_pixels, rect.mode_width, psx::VRAM_WIDTH)
            : clamp_display_dimension(rect.mode_width, 320, psx::VRAM_WIDTH);
        if (horizontal_pixels == 1) {
            rect.vram_width = 4;
        } else if (horizontal_pixels > 1) {
            rect.vram_width = ((horizontal_pixels + 2) & ~3);
        } else {
            rect.vram_width = rect.width;
        }

        const int vertical_start = std::min<int>(display.y1, vertical_total);
        const int vertical_end = std::min<int>(display.y2, vertical_total);
        const int visible_v_start =
            std::clamp(vertical_start, active_v_start, active_v_end);
        const int visible_v_end =
            std::clamp(vertical_end, visible_v_start, active_v_end);
        const int vertical_lines =
            std::max(0, vertical_end - vertical_start);
        const int visible_lines =
            std::max(0, visible_v_end - visible_v_start);
        const int height_shift =
            (display.interlaced && display.vres != 0) ? 1 : 0;

        rect.height = (visible_lines > 0)
            ? clamp_display_dimension(visible_lines << height_shift,
                                      rect.mode_height, psx::VRAM_HEIGHT)
            : clamp_display_dimension(rect.mode_height, 240, psx::VRAM_HEIGHT);

        rect.vram_left = static_cast<int>(display.x_start);
        rect.vram_top = static_cast<int>(display.y_start);
        rect.skip_x = 0;
        if (horizontal_start < visible_h_start) {
            rect.skip_x = (visible_h_start - horizontal_start) / rect.divisor;
            rect.vram_left =
                (rect.vram_left + rect.skip_x) % static_cast<int>(psx::VRAM_WIDTH);
            rect.vram_width -= std::min(rect.vram_width, rect.skip_x);
        }
        rect.vram_width = std::min(rect.vram_width, rect.width);

        if (vertical_start < visible_v_start) {
            const int vertical_skip = (visible_v_start - vertical_start) << height_shift;
            rect.vram_top =
                (rect.vram_top + vertical_skip) % static_cast<int>(psx::VRAM_HEIGHT);
        }
        rect.vram_height = std::min(rect.height, std::max(0, vertical_lines << height_shift));
        if (rect.vram_height <= 0) {
            rect.vram_height = rect.height;
        }

        rect.vram_left = std::clamp(rect.vram_left, 0, static_cast<int>(psx::VRAM_WIDTH) - 1);
        rect.vram_top = std::clamp(rect.vram_top, 0, static_cast<int>(psx::VRAM_HEIGHT) - 1);
        rect.vram_width = clamp_display_dimension(rect.vram_width, rect.width, psx::VRAM_WIDTH);
        rect.vram_height = clamp_display_dimension(rect.vram_height, rect.height, psx::VRAM_HEIGHT);
        return rect;
    }

    constexpr int kDitherTable[4][4] = {
        {-4, 0, -3, 1},
        {2, -2, 3, -1},
        {-3, 1, -4, 0},
        {3, -1, 2, -2},
    };

    inline int clamp_u8_i(int value) {
        return std::max(0, std::min(value, 255));
    }

    inline float edge_float(float ax, float ay, float bx, float by, float cx, float cy) {
        return (cx - ax) * (by - ay) - (cy - ay) * (bx - ax);
    }

    inline bool is_top_left_edge(const Vertex& a, const Vertex& b) {
        const int dy = static_cast<int>(b.y) - static_cast<int>(a.y);
        const int dx = static_cast<int>(b.x) - static_cast<int>(a.x);
        return (dy < 0) || (dy == 0 && dx > 0);
    }

    inline bool edge_inside_ccw(s32 w, bool top_left) {
        return (w > 0) || (w == 0 && top_left);
    }

    inline s64 floor_div_positive(s64 numerator, s64 denominator) {
        s64 q = numerator / denominator;
        const s64 r = numerator % denominator;
        if (r != 0 && numerator < 0) {
            --q;
        }
        return q;
    }

    inline s64 ceil_div_positive(s64 numerator, s64 denominator) {
        s64 q = numerator / denominator;
        const s64 r = numerator % denominator;
        if (r != 0 && numerator > 0) {
            ++q;
        }
        return q;
    }

    inline bool constrain_triangle_span(s32 w_at_min_x, s32 step_x,
                                        bool top_left, int& lo, int& hi) {
        const s64 threshold = top_left ? 0 : 1;
        const s64 w = static_cast<s64>(w_at_min_x);
        const s64 step = static_cast<s64>(step_x);
        if (step > 0) {
            lo = std::max(
                lo, static_cast<int>(
                        ceil_div_positive(threshold - w, step)));
        }
        else if (step < 0) {
            hi = std::min(
                hi, static_cast<int>(
                        floor_div_positive(w - threshold, -step)));
        }
        else if (w < threshold) {
            return false;
        }
        return lo <= hi;
    }

    inline bool triangle_scanline_span(
        s16 min_x, s16 max_x,
        s32 w0_row, s32 w1_row, s32 w2_row,
        s32 step_w0_x, s32 step_w1_x, s32 step_w2_x,
        bool edge0_top_left, bool edge1_top_left, bool edge2_top_left,
        s16& span_min_x, s16& span_max_x) {
        int lo = 0;
        int hi = static_cast<int>(max_x) - static_cast<int>(min_x);
        if (hi < 0 ||
            !constrain_triangle_span(
                w0_row, step_w0_x, edge0_top_left, lo, hi) ||
            !constrain_triangle_span(
                w1_row, step_w1_x, edge1_top_left, lo, hi) ||
            !constrain_triangle_span(
                w2_row, step_w2_x, edge2_top_left, lo, hi)) {
            return false;
        }
        span_min_x = static_cast<s16>(static_cast<int>(min_x) + lo);
        span_max_x = static_cast<s16>(static_cast<int>(min_x) + hi);
        return true;
    }

    inline int dither_bias(s16 x, s16 y) {
        return kDitherTable[static_cast<u16>(y) & 0x3u]
            [static_cast<u16>(x) & 0x3u];
    }

    Color average_color(Color a, Color b, Color c) {
        return Color(
            static_cast<u8>((static_cast<u32>(a.r) + b.r + c.r) / 3u),
            static_cast<u8>((static_cast<u32>(a.g) + b.g + c.g) / 3u),
            static_cast<u8>((static_cast<u32>(a.b) + b.b + c.b) / 3u));
    }

    u16 modulate_texel_15bit(u16 texel, u8 mr, u8 mg, u8 mb) {
        const int tr = texel & 0x1F;
        const int tg = (texel >> 5) & 0x1F;
        const int tb = (texel >> 10) & 0x1F;

        // PS1 textured color modulation uses 0x80 as neutral gain.
        const int rr = std::min(31, (tr * static_cast<int>(mr)) >> 7);
        const int rg = std::min(31, (tg * static_cast<int>(mg)) >> 7);
        const int rb = std::min(31, (tb * static_cast<int>(mb)) >> 7);

        return static_cast<u16>((rr & 0x1F) | ((rg & 0x1F) << 5) |
            ((rb & 0x1F) << 10) | (texel & 0x8000u));
    }

    u16 modulate_texel_dithered_15bit(u16 texel, u8 mr, u8 mg, u8 mb, s16 x, s16 y) {
        const int tr = texel & 0x1F;
        const int tg = (texel >> 5) & 0x1F;
        const int tb = (texel >> 10) & 0x1F;

        const int rr5 = std::min(31, (tr * static_cast<int>(mr)) >> 7);
        const int rg5 = std::min(31, (tg * static_cast<int>(mg)) >> 7);
        const int rb5 = std::min(31, (tb * static_cast<int>(mb)) >> 7);

        const int d = dither_bias(x, y);
        const int rr = clamp_u8_i((rr5 << 3) + d) >> 3;
        const int rg = clamp_u8_i((rg5 << 3) + d) >> 3;
        const int rb = clamp_u8_i((rb5 << 3) + d) >> 3;

        return static_cast<u16>((rr & 0x1F) | ((rg & 0x1F) << 5) |
            ((rb & 0x1F) << 10) | (texel & 0x8000u));
    }

    u16 pack_rgb15_dithered(u8 r, u8 g, u8 b, u16 preserve_bits, s16 x, s16 y,
        bool dither) {
        if (dither) {
            const int d = dither_bias(x, y);
            r = static_cast<u8>(clamp_u8_i(static_cast<int>(r) + d));
            g = static_cast<u8>(clamp_u8_i(static_cast<int>(g) + d));
            b = static_cast<u8>(clamp_u8_i(static_cast<int>(b) + d));
        }
        return static_cast<u16>(((r >> 3) & 0x1F) | (((g >> 3) & 0x1F) << 5) |
            (((b >> 3) & 0x1F) << 10) |
            (preserve_bits & 0x8000u));
    }
    // The GPU skips any triangle whose vertices are 1024 or more pixels apart
    // horizontally or 512 or more vertically, judged on the unclipped (but
    // 11-bit wrapped) coordinates before the drawing area is applied.
    bool exceeds_primitive_limits(const Vertex& a, const Vertex& b,
                                  const Vertex& c) {
        const int min_x = std::min({ a.x, b.x, c.x });
        const int max_x = std::max({ a.x, b.x, c.x });
        const int min_y = std::min({ a.y, b.y, c.y });
        const int max_y = std::max({ a.y, b.y, c.y });
        return max_x - min_x >= 1024 || max_y - min_y >= 512;
    }
} // namespace

// ── Command Length Table ───────────────────────────────────────────
// Returns the number of 32-bit words a GP0 command consumes.

u32 Gpu::gp0_command_length(u8 opcode) {
    switch (opcode) {
    case 0x00:
        return 1; // NOP
    case 0x01:
        return 1; // Clear cache
    case 0x02:
        return 3; // Fill rectangle
    case 0x20:
    case 0x21:
    case 0x22:
    case 0x23:
        return 4; // Mono triangle
    case 0x24:
    case 0x25:
    case 0x26:
    case 0x27:
        return 7; // Textured triangle
    case 0x28:
    case 0x29:
    case 0x2A:
    case 0x2B:
        return 5; // Mono quad
    case 0x2C:
    case 0x2D:
    case 0x2E:
    case 0x2F:
        return 9; // Textured quad
    case 0x30:
    case 0x31:
    case 0x32:
    case 0x33:
        return 6; // Shaded triangle
    case 0x34:
    case 0x35:
    case 0x36:
    case 0x37:
        return 9; // Shaded textured triangle
    case 0x38:
    case 0x39:
    case 0x3A:
    case 0x3B:
        return 8; // Shaded quad
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F:
        return 12; // Shaded textured quad
    case 0x40:
    case 0x41:
    case 0x42:
    case 0x43:
    case 0x44:
    case 0x45:
    case 0x46:
    case 0x47:
        return 3; // Mono line
    case 0x48:
    case 0x49:
    case 0x4A:
    case 0x4B:
    case 0x4C:
    case 0x4D:
    case 0x4E:
    case 0x4F:
        return 3; // Mono polyline (variable tail)
    case 0x50:
    case 0x51:
    case 0x52:
    case 0x53:
    case 0x54:
    case 0x55:
    case 0x56:
    case 0x57:
        return 4; // Shaded line
    case 0x58:
    case 0x59:
    case 0x5A:
    case 0x5B:
    case 0x5C:
    case 0x5D:
    case 0x5E:
    case 0x5F:
        return 4; // Shaded polyline (variable tail)
    case 0x60:
    case 0x61:
    case 0x62:
    case 0x63:
        return 3; // Mono rectangle (variable)
    case 0x64:
    case 0x65:
    case 0x66:
    case 0x67:
        return 4; // Textured rectangle (variable)
    case 0x68:
    case 0x69:
    case 0x6A:
    case 0x6B:
        return 2; // Mono rect 1x1
    case 0x6C:
    case 0x6D:
    case 0x6E:
    case 0x6F:
        return 3; // Textured rect 1x1
    case 0x70:
    case 0x71:
    case 0x72:
    case 0x73:
        return 2; // Mono rect 8x8
    case 0x74:
    case 0x75:
    case 0x76:
    case 0x77:
        return 3; // Textured rect 8x8
    case 0x78:
    case 0x79:
    case 0x7A:
    case 0x7B:
        return 2; // Mono rect 16x16
    case 0x7C:
    case 0x7D:
    case 0x7E:
    case 0x7F:
        return 3; // Textured rect 16x16
    case 0x80:
    case 0x81:
    case 0x82:
    case 0x83:
    case 0x84:
    case 0x85:
    case 0x86:
    case 0x87:
    case 0x88:
    case 0x89:
    case 0x8A:
    case 0x8B:
    case 0x8C:
    case 0x8D:
    case 0x8E:
    case 0x8F:
    case 0x90:
    case 0x91:
    case 0x92:
    case 0x93:
    case 0x94:
    case 0x95:
    case 0x96:
    case 0x97:
    case 0x98:
    case 0x99:
    case 0x9A:
    case 0x9B:
    case 0x9C:
    case 0x9D:
    case 0x9E:
    case 0x9F:
        return 4; // VRAM-to-VRAM copy
    case 0xA0:
    case 0xA1:
    case 0xA2:
    case 0xA3:
    case 0xA4:
    case 0xA5:
    case 0xA6:
    case 0xA7:
    case 0xA8:
    case 0xA9:
    case 0xAA:
    case 0xAB:
    case 0xAC:
    case 0xAD:
    case 0xAE:
    case 0xAF:
    case 0xB0:
    case 0xB1:
    case 0xB2:
    case 0xB3:
    case 0xB4:
    case 0xB5:
    case 0xB6:
    case 0xB7:
    case 0xB8:
    case 0xB9:
    case 0xBA:
    case 0xBB:
    case 0xBC:
    case 0xBD:
    case 0xBE:
    case 0xBF:
        return 3; // CPU-to-VRAM (header, data follows)
    case 0xC0:
    case 0xC1:
    case 0xC2:
    case 0xC3:
    case 0xC4:
    case 0xC5:
    case 0xC6:
    case 0xC7:
    case 0xC8:
    case 0xC9:
    case 0xCA:
    case 0xCB:
    case 0xCC:
    case 0xCD:
    case 0xCE:
    case 0xCF:
    case 0xD0:
    case 0xD1:
    case 0xD2:
    case 0xD3:
    case 0xD4:
    case 0xD5:
    case 0xD6:
    case 0xD7:
    case 0xD8:
    case 0xD9:
    case 0xDA:
    case 0xDB:
    case 0xDC:
    case 0xDD:
    case 0xDE:
    case 0xDF:
        return 3; // VRAM-to-CPU
    case 0xE1:
        return 1; // Draw mode
    case 0xE2:
        return 1; // Texture window
    case 0xE3:
        return 1; // Draw area top-left
    case 0xE4:
        return 1; // Draw area bottom-right
    case 0xE5:
        return 1; // Draw offset
    case 0xE6:
        return 1; // Mask bit setting
    default:
        return 1;
    }
}

// ── Reset ──────────────────────────────────────────────────────────

void Gpu::reset() {
    vram_.fill(0);
    hw_full_sync_pending_ = true;
    if (gp0_buffer_.capacity() < 12) {
        gp0_buffer_.reserve(12);
    }
    gp0_fifo_.clear();
    gp0_buffer_.clear();
    gp0_buffer_src_.clear();
    gp0_mode_ = Gp0Mode::Command;
    gp0_words_remaining_ = 0;
    display_ = {};
    draw_x_min_ = 0;
    draw_y_min_ = 0;
    draw_x_max_ = static_cast<s16>(psx::VRAM_WIDTH - 1);
    draw_y_max_ = static_cast<s16>(psx::VRAM_HEIGHT - 1);
    draw_x_offset_ = 0;
    draw_y_offset_ = 0;
    texpage_ = 0;
    clut_ = 0;
    tex_window_mask_x_ = 0;
    tex_window_mask_y_ = 0;
    tex_window_off_x_ = 0;
    tex_window_off_y_ = 0;
    dma_direction_ = 0;
    dither_enabled_ = false;
    draw_to_display_ = false;
    texture_disable_ = false;
    semi_transparency_mode_ = false;
    semi_transparency_ = 0;
    tex_rect_x_flip_ = false;
    tex_rect_y_flip_ = false;
    irq1_pending_ = false;
    force_set_mask_bit_ = false;
    check_mask_before_draw_ = false;
    interlace_field_ = false;
    gpuread_latch_ = 0;
    frame_complete_ = false;
    presented_display_valid_ = false;
    presented_display_info_ = {};
    presented_display_rgba_.clear();
    command_debug_ = {};
    polyline_active_ = false;
    polyline_gouraud_ = false;
    polyline_waiting_vertex_ = false;
    polyline_prev_vertex_ = {};
    polyline_flat_color_ = {};
    polyline_prev_color_ = {};
    polyline_pending_color_word_ = 0;
}

// ── GP0 (Rendering Commands) ──────────────────────────────────────

void Gpu::corrupt_vram_word(u32 index, u16 value) {
    const size_t safe_index = static_cast<size_t>(index) % vram_.size();
    vram_[safe_index] = value;
    hw_full_sync_pending_ = true;
}

void Gpu::corrupt_render_state(u32 selector, u32 value) {
    switch (selector % 10u) {
    case 0:
        draw_x_offset_ = sign_extend_11(value);
        draw_y_offset_ = sign_extend_11(value >> 11);
        break;
    case 1:
        draw_x_min_ = static_cast<s16>(value % psx::VRAM_WIDTH);
        draw_y_min_ = static_cast<s16>((value >> 10) % psx::VRAM_HEIGHT);
        if (draw_x_max_ < draw_x_min_) {
            draw_x_max_ = draw_x_min_;
        }
        if (draw_y_max_ < draw_y_min_) {
            draw_y_max_ = draw_y_min_;
        }
        break;
    case 2:
        draw_x_max_ = static_cast<s16>(value % psx::VRAM_WIDTH);
        draw_y_max_ = static_cast<s16>((value >> 10) % psx::VRAM_HEIGHT);
        if (draw_x_min_ > draw_x_max_) {
            draw_x_min_ = draw_x_max_;
        }
        if (draw_y_min_ > draw_y_max_) {
            draw_y_min_ = draw_y_max_;
        }
        break;
    case 3:
        texpage_ = static_cast<u16>(value & 0x1FFu);
        semi_transparency_ = static_cast<u8>((value >> 9) & 0x3u);
        dither_enabled_ = ((value >> 11) & 0x1u) != 0;
        texture_disable_ = ((value >> 12) & 0x1u) != 0;
        tex_rect_x_flip_ = ((value >> 13) & 0x1u) != 0;
        tex_rect_y_flip_ = ((value >> 14) & 0x1u) != 0;
        break;
    case 4:
        clut_ = static_cast<u16>(value & 0x7FFFu);
        break;
    case 5:
        tex_window_mask_x_ = static_cast<u16>((value & 0x1Fu) * 8u);
        tex_window_mask_y_ = static_cast<u16>(((value >> 5) & 0x1Fu) * 8u);
        tex_window_off_x_ = static_cast<u16>(((value >> 10) & 0x1Fu) * 8u);
        tex_window_off_y_ = static_cast<u16>(((value >> 15) & 0x1Fu) * 8u);
        break;
    case 6:
        display_.x_start = static_cast<u16>(value & 0x3FEu);
        display_.y_start = static_cast<u16>((value >> 10) & 0x1FFu);
        break;
    case 7:
        display_.hres = static_cast<u8>(value % 5u);
        display_.vres = static_cast<u8>((value >> 3) & 0x1u);
        display_.is_pal = ((value >> 4) & 0x1u) != 0;
        display_.is_24bit = ((value >> 5) & 0x1u) != 0;
        display_.interlaced = ((value >> 6) & 0x1u) != 0;
        display_.display_enabled = ((value >> 7) & 0x1u) == 0;
        break;
    case 8:
        draw_to_display_ = ((value >> 0) & 0x1u) != 0;
        force_set_mask_bit_ = ((value >> 1) & 0x1u) != 0;
        check_mask_before_draw_ = ((value >> 2) & 0x1u) != 0;
        irq1_pending_ = ((value >> 3) & 0x1u) != 0;
        break;
    case 9:
        if (!gp0_buffer_.empty()) {
            const size_t index = static_cast<size_t>(value) % gp0_buffer_.size();
            gp0_buffer_[index] ^= (value | 1u);
        }
        else {
            gpuread_latch_ ^= value;
        }
        break;
    default:
        break;
    }
}

void Gpu::set_reaper_pulse(u32 geometry_mutations, u32 texture_mutations,
    u32 seed) {
    // Mutations a VBlank period had no draws for carry over (games below
    // 60 fps draw only every other period); seed 0 (Reaper off) clears them.
    constexpr u32 kMaxPending = 4096;
    reaper_pending_geometry_ = seed == 0 ? 0 :
        std::min(kMaxPending, reaper_pending_geometry_ + geometry_mutations);
    reaper_pending_texture_ = seed == 0 ? 0 :
        std::min(kMaxPending, reaper_pending_texture_ + texture_mutations);
    reaper_state_ = seed ? seed : 0xA341316Cu;
}

u32 Gpu::next_reaper_noise() {
    reaper_state_ ^= (reaper_state_ << 13);
    reaper_state_ ^= (reaper_state_ >> 17);
    reaper_state_ ^= (reaper_state_ << 5);
    return reaper_state_;
}

bool Gpu::is_textured_primitive_opcode(u8 opcode) {
    return (opcode >= 0x24 && opcode <= 0x27) ||
        (opcode >= 0x2C && opcode <= 0x2F) ||
        (opcode >= 0x34 && opcode <= 0x37) ||
        (opcode >= 0x3C && opcode <= 0x3F) ||
        (opcode >= 0x64 && opcode <= 0x67) ||
        (opcode >= 0x74 && opcode <= 0x77) ||
        (opcode >= 0x7C && opcode <= 0x7F);
}

bool Gpu::is_draw_primitive_opcode(u8 opcode) {
    return opcode >= 0x20 && opcode <= 0x7F;
}

u32 Gpu::mutate_vertex_word(u32 word, u32 noise) {
    s32 x = static_cast<s16>(word & 0xFFFFu);
    s32 y = static_cast<s16>((word >> 16) & 0xFFFFu);
    x += static_cast<s32>(static_cast<int>(noise & 0xFFu) - 128);
    y += static_cast<s32>(static_cast<int>((noise >> 8) & 0xFFu) - 128);
    x = std::max<s32>(-1024, std::min<s32>(1023, x));
    y = std::max<s32>(-512, std::min<s32>(511, y));
    return (static_cast<u32>(static_cast<u16>(x)) & 0xFFFFu) |
        ((static_cast<u32>(static_cast<u16>(y)) & 0xFFFFu) << 16);
}

void Gpu::apply_reaper_to_gp0_command() {
    if (gp0_buffer_.empty()) {
        return;
    }

    const u8 opcode = static_cast<u8>(gp0_command_);
    if (is_draw_primitive_opcode(opcode)) {
        ++reaper_draws_this_frame_;
    }
    if (reaper_pending_geometry_ == 0 && reaper_pending_texture_ == 0) {
        return;
    }

    // Spread the frame's mutations over its draws (estimated from the last
    // frame) instead of spending them all on the first few, which are usually
    // backgrounds that later draws cover.
    const auto hit = [&](u32 pending) {
        if (pending == 0) {
            return false;
        }
        // Games below 60 fps draw in every other VBlank period or so; use the
        // busier of the last two.
        const u32 expected =
            std::max(reaper_draws_last_frame_, reaper_draws_prev_frame_);
        const u32 remaining = expected > reaper_draws_this_frame_
            ? expected - reaper_draws_this_frame_ + 1u
            : 1u;
        return pending >= remaining || (next_reaper_noise() % remaining) < pending;
    };

    // Word index of each vertex (and its texcoord word, 0 if none).
    size_t vertex_slots[4] = {};
    size_t uv_slots[4] = {};
    size_t vertex_count = 0;
    if (opcode >= 0x20 && opcode <= 0x3F) {
        const bool gouraud = (opcode & 0x10u) != 0;
        const bool textured = (opcode & 0x04u) != 0;
        const size_t stride = 1u + (textured ? 1u : 0u) + (gouraud ? 1u : 0u);
        vertex_count = (opcode & 0x08u) ? 4u : 3u;
        for (size_t i = 0; i < vertex_count; ++i) {
            vertex_slots[i] = 1u + i * stride;
            uv_slots[i] = textured ? vertex_slots[i] + 1u : 0u;
        }
    } else if (opcode >= 0x40 && opcode <= 0x5F) {
        const bool gouraud = (opcode & 0x10u) != 0;
        vertex_count = 2;
        vertex_slots[0] = 1;
        vertex_slots[1] = gouraud ? 3u : 2u;
    } else if (opcode >= 0x60 && opcode <= 0x7F) {
        vertex_count = 1;
        vertex_slots[0] = 1;
        uv_slots[0] = (opcode & 0x04u) ? 2u : 0u;
    }
    while (vertex_count > 0 && vertex_slots[vertex_count - 1] >= gp0_buffer_.size()) {
        --vertex_count;
    }

    if (vertex_count > 0 && hit(reaper_pending_geometry_)) {
        const u32 noise = next_reaper_noise();
        const size_t slot = vertex_slots[noise % vertex_count];
        gp0_buffer_[slot] = mutate_vertex_word(gp0_buffer_[slot], noise);
        --reaper_pending_geometry_;
    }

    if (reaper_pending_texture_ == 0) {
        return;
    }

    if (is_textured_primitive_opcode(opcode)) {
        if (!hit(reaper_pending_texture_)) {
            return;
        }
        size_t candidates[4] = {};
        size_t candidate_count = 0;
        for (size_t i = 0; i < vertex_count; ++i) {
            if (uv_slots[i] != 0 && uv_slots[i] < gp0_buffer_.size()) {
                candidates[candidate_count++] = uv_slots[i];
            }
        }
        if (candidate_count > 0) {
            const u32 noise = next_reaper_noise();
            gp0_buffer_[candidates[noise % candidate_count]] ^= noise;
            --reaper_pending_texture_;
        }
        return;
    }

    if (opcode >= 0xE1 && opcode <= 0xE2) {
        gp0_buffer_[0] ^= (next_reaper_noise() & 0x00FFFFFFu);
        --reaper_pending_texture_;
    }
}

void Gpu::consume_vram_write_word(u32 word) {
    if (sys_ && g_mdec_debug_upload_probe) {
        sys_->debug_note_gpu_image_load_word(word);
    }

    const u16 pixel0 = static_cast<u16>(word & 0xFFFFu);
    const u16 pixel1 = static_cast<u16>(word >> 16);
    const auto write_pixel = [&](u16 x, u16 y, u16 pixel) {
        const size_t index = static_cast<size_t>(y) * psx::VRAM_WIDTH + x;
        if (check_mask_before_draw_ && (vram_[index] & 0x8000u)) {
            return;
        }
        if (force_set_mask_bit_) {
            pixel |= 0x8000u;
        }
        vram_[index] = pixel;
    };

    if (vram_tx_pos_ < vram_tx_total_) {
        const u16 x = static_cast<u16>((vram_tx_x_ + (vram_tx_pos_ % vram_tx_w_)) &
            (psx::VRAM_WIDTH - 1));
        const u16 y = static_cast<u16>((vram_tx_y_ + (vram_tx_pos_ / vram_tx_w_)) &
            (psx::VRAM_HEIGHT - 1));
        write_pixel(x, y, pixel0);
        vram_tx_pos_++;
    }
    if (vram_tx_pos_ < vram_tx_total_) {
        const u16 x = static_cast<u16>((vram_tx_x_ + (vram_tx_pos_ % vram_tx_w_)) &
            (psx::VRAM_WIDTH - 1));
        const u16 y = static_cast<u16>((vram_tx_y_ + (vram_tx_pos_ / vram_tx_w_)) &
            (psx::VRAM_HEIGHT - 1));
        write_pixel(x, y, pixel1);
        vram_tx_pos_++;
    }

    if (vram_tx_pos_ >= vram_tx_total_) {
        gp0_mode_ = Gp0Mode::Command;
        hw_record_vram_write(vram_tx_x_, vram_tx_y_, vram_tx_w_, vram_tx_h_);
    }
}

void Gpu::gp0_from_ram(u32 command, u32 phys) {
    gp0_word_source_ = phys;
    gp0(command);
    gp0_word_source_ = Pgxp::kNoSource;
}

void Gpu::gp0(u32 command) {
    const bool profile_detailed = g_profile_detailed_timing;
    std::chrono::high_resolution_clock::time_point start{};
    if (sys_) {
        // Cheap frame-work counters stay enabled even when detailed timing is
        // off. Spyro's oscillation probe needs the real command cadence without
        // injecting clock reads into the GP0 hot path.
        sys_->add_gpu_gp0_word();
    }
    if (profile_detailed) {
        start = std::chrono::high_resolution_clock::now();
    }
    static u64 gp0_count = 0;
    if (g_trace_gpu &&
        trace_should_log(gp0_count, g_trace_burst_gpu, g_trace_stride_gpu)) {
        LOG_CAT_DEBUG(LogCategory::Gpu, "GPU: GP0[%llu] = 0x%08X mode=%d",
            static_cast<unsigned long long>(gp0_count), command,
            static_cast<int>(gp0_mode_));
    }

    // Only the incoming word's RAM source is known; words that queued behind
    // a VRAM read lose theirs (and just miss PGXP).
    u32 word_source = gp0_fifo_.empty() ? gp0_word_source_ : Pgxp::kNoSource;
    gp0_fifo_.push_back(command);
    while (!gp0_fifo_.empty()) {
        if (gp0_mode_ == Gp0Mode::VramRead) {
            break;
        }
        command = gp0_fifo_.front();
        gp0_fifo_.pop_front();

    // Handle VRAM write mode
    if (gp0_mode_ == Gp0Mode::VramWrite) {
        consume_vram_write_word(command);
        continue;
    }

    if (polyline_active_) {
        handle_polyline_word(command);
        continue;
    }

    // Accumulate command words
    if (gp0_buffer_.empty()) {
        u8 opcode = (command >> 24) & 0xFF;
        u32 length = gp0_command_length(opcode);
        gp0_command_ = opcode;
        gp0_words_remaining_ = length;
    }

    gp0_buffer_.push_back(command);
    gp0_buffer_src_.push_back(word_source);
    word_source = Pgxp::kNoSource;
    gp0_words_remaining_--;

    if (gp0_words_remaining_ > 0)
        continue;

    apply_reaper_to_gp0_command();

    // Full command received — dispatch
    u8 op = gp0_command_;
    if (sys_) {
        sys_->add_gpu_gp0_command();
        sys_->add_gpu_command_bucket(gpu_profile_bucket_for_opcode(op));
        if (op >= 0x20 && op <= 0x7Fu) {
            sys_->add_gpu_draw_command();
        }
    }
    // GP0 draw command bit1 selects semi-transparency for that command.
    semi_transparency_mode_ = (op >= 0x20 && op <= 0x7F) && ((op & 0x02u) != 0);
    std::chrono::high_resolution_clock::time_point command_start{};
    if (profile_detailed) {
        command_start = std::chrono::high_resolution_clock::now();
    }
    switch (op) {
    case 0x00:
        gp0_nop();
        break;
    case 0x01:
        gp0_clear_cache();
        break;
    case 0x02:
        gp0_fill_rect();
        break;
    case 0x20:
    case 0x21:
    case 0x22:
    case 0x23:
        gp0_mono_tri();
        break;
    case 0x24:
    case 0x25:
    case 0x26:
    case 0x27:
        gp0_textured_tri();
        break;
    case 0x28:
    case 0x29:
    case 0x2A:
    case 0x2B:
        gp0_mono_quad();
        break;
    case 0x2C:
    case 0x2D:
    case 0x2E:
    case 0x2F:
        gp0_textured_quad();
        break;
    case 0x30:
    case 0x31:
    case 0x32:
    case 0x33:
        gp0_shaded_tri();
        break;
    case 0x34:
    case 0x35:
    case 0x36:
    case 0x37:
        gp0_shaded_textured_tri();
        break;
    case 0x38:
    case 0x39:
    case 0x3A:
    case 0x3B:
        gp0_shaded_quad();
        break;
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F:
        gp0_shaded_textured_quad();
        break;
    case 0x40:
    case 0x41:
    case 0x42:
    case 0x43:
    case 0x44:
    case 0x45:
    case 0x46:
    case 0x47:
        gp0_mono_line();
        break;
    case 0x48:
    case 0x49:
    case 0x4A:
    case 0x4B:
    case 0x4C:
    case 0x4D:
    case 0x4E:
    case 0x4F:
        gp0_mono_polyline_start();
        break;
    case 0x50:
    case 0x51:
    case 0x52:
    case 0x53:
    case 0x54:
    case 0x55:
    case 0x56:
    case 0x57:
        gp0_shaded_line();
        break;
    case 0x58:
    case 0x59:
    case 0x5A:
    case 0x5B:
    case 0x5C:
    case 0x5D:
    case 0x5E:
    case 0x5F:
        gp0_shaded_polyline_start();
        break;
    case 0x60:
    case 0x61:
    case 0x62:
    case 0x63:
        gp0_mono_rect();
        break;
    case 0x64:
    case 0x65:
    case 0x66:
    case 0x67:
        gp0_textured_rect();
        break;
    case 0x68:
    case 0x69:
    case 0x6A:
    case 0x6B:
        gp0_mono_rect_1();
        break;
    case 0x6C:
    case 0x6D:
    case 0x6E:
    case 0x6F:
        gp0_textured_rect();
        break;
    case 0x70:
    case 0x71:
    case 0x72:
    case 0x73:
        gp0_mono_rect_8();
        break;
    case 0x74:
    case 0x75:
    case 0x76:
    case 0x77:
        gp0_textured_rect();
        break;
    case 0x78:
    case 0x79:
    case 0x7A:
    case 0x7B:
        gp0_mono_rect_16();
        break;
    case 0x7C:
    case 0x7D:
    case 0x7E:
    case 0x7F:
        gp0_textured_rect();
        break;
    case 0xA0:
    case 0xA1:
    case 0xA2:
    case 0xA3:
    case 0xA4:
    case 0xA5:
    case 0xA6:
    case 0xA7:
    case 0xA8:
    case 0xA9:
    case 0xAA:
    case 0xAB:
    case 0xAC:
    case 0xAD:
    case 0xAE:
    case 0xAF:
    case 0xB0:
    case 0xB1:
    case 0xB2:
    case 0xB3:
    case 0xB4:
    case 0xB5:
    case 0xB6:
    case 0xB7:
    case 0xB8:
    case 0xB9:
    case 0xBA:
    case 0xBB:
    case 0xBC:
    case 0xBD:
    case 0xBE:
    case 0xBF:
        gp0_image_load();
        break;
    case 0x80:
    case 0x81:
    case 0x82:
    case 0x83:
    case 0x84:
    case 0x85:
    case 0x86:
    case 0x87:
    case 0x88:
    case 0x89:
    case 0x8A:
    case 0x8B:
    case 0x8C:
    case 0x8D:
    case 0x8E:
    case 0x8F:
    case 0x90:
    case 0x91:
    case 0x92:
    case 0x93:
    case 0x94:
    case 0x95:
    case 0x96:
    case 0x97:
    case 0x98:
    case 0x99:
    case 0x9A:
    case 0x9B:
    case 0x9C:
    case 0x9D:
    case 0x9E:
    case 0x9F:
        gp0_vram_copy();
        break;
    case 0xC0:
    case 0xC1:
    case 0xC2:
    case 0xC3:
    case 0xC4:
    case 0xC5:
    case 0xC6:
    case 0xC7:
    case 0xC8:
    case 0xC9:
    case 0xCA:
    case 0xCB:
    case 0xCC:
    case 0xCD:
    case 0xCE:
    case 0xCF:
    case 0xD0:
    case 0xD1:
    case 0xD2:
    case 0xD3:
    case 0xD4:
    case 0xD5:
    case 0xD6:
    case 0xD7:
    case 0xD8:
    case 0xD9:
    case 0xDA:
    case 0xDB:
    case 0xDC:
    case 0xDD:
    case 0xDE:
    case 0xDF:
        gp0_image_store();
        break;
    case 0xE1:
        gp0_draw_mode();
        break;
    case 0xE2:
        gp0_tex_window();
        break;
    case 0xE3:
        gp0_draw_area_top_left();
        break;
    case 0xE4:
        gp0_draw_area_bottom_right();
        break;
    case 0xE5:
        gp0_draw_offset();
        break;
    case 0xE6:
        gp0_mask_bit();
        break;
    case 0x1F:
        gp0_irq_request();
        break;
    default:
        if ((op >= 0x03 && op <= 0x1E) || op == 0xE0 || op >= 0xE7) {
            // Legal NOP commands on PS1 hardware.
        }
        else {
            LOG_WARN("GPU: Unhandled GP0 command 0x%02X", op);
        }
        break;
    }

    if (profile_detailed && sys_) {
        const auto command_end = std::chrono::high_resolution_clock::now();
        sys_->add_gpu_profile_bucket(
            gpu_profile_bucket_for_opcode(op),
            std::chrono::duration<double, std::milli>(
                command_end - command_start).count());
    }

    gp0_buffer_.clear();
    gp0_buffer_src_.clear();
    }
    if (profile_detailed && sys_) {
        const auto end = std::chrono::high_resolution_clock::now();
        sys_->add_gpu_time(
            std::chrono::duration<double, std::milli>(end - start).count());
    }
}

// ── GP0 Command Implementations ────────────────────────────────────

void Gpu::gp0_nop() {}
void Gpu::gp0_clear_cache() {}

void Gpu::debug_note_polygon(u8 opcode, const Vertex* vertices, int vertex_count,
                             bool textured, bool shaded, bool raw_texture) {
    if (!gpu_command_diagnostics_enabled() || vertices == nullptr ||
        vertex_count <= 0) {
        return;
    }

    const u32 index = command_debug_.gp0_poly_count %
        static_cast<u32>(GpuCommandDebugInfo::kRecentPolys);
    ++command_debug_.gp0_poly_count;
    command_debug_.poly_opcode[index] = opcode;
    command_debug_.poly_vertex_count[index] =
        static_cast<u8>(std::min(vertex_count, 4));
    command_debug_.poly_textured[index] = textured ? 1u : 0u;
    command_debug_.poly_shaded[index] = shaded ? 1u : 0u;
    command_debug_.poly_raw[index] = raw_texture ? 1u : 0u;
    command_debug_.poly_semi[index] = semi_transparency_mode_ ? 1u : 0u;
    command_debug_.poly_blend[index] = semi_transparency_;
    command_debug_.poly_depth[index] =
        textured ? static_cast<u8>((texpage_ >> 7) & 0x3u) : 0u;
    command_debug_.poly_clut[index] = textured ? clut_ : 0u;
    command_debug_.poly_texpage[index] = textured ? texpage_ : 0u;
    for (int i = 0; i < 4; ++i) {
        if (i < vertex_count) {
            command_debug_.poly_x[index][i] = vertices[i].x;
            command_debug_.poly_y[index][i] = vertices[i].y;
            command_debug_.poly_u[index][i] = vertices[i].u;
            command_debug_.poly_v[index][i] = vertices[i].v;
            command_debug_.poly_r[index][i] = vertices[i].color.r;
            command_debug_.poly_g[index][i] = vertices[i].color.g;
            command_debug_.poly_b[index][i] = vertices[i].color.b;
        } else {
            command_debug_.poly_x[index][i] = 0;
            command_debug_.poly_y[index][i] = 0;
            command_debug_.poly_u[index][i] = 0;
            command_debug_.poly_v[index][i] = 0;
            command_debug_.poly_r[index][i] = 0;
            command_debug_.poly_g[index][i] = 0;
            command_debug_.poly_b[index][i] = 0;
        }
    }
}

void Gpu::gp0_fill_rect() {
    Color c(gp0_buffer_[0]);
    u16 x = gp0_buffer_[1] & 0x3F0; // Rounded down to 16-pixel boundary
    u16 y = (gp0_buffer_[1] >> 16) & 0x1FF;
    u16 w = ((gp0_buffer_[2] & 0x3FF) + 0xF) & ~0xF; // Rounded up to 16 pixels
    u16 h = (gp0_buffer_[2] >> 16) & 0x1FF;

    const u16 color15 = c.to_15bit();
    const u16 out = force_set_mask_bit_ ? static_cast<u16>(color15 | 0x8000u) : color15;
    for (u16 dy = 0; dy < h; dy++) {
        for (u16 dx = 0; dx < w; dx++) {
            u16 px = (x + dx) % psx::VRAM_WIDTH;
            u16 py = (y + dy) % psx::VRAM_HEIGHT;
            const size_t index = static_cast<size_t>(py) * psx::VRAM_WIDTH + px;
            if (check_mask_before_draw_ && (vram_[index] & 0x8000u)) {
                continue;
            }
            vram_[index] = out;
        }
    }
    if (x + w > psx::VRAM_WIDTH || y + h > psx::VRAM_HEIGHT ||
        check_mask_before_draw_) {
        hw_record_vram_write(x, y, w, h);
    } else {
        hw_record_fill(x, y, w, h, gp0_buffer_[0]);
    }
}

void Gpu::gp0_mono_tri() {
    Color c(gp0_buffer_[0]);
    Vertex v[3];
    v[0] = decode_buffer_vertex(1);
    v[1] = decode_buffer_vertex(2);
    v[2] = decode_buffer_vertex(3);
    v[0].color = c;
    v[1].color = c;
    v[2].color = c;
    debug_note_polygon(gp0_command_, v, 3, false, false);
    hw_record_polygon(v, 3, false, false);
    draw_flat_triangle(v[0], v[1], v[2], c);
}

void Gpu::gp0_mono_quad() {
    Color c(gp0_buffer_[0]);
    Vertex v[4];
    for (int i = 0; i < 4; i++) {
        v[i] = decode_buffer_vertex(1 + i);
        v[i].color = c;
    }
    debug_note_polygon(gp0_command_, v, 4, false, false);
    hw_record_polygon(v, 4, false, false);
    draw_flat_triangle(v[0], v[1], v[2], c);
    draw_flat_triangle(v[1], v[2], v[3], c);
}

void Gpu::gp0_textured_tri() {
    if (gpu_command_diagnostics_enabled()) {
        ++command_debug_.gp0_textured_tri_count;
    }
    Color c(gp0_buffer_[0]);
    clut_ = static_cast<u16>(gp0_buffer_[2] >> 16);
    texpage_ = static_cast<u16>(gp0_buffer_[4] >> 16);
    semi_transparency_ = static_cast<u8>((texpage_ >> 5) & 0x3u);
    Vertex v[3];
    v[0] = decode_buffer_vertex(1);
    v[0].u = gp0_buffer_[2] & 0xFF;
    v[0].v = (gp0_buffer_[2] >> 8) & 0xFF;
    v[1] = decode_buffer_vertex(3);
    v[1].u = gp0_buffer_[4] & 0xFF;
    v[1].v = (gp0_buffer_[4] >> 8) & 0xFF;
    v[2] = decode_buffer_vertex(5);
    v[2].u = gp0_buffer_[6] & 0xFF;
    v[2].v = (gp0_buffer_[6] >> 8) & 0xFF;
    v[0].color = c;
    v[1].color = c;
    v[2].color = c;
    debug_note_polygon(gp0_command_, v, 3, true, false,
                       (gp0_command_ & 0x1u) != 0);
    hw_record_polygon(v, 3, true, (gp0_command_ & 0x1u) != 0);
    draw_textured_triangle(v[0], v[1], v[2], c);
}

void Gpu::gp0_textured_quad() {
    if (gpu_command_diagnostics_enabled()) {
        ++command_debug_.gp0_textured_quad_count;
    }
    Color c(gp0_buffer_[0]);
    clut_ = static_cast<u16>(gp0_buffer_[2] >> 16);
    texpage_ = static_cast<u16>(gp0_buffer_[4] >> 16);
    semi_transparency_ = static_cast<u8>((texpage_ >> 5) & 0x3u);
    Vertex v[4];
    for (int i = 0; i < 4; i++) {
        int base = 1 + i * 2;
        v[i] = decode_buffer_vertex(base);
        v[i].u = gp0_buffer_[base + 1] & 0xFF;
        v[i].v = (gp0_buffer_[base + 1] >> 8) & 0xFF;
        v[i].color = c;
    }
    debug_note_polygon(gp0_command_, v, 4, true, false,
                       (gp0_command_ & 0x1u) != 0);
    hw_record_polygon(v, 4, true, (gp0_command_ & 0x1u) != 0);
    draw_textured_triangle(v[0], v[1], v[2], c);
    draw_textured_triangle(v[1], v[2], v[3], c);
}

void Gpu::gp0_shaded_tri() {
    Vertex v[3];
    for (int i = 0; i < 3; i++) {
        v[i] = decode_buffer_vertex(i * 2 + 1);
        v[i].color = Color(gp0_buffer_[i * 2]);
    }
    debug_note_polygon(gp0_command_, v, 3, false, true);
    hw_record_polygon(v, 3, false, false);
    draw_shaded_triangle(v[0], v[1], v[2]);
}

void Gpu::gp0_shaded_quad() {
    Vertex v[4];
    for (int i = 0; i < 4; i++) {
        v[i] = decode_buffer_vertex(i * 2 + 1);
        v[i].color = Color(gp0_buffer_[i * 2]);
    }
    debug_note_polygon(gp0_command_, v, 4, false, true);
    hw_record_polygon(v, 4, false, false);
    draw_shaded_triangle(v[0], v[1], v[2]);
    draw_shaded_triangle(v[1], v[2], v[3]);
}

void Gpu::gp0_shaded_textured_tri() {
    Vertex v[3];
    clut_ = static_cast<u16>(gp0_buffer_[2] >> 16);
    texpage_ = static_cast<u16>(gp0_buffer_[5] >> 16);
    semi_transparency_ = static_cast<u8>((texpage_ >> 5) & 0x3u);
    for (int i = 0; i < 3; i++) {
        int base = i * 3;
        v[i] = decode_buffer_vertex(base + 1);
        v[i].color = Color(gp0_buffer_[base]);
        v[i].u = gp0_buffer_[base + 2] & 0xFF;
        v[i].v = (gp0_buffer_[base + 2] >> 8) & 0xFF;
    }
    debug_note_polygon(gp0_command_, v, 3, true, true,
                       (gp0_command_ & 0x1u) != 0);
    hw_record_polygon(v, 3, true, (gp0_command_ & 0x1u) != 0);
    draw_shaded_textured_triangle(v[0], v[1], v[2]);
}

void Gpu::gp0_shaded_textured_quad() {
    Vertex v[4];
    clut_ = static_cast<u16>(gp0_buffer_[2] >> 16);
    texpage_ = static_cast<u16>(gp0_buffer_[5] >> 16);
    semi_transparency_ = static_cast<u8>((texpage_ >> 5) & 0x3u);
    for (int i = 0; i < 4; i++) {
        int base = i * 3;
        v[i] = decode_buffer_vertex(base + 1);
        v[i].color = Color(gp0_buffer_[base]);
        v[i].u = gp0_buffer_[base + 2] & 0xFF;
        v[i].v = (gp0_buffer_[base + 2] >> 8) & 0xFF;
    }
    debug_note_polygon(gp0_command_, v, 4, true, true,
                       (gp0_command_ & 0x1u) != 0);
    hw_record_polygon(v, 4, true, (gp0_command_ & 0x1u) != 0);
    draw_shaded_textured_triangle(v[0], v[1], v[2]);
    draw_shaded_textured_triangle(v[1], v[2], v[3]);
}

void Gpu::gp0_mono_line() {
    Color c(gp0_buffer_[0]);
    Vertex v0 = decode_vertex_word(gp0_buffer_[1]);
    Vertex v1 = decode_vertex_word(gp0_buffer_[2]);
    draw_line_segment(v0, v1, c, semi_transparency_mode_);
}

void Gpu::gp0_mono_polyline_start() {
    const Color c(gp0_buffer_[0]);
    const Vertex v0 = decode_vertex_word(gp0_buffer_[1]);
    const Vertex v1 = decode_vertex_word(gp0_buffer_[2]);
    draw_line_segment(v0, v1, c, semi_transparency_mode_);

    polyline_active_ = true;
    polyline_gouraud_ = false;
    polyline_waiting_vertex_ = false;
    polyline_prev_vertex_ = v1;
    polyline_flat_color_ = c;
    polyline_prev_color_ = c;
    polyline_pending_color_word_ = 0;
}

void Gpu::gp0_shaded_line() {
    const Color c0(gp0_buffer_[0]);
    const Color c1(gp0_buffer_[2]);
    const Vertex v0 = decode_vertex_word(gp0_buffer_[1]);
    const Vertex v1 = decode_vertex_word(gp0_buffer_[3]);
    draw_gouraud_line_segment(v0, c0, v1, c1, semi_transparency_mode_);
}

void Gpu::gp0_shaded_polyline_start() {
    const Color c0(gp0_buffer_[0]);
    const Color c1(gp0_buffer_[2]);
    const Vertex v0 = decode_vertex_word(gp0_buffer_[1]);
    const Vertex v1 = decode_vertex_word(gp0_buffer_[3]);
    draw_line_segment(v0, v1, c0, semi_transparency_mode_);

    polyline_active_ = true;
    polyline_gouraud_ = true;
    polyline_waiting_vertex_ = false;
    polyline_prev_vertex_ = v1;
    polyline_flat_color_ = c0;
    polyline_prev_color_ = c1;
    polyline_pending_color_word_ = 0;
}

Vertex Gpu::decode_vertex_word(u32 word) const {
    Vertex v{};
    // Coordinates are signed 11-bit; the drawing offset is added after, without
    // wrapping (double-buffered games draw past x=1023 from a buffer at x=512).
    v.x = static_cast<s16>(sign_extend_11(word) + draw_x_offset_);
    v.y = static_cast<s16>(sign_extend_11(word >> 16) + draw_y_offset_);
    v.fx = static_cast<float>(v.x);
    v.fy = static_cast<float>(v.y);
    return v;
}

Vertex Gpu::decode_buffer_vertex(size_t index) const {
    const u32 word = gp0_buffer_[index];
    Vertex v = decode_vertex_word(word);
    if (!g_pgxp_enabled || pgxp_ == nullptr || index >= gp0_buffer_src_.size()) {
        return v;
    }
    Pgxp::PreciseVertex precise;
    // The GPU ignores the top 5 bits of each coordinate, but the stored word
    // must match exactly for the precise vertex to be this one.
    if (pgxp_->lookup(gp0_buffer_src_[index], word, precise)) {
        v.fx = precise.x + static_cast<float>(draw_x_offset_);
        v.fy = precise.y + static_cast<float>(draw_y_offset_);
        v.w = precise.w;
        v.has_w = true;
    }
    return v;
}

void Gpu::draw_line_segment(Vertex a, Vertex b, Color c, bool semi_transparent) {
    // PS1 hardware rejects lines exceeding 1023x511 span, same as triangles.
    const int dx_span = std::abs(static_cast<int>(b.x) - static_cast<int>(a.x));
    const int dy_span = std::abs(static_cast<int>(b.y) - static_cast<int>(a.y));
    if (dx_span > 1023 || dy_span > 511) {
        return;
    }
    hw_record_line(a, c, b, c);
    s16 x0 = a.x;
    s16 y0 = a.y;
    const s16 x1 = b.x;
    const s16 y1 = b.y;
    const u16 color15 = c.to_15bit();
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        set_pixel(x0, y0, color15, semi_transparent);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int e2 = err * 2;
        if (e2 > -dy) {
            err -= dy;
            x0 = static_cast<s16>(x0 + sx);
        }
        if (e2 < dx) {
            err += dx;
            y0 = static_cast<s16>(y0 + sy);
        }
    }
}

void Gpu::draw_gouraud_line_segment(Vertex a, Color ca, Vertex b, Color cb,
    bool semi_transparent) {
    const int dx_span = std::abs(static_cast<int>(b.x) - static_cast<int>(a.x));
    const int dy_span = std::abs(static_cast<int>(b.y) - static_cast<int>(a.y));
    if (dx_span > 1023 || dy_span > 511) {
        return;
    }
    hw_record_line(a, ca, b, cb);
    s16 x0 = a.x;
    s16 y0 = a.y;
    const s16 x1 = b.x;
    const s16 y1 = b.y;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    const int total = std::max(1, dx + dy);
    int step = 0;
    while (true) {
        const float t = static_cast<float>(step) / static_cast<float>(total);
        const u8 r = static_cast<u8>(static_cast<float>(ca.r) +
            (static_cast<float>(cb.r) - static_cast<float>(ca.r)) * t + 0.5f);
        const u8 g = static_cast<u8>(static_cast<float>(ca.g) +
            (static_cast<float>(cb.g) - static_cast<float>(ca.g)) * t + 0.5f);
        const u8 b_c = static_cast<u8>(static_cast<float>(ca.b) +
            (static_cast<float>(cb.b) - static_cast<float>(ca.b)) * t + 0.5f);
        const Color c(r, g, b_c);
        set_pixel(x0, y0, c.to_15bit(), semi_transparent);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        ++step;
        const int e2 = err * 2;
        if (e2 > -dy) {
            err -= dy;
            x0 = static_cast<s16>(x0 + sx);
        }
        if (e2 < dx) {
            err += dx;
            y0 = static_cast<s16>(y0 + sy);
        }
    }
}

void Gpu::handle_polyline_word(u32 word) {
    if (!polyline_gouraud_) {
        if (is_polyline_terminator(word)) {
            polyline_active_ = false;
            polyline_waiting_vertex_ = false;
            polyline_pending_color_word_ = 0;
            return;
        }

        const Vertex next = decode_vertex_word(word);
        draw_line_segment(polyline_prev_vertex_, next, polyline_flat_color_,
            semi_transparency_mode_);
        polyline_prev_vertex_ = next;
        return;
    }

    if (!polyline_waiting_vertex_) {
        if (is_polyline_terminator(word)) {
            polyline_active_ = false;
            polyline_waiting_vertex_ = false;
            polyline_pending_color_word_ = 0;
            return;
        }

        polyline_pending_color_word_ = word;
        polyline_waiting_vertex_ = true;
        return;
    }

    const Vertex next = decode_vertex_word(word);
    const Color new_color(polyline_pending_color_word_);
    draw_gouraud_line_segment(polyline_prev_vertex_, polyline_prev_color_,
        next, new_color, semi_transparency_mode_);
    polyline_prev_color_ = new_color;
    polyline_prev_vertex_ = next;
    polyline_waiting_vertex_ = false;
}

void Gpu::gp0_mono_rect() {
    Color c(gp0_buffer_[0]);
    const Vertex origin = decode_vertex_word(gp0_buffer_[1]);
    s16 x = origin.x;
    s16 y = origin.y;
    u16 w = static_cast<u16>(gp0_buffer_[2] & 0x3FFu);
    u16 h = static_cast<u16>((gp0_buffer_[2] >> 16) & 0x1FFu);
    hw_record_rect(x, y, w, h, c, false, 0, 0, false);
    draw_rect(x, y, w, h, c);
}

void Gpu::gp0_textured_rect() {
    if (gpu_command_diagnostics_enabled()) {
        ++command_debug_.gp0_textured_rect_count;
    }
    Color c(gp0_buffer_[0]);
    const Vertex origin = decode_vertex_word(gp0_buffer_[1]);
    s16 x = origin.x;
    s16 y = origin.y;
    u8 u = gp0_buffer_[2] & 0xFF;
    u8 v = (gp0_buffer_[2] >> 8) & 0xFF;
    clut_ = static_cast<u16>(gp0_buffer_[2] >> 16);
    semi_transparency_ = static_cast<u8>((texpage_ >> 5) & 0x3u);
    u16 w, h;
    u8 opcode = gp0_command_;
    if (opcode >= 0x74 && opcode <= 0x77) {
        w = 8;
        h = 8;
    }
    else if (opcode >= 0x7C && opcode <= 0x7F) {
        w = 16;
        h = 16;
    }
    else if (gp0_buffer_.size() >= 4) {
        w = static_cast<u16>(gp0_buffer_[3] & 0x3FFu);
        h = static_cast<u16>((gp0_buffer_[3] >> 16) & 0x1FFu);
    }
    else {
        w = 1;
        h = 1;
    }

    const bool raw_texture = (gp0_command_ & 0x1u) != 0;
    hw_record_rect(x, y, w, h, c, true, u, v, raw_texture);
    if (gpu_command_diagnostics_enabled()) {
        const u32 rect_index =
            (command_debug_.gp0_textured_rect_count - 1u) %
            static_cast<u32>(GpuCommandDebugInfo::kRecentRects);
        command_debug_.rect_x[rect_index] = x;
        command_debug_.rect_y[rect_index] = y;
        command_debug_.rect_w[rect_index] = w;
        command_debug_.rect_h[rect_index] = h;
        command_debug_.rect_u[rect_index] = u;
        command_debug_.rect_v[rect_index] = v;
        command_debug_.rect_clut[rect_index] = clut_;
        command_debug_.rect_texpage[rect_index] = texpage_;
        command_debug_.rect_opcode[rect_index] = opcode;
        command_debug_.rect_semi[rect_index] = semi_transparency_mode_ ? 1u : 0u;
        command_debug_.rect_blend[rect_index] = semi_transparency_;
        command_debug_.rect_raw[rect_index] = raw_texture ? 1u : 0u;
        command_debug_.rect_r[rect_index] = c.r;
        command_debug_.rect_g[rect_index] = c.g;
        command_debug_.rect_b[rect_index] = c.b;
        command_debug_.rect_depth[rect_index] =
            static_cast<u8>((texpage_ >> 7) & 0x3u);
    }

    // Textured rectangle draw path (supports 4/8/15-bit tex fetch via
    // read_texel).
    const TextureSampleState texture = prepare_texture_sample_state();
    const bool profile_raster = g_profile_detailed_timing && sys_ != nullptr;
    const u64 profile_samples =
        static_cast<u64>(w) * static_cast<u64>(h);
    u64 profile_transparent = 0;
    u64 profile_semi = 0;
    for (u16 dy = 0; dy < h; ++dy) {
        for (u16 dx = 0; dx < w; ++dx) {
            const u16 src_dx = tex_rect_x_flip_ ? static_cast<u16>(w - 1 - dx) : dx;
            const u16 src_dy = tex_rect_y_flip_ ? static_cast<u16>(h - 1 - dy) : dy;
            const u8 src_u = static_cast<u8>(u + src_dx);
            const u8 src_v = static_cast<u8>(v + src_dy);
            u16 texel = read_texel(texture, src_u, src_v);
            if (texel == 0) {
                if (profile_raster) {
                    ++profile_transparent;
                }
                continue; // Color 0 transparent in many textured modes.
            }

            u16 out15 = texel;
            if (!raw_texture) {
                if (dither_enabled_) {
                    out15 = modulate_texel_dithered_15bit(
                        texel, c.r, c.g, c.b, static_cast<s16>(x + dx),
                        static_cast<s16>(y + dy));
                }
                else {
                    out15 = modulate_texel_15bit(texel, c.r, c.g, c.b);
                }
            }
            const bool texel_semi = semi_transparency_mode_ && ((texel & 0x8000u) != 0);
            if (profile_raster && texel_semi) {
                ++profile_semi;
            }
            set_pixel(static_cast<s16>(x + dx), static_cast<s16>(y + dy), out15,
                texel_semi);
        }
    }
    if (profile_raster) {
        sys_->add_gpu_raster_work(
            profile_samples, profile_samples, profile_samples,
            texture.depth, profile_transparent, profile_semi);
    }
}

void Gpu::gp0_mono_rect_1() {
    Color c(gp0_buffer_[0]);
    const Vertex origin = decode_vertex_word(gp0_buffer_[1]);
    s16 x = origin.x;
    s16 y = origin.y;
    hw_record_rect(x, y, 1, 1, c, false, 0, 0, false);
    draw_rect(x, y, 1, 1, c);
}

void Gpu::gp0_mono_rect_8() {
    Color c(gp0_buffer_[0]);
    const Vertex origin = decode_vertex_word(gp0_buffer_[1]);
    s16 x = origin.x;
    s16 y = origin.y;
    hw_record_rect(x, y, 8, 8, c, false, 0, 0, false);
    draw_rect(x, y, 8, 8, c);
}

void Gpu::gp0_mono_rect_16() {
    Color c(gp0_buffer_[0]);
    const Vertex origin = decode_vertex_word(gp0_buffer_[1]);
    s16 x = origin.x;
    s16 y = origin.y;
    hw_record_rect(x, y, 16, 16, c, false, 0, 0, false);
    draw_rect(x, y, 16, 16, c);
}

void Gpu::gp0_draw_mode() {
    u32 val = gp0_buffer_[0];
    texpage_ = static_cast<u16>(val & 0x1FF);
    dither_enabled_ = ((val >> 9) & 0x1u) != 0;
    draw_to_display_ = ((val >> 10) & 0x1u) != 0;
    semi_transparency_ = static_cast<u8>((val >> 5) & 0x3);
    texture_disable_ = (val >> 11) & 1;
    tex_rect_x_flip_ = ((val >> 12) & 0x1u) != 0;
    tex_rect_y_flip_ = ((val >> 13) & 0x1u) != 0;
}

void Gpu::gp0_tex_window() {
    u32 val = gp0_buffer_[0];
    tex_window_mask_x_ = (val & 0x1F) * 8;
    tex_window_mask_y_ = ((val >> 5) & 0x1F) * 8;
    tex_window_off_x_ = ((val >> 10) & 0x1F) * 8;
    tex_window_off_y_ = ((val >> 15) & 0x1F) * 8;
}

void Gpu::gp0_draw_area_top_left() {
    u32 val = gp0_buffer_[0];
    draw_x_min_ = static_cast<s16>(val & 0x3FF);
    draw_y_min_ = static_cast<s16>((val >> 10) & 0x1FF);
}

void Gpu::gp0_draw_area_bottom_right() {
    u32 val = gp0_buffer_[0];
    draw_x_max_ = static_cast<s16>(val & 0x3FF);
    draw_y_max_ = static_cast<s16>((val >> 10) & 0x1FF);
}

void Gpu::gp0_draw_offset() {
    u32 val = gp0_buffer_[0];
    // 11-bit signed values
    draw_x_offset_ = static_cast<s16>((val & 0x7FF) << 5) >> 5;
    draw_y_offset_ = static_cast<s16>(((val >> 11) & 0x7FF) << 5) >> 5;
}

void Gpu::gp0_mask_bit() {
    // Bit 0: Set mask while drawing
    // Bit 1: Check mask during drawing
    const u32 val = gp0_buffer_[0];
    force_set_mask_bit_ = (val & 0x1u) != 0;
    check_mask_before_draw_ = (val & 0x2u) != 0;
}

void Gpu::gp0_irq_request() {
    irq1_pending_ = true;
    if (sys_) {
        sys_->irq().request(Interrupt::GPU);
    }
}

void Gpu::gp0_image_load() {
    // CPU → VRAM transfer
    vram_tx_x_ = gp0_buffer_[1] & 0x3FF;
    vram_tx_y_ = (gp0_buffer_[1] >> 16) & 0x1FF;
    vram_tx_w_ = gp0_buffer_[2] & 0x3FF;
    vram_tx_h_ = (gp0_buffer_[2] >> 16) & 0x1FF;

    if (vram_tx_w_ == 0)
        vram_tx_w_ = 1024;
    if (vram_tx_h_ == 0)
        vram_tx_h_ = 512;

    vram_tx_pos_ = 0;
    vram_tx_total_ = static_cast<u32>(vram_tx_w_) * vram_tx_h_;

    if (sys_) {
        if (gpu_command_diagnostics_enabled()) {
            sys_->debug_note_gpu_image_load_begin(vram_tx_x_, vram_tx_y_,
                                                  vram_tx_w_, vram_tx_h_);
        }
    }
    gp0_mode_ = Gp0Mode::VramWrite;
}

void Gpu::gp0_image_store() {
    // VRAM → CPU transfer
    vram_tx_x_ = gp0_buffer_[1] & 0x3FF;
    vram_tx_y_ = (gp0_buffer_[1] >> 16) & 0x1FF;
    vram_tx_w_ = gp0_buffer_[2] & 0x3FF;
    vram_tx_h_ = (gp0_buffer_[2] >> 16) & 0x1FF;

    if (vram_tx_w_ == 0)
        vram_tx_w_ = 1024;
    if (vram_tx_h_ == 0)
        vram_tx_h_ = 512;

    vram_tx_pos_ = 0;
    vram_tx_total_ = static_cast<u32>(vram_tx_w_) * vram_tx_h_;
    gp0_mode_ = Gp0Mode::VramRead;
}

void Gpu::gp0_vram_copy() {
    // GP0(80h): VRAM->VRAM block copy
    u16 src_x = gp0_buffer_[1] & 0x3FF;
    u16 src_y = (gp0_buffer_[1] >> 16) & 0x1FF;
    u16 dst_x = gp0_buffer_[2] & 0x3FF;
    u16 dst_y = (gp0_buffer_[2] >> 16) & 0x1FF;
    u16 w = gp0_buffer_[3] & 0x3FF;
    u16 h = (gp0_buffer_[3] >> 16) & 0x1FF;

    if (w == 0)
        w = 1024;
    if (h == 0)
        h = 512;

    if (sys_) {
        if (gpu_command_diagnostics_enabled()) {
            sys_->debug_note_gpu_vram_copy(src_x, src_y, dst_x, dst_y, w, h);
        }
    }

    // Copy through a temp line buffer to handle overlaps safely.
    const size_t copy_pixels = static_cast<size_t>(w) * static_cast<size_t>(h);
    vram_copy_buffer_.resize(copy_pixels);
    for (u16 y = 0; y < h; ++y) {
        for (u16 x = 0; x < w; ++x) {
            u16 sx = static_cast<u16>((src_x + x) & (psx::VRAM_WIDTH - 1));
            u16 sy = static_cast<u16>((src_y + y) & (psx::VRAM_HEIGHT - 1));
            vram_copy_buffer_[static_cast<size_t>(y) * w + x] =
                vram_[sy * psx::VRAM_WIDTH + sx];
        }
    }
    for (u16 y = 0; y < h; ++y) {
        for (u16 x = 0; x < w; ++x) {
            u16 dx = static_cast<u16>((dst_x + x) & (psx::VRAM_WIDTH - 1));
            u16 dy = static_cast<u16>((dst_y + y) & (psx::VRAM_HEIGHT - 1));
            const size_t dst_index = static_cast<size_t>(dy) * psx::VRAM_WIDTH + dx;
            if (check_mask_before_draw_ && (vram_[dst_index] & 0x8000u)) {
                continue;
            }
            u16 pixel = vram_copy_buffer_[static_cast<size_t>(y) * w + x];
            if (force_set_mask_bit_) {
                pixel |= 0x8000u;
            }
            vram_[dst_index] = pixel;
        }
    }
    if (check_mask_before_draw_ || force_set_mask_bit_) {
        hw_record_vram_write(dst_x, dst_y, w, h);
    } else {
        hw_record_vram_copy(src_x, src_y, dst_x, dst_y, w, h);
    }
}

// ── GP1 (Display Control) ──────────────────────────────────────────

void Gpu::gp1(u32 command) {
    static u64 gp1_count = 0;
    if (g_trace_gpu &&
        trace_should_log(gp1_count, g_trace_burst_gpu, g_trace_stride_gpu)) {
        LOG_CAT_DEBUG(LogCategory::Gpu, "GPU: GP1[%llu] = 0x%08X",
            static_cast<unsigned long long>(gp1_count), command);
    }
    u8 op = (command >> 24) & 0x3F;

    switch (op) {
    case 0x00:
        gp1_reset();
        break;
    case 0x01:
        gp1_reset_command_buffer();
        break;
    case 0x02:
        gp1_ack_irq();
        break;
    case 0x03:
        gp1_display_enable(command);
        break;
    case 0x04:
        gp1_dma_direction(command);
        break;
    case 0x05:
        gp1_display_area(command);
        break;
    case 0x06:
        gp1_horizontal_range(command);
        break;
    case 0x07:
        gp1_vertical_range(command);
        break;
    case 0x08:
        gp1_display_mode(command);
        break;
    case 0x10:
    case 0x11:
    case 0x12:
    case 0x13:
    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17:
    case 0x18:
    case 0x19:
    case 0x1A:
    case 0x1B:
    case 0x1C:
    case 0x1D:
    case 0x1E:
    case 0x1F:
        gp1_get_info(command);
        break;
    default:
        if ((op >= 0x09 && op <= 0x0F) || op >= 0x20) {
            // Legal NOP commands on PS1 hardware.
        }
        else {
            LOG_WARN("GPU: Unhandled GP1 command 0x%02X", op);
        }
        break;
    }
}

void Gpu::gp1_reset() {
    reset();
    // PSX-SPX: GP1(00h) sets specific defaults after reset
    display_.display_enabled = false; // GP1(03h) display off
    dma_direction_ = 0;               // GP1(04h) dma off
    display_.x_start = 0;             // GP1(05h) display address (0)
    display_.y_start = 0;
    display_.x1 = 0x200;            // GP1(06h) x1=200h
    display_.x2 = 0x200 + 256 * 10; // GP1(06h) x2=200h+256*10
    display_.y1 = 0x010;            // GP1(07h) y1=010h
    display_.y2 = 0x010 + 240;      // GP1(07h) y2=010h+240
}

void Gpu::gp1_reset_command_buffer() {
    gp0_fifo_.clear();
    gp0_buffer_.clear();
    gp0_buffer_src_.clear();
    gp0_words_remaining_ = 0;
    gp0_mode_ = Gp0Mode::Command;
    polyline_active_ = false;
    polyline_waiting_vertex_ = false;
    polyline_pending_color_word_ = 0;
}

void Gpu::gp1_ack_irq() {
    irq1_pending_ = false;
}

void Gpu::gp1_display_enable(u32 val) { display_.display_enabled = !(val & 1); }

void Gpu::gp1_dma_direction(u32 val) {
    dma_direction_ = static_cast<u8>(val & 0x3u);
}

void Gpu::gp1_display_area(u32 val) {
    const u32 value = val & 0x00FFFFFFu;
    display_.x_start = value & 0x3FE; // 10 bits, aligned to 2
    display_.y_start = (value >> 10) & 0x1FF;
    if (gpu_command_diagnostics_enabled()) {
        ++command_debug_.gp1_display_area_count;
        command_debug_.gp1_display_area_raw = value;
        command_debug_.gp1_display_area_x = display_.x_start;
        command_debug_.gp1_display_area_y = display_.y_start;
    }
}

void Gpu::gp1_horizontal_range(u32 val) {
    display_.x1 = val & 0xFFF;
    display_.x2 = (val >> 12) & 0xFFF;
    if (gpu_command_diagnostics_enabled()) {
        ++command_debug_.gp1_horizontal_range_count;
        command_debug_.gp1_horizontal_range_raw = val & 0x00FFFFFFu;
        command_debug_.gp1_horizontal_range_x1 = display_.x1;
        command_debug_.gp1_horizontal_range_x2 = display_.x2;
    }
}

void Gpu::gp1_vertical_range(u32 val) {
    display_.y1 = val & 0x3FF;
    display_.y2 = (val >> 10) & 0x3FF;
    if (gpu_command_diagnostics_enabled()) {
        ++command_debug_.gp1_vertical_range_count;
        command_debug_.gp1_vertical_range_raw = val & 0x001FFFFFu;
        command_debug_.gp1_vertical_range_y1 = display_.y1;
        command_debug_.gp1_vertical_range_y2 = display_.y2;
    }
}

void Gpu::gp1_display_mode(u32 val) {
    u8 hres1 = val & 3;
    u8 hres2 = (val >> 6) & 1;
    display_.hres = hres2 ? 4 : hres1;
    display_.vres = (val >> 2) & 1;
    display_.is_pal = (val >> 3) & 1;
    display_.is_24bit = (val >> 4) & 1;
    display_.interlaced = (val >> 5) & 1;
    if (gpu_command_diagnostics_enabled()) {
        ++command_debug_.gp1_display_mode_count;
        command_debug_.gp1_display_mode_raw = val & 0x7Fu;
    }
}

void Gpu::gp1_get_info(u32 val) {
    // GP1(10h..1Fh): GPU info command; low nibble selects info register.
    gpuread_latch_ = gp1_info_value(val & 0x0Fu);
}

u32 Gpu::gp1_info_value(u32 index) const {
    switch (index & 0x0Fu) {
    case 0x00: { // Draw mode setting
        u32 value = texpage_ & 0x1FFu;
        value |= (dither_enabled_ ? 1u : 0u) << 9;
        value |= (draw_to_display_ ? 1u : 0u) << 10;
        value |= (texture_disable_ ? 1u : 0u) << 11;
        value |= (tex_rect_x_flip_ ? 1u : 0u) << 12;
        value |= (tex_rect_y_flip_ ? 1u : 0u) << 13;
        return value;
    }
    case 0x01: // Texture window setting
        return ((tex_window_mask_x_ / 8u) & 0x1Fu) |
            (((tex_window_mask_y_ / 8u) & 0x1Fu) << 5) |
            (((tex_window_off_x_ / 8u) & 0x1Fu) << 10) |
            (((tex_window_off_y_ / 8u) & 0x1Fu) << 15);
    case 0x02: // Drawing area top-left
        return (static_cast<u32>(draw_x_min_) & 0x3FFu) |
            ((static_cast<u32>(draw_y_min_) & 0x1FFu) << 10);
    case 0x03: // Drawing area bottom-right
        return (static_cast<u32>(draw_x_max_) & 0x3FFu) |
            ((static_cast<u32>(draw_y_max_) & 0x1FFu) << 10);
    case 0x04: { // Drawing offset
        const u32 x = static_cast<u32>(draw_x_offset_) & 0x7FFu;
        const u32 y = static_cast<u32>(draw_y_offset_) & 0x7FFu;
        return x | (y << 11);
    }
    default:
        return 0;
    }
}

// ── GPUREAD / GPUSTAT ──────────────────────────────────────────────

u32 Gpu::read_data() {
    if (gp0_mode_ == Gp0Mode::VramRead && vram_tx_pos_ < vram_tx_total_) {
        u16 p0 = 0, p1 = 0;
        u16 x = static_cast<u16>((vram_tx_x_ + (vram_tx_pos_ % vram_tx_w_)) &
            (psx::VRAM_WIDTH - 1));
        u16 y = static_cast<u16>((vram_tx_y_ + (vram_tx_pos_ / vram_tx_w_)) &
            (psx::VRAM_HEIGHT - 1));
        p0 = vram_[y * psx::VRAM_WIDTH + x];
        vram_tx_pos_++;

        if (vram_tx_pos_ < vram_tx_total_) {
            x = static_cast<u16>((vram_tx_x_ + (vram_tx_pos_ % vram_tx_w_)) &
                (psx::VRAM_WIDTH - 1));
            y = static_cast<u16>((vram_tx_y_ + (vram_tx_pos_ / vram_tx_w_)) &
                (psx::VRAM_HEIGHT - 1));
            p1 = vram_[y * psx::VRAM_WIDTH + x];
            vram_tx_pos_++;
        }

        if (vram_tx_pos_ >= vram_tx_total_) {
            gp0_mode_ = Gp0Mode::Command;
            if (!gp0_fifo_.empty()) {
                auto queued_words = std::move(gp0_fifo_);
                gp0_fifo_.clear();
                for (u32 queued_word : queued_words) {
                    gp0(queued_word);
                }
            }
        }

        return static_cast<u32>(p0) | (static_cast<u32>(p1) << 16);
    }
    return gpuread_latch_;
}

u32 Gpu::read_stat() const {
    // Match the reference core's dynamic GPUSTAT bits more closely:
    // bit 26 reflects GPU idle, bit 27 only reflects an active VRAM->CPU read,
    // and bit 28 is the receive side of the GP0/DMA path.
    const bool gpu_idle = (gp0_mode_ == Gp0Mode::Command) &&
        gp0_fifo_.empty() && gp0_buffer_.empty() && !polyline_active_;
    const bool vram_read_ready =
        (gp0_mode_ == Gp0Mode::VramRead) && (vram_tx_pos_ < vram_tx_total_);
    const bool dma_block_ready = (gp0_mode_ != Gp0Mode::VramRead);
    u32 dma_req = 0;
    switch (dma_direction_ & 0x3u) {
    case 0:
        // GP1(04h)=0: DMA/data request must stay deasserted.
        dma_req = 0;
        break;
    case 1:
        // FIFO mode: report "not full" in this simplified implementation.
        dma_req = 1;
        break;
    case 2:
        // GP1(04h)=2: DMA/data request follows GPUSTAT bit 28.
        dma_req = dma_block_ready ? 1u : 0u;
        break;
    case 3:
        // GP1(04h)=3: DMA/data request follows GPUSTAT bit 27.
        dma_req = vram_read_ready ? 1u : 0u;
        break;
    default:
        dma_req = 0;
        break;
    }

    u32 stat = 0;
    stat |= (texpage_ & 0xF);           // Texture page X base
    stat |= ((texpage_ >> 4) & 1) << 4; // Texture page Y base
    stat |= (semi_transparency_ & 3) << 5;
    stat |= ((texpage_ >> 7) & 3) << 7; // Texture page colors
    stat |= (dither_enabled_ ? 1u : 0u) << 9;
    stat |= (draw_to_display_ ? 1u : 0u) << 10;
    stat |= (force_set_mask_bit_ ? 1u : 0u) << 11;
    stat |= (check_mask_before_draw_ ? 1u : 0u) << 12;
    // Bit 13: Interlace Field. Always 1 when interlace is off (GP1(08h).5=0).
    const bool interlace_field_bit = display_.interlaced ? interlace_field_ : true;
    stat |= (interlace_field_bit ? 1u : 0u) << 13;
    stat |= (texture_disable_ ? 1 : 0) << 15;
    stat |= (display_.hres == 4 ? 1 : 0) << 16; // HR2
    stat |= (display_.hres & 3) << 17;          // HR1
    stat |= (display_.vres & 1) << 19;
    stat |= (display_.is_pal ? 1 : 0) << 20;
    stat |= (display_.is_24bit ? 1 : 0) << 21;
    stat |= (display_.interlaced ? 1 : 0) << 22;
    stat |= (display_.display_enabled ? 0 : 1) << 23; // Display disable
    stat |= (irq1_pending_ ? 1u : 0u) << 24;
    stat |= dma_req << 25;
    stat |= (gpu_idle ? 1u : 0u) << 26;
    stat |= (vram_read_ready ? 1u : 0u) << 27;
    stat |= (dma_block_ready ? 1u : 0u) << 28;
    stat |= (static_cast<u32>(dma_direction_ & 0x3u) << 29);
    // PSX-SPX: bit 31 = "Drawing even/odd lines in interlace mode
    //   (0=Even or Vblank, 1=Odd)"
    // bit 13 = "Interlace Field (or, always 1 when GP1(08h).5=0)"
    // When interlace is off, bit 31 should always report 1 (odd).
    bool field_bit = display_.interlaced ? interlace_field_ : true;
    stat |= (field_bit ? 1u : 0u) << 31;
    return stat;
}

bool Gpu::dma_request() const {
    const bool vram_read_ready =
        (gp0_mode_ == Gp0Mode::VramRead) && (vram_tx_pos_ < vram_tx_total_);
    const bool dma_block_ready = (gp0_mode_ != Gp0Mode::VramRead);
    switch (dma_direction_ & 0x3u) {
    case 1:
        return true;
    case 2:
        return dma_block_ready;
    case 3:
        return vram_read_ready;
    default:
        return false;
    }
}

DisplaySampleInfo Gpu::build_display_rgba(std::vector<u32>* rgba,
    bool include_stats) const {
    DisplaySampleInfo info{};
    info.display_enabled = display_.display_enabled;
    info.is_24bit = display_.is_24bit;
    info.x_start = static_cast<int>(display_.x_start);
    info.y_start = static_cast<int>(display_.y_start);

    const CrtcRect crtc = calculate_crtc_rect(display_);
    const int width = crtc.width;
    const int height = crtc.height;
    const int display_vram_width = crtc.vram_width;
    const int display_vram_left = crtc.vram_left;
    const int display_vram_top = crtc.vram_top;
    const int display_skip_x = crtc.skip_x;
    const int src_width = std::max(1, width);
    const int src_height = std::max(1, height);
    // Present the actual CRTC-sized frame. The UI layer already scales this
    // texture for display, so forcing a fixed software output resolution here
    // only adds an extra resample and can hide the true GPU output.
    info.width = std::max(1, width);
    info.height = std::max(1, height);

    const size_t output_pixel_count =
        static_cast<size_t>(info.width) * static_cast<size_t>(info.height);
    if (!info.display_enabled) {
        if (rgba != nullptr) {
            rgba->assign(output_pixel_count, 0xFF000000u);
        }
        return info;
    }

    u32 hash = 2166136261u;
    u64 non_black = 0;
    const bool interlaced_field_output =
        display_.interlaced && (display_.vres != 0);
    const int field_parity = interlaced_field_output
        ? (interlace_field_ ? 1 : 0)
        : 0;
    const DeinterlaceMode deinterlace_mode =
        interlaced_field_output ? g_deinterlace_mode : DeinterlaceMode::Weave;

    if (rgba != nullptr && !include_stats && !display_.is_24bit &&
        !interlaced_field_output && display_vram_left >= 0 && display_vram_top >= 0 &&
        display_vram_left + src_width <= static_cast<int>(psx::VRAM_WIDTH) &&
        display_vram_top + src_height <= static_cast<int>(psx::VRAM_HEIGHT)) {
        std::vector<u32>& out = *rgba;
        out.resize(output_pixel_count);
        const auto& rgb_lut = rgb555_to_rgba_lut();
        for (int y = 0; y < info.height; ++y) {
            const size_t src_row =
                static_cast<size_t>(display_vram_top + y) * psx::VRAM_WIDTH +
                static_cast<size_t>(display_vram_left);
            const size_t dst_row = static_cast<size_t>(y) * static_cast<size_t>(info.width);
            const u16* src = vram_.data() + src_row;
            u32* dst = out.data() + dst_row;
            for (int x = 0; x < info.width; ++x) {
                dst[x] = rgb_lut[src[x] & 0x7FFFu];
            }
        }
        suppress_isolated_bottom_noise_row(rgba, info.width, info.height);
        return info;
    }

    if (rgba != nullptr) {
        rgba->assign(output_pixel_count, 0xFF000000u);
    }

    auto read_rgb = [&](int vram_y, int x, u8& r, u8& g, u8& b) -> bool {
        if (vram_y < 0 || vram_y >= static_cast<int>(psx::VRAM_HEIGHT)) {
            return false;
        }

        if (!display_.is_24bit) {
            const int vram_x = display_vram_left + x;
            if (vram_x < 0 || vram_x >= static_cast<int>(psx::VRAM_WIDTH)) {
                return false;
            }
            const u16 pixel = vram_[static_cast<size_t>(vram_y) * psx::VRAM_WIDTH +
                static_cast<size_t>(vram_x)];
            const u8 r5 = static_cast<u8>(pixel & 0x1F);
            const u8 g5 = static_cast<u8>((pixel >> 5) & 0x1F);
            const u8 b5 = static_cast<u8>((pixel >> 10) & 0x1F);
            r = static_cast<u8>((r5 << 3) | (r5 >> 2));
            g = static_cast<u8>((g5 << 3) | (g5 >> 2));
            b = static_cast<u8>((b5 << 3) | (b5 >> 2));
            return true;
        }

        const u32 sample_x = static_cast<u32>(display_skip_x + x);
        const u32 vram_word_x =
            static_cast<u32>(info.x_start) + ((sample_x * 3u) / 2u);
        if (vram_word_x >= psx::VRAM_WIDTH) {
            return false;
        }

        const size_t row_base = static_cast<size_t>(vram_y) * psx::VRAM_WIDTH;
        const u16 s0 = vram_[row_base + (vram_word_x % psx::VRAM_WIDTH)];
        const u16 s1 = vram_[row_base + ((vram_word_x + 1u) % psx::VRAM_WIDTH)];
        const u32 packed =
            ((static_cast<u32>(s1) << 16) | static_cast<u32>(s0)) >>
            ((static_cast<u32>(x) & 1u) * 8u);
        r = static_cast<u8>(packed & 0xFFu);
        g = static_cast<u8>((packed >> 8) & 0xFFu);
        b = static_cast<u8>((packed >> 16) & 0xFFu);
        return true;
        };

    // info.width/height equal src_width/src_height, so output pixels map 1:1
    // onto source pixels (no scaling).
    const bool rows_in_vram_x =
        display_vram_left >= 0 &&
        display_vram_left + info.width <= static_cast<int>(psx::VRAM_WIDTH);
    const auto& rgb_lut = rgb555_to_rgba_lut();
    for (int y = 0; y < info.height; ++y) {
        const int src_y = y;
        int vram_y0 = display_vram_top + src_y;
        int vram_y1 = vram_y0;
        bool blend_fields = false;

        if (interlaced_field_output) {
            switch (deinterlace_mode) {
            case DeinterlaceMode::Bob:
                vram_y0 = display_vram_top + ((src_y >> 1) * 2) + field_parity;
                break;
            case DeinterlaceMode::Blend:
                vram_y0 = display_vram_top + ((src_y >> 1) * 2) + field_parity;
                vram_y1 = vram_y0 + (field_parity ? -1 : 1);
                blend_fields = true;
                break;
            case DeinterlaceMode::Weave:
            default:
                // Stable placement: map output lines directly to source scanlines.
                vram_y0 = display_vram_top + src_y;
                break;
            }
        }
        if (vram_y0 < 0 || vram_y0 >= static_cast<int>(psx::VRAM_HEIGHT)) {
            continue;
        }
        if (blend_fields &&
            (vram_y1 < 0 || vram_y1 >= static_cast<int>(psx::VRAM_HEIGHT))) {
            blend_fields = false;
            vram_y1 = vram_y0;
        }

        const size_t row_base =
            static_cast<size_t>(y) * static_cast<size_t>(info.width);
        if (!display_.is_24bit && !blend_fields && rows_in_vram_x) {
            // Same bytes as read_rgb() below, via the RGB555 LUT.
            const u16* src = vram_.data() +
                static_cast<size_t>(vram_y0) * psx::VRAM_WIDTH +
                static_cast<size_t>(display_vram_left);
            u32* dst = rgba != nullptr ? rgba->data() + row_base : nullptr;
            for (int x = 0; x < info.width; ++x) {
                const u32 pixel = rgb_lut[src[x] & 0x7FFFu];
                if (dst != nullptr) {
                    dst[x] = pixel;
                }
                if (include_stats) {
                    hash ^= pixel & 0xFFu;
                    hash *= 16777619u;
                    hash ^= (pixel >> 8) & 0xFFu;
                    hash *= 16777619u;
                    hash ^= (pixel >> 16) & 0xFFu;
                    hash *= 16777619u;
                    if ((pixel & 0x00FFFFFFu) != 0u) {
                        ++non_black;
                    }
                }
            }
            continue;
        }
        for (int x = 0; x < info.width; ++x) {
            u8 r = 0;
            u8 g = 0;
            u8 b = 0;
            const int src_x = x;
            if (!read_rgb(vram_y0, src_x, r, g, b)) {
                continue;
            }
            if (blend_fields) {
                u8 r1 = 0;
                u8 g1 = 0;
                u8 b1 = 0;
                if (read_rgb(vram_y1, src_x, r1, g1, b1)) {
                    r = static_cast<u8>((static_cast<u16>(r) + r1) / 2u);
                    g = static_cast<u8>((static_cast<u16>(g) + g1) / 2u);
                    b = static_cast<u8>((static_cast<u16>(b) + b1) / 2u);
                }
            }
            if (rgba != nullptr) {
                (*rgba)[row_base + static_cast<size_t>(x)] =
                    static_cast<u32>(r) | (static_cast<u32>(g) << 8) |
                    (static_cast<u32>(b) << 16) | 0xFF000000u;
            }
            if (include_stats) {
                hash ^= r;
                hash *= 16777619u;
                hash ^= g;
                hash *= 16777619u;
                hash ^= b;
                hash *= 16777619u;
                if ((r | g | b) != 0) {
                    ++non_black;
                }
            }
        }
    }

    if (include_stats) {
        info.non_black_pixels = non_black;
        info.hash = hash;
    }
    suppress_isolated_bottom_noise_row(rgba, info.width, info.height);
    return info;
}

DisplaySampleInfo Gpu::build_presented_display_rgba(std::vector<u32>* rgba,
    bool include_stats) const {
    if (!presented_display_valid_) {
        return build_display_rgba(rgba, include_stats);
    }

    if (rgba != nullptr) {
        *rgba = presented_display_rgba_;
    }

    if (!include_stats) {
        return presented_display_info_;
    }

    DisplaySampleInfo info = presented_display_info_;
    u32 hash = 2166136261u;
    u64 non_black = 0;
    for (u32 pixel : presented_display_rgba_) {
        const u8 r = static_cast<u8>(pixel & 0xFFu);
        const u8 g = static_cast<u8>((pixel >> 8) & 0xFFu);
        const u8 b = static_cast<u8>((pixel >> 16) & 0xFFu);
        hash ^= r;
        hash *= 16777619u;
        hash ^= g;
        hash *= 16777619u;
        hash ^= b;
        hash *= 16777619u;
        if ((r | g | b) != 0) {
            ++non_black;
        }
    }
    info.hash = hash;
    info.non_black_pixels = non_black;
    return info;
}

DisplayDebugInfo Gpu::debug_display_info() const {
    DisplayDebugInfo info{};
    info.mode_width = display_.width();
    info.mode_height = display_.height();
    info.x_start = static_cast<int>(display_.x_start);
    info.y_start = static_cast<int>(display_.y_start);
    info.x1 = static_cast<int>(display_.x1);
    info.x2 = static_cast<int>(display_.x2);
    info.y1 = static_cast<int>(display_.y1);
    info.y2 = static_cast<int>(display_.y2);
    info.is_24bit = display_.is_24bit;
    info.interlaced = display_.interlaced;
    const CrtcRect crtc = calculate_crtc_rect(display_);
    info.divisor = crtc.divisor;
    info.width = crtc.width;
    info.height = crtc.height;
    info.display_vram_width = crtc.vram_width;
    info.display_vram_height = crtc.vram_height;
    info.display_vram_left = crtc.vram_left;
    info.display_vram_top = crtc.vram_top;
    info.display_skip_x = crtc.skip_x;
    info.src_width = std::max(1, info.width);
    info.src_height = std::max(1, info.height);
    info.tex_window_mask_x = static_cast<int>(tex_window_mask_x_);
    info.tex_window_mask_y = static_cast<int>(tex_window_mask_y_);
    info.tex_window_off_x = static_cast<int>(tex_window_off_x_);
    info.tex_window_off_y = static_cast<int>(tex_window_off_y_);
    return info;
}

void Gpu::vblank() {
    static u64 vblank_count = 0;
    // Presentation needs pixels every VBlank, but display hashing/non-black
    // statistics are diagnostic work and are recomputed on demand.
    presented_display_info_ = build_display_rgba(presented_display_rgba_, false);
    hw_record_present();
    reaper_draws_prev_frame_ = reaper_draws_last_frame_;
    reaper_draws_last_frame_ = reaper_draws_this_frame_;
    reaper_draws_this_frame_ = 0;
    presented_display_valid_ = true;
    frame_complete_ = true;
    if (sys_ && fmv_diagnostics_enabled()) {
        sys_->debug_note_gpu_vblank();
    }
    interlace_field_ = !interlace_field_;
    if (g_trace_gpu &&
        trace_should_log(vblank_count, g_trace_burst_gpu, g_trace_stride_gpu)) {
        LOG_DEBUG("GPU: VBlank field=%d", interlace_field_ ? 1 : 0);
    }
}

// ── Rasterization ──────────────────────────────────────────────────

void Gpu::set_pixel(s16 x, s16 y, u16 color, bool semi_transparent) {
    if (x < draw_x_min_ || x > draw_x_max_)
        return;
    if (y < draw_y_min_ || y > draw_y_max_)
        return;
    if (x < 0 || x >= (s16)psx::VRAM_WIDTH)
        return;
    if (y < 0 || y >= (s16)psx::VRAM_HEIGHT)
        return;

    set_pixel_clipped(x, y, color, semi_transparent);
}

void Gpu::set_pixel_clipped(s16 x, s16 y, u16 color, bool semi_transparent) {
    // Caller guarantees x/y are already inside draw area and VRAM bounds.

    const size_t index = static_cast<size_t>(y) * psx::VRAM_WIDTH + x;
    const u16 dst = vram_[index];
    if (check_mask_before_draw_ && (dst & 0x8000u)) {
        return;
    }

    const u16 color15 = static_cast<u16>(color & 0x7FFFu);
    u16 out = static_cast<u16>(color & 0x8000u);
    if (semi_transparent) {
        const int fr = color15 & 0x1F;
        const int fg = (color15 >> 5) & 0x1F;
        const int fb = (color15 >> 10) & 0x1F;
        const int br = dst & 0x1F;
        const int bg = (dst >> 5) & 0x1F;
        const int bb = (dst >> 10) & 0x1F;
        int rr = fr, rg = fg, rb = fb;
        switch (semi_transparency_ & 0x3u) {
        case 0:
            rr = (br + fr) >> 1;
            rg = (bg + fg) >> 1;
            rb = (bb + fb) >> 1;
            break;
        case 1:
            rr = std::min(31, br + fr);
            rg = std::min(31, bg + fg);
            rb = std::min(31, bb + fb);
            break;
        case 2:
            rr = std::max(0, br - fr);
            rg = std::max(0, bg - fg);
            rb = std::max(0, bb - fb);
            break;
        case 3:
            rr = std::min(31, br + (fr >> 2));
            rg = std::min(31, bg + (fg >> 2));
            rb = std::min(31, bb + (fb >> 2));
            break;
        }
        out = static_cast<u16>(out | (rr & 0x1F) | ((rg & 0x1F) << 5) |
            ((rb & 0x1F) << 10));
    } else {
        out = static_cast<u16>(out | color15);
    }

    if (force_set_mask_bit_) {
        out |= 0x8000u;
    }
    vram_[index] = out;
}

void Gpu::write_pixel_opaque_clipped(s16 x, s16 y, u16 color) {
    const size_t index = static_cast<size_t>(y) * psx::VRAM_WIDTH + x;
    if (check_mask_before_draw_ && (vram_[index] & 0x8000u)) {
        return;
    }

    u16 out = color;
    if (force_set_mask_bit_) {
        out |= 0x8000u;
    }
    vram_[index] = out;
}

Gpu::TextureSampleState Gpu::prepare_texture_sample_state() const {
    TextureSampleState state{};
    const u8 mask_x = static_cast<u8>(tex_window_mask_x_ & 0xFFu);
    const u8 mask_y = static_cast<u8>(tex_window_mask_y_ & 0xFFu);
    const u8 off_x = static_cast<u8>(tex_window_off_x_ & 0xFFu);
    const u8 off_y = static_cast<u8>(tex_window_off_y_ & 0xFFu);

    state.keep_x = static_cast<u8>(~mask_x);
    state.keep_y = static_cast<u8>(~mask_y);
    state.replace_x = static_cast<u8>(off_x & mask_x);
    state.replace_y = static_cast<u8>(off_y & mask_y);
    state.depth = static_cast<u8>((texpage_ >> 7) & 0x3u);
    state.tex_base_x = static_cast<u16>((texpage_ & 0xFu) * 64u);
    state.tex_base_y = static_cast<u16>(((texpage_ >> 4) & 1u) * 256u);
    state.clut_x = static_cast<u16>((clut_ & 0x3Fu) * 16u);
    const u16 clut_y = static_cast<u16>((clut_ >> 6) & 0x1FFu);
    state.clut_row = static_cast<size_t>(clut_y) * psx::VRAM_WIDTH;
    return state;
}

u16 Gpu::read_texel(const TextureSampleState &state, u8 u, u8 v) const {
    const u8 uw = static_cast<u8>((u & state.keep_x) | state.replace_x);
    const u8 vw = static_cast<u8>((v & state.keep_y) | state.replace_y);
    const u16 ty = static_cast<u16>(
        (state.tex_base_y + vw) & (psx::VRAM_HEIGHT - 1));
    const size_t texture_row = static_cast<size_t>(ty) * psx::VRAM_WIDTH;

    switch (state.depth) {
    case 0: {
        const u16 word_x = static_cast<u16>(
            (state.tex_base_x + (uw >> 2)) & (psx::VRAM_WIDTH - 1));
        const u16 packed = vram_[texture_row + word_x];
        const u16 index =
            static_cast<u16>((packed >> ((uw & 3u) * 4u)) & 0xFu);
        const u16 cx = static_cast<u16>(
            (state.clut_x + index) & (psx::VRAM_WIDTH - 1));
        return vram_[state.clut_row + cx];
    }
    case 1: {
        const u16 word_x = static_cast<u16>(
            (state.tex_base_x + (uw >> 1)) & (psx::VRAM_WIDTH - 1));
        const u16 packed = vram_[texture_row + word_x];
        const u16 index =
            static_cast<u16>((packed >> ((uw & 1u) * 8u)) & 0xFFu);
        const u16 cx = static_cast<u16>(
            (state.clut_x + index) & (psx::VRAM_WIDTH - 1));
        return vram_[state.clut_row + cx];
    }
    case 2:
    case 3: {
        const u16 tx = static_cast<u16>(
            (state.tex_base_x + uw) & (psx::VRAM_WIDTH - 1));
        return vram_[texture_row + tx];
    }
    default:
        return 0;
    }
}

void Gpu::draw_flat_triangle(Vertex v0, Vertex v1, Vertex v2, Color c) {
    if (exceeds_primitive_limits(v0, v1, v2)) {
        return;
    }
    const u16 color15 = c.to_15bit();
    s32 area = edge(v0, v1, v2.x, v2.y);
    if (area == 0) {
        return;
    }
    if (area < 0) {
        std::swap(v1, v2);
        area = -area;
    }
    const bool edge0_top_left = is_top_left_edge(v1, v2);
    const bool edge1_top_left = is_top_left_edge(v2, v0);
    const bool edge2_top_left = is_top_left_edge(v0, v1);
    // Flat fill has no per-pixel interpolation to cheapen, so every mode
    // uses the exact span rasterizer.
    s16 min_x = std::min({ v0.x, v1.x, v2.x });
    s16 max_x = std::max({ v0.x, v1.x, v2.x });
    s16 min_y = std::min({ v0.y, v1.y, v2.y });
    s16 max_y = std::max({ v0.y, v1.y, v2.y });
    min_x = std::max(min_x, draw_x_min_);
    max_x = std::min(max_x, draw_x_max_);
    min_y = std::max(min_y, draw_y_min_);
    max_y = std::min(max_y, draw_y_max_);
    if (max_x - min_x > 1023 || max_y - min_y > 511) {
        return;
    }

    const bool opaque_path = !semi_transparency_mode_;
    const s32 step_w0_x = -(v2.y - v1.y);
    const s32 step_w0_y = (v2.x - v1.x);
    const s32 step_w1_x = -(v0.y - v2.y);
    const s32 step_w1_y = (v0.x - v2.x);
    const s32 step_w2_x = -(v1.y - v0.y);
    const s32 step_w2_y = (v1.x - v0.x);

    s32 w0_row = edge(v1, v2, min_x, min_y);
    s32 w1_row = edge(v2, v0, min_x, min_y);
    s32 w2_row = edge(v0, v1, min_x, min_y);

    for (s16 y = min_y; y <= max_y; ++y) {
        s16 span_min_x = 0;
        s16 span_max_x = -1;
        if (triangle_scanline_span(
                min_x, max_x, w0_row, w1_row, w2_row,
                step_w0_x, step_w1_x, step_w2_x,
                edge0_top_left, edge1_top_left, edge2_top_left,
                span_min_x, span_max_x)) {
            for (s16 x = span_min_x; x <= span_max_x; ++x) {
                if (opaque_path) {
                    write_pixel_opaque_clipped(x, y, color15);
                }
                else {
                    set_pixel_clipped(x, y, color15, true);
                }
            }
        }
        w0_row += step_w0_y;
        w1_row += step_w1_y;
        w2_row += step_w2_y;
    }
}

void Gpu::draw_shaded_triangle(Vertex v0, Vertex v1, Vertex v2) {
    if (exceeds_primitive_limits(v0, v1, v2)) {
        return;
    }
    s32 area = edge(v0, v1, v2.x, v2.y);
    if (area == 0) {
        return;
    }
    if (area < 0) {
        std::swap(v1, v2);
        area = -area;
    }
    const bool edge0_top_left = is_top_left_edge(v1, v2);
    const bool edge1_top_left = is_top_left_edge(v2, v0);
    const bool edge2_top_left = is_top_left_edge(v0, v1);
    if (!g_gpu_fast_mode) {
        s16 min_x = std::min({ v0.x, v1.x, v2.x });
        s16 max_x = std::max({ v0.x, v1.x, v2.x });
        s16 min_y = std::min({ v0.y, v1.y, v2.y });
        s16 max_y = std::max({ v0.y, v1.y, v2.y });
        min_x = std::max(min_x, draw_x_min_);
        max_x = std::min(max_x, draw_x_max_);
        min_y = std::max(min_y, draw_y_min_);
        max_y = std::min(max_y, draw_y_max_);
        if (max_x - min_x > 1023 || max_y - min_y > 511) {
            return;
        }

        const bool opaque_path = !semi_transparency_mode_;
        const s32 step_w0_x = -(v2.y - v1.y);
        const s32 step_w0_y = (v2.x - v1.x);
        const s32 step_w1_x = -(v0.y - v2.y);
        const s32 step_w1_y = (v0.x - v2.x);
        const s32 step_w2_x = -(v1.y - v0.y);
        const s32 step_w2_y = (v1.x - v0.x);

        s32 w0_row = edge(v1, v2, min_x, min_y);
        s32 w1_row = edge(v2, v0, min_x, min_y);
        s32 w2_row = edge(v0, v1, min_x, min_y);
        const s32 step_r_x = step_w0_x * static_cast<s32>(v0.color.r) +
            step_w1_x * static_cast<s32>(v1.color.r) +
            step_w2_x * static_cast<s32>(v2.color.r);
        const s32 step_r_y = step_w0_y * static_cast<s32>(v0.color.r) +
            step_w1_y * static_cast<s32>(v1.color.r) +
            step_w2_y * static_cast<s32>(v2.color.r);
        const s32 step_g_x = step_w0_x * static_cast<s32>(v0.color.g) +
            step_w1_x * static_cast<s32>(v1.color.g) +
            step_w2_x * static_cast<s32>(v2.color.g);
        const s32 step_g_y = step_w0_y * static_cast<s32>(v0.color.g) +
            step_w1_y * static_cast<s32>(v1.color.g) +
            step_w2_y * static_cast<s32>(v2.color.g);
        const s32 step_b_x = step_w0_x * static_cast<s32>(v0.color.b) +
            step_w1_x * static_cast<s32>(v1.color.b) +
            step_w2_x * static_cast<s32>(v2.color.b);
        const s32 step_b_y = step_w0_y * static_cast<s32>(v0.color.b) +
            step_w1_y * static_cast<s32>(v1.color.b) +
            step_w2_y * static_cast<s32>(v2.color.b);
        s32 r_row = w0_row * static_cast<s32>(v0.color.r) +
            w1_row * static_cast<s32>(v1.color.r) +
            w2_row * static_cast<s32>(v2.color.r);
        s32 g_row = w0_row * static_cast<s32>(v0.color.g) +
            w1_row * static_cast<s32>(v1.color.g) +
            w2_row * static_cast<s32>(v2.color.g);
        s32 b_row = w0_row * static_cast<s32>(v0.color.b) +
            w1_row * static_cast<s32>(v1.color.b) +
            w2_row * static_cast<s32>(v2.color.b);

        for (s16 y = min_y; y <= max_y; ++y) {
            s16 span_min_x = 0;
            s16 span_max_x = -1;
            if (triangle_scanline_span(
                    min_x, max_x, w0_row, w1_row, w2_row,
                    step_w0_x, step_w1_x, step_w2_x,
                    edge0_top_left, edge1_top_left, edge2_top_left,
                    span_min_x, span_max_x)) {
                const s32 span_dx =
                    static_cast<s32>(span_min_x) - static_cast<s32>(min_x);
                s32 r_num = r_row + step_r_x * span_dx;
                s32 g_num = g_row + step_g_x * span_dx;
                s32 b_num = b_row + step_b_x * span_dx;
                for (s16 x = span_min_x; x <= span_max_x; ++x) {
                    const u8 r = static_cast<u8>(clamp_u8_i(r_num / area));
                    const u8 g = static_cast<u8>(clamp_u8_i(g_num / area));
                    const u8 b = static_cast<u8>(clamp_u8_i(b_num / area));
                    const u16 out15 =
                        pack_rgb15_dithered(r, g, b, 0, x, y, dither_enabled_);
                    if (opaque_path) {
                        write_pixel_opaque_clipped(x, y, out15);
                    }
                    else {
                        set_pixel_clipped(x, y, out15, true);
                    }
                    r_num += step_r_x;
                    g_num += step_g_x;
                    b_num += step_b_x;
                }
            }
            w0_row += step_w0_y;
            w1_row += step_w1_y;
            w2_row += step_w2_y;
            r_row += step_r_y;
            g_row += step_g_y;
            b_row += step_b_y;
        }
        return;
    }
    if (g_gpu_extreme_fast_mode) {
        draw_flat_triangle(v0, v1, v2, average_color(v0.color, v1.color, v2.color));
        return;
    }

    const bool opaque_fast_path = !semi_transparency_mode_;
    s16 min_x = std::min({ v0.x, v1.x, v2.x });
    s16 max_x = std::max({ v0.x, v1.x, v2.x });
    s16 min_y = std::min({ v0.y, v1.y, v2.y });
    s16 max_y = std::max({ v0.y, v1.y, v2.y });
    min_x = std::max(min_x, draw_x_min_);
    max_x = std::min(max_x, draw_x_max_);
    min_y = std::max(min_y, draw_y_min_);
    max_y = std::min(max_y, draw_y_max_);
    if (max_x - min_x > 1023 || max_y - min_y > 511)
        return;

    const s32 step_w0_x = -(v2.y - v1.y);
    const s32 step_w0_y = (v2.x - v1.x);
    const s32 step_w1_x = -(v0.y - v2.y);
    const s32 step_w1_y = (v0.x - v2.x);
    const s32 step_w2_x = -(v1.y - v0.y);
    const s32 step_w2_y = (v1.x - v0.x);

    s32 w0_row = edge(v1, v2, min_x, min_y);
    s32 w1_row = edge(v2, v0, min_x, min_y);
    s32 w2_row = edge(v0, v1, min_x, min_y);

    // Fast mode: exact coverage spans, but colors stepped in float instead of
    // three integer divides per pixel.
    const float inv_area = 1.0f / static_cast<float>(area);
    const auto color_num = [&](s32 a, s32 b, s32 c, u8 Color::*channel) {
        return a * static_cast<s32>(v0.color.*channel) +
            b * static_cast<s32>(v1.color.*channel) +
            c * static_cast<s32>(v2.color.*channel);
    };
    const float dr_dx =
        static_cast<float>(color_num(step_w0_x, step_w1_x, step_w2_x, &Color::r)) * inv_area;
    const float dg_dx =
        static_cast<float>(color_num(step_w0_x, step_w1_x, step_w2_x, &Color::g)) * inv_area;
    const float db_dx =
        static_cast<float>(color_num(step_w0_x, step_w1_x, step_w2_x, &Color::b)) * inv_area;

    for (s16 y = min_y; y <= max_y; ++y) {
        s16 span_min_x = 0;
        s16 span_max_x = -1;
        if (triangle_scanline_span(
                min_x, max_x, w0_row, w1_row, w2_row,
                step_w0_x, step_w1_x, step_w2_x,
                edge0_top_left, edge1_top_left, edge2_top_left,
                span_min_x, span_max_x)) {
            const s32 span_dx =
                static_cast<s32>(span_min_x) - static_cast<s32>(min_x);
            const s32 w0 = w0_row + step_w0_x * span_dx;
            const s32 w1 = w1_row + step_w1_x * span_dx;
            const s32 w2 = w2_row + step_w2_x * span_dx;
            float r_value = static_cast<float>(color_num(w0, w1, w2, &Color::r)) * inv_area;
            float g_value = static_cast<float>(color_num(w0, w1, w2, &Color::g)) * inv_area;
            float b_value = static_cast<float>(color_num(w0, w1, w2, &Color::b)) * inv_area;
            for (s16 x = span_min_x; x <= span_max_x; ++x) {
                const u8 r = static_cast<u8>(clamp_u8_i(static_cast<int>(r_value)));
                const u8 g = static_cast<u8>(clamp_u8_i(static_cast<int>(g_value)));
                const u8 b = static_cast<u8>(clamp_u8_i(static_cast<int>(b_value)));
                const u16 out15 = pack_rgb15_dithered(r, g, b, 0, x, y, dither_enabled_);
                if (opaque_fast_path) {
                    write_pixel_opaque_clipped(x, y, out15);
                }
                else {
                    set_pixel_clipped(x, y, out15, true);
                }
                r_value += dr_dx;
                g_value += dg_dx;
                b_value += db_dx;
            }
        }
        w0_row += step_w0_y;
        w1_row += step_w1_y;
        w2_row += step_w2_y;
    }
}

void Gpu::draw_textured_triangle(Vertex v0, Vertex v1, Vertex v2, Color /*c*/) {
    if (exceeds_primitive_limits(v0, v1, v2)) {
        return;
    }
    if (g_pgxp_enabled && v0.has_w && v1.has_w && v2.has_w &&
        draw_textured_triangle_pgxp(v0, v1, v2, false)) {
        return;
    }
    s32 area = edge(v0, v1, v2.x, v2.y);
    if (area == 0) {
        return;
    }
    if (area < 0) {
        std::swap(v1, v2);
        area = -area;
    }
    const bool edge0_top_left = is_top_left_edge(v1, v2);
    const bool edge1_top_left = is_top_left_edge(v2, v0);
    const bool edge2_top_left = is_top_left_edge(v0, v1);
    const TextureSampleState texture = prepare_texture_sample_state();
    const bool profile_raster = g_profile_detailed_timing && sys_ != nullptr;
    u64 profile_candidates = 0;
    u64 profile_covered = 0;
    u64 profile_transparent = 0;
    u64 profile_semi = 0;
    if (!g_gpu_fast_mode) {
        s16 min_x = std::min({ v0.x, v1.x, v2.x });
        s16 max_x = std::max({ v0.x, v1.x, v2.x });
        s16 min_y = std::min({ v0.y, v1.y, v2.y });
        s16 max_y = std::max({ v0.y, v1.y, v2.y });
        min_x = std::max(min_x, draw_x_min_);
        max_x = std::min(max_x, draw_x_max_);
        min_y = std::max(min_y, draw_y_min_);
        max_y = std::min(max_y, draw_y_max_);
        if (max_x - min_x > 1023 || max_y - min_y > 511) {
            return;
        }
        if (profile_raster && min_x <= max_x && min_y <= max_y) {
            profile_candidates +=
                static_cast<u64>(static_cast<int>(max_x) - min_x + 1) *
                static_cast<u64>(static_cast<int>(max_y) - min_y + 1);
        }

        const bool raw_texture = (gp0_command_ & 0x1u) != 0;
        const u8 mr = v0.color.r;
        const u8 mg = v0.color.g;
        const u8 mb = v0.color.b;
        const s32 step_w0_x = -(v2.y - v1.y);
        const s32 step_w0_y = (v2.x - v1.x);
        const s32 step_w1_x = -(v0.y - v2.y);
        const s32 step_w1_y = (v0.x - v2.x);
        const s32 step_w2_x = -(v1.y - v0.y);
        const s32 step_w2_y = (v1.x - v0.x);

        s32 w0_row = edge(v1, v2, min_x, min_y);
        s32 w1_row = edge(v2, v0, min_x, min_y);
        s32 w2_row = edge(v0, v1, min_x, min_y);
        const s32 step_u_x = step_w0_x * static_cast<s32>(v0.u) +
            step_w1_x * static_cast<s32>(v1.u) +
            step_w2_x * static_cast<s32>(v2.u);
        const s32 step_u_y = step_w0_y * static_cast<s32>(v0.u) +
            step_w1_y * static_cast<s32>(v1.u) +
            step_w2_y * static_cast<s32>(v2.u);
        const s32 step_v_x = step_w0_x * static_cast<s32>(v0.v) +
            step_w1_x * static_cast<s32>(v1.v) +
            step_w2_x * static_cast<s32>(v2.v);
        const s32 step_v_y = step_w0_y * static_cast<s32>(v0.v) +
            step_w1_y * static_cast<s32>(v1.v) +
            step_w2_y * static_cast<s32>(v2.v);
        s32 u_row = w0_row * static_cast<s32>(v0.u) +
            w1_row * static_cast<s32>(v1.u) +
            w2_row * static_cast<s32>(v2.u);
        s32 v_row = w0_row * static_cast<s32>(v0.v) +
            w1_row * static_cast<s32>(v1.v) +
            w2_row * static_cast<s32>(v2.v);

        for (s16 y = min_y; y <= max_y; ++y) {
            s16 span_min_x = 0;
            s16 span_max_x = -1;
            if (triangle_scanline_span(
                    min_x, max_x, w0_row, w1_row, w2_row,
                    step_w0_x, step_w1_x, step_w2_x,
                    edge0_top_left, edge1_top_left, edge2_top_left,
                    span_min_x, span_max_x)) {
                const s32 span_dx =
                    static_cast<s32>(span_min_x) - static_cast<s32>(min_x);
                const u64 span_pixels = static_cast<u64>(
                    static_cast<int>(span_max_x) -
                    static_cast<int>(span_min_x) + 1);
                if (profile_raster) {
                    profile_covered += span_pixels;
                }

                s32 u_num = u_row + step_u_x * span_dx;
                s32 v_num = v_row + step_v_x * span_dx;
                for (s16 x = span_min_x; x <= span_max_x; ++x) {
                    const u8 u = static_cast<u8>(u_num / area);
                    const u8 v_coord = static_cast<u8>(v_num / area);
                    const u16 texel = read_texel(texture, u, v_coord);
                    if (profile_raster && texel == 0) {
                        ++profile_transparent;
                    }
                    if (texel != 0) {
                        u16 out15 = texel;
                        if (!raw_texture) {
                            if (dither_enabled_) {
                                out15 =
                                    modulate_texel_dithered_15bit(
                                        texel, mr, mg, mb, x, y);
                            }
                            else {
                                out15 =
                                    modulate_texel_15bit(texel, mr, mg, mb);
                            }
                        }
                        const bool texel_semi =
                            semi_transparency_mode_ &&
                            ((texel & 0x8000u) != 0);
                        if (profile_raster && texel_semi) {
                            ++profile_semi;
                        }
                        if (texel_semi) {
                            set_pixel_clipped(x, y, out15, true);
                        }
                        else {
                            write_pixel_opaque_clipped(x, y, out15);
                        }
                    }
                    u_num += step_u_x;
                    v_num += step_v_x;
                }
            }
            w0_row += step_w0_y;
            w1_row += step_w1_y;
            w2_row += step_w2_y;
            u_row += step_u_y;
            v_row += step_v_y;
        }
        if (profile_raster) {
            sys_->add_gpu_raster_work(
                profile_candidates, profile_covered, profile_covered,
                texture.depth, profile_transparent, profile_semi);
        }
        return;
    }
    s16 min_x = std::min({ v0.x, v1.x, v2.x });
    s16 max_x = std::max({ v0.x, v1.x, v2.x });
    s16 min_y = std::min({ v0.y, v1.y, v2.y });
    s16 max_y = std::max({ v0.y, v1.y, v2.y });
    min_x = std::max(min_x, draw_x_min_);
    max_x = std::min(max_x, draw_x_max_);
    min_y = std::max(min_y, draw_y_min_);
    max_y = std::min(max_y, draw_y_max_);
    if (max_x - min_x > 1023 || max_y - min_y > 511)
        return;
    if (profile_raster && min_x <= max_x && min_y <= max_y) {
        profile_candidates +=
            static_cast<u64>(static_cast<int>(max_x) - min_x + 1) *
            static_cast<u64>(static_cast<int>(max_y) - min_y + 1);
    }

    const bool raw_texture = (gp0_command_ & 0x1u) != 0;
    const u8 mr = v0.color.r;
    const u8 mg = v0.color.g;
    const u8 mb = v0.color.b;
    const bool opaque_fast_path = g_gpu_extreme_fast_mode || !semi_transparency_mode_;
    const float inv_area = 1.0f / static_cast<float>(area);

    const s32 step_w0_x = -(v2.y - v1.y);
    const s32 step_w0_y = (v2.x - v1.x);
    const s32 step_w1_x = -(v0.y - v2.y);
    const s32 step_w1_y = (v0.x - v2.x);
    const s32 step_w2_x = -(v1.y - v0.y);
    const s32 step_w2_y = (v1.x - v0.x);

    s32 w0_row = edge(v1, v2, min_x, min_y);
    s32 w1_row = edge(v2, v0, min_x, min_y);
    s32 w2_row = edge(v0, v1, min_x, min_y);
    const s32 step_u_x_num =
        step_w0_x * static_cast<s32>(v0.u) + step_w1_x * static_cast<s32>(v1.u) +
        step_w2_x * static_cast<s32>(v2.u);
    const s32 step_u_y_num =
        step_w0_y * static_cast<s32>(v0.u) + step_w1_y * static_cast<s32>(v1.u) +
        step_w2_y * static_cast<s32>(v2.u);
    const s32 step_v_x_num =
        step_w0_x * static_cast<s32>(v0.v) + step_w1_x * static_cast<s32>(v1.v) +
        step_w2_x * static_cast<s32>(v2.v);
    const s32 step_v_y_num =
        step_w0_y * static_cast<s32>(v0.v) + step_w1_y * static_cast<s32>(v1.v) +
        step_w2_y * static_cast<s32>(v2.v);
    s32 u_row_num = w0_row * static_cast<s32>(v0.u) +
        w1_row * static_cast<s32>(v1.u) +
        w2_row * static_cast<s32>(v2.u);
    s32 v_row_num = w0_row * static_cast<s32>(v0.v) +
        w1_row * static_cast<s32>(v1.v) +
        w2_row * static_cast<s32>(v2.v);

    const float du_dx = static_cast<float>(step_u_x_num) * inv_area;
    const float dv_dx = static_cast<float>(step_v_x_num) * inv_area;
    for (s16 y = min_y; y <= max_y; ++y) {
        s16 span_min_x = 0;
        s16 span_max_x = -1;
        if (triangle_scanline_span(
                min_x, max_x, w0_row, w1_row, w2_row,
                step_w0_x, step_w1_x, step_w2_x,
                edge0_top_left, edge1_top_left, edge2_top_left,
                span_min_x, span_max_x)) {
            const s32 span_dx =
                static_cast<s32>(span_min_x) - static_cast<s32>(min_x);
            if (profile_raster) {
                profile_covered += static_cast<u64>(
                    static_cast<int>(span_max_x) -
                    static_cast<int>(span_min_x) + 1);
            }
            float u_value =
                static_cast<float>(u_row_num + step_u_x_num * span_dx) * inv_area;
            float v_value =
                static_cast<float>(v_row_num + step_v_x_num * span_dx) * inv_area;
            for (s16 x = span_min_x; x <= span_max_x; ++x) {
                const u8 u =
                    static_cast<u8>(static_cast<s32>(u_value) & 0xFF);
                const u8 v_coord =
                    static_cast<u8>(static_cast<s32>(v_value) & 0xFF);
                const u16 texel = read_texel(texture, u, v_coord);
                if (profile_raster && texel == 0) {
                    ++profile_transparent;
                }
                if (texel != 0) {
                    u16 out15 = texel;
                    if (!raw_texture) {
                        if (!g_gpu_extreme_fast_mode && dither_enabled_) {
                            out15 = modulate_texel_dithered_15bit(texel, mr, mg, mb, x, y);
                        }
                        else {
                            out15 = modulate_texel_15bit(texel, mr, mg, mb);
                        }
                    }
                    if (opaque_fast_path) {
                        write_pixel_opaque_clipped(x, y, out15);
                    }
                    else {
                        const bool texel_semi = (texel & 0x8000u) != 0;
                        if (profile_raster && texel_semi) {
                            ++profile_semi;
                        }
                        set_pixel_clipped(x, y, out15, texel_semi);
                    }
                }
                u_value += du_dx;
                v_value += dv_dx;
            }
        }
        w0_row += step_w0_y;
        w1_row += step_w1_y;
        w2_row += step_w2_y;
        u_row_num += step_u_y_num;
        v_row_num += step_v_y_num;
    }
    if (profile_raster) {
        sys_->add_gpu_raster_work(
            profile_candidates, profile_covered, profile_covered,
            texture.depth, profile_transparent, profile_semi);
    }
}

void Gpu::draw_shaded_textured_triangle(Vertex v0, Vertex v1, Vertex v2) {
    if (exceeds_primitive_limits(v0, v1, v2)) {
        return;
    }
    if (g_pgxp_enabled && v0.has_w && v1.has_w && v2.has_w &&
        !g_gpu_extreme_fast_mode &&
        draw_textured_triangle_pgxp(v0, v1, v2, true)) {
        return;
    }
    s32 area = edge(v0, v1, v2.x, v2.y);
    if (area == 0) {
        return;
    }
    if (area < 0) {
        std::swap(v1, v2);
        area = -area;
    }
    const bool edge0_top_left = is_top_left_edge(v1, v2);
    const bool edge1_top_left = is_top_left_edge(v2, v0);
    const bool edge2_top_left = is_top_left_edge(v0, v1);
    const TextureSampleState texture = prepare_texture_sample_state();
    const bool profile_raster = g_profile_detailed_timing && sys_ != nullptr;
    u64 profile_candidates = 0;
    u64 profile_covered = 0;
    u64 profile_transparent = 0;
    u64 profile_semi = 0;
    if (!g_gpu_fast_mode) {
        s16 min_x = std::min({ v0.x, v1.x, v2.x });
        s16 max_x = std::max({ v0.x, v1.x, v2.x });
        s16 min_y = std::min({ v0.y, v1.y, v2.y });
        s16 max_y = std::max({ v0.y, v1.y, v2.y });
        min_x = std::max(min_x, draw_x_min_);
        max_x = std::min(max_x, draw_x_max_);
        min_y = std::max(min_y, draw_y_min_);
        max_y = std::min(max_y, draw_y_max_);
        if (max_x - min_x > 1023 || max_y - min_y > 511) {
            return;
        }
        if (profile_raster && min_x <= max_x && min_y <= max_y) {
            profile_candidates +=
                static_cast<u64>(static_cast<int>(max_x) - min_x + 1) *
                static_cast<u64>(static_cast<int>(max_y) - min_y + 1);
        }

        const bool raw_texture = (gp0_command_ & 0x1u) != 0;
        const s32 step_w0_x = -(v2.y - v1.y);
        const s32 step_w0_y = (v2.x - v1.x);
        const s32 step_w1_x = -(v0.y - v2.y);
        const s32 step_w1_y = (v0.x - v2.x);
        const s32 step_w2_x = -(v1.y - v0.y);
        const s32 step_w2_y = (v1.x - v0.x);

        s32 w0_row = edge(v1, v2, min_x, min_y);
        s32 w1_row = edge(v2, v0, min_x, min_y);
        s32 w2_row = edge(v0, v1, min_x, min_y);
        const s32 step_u_x = step_w0_x * static_cast<s32>(v0.u) +
            step_w1_x * static_cast<s32>(v1.u) +
            step_w2_x * static_cast<s32>(v2.u);
        const s32 step_u_y = step_w0_y * static_cast<s32>(v0.u) +
            step_w1_y * static_cast<s32>(v1.u) +
            step_w2_y * static_cast<s32>(v2.u);
        const s32 step_v_x = step_w0_x * static_cast<s32>(v0.v) +
            step_w1_x * static_cast<s32>(v1.v) +
            step_w2_x * static_cast<s32>(v2.v);
        const s32 step_v_y = step_w0_y * static_cast<s32>(v0.v) +
            step_w1_y * static_cast<s32>(v1.v) +
            step_w2_y * static_cast<s32>(v2.v);
        const s32 step_r_x = step_w0_x * static_cast<s32>(v0.color.r) +
            step_w1_x * static_cast<s32>(v1.color.r) +
            step_w2_x * static_cast<s32>(v2.color.r);
        const s32 step_r_y = step_w0_y * static_cast<s32>(v0.color.r) +
            step_w1_y * static_cast<s32>(v1.color.r) +
            step_w2_y * static_cast<s32>(v2.color.r);
        const s32 step_g_x = step_w0_x * static_cast<s32>(v0.color.g) +
            step_w1_x * static_cast<s32>(v1.color.g) +
            step_w2_x * static_cast<s32>(v2.color.g);
        const s32 step_g_y = step_w0_y * static_cast<s32>(v0.color.g) +
            step_w1_y * static_cast<s32>(v1.color.g) +
            step_w2_y * static_cast<s32>(v2.color.g);
        const s32 step_b_x = step_w0_x * static_cast<s32>(v0.color.b) +
            step_w1_x * static_cast<s32>(v1.color.b) +
            step_w2_x * static_cast<s32>(v2.color.b);
        const s32 step_b_y = step_w0_y * static_cast<s32>(v0.color.b) +
            step_w1_y * static_cast<s32>(v1.color.b) +
            step_w2_y * static_cast<s32>(v2.color.b);

        s32 u_row = w0_row * static_cast<s32>(v0.u) +
            w1_row * static_cast<s32>(v1.u) +
            w2_row * static_cast<s32>(v2.u);
        s32 v_row = w0_row * static_cast<s32>(v0.v) +
            w1_row * static_cast<s32>(v1.v) +
            w2_row * static_cast<s32>(v2.v);
        s32 r_row = w0_row * static_cast<s32>(v0.color.r) +
            w1_row * static_cast<s32>(v1.color.r) +
            w2_row * static_cast<s32>(v2.color.r);
        s32 g_row = w0_row * static_cast<s32>(v0.color.g) +
            w1_row * static_cast<s32>(v1.color.g) +
            w2_row * static_cast<s32>(v2.color.g);
        s32 b_row = w0_row * static_cast<s32>(v0.color.b) +
            w1_row * static_cast<s32>(v1.color.b) +
            w2_row * static_cast<s32>(v2.color.b);

        for (s16 y = min_y; y <= max_y; ++y) {
            s16 span_min_x = 0;
            s16 span_max_x = -1;
            if (triangle_scanline_span(
                    min_x, max_x, w0_row, w1_row, w2_row,
                    step_w0_x, step_w1_x, step_w2_x,
                    edge0_top_left, edge1_top_left, edge2_top_left,
                    span_min_x, span_max_x)) {
                const s32 span_dx =
                    static_cast<s32>(span_min_x) - static_cast<s32>(min_x);
                const u64 span_pixels = static_cast<u64>(
                    static_cast<int>(span_max_x) -
                    static_cast<int>(span_min_x) + 1);
                if (profile_raster) {
                    profile_covered += span_pixels;
                }

                s32 u_num = u_row + step_u_x * span_dx;
                s32 v_num = v_row + step_v_x * span_dx;
                s32 r_num = r_row + step_r_x * span_dx;
                s32 g_num = g_row + step_g_x * span_dx;
                s32 b_num = b_row + step_b_x * span_dx;
                for (s16 x = span_min_x; x <= span_max_x; ++x) {
                    const u8 u = static_cast<u8>(u_num / area);
                    const u8 v_coord = static_cast<u8>(v_num / area);
                    const u16 texel = read_texel(texture, u, v_coord);
                    if (profile_raster && texel == 0) {
                        ++profile_transparent;
                    }
                    if (texel != 0) {
                        const u8 mr =
                            static_cast<u8>(clamp_u8_i(r_num / area));
                        const u8 mg =
                            static_cast<u8>(clamp_u8_i(g_num / area));
                        const u8 mb =
                            static_cast<u8>(clamp_u8_i(b_num / area));

                        u16 out15 = texel;
                        if (!raw_texture) {
                            if (dither_enabled_) {
                                out15 = modulate_texel_dithered_15bit(
                                    texel, mr, mg, mb, x, y);
                            }
                            else {
                                out15 =
                                    modulate_texel_15bit(texel, mr, mg, mb);
                            }
                        }
                        const bool texel_semi =
                            semi_transparency_mode_ &&
                            ((texel & 0x8000u) != 0);
                        if (profile_raster && texel_semi) {
                            ++profile_semi;
                        }
                        if (texel_semi) {
                            set_pixel_clipped(x, y, out15, true);
                        }
                        else {
                            write_pixel_opaque_clipped(x, y, out15);
                        }
                    }
                    u_num += step_u_x;
                    v_num += step_v_x;
                    r_num += step_r_x;
                    g_num += step_g_x;
                    b_num += step_b_x;
                }
            }
            w0_row += step_w0_y;
            w1_row += step_w1_y;
            w2_row += step_w2_y;
            u_row += step_u_y;
            v_row += step_v_y;
            r_row += step_r_y;
            g_row += step_g_y;
            b_row += step_b_y;
        }
        if (profile_raster) {
            sys_->add_gpu_raster_work(
                profile_candidates, profile_covered, profile_covered,
                texture.depth, profile_transparent, profile_semi);
        }
        return;
    }
    if (g_gpu_extreme_fast_mode) {
        const Color flat_color = average_color(v0.color, v1.color, v2.color);
        v0.color = flat_color;
        v1.color = flat_color;
        v2.color = flat_color;
        draw_textured_triangle(v0, v1, v2, flat_color);
        return;
    }

    s16 min_x = std::min({ v0.x, v1.x, v2.x });
    s16 max_x = std::max({ v0.x, v1.x, v2.x });
    s16 min_y = std::min({ v0.y, v1.y, v2.y });
    s16 max_y = std::max({ v0.y, v1.y, v2.y });
    min_x = std::max(min_x, draw_x_min_);
    max_x = std::min(max_x, draw_x_max_);
    min_y = std::max(min_y, draw_y_min_);
    max_y = std::min(max_y, draw_y_max_);
    if (max_x - min_x > 1023 || max_y - min_y > 511)
        return;
    if (profile_raster && min_x <= max_x && min_y <= max_y) {
        profile_candidates +=
            static_cast<u64>(static_cast<int>(max_x) - min_x + 1) *
            static_cast<u64>(static_cast<int>(max_y) - min_y + 1);
    }

    const bool raw_texture = (gp0_command_ & 0x1u) != 0;
    const bool opaque_fast_path = !semi_transparency_mode_;
    const float inv_area = 1.0f / static_cast<float>(area);

    const s32 step_w0_x = -(v2.y - v1.y);
    const s32 step_w0_y = (v2.x - v1.x);
    const s32 step_w1_x = -(v0.y - v2.y);
    const s32 step_w1_y = (v0.x - v2.x);
    const s32 step_w2_x = -(v1.y - v0.y);
    const s32 step_w2_y = (v1.x - v0.x);

    s32 w0_row = edge(v1, v2, min_x, min_y);
    s32 w1_row = edge(v2, v0, min_x, min_y);
    s32 w2_row = edge(v0, v1, min_x, min_y);
    const s32 step_u_x_num =
        step_w0_x * static_cast<s32>(v0.u) + step_w1_x * static_cast<s32>(v1.u) +
        step_w2_x * static_cast<s32>(v2.u);
    const s32 step_u_y_num =
        step_w0_y * static_cast<s32>(v0.u) + step_w1_y * static_cast<s32>(v1.u) +
        step_w2_y * static_cast<s32>(v2.u);
    const s32 step_v_x_num =
        step_w0_x * static_cast<s32>(v0.v) + step_w1_x * static_cast<s32>(v1.v) +
        step_w2_x * static_cast<s32>(v2.v);
    const s32 step_v_y_num =
        step_w0_y * static_cast<s32>(v0.v) + step_w1_y * static_cast<s32>(v1.v) +
        step_w2_y * static_cast<s32>(v2.v);
    const s32 step_r_x_num =
        step_w0_x * static_cast<s32>(v0.color.r) +
        step_w1_x * static_cast<s32>(v1.color.r) +
        step_w2_x * static_cast<s32>(v2.color.r);
    const s32 step_r_y_num =
        step_w0_y * static_cast<s32>(v0.color.r) +
        step_w1_y * static_cast<s32>(v1.color.r) +
        step_w2_y * static_cast<s32>(v2.color.r);
    const s32 step_g_x_num =
        step_w0_x * static_cast<s32>(v0.color.g) +
        step_w1_x * static_cast<s32>(v1.color.g) +
        step_w2_x * static_cast<s32>(v2.color.g);
    const s32 step_g_y_num =
        step_w0_y * static_cast<s32>(v0.color.g) +
        step_w1_y * static_cast<s32>(v1.color.g) +
        step_w2_y * static_cast<s32>(v2.color.g);
    const s32 step_b_x_num =
        step_w0_x * static_cast<s32>(v0.color.b) +
        step_w1_x * static_cast<s32>(v1.color.b) +
        step_w2_x * static_cast<s32>(v2.color.b);
    const s32 step_b_y_num =
        step_w0_y * static_cast<s32>(v0.color.b) +
        step_w1_y * static_cast<s32>(v1.color.b) +
        step_w2_y * static_cast<s32>(v2.color.b);
    s32 u_row_num = w0_row * static_cast<s32>(v0.u) +
        w1_row * static_cast<s32>(v1.u) +
        w2_row * static_cast<s32>(v2.u);
    s32 v_row_num = w0_row * static_cast<s32>(v0.v) +
        w1_row * static_cast<s32>(v1.v) +
        w2_row * static_cast<s32>(v2.v);
    s32 r_row_num = w0_row * static_cast<s32>(v0.color.r) +
        w1_row * static_cast<s32>(v1.color.r) +
        w2_row * static_cast<s32>(v2.color.r);
    s32 g_row_num = w0_row * static_cast<s32>(v0.color.g) +
        w1_row * static_cast<s32>(v1.color.g) +
        w2_row * static_cast<s32>(v2.color.g);
    s32 b_row_num = w0_row * static_cast<s32>(v0.color.b) +
        w1_row * static_cast<s32>(v1.color.b) +
        w2_row * static_cast<s32>(v2.color.b);

    const float du_dx = static_cast<float>(step_u_x_num) * inv_area;
    const float dv_dx = static_cast<float>(step_v_x_num) * inv_area;
    const float dr_dx = static_cast<float>(step_r_x_num) * inv_area;
    const float dg_dx = static_cast<float>(step_g_x_num) * inv_area;
    const float db_dx = static_cast<float>(step_b_x_num) * inv_area;
    for (s16 y = min_y; y <= max_y; ++y) {
        s16 span_min_x = 0;
        s16 span_max_x = -1;
        if (triangle_scanline_span(
                min_x, max_x, w0_row, w1_row, w2_row,
                step_w0_x, step_w1_x, step_w2_x,
                edge0_top_left, edge1_top_left, edge2_top_left,
                span_min_x, span_max_x)) {
            const s32 span_dx =
                static_cast<s32>(span_min_x) - static_cast<s32>(min_x);
            if (profile_raster) {
                profile_covered += static_cast<u64>(
                    static_cast<int>(span_max_x) -
                    static_cast<int>(span_min_x) + 1);
            }
            float u_value =
                static_cast<float>(u_row_num + step_u_x_num * span_dx) * inv_area;
            float v_value =
                static_cast<float>(v_row_num + step_v_x_num * span_dx) * inv_area;
            float r_value =
                static_cast<float>(r_row_num + step_r_x_num * span_dx) * inv_area;
            float g_value =
                static_cast<float>(g_row_num + step_g_x_num * span_dx) * inv_area;
            float b_value =
                static_cast<float>(b_row_num + step_b_x_num * span_dx) * inv_area;
            for (s16 x = span_min_x; x <= span_max_x; ++x) {
                const u8 u =
                    static_cast<u8>(static_cast<s32>(u_value) & 0xFF);
                const u8 v_coord =
                    static_cast<u8>(static_cast<s32>(v_value) & 0xFF);
                const u16 texel = read_texel(texture, u, v_coord);
                if (profile_raster && texel == 0) {
                    ++profile_transparent;
                }
                if (texel != 0) {
                    const u8 mr = static_cast<u8>(std::clamp(static_cast<int>(r_value), 0, 255));
                    const u8 mg = static_cast<u8>(std::clamp(static_cast<int>(g_value), 0, 255));
                    const u8 mb = static_cast<u8>(std::clamp(static_cast<int>(b_value), 0, 255));

                    u16 out15 = texel;
                    if (!raw_texture) {
                        if (dither_enabled_) {
                            out15 = modulate_texel_dithered_15bit(texel, mr, mg, mb, x, y);
                        }
                        else {
                            out15 = modulate_texel_15bit(texel, mr, mg, mb);
                        }
                    }
                    if (opaque_fast_path) {
                        write_pixel_opaque_clipped(x, y, out15);
                    }
                    else {
                        const bool texel_semi = (texel & 0x8000u) != 0;
                        if (profile_raster && texel_semi) {
                            ++profile_semi;
                        }
                        set_pixel_clipped(x, y, out15, texel_semi);
                    }
                }
                u_value += du_dx;
                v_value += dv_dx;
                r_value += dr_dx;
                g_value += dg_dx;
                b_value += db_dx;
            }
        }
        w0_row += step_w0_y;
        w1_row += step_w1_y;
        w2_row += step_w2_y;
        u_row_num += step_u_y_num;
        v_row_num += step_v_y_num;
        r_row_num += step_r_y_num;
        g_row_num += step_g_y_num;
        b_row_num += step_b_y_num;
    }
    if (profile_raster) {
        sys_->add_gpu_raster_work(
            profile_candidates, profile_covered, profile_covered,
            texture.depth, profile_transparent, profile_semi);
    }
}

bool Gpu::draw_textured_triangle_pgxp(const Vertex &p0, const Vertex &p1,
                                      const Vertex &p2, bool gouraud) {
    // Attribute setup uses the sub-pixel PGXP positions. Barycentric weight
    // i is a linear function of the pixel position: l_i(x, y) =
    // edge(P_{i+1}, P_{i+2}, (x, y)) / area.
    struct Linear {
        double dx = 0.0, dy = 0.0, c = 0.0;
        double at(double x, double y) const { return dx * x + dy * y + c; }
        Linear scaled(double s) const { return { dx * s, dy * s, c * s }; }
        Linear plus(const Linear &o) const {
            return { dx + o.dx, dy + o.dy, c + o.c };
        }
    };
    const Vertex *pv[3] = { &p0, &p1, &p2 };
    const double area =
        (static_cast<double>(p1.fx) - p0.fx) * (static_cast<double>(p2.fy) - p0.fy) -
        (static_cast<double>(p1.fy) - p0.fy) * (static_cast<double>(p2.fx) - p0.fx);
    if (std::abs(area) < 0.5) {
        return false; // too thin for sub-pixel setup to be trustworthy
    }
    Linear bary[3];
    for (int i = 0; i < 3; ++i) {
        const Vertex &a = *pv[(i + 1) % 3];
        const Vertex &b = *pv[(i + 2) % 3];
        if (!(pv[i]->w > 0.0f)) {
            return false;
        }
        const double ex = static_cast<double>(b.fx) - a.fx;
        const double ey = static_cast<double>(b.fy) - a.fy;
        // edge(a, b, p) = ex * (p.y - a.y) - ey * (p.x - a.x)
        bary[i] = Linear{ -ey / area, ex / area,
                          (ey * a.fx - ex * a.fy) / area };
    }

    // Perspective-correct texture coordinates: interpolate attr/w and 1/w,
    // then divide per pixel. Gouraud colour stays affine, as on hardware.
    Linear inv_w{}, u_over_w{}, v_over_w{}, red{}, green{}, blue{};
    u8 u_min = 255, u_max = 0, v_min = 255, v_max = 0;
    for (int i = 0; i < 3; ++i) {
        const Vertex &vx = *pv[i];
        const double rw = 1.0 / static_cast<double>(vx.w);
        inv_w = inv_w.plus(bary[i].scaled(rw));
        u_over_w = u_over_w.plus(bary[i].scaled(rw * vx.u));
        v_over_w = v_over_w.plus(bary[i].scaled(rw * vx.v));
        red = red.plus(bary[i].scaled(vx.color.r));
        green = green.plus(bary[i].scaled(vx.color.g));
        blue = blue.plus(bary[i].scaled(vx.color.b));
        u_min = std::min(u_min, vx.u);
        u_max = std::max(u_max, vx.u);
        v_min = std::min(v_min, vx.v);
        v_max = std::max(v_max, vx.v);
    }

    // Coverage is exactly the integer rasterizer's.
    Vertex v0 = p0;
    Vertex v1 = p1;
    Vertex v2 = p2;
    const s32 int_area = edge(v0, v1, v2.x, v2.y);
    if (int_area == 0) {
        return true;
    }
    if (int_area < 0) {
        std::swap(v1, v2);
    }
    const bool edge0_top_left = is_top_left_edge(v1, v2);
    const bool edge1_top_left = is_top_left_edge(v2, v0);
    const bool edge2_top_left = is_top_left_edge(v0, v1);
    s16 min_x = std::min({ v0.x, v1.x, v2.x });
    s16 max_x = std::max({ v0.x, v1.x, v2.x });
    s16 min_y = std::min({ v0.y, v1.y, v2.y });
    s16 max_y = std::max({ v0.y, v1.y, v2.y });
    min_x = std::max(min_x, draw_x_min_);
    max_x = std::min(max_x, draw_x_max_);
    min_y = std::max(min_y, draw_y_min_);
    max_y = std::min(max_y, draw_y_max_);
    if (max_x - min_x > 1023 || max_y - min_y > 511) {
        return true;
    }

    const TextureSampleState texture = prepare_texture_sample_state();
    const bool raw_texture = (gp0_command_ & 0x1u) != 0;
    const bool dither = dither_enabled_ && !g_gpu_extreme_fast_mode;
    const bool semi_enabled = semi_transparency_mode_ && !g_gpu_extreme_fast_mode;
    const Color flat = p0.color;
    const s32 step_w0_x = -(v2.y - v1.y);
    const s32 step_w0_y = (v2.x - v1.x);
    const s32 step_w1_x = -(v0.y - v2.y);
    const s32 step_w1_y = (v0.x - v2.x);
    const s32 step_w2_x = -(v1.y - v0.y);
    const s32 step_w2_y = (v1.x - v0.x);
    s32 w0_row = edge(v1, v2, min_x, min_y);
    s32 w1_row = edge(v2, v0, min_x, min_y);
    s32 w2_row = edge(v0, v1, min_x, min_y);

    // Floor with a little slack so values that are mathematically integral
    // do not drop a texel to float error. 1e-6 stays below the smallest real
    // fraction an integer triangle can produce (1/area, area <= 2^19).
    const auto to_coord = [](double value, u8 lo, u8 hi) {
        const int c = static_cast<int>(std::floor(value + 1.0e-6));
        return static_cast<u8>(std::clamp(c, static_cast<int>(lo), static_cast<int>(hi)));
    };
    const auto to_channel = [](double value) {
        return static_cast<u8>(std::clamp(static_cast<int>(std::floor(value + 1.0e-6)), 0, 255));
    };

    for (s16 y = min_y; y <= max_y; ++y) {
        s16 span_min_x = 0;
        s16 span_max_x = -1;
        if (triangle_scanline_span(
                min_x, max_x, w0_row, w1_row, w2_row,
                step_w0_x, step_w1_x, step_w2_x,
                edge0_top_left, edge1_top_left, edge2_top_left,
                span_min_x, span_max_x)) {
            const double fy = static_cast<double>(y);
            const double fx0 = static_cast<double>(span_min_x);
            double q = inv_w.at(fx0, fy);
            double nu = u_over_w.at(fx0, fy);
            double nv = v_over_w.at(fx0, fy);
            double cr = red.at(fx0, fy);
            double cg = green.at(fx0, fy);
            double cb = blue.at(fx0, fy);
            for (s16 x = span_min_x; x <= span_max_x; ++x) {
                // Pixels on the integer edge can sit just outside the
                // sub-pixel triangle; keep 1/w positive there.
                const double rq = 1.0 / std::max(q, 1.0e-12);
                const u8 u = to_coord(nu * rq, u_min, u_max);
                const u8 v_coord = to_coord(nv * rq, v_min, v_max);
                const u16 texel = read_texel(texture, u, v_coord);
                if (texel != 0) {
                    u16 out15 = texel;
                    if (!raw_texture) {
                        const u8 mr = gouraud ? to_channel(cr) : flat.r;
                        const u8 mg = gouraud ? to_channel(cg) : flat.g;
                        const u8 mb = gouraud ? to_channel(cb) : flat.b;
                        out15 = dither
                            ? modulate_texel_dithered_15bit(texel, mr, mg, mb, x, y)
                            : modulate_texel_15bit(texel, mr, mg, mb);
                    }
                    if (semi_enabled && (texel & 0x8000u) != 0) {
                        set_pixel_clipped(x, y, out15, true);
                    }
                    else {
                        write_pixel_opaque_clipped(x, y, out15);
                    }
                }
                q += inv_w.dx;
                nu += u_over_w.dx;
                nv += v_over_w.dx;
                cr += red.dx;
                cg += green.dx;
                cb += blue.dx;
            }
        }
        w0_row += step_w0_y;
        w1_row += step_w1_y;
        w2_row += step_w2_y;
    }
    return true;
}

void Gpu::draw_rect(s16 x, s16 y, u16 w, u16 h, Color c) {
    if (w == 0 || h == 0) {
        return;
    }
    const u16 color15 = c.to_15bit();
    if (!g_gpu_fast_mode || semi_transparency_mode_) {
        for (u16 dy = 0; dy < h; dy++) {
            for (u16 dx = 0; dx < w; dx++) {
                set_pixel(x + dx, y + dy, color15, semi_transparency_mode_);
            }
        }
        return;
    }

    const s16 min_x = std::max(x, draw_x_min_);
    const s16 min_y = std::max(y, draw_y_min_);
    s16 max_x = std::min<s16>(static_cast<s16>(x + static_cast<s16>(w) - 1), draw_x_max_);
    s16 max_y = std::min<s16>(static_cast<s16>(y + static_cast<s16>(h) - 1), draw_y_max_);
    max_x = std::min<s16>(max_x, static_cast<s16>(psx::VRAM_WIDTH - 1));
    max_y = std::min<s16>(max_y, static_cast<s16>(psx::VRAM_HEIGHT - 1));
    if (min_x > max_x || min_y > max_y) {
        return;
    }

    const u16 out = force_set_mask_bit_ ? static_cast<u16>(color15 | 0x8000u) : color15;
    for (s16 py = min_y; py <= max_y; ++py) {
        u16* row = &vram_[static_cast<size_t>(py) * psx::VRAM_WIDTH];
        if (!check_mask_before_draw_) {
            std::fill(row + min_x, row + max_x + 1, out);
            continue;
        }
        for (s16 px = min_x; px <= max_x; ++px) {
            if ((row[px] & 0x8000u) == 0) {
                row[px] = out;
            }
        }
    }
}

// ── OpenGL upscaler recording ─────────────────────────────────────
// Mirrors what the software rasterizer drew so the UI thread can replay it
// at a higher resolution. Nothing here touches vram_; the software result
// stays authoritative and is only copied out (page syncs, VRAM writes).

void Gpu::set_hw_stream(GpuHwStream *stream) {
    hw_stream_ = stream;
    hw_full_sync_pending_ = true;
}

void Gpu::hw_emit_pending_full_sync() {
    if (!hw_full_sync_pending_) {
        return;
    }
    hw_full_sync_pending_ = false;
    GpuHwCommand cmd{};
    cmd.op = GpuHwOp::FullSync;
    cmd.first = static_cast<u32>(hw_stream_->pixels.size());
    cmd.count = static_cast<u32>(vram_.size());
    hw_stream_->pixels.insert(hw_stream_->pixels.end(), vram_.begin(), vram_.end());
    hw_stream_->commands.push_back(cmd);
    hw_page_dirty_.fill(false);
}

void Gpu::hw_mark_dirty(int x0, int y0, int x1, int y1) {
    x0 = std::clamp(x0, 0, static_cast<int>(psx::VRAM_WIDTH) - 1);
    x1 = std::clamp(x1, 0, static_cast<int>(psx::VRAM_WIDTH) - 1);
    y0 = std::clamp(y0, 0, static_cast<int>(psx::VRAM_HEIGHT) - 1);
    y1 = std::clamp(y1, 0, static_cast<int>(psx::VRAM_HEIGHT) - 1);
    if (x1 < x0 || y1 < y0) {
        return;
    }
    for (int py = y0 / gpu_hw::kPageHeight; py <= y1 / gpu_hw::kPageHeight; ++py) {
        for (int px = x0 / gpu_hw::kPageWidth; px <= x1 / gpu_hw::kPageWidth; ++px) {
            hw_page_dirty_[static_cast<size_t>(py * gpu_hw::kPageColumns + px)] = true;
        }
    }
}

void Gpu::hw_sync_texture_pages() {
    const auto sync_page = [this](int px, int py) {
        px &= gpu_hw::kPageColumns - 1;
        py &= 1;
        const size_t page = static_cast<size_t>(py * gpu_hw::kPageColumns + px);
        if (!hw_page_dirty_[page]) {
            return;
        }
        hw_page_dirty_[page] = false;
        GpuHwCommand cmd{};
        cmd.op = GpuHwOp::SyncPage;
        cmd.x = static_cast<u16>(px * gpu_hw::kPageWidth);
        cmd.y = static_cast<u16>(py * gpu_hw::kPageHeight);
        cmd.w = gpu_hw::kPageWidth;
        cmd.h = gpu_hw::kPageHeight;
        cmd.first = static_cast<u32>(hw_stream_->pixels.size());
        cmd.count = static_cast<u32>(gpu_hw::kPageWidth * gpu_hw::kPageHeight);
        for (int y = 0; y < gpu_hw::kPageHeight; ++y) {
            const u16 *row = vram_.data() +
                static_cast<size_t>(cmd.y + y) * psx::VRAM_WIDTH + cmd.x;
            hw_stream_->pixels.insert(hw_stream_->pixels.end(), row,
                                      row + gpu_hw::kPageWidth);
        }
        hw_stream_->commands.push_back(cmd);
    };

    const int depth = std::min((texpage_ >> 7) & 0x3, 2);
    const int page_x = texpage_ & 0xF;
    const int page_y = (texpage_ >> 4) & 0x1;
    const int texture_pages = 1 << depth; // 4-bit: 1, 8-bit: 2, 15-bit: 4
    for (int i = 0; i < texture_pages; ++i) {
        sync_page(page_x + i, page_y);
    }
    if (depth < 2) {
        const int clut_x = (clut_ & 0x3F) * 16;
        const int clut_y = (clut_ >> 6) & 0x1FF;
        const int clut_entries = depth == 0 ? 16 : 256;
        const int first_page = clut_x / gpu_hw::kPageWidth;
        const int last_page = (clut_x + clut_entries - 1) / gpu_hw::kPageWidth;
        for (int p = first_page; p <= last_page; ++p) {
            sync_page(p, clut_y / gpu_hw::kPageHeight);
        }
    }
}

GpuHwCommand Gpu::hw_draw_state(bool textured, bool raw, bool sprite) const {
    GpuHwCommand cmd{};
    cmd.op = GpuHwOp::Triangles;
    cmd.flags = static_cast<u8>(
        (textured ? gpu_hw::kTextured : 0u) |
        (textured && raw ? gpu_hw::kRawTexture : 0u) |
        (semi_transparency_mode_ ? gpu_hw::kSemiTransparent : 0u) |
        (force_set_mask_bit_ ? gpu_hw::kSetMask : 0u) |
        (check_mask_before_draw_ ? gpu_hw::kCheckMask : 0u) |
        (sprite ? gpu_hw::kSprite : 0u));
    // The PS1 dithers gouraud-shaded and texture-modulated polygons and lines,
    // never rectangles.
    const u8 op = static_cast<u8>(gp0_command_);
    const bool gouraud = op >= 0x20 && op <= 0x5F && (op & 0x10u) != 0;
    if (dither_enabled_ && !sprite && (textured ? !raw : gouraud)) {
        cmd.flags |= gpu_hw::kDither;
    }
    cmd.semi_mode = semi_transparency_;
    if (textured) {
        cmd.tex_depth = static_cast<u8>(std::min((texpage_ >> 7) & 0x3, 2));
        cmd.tex_base_x = static_cast<u16>((texpage_ & 0xF) * 64);
        cmd.tex_base_y = static_cast<u16>(((texpage_ >> 4) & 0x1) * 256);
        cmd.clut_x = static_cast<u16>((clut_ & 0x3F) * 16);
        cmd.clut_y = static_cast<u16>((clut_ >> 6) & 0x1FF);
        cmd.tw_mask_x = static_cast<u8>(tex_window_mask_x_);
        cmd.tw_mask_y = static_cast<u8>(tex_window_mask_y_);
        cmd.tw_off_x = static_cast<u8>(tex_window_off_x_);
        cmd.tw_off_y = static_cast<u8>(tex_window_off_y_);
    }
    cmd.clip_x0 = draw_x_min_;
    cmd.clip_y0 = draw_y_min_;
    cmd.clip_x1 = draw_x_max_;
    cmd.clip_y1 = draw_y_max_;
    return cmd;
}

void Gpu::hw_push_triangle(const GpuHwCommand &state, const GpuHwVertex &a,
                           const GpuHwVertex &b, const GpuHwVertex &c) {
    std::vector<GpuHwCommand> &cmds = hw_stream_->commands;
    if (cmds.empty() || !cmds.back().same_draw_state(state)) {
        GpuHwCommand cmd = state;
        cmd.first = static_cast<u32>(hw_stream_->vertices.size());
        cmd.count = 0;
        cmds.push_back(cmd);
    }
    hw_stream_->vertices.push_back(a);
    hw_stream_->vertices.push_back(b);
    hw_stream_->vertices.push_back(c);
    cmds.back().count += 3;

    const float min_x = std::min({a.x, b.x, c.x});
    const float max_x = std::max({a.x, b.x, c.x});
    const float min_y = std::min({a.y, b.y, c.y});
    const float max_y = std::max({a.y, b.y, c.y});
    hw_mark_dirty(std::max<int>(static_cast<int>(std::floor(min_x)), state.clip_x0),
                  std::max<int>(static_cast<int>(std::floor(min_y)), state.clip_y0),
                  std::min<int>(static_cast<int>(std::ceil(max_x)), state.clip_x1),
                  std::min<int>(static_cast<int>(std::ceil(max_y)), state.clip_y1));
}

namespace {
u32 hw_pack_color(Color c) {
    return static_cast<u32>(c.r) | (static_cast<u32>(c.g) << 8) |
        (static_cast<u32>(c.b) << 16);
}

GpuHwVertex hw_vertex(const Vertex &v, bool perspective) {
    GpuHwVertex out;
    out.x = v.fx;
    out.y = v.fy;
    out.w = perspective ? v.w : 1.0f;
    out.color = hw_pack_color(v.color);
    out.u = static_cast<float>(v.u);
    out.v = static_cast<float>(v.v);
    return out;
}
} // namespace

void Gpu::hw_record_polygon(const Vertex *v, int count, bool textured, bool raw) {
    if (!hw_recording()) {
        return;
    }
    hw_emit_pending_full_sync();
    if (textured) {
        hw_sync_texture_pages();
    }
    const GpuHwCommand state = hw_draw_state(textured, raw, false);
    // Perspective only when every vertex of the triangle carries PGXP depth.
    const auto tri = [&](int i0, int i1, int i2) {
        // Same oversized-primitive rejection as the software rasterizer.
        if (exceeds_primitive_limits(v[i0], v[i1], v[i2])) {
            return;
        }
        const bool perspective = v[i0].has_w && v[i1].has_w && v[i2].has_w;
        hw_push_triangle(state, hw_vertex(v[i0], perspective),
                         hw_vertex(v[i1], perspective),
                         hw_vertex(v[i2], perspective));
    };
    tri(0, 1, 2);
    if (count == 4) {
        tri(1, 2, 3);
    }
}

void Gpu::hw_record_rect(s16 x, s16 y, u16 w, u16 h, Color c, bool textured,
                         u8 u, u8 v, bool raw) {
    if (!hw_recording() || w == 0 || h == 0) {
        return;
    }
    hw_emit_pending_full_sync();
    if (textured) {
        hw_sync_texture_pages();
    }
    const GpuHwCommand state = hw_draw_state(textured, raw, true);
    // Texel coordinates at the rectangle edges; a fragment inside column dx
    // interpolates to u + dx + f (0 < f < 1) and floors to u + dx. Flipped
    // rectangles run backwards so they floor to u + w - 1 - dx.
    const float u0 = tex_rect_x_flip_ ? static_cast<float>(u) + w : u;
    const float u1 = tex_rect_x_flip_ ? static_cast<float>(u) : static_cast<float>(u) + w;
    const float v0 = tex_rect_y_flip_ ? static_cast<float>(v) + h : v;
    const float v1 = tex_rect_y_flip_ ? static_cast<float>(v) : static_cast<float>(v) + h;
    const auto corner = [&](float px, float py, float pu, float pv) {
        GpuHwVertex out;
        out.x = px;
        out.y = py;
        out.color = hw_pack_color(c);
        out.u = pu;
        out.v = pv;
        return out;
    };
    const float x0 = x;
    const float y0 = y;
    const float x1 = static_cast<float>(x) + w;
    const float y1 = static_cast<float>(y) + h;
    const GpuHwVertex tl = corner(x0, y0, u0, v0);
    const GpuHwVertex tr = corner(x1, y0, u1, v0);
    const GpuHwVertex bl = corner(x0, y1, u0, v1);
    const GpuHwVertex br = corner(x1, y1, u1, v1);
    hw_push_triangle(state, tl, tr, bl);
    hw_push_triangle(state, tr, br, bl);
}

void Gpu::hw_record_line(const Vertex &a, Color ca, const Vertex &b, Color cb) {
    if (!hw_recording()) {
        return;
    }
    hw_emit_pending_full_sync();
    const GpuHwCommand state = hw_draw_state(false, false, false);
    // A one-pixel-wide band along the major axis, covering both endpoints.
    const float dx = static_cast<float>(b.x - a.x);
    const float dy = static_cast<float>(b.y - a.y);
    const bool x_major = std::abs(dx) >= std::abs(dy);
    const float ox = x_major ? 0.0f : 1.0f;
    const float oy = x_major ? 1.0f : 0.0f;
    float ax = a.x, ay = a.y, bx = b.x, by = b.y;
    if (x_major) {
        (bx >= ax ? bx : ax) += 1.0f;
    } else {
        (by >= ay ? by : ay) += 1.0f;
    }
    GpuHwVertex p0, p1, p2, p3;
    p0.x = ax;      p0.y = ay;      p0.color = hw_pack_color(ca);
    p1.x = bx;      p1.y = by;      p1.color = hw_pack_color(cb);
    p2.x = ax + ox; p2.y = ay + oy; p2.color = hw_pack_color(ca);
    p3.x = bx + ox; p3.y = by + oy; p3.color = hw_pack_color(cb);
    hw_push_triangle(state, p0, p1, p2);
    hw_push_triangle(state, p1, p3, p2);
}

void Gpu::hw_record_fill(u16 x, u16 y, u16 w, u16 h, u32 color) {
    if (!hw_recording() || w == 0 || h == 0) {
        return;
    }
    hw_emit_pending_full_sync();
    GpuHwCommand cmd{};
    cmd.op = GpuHwOp::Fill;
    cmd.x = x;
    cmd.y = y;
    cmd.w = static_cast<u16>(std::min<int>(w, psx::VRAM_WIDTH - x));
    cmd.h = static_cast<u16>(std::min<int>(h, psx::VRAM_HEIGHT - y));
    cmd.color = color & 0x00FFFFFFu;
    hw_stream_->commands.push_back(cmd);
    hw_mark_dirty(x, y, x + w - 1, y + h - 1);
}

void Gpu::hw_record_vram_write(u16 x, u16 y, u16 w, u16 h) {
    if (!hw_recording() || w == 0 || h == 0) {
        return;
    }
    hw_emit_pending_full_sync();
    // Split at the VRAM edges so every recorded rectangle is contiguous.
    const int widths[2] = {std::min<int>(w, psx::VRAM_WIDTH - x),
                           w - std::min<int>(w, psx::VRAM_WIDTH - x)};
    const int heights[2] = {std::min<int>(h, psx::VRAM_HEIGHT - y),
                            h - std::min<int>(h, psx::VRAM_HEIGHT - y)};
    for (int iy = 0; iy < 2; ++iy) {
        for (int ix = 0; ix < 2; ++ix) {
            const int rw = widths[ix];
            const int rh = heights[iy];
            if (rw <= 0 || rh <= 0) {
                continue;
            }
            GpuHwCommand cmd{};
            cmd.op = GpuHwOp::VramWrite;
            cmd.x = static_cast<u16>(ix == 0 ? x : 0);
            cmd.y = static_cast<u16>(iy == 0 ? y : 0);
            cmd.w = static_cast<u16>(rw);
            cmd.h = static_cast<u16>(rh);
            cmd.first = static_cast<u32>(hw_stream_->pixels.size());
            cmd.count = static_cast<u32>(rw * rh);
            for (int row = 0; row < rh; ++row) {
                const u16 *src = vram_.data() +
                    static_cast<size_t>(cmd.y + row) * psx::VRAM_WIDTH + cmd.x;
                hw_stream_->pixels.insert(hw_stream_->pixels.end(), src, src + rw);
            }
            hw_stream_->commands.push_back(cmd);
            hw_mark_dirty(cmd.x, cmd.y, cmd.x + rw - 1, cmd.y + rh - 1);
        }
    }
}

void Gpu::hw_record_vram_copy(u16 src_x, u16 src_y, u16 dst_x, u16 dst_y,
                              u16 w, u16 h) {
    if (!hw_recording() || w == 0 || h == 0) {
        return;
    }
    const bool wraps = src_x + w > psx::VRAM_WIDTH || dst_x + w > psx::VRAM_WIDTH ||
        src_y + h > psx::VRAM_HEIGHT || dst_y + h > psx::VRAM_HEIGHT;
    if (wraps) {
        // Rare: fall back to the native result of the copy.
        hw_record_vram_write(dst_x, dst_y, w, h);
        return;
    }
    hw_emit_pending_full_sync();
    GpuHwCommand cmd{};
    cmd.op = GpuHwOp::VramCopy;
    cmd.src_x = src_x;
    cmd.src_y = src_y;
    cmd.x = dst_x;
    cmd.y = dst_y;
    cmd.w = w;
    cmd.h = h;
    hw_stream_->commands.push_back(cmd);
    hw_mark_dirty(dst_x, dst_y, dst_x + w - 1, dst_y + h - 1);
}

void Gpu::hw_record_present() {
    if (!hw_recording()) {
        return;
    }
    hw_emit_pending_full_sync();
    const CrtcRect crtc = calculate_crtc_rect(display_);
    const bool interlaced_field_output = display_.interlaced && display_.vres != 0;
    const bool in_vram = crtc.vram_left >= 0 && crtc.vram_top >= 0 &&
        crtc.vram_left + crtc.width <= static_cast<int>(psx::VRAM_WIDTH) &&
        crtc.vram_top + crtc.height <= static_cast<int>(psx::VRAM_HEIGHT);
    GpuHwCommand cmd{};
    cmd.op = GpuHwOp::Present;
    cmd.x = static_cast<u16>(std::max(0, crtc.vram_left));
    cmd.y = static_cast<u16>(std::max(0, crtc.vram_top));
    cmd.w = static_cast<u16>(std::max(1, crtc.width));
    cmd.h = static_cast<u16>(std::max(1, crtc.height));
    // 24-bit (FMV) output packs pixels across VRAM words and field-based
    // deinterlacing reads alternating lines; both are shown from the exact
    // software frame instead.
    const bool software = !display_.display_enabled || display_.is_24bit ||
        !in_vram ||
        (interlaced_field_output && g_deinterlace_mode != DeinterlaceMode::Weave);
    cmd.flags = software ? gpu_hw::kSoftwarePresent : 0u;
    hw_stream_->commands.push_back(cmd);
}
