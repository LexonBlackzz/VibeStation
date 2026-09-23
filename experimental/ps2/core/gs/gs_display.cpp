#include "core/gs/gs_display.h"

#include "core/gs/gs_privileged.h"
#include "core/gs/gs_vram.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u32 kPmode = 0x12000000u;
constexpr u32 kSmode2 = 0x12000020u;
constexpr u32 kDispfb1 = 0x12000070u;
constexpr u32 kDisplay1 = 0x12000080u;
constexpr u32 kBgcolor = 0x120000E0u;
constexpr u32 kCircuitStride = 0x20u;

u32 expand5(u32 v) {
    return (v << 3) | (v >> 2);
}

u32 to_rgba8(u32 psm, u32 raw) {
    if (psm == 0u) {
        return raw;
    }
    if (psm == 1u) {
        return (raw & 0x00FFFFFFu) | 0xFF000000u;
    }

    const u32 r = expand5(raw & 0x1Fu);
    const u32 g = expand5((raw >> 5) & 0x1Fu);
    const u32 b = expand5((raw >> 10) & 0x1Fu);
    const u32 a = (raw & 0x8000u) != 0 ? 0xFFu : 0x80u;
    return r | (g << 8) | (b << 16) | (a << 24);
}

} // namespace

void GsDisplay::reset() {
    valid_ = false;
    width_ = 0;
    height_ = 0;
    circuit_ = 0;
    psm_ = 0;
    nonzero_pixel_count_ = 0;
    scanout_cache_valid_ = false;
    ++generation_;
    rgba8_.clear();
}

void GsDisplay::update(const GsPrivileged& regs, const GsVram& vram) {
    constexpr std::array<u32, 7> kScanoutRegisters = {
        kPmode, kSmode2, kDispfb1, kDisplay1,
        kDispfb1 + kCircuitStride, kDisplay1 + kCircuitStride, kBgcolor};
    std::array<u64, 7> scanout_registers{};
    for (std::size_t i = 0; i < kScanoutRegisters.size(); ++i) {
        if (!regs.read64(kScanoutRegisters[i], scanout_registers[i])) {
            reset();
            return;
        }
    }
    if (scanout_cache_valid_ &&
        cached_vram_generation_ == vram.generation() &&
        cached_scanout_registers_ == scanout_registers) {
        return;
    }
    scanout_cache_valid_ = true;
    cached_vram_generation_ = vram.generation();
    cached_scanout_registers_ = scanout_registers;

    u64 pmode = 0;
    u64 smode2 = 0;
    if (!regs.read64(kPmode, pmode) ||
        !regs.read64(kSmode2, smode2)) {
        reset();
        return;
    }

    const bool enabled[2] = {
        (pmode & 1u) != 0,
        (pmode & 2u) != 0,
    };
    const bool interlaced = (smode2 & 1u) != 0;
    const bool field_mode = (smode2 & 2u) != 0;
    if (!enabled[0] && !enabled[1]) {
        if (valid_) reset();
        return;
    }

    struct CircuitFrame {
        bool valid = false;
        u32 width = 0;
        u32 height = 0;
        u32 psm = 0;
        std::vector<u32> pixels;
    };

    auto extract_circuit = [&](u32 circuit) {
        CircuitFrame frame{};

        u64 dispfb = 0;
        u64 display = 0;
        if (!regs.read64(
                kDispfb1 + circuit * kCircuitStride,
                dispfb) ||
            !regs.read64(
                kDisplay1 + circuit * kCircuitStride,
                display)) {
            return frame;
        }

        const u32 fbp =
            static_cast<u32>(dispfb & 0x1FFu) << 5;
        const u32 fbw =
            static_cast<u32>((dispfb >> 9) & 0x3Fu);
        const u32 psm =
            static_cast<u32>((dispfb >> 15) & 0x1Fu);
        const u32 dbx =
            static_cast<u32>((dispfb >> 32) & 0x7FFu);
        const u32 dby =
            static_cast<u32>((dispfb >> 43) & 0x7FFu);

        const u32 magh =
            static_cast<u32>((display >> 23) & 0xFu) + 1u;
        const u32 magv =
            static_cast<u32>((display >> 27) & 0x3u) + 1u;
        const u32 dw =
            static_cast<u32>((display >> 32) & 0xFFFu) + 1u;
        const u32 dh =
            static_cast<u32>((display >> 44) & 0x7FFu) + 1u;

        const u32 width = dw / magh;
        const u32 display_height = dh / magv;

        // In interlaced field mode the GS scans only one field's worth of
        // framebuffer lines for each displayed field.  DISPLAY still
        // describes the full output height.  Reading display_height lines
        // here incorrectly walks into the next field/image in VRAM, which
        // presents as a vertically duplicated picture and field-to-field
        // jumping.  Read the half-height field and bob it to a stable host
        // frame instead.
        const bool half_height_field = interlaced && field_mode;
        const u32 source_height = half_height_field
            ? (display_height + 1u) / 2u
            : display_height;

        if (fbw == 0u || width == 0u || display_height == 0u ||
            source_height == 0u ||
            width > 2048u || display_height > 2048u ||
            !GsVram::supported_color_psm(psm)) {
            return frame;
        }

        frame.pixels.resize(
            static_cast<std::size_t>(width) * display_height);
        for (u32 y = 0; y < source_height; ++y) {
            for (u32 x = 0; x < width; ++x) {
                const u32 raw =
                    vram.read_pixel(
                        psm,
                        dbx + x,
                        dby + y,
                        fbp,
                        fbw);
                const u32 rgba = to_rgba8(psm, raw);

                if (half_height_field) {
                    const u32 out_y0 = y * 2u;
                    const u32 out_y1 = out_y0 + 1u;
                    frame.pixels[
                        static_cast<std::size_t>(out_y0) * width + x] =
                        rgba;
                    if (out_y1 < display_height) {
                        frame.pixels[
                            static_cast<std::size_t>(out_y1) * width + x] =
                            rgba;
                    }
                } else {
                    frame.pixels[
                        static_cast<std::size_t>(y) * width + x] =
                        rgba;
                }
            }
        }

        frame.valid = true;
        frame.width = width;
        frame.height = display_height;
        frame.psm = psm;
        return frame;
    };

    CircuitFrame frames[2];
    for (u32 circuit = 0; circuit < 2u; ++circuit) {
        if (enabled[circuit]) {
            frames[circuit] = extract_circuit(circuit);
        }
    }

    if (!frames[0].valid && !frames[1].valid) {
        if (valid_) reset();
        return;
    }

    // PCRTC first lays circuit 2 (or BGCOLOR) down, then blends circuit 1 on
    // top.  This matters during BIOS transitions where both EN1 and EN2 are
    // set but circuit 1 carries zero alpha; picking circuit 1 directly would
    // incorrectly present a black frame.
    const u32 base_circuit =
        frames[0].valid ? 0u : 1u;
    const u32 width = frames[base_circuit].width;
    const u32 height = frames[base_circuit].height;

    u64 bgcolor = 0;
    (void)regs.read64(kBgcolor, bgcolor);
    const u32 background =
        static_cast<u32>(bgcolor & 0x00FFFFFFu) |
        0xFF000000u;

    rgba8_.assign(
        static_cast<std::size_t>(width) * height,
        background);

    const bool slbg = ((pmode >> 7) & 1u) != 0;
    if (!slbg && frames[1].valid) {
        // The GS framebuffer alpha is available to PCRTC blending, but
        // scanout itself is opaque. Do not let the UI blend circuit 2 into
        // its own background when BIOS artwork stores zero alpha in VRAM.
        const u32 copy_width = std::min(width, frames[1].width);
        const u32 copy_height = std::min(height, frames[1].height);
        for (u32 y = 0; y < copy_height; ++y) {
            for (u32 x = 0; x < copy_width; ++x) {
                rgba8_[static_cast<std::size_t>(y) * width + x] =
                    frames[1].pixels[
                        static_cast<std::size_t>(y) *
                            frames[1].width +
                        x] | 0xFF000000u;
            }
        }
    }

    if (frames[0].valid) {
        const bool constant_alpha = ((pmode >> 5) & 1u) != 0;
        const u32 alp = static_cast<u32>((pmode >> 8) & 0xFFu);
        const u32 copy_width = std::min(width, frames[0].width);
        const u32 copy_height = std::min(height, frames[0].height);

        auto blend_channel = [](u32 src, u32 dst, u32 alpha128) {
            const u32 inv = 128u - std::min(alpha128, 128u);
            return std::min(
                255u,
                (src * std::min(alpha128, 128u) +
                 dst * inv +
                 64u) >> 7);
        };

        for (u32 y = 0; y < copy_height; ++y) {
            for (u32 x = 0; x < copy_width; ++x) {
                const std::size_t dst_index =
                    static_cast<std::size_t>(y) * width + x;
                const u32 src =
                    frames[0].pixels[
                        static_cast<std::size_t>(y) *
                            frames[0].width +
                        x];
                const u32 dst = rgba8_[dst_index];

                const u32 alpha128 =
                    constant_alpha
                        ? std::min(alp, 128u)
                        : std::min(
                              128u,
                              ((src >> 24) & 0xFFu));

                const u32 r = blend_channel(
                    src & 0xFFu,
                    dst & 0xFFu,
                    alpha128);
                const u32 g = blend_channel(
                    (src >> 8) & 0xFFu,
                    (dst >> 8) & 0xFFu,
                    alpha128);
                const u32 b = blend_channel(
                    (src >> 16) & 0xFFu,
                    (dst >> 16) & 0xFFu,
                    alpha128);

                rgba8_[dst_index] =
                    r | (g << 8) | (b << 16) | 0xFF000000u;
            }
        }
    }

    valid_ = true;
    width_ = width;
    height_ = height;
    circuit_ =
        frames[0].valid && frames[1].valid
            ? 3u
            : (frames[0].valid ? 1u : 2u);
    psm_ = frames[base_circuit].psm;
    nonzero_pixel_count_ = 0;
    for (const u32 pixel : rgba8_) {
        if ((pixel & 0x00FFFFFFu) != 0) {
            ++nonzero_pixel_count_;
        }
    }
    ++generation_;
}

} // namespace ps2
