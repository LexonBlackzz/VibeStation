#include "core/gs/gs_display.h"

#include "core/gs/gs_privileged.h"
#include "core/gs/gs_vram.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u32 kPmode = 0x12000000u;
constexpr u32 kDispfb1 = 0x12000070u;
constexpr u32 kDisplay1 = 0x12000080u;
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
    ++generation_;
    rgba8_.clear();
}

void GsDisplay::update(const GsPrivileged& regs, const GsVram& vram) {
    u64 pmode = 0;
    if (!regs.read64(kPmode, pmode)) {
        reset();
        return;
    }

    const bool enabled[2] = {
        (pmode & 1u) != 0,
        (pmode & 2u) != 0,
    };
    if (!enabled[0] && !enabled[1]) {
        if (valid_) reset();
        return;
    }

    auto present_circuit = [&](u32 circuit) -> bool {
        u64 dispfb = 0;
        u64 display = 0;
        if (!regs.read64(
                kDispfb1 + circuit * kCircuitStride,
                dispfb) ||
            !regs.read64(
                kDisplay1 + circuit * kCircuitStride,
                display)) {
            return false;
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
        const u32 height = dh / magv;

        if (fbw == 0 || width == 0 || height == 0 ||
            width > 2048u || height > 2048u ||
            !GsVram::supported_color_psm(psm)) {
            return false;
        }

        rgba8_.resize(
            static_cast<std::size_t>(width) * height);
        for (u32 y = 0; y < height; ++y) {
            for (u32 x = 0; x < width; ++x) {
                const u32 raw =
                    vram.read_pixel(
                        psm,
                        dbx + x,
                        dby + y,
                        fbp,
                        fbw);
                rgba8_[
                    static_cast<std::size_t>(y) * width + x] =
                    to_rgba8(psm, raw);
            }
        }

        valid_ = true;
        width_ = width;
        height_ = height;
        circuit_ = circuit + 1u;
        psm_ = psm;
        ++generation_;
        return true;
    };

    // Prefer circuit 1, but do not blank the output merely because BIOS
    // enabled it before finishing DISPFB1/DISPLAY1 setup.  Circuit 2 is a
    // valid independent scanout source and is used during some transitions.
    for (u32 circuit = 0; circuit < 2u; ++circuit) {
        if (enabled[circuit] && present_circuit(circuit)) {
            return;
        }
    }

    if (valid_) reset();
}

} // namespace ps2
