#include "core/gs/gs_vram.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u8 kBlock32[4][8] = {
    { 0, 1, 4, 5, 16, 17, 20, 21 },
    { 2, 3, 6, 7, 18, 19, 22, 23 },
    { 8, 9, 12, 13, 24, 25, 28, 29 },
    { 10, 11, 14, 15, 26, 27, 30, 31 },
};

constexpr u8 kBlock16[8][4] = {
    { 0, 2, 8, 10 },
    { 1, 3, 9, 11 },
    { 4, 6, 12, 14 },
    { 5, 7, 13, 15 },
    { 16, 18, 24, 26 },
    { 17, 19, 25, 27 },
    { 20, 22, 28, 30 },
    { 21, 23, 29, 31 },
};

constexpr u8 kBlock16S[8][4] = {
    { 0, 2, 16, 18 },
    { 1, 3, 17, 19 },
    { 8, 10, 24, 26 },
    { 9, 11, 25, 27 },
    { 4, 6, 20, 22 },
    { 5, 7, 21, 23 },
    { 12, 14, 28, 30 },
    { 13, 15, 29, 31 },
};

constexpr u8 kColumn32[8][8] = {
    { 0, 1, 4, 5, 8, 9, 12, 13 },
    { 2, 3, 6, 7, 10, 11, 14, 15 },
    { 16, 17, 20, 21, 24, 25, 28, 29 },
    { 18, 19, 22, 23, 26, 27, 30, 31 },
    { 32, 33, 36, 37, 40, 41, 44, 45 },
    { 34, 35, 38, 39, 42, 43, 46, 47 },
    { 48, 49, 52, 53, 56, 57, 60, 61 },
    { 50, 51, 54, 55, 58, 59, 62, 63 },
};

constexpr u8 kColumn16[8][16] = {
    { 0,2,8,10,16,18,24,26,1,3,9,11,17,19,25,27 },
    { 4,6,12,14,20,22,28,30,5,7,13,15,21,23,29,31 },
    { 32,34,40,42,48,50,56,58,33,35,41,43,49,51,57,59 },
    { 36,38,44,46,52,54,60,62,37,39,45,47,53,55,61,63 },
    { 64,66,72,74,80,82,88,90,65,67,73,75,81,83,89,91 },
    { 68,70,76,78,84,86,92,94,69,71,77,79,85,87,93,95 },
    { 96,98,104,106,112,114,120,122,97,99,105,107,113,115,121,123 },
    { 100,102,108,110,116,118,124,126,101,103,109,111,117,119,125,127 },
};

u32 address32(u32 x, u32 y, u32 bp, u32 bw) {
    const u32 page_x = x >> 6;
    const u32 page_y = y >> 5;
    const u32 px = x & 63u;
    const u32 py = y & 31u;
    const u32 block = kBlock32[py >> 3][px >> 3];
    const u32 column = kColumn32[py & 7u][px & 7u];

    const u32 word =
        (bp << 6) +
        ((page_y * bw + page_x) << 11) +
        block * 64u +
        column;
    return (word & ((GsVram::kSize / 4u) - 1u)) * 4u;
}

u32 address16(u32 x, u32 y, u32 bp, u32 bw, bool s_layout) {
    const u32 page_x = x >> 6;
    const u32 page_y = y >> 6;
    const u32 px = x & 63u;
    const u32 py = y & 63u;
    const u32 block =
        s_layout ? kBlock16S[py >> 3][px >> 4] : kBlock16[py >> 3][px >> 4];
    const u32 column = kColumn16[py & 7u][px & 15u];

    const u32 halfword =
        (bp << 7) +
        ((page_y * bw + page_x) << 12) +
        block * 128u +
        column;
    return (halfword & ((GsVram::kSize / 2u) - 1u)) * 2u;
}

u32 address32z(u32 x, u32 y, u32 bp, u32 bw) {
    // Z32/Z24 use the normal 32-bit pixel swizzle with the GS depth block
    // XOR (0x18). At pixel-address granularity that is 0x18 << 6 words.
    const u32 color_byte = address32(x, y, bp, bw);
    const u32 word = (color_byte >> 2) ^ 0x600u;
    return (word & ((GsVram::kSize / 4u) - 1u)) * 4u;
}

u32 address16z(u32 x, u32 y, u32 bp, u32 bw, bool s_layout) {
    // Z16/Z16S use the corresponding 16-bit swizzle with the same block XOR.
    // 16-bit blocks contain 128 halfwords, so the pixel-address XOR is 0xC00.
    const u32 color_byte = address16(x, y, bp, bw, s_layout);
    const u32 halfword = (color_byte >> 1) ^ 0xC00u;
    return (halfword & ((GsVram::kSize / 2u) - 1u)) * 2u;
}

} // namespace

GsVram::GsVram() : data_(kSize, 0) {}

void GsVram::reset() {
    std::fill(data_.begin(), data_.end(), 0);
}

bool GsVram::supported_color_psm(u32 psm) {
    return psm == 0u || psm == 1u || psm == 2u || psm == 10u;
}

bool GsVram::supported_depth_psm(u32 psm) {
    return psm == 48u || psm == 49u || psm == 50u || psm == 58u;
}

u32 GsVram::pixel_address_bytes(
    u32 psm, u32 x, u32 y, u32 bp, u32 bw) {
    switch (psm) {
    case 0: // PSMCT32
    case 1: // PSMCT24
        return address32(x, y, bp, bw);
    case 2: // PSMCT16
        return address16(x, y, bp, bw, false);
    case 10: // PSMCT16S
        return address16(x, y, bp, bw, true);
    default:
        return 0;
    }
}

u32 GsVram::depth_address_bytes(
    u32 psm, u32 x, u32 y, u32 bp, u32 bw) {
    switch (psm) {
    case 48: // PSMZ32
    case 49: // PSMZ24
        return address32z(x, y, bp, bw);
    case 50: // PSMZ16
        return address16z(x, y, bp, bw, false);
    case 58: // PSMZ16S
        return address16z(x, y, bp, bw, true);
    default:
        return 0;
    }
}

bool GsVram::write_pixel(
    u32 psm, u32 x, u32 y, u32 bp, u32 bw, u32 value) {
    if (!supported_color_psm(psm)) return false;

    const u32 a = pixel_address_bytes(psm, x, y, bp, bw);
    if (psm == 0u) {
        data_[a + 0] = static_cast<u8>(value);
        data_[a + 1] = static_cast<u8>(value >> 8);
        data_[a + 2] = static_cast<u8>(value >> 16);
        data_[a + 3] = static_cast<u8>(value >> 24);
    } else if (psm == 1u) {
        data_[a + 0] = static_cast<u8>(value);
        data_[a + 1] = static_cast<u8>(value >> 8);
        data_[a + 2] = static_cast<u8>(value >> 16);
    } else {
        data_[a + 0] = static_cast<u8>(value);
        data_[a + 1] = static_cast<u8>(value >> 8);
    }
    return true;
}

u32 GsVram::read_pixel(
    u32 psm, u32 x, u32 y, u32 bp, u32 bw) const {
    if (!supported_color_psm(psm)) return 0;

    const u32 a = pixel_address_bytes(psm, x, y, bp, bw);
    if (psm == 0u) {
        return static_cast<u32>(data_[a + 0]) |
               (static_cast<u32>(data_[a + 1]) << 8) |
               (static_cast<u32>(data_[a + 2]) << 16) |
               (static_cast<u32>(data_[a + 3]) << 24);
    }
    if (psm == 1u) {
        return static_cast<u32>(data_[a + 0]) |
               (static_cast<u32>(data_[a + 1]) << 8) |
               (static_cast<u32>(data_[a + 2]) << 16);
    }
    return static_cast<u32>(data_[a + 0]) |
           (static_cast<u32>(data_[a + 1]) << 8);
}

bool GsVram::write_depth(
    u32 psm, u32 x, u32 y, u32 bp, u32 bw, u32 value) {
    if (!supported_depth_psm(psm)) return false;
    const u32 a = depth_address_bytes(psm, x, y, bp, bw);

    if (psm == 48u) {
        data_[a + 0] = static_cast<u8>(value);
        data_[a + 1] = static_cast<u8>(value >> 8);
        data_[a + 2] = static_cast<u8>(value >> 16);
        data_[a + 3] = static_cast<u8>(value >> 24);
    } else if (psm == 49u) {
        data_[a + 0] = static_cast<u8>(value);
        data_[a + 1] = static_cast<u8>(value >> 8);
        data_[a + 2] = static_cast<u8>(value >> 16);
    } else {
        data_[a + 0] = static_cast<u8>(value);
        data_[a + 1] = static_cast<u8>(value >> 8);
    }
    return true;
}

u32 GsVram::read_depth(
    u32 psm, u32 x, u32 y, u32 bp, u32 bw) const {
    if (!supported_depth_psm(psm)) return 0;
    const u32 a = depth_address_bytes(psm, x, y, bp, bw);

    if (psm == 48u) {
        return static_cast<u32>(data_[a + 0]) |
               (static_cast<u32>(data_[a + 1]) << 8) |
               (static_cast<u32>(data_[a + 2]) << 16) |
               (static_cast<u32>(data_[a + 3]) << 24);
    }
    if (psm == 49u) {
        return static_cast<u32>(data_[a + 0]) |
               (static_cast<u32>(data_[a + 1]) << 8) |
               (static_cast<u32>(data_[a + 2]) << 16);
    }
    return static_cast<u32>(data_[a + 0]) |
           (static_cast<u32>(data_[a + 1]) << 8);
}

} // namespace ps2
