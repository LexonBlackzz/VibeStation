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


constexpr u8 kColumn8[16][16] = {
    {0,4,16,20,32,36,48,52,2,6,18,22,34,38,50,54},
    {8,12,24,28,40,44,56,60,10,14,26,30,42,46,58,62},
    {33,37,49,53,1,5,17,21,35,39,51,55,3,7,19,23},
    {41,45,57,61,9,13,25,29,43,47,59,63,11,15,27,31},
    {96,100,112,116,64,68,80,84,98,102,114,118,66,70,82,86},
    {104,108,120,124,72,76,88,92,106,110,122,126,74,78,90,94},
    {65,69,81,85,97,101,113,117,67,71,83,87,99,103,115,119},
    {73,77,89,93,105,109,121,125,75,79,91,95,107,111,123,127},
    {128,132,144,148,160,164,176,180,130,134,146,150,162,166,178,182},
    {136,140,152,156,168,172,184,188,138,142,154,158,170,174,186,190},
    {161,165,177,181,129,133,145,149,163,167,179,183,131,135,147,151},
    {169,173,185,189,137,141,153,157,171,175,187,191,139,143,155,159},
    {224,228,240,244,192,196,208,212,226,230,242,246,194,198,210,214},
    {232,236,248,252,200,204,216,220,234,238,250,254,202,206,218,222},
    {193,197,209,213,225,229,241,245,195,199,211,215,227,231,243,247},
    {201,205,217,221,233,237,249,253,203,207,219,223,235,239,251,255},
};

constexpr u16 kColumn4[16][32] = {
    {0,8,32,40,64,72,96,104,2,10,34,42,66,74,98,106,4,12,36,44,68,76,100,108,6,14,38,46,70,78,102,110},
    {16,24,48,56,80,88,112,120,18,26,50,58,82,90,114,122,20,28,52,60,84,92,116,124,22,30,54,62,86,94,118,126},
    {65,73,97,105,1,9,33,41,67,75,99,107,3,11,35,43,69,77,101,109,5,13,37,45,71,79,103,111,7,15,39,47},
    {81,89,113,121,17,25,49,57,83,91,115,123,19,27,51,59,85,93,117,125,21,29,53,61,87,95,119,127,23,31,55,63},
    {192,200,224,232,128,136,160,168,194,202,226,234,130,138,162,170,196,204,228,236,132,140,164,172,198,206,230,238,134,142,166,174},
    {208,216,240,248,144,152,176,184,210,218,242,250,146,154,178,186,212,220,244,252,148,156,180,188,214,222,246,254,150,158,182,190},
    {129,137,161,169,193,201,225,233,131,139,163,171,195,203,227,235,133,141,165,173,197,205,229,237,135,143,167,175,199,207,231,239},
    {145,153,177,185,209,217,241,249,147,155,179,187,211,219,243,251,149,157,181,189,213,221,245,253,151,159,183,191,215,223,247,255},
    {256,264,288,296,320,328,352,360,258,266,290,298,322,330,354,362,260,268,292,300,324,332,356,364,262,270,294,302,326,334,358,366},
    {272,280,304,312,336,344,368,376,274,282,306,314,338,346,370,378,276,284,308,316,340,348,372,380,278,286,310,318,342,350,374,382},
    {321,329,353,361,257,265,289,297,323,331,355,363,259,267,291,299,325,333,357,365,261,269,293,301,327,335,359,367,263,271,295,303},
    {337,345,369,377,273,281,305,313,339,347,371,379,275,283,307,315,341,349,373,381,277,285,309,317,343,351,375,383,279,287,311,319},
    {448,456,480,488,384,392,416,424,450,458,482,490,386,394,418,426,452,460,484,492,388,396,420,428,454,462,486,494,390,398,422,430},
    {464,472,496,504,400,408,432,440,466,474,498,506,402,410,434,442,468,476,500,508,404,412,436,444,470,478,502,510,406,414,438,446},
    {385,393,417,425,449,457,481,489,387,395,419,427,451,459,483,491,389,397,421,429,453,461,485,493,391,399,423,431,455,463,487,495},
    {401,409,433,441,465,473,497,505,403,411,435,443,467,475,499,507,405,413,437,445,469,477,501,509,407,415,439,447,471,479,503,511},
};

constexpr u8 kClutT32I8[128] = {
    0,1,4,5,8,9,12,13,2,3,6,7,10,11,14,15,
    64,65,68,69,72,73,76,77,66,67,70,71,74,75,78,79,
    16,17,20,21,24,25,28,29,18,19,22,23,26,27,30,31,
    80,81,84,85,88,89,92,93,82,83,86,87,90,91,94,95,
    32,33,36,37,40,41,44,45,34,35,38,39,42,43,46,47,
    96,97,100,101,104,105,108,109,98,99,102,103,106,107,110,111,
    48,49,52,53,56,57,60,61,50,51,54,55,58,59,62,63,
    112,113,116,117,120,121,124,125,114,115,118,119,122,123,126,127
};
constexpr u8 kClutT32I4[16] = {0,1,4,5,8,9,12,13,2,3,6,7,10,11,14,15};
constexpr u8 kClutT16I8[32] = {0,2,8,10,16,18,24,26,4,6,12,14,20,22,28,30,1,3,9,11,17,19,25,27,5,7,13,15,21,23,29,31};
constexpr u8 kClutT16I4[16] = {0,2,8,10,16,18,24,26,4,6,12,14,20,22,28,30};

u32 expand24(u32 value, u32 ta0, bool aem) {
    const u32 rgb = value & 0x00FFFFFFu;
    const u32 alpha = (aem && rgb == 0) ? 0u : (ta0 & 0xFFu);
    return rgb | (alpha << 24);
}

u32 expand16(u16 value, u32 ta0, u32 ta1, bool aem) {
    const u32 color = value;
    const u32 alpha =
        (color & 0x8000u) != 0 ? (ta1 & 0xFFu) :
        (aem && (color & 0x7FFFu) == 0) ? 0u :
        (ta0 & 0xFFu);
    return (alpha << 24) |
           ((color & 0x7C00u) << 9) |
           ((color & 0x03E0u) << 6) |
           ((color & 0x001Fu) << 3);
}

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


u32 address8(u32 x, u32 y, u32 bp, u32 bw) {
    const u32 page_x = x >> 7;
    const u32 page_y = y >> 6;
    const u32 px = x & 127u;
    const u32 py = y & 63u;
    const u32 block = kBlock32[py >> 4][px >> 4];
    const u32 column = kColumn8[py & 15u][px & 15u];
    const u32 pages_wide = bw >> 1;

    const u32 byte =
        (bp << 8) +
        ((page_y * pages_wide + page_x) << 13) +
        block * 256u +
        column;
    return byte & (GsVram::kSize - 1u);
}

u32 address4_nibble(u32 x, u32 y, u32 bp, u32 bw) {
    const u32 page_x = x >> 7;
    const u32 page_y = y >> 7;
    const u32 px = x & 127u;
    const u32 py = y & 127u;
    const u32 block = kBlock16[py >> 4][px >> 5];
    const u32 column = kColumn4[py & 15u][px & 31u];
    const u32 pages_wide = bw >> 1;

    const u32 nibble =
        (bp << 9) +
        ((page_y * pages_wide + page_x) << 14) +
        block * 512u +
        column;
    return nibble & ((GsVram::kSize * 2u) - 1u);
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
    ++generation_;
}

bool GsVram::supported_color_psm(u32 psm) {
    return psm == 0u || psm == 1u || psm == 2u || psm == 10u;
}

bool GsVram::supported_depth_psm(u32 psm) {
    return psm == 48u || psm == 49u || psm == 50u || psm == 58u;
}

bool GsVram::supported_texture_psm(u32 psm) {
    return supported_color_psm(psm) ||
           psm == 19u || psm == 20u || psm == 27u ||
           psm == 36u || psm == 44u;
}

bool GsVram::supported_transfer_psm(u32 psm) {
    return supported_texture_psm(psm) || supported_depth_psm(psm);
}

u32 GsVram::transfer_bpp(u32 psm) {
    switch (psm) {
    case 0: case 48: return 32;
    case 1: case 49: return 24;
    case 2: case 10: case 50: case 58: return 16;
    case 19: case 27: return 8;
    case 20: case 36: case 44: return 4;
    default: return 0;
    }
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
    ++generation_;

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
    ++generation_;
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


bool GsVram::write_index(
    u32 psm, u32 x, u32 y, u32 bp, u32 bw, u32 value) {
    switch (psm) {
    case 19: { // PSMT8
        data_[address8(x, y, bp, bw)] = static_cast<u8>(value);
        ++generation_;
        return true;
    }
    case 20: { // PSMT4
        const u32 nibble = address4_nibble(x, y, bp, bw);
        const u32 a = nibble >> 1;
        const u32 shift = (nibble & 1u) * 4u;
        data_[a] = static_cast<u8>(
            (data_[a] & ~(0xFu << shift)) | ((value & 0xFu) << shift));
        ++generation_;
        return true;
    }
    case 27: { // PSMT8H
        const u32 a = address32(x, y, bp, bw);
        data_[a + 3] = static_cast<u8>(value);
        ++generation_;
        return true;
    }
    case 36: { // PSMT4HL
        const u32 a = address32(x, y, bp, bw);
        data_[a + 3] = static_cast<u8>(
            (data_[a + 3] & 0xF0u) | (value & 0xFu));
        ++generation_;
        return true;
    }
    case 44: { // PSMT4HH
        const u32 a = address32(x, y, bp, bw);
        data_[a + 3] = static_cast<u8>(
            (data_[a + 3] & 0x0Fu) | ((value & 0xFu) << 4));
        ++generation_;
        return true;
    }
    default:
        return false;
    }
}

u32 GsVram::read_index(
    u32 psm, u32 x, u32 y, u32 bp, u32 bw) const {
    switch (psm) {
    case 19:
        return data_[address8(x, y, bp, bw)];
    case 20: {
        const u32 nibble = address4_nibble(x, y, bp, bw);
        return (data_[nibble >> 1] >> ((nibble & 1u) * 4u)) & 0xFu;
    }
    case 27:
        return data_[address32(x, y, bp, bw) + 3];
    case 36:
        return data_[address32(x, y, bp, bw) + 3] & 0xFu;
    case 44:
        return (data_[address32(x, y, bp, bw) + 3] >> 4) & 0xFu;
    default:
        return 0;
    }
}

bool GsVram::write_transfer_pixel(
    u32 psm, u32 x, u32 y, u32 bp, u32 bw, u32 value) {
    if (supported_color_psm(psm))
        return write_pixel(psm, x, y, bp, bw, value);
    if (supported_depth_psm(psm))
        return write_depth(psm, x, y, bp, bw, value);
    if (supported_texture_psm(psm))
        return write_index(psm, x, y, bp, bw, value);
    return false;
}

u32 GsVram::read_transfer_pixel(
    u32 psm, u32 x, u32 y, u32 bp, u32 bw) const {
    if (supported_color_psm(psm))
        return read_pixel(psm, x, y, bp, bw);
    if (supported_depth_psm(psm))
        return read_depth(psm, x, y, bp, bw);
    if (supported_texture_psm(psm))
        return read_index(psm, x, y, bp, bw);
    return 0;
}

bool GsVram::write_linear32(u32 bp, u32 word_index, u32 value) {
    ++generation_;
    const u32 a = ((bp << 8) + word_index * 4u) & (kSize - 1u);
    data_[a + 0] = static_cast<u8>(value);
    data_[a + 1] = static_cast<u8>(value >> 8);
    data_[a + 2] = static_cast<u8>(value >> 16);
    data_[a + 3] = static_cast<u8>(value >> 24);
    return true;
}

bool GsVram::write_linear16(u32 bp, u32 halfword_index, u16 value) {
    ++generation_;
    const u32 a = ((bp << 8) + halfword_index * 2u) & (kSize - 1u);
    data_[a + 0] = static_cast<u8>(value);
    data_[a + 1] = static_cast<u8>(value >> 8);
    return true;
}

u32 GsVram::read_linear32(u32 bp, u32 word_index) const {
    const u32 a = ((bp << 8) + word_index * 4u) & (kSize - 1u);
    return static_cast<u32>(data_[a + 0]) |
           (static_cast<u32>(data_[a + 1]) << 8) |
           (static_cast<u32>(data_[a + 2]) << 16) |
           (static_cast<u32>(data_[a + 3]) << 24);
}

u16 GsVram::read_linear16(u32 bp, u32 halfword_index) const {
    const u32 a = ((bp << 8) + halfword_index * 2u) & (kSize - 1u);
    return static_cast<u16>(
        static_cast<u16>(data_[a + 0]) |
        (static_cast<u16>(data_[a + 1]) << 8));
}

u32 GsVram::read_clut_color(
    u32 texture_psm,
    u32 index,
    u32 cbp,
    u32 cpsm,
    bool csm2,
    u32 csa,
    u32 cbw,
    u32 cou,
    u32 cov,
    u32 ta0,
    u32 ta1,
    bool aem) const {
    const bool eight_bit = texture_psm == 19u || texture_psm == 27u;
    index &= eight_bit ? 0xFFu : 0x0Fu;

    if (csm2) {
        if (cbw == 0) return 0;
        const u32 x = (cou << 4) + index;
        const u32 y = cov;
        if (cpsm == 0u) return read_pixel(0, x, y, cbp, cbw);
        if (cpsm == 1u)
            return expand24(read_pixel(1, x, y, cbp, cbw), ta0, aem);
        if (cpsm == 2u || cpsm == 10u)
            return expand16(
                static_cast<u16>(read_pixel(cpsm, x, y, cbp, cbw)),
                ta0, ta1, aem);
        return 0;
    }

    // CSM1 source data is loaded from the CLUT block using the GS palette
    // permutation. We sample that layout directly. This intentionally avoids
    // emulating CLD cache lifetime for now; the current TEX0 sees current VRAM.
    (void)csa;

    if (cpsm == 0u || cpsm == 1u) {
        u32 source_word = 0;
        if (eight_bit) {
            const u32 chunk = index & 0xF0u;
            source_word =
                (static_cast<u32>(kClutT32I8[chunk & 0x70u]) |
                 (chunk & 0x80u)) +
                kClutT32I4[index & 0x0Fu];
        } else {
            source_word = kClutT32I4[index & 0x0Fu];
        }
        const u32 raw = read_linear32(cbp, source_word);
        return cpsm == 0u ? raw : expand24(raw, ta0, aem);
    }

    if (cpsm == 2u || cpsm == 10u) {
        u32 source_halfword = 0;
        if (eight_bit) {
            source_halfword =
                (index & ~31u) + kClutT16I8[index & 31u];
        } else {
            source_halfword = kClutT16I4[index & 15u];
        }
        return expand16(
            read_linear16(cbp, source_halfword), ta0, ta1, aem);
    }

    return 0;
}

} // namespace ps2
