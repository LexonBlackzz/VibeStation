#include "gte.h"

// ── Register Read/Write ────────────────────────────────────────────

u32 Gte::read_data(u32 reg) const {
  switch (reg) {
  case 0:
    return (static_cast<u16>(v0[0])) |
           (static_cast<u32>(static_cast<u16>(v0[1])) << 16);
  case 1:
    return static_cast<u16>(v0[2]);
  case 2:
    return (static_cast<u16>(v1[0])) |
           (static_cast<u32>(static_cast<u16>(v1[1])) << 16);
  case 3:
    return static_cast<u16>(v1[2]);
  case 4:
    return (static_cast<u16>(v2[0])) |
           (static_cast<u32>(static_cast<u16>(v2[1])) << 16);
  case 5:
    return static_cast<u16>(v2[2]);
  case 6:
    return rgbc[0] | (rgbc[1] << 8) | (rgbc[2] << 16) | (rgbc[3] << 24);
  case 7:
    return static_cast<u32>(otz);
  case 8:
    return static_cast<u32>(static_cast<s32>(ir[0]));
  case 9:
    return static_cast<u32>(static_cast<s32>(ir[1]));
  case 10:
    return static_cast<u32>(static_cast<s32>(ir[2]));
  case 11:
    return static_cast<u32>(static_cast<s32>(ir[3]));
  case 12:
    return (static_cast<u16>(sx[0])) |
           (static_cast<u32>(static_cast<u16>(sy[0])) << 16);
  case 13:
    return (static_cast<u16>(sx[1])) |
           (static_cast<u32>(static_cast<u16>(sy[1])) << 16);
  case 14:
    return (static_cast<u16>(sx[2])) |
           (static_cast<u32>(static_cast<u16>(sy[2])) << 16);
  case 15:
    return (static_cast<u16>(sx[2])) |
           (static_cast<u32>(static_cast<u16>(sy[2])) << 16); // Mirror of SXY2
  case 16:
    return sz[0];
  case 17:
    return sz[1];
  case 18:
    return sz[2];
  case 19:
    return sz[3];
  case 20:
    return rgb_fifo[0];
  case 21:
    return rgb_fifo[1];
  case 22:
    return rgb_fifo[2];
  case 24:
    return static_cast<u32>(mac[0]);
  case 25:
    return static_cast<u32>(mac[1]);
  case 26:
    return static_cast<u32>(mac[2]);
  case 27:
    return static_cast<u32>(mac[3]);
  case 28:
  case 29: {
    // IRGB / ORGB — pack IR1-IR3 into 5-bit components
    u32 r =
        static_cast<u32>(std::clamp(static_cast<int>(ir[1]) / 0x80, 0, 0x1F));
    u32 g =
        static_cast<u32>(std::clamp(static_cast<int>(ir[2]) / 0x80, 0, 0x1F));
    u32 b =
        static_cast<u32>(std::clamp(static_cast<int>(ir[3]) / 0x80, 0, 0x1F));
    return r | (g << 5) | (b << 10);
  }
  case 30:
    return lzcs;
  case 31:
    return lzcr;
  default:
    return 0;
  }
}

u32 Gte::read_ctrl(u32 reg) const {
  switch (reg) {
  case 0:
    return (static_cast<u16>(rotation[0][0])) |
           (static_cast<u32>(static_cast<u16>(rotation[0][1])) << 16);
  case 1:
    return (static_cast<u16>(rotation[0][2])) |
           (static_cast<u32>(static_cast<u16>(rotation[1][0])) << 16);
  case 2:
    return (static_cast<u16>(rotation[1][1])) |
           (static_cast<u32>(static_cast<u16>(rotation[1][2])) << 16);
  case 3:
    return (static_cast<u16>(rotation[2][0])) |
           (static_cast<u32>(static_cast<u16>(rotation[2][1])) << 16);
  case 4:
    return static_cast<u32>(static_cast<s32>(rotation[2][2]));
  case 5:
    return static_cast<u32>(translation[0]);
  case 6:
    return static_cast<u32>(translation[1]);
  case 7:
    return static_cast<u32>(translation[2]);
  case 8:
    return (static_cast<u16>(light[0][0])) |
           (static_cast<u32>(static_cast<u16>(light[0][1])) << 16);
  case 9:
    return (static_cast<u16>(light[0][2])) |
           (static_cast<u32>(static_cast<u16>(light[1][0])) << 16);
  case 10:
    return (static_cast<u16>(light[1][1])) |
           (static_cast<u32>(static_cast<u16>(light[1][2])) << 16);
  case 11:
    return (static_cast<u16>(light[2][0])) |
           (static_cast<u32>(static_cast<u16>(light[2][1])) << 16);
  case 12:
    return static_cast<u32>(static_cast<s32>(light[2][2]));
  case 13:
    return static_cast<u32>(bg_color[0]);
  case 14:
    return static_cast<u32>(bg_color[1]);
  case 15:
    return static_cast<u32>(bg_color[2]);
  case 16:
    return (static_cast<u16>(color_matrix[0][0])) |
           (static_cast<u32>(static_cast<u16>(color_matrix[0][1])) << 16);
  case 17:
    return (static_cast<u16>(color_matrix[0][2])) |
           (static_cast<u32>(static_cast<u16>(color_matrix[1][0])) << 16);
  case 18:
    return (static_cast<u16>(color_matrix[1][1])) |
           (static_cast<u32>(static_cast<u16>(color_matrix[1][2])) << 16);
  case 19:
    return (static_cast<u16>(color_matrix[2][0])) |
           (static_cast<u32>(static_cast<u16>(color_matrix[2][1])) << 16);
  case 20:
    return static_cast<u32>(static_cast<s32>(color_matrix[2][2]));
  case 21:
    return static_cast<u32>(far_color[0]);
  case 22:
    return static_cast<u32>(far_color[1]);
  case 23:
    return static_cast<u32>(far_color[2]);
  case 24:
    return static_cast<u32>(ofx);
  case 25:
    return static_cast<u32>(ofy);
  case 26:
    return static_cast<u32>(static_cast<s32>(h));
  case 27:
    return static_cast<u32>(static_cast<s32>(dqa));
  case 28:
    return static_cast<u32>(dqb);
  case 29:
    return static_cast<u32>(static_cast<s32>(zsf3));
  case 30:
    return static_cast<u32>(static_cast<s32>(zsf4));
  case 31:
    return flags;
  default:
    return 0;
  }
}

void Gte::write_data(u32 reg, u32 value) {
  switch (reg) {
  case 0:
    v0[0] = static_cast<s16>(value);
    v0[1] = static_cast<s16>(value >> 16);
    break;
  case 1:
    v0[2] = static_cast<s16>(value);
    break;
  case 2:
    v1[0] = static_cast<s16>(value);
    v1[1] = static_cast<s16>(value >> 16);
    break;
  case 3:
    v1[2] = static_cast<s16>(value);
    break;
  case 4:
    v2[0] = static_cast<s16>(value);
    v2[1] = static_cast<s16>(value >> 16);
    break;
  case 5:
    v2[2] = static_cast<s16>(value);
    break;
  case 6:
    rgbc[0] = value & 0xFF;
    rgbc[1] = (value >> 8) & 0xFF;
    rgbc[2] = (value >> 16) & 0xFF;
    rgbc[3] = (value >> 24) & 0xFF;
    break;
  case 7:
    otz = value & 0xFFFF;
    break;
  case 8:
    ir[0] = static_cast<s16>(value);
    break;
  case 9:
    ir[1] = static_cast<s16>(value);
    break;
  case 10:
    ir[2] = static_cast<s16>(value);
    break;
  case 11:
    ir[3] = static_cast<s16>(value);
    break;
  case 12:
    sx[0] = static_cast<s16>(value);
    sy[0] = static_cast<s16>(value >> 16);
    break;
  case 13:
    sx[1] = static_cast<s16>(value);
    sy[1] = static_cast<s16>(value >> 16);
    break;
  case 14:
    sx[2] = static_cast<s16>(value);
    sy[2] = static_cast<s16>(value >> 16);
    break;
  case 15: // SXY2 FIFO — pushes
    push_sx(static_cast<s16>(value));
    push_sy(static_cast<s16>(value >> 16));
    break;
  case 16:
    sz[0] = static_cast<u16>(value);
    break;
  case 17:
    sz[1] = static_cast<u16>(value);
    break;
  case 18:
    sz[2] = static_cast<u16>(value);
    break;
  case 19:
    sz[3] = static_cast<u16>(value);
    break;
  case 20:
    rgb_fifo[0] = value;
    break;
  case 21:
    rgb_fifo[1] = value;
    break;
  case 22:
    rgb_fifo[2] = value;
    break;
  case 28: // IRGB
    ir[1] = static_cast<s16>((value & 0x1F) * 0x80);
    ir[2] = static_cast<s16>(((value >> 5) & 0x1F) * 0x80);
    ir[3] = static_cast<s16>(((value >> 10) & 0x1F) * 0x80);
    break;
  case 30:
    lzcs = value;
    lzcr = static_cast<u32>(count_leading_bits(value));
    break;
  default:
    break;
  }
}

void Gte::write_ctrl(u32 reg, u32 value) {
  switch (reg) {
  case 0:
    rotation[0][0] = static_cast<s16>(value);
    rotation[0][1] = static_cast<s16>(value >> 16);
    break;
  case 1:
    rotation[0][2] = static_cast<s16>(value);
    rotation[1][0] = static_cast<s16>(value >> 16);
    break;
  case 2:
    rotation[1][1] = static_cast<s16>(value);
    rotation[1][2] = static_cast<s16>(value >> 16);
    break;
  case 3:
    rotation[2][0] = static_cast<s16>(value);
    rotation[2][1] = static_cast<s16>(value >> 16);
    break;
  case 4:
    rotation[2][2] = static_cast<s16>(value);
    break;
  case 5:
    translation[0] = static_cast<s32>(value);
    break;
  case 6:
    translation[1] = static_cast<s32>(value);
    break;
  case 7:
    translation[2] = static_cast<s32>(value);
    break;
  case 8:
    light[0][0] = static_cast<s16>(value);
    light[0][1] = static_cast<s16>(value >> 16);
    break;
  case 9:
    light[0][2] = static_cast<s16>(value);
    light[1][0] = static_cast<s16>(value >> 16);
    break;
  case 10:
    light[1][1] = static_cast<s16>(value);
    light[1][2] = static_cast<s16>(value >> 16);
    break;
  case 11:
    light[2][0] = static_cast<s16>(value);
    light[2][1] = static_cast<s16>(value >> 16);
    break;
  case 12:
    light[2][2] = static_cast<s16>(value);
    break;
  case 13:
    bg_color[0] = static_cast<s32>(value);
    break;
  case 14:
    bg_color[1] = static_cast<s32>(value);
    break;
  case 15:
    bg_color[2] = static_cast<s32>(value);
    break;
  case 16:
    color_matrix[0][0] = static_cast<s16>(value);
    color_matrix[0][1] = static_cast<s16>(value >> 16);
    break;
  case 17:
    color_matrix[0][2] = static_cast<s16>(value);
    color_matrix[1][0] = static_cast<s16>(value >> 16);
    break;
  case 18:
    color_matrix[1][1] = static_cast<s16>(value);
    color_matrix[1][2] = static_cast<s16>(value >> 16);
    break;
  case 19:
    color_matrix[2][0] = static_cast<s16>(value);
    color_matrix[2][1] = static_cast<s16>(value >> 16);
    break;
  case 20:
    color_matrix[2][2] = static_cast<s16>(value);
    break;
  case 21:
    far_color[0] = static_cast<s32>(value);
    break;
  case 22:
    far_color[1] = static_cast<s32>(value);
    break;
  case 23:
    far_color[2] = static_cast<s32>(value);
    break;
  case 24:
    ofx = static_cast<s32>(value);
    break;
  case 25:
    ofy = static_cast<s32>(value);
    break;
  case 26:
    h = static_cast<u16>(value);
    break;
  case 27:
    dqa = static_cast<s16>(value);
    break;
  case 28:
    dqb = static_cast<s32>(value);
    break;
  case 29:
    zsf3 = static_cast<s16>(value);
    break;
  case 30:
    zsf4 = static_cast<s16>(value);
    break;
  case 31:
    flags = value & 0x7FFFF000;
    if (flags & 0x7F87E000)
      flags |= 0x80000000;
    break;
  default:
    break;
  }
}

// ── Command Dispatch ───────────────────────────────────────────────

void Gte::execute(u32 command) {
  flags = 0;
  current_command_ = command;
  sf = (command & (1 << 19)) ? 12 : 0;
  lm = (command & (1 << 10)) != 0;

  u32 opcode = command & 0x3F;

  switch (opcode) {
  case 0x01:
    cmd_rtps(0, true);
    break;
  case 0x06:
    cmd_nclip();
    break;
  case 0x0C:
    cmd_op();
    break;
  case 0x10:
    cmd_dpcs();
    break;
  case 0x11:
    cmd_intpl();
    break;
  case 0x12:
    cmd_mvmva();
    break;
  case 0x13:
    cmd_ncds(0);
    break;
  case 0x14:
    cmd_cdp();
    break;
  case 0x16:
    cmd_ncdt();
    break;
  case 0x1B:
    cmd_nccs(0);
    break;
  case 0x1E:
    cmd_ncs(0);
    break;
  case 0x20:
    cmd_nct();
    break;
  case 0x1C:
    cmd_cc();
    break;
  case 0x28:
    cmd_sqr();
    break;
  case 0x29:
    cmd_dcpl();
    break;
  case 0x2A:
    cmd_dpct();
    break;
  case 0x2D:
    cmd_avsz3();
    break;
  case 0x2E:
    cmd_avsz4();
    break;
  case 0x30:
    cmd_rtpt();
    break;
  case 0x3D:
    cmd_gpf();
    break;
  case 0x3E:
    cmd_gpl();
    break;
  case 0x3F:
    cmd_ncct();
    break;
  default:
    LOG_WARN("GTE: Unhandled command 0x%02X", opcode);
    break;
  }

  // Update error flag (bit 31 = OR of bits 30-23 and bits 18-13)
  u32 error_bits = (flags & 0x7F87E000);
  if (error_bits)
    flags |= 0x80000000;
}

// ── Helpers ────────────────────────────────────────────────────────

s64 Gte::set_mac(int idx, s64 value) {
    // MAC0 is 32-bit signed range, MAC1..3 are 44-bit signed range.
    if (idx == 0) {
        if (value > 0x7FFFFFFFLL) {
            flags |= (1u << 16);
        }
        if (value < -0x80000000LL) {
            flags |= (1u << 15);
        }
    }
    else {
        constexpr s64 kMacPosLimit = 0x7FFFFFFFFFFLL;  // +2^43-1
        constexpr s64 kMacNegLimit = -0x80000000000LL; // -2^43
        if (value > kMacPosLimit) {
            flags |= (1u << (31 - idx)); // bits 30..28 for MAC1..3 positive overflow
        }
        if (value < kMacNegLimit) {
            flags |= (1u << (28 - idx)); // bits 27..25 for MAC1..3 negative overflow
        }
    }
    mac[idx] = static_cast<s32>(idx == 0 ? value : (value >> sf));
    return value;
}

void Gte::set_ir(int idx, s32 value, bool lm_flag) {
    // IR0 uses a dedicated 0..0x1000 clamp/flag, independent of LM.
    if (idx == 0) {
        const s32 lower = 0;
        const s32 upper = 0x1000;
        if (value < lower) {
            ir[0] = static_cast<s16>(lower);
            flags |= (1u << 12);
        }
        else if (value > upper) {
            ir[0] = static_cast<s16>(upper);
            flags |= (1u << 12);
        }
        else {
            ir[0] = static_cast<s16>(value);
        }
        return;
    }

    const s32 lower = lm_flag ? 0 : -0x8000;
    const s32 upper = 0x7FFF;
    const u32 sat_flag = (1u << (25 - idx)); // IR1/2/3 -> bits 24/23/22

    if (value < lower) {
        ir[idx] = static_cast<s16>(lower);
        flags |= sat_flag;
    }
    else if (value > upper) {
        ir[idx] = static_cast<s16>(upper);
        flags |= sat_flag;
    }
    else {
        ir[idx] = static_cast<s16>(value);
    }
}

void Gte::push_sx(s16 val) {
  sx[0] = sx[1];
  sx[1] = sx[2];
  sx[2] = val;
}

void Gte::push_sy(s16 val) {
  sy[0] = sy[1];
  sy[1] = sy[2];
  sy[2] = val;
}

void Gte::push_sz(u16 val) {
  sz[0] = sz[1];
  sz[1] = sz[2];
  sz[2] = sz[3];
  sz[3] = val;
}

void Gte::push_rgb(u32 val) {
  rgb_fifo[0] = rgb_fifo[1];
  rgb_fifo[1] = rgb_fifo[2];
  rgb_fifo[2] = val;
}

s32 Gte::clamp(s32 value, s32 min_val, s32 max_val, u32 flag_bit) {
  if (value < min_val) {
    flags |= flag_bit;
    return min_val;
  }
  if (value > max_val) {
    flags |= flag_bit;
    return max_val;
  }
  return value;
}

int Gte::count_leading_bits(u32 value) const {
  if (value == 0)
    return 32;
  if (value & 0x80000000u)
    value = ~value;
  int count = 0;
  while (!(value & 0x80000000u) && count < 32) {
    value <<= 1;
    count++;
  }
  return count;
}

int Gte::count_leading_zeros16(u16 value) const {
  if (value == 0)
    return 16;
  int count = 0;
  u32 wide = static_cast<u32>(value) << 16;
  while (!(wide & 0x80000000u) && count < 16) {
    wide <<= 1;
    count++;
  }
  return count;
}

u32 Gte::divide(u16 h_val, u16 sz3_val) {
  if (sz3_val == 0 || (static_cast<u32>(sz3_val) * 2u) <= h_val) {
    flags |= (1u << 17);
    return 0x1FFFF;
  }

  const u32 shift = static_cast<u32>(count_leading_zeros16(sz3_val));
  const u32 lhs = static_cast<u32>(h_val) << shift;
  const u32 rhs = static_cast<u32>(sz3_val) << shift;
  const u32 divisor = rhs | 0x8000u;

  static constexpr u8 kUnrTable[257] = {
      0xFF, 0xFD, 0xFB, 0xF9, 0xF7, 0xF5, 0xF3, 0xF1, 0xEF, 0xEE, 0xEC, 0xEA,
      0xE8, 0xE6, 0xE4, 0xE3, 0xE1, 0xDF, 0xDD, 0xDC, 0xDA, 0xD8, 0xD6, 0xD5,
      0xD3, 0xD1, 0xD0, 0xCE, 0xCD, 0xCB, 0xC9, 0xC8, 0xC6, 0xC5, 0xC3, 0xC1,
      0xC0, 0xBE, 0xBD, 0xBB, 0xBA, 0xB8, 0xB7, 0xB5, 0xB4, 0xB2, 0xB1, 0xB0,
      0xAE, 0xAD, 0xAB, 0xAA, 0xA9, 0xA7, 0xA6, 0xA4, 0xA3, 0xA2, 0xA0, 0x9F,
      0x9E, 0x9C, 0x9B, 0x9A, 0x99, 0x97, 0x96, 0x95, 0x94, 0x92, 0x91, 0x90,
      0x8F, 0x8D, 0x8C, 0x8B, 0x8A, 0x89, 0x87, 0x86, 0x85, 0x84, 0x83, 0x82,
      0x81, 0x7F, 0x7E, 0x7D, 0x7C, 0x7B, 0x7A, 0x79, 0x78, 0x77, 0x75, 0x74,
      0x73, 0x72, 0x71, 0x70, 0x6F, 0x6E, 0x6D, 0x6C, 0x6B, 0x6A, 0x69, 0x68,
      0x67, 0x66, 0x65, 0x64, 0x63, 0x62, 0x61, 0x60, 0x5F, 0x5E, 0x5D, 0x5D,
      0x5C, 0x5B, 0x5A, 0x59, 0x58, 0x57, 0x56, 0x55, 0x54, 0x53, 0x53, 0x52,
      0x51, 0x50, 0x4F, 0x4E, 0x4D, 0x4D, 0x4C, 0x4B, 0x4A, 0x49, 0x48, 0x48,
      0x47, 0x46, 0x45, 0x44, 0x43, 0x43, 0x42, 0x41, 0x40, 0x3F, 0x3F, 0x3E,
      0x3D, 0x3C, 0x3C, 0x3B, 0x3A, 0x39, 0x39, 0x38, 0x37, 0x36, 0x36, 0x35,
      0x34, 0x33, 0x33, 0x32, 0x31, 0x31, 0x30, 0x2F, 0x2E, 0x2E, 0x2D, 0x2C,
      0x2C, 0x2B, 0x2A, 0x2A, 0x29, 0x28, 0x28, 0x27, 0x26, 0x26, 0x25, 0x24,
      0x24, 0x23, 0x22, 0x22, 0x21, 0x20, 0x20, 0x1F, 0x1E, 0x1E, 0x1D, 0x1D,
      0x1C, 0x1B, 0x1B, 0x1A, 0x19, 0x19, 0x18, 0x18, 0x17, 0x16, 0x16, 0x15,
      0x15, 0x14, 0x14, 0x13, 0x12, 0x12, 0x11, 0x11, 0x10, 0x0F, 0x0F, 0x0E,
      0x0E, 0x0D, 0x0D, 0x0C, 0x0C, 0x0B, 0x0A, 0x0A, 0x09, 0x09, 0x08, 0x08,
      0x07, 0x07, 0x06, 0x06, 0x05, 0x05, 0x04, 0x04, 0x03, 0x03, 0x02, 0x02,
      0x01, 0x01, 0x00, 0x00, 0x00};

  const s32 x =
      0x101 + static_cast<s32>(kUnrTable[((divisor & 0x7FFFu) + 0x40u) >> 7]);
  const s32 d = ((-static_cast<s32>(divisor) * x) + 0x80) >> 8;
  const u32 recip =
      static_cast<u32>(((x * (0x20000 + d)) + 0x80) >> 8);
  const u32 result = static_cast<u32>(
      ((static_cast<u64>(lhs) * static_cast<u64>(recip)) + 0x8000u) >> 16);
  return std::min<u32>(0x1FFFFu, result);
}

// ── Core Commands ──────────────────────────────────────────────────

void Gte::cmd_rtps(int v_idx, bool set_mac0) {
  // Rotate, Translate, Perspective Single
  s16 *v;
  switch (v_idx) {
  case 0:
    v = v0;
    break;
  case 1:
    v = v1;
    break;
  case 2:
    v = v2;
    break;
  default:
    v = v0;
    break;
  }

  // MAC1-3 = TR + RT * V
  s64 raw_mac[3] = {};
  for (int i = 0; i < 3; i++) {
    s64 result = static_cast<s64>(translation[i]) << 12;
    result += static_cast<s64>(rotation[i][0]) * v[0];
    result += static_cast<s64>(rotation[i][1]) * v[1];
    result += static_cast<s64>(rotation[i][2]) * v[2];
    raw_mac[i] = result;
    set_mac(i + 1, result);
    if (i < 2)
      set_ir(i + 1, mac[i + 1], lm);
  }

  // RTPS handles IR3 saturation slightly differently from generic IR writes:
  // the stored IR3 value follows MAC3/lm clamping, but FLAG.22 is based on
  // raw MAC3 >> 12 range checking.
  const s32 ir3_flag_value = static_cast<s32>(raw_mac[2] >> 12);
  if (ir3_flag_value < -0x8000 || ir3_flag_value > 0x7FFF)
    flags |= (1u << 22);
  const s32 ir3_lower = lm ? 0 : -0x8000;
  if (mac[3] < ir3_lower)
    ir[3] = static_cast<s16>(ir3_lower);
  else if (mac[3] > 0x7FFF)
    ir[3] = 0x7FFF;
  else
    ir[3] = static_cast<s16>(mac[3]);

  // Push SZ FIFO.
  // PSX-SPX: SZ3 = MAC3 SAR ((1-sf)*12). Since MAC3 already applies sf shift,
  // this resolves to raw MAC3 >> 12 for both sf=0 and sf=1.
  u16 sz_val =
      static_cast<u16>(clamp(static_cast<s32>(raw_mac[2] >> 12), 0, 0xFFFF,
                             1u << 18));
  push_sz(sz_val);

  // Perspective division
  u32 div_result = divide(h, sz[3]);

  // SXY2 = OFX/OFY + IR1/IR2 * div_result
  s64 screen_x = static_cast<s64>(ofx) + static_cast<s64>(ir[1]) * div_result;
  s64 screen_y = static_cast<s64>(ofy) + static_cast<s64>(ir[2]) * div_result;

  s16 sx_val = static_cast<s16>(
      clamp(static_cast<s32>(screen_x >> 16), -0x400, 0x3FF, 1u << 14));
  s16 sy_val = static_cast<s16>(
      clamp(static_cast<s32>(screen_y >> 16), -0x400, 0x3FF, 1u << 13));
  push_sx(sx_val);
  push_sy(sy_val);

  // Depth cueing
  if (set_mac0) {
    s64 mac0_val = static_cast<s64>(dqb) + static_cast<s64>(dqa) * div_result;
    set_mac(0, mac0_val);
    set_ir(0, static_cast<s32>(mac[0] >> 12), false);
  }
}

void Gte::cmd_rtpt() {
  // RTPS for all 3 vertices
  cmd_rtps(0, false);
  cmd_rtps(1, false);
  cmd_rtps(2, true);
}

void Gte::cmd_nclip() {
  // Normal clipping: SX0*(SY1-SY2) + SX1*(SY2-SY0) + SX2*(SY0-SY1)
  s64 result = static_cast<s64>(sx[0]) * (sy[1] - sy[2]) +
               static_cast<s64>(sx[1]) * (sy[2] - sy[0]) +
               static_cast<s64>(sx[2]) * (sy[0] - sy[1]);
  set_mac(0, result);
}

void Gte::cmd_avsz3() {
  // MAC0 = ZSF3 * (SZ1 + SZ2 + SZ3)
  s64 result = static_cast<s64>(zsf3) * (sz[1] + sz[2] + sz[3]);
  set_mac(0, result);
  otz = static_cast<u32>(
      clamp(static_cast<s32>(result >> 12), 0, 0xFFFF, 1u << 18));
}

void Gte::cmd_avsz4() {
  s64 result = static_cast<s64>(zsf4) * (sz[0] + sz[1] + sz[2] + sz[3]);
  set_mac(0, result);
  otz = static_cast<u32>(
      clamp(static_cast<s32>(result >> 12), 0, 0xFFFF, 1u << 18));
}

void Gte::cmd_mvmva() {
  // Command bits (for opcode 12h):
  // 17-18 matrix select, 15-16 vector select, 13-14 translation select.
  // This models the PSX-SPX selector behavior instead of hardwiring RT*V0+TR.
  const u32 cmd = current_command_;
  const u32 mx = (cmd >> 17) & 0x3;
  const u32 vx = (cmd >> 15) & 0x3;
  const u32 cv = (cmd >> 13) & 0x3;

  const s16(*mat)[3] = rotation;
  switch (mx) {
  case 0:
    mat = rotation;
    break;
  case 1:
    mat = light;
    break;
  case 2:
    mat = color_matrix;
    break;
  default:
    mat = rotation;
    break;
  }

  s16 vec[3] = {0, 0, 0};
  switch (vx) {
  case 0:
    vec[0] = v0[0];
    vec[1] = v0[1];
    vec[2] = v0[2];
    break;
  case 1:
    vec[0] = v1[0];
    vec[1] = v1[1];
    vec[2] = v1[2];
    break;
  case 2:
    vec[0] = v2[0];
    vec[1] = v2[1];
    vec[2] = v2[2];
    break;
  default:
    vec[0] = ir[1];
    vec[1] = ir[2];
    vec[2] = ir[3];
    break;
  }

  s32 add[3] = {0, 0, 0};
  switch (cv) {
  case 0:
    add[0] = translation[0];
    add[1] = translation[1];
    add[2] = translation[2];
    break;
  case 1:
    add[0] = bg_color[0];
    add[1] = bg_color[1];
    add[2] = bg_color[2];
    break;
  case 2:
    add[0] = far_color[0];
    add[1] = far_color[1];
    add[2] = far_color[2];
    break;
  default:
    add[0] = 0;
    add[1] = 0;
    add[2] = 0;
    break;
  }

  for (int i = 0; i < 3; i++) {
    s64 result = static_cast<s64>(add[i]) << 12;
    result += static_cast<s64>(mat[i][0]) * vec[0];
    result += static_cast<s64>(mat[i][1]) * vec[1];
    result += static_cast<s64>(mat[i][2]) * vec[2];
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
}

void Gte::normal_color_stage(int v_idx) {
  s16 *v = v0;
  switch (v_idx) {
  case 0:
    v = v0;
    break;
  case 1:
    v = v1;
    break;
  case 2:
    v = v2;
    break;
  default:
    break;
  }

  // Stage 1: IR = LLM * V
  for (int i = 0; i < 3; ++i) {
    s64 result = 0;
    result += static_cast<s64>(light[i][0]) * v[0];
    result += static_cast<s64>(light[i][1]) * v[1];
    result += static_cast<s64>(light[i][2]) * v[2];
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }

  // Stage 2: IR = BK + LCM * IR
  const s16 ir_copy[3] = {ir[1], ir[2], ir[3]};
  for (int i = 0; i < 3; ++i) {
    s64 result = static_cast<s64>(bg_color[i]) << 12;
    result += static_cast<s64>(color_matrix[i][0]) * ir_copy[0];
    result += static_cast<s64>(color_matrix[i][1]) * ir_copy[1];
    result += static_cast<s64>(color_matrix[i][2]) * ir_copy[2];
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
}

void Gte::push_rgb_from_mac() {
  const u8 r = static_cast<u8>(clamp(mac[1] >> 4, 0, 0xFF, 1u << 21));
  const u8 g = static_cast<u8>(clamp(mac[2] >> 4, 0, 0xFF, 1u << 20));
  const u8 b = static_cast<u8>(clamp(mac[3] >> 4, 0, 0xFF, 1u << 19));
  push_rgb(r | (g << 8) | (b << 16) | (rgbc[3] << 24));
}

void Gte::apply_depth_cue_mac() {
  for (int i = 0; i < 3; ++i) {
    const s64 mac_in = static_cast<s64>(mac[i + 1]) << sf;
    const s64 fc = static_cast<s64>(far_color[i]) << 12;
    const s64 result =
        mac_in + (((fc - mac_in) * static_cast<s64>(ir[0])) >> 12);
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
}

void Gte::cmd_ncs(int v_idx) {
  normal_color_stage(v_idx);
  push_rgb_from_mac();
}

void Gte::cmd_nct() {
  cmd_ncs(0);
  cmd_ncs(1);
  cmd_ncs(2);
}

void Gte::cmd_nccs(int v_idx) {
  normal_color_stage(v_idx);
  for (int i = 0; i < 3; ++i) {
    const s64 result = (static_cast<s64>(rgbc[i]) * static_cast<s64>(ir[i + 1])) << 4;
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
  push_rgb_from_mac();
}

void Gte::cmd_ncct() {
  cmd_nccs(0);
  cmd_nccs(1);
  cmd_nccs(2);
}

void Gte::cmd_ncds(int v_idx) {
  normal_color_stage(v_idx);
  for (int i = 0; i < 3; ++i) {
    const s64 result = (static_cast<s64>(rgbc[i]) * static_cast<s64>(ir[i + 1])) << 4;
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
  apply_depth_cue_mac();
  push_rgb_from_mac();
}

void Gte::cmd_ncdt() {
  cmd_ncds(0);
  cmd_ncds(1);
  cmd_ncds(2);
}

void Gte::cmd_cdp() {
  // CC stage
  const s16 ir_copy[3] = {ir[1], ir[2], ir[3]};
  for (int i = 0; i < 3; ++i) {
    s64 result = static_cast<s64>(bg_color[i]) << 12;
    result += static_cast<s64>(color_matrix[i][0]) * ir_copy[0];
    result += static_cast<s64>(color_matrix[i][1]) * ir_copy[1];
    result += static_cast<s64>(color_matrix[i][2]) * ir_copy[2];
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
  for (int i = 0; i < 3; ++i) {
    const s64 result = (static_cast<s64>(rgbc[i]) * static_cast<s64>(ir[i + 1])) << 4;
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
  apply_depth_cue_mac();
  push_rgb_from_mac();
}

void Gte::cmd_dpcs() {
  for (int i = 0; i < 3; ++i) {
    const s64 color = static_cast<s64>(rgbc[i]) << 16;
    const s64 fc = static_cast<s64>(far_color[i]) << 12;
    const s64 result = color + (((fc - color) * static_cast<s64>(ir[0])) >> 12);
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
  push_rgb_from_mac();
}

void Gte::cmd_dcpl() {
  for (int i = 0; i < 3; ++i) {
    const s64 col = (static_cast<s64>(rgbc[i]) * static_cast<s64>(ir[i + 1])) << 4;
    const s64 fc = static_cast<s64>(far_color[i]) << 12;
    const s64 result = col + (((fc - col) * static_cast<s64>(ir[0])) >> 12);
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
  push_rgb_from_mac();
}

void Gte::cmd_dpct() {
  for (int n = 0; n < 3; ++n) {
    const u32 src = rgb_fifo[0];
    for (int i = 0; i < 3; ++i) {
      const u8 src_comp = static_cast<u8>((src >> (i * 8)) & 0xFFu);
      const s64 color = static_cast<s64>(src_comp) << 16;
      const s64 fc = static_cast<s64>(far_color[i]) << 12;
      const s64 result =
          color + (((fc - color) * static_cast<s64>(ir[0])) >> 12);
      set_mac(i + 1, result);
      set_ir(i + 1, mac[i + 1], lm);
    }
    push_rgb_from_mac();
  }
}

void Gte::cmd_intpl() {
  for (int i = 0; i < 3; ++i) {
    const s64 ir_val = static_cast<s64>(ir[i + 1]) << 12;
    const s64 fc = static_cast<s64>(far_color[i]) << 12;
    const s64 result =
        ir_val + (((fc - ir_val) * static_cast<s64>(ir[0])) >> 12);
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
  push_rgb_from_mac();
}

void Gte::cmd_sqr() {
  for (int i = 1; i <= 3; i++) {
    set_mac(i, static_cast<s64>(ir[i]) * ir[i]);
    set_ir(i, mac[i], lm);
  }
}

void Gte::cmd_op() {
  // Cross product of rotation matrix diagonal with IR
  set_mac(1, static_cast<s64>(rotation[1][1]) * ir[3] -
                 static_cast<s64>(rotation[2][2]) * ir[2]);
  set_mac(2, static_cast<s64>(rotation[2][2]) * ir[1] -
                 static_cast<s64>(rotation[0][0]) * ir[3]);
  set_mac(3, static_cast<s64>(rotation[0][0]) * ir[2] -
                 static_cast<s64>(rotation[1][1]) * ir[1]);
  set_ir(1, mac[1], lm);
  set_ir(2, mac[2], lm);
  set_ir(3, mac[3], lm);
}

void Gte::cmd_gpf() {
  for (int i = 1; i <= 3; i++) {
    set_mac(i, static_cast<s64>(ir[0]) * ir[i]);
    set_ir(i, mac[i], lm);
  }
  u8 r = static_cast<u8>(clamp(mac[1] >> 4, 0, 0xFF, 1u << 21));
  u8 g = static_cast<u8>(clamp(mac[2] >> 4, 0, 0xFF, 1u << 20));
  u8 b = static_cast<u8>(clamp(mac[3] >> 4, 0, 0xFF, 1u << 19));
  push_rgb(r | (g << 8) | (b << 16) | (rgbc[3] << 24));
}

void Gte::cmd_gpl() {
  for (int i = 1; i <= 3; i++) {
    s64 result =
        (static_cast<s64>(mac[i]) << sf) + static_cast<s64>(ir[0]) * ir[i];
    set_mac(i, result);
    set_ir(i, mac[i], lm);
  }
  u8 r = static_cast<u8>(clamp(mac[1] >> 4, 0, 0xFF, 1u << 21));
  u8 g = static_cast<u8>(clamp(mac[2] >> 4, 0, 0xFF, 1u << 20));
  u8 b = static_cast<u8>(clamp(mac[3] >> 4, 0, 0xFF, 1u << 19));
  push_rgb(r | (g << 8) | (b << 16) | (rgbc[3] << 24));
}

void Gte::cmd_cc() {
  s16 ir_copy[3] = {ir[1], ir[2], ir[3]};
  for (int i = 0; i < 3; i++) {
    s64 result = static_cast<s64>(bg_color[i]) << 12;
    result += static_cast<s64>(color_matrix[i][0]) * ir_copy[0];
    result += static_cast<s64>(color_matrix[i][1]) * ir_copy[1];
    result += static_cast<s64>(color_matrix[i][2]) * ir_copy[2];
    set_mac(i + 1, result);
    set_ir(i + 1, mac[i + 1], lm);
  }
  for (int i = 0; i < 3; i++) {
    set_mac(i + 1, static_cast<s64>(rgbc[i]) * ir[i + 1] << 4);
    set_ir(i + 1, mac[i + 1], lm);
  }
  u8 r = static_cast<u8>(clamp(mac[1] >> 4, 0, 0xFF, 1u << 21));
  u8 g = static_cast<u8>(clamp(mac[2] >> 4, 0, 0xFF, 1u << 20));
  u8 b = static_cast<u8>(clamp(mac[3] >> 4, 0, 0xFF, 1u << 19));
  push_rgb(r | (g << 8) | (b << 16) | (rgbc[3] << 24));
}

