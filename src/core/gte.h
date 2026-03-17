#pragma once
#include "types.h"
#include <array>

// ── Geometry Transformation Engine (GTE / COP2) ────────────────────
// The GTE performs 3D math operations: perspective projection,
// lighting, color interpolation, and coordinate transformations.
// All operations use fixed-point arithmetic.

class Gte {
public:
  // Register access
  u32 read_data(u32 reg) const;
  u32 read_ctrl(u32 reg) const;
  void write_data(u32 reg, u32 value);
  void write_ctrl(u32 reg, u32 value);

  // Execute GTE command
  void execute(u32 command);

private:
  // ── Data Registers (COP2 data) ─────────────────────────────────
  // Vectors
  s16 v0[3] = {};   // V0 (input vector 0)
  s16 v1[3] = {};   // V1 (input vector 1)
  s16 v2[3] = {};   // V2 (input vector 2)
  u16 rgbc[4] = {}; // RGBC (color + code)

  // Output
  s16 ir[4] = {};             // IR0-IR3 (intermediate results)
  s16 sx[3] = {}, sy[3] = {}; // SXY0-SXY2 (screen XY FIFO)
  u16 sz[4] = {};             // SZ0-SZ3 (screen Z FIFO)
  u32 rgb_fifo[3] = {};       // RGB FIFO

  s32 mac[4] = {};        // MAC0-MAC3 (accumulator)
  u32 otz = 0;            // Average Z
  u32 lzcs = 0, lzcr = 0; // Leading zero count

  // ── Control Registers (COP2 control) ───────────────────────────
  s16 rotation[3][3] = {};     // Rotation matrix
  s32 translation[3] = {};     // Translation vector
  s16 light[3][3] = {};        // Light source matrix
  s32 bg_color[3] = {};        // Background color
  s16 color_matrix[3][3] = {}; // Color matrix
  s32 far_color[3] = {};       // Far color
  s32 ofx = 0, ofy = 0;        // Screen offset (16.16 fixed)
  u16 h = 0;                   // Projection plane distance
  s16 dqa = 0;                 // Depth queing coefficient
  s32 dqb = 0;                 // Depth queing offset
  s16 zsf3 = 0, zsf4 = 0;      // Average Z scale factors
  u32 flags = 0;               // GTE flags (overflow, etc.)

  bool lm = false; // Limiter flag for current command
  int sf = 0;      // Shift factor (0 or 12)
  u32 current_command_ = 0;

  // ── Commands ───────────────────────────────────────────────────
  void cmd_rtps(int v_idx, bool set_mac0); // Perspective transform (single)
  void cmd_rtpt();                         // Perspective transform (triple)
  void cmd_nclip();                        // Normal clipping
  void cmd_avsz3();                        // Average of 3 Z values
  void cmd_avsz4();                        // Average of 4 Z values
  void cmd_mvmva();     // Multiply vector by matrix + add vector
  void cmd_ncds(int v); // Normal color depth (single)
  void cmd_ncdt();      // Normal color depth (triple)
  void cmd_nccs(int v); // Normal color color (single)
  void cmd_ncct();      // Normal color color (triple)
  void cmd_ncs(int v);  // Normal color (single)
  void cmd_nct();       // Normal color (triple)
  void cmd_cdp();       // Color depth cue
  void cmd_dpcs();      // Depth cueing (single)
  void cmd_dcpl();      // Depth cueing of current color
  void cmd_dpct();      // Depth cueing (triple)
  void cmd_intpl();     // Interpolation
  void cmd_sqr();       // Square of vector
  void cmd_op();        // Outer product (cross product)
  void cmd_gpf();       // General purpose interpolation
  void cmd_gpl();       // General purpose interpolation with base
  void cmd_cc();        // Color color

  // ── Helpers ────────────────────────────────────────────────────
  s64 set_mac(int idx, s64 value);
  void set_ir(int idx, s32 value, bool lm_flag);
  void push_sx(s16 val);
  void push_sy(s16 val);
  void push_sz(u16 val);
  void push_rgb(u32 val);
  u32 divide(u16 h_val, u16 sz3);
  s32 clamp(s32 value, s32 min_val, s32 max_val, u32 flag_bit);
  int count_leading_bits(u32 value) const;
  int count_leading_zeros16(u16 value) const;
  void normal_color_stage(int v_idx);
  void push_rgb_from_mac();
  void apply_depth_cue_mac();
};
