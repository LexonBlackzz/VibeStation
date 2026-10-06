#pragma once
#include "types.h"
#include <vector>

// ── Hardware upscaler draw stream ──────────────────────────────────
// The software rasterizer stays authoritative for VRAM. When the OpenGL
// upscaler is enabled the GPU additionally records what it drew, in order,
// so the UI thread can replay it into a higher-resolution colour buffer
// that is only used for display.
//
// Textures are never re-derived on the GL side: whenever a textured draw
// samples a 64x256 VRAM page the software renderer changed since the last
// sync, the stream first carries a copy of that page (SyncPage). The GL side
// samples those native pages with exact integer lookups.

struct GpuHwVertex {
  float x = 0.0f; // VRAM pixel coordinates, draw offset applied, sub-pixel
  float y = 0.0f;
  float w = 1.0f; // PGXP depth for perspective; 1 when unknown (affine)
  u32 color = 0;  // r | g << 8 | b << 16
  float u = 0.0f; // texel coordinates (before window/wrap)
  float v = 0.0f;
  // Texel range the primitive covers (min u, min v, max u, max v); texture
  // filtering stays inside it so neighbouring atlas texels never bleed in.
  u8 uv_limits[4] = {0, 0, 255, 255};
};

enum class GpuHwOp : u8 {
  Triangles, // vertices [first, first + count)
  Fill,      // x, y, w, h, color (ignores draw area and mask)
  VramWrite, // x, y, w, h with pixels [first, first + w*h)
  VramCopy,  // src_x, src_y -> x, y, size w, h
  SyncPage,  // page (0..31) native pixels [first, first + 64*256)
  FullSync,  // all of VRAM, native pixels [first, first + 1024*512)
  Present,   // display rectangle x, y, w, h in VRAM pixels
};

namespace gpu_hw {
constexpr u8 kTextured = 1u << 0;
constexpr u8 kRawTexture = 1u << 1;
constexpr u8 kSemiTransparent = 1u << 2; // command requested blending
constexpr u8 kSetMask = 1u << 3;
constexpr u8 kCheckMask = 1u << 4;
constexpr u8 kSprite = 1u << 5;          // rectangle: texels map 1:1
constexpr u8 kSoftwarePresent = 1u << 6; // Present: show the software frame
constexpr u8 kDither = 1u << 7;          // PS1 dithering applies to this draw
constexpr int kPageWidth = 64;
constexpr int kPageHeight = 256;
constexpr int kPageColumns = 16;
constexpr int kPageCount = 32;
} // namespace gpu_hw

struct GpuHwCommand {
  GpuHwOp op = GpuHwOp::Triangles;
  u8 flags = 0;
  u8 semi_mode = 0; // 0: B/2+F/2, 1: B+F, 2: B-F, 3: B+F/4
  u8 tex_depth = 0; // 0: 4-bit CLUT, 1: 8-bit CLUT, 2: 15-bit direct
  u16 tex_base_x = 0;
  u16 tex_base_y = 0;
  u16 clut_x = 0;
  u16 clut_y = 0;
  u8 tw_mask_x = 0; // texture window, in texels (multiples of 8)
  u8 tw_mask_y = 0;
  u8 tw_off_x = 0;
  u8 tw_off_y = 0;
  s16 clip_x0 = 0; // draw area, inclusive
  s16 clip_y0 = 0;
  s16 clip_x1 = 0;
  s16 clip_y1 = 0;
  u32 first = 0;
  u32 count = 0;
  u16 x = 0;
  u16 y = 0;
  u16 w = 0;
  u16 h = 0;
  u16 src_x = 0;
  u16 src_y = 0;
  u32 color = 0;

  // Triangles with identical state are merged into one command.
  bool same_draw_state(const GpuHwCommand &o) const {
    return op == o.op && flags == o.flags && semi_mode == o.semi_mode &&
           tex_depth == o.tex_depth && tex_base_x == o.tex_base_x &&
           tex_base_y == o.tex_base_y && clut_x == o.clut_x &&
           clut_y == o.clut_y && tw_mask_x == o.tw_mask_x &&
           tw_mask_y == o.tw_mask_y && tw_off_x == o.tw_off_x &&
           tw_off_y == o.tw_off_y && clip_x0 == o.clip_x0 &&
           clip_y0 == o.clip_y0 && clip_x1 == o.clip_x1 && clip_y1 == o.clip_y1;
  }
};

struct GpuHwStream {
  std::vector<GpuHwCommand> commands;
  std::vector<GpuHwVertex> vertices;
  std::vector<u16> pixels;

  bool empty() const { return commands.empty(); }
  void clear() {
    commands.clear();
    vertices.clear();
    pixels.clear();
  }
  size_t byte_size() const {
    return commands.size() * sizeof(GpuHwCommand) +
           vertices.size() * sizeof(GpuHwVertex) + pixels.size() * sizeof(u16);
  }
  // Appends `other`, rebasing its vertex/pixel offsets.
  void append(const GpuHwStream &other) {
    const u32 vbase = static_cast<u32>(vertices.size());
    const u32 pbase = static_cast<u32>(pixels.size());
    vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
    pixels.insert(pixels.end(), other.pixels.begin(), other.pixels.end());
    for (GpuHwCommand c : other.commands) {
      if (c.op == GpuHwOp::Triangles) {
        c.first += vbase;
      } else if (c.op == GpuHwOp::VramWrite || c.op == GpuHwOp::SyncPage ||
                 c.op == GpuHwOp::FullSync) {
        c.first += pbase;
      }
      commands.push_back(c);
    }
  }
};
