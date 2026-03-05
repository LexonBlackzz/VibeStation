#pragma once
#include "types.h"
#include <array>
#include <vector>

// ── Graphics Processing Unit ───────────────────────────────────────
// The PS1 GPU handles 2D rendering with 3D-accelerated primitives.
// VRAM is 1024x512 pixels at 16bpp (1MB).

class System;

// Color type
struct Color {
  u8 r, g, b;
  Color() : r(0), g(0), b(0) {}
  Color(u8 r, u8 g, u8 b) : r(r), g(g), b(b) {}
  Color(u32 val) : r(val & 0xFF), g((val >> 8) & 0xFF), b((val >> 16) & 0xFF) {}
  u16 to_15bit() const {
    return (((u16)r >> 3) | (((u16)g >> 3) << 5) | (((u16)b >> 3) << 10));
  }
};

// Vertex type
struct Vertex {
  s16 x, y;
  float fx, fy;
  Color color;
  u8 u, v; // Texture coordinates
  Vertex() : x(0), y(0), fx(0.0f), fy(0.0f), u(0), v(0) {}
};

// Display mode
struct DisplayMode {
  u16 x_start = 0, y_start = 0; // Display area start in VRAM
  u16 x1 = 0, x2 = 0;           // Horizontal display range
  u16 y1 = 0, y2 = 0;           // Vertical display range
  u8 hres = 0;                  // Horizontal resolution
  u8 vres = 0;                  // Vertical resolution (0=240, 1=480)
  bool is_pal = false;
  bool is_24bit = false;
  bool interlaced = false;
  bool display_enabled = true;

  int width() const {
    static const int widths[] = {256, 320, 512, 640, 368};
    return (hres < 5) ? widths[hres] : 256;
  }
  int height() const { return vres ? 480 : 240; }
};

struct DisplaySampleInfo {
  int width = 0;
  int height = 0;
  int x_start = 0;
  int y_start = 0;
  bool display_enabled = false;
  bool is_24bit = false;
  u64 non_black_pixels = 0;
  u32 hash = 2166136261u;
};

class Gpu {
public:
  void init(System *sys) { sys_ = sys; }
  void reset();

  // GP0 (Rendering commands) and GP1 (Display control)
  void gp0(u32 command);
  void gp1(u32 command);

  // Read from GPU (GPUREAD port + GPUSTAT)
  u32 read_data();
  u32 read_stat() const;
  bool dma_request() const;

  // VRAM access
  const u16 *vram() const { return vram_.data(); }
  const DisplayMode &display_mode() const { return display_; }
  DisplaySampleInfo build_display_rgba(std::vector<u32> *rgba,
                                       bool include_stats = true) const;
  DisplaySampleInfo build_display_rgba(std::vector<u32> &rgba,
                                       bool include_stats = true) const {
    return build_display_rgba(&rgba, include_stats);
  }

  // Frame tracking
  bool frame_complete() const { return frame_complete_; }
  void clear_frame_flag() { frame_complete_ = false; }

  // VBlank
  void vblank();

  System *sys() const { return sys_; }

private:
  System *sys_ = nullptr;

  // 1MB VRAM: 1024 x 512 x 16bpp
  std::array<u16, psx::VRAM_WIDTH * psx::VRAM_HEIGHT> vram_{};

  // GP0 command buffer (commands can span multiple words)
  std::vector<u32> gp0_buffer_;
  std::vector<u16> vram_copy_buffer_;
  u32 gp0_words_remaining_ = 0;
  u32 gp0_command_ = 0;
  enum class Gp0Mode { Command, VramWrite, VramRead };
  Gp0Mode gp0_mode_ = Gp0Mode::Command;

  // VRAM transfer state
  u16 vram_tx_x_ = 0, vram_tx_y_ = 0;
  u16 vram_tx_w_ = 0, vram_tx_h_ = 0;
  u32 vram_tx_pos_ = 0;
  u32 vram_tx_total_ = 0;

  // Drawing area
  s16 draw_x_min_ = 0, draw_y_min_ = 0;
  s16 draw_x_max_ = 0, draw_y_max_ = 0;
  s16 draw_x_offset_ = 0, draw_y_offset_ = 0;

  // Texture settings
  u16 texpage_ = 0;
  u16 clut_ = 0;
  u16 tex_window_mask_x_ = 0, tex_window_mask_y_ = 0;
  u16 tex_window_off_x_ = 0, tex_window_off_y_ = 0;

  // Status register fields
  DisplayMode display_;
  u8 dma_direction_ = 0;
  bool dither_enabled_ = false;
  bool draw_to_display_ = false;
  bool texture_disable_ = false;
  bool semi_transparency_mode_ = false;
  u8 semi_transparency_ = 0;
  bool tex_rect_x_flip_ = false;
  bool tex_rect_y_flip_ = false;
  bool irq1_pending_ = false;
  bool force_set_mask_bit_ = false;
  bool check_mask_before_draw_ = false;
  bool interlace_field_ = false;
  u32 gpuread_latch_ = 0;

  bool frame_complete_ = false;

  // ── GP0 Command Handlers ───────────────────────────────────────
  void gp0_nop();
  void gp0_clear_cache();
  void gp0_fill_rect();
  void gp0_mono_quad();
  void gp0_textured_quad();
  void gp0_shaded_tri();
  void gp0_shaded_quad();
  void gp0_mono_tri();
  void gp0_mono_rect();
  void gp0_textured_rect();
  void gp0_mono_rect_1();
  void gp0_mono_rect_8();
  void gp0_mono_rect_16();
  void gp0_mono_line();
  void gp0_mono_polyline_start();
  void gp0_shaded_line();
  void gp0_shaded_polyline_start();
  void gp0_draw_mode();
  void gp0_tex_window();
  void gp0_draw_area_top_left();
  void gp0_draw_area_bottom_right();
  void gp0_draw_offset();
  void gp0_mask_bit();
  void gp0_irq_request();
  void gp0_image_load();
  void gp0_image_store();
  void gp0_vram_copy();
  void gp0_textured_tri();
  void gp0_shaded_textured_tri();
  void gp0_shaded_textured_quad();

  // ── GP1 Command Handlers ───────────────────────────────────────
  void gp1_reset();
  void gp1_reset_command_buffer();
  void gp1_ack_irq();
  void gp1_display_enable(u32 val);
  void gp1_dma_direction(u32 val);
  void gp1_display_area(u32 val);
  void gp1_horizontal_range(u32 val);
  void gp1_vertical_range(u32 val);
  void gp1_display_mode(u32 val);
  void gp1_get_info(u32 val);
  u32 gp1_info_value(u32 index) const;

  // ── Rasterization ──────────────────────────────────────────────
  void draw_flat_triangle(Vertex v0, Vertex v1, Vertex v2, Color c);
  void draw_shaded_triangle(Vertex v0, Vertex v1, Vertex v2);
  void draw_textured_triangle(Vertex v0, Vertex v1, Vertex v2, Color c);
  void draw_shaded_textured_triangle(Vertex v0, Vertex v1, Vertex v2);
  void draw_rect(s16 x, s16 y, u16 w, u16 h, Color c);
  void draw_line_segment(Vertex a, Vertex b, Color c, bool semi_transparent);

  void set_pixel(s16 x, s16 y, u16 color, bool semi_transparent = false);
  void set_pixel_clipped(s16 x, s16 y, u16 color,
                         bool semi_transparent = false);
  void write_pixel_opaque_clipped(s16 x, s16 y, u16 color);
  u16 read_texel(u8 u, u8 v) const;
  static bool is_polyline_terminator(u32 word) {
    return (word & 0xF000F000u) == 0x50005000u;
  }
  Vertex decode_vertex_word(u32 word) const;
  void handle_polyline_word(u32 word);

  // Edge function for triangle rasterization
  static s32 edge(const Vertex &a, const Vertex &b, s16 px, s16 py) {
    return static_cast<s32>(b.x - a.x) * (py - a.y) -
           static_cast<s32>(b.y - a.y) * (px - a.x);
  }

  // Lookup table for GP0 command lengths
  static u32 gp0_command_length(u8 opcode);

  // Variable-length polyline stream state.
  bool polyline_active_ = false;
  bool polyline_gouraud_ = false;
  bool polyline_waiting_vertex_ = false;
  Vertex polyline_prev_vertex_{};
  Color polyline_flat_color_{};
  Color polyline_prev_color_{};
  u32 polyline_pending_color_word_ = 0;
};
