#include "gpu.h"
#include "system.h"
#include <chrono>

namespace {
int clamp_display_dimension(int value, int fallback, int max_value) {
  const int candidate = (value > 0) ? value : fallback;
  return std::max(1, std::min(candidate, max_value));
}

int horizontal_divisor(u8 hres_mode) {
  static const int kDivisors[] = {10, 8, 5, 4, 7};
  if (hres_mode < 5) {
    return kDivisors[hres_mode];
  }
  return 10;
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
    return 4; // VRAM-to-VRAM copy
  case 0xA0:
    return 3; // CPU-to-VRAM (header, data follows)
  case 0xC0:
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
  gp0_buffer_.clear();
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
  dma_direction_ = 0;
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
  polyline_active_ = false;
  polyline_gouraud_ = false;
  polyline_waiting_vertex_ = false;
  polyline_prev_vertex_ = {};
  polyline_flat_color_ = {};
  polyline_prev_color_ = {};
  polyline_pending_color_word_ = 0;
}

// ── GP0 (Rendering Commands) ──────────────────────────────────────

void Gpu::gp0(u32 command) {
  const bool profile_detailed = g_profile_detailed_timing;
  std::chrono::high_resolution_clock::time_point start{};
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

  // Handle VRAM write mode
  if (gp0_mode_ == Gp0Mode::VramWrite) {
    // Write two 16-bit pixels per 32-bit word
    u16 pixel0 = command & 0xFFFF;
    u16 pixel1 = command >> 16;

    if (vram_tx_pos_ < vram_tx_total_) {
      u16 x = vram_tx_x_ + (vram_tx_pos_ % vram_tx_w_);
      u16 y = vram_tx_y_ + (vram_tx_pos_ / vram_tx_w_);
      if (x < psx::VRAM_WIDTH && y < psx::VRAM_HEIGHT) {
        vram_[y * psx::VRAM_WIDTH + x] = pixel0;
      }
      vram_tx_pos_++;
    }
    if (vram_tx_pos_ < vram_tx_total_) {
      u16 x = vram_tx_x_ + (vram_tx_pos_ % vram_tx_w_);
      u16 y = vram_tx_y_ + (vram_tx_pos_ / vram_tx_w_);
      if (x < psx::VRAM_WIDTH && y < psx::VRAM_HEIGHT) {
        vram_[y * psx::VRAM_WIDTH + x] = pixel1;
      }
      vram_tx_pos_++;
    }

    if (vram_tx_pos_ >= vram_tx_total_) {
      gp0_mode_ = Gp0Mode::Command;
    }
    return;
  }

  if (polyline_active_) {
    handle_polyline_word(command);
    return;
  }

  // Accumulate command words
  if (gp0_buffer_.empty()) {
    u8 opcode = (command >> 24) & 0xFF;
    u32 length = gp0_command_length(opcode);
    gp0_command_ = opcode;
    gp0_words_remaining_ = length;
  }

  gp0_buffer_.push_back(command);
  gp0_words_remaining_--;

  if (gp0_words_remaining_ > 0)
    return;

  // Full command received — dispatch
  u8 op = gp0_command_;
  // GP0 draw command bit1 selects semi-transparency for that command.
  semi_transparency_mode_ = (op >= 0x20 && op <= 0x7F) && ((op & 0x02u) != 0);
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
    gp0_image_load();
    break;
  case 0x80:
    gp0_vram_copy();
    break;
  case 0xC0:
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
    LOG_WARN("GPU: Unhandled GP0 command 0x%02X", op);
    break;
  }

  gp0_buffer_.clear();
  if (profile_detailed && sys_) {
    const auto end = std::chrono::high_resolution_clock::now();
    sys_->add_gpu_time(
        std::chrono::duration<double, std::milli>(end - start).count());
  }
}

// ── GP0 Command Implementations ────────────────────────────────────

void Gpu::gp0_nop() {}
void Gpu::gp0_clear_cache() {}

void Gpu::gp0_fill_rect() {
  Color c(gp0_buffer_[0]);
  u16 x = gp0_buffer_[1] & 0x3F0; // Rounded down to 16-pixel boundary
  u16 y = (gp0_buffer_[1] >> 16) & 0x1FF;
  u16 w = ((gp0_buffer_[2] & 0x3FF) + 0xF) & ~0xF; // Rounded up to 16 pixels
  u16 h = (gp0_buffer_[2] >> 16) & 0x1FF;

  u16 color15 = c.to_15bit();
  for (u16 dy = 0; dy < h; dy++) {
    for (u16 dx = 0; dx < w; dx++) {
      u16 px = (x + dx) % psx::VRAM_WIDTH;
      u16 py = (y + dy) % psx::VRAM_HEIGHT;
      vram_[py * psx::VRAM_WIDTH + px] = color15;
    }
  }
}

void Gpu::gp0_mono_tri() {
  Vertex v0, v1, v2;
  Color c(gp0_buffer_[0]);
  v0.x = static_cast<s16>(gp0_buffer_[1] & 0xFFFF) + draw_x_offset_;
  v0.y = static_cast<s16>(gp0_buffer_[1] >> 16) + draw_y_offset_;
  v1.x = static_cast<s16>(gp0_buffer_[2] & 0xFFFF) + draw_x_offset_;
  v1.y = static_cast<s16>(gp0_buffer_[2] >> 16) + draw_y_offset_;
  v2.x = static_cast<s16>(gp0_buffer_[3] & 0xFFFF) + draw_x_offset_;
  v2.y = static_cast<s16>(gp0_buffer_[3] >> 16) + draw_y_offset_;
  draw_flat_triangle(v0, v1, v2, c);
}

void Gpu::gp0_mono_quad() {
  Color c(gp0_buffer_[0]);
  Vertex v[4];
  for (int i = 0; i < 4; i++) {
    v[i].x = static_cast<s16>(gp0_buffer_[1 + i] & 0xFFFF) + draw_x_offset_;
    v[i].y = static_cast<s16>(gp0_buffer_[1 + i] >> 16) + draw_y_offset_;
  }
  draw_flat_triangle(v[0], v[1], v[2], c);
  draw_flat_triangle(v[1], v[2], v[3], c);
}

void Gpu::gp0_textured_tri() {
  Color c(gp0_buffer_[0]);
  clut_ = static_cast<u16>(gp0_buffer_[2] >> 16);
  texpage_ = static_cast<u16>(gp0_buffer_[4] >> 16);
  Vertex v[3];
  v[0].x = static_cast<s16>(gp0_buffer_[1] & 0xFFFF) + draw_x_offset_;
  v[0].y = static_cast<s16>(gp0_buffer_[1] >> 16) + draw_y_offset_;
  v[0].u = gp0_buffer_[2] & 0xFF;
  v[0].v = (gp0_buffer_[2] >> 8) & 0xFF;
  v[1].x = static_cast<s16>(gp0_buffer_[3] & 0xFFFF) + draw_x_offset_;
  v[1].y = static_cast<s16>(gp0_buffer_[3] >> 16) + draw_y_offset_;
  v[1].u = gp0_buffer_[4] & 0xFF;
  v[1].v = (gp0_buffer_[4] >> 8) & 0xFF;
  v[2].x = static_cast<s16>(gp0_buffer_[5] & 0xFFFF) + draw_x_offset_;
  v[2].y = static_cast<s16>(gp0_buffer_[5] >> 16) + draw_y_offset_;
  v[2].u = gp0_buffer_[6] & 0xFF;
  v[2].v = (gp0_buffer_[6] >> 8) & 0xFF;
  v[0].color = c;
  v[1].color = c;
  v[2].color = c;
  draw_textured_triangle(v[0], v[1], v[2], c);
}

void Gpu::gp0_textured_quad() {
  Color c(gp0_buffer_[0]);
  clut_ = static_cast<u16>(gp0_buffer_[2] >> 16);
  texpage_ = static_cast<u16>(gp0_buffer_[4] >> 16);
  Vertex v[4];
  for (int i = 0; i < 4; i++) {
    int base = 1 + i * 2;
    v[i].x = static_cast<s16>(gp0_buffer_[base] & 0xFFFF) + draw_x_offset_;
    v[i].y = static_cast<s16>(gp0_buffer_[base] >> 16) + draw_y_offset_;
    v[i].u = gp0_buffer_[base + 1] & 0xFF;
    v[i].v = (gp0_buffer_[base + 1] >> 8) & 0xFF;
    v[i].color = c;
  }
  draw_textured_triangle(v[0], v[1], v[2], c);
  draw_textured_triangle(v[1], v[2], v[3], c);
}

void Gpu::gp0_shaded_tri() {
  Vertex v[3];
  for (int i = 0; i < 3; i++) {
    v[i].color = Color(gp0_buffer_[i * 2]);
    v[i].x = static_cast<s16>(gp0_buffer_[i * 2 + 1] & 0xFFFF) + draw_x_offset_;
    v[i].y = static_cast<s16>(gp0_buffer_[i * 2 + 1] >> 16) + draw_y_offset_;
  }
  draw_shaded_triangle(v[0], v[1], v[2]);
}

void Gpu::gp0_shaded_quad() {
  Vertex v[4];
  for (int i = 0; i < 4; i++) {
    v[i].color = Color(gp0_buffer_[i * 2]);
    v[i].x = static_cast<s16>(gp0_buffer_[i * 2 + 1] & 0xFFFF) + draw_x_offset_;
    v[i].y = static_cast<s16>(gp0_buffer_[i * 2 + 1] >> 16) + draw_y_offset_;
  }
  draw_shaded_triangle(v[0], v[1], v[2]);
  draw_shaded_triangle(v[1], v[2], v[3]);
}

void Gpu::gp0_shaded_textured_tri() {
  Vertex v[3];
  clut_ = static_cast<u16>(gp0_buffer_[2] >> 16);
  texpage_ = static_cast<u16>(gp0_buffer_[5] >> 16);
  for (int i = 0; i < 3; i++) {
    int base = i * 3;
    v[i].color = Color(gp0_buffer_[base]);
    v[i].x = static_cast<s16>(gp0_buffer_[base + 1] & 0xFFFF) + draw_x_offset_;
    v[i].y = static_cast<s16>(gp0_buffer_[base + 1] >> 16) + draw_y_offset_;
    v[i].u = gp0_buffer_[base + 2] & 0xFF;
    v[i].v = (gp0_buffer_[base + 2] >> 8) & 0xFF;
  }
  draw_shaded_textured_triangle(v[0], v[1], v[2]);
}

void Gpu::gp0_shaded_textured_quad() {
  Vertex v[4];
  clut_ = static_cast<u16>(gp0_buffer_[2] >> 16);
  texpage_ = static_cast<u16>(gp0_buffer_[5] >> 16);
  for (int i = 0; i < 4; i++) {
    int base = i * 3;
    v[i].color = Color(gp0_buffer_[base]);
    v[i].x = static_cast<s16>(gp0_buffer_[base + 1] & 0xFFFF) + draw_x_offset_;
    v[i].y = static_cast<s16>(gp0_buffer_[base + 1] >> 16) + draw_y_offset_;
    v[i].u = gp0_buffer_[base + 2] & 0xFF;
    v[i].v = (gp0_buffer_[base + 2] >> 8) & 0xFF;
  }
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
  // Keep line shading simple for now: use color of first vertex.
  const Color c(gp0_buffer_[0]);
  const Vertex v0 = decode_vertex_word(gp0_buffer_[1]);
  const Vertex v1 = decode_vertex_word(gp0_buffer_[3]);
  draw_line_segment(v0, v1, c, semi_transparency_mode_);
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
  v.x = static_cast<s16>(word & 0xFFFF) + draw_x_offset_;
  v.y = static_cast<s16>(word >> 16) + draw_y_offset_;
  return v;
}

void Gpu::draw_line_segment(Vertex a, Vertex b, Color c, bool semi_transparent) {
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
  // Keep Gouraud polyline visual path simple: draw segment using prior color.
  draw_line_segment(polyline_prev_vertex_, next, polyline_prev_color_,
                    semi_transparency_mode_);
  polyline_prev_color_ = Color(polyline_pending_color_word_);
  polyline_prev_vertex_ = next;
  polyline_waiting_vertex_ = false;
}

void Gpu::gp0_mono_rect() {
  Color c(gp0_buffer_[0]);
  s16 x = static_cast<s16>(gp0_buffer_[1] & 0xFFFF) + draw_x_offset_;
  s16 y = static_cast<s16>(gp0_buffer_[1] >> 16) + draw_y_offset_;
  u16 w = gp0_buffer_[2] & 0xFFFF;
  u16 h = gp0_buffer_[2] >> 16;
  draw_rect(x, y, w, h, c);
}

void Gpu::gp0_textured_rect() {
  Color c(gp0_buffer_[0]);
  s16 x = static_cast<s16>(gp0_buffer_[1] & 0xFFFF) + draw_x_offset_;
  s16 y = static_cast<s16>(gp0_buffer_[1] >> 16) + draw_y_offset_;
  u8 u = gp0_buffer_[2] & 0xFF;
  u8 v = (gp0_buffer_[2] >> 8) & 0xFF;
  clut_ = static_cast<u16>(gp0_buffer_[2] >> 16);
  u16 w, h;
  u8 opcode = gp0_command_;
  if (opcode >= 0x74 && opcode <= 0x77) {
    w = 8;
    h = 8;
  } else if (opcode >= 0x7C && opcode <= 0x7F) {
    w = 16;
    h = 16;
  } else if (gp0_buffer_.size() >= 4) {
    w = gp0_buffer_[3] & 0xFFFF;
    h = gp0_buffer_[3] >> 16;
  } else {
    w = 1;
    h = 1;
  }

  const bool raw_texture = (gp0_command_ & 0x1u) != 0;
  // Textured rectangle draw path (supports 4/8/15-bit tex fetch via
  // read_texel).
  for (u16 dy = 0; dy < h; ++dy) {
    for (u16 dx = 0; dx < w; ++dx) {
      const u16 src_dx = tex_rect_x_flip_ ? static_cast<u16>(w - 1 - dx) : dx;
      const u16 src_dy = tex_rect_y_flip_ ? static_cast<u16>(h - 1 - dy) : dy;
      const u8 src_u = static_cast<u8>(u + src_dx);
      const u8 src_v = static_cast<u8>(v + src_dy);
      u16 texel = read_texel(src_u, src_v);
      if ((texel & 0x7FFF) == 0) {
        continue; // Color 0 transparent in many textured modes.
      }

      const u16 out15 =
          raw_texture ? texel : modulate_texel_15bit(texel, c.r, c.g, c.b);
      const bool texel_semi = semi_transparency_mode_ && ((texel & 0x8000u) != 0);
      set_pixel(static_cast<s16>(x + dx), static_cast<s16>(y + dy), out15,
                texel_semi);
    }
  }
}

void Gpu::gp0_mono_rect_1() {
  Color c(gp0_buffer_[0]);
  s16 x = static_cast<s16>(gp0_buffer_[1] & 0xFFFF) + draw_x_offset_;
  s16 y = static_cast<s16>(gp0_buffer_[1] >> 16) + draw_y_offset_;
  draw_rect(x, y, 1, 1, c);
}

void Gpu::gp0_mono_rect_8() {
  Color c(gp0_buffer_[0]);
  s16 x = static_cast<s16>(gp0_buffer_[1] & 0xFFFF) + draw_x_offset_;
  s16 y = static_cast<s16>(gp0_buffer_[1] >> 16) + draw_y_offset_;
  draw_rect(x, y, 8, 8, c);
}

void Gpu::gp0_mono_rect_16() {
  Color c(gp0_buffer_[0]);
  s16 x = static_cast<s16>(gp0_buffer_[1] & 0xFFFF) + draw_x_offset_;
  s16 y = static_cast<s16>(gp0_buffer_[1] >> 16) + draw_y_offset_;
  draw_rect(x, y, 16, 16, c);
}

void Gpu::gp0_draw_mode() {
  u32 val = gp0_buffer_[0];
  texpage_ = static_cast<u16>(val & 0x1FF);
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
  vram_tx_w_ = gp0_buffer_[2] & 0xFFFF;
  vram_tx_h_ = gp0_buffer_[2] >> 16;

  if (vram_tx_w_ == 0)
    vram_tx_w_ = 1024;
  if (vram_tx_h_ == 0)
    vram_tx_h_ = 512;

  vram_tx_pos_ = 0;
  vram_tx_total_ = static_cast<u32>(vram_tx_w_) * vram_tx_h_;

  gp0_mode_ = Gp0Mode::VramWrite;
}

void Gpu::gp0_image_store() {
  // VRAM → CPU transfer
  vram_tx_x_ = gp0_buffer_[1] & 0x3FF;
  vram_tx_y_ = (gp0_buffer_[1] >> 16) & 0x1FF;
  vram_tx_w_ = gp0_buffer_[2] & 0xFFFF;
  vram_tx_h_ = gp0_buffer_[2] >> 16;

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

  // Copy through a temp line buffer to handle overlaps safely.
  std::vector<u16> tmp(static_cast<size_t>(w) * h);
  for (u16 y = 0; y < h; ++y) {
    for (u16 x = 0; x < w; ++x) {
      u16 sx = static_cast<u16>((src_x + x) & (psx::VRAM_WIDTH - 1));
      u16 sy = static_cast<u16>((src_y + y) & (psx::VRAM_HEIGHT - 1));
      tmp[static_cast<size_t>(y) * w + x] = vram_[sy * psx::VRAM_WIDTH + sx];
    }
  }
  for (u16 y = 0; y < h; ++y) {
    for (u16 x = 0; x < w; ++x) {
      u16 dx = static_cast<u16>((dst_x + x) & (psx::VRAM_WIDTH - 1));
      u16 dy = static_cast<u16>((dst_y + y) & (psx::VRAM_HEIGHT - 1));
      vram_[dy * psx::VRAM_WIDTH + dx] = tmp[static_cast<size_t>(y) * w + x];
    }
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
    LOG_WARN("GPU: Unhandled GP1 command 0x%02X", op);
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
  gp0_buffer_.clear();
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
  display_.x_start = val & 0x3FE; // 10 bits, aligned to 2
  display_.y_start = (val >> 10) & 0x1FF;
}

void Gpu::gp1_horizontal_range(u32 val) {
  display_.x1 = val & 0xFFF;
  display_.x2 = (val >> 12) & 0xFFF;
}

void Gpu::gp1_vertical_range(u32 val) {
  display_.y1 = val & 0x3FF;
  display_.y2 = (val >> 10) & 0x3FF;
}

void Gpu::gp1_display_mode(u32 val) {
  u8 hres1 = val & 3;
  u8 hres2 = (val >> 6) & 1;
  display_.hres = hres2 ? 4 : hres1;
  display_.vres = (val >> 2) & 1;
  display_.is_pal = (val >> 3) & 1;
  display_.is_24bit = (val >> 4) & 1;
  display_.interlaced = (val >> 5) & 1;
}

void Gpu::gp1_get_info(u32 val) {
  // GP1(10h..1Fh): GPU info command; low nibble selects info register.
  gpuread_latch_ = gp1_info_value(val & 0x0Fu);
}

u32 Gpu::gp1_info_value(u32 index) const {
  switch (index & 0x0Fu) {
  case 0x00: { // Draw mode setting
    u32 value = texpage_ & 0x1FFu;
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
    u16 x = vram_tx_x_ + (vram_tx_pos_ % vram_tx_w_);
    u16 y = vram_tx_y_ + (vram_tx_pos_ / vram_tx_w_);
    if (x < psx::VRAM_WIDTH && y < psx::VRAM_HEIGHT)
      p0 = vram_[y * psx::VRAM_WIDTH + x];
    vram_tx_pos_++;

    if (vram_tx_pos_ < vram_tx_total_) {
      x = vram_tx_x_ + (vram_tx_pos_ % vram_tx_w_);
      y = vram_tx_y_ + (vram_tx_pos_ / vram_tx_w_);
      if (x < psx::VRAM_WIDTH && y < psx::VRAM_HEIGHT)
        p1 = vram_[y * psx::VRAM_WIDTH + x];
      vram_tx_pos_++;
    }

    if (vram_tx_pos_ >= vram_tx_total_) {
      gp0_mode_ = Gp0Mode::Command;
    }

    return static_cast<u32>(p0) | (static_cast<u32>(p1) << 16);
  }
  return gpuread_latch_;
}

u32 Gpu::read_stat() const {
  const bool cmd_ready = (gp0_mode_ == Gp0Mode::Command);
  const bool vram_read_ready = (gp0_mode_ != Gp0Mode::VramWrite);
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
  stat |= (force_set_mask_bit_ ? 1u : 0u) << 11;
  stat |= (check_mask_before_draw_ ? 1u : 0u) << 12;
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
  stat |= (cmd_ready ? 1u : 0u) << 26;
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
  const bool vram_read_ready = (gp0_mode_ != Gp0Mode::VramWrite);
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

DisplaySampleInfo Gpu::build_display_rgba(std::vector<u32> *rgba) const {
  DisplaySampleInfo info{};
  info.display_enabled = display_.display_enabled;
  info.is_24bit = display_.is_24bit;
  info.x_start = static_cast<int>(display_.x_start);
  info.y_start = static_cast<int>(display_.y_start);

  const int mode_w = display_.width();
  const int mode_h = display_.height();

  // Safe baseline: mode-derived size is authoritative unless range values are
  // close enough to be trustworthy for the active mode.
  int width = clamp_display_dimension(mode_w, 320, psx::VRAM_WIDTH);
  int height = clamp_display_dimension(mode_h, 240, psx::VRAM_HEIGHT);

  if (display_.x2 > display_.x1) {
    const int dots = static_cast<int>(display_.x2 - display_.x1);
    const int divisor = horizontal_divisor(display_.hres);
    if (divisor > 0) {
      const int candidate = dots / divisor;
      const int min_ok = std::max(1, (mode_w * 3) / 4);
      const int max_ok = std::max(min_ok, (mode_w * 5) / 4);
      if (candidate >= min_ok && candidate <= max_ok) {
        width = clamp_display_dimension(candidate, width, psx::VRAM_WIDTH);
      }
    }
  }
  if (display_.y2 > display_.y1) {
    const int candidate = static_cast<int>(display_.y2 - display_.y1);
    const int min_ok = std::max(1, (mode_h * 3) / 4);
    const int max_ok = std::max(min_ok, (mode_h * 5) / 4);
    if (candidate >= min_ok && candidate <= max_ok) {
      height = clamp_display_dimension(candidate, height, psx::VRAM_HEIGHT);
    }
  }

  info.width = width;
  info.height = height;

  if (rgba != nullptr) {
    rgba->assign(static_cast<size_t>(info.width) * static_cast<size_t>(info.height),
                 0xFF000000u);
  }
  if (!info.display_enabled) {
    return info;
  }

  const int row_bytes = static_cast<int>(psx::VRAM_WIDTH * sizeof(u16));
  u32 hash = 2166136261u;
  u64 non_black = 0;
  const bool interlaced_field_output =
      display_.interlaced && (display_.vres != 0);
  const int field_parity = interlaced_field_output
                               ? (interlace_field_ ? 1 : 0)
                               : 0;
  const DeinterlaceMode deinterlace_mode =
      interlaced_field_output ? g_deinterlace_mode : DeinterlaceMode::Weave;

  auto sample_byte = [&](int vram_y, int byte_index) -> u8 {
    if (vram_y < 0 || vram_y >= static_cast<int>(psx::VRAM_HEIGHT) ||
        byte_index < 0 || byte_index >= row_bytes) {
      return 0;
    }
    const int word_index = byte_index >> 1;
    const u16 word =
        vram_[static_cast<size_t>(vram_y) * psx::VRAM_WIDTH + word_index];
    return (byte_index & 1) ? static_cast<u8>(word >> 8)
                            : static_cast<u8>(word & 0xFF);
  };

  auto read_rgb = [&](int vram_y, int x, u8 &r, u8 &g, u8 &b) -> bool {
    if (vram_y < 0 || vram_y >= static_cast<int>(psx::VRAM_HEIGHT)) {
      return false;
    }
    const int vram_x = info.x_start + x;
    if (vram_x < 0 || vram_x >= static_cast<int>(psx::VRAM_WIDTH)) {
      return false;
    }

    if (!display_.is_24bit) {
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

    const int row_start_byte = info.x_start * 2;
    const int byte_index = row_start_byte + x * 3;
    r = sample_byte(vram_y, byte_index + 0);
    g = sample_byte(vram_y, byte_index + 1);
    b = sample_byte(vram_y, byte_index + 2);
    return true;
  };

  for (int y = 0; y < info.height; ++y) {
    int vram_y0 = info.y_start + y;
    int vram_y1 = vram_y0;
    bool blend_fields = false;

    if (interlaced_field_output) {
      switch (deinterlace_mode) {
      case DeinterlaceMode::Bob:
        vram_y0 = info.y_start + ((y >> 1) * 2) + field_parity;
        break;
      case DeinterlaceMode::Blend:
        vram_y0 = info.y_start + ((y >> 1) * 2) + field_parity;
        vram_y1 = vram_y0 + (field_parity ? -1 : 1);
        blend_fields = true;
        break;
      case DeinterlaceMode::Weave:
      default:
        // Stable placement: map output lines directly to source scanlines.
        vram_y0 = info.y_start + y;
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
    for (int x = 0; x < info.width; ++x) {
      u8 r = 0;
      u8 g = 0;
      u8 b = 0;
      if (!read_rgb(vram_y0, x, r, g, b)) {
        continue;
      }
      if (blend_fields) {
        u8 r1 = 0;
        u8 g1 = 0;
        u8 b1 = 0;
        if (read_rgb(vram_y1, x, r1, g1, b1)) {
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

  info.non_black_pixels = non_black;
  info.hash = hash;
  return info;
}

void Gpu::vblank() {
  static u64 vblank_count = 0;
  frame_complete_ = true;
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

  const size_t index = static_cast<size_t>(y) * psx::VRAM_WIDTH + x;
  const u16 dst = vram_[index];
  if (check_mask_before_draw_ && (dst & 0x8000u)) {
    return;
  }

  u16 out = color;
  if (semi_transparent) {
    const int fr = color & 0x1F;
    const int fg = (color >> 5) & 0x1F;
    const int fb = (color >> 10) & 0x1F;
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
    out = static_cast<u16>((rr & 0x1F) | ((rg & 0x1F) << 5) |
                           ((rb & 0x1F) << 10) | (color & 0x8000u));
  }

  if (force_set_mask_bit_) {
    out |= 0x8000u;
  }
  vram_[index] = out;
}

u16 Gpu::read_texel(u8 u, u8 v) const {
  const u16 uw = static_cast<u16>((static_cast<u16>(u) & ~tex_window_mask_x_) |
                                  (tex_window_off_x_ & tex_window_mask_x_));
  const u16 vw = static_cast<u16>((static_cast<u16>(v) & ~tex_window_mask_y_) |
                                  (tex_window_off_y_ & tex_window_mask_y_));

  const u16 tex_base_x = static_cast<u16>((texpage_ & 0xF) * 64);
  const u16 tex_base_y = static_cast<u16>(((texpage_ >> 4) & 1) * 256);
  const u16 clut_x = static_cast<u16>((clut_ & 0x3F) * 16);
  const u16 clut_y = static_cast<u16>((clut_ >> 6) & 0x1FF);
  const u8 depth = static_cast<u8>((texpage_ >> 7) & 0x3);

  const u16 tx = static_cast<u16>((tex_base_x + uw) & (psx::VRAM_WIDTH - 1));
  const u16 ty = static_cast<u16>((tex_base_y + vw) & (psx::VRAM_HEIGHT - 1));

  switch (depth) {
  case 0: { // 4-bit indexed
    const u16 word_x =
        static_cast<u16>((tex_base_x + (uw >> 2)) & (psx::VRAM_WIDTH - 1));
    const u16 packed = vram_[ty * psx::VRAM_WIDTH + word_x];
    const u16 index = static_cast<u16>((packed >> ((uw & 3) * 4)) & 0xF);
    const u16 cx = static_cast<u16>((clut_x + index) & (psx::VRAM_WIDTH - 1));
    return vram_[clut_y * psx::VRAM_WIDTH + cx];
  }
  case 1: { // 8-bit indexed
    const u16 word_x =
        static_cast<u16>((tex_base_x + (uw >> 1)) & (psx::VRAM_WIDTH - 1));
    const u16 packed = vram_[ty * psx::VRAM_WIDTH + word_x];
    const u16 index = static_cast<u16>((packed >> ((uw & 1) * 8)) & 0xFF);
    const u16 cx = static_cast<u16>((clut_x + index) & (psx::VRAM_WIDTH - 1));
    return vram_[clut_y * psx::VRAM_WIDTH + cx];
  }
  case 2: // 15-bit direct
    return vram_[ty * psx::VRAM_WIDTH + tx];
  default:
    return 0;
  }
}

void Gpu::draw_flat_triangle(Vertex v0, Vertex v1, Vertex v2, Color c) {
  u16 color15 = c.to_15bit();

  // Bounding box
  s16 min_x = std::min({v0.x, v1.x, v2.x});
  s16 max_x = std::max({v0.x, v1.x, v2.x});
  s16 min_y = std::min({v0.y, v1.y, v2.y});
  s16 max_y = std::max({v0.y, v1.y, v2.y});

  // Clamp to drawing area
  min_x = std::max(min_x, draw_x_min_);
  max_x = std::min(max_x, draw_x_max_);
  min_y = std::max(min_y, draw_y_min_);
  max_y = std::min(max_y, draw_y_max_);

  // Limit to reasonable size to avoid hangs on degenerate geometry
  if (max_x - min_x > 1023 || max_y - min_y > 511)
    return;

  for (s16 y = min_y; y <= max_y; y++) {
    for (s16 x = min_x; x <= max_x; x++) {
      s32 w0 = edge(v1, v2, x, y);
      s32 w1 = edge(v2, v0, x, y);
      s32 w2 = edge(v0, v1, x, y);
      if ((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0)) {
        set_pixel(x, y, color15, semi_transparency_mode_);
      }
    }
  }
}

void Gpu::draw_shaded_triangle(Vertex v0, Vertex v1, Vertex v2) {
  s16 min_x = std::min({v0.x, v1.x, v2.x});
  s16 max_x = std::max({v0.x, v1.x, v2.x});
  s16 min_y = std::min({v0.y, v1.y, v2.y});
  s16 max_y = std::max({v0.y, v1.y, v2.y});
  min_x = std::max(min_x, draw_x_min_);
  max_x = std::min(max_x, draw_x_max_);
  min_y = std::max(min_y, draw_y_min_);
  max_y = std::min(max_y, draw_y_max_);
  if (max_x - min_x > 1023 || max_y - min_y > 511)
    return;

  s32 area = edge(v0, v1, v2.x, v2.y);
  if (area == 0)
    return;

  for (s16 y = min_y; y <= max_y; y++) {
    for (s16 x = min_x; x <= max_x; x++) {
      s32 w0 = edge(v1, v2, x, y);
      s32 w1 = edge(v2, v0, x, y);
      s32 w2 = edge(v0, v1, x, y);
      if ((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0)) {
        // Interpolate color
        u8 r = static_cast<u8>(
            (w0 * v0.color.r + w1 * v1.color.r + w2 * v2.color.r) / area);
        u8 g = static_cast<u8>(
            (w0 * v0.color.g + w1 * v1.color.g + w2 * v2.color.g) / area);
        u8 b = static_cast<u8>(
            (w0 * v0.color.b + w1 * v1.color.b + w2 * v2.color.b) / area);
        Color c(r, g, b);
        set_pixel(x, y, c.to_15bit(), semi_transparency_mode_);
      }
    }
  }
}

void Gpu::draw_textured_triangle(Vertex v0, Vertex v1, Vertex v2, Color /*c*/) {
  s16 min_x = std::min({v0.x, v1.x, v2.x});
  s16 max_x = std::max({v0.x, v1.x, v2.x});
  s16 min_y = std::min({v0.y, v1.y, v2.y});
  s16 max_y = std::max({v0.y, v1.y, v2.y});
  min_x = std::max(min_x, draw_x_min_);
  max_x = std::min(max_x, draw_x_max_);
  min_y = std::max(min_y, draw_y_min_);
  max_y = std::min(max_y, draw_y_max_);
  if (max_x - min_x > 1023 || max_y - min_y > 511)
    return;

  s32 area = edge(v0, v1, v2.x, v2.y);
  if (area == 0)
    return;

  const bool raw_texture = (gp0_command_ & 0x1u) != 0;
  const u8 mr = v0.color.r;
  const u8 mg = v0.color.g;
  const u8 mb = v0.color.b;

  for (s16 y = min_y; y <= max_y; y++) {
    for (s16 x = min_x; x <= max_x; x++) {
      s32 w0 = edge(v1, v2, x, y);
      s32 w1 = edge(v2, v0, x, y);
      s32 w2 = edge(v0, v1, x, y);
      if ((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0)) {
        u8 u = static_cast<u8>((w0 * v0.u + w1 * v1.u + w2 * v2.u) / area);
        u8 v_coord =
            static_cast<u8>((w0 * v0.v + w1 * v1.v + w2 * v2.v) / area);
        const u16 texel = read_texel(u, v_coord);
        if ((texel & 0x7FFF) != 0) { // Transparent texel index/color 0.
          const u16 out15 =
              raw_texture ? texel : modulate_texel_15bit(texel, mr, mg, mb);
          const bool texel_semi =
              semi_transparency_mode_ && ((texel & 0x8000u) != 0);
          set_pixel(x, y, out15, texel_semi);
        }
      }
    }
  }
}

void Gpu::draw_shaded_textured_triangle(Vertex v0, Vertex v1, Vertex v2) {
  s16 min_x = std::min({v0.x, v1.x, v2.x});
  s16 max_x = std::max({v0.x, v1.x, v2.x});
  s16 min_y = std::min({v0.y, v1.y, v2.y});
  s16 max_y = std::max({v0.y, v1.y, v2.y});
  min_x = std::max(min_x, draw_x_min_);
  max_x = std::min(max_x, draw_x_max_);
  min_y = std::max(min_y, draw_y_min_);
  max_y = std::min(max_y, draw_y_max_);
  if (max_x - min_x > 1023 || max_y - min_y > 511)
    return;

  const s32 area = edge(v0, v1, v2.x, v2.y);
  if (area == 0)
    return;

  const bool raw_texture = (gp0_command_ & 0x1u) != 0;

  for (s16 y = min_y; y <= max_y; ++y) {
    for (s16 x = min_x; x <= max_x; ++x) {
      const s32 w0 = edge(v1, v2, x, y);
      const s32 w1 = edge(v2, v0, x, y);
      const s32 w2 = edge(v0, v1, x, y);
      if (!((w0 >= 0 && w1 >= 0 && w2 >= 0) ||
            (w0 <= 0 && w1 <= 0 && w2 <= 0))) {
        continue;
      }

      const u8 u = static_cast<u8>((w0 * v0.u + w1 * v1.u + w2 * v2.u) / area);
      const u8 v_coord =
          static_cast<u8>((w0 * v0.v + w1 * v1.v + w2 * v2.v) / area);
      const u16 texel = read_texel(u, v_coord);
      if ((texel & 0x7FFF) == 0) {
        continue;
      }

      const s32 r_mix = (w0 * v0.color.r + w1 * v1.color.r + w2 * v2.color.r) / area;
      const s32 g_mix = (w0 * v0.color.g + w1 * v1.color.g + w2 * v2.color.g) / area;
      const s32 b_mix = (w0 * v0.color.b + w1 * v1.color.b + w2 * v2.color.b) / area;
      const u8 mr = static_cast<u8>(std::clamp(r_mix, 0, 255));
      const u8 mg = static_cast<u8>(std::clamp(g_mix, 0, 255));
      const u8 mb = static_cast<u8>(std::clamp(b_mix, 0, 255));

      const u16 out15 =
          raw_texture ? texel : modulate_texel_15bit(texel, mr, mg, mb);
      const bool texel_semi = semi_transparency_mode_ && ((texel & 0x8000u) != 0);
      set_pixel(x, y, out15, texel_semi);
    }
  }
}

void Gpu::draw_rect(s16 x, s16 y, u16 w, u16 h, Color c) {
  u16 color15 = c.to_15bit();
  for (u16 dy = 0; dy < h; dy++) {
    for (u16 dx = 0; dx < w; dx++) {
      set_pixel(x + dx, y + dy, color15, semi_transparency_mode_);
    }
  }
}
