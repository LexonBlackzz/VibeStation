#pragma once

#include "core/types.h"

#include <cstddef>
#include <string>
#include <vector>

struct SDL_Window;
typedef void* SDL_GLContext;

// Experimental OpenGL 4.3 compute rasterizer for the PS1 GPU.
//
// The backend owns an independent hidden GL context and a GPU-resident mirror
// of the PS1's 1024x512 16-bit VRAM.  It is deliberately separated from the
// presentation Renderer: emulation can enqueue raster work from EmuRunner's
// thread without borrowing the UI/OpenGL context.
class GpuHardwareRasterizer {
public:
  enum class TriangleMode : int {
    Flat = 0,
    Gouraud = 1,
    Textured = 2,
    GouraudTextured = 3,
  };

  struct Vertex {
    s16 x = 0;
    s16 y = 0;
    u8 u = 0;
    u8 v = 0;
    u8 r = 0;
    u8 g = 0;
    u8 b = 0;
  };

  struct TextureState {
    u8 keep_x = 0xFF;
    u8 keep_y = 0xFF;
    u8 replace_x = 0;
    u8 replace_y = 0;
    u8 depth = 0;
    u16 tex_base_x = 0;
    u16 tex_base_y = 0;
    u16 clut_x = 0;
    u32 clut_row = 0;
  };

  struct RasterState {
    s16 draw_x_min = 0;
    s16 draw_y_min = 0;
    s16 draw_x_max = 0;
    s16 draw_y_max = 0;
    bool dither = false;
    bool semi_transparent = false;
    u8 semi_mode = 0;
    bool force_set_mask_bit = false;
    bool check_mask_before_draw = false;
    bool raw_texture = false;
    bool rect_x_flip = false;
    bool rect_y_flip = false;
    TextureState texture{};
  };

  GpuHardwareRasterizer() = default;
  ~GpuHardwareRasterizer();

  GpuHardwareRasterizer(const GpuHardwareRasterizer&) = delete;
  GpuHardwareRasterizer& operator=(const GpuHardwareRasterizer&) = delete;

  // Must be called on the UI thread after SDL video initialization.
  bool initialize();

  // Must be called after the emulation thread has stopped.
  void shutdown();

  // The compute context is created on initialize(), then transferred to the
  // EmuRunner thread for all actual raster work.
  bool bind_to_current_thread();
  void unbind_from_current_thread();

  bool available() const { return available_; }
  const std::string& status() const { return status_; }

  bool upload_vram(const u16* vram, size_t pixel_count);
  bool download_vram(u16* vram, size_t pixel_count);

  bool draw_triangle(
      TriangleMode mode,
      Vertex v0,
      Vertex v1,
      Vertex v2,
      const RasterState& state);

  bool draw_flat_rect(
      s16 x, s16 y, u16 width, u16 height,
      u8 r, u8 g, u8 b,
      const RasterState& state);

  bool draw_textured_rect(
      s16 x, s16 y, u16 width, u16 height,
      u8 u, u8 v,
      u8 r, u8 g, u8 b,
      const RasterState& state);

  u64 dispatch_count() const { return dispatch_count_; }
  u64 upload_count() const { return upload_count_; }
  u64 download_count() const { return download_count_; }

private:
  SDL_Window* window_ = nullptr;
  SDL_GLContext context_ = nullptr;
  bool available_ = false;
  bool thread_bound_ = false;
  std::string status_ = "not initialized";

  unsigned int program_ = 0;
  unsigned int vram_buffer_ = 0;

  int loc_mode_ = -1;
  int loc_bounds_ = -1;
  int loc_v0_ = -1;
  int loc_v1_ = -1;
  int loc_v2_ = -1;
  int loc_c0_ = -1;
  int loc_c1_ = -1;
  int loc_c2_ = -1;
  int loc_tex0_ = -1;
  int loc_tex1_ = -1;
  int loc_clut_row_ = -1;
  int loc_flags_ = -1;
  int loc_semi_mode_ = -1;
  int loc_rect_ = -1;

  std::vector<u32> staging_;
  u64 dispatch_count_ = 0;
  u64 upload_count_ = 0;
  u64 download_count_ = 0;

  bool load_functions();
  bool create_program();
  bool create_vram_buffer();
  void destroy_gl_objects();

  bool set_common_uniforms(
      int mode,
      int min_x, int min_y, int max_x, int max_y,
      const Vertex& v0,
      const Vertex& v1,
      const Vertex& v2,
      const RasterState& state,
      int rect_x, int rect_y, int rect_w, int rect_h);

  bool dispatch_bounds(int min_x, int min_y, int max_x, int max_y);
};

