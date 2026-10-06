#pragma once
#include "gpu_hw_stream.h"
#include "types.h"

#include <array>
#include <memory>
#include <vector>

// ── OpenGL upscaler ────────────────────────────────────────────────
// Replays a GpuHwStream into a colour buffer `scale` times the size of
// VRAM and keeps the most recently presented display area in an output
// texture. Requires a current OpenGL 3.3 context on the calling thread for
// every call. The software renderer remains the source of truth; this only
// produces a sharper picture of it.

class GpuHwRenderer {
public:
  static constexpr int kMaxScale = 8;

  GpuHwRenderer();
  ~GpuHwRenderer();

  // Creates GL resources. Returns false (and leaves the renderer unusable)
  // when the context lacks what is needed.
  bool init(int scale);
  void shutdown();
  bool ready() const;
  int scale() const { return scale_; }

  // Changes the internal resolution. The colour buffer is rebuilt from the
  // native VRAM copy, so the picture stays valid until the game redraws.
  bool set_scale(int scale);

  void replay(const GpuHwStream &stream);

  // Result of the last Present command.
  bool has_output() const { return has_output_; }
  bool software_present() const { return software_present_; }
  unsigned int output_texture() const;
  int output_width() const { return output_width_; }
  int output_height() const { return output_height_; }
  // Reads the presented picture back as RGBA (r | g<<8 | b<<16 | a<<24),
  // top row first. For tests and screenshots; stalls the GPU.
  bool read_output(std::vector<u32> &rgba) const;
  // Reads the whole upscaled VRAM colour buffer (debugging).
  bool read_color_buffer(std::vector<u32> &rgba, int &width, int &height) const;

  struct Gl; // OpenGL entry points and objects (defined in the .cpp)

private:
  std::unique_ptr<Gl> gl_;
  int scale_ = 0;
  bool has_output_ = false;
  bool software_present_ = true;
  int output_width_ = 0;
  int output_height_ = 0;
  // Pages whose colour buffer changed since they were last copied into the
  // texture-sampling snapshot (see refresh_sample_pages).
  std::array<bool, gpu_hw::kPageCount> sample_stale_{};

  bool create_targets();
  void release_targets();
  void draw_triangles(const GpuHwStream &stream, const GpuHwCommand &cmd);
  void fill(const GpuHwCommand &cmd);
  void upload_native(int x, int y, int w, int h, const u16 *pixels);
  void native_to_color(int x, int y, int w, int h);
  void copy(const GpuHwCommand &cmd);
  void present(const GpuHwCommand &cmd);
  void mark_stale(int x0, int y0, int x1, int y1);
  void refresh_sample_pages(const GpuHwCommand &cmd);
  void refresh_sample_rect(int x0, int y0, int x1, int y1);
  void refresh_sample_page(int page_x, int page_y);
};
