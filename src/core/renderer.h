#pragma once
#include "types.h"
#include <vector>

// ── OpenGL Renderer ────────────────────────────────────────────────
// Takes the GPU's VRAM and displays the active display area on screen
// as an OpenGL texture.

struct SDL_Window;
typedef void *SDL_GLContext;

class Gpu;

class Renderer {
public:
  bool init(SDL_Window *window);
  void shutdown();

  // Upload VRAM to OpenGL texture and render to screen
  void render(const Gpu &gpu);

  unsigned int get_texture_id() const { return texture_id_; }
  int last_frame_width() const { return last_frame_width_; }
  int last_frame_height() const { return last_frame_height_; }

private:
  SDL_Window *window_ = nullptr;
  SDL_GLContext gl_context_ = nullptr;
  unsigned int texture_id_ = 0;
  unsigned int vao_ = 0, vbo_ = 0;
  unsigned int shader_ = 0;
  int last_frame_width_ = 320;
  int last_frame_height_ = 240;
  int texture_width_ = 0;
  int texture_height_ = 0;
  std::vector<u32> frame_rgba_;

  bool create_shader();
  bool create_texture();
};
