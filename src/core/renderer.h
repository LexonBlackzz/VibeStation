#pragma once
#include "types.h"
#include <vector>

// ── OpenGL Renderer ────────────────────────────────────────────────
// Uploads RGBA frame snapshots to an OpenGL texture for presentation in ImGui.

struct SDL_Window;
typedef void *SDL_GLContext;

class Renderer {
public:
  bool init(SDL_Window *window);
  void shutdown();

  // Upload an RGBA frame into the presentation texture.
  void upload_frame(const std::vector<u32> &rgba, int width, int height);

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
