#pragma once
#include "types.h"

#include <memory>
#include <vector>

// ── OpenGL Renderer ────────────────────────────────────────────────
// Uploads RGBA frame snapshots to an OpenGL texture for presentation in ImGui.

struct SDL_Window;
typedef void *SDL_GLContext;

class Renderer {
public:
  enum class ShaderMode {
    Off = 0,
    CrtTv,
    CrtPc,
  };

  Renderer();
  ~Renderer();

  bool init(SDL_Window *window);
  void shutdown();
  void set_bilinear_filtering(bool enabled);

  // Upload an RGBA frame into the source presentation texture.
  void upload_frame(const std::vector<u32> &rgba, int width, int height);

  // Run the selected GPU post-process at the final on-screen presentation
  // size. This keeps CRT scanlines/masks tied to display pixels instead of
  // baking them into the low-resolution PS1 framebuffer.
  void prepare_present(int width, int height);

  bool set_shader_mode(ShaderMode mode);
  ShaderMode shader_mode() const { return shader_mode_; }
  bool shader_supported() const;
  static const char *shader_mode_name(ShaderMode mode);

  unsigned int get_texture_id() const;
  int last_frame_width() const { return last_frame_width_; }
  int last_frame_height() const { return last_frame_height_; }

private:
  struct ShaderState;

  SDL_Window *window_ = nullptr;
  SDL_GLContext gl_context_ = nullptr;
  unsigned int texture_id_ = 0;
  int last_frame_width_ = 320;
  int last_frame_height_ = 240;
  int texture_width_ = 0;
  int texture_height_ = 0;
  bool bilinear_filtering_ = false;
  ShaderMode shader_mode_ = ShaderMode::Off;
  std::unique_ptr<ShaderState> shader_state_;

  void apply_texture_filtering();
  bool create_texture();
  bool init_shader_support();
  bool ensure_shader_program();
  bool ensure_shader_target(int width, int height);
  bool render_shader_pass(int width, int height);
  void release_shader_resources();
};
