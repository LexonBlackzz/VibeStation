#include "renderer.h"
#include <algorithm>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#endif
#include <SDL.h>
#include <SDL_opengl.h>

bool Renderer::init(SDL_Window *window) {
  window_ = window;

  if (!create_texture()) {
    return false;
  }

  const GLubyte *gl_version = glGetString(GL_VERSION);
  LOG_INFO("Renderer: Initialized OpenGL %s",
           gl_version ? reinterpret_cast<const char *>(gl_version) : "Unknown");
  return true;
}

void Renderer::shutdown() {
  if (texture_id_) {
    glDeleteTextures(1, &texture_id_);
    texture_id_ = 0;
  }
}

void Renderer::set_bilinear_filtering(bool enabled) {
  bilinear_filtering_ = enabled;
  apply_texture_filtering();
}

void Renderer::apply_texture_filtering() {
  if (!texture_id_) {
    return;
  }

  glBindTexture(GL_TEXTURE_2D, texture_id_);
  const GLint filter = bilinear_filtering_ ? GL_LINEAR : GL_NEAREST;
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
}

bool Renderer::create_texture() {
  glGenTextures(1, &texture_id_);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  bilinear_filtering_ = g_bilinear_filtering;
  apply_texture_filtering();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  texture_width_ = 320;
  texture_height_ = 240;
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_width_, texture_height_, 0,
               GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  return true;
}

void Renderer::upload_frame(const std::vector<u32> &rgba, int width,
                            int height) {
  const int w = (std::max)(1, width);
  const int h = (std::max)(1, height);
  const size_t expected = static_cast<size_t>(w) * static_cast<size_t>(h);
  if (rgba.size() < expected) {
    return;
  }

  last_frame_width_ = w;
  last_frame_height_ = h;

  glBindTexture(GL_TEXTURE_2D, texture_id_);
  if (texture_width_ != w || texture_height_ != h) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, rgba.data());
    texture_width_ = w;
    texture_height_ = h;
    return;
  }

  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE,
                  rgba.data());
}
