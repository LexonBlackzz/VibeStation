#include "renderer.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#endif
#include <SDL.h>
#include <SDL_opengl.h>

#ifndef APIENTRY
#define APIENTRY
#endif

#ifndef GL_VERTEX_SHADER
#define GL_VERTEX_SHADER 0x8B31
#endif
#ifndef GL_FRAGMENT_SHADER
#define GL_FRAGMENT_SHADER 0x8B30
#endif
#ifndef GL_COMPILE_STATUS
#define GL_COMPILE_STATUS 0x8B81
#endif
#ifndef GL_LINK_STATUS
#define GL_LINK_STATUS 0x8B82
#endif
#ifndef GL_INFO_LOG_LENGTH
#define GL_INFO_LOG_LENGTH 0x8B84
#endif
#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#endif
#ifndef GL_FRAMEBUFFER_BINDING
#define GL_FRAMEBUFFER_BINDING 0x8CA6
#endif
#ifndef GL_COLOR_ATTACHMENT0
#define GL_COLOR_ATTACHMENT0 0x8CE0
#endif
#ifndef GL_FRAMEBUFFER_COMPLETE
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#endif
#ifndef GL_CURRENT_PROGRAM
#define GL_CURRENT_PROGRAM 0x8B8D
#endif
#ifndef GL_VERTEX_ARRAY_BINDING
#define GL_VERTEX_ARRAY_BINDING 0x85B5
#endif
#ifndef GL_ACTIVE_TEXTURE
#define GL_ACTIVE_TEXTURE 0x84E0
#endif
#ifndef GL_TEXTURE0
#define GL_TEXTURE0 0x84C0
#endif

namespace {

// CRTView exposes a television-style CRT path, a PC-monitor CRT path, and an
// unfiltered path. VibeStation keeps that same useful preset split, but uses a
// native post-process shader so the effect can be rendered into an ImGui-owned
// game surface instead of taking over the whole OpenGL backbuffer.
//
// Reference project:
//   https://github.com/mattiasgustavsson/crtview
// The reference implementation is dual-licensed MIT / public domain.

constexpr const char *kPostVertexShader = R"GLSL(
#version 330 core

out vec2 v_uv;

void main() {
    vec2 p = vec2(
        float((gl_VertexID << 1) & 2),
        float(gl_VertexID & 2));

    // Full-screen triangle vertices use p in the 0..2 range. The visible
    // viewport only covers the 0..1 interpolated portion, so passing p
    // directly yields the correct 0..1 UVs across the screen. Multiplying by
    // 0.5 here would sample only the upper-left quarter of the source texture.
    v_uv = p;
    gl_Position = vec4(
        p * 2.0 - 1.0,
        0.0,
        1.0);
}
)GLSL";

constexpr const char *kPostFragmentShader = R"GLSL(
#version 330 core

uniform sampler2D u_texture;
uniform vec2 u_source_resolution;
uniform vec2 u_output_resolution;
uniform float u_time;
uniform int u_mode;

in vec2 v_uv;
out vec4 out_color;

const float PI = 3.14159265358979323846;

float hash21(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

vec2 crt_warp(vec2 uv, float amount) {
    vec2 p = uv * 2.0 - 1.0;
    p.x *= 1.0 + amount * p.y * p.y;
    p.y *= 1.0 + amount * p.x * p.x;
    return p * 0.5 + 0.5;
}

vec3 sample_chromatic(
    sampler2D tex,
    vec2 uv,
    vec2 texel,
    float separation) {
    float r = texture(
        tex,
        uv + vec2(texel.x * separation, 0.0)).r;
    float g = texture(tex, uv).g;
    float b = texture(
        tex,
        uv - vec2(texel.x * separation, 0.0)).b;
    return vec3(r, g, b);
}

void main() {
    bool tv_mode = u_mode == 1;

    float curvature =
        tv_mode ? 0.082 : 0.026;
    float separation =
        tv_mode ? 0.72 : 0.16;
    float scan_strength =
        tv_mode ? 0.22 : 0.15;
    float vignette_strength =
        tv_mode ? 0.34 : 0.20;

    vec2 uv = crt_warp(
        v_uv,
        curvature);

    if (uv.x <= 0.0 ||
        uv.x >= 1.0 ||
        uv.y <= 0.0 ||
        uv.y >= 1.0) {
        out_color = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    vec2 source_size =
        max(u_source_resolution, vec2(1.0));
    vec2 output_size =
        max(u_output_resolution, vec2(1.0));
    vec2 texel = 1.0 / source_size;

    vec3 color =
        sample_chromatic(
            u_texture,
            uv,
            texel,
            separation);

    if (tv_mode) {
        // Consumer sets are intentionally a little softer, with a tiny
        // horizontal phosphor/ghost trail.
        vec3 left =
            sample_chromatic(
                u_texture,
                uv - vec2(texel.x * 0.85, 0.0),
                texel,
                separation);
        vec3 right =
            sample_chromatic(
                u_texture,
                uv + vec2(texel.x * 0.85, 0.0),
                texel,
                separation);
        color =
            color * 0.72 +
            (left + right) * 0.14;

        vec3 ghost =
            texture(
                u_texture,
                uv - vec2(texel.x * 2.2, 0.0)).rgb;
        color += ghost * 0.035;
    }

    // Scanlines follow the emulated raster, not the desktop pixel grid.
    float raster_y =
        uv.y * source_size.y;
    float scan_wave =
        0.5 +
        0.5 * cos(
            raster_y * PI * 2.0);
    color *=
        1.0 -
        scan_strength * scan_wave;

    if (tv_mode) {
        // RGB shadow-mask triads live in output/display pixels.
        float triad =
            mod(
                floor(gl_FragCoord.x),
                3.0);
        vec3 mask = vec3(0.91);
        if (triad < 1.0) {
            mask.r = 1.0;
        }
        else if (triad < 2.0) {
            mask.g = 1.0;
        }
        else {
            mask.b = 1.0;
        }
        color *= mask;
    }
    else {
        // PC CRT preset: tighter, subtler aperture grille.
        float grille =
            0.965 +
            0.035 *
                cos(
                    gl_FragCoord.x *
                    PI);
        color *= grille;
    }

    vec2 centered =
        uv * 2.0 - 1.0;
    float edge =
        dot(centered, centered);
    float vignette =
        1.0 -
        vignette_strength *
            smoothstep(
                0.30,
                1.28,
                edge);
    color *= vignette;

    // A trace of temporal noise prevents the CRT surface from looking like a
    // static Photoshop overlay. Keep it deliberately restrained.
    float noise_amount =
        tv_mode ? 0.010 : 0.003;
    float noise =
        hash21(
            gl_FragCoord.xy +
            vec2(
                floor(u_time * 60.0),
                floor(u_time * 37.0))) -
        0.5;
    color += noise * noise_amount;

    // TV preset carries a little more phosphor brightness; the PC preset stays
    // comparatively crisp and neutral.
    color *= tv_mode ? 1.085 : 1.035;

    // Very gentle glass falloff at the extreme border.
    vec2 border_dist =
        min(uv, 1.0 - uv);
    float border =
        smoothstep(
            0.0,
            tv_mode ? 0.018 : 0.010,
            min(border_dist.x, border_dist.y));
    color *= border;

    out_color =
        vec4(
            clamp(color, 0.0, 1.0),
            1.0);
}
)GLSL";

} // namespace

struct Renderer::ShaderState {
  using CreateShaderFn =
      GLuint(APIENTRY *)(GLenum);
  using ShaderSourceFn =
      void(APIENTRY *)(
          GLuint,
          GLsizei,
          const char *const *,
          const GLint *);
  using CompileShaderFn =
      void(APIENTRY *)(GLuint);
  using GetShaderivFn =
      void(APIENTRY *)(GLuint, GLenum, GLint *);
  using GetShaderInfoLogFn =
      void(APIENTRY *)(
          GLuint,
          GLsizei,
          GLsizei *,
          char *);
  using DeleteShaderFn =
      void(APIENTRY *)(GLuint);

  using CreateProgramFn =
      GLuint(APIENTRY *)();
  using AttachShaderFn =
      void(APIENTRY *)(GLuint, GLuint);
  using LinkProgramFn =
      void(APIENTRY *)(GLuint);
  using GetProgramivFn =
      void(APIENTRY *)(GLuint, GLenum, GLint *);
  using GetProgramInfoLogFn =
      void(APIENTRY *)(
          GLuint,
          GLsizei,
          GLsizei *,
          char *);
  using DeleteProgramFn =
      void(APIENTRY *)(GLuint);
  using UseProgramFn =
      void(APIENTRY *)(GLuint);

  using GetUniformLocationFn =
      GLint(APIENTRY *)(GLuint, const char *);
  using Uniform1iFn =
      void(APIENTRY *)(GLint, GLint);
  using Uniform1fFn =
      void(APIENTRY *)(GLint, GLfloat);
  using Uniform2fFn =
      void(APIENTRY *)(GLint, GLfloat, GLfloat);

  using GenFramebuffersFn =
      void(APIENTRY *)(GLsizei, GLuint *);
  using BindFramebufferFn =
      void(APIENTRY *)(GLenum, GLuint);
  using FramebufferTexture2DFn =
      void(APIENTRY *)(
          GLenum,
          GLenum,
          GLenum,
          GLuint,
          GLint);
  using CheckFramebufferStatusFn =
      GLenum(APIENTRY *)(GLenum);
  using DeleteFramebuffersFn =
      void(APIENTRY *)(GLsizei, const GLuint *);

  using GenVertexArraysFn =
      void(APIENTRY *)(GLsizei, GLuint *);
  using BindVertexArrayFn =
      void(APIENTRY *)(GLuint);
  using DeleteVertexArraysFn =
      void(APIENTRY *)(GLsizei, const GLuint *);

  using ActiveTextureFn =
      void(APIENTRY *)(GLenum);

  CreateShaderFn CreateShader = nullptr;
  ShaderSourceFn ShaderSource = nullptr;
  CompileShaderFn CompileShader = nullptr;
  GetShaderivFn GetShaderiv = nullptr;
  GetShaderInfoLogFn GetShaderInfoLog = nullptr;
  DeleteShaderFn DeleteShader = nullptr;

  CreateProgramFn CreateProgram = nullptr;
  AttachShaderFn AttachShader = nullptr;
  LinkProgramFn LinkProgram = nullptr;
  GetProgramivFn GetProgramiv = nullptr;
  GetProgramInfoLogFn GetProgramInfoLog = nullptr;
  DeleteProgramFn DeleteProgram = nullptr;
  UseProgramFn UseProgram = nullptr;

  GetUniformLocationFn GetUniformLocation = nullptr;
  Uniform1iFn Uniform1i = nullptr;
  Uniform1fFn Uniform1f = nullptr;
  Uniform2fFn Uniform2f = nullptr;

  GenFramebuffersFn GenFramebuffers = nullptr;
  BindFramebufferFn BindFramebuffer = nullptr;
  FramebufferTexture2DFn FramebufferTexture2D = nullptr;
  CheckFramebufferStatusFn CheckFramebufferStatus = nullptr;
  DeleteFramebuffersFn DeleteFramebuffers = nullptr;

  GenVertexArraysFn GenVertexArrays = nullptr;
  BindVertexArrayFn BindVertexArray = nullptr;
  DeleteVertexArraysFn DeleteVertexArrays = nullptr;

  ActiveTextureFn ActiveTexture = nullptr;

  bool supported = false;
  bool program_failed = false;
  bool last_pass_valid = false;

  GLuint program = 0;
  GLuint framebuffer = 0;
  GLuint output_texture = 0;
  GLuint vertex_array = 0;

  int target_width = 0;
  int target_height = 0;

  GLint texture_location = -1;
  GLint source_resolution_location = -1;
  GLint output_resolution_location = -1;
  GLint time_location = -1;
  GLint mode_location = -1;
};

namespace {

template <typename T>
bool load_gl_proc(T &proc, const char *name) {
  proc =
      reinterpret_cast<T>(
          SDL_GL_GetProcAddress(name));
  return proc != nullptr;
}

} // namespace

Renderer::Renderer() = default;

Renderer::~Renderer() {
  shutdown();
}

bool Renderer::init(SDL_Window *window) {
  window_ = window;

  if (!create_texture()) {
    return false;
  }

  if (!init_shader_support()) {
    LOG_INFO(
        "Renderer: GPU post-process shaders unavailable; continuing without them");
  }

  const GLubyte *gl_version =
      glGetString(GL_VERSION);
  LOG_INFO(
      "Renderer: Initialized OpenGL %s",
      gl_version
          ? reinterpret_cast<const char *>(gl_version)
          : "Unknown");
  return true;
}

void Renderer::shutdown() {
  release_shader_resources();

  if (texture_id_) {
    glDeleteTextures(1, &texture_id_);
    texture_id_ = 0;
  }

  texture_width_ = 0;
  texture_height_ = 0;
  window_ = nullptr;
}

void Renderer::set_bilinear_filtering(bool enabled) {
  bilinear_filtering_ = enabled;
  apply_texture_filtering();
}

void Renderer::apply_texture_filtering() {
  if (!texture_id_) {
    return;
  }

  glBindTexture(
      GL_TEXTURE_2D,
      texture_id_);
  const GLint filter =
      bilinear_filtering_
          ? GL_LINEAR
          : GL_NEAREST;
  glTexParameteri(
      GL_TEXTURE_2D,
      GL_TEXTURE_MIN_FILTER,
      filter);
  glTexParameteri(
      GL_TEXTURE_2D,
      GL_TEXTURE_MAG_FILTER,
      filter);
}

bool Renderer::create_texture() {
  glGenTextures(1, &texture_id_);
  glBindTexture(
      GL_TEXTURE_2D,
      texture_id_);

  bilinear_filtering_ =
      g_bilinear_filtering;
  apply_texture_filtering();

  glTexParameteri(
      GL_TEXTURE_2D,
      GL_TEXTURE_WRAP_S,
      GL_CLAMP_TO_EDGE);
  glTexParameteri(
      GL_TEXTURE_2D,
      GL_TEXTURE_WRAP_T,
      GL_CLAMP_TO_EDGE);

  texture_width_ = 320;
  texture_height_ = 240;

  glTexImage2D(
      GL_TEXTURE_2D,
      0,
      GL_RGBA,
      texture_width_,
      texture_height_,
      0,
      GL_RGBA,
      GL_UNSIGNED_BYTE,
      nullptr);
  return true;
}

bool Renderer::init_shader_support() {
  if (shader_state_) {
    return shader_state_->supported;
  }

  shader_state_ =
      std::make_unique<ShaderState>();
  ShaderState &s =
      *shader_state_;

  const bool loaded =
      load_gl_proc(
          s.CreateShader,
          "glCreateShader") &&
      load_gl_proc(
          s.ShaderSource,
          "glShaderSource") &&
      load_gl_proc(
          s.CompileShader,
          "glCompileShader") &&
      load_gl_proc(
          s.GetShaderiv,
          "glGetShaderiv") &&
      load_gl_proc(
          s.GetShaderInfoLog,
          "glGetShaderInfoLog") &&
      load_gl_proc(
          s.DeleteShader,
          "glDeleteShader") &&
      load_gl_proc(
          s.CreateProgram,
          "glCreateProgram") &&
      load_gl_proc(
          s.AttachShader,
          "glAttachShader") &&
      load_gl_proc(
          s.LinkProgram,
          "glLinkProgram") &&
      load_gl_proc(
          s.GetProgramiv,
          "glGetProgramiv") &&
      load_gl_proc(
          s.GetProgramInfoLog,
          "glGetProgramInfoLog") &&
      load_gl_proc(
          s.DeleteProgram,
          "glDeleteProgram") &&
      load_gl_proc(
          s.UseProgram,
          "glUseProgram") &&
      load_gl_proc(
          s.GetUniformLocation,
          "glGetUniformLocation") &&
      load_gl_proc(
          s.Uniform1i,
          "glUniform1i") &&
      load_gl_proc(
          s.Uniform1f,
          "glUniform1f") &&
      load_gl_proc(
          s.Uniform2f,
          "glUniform2f") &&
      load_gl_proc(
          s.GenFramebuffers,
          "glGenFramebuffers") &&
      load_gl_proc(
          s.BindFramebuffer,
          "glBindFramebuffer") &&
      load_gl_proc(
          s.FramebufferTexture2D,
          "glFramebufferTexture2D") &&
      load_gl_proc(
          s.CheckFramebufferStatus,
          "glCheckFramebufferStatus") &&
      load_gl_proc(
          s.DeleteFramebuffers,
          "glDeleteFramebuffers") &&
      load_gl_proc(
          s.GenVertexArrays,
          "glGenVertexArrays") &&
      load_gl_proc(
          s.BindVertexArray,
          "glBindVertexArray") &&
      load_gl_proc(
          s.DeleteVertexArrays,
          "glDeleteVertexArrays") &&
      load_gl_proc(
          s.ActiveTexture,
          "glActiveTexture");

  s.supported = loaded;
  return loaded;
}

bool Renderer::ensure_shader_program() {
  if (!shader_state_ &&
      !init_shader_support()) {
    return false;
  }

  ShaderState &s =
      *shader_state_;

  if (!s.supported ||
      s.program_failed) {
    return false;
  }

  if (s.program != 0) {
    return true;
  }

  const auto compile_shader =
      [&](GLenum type,
          const char *source,
          const char *label) -> GLuint {
    const GLuint shader =
        s.CreateShader(type);
    if (!shader) {
      LOG_ERROR(
          "Renderer: Failed to create %s shader",
          label);
      return 0;
    }

    s.ShaderSource(
        shader,
        1,
        &source,
        nullptr);
    s.CompileShader(shader);

    GLint compiled = GL_FALSE;
    s.GetShaderiv(
        shader,
        GL_COMPILE_STATUS,
        &compiled);

    if (compiled != GL_TRUE) {
      std::array<char, 2048> log{};
      GLsizei written = 0;
      s.GetShaderInfoLog(
          shader,
          static_cast<GLsizei>(
              log.size() - 1),
          &written,
          log.data());

      LOG_ERROR(
          "Renderer: %s shader compile failed: %s",
          label,
          log.data());

      s.DeleteShader(shader);
      return 0;
    }

    return shader;
  };

  const GLuint vertex =
      compile_shader(
          GL_VERTEX_SHADER,
          kPostVertexShader,
          "post-process vertex");
  const GLuint fragment =
      compile_shader(
          GL_FRAGMENT_SHADER,
          kPostFragmentShader,
          "post-process fragment");

  if (!vertex || !fragment) {
    if (vertex) {
      s.DeleteShader(vertex);
    }
    if (fragment) {
      s.DeleteShader(fragment);
    }
    s.program_failed = true;
    return false;
  }

  const GLuint program =
      s.CreateProgram();
  s.AttachShader(
      program,
      vertex);
  s.AttachShader(
      program,
      fragment);
  s.LinkProgram(program);

  s.DeleteShader(vertex);
  s.DeleteShader(fragment);

  GLint linked = GL_FALSE;
  s.GetProgramiv(
      program,
      GL_LINK_STATUS,
      &linked);

  if (linked != GL_TRUE) {
    std::array<char, 2048> log{};
    GLsizei written = 0;
    s.GetProgramInfoLog(
        program,
        static_cast<GLsizei>(
            log.size() - 1),
        &written,
        log.data());

    LOG_ERROR(
        "Renderer: Post-process shader link failed: %s",
        log.data());

    s.DeleteProgram(program);
    s.program_failed = true;
    return false;
  }

  s.program = program;
  s.texture_location =
      s.GetUniformLocation(
          program,
          "u_texture");
  s.source_resolution_location =
      s.GetUniformLocation(
          program,
          "u_source_resolution");
  s.output_resolution_location =
      s.GetUniformLocation(
          program,
          "u_output_resolution");
  s.time_location =
      s.GetUniformLocation(
          program,
          "u_time");
  s.mode_location =
      s.GetUniformLocation(
          program,
          "u_mode");

  s.GenVertexArrays(
      1,
      &s.vertex_array);

  LOG_INFO(
      "Renderer: CRT shader pipeline initialized");
  return true;
}

bool Renderer::ensure_shader_target(
    int width,
    int height) {
  if (!shader_state_ ||
      !shader_state_->supported) {
    return false;
  }

  ShaderState &s =
      *shader_state_;
  const int w =
      (std::max)(1, width);
  const int h =
      (std::max)(1, height);

  if (s.output_texture != 0 &&
      s.framebuffer != 0 &&
      s.target_width == w &&
      s.target_height == h) {
    return true;
  }

  GLint previous_active_texture =
      static_cast<GLint>(GL_TEXTURE0);
  GLint previous_texture = 0;
  GLint previous_framebuffer = 0;

  glGetIntegerv(
      GL_ACTIVE_TEXTURE,
      &previous_active_texture);
  s.ActiveTexture(GL_TEXTURE0);
  glGetIntegerv(
      GL_TEXTURE_BINDING_2D,
      &previous_texture);
  glGetIntegerv(
      GL_FRAMEBUFFER_BINDING,
      &previous_framebuffer);

  if (s.output_texture == 0) {
    glGenTextures(
        1,
        &s.output_texture);
  }

  glBindTexture(
      GL_TEXTURE_2D,
      s.output_texture);
  glTexParameteri(
      GL_TEXTURE_2D,
      GL_TEXTURE_MIN_FILTER,
      GL_LINEAR);
  glTexParameteri(
      GL_TEXTURE_2D,
      GL_TEXTURE_MAG_FILTER,
      GL_LINEAR);
  glTexParameteri(
      GL_TEXTURE_2D,
      GL_TEXTURE_WRAP_S,
      GL_CLAMP_TO_EDGE);
  glTexParameteri(
      GL_TEXTURE_2D,
      GL_TEXTURE_WRAP_T,
      GL_CLAMP_TO_EDGE);
  glTexImage2D(
      GL_TEXTURE_2D,
      0,
      GL_RGBA,
      w,
      h,
      0,
      GL_RGBA,
      GL_UNSIGNED_BYTE,
      nullptr);

  if (s.framebuffer == 0) {
    s.GenFramebuffers(
        1,
        &s.framebuffer);
  }

  s.BindFramebuffer(
      GL_FRAMEBUFFER,
      s.framebuffer);
  s.FramebufferTexture2D(
      GL_FRAMEBUFFER,
      GL_COLOR_ATTACHMENT0,
      GL_TEXTURE_2D,
      s.output_texture,
      0);

  const GLenum status =
      s.CheckFramebufferStatus(
          GL_FRAMEBUFFER);

  s.BindFramebuffer(
      GL_FRAMEBUFFER,
      static_cast<GLuint>(
          previous_framebuffer));
  glBindTexture(
      GL_TEXTURE_2D,
      static_cast<GLuint>(
          previous_texture));
  s.ActiveTexture(
      static_cast<GLenum>(
          previous_active_texture));

  if (status !=
      GL_FRAMEBUFFER_COMPLETE) {
    LOG_ERROR(
        "Renderer: CRT framebuffer incomplete (0x%X)",
        static_cast<unsigned int>(
            status));
    s.last_pass_valid = false;
    return false;
  }

  s.target_width = w;
  s.target_height = h;
  s.last_pass_valid = false;
  return true;
}

bool Renderer::render_shader_pass(
    int width,
    int height) {
  if (shader_mode_ ==
      ShaderMode::Off) {
    return false;
  }

  if (!ensure_shader_program() ||
      !ensure_shader_target(
          width,
          height)) {
    return false;
  }

  ShaderState &s =
      *shader_state_;

  GLint previous_framebuffer = 0;
  GLint previous_program = 0;
  GLint previous_vertex_array = 0;
  GLint previous_active_texture =
      static_cast<GLint>(GL_TEXTURE0);
  GLint previous_texture = 0;
  GLint previous_viewport[4] = {
      0, 0, 0, 0};

  glGetIntegerv(
      GL_FRAMEBUFFER_BINDING,
      &previous_framebuffer);
  glGetIntegerv(
      GL_CURRENT_PROGRAM,
      &previous_program);
  glGetIntegerv(
      GL_VERTEX_ARRAY_BINDING,
      &previous_vertex_array);
  glGetIntegerv(
      GL_ACTIVE_TEXTURE,
      &previous_active_texture);
  glGetIntegerv(
      GL_VIEWPORT,
      previous_viewport);

  s.ActiveTexture(GL_TEXTURE0);
  glGetIntegerv(
      GL_TEXTURE_BINDING_2D,
      &previous_texture);

  const GLboolean blend_enabled =
      glIsEnabled(GL_BLEND);
  const GLboolean scissor_enabled =
      glIsEnabled(GL_SCISSOR_TEST);
  const GLboolean depth_enabled =
      glIsEnabled(GL_DEPTH_TEST);
  const GLboolean cull_enabled =
      glIsEnabled(GL_CULL_FACE);

  glDisable(GL_BLEND);
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  s.BindFramebuffer(
      GL_FRAMEBUFFER,
      s.framebuffer);
  glViewport(
      0,
      0,
      s.target_width,
      s.target_height);
  glClearColor(
      0.0f,
      0.0f,
      0.0f,
      1.0f);
  glClear(
      GL_COLOR_BUFFER_BIT);

  s.UseProgram(
      s.program);
  s.BindVertexArray(
      s.vertex_array);

  s.ActiveTexture(
      GL_TEXTURE0);
  glBindTexture(
      GL_TEXTURE_2D,
      texture_id_);

  if (s.texture_location >= 0) {
    s.Uniform1i(
        s.texture_location,
        0);
  }
  if (s.source_resolution_location >= 0) {
    s.Uniform2f(
        s.source_resolution_location,
        static_cast<float>(
            (std::max)(
                1,
                last_frame_width_)),
        static_cast<float>(
            (std::max)(
                1,
                last_frame_height_)));
  }
  if (s.output_resolution_location >= 0) {
    s.Uniform2f(
        s.output_resolution_location,
        static_cast<float>(
            s.target_width),
        static_cast<float>(
            s.target_height));
  }
  if (s.time_location >= 0) {
    s.Uniform1f(
        s.time_location,
        static_cast<float>(
            SDL_GetTicks()) *
            0.001f);
  }
  if (s.mode_location >= 0) {
    s.Uniform1i(
        s.mode_location,
        shader_mode_ ==
                ShaderMode::CrtTv
            ? 1
            : 2);
  }

  glDrawArrays(
      GL_TRIANGLES,
      0,
      3);

  glBindTexture(
      GL_TEXTURE_2D,
      static_cast<GLuint>(
          previous_texture));
  s.BindVertexArray(
      static_cast<GLuint>(
          previous_vertex_array));
  s.UseProgram(
      static_cast<GLuint>(
          previous_program));
  s.BindFramebuffer(
      GL_FRAMEBUFFER,
      static_cast<GLuint>(
          previous_framebuffer));
  glViewport(
      previous_viewport[0],
      previous_viewport[1],
      previous_viewport[2],
      previous_viewport[3]);
  s.ActiveTexture(
      static_cast<GLenum>(
          previous_active_texture));

  if (blend_enabled) {
    glEnable(GL_BLEND);
  }
  if (scissor_enabled) {
    glEnable(GL_SCISSOR_TEST);
  }
  if (depth_enabled) {
    glEnable(GL_DEPTH_TEST);
  }
  if (cull_enabled) {
    glEnable(GL_CULL_FACE);
  }

  s.last_pass_valid = true;
  return true;
}

void Renderer::release_shader_resources() {
  if (!shader_state_) {
    shader_mode_ =
        ShaderMode::Off;
    return;
  }

  ShaderState &s =
      *shader_state_;

  if (s.supported) {
    if (s.program != 0 &&
        s.DeleteProgram != nullptr) {
      s.DeleteProgram(
          s.program);
      s.program = 0;
    }

    if (s.framebuffer != 0 &&
        s.DeleteFramebuffers != nullptr) {
      s.DeleteFramebuffers(
          1,
          &s.framebuffer);
      s.framebuffer = 0;
    }

    if (s.vertex_array != 0 &&
        s.DeleteVertexArrays != nullptr) {
      s.DeleteVertexArrays(
          1,
          &s.vertex_array);
      s.vertex_array = 0;
    }
  }

  if (s.output_texture != 0) {
    glDeleteTextures(
        1,
        &s.output_texture);
    s.output_texture = 0;
  }

  shader_state_.reset();
  shader_mode_ =
      ShaderMode::Off;
}

bool Renderer::set_shader_mode(
    ShaderMode mode) {
  if (mode ==
      ShaderMode::Off) {
    shader_mode_ =
        ShaderMode::Off;
    if (shader_state_) {
      shader_state_->
          last_pass_valid = false;
    }
    return true;
  }

  if (!init_shader_support() ||
      !ensure_shader_program()) {
    shader_mode_ =
        ShaderMode::Off;
    return false;
  }

  shader_mode_ = mode;

  if (shader_state_) {
    shader_state_->
        last_pass_valid = false;

    if (shader_state_->
            target_width > 0 &&
        shader_state_->
            target_height > 0) {
      render_shader_pass(
          shader_state_->
              target_width,
          shader_state_->
              target_height);
    }
  }

  return true;
}

bool Renderer::shader_supported() const {
  return shader_state_ &&
      shader_state_->supported &&
      !shader_state_->program_failed;
}

const char *Renderer::shader_mode_name(
    ShaderMode mode) {
  switch (mode) {
  case ShaderMode::CrtTv:
    return "CRT - Consumer TV";
  case ShaderMode::CrtPc:
    return "CRT - PC Monitor";
  case ShaderMode::Off:
  default:
    return "Off";
  }
}

unsigned int Renderer::get_texture_id() const {
  if (shader_mode_ !=
          ShaderMode::Off &&
      shader_state_ &&
      shader_state_->
          last_pass_valid &&
      shader_state_->
          output_texture != 0) {
    return shader_state_->
        output_texture;
  }

  return texture_id_;
}

void Renderer::prepare_present(
    int width,
    int height) {
  if (shader_mode_ ==
      ShaderMode::Off) {
    return;
  }

  if (!render_shader_pass(
          width,
          height)) {
    if (shader_state_) {
      shader_state_->
          last_pass_valid = false;
    }
  }
}

void Renderer::upload_frame(
    const std::vector<u32> &rgba,
    int width,
    int height) {
  const int w =
      (std::max)(1, width);
  const int h =
      (std::max)(1, height);
  const size_t expected =
      static_cast<size_t>(w) *
      static_cast<size_t>(h);

  if (rgba.size() < expected) {
    return;
  }

  last_frame_width_ = w;
  last_frame_height_ = h;

  glBindTexture(
      GL_TEXTURE_2D,
      texture_id_);

  if (texture_width_ != w ||
      texture_height_ != h) {
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGBA,
        w,
        h,
        0,
        GL_RGBA,
        GL_UNSIGNED_BYTE,
        rgba.data());

    texture_width_ = w;
    texture_height_ = h;
  }
  else {
    glTexSubImage2D(
        GL_TEXTURE_2D,
        0,
        0,
        0,
        w,
        h,
        GL_RGBA,
        GL_UNSIGNED_BYTE,
        rgba.data());
  }

  if (shader_state_) {
    shader_state_->
        last_pass_valid = false;
  }
}
