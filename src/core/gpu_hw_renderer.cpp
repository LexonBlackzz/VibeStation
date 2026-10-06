#include "gpu_hw_renderer.h"

#include <algorithm>
#include <cstddef>
#include <type_traits>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif
#include <SDL.h>
#include <SDL_opengl.h>

#ifndef APIENTRY
#define APIENTRY
#endif

// Tokens past OpenGL 1.1 (Windows headers stop there).
#ifndef GL_R16UI
#define GL_R16UI 0x8234
#endif
#ifndef GL_RED_INTEGER
#define GL_RED_INTEGER 0x8D94
#endif
#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#endif
#ifndef GL_READ_FRAMEBUFFER
#define GL_READ_FRAMEBUFFER 0x8CA8
#endif
#ifndef GL_DRAW_FRAMEBUFFER
#define GL_DRAW_FRAMEBUFFER 0x8CA9
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
#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#endif
#ifndef GL_ARRAY_BUFFER_BINDING
#define GL_ARRAY_BUFFER_BINDING 0x8894
#endif
#ifndef GL_STREAM_DRAW
#define GL_STREAM_DRAW 0x88E0
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
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif
#ifndef GL_FUNC_ADD
#define GL_FUNC_ADD 0x8006
#endif
#ifndef GL_FUNC_REVERSE_SUBTRACT
#define GL_FUNC_REVERSE_SUBTRACT 0x800B
#endif
#ifndef GL_CONSTANT_COLOR
#define GL_CONSTANT_COLOR 0x8001
#endif
#ifndef GL_MAX_TEXTURE_SIZE
#define GL_MAX_TEXTURE_SIZE 0x0D33
#endif

namespace {

constexpr int kVramW = psx::VRAM_WIDTH;
constexpr int kVramH = psx::VRAM_HEIGHT;

constexpr const char *kDrawVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 a_pos;   // VRAM x, y and PGXP depth w
layout(location = 1) in vec4 a_color; // normalized RGB
layout(location = 2) in vec2 a_uv;

noperspective out vec3 v_color;  // PS1 gouraud shading is affine
out vec2 v_uv;                   // perspective-correct when w varies

void main() {
    vec2 ndc = a_pos.xy / vec2(1024.0, 512.0) * 2.0 - 1.0;
    float w = a_pos.z;
    gl_Position = vec4(ndc * w, 0.0, w);
    v_color = a_color.rgb;
    v_uv = a_uv;
}
)GLSL";

constexpr const char *kDrawFragmentShader = R"GLSL(
#version 330 core
uniform usampler2D u_vram;  // native VRAM, one 16-bit word per texel
uniform int u_textured;
uniform int u_raw;
uniform int u_depth;        // 0: 4-bit CLUT, 1: 8-bit CLUT, 2: 15-bit
uniform ivec2 u_tex_base;
uniform ivec2 u_clut;
uniform ivec4 u_window;     // mask x, mask y, offset x, offset y
uniform int u_pass;         // 0: all, 1: opaque texels, 2: semi texels
uniform int u_set_mask;
uniform sampler2D u_color_copy; // upscaled VRAM snapshot for 15-bit textures
uniform int u_scale;
uniform int u_check_mask; // discard where the destination mask bit is set

noperspective in vec3 v_color;
in vec2 v_uv;
out vec4 o_color;

uint fetch(ivec2 p) {
    return texelFetch(u_vram, ivec2(p.x & 1023, p.y & 511), 0).r;
}

void main() {
    if (u_check_mask != 0 &&
        texelFetch(u_color_copy, ivec2(gl_FragCoord.xy), 0).a > 0.5) {
        discard;
    }
    vec3 rgb;
    float mask = 0.0;
    if (u_textured != 0) {
        ivec2 uv = ivec2(floor(v_uv)) & 255;
        uv = (uv & ~u_window.xy) | (u_window.zw & u_window.xy);
        uint texel;
        if (u_depth == 0) {
            uint word = fetch(u_tex_base + ivec2(uv.x >> 2, uv.y));
            uint index = (word >> uint((uv.x & 3) * 4)) & 15u;
            texel = fetch(u_clut + ivec2(int(index), 0));
        } else if (u_depth == 1) {
            uint word = fetch(u_tex_base + ivec2(uv.x >> 1, uv.y));
            uint index = (word >> uint((uv.x & 1) * 8)) & 255u;
            texel = fetch(u_clut + ivec2(int(index), 0));
        } else {
            // 15-bit texels come from the upscaled colour buffer so textures
            // the game rendered itself (render-to-texture) stay sharp. The
            // fractional texel position picks the sub-texel sample.
            vec2 p = (vec2(u_tex_base + uv) + fract(v_uv)) * float(u_scale);
            ivec2 size = textureSize(u_color_copy, 0);
            ivec2 ip = ivec2(p);
            ip = ivec2(ip.x % size.x, ip.y % size.y);
            vec4 c = texelFetch(u_color_copy, ip, 0);
            uvec3 c5 = uvec3(floor(c.rgb * 31.0 + 0.5));
            texel = c5.r | (c5.g << 5) | (c5.b << 10) | (c.a > 0.5 ? 0x8000u : 0u);
        }
        if (texel == 0u) {
            discard; // fully transparent texel
        }
        bool semi_texel = (texel & 0x8000u) != 0u;
        if ((u_pass == 1 && semi_texel) || (u_pass == 2 && !semi_texel)) {
            discard;
        }
        vec3 t = vec3(float(texel & 31u), float((texel >> 5) & 31u),
                      float((texel >> 10) & 31u));
        if (u_raw == 0) {
            // (texel5 * color8) >> 7, saturated, as the PS1 modulates.
            vec3 c = floor(v_color * 255.0 + 0.5);
            t = min(floor(t * c / 128.0), vec3(31.0));
        }
        rgb = t / 31.0;
        mask = semi_texel ? 1.0 : 0.0;
    } else {
        rgb = v_color;
    }
    if (u_set_mask != 0) {
        mask = 1.0;
    }
    o_color = vec4(rgb, mask);
}
)GLSL";

// Native VRAM words -> colour buffer, for CPU uploads and full syncs.
constexpr const char *kCopyVertexShader = R"GLSL(
#version 330 core
void main() {
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

constexpr const char *kCopyFragmentShader = R"GLSL(
#version 330 core
uniform usampler2D u_vram;
uniform int u_scale;
out vec4 o_color;
void main() {
    ivec2 p = ivec2(gl_FragCoord.xy) / u_scale;
    uint c = texelFetch(u_vram, p, 0).r;
    vec3 rgb = vec3(float(c & 31u), float((c >> 5) & 31u),
                    float((c >> 10) & 31u)) / 31.0;
    o_color = vec4(rgb, (c & 0x8000u) != 0u ? 1.0 : 0.0);
}
)GLSL";

} // namespace

struct GpuHwRenderer::Gl {
  using GenFn = void(APIENTRY *)(GLsizei, GLuint *);
  using DelFn = void(APIENTRY *)(GLsizei, const GLuint *);
  using BindFn = void(APIENTRY *)(GLenum, GLuint);
  using FramebufferTexture2DFn =
      void(APIENTRY *)(GLenum, GLenum, GLenum, GLuint, GLint);
  using CheckFramebufferStatusFn = GLenum(APIENTRY *)(GLenum);
  using BlitFramebufferFn = void(APIENTRY *)(GLint, GLint, GLint, GLint, GLint,
                                             GLint, GLint, GLint, GLbitfield,
                                             GLenum);
  using CreateShaderFn = GLuint(APIENTRY *)(GLenum);
  using ShaderSourceFn =
      void(APIENTRY *)(GLuint, GLsizei, const char *const *, const GLint *);
  using CompileShaderFn = void(APIENTRY *)(GLuint);
  using GetivFn = void(APIENTRY *)(GLuint, GLenum, GLint *);
  using GetInfoLogFn = void(APIENTRY *)(GLuint, GLsizei, GLsizei *, char *);
  using CreateProgramFn = GLuint(APIENTRY *)();
  using AttachShaderFn = void(APIENTRY *)(GLuint, GLuint);
  using LinkProgramFn = void(APIENTRY *)(GLuint);
  using DeleteFn = void(APIENTRY *)(GLuint);
  using UseProgramFn = void(APIENTRY *)(GLuint);
  using GetUniformLocationFn = GLint(APIENTRY *)(GLuint, const char *);
  using Uniform1iFn = void(APIENTRY *)(GLint, GLint);
  using Uniform2iFn = void(APIENTRY *)(GLint, GLint, GLint);
  using Uniform4iFn = void(APIENTRY *)(GLint, GLint, GLint, GLint, GLint);
  using BindVertexArrayFn = void(APIENTRY *)(GLuint);
  using BufferDataFn =
      void(APIENTRY *)(GLenum, std::ptrdiff_t, const void *, GLenum);
  using VertexAttribPointerFn = void(APIENTRY *)(GLuint, GLint, GLenum,
                                                 GLboolean, GLsizei,
                                                 const void *);
  using EnableVertexAttribArrayFn = void(APIENTRY *)(GLuint);
  using BlendEquationSeparateFn = void(APIENTRY *)(GLenum, GLenum);
  using BlendFuncSeparateFn = void(APIENTRY *)(GLenum, GLenum, GLenum, GLenum);
  using BlendColorFn = void(APIENTRY *)(GLfloat, GLfloat, GLfloat, GLfloat);
  using ActiveTextureFn = void(APIENTRY *)(GLenum);

  GenFn GenFramebuffers = nullptr;
  DelFn DeleteFramebuffers = nullptr;
  BindFn BindFramebuffer = nullptr;
  FramebufferTexture2DFn FramebufferTexture2D = nullptr;
  CheckFramebufferStatusFn CheckFramebufferStatus = nullptr;
  BlitFramebufferFn BlitFramebuffer = nullptr;
  CreateShaderFn CreateShader = nullptr;
  ShaderSourceFn ShaderSource = nullptr;
  CompileShaderFn CompileShader = nullptr;
  GetivFn GetShaderiv = nullptr;
  GetInfoLogFn GetShaderInfoLog = nullptr;
  CreateProgramFn CreateProgram = nullptr;
  AttachShaderFn AttachShader = nullptr;
  LinkProgramFn LinkProgram = nullptr;
  GetivFn GetProgramiv = nullptr;
  GetInfoLogFn GetProgramInfoLog = nullptr;
  DeleteFn DeleteShader = nullptr;
  DeleteFn DeleteProgram = nullptr;
  UseProgramFn UseProgram = nullptr;
  GetUniformLocationFn GetUniformLocation = nullptr;
  Uniform1iFn Uniform1i = nullptr;
  Uniform2iFn Uniform2i = nullptr;
  Uniform4iFn Uniform4i = nullptr;
  GenFn GenVertexArrays = nullptr;
  DelFn DeleteVertexArrays = nullptr;
  BindVertexArrayFn BindVertexArray = nullptr;
  GenFn GenBuffers = nullptr;
  DelFn DeleteBuffers = nullptr;
  BindFn BindBuffer = nullptr;
  BufferDataFn BufferData = nullptr;
  VertexAttribPointerFn VertexAttribPointer = nullptr;
  EnableVertexAttribArrayFn EnableVertexAttribArray = nullptr;
  BlendEquationSeparateFn BlendEquationSeparate = nullptr;
  BlendFuncSeparateFn BlendFuncSeparate = nullptr;
  BlendColorFn BlendColor = nullptr;
  ActiveTextureFn ActiveTexture = nullptr;

  GLuint draw_program = 0;
  GLuint copy_program = 0;
  GLuint vao = 0;
  GLuint vbo = 0;
  GLuint empty_vao = 0;

  GLuint native_texture = 0;  // R16UI 1024x512
  GLuint color_texture = 0;   // RGBA8 (1024*S)x(512*S)
  GLuint color_fbo = 0;
  GLuint temp_texture = 0;    // scratch for overlapping copies
  GLuint temp_fbo = 0;
  GLuint sample_texture = 0; // per-page copies of color_texture to sample
  GLuint sample_fbo = 0;
  GLuint output_texture = 0;  // last presented display area
  GLuint output_fbo = 0;
  int output_alloc_w = 0;
  int output_alloc_h = 0;

  struct DrawUniforms {
    GLint vram = -1, textured = -1, raw = -1, depth = -1, tex_base = -1,
          clut = -1, window = -1, pass = -1, set_mask = -1, color_copy = -1,
          scale = -1, check_mask = -1;
  } draw_u;
  GLint copy_vram = -1;
  GLint copy_scale = -1;

  bool load() {
    bool ok = true;
    const auto load_proc = [&ok](auto &fn, const char *name) {
      fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(
          SDL_GL_GetProcAddress(name));
      ok = ok && fn != nullptr;
    };
    load_proc(GenFramebuffers, "glGenFramebuffers");
    load_proc(DeleteFramebuffers, "glDeleteFramebuffers");
    load_proc(BindFramebuffer, "glBindFramebuffer");
    load_proc(FramebufferTexture2D, "glFramebufferTexture2D");
    load_proc(CheckFramebufferStatus, "glCheckFramebufferStatus");
    load_proc(BlitFramebuffer, "glBlitFramebuffer");
    load_proc(CreateShader, "glCreateShader");
    load_proc(ShaderSource, "glShaderSource");
    load_proc(CompileShader, "glCompileShader");
    load_proc(GetShaderiv, "glGetShaderiv");
    load_proc(GetShaderInfoLog, "glGetShaderInfoLog");
    load_proc(CreateProgram, "glCreateProgram");
    load_proc(AttachShader, "glAttachShader");
    load_proc(LinkProgram, "glLinkProgram");
    load_proc(GetProgramiv, "glGetProgramiv");
    load_proc(GetProgramInfoLog, "glGetProgramInfoLog");
    load_proc(DeleteShader, "glDeleteShader");
    load_proc(DeleteProgram, "glDeleteProgram");
    load_proc(UseProgram, "glUseProgram");
    load_proc(GetUniformLocation, "glGetUniformLocation");
    load_proc(Uniform1i, "glUniform1i");
    load_proc(Uniform2i, "glUniform2i");
    load_proc(Uniform4i, "glUniform4i");
    load_proc(GenVertexArrays, "glGenVertexArrays");
    load_proc(DeleteVertexArrays, "glDeleteVertexArrays");
    load_proc(BindVertexArray, "glBindVertexArray");
    load_proc(GenBuffers, "glGenBuffers");
    load_proc(DeleteBuffers, "glDeleteBuffers");
    load_proc(BindBuffer, "glBindBuffer");
    load_proc(BufferData, "glBufferData");
    load_proc(VertexAttribPointer, "glVertexAttribPointer");
    load_proc(EnableVertexAttribArray, "glEnableVertexAttribArray");
    load_proc(BlendEquationSeparate, "glBlendEquationSeparate");
    load_proc(BlendFuncSeparate, "glBlendFuncSeparate");
    load_proc(BlendColor, "glBlendColor");
    load_proc(ActiveTexture, "glActiveTexture");
    return ok;
  }

  GLuint compile(GLenum type, const char *source) {
    const GLuint shader = CreateShader(type);
    ShaderSource(shader, 1, &source, nullptr);
    CompileShader(shader);
    GLint status = 0;
    GetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status == 0) {
      char log[1024] = {};
      GetShaderInfoLog(shader, sizeof(log), nullptr, log);
      LOG_ERROR("GpuHwRenderer: shader compile failed: %s", log);
      DeleteShader(shader);
      return 0;
    }
    return shader;
  }

  GLuint link(const char *vs_source, const char *fs_source) {
    const GLuint vs = compile(GL_VERTEX_SHADER, vs_source);
    const GLuint fs = compile(GL_FRAGMENT_SHADER, fs_source);
    if (vs == 0 || fs == 0) {
      if (vs) DeleteShader(vs);
      if (fs) DeleteShader(fs);
      return 0;
    }
    const GLuint program = CreateProgram();
    AttachShader(program, vs);
    AttachShader(program, fs);
    LinkProgram(program);
    DeleteShader(vs);
    DeleteShader(fs);
    GLint status = 0;
    GetProgramiv(program, GL_LINK_STATUS, &status);
    if (status == 0) {
      char log[1024] = {};
      GetProgramInfoLog(program, sizeof(log), nullptr, log);
      LOG_ERROR("GpuHwRenderer: program link failed: %s", log);
      DeleteProgram(program);
      return 0;
    }
    return program;
  }

  GLuint make_fbo(GLuint texture) {
    GLuint fbo = 0;
    GenFramebuffers(1, &fbo);
    BindFramebuffer(GL_FRAMEBUFFER, fbo);
    FramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         texture, 0);
    if (CheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      LOG_ERROR("GpuHwRenderer: incomplete framebuffer");
    }
    return fbo;
  }
};

namespace {

GLuint make_texture(GLint internal_format, int w, int h, GLenum format,
                    GLenum type) {
  GLuint tex = 0;
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, internal_format, w, h, 0, format, type,
               nullptr);
  return tex;
}

// Restores the GL state the UI relies on after a replay or present.
class GlStateGuard {
public:
  explicit GlStateGuard(GpuHwRenderer::Gl &gl) : gl_(gl) {
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &fbo_);
    glGetIntegerv(GL_CURRENT_PROGRAM, &program_);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vao_);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &vbo_);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture_);
    gl_.ActiveTexture(GL_TEXTURE0);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_);
    glGetIntegerv(GL_VIEWPORT, viewport_);
    glGetIntegerv(GL_SCISSOR_BOX, scissor_box_);
    scissor_ = glIsEnabled(GL_SCISSOR_TEST);
    blend_ = glIsEnabled(GL_BLEND);
    depth_ = glIsEnabled(GL_DEPTH_TEST);
    cull_ = glIsEnabled(GL_CULL_FACE);
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &unpack_alignment_);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
  }
  ~GlStateGuard() {
    gl_.BindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(fbo_));
    gl_.UseProgram(static_cast<GLuint>(program_));
    gl_.BindVertexArray(static_cast<GLuint>(vao_));
    gl_.BindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(vbo_));
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture_));
    gl_.ActiveTexture(static_cast<GLenum>(active_texture_));
    glViewport(viewport_[0], viewport_[1], viewport_[2], viewport_[3]);
    glScissor(scissor_box_[0], scissor_box_[1], scissor_box_[2], scissor_box_[3]);
    scissor_ ? glEnable(GL_SCISSOR_TEST) : glDisable(GL_SCISSOR_TEST);
    blend_ ? glEnable(GL_BLEND) : glDisable(GL_BLEND);
    depth_ ? glEnable(GL_DEPTH_TEST) : glDisable(GL_DEPTH_TEST);
    cull_ ? glEnable(GL_CULL_FACE) : glDisable(GL_CULL_FACE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, unpack_alignment_);
  }

private:
  GpuHwRenderer::Gl &gl_;
  GLint fbo_ = 0, program_ = 0, vao_ = 0, vbo_ = 0, active_texture_ = 0,
        texture_ = 0, unpack_alignment_ = 4;
  GLint viewport_[4] = {};
  GLint scissor_box_[4] = {};
  GLboolean scissor_ = GL_FALSE, blend_ = GL_FALSE, depth_ = GL_FALSE,
            cull_ = GL_FALSE;
};

} // namespace

GpuHwRenderer::GpuHwRenderer() = default;

GpuHwRenderer::~GpuHwRenderer() { shutdown(); }

bool GpuHwRenderer::ready() const { return gl_ != nullptr; }

unsigned int GpuHwRenderer::output_texture() const {
  return gl_ ? gl_->output_texture : 0u;
}

bool GpuHwRenderer::init(int scale) {
  shutdown();
  auto gl = std::make_unique<Gl>();
  if (!gl->load()) {
    LOG_WARN("GpuHwRenderer: OpenGL 3.3 functions unavailable; upscaling disabled");
    return false;
  }
  gl->draw_program = gl->link(kDrawVertexShader, kDrawFragmentShader);
  gl->copy_program = gl->link(kCopyVertexShader, kCopyFragmentShader);
  if (gl->draw_program == 0 || gl->copy_program == 0) {
    if (gl->draw_program) gl->DeleteProgram(gl->draw_program);
    if (gl->copy_program) gl->DeleteProgram(gl->copy_program);
    return false;
  }
  Gl::DrawUniforms &u = gl->draw_u;
  const GLuint p = gl->draw_program;
  u.vram = gl->GetUniformLocation(p, "u_vram");
  u.textured = gl->GetUniformLocation(p, "u_textured");
  u.raw = gl->GetUniformLocation(p, "u_raw");
  u.depth = gl->GetUniformLocation(p, "u_depth");
  u.tex_base = gl->GetUniformLocation(p, "u_tex_base");
  u.clut = gl->GetUniformLocation(p, "u_clut");
  u.window = gl->GetUniformLocation(p, "u_window");
  u.pass = gl->GetUniformLocation(p, "u_pass");
  u.set_mask = gl->GetUniformLocation(p, "u_set_mask");
  u.color_copy = gl->GetUniformLocation(p, "u_color_copy");
  u.scale = gl->GetUniformLocation(p, "u_scale");
  u.check_mask = gl->GetUniformLocation(p, "u_check_mask");
  gl->copy_vram = gl->GetUniformLocation(gl->copy_program, "u_vram");
  gl->copy_scale = gl->GetUniformLocation(gl->copy_program, "u_scale");

  GlStateGuard guard(*gl);
  gl->GenVertexArrays(1, &gl->vao);
  gl->GenBuffers(1, &gl->vbo);
  gl->BindVertexArray(gl->vao);
  gl->BindBuffer(GL_ARRAY_BUFFER, gl->vbo);
  const GLsizei stride = sizeof(GpuHwVertex);
  gl->VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<const void *>(offsetof(GpuHwVertex, x)));
  gl->VertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, stride,
                          reinterpret_cast<const void *>(offsetof(GpuHwVertex, color)));
  gl->VertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<const void *>(offsetof(GpuHwVertex, u)));
  gl->EnableVertexAttribArray(0);
  gl->EnableVertexAttribArray(1);
  gl->EnableVertexAttribArray(2);
  gl->GenVertexArrays(1, &gl->empty_vao);

  gl->native_texture =
      make_texture(GL_R16UI, kVramW, kVramH, GL_RED_INTEGER, GL_UNSIGNED_SHORT);

  gl_ = std::move(gl);
  scale_ = 0;
  if (!set_scale(scale)) {
    shutdown();
    return false;
  }
  LOG_INFO("GpuHwRenderer: OpenGL upscaler ready at %dx", scale_);
  return true;
}

void GpuHwRenderer::shutdown() {
  if (!gl_) {
    return;
  }
  release_targets();
  Gl &gl = *gl_;
  if (gl.native_texture) glDeleteTextures(1, &gl.native_texture);
  if (gl.vbo) gl.DeleteBuffers(1, &gl.vbo);
  if (gl.vao) gl.DeleteVertexArrays(1, &gl.vao);
  if (gl.empty_vao) gl.DeleteVertexArrays(1, &gl.empty_vao);
  if (gl.draw_program) gl.DeleteProgram(gl.draw_program);
  if (gl.copy_program) gl.DeleteProgram(gl.copy_program);
  gl_.reset();
  has_output_ = false;
  software_present_ = true;
}

void GpuHwRenderer::release_targets() {
  Gl &gl = *gl_;
  const GLuint fbos[] = {gl.color_fbo, gl.temp_fbo, gl.output_fbo, gl.sample_fbo};
  for (GLuint fbo : fbos) {
    if (fbo) gl.DeleteFramebuffers(1, &fbo);
  }
  const GLuint textures[] = {gl.color_texture, gl.temp_texture, gl.sample_texture,
                             gl.output_texture};
  for (GLuint tex : textures) {
    if (tex) glDeleteTextures(1, &tex);
  }
  gl.color_fbo = gl.temp_fbo = gl.output_fbo = gl.sample_fbo = 0;
  gl.sample_texture = 0;
  gl.color_texture = gl.temp_texture = gl.output_texture = 0;
  gl.output_alloc_w = gl.output_alloc_h = 0;
}

bool GpuHwRenderer::create_targets() {
  Gl &gl = *gl_;
  const int w = kVramW * scale_;
  const int h = kVramH * scale_;
  gl.color_texture = make_texture(GL_RGBA8, w, h, GL_RGBA, GL_UNSIGNED_BYTE);
  gl.temp_texture = make_texture(GL_RGBA8, w, h, GL_RGBA, GL_UNSIGNED_BYTE);
  gl.color_fbo = gl.make_fbo(gl.color_texture);
  gl.temp_fbo = gl.make_fbo(gl.temp_texture);
  gl.sample_texture = make_texture(GL_RGBA8, w, h, GL_RGBA, GL_UNSIGNED_BYTE);
  gl.sample_fbo = gl.make_fbo(gl.sample_texture);
  sample_stale_.fill(true);
  return gl.color_fbo != 0 && gl.temp_fbo != 0;
}

bool GpuHwRenderer::set_scale(int scale) {
  if (!gl_) {
    return false;
  }
  GLint max_size = 0;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_size);
  scale = std::clamp(scale, 1, kMaxScale);
  while (scale > 1 && kVramW * scale > max_size) {
    --scale;
  }
  if (scale == scale_) {
    return true;
  }
  GlStateGuard guard(*gl_);
  release_targets();
  scale_ = scale;
  if (!create_targets()) {
    return false;
  }
  native_to_color(0, 0, kVramW, kVramH);
  has_output_ = false;
  return true;
}

void GpuHwRenderer::replay(const GpuHwStream &stream) {
  if (!gl_ || stream.empty()) {
    return;
  }
  GlStateGuard guard(*gl_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
  for (const GpuHwCommand &cmd : stream.commands) {
    switch (cmd.op) {
    case GpuHwOp::Triangles:
      draw_triangles(stream, cmd);
      break;
    case GpuHwOp::Fill:
      fill(cmd);
      break;
    case GpuHwOp::VramWrite:
      upload_native(cmd.x, cmd.y, cmd.w, cmd.h, stream.pixels.data() + cmd.first);
      native_to_color(cmd.x, cmd.y, cmd.w, cmd.h);
      break;
    case GpuHwOp::VramCopy:
      copy(cmd);
      break;
    case GpuHwOp::SyncPage:
      upload_native(cmd.x, cmd.y, cmd.w, cmd.h, stream.pixels.data() + cmd.first);
      break;
    case GpuHwOp::FullSync:
      upload_native(0, 0, kVramW, kVramH, stream.pixels.data() + cmd.first);
      native_to_color(0, 0, kVramW, kVramH);
      break;
    case GpuHwOp::Present:
      present(cmd);
      break;
    }
  }
}

void GpuHwRenderer::upload_native(int x, int y, int w, int h, const u16 *pixels) {
  glBindTexture(GL_TEXTURE_2D, gl_->native_texture);
  glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, w, h, GL_RED_INTEGER,
                  GL_UNSIGNED_SHORT, pixels);
}

void GpuHwRenderer::native_to_color(int x, int y, int w, int h) {
  Gl &gl = *gl_;
  gl.BindFramebuffer(GL_FRAMEBUFFER, gl.color_fbo);
  glViewport(0, 0, kVramW * scale_, kVramH * scale_);
  glEnable(GL_SCISSOR_TEST);
  glScissor(x * scale_, y * scale_, w * scale_, h * scale_);
  glDisable(GL_BLEND);
  gl.UseProgram(gl.copy_program);
  gl.ActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, gl.native_texture);
  gl.Uniform1i(gl.copy_vram, 0);
  gl.Uniform1i(gl.copy_scale, scale_);
  gl.BindVertexArray(gl.empty_vao);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  mark_stale(x, y, x + w - 1, y + h - 1);
}

void GpuHwRenderer::fill(const GpuHwCommand &cmd) {
  Gl &gl = *gl_;
  gl.BindFramebuffer(GL_FRAMEBUFFER, gl.color_fbo);
  glEnable(GL_SCISSOR_TEST);
  glScissor(cmd.x * scale_, cmd.y * scale_, cmd.w * scale_, cmd.h * scale_);
  glClearColor(static_cast<float>(cmd.color & 0xFFu) / 255.0f,
               static_cast<float>((cmd.color >> 8) & 0xFFu) / 255.0f,
               static_cast<float>((cmd.color >> 16) & 0xFFu) / 255.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  mark_stale(cmd.x, cmd.y, cmd.x + cmd.w - 1, cmd.y + cmd.h - 1);
}

void GpuHwRenderer::copy(const GpuHwCommand &cmd) {
  // Source and destination may overlap: go through the scratch target.
  Gl &gl = *gl_;
  glDisable(GL_SCISSOR_TEST);
  const int s = scale_;
  gl.BindFramebuffer(GL_READ_FRAMEBUFFER, gl.color_fbo);
  gl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, gl.temp_fbo);
  gl.BlitFramebuffer(cmd.src_x * s, cmd.src_y * s, (cmd.src_x + cmd.w) * s,
                     (cmd.src_y + cmd.h) * s, 0, 0, cmd.w * s, cmd.h * s,
                     GL_COLOR_BUFFER_BIT, GL_NEAREST);
  gl.BindFramebuffer(GL_READ_FRAMEBUFFER, gl.temp_fbo);
  gl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, gl.color_fbo);
  gl.BlitFramebuffer(0, 0, cmd.w * s, cmd.h * s, cmd.x * s, cmd.y * s,
                     (cmd.x + cmd.w) * s, (cmd.y + cmd.h) * s,
                     GL_COLOR_BUFFER_BIT, GL_NEAREST);
  mark_stale(cmd.x, cmd.y, cmd.x + cmd.w - 1, cmd.y + cmd.h - 1);
}

void GpuHwRenderer::draw_triangles(const GpuHwStream &stream,
                                   const GpuHwCommand &cmd) {
  if (cmd.count == 0 || cmd.clip_x1 < cmd.clip_x0 || cmd.clip_y1 < cmd.clip_y0) {
    return;
  }
  Gl &gl = *gl_;
  // Affected VRAM area: vertex bounds within the draw area.
  float min_x = 1e9f, min_y = 1e9f, max_x = -1e9f, max_y = -1e9f;
  for (u32 i = 0; i < cmd.count; ++i) {
    const GpuHwVertex &v = stream.vertices[cmd.first + i];
    min_x = std::min(min_x, v.x);
    max_x = std::max(max_x, v.x);
    min_y = std::min(min_y, v.y);
    max_y = std::max(max_y, v.y);
  }
  const int area_x0 = std::max<int>(static_cast<int>(min_x), cmd.clip_x0);
  const int area_y0 = std::max<int>(static_cast<int>(min_y), cmd.clip_y0);
  const int area_x1 = std::min<int>(static_cast<int>(max_x) + 1, cmd.clip_x1);
  const int area_y1 = std::min<int>(static_cast<int>(max_y) + 1, cmd.clip_y1);
  const bool textured = (cmd.flags & gpu_hw::kTextured) != 0;
  const bool semi = (cmd.flags & gpu_hw::kSemiTransparent) != 0;
  const bool check_mask = (cmd.flags & gpu_hw::kCheckMask) != 0;

  // Snapshot what the draw reads from the colour buffer before it writes:
  // 15-bit texture pages, and for blended mask-checked draws the mask bits.
  if (textured && cmd.tex_depth == 2) {
    refresh_sample_pages(cmd);
  }
  if (semi && check_mask) {
    refresh_sample_rect(area_x0, area_y0, area_x1, area_y1);
  }

  gl.BindFramebuffer(GL_FRAMEBUFFER, gl.color_fbo);
  glViewport(0, 0, kVramW * scale_, kVramH * scale_);
  glEnable(GL_SCISSOR_TEST);
  glScissor(cmd.clip_x0 * scale_, cmd.clip_y0 * scale_,
            (cmd.clip_x1 - cmd.clip_x0 + 1) * scale_,
            (cmd.clip_y1 - cmd.clip_y0 + 1) * scale_);

  gl.UseProgram(gl.draw_program);
  gl.ActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, gl.native_texture);
  const Gl::DrawUniforms &u = gl.draw_u;
  gl.Uniform1i(u.vram, 0);
  gl.Uniform1i(u.textured, textured ? 1 : 0);
  gl.Uniform1i(u.raw, (cmd.flags & gpu_hw::kRawTexture) ? 1 : 0);
  gl.Uniform1i(u.depth, cmd.tex_depth);
  gl.Uniform2i(u.tex_base, cmd.tex_base_x, cmd.tex_base_y);
  gl.Uniform2i(u.clut, cmd.clut_x, cmd.clut_y);
  gl.Uniform4i(u.window, cmd.tw_mask_x, cmd.tw_mask_y, cmd.tw_off_x,
               cmd.tw_off_y);
  gl.Uniform1i(u.set_mask, (cmd.flags & gpu_hw::kSetMask) ? 1 : 0);
  gl.Uniform1i(u.scale, scale_);
  gl.Uniform1i(u.color_copy, 1);
  gl.Uniform1i(u.check_mask, 0);
  gl.ActiveTexture(GL_TEXTURE0 + 1);
  glBindTexture(GL_TEXTURE_2D, gl.sample_texture);
  gl.ActiveTexture(GL_TEXTURE0);

  gl.BindVertexArray(gl.vao);
  gl.BindBuffer(GL_ARRAY_BUFFER, gl.vbo);
  gl.BufferData(GL_ARRAY_BUFFER,
                static_cast<std::ptrdiff_t>(cmd.count * sizeof(GpuHwVertex)),
                stream.vertices.data() + cmd.first, GL_STREAM_DRAW);

  const auto set_blend = [&](u8 mode) {
    glEnable(GL_BLEND);
    // Colour follows the PS1 equation; alpha carries the mask bit as-is.
    switch (mode & 3u) {
    case 0: // B/2 + F/2
      gl.BlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
      gl.BlendColor(0.5f, 0.5f, 0.5f, 0.5f);
      gl.BlendFuncSeparate(GL_CONSTANT_COLOR, GL_CONSTANT_COLOR, GL_ONE, GL_ZERO);
      break;
    case 1: // B + F
      gl.BlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
      gl.BlendFuncSeparate(GL_ONE, GL_ONE, GL_ONE, GL_ZERO);
      break;
    case 2: // B - F
      gl.BlendEquationSeparate(GL_FUNC_REVERSE_SUBTRACT, GL_FUNC_ADD);
      gl.BlendFuncSeparate(GL_ONE, GL_ONE, GL_ONE, GL_ZERO);
      break;
    default: // B + F/4
      gl.BlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
      gl.BlendColor(0.25f, 0.25f, 0.25f, 0.25f);
      gl.BlendFuncSeparate(GL_CONSTANT_COLOR, GL_ONE, GL_ONE, GL_ZERO);
      break;
    }
  };

  // Opaque output; with mask checking, pixels whose mask bit (alpha) is set
  // keep their colour: out = src * (1 - dst_a) + dst * dst_a. Blending
  // reads the live destination, so this is exact even within one batch.
  const auto set_opaque = [&]() {
    if (!check_mask) {
      glDisable(GL_BLEND);
      return;
    }
    glEnable(GL_BLEND);
    gl.BlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    gl.BlendFuncSeparate(GL_ONE_MINUS_DST_ALPHA, GL_DST_ALPHA,
                         GL_ONE_MINUS_DST_ALPHA, GL_DST_ALPHA);
  };

  // Blended output cannot also express the mask test, so blended
  // mask-checked fragments are discarded in the shader using the snapshot
  // taken above (exact while a batch's triangles do not overlap).
  const auto set_semi = [&](u8 mode) {
    set_blend(mode);
    gl.Uniform1i(u.check_mask, check_mask ? 1 : 0);
  };

  const GLsizei count = static_cast<GLsizei>(cmd.count);
  if (!semi) {
    set_opaque();
    gl.Uniform1i(u.pass, 0);
    glDrawArrays(GL_TRIANGLES, 0, count);
  } else if (!textured) {
    set_semi(cmd.semi_mode);
    gl.Uniform1i(u.pass, 0);
    glDrawArrays(GL_TRIANGLES, 0, count);
  } else {
    // Texel bit 15 selects blending per texel: opaque texels first, then
    // the semi-transparent ones blended.
    set_opaque();
    gl.Uniform1i(u.pass, 1);
    glDrawArrays(GL_TRIANGLES, 0, count);
    set_semi(cmd.semi_mode);
    gl.Uniform1i(u.pass, 2);
    glDrawArrays(GL_TRIANGLES, 0, count);
  }
  glDisable(GL_BLEND);
  mark_stale(area_x0, area_y0, area_x1, area_y1);
}

void GpuHwRenderer::mark_stale(int x0, int y0, int x1, int y1) {
  x0 = std::clamp(x0, 0, kVramW - 1);
  x1 = std::clamp(x1, 0, kVramW - 1);
  y0 = std::clamp(y0, 0, kVramH - 1);
  y1 = std::clamp(y1, 0, kVramH - 1);
  if (x1 < x0 || y1 < y0) {
    return;
  }
  for (int py = y0 / gpu_hw::kPageHeight; py <= y1 / gpu_hw::kPageHeight; ++py) {
    for (int px = x0 / gpu_hw::kPageWidth; px <= x1 / gpu_hw::kPageWidth; ++px) {
      sample_stale_[static_cast<size_t>(py * gpu_hw::kPageColumns + px)] = true;
    }
  }
}

void GpuHwRenderer::refresh_sample_pages(const GpuHwCommand &cmd) {
  // A 15-bit texture spans 256 VRAM columns (four pages) from its base.
  const int page_x = cmd.tex_base_x / gpu_hw::kPageWidth;
  const int page_y = cmd.tex_base_y / gpu_hw::kPageHeight;
  for (int i = 0; i < 4; ++i) {
    refresh_sample_page((page_x + i) & (gpu_hw::kPageColumns - 1), page_y);
  }
}

void GpuHwRenderer::refresh_sample_rect(int x0, int y0, int x1, int y1) {
  x0 = std::clamp(x0, 0, kVramW - 1);
  x1 = std::clamp(x1, 0, kVramW - 1);
  y0 = std::clamp(y0, 0, kVramH - 1);
  y1 = std::clamp(y1, 0, kVramH - 1);
  for (int py = y0 / gpu_hw::kPageHeight; py <= y1 / gpu_hw::kPageHeight; ++py) {
    for (int px = x0 / gpu_hw::kPageWidth; px <= x1 / gpu_hw::kPageWidth; ++px) {
      refresh_sample_page(px, py);
    }
  }
}

void GpuHwRenderer::refresh_sample_page(int px, int page_y) {
  Gl &gl = *gl_;
  {
    const size_t page = static_cast<size_t>(page_y * gpu_hw::kPageColumns + px);
    if (!sample_stale_[page]) {
      return;
    }
    sample_stale_[page] = false;
    const int x0 = px * gpu_hw::kPageWidth * scale_;
    const int y0 = page_y * gpu_hw::kPageHeight * scale_;
    const int x1 = x0 + gpu_hw::kPageWidth * scale_;
    const int y1 = y0 + gpu_hw::kPageHeight * scale_;
    gl.BindFramebuffer(GL_READ_FRAMEBUFFER, gl.color_fbo);
    gl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, gl.sample_fbo);
    // Blits ignore the scissor rectangle only when it is disabled.
    GLboolean scissor = glIsEnabled(GL_SCISSOR_TEST);
    glDisable(GL_SCISSOR_TEST);
    gl.BlitFramebuffer(x0, y0, x1, y1, x0, y0, x1, y1, GL_COLOR_BUFFER_BIT,
                       GL_NEAREST);
    if (scissor) {
      glEnable(GL_SCISSOR_TEST);
    }
  }
}

void GpuHwRenderer::present(const GpuHwCommand &cmd) {
  software_present_ = (cmd.flags & gpu_hw::kSoftwarePresent) != 0;
  has_output_ = true;
  if (software_present_) {
    return;
  }
  Gl &gl = *gl_;
  const int w = cmd.w * scale_;
  const int h = cmd.h * scale_;
  if (w != gl.output_alloc_w || h != gl.output_alloc_h) {
    if (gl.output_fbo) gl.DeleteFramebuffers(1, &gl.output_fbo);
    if (gl.output_texture) glDeleteTextures(1, &gl.output_texture);
    gl.output_texture = make_texture(GL_RGBA8, w, h, GL_RGBA, GL_UNSIGNED_BYTE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gl.output_fbo = gl.make_fbo(gl.output_texture);
    gl.output_alloc_w = w;
    gl.output_alloc_h = h;
  }
  glDisable(GL_SCISSOR_TEST);
  gl.BindFramebuffer(GL_READ_FRAMEBUFFER, gl.color_fbo);
  gl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, gl.output_fbo);
  gl.BlitFramebuffer(cmd.x * scale_, cmd.y * scale_, (cmd.x + cmd.w) * scale_,
                     (cmd.y + cmd.h) * scale_, 0, 0, w, h, GL_COLOR_BUFFER_BIT,
                     GL_NEAREST);
  // Alpha holds VRAM mask bits; the presented picture must be opaque.
  gl.BindFramebuffer(GL_FRAMEBUFFER, gl.output_fbo);
  glViewport(0, 0, w, h);
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_TRUE);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  output_width_ = w;
  output_height_ = h;
}

bool GpuHwRenderer::read_output(std::vector<u32> &rgba) const {
  if (!gl_ || !has_output_ || software_present_ || gl_->output_fbo == 0) {
    return false;
  }
  Gl &gl = *gl_;
  GLint previous = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previous);
  GLint pack_alignment = 4;
  glGetIntegerv(GL_PACK_ALIGNMENT, &pack_alignment);
  gl.BindFramebuffer(GL_FRAMEBUFFER, gl.output_fbo);
  glPixelStorei(GL_PACK_ALIGNMENT, 4);
  rgba.resize(static_cast<size_t>(output_width_) * output_height_);
  glReadPixels(0, 0, output_width_, output_height_, GL_RGBA, GL_UNSIGNED_BYTE,
               rgba.data());
  glPixelStorei(GL_PACK_ALIGNMENT, pack_alignment);
  gl.BindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(previous));
  return true;
}
bool GpuHwRenderer::read_color_buffer(std::vector<u32> &rgba, int &width,
                                      int &height) const {
  if (!gl_ || gl_->color_fbo == 0) {
    return false;
  }
  Gl &gl = *gl_;
  width = kVramW * scale_;
  height = kVramH * scale_;
  GLint previous = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previous);
  gl.BindFramebuffer(GL_FRAMEBUFFER, gl.color_fbo);
  glPixelStorei(GL_PACK_ALIGNMENT, 4);
  rgba.resize(static_cast<size_t>(width) * height);
  glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
  gl.BindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(previous));
  return true;
}