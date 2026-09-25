#include "platform/gpu_hardware_rasterizer.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <utility>

#ifndef GL_COMPUTE_SHADER
#define GL_COMPUTE_SHADER 0x91B9
#endif
#ifndef GL_SHADER_STORAGE_BUFFER
#define GL_SHADER_STORAGE_BUFFER 0x90D2
#endif
#ifndef GL_SHADER_STORAGE_BARRIER_BIT
#define GL_SHADER_STORAGE_BARRIER_BIT 0x2000
#endif
#ifndef GL_DYNAMIC_DRAW
#define GL_DYNAMIC_DRAW 0x88E8
#endif

namespace {

constexpr size_t kVramPixels =
    static_cast<size_t>(psx::VRAM_WIDTH) *
    static_cast<size_t>(psx::VRAM_HEIGHT);

template <typename T>
bool load_proc(T& out, const char* name) {
  out = reinterpret_cast<T>(SDL_GL_GetProcAddress(name));
  return out != nullptr;
}

using CreateShaderFn = GLuint (APIENTRY*)(GLenum);
using ShaderSourceFn = void (APIENTRY*)(GLuint, GLsizei, const GLchar* const*, const GLint*);
using CompileShaderFn = void (APIENTRY*)(GLuint);
using GetShaderivFn = void (APIENTRY*)(GLuint, GLenum, GLint*);
using GetShaderInfoLogFn = void (APIENTRY*)(GLuint, GLsizei, GLsizei*, GLchar*);
using DeleteShaderFn = void (APIENTRY*)(GLuint);
using CreateProgramFn = GLuint (APIENTRY*)();
using AttachShaderFn = void (APIENTRY*)(GLuint, GLuint);
using LinkProgramFn = void (APIENTRY*)(GLuint);
using GetProgramivFn = void (APIENTRY*)(GLuint, GLenum, GLint*);
using GetProgramInfoLogFn = void (APIENTRY*)(GLuint, GLsizei, GLsizei*, GLchar*);
using DeleteProgramFn = void (APIENTRY*)(GLuint);
using UseProgramFn = void (APIENTRY*)(GLuint);
using GetUniformLocationFn = GLint (APIENTRY*)(GLuint, const GLchar*);
using Uniform1iFn = void (APIENTRY*)(GLint, GLint);
using Uniform4iFn = void (APIENTRY*)(GLint, GLint, GLint, GLint, GLint);
using GenBuffersFn = void (APIENTRY*)(GLsizei, GLuint*);
using BindBufferFn = void (APIENTRY*)(GLenum, GLuint);
using BufferDataFn = void (APIENTRY*)(GLenum, GLsizeiptr, const void*, GLenum);
using BufferSubDataFn = void (APIENTRY*)(GLenum, GLintptr, GLsizeiptr, const void*);
using GetBufferSubDataFn = void (APIENTRY*)(GLenum, GLintptr, GLsizeiptr, void*);
using DeleteBuffersFn = void (APIENTRY*)(GLsizei, const GLuint*);
using BindBufferBaseFn = void (APIENTRY*)(GLenum, GLuint, GLuint);
using DispatchComputeFn = void (APIENTRY*)(GLuint, GLuint, GLuint);
using MemoryBarrierFn = void (APIENTRY*)(GLbitfield);

CreateShaderFn pCreateShader = nullptr;
ShaderSourceFn pShaderSource = nullptr;
CompileShaderFn pCompileShader = nullptr;
GetShaderivFn pGetShaderiv = nullptr;
GetShaderInfoLogFn pGetShaderInfoLog = nullptr;
DeleteShaderFn pDeleteShader = nullptr;
CreateProgramFn pCreateProgram = nullptr;
AttachShaderFn pAttachShader = nullptr;
LinkProgramFn pLinkProgram = nullptr;
GetProgramivFn pGetProgramiv = nullptr;
GetProgramInfoLogFn pGetProgramInfoLog = nullptr;
DeleteProgramFn pDeleteProgram = nullptr;
UseProgramFn pUseProgram = nullptr;
GetUniformLocationFn pGetUniformLocation = nullptr;
Uniform1iFn pUniform1i = nullptr;
Uniform4iFn pUniform4i = nullptr;
GenBuffersFn pGenBuffers = nullptr;
BindBufferFn pBindBuffer = nullptr;
BufferDataFn pBufferData = nullptr;
BufferSubDataFn pBufferSubData = nullptr;
GetBufferSubDataFn pGetBufferSubData = nullptr;
DeleteBuffersFn pDeleteBuffers = nullptr;
BindBufferBaseFn pBindBufferBase = nullptr;
DispatchComputeFn pDispatchCompute = nullptr;
MemoryBarrierFn pMemoryBarrier = nullptr;

const char* kComputeShader = R"GLSL(
#version 430 core

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(std430, binding = 0) buffer VramBuffer {
    uint vram[];
};

uniform int uMode;
uniform ivec4 uBounds;
uniform ivec4 uV0;
uniform ivec4 uV1;
uniform ivec4 uV2;
uniform ivec4 uC0;
uniform ivec4 uC1;
uniform ivec4 uC2;
uniform ivec4 uTex0;
uniform ivec4 uTex1;
uniform int uClutRow;
uniform int uFlags;
uniform int uSemiMode;
uniform ivec4 uRect;

const int FLAG_DITHER = 1;
const int FLAG_SEMI = 2;
const int FLAG_FORCE_MASK = 4;
const int FLAG_CHECK_MASK = 8;
const int FLAG_RAW_TEXTURE = 16;
const int FLAG_RECT_X_FLIP = 32;
const int FLAG_RECT_Y_FLIP = 64;

int edge(ivec2 a, ivec2 b, ivec2 p) {
    return (b.x - a.x) * (p.y - a.y) -
           (b.y - a.y) * (p.x - a.x);
}

bool top_left(ivec2 a, ivec2 b) {
    int dy = b.y - a.y;
    int dx = b.x - a.x;
    return (dy < 0) || (dy == 0 && dx > 0);
}

bool edge_inside(int w, bool tl) {
    return (w > 0) || (w == 0 && tl);
}

int dither_bias(int x, int y) {
    const int table[16] = int[16](
        -4,  0, -3,  1,
         2, -2,  3, -1,
        -3,  1, -4,  0,
         3, -1,  2, -2
    );
    return table[((y & 3) << 2) | (x & 3)];
}

int clamp_u8(int v) {
    return clamp(v, 0, 255);
}

uint load_vram(int x, int y) {
    int px = x & 1023;
    int py = y & 511;
    return vram[py * 1024 + px] & 0xFFFFu;
}

void store_vram(int x, int y, uint value) {
    vram[y * 1024 + x] = value & 0xFFFFu;
}

uint blend_pixel(uint src, uint dst) {
    uint color15 = src & 0x7FFFu;
    uint outv = src & 0x8000u;

    int fr = int(color15 & 31u);
    int fg = int((color15 >> 5) & 31u);
    int fb = int((color15 >> 10) & 31u);
    int br = int(dst & 31u);
    int bg = int((dst >> 5) & 31u);
    int bb = int((dst >> 10) & 31u);
    int rr = fr;
    int rg = fg;
    int rb = fb;

    switch (uSemiMode & 3) {
    case 0:
        rr = (br + fr) >> 1;
        rg = (bg + fg) >> 1;
        rb = (bb + fb) >> 1;
        break;
    case 1:
        rr = min(31, br + fr);
        rg = min(31, bg + fg);
        rb = min(31, bb + fb);
        break;
    case 2:
        rr = max(0, br - fr);
        rg = max(0, bg - fg);
        rb = max(0, bb - fb);
        break;
    default:
        rr = min(31, br + (fr >> 2));
        rg = min(31, bg + (fg >> 2));
        rb = min(31, bb + (fb >> 2));
        break;
    }

    return outv |
           uint(rr & 31) |
           (uint(rg & 31) << 5) |
           (uint(rb & 31) << 10);
}

void write_pixel(int x, int y, uint color, bool semi) {
    int index = y * 1024 + x;
    uint dst = vram[index] & 0xFFFFu;
    if ((uFlags & FLAG_CHECK_MASK) != 0 && (dst & 0x8000u) != 0u) {
        return;
    }

    uint outv = semi ? blend_pixel(color, dst) : (color & 0xFFFFu);
    if ((uFlags & FLAG_FORCE_MASK) != 0) {
        outv |= 0x8000u;
    }
    vram[index] = outv & 0xFFFFu;
}

uint pack_rgb15(int r, int g, int b, int x, int y, bool dither) {
    if (dither) {
        int d = dither_bias(x, y);
        r = clamp_u8(r + d);
        g = clamp_u8(g + d);
        b = clamp_u8(b + d);
    }
    return uint((r >> 3) & 31) |
           (uint((g >> 3) & 31) << 5) |
           (uint((b >> 3) & 31) << 10);
}

uint modulate_texel(uint texel, int mr, int mg, int mb, int x, int y) {
    int tr = int(texel & 31u);
    int tg = int((texel >> 5) & 31u);
    int tb = int((texel >> 10) & 31u);

    int rr5 = min(31, (tr * mr) >> 7);
    int rg5 = min(31, (tg * mg) >> 7);
    int rb5 = min(31, (tb * mb) >> 7);

    if ((uFlags & FLAG_DITHER) != 0) {
        int d = dither_bias(x, y);
        int rr = clamp_u8((rr5 << 3) + d) >> 3;
        int rg = clamp_u8((rg5 << 3) + d) >> 3;
        int rb = clamp_u8((rb5 << 3) + d) >> 3;
        return uint(rr & 31) |
               (uint(rg & 31) << 5) |
               (uint(rb & 31) << 10) |
               (texel & 0x8000u);
    }

    return uint(rr5 & 31) |
           (uint(rg5 & 31) << 5) |
           (uint(rb5 & 31) << 10) |
           (texel & 0x8000u);
}

uint sample_texel(int u, int v) {
    int uw = ((u & 255) & uTex0.x) | uTex0.z;
    int vw = ((v & 255) & uTex0.y) | uTex0.w;
    int depth = uTex1.x;
    int texBaseX = uTex1.y;
    int texBaseY = uTex1.z;
    int clutX = uTex1.w;

    int ty = (texBaseY + vw) & 511;
    int row = ty * 1024;

    if (depth == 0) {
        int wordX = (texBaseX + (uw >> 2)) & 1023;
        uint packed = vram[row + wordX] & 0xFFFFu;
        int index = int((packed >> uint((uw & 3) * 4)) & 15u);
        int cx = (clutX + index) & 1023;
        return vram[uClutRow + cx] & 0xFFFFu;
    }
    if (depth == 1) {
        int wordX = (texBaseX + (uw >> 1)) & 1023;
        uint packed = vram[row + wordX] & 0xFFFFu;
        int index = int((packed >> uint((uw & 1) * 8)) & 255u);
        int cx = (clutX + index) & 1023;
        return vram[uClutRow + cx] & 0xFFFFu;
    }

    int tx = (texBaseX + uw) & 1023;
    return vram[row + tx] & 0xFFFFu;
}

void draw_triangle_pixel(int x, int y) {
    ivec2 p = ivec2(x, y);
    ivec2 p0 = uV0.xy;
    ivec2 p1 = uV1.xy;
    ivec2 p2 = uV2.xy;

    int area = edge(p0, p1, p2);
    if (area <= 0) {
        return;
    }

    int w0 = edge(p1, p2, p);
    int w1 = edge(p2, p0, p);
    int w2 = edge(p0, p1, p);

    if (!edge_inside(w0, top_left(p1, p2)) ||
        !edge_inside(w1, top_left(p2, p0)) ||
        !edge_inside(w2, top_left(p0, p1))) {
        return;
    }

    bool primitiveSemi = (uFlags & FLAG_SEMI) != 0;

    if (uMode == 0) {
        uint outv = uint((uC0.x >> 3) & 31) |
                    (uint((uC0.y >> 3) & 31) << 5) |
                    (uint((uC0.z >> 3) & 31) << 10);
        write_pixel(x, y, outv, primitiveSemi);
        return;
    }

    if (uMode == 1) {
        int r = clamp_u8((w0 * uC0.x + w1 * uC1.x + w2 * uC2.x) / area);
        int g = clamp_u8((w0 * uC0.y + w1 * uC1.y + w2 * uC2.y) / area);
        int b = clamp_u8((w0 * uC0.z + w1 * uC1.z + w2 * uC2.z) / area);
        uint outv = pack_rgb15(
            r, g, b, x, y,
            (uFlags & FLAG_DITHER) != 0);
        write_pixel(x, y, outv, primitiveSemi);
        return;
    }

    int tu = (w0 * uV0.z + w1 * uV1.z + w2 * uV2.z) / area;
    int tv = (w0 * uV0.w + w1 * uV1.w + w2 * uV2.w) / area;
    uint texel = sample_texel(tu, tv);
    if (texel == 0u) {
        return;
    }

    int mr = uC0.x;
    int mg = uC0.y;
    int mb = uC0.z;
    if (uMode == 3) {
        mr = clamp_u8((w0 * uC0.x + w1 * uC1.x + w2 * uC2.x) / area);
        mg = clamp_u8((w0 * uC0.y + w1 * uC1.y + w2 * uC2.y) / area);
        mb = clamp_u8((w0 * uC0.z + w1 * uC1.z + w2 * uC2.z) / area);
    }

    uint outv = texel;
    if ((uFlags & FLAG_RAW_TEXTURE) == 0) {
        outv = modulate_texel(texel, mr, mg, mb, x, y);
    }

    bool texelSemi =
        primitiveSemi && ((texel & 0x8000u) != 0u);
    write_pixel(x, y, outv, texelSemi);
}

void draw_rect_pixel(int x, int y) {
    int dx = x - uRect.x;
    int dy = y - uRect.y;

    if (uMode == 4) {
        uint outv = uint((uC0.x >> 3) & 31) |
                    (uint((uC0.y >> 3) & 31) << 5) |
                    (uint((uC0.z >> 3) & 31) << 10);
        write_pixel(
            x, y, outv,
            (uFlags & FLAG_SEMI) != 0);
        return;
    }

    int srcDx =
        ((uFlags & FLAG_RECT_X_FLIP) != 0)
            ? (uRect.z - 1 - dx)
            : dx;
    int srcDy =
        ((uFlags & FLAG_RECT_Y_FLIP) != 0)
            ? (uRect.w - 1 - dy)
            : dy;

    int tu = uV0.z + srcDx;
    int tv = uV0.w + srcDy;
    uint texel = sample_texel(tu, tv);
    if (texel == 0u) {
        return;
    }

    uint outv = texel;
    if ((uFlags & FLAG_RAW_TEXTURE) == 0) {
        outv = modulate_texel(
            texel, uC0.x, uC0.y, uC0.z, x, y);
    }

    bool texelSemi =
        ((uFlags & FLAG_SEMI) != 0) &&
        ((texel & 0x8000u) != 0u);
    write_pixel(x, y, outv, texelSemi);
}

void main() {
    int x = uBounds.x + int(gl_GlobalInvocationID.x);
    int y = uBounds.y + int(gl_GlobalInvocationID.y);
    if (x > uBounds.z || y > uBounds.w) {
        return;
    }

    if (uMode <= 3) {
        draw_triangle_pixel(x, y);
    }
    else {
        draw_rect_pixel(x, y);
    }
}
)GLSL";

} // namespace

GpuHardwareRasterizer::~GpuHardwareRasterizer() {
  shutdown();
}

bool GpuHardwareRasterizer::load_functions() {
  return
      load_proc(pCreateShader, "glCreateShader") &&
      load_proc(pShaderSource, "glShaderSource") &&
      load_proc(pCompileShader, "glCompileShader") &&
      load_proc(pGetShaderiv, "glGetShaderiv") &&
      load_proc(pGetShaderInfoLog, "glGetShaderInfoLog") &&
      load_proc(pDeleteShader, "glDeleteShader") &&
      load_proc(pCreateProgram, "glCreateProgram") &&
      load_proc(pAttachShader, "glAttachShader") &&
      load_proc(pLinkProgram, "glLinkProgram") &&
      load_proc(pGetProgramiv, "glGetProgramiv") &&
      load_proc(pGetProgramInfoLog, "glGetProgramInfoLog") &&
      load_proc(pDeleteProgram, "glDeleteProgram") &&
      load_proc(pUseProgram, "glUseProgram") &&
      load_proc(pGetUniformLocation, "glGetUniformLocation") &&
      load_proc(pUniform1i, "glUniform1i") &&
      load_proc(pUniform4i, "glUniform4i") &&
      load_proc(pGenBuffers, "glGenBuffers") &&
      load_proc(pBindBuffer, "glBindBuffer") &&
      load_proc(pBufferData, "glBufferData") &&
      load_proc(pBufferSubData, "glBufferSubData") &&
      load_proc(pGetBufferSubData, "glGetBufferSubData") &&
      load_proc(pDeleteBuffers, "glDeleteBuffers") &&
      load_proc(pBindBufferBase, "glBindBufferBase") &&
      load_proc(pDispatchCompute, "glDispatchCompute") &&
      load_proc(pMemoryBarrier, "glMemoryBarrier");
}

bool GpuHardwareRasterizer::create_program() {
  const GLuint shader = pCreateShader(GL_COMPUTE_SHADER);
  if (shader == 0) {
    status_ = "glCreateShader(GL_COMPUTE_SHADER) failed";
    return false;
  }

  const GLchar* source = kComputeShader;
  pShaderSource(shader, 1, &source, nullptr);
  pCompileShader(shader);

  GLint compiled = GL_FALSE;
  pGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
  if (compiled != GL_TRUE) {
    std::array<GLchar, 8192> log{};
    GLsizei len = 0;
    pGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), &len, log.data());
    status_ = "compute shader compile failed: ";
    status_.append(log.data(), static_cast<size_t>(std::max<GLsizei>(0, len)));
    pDeleteShader(shader);
    return false;
  }

  const GLuint program = pCreateProgram();
  if (program == 0) {
    status_ = "glCreateProgram failed";
    pDeleteShader(shader);
    return false;
  }

  pAttachShader(program, shader);
  pLinkProgram(program);
  pDeleteShader(shader);

  GLint linked = GL_FALSE;
  pGetProgramiv(program, GL_LINK_STATUS, &linked);
  if (linked != GL_TRUE) {
    std::array<GLchar, 8192> log{};
    GLsizei len = 0;
    pGetProgramInfoLog(program, static_cast<GLsizei>(log.size()), &len, log.data());
    status_ = "compute shader link failed: ";
    status_.append(log.data(), static_cast<size_t>(std::max<GLsizei>(0, len)));
    pDeleteProgram(program);
    return false;
  }

  program_ = program;
  loc_mode_ = pGetUniformLocation(program_, "uMode");
  loc_bounds_ = pGetUniformLocation(program_, "uBounds");
  loc_v0_ = pGetUniformLocation(program_, "uV0");
  loc_v1_ = pGetUniformLocation(program_, "uV1");
  loc_v2_ = pGetUniformLocation(program_, "uV2");
  loc_c0_ = pGetUniformLocation(program_, "uC0");
  loc_c1_ = pGetUniformLocation(program_, "uC1");
  loc_c2_ = pGetUniformLocation(program_, "uC2");
  loc_tex0_ = pGetUniformLocation(program_, "uTex0");
  loc_tex1_ = pGetUniformLocation(program_, "uTex1");
  loc_clut_row_ = pGetUniformLocation(program_, "uClutRow");
  loc_flags_ = pGetUniformLocation(program_, "uFlags");
  loc_semi_mode_ = pGetUniformLocation(program_, "uSemiMode");
  loc_rect_ = pGetUniformLocation(program_, "uRect");

  const int required[] = {
      loc_mode_, loc_bounds_, loc_v0_, loc_v1_, loc_v2_,
      loc_c0_, loc_c1_, loc_c2_, loc_tex0_, loc_tex1_,
      loc_clut_row_, loc_flags_, loc_semi_mode_, loc_rect_};
  for (int loc : required) {
    if (loc < 0) {
      status_ = "compute shader required uniform missing";
      pDeleteProgram(program_);
      program_ = 0;
      return false;
    }
  }

  return true;
}

bool GpuHardwareRasterizer::create_vram_buffer() {
  pGenBuffers(1, &vram_buffer_);
  if (vram_buffer_ == 0) {
    status_ = "failed to create VRAM SSBO";
    return false;
  }

  pBindBuffer(GL_SHADER_STORAGE_BUFFER, vram_buffer_);
  pBufferData(
      GL_SHADER_STORAGE_BUFFER,
      static_cast<GLsizeiptr>(kVramPixels * sizeof(u32)),
      nullptr,
      GL_DYNAMIC_DRAW);
  pBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, vram_buffer_);
  pBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

  staging_.resize(kVramPixels);
  return true;
}

bool GpuHardwareRasterizer::initialize() {
  shutdown();

  SDL_GL_ResetAttributes();
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(
      SDL_GL_CONTEXT_PROFILE_MASK,
      SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 0);

  window_ = SDL_CreateWindow(
      "VibeStation GPU Raster",
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      1,
      1,
      SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
  if (window_ == nullptr) {
    status_ = std::string("OpenGL 4.3 hidden window unavailable: ") +
              SDL_GetError();
    return false;
  }

  context_ = SDL_GL_CreateContext(window_);
  if (context_ == nullptr) {
    status_ = std::string("OpenGL 4.3 compute context unavailable: ") +
              SDL_GetError();
    SDL_DestroyWindow(window_);
    window_ = nullptr;
    return false;
  }

  if (SDL_GL_MakeCurrent(window_, context_) != 0) {
    status_ = std::string("failed to make compute context current: ") +
              SDL_GetError();
    SDL_GL_DeleteContext(context_);
    context_ = nullptr;
    SDL_DestroyWindow(window_);
    window_ = nullptr;
    return false;
  }

  thread_bound_ = true;

  if (!load_functions()) {
    status_ = "OpenGL 4.3 compute entry points unavailable";
    shutdown();
    return false;
  }
  if (!create_program() || !create_vram_buffer()) {
    shutdown();
    return false;
  }

  const GLubyte* version = glGetString(GL_VERSION);
  status_ = "OpenGL compute ready";
  if (version != nullptr) {
    status_ += " (";
    status_ += reinterpret_cast<const char*>(version);
    status_ += ")";
  }

  available_ = true;

  // Release the context from the initialization thread. EmuRunner claims it
  // once at worker startup.
  SDL_GL_MakeCurrent(window_, nullptr);
  thread_bound_ = false;
  return true;
}

void GpuHardwareRasterizer::destroy_gl_objects() {
  if (vram_buffer_ != 0 && pDeleteBuffers != nullptr) {
    pDeleteBuffers(1, &vram_buffer_);
    vram_buffer_ = 0;
  }
  if (program_ != 0 && pDeleteProgram != nullptr) {
    pDeleteProgram(program_);
    program_ = 0;
  }
}

void GpuHardwareRasterizer::shutdown() {
  available_ = false;

  if (context_ != nullptr && window_ != nullptr) {
    if (SDL_GL_MakeCurrent(window_, context_) == 0) {
      thread_bound_ = true;
      destroy_gl_objects();
      SDL_GL_MakeCurrent(window_, nullptr);
      thread_bound_ = false;
    }
  }

  if (context_ != nullptr) {
    SDL_GL_DeleteContext(context_);
    context_ = nullptr;
  }
  if (window_ != nullptr) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }

  staging_.clear();
  dispatch_count_ = 0;
  upload_count_ = 0;
  download_count_ = 0;
}

bool GpuHardwareRasterizer::bind_to_current_thread() {
  if (!available_ || window_ == nullptr || context_ == nullptr) {
    return false;
  }
  if (SDL_GL_MakeCurrent(window_, context_) != 0) {
    status_ = std::string("failed to bind compute context: ") +
              SDL_GetError();
    return false;
  }
  thread_bound_ = true;
  pBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, vram_buffer_);
  return true;
}

void GpuHardwareRasterizer::unbind_from_current_thread() {
  if (!thread_bound_ || window_ == nullptr) {
    return;
  }
  SDL_GL_MakeCurrent(window_, nullptr);
  thread_bound_ = false;
}

bool GpuHardwareRasterizer::upload_vram(
    const u16* vram,
    size_t pixel_count) {
  if (!available_ || !thread_bound_ || vram == nullptr ||
      pixel_count != kVramPixels) {
    return false;
  }

  if (staging_.size() != pixel_count) {
    staging_.resize(pixel_count);
  }
  for (size_t i = 0; i < pixel_count; ++i) {
    staging_[i] = static_cast<u32>(vram[i]);
  }

  pBindBuffer(GL_SHADER_STORAGE_BUFFER, vram_buffer_);
  pBufferSubData(
      GL_SHADER_STORAGE_BUFFER,
      0,
      static_cast<GLsizeiptr>(pixel_count * sizeof(u32)),
      staging_.data());
  pBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, vram_buffer_);
  pBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
  ++upload_count_;
  return true;
}

bool GpuHardwareRasterizer::download_vram(
    u16* vram,
    size_t pixel_count) {
  if (!available_ || !thread_bound_ || vram == nullptr ||
      pixel_count != kVramPixels) {
    return false;
  }

  pMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  glFinish();

  if (staging_.size() != pixel_count) {
    staging_.resize(pixel_count);
  }

  pBindBuffer(GL_SHADER_STORAGE_BUFFER, vram_buffer_);
  pGetBufferSubData(
      GL_SHADER_STORAGE_BUFFER,
      0,
      static_cast<GLsizeiptr>(pixel_count * sizeof(u32)),
      staging_.data());
  pBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

  for (size_t i = 0; i < pixel_count; ++i) {
    vram[i] = static_cast<u16>(staging_[i] & 0xFFFFu);
  }

  ++download_count_;
  return true;
}

bool GpuHardwareRasterizer::set_common_uniforms(
    int mode,
    int min_x, int min_y, int max_x, int max_y,
    const Vertex& v0,
    const Vertex& v1,
    const Vertex& v2,
    const DrawState& state,
    int rect_x, int rect_y, int rect_w, int rect_h) {
  if (!available_ || !thread_bound_ || program_ == 0) {
    return false;
  }

  int flags = 0;
  if (state.dither) {
    flags |= 1;
  }
  if (state.semi_transparent) {
    flags |= 2;
  }
  if (state.force_set_mask_bit) {
    flags |= 4;
  }
  if (state.check_mask_before_draw) {
    flags |= 8;
  }
  if (state.raw_texture) {
    flags |= 16;
  }
  if (state.rect_x_flip) {
    flags |= 32;
  }
  if (state.rect_y_flip) {
    flags |= 64;
  }

  pUseProgram(program_);
  pBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, vram_buffer_);

  pUniform1i(loc_mode_, mode);
  pUniform4i(loc_bounds_, min_x, min_y, max_x, max_y);
  pUniform4i(loc_v0_, v0.x, v0.y, v0.u, v0.v);
  pUniform4i(loc_v1_, v1.x, v1.y, v1.u, v1.v);
  pUniform4i(loc_v2_, v2.x, v2.y, v2.u, v2.v);
  pUniform4i(loc_c0_, v0.r, v0.g, v0.b, 0);
  pUniform4i(loc_c1_, v1.r, v1.g, v1.b, 0);
  pUniform4i(loc_c2_, v2.r, v2.g, v2.b, 0);
  pUniform4i(
      loc_tex0_,
      state.texture.keep_x,
      state.texture.keep_y,
      state.texture.replace_x,
      state.texture.replace_y);
  pUniform4i(
      loc_tex1_,
      state.texture.depth,
      state.texture.tex_base_x,
      state.texture.tex_base_y,
      state.texture.clut_x);
  pUniform1i(loc_clut_row_, static_cast<int>(state.texture.clut_row));
  pUniform1i(loc_flags_, flags);
  pUniform1i(loc_semi_mode_, state.semi_mode);
  pUniform4i(loc_rect_, rect_x, rect_y, rect_w, rect_h);
  return true;
}

bool GpuHardwareRasterizer::dispatch_bounds(
    int min_x, int min_y, int max_x, int max_y) {
  if (min_x > max_x || min_y > max_y) {
    return true;
  }

  const GLuint width =
      static_cast<GLuint>(max_x - min_x + 1);
  const GLuint height =
      static_cast<GLuint>(max_y - min_y + 1);

  pDispatchCompute(
      (width + 7u) / 8u,
      (height + 7u) / 8u,
      1u);
  // Every PS1 primitive can depend on destination or texture data written by
  // the previous primitive. Keep dispatches ordered while remaining
  // asynchronous with respect to the CPU.
  pMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  ++dispatch_count_;
  return true;
}

bool GpuHardwareRasterizer::draw_triangle(
    TriangleMode mode,
    Vertex v0,
    Vertex v1,
    Vertex v2,
    const DrawState& state) {
  if (!available_ || !thread_bound_) {
    return false;
  }

  auto edge_cpu = [](const Vertex& a, const Vertex& b, const Vertex& p) -> s32 {
    return static_cast<s32>(b.x - a.x) *
               static_cast<s32>(p.y - a.y) -
           static_cast<s32>(b.y - a.y) *
               static_cast<s32>(p.x - a.x);
  };

  s32 area = edge_cpu(v0, v1, v2);
  if (area == 0) {
    return true;
  }
  if (area < 0) {
    std::swap(v1, v2);
    area = -area;
  }

  int min_x = std::min({static_cast<int>(v0.x),
                        static_cast<int>(v1.x),
                        static_cast<int>(v2.x)});
  int max_x = std::max({static_cast<int>(v0.x),
                        static_cast<int>(v1.x),
                        static_cast<int>(v2.x)});
  int min_y = std::min({static_cast<int>(v0.y),
                        static_cast<int>(v1.y),
                        static_cast<int>(v2.y)});
  int max_y = std::max({static_cast<int>(v0.y),
                        static_cast<int>(v1.y),
                        static_cast<int>(v2.y)});

  min_x = std::max({min_x, static_cast<int>(state.draw_x_min), 0});
  min_y = std::max({min_y, static_cast<int>(state.draw_y_min), 0});
  max_x = std::min({max_x, static_cast<int>(state.draw_x_max),
                    static_cast<int>(psx::VRAM_WIDTH) - 1});
  max_y = std::min({max_y, static_cast<int>(state.draw_y_max),
                    static_cast<int>(psx::VRAM_HEIGHT) - 1});

  if (min_x > max_x || min_y > max_y) {
    return true;
  }
  if ((max_x - min_x) > 1023 || (max_y - min_y) > 511) {
    return true;
  }

  if (!set_common_uniforms(
          static_cast<int>(mode),
          min_x, min_y, max_x, max_y,
          v0, v1, v2, state,
          0, 0, 0, 0)) {
    return false;
  }

  return dispatch_bounds(min_x, min_y, max_x, max_y);
}

bool GpuHardwareRasterizer::draw_flat_rect(
    s16 x, s16 y, u16 width, u16 height,
    u8 r, u8 g, u8 b,
    const DrawState& state) {
  if (!available_ || !thread_bound_) {
    return false;
  }
  if (width == 0 || height == 0) {
    return true;
  }

  const int rect_x = static_cast<int>(x);
  const int rect_y = static_cast<int>(y);
  const int rect_w = static_cast<int>(width);
  const int rect_h = static_cast<int>(height);

  int min_x = std::max({rect_x, static_cast<int>(state.draw_x_min), 0});
  int min_y = std::max({rect_y, static_cast<int>(state.draw_y_min), 0});
  int max_x = std::min({
      rect_x + rect_w - 1,
      static_cast<int>(state.draw_x_max),
      static_cast<int>(psx::VRAM_WIDTH) - 1});
  int max_y = std::min({
      rect_y + rect_h - 1,
      static_cast<int>(state.draw_y_max),
      static_cast<int>(psx::VRAM_HEIGHT) - 1});

  if (min_x > max_x || min_y > max_y) {
    return true;
  }

  Vertex v0{};
  v0.r = r;
  v0.g = g;
  v0.b = b;
  Vertex empty{};

  if (!set_common_uniforms(
          4,
          min_x, min_y, max_x, max_y,
          v0, empty, empty, state,
          rect_x, rect_y, rect_w, rect_h)) {
    return false;
  }

  return dispatch_bounds(min_x, min_y, max_x, max_y);
}

bool GpuHardwareRasterizer::draw_textured_rect(
    s16 x, s16 y, u16 width, u16 height,
    u8 u, u8 v,
    u8 r, u8 g, u8 b,
    const DrawState& state) {
  if (!available_ || !thread_bound_) {
    return false;
  }
  if (width == 0 || height == 0) {
    return true;
  }

  const int rect_x = static_cast<int>(x);
  const int rect_y = static_cast<int>(y);
  const int rect_w = static_cast<int>(width);
  const int rect_h = static_cast<int>(height);

  int min_x = std::max({rect_x, static_cast<int>(state.draw_x_min), 0});
  int min_y = std::max({rect_y, static_cast<int>(state.draw_y_min), 0});
  int max_x = std::min({
      rect_x + rect_w - 1,
      static_cast<int>(state.draw_x_max),
      static_cast<int>(psx::VRAM_WIDTH) - 1});
  int max_y = std::min({
      rect_y + rect_h - 1,
      static_cast<int>(state.draw_y_max),
      static_cast<int>(psx::VRAM_HEIGHT) - 1});

  if (min_x > max_x || min_y > max_y) {
    return true;
  }

  Vertex v0{};
  v0.u = u;
  v0.v = v;
  v0.r = r;
  v0.g = g;
  v0.b = b;
  Vertex empty{};

  if (!set_common_uniforms(
          5,
          min_x, min_y, max_x, max_y,
          v0, empty, empty, state,
          rect_x, rect_y, rect_w, rect_h)) {
    return false;
  }

  return dispatch_bounds(min_x, min_y, max_x, max_y);
}
