#include "ui/ps2_gl_gs_backend.h"

#include "core/gs/gs_vram.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

#ifndef GL_COMPUTE_SHADER
#define GL_COMPUTE_SHADER 0x91B9
#endif
#ifndef GL_SHADER_STORAGE_BUFFER
#define GL_SHADER_STORAGE_BUFFER 0x90D2
#endif
#ifndef GL_DYNAMIC_COPY
#define GL_DYNAMIC_COPY 0x88EA
#endif
#ifndef GL_SHADER_STORAGE_BARRIER_BIT
#define GL_SHADER_STORAGE_BARRIER_BIT 0x00002000
#endif
#ifndef GL_BUFFER_UPDATE_BARRIER_BIT
#define GL_BUFFER_UPDATE_BARRIER_BIT 0x00000200
#endif

namespace ps2::ui {
namespace {

using GlCreateShaderFn = GLuint (APIENTRY*)(GLenum);
using GlShaderSourceFn =
    void (APIENTRY*)(GLuint, GLsizei, const GLchar* const*, const GLint*);
using GlCompileShaderFn = void (APIENTRY*)(GLuint);
using GlGetShaderivFn = void (APIENTRY*)(GLuint, GLenum, GLint*);
using GlGetShaderInfoLogFn =
    void (APIENTRY*)(GLuint, GLsizei, GLsizei*, GLchar*);
using GlDeleteShaderFn = void (APIENTRY*)(GLuint);
using GlCreateProgramFn = GLuint (APIENTRY*)();
using GlAttachShaderFn = void (APIENTRY*)(GLuint, GLuint);
using GlLinkProgramFn = void (APIENTRY*)(GLuint);
using GlGetProgramivFn = void (APIENTRY*)(GLuint, GLenum, GLint*);
using GlGetProgramInfoLogFn =
    void (APIENTRY*)(GLuint, GLsizei, GLsizei*, GLchar*);
using GlDeleteProgramFn = void (APIENTRY*)(GLuint);
using GlUseProgramFn = void (APIENTRY*)(GLuint);
using GlGenBuffersFn = void (APIENTRY*)(GLsizei, GLuint*);
using GlDeleteBuffersFn = void (APIENTRY*)(GLsizei, const GLuint*);
using GlBindBufferFn = void (APIENTRY*)(GLenum, GLuint);
using GlBindBufferBaseFn = void (APIENTRY*)(GLenum, GLuint, GLuint);
using GlBufferDataFn =
    void (APIENTRY*)(GLenum, std::ptrdiff_t, const void*, GLenum);
using GlBufferSubDataFn =
    void (APIENTRY*)(GLenum, std::ptrdiff_t, std::ptrdiff_t, const void*);
using GlGetBufferSubDataFn =
    void (APIENTRY*)(GLenum, std::ptrdiff_t, std::ptrdiff_t, void*);
using GlDispatchComputeFn = void (APIENTRY*)(GLuint, GLuint, GLuint);
using GlMemoryBarrierFn = void (APIENTRY*)(GLbitfield);
using GlGetUniformLocationFn = GLint (APIENTRY*)(GLuint, const GLchar*);
using GlUniform1iFn = void (APIENTRY*)(GLint, GLint);
using GlUniform1uiFn = void (APIENTRY*)(GLint, GLuint);

struct GlApi {
    GlCreateShaderFn create_shader = nullptr;
    GlShaderSourceFn shader_source = nullptr;
    GlCompileShaderFn compile_shader = nullptr;
    GlGetShaderivFn get_shader_iv = nullptr;
    GlGetShaderInfoLogFn get_shader_log = nullptr;
    GlDeleteShaderFn delete_shader = nullptr;
    GlCreateProgramFn create_program = nullptr;
    GlAttachShaderFn attach_shader = nullptr;
    GlLinkProgramFn link_program = nullptr;
    GlGetProgramivFn get_program_iv = nullptr;
    GlGetProgramInfoLogFn get_program_log = nullptr;
    GlDeleteProgramFn delete_program = nullptr;
    GlUseProgramFn use_program = nullptr;
    GlGenBuffersFn gen_buffers = nullptr;
    GlDeleteBuffersFn delete_buffers = nullptr;
    GlBindBufferFn bind_buffer = nullptr;
    GlBindBufferBaseFn bind_buffer_base = nullptr;
    GlBufferDataFn buffer_data = nullptr;
    GlBufferSubDataFn buffer_sub_data = nullptr;
    GlGetBufferSubDataFn get_buffer_sub_data = nullptr;
    GlDispatchComputeFn dispatch_compute = nullptr;
    GlMemoryBarrierFn memory_barrier = nullptr;
    GlGetUniformLocationFn get_uniform_location = nullptr;
    GlUniform1iFn uniform1i = nullptr;
    GlUniform1uiFn uniform1ui = nullptr;
};

GlApi g_gl;

template <typename T>
bool load_gl(T& target, const char* name) {
    target = reinterpret_cast<T>(
        SDL_GL_GetProcAddress(name));
    if (target != nullptr) return true;
    std::fprintf(stderr, "GPU GS: missing OpenGL symbol %s\n", name);
    return false;
}

s32 ceil_div16(s32 value) {
    if (value >= 0) return (value + 15) >> 4;
    return -static_cast<s32>(
        static_cast<u32>(-value) >> 4);
}

constexpr const char* kComputeShader = R"GLSL(
#version 430 core
layout(local_size_x=16, local_size_y=16) in;
layout(std430, binding=0) buffer VramBuffer {
    uint words[];
};

uniform int u_left;
uniform int u_right;
uniform int u_top;
uniform int u_bottom;
uniform int u_ax;
uniform int u_ay;
uniform int u_au;
uniform int u_av;
uniform int u_bu;
uniform int u_bv;
uniform int u_dx;
uniform int u_dy;
uniform uint u_vertex_rgba;
uniform uint u_z;
uniform uint u_fbp;
uniform uint u_fbw;
uniform uint u_zbp;
uniform uint u_tbp;
uniform uint u_tbw;
uniform uint u_tw;
uniform uint u_th;
uniform uint u_wms;
uniform uint u_wmt;
uniform uint u_minu;
uniform uint u_maxu;
uniform uint u_minv;
uniform uint u_maxv;
uniform uint u_ta0;
uniform uint u_ta1;
uniform uint u_aem;
uniform uint u_tcc;

const uint block32[32] = uint[32](
    0,1,4,5,16,17,20,21,
    2,3,6,7,18,19,22,23,
    8,9,12,13,24,25,28,29,
    10,11,14,15,26,27,30,31
);
const uint column32[64] = uint[64](
    0,1,4,5,8,9,12,13,
    2,3,6,7,10,11,14,15,
    16,17,20,21,24,25,28,29,
    18,19,22,23,26,27,30,31,
    32,33,36,37,40,41,44,45,
    34,35,38,39,42,43,46,47,
    48,49,52,53,56,57,60,61,
    50,51,54,55,58,59,62,63
);
const uint block16[32] = uint[32](
    0,2,8,10, 1,3,9,11,
    4,6,12,14, 5,7,13,15,
    16,18,24,26, 17,19,25,27,
    20,22,28,30, 21,23,29,31
);
const uint column16[128] = uint[128](
    0,2,8,10,16,18,24,26,1,3,9,11,17,19,25,27,
    4,6,12,14,20,22,28,30,5,7,13,15,21,23,29,31,
    32,34,40,42,48,50,56,58,33,35,41,43,49,51,57,59,
    36,38,44,46,52,54,60,62,37,39,45,47,53,55,61,63,
    64,66,72,74,80,82,88,90,65,67,73,75,81,83,89,91,
    68,70,76,78,84,86,92,94,69,71,77,79,85,87,93,95,
    96,98,104,106,112,114,120,122,97,99,105,107,113,115,121,123,
    100,102,108,110,116,118,124,126,101,103,109,111,117,119,125,127
);

uint address32(uint x, uint y, uint bp, uint bw) {
    uint page_x = x >> 6u;
    uint page_y = y >> 5u;
    uint block = block32[((y & 31u) >> 3u) * 8u + ((x & 63u) >> 3u)];
    uint column = column32[(y & 7u) * 8u + (x & 7u)];
    uint word = (bp << 6u) +
        ((page_y * bw + page_x) << 11u) +
        block * 64u + column;
    return (word & 0xFFFFFu) << 2u;
}

uint address16(uint x, uint y, uint bp, uint bw) {
    uint page_x = x >> 6u;
    uint page_y = y >> 6u;
    uint block = block16[((y & 63u) >> 3u) * 4u + ((x & 63u) >> 4u)];
    uint column = column16[(y & 7u) * 16u + (x & 15u)];
    uint halfword = (bp << 7u) +
        ((page_y * bw + page_x) << 12u) +
        block * 128u + column;
    return (halfword & 0x1FFFFFu) << 1u;
}

uint read16(uint address) {
    uint packed_word = words[address >> 2u];
    uint bit_shift = (address & 2u) * 8u;
    return (packed_word >> bit_shift) & 0xFFFFu;
}

int wrap_coord(int c, uint size, uint mode, uint lo_value, uint hi_value) {
    if (size == 0u) return 0;
    int maximum = int(size - 1u);
    uint m = mode & 3u;
    if (m == 0u) return c & maximum;
    if (m == 1u) return clamp(c, 0, maximum);
    if (m == 2u) {
        int lo = int(min(lo_value, size - 1u));
        int hi = int(min(hi_value, size - 1u));
        return clamp(c, min(lo, hi), max(lo, hi));
    }
    return (c & int(lo_value & (size - 1u))) | int(hi_value);
}

uint channel(uint c, uint shift) {
    return (c >> shift) & 255u;
}

uint modulate(uint a, uint b) {
    return min(255u, (a * b) >> 7u);
}

void main() {
    int x = u_left + int(gl_GlobalInvocationID.x);
    int y = u_top + int(gl_GlobalInvocationID.y);
    if (x >= u_right || y >= u_bottom) return;

    int px = x * 16 + 8;
    int py = y * 16 + 8;
    int uf = u_au;
    int vf = u_av;
    if (u_dx != 0) {
        uf += ((u_bu - u_au) * (px - u_ax)) / u_dx;
    }
    if (u_dy != 0) {
        vf += ((u_bv - u_av) * (py - u_ay)) / u_dy;
    }

    uint tx = uint(wrap_coord(
        uf >> 4, u_tw, u_wms, u_minu, u_maxu));
    uint ty = uint(wrap_coord(
        vf >> 4, u_th, u_wmt, u_minv, u_maxv));
    uint tex = read16(address16(tx, ty, u_tbp, u_tbw));

    uint vr = channel(u_vertex_rgba, 0u);
    uint vg = channel(u_vertex_rgba, 8u);
    uint vb = channel(u_vertex_rgba, 16u);
    uint va = channel(u_vertex_rgba, 24u);

    uint sr = min(255u, ((tex & 31u) * vr) >> 4u);
    uint sg = min(255u, (((tex >> 5u) & 31u) * vg) >> 4u);
    uint sb = min(255u, (((tex >> 10u) & 31u) * vb) >> 4u);
    uint alpha = (tex & 0x8000u) != 0u
        ? (u_ta1 & 255u)
        : ((u_aem != 0u && (tex & 0x7FFFu) == 0u)
            ? 0u : (u_ta0 & 255u));
    alpha = u_tcc != 0u ? modulate(alpha, va) : va;
    uint source = sr | (sg << 8u) | (sb << 16u) | (alpha << 24u);

    uint frame_address =
        address32(uint(x), uint(y), u_fbp, u_fbw);
    uint frame_word = frame_address >> 2u;
    uint depth_word =
        (address32(uint(x), uint(y), u_zbp, u_fbw) >> 2u) ^ 0x600u;
    uint destination_z = words[depth_word & 0xFFFFFu];
    if (u_z < destination_z) return;

    uint destination = words[frame_word];
    uint dr = destination & 255u;
    uint dg = (destination >> 8u) & 255u;
    uint db = (destination >> 16u) & 255u;

    uint result_color = source & 0xFF000000u;
    result_color |= min(255u, dr + ((sr * 20u) >> 7u));
    result_color |= min(255u, dg + ((sg * 20u) >> 7u)) << 8u;
    result_color |= min(255u, db + ((sb * 20u) >> 7u)) << 16u;
    words[frame_word] = result_color;
}
)GLSL";

template <typename T>
T proc(const char* name) {
    return reinterpret_cast<T>(
        SDL_GL_GetProcAddress(name));
}

} // namespace

Ps2GlGsBackend::Ps2GlGsBackend(
    SDL_Window* window,
    SDL_GLContext context)
    : window_(window), context_(context) {
    worker_ = std::thread(
        &Ps2GlGsBackend::worker_main, this);
    std::unique_lock lock(mutex_);
    ready_condition_.wait(lock, [this] {
        return init_finished_;
    });
}

Ps2GlGsBackend::~Ps2GlGsBackend() {
    {
        std::lock_guard lock(mutex_);
        stop_ = true;
        jobs_.push_back(Job{JobType::Stop});
    }
    condition_.notify_one();
    if (worker_.joinable()) worker_.join();
    if (context_ != nullptr) {
        SDL_GL_DeleteContext(context_);
        context_ = nullptr;
    }
}

void Ps2GlGsBackend::invalidate_cpu_source() {
    std::lock_guard lock(mutex_);
    queued_cpu_generation_ = ~u64{0};
}

bool Ps2GlGsBackend::sprite_supported(
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    s32 top,
    s32 bottom,
    SpriteJob& job) const {
    if (!ctx.texture.enabled ||
        !ctx.texture.fst ||
        ctx.texture.psm != 2u ||
        ctx.texture.tfx != 0u ||
        ctx.psm != 0u ||
        !ctx.zte ||
        (ctx.ztst & 3u) != 2u ||
        ctx.zpsm != 48u ||
        !ctx.zmask ||
        !ctx.alpha_blend ||
        ctx.pabe ||
        (ctx.alpha_a & 3u) != 0u ||
        (ctx.alpha_b & 3u) != 2u ||
        (ctx.alpha_c & 3u) != 2u ||
        (ctx.alpha_d & 3u) != 1u ||
        (ctx.alpha_fix & 0xFFu) != 20u ||
        !ctx.color_clamp ||
        ctx.ate || ctx.date || ctx.fba ||
        ctx.fbmask != 0u ||
        (ctx.scanmask & 2u) != 0u ||
        ctx.fog_enabled ||
        ctx.nonzero_colors != nullptr ||
        ctx.nonzero_inputs != nullptr ||
        ctx.nonzero_input_alpha != nullptr ||
        ctx.texture.nonzero_samples != nullptr ||
        ctx.texture.alpha_samples != nullptr ||
        ctx.texture.nonzero_shaded != nullptr) {
        return false;
    }

    s32 left = ceil_div16(std::min(a.x, b.x));
    s32 right = ceil_div16(std::max(a.x, b.x));
    left = std::max(left, ctx.scax0);
    right = std::min(right, ctx.scax1 + 1);
    top = std::max(top, ctx.scay0);
    bottom = std::min(bottom, ctx.scay1 + 1);
    if (left >= right || top >= bottom) return false;

    const u32 page_x0 =
        static_cast<u32>(left) >> 6u;
    const u32 page_x1 =
        static_cast<u32>(right - 1) >> 6u;
    const u32 page_y0 =
        static_cast<u32>(top) >> 5u;
    const u32 page_y1 =
        static_cast<u32>(bottom - 1) >> 5u;
    if (ctx.fbw == 0u || page_x1 >= ctx.fbw) {
        return false;
    }

    const u64 base =
        static_cast<u64>(ctx.fbp) * 256u;
    const u64 first_page =
        static_cast<u64>(page_y0) * ctx.fbw + page_x0;
    const u64 last_page =
        static_cast<u64>(page_y1) * ctx.fbw + page_x1;
    const u64 dirty_begin =
        base + first_page * 8192u;
    const u64 dirty_end =
        base + (last_page + 1u) * 8192u;
    if (dirty_end > GsVram::kSize) return false;

    job.ctx = ctx;
    job.a = a;
    job.b = b;
    job.left = left;
    job.right = right;
    job.top = top;
    job.bottom = bottom;
    job.dirty_begin =
        static_cast<u32>(dirty_begin);
    job.dirty_end =
        static_cast<u32>(dirty_end);
    return true;
}

bool Ps2GlGsBackend::submit_sprite(
    const GsVram& vram,
    const GsRasterContext& ctx,
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    s32 top,
    s32 bottom,
    u64 area) {
    (void)area;
    if (!available()) return false;

    SpriteJob sprite{};
    if (!sprite_supported(
            ctx, a, b, top, bottom, sprite)) {
        return false;
    }

    std::lock_guard lock(mutex_);
    const u64 generation = vram.generation();
    if (queued_cpu_generation_ != generation) {
        Job upload{};
        upload.type = JobType::Upload;
        upload.cpu_generation = generation;
        upload.upload.assign(
            vram.raw_data(),
            vram.raw_data() + GsVram::kSize);
        jobs_.push_back(std::move(upload));
        queued_cpu_generation_ = generation;
    }

    Job draw{};
    draw.type = JobType::Sprite;
    draw.sprite = sprite;
    jobs_.push_back(std::move(draw));
    ++outstanding_draws_;
    condition_.notify_one();
    return true;
}

bool Ps2GlGsBackend::synchronize_to_cpu(
    GsVram& vram) {
    if (!available()) return false;

    const u64 generation_before = vram.generation();
    u64 serial = 0;
    {
        std::unique_lock lock(mutex_);
        if (!gpu_dirty_ &&
            outstanding_draws_ == 0u &&
            jobs_.empty()) {
            return false;
        }
        serial = next_sync_serial_++;
        Job sync{};
        sync.type = JobType::Sync;
        sync.sync_vram = &vram;
        sync.sync_serial = serial;
        jobs_.push_back(std::move(sync));
        condition_.notify_one();
        sync_condition_.wait(lock, [&] {
            return completed_sync_serial_ >= serial ||
                   !available_.load(std::memory_order_acquire);
        });
        if (!available()) return false;
        synced_cpu_generation_ = vram.generation();
        queued_cpu_generation_ = synced_cpu_generation_;
    }
    return vram.generation() != generation_before;
}

bool Ps2GlGsBackend::initialize_gl() {
    bool ok = true;
    ok &= load_gl(g_gl.create_shader, "glCreateShader");
    ok &= load_gl(g_gl.shader_source, "glShaderSource");
    ok &= load_gl(g_gl.compile_shader, "glCompileShader");
    ok &= load_gl(g_gl.get_shader_iv, "glGetShaderiv");
    ok &= load_gl(g_gl.get_shader_log, "glGetShaderInfoLog");
    ok &= load_gl(g_gl.delete_shader, "glDeleteShader");
    ok &= load_gl(g_gl.create_program, "glCreateProgram");
    ok &= load_gl(g_gl.attach_shader, "glAttachShader");
    ok &= load_gl(g_gl.link_program, "glLinkProgram");
    ok &= load_gl(g_gl.get_program_iv, "glGetProgramiv");
    ok &= load_gl(g_gl.get_program_log, "glGetProgramInfoLog");
    ok &= load_gl(g_gl.delete_program, "glDeleteProgram");
    ok &= load_gl(g_gl.use_program, "glUseProgram");
    ok &= load_gl(g_gl.gen_buffers, "glGenBuffers");
    ok &= load_gl(g_gl.delete_buffers, "glDeleteBuffers");
    ok &= load_gl(g_gl.bind_buffer, "glBindBuffer");
    ok &= load_gl(g_gl.bind_buffer_base, "glBindBufferBase");
    ok &= load_gl(g_gl.buffer_data, "glBufferData");
    ok &= load_gl(g_gl.buffer_sub_data, "glBufferSubData");
    ok &= load_gl(g_gl.get_buffer_sub_data, "glGetBufferSubData");
    ok &= load_gl(g_gl.dispatch_compute, "glDispatchCompute");
    ok &= load_gl(g_gl.memory_barrier, "glMemoryBarrier");
    ok &= load_gl(g_gl.get_uniform_location, "glGetUniformLocation");
    ok &= load_gl(g_gl.uniform1i, "glUniform1i");
    ok &= load_gl(g_gl.uniform1ui, "glUniform1ui");
    if (!ok) return false;

    const GLuint shader =
        g_gl.create_shader(GL_COMPUTE_SHADER);
    const GLchar* source = kComputeShader;
    g_gl.shader_source(shader, 1, &source, nullptr);
    g_gl.compile_shader(shader);

    GLint compiled = 0;
    g_gl.get_shader_iv(
        shader, GL_COMPILE_STATUS, &compiled);
    if (compiled == 0) {
        std::array<GLchar, 4096> log{};
        GLsizei length = 0;
        g_gl.get_shader_log(
            shader,
            static_cast<GLsizei>(log.size()),
            &length,
            log.data());
        std::fprintf(
            stderr,
            "GPU GS compute shader compile failed: %s\n",
            log.data());
        g_gl.delete_shader(shader);
        return false;
    }

    program_ = g_gl.create_program();
    g_gl.attach_shader(program_, shader);
    g_gl.link_program(program_);
    g_gl.delete_shader(shader);

    GLint linked = 0;
    g_gl.get_program_iv(
        program_, GL_LINK_STATUS, &linked);
    if (linked == 0) {
        std::array<GLchar, 4096> log{};
        GLsizei length = 0;
        g_gl.get_program_log(
            program_,
            static_cast<GLsizei>(log.size()),
            &length,
            log.data());
        std::fprintf(
            stderr,
            "GPU GS compute program link failed: %s\n",
            log.data());
        return false;
    }

    g_gl.gen_buffers(1, &vram_buffer_);
    g_gl.bind_buffer(
        GL_SHADER_STORAGE_BUFFER, vram_buffer_);
    g_gl.buffer_data(
        GL_SHADER_STORAGE_BUFFER,
        static_cast<std::ptrdiff_t>(GsVram::kSize),
        nullptr,
        GL_DYNAMIC_COPY);
    g_gl.bind_buffer_base(
        GL_SHADER_STORAGE_BUFFER, 0, vram_buffer_);
    return true;
}

void Ps2GlGsBackend::destroy_gl() {
    if (vram_buffer_ != 0u &&
        g_gl.delete_buffers != nullptr) {
        g_gl.delete_buffers(1, &vram_buffer_);
        vram_buffer_ = 0u;
    }
    if (program_ != 0u &&
        g_gl.delete_program != nullptr) {
        g_gl.delete_program(program_);
        program_ = 0u;
    }
}

bool Ps2GlGsBackend::execute_sprite(
    const SpriteJob& job) {
    g_gl.use_program(program_);
    g_gl.bind_buffer_base(
        GL_SHADER_STORAGE_BUFFER, 0, vram_buffer_);

    auto ui = [&](const char* name, int value) {
        const GLint location =
            g_gl.get_uniform_location(program_, name);
        if (location >= 0) g_gl.uniform1i(location, value);
    };
    auto uu = [&](const char* name, u32 value) {
        const GLint location =
            g_gl.get_uniform_location(program_, name);
        if (location >= 0) g_gl.uniform1ui(location, value);
    };

    const auto& c = job.ctx;
    ui("u_left", job.left);
    ui("u_right", job.right);
    ui("u_top", job.top);
    ui("u_bottom", job.bottom);
    ui("u_ax", job.a.x);
    ui("u_ay", job.a.y);
    ui("u_au", job.a.u);
    ui("u_av", job.a.v);
    ui("u_bu", job.b.u);
    ui("u_bv", job.b.v);
    ui("u_dx", job.b.x - job.a.x);
    ui("u_dy", job.b.y - job.a.y);
    uu("u_vertex_rgba", job.b.rgba);
    uu("u_z", job.b.z);
    uu("u_fbp", c.fbp);
    uu("u_fbw", c.fbw);
    uu("u_zbp", c.zbp);
    uu("u_tbp", c.texture.bp);
    uu("u_tbw", c.texture.bw);
    uu("u_tw", c.texture.width);
    uu("u_th", c.texture.height);
    uu("u_wms", c.texture.wms);
    uu("u_wmt", c.texture.wmt);
    uu("u_minu", c.texture.minu);
    uu("u_maxu", c.texture.maxu);
    uu("u_minv", c.texture.minv);
    uu("u_maxv", c.texture.maxv);
    uu("u_ta0", c.texture.ta0);
    uu("u_ta1", c.texture.ta1);
    uu("u_aem", c.texture.aem ? 1u : 0u);
    uu("u_tcc", c.texture.tcc ? 1u : 0u);

    const u32 width =
        static_cast<u32>(job.right - job.left);
    const u32 height =
        static_cast<u32>(job.bottom - job.top);
    g_gl.dispatch_compute(
        (width + 15u) / 16u,
        (height + 15u) / 16u,
        1u);
    // Later sprite dispatches may read framebuffer results from this one.
    g_gl.memory_barrier(
        GL_SHADER_STORAGE_BARRIER_BIT);
    return true;
}

void Ps2GlGsBackend::worker_main() {
    if (SDL_GL_MakeCurrent(
            window_, context_) != 0) {
        std::fprintf(
            stderr,
            "GPU GS shared context activation failed: %s\n",
            SDL_GetError());
        std::lock_guard lock(mutex_);
        init_finished_ = true;
        ready_condition_.notify_all();
        return;
    }

    const bool initialized = initialize_gl();
    {
        std::lock_guard lock(mutex_);
        available_.store(
            initialized,
            std::memory_order_release);
        init_finished_ = true;
    }
    ready_condition_.notify_all();
    if (!initialized) {
        SDL_GL_MakeCurrent(window_, nullptr);
        return;
    }

    for (;;) {
        Job job{};
        {
            std::unique_lock lock(mutex_);
            condition_.wait(lock, [this] {
                return stop_ || !jobs_.empty();
            });
            if (jobs_.empty() && stop_) break;
            job = std::move(jobs_.front());
            jobs_.pop_front();
        }

        if (job.type == JobType::Stop) break;

        if (job.type == JobType::Upload) {
            g_gl.bind_buffer(
                GL_SHADER_STORAGE_BUFFER,
                vram_buffer_);
            g_gl.buffer_sub_data(
                GL_SHADER_STORAGE_BUFFER,
                0,
                static_cast<std::ptrdiff_t>(
                    job.upload.size()),
                job.upload.data());
            {
                std::lock_guard lock(mutex_);
                gpu_dirty_ = false;
                dirty_begin_ = GsVram::kSize;
                dirty_end_ = 0u;
            }
            continue;
        }

        if (job.type == JobType::Sprite) {
            const bool executed =
                execute_sprite(job.sprite);
            {
                std::lock_guard lock(mutex_);
                if (outstanding_draws_ != 0u) {
                    --outstanding_draws_;
                }
                if (executed) {
                    gpu_dirty_ = true;
                    dirty_begin_ = std::min(
                        dirty_begin_,
                        job.sprite.dirty_begin);
                    dirty_end_ = std::max(
                        dirty_end_,
                        job.sprite.dirty_end);
                }
            }
            if (!executed) {
                available_.store(
                    false,
                    std::memory_order_release);
                sync_condition_.notify_all();
                break;
            }
            continue;
        }

        if (job.type == JobType::Sync) {
            bool changed = false;
            u32 begin = 0u;
            u32 end = 0u;
            {
                std::lock_guard lock(mutex_);
                changed = gpu_dirty_;
                begin = dirty_begin_;
                end = dirty_end_;
            }

            if (changed &&
                job.sync_vram != nullptr &&
                begin < end) {
                g_gl.memory_barrier(
                    GL_SHADER_STORAGE_BARRIER_BIT |
                    GL_BUFFER_UPDATE_BARRIER_BIT);
                g_gl.bind_buffer(
                    GL_SHADER_STORAGE_BUFFER,
                    vram_buffer_);
                g_gl.get_buffer_sub_data(
                    GL_SHADER_STORAGE_BUFFER,
                    static_cast<std::ptrdiff_t>(begin),
                    static_cast<std::ptrdiff_t>(end - begin),
                    job.sync_vram->raw_data() + begin);
                job.sync_vram->mark_modified();
            }

            {
                std::lock_guard lock(mutex_);
                gpu_dirty_ = false;
                dirty_begin_ = GsVram::kSize;
                dirty_end_ = 0u;
                if (job.sync_vram != nullptr) {
                    queued_cpu_generation_ =
                        job.sync_vram->generation();
                }
                completed_sync_serial_ =
                    std::max(
                        completed_sync_serial_,
                        job.sync_serial);
            }
            sync_condition_.notify_all();
        }
    }

    destroy_gl();
    available_.store(false, std::memory_order_release);
    SDL_GL_MakeCurrent(window_, nullptr);
    {
        std::lock_guard lock(mutex_);
        completed_sync_serial_ =
            std::numeric_limits<u64>::max();
    }
    sync_condition_.notify_all();
}

} // namespace ps2::ui
