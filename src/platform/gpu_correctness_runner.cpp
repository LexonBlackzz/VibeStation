#include "platform/gpu_correctness_runner.h"

#include "core/gpu.h"
#include "core/types.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

struct RefVertex {
  s16 x = 0;
  s16 y = 0;
  u8 r = 0;
  u8 g = 0;
  u8 b = 0;
  u8 u = 0;
  u8 v = 0;
};

constexpr size_t kVramPixels =
    static_cast<size_t>(psx::VRAM_WIDTH) * psx::VRAM_HEIGHT;

s32 edge(const RefVertex &a, const RefVertex &b, s16 x, s16 y) {
  return static_cast<s32>(b.x - a.x) * (y - a.y) -
         static_cast<s32>(b.y - a.y) * (x - a.x);
}

bool is_top_left(const RefVertex &a, const RefVertex &b) {
  const int dy = static_cast<int>(b.y) - static_cast<int>(a.y);
  const int dx = static_cast<int>(b.x) - static_cast<int>(a.x);
  return (dy < 0) || (dy == 0 && dx > 0);
}

bool inside_edge(s32 value, bool top_left) {
  return value > 0 || (value == 0 && top_left);
}

int clamp_u8(int value) {
  return std::max(0, std::min(value, 255));
}

u16 pack_rgb15(int r, int g, int b) {
  const u16 rr = static_cast<u16>(clamp_u8(r) >> 3);
  const u16 gg = static_cast<u16>(clamp_u8(g) >> 3);
  const u16 bb = static_cast<u16>(clamp_u8(b) >> 3);
  return static_cast<u16>(rr | (gg << 5) | (bb << 10));
}

u16 modulate_texel(u16 texel, int mr, int mg, int mb) {
  const int tr = texel & 0x1F;
  const int tg = (texel >> 5) & 0x1F;
  const int tb = (texel >> 10) & 0x1F;
  const int rr = std::min(31, (tr * mr) >> 7);
  const int rg = std::min(31, (tg * mg) >> 7);
  const int rb = std::min(31, (tb * mb) >> 7);
  return static_cast<u16>((rr & 0x1F) | ((rg & 0x1F) << 5) |
                          ((rb & 0x1F) << 10) | (texel & 0x8000u));
}

u32 rgb_command(u8 opcode, u8 r, u8 g, u8 b) {
  return (static_cast<u32>(opcode) << 24) |
         (static_cast<u32>(b) << 16) |
         (static_cast<u32>(g) << 8) | static_cast<u32>(r);
}

u32 rgb_word(u8 r, u8 g, u8 b) {
  return (static_cast<u32>(b) << 16) |
         (static_cast<u32>(g) << 8) | static_cast<u32>(r);
}

u32 vertex_word(s16 x, s16 y) {
  return static_cast<u32>(static_cast<u16>(x)) |
         (static_cast<u32>(static_cast<u16>(y)) << 16);
}

u32 uv_word(u8 u, u8 v, u16 high) {
  return static_cast<u32>(u) | (static_cast<u32>(v) << 8) |
         (static_cast<u32>(high) << 16);
}

template <typename PixelFn>
void reference_triangle(std::vector<u16> &vram, RefVertex v0, RefVertex v1,
                        RefVertex v2, PixelFn &&pixel_fn) {
  s32 area = edge(v0, v1, v2.x, v2.y);
  if (area == 0) {
    return;
  }
  if (area < 0) {
    std::swap(v1, v2);
    area = -area;
  }

  const bool edge0_top_left = is_top_left(v1, v2);
  const bool edge1_top_left = is_top_left(v2, v0);
  const bool edge2_top_left = is_top_left(v0, v1);

  const int min_x = std::max(
      0, std::min({static_cast<int>(v0.x), static_cast<int>(v1.x),
                   static_cast<int>(v2.x)}));
  const int max_x = std::min(
      static_cast<int>(psx::VRAM_WIDTH) - 1,
      std::max({static_cast<int>(v0.x), static_cast<int>(v1.x),
                static_cast<int>(v2.x)}));
  const int min_y = std::max(
      0, std::min({static_cast<int>(v0.y), static_cast<int>(v1.y),
                   static_cast<int>(v2.y)}));
  const int max_y = std::min(
      static_cast<int>(psx::VRAM_HEIGHT) - 1,
      std::max({static_cast<int>(v0.y), static_cast<int>(v1.y),
                static_cast<int>(v2.y)}));

  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      const s32 w0 = edge(v1, v2, static_cast<s16>(x), static_cast<s16>(y));
      const s32 w1 = edge(v2, v0, static_cast<s16>(x), static_cast<s16>(y));
      const s32 w2 = edge(v0, v1, static_cast<s16>(x), static_cast<s16>(y));
      if (!inside_edge(w0, edge0_top_left) ||
          !inside_edge(w1, edge1_top_left) ||
          !inside_edge(w2, edge2_top_left)) {
        continue;
      }

      const size_t index =
          static_cast<size_t>(y) * psx::VRAM_WIDTH + static_cast<size_t>(x);
      vram[index] = pixel_fn(vram, v0, v1, v2, w0, w1, w2, area, x, y);
    }
  }
}

bool compare_vram(const char *name, const Gpu &gpu,
                  const std::vector<u16> &expected) {
  const u16 *actual = gpu.vram();
  size_t mismatch_count = 0;
  for (size_t i = 0; i < expected.size(); ++i) {
    if (actual[i] == expected[i]) {
      continue;
    }

    if (mismatch_count < 8) {
      const size_t x = i % psx::VRAM_WIDTH;
      const size_t y = i / psx::VRAM_WIDTH;
      std::fprintf(stderr,
                   "[GPU TEST] %s mismatch at (%zu,%zu): expected=%04X actual=%04X\n",
                   name, x, y, static_cast<unsigned>(expected[i]),
                   static_cast<unsigned>(actual[i]));
    }
    ++mismatch_count;
  }

  if (mismatch_count != 0) {
    std::fprintf(stderr, "[GPU TEST] FAIL %-24s (%zu mismatched pixels)\n",
                 name, mismatch_count);
    return false;
  }

  std::fprintf(stdout, "[GPU TEST] PASS %-24s\n", name);
  return true;
}

bool test_flat_triangle() {
  auto gpu = std::make_unique<Gpu>();
  gpu->init(nullptr);
  gpu->reset();

  std::vector<u16> expected(kVramPixels, 0);

  const RefVertex v0{10, 10, 248, 120, 40, 0, 0};
  const RefVertex v1{43, 15, 248, 120, 40, 0, 0};
  const RefVertex v2{17, 39, 248, 120, 40, 0, 0};
  const u16 color = pack_rgb15(v0.r, v0.g, v0.b);

  reference_triangle(
      expected, v0, v1, v2,
      [color](const std::vector<u16> &, const RefVertex &, const RefVertex &,
              const RefVertex &, s32, s32, s32, s32, int, int) {
        return color;
      });

  gpu->gp0(rgb_command(0x20, v0.r, v0.g, v0.b));
  gpu->gp0(vertex_word(v0.x, v0.y));
  gpu->gp0(vertex_word(v1.x, v1.y));
  gpu->gp0(vertex_word(v2.x, v2.y));

  return compare_vram("flat triangle", *gpu, expected);
}

bool test_gouraud_triangle() {
  auto gpu = std::make_unique<Gpu>();
  gpu->init(nullptr);
  gpu->reset();

  std::vector<u16> expected(kVramPixels, 0);

  const RefVertex v0{50, 12, 240, 24, 32, 0, 0};
  const RefVertex v1{91, 23, 20, 232, 48, 0, 0};
  const RefVertex v2{59, 55, 36, 64, 248, 0, 0};

  reference_triangle(
      expected, v0, v1, v2,
      [](const std::vector<u16> &, const RefVertex &a, const RefVertex &b,
         const RefVertex &c, s32 w0, s32 w1, s32 w2, s32 area, int, int) {
        const int r = (w0 * a.r + w1 * b.r + w2 * c.r) / area;
        const int g = (w0 * a.g + w1 * b.g + w2 * c.g) / area;
        const int bl = (w0 * a.b + w1 * b.b + w2 * c.b) / area;
        return pack_rgb15(r, g, bl);
      });

  gpu->gp0(rgb_command(0x30, v0.r, v0.g, v0.b));
  gpu->gp0(vertex_word(v0.x, v0.y));
  gpu->gp0(rgb_word(v1.r, v1.g, v1.b));
  gpu->gp0(vertex_word(v1.x, v1.y));
  gpu->gp0(rgb_word(v2.r, v2.g, v2.b));
  gpu->gp0(vertex_word(v2.x, v2.y));

  return compare_vram("gouraud triangle", *gpu, expected);
}

void seed_direct_texture(Gpu &gpu, std::vector<u16> &expected, int base_x,
                         int base_y, int width, int height, bool white) {
  u16 *actual = gpu.vram_mut_data();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const u16 pixel = white
                            ? static_cast<u16>(0x7FFFu)
                            : static_cast<u16>(
                                  1u | ((x & 0x1F) << 0) |
                                  ((y & 0x1F) << 5) |
                                  (((x + y) & 0x1F) << 10));
      const size_t index =
          static_cast<size_t>(base_y + y) * psx::VRAM_WIDTH +
          static_cast<size_t>(base_x + x);
      actual[index] = pixel;
      expected[index] = pixel;
    }
  }
}

bool test_raw_textured_triangle() {
  auto gpu = std::make_unique<Gpu>();
  gpu->init(nullptr);
  gpu->reset();

  std::vector<u16> expected(kVramPixels, 0);
  constexpr u16 texpage = 0x0104u; // X page 4 (256px), 15-bit direct.
  constexpr int tex_base_x = 256;
  constexpr int tex_base_y = 0;
  seed_direct_texture(*gpu, expected, tex_base_x, tex_base_y, 64, 64, false);

  const RefVertex v0{20, 70, 128, 128, 128, 2, 3};
  const RefVertex v1{70, 75, 128, 128, 128, 42, 5};
  const RefVertex v2{26, 120, 128, 128, 128, 6, 45};

  reference_triangle(
      expected, v0, v1, v2,
      [=](const std::vector<u16> &vram, const RefVertex &a, const RefVertex &b,
         const RefVertex &c, s32 w0, s32 w1, s32 w2, s32 area, int, int) {
        const u8 u = static_cast<u8>((w0 * a.u + w1 * b.u + w2 * c.u) / area);
        const u8 v = static_cast<u8>((w0 * a.v + w1 * b.v + w2 * c.v) / area);
        const size_t source =
            static_cast<size_t>(tex_base_y + v) * psx::VRAM_WIDTH +
            static_cast<size_t>(tex_base_x + u);
        return vram[source];
      });

  gpu->gp0(rgb_command(0x25, 128, 128, 128)); // Raw textured triangle.
  gpu->gp0(vertex_word(v0.x, v0.y));
  gpu->gp0(uv_word(v0.u, v0.v, 0));
  gpu->gp0(vertex_word(v1.x, v1.y));
  gpu->gp0(uv_word(v1.u, v1.v, texpage));
  gpu->gp0(vertex_word(v2.x, v2.y));
  gpu->gp0(uv_word(v2.u, v2.v, 0));

  return compare_vram("raw textured triangle", *gpu, expected);
}

bool test_gouraud_textured_triangle() {
  auto gpu = std::make_unique<Gpu>();
  gpu->init(nullptr);
  gpu->reset();

  std::vector<u16> expected(kVramPixels, 0);
  constexpr u16 texpage = 0x0105u; // X page 5 (320px), 15-bit direct.
  constexpr int tex_base_x = 320;
  constexpr int tex_base_y = 0;
  seed_direct_texture(*gpu, expected, tex_base_x, tex_base_y, 64, 64, true);

  const RefVertex v0{100, 70, 128, 72, 220, 3, 4};
  const RefVertex v1{152, 80, 240, 128, 52, 47, 8};
  const RefVertex v2{109, 130, 64, 244, 160, 9, 51};

  reference_triangle(
      expected, v0, v1, v2,
      [=](const std::vector<u16> &vram, const RefVertex &a, const RefVertex &b,
         const RefVertex &c, s32 w0, s32 w1, s32 w2, s32 area, int, int) {
        const u8 u = static_cast<u8>((w0 * a.u + w1 * b.u + w2 * c.u) / area);
        const u8 v = static_cast<u8>((w0 * a.v + w1 * b.v + w2 * c.v) / area);
        const int mr = (w0 * a.r + w1 * b.r + w2 * c.r) / area;
        const int mg = (w0 * a.g + w1 * b.g + w2 * c.g) / area;
        const int mb = (w0 * a.b + w1 * b.b + w2 * c.b) / area;
        const size_t source =
            static_cast<size_t>(tex_base_y + v) * psx::VRAM_WIDTH +
            static_cast<size_t>(tex_base_x + u);
        return modulate_texel(vram[source], clamp_u8(mr), clamp_u8(mg),
                              clamp_u8(mb));
      });

  gpu->gp0(rgb_command(0x34, v0.r, v0.g, v0.b));
  gpu->gp0(vertex_word(v0.x, v0.y));
  gpu->gp0(uv_word(v0.u, v0.v, 0));
  gpu->gp0(rgb_word(v1.r, v1.g, v1.b));
  gpu->gp0(vertex_word(v1.x, v1.y));
  gpu->gp0(uv_word(v1.u, v1.v, texpage));
  gpu->gp0(rgb_word(v2.r, v2.g, v2.b));
  gpu->gp0(vertex_word(v2.x, v2.y));
  gpu->gp0(uv_word(v2.u, v2.v, 0));

  return compare_vram("gouraud textured triangle", *gpu, expected);
}

} // namespace

int run_gpu_correctness_tests() {
  const bool old_fast = g_gpu_fast_mode;
  const bool old_extreme = g_gpu_extreme_fast_mode;
  g_gpu_fast_mode = false;
  g_gpu_extreme_fast_mode = false;

  const std::array<std::pair<const char *, bool (*)()>, 4> tests = {{
      {"flat triangle", &test_flat_triangle},
      {"gouraud triangle", &test_gouraud_triangle},
      {"raw textured triangle", &test_raw_textured_triangle},
      {"gouraud textured triangle", &test_gouraud_textured_triangle},
  }};

  int failed = 0;
  std::fprintf(stdout, "[GPU TEST] Running %zu deterministic GPU tests...\n",
               tests.size());
  for (const auto &test : tests) {
    if (!test.second()) {
      ++failed;
    }
  }

  g_gpu_fast_mode = old_fast;
  g_gpu_extreme_fast_mode = old_extreme;

  if (failed != 0) {
    std::fprintf(stderr, "[GPU TEST] %d/%zu tests failed.\n", failed,
                 tests.size());
    return 1;
  }

  std::fprintf(stdout, "[GPU TEST] All %zu tests passed.\n", tests.size());
  return 0;
}
