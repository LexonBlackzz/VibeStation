#include "ui/definitive/definitive_shared.h"
#include "ui/embedded_resource_ids.h"
#include "ui/embedded_resources.h"

#include <SDL.h>
#define STB_VORBIS_HEADER_ONLY
#include <stb_vorbis.c>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <utility>
#include <vector>

// The launcher photo's CRT shows a small real-time recreation of the console
// logo: a red block "P" standing on a flat "S" of yellow, teal and blue bands,
// slowly turning, glowing on the photo's CRT glass. Clicking it glitches the
// picture; rarely the TV then freezes on that frame until the next launch.
// Drawn with ImGui polygons from a tiny painter-sorted 3D mesh.

namespace {

constexpr float kPi = 3.14159265358979f;

// The lit glass of the TV, in background-photo pixels: the set is angled, so
// its top edge slopes up to the right. Corners (TL, TR, BR, BL), inset a
// little from the bezel.
constexpr std::array<std::array<float, 2>, 4> kScreen = {{
    {915.0f, 75.0f}, {1176.0f, 33.0f}, {1179.0f, 298.0f}, {914.0f, 298.0f}}};

struct Vec3 {
    float x, y, z;
};

Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 normalized(const Vec3& v) {
    const float len = std::sqrt(dot(v, v));
    return len > 1e-6f ? Vec3{v.x / len, v.y / len, v.z / len} : Vec3{0, 0, 1};
}

struct Rgb {
    float r, g, b;
};

struct Face {
    std::array<Vec3, 4> v;
    Rgb color;
    int layer = 0; // drawn in layer order, depth-sorted within a layer
};

// A band of half-width `hw` along a 2D centre line, extruded `thickness`
// along the third axis. `place(a, b, h)` maps band coordinates (a, b) and
// height h to model space. `color_at(a, b)` colours each segment by
// where its middle lies on the centre line.
template <typename Place, typename ColorAt>
void add_band(std::vector<Face>& faces, const std::vector<std::array<float, 2>>& path,
              float hw, float h0, float h1, Place place, ColorAt color_at) {
    const size_t n = path.size();
    if (n < 2) return;
    std::vector<std::array<float, 2>> inner(n), outer(n);
    for (size_t i = 0; i < n; ++i) {
        const auto& prev = path[i == 0 ? 0 : i - 1];
        const auto& next = path[i + 1 < n ? i + 1 : n - 1];
        float dx = next[0] - prev[0], dy = next[1] - prev[1];
        const float len = std::max(1e-6f, std::sqrt(dx * dx + dy * dy));
        dx /= len;
        dy /= len;
        inner[i] = {path[i][0] - dy * hw, path[i][1] + dx * hw};
        outer[i] = {path[i][0] + dy * hw, path[i][1] - dx * hw};
    }
    const auto p = [&](const std::array<float, 2>& q, float h) { return place(q[0], q[1], h); };
    for (size_t i = 0; i + 1 < n; ++i) {
        const Rgb c = color_at((path[i][0] + path[i + 1][0]) * 0.5f, (path[i][1] + path[i + 1][1]) * 0.5f);
        faces.push_back({{p(inner[i], h1), p(outer[i], h1), p(outer[i + 1], h1), p(inner[i + 1], h1)}, c});
        faces.push_back({{p(inner[i], h0), p(inner[i + 1], h0), p(outer[i + 1], h0), p(outer[i], h0)}, c});
        faces.push_back({{p(outer[i], h0), p(outer[i + 1], h0), p(outer[i + 1], h1), p(outer[i], h1)}, c});
        faces.push_back({{p(inner[i], h0), p(inner[i], h1), p(inner[i + 1], h1), p(inner[i + 1], h0)}, c});
    }
    const Rgb c0 = color_at(path[0][0], path[0][1]);
    const Rgb c1 = color_at(path[n - 1][0], path[n - 1][1]);
    faces.push_back({{p(inner[0], h0), p(outer[0], h0), p(outer[0], h1), p(inner[0], h1)}, c0});
    faces.push_back({{p(inner[n - 1], h0), p(inner[n - 1], h1), p(outer[n - 1], h1), p(outer[n - 1], h0)}, c1});
}

void arc(std::vector<std::array<float, 2>>& path, float cx, float cy, float r, float a0,
         float a1, int steps) {
    for (int i = 0; i <= steps; ++i) {
        const float a = a0 + (a1 - a0) * static_cast<float>(i) / static_cast<float>(steps);
        path.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
}

// The logo mesh in model space: y up, the S lying in the x/z plane.
const std::vector<Face>& logo_mesh() {
    static const std::vector<Face> faces = [] {
        std::vector<Face> out;
        const Rgb red{0.86f, 0.10f, 0.13f};
        const Rgb yellow{0.96f, 0.72f, 0.12f};
        const Rgb teal{0.12f, 0.66f, 0.62f};
        const Rgb blue{0.20f, 0.36f, 0.82f};

        // S: a thin strip lying on the floor, snaking through three parallel
        // lanes joined by two U-turns, built along a (lane length) and b (lane
        // spacing). Coloured in bands along a: yellow at the P's end, teal in
        // the middle, blue at the far end, so every lane and turn repeats them.
        constexpr float kLane = 0.7f;   // lane length
        constexpr float kLaneW = 0.18f; // strip width
        constexpr float kPitch = 0.30f; // lane centre spacing
        constexpr float kStrip = 0.06f; // strip thickness
        constexpr int kLaneSteps = 12;
        std::vector<std::array<float, 2>> s_path;
        // The front lane starts level with the near U-turn's tip, so nothing
        // pokes out in front of the P standing there.
        constexpr float kLead = kPitch * 0.5f;
        s_path.push_back({-kLead, 0.0f});
        for (int i = 0; i <= kLaneSteps; ++i) {
            s_path.push_back({kLane * static_cast<float>(i) / kLaneSteps, 0.0f});
        }
        std::vector<std::array<float, 2>> turn;
        arc(turn, kLane, kPitch * 0.5f, kPitch * 0.5f, -0.5f * kPi, 0.5f * kPi, 14);
        s_path.insert(s_path.end(), turn.begin() + 1, turn.end());
        for (int i = kLaneSteps - 1; i >= 0; --i) {
            s_path.push_back({kLane * static_cast<float>(i) / kLaneSteps, kPitch});
        }
        turn.clear();
        arc(turn, 0.0f, kPitch * 1.5f, kPitch * 0.5f, 1.5f * kPi, 0.5f * kPi, 14);
        s_path.insert(s_path.end(), turn.begin() + 1, turn.end());
        for (int i = 1; i <= kLaneSteps; ++i) {
            s_path.push_back({kLane * static_cast<float>(i) / kLaneSteps, 2.0f * kPitch});
        }
        // Turned a quarter anticlockwise (seen from above): lanes run away
        // from the viewer (strip a -> depth z), stacked to the left (b -> -x),
        // so the front lane's near end is where the P stands. kShiftX roughly
        // centres the whole logo.
        constexpr float kShiftX = -0.15f;
        constexpr float kCz = kLane * 0.5f;
        add_band(out, s_path, kLaneW * 0.5f, 0.0f, kStrip,
                 [](float a, float b, float h) { return Vec3{-b - kShiftX, h, a - kCz}; },
                 [&](float a, float) {
                     return a < kLane / 3.0f ? yellow : (a < kLane * 2.0f / 3.0f ? teal : blue);
                 });

        // The P stands in front of the rest of the S, so it is drawn after
        // it: sorting single faces by depth cannot order these long bands.
        const size_t s_faces = out.size();
        // P: a thin slab standing on the start of the front lane, its stem
        // as wide as the strip, its bowl a rounded loop with a narrow slot.
        constexpr float kHeight = 1.0f;
        constexpr float kSlab = 0.07f;      // thickness
        constexpr float kBowlHalf = 0.085f; // half the bowl band's width
        constexpr float kBowlR = 0.15f;     // bowl centre-line radius
        // Facing the viewer, its stem centred on the front lane at the S's end.
        const auto stand = [](float a, float b, float h) {
            return Vec3{a - kLaneW * 0.5f - kShiftX, b, h - kCz - kLead + kSlab * 0.5f};
        };
        const auto solid_red = [&](float, float) { return red; };
        add_band(out, {{kLaneW * 0.5f, 0.0f}, {kLaneW * 0.5f, kHeight}}, kLaneW * 0.5f,
                 -kSlab * 0.5f, kSlab * 0.5f, stand, solid_red);
        const float bowl_cx = kLaneW + 0.05f;
        const float bowl_cy = kHeight - kBowlHalf - kBowlR;
        std::vector<std::array<float, 2>> bowl;
        bowl.push_back({kLaneW - 0.02f, kHeight - kBowlHalf});
        arc(bowl, bowl_cx, bowl_cy, kBowlR, 0.5f * kPi, -0.5f * kPi, 18);
        bowl.push_back({kLaneW - 0.02f, bowl_cy - kBowlR});
        add_band(out, bowl, kBowlHalf, -kSlab * 0.5f, kSlab * 0.5f, stand, solid_red);
        for (size_t i = s_faces; i < out.size(); ++i) out[i].layer = 1;
        return out;
    }();
    return faces;
}

ImU32 with_alpha(Rgb c, float a) {
    const auto ch = [](float v) { return static_cast<int>(std::clamp(v, 0.0f, 1.0f) * 255.0f); };
    return IM_COL32(ch(c.r), ch(c.g), ch(c.b), static_cast<int>(std::clamp(a, 0.0f, 1.0f) * 255.0f));
}

struct Projected {
    std::array<ImVec2, 4> pts;
    float depth;
    int layer;
    Rgb rgb;
};

// The logo's faces in screen space, in drawing order.
std::vector<Projected> project_logo(ImVec2 centre, float scale, float time) {
    // A slow turn back and forth, a little tilt, a gentle bob.
    const float yaw = 0.5f * std::sin(time * 0.45f);
    const float pitch = 0.62f + 0.04f * std::sin(time * 0.31f);
    const float bob = 0.04f * std::sin(time * 0.9f);
    const float cy = std::cos(yaw), sy = std::sin(yaw);
    const float cp = std::cos(pitch), sp = std::sin(pitch);
    constexpr float kCamera = 5.2f;
    const Vec3 light = normalized({-0.5f, 0.9f, -0.6f});

    std::vector<Projected> out;
    const auto& mesh = logo_mesh();
    out.reserve(mesh.size());
    for (const Face& f : mesh) {
        std::array<Vec3, 4> v;
        for (int i = 0; i < 4; ++i) {
            const Vec3 m{f.v[i].x, f.v[i].y - 0.30f + bob, f.v[i].z};
            const Vec3 r{m.x * cy + m.z * sy, m.y, -m.x * sy + m.z * cy}; // yaw
            // Look down on it (near parts drop), then push it away.
            v[i] = {r.x, r.y * cp + r.z * sp, -r.y * sp + r.z * cp + kCamera};
        }
        Vec3 n = normalized(cross(v[1] - v[0], v[2] - v[0]));
        const Vec3 mid{(v[0].x + v[2].x) * 0.5f, (v[0].y + v[2].y) * 0.5f, (v[0].z + v[2].z) * 0.5f};
        if (dot(n, mid) > 0.0f) n = {-n.x, -n.y, -n.z}; // face the camera
        // Dimmed to sit in the dark room: a lit CRT, not a monitor.
        const float shade = 0.72f * (0.38f + 0.62f * std::max(0.0f, dot(n, light)));
        Projected p;
        for (int i = 0; i < 4; ++i) {
            const float k = scale / v[i].z;
            p.pts[i] = ImVec2(centre.x + v[i].x * k, centre.y - v[i].y * k);
        }
        p.depth = (v[0].z + v[1].z + v[2].z + v[3].z) * 0.25f;
        p.layer = f.layer;
        p.rgb = {f.color.r * shade, f.color.g * shade, f.color.b * shade};
        out.push_back(p);
    }
    std::sort(out.begin(), out.end(),
              [](const Projected& a, const Projected& b) {
                  return a.layer != b.layer ? a.layer < b.layer : a.depth > b.depth;
              });
    return out;
}

// Fills a quad as an n x n grid of cells (no anti-aliased fringe, so the
// cells meet without seams), for faces big enough that the per-vertex edge
// fade would otherwise stretch across them.
void fill_quad_grid(ImDrawList* draw, const std::array<ImVec2, 4>& pts, ImU32 col, int n) {
    const ImVec2 white = ImGui::GetFontTexUvWhitePixel();
    const auto at = [&](int c, int r) {
        const float u = static_cast<float>(c) / n;
        const float v = static_cast<float>(r) / n;
        const ImVec2 top(pts[0].x + (pts[1].x - pts[0].x) * u, pts[0].y + (pts[1].y - pts[0].y) * u);
        const ImVec2 bot(pts[3].x + (pts[2].x - pts[3].x) * u, pts[3].y + (pts[2].y - pts[3].y) * u);
        return ImVec2(top.x + (bot.x - top.x) * v, top.y + (bot.y - top.y) * v);
    };
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            draw->PrimReserve(6, 4);
            const auto base = static_cast<ImDrawIdx>(draw->_VtxCurrentIdx);
            draw->PrimWriteIdx(base);
            draw->PrimWriteIdx(static_cast<ImDrawIdx>(base + 1));
            draw->PrimWriteIdx(static_cast<ImDrawIdx>(base + 2));
            draw->PrimWriteIdx(base);
            draw->PrimWriteIdx(static_cast<ImDrawIdx>(base + 2));
            draw->PrimWriteIdx(static_cast<ImDrawIdx>(base + 3));
            draw->PrimWriteVtx(at(c, r), white, col);
            draw->PrimWriteVtx(at(c + 1, r), white, col);
            draw->PrimWriteVtx(at(c + 1, r + 1), white, col);
            draw->PrimWriteVtx(at(c, r + 1), white, col);
        }
    }
}

// edge_fade (1 well inside the glass, 0 at its edge) decides which faces
// are split up so the caller's per-vertex fade can follow the edge.
void draw_logo(ImDrawList* draw, const std::vector<Projected>& faces, ImVec2 centre, float scale,
               float alpha, const std::function<float(const ImVec2&)>& edge_fade) {
    const float max_piece = scale * 0.03f;
    // One pass of the logo, grown about the centre, nudged, brightened and
    // faded; the passes below build the phosphor bloom.
    const auto pass = [&](float grow, ImVec2 nudge, float boost, float pass_alpha) {
        for (const Projected& p : faces) {
            std::array<ImVec2, 4> pts;
            for (int i = 0; i < 4; ++i) {
                pts[i] = ImVec2(centre.x + (p.pts[i].x - centre.x) * grow + nudge.x,
                                centre.y + (p.pts[i].y - centre.y) * grow + nudge.y);
            }
            const ImU32 col =
                with_alpha({p.rgb.r * boost, p.rgb.g * boost, p.rgb.b * boost}, alpha * pass_alpha);
            float longest = 0.0f;
            for (int i = 0; i < 4; ++i) {
                const ImVec2& a = pts[i];
                const ImVec2& b = pts[(i + 1) % 4];
                longest = std::max(longest, std::hypot(b.x - a.x, b.y - a.y));
            }
            bool inside = true;
            for (const ImVec2& pt : pts) inside = inside && edge_fade(pt) >= 1.0f;
            if (inside || longest <= max_piece) {
                draw->AddConvexPolyFilled(pts.data(), 4, col);
            } else {
                // Reaching into the fading edge.
                fill_quad_grid(draw, pts, col, std::min(24, static_cast<int>(longest / max_piece) + 1));
            }
        }
    };
    // Everything but the last pass sits underneath the logo, so faces it
    // hides never show through. Light bleeding past the edges, widest and
    // faintest first...
    pass(1.18f, ImVec2(0, 0), 1.6f, 0.035f);
    pass(1.10f, ImVec2(0, 0), 1.5f, 0.06f);
    pass(1.05f, ImVec2(0, 0), 1.4f, 0.09f);
    // ...a soft fringe so the edges are slightly diffused, never razor-sharp...
    const float d = std::max(0.8f, scale * 0.005f);
    pass(1.0f, ImVec2(d, 0), 1.3f, 0.22f);
    pass(1.0f, ImVec2(-d, 0), 1.3f, 0.22f);
    pass(1.0f, ImVec2(0, d), 1.3f, 0.22f);
    pass(1.0f, ImVec2(0, -d), 1.3f, 0.22f);
    // ...and the image itself.
    pass(1.0f, ImVec2(0, 0), 1.0f, 1.0f);
}

// ------------------------------------------------------------------ glitch
// Clicking the logo corrupts it for a moment, like a console whose GPU is
// being fed garbage: vertices flung into long shards, light rays, tearing.

constexpr float kGlitchSeconds = 3.2f;
float g_glitch_start = -1.0f; // ImGui time of the click; < 0 when idle
// The rare glitch ("fearful harmony") freezes the TV on one of its frames
// for the rest of the run: logo, shards and flicker all stop.
constexpr float kRareGlitchChance = 0.10f;
constexpr float kFreezeAt = 0.45f; // seconds into the glitch, at full strength
bool g_freeze_pending = false;
bool g_frozen = false;
float g_frozen_time = 0.0f;
std::uint32_t g_glitch_seed = 0;

float hash01(std::uint32_t a, std::uint32_t b, std::uint32_t c) {
    std::uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u) * 0x85EBCA77u ^
                      (c + 0x165667B1u) * 0xC2B2AE3Du;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return static_cast<float>(h & 0xFFFFFFu) / 16777216.0f;
}

// 0 when idle; rises at once, holds, then dies away over the last second.
float glitch_intensity(float elapsed) {
    if (elapsed < 0.0f || elapsed >= kGlitchSeconds) return 0.0f;
    const float up = std::min(1.0f, 0.25f + elapsed / 0.06f);
    const float fall = std::clamp((elapsed - 2.1f) / (kGlitchSeconds - 2.1f), 0.0f, 1.0f);
    return up * (1.0f - fall * fall * (3.0f - 2.0f * fall));
}

// Flings vertices of some faces far out from the centre; which ones changes
// many times a second.
void corrupt_logo(std::vector<Projected>& faces, ImVec2 centre, float scale, float intensity,
                  std::uint32_t bucket) {
    for (std::size_t i = 0; i < faces.size(); ++i) {
        const auto id = static_cast<std::uint32_t>(i);
        if (hash01(id, bucket, g_glitch_seed) >= intensity * 0.22f) continue;
        const int k = static_cast<int>(hash01(id, bucket, g_glitch_seed ^ 1u) * 4.0f) & 3;
        const float angle = hash01(id, bucket, g_glitch_seed ^ 2u) * 2.0f * kPi;
        const float len = scale * (0.22f + 0.95f * hash01(id, bucket, g_glitch_seed ^ 3u));
        faces[i].pts[static_cast<std::size_t>(k)] =
            ImVec2(centre.x + std::cos(angle) * len, centre.y + std::sin(angle) * len);
        // Shards burn brighter than the logo they came from.
        faces[i].rgb = {faces[i].rgb.r * 1.6f, faces[i].rgb.g * 1.6f, faces[i].rgb.b * 1.6f};
    }
}

void draw_glitch_overlay(ImDrawList* draw, ImVec2 centre, float span, float intensity,
                         std::uint32_t bucket, float min_x, float max_x, float min_y, float max_y,
                         float alpha) {
    constexpr std::array<Rgb, 5> kRayColors = {{
        {1.0f, 0.82f, 0.18f}, {0.25f, 0.45f, 1.0f}, {0.95f, 0.15f, 0.18f},
        {0.15f, 0.85f, 0.75f}, {0.95f, 0.95f, 1.0f}}};
    // Thin rays and narrow wedges of light out of the centre. Everything is
    // drawn in short pieces so the caller's per-vertex edge fade follows the
    // glass instead of being stretched along one long primitive.
    constexpr int kPieces = 16;
    const auto lerp = [](const ImVec2& p, const ImVec2& q, float t) {
        return ImVec2(p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t);
    };
    const int rays = static_cast<int>(intensity * 34.0f);
    for (int r = 0; r < rays; ++r) {
        const auto id = static_cast<std::uint32_t>(r) + 1000u;
        const float angle = hash01(id, bucket, g_glitch_seed) * 2.0f * kPi;
        const float r0 = span * 0.06f * hash01(id, bucket, g_glitch_seed ^ 5u);
        const float len = span * (0.35f + 1.1f * hash01(id, bucket, g_glitch_seed ^ 6u));
        const Rgb c = kRayColors[static_cast<std::size_t>(
            hash01(id, bucket, g_glitch_seed ^ 7u) * kRayColors.size()) % kRayColors.size()];
        const ImVec2 dir(std::cos(angle), std::sin(angle));
        const ImVec2 a(centre.x + dir.x * r0, centre.y + dir.y * r0);
        if (hash01(id, bucket, g_glitch_seed ^ 8u) < 0.3f) {
            const float spread = 0.015f + 0.05f * hash01(id, bucket, g_glitch_seed ^ 9u);
            const ImVec2 b(centre.x + std::cos(angle - spread) * len,
                           centre.y + std::sin(angle - spread) * len);
            const ImVec2 e(centre.x + std::cos(angle + spread) * len,
                           centre.y + std::sin(angle + spread) * len);
            const ImU32 col = with_alpha(c, alpha * intensity * 0.35f);
            for (int p = 0; p < kPieces; ++p) {
                const float t0 = static_cast<float>(p) / kPieces;
                const float t1 = static_cast<float>(p + 1) / kPieces;
                if (p == 0) {
                    draw->AddTriangleFilled(a, lerp(a, b, t1), lerp(a, e, t1), col);
                } else {
                    draw->AddQuadFilled(lerp(a, b, t0), lerp(a, b, t1), lerp(a, e, t1),
                                        lerp(a, e, t0), col);
                }
            }
        } else {
            const ImVec2 b(centre.x + dir.x * (r0 + len), centre.y + dir.y * (r0 + len));
            const ImU32 col = with_alpha(c, alpha * intensity * 0.75f);
            const float thickness = 1.0f + 1.6f * hash01(id, bucket, g_glitch_seed ^ 10u);
            for (int p = 0; p < kPieces; ++p) {
                draw->AddLine(lerp(a, b, static_cast<float>(p) / kPieces),
                              lerp(a, b, static_cast<float>(p + 1) / kPieces), col, thickness);
            }
        }
    }
    // Horizontal tearing: bands of the picture's light smeared sideways.
    const int bands = static_cast<int>(intensity * 7.0f);
    for (int t = 0; t < bands; ++t) {
        const auto id = static_cast<std::uint32_t>(t) + 2000u;
        const float y = min_y + (max_y - min_y) * hash01(id, bucket, g_glitch_seed);
        const float hgt = 1.0f + (max_y - min_y) * 0.025f * hash01(id, bucket, g_glitch_seed ^ 1u);
        const Rgb c = kRayColors[static_cast<std::size_t>(
            hash01(id, bucket, g_glitch_seed ^ 2u) * kRayColors.size()) % kRayColors.size()];
        const ImU32 col = with_alpha(c, alpha * intensity * 0.22f);
        for (int p = 0; p < kPieces; ++p) {
            const float x0 = min_x + (max_x - min_x) * static_cast<float>(p) / kPieces;
            const float x1 = min_x + (max_x - min_x) * static_cast<float>(p + 1) / kPieces;
            draw->AddRectFilled(ImVec2(x0, y), ImVec2(x1, y + hgt), col);
        }
    }
}

// ------------------------------------------------------------------ sound
// Embedded in the exe on Windows (copied next to it elsewhere): feared.ogg
// for an ordinary glitch, fearfulambience.ogg ("fearful harmony") for the
// rare one. Their own device, so launcher sounds never cut them off.

struct GlitchSound {
    int resource_id;
    const char* file;
    std::vector<short> pcm;
    int channels = 0;
    int rate = 0;
    bool load_tried = false;
};

GlitchSound g_feared_sound{vibestation::resource_ids::TvFearedOgg, "feared.ogg", {}};
GlitchSound g_harmony_sound{vibestation::resource_ids::TvGlitchOgg, "fearfulambience.ogg", {}};

// Each press plays on its own device, so a new sound layers over one still
// playing instead of cutting it off.
struct Voice {
    SDL_AudioDeviceID device = 0;
    const GlitchSound* sound = nullptr;
    bool fading = false;
};
std::vector<Voice> g_voices;

std::vector<unsigned char> read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return {};
    return std::vector<unsigned char>((std::istreambuf_iterator<char>(in)),
                                      std::istreambuf_iterator<char>());
}

bool load_sound(GlitchSound& sound) {
    if (!sound.pcm.empty()) return true;
    if (sound.load_tried) return false;
    sound.load_tried = true;

    std::vector<unsigned char> file;
    if (const auto res = vibestation::embedded_resource(sound.resource_id)) {
        file.assign(res.data, res.data + res.size);
    } else {
        std::vector<std::filesystem::path> candidates;
        if (char* base = SDL_GetBasePath()) {
            candidates.push_back(std::filesystem::path(base) / "resources" / "ui" / "definitive" /
                                 "sounds" / sound.file);
            SDL_free(base);
        }
        candidates.push_back(std::filesystem::current_path() / "resources" / "ui" / "definitive" /
                             "sounds" / sound.file);
        for (const auto& path : candidates) {
            file = read_file(path);
            if (!file.empty()) break;
        }
    }
    if (file.empty()) return false;

    short* samples = nullptr;
    const int frames = stb_vorbis_decode_memory(file.data(), static_cast<int>(file.size()),
                                                &sound.channels, &sound.rate, &samples);
    if (frames <= 0 || samples == nullptr || sound.channels <= 0) {
        std::free(samples);
        return false;
    }
    sound.pcm.assign(samples, samples + static_cast<std::size_t>(frames) * sound.channels);
    std::free(samples);
    return true;
}

void close_voice(Voice& v) {
    if (v.device != 0) {
        SDL_ClearQueuedAudio(v.device);
        SDL_CloseAudioDevice(v.device);
        v.device = 0;
    }
}

// Closes voices that have finished playing.
void reap_voices() {
    for (Voice& v : g_voices) {
        if (v.device != 0 && SDL_GetQueuedAudioSize(v.device) == 0) close_voice(v);
    }
    g_voices.erase(std::remove_if(g_voices.begin(), g_voices.end(),
                                  [](const Voice& v) { return v.device == 0; }),
                   g_voices.end());
}

void play_sound(GlitchSound& sound) {
    if (!load_sound(sound)) return;
    reap_voices();
    SDL_AudioSpec spec{};
    spec.freq = sound.rate;
    spec.format = AUDIO_S16SYS;
    spec.channels = static_cast<Uint8>(sound.channels);
    spec.samples = 1024;
    // allowed_changes = 0: SDL converts to the hardware format.
    Voice v;
    v.device = SDL_OpenAudioDevice(nullptr, 0, &spec, nullptr, 0);
    if (v.device == 0) return;
    v.sound = &sound;
    SDL_QueueAudio(v.device, sound.pcm.data(),
                   static_cast<Uint32>(sound.pcm.size() * sizeof(short)));
    SDL_PauseAudioDevice(v.device, 0);
    g_voices.push_back(v);
}

// Re-queues the next 0.4 s of what the voice has left, ramped to silence.
void fade_voice(Voice& v) {
    if (v.fading || v.device == 0 || v.sound == nullptr) return;
    v.fading = true;
    const GlitchSound& sound = *v.sound;
    const std::size_t frame_shorts = static_cast<std::size_t>(sound.channels);
    const std::size_t total = sound.pcm.size() / frame_shorts;
    const std::size_t queued = std::min<std::size_t>(
        SDL_GetQueuedAudioSize(v.device) / (sizeof(short) * frame_shorts), total);
    const std::size_t start = total - queued;
    const std::size_t frames = std::min(queued, static_cast<std::size_t>(sound.rate) * 2u / 5u);
    std::vector<short> tail(sound.pcm.begin() + static_cast<std::ptrdiff_t>(start * frame_shorts),
                            sound.pcm.begin() +
                                static_cast<std::ptrdiff_t>((start + frames) * frame_shorts));
    for (std::size_t f = 0; f < frames; ++f) {
        const float gain = frames <= 1 ? 0.0f
                                       : 1.0f - static_cast<float>(f) / static_cast<float>(frames - 1);
        for (std::size_t c = 0; c < frame_shorts; ++c) {
            short& s = tail[f * frame_shorts + c];
            s = static_cast<short>(static_cast<float>(s) * gain);
        }
    }
    SDL_ClearQueuedAudio(v.device);
    SDL_QueueAudio(v.device, tail.data(), static_cast<Uint32>(tail.size() * sizeof(short)));
}

} // namespace

void definitive_ui::stop_tv_glitch_sound(bool fade) {
    if (g_voices.empty()) return;
    for (Voice& v : g_voices) {
        if (fade) {
            fade_voice(v);
        } else {
            close_voice(v);
        }
    }
    if (!fade) g_voices.clear();
}

void definitive_ui::draw_launcher_tv(ImDrawList* draw, const ImVec2& pos, const ImVec2& size,
                                     float opacity, float zoom, float real_time, bool interactive) {
    if (draw == nullptr || opacity <= 0.001f) return;
    if (g_freeze_pending && g_glitch_start >= 0.0f && real_time - g_glitch_start >= kFreezeAt) {
        g_freeze_pending = false;
        g_frozen = true;
        g_frozen_time = g_glitch_start + kFreezeAt;
    }
    // Everything below runs on this clock, which stops for good once frozen.
    const float time = g_frozen ? g_frozen_time : real_time;
    std::array<ImVec2, 4> q;
    for (int i = 0; i < 4; ++i) {
        if (!launcher_photo_point(pos, size, zoom, kScreen[i][0], kScreen[i][1], q[i])) return;
    }
    float min_x = q[0].x, max_x = q[0].x, min_y = q[0].y, max_y = q[0].y;
    for (const ImVec2& p : q) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    const float w = max_x - min_x;
    const float h = max_y - min_y;
    if (w < 8.0f || h < 8.0f) return;
    const float px_per_photo_px = w / (kScreen[2][0] - kScreen[3][0]);
    // A CRT never holds perfectly still.
    float flicker = 0.96f + 0.04f * std::sin(time * 57.0f) * std::sin(time * 3.1f);
    const float a = std::clamp(opacity, 0.0f, 1.0f);

    // The span of the (convex) screen quad on row y, or false outside it.
    const auto row_span = [&](float y, float& x0, float& x1) {
        x0 = 1e9f;
        x1 = -1e9f;
        for (int i = 0; i < 4; ++i) {
            const ImVec2& p = q[i];
            const ImVec2& n = q[(i + 1) % 4];
            if ((y < std::min(p.y, n.y)) || (y > std::max(p.y, n.y)) || p.y == n.y) continue;
            const float x = p.x + (n.x - p.x) * (y - p.y) / (n.y - p.y);
            x0 = std::min(x0, x);
            x1 = std::max(x1, x);
        }
        return x1 > x0;
    };

    // The photo's own screen shows through; the logo only adds its light,
    // centred on the glass.
    ImVec2 centre((q[0].x + q[1].x + q[2].x + q[3].x) * 0.25f,
                  (q[0].y + q[1].y + q[2].y + q[3].y) * 0.25f + h * 0.03f);
    const float span = std::min(w, h);
    const float scale = span * 1.55f;

    // A click on the logo starts the glitch, once the last one has worn off
    // and never once the TV has frozen.
    std::vector<Projected> faces = project_logo(centre, scale, time);
    if (interactive && !g_frozen && g_glitch_start < 0.0f && opacity > 0.98f &&
        ImGui::IsWindowHovered() && !ImGui::IsAnyItemHovered() &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        float lx0 = 1e9f, ly0 = 1e9f, lx1 = -1e9f, ly1 = -1e9f;
        for (const Projected& p : faces) {
            for (const ImVec2& pt : p.pts) {
                lx0 = std::min(lx0, pt.x);
                ly0 = std::min(ly0, pt.y);
                lx1 = std::max(lx1, pt.x);
                ly1 = std::max(ly1, pt.y);
            }
        }
        const ImVec2 m = ImGui::GetMousePos();
        if (m.x >= lx0 && m.x <= lx1 && m.y >= ly0 && m.y <= ly1) {
            g_glitch_start = time;
            g_glitch_seed = SDL_GetTicks() * 2654435761u ^ static_cast<std::uint32_t>(m.x * 97.0f);
            if (hash01(g_glitch_seed, 77u, 1u) < kRareGlitchChance) {
                g_freeze_pending = true;
                play_sound(g_harmony_sound);
            } else {
                play_sound(g_feared_sound);
            }
        }
    }
    float intensity = g_glitch_start >= 0.0f ? glitch_intensity(time - g_glitch_start) : 0.0f;
    if (g_glitch_start >= 0.0f && time - g_glitch_start >= kGlitchSeconds) g_glitch_start = -1.0f;
    const auto bucket = static_cast<std::uint32_t>(std::max(0.0f, time - g_glitch_start) * 18.0f);
    if (intensity > 0.0f) {
        // Strobing, a jumping picture and a harsher flicker.
        if (hash01(bucket, 1u, g_glitch_seed) < 0.15f) intensity *= 0.35f;
        centre.x += (hash01(bucket, 2u, g_glitch_seed) - 0.5f) * span * 0.07f * intensity;
        flicker *= 1.0f - 0.35f * intensity * hash01(bucket, 3u, g_glitch_seed);
        faces = project_logo(centre, scale, time);
        corrupt_logo(faces, centre, scale, intensity, bucket);
    }

    // How far inside the glass a point is, as a 0..1 fade over the outer
    // feather, so everything drawn melts into the photo's own screen instead
    // of stopping at a hard edge.
    const float feather = span * 0.14f;
    const auto edge_fade = [&](const ImVec2& pt) {
        float d = 1e9f;
        for (int i = 0; i < 4; ++i) {
            const ImVec2& p = q[i];
            const ImVec2& n = q[(i + 1) % 4];
            const float ex = n.x - p.x;
            const float ey = n.y - p.y;
            const float len = std::sqrt(ex * ex + ey * ey);
            if (len <= 0.0f) continue;
            d = std::min(d, ((pt.x - p.x) * -ey + (pt.y - p.y) * ex) / len);
        }
        const float t = std::clamp(d / feather, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    };
    // Fades every vertex added since first_vtx by edge_fade.
    const auto fade_edges = [&](int first_vtx) {
        for (int i = first_vtx; i < draw->VtxBuffer.Size; ++i) {
            ImDrawVert& v = draw->VtxBuffer[i];
            const auto al = static_cast<ImU32>(
                static_cast<float>((v.col >> IM_COL32_A_SHIFT) & 0xFF) * edge_fade(v.pos) + 0.5f);
            v.col = (v.col & ~IM_COL32_A_MASK) | (al << IM_COL32_A_SHIFT);
        }
    };

    // The lit tube: the whole glass a touch brighter, and a soft haze around
    // the image, warm from the red P. Drawn as a fine grid over the quad so
    // its colour and the edge fade can vary per vertex.
    {
        constexpr int kCols = 24;
        constexpr int kRows = 18;
        const float haze_r = span * 0.455f;
        const auto grid_point = [&](int c, int r) {
            const float u = static_cast<float>(c) / kCols;
            const float v = static_cast<float>(r) / kRows;
            const ImVec2 top(q[0].x + (q[1].x - q[0].x) * u, q[0].y + (q[1].y - q[0].y) * u);
            const ImVec2 bot(q[3].x + (q[2].x - q[3].x) * u, q[3].y + (q[2].y - q[3].y) * u);
            return ImVec2(top.x + (bot.x - top.x) * v, top.y + (bot.y - top.y) * v);
        };
        const auto grid_col = [&](const ImVec2& pt) {
            const float dx = pt.x - centre.x;
            const float dy = pt.y - centre.y;
            const float haze =
                42.0f * std::clamp(1.0f - std::sqrt(dx * dx + dy * dy) / haze_r, 0.0f, 1.0f);
            const float total = 12.0f + haze;
            const float k = haze / total;
            const auto mix = [&](float tube, float warm) { return static_cast<int>(tube + (warm - tube) * k); };
            return IM_COL32(mix(150.0f, 205.0f), mix(172.0f, 160.0f), mix(205.0f, 160.0f),
                            static_cast<int>(total * a * flicker * edge_fade(pt)));
        };
        const ImVec2 white = ImGui::GetFontTexUvWhitePixel();
        for (int r = 0; r < kRows; ++r) {
            for (int c = 0; c < kCols; ++c) {
                const ImVec2 p0 = grid_point(c, r);
                const ImVec2 p1 = grid_point(c + 1, r);
                const ImVec2 p2 = grid_point(c + 1, r + 1);
                const ImVec2 p3 = grid_point(c, r + 1);
                draw->PrimReserve(6, 4);
                const auto base = static_cast<ImDrawIdx>(draw->_VtxCurrentIdx);
                draw->PrimWriteIdx(base);
                draw->PrimWriteIdx(static_cast<ImDrawIdx>(base + 1));
                draw->PrimWriteIdx(static_cast<ImDrawIdx>(base + 2));
                draw->PrimWriteIdx(base);
                draw->PrimWriteIdx(static_cast<ImDrawIdx>(base + 2));
                draw->PrimWriteIdx(static_cast<ImDrawIdx>(base + 3));
                draw->PrimWriteVtx(p0, white, grid_col(p0));
                draw->PrimWriteVtx(p1, white, grid_col(p1));
                draw->PrimWriteVtx(p2, white, grid_col(p2));
                draw->PrimWriteVtx(p3, white, grid_col(p3));
            }
        }
    }

    const int logo_vtx = draw->VtxBuffer.Size;

    // Clipped to the glass's bounds only: the edge fade below takes
    // everything to nothing along its (sloped) edges, so no hard cut shows.
    draw->PushClipRect(ImVec2(min_x, min_y), ImVec2(max_x, max_y), true);
    draw_logo(draw, faces, centre, scale, a * 0.92f * flicker, edge_fade);
    if (intensity > 0.0f) {
        draw_glitch_overlay(draw, centre, span, intensity, bucket, min_x, max_x, min_y, max_y,
                            a * flicker);
    }
    draw->PopClipRect();

    fade_edges(logo_vtx);

    // Faint scanlines every other screen pixel row (at least 2 px apart),
    // each clipped to the glass and split into segments so they fade out
    // towards its edges; heavier while glitching.
    const float spacing = std::max(2.0f, std::round(px_per_photo_px * 1.5f));
    const int line_alpha = static_cast<int>((22.0f + 40.0f * intensity) * a);
    const int lines_vtx = draw->VtxBuffer.Size;
    constexpr int kSegments = 20;
    for (float y = min_y; y < max_y; y += spacing) {
        float x0, x1;
        if (!row_span(y, x0, x1)) continue;
        for (int s = 0; s < kSegments; ++s) {
            const float sa = x0 + (x1 - x0) * static_cast<float>(s) / kSegments;
            const float sb = x0 + (x1 - x0) * static_cast<float>(s + 1) / kSegments;
            draw->AddLine(ImVec2(sa, y), ImVec2(sb, y), IM_COL32(0, 0, 0, line_alpha),
                          std::max(1.0f, spacing * 0.45f));
        }
    }
    fade_edges(lines_vtx);
}
