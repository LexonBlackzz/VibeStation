#include "ui/vs2/vs2_boot.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <array>
#include <cstring>
#include <vector>

namespace ps2::ui::vs2 {

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kCx = 640.0f;          // scene centre, design units
constexpr float kCy = 380.0f;
constexpr float kFocal = 700.0f;
constexpr float kViewDistance = 8.0f;
constexpr float kCollide = 3.15f;      // the lights meet here

// ------------------------------------------------------------------ math

struct Vec3 {
    float x = 0, y = 0, z = 0;
};

// Rotate about Y, then X.
Vec3 rotate(Vec3 p, float yaw, float pitch) {
    const float cy = std::cos(yaw), sy = std::sin(yaw);
    p = {p.x * cy + p.z * sy, p.y, -p.x * sy + p.z * cy};
    const float cx = std::cos(pitch), sx = std::sin(pitch);
    return {p.x, p.y * cx - p.z * sx, p.y * sx + p.z * cx};
}

float hash(float n) {
    const float x = std::sin(n * 12.9898f + 78.233f) * 43758.5453f;
    return x - std::floor(x);
}

// Mostly top-down view of the XZ plane, tilted a little and turning slowly.
// Returns design coordinates and the perspective scale.
ImVec2 project(Vec3 p, float t, float& scale) {
    const Vec3 q = rotate(p, 0.15f * t, 0.5f);
    scale = kFocal / std::max(0.5f, kViewDistance - q.y);
    return ImVec2(kCx + q.x * scale, kCy + q.z * scale);
}

// 0 -> 1 over the 1.1 s before the collision, easing in and out, so the
// spiral draws in smoothly instead of whipping round at the end.
float converge_at(float t) { return smoothstep(kCollide - 1.1f, kCollide, t); }

// ------------------------------------------------------------------ the four lights

struct Spark {
    int r, g, b;
    float radius, incline, node, speed, phase;
};

// Green, red, blue and pink, as on the controller's face buttons.
constexpr std::array<Spark, 4> kSparks = {{
    {80, 255, 150, 2.6f, 0.35f, 0.0f, 1.6f, 0.0f},
    {255, 70, 85, 2.2f, -0.95f, 0.8f, -1.9f, 1.7f},
    {90, 155, 255, 2.9f, 1.2f, 1.9f, 1.3f, 3.3f},
    {255, 115, 225, 2.4f, -0.45f, 2.7f, -1.5f, 4.6f},
}};

// World position of light i at time t: bursts out of the centre, orbits on
// its own tilted ring, then spirals in to the collision.
Vec3 spark_position(int i, float t) {
    const Spark& s = kSparks[static_cast<std::size_t>(i)];
    const float emerge = ease_out_cubic((t - 0.3f - i * 0.08f) / 0.7f);
    const float converge = converge_at(t);
    const float r = s.radius * emerge * (1.0f - converge);
    // Spinning up as the ring tightens, like a skater pulling in.
    const float a = s.phase + s.speed * t + (s.speed > 0 ? 1.0f : -1.0f) * 4.0f * converge;
    const Vec3 flat{std::cos(a) * r, 0.18f * std::sin(t * 2.3f + s.phase) * r, std::sin(a) * r};
    return rotate(flat, s.node, s.incline);
}

// ------------------------------------------------------------------ textures

float value_noise(float x, float y, int seed) {
    const auto lattice = [seed](int ix, int iy) {
        return hash(static_cast<float>(ix * 57 + iy * 131 + seed * 977));
    };
    const int ix = static_cast<int>(std::floor(x)), iy = static_cast<int>(std::floor(y));
    float fx = x - ix, fy = y - iy;
    fx = fx * fx * (3 - 2 * fx);
    fy = fy * fy * (3 - 2 * fy);
    const float a = lattice(ix, iy), b = lattice(ix + 1, iy);
    const float c = lattice(ix, iy + 1), d = lattice(ix + 1, iy + 1);
    return (a + (b - a) * fx) + ((c + (d - c) * fx) - (a + (b - a) * fx)) * fy;
}

// Nebula: fractal noise, densest towards the middle, deep blue to violet.
std::vector<unsigned char> fog_pixels(int size, int seed, bool violet) {
    std::vector<unsigned char> px(static_cast<std::size_t>(size) * size * 4u);
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            const float u = static_cast<float>(x) / size, v = static_cast<float>(y) / size;
            float n = 0, amp = 0.5f, freq = 4.0f;
            for (int o = 0; o < 5; ++o) {
                n += amp * value_noise(u * freq, v * freq, seed + o);
                amp *= 0.5f;
                freq *= 2.0f;
            }
            const float dx = u - 0.5f, dy = v - 0.5f;
            const float r = std::sqrt(dx * dx + dy * dy) * 2.0f;
            const float density = saturate((n - 0.3f) * 2.0f) * (1.0f - smoothstep(0.2f, 1.0f, r));
            unsigned char* p = &px[(static_cast<std::size_t>(y) * size + x) * 4u];
            p[0] = static_cast<unsigned char>(violet ? 70 + 60 * n : 14 + 30 * n);
            p[1] = static_cast<unsigned char>(violet ? 30 + 30 * n : 40 + 55 * n);
            p[2] = static_cast<unsigned char>(170 + 85 * n);
            p[3] = static_cast<unsigned char>(255 * density);
        }
    }
    return px;
}

std::vector<unsigned char> glow_pixels(int size) {
    std::vector<unsigned char> px(static_cast<std::size_t>(size) * size * 4u);
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            const float dx = (x + .5f) / size - .5f, dy = (y + .5f) / size - .5f;
            const float r = std::sqrt(dx * dx + dy * dy) * 2.0f;
            unsigned char* p = &px[(static_cast<std::size_t>(y) * size + x) * 4u];
            p[0] = p[1] = p[2] = 255;
            p[3] = static_cast<unsigned char>(255 * std::pow(saturate(1.0f - r), 2.4f));
        }
    }
    return px;
}

// ------------------------------------------------------------------ drawing helpers

void set_additive_blend(const ImDrawList*, const ImDrawCmd*) {
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
}

ImU32 col8(float r, float g, float b, float a) {
    return IM_COL32(static_cast<int>(std::clamp(r, 0.0f, 255.0f)), static_cast<int>(std::clamp(g, 0.0f, 255.0f)),
                    static_cast<int>(std::clamp(b, 0.0f, 255.0f)), static_cast<int>(255 * saturate(a)));
}

void sprite(ImDrawList* draw, unsigned int tex, ImVec2 c, float radius, float angle, ImU32 col) {
    if (radius <= 0.0f || ((col >> IM_COL32_A_SHIFT) & 0xFF) == 0) return;
    const float cs = std::cos(angle) * radius, sn = std::sin(angle) * radius;
    draw->AddImageQuad(static_cast<ImTextureID>(tex),
                       ImVec2(c.x - cs + sn, c.y - sn - cs), ImVec2(c.x + cs + sn, c.y + sn - cs),
                       ImVec2(c.x + cs - sn, c.y + sn + cs), ImVec2(c.x - cs - sn, c.y - sn + cs),
                       ImVec2(0, 0), ImVec2(1, 0), ImVec2(1, 1), ImVec2(0, 1), col);
}


} // namespace

void BootAnimation::create_textures() {
    if (fog_a_ != 0) return;
    const auto a = fog_pixels(256, 3, false);
    const auto b = fog_pixels(256, 11, true);
    const auto g = glow_pixels(64);
    fog_a_ = create_texture_rgba(256, 256, a.data(), true);
    fog_b_ = create_texture_rgba(256, 256, b.data(), true);
    glow_ = create_texture_rgba(64, 64, g.data(), true);
}

void BootAnimation::destroy_textures() {
    destroy_texture(fog_a_);
    destroy_texture(fog_b_);
    destroy_texture(glow_);
}

void BootAnimation::draw(ImDrawList* draw, const Layout& layout, float t) const {
    const float fade = smoothstep(0.0f, 0.15f, t) * (1.0f - smoothstep(4.15f, kDuration, t));
    if (fade <= 0.001f || fog_a_ == 0) return;

    const float converge = converge_at(t);
    const float since_collide = t - kCollide;
    const float flash = since_collide >= 0.0f ? std::exp(-since_collide * 2.6f) : 0.0f;
    const ImVec2 centre = layout.point(kCx, kCy);

    draw->PushClipRect(layout.point(0, 0), layout.point(kDesignWidth, kDesignHeight), true);
    draw->AddCallback(set_additive_blend, nullptr);

    // Nebula: two layers turning against each other while the camera flies
    // into them, slowly at first and rushing in after the collision. Zoom
    // blur: fainter copies trail inwards, spread by how fast the zoom is.
    const auto zoom_at = [](float time) {
        // A slow, steady drift, then an ease-in rush into the cloud over the
        // last 0.85 s, fastest in the final frames as everything fades.
        const float rush = saturate((time - 3.6f) / (kDuration - 3.6f));
        return std::exp(0.16f * time) * (1.0f + 6.0f * rush * rush * rush);
    };
    const float zoom = zoom_at(t);
    // Extra spin for the nebula and specks, on the same ease-in as the final
    // rush: nothing until 3.6 s, fastest in the last frames.
    const float rush = saturate((t - 3.6f) / (kDuration - 3.6f));
    const float rush_spin = rush * rush * rush;
    {
        const float zoom_rate = (zoom_at(t + 0.02f) - zoom) / (0.02f * zoom);
        const float spread = std::clamp(0.015f + 0.07f * zoom_rate, 0.0f, 0.4f);
        const float swirl = 0.08f * t + 0.9f * rush_spin;
        const float intro = smoothstep(0.0f, 1.0f, t);
        const float blowout = saturate((t - 3.6f) / (kDuration - 3.6f));
        const float glow = 0.5f + 0.35f * converge + 0.25f * flash + 0.5f * blowout * blowout;
        constexpr int kCopies = 7;
        for (int i = kCopies - 1; i >= 0; --i) {
            const float k = static_cast<float>(i) / kCopies;
            const float radius = layout.px(520.0f * zoom * (1.0f - spread * k));
            const float weight = (1.0f - k) * 2.0f / (kCopies + 1);
            sprite(draw, fog_a_, centre, radius, swirl,
                   col8(255, 255, 255, 0.55f * glow * intro * weight * fade));
            sprite(draw, fog_b_, centre, radius * 0.75f, -swirl * 1.3f - 0.6f,
                   col8(255, 255, 255, 0.32f * glow * intro * weight * fade));
        }
    }

    // A galaxy of specks: inner ones turn faster, the vortex drags them in.
    for (int j = 0; j < 300; ++j) {
        const float n = static_cast<float>(j);
        const float r0 = 50.0f + 640.0f * std::sqrt(hash(n + 0.5f));
        // Nearer than the nebula's far side, so they spread out a little
        // slower than it as the camera flies in.
        const float r = r0 * (1.0f - 0.45f * converge) * std::sqrt(zoom);
        const float a = hash(n + 1.5f) * 2.0f * kPi + (40.0f / r0) * (t + 1.5f * rush_spin);
        const float twinkle = 0.55f + 0.45f * std::sin(t * (2.0f + 3.0f * hash(n + 2.5f)) + n);
        const float alpha = (0.12f + 0.35f * hash(n + 3.5f)) * twinkle * smoothstep(0.1f, 0.8f, t) *
                            (1.0f - smoothstep(kCollide, kCollide + 1.1f, t)) * fade;
        const ImVec2 p = layout.point(kCx + std::cos(a) * r, kCy + std::sin(a) * r * 0.82f);
        const float s = std::max(1.0f, layout.px(0.8f + 1.4f * hash(n + 4.5f)));
        draw->AddRectFilled(ImVec2(p.x - s * .5f, p.y - s * .5f), ImVec2(p.x + s * .5f, p.y + s * .5f),
                            col8(200, 215, 255, alpha));
    }

    // The opening spark, and the heat building before the collision.
    {
        const float spark = smoothstep(0.05f, 0.35f, t) * (1.0f - smoothstep(0.5f, 1.1f, t));
        const float heat = std::pow(converge, 3.0f);
        const float core = spark + heat;
        if (core > 0.01f) {
            sprite(draw, glow_, centre, layout.px(40.0f + 90.0f * core), 0, col8(170, 200, 255, 0.7f * core * fade));
            sprite(draw, glow_, centre, layout.px(10.0f + 26.0f * core), 0, col8(255, 255, 255, core * fade));
        }
    }


    // The four lights: comet tails, spark swarms, glowing heads.
    if (t < kCollide) {
        for (int i = 0; i < static_cast<int>(kSparks.size()); ++i) {
            const Spark& s = kSparks[static_cast<std::size_t>(i)];
            const float visible = smoothstep(0.3f + i * 0.08f, 0.5f + i * 0.08f, t);
            if (visible <= 0.0f) continue;

            // Tail: the same path a moment ago, tapering off. Sampled finer as
            // the lights speed up, so fast arcs stay curved.
            const float step = 0.016f / (1.0f + 1.5f * converge);
            constexpr int kTail = 24;
            ImVec2 prev;
            for (int k = kTail; k >= 0; --k) {
                float scale = 0;
                const ImVec2 p = project(spark_position(i, t - k * step), t, scale);
                const ImVec2 sp = layout.point(p.x, p.y);
                if (k < kTail) {
                    const float f = 1.0f - static_cast<float>(k) / kTail;
                    draw->AddLine(prev, sp, col8(s.r, s.g, s.b, f * f * 0.75f * visible * fade),
                                  std::max(0.8f, layout.px((0.8f + 5.5f * f) * scale / 90.0f)));
                }
                prev = sp;
            }
            // Swarm: sparks trailing behind, each wobbling around the path.
            for (int k = 0; k < 9; ++k) {
                const float lag = 0.05f + 0.045f * k;
                float scale = 0;
                ImVec2 p = project(spark_position(i, t - lag), t, scale);
                const float wobble = 9.0f + 5.0f * k;
                p.x += std::cos(t * (5.0f + k) + k * 2.1f + i) * wobble * (scale / 90.0f);
                p.y += std::sin(t * (4.0f + k) + k * 1.3f + i) * wobble * (scale / 90.0f);
                const float a = (1.0f - k / 9.0f) * 0.75f * visible * fade;
                sprite(draw, glow_, layout.point(p.x, p.y), layout.px(5.0f * scale / 90.0f), 0, col8(s.r, s.g, s.b, a));
            }
            // Head.
            float scale = 0;
            const ImVec2 head = project(spark_position(i, t), t, scale);
            const ImVec2 hp = layout.point(head.x, head.y);
            const float pulse = 1.0f + 0.12f * std::sin(t * 9.0f + i);
            sprite(draw, glow_, hp, layout.px(46.0f * pulse * scale / 90.0f), 0, col8(s.r, s.g, s.b, 0.95f * visible * fade));
            sprite(draw, glow_, hp, layout.px(20.0f * scale / 90.0f), 0, col8(s.r, s.g, s.b, 0.9f * visible * fade));
            sprite(draw, glow_, hp, layout.px(11.0f * scale / 90.0f), 0, col8(255, 255, 255, visible * fade));
        }
    }

    // The collision: flash and a burst of rays that streak outwards.
    if (since_collide >= 0.0f) {
        sprite(draw, glow_, centre, layout.px(110.0f + 380.0f * (1.0f - flash)), 0,
               col8(160, 195, 255, 0.5f * flash * fade));
        sprite(draw, glow_, centre, layout.px(40.0f + 90.0f * flash), 0, col8(255, 255, 255, 0.75f * flash * fade));

        for (int j = 0; j < 56; ++j) {
            const float n = static_cast<float>(j) + 100.0f;
            const float a = hash(n) * 2.0f * kPi;
            const float speed = 260.0f + 520.0f * hash(n + 1.0f);
            // Eases out: the rays glide to a stop instead of accelerating.
            const float travel = speed * (1.0f - std::exp(-1.8f * since_collide)) / 1.8f * 1.6f;
            const float length = 20.0f + speed * 0.12f * std::exp(-1.2f * since_collide);
            const float r1 = 20.0f + travel, r0 = std::max(0.0f, r1 - length);
            const std::array<int, 3> tint = j % 4 == 0 ? std::array<int, 3>{kSparks[j / 4 % 4].r, kSparks[j / 4 % 4].g, kSparks[j / 4 % 4].b}
                                                       : std::array<int, 3>{220, 230, 255};
            const float alpha = (1.0f - smoothstep(0.25f, 1.25f, since_collide)) * (0.22f + 0.33f * hash(n + 2.0f)) * fade;
            draw->AddLine(layout.point(kCx + std::cos(a) * r0, kCy + std::sin(a) * r0),
                          layout.point(kCx + std::cos(a) * r1, kCy + std::sin(a) * r1),
                          col8(tint[0], tint[1], tint[2], alpha), std::max(1.0f, layout.px(1.0f + 1.5f * hash(n + 3.0f))));
        }
    }

    draw->AddCallback(ImDrawCallback_ResetRenderState, nullptr);

    // Title, in the menu's light face: fades in, holds, fades out.
    const float title = smoothstep(1.0f, 1.4f, t) * (1.0f - smoothstep(2.55f, 2.9f, t)) * fade;
    if (title > 0.01f) {
        const char* label = "VibeStation 2";
        const float size = layout.px(62);
        const ImVec2 ts = text_size(FontRole::Light, size, label);
        const ImVec2 p(centre.x - ts.x * 0.5f, layout.point(0, 404).y - ts.y * 0.5f);
        text(draw, FontRole::Light, size, ImVec2(p.x + layout.px(2), p.y + layout.px(3)),
             col8(0, 0, 20, 0.5f * title), label);
        text(draw, FontRole::Light, size, p, col8(236, 241, 250, title), label);
    }

    draw->PopClipRect();
}

} // namespace ps2::ui::vs2
