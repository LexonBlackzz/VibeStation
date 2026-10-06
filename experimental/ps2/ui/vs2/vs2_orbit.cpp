#include "ui/vs2/vs2_orbit.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <vector>

namespace ps2::ui::vs2 {

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kCx = 430.0f;        // ring centre, design units
constexpr float kCy = 380.0f;
constexpr float kRadius = 150.0f;
constexpr float kIncline = 1.05f;    // ring plane tipped towards the viewer
constexpr float kTilt = -0.42f;      // screen-space roll of the whole ring
constexpr float kFocal = 520.0f;     // perspective strength
constexpr float kCycle = 30.0f;
constexpr std::size_t kTrailLength = 26;
constexpr int kDustCount = 70;

struct Stop {
    float offset;
    float r, g, b, a;
};

// Radial gradient sprite, straight (non-premultiplied) alpha.
std::vector<unsigned char> radial(int size, std::initializer_list<Stop> stops) {
    std::vector<unsigned char> pixels(static_cast<std::size_t>(size) * size * 4u);
    const std::vector<Stop> s(stops);
    const float half = size * 0.5f;
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            const float dx = (x + 0.5f - half) / half;
            const float dy = (y + 0.5f - half) / half;
            const float d = std::min(1.0f, std::sqrt(dx * dx + dy * dy));
            std::size_t j = 1;
            while (j + 1 < s.size() && d > s[j].offset) ++j;
            const Stop& a = s[j - 1];
            const Stop& b = s[j];
            const float t = saturate((d - a.offset) / std::max(1e-5f, b.offset - a.offset));
            unsigned char* p = &pixels[(static_cast<std::size_t>(y) * size + x) * 4u];
            p[0] = static_cast<unsigned char>(a.r + (b.r - a.r) * t);
            p[1] = static_cast<unsigned char>(a.g + (b.g - a.g) * t);
            p[2] = static_cast<unsigned char>(a.b + (b.b - a.b) * t);
            p[3] = static_cast<unsigned char>(255.0f * (a.a + (b.a - a.a) * t));
        }
    }
    return pixels;
}

// Deterministic per-orb randomness, so the scattered layout repeats.
float rnd(float n) {
    const float x = std::sin(n * 127.1f + 311.7f) * 43758.5453f;
    return x - std::floor(x);
}

// Rodrigues rotation of (x, y, 0) about the in-plane axis (cos n, sin n, 0).
void tip_off(float x, float y, float n, float th, float out[3]) {
    const float kx = std::cos(n), ky = std::sin(n);
    const float c = std::cos(th), s = std::sin(th);
    const float d = kx * x + ky * y;
    out[0] = x * c + kx * d * (1.0f - c);
    out[1] = y * c + ky * d * (1.0f - c);
    out[2] = (kx * y - ky * x) * s;
}

void set_additive_blend(const ImDrawList*, const ImDrawCmd*) {
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
}

void sprite(ImDrawList* draw, unsigned int texture, const ImVec2& c, float r, ImU32 tint) {
    if (texture == 0 || r <= 0.0f || ((tint >> IM_COL32_A_SHIFT) & 0xFF) == 0) return;
    draw->AddImage(static_cast<ImTextureID>(texture),
                   ImVec2(c.x - r, c.y - r), ImVec2(c.x + r, c.y + r),
                   ImVec2(0, 0), ImVec2(1, 1), tint);
}

ImU32 tint(float r, float g, float b, float alpha) {
    return IM_COL32(static_cast<int>(r), static_cast<int>(g), static_cast<int>(b),
                    static_cast<int>(255.0f * saturate(alpha)));
}

} // namespace

void Orbit::create_textures() {
    if (halo_ != 0) return;
    const auto halo = radial(128, {{0.0f, 150, 200, 255, .75f}, {.12f, 70, 150, 255, .5f},
                                   {.4f, 30, 100, 255, .14f}, {1.0f, 0, 0, 0, 0}});
    const auto core = radial(32, {{0.0f, 255, 255, 255, 1}, {.4f, 255, 255, 255, .95f},
                                  {.7f, 170, 215, 255, .35f}, {1.0f, 0, 0, 0, 0}});
    const auto halo_red = radial(128, {{0.0f, 255, 120, 110, .6f}, {.12f, 255, 70, 60, .35f},
                                       {.4f, 200, 30, 30, .1f}, {1.0f, 0, 0, 0, 0}});
    const auto core_red = radial(32, {{0.0f, 255, 240, 235, 1}, {.35f, 255, 150, 140, .9f},
                                      {.7f, 255, 60, 50, .25f}, {1.0f, 0, 0, 0, 0}});
    // Thin flare streak: white, fading out towards both ends.
    std::vector<unsigned char> streak(64u * 4u * 4u);
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 64; ++x) {
            unsigned char* p = &streak[(static_cast<std::size_t>(y) * 64 + x) * 4u];
            const float u = (x + 0.5f) / 64.0f;
            const float v = 1.0f - std::fabs((y + 0.5f) / 4.0f - 0.5f) * 2.0f;
            p[0] = p[1] = p[2] = 255;
            p[3] = static_cast<unsigned char>(
                255.0f * 0.9f * (1.0f - std::fabs(u * 2.0f - 1.0f)) * v);
        }
    }
    halo_ = create_texture_rgba(128, 128, halo.data(), true);
    core_ = create_texture_rgba(32, 32, core.data(), true);
    halo_red_ = create_texture_rgba(128, 128, halo_red.data(), true);
    core_red_ = create_texture_rgba(32, 32, core_red.data(), true);
    streak_ = create_texture_rgba(64, 4, streak.data(), true);
}

void Orbit::destroy_textures() {
    destroy_texture(halo_);
    destroy_texture(core_);
    destroy_texture(halo_red_);
    destroy_texture(core_red_);
    destroy_texture(streak_);
}

void Orbit::init_orbs() {
    for (std::size_t i = 0; i < orbs_.size(); ++i) {
        Orb& o = orbs_[i];
        const float n = static_cast<float>(i);
        o.phase = n * (2.0f * kPi / static_cast<float>(orbs_.size()));
        o.twinkle_rate = 1.3f + static_cast<float>(i * 29 % 9) / 6.0f;
        o.twinkle_phase = n * 1.9f;
        o.own_radius = kRadius * (.55f + .7f * rnd(n + 1));
        o.own_tilt = (.35f + .85f * rnd(n + 11)) * (rnd(n + 21) > .5f ? 1.0f : -1.0f);
        o.own_node = rnd(n + 31) * 2.0f * kPi;
        o.own_speed = (.18f + .3f * rnd(n + 41)) * (rnd(n + 51) > .5f ? 1.0f : -1.0f);
        o.own_phase = rnd(n + 61) * 2.0f * kPi;
    }
    initialised_ = true;
}

void Orbit::begin_reveal(double now) {
    reveal_t0_ = now;
    hidden_ = false;
    dust_ = 0.0f;
    for (Orb& o : orbs_) o.trail.clear();
}

void Orbit::hide() {
    hidden_ = true;
    dust_ = 0.0f;
}

void Orbit::set_targets(float presence, float reaper, float collapse) {
    presence_target_ = presence;
    reaper_target_ = reaper;
    collapse_target_ = collapse;
}

void Orbit::update(double now, float dt) {
    if (!initialised_) init_orbs();
    now_ = now;
    presence_ = approach(presence_, presence_target_, 5.0f, dt);
    reaper_ = approach(reaper_, reaper_target_, 3.6f, dt);
    collapse_ = approach(collapse_, collapse_target_, 3.0f, dt);
    dust_ = hidden_ ? 0.0f : approach(dust_, presence_target_ > .5f ? 1.0f : .6f, 1.4f, dt);

    const float t = static_cast<float>(now);
    const double cycle_t0 = reveal_t0_ >= 0.0 && reveal_t0_ < now ? reveal_t0_ : 0.0;
    float sc = static_cast<float>(std::fmod(now - cycle_t0, static_cast<double>(kCycle)));
    if (sc < 0.0f) sc += kCycle;

    const float ring = sc < 7.5f ? 1.0f : smoothstep(23.0f, 30.0f, sc);
    merge_ = sc < 7.5f ? smoothstep(3.0f, 5.5f, sc) : 1.0f - smoothstep(7.5f, 15.0f, sc);
    lock_ = std::exp(-std::pow((sc - 1.5f) / .8f, 2.0f));

    const float spin = t * .22f;
    const float shrink = 1.0f - collapse_;
    for (std::size_t i = 0; i < orbs_.size(); ++i) {
        Orb& o = orbs_[i];
        float k = 0.0f;
        if (!hidden_) {
            if (reveal_t0_ < 0.0) {
                k = presence_;
            } else {
                const float local = static_cast<float>(
                    (now - reveal_t0_ - .4 - static_cast<double>(i) * .12) / .9);
                k = ease_out_cubic(local) * presence_;
            }
        }

        const float a = spin + o.phase;
        const float ring_p[3] = {std::cos(a) * kRadius, std::sin(a) * kRadius, 0.0f};
        const float b = o.own_phase + t * o.own_speed;
        float own_p[3];
        tip_off(std::cos(b) * o.own_radius, std::sin(b) * o.own_radius,
                o.own_node + t * .05f, o.own_tilt, own_p);

        const float m = (1.0f - merge_) * k * shrink;
        const float sx = (own_p[0] + (ring_p[0] - own_p[0]) * ring) * m;
        const float sy = (own_p[1] + (ring_p[1] - own_p[1]) * ring) * m;
        const float sz = (own_p[2] + (ring_p[2] - own_p[2]) * ring) * m;
        const float x = sx;
        const float y = sy * std::cos(kIncline) - sz * std::sin(kIncline);
        const float z = sy * std::sin(kIncline) + sz * std::cos(kIncline);

        const float c = std::cos(kTilt), s = std::sin(kTilt);
        const float f = kFocal / (kFocal + z);
        const float px = kCx + (x * c - y * s) * f;
        const float py = kCy + (x * s + y * c) * f;

        o.trail.push_back({px, py, f});
        while (o.trail.size() > kTrailLength) o.trail.pop_front();
        projected_[i] = {&o, px, py, f, z, k};
    }
    std::sort(projected_.begin(), projected_.end(),
              [](const Projected& a, const Projected& b) { return a.z > b.z; });
}

void Orbit::draw(ImDrawList* draw, const Layout& layout) const {
    if (!initialised_ || halo_ == 0 || hidden_) return;
    const float t = static_cast<float>(now_);
    const float red = reaper_;
    const float shrink = 1.0f - collapse_;

    // Dust: faint neutral specks drifting upward over the black.
    if (dust_ > 0.01f) {
        for (int i = 0; i < kDustCount; ++i) {
            const float x = static_cast<float>((i * 997) % 1280);
            const float base_y = static_cast<float>((i * 613) % 800);
            const float speed = 3.0f + static_cast<float>(i % 7) * 1.5f;
            float y = std::fmod(base_y - t * speed, 800.0f);
            if (y < 0.0f) y += 800.0f;
            const float size = .4f + static_cast<float>(i % 5) * .25f;
            const float alpha = dust_ * (.10f + .08f * std::sin(t * 1.3f + i * 2.3f));
            const ImVec2 p = layout.point(x, y);
            const float s = std::max(1.0f, layout.px(size));
            draw->AddRectFilled(p, ImVec2(p.x + s, p.y + s), tint(216, 220, 228, alpha));
        }
    }

    draw->AddCallback(set_additive_blend, nullptr);

    // Grim Reaper: a faint red bloom behind the ring. Otherwise pure black.
    if (red > 0.01f) {
        sprite(draw, halo_red_, layout.point(kCx, kCy), layout.px(520.0f),
               tint(255, 255, 255, .22f * red * dust_));
    }

    // One big orb while the ring is merged.
    if (merge_ > .02f && shrink > .5f) {
        const float pulse = 1.0f + .08f * std::sin(t * 3.2f);
        const float big = merge_ * merge_ * presence_;
        const ImVec2 c = layout.point(kCx, kCy);
        const float bh = layout.px(150.0f * pulse * (.4f + .6f * merge_));
        const float bc = layout.px(34.0f * pulse * merge_);
        sprite(draw, halo_, c, bh, tint(255, 255, 255, .85f * big * (1 - red)));
        sprite(draw, halo_red_, c, bh, tint(255, 255, 255, .85f * big * red));
        sprite(draw, core_, c, bc, tint(255, 255, 255, big * (1 - red)));
        sprite(draw, core_red_, c, bc, tint(255, 255, 255, big * red));
    }

    // Faint ring flash as the formation locks in.
    if (lock_ > .01f && shrink > .5f) {
        draw->AddEllipse(layout.point(kCx, kCy),
                         layout.size(kRadius * shrink, kRadius * std::cos(kIncline) * shrink),
                         red > .5f ? tint(255, 154, 144, .22f * lock_ * presence_)
                                   : tint(127, 188, 255, .22f * lock_ * presence_),
                         kTilt, 96, std::max(1.0f, layout.px(1.2f)));
    }

    const auto flare = [&](const ImVec2& c, float len, float alpha, float angle) {
        if (alpha <= 0.0f || streak_ == 0) return;
        const ImU32 col = red > .5f ? tint(255, 200, 190, alpha) : tint(190, 225, 255, alpha);
        const float half_w = std::max(0.6f, layout.px(0.6f));
        for (int arm = 0; arm < 2; ++arm) {
            const float ang = angle + arm * kPi * 0.5f;
            const ImVec2 d(std::cos(ang) * len, std::sin(ang) * len);
            const ImVec2 n(-std::sin(ang) * half_w, std::cos(ang) * half_w);
            draw->AddImageQuad(static_cast<ImTextureID>(streak_),
                               ImVec2(c.x - d.x - n.x, c.y - d.y - n.y),
                               ImVec2(c.x + d.x - n.x, c.y + d.y - n.y),
                               ImVec2(c.x + d.x + n.x, c.y + d.y + n.y),
                               ImVec2(c.x - d.x + n.x, c.y - d.y + n.y),
                               ImVec2(0, 0), ImVec2(1, 0), ImVec2(1, 1), ImVec2(0, 1), col);
        }
    };

    if (merge_ > .02f && shrink > .5f) {
        const float pulse = 1.0f + .08f * std::sin(t * 3.2f);
        const float big = merge_ * merge_ * presence_;
        flare(layout.point(kCx, kCy), layout.px(70.0f * pulse * merge_), .55f * big, .25f + t * .05f);
    }

    for (const Projected& p : projected_) {
        if (p.orb == nullptr || p.k <= 0.001f) continue;
        const float depth = saturate((p.f - .8f) / .4f); // 0 back, 1 front
        const float bright = std::min(1.3f, (.45f + .55f * depth) * (1.0f + .35f * lock_)) * p.k;

        // Comet tail: tapered segments along recent positions.
        const auto& tr = p.orb->trail;
        for (std::size_t j = 1; j < tr.size(); ++j) {
            const float f = static_cast<float>(j) / static_cast<float>(tr.size());
            const float alpha = f * f * .5f * bright;
            const ImU32 col = red > .5f ? tint(255, 138, 128, alpha) : tint(111, 180, 255, alpha);
            draw->AddLine(layout.point(tr[j - 1][0], tr[j - 1][1]),
                          layout.point(tr[j][0], tr[j][1]), col,
                          std::max(0.5f, layout.px((.6f + 3.2f * f) * tr[j][2])));
        }

        const float tw = .75f + .25f * std::sin(t * p.orb->twinkle_rate * 3.0f + p.orb->twinkle_phase);
        const ImVec2 c = layout.point(p.x, p.y);
        const float hs = layout.px((46.0f + 30.0f * depth) * p.f);
        const float cs = layout.px((9.0f + 5.0f * depth) * p.f);
        sprite(draw, halo_, c, hs, tint(255, 255, 255, bright * tw * (1 - red)));
        sprite(draw, halo_red_, c, hs, tint(255, 255, 255, bright * tw * red));
        sprite(draw, core_, c, cs, tint(255, 255, 255, bright * (1 - red)));
        sprite(draw, core_red_, c, cs, tint(255, 255, 255, bright * red));
        if (depth > .35f) {
            flare(c, layout.px((14.0f + 22.0f * depth) * tw * p.f),
                  (depth - .35f) * .9f * bright * tw, .25f + t * .05f);
        }
    }

    draw->AddCallback(ImDrawCallback_ResetRenderState, nullptr);
}

} // namespace ps2::ui::vs2
