#pragma once

#include "ui/vs2/vs2_shared.h"

#include <array>
#include <deque>

namespace ps2::ui::vs2 {

// The menu's orbiting lights. One 30 s cycle, timed from the reveal:
//   0-3 s     ring formation, locks in with a flash
//   3-5.5 s   the ring pulls into one big orb
//   5.5-7.5 s the big orb holds and pulses
//   7.5-15 s  orbs drift apart onto their own tilted orbits
//   15-23 s   they roam the sphere
//   23-30 s   they gather back into the ring
class Orbit {
public:
    void create_textures();
    void destroy_textures();

    // Restarts the cycle and flies the orbs out of the centre.
    void begin_reveal(double now);
    // Hides everything until begin_reveal() (used while the intro plays).
    void hide();

    // presence: 1 on the main menu, lower behind sub-screens.
    // reaper: 1 tints everything red. collapse: 1 pulls the ring into the centre.
    void set_targets(float presence, float reaper, float collapse);
    void update(double now, float dt);
    void draw(ImDrawList* draw, const Layout& layout) const;

private:
    struct Orb {
        float phase = 0.0f;
        float twinkle_rate = 0.0f;
        float twinkle_phase = 0.0f;
        float own_radius = 0.0f;
        float own_tilt = 0.0f;
        float own_node = 0.0f;
        float own_speed = 0.0f;
        float own_phase = 0.0f;
        std::deque<std::array<float, 3>> trail; // design x, y, perspective
    };
    struct Projected {
        const Orb* orb = nullptr;
        float x = 0.0f, y = 0.0f, f = 1.0f, z = 0.0f, k = 0.0f;
    };

    void init_orbs();

    std::array<Orb, 8> orbs_{};
    std::array<Projected, 8> projected_{};
    bool initialised_ = false;
    double now_ = 0.0;
    double reveal_t0_ = -1.0;
    bool hidden_ = false;

    float presence_ = 1.0f, presence_target_ = 1.0f;
    float reaper_ = 0.0f, reaper_target_ = 0.0f;
    float collapse_ = 0.0f, collapse_target_ = 0.0f;
    float dust_ = 1.0f;
    float merge_ = 0.0f;
    float lock_ = 0.0f;

    unsigned int halo_ = 0, core_ = 0, halo_red_ = 0, core_red_ = 0, streak_ = 0;
};

} // namespace ps2::ui::vs2
