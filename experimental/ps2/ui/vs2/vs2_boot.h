#pragma once

#include "ui/vs2/vs2_shared.h"

namespace ps2::ui::vs2 {

// The VibeStation 2 boot animation ("ignition"), drawn in real time:
//   0-0.4 s    a white spark lights up in a dark, swirling nebula
//   0.3-1.0 s  four coloured lights (the PlayStation button colours) burst
//              out of it, each trailing a comet tail and a swarm of sparks
//   1.0-2.05 s  they orbit on tilted 3D paths while a galaxy of specks turns;
//               "VibeStation 2" fades in and out
//   2.05-3.15 s the lights spiral in, easing into the centre
//   3.15 s      they collide: a soft flash and a burst of light rays
//   3.15-4.45 s the rays glide outwards
//   3.6-4.45 s  the camera, drifting slowly into the nebula until now, rushes
//               in (ease-in, zoom blur) and the nebula blows out as
//               everything fades to black from 4.15 s
// The menu reveal (orbs flying out of the centre) follows at 4.5 s.
class BootAnimation {
public:
    // Ends just before the menu reveal at 4.5 s.
    static constexpr float kDuration = 4.45f;

    void create_textures();
    void destroy_textures();
    void draw(ImDrawList* draw, const Layout& layout, float t) const;

private:
    unsigned int fog_a_ = 0;
    unsigned int fog_b_ = 0;
    unsigned int glow_ = 0;
};

} // namespace ps2::ui::vs2
