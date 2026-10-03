#pragma once

#include <imgui.h>

#include <array>
#include <cstdint>

namespace ps2::ui::vs2 {

// Ambient light around the game picture, sampled from the picture's own
// edges: the same effect as VibeStation 1's gameplay screen
// (src/ui/definitive/definitive_gameplay_ambient.cpp), with its own state so
// the two apps never share colours.
class EdgeLight {
public:
    // A new frame, RGBA8 with red in the low byte. UI thread only.
    void update(const std::uint32_t* rgba, int width, int height);
    // Near-black wall over `area` with light spilling from the game edges.
    void draw(ImDrawList* draw, const ImVec2& area_pos, const ImVec2& area_size,
              const ImVec2& game_pos, const ImVec2& game_size, float alpha) const;
    // Forget the last colours (a new game should not inherit them).
    void reset() { initialized_ = false; }

private:
    static constexpr int kVertical = 12;
    static constexpr int kHorizontal = 16;
    using Vertical = std::array<ImVec4, kVertical>;
    using Horizontal = std::array<ImVec4, kHorizontal>;

    Vertical left_{};
    Vertical right_{};
    Horizontal top_{};
    Horizontal bottom_{};
    bool initialized_ = false;
};

} // namespace ps2::ui::vs2
