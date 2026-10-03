#pragma once

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace ps2::ui::vs2 {

// Same design space as the PS1 definitive UI and vs2start.mp4.
inline constexpr float kDesignWidth = 1280.0f;
inline constexpr float kDesignHeight = 800.0f;
inline constexpr const char* kVersionLabel = "v0.6.0-ps2lab";

// ---------------------------------------------------------------- layout
// Maps 1280x800 design units onto the window, letterboxed and centred
// (identical to definitive_ui::Layout).
struct Layout {
    ImVec2 origin{};
    float scale = 1.0f;

    [[nodiscard]] ImVec2 point(float x, float y) const {
        return ImVec2(origin.x + x * scale, origin.y + y * scale);
    }
    [[nodiscard]] ImVec2 size(float x, float y) const {
        return ImVec2(x * scale, y * scale);
    }
    [[nodiscard]] float px(float value) const { return value * scale; }
};

inline Layout make_layout(const ImVec2& pos, const ImVec2& size) {
    Layout layout{};
    layout.scale = std::max(
        0.45f, std::min(size.x / kDesignWidth, size.y / kDesignHeight));
    layout.origin = ImVec2(
        pos.x + (size.x - kDesignWidth * layout.scale) * 0.5f,
        pos.y + (size.y - kDesignHeight * layout.scale) * 0.5f);
    return layout;
}

// ---------------------------------------------------------------- colour
inline ImU32 rgba(int r, int g, int b, int a = 255) {
    return IM_COL32(r, g, b, a);
}
inline ImU32 with_alpha(ImU32 color, float alpha) {
    const int a = static_cast<int>(
        ((color >> IM_COL32_A_SHIFT) & 0xFF) * std::clamp(alpha, 0.0f, 1.0f));
    return (color & ~IM_COL32_A_MASK) | (static_cast<ImU32>(a) << IM_COL32_A_SHIFT);
}

namespace color {
inline constexpr ImU32 kSelect = IM_COL32(56, 182, 255, 255);   // selected item
inline constexpr ImU32 kIdle = IM_COL32(140, 149, 163, 255);    // other items
inline constexpr ImU32 kSub = IM_COL32(125, 138, 160, 255);     // descriptions
inline constexpr ImU32 kText = IM_COL32(215, 223, 234, 255);
inline constexpr ImU32 kDim = IM_COL32(102, 115, 138, 255);
inline constexpr ImU32 kReaper = IM_COL32(255, 74, 74, 255);
} // namespace color

// ---------------------------------------------------------------- easing
inline float saturate(float v) { return std::clamp(v, 0.0f, 1.0f); }
inline float ease_out_cubic(float t) {
    t = saturate(t);
    return 1.0f - (1.0f - t) * (1.0f - t) * (1.0f - t);
}
inline float smoothstep(float a, float b, float x) {
    const float t = saturate((x - a) / (b - a));
    return t * t * (3.0f - 2.0f * t);
}
// Frame-rate independent approach of `value` to `target`.
inline float approach(float value, float target, float rate, float dt) {
    return value + (target - value) * (1.0f - std::exp(-rate * dt));
}

// ---------------------------------------------------------------- fonts
enum class FontRole {
    Light,   // menu labels, titles
    Regular, // body text
    Mono,    // corner block, data values
    Bold,    // boot animation title
};
void load_fonts();
ImFont* font(FontRole role, float pixel_size);
// Draws text with a font chosen for the scaled size.
void text(ImDrawList* draw, FontRole role, float size, const ImVec2& pos,
          ImU32 color, const char* str);
ImVec2 text_size(FontRole role, float size, const char* str);

// ---------------------------------------------------------------- audio
void preload_sounds();
void release_sounds();
void play_highlight_sound();
void play_open_sound();
void play_close_sound();
void play_select_sound();
void set_sounds_enabled(bool enabled);
// The boot animation's sound (vs2-boot.wav) on its own device, so menu
// sounds never cut it off.
void play_boot_sound();
// Coming back to the menu without the boot animation (vs2-backtomenu.wav),
// on the same device; silent when menu sounds are off.
void play_back_to_menu_sound();
// Stops either of them.
void stop_boot_sound();
// Frontend mixer (vs2_mixer.cpp): menu sounds and the menu ambience
// (vs2-ambientbg.wav loop + vs2-certainstatic.wav waves) share one device
// and one reverb. ambience_set_active fades the ambience in or out;
// ambience_update schedules the waves.
inline constexpr int kMixerRate = 48000;
// Loads a WAV as interleaved stereo float at kMixerRate.
bool load_wav_stereo(const char* name, std::vector<float>& out);
// Plays a loaded clip (kept alive by the caller) as a menu sound.
void mixer_play_ui(const std::vector<float>* clip);
void ambience_set_active(bool active);
void ambience_update(float dt);
void mixer_open();
void mixer_release();

// ---------------------------------------------------------------- assets
// Finds resources/vs2/<name> next to the executable, in the working
// directory, or one level up. Empty when missing.
std::filesystem::path find_asset(const char* name);

// RGBA8 texture helpers (OpenGL, UI thread only).
unsigned int create_texture_rgba(int width, int height, const void* pixels, bool linear);
void destroy_texture(unsigned int& texture);
// An image from resources/vs2 as a texture (0 if missing). A picture with an
// opaque white background (dark artwork on white) is turned into light
// artwork on transparency so it shows on the dark UI.
unsigned int load_image_texture(const char* name, int& width, int& height);

} // namespace ps2::ui::vs2
