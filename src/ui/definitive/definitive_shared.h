#pragma once

#include "ui/theme_settings.h"
#include "core/types.h"

#include <imgui.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>

namespace definitive_ui {

inline constexpr float kDesignWidth = 1280.0f;
inline constexpr float kDesignHeight = 800.0f;
// Launcher startup, one clock from the first black frame (seconds). The intro
// owns the screen until the glide, when its title moves into the launcher's
// title spot while the menu arrives underneath (see definitive_home.cpp).
inline constexpr float kIntroGlideStart = 3.45f;
inline constexpr float kIntroGlideEnd = 4.45f;
inline constexpr float kLauncherInteractiveAt = 4.76f;
inline constexpr float kLauncherSequenceEnd = 6.45f;

// Where the intro's title and colour lights land in the launcher.
struct IntroTargets {
    ImVec2 brand_pos{};
    float brand_size = 54.0f;
    ImVec2 bar_centers[4]{};
    float ui_scale = 1.0f;
};

ImFont* font_for_size(float pixel_size);

void preload_intro_assets();
void release_intro_assets();
// Draws on the foreground list. Before kIntroGlideStart it covers the window
// in black; afterwards only the departing icon, title, lights and grain.
void draw_intro_presentation(
    const ImVec2& pos, const ImVec2& size, float elapsed,
    const IntroTargets& targets);
// Opacity of launcher colour bar `index`, which fades in as its light lands.
float intro_color_bar_alpha(int index, float elapsed);

// Non-affiliation notice on black before the startup sequence, with "Don't
// show this disclaimer again" (`remember`) and "I understand". Draws at
// `alpha` (for the fades) and returns true on the frame it is confirmed.
// Uses ImGui items, so call it inside the launcher window.
bool draw_startup_disclaimer(const ImVec2& pos, const ImVec2& size, float alpha,
                             bool& remember);

void preload_audio_assets();
void release_audio_assets();

void preload_background_assets();
void release_background_assets();
// zoom > 1 crops into the photo; blur crossfades towards the blurred copy.
void draw_launcher_background(
    ImDrawList* draw,
    const ImVec2& pos,
    const ImVec2& size,
    float opacity = 1.0f,
    float zoom = 1.0f,
    float blur = 0.0f);
// Where photo pixel (photo_x, photo_y) of the launcher background lands in
// the window at this zoom. False until the photo has loaded.
bool launcher_photo_point(const ImVec2& pos, const ImVec2& size, float zoom,
                          float photo_x, float photo_y, ImVec2& out);
// The animated logo on the photo's CRT screen (definitive_tv.cpp).
void draw_launcher_tv(ImDrawList* draw, const ImVec2& pos, const ImVec2& size,
                      float opacity, float zoom, float time, bool interactive);
// Stops the TV glitch's ambience (fade: over 0.4 s).
void stop_tv_glitch_sound(bool fade);
void draw_launcher_readability_shade(
    ImDrawList* draw,
    const ImVec2& pos,
    const ImVec2& size);
void draw_settings_background(
    ImDrawList* draw,
    const ImVec2& pos,
    const ImVec2& size);

void update_gameplay_ambient(
    const std::vector<u32>& rgba,
    int width,
    int height);
void draw_gameplay_ambient(
    ImDrawList* draw,
    const ImVec2& area_pos,
    const ImVec2& area_size,
    const ImVec2& game_pos,
    const ImVec2& game_size,
    float bottom_overscan_v = 0.0f);
void release_gameplay_ambient_assets();

void play_cursor_sound();
void play_open_sound();
void play_close_sound();
void play_startup_sound();
void stop_startup_sound();
// Fades the startup sound to silence over fade_ms (used when the intro is skipped).
void fade_out_startup_sound(float fade_ms);
// Counts the startup sound as played without playing it.
void skip_startup_sound();

struct Layout {
    ImVec2 origin{};
    float scale = 1.0f;

    ImVec2 point(float x, float y) const {
        return ImVec2(
            origin.x + x * scale,
            origin.y + y * scale);
    }

    ImVec2 size(float x, float y) const {
        return ImVec2(x * scale, y * scale);
    }

    float px(float value) const {
        return value * scale;
    }
};

inline Layout make_layout(
    const ImVec2& window_pos,
    const ImVec2& window_size) {
    Layout layout{};
    layout.scale = std::max(
        0.55f,
        std::min(
            window_size.x / kDesignWidth,
            window_size.y / kDesignHeight));

    const ImVec2 design_size(
        kDesignWidth * layout.scale,
        kDesignHeight * layout.scale);

    layout.origin = ImVec2(
        window_pos.x +
            (window_size.x - design_size.x) * 0.5f,
        window_pos.y +
            (window_size.y - design_size.y) * 0.5f);
    return layout;
}

inline ImU32 rgba(
    int r, int g, int b, int a = 255) {
    return IM_COL32(r, g, b, a);
}

inline bool theme_color_differs(
    const ImVec4& a,
    const ImVec4& b) {
    constexpr float kEpsilon = 0.0025f;
    return std::fabs(a.x - b.x) > kEpsilon ||
        std::fabs(a.y - b.y) > kEpsilon ||
        std::fabs(a.z - b.z) > kEpsilon ||
        std::fabs(a.w - b.w) > kEpsilon;
}

inline bool theme_active() {
    const ui_theme::ThemeSettings& theme =
        ui_theme::g_theme_settings;

    return theme_color_differs(
               theme.background,
               ui_theme::kDefaultThemeBackground) ||
        theme_color_differs(
               theme.surface,
               ui_theme::kDefaultThemeSurface) ||
        theme_color_differs(
               theme.accent,
               ui_theme::kDefaultThemeAccent) ||
        theme_color_differs(
               theme.text,
               ui_theme::kDefaultThemeText) ||
        theme_color_differs(
               theme.lists,
               ui_theme::kDefaultThemeLists);
}

inline ImU32 mix_theme_color(
    ImU32 fallback,
    const ImVec4& theme_color,
    float mix) {
    if (!theme_active()) {
        return fallback;
    }

    const ImVec4 base =
        ImGui::ColorConvertU32ToFloat4(fallback);
    ImVec4 out = ui_theme::theme_lerp(
        base,
        theme_color,
        std::clamp(mix, 0.0f, 1.0f));

    out.w = base.w;
    return ImGui::ColorConvertFloat4ToU32(out);
}

inline ImU32 text_color(ImU32 fallback) {
    if (!theme_active()) {
        return fallback;
    }

    const ImVec4 base =
        ImGui::ColorConvertU32ToFloat4(fallback);
    const float max_rgb =
        std::max(base.x, std::max(base.y, base.z));
    const float min_rgb =
        std::min(base.x, std::min(base.y, base.z));

    // Preserve semantic colors such as warnings and the
    // multi-color VibeStation brand.
    if ((max_rgb - min_rgb) > 0.22f) {
        return fallback;
    }

    return mix_theme_color(
        fallback,
        ui_theme::g_theme_settings.text,
        0.92f);
}

inline ImU32 surface_color(
    ImU32 fallback, float mix = 0.82f) {
    return mix_theme_color(
        fallback,
        ui_theme::g_theme_settings.surface,
        mix);
}

inline ImU32 list_color(
    ImU32 fallback, float mix = 0.84f) {
    return mix_theme_color(
        fallback,
        ui_theme::g_theme_settings.lists,
        mix);
}

inline ImU32 accent_color(
    ImU32 fallback, float mix = 0.92f) {
    return mix_theme_color(
        fallback,
        ui_theme::g_theme_settings.accent,
        mix);
}

inline ImU32 background_color(
    ImU32 fallback, float mix = 0.82f) {
    return mix_theme_color(
        fallback,
        ui_theme::g_theme_settings.background,
        mix);
}

inline int glow_alpha(float value) {
    return std::clamp(
        static_cast<int>(std::round(value)),
        0,
        255);
}

inline float smoothstep01(float value) {
    const float t =
        std::clamp(value, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

inline float timeline_progress(
    float time, float start, float end) {
    if (end <= start) {
        return time >= end ? 1.0f : 0.0f;
    }

    return smoothstep01(
        (time - start) / (end - start));
}

inline float ease_out_cubic(float t) {
    const float q = 1.0f - std::clamp(t, 0.0f, 1.0f);
    return 1.0f - q * q * q;
}

inline float ease_out_expo(float t) {
    if (t <= 0.0f) {
        return 0.0f;
    }
    return t >= 1.0f ? 1.0f : 1.0f - std::pow(2.0f, -10.0f * t);
}

inline float ease_in_out_cubic(float t) {
    const float q = std::clamp(t, 0.0f, 1.0f);
    if (q < 0.5f) {
        return 4.0f * q * q * q;
    }
    const float r = -2.0f * q + 2.0f;
    return 1.0f - r * r * r * 0.5f;
}

// Moves and fades everything drawn into `draw` since `vertex_start`.
inline void offset_draw_vertices(
    ImDrawList* draw,
    int vertex_start,
    const ImVec2& offset,
    float alpha) {
    if (draw == nullptr || vertex_start < 0) {
        return;
    }

    const float a = std::clamp(alpha, 0.0f, 1.0f);
    for (int i = vertex_start; i < draw->VtxBuffer.Size; ++i) {
        ImDrawVert& vertex = draw->VtxBuffer[i];
        vertex.pos.x += offset.x;
        vertex.pos.y += offset.y;
        if (a < 1.0f) {
            const ImU32 old_alpha = (vertex.col >> IM_COL32_A_SHIFT) & 0xFFu;
            const ImU32 new_alpha = static_cast<ImU32>(
                std::lround(static_cast<float>(old_alpha) * a));
            vertex.col = (vertex.col & ~(0xFFu << IM_COL32_A_SHIFT)) |
                (new_alpha << IM_COL32_A_SHIFT);
        }
    }
}

inline void animate_draw_vertices(
    ImDrawList* draw,
    int vertex_start,
    const ImVec2& center,
    float scale,
    float alpha,
    float y_offset) {
    if (draw == nullptr ||
        vertex_start < 0 ||
        vertex_start >= draw->VtxBuffer.Size) {
        return;
    }

    const float clamped_alpha =
        std::clamp(alpha, 0.0f, 1.0f);

    for (int i = vertex_start;
         i < draw->VtxBuffer.Size;
         ++i) {
        ImDrawVert& vertex = draw->VtxBuffer[i];

        vertex.pos.x =
            center.x +
            (vertex.pos.x - center.x) * scale;
        vertex.pos.y =
            center.y +
            (vertex.pos.y - center.y) * scale +
            y_offset;

        const ImU32 old_alpha =
            (vertex.col >> IM_COL32_A_SHIFT) & 0xFFu;
        const ImU32 new_alpha =
            static_cast<ImU32>(
                std::clamp(
                    static_cast<int>(
                        std::round(
                            static_cast<float>(
                                old_alpha) *
                            clamped_alpha)),
                    0,
                    255));

        vertex.col =
            (vertex.col &
                ~(0xFFu << IM_COL32_A_SHIFT)) |
            (new_alpha << IM_COL32_A_SHIFT);
    }
}

inline void add_text(
    ImDrawList* draw,
    const Layout& layout,
    float x,
    float y,
    float size,
    ImU32 color,
    const char* text) {
    const float font_size = layout.px(size);
    ImFont* font = font_for_size(font_size);

    draw->AddText(
        font,
        font_size,
        layout.point(x, y),
        text_color(color),
        text);
}

inline void add_text_right(
    ImDrawList* draw,
    const Layout& layout,
    float right_x,
    float y,
    float size,
    ImU32 color,
    const char* text) {
    const float font_size = layout.px(size);
    ImFont* font = font_for_size(font_size);
    const ImVec2 text_size =
        font->CalcTextSizeA(
            font_size,
            FLT_MAX,
            0.0f,
            text);
    const ImVec2 p =
        layout.point(right_x, y);

    draw->AddText(
        font,
        font_size,
        ImVec2(p.x - text_size.x, p.y),
        text_color(color),
        text);
}

} // namespace definitive_ui
