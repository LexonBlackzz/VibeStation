#pragma once

#include "ui/theme_settings.h"

#include <imgui.h>

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace definitive_ui {

inline constexpr float kDesignWidth = 1280.0f;
inline constexpr float kDesignHeight = 800.0f;
inline constexpr float kLauncherIntroDuration = 3.82f;

ImFont* font_for_size(float pixel_size);

void preload_intro_assets();
void release_intro_assets();
void draw_intro_presentation(
    const ImVec2& pos, const ImVec2& size, float elapsed);

void preload_audio_assets();
void release_audio_assets();

void preload_background_assets();
void release_background_assets();
void draw_launcher_background(
    ImDrawList* draw,
    const ImVec2& pos,
    const ImVec2& size,
    float opacity = 1.0f);
void draw_launcher_readability_shade(
    ImDrawList* draw,
    const ImVec2& pos,
    const ImVec2& size);
void draw_settings_background(
    ImDrawList* draw,
    const ImVec2& pos,
    const ImVec2& size);

void play_cursor_sound();
void play_open_sound();
void play_close_sound();
void play_startup_sound();
void stop_startup_sound();

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
