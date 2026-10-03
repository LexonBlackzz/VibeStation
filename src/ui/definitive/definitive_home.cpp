#include "ui/app.h"
#include "ui/definitive/definitive_shared.h"
#include "ui/output_resolution_utils.h"
#include "ui/screenshot_utils.h"
#include "ui/theme_settings.h"
#include "version.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace {
constexpr float kDesignWidth = 1280.0f;
constexpr float kDesignHeight = 800.0f;

std::array<bool, 7> g_menu_was_engaged = {};

struct LauncherQuote {
    const char* line1;
    const char* line2;
};

constexpr std::array<LauncherQuote, 48> kLauncherQuotes = {{
    {"A SMALLER PAST", "STILL PLAYS"},
    {"OLD DISC", "NEW NIGHT"},
    {"MEMORY CARD", "STILL WARM"},
    {"BOOT AGAIN", "STAY A WHILE"},
    {"PRESS START", "SEE WHAT RETURNS"},
    {"ONE MORE SAVE", "ONE MORE RUN"},
    {"THE CRT HUMS", "THE DISC SPINS"},
    {"NO PATCH NOTES", "JUST MEMORIES"},
    {"LOAD THE PAST", "PLAY IT FORWARD"},
    {"PIXELS FADE", "MEMORIES DON'T"},
    {"THE ROOM IS DARK", "THE SCREEN IS ON"},
    {"KEEP THE STATIC", "LOSE THE DUST"},
    {"THE DISC KNOWS", "WHERE YOU LEFT OFF"},
    {"OLD HARDWARE", "NEW VIBES"},
    {"WAIT FOR THE CHIME", "THEN BEGIN"},
    {"INSERT MEMORY", "REMOVE TIME"},
    {"THE SAVE IS THERE", "GO FIND IT"},
    {"THE NIGHT IS YOUNG", "THE DISC IS OLD"},
    {"LOW POLY", "HIGH MEMORY"},
    {"PAUSE THE WORLD", "LOAD THE GAME"},
    {"ONE CONSOLE", "MANY NIGHTS"},
    {"LET IT BOOT", "LET IT BREATHE"},
    {"THE PAST", "HAS A FRAME RATE"},
    {"STILL LOADING", "STILL WORTH IT"},
    {"FROM DISC", "TO MEMORY"},
    {"NO CLOUD", "JUST CARDS"},
    {"32 BITS", "ENDLESS NIGHTS"},
    {"START BUTTON", "SAME FEELING"},
    {"OLD SAVE", "NEW CHANCE"},
    {"THE LID CLOSES", "THE WORLD OPENS"},
    {"ONE MORE BOOT", "ONE MORE MEMORY"},
    {"THE SCREEN GLOWS", "THE ROOM DISAPPEARS"},
    {"THEN: YESTERDAY", "NOW: TONIGHT"},
    {"MOTION BLUR", "MEMORY SHARP"},
    {"SAME BUTTONS", "DIFFERENT NIGHT"},
    {"DISC IN", "WORLD OUT"},
    {"THE LOGO FADES", "THE GAME REMAINS"},
    {"READY WHEN", "YOU ARE"},
    {"THE PAST WAITS", "AT 60 FPS"},
    {"LOAD. SAVE.", "REPEAT."},
    {"GOOD GAMES", "GOOD TIMES"},
    {"OLD WORLDS", "STILL OPEN"},
    {"SAVE OFTEN", "STAY LONGER"},
    {"ANOTHER BOOT", "ANOTHER STORY"},
    {"TURN IT ON", "LET TIME STOP"},
    {"THE DISC TURNS", "THE NIGHT MOVES"},
    {"SAME START", "NEW MEMORY"},
    {"WELCOME BACK", "PLAYER ONE"},
}};

size_t g_launcher_quote_index = 0;
bool g_launcher_quote_selected = false;

std::array<float, 7> g_menu_highlight_mix = {};

enum class LauncherStartTransition {
    None,
    Bios,
    Disc
};

LauncherStartTransition g_launcher_start_transition =
    LauncherStartTransition::None;
float g_launcher_start_transition_elapsed = 0.0f;
constexpr float kLauncherStartFadeSeconds = 0.42f;

// Startup runs on one clock (definitive_shared.h): intro, then the title
// glides into place while the launcher arrives underneath it.
float g_startup_elapsed = 0.0f;
bool g_startup_complete = false;

// Launcher arrival, in seconds on the startup clock.
constexpr float kRowsStart = definitive_ui::kIntroGlideStart + 0.35f;
constexpr float kRowStagger = 0.06f;
constexpr float kRowDuration = 0.6f;
constexpr float kRowSlide = 32.0f;
constexpr float kMetaStart = definitive_ui::kIntroGlideStart + 0.55f;
constexpr float kPanelsStart = kRowsStart + 0.45f;
constexpr float kPanelStagger = 0.1f;
constexpr float kPanelDuration = 0.7f;
constexpr float kPanelRise = 20.0f;
constexpr float kPhotoStart = definitive_ui::kIntroGlideStart + 0.6f;
constexpr float kPhotoZoom = 1.08f;
// After the last row lands, a highlight slides onto Start Emulation. It is a
// hint only: it goes once anything is hovered or focused.
constexpr float kIntroHighlightStart = definitive_ui::kLauncherInteractiveAt - 0.12f;
constexpr float kIntroHighlightDuration = 0.45f;
float g_intro_highlight = 0.0f;
bool g_intro_highlight_dismissed = false;


ImU32 rgba(int r, int g, int b, int a = 255) {
    return IM_COL32(r, g, b, a);
}

bool definitive_theme_color_differs(
    const ImVec4& a, const ImVec4& b) {
    constexpr float kEpsilon = 0.0025f;
    return std::fabs(a.x - b.x) > kEpsilon ||
        std::fabs(a.y - b.y) > kEpsilon ||
        std::fabs(a.z - b.z) > kEpsilon ||
        std::fabs(a.w - b.w) > kEpsilon;
}

bool definitive_theme_active() {
    const ui_theme::ThemeSettings& theme = ui_theme::g_theme_settings;
    return definitive_theme_color_differs(
               theme.background, ui_theme::kDefaultThemeBackground) ||
        definitive_theme_color_differs(
               theme.surface, ui_theme::kDefaultThemeSurface) ||
        definitive_theme_color_differs(
               theme.accent, ui_theme::kDefaultThemeAccent) ||
        definitive_theme_color_differs(
               theme.text, ui_theme::kDefaultThemeText) ||
        definitive_theme_color_differs(
               theme.lists, ui_theme::kDefaultThemeLists);
}

ImU32 definitive_mix_theme_color(
    ImU32 fallback, const ImVec4& theme_color, float mix) {
    if (!definitive_theme_active()) {
        return fallback;
    }

    const ImVec4 base = ImGui::ColorConvertU32ToFloat4(fallback);
    ImVec4 out = ui_theme::theme_lerp(
        base, theme_color, std::clamp(mix, 0.0f, 1.0f));
    // Preserve the role-specific opacity from the original Definitive color.
    out.w = base.w;
    return ImGui::ColorConvertFloat4ToU32(out);
}

ImU32 definitive_text_color(ImU32 fallback) {
    if (!definitive_theme_active()) {
        return fallback;
    }

    const ImVec4 base = ImGui::ColorConvertU32ToFloat4(fallback);
    const float max_rgb = std::max(base.x, std::max(base.y, base.z));
    const float min_rgb = std::min(base.x, std::min(base.y, base.z));

    // Keep deliberate semantic colors such as warnings and the four-color
    // VibeStation brand accents intact. Neutral UI text follows Text.
    if ((max_rgb - min_rgb) > 0.22f) {
        return fallback;
    }

    return definitive_mix_theme_color(
        fallback, ui_theme::g_theme_settings.text, 0.92f);
}

ImU32 definitive_surface_color(ImU32 fallback, float mix = 0.82f) {
    return definitive_mix_theme_color(
        fallback, ui_theme::g_theme_settings.surface, mix);
}

ImU32 definitive_list_color(ImU32 fallback, float mix = 0.84f) {
    return definitive_mix_theme_color(
        fallback, ui_theme::g_theme_settings.lists, mix);
}

ImU32 definitive_accent_color(ImU32 fallback, float mix = 0.92f) {
    return definitive_mix_theme_color(
        fallback, ui_theme::g_theme_settings.accent, mix);
}

ImU32 definitive_background_color(ImU32 fallback, float mix = 0.82f) {
    return definitive_mix_theme_color(
        fallback, ui_theme::g_theme_settings.background, mix);
}


float animate_towards(float current, float target, float response = 13.0f) {
    const float dt = std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);
    if (dt <= 0.0f) {
        return target;
    }
    const float alpha = 1.0f - std::exp(-response * dt);
    return current + (target - current) * alpha;
}

int glow_alpha(float value) {
    return std::clamp(static_cast<int>(std::round(value)), 0, 255);
}

float smoothstep01(float value) {
    const float t = std::clamp(value, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float timeline_progress(float time, float start, float end) {
    if (end <= start) {
        return time >= end ? 1.0f : 0.0f;
    }
    return smoothstep01((time - start) / (end - start));
}

ImVec2 lerp_point(const ImVec2& a, const ImVec2& b, float t) {
    const float clamped = std::clamp(t, 0.0f, 1.0f);
    return ImVec2(
        a.x + (b.x - a.x) * clamped,
        a.y + (b.y - a.y) * clamped);
}


struct Layout {
    ImVec2 origin{};
    float scale = 1.0f;

    ImVec2 point(float x, float y) const {
        return ImVec2(origin.x + x * scale, origin.y + y * scale);
    }
    ImVec2 size(float x, float y) const {
        return ImVec2(x * scale, y * scale);
    }
    float px(float value) const {
        return value * scale;
    }
};

Layout make_layout(const ImVec2& window_pos, const ImVec2& window_size) {
    Layout layout{};
    layout.scale = std::max(
        0.55f, std::min(window_size.x / kDesignWidth, window_size.y / kDesignHeight));
    const ImVec2 design_size(kDesignWidth * layout.scale, kDesignHeight * layout.scale);
    layout.origin = ImVec2(
        window_pos.x + (window_size.x - design_size.x) * 0.5f,
        window_pos.y + (window_size.y - design_size.y) * 0.5f);
    return layout;
}

void add_text(ImDrawList* draw, const Layout& layout, float x, float y,
    float size, ImU32 color, const char* text) {
    const float font_size = layout.px(size);
    ImFont* font = definitive_ui::font_for_size(font_size);
    draw->AddText(
        font, font_size, layout.point(x, y),
        definitive_text_color(color), text);
}

void add_text_right(ImDrawList* draw, const Layout& layout, float right_x, float y,
    float size, ImU32 color, const char* text) {
    const float font_size = layout.px(size);
    ImFont* font = definitive_ui::font_for_size(font_size);
    const ImVec2 text_size = font->CalcTextSizeA(
        font_size, FLT_MAX, 0.0f, text);
    const ImVec2 p = layout.point(right_x, y);
    draw->AddText(font, font_size,
        ImVec2(p.x - text_size.x, p.y),
        definitive_text_color(color), text);
}

enum class MenuIcon {
    Play,
    Folder,
    Chip,
    Skull,
    Settings,
    Orbit,
    Exit
};

void draw_icon(ImDrawList* draw, const Layout& layout, MenuIcon icon,
    float x, float y, ImU32 color) {
    const ImVec2 p = layout.point(x, y);
    const float s = layout.scale;

    switch (icon) {
    case MenuIcon::Play:
        draw->AddTriangleFilled(
            ImVec2(p.x, p.y),
            ImVec2(p.x, p.y + 22.0f * s),
            ImVec2(p.x + 18.0f * s, p.y + 11.0f * s),
            color);
        break;
    case MenuIcon::Folder:
        draw->AddLine(
            ImVec2(p.x, p.y + 5.0f * s),
            ImVec2(p.x + 8.0f * s, p.y + 5.0f * s), color, 2.0f * s);
        draw->AddLine(
            ImVec2(p.x + 8.0f * s, p.y + 5.0f * s),
            ImVec2(p.x + 12.0f * s, p.y + 9.0f * s), color, 2.0f * s);
        draw->AddRect(
            ImVec2(p.x, p.y + 8.0f * s),
            ImVec2(p.x + 24.0f * s, p.y + 24.0f * s),
            color, 1.5f * s, 0, 2.0f * s);
        break;
    case MenuIcon::Chip:
        draw->AddRect(
            ImVec2(p.x + 4.0f * s, p.y + 3.0f * s),
            ImVec2(p.x + 22.0f * s, p.y + 25.0f * s),
            color, 1.0f * s, 0, 2.0f * s);
        for (int i = 0; i < 4; ++i) {
            const float py = p.y + (6.0f + i * 5.0f) * s;
            draw->AddLine(ImVec2(p.x, py), ImVec2(p.x + 4.0f * s, py),
                color, 1.5f * s);
            draw->AddLine(ImVec2(p.x + 22.0f * s, py),
                ImVec2(p.x + 26.0f * s, py), color, 1.5f * s);
        }
        break;
    case MenuIcon::Skull:
        draw->AddCircleFilled(
            ImVec2(p.x + 13.0f * s, p.y + 10.0f * s),
            9.0f * s, color, 20);
        draw->AddRectFilled(
            ImVec2(p.x + 7.0f * s, p.y + 15.0f * s),
            ImVec2(p.x + 19.0f * s, p.y + 23.0f * s),
            color, 2.0f * s);
        draw->AddCircleFilled(
            ImVec2(p.x + 9.5f * s, p.y + 9.0f * s),
            2.1f * s, rgba(8, 11, 15, 255), 10);
        draw->AddCircleFilled(
            ImVec2(p.x + 16.5f * s, p.y + 9.0f * s),
            2.1f * s, rgba(8, 11, 15, 255), 10);
        draw->AddTriangleFilled(
            ImVec2(p.x + 13.0f * s, p.y + 11.0f * s),
            ImVec2(p.x + 11.0f * s, p.y + 15.0f * s),
            ImVec2(p.x + 15.0f * s, p.y + 15.0f * s),
            rgba(8, 11, 15, 255));
        break;
    case MenuIcon::Settings:
        draw->AddCircle(
            ImVec2(p.x + 13.0f * s, p.y + 14.0f * s),
            9.0f * s, color, 12, 2.0f * s);
        draw->AddCircle(
            ImVec2(p.x + 13.0f * s, p.y + 14.0f * s),
            3.0f * s, color, 12, 2.0f * s);
        for (int i = 0; i < 8; ++i) {
            const float a = static_cast<float>(i) * 3.14159265f / 4.0f;
            const ImVec2 a0(
                p.x + 10.0f * s * std::cos(a) + 13.0f * s,
                p.y + 10.0f * s * std::sin(a) + 14.0f * s);
            const ImVec2 a1(
                p.x + 14.0f * s * std::cos(a) + 13.0f * s,
                p.y + 14.0f * s * std::sin(a) + 14.0f * s);
            draw->AddLine(a0, a1, color, 2.0f * s);
        }
        break;
    case MenuIcon::Orbit: {
        // VibeStation 2's orbiting lights: a tilted ring with three of them.
        const ImVec2 c(p.x + 13.0f * s, p.y + 14.0f * s);
        draw->AddEllipse(c, ImVec2(12.0f * s, 6.5f * s), color, -0.42f, 24, 1.4f * s);
        static constexpr float kAngles[] = {0.4f, 2.5f, 4.4f};
        for (const float a : kAngles) {
            const float ex = 12.0f * s * std::cos(a), ey = 6.5f * s * std::sin(a);
            const float cr = std::cos(-0.42f), sr = std::sin(-0.42f);
            draw->AddCircleFilled(ImVec2(c.x + ex * cr - ey * sr, c.y + ex * sr + ey * cr),
                                  2.6f * s, color, 10);
        }
        draw->AddCircleFilled(c, 2.0f * s, color, 10);
        break;
    }
    case MenuIcon::Exit:
        draw->AddRect(
            ImVec2(p.x + 8.0f * s, p.y + 2.0f * s),
            ImVec2(p.x + 25.0f * s, p.y + 26.0f * s),
            color, 0.0f, 0, 1.8f * s);
        draw->AddLine(
            ImVec2(p.x, p.y + 14.0f * s),
            ImVec2(p.x + 16.0f * s, p.y + 14.0f * s),
            color, 2.0f * s);
        draw->AddLine(
            ImVec2(p.x + 11.0f * s, p.y + 9.0f * s),
            ImVec2(p.x + 16.0f * s, p.y + 14.0f * s),
            color, 2.0f * s);
        draw->AddLine(
            ImVec2(p.x + 11.0f * s, p.y + 19.0f * s),
            ImVec2(p.x + 16.0f * s, p.y + 14.0f * s),
            color, 2.0f * s);
        break;
    }
}

bool menu_button(const Layout& layout, ImDrawList* draw, int index,
    MenuIcon icon, const char* title, const char* subtitle,
    bool interaction_enabled = true) {
    constexpr float kX = 36.0f;
    constexpr float kY = 168.0f;
    constexpr float kWidth = 396.0f;
    constexpr float kHeight = 50.0f;
    constexpr float kGap = 4.0f;

    const float y = kY + index * (kHeight + kGap);
    const ImVec2 p = layout.point(kX, y);
    const ImVec2 size = layout.size(kWidth, kHeight);

    ImGui::SetCursorScreenPos(p);
    ImGui::PushID(index);
    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(0, 0, 0, 0));
    const bool pressed = ImGui::Button("##definitive_menu", size);
    ImGui::PopStyleColor(3);

    const bool hovered =
        interaction_enabled && ImGui::IsItemHovered();
    const bool focused =
        interaction_enabled && ImGui::IsItemFocused();
    const bool active =
        interaction_enabled && ImGui::IsItemActive();
    const bool engaged = hovered || focused || active;

    bool& was_engaged =
        g_menu_was_engaged[static_cast<size_t>(index)];
    if (interaction_enabled && engaged && !was_engaged) {
        definitive_ui::play_cursor_sound();
    }
    was_engaged = engaged;

    // Every item gets an explicit zero target whenever it is not engaged.
    // This prevents the previous menu item from remaining lit after focus moves.
    const float target_mix = engaged ? 1.0f : 0.0f;
    float& highlight_mix = g_menu_highlight_mix[static_cast<size_t>(index)];
    highlight_mix = animate_towards(
        highlight_mix, target_mix, engaged ? 16.0f : 9.5f);
    if (!engaged && highlight_mix < 0.004f) {
        highlight_mix = 0.0f;
    }

    // Startup hint on Start Emulation: the highlight grows in from the left.
    // Touching any item ends it, and the item then fades as usual.
    if (engaged) {
        g_intro_highlight_dismissed = true;
    }
    float box_w = size.x;
    if (index == 0 && !g_intro_highlight_dismissed) {
        highlight_mix = std::max(highlight_mix, g_intro_highlight);
        box_w = size.x * std::max(0.02f, g_intro_highlight);
    }

    const float pulse = engaged
        ? (0.92f + 0.08f *
            std::sin(static_cast<float>(ImGui::GetTime()) * 2.1f))
        : 1.0f;
    const float glow = std::clamp(
        highlight_mix * pulse + (active ? 0.14f : 0.0f), 0.0f, 1.1f);

    if (glow > 0.01f) {
        const float glow_outer = layout.px(6.0f + glow * 2.0f);
        const float glow_mid = layout.px(3.0f + glow);
        const ImVec2 outer0(p.x - glow_outer, p.y - glow_outer);
        const ImVec2 outer1(p.x + box_w + glow_outer, p.y + size.y + glow_outer);
        const ImVec2 mid0(p.x - glow_mid, p.y - glow_mid);
        const ImVec2 mid1(p.x + box_w + glow_mid, p.y + size.y + glow_mid);

        draw->AddRect(outer0, outer1,
            definitive_accent_color(
                rgba(90, 154, 216, glow_alpha(17.0f * glow))),
            0.0f, 0, layout.px(1.0f));
        draw->AddRect(mid0, mid1,
            definitive_accent_color(
                rgba(126, 184, 236, glow_alpha(34.0f * glow))),
            0.0f, 0, layout.px(1.2f));
    }

    if (highlight_mix > 0.01f) {
        const int fill_alpha =
            glow_alpha(116.0f * highlight_mix + (hovered ? 10.0f : 0.0f));
        draw->AddRectFilled(
            p, ImVec2(p.x + box_w, p.y + size.y),
            definitive_surface_color(rgba(12, 17, 23, fill_alpha), 0.90f));

        draw->AddRect(
            p, ImVec2(p.x + box_w, p.y + size.y),
            definitive_accent_color(
                rgba(211, 229, 246, glow_alpha(235.0f * highlight_mix))),
            0.0f, 0, layout.px(1.35f));
        draw->AddRect(
            ImVec2(p.x + layout.px(2.0f), p.y + layout.px(2.0f)),
            ImVec2(p.x + box_w - layout.px(2.0f),
                p.y + size.y - layout.px(2.0f)),
            definitive_accent_color(
                rgba(103, 154, 205, glow_alpha(128.0f * highlight_mix))),
            0.0f, 0, layout.px(0.8f));

        const float rail_half = layout.px(16.0f + 5.0f * highlight_mix);
        const float center_y = p.y + size.y * 0.5f;
        draw->AddRectFilled(
            ImVec2(p.x - layout.px(2.0f), center_y - rail_half),
            ImVec2(p.x, center_y + rail_half),
            definitive_accent_color(
                rgba(205, 231, 255, glow_alpha(235.0f * highlight_mix))));
    }

    const float emphasis = std::clamp(highlight_mix, 0.0f, 1.0f);
    const ImU32 main_color = rgba(
        static_cast<int>(206 + 36 * emphasis),
        static_cast<int>(208 + 38 * emphasis),
        static_cast<int>(211 + 39 * emphasis),
        static_cast<int>(228 + 27 * emphasis));
    const ImU32 sub_color = rgba(
        static_cast<int>(150 + 34 * emphasis),
        static_cast<int>(154 + 38 * emphasis),
        static_cast<int>(161 + 40 * emphasis),
        static_cast<int>(210 + 28 * emphasis));

    const float content_shift = 2.0f * highlight_mix;
    draw_icon(draw, layout, icon,
        kX + 24.0f + content_shift, y + 11.0f,
        definitive_text_color(main_color));
    add_text(draw, layout, kX + 72.0f + content_shift, y + 6.0f, 18.5f,
        main_color, title);
    add_text(draw, layout, kX + 72.0f + content_shift, y + 31.0f, 11.5f,
        sub_color, subtitle);

    ImGui::PopID();
    return interaction_enabled && pressed;
}

bool small_button(const Layout& layout, const char* id, const char* label,
    float x, float y, float w, float h, bool enabled = true) {
    ImGui::SetCursorScreenPos(layout.point(x, y));
    if (!enabled) {
        ImGui::BeginDisabled();
    }
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button,
        definitive_surface_color(rgba(12, 15, 19, 205), 0.88f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
        definitive_surface_color(rgba(24, 31, 39, 225), 0.72f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
        definitive_accent_color(rgba(32, 42, 52, 235), 0.76f));
    ImGui::PushStyleColor(ImGuiCol_Border,
        definitive_accent_color(rgba(128, 145, 163, 190), 0.68f));
    ImGui::PushStyleColor(ImGuiCol_Text,
        definitive_text_color(rgba(226, 230, 235, 245)));
    ImGui::PushID(id);
    const bool pressed = ImGui::Button(label, layout.size(w, h));
    ImGui::PopID();
    ImGui::PopStyleColor(5);
    ImGui::PopStyleVar(2);
    if (!enabled) {
        ImGui::EndDisabled();
    }
    return pressed;
}

void draw_panel(ImDrawList* draw, const Layout& layout,
    float x, float y, float w, float h) {
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 p1 = layout.point(x + w, y + h);
    draw->AddRectFilled(
        p0, p1, definitive_list_color(rgba(5, 8, 11, 178), 0.88f));
    draw->AddRect(
        p0, p1, definitive_accent_color(rgba(102, 116, 130, 205), 0.58f),
        0.0f, 0, layout.px(1.0f));
}

void draw_folder_badge(ImDrawList* draw, const Layout& layout, float x, float y) {
    const ImVec2 p = layout.point(x, y);
    const float s = layout.scale;
    const ImU32 c = definitive_text_color(rgba(226, 230, 235, 240));
    draw->AddLine(ImVec2(p.x, p.y + 4.0f * s),
        ImVec2(p.x + 8.0f * s, p.y + 4.0f * s), c, 1.6f * s);
    draw->AddLine(ImVec2(p.x + 8.0f * s, p.y + 4.0f * s),
        ImVec2(p.x + 12.0f * s, p.y + 8.0f * s), c, 1.6f * s);
    draw->AddRect(ImVec2(p.x, p.y + 7.0f * s),
        ImVec2(p.x + 22.0f * s, p.y + 20.0f * s), c, 1.0f * s, 0, 1.6f * s);
}

void draw_info_badge(ImDrawList* draw, const Layout& layout, float x, float y) {
    const ImVec2 p = layout.point(x, y);
    const float s = layout.scale;
    const ImU32 c = definitive_text_color(rgba(226, 230, 235, 240));
    draw->AddCircle(ImVec2(p.x + 9.0f * s, p.y + 10.0f * s),
        8.0f * s, c, 16, 1.5f * s);
    draw->AddCircleFilled(ImVec2(p.x + 9.0f * s, p.y + 6.0f * s),
        1.0f * s, c);
    draw->AddLine(ImVec2(p.x + 9.0f * s, p.y + 9.0f * s),
        ImVec2(p.x + 9.0f * s, p.y + 15.0f * s), c, 1.5f * s);
}
}

void App::release_definitive_ui_assets() {
    definitive_ui::release_audio_assets();
    g_menu_was_engaged.fill(false);

    definitive_ui::release_intro_assets();

    definitive_ui::release_background_assets();
    definitive_ui::release_gameplay_ambient_assets();
    g_launcher_start_transition = LauncherStartTransition::None;
    g_launcher_start_transition_elapsed = 0.0f;
    g_startup_elapsed = 0.0f;
    g_startup_complete = false;
    g_intro_highlight = 0.0f;
    g_intro_highlight_dismissed = false;
    definitive_settings_transition_ =
        DefinitiveSettingsTransition::Closed;
    definitive_settings_transition_elapsed_ = 0.0f;
    g_menu_highlight_mix.fill(0.0f);
}

void App::panel_definitive_home() {
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 window_pos = ImGui::GetWindowPos();
    const ImVec2 window_size = ImGui::GetWindowSize();

    definitive_ui::play_startup_sound();

    const float dt = std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);
    if (!g_startup_complete) {
        // Space, Enter, A or a click skips to the glide. The launcher is not
        // clickable until well after that, so the same press cannot land on
        // a button underneath.
        const bool skip_intro =
            g_startup_elapsed < definitive_ui::kIntroGlideStart &&
            (ImGui::IsKeyPressed(ImGuiKey_Space, false) ||
                ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
                ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false) ||
                ImGui::IsKeyPressed(ImGuiKey_GamepadFaceDown, false) ||
                ImGui::IsMouseClicked(ImGuiMouseButton_Left));

        if (skip_intro) {
            definitive_ui::stop_startup_sound();
            g_startup_elapsed = definitive_ui::kIntroGlideStart;
        }
        else {
            g_startup_elapsed += dt;
        }
        if (g_startup_elapsed >= definitive_ui::kLauncherSequenceEnd) {
            g_startup_elapsed = definitive_ui::kLauncherSequenceEnd;
            g_startup_complete = true;
        }
    }
    const float startup = g_startup_elapsed;
    const bool launcher_ready =
        startup >= definitive_ui::kLauncherInteractiveAt;

    if (!g_launcher_quote_selected) {
        const Uint64 entropy =
            SDL_GetPerformanceCounter() ^
            (static_cast<Uint64>(SDL_GetTicks()) << 32);
        g_launcher_quote_index =
            static_cast<size_t>(entropy % kLauncherQuotes.size());
        g_launcher_quote_selected = true;
    }

    // Load/soften the photograph while the boot presentation is still on
    // black so the transition into the launcher is hitch-free.
    definitive_ui::preload_background_assets();
    definitive_ui::preload_intro_assets();
    definitive_ui::preload_audio_assets();

    Layout layout = make_layout(window_pos, window_size);

    definitive_ui::IntroTargets intro_targets{};
    intro_targets.brand_pos = layout.point(48.0f, 36.0f);
    intro_targets.brand_size = layout.px(54.0f);
    for (int i = 0; i < 4; ++i) {
        intro_targets.bar_centers[i] = layout.point(63.0f + i * 32.0f, 121.0f);
    }
    intro_targets.ui_scale = layout.scale;

    // The intro owns the screen until the glide.
    if (startup < definitive_ui::kIntroGlideStart) {
        definitive_ui::draw_intro_presentation(
            window_pos, window_size, startup, intro_targets);
        return;
    }

    // The photo fades in quickly, then keeps settling: it zooms out from
    // kPhotoZoom and sharpens from its blurred copy.
    const float photo_t = std::clamp(
        (startup - kPhotoStart) /
            (definitive_ui::kLauncherSequenceEnd - kPhotoStart),
        0.0f, 1.0f);
    const float photo_settle = definitive_ui::ease_out_cubic(photo_t);
    definitive_ui::draw_launcher_background(
        draw, window_pos, window_size,
        smoothstep01(photo_t / 0.55f),
        1.0f + (kPhotoZoom - 1.0f) * (1.0f - photo_settle),
        1.0f - photo_settle);

    definitive_ui::draw_launcher_readability_shade(
        draw,
        window_pos,
        window_size);

    const ImVec2 bottom0(window_pos.x, window_pos.y + window_size.y * 0.64f);
    const ImVec2 bottom1(window_pos.x + window_size.x, window_pos.y + window_size.y);
    draw->AddRectFilledMultiColor(bottom0, bottom1,
        definitive_background_color(rgba(1, 3, 6, 10), 0.76f),
        definitive_background_color(rgba(1, 3, 6, 10), 0.76f),
        definitive_background_color(rgba(1, 3, 6, 206), 0.82f),
        definitive_background_color(rgba(1, 3, 6, 206), 0.82f));

    // Until the glide lands, the intro draws the title on its way here.
    if (startup >= definitive_ui::kIntroGlideEnd) {
        add_text(draw, layout, 48.0f, 36.0f, 54.0f,
            rgba(223, 225, 228, 248), "VibeStation");
    }

    constexpr std::array<ImU32, 4> accent_colors = {
        IM_COL32(194, 44, 56, 255),
        IM_COL32(52, 128, 125, 255),
        IM_COL32(177, 145, 72, 255),
        IM_COL32(52, 93, 157, 255),
    };
    constexpr std::array<ImU32, 4> accent_glow_colors = {
        IM_COL32(194, 44, 56, 40),
        IM_COL32(52, 128, 125, 40),
        IM_COL32(177, 145, 72, 40),
        IM_COL32(52, 93, 157, 40),
    };
    const float accent_time = static_cast<float>(ImGui::GetTime());
    for (int i = 0; i < 4; ++i) {
        // Each bar appears as the intro light flying into it lands.
        const float bar_alpha =
            definitive_ui::intro_color_bar_alpha(i, startup);
        if (bar_alpha <= 0.001f) {
            continue;
        }
        const int bar_vertices = draw->VtxBuffer.Size;
        const float pulse = 0.55f +
            0.45f * std::sin(accent_time * 1.35f + static_cast<float>(i) * 0.78f);
        const ImVec2 p0 = layout.point(50.0f + i * 32.0f, 116.0f);
        const ImVec2 p1 = layout.point(76.0f + i * 32.0f, 126.0f);
        const float spread = layout.px(1.5f + pulse * 1.25f);
        draw->AddRectFilled(
            ImVec2(p0.x - spread, p0.y - spread),
            ImVec2(p1.x + spread, p1.y + spread),
            accent_glow_colors[static_cast<size_t>(i)]);
        draw->AddRectFilled(p0, p1, accent_colors[static_cast<size_t>(i)]);
        definitive_ui::offset_draw_vertices(
            draw, bar_vertices, ImVec2(0.0f, 0.0f), bar_alpha);
    }

    // Version and quote drift in from the right.
    const float meta_in =
        definitive_ui::ease_out_cubic((startup - kMetaStart) / 0.6f);
    const int meta_vertices = draw->VtxBuffer.Size;
    add_text_right(draw, layout, 1235.0f, 34.0f, 11.5f,
        rgba(176, 183, 191, 232), vibestation_version_string());
    const LauncherQuote& launcher_quote =
        kLauncherQuotes[g_launcher_quote_index];
    add_text_right(draw, layout, 1235.0f, 57.0f, 12.5f,
        rgba(198, 203, 210, 238), launcher_quote.line1);
    add_text_right(draw, layout, 1235.0f, 77.0f, 12.5f,
        rgba(198, 203, 210, 238), launcher_quote.line2);
    const ImVec2 dash0 = layout.point(1208.0f, 103.0f);
    const ImVec2 dash1 = layout.point(1235.0f, 103.0f);
    draw->AddLine(dash0, dash1, rgba(180, 184, 190, 190), layout.px(1.0f));
    definitive_ui::offset_draw_vertices(
        draw, meta_vertices,
        ImVec2(layout.px(14.0f) * (1.0f - meta_in), 0.0f), meta_in);

    g_intro_highlight =
        startup < kIntroHighlightStart
            ? 0.0f
            : definitive_ui::ease_out_cubic(
                (startup - kIntroHighlightStart) / kIntroHighlightDuration);

    // Rows slide in from the left one after another, fading as they come.
    const auto menu_row = [&](int index, MenuIcon icon,
                              const char* title, const char* subtitle) {
        const float start = kRowsStart + kRowStagger * static_cast<float>(index);
        const float q = (startup - start) / kRowDuration;
        const int row_vertices = draw->VtxBuffer.Size;
        const bool pressed = menu_button(
            layout, draw, index, icon, title, subtitle, launcher_ready);
        if (q < 1.0f) {
            definitive_ui::offset_draw_vertices(
                draw, row_vertices,
                ImVec2(-layout.px(kRowSlide) *
                        (1.0f - definitive_ui::ease_out_expo(q)),
                    0.0f),
                smoothstep01(q / 0.6f));
        }
        return pressed;
    };

    bool start_pressed = menu_row(0, MenuIcon::Play,
        "Start Emulation", "Load BIOS and start playing");
    const bool load_game_pressed = menu_row(1, MenuIcon::Folder,
        "Load Game", "Choose a game from your library");
    const bool change_bios_pressed = menu_row(2, MenuIcon::Chip,
        "Change BIOS", "Manage BIOS files");
    const bool grim_reaper_pressed = menu_row(3, MenuIcon::Skull,
        "Grim Reaper", "Corrupt BIOS, RAM, GPU and audio");
    const bool settings_pressed = menu_row(4, MenuIcon::Settings,
        "Settings", "Configure emulator options");
    const bool vs2_pressed = menu_row(5, MenuIcon::Orbit,
        "VibeStation 2", "Switch to the PS2 emulator (experimental)");
    const bool exit_pressed = menu_row(6, MenuIcon::Exit,
        "Exit", "Close VibeStation");

    // Enter accepts the startup highlight on Start Emulation while nothing
    // else has keyboard focus.
    if (launcher_ready && !g_intro_highlight_dismissed &&
        g_intro_highlight > 0.5f && !ImGui::IsAnyItemFocused() &&
        !vs2_warning_open_ &&
        (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false))) {
        g_intro_highlight_dismissed = true;
        start_pressed = true;
    }

    const auto choose_bios = [this]() -> bool {
        std::string path = open_file_dialog(
            "BIOS Files (*.bin)\0*.bin\0All Files\0*.*\0", "Select PS1 BIOS");
        if (path.empty()) {
            return false;
        }

        emu_runner_.pause_and_wait_idle();
        disable_ram_reaper_mode();
        disable_gpu_reaper_mode();
        disable_sound_reaper_mode();
        if (!system_->load_bios(path)) {
            status_message_ = "Failed to load BIOS!";
            return false;
        }

        bios_path_ = path;
        save_persistent_config();
        has_started_emulation_ = false;
        set_grim_reaper_mode(false);
        status_message_ = "BIOS loaded: " + system_->bios().get_info();
        return true;
    };

    if (start_pressed && launcher_ready &&
        g_launcher_start_transition == LauncherStartTransition::None) {
        play_ui_open_sound();
        if (!system_->bios_loaded() && !choose_bios()) {
            // File picker cancelled or BIOS failed to load.
        }
        else {
            const bool has_selected_game =
                !game_bin_path_.empty() || system_->disc_loaded();
            g_launcher_start_transition = has_selected_game
                ? LauncherStartTransition::Disc
                : LauncherStartTransition::Bios;
            g_launcher_start_transition_elapsed = 0.0f;
            status_message_ = has_selected_game
                ? "Starting selected game..."
                : "Starting emulation...";
        }
    }

    const bool launcher_transitioning =
        g_launcher_start_transition != LauncherStartTransition::None;

    if (load_game_pressed && launcher_ready &&
        !launcher_transitioning) {
        play_ui_open_sound();
        std::string path = open_file_dialog(
            "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
            "Select PS1 Game");
        play_ui_close_sound();
        if (!path.empty()) {
            std::string bin;
            std::string cue;
            std::string error;
            if (!resolve_disc_paths(path, bin, cue, error)) {
                status_message_ = error;
            }
            else {
                load_disc_from_ui(bin, cue);
            }
        }
    }

    if (change_bios_pressed && launcher_ready &&
        !launcher_transitioning) {
        play_ui_open_sound();
        choose_bios();
        play_ui_close_sound();
    }
    if (grim_reaper_pressed && launcher_ready &&
        !launcher_transitioning) {
        if (definitive_grim_reaper_active_ &&
            !definitive_grim_reaper_closing_) {
            play_ui_close_sound();
            close_definitive_grim_reaper();
        }
        else {
            play_ui_open_sound();
            open_definitive_grim_reaper();
        }
    }
    if (settings_pressed && launcher_ready &&
        !launcher_transitioning) {
        play_ui_open_sound();
        open_definitive_settings();
    }
    if (vs2_pressed && launcher_ready &&
        !launcher_transitioning && !vs2_warning_open_) {
        // Experimental: always confirm first (definitive_vs2_switch.cpp).
        play_ui_open_sound();
        vs2_warning_open_ = true;
    }
    if (exit_pressed && launcher_ready &&
        !launcher_transitioning) {
        play_ui_close_sound();
        SDL_Event quit_event{};
        quit_event.type = SDL_QUIT;
        SDL_PushEvent(&quit_event);
    }

    float launcher_fade_alpha = 0.0f;
    bool launcher_started_this_frame = false;
    if (g_launcher_start_transition != LauncherStartTransition::None) {
        const float dt =
            std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);
        g_launcher_start_transition_elapsed += dt;

        const float fade_progress = std::clamp(
            g_launcher_start_transition_elapsed /
                kLauncherStartFadeSeconds,
            0.0f, 1.0f);
        launcher_fade_alpha = smoothstep01(fade_progress);

        if (fade_progress >= 1.0f) {
            const LauncherStartTransition requested =
                g_launcher_start_transition;
            g_launcher_start_transition = LauncherStartTransition::None;
            g_launcher_start_transition_elapsed = 0.0f;

            launcher_started_this_frame =
                requested == LauncherStartTransition::Disc
                    ? boot_disc_from_ui()
                    : start_bios_from_ui();

            // Keep this final launcher frame fully black. The next frame is
            // owned by the emulator screen if startup succeeded.
            launcher_fade_alpha = 1.0f;
            if (!launcher_started_this_frame) {
                // The boot helper has already supplied the useful error text.
                launcher_fade_alpha = 0.0f;
            }
        }
    }

    if (game_library_dirty_ ||
        (rom_directory_valid_ &&
            (SDL_GetTicks() - game_library_last_scan_ms_ > 15000u))) {
        refresh_game_library();
    }

    // The two cards rise into place, the system card just behind the library.
    const auto panel_in = [startup](int index) {
        return (startup - kPanelsStart - kPanelStagger * static_cast<float>(index)) /
            kPanelDuration;
    };
    const float library_in = panel_in(0);
    const float library_alpha = smoothstep01(library_in / 0.7f);
    const float library_y =
        585.0f + kPanelRise * (1.0f - definitive_ui::ease_out_cubic(library_in));
    const int library_vertices = draw->VtxBuffer.Size;
    draw_panel(draw, layout, 32.0f, library_y, 808.0f, 183.0f);

    draw_folder_badge(draw, layout, 53.0f, library_y + 17.0f);
    add_text(draw, layout, 86.0f, library_y + 20.0f, 16.5f,
        rgba(240, 243, 247, 255), "Game Library");
    draw->AddLine(layout.point(46.0f, library_y + 44.0f),
        layout.point(826.0f, library_y + 44.0f),
        rgba(105, 116, 128, 165), layout.px(1.0f));

    const std::string rom_label = rom_directory_valid_
        ? "ROM Directory: " + rom_directory_
        : "ROM Directory: not set";
    add_text(draw, layout, 53.0f, library_y + 52.0f, 12.8f,
        rgba(218, 223, 229, 250), rom_label.c_str());

    if (!rom_directory_valid_) {
        add_text(draw, layout, 53.0f, library_y + 87.0f, 10.2f,
            rgba(224, 74, 74, 245), "No ROM directory configured.");
        add_text(draw, layout, 53.0f, library_y + 111.0f, 10.0f,
            rgba(205, 210, 217, 238),
            "Set a ROM directory to scan and list games here.");
    }
    else if (game_library_.empty()) {
        add_text(draw, layout, 53.0f, library_y + 89.0f, 10.0f,
            rgba(207, 180, 108, 235), "No playable disc images found.");
    }
    else {
        const std::string count_label =
            std::to_string(game_library_.size()) + " games";
        add_text_right(draw, layout, 796.0f, library_y + 52.0f, 12.0f,
            rgba(213, 220, 228, 248), count_label.c_str());

        ImGui::SetCursorScreenPos(layout.point(53.0f, library_y + 76.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::PushStyleVar(
            ImGuiStyleVar_ItemSpacing, layout.size(5.0f, 2.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ScrollbarSize, layout.px(7.0f));
        ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_ScrollbarBg, definitive_list_color(rgba(4, 7, 10, 90), 0.88f));
        ImGui::PushStyleColor(ImGuiCol_ScrollbarGrab, definitive_accent_color(rgba(104, 120, 137, 145), 0.58f));
        ImGui::PushStyleColor(
            ImGuiCol_ScrollbarGrabHovered, definitive_accent_color(rgba(150, 172, 194, 190), 0.74f));
        ImGui::PushStyleColor(
            ImGuiCol_ScrollbarGrabActive, definitive_accent_color(rgba(193, 216, 238, 220), 0.88f));

        const ImGuiWindowFlags library_flags =
            ImGuiWindowFlags_NoBackground |
            (game_library_.size() > 3
                ? ImGuiWindowFlags_AlwaysVerticalScrollbar
                : ImGuiWindowFlags_None);
        // The list is its own window, so fade it through the style instead.
        ImGui::PushStyleVar(
            ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * library_alpha);
        ImGui::BeginChild(
            "##DefinitiveGameLibraryScroll",
            layout.size(755.0f, 59.0f), false, library_flags);

        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(game_library_.size()),
            layout.px(19.0f) + ImGui::GetStyle().ItemSpacing.y);
        while (clipper.Step()) {
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                const auto& entry = game_library_[static_cast<size_t>(i)];
                const bool is_selected =
                    entry.bin_path == game_bin_path_ &&
                    entry.cue_path == game_cue_path_;

                ImGui::PushID(i);
                ImGui::PushStyleColor(
                    ImGuiCol_Header, is_selected
                        ? definitive_accent_color(rgba(58, 79, 98, 125), 0.72f)
                        : IM_COL32(0, 0, 0, 0));
                ImGui::PushStyleColor(
                    ImGuiCol_HeaderHovered,
                    definitive_surface_color(rgba(53, 68, 83, 150), 0.78f));
                ImGui::PushStyleColor(
                    ImGuiCol_HeaderActive,
                    definitive_accent_color(rgba(69, 91, 111, 175), 0.76f));
                const bool chosen = ImGui::Selectable(
                    entry.title.c_str(), is_selected, 0,
                    ImVec2(0.0f, layout.px(19.0f)));
                ImGui::PopStyleColor(3);
                ImGui::PopID();

                if (chosen && launcher_ready) {
                    load_disc_from_ui(entry.bin_path, entry.cue_path);
                }
            }
        }

        ImGui::EndChild();
        ImGui::PopStyleVar();
        ImGui::PopStyleColor(5);
        ImGui::PopStyleVar(3);
    }

    if (small_button(layout, "set_rom_dir", "Set Directory",
        53.0f, library_y + 145.0f, 130.0f, 25.0f) &&
        launcher_ready) {
        play_ui_open_sound();
        const std::string selected = open_folder_dialog("Select ROM Directory");
        play_ui_close_sound();
        if (!selected.empty()) {
            rom_directory_ = selected;
            game_library_dirty_ = true;
            save_persistent_config();
            refresh_game_library();
            status_message_ = "ROM directory set: " + rom_directory_;
        }
    }
    if (small_button(layout, "refresh_rom_dir", "Refresh",
        196.0f, library_y + 145.0f, 90.0f, 25.0f, rom_directory_valid_) &&
        launcher_ready) {
        game_library_dirty_ = true;
        refresh_game_library();
    }
    definitive_ui::offset_draw_vertices(
        draw, library_vertices, ImVec2(0.0f, 0.0f), library_alpha);

    const float info_in = panel_in(1);
    const float info_alpha = smoothstep01(info_in / 0.7f);
    const float info_y =
        585.0f + kPanelRise * (1.0f - definitive_ui::ease_out_cubic(info_in));
    const int info_vertices = draw->VtxBuffer.Size;
    draw_panel(draw, layout, 854.0f, info_y, 394.0f, 183.0f);
    draw_info_badge(draw, layout, 875.0f, info_y + 17.0f);
    add_text(draw, layout, 905.0f, info_y + 19.0f, 16.5f,
        rgba(240, 243, 247, 255), "System Info");
    draw->AddLine(layout.point(868.0f, info_y + 44.0f),
        layout.point(1232.0f, info_y + 44.0f),
        rgba(105, 116, 128, 165), layout.px(1.0f));

    const ImU32 label_color = rgba(216, 222, 229, 250);
    const ImU32 value_color = rgba(244, 247, 250, 255);
    add_text(draw, layout, 875.0f, info_y + 58.0f, 12.2f,
        label_color, "Emulator:");
    add_text(draw, layout, 995.0f, info_y + 58.0f, 12.4f,
        value_color, "VibeStation");
    add_text(draw, layout, 875.0f, info_y + 77.0f, 12.2f,
        label_color, "Version:");
    add_text(draw, layout, 995.0f, info_y + 77.0f, 12.4f,
        value_color, vibestation_version_string());
    add_text(draw, layout, 875.0f, info_y + 96.0f, 12.2f,
        label_color, "BIOS:");
    add_text(draw, layout, 995.0f, info_y + 96.0f, 12.4f,
        value_color, system_->bios_loaded() ? "Loaded" : "Not loaded");
    add_text(draw, layout, 875.0f, info_y + 115.0f, 12.2f,
        label_color, "ROM Directory:");
    add_text(draw, layout, 995.0f, info_y + 115.0f, 12.4f,
        value_color, rom_directory_valid_ ? "Set" : "Not set");
    add_text(draw, layout, 875.0f, info_y + 134.0f, 12.2f,
        label_color, "Games Found:");
    const std::string games_found = std::to_string(game_library_.size());
    add_text(draw, layout, 995.0f, info_y + 134.0f, 12.4f,
        value_color, games_found.c_str());
    definitive_ui::offset_draw_vertices(
        draw, info_vertices, ImVec2(0.0f, 0.0f), info_alpha);

    // The departing icon, gliding title and colour lights go on top.
    if (!g_startup_complete) {
        definitive_ui::draw_intro_presentation(
            window_pos, window_size, startup, intro_targets);
    }

    // Launcher-to-emulator transition. Use the viewport foreground draw list
    // so the fade also covers child windows (notably the scrollable game list).
    if (launcher_fade_alpha > 0.0f || launcher_started_this_frame) {
        const int fade_alpha = glow_alpha(255.0f * launcher_fade_alpha);
        ImDrawList* fade_draw =
            ImGui::GetForegroundDrawList();
        fade_draw->AddRectFilled(
            window_pos,
            ImVec2(window_pos.x + window_size.x, window_pos.y + window_size.y),
            rgba(0, 0, 0, fade_alpha));
    }

}
