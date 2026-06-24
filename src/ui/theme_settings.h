#pragma once

#include <imgui.h>
#include <array>
#include <cstddef>

namespace ui_theme {

struct ThemeColorSlot {
    ImGuiCol color_id;
    const char* label;
    const char* key;
    ImVec4 default_color;
};

inline constexpr ThemeColorSlot kThemeColorSlots[] = {
    {ImGuiCol_WindowBg, "Window Background", "WindowBg", ImVec4(0.08f, 0.06f, 0.12f, 0.95f)},
    {ImGuiCol_TitleBg, "Title Background", "TitleBg", ImVec4(0.10f, 0.08f, 0.18f, 1.00f)},
    {ImGuiCol_TitleBgActive, "Title Active", "TitleBgActive", ImVec4(0.16f, 0.10f, 0.30f, 1.00f)},
    {ImGuiCol_MenuBarBg, "Menu Bar", "MenuBarBg", ImVec4(0.10f, 0.08f, 0.15f, 1.00f)},
    {ImGuiCol_PopupBg, "List Background", "PopupBg", ImVec4(0.09f, 0.07f, 0.16f, 0.98f)},
    {ImGuiCol_Tab, "Tab", "Tab", ImVec4(0.14f, 0.10f, 0.25f, 1.00f)},
    {ImGuiCol_TabHovered, "Tab Hovered", "TabHovered", ImVec4(0.30f, 0.20f, 0.55f, 1.00f)},
    {ImGuiCol_TabActive, "Tab Active", "TabActive", ImVec4(0.24f, 0.15f, 0.45f, 1.00f)},
    {ImGuiCol_Header, "Header", "Header", ImVec4(0.20f, 0.14f, 0.36f, 1.00f)},
    {ImGuiCol_HeaderHovered, "Header Hovered", "HeaderHovered", ImVec4(0.30f, 0.20f, 0.50f, 1.00f)},
    {ImGuiCol_HeaderActive, "Header Active", "HeaderActive", ImVec4(0.35f, 0.25f, 0.60f, 1.00f)},
    {ImGuiCol_Button, "Button", "Button", ImVec4(0.20f, 0.14f, 0.36f, 1.00f)},
    {ImGuiCol_ButtonHovered, "Button Hovered", "ButtonHovered", ImVec4(0.34f, 0.22f, 0.58f, 1.00f)},
    {ImGuiCol_ButtonActive, "Button Active", "ButtonActive", ImVec4(0.40f, 0.28f, 0.65f, 1.00f)},
    {ImGuiCol_FrameBg, "Frame Background", "FrameBg", ImVec4(0.12f, 0.09f, 0.20f, 1.00f)},
    {ImGuiCol_FrameBgHovered, "Frame Hovered", "FrameBgHovered", ImVec4(0.18f, 0.12f, 0.30f, 1.00f)},
    {ImGuiCol_FrameBgActive, "Frame Active", "FrameBgActive", ImVec4(0.24f, 0.16f, 0.40f, 1.00f)},
    {ImGuiCol_CheckMark, "Check Mark", "CheckMark", ImVec4(0.60f, 0.40f, 1.00f, 1.00f)},
    {ImGuiCol_SliderGrab, "Slider Grab", "SliderGrab", ImVec4(0.50f, 0.35f, 0.85f, 1.00f)},
    {ImGuiCol_SliderGrabActive, "Slider Grab Active", "SliderGrabActive", ImVec4(0.60f, 0.40f, 1.00f, 1.00f)},
    {ImGuiCol_Separator, "Separator", "Separator", ImVec4(0.20f, 0.15f, 0.35f, 1.00f)},
    {ImGuiCol_Text, "Text", "Text", ImVec4(0.90f, 0.88f, 0.95f, 1.00f)},
};

inline constexpr size_t kThemeColorSlotCount =
    sizeof(kThemeColorSlots) / sizeof(kThemeColorSlots[0]);

inline constexpr ImVec4 kDefaultThemeBackground = ImVec4(0.08f, 0.06f, 0.12f, 0.95f);
inline constexpr ImVec4 kDefaultThemeSurface = ImVec4(0.12f, 0.09f, 0.20f, 1.00f);
inline constexpr ImVec4 kDefaultThemeOverall = ImVec4(0.60f, 0.40f, 1.00f, 1.00f);
inline constexpr ImVec4 kDefaultThemeAccent = ImVec4(0.60f, 0.40f, 1.00f, 1.00f);
inline constexpr ImVec4 kDefaultThemeText = ImVec4(0.90f, 0.88f, 0.95f, 1.00f);
inline constexpr ImVec4 kDefaultThemeLists = ImVec4(0.09f, 0.07f, 0.16f, 0.98f);

struct ThemeSettings {
    ImVec4 overall = kDefaultThemeOverall;
    ImVec4 background = kDefaultThemeBackground;
    ImVec4 surface = kDefaultThemeSurface;
    ImVec4 accent = kDefaultThemeAccent;
    ImVec4 text = kDefaultThemeText;
    ImVec4 lists = kDefaultThemeLists;
    ImVec4 startup_title = kDefaultThemeAccent;
    ImVec4 startup_text = kDefaultThemeText;
    bool simple = false;
    bool advanced = false;
    std::array<ImVec4, kThemeColorSlotCount> colors{};
};

struct ThemePreset {
    const char* label;
    ImVec4 background;
    ImVec4 surface;
    ImVec4 accent;
    ImVec4 text;
    ImVec4 lists;
    std::array<ImVec4, kThemeColorSlotCount> colors;
};

extern ThemeSettings g_theme_settings;
extern int g_selected_theme_preset_index;

int theme_preset_count();
const ThemePreset& theme_preset_by_index(int index);

void ensure_theme_settings_initialized();
void mark_theme_settings_dirty();
void apply_theme_style(ImGuiStyle& style);
void register_theme_settings_handler();
void reset_theme_settings();
void apply_theme_preset_by_index(int index);
ImVec4 current_startup_title_color(const ThemeSettings& settings);
ImVec4 current_startup_text_color(const ThemeSettings& settings);
ImVec4 theme_lerp(const ImVec4& a, const ImVec4& b, float t);
void rebuild_theme_basics_from_overall(ThemeSettings& settings);
void sync_theme_overall_from_basics(ThemeSettings& settings);
void reset_theme_startup_colors(ThemeSettings& settings);
void rebuild_theme_colors_from_basics(ThemeSettings& settings);

} // namespace ui_theme
