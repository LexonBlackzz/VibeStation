#include "ui/theme_settings.h"

#include <imgui.h>
#include <imgui_internal.h>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <string>

namespace ui_theme {

namespace {

constexpr ImVec4 theme_color_rgba(int r, int g, int b, int a) {
    return ImVec4(
        static_cast<float>(r) / 255.0f,
        static_cast<float>(g) / 255.0f,
        static_cast<float>(b) / 255.0f,
        static_cast<float>(a) / 255.0f);
}

constexpr ThemePreset kThemePresets[] = {
    {
        "Dark Mode",
        theme_color_rgba(0, 0, 0, 242),
        theme_color_rgba(20, 20, 20, 255),
        theme_color_rgba(41, 40, 40, 255),
        theme_color_rgba(255, 255, 255, 255),
        theme_color_rgba(12, 12, 12, 252),
        {
            theme_color_rgba(0, 0, 0, 242),
            theme_color_rgba(7, 7, 7, 247),
            theme_color_rgba(28, 28, 28, 255),
            theme_color_rgba(4, 4, 4, 245),
            theme_color_rgba(12, 12, 12, 252),
            theme_color_rgba(17, 17, 17, 253),
            theme_color_rgba(35, 34, 34, 255),
            theme_color_rgba(30, 30, 30, 255),
            theme_color_rgba(26, 26, 26, 255),
            theme_color_rgba(31, 31, 31, 255),
            theme_color_rgba(35, 34, 34, 255),
            theme_color_rgba(26, 25, 25, 255),
            theme_color_rgba(32, 32, 32, 255),
            theme_color_rgba(36, 36, 36, 255),
            theme_color_rgba(20, 20, 20, 255),
            theme_color_rgba(25, 25, 25, 255),
            theme_color_rgba(28, 28, 28, 255),
            theme_color_rgba(41, 40, 40, 255),
            theme_color_rgba(32, 32, 32, 255),
            theme_color_rgba(41, 40, 40, 255),
            theme_color_rgba(62, 62, 62, 255),
            theme_color_rgba(255, 255, 255, 255),
        },
    },
    {
        "Light Mode",
        theme_color_rgba(255, 255, 255, 242),
        theme_color_rgba(221, 221, 221, 255),
        theme_color_rgba(200, 200, 200, 255),
        theme_color_rgba(0, 0, 0, 255),
        theme_color_rgba(235, 235, 235, 252),
        {
            theme_color_rgba(255, 255, 255, 242),
            theme_color_rgba(243, 243, 243, 247),
            theme_color_rgba(213, 213, 213, 255),
            theme_color_rgba(248, 248, 248, 245),
            theme_color_rgba(235, 235, 235, 252),
            theme_color_rgba(226, 226, 226, 253),
            theme_color_rgba(206, 206, 206, 255),
            theme_color_rgba(211, 211, 211, 255),
            theme_color_rgba(215, 215, 215, 255),
            theme_color_rgba(210, 210, 210, 255),
            theme_color_rgba(206, 206, 206, 255),
            theme_color_rgba(215, 215, 215, 255),
            theme_color_rgba(209, 209, 209, 255),
            theme_color_rgba(205, 205, 205, 255),
            theme_color_rgba(221, 221, 221, 255),
            theme_color_rgba(216, 216, 216, 255),
            theme_color_rgba(213, 213, 213, 255),
            theme_color_rgba(200, 200, 200, 255),
            theme_color_rgba(209, 209, 209, 255),
            theme_color_rgba(200, 200, 200, 255),
            theme_color_rgba(181, 181, 181, 255),
            theme_color_rgba(0, 0, 0, 255),
        },
    },
};

constexpr int kThemePresetCount =
    static_cast<int>(sizeof(kThemePresets) / sizeof(kThemePresets[0]));

float theme_luminance(const ImVec4& color) {
    return (color.x * 0.2126f) + (color.y * 0.7152f) + (color.z * 0.0722f);
}

ImVec4 tint_theme_color(const ImVec4& base, const ImVec4& tint, float mix_val, float alpha) {
    ImVec4 out = theme_lerp(base, tint, mix_val);
    out.w = alpha;
    return out;
}

void set_theme_slot(ThemeSettings& settings, ImGuiCol color_id, const ImVec4& color) {
    for (size_t i = 0; i < kThemeColorSlotCount; ++i) {
        if (kThemeColorSlots[i].color_id == color_id) {
            settings.colors[i] = color;
            return;
        }
    }
}

ImVec4 startup_title_theme_color(const ThemeSettings& settings) {
    const float luma = theme_luminance(settings.overall);
    return tint_theme_color(
        settings.accent, settings.overall, luma >= 0.72f ? 0.55f : 0.82f, 1.00f);
}

ImVec4 startup_text_theme_color(const ThemeSettings& settings) {
    const float luma = theme_luminance(settings.overall);
    const ImVec4 tinted_text = tint_theme_color(
        settings.text, settings.overall, luma >= 0.72f ? 0.24f : 0.36f, 1.00f);
    return theme_lerp(
        tinted_text, settings.background, luma >= 0.72f ? 0.08f : 0.18f);
}

void apply_theme_preset_impl(const ThemePreset& preset) {
    g_theme_settings.overall = preset.accent;
    g_theme_settings.background = preset.background;
    g_theme_settings.surface = preset.surface;
    g_theme_settings.accent = preset.accent;
    g_theme_settings.text = preset.text;
    g_theme_settings.lists = preset.lists;
    reset_theme_startup_colors(g_theme_settings);
    g_theme_settings.simple = false;
    g_theme_settings.advanced = true;
    g_theme_settings.colors = preset.colors;
}

bool parse_theme_color(const char* text, ImVec4& color) {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float a = 0.0f;
    if (std::sscanf(text, "%f,%f,%f,%f", &r, &g, &b, &a) != 4) {
        return false;
    }
    color = ImVec4(r, g, b, a);
    return true;
}

bool parse_theme_bool(const char* text, bool fallback) {
    if (std::strcmp(text, "1") == 0 || std::strcmp(text, "true") == 0 ||
        std::strcmp(text, "TRUE") == 0) {
        return true;
    }
    if (std::strcmp(text, "0") == 0 || std::strcmp(text, "false") == 0 ||
        std::strcmp(text, "FALSE") == 0) {
        return false;
    }
    return fallback;
}

bool g_theme_settings_initialized = false;

void* theme_settings_read_open(ImGuiContext*, ImGuiSettingsHandler*, const char* name) {
    if (std::strcmp(name, "Colors") != 0) {
        return nullptr;
    }
    ensure_theme_settings_initialized();
    return &g_theme_settings;
}

void theme_settings_read_line(ImGuiContext*, ImGuiSettingsHandler*, void* entry,
    const char* line) {
    auto* settings = static_cast<ThemeSettings*>(entry);
    const char* equals = std::strchr(line, '=');
    if (equals == nullptr) {
        return;
    }

    const std::string key(line, static_cast<size_t>(equals - line));
    if (key == "Overall") {
        parse_theme_color(equals + 1, settings->overall);
        return;
    }
    if (key == "Background") {
        parse_theme_color(equals + 1, settings->background);
        return;
    }
    if (key == "Surface") {
        parse_theme_color(equals + 1, settings->surface);
        return;
    }
    if (key == "Accent") {
        parse_theme_color(equals + 1, settings->accent);
        return;
    }
    if (key == "TextColor") {
        parse_theme_color(equals + 1, settings->text);
        return;
    }
    if (key == "Lists") {
        parse_theme_color(equals + 1, settings->lists);
        return;
    }
    if (key == "StartupTitle") {
        parse_theme_color(equals + 1, settings->startup_title);
        return;
    }
    if (key == "StartupText") {
        parse_theme_color(equals + 1, settings->startup_text);
        return;
    }
    if (key == "Simple") {
        settings->simple = parse_theme_bool(equals + 1, settings->simple);
        return;
    }
    if (key == "Advanced") {
        settings->advanced = parse_theme_bool(equals + 1, settings->advanced);
        return;
    }

    ImVec4 parsed{};
    if (!parse_theme_color(equals + 1, parsed)) {
        return;
    }
    for (size_t i = 0; i < kThemeColorSlotCount; ++i) {
        if (key == kThemeColorSlots[i].key) {
            settings->colors[i] = parsed;
            return;
        }
    }
}

void theme_settings_apply_all(ImGuiContext*, ImGuiSettingsHandler*) {
    ensure_theme_settings_initialized();
    if (!g_theme_settings.advanced) {
        rebuild_theme_colors_from_basics(g_theme_settings);
    }
    apply_theme_style(ImGui::GetStyle());
}

void theme_settings_write_all(ImGuiContext*, ImGuiSettingsHandler* handler,
    ImGuiTextBuffer* out_buf) {
    ensure_theme_settings_initialized();
    out_buf->appendf("[%s][Colors]\n", handler->TypeName);
    out_buf->appendf("Overall=%.3f,%.3f,%.3f,%.3f\n",
        g_theme_settings.overall.x, g_theme_settings.overall.y,
        g_theme_settings.overall.z, g_theme_settings.overall.w);
    out_buf->appendf("Background=%.3f,%.3f,%.3f,%.3f\n",
        g_theme_settings.background.x, g_theme_settings.background.y,
        g_theme_settings.background.z, g_theme_settings.background.w);
    out_buf->appendf("Surface=%.3f,%.3f,%.3f,%.3f\n",
        g_theme_settings.surface.x, g_theme_settings.surface.y,
        g_theme_settings.surface.z, g_theme_settings.surface.w);
    out_buf->appendf("Accent=%.3f,%.3f,%.3f,%.3f\n",
        g_theme_settings.accent.x, g_theme_settings.accent.y,
        g_theme_settings.accent.z, g_theme_settings.accent.w);
    out_buf->appendf("TextColor=%.3f,%.3f,%.3f,%.3f\n",
        g_theme_settings.text.x, g_theme_settings.text.y,
        g_theme_settings.text.z, g_theme_settings.text.w);
    out_buf->appendf("Lists=%.3f,%.3f,%.3f,%.3f\n",
        g_theme_settings.lists.x, g_theme_settings.lists.y,
        g_theme_settings.lists.z, g_theme_settings.lists.w);
    out_buf->appendf("StartupTitle=%.3f,%.3f,%.3f,%.3f\n",
        g_theme_settings.startup_title.x, g_theme_settings.startup_title.y,
        g_theme_settings.startup_title.z, g_theme_settings.startup_title.w);
    out_buf->appendf("StartupText=%.3f,%.3f,%.3f,%.3f\n",
        g_theme_settings.startup_text.x, g_theme_settings.startup_text.y,
        g_theme_settings.startup_text.z, g_theme_settings.startup_text.w);
    out_buf->appendf("Simple=%d\n", g_theme_settings.simple ? 1 : 0);
    out_buf->appendf("Advanced=%d\n", g_theme_settings.advanced ? 1 : 0);
    for (size_t i = 0; i < kThemeColorSlotCount; ++i) {
        const ImVec4& color = g_theme_settings.colors[i];
        out_buf->appendf("%s=%.3f,%.3f,%.3f,%.3f\n",
            kThemeColorSlots[i].key, color.x, color.y, color.z, color.w);
    }
    out_buf->append("\n");
}

} // anonymous namespace

ThemeSettings g_theme_settings{};
int g_selected_theme_preset_index = 0;

int theme_preset_count() {
    return kThemePresetCount;
}

const ThemePreset& theme_preset_by_index(int index) {
    return kThemePresets[index];
}

ImVec4 theme_lerp(const ImVec4& a, const ImVec4& b, float t) {
    return ImVec4(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
        a.w + (b.w - a.w) * t);
}

void rebuild_theme_basics_from_overall(ThemeSettings& settings) {
    const float luma = theme_luminance(settings.overall);
    const bool light_theme = luma >= 0.72f;

    const ImVec4 background_base =
        light_theme ? ImVec4(0.97f, 0.97f, 0.98f, 0.95f)
        : ImVec4(0.05f, 0.05f, 0.06f, 0.95f);
    const ImVec4 surface_base =
        light_theme ? ImVec4(0.88f, 0.88f, 0.90f, 1.00f)
        : ImVec4(0.12f, 0.11f, 0.14f, 1.00f);
    const ImVec4 accent_base =
        light_theme ? ImVec4(0.78f, 0.78f, 0.80f, 1.00f)
        : ImVec4(0.32f, 0.30f, 0.36f, 1.00f);
    const ImVec4 lists_base =
        light_theme ? ImVec4(0.92f, 0.92f, 0.94f, 0.98f)
        : ImVec4(0.08f, 0.08f, 0.10f, 0.98f);
    const ImVec4 text_base =
        light_theme ? ImVec4(0.04f, 0.04f, 0.05f, 1.00f)
        : ImVec4(0.96f, 0.96f, 0.97f, 1.00f);

    settings.background =
        tint_theme_color(background_base, settings.overall, light_theme ? 0.18f : 0.16f, 0.95f);
    settings.surface =
        tint_theme_color(surface_base, settings.overall, light_theme ? 0.26f : 0.24f, 1.00f);
    settings.accent =
        tint_theme_color(accent_base, settings.overall, light_theme ? 0.70f : 0.88f, 1.00f);
    settings.lists =
        tint_theme_color(lists_base, settings.overall, light_theme ? 0.20f : 0.18f, 0.98f);
    settings.text =
        tint_theme_color(text_base, settings.overall, light_theme ? 0.06f : 0.08f, 1.00f);
}

void sync_theme_overall_from_basics(ThemeSettings& settings) {
    settings.overall = theme_lerp(settings.surface, settings.accent, 0.75f);
    settings.overall.w = 1.0f;
}

void reset_theme_startup_colors(ThemeSettings& settings) {
    settings.startup_title = startup_title_theme_color(settings);
    settings.startup_text = startup_text_theme_color(settings);
}

ImVec4 current_startup_title_color(const ThemeSettings& settings) {
    return settings.advanced ? settings.startup_title
                             : startup_title_theme_color(settings);
}

ImVec4 current_startup_text_color(const ThemeSettings& settings) {
    return settings.advanced ? settings.startup_text
                             : startup_text_theme_color(settings);
}

void rebuild_theme_colors_from_basics(ThemeSettings& settings) {
    set_theme_slot(settings, ImGuiCol_WindowBg, settings.background);
    set_theme_slot(settings, ImGuiCol_TitleBg,
        theme_lerp(settings.background, settings.surface, 0.35f));
    set_theme_slot(settings, ImGuiCol_TitleBgActive,
        theme_lerp(settings.surface, settings.accent, 0.40f));
    set_theme_slot(settings, ImGuiCol_MenuBarBg,
        theme_lerp(settings.background, settings.surface, 0.20f));
    set_theme_slot(settings, ImGuiCol_PopupBg, settings.lists);
    set_theme_slot(settings, ImGuiCol_Tab,
        theme_lerp(settings.surface, settings.background, 0.15f));
    set_theme_slot(settings, ImGuiCol_TabHovered,
        theme_lerp(settings.surface, settings.accent, 0.72f));
    set_theme_slot(settings, ImGuiCol_TabActive,
        theme_lerp(settings.surface, settings.accent, 0.50f));
    set_theme_slot(settings, ImGuiCol_Header,
        theme_lerp(settings.surface, settings.accent, 0.30f));
    set_theme_slot(settings, ImGuiCol_HeaderHovered,
        theme_lerp(settings.surface, settings.accent, 0.55f));
    set_theme_slot(settings, ImGuiCol_HeaderActive,
        theme_lerp(settings.surface, settings.accent, 0.72f));
    set_theme_slot(settings, ImGuiCol_Button,
        theme_lerp(settings.surface, settings.accent, 0.28f));
    set_theme_slot(settings, ImGuiCol_ButtonHovered,
        theme_lerp(settings.surface, settings.accent, 0.60f));
    set_theme_slot(settings, ImGuiCol_ButtonActive,
        theme_lerp(settings.surface, settings.accent, 0.78f));
    set_theme_slot(settings, ImGuiCol_FrameBg, settings.surface);
    set_theme_slot(settings, ImGuiCol_FrameBgHovered,
        theme_lerp(settings.surface, settings.accent, 0.25f));
    set_theme_slot(settings, ImGuiCol_FrameBgActive,
        theme_lerp(settings.surface, settings.accent, 0.40f));
    set_theme_slot(settings, ImGuiCol_CheckMark, settings.accent);
    set_theme_slot(settings, ImGuiCol_SliderGrab,
        theme_lerp(settings.surface, settings.accent, 0.60f));
    set_theme_slot(settings, ImGuiCol_SliderGrabActive, settings.accent);
    set_theme_slot(settings, ImGuiCol_Separator,
        theme_lerp(settings.surface, settings.text, 0.18f));
    set_theme_slot(settings, ImGuiCol_Text, settings.text);
}

void reset_theme_settings() {
    g_theme_settings.overall = kDefaultThemeOverall;
    g_theme_settings.background = kDefaultThemeBackground;
    g_theme_settings.surface = kDefaultThemeSurface;
    g_theme_settings.accent = kDefaultThemeAccent;
    g_theme_settings.text = kDefaultThemeText;
    g_theme_settings.lists = kDefaultThemeLists;
    reset_theme_startup_colors(g_theme_settings);
    g_theme_settings.simple = false;
    g_theme_settings.advanced = false;
    rebuild_theme_colors_from_basics(g_theme_settings);
    g_theme_settings_initialized = true;
}

void apply_theme_preset_by_index(int index) {
    apply_theme_preset_impl(kThemePresets[index]);
}

void apply_theme_style(ImGuiStyle& style) {
    style.WindowRounding = 6.0f;
    style.FrameRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.TabRounding = 4.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.ScrollbarRounding = 6.0f;
    style.WindowPadding = ImVec2(10.0f, 10.0f);

    for (size_t i = 0; i < kThemeColorSlotCount; ++i) {
        style.Colors[kThemeColorSlots[i].color_id] = g_theme_settings.colors[i];
    }
}

void ensure_theme_settings_initialized() {
    if (!g_theme_settings_initialized) {
        reset_theme_settings();
    }
}

void mark_theme_settings_dirty() {
    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui::MarkIniSettingsDirty();
    }
}

void register_theme_settings_handler() {
    ImGuiContext* ctx = ImGui::GetCurrentContext();
    if (ctx == nullptr) {
        return;
    }

    const ImGuiID type_hash = ImHashStr("VibeStationTheme");
    for (const ImGuiSettingsHandler& handler : ctx->SettingsHandlers) {
        if (handler.TypeHash == type_hash) {
            return;
        }
    }

    ImGuiSettingsHandler handler{};
    handler.TypeName = "VibeStationTheme";
    handler.TypeHash = type_hash;
    handler.ReadOpenFn = theme_settings_read_open;
    handler.ReadLineFn = theme_settings_read_line;
    handler.ApplyAllFn = theme_settings_apply_all;
    handler.WriteAllFn = theme_settings_write_all;
    ImGui::AddSettingsHandler(&handler);
}

} // namespace ui_theme
