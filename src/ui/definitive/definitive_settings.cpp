#include "ui/app.h"
#include "ui/definitive/definitive_shared.h"
#include "ui/output_resolution_utils.h"
#include "ui/theme_settings.h"
#include "version.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <string>

namespace {

using namespace definitive_ui;

constexpr float kDefinitiveSettingsOpenDuration = 0.34f;
constexpr float kDefinitiveSettingsCloseDuration = 0.26f;

void draw_settings_section(ImDrawList* draw, const Layout& layout,
    float x, float y, float w, float h, const char* title) {
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 p1 = layout.point(x + w, y + h);
    draw->AddRectFilled(
        p0, p1, surface_color(rgba(7, 11, 16, 226), 0.92f),
        layout.px(4.0f));
    draw->AddRect(
        p0, p1, accent_color(rgba(91, 109, 126, 175), 0.62f),
        layout.px(4.0f), 0, layout.px(1.0f));
    add_text(draw, layout, x + 18.0f, y + 11.0f, 15.5f,
        rgba(209, 218, 227, 235), title);
    draw->AddLine(
        layout.point(x + 18.0f, y + 39.0f),
        layout.point(x + w - 18.0f, y + 39.0f),
        accent_color(rgba(78, 92, 106, 145), 0.52f),
        layout.px(1.0f));
}

bool definitive_settings_switch(ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w, bool& value) {
    constexpr float kHeight = 68.0f;
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 row_size = layout.size(w, kHeight);

    ImGui::SetCursorScreenPos(p0);
    ImGui::PushID(id);
    const bool pressed = ImGui::InvisibleButton("##settings_switch", row_size);
    const bool hovered = ImGui::IsItemHovered();
    if (pressed) {
        value = !value;
    }

    if (hovered) {
        draw->AddRectFilled(
            p0, ImVec2(p0.x + row_size.x, p0.y + row_size.y),
            surface_color(rgba(31, 43, 55, 92), 0.78f), layout.px(2.0f));
    }

    add_text(draw, layout, x + 18.0f, y + 6.0f, 16.5f,
        hovered ? rgba(240, 244, 248, 255) : rgba(221, 226, 232, 244),
        label);

    const ImVec2 track0 = layout.point(x + w - 63.0f, y + 18.0f);
    const ImVec2 track1 = layout.point(x + w - 19.0f, y + 40.0f);
    draw->AddRectFilled(
        track0, track1,
        value
            ? accent_color(rgba(78, 126, 166, 235), 0.90f)
            : surface_color(rgba(55, 62, 70, 230), 0.72f),
        layout.px(11.0f));

    const float knob_x = value ? (x + w - 31.0f) : (x + w - 51.0f);
    draw->AddCircleFilled(
        layout.point(knob_x, y + 29.0f),
        layout.px(8.0f),
        value ? rgba(232, 240, 247, 255) : rgba(178, 184, 191, 245),
        20);

    ImGui::PopID();
    return pressed;
}

bool definitive_settings_action(ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w) {
    constexpr float kHeight = 68.0f;
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 row_size = layout.size(w, kHeight);

    ImGui::SetCursorScreenPos(p0);
    ImGui::PushID(id);
    const bool pressed = ImGui::InvisibleButton("##settings_action", row_size);
    const bool hovered = ImGui::IsItemHovered();

    if (hovered) {
        draw->AddRectFilled(
            p0, ImVec2(p0.x + row_size.x, p0.y + row_size.y),
            surface_color(rgba(31, 43, 55, 92), 0.78f), layout.px(2.0f));
    }

    add_text(draw, layout, x + 18.0f, y + 6.0f, 16.5f,
        hovered ? rgba(240, 244, 248, 255) : rgba(221, 226, 232, 244),
        label);

    const ImU32 arrow_color = text_color(
        hovered ? rgba(226, 237, 247, 250) : rgba(145, 158, 170, 220));
    const ImVec2 a = layout.point(x + w - 34.0f, y + 22.0f);
    draw->AddLine(a, layout.point(x + w - 27.0f, y + 29.0f),
        arrow_color, layout.px(1.5f));
    draw->AddLine(layout.point(x + w - 27.0f, y + 29.0f),
        layout.point(x + w - 34.0f, y + 36.0f),
        arrow_color, layout.px(1.5f));

    ImGui::PopID();
    return pressed;
}

bool definitive_settings_combo(ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w,
    int& current, const char* const items[], int item_count) {
    constexpr float kHeight = 68.0f;
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 row_size = layout.size(w, kHeight);

    const bool row_hovered = ImGui::IsMouseHoveringRect(
        p0, ImVec2(p0.x + row_size.x, p0.y + row_size.y), false);

    if (row_hovered) {
        draw->AddRectFilled(
            p0, ImVec2(p0.x + row_size.x, p0.y + row_size.y),
            surface_color(rgba(31, 43, 55, 60), 0.78f), layout.px(2.0f));
    }

    add_text(draw, layout, x + 18.0f, y + 6.0f, 16.5f,
        rgba(221, 226, 232, 244), label);

    ImGui::SetCursorScreenPos(layout.point(x + w - 196.0f, y + 14.0f));
    ImGui::PushID(id);
    ImGui::SetNextItemWidth(layout.px(176.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, layout.px(2.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, layout.size(8.0f, 6.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,
        surface_color(rgba(14, 20, 27, 245), 0.90f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,
        surface_color(rgba(25, 35, 45, 250), 0.74f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive,
        accent_color(rgba(30, 42, 54, 255), 0.72f));
    ImGui::PushStyleColor(ImGuiCol_PopupBg,
        list_color(rgba(8, 12, 17, 252), 0.94f));
    ImGui::PushStyleColor(ImGuiCol_Border,
        accent_color(rgba(96, 118, 138, 195), 0.62f));
    ImGui::PushStyleColor(ImGuiCol_Text,
        text_color(rgba(231, 236, 241, 250)));
    ImGui::PushStyleColor(ImGuiCol_Header,
        accent_color(rgba(54, 77, 97, 215), 0.72f));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered,
        accent_color(rgba(69, 96, 120, 230), 0.84f));
    ImGui::PushFont(font_for_size(layout.px(16.0f)));
    const bool changed =
        ImGui::Combo("##value", &current, items, item_count);
    ImGui::PopFont();
    ImGui::PopStyleColor(8);
    ImGui::PopStyleVar(2);
    ImGui::PopID();
    return changed;
}

void definitive_settings_note(
    ImDrawList* draw, const Layout& layout,
    float x, float y, const char* text) {
    const float font_size = layout.px(13.5f);
    const ImVec2 pos = layout.point(x, y);

    // Keep descriptions on the text side of the row. They may wrap to a
    // second line, but are clipped before the right-aligned slider/combo area.
    const float wrap_width = layout.px(292.0f);
    const ImVec4 clip_rect(
        pos.x,
        pos.y,
        pos.x + wrap_width,
        pos.y + layout.px(34.0f));

    ImFont* font = font_for_size(font_size);
    draw->AddText(
        font,
        font_size,
        pos,
        text_color(rgba(171, 181, 191, 235)),
        text,
        nullptr,
        wrap_width,
        &clip_rect);
}

bool definitive_settings_color(
    ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w, ImVec4& value) {
    add_text(draw, layout, x + 18.0f, y + 6.0f, 16.5f,
        rgba(221, 226, 232, 244), label);

    ImGui::SetCursorScreenPos(layout.point(x + w - 206.0f, y + 12.0f));
    ImGui::PushID(id);
    ImGui::SetNextItemWidth(layout.px(186.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, layout.px(2.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,
        surface_color(rgba(14, 20, 27, 245), 0.90f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,
        surface_color(rgba(25, 35, 45, 250), 0.74f));
    ImGui::PushStyleColor(ImGuiCol_Border,
        accent_color(rgba(96, 118, 138, 195), 0.62f));
    ImGui::PushFont(font_for_size(layout.px(16.0f)));
    const bool changed = ImGui::ColorEdit4(
        "##value", &value.x,
        ImGuiColorEditFlags_DisplayRGB |
        ImGuiColorEditFlags_AlphaBar |
        ImGuiColorEditFlags_NoInputs);
    ImGui::PopFont();
    ImGui::PopStyleColor(3);
    ImGui::PopStyleVar();
    ImGui::PopID();
    return changed;
}

bool definitive_settings_tab_button(
    ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w, bool selected) {
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 sz = layout.size(w, 40.0f);
    ImGui::SetCursorScreenPos(p0);
    ImGui::PushID(id);
    const bool pressed = ImGui::InvisibleButton("##settings_tab", sz);
    const bool hovered = ImGui::IsItemHovered();
    ImGui::PopID();

    if (selected || hovered) {
        draw->AddRectFilled(
            p0, ImVec2(p0.x + sz.x, p0.y + sz.y),
            selected
                ? surface_color(rgba(28, 42, 55, 220), 0.86f)
                : surface_color(rgba(28, 39, 50, 130), 0.72f),
            layout.px(2.0f));
    }
    if (selected) {
        draw->AddRectFilled(
            layout.point(x, y + 38.0f),
            layout.point(x + w, y + 40.0f),
            accent_color(rgba(175, 210, 238, 235), 0.92f));
    }

    add_text(draw, layout, x + 12.0f, y + 10.0f, 14.5f,
        selected ? rgba(239, 244, 248, 255)
                 : rgba(174, 184, 194, hovered ? 244 : 215),
        label);
    return pressed;
}

bool definitive_settings_slider_int(
    ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w,
    int& value, int min_value, int max_value,
    const char* format) {
    constexpr float kHeight = 68.0f;
    add_text(draw, layout, x + 18.0f, y + 6.0f, 16.5f,
        rgba(221, 226, 232, 244), label);

    ImGui::SetCursorScreenPos(layout.point(x + w - 206.0f, y + 14.0f));
    ImGui::PushID(id);
    ImGui::SetNextItemWidth(layout.px(186.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, layout.px(2.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, layout.size(7.0f, 5.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,
        surface_color(rgba(14, 20, 27, 245), 0.90f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,
        surface_color(rgba(25, 35, 45, 250), 0.74f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive,
        accent_color(rgba(30, 42, 54, 255), 0.72f));
    ImGui::PushStyleColor(ImGuiCol_SliderGrab,
        accent_color(rgba(167, 201, 230, 235), 0.88f));
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive,
        accent_color(rgba(219, 235, 248, 255), 0.94f));
    ImGui::PushStyleColor(ImGuiCol_Text,
        text_color(rgba(231, 236, 241, 250)));
    ImGui::PushFont(font_for_size(layout.px(16.0f)));
    const bool changed =
        ImGui::SliderInt("##value", &value, min_value, max_value, format);
    ImGui::PopFont();
    ImGui::PopStyleColor(6);
    ImGui::PopStyleVar(2);
    ImGui::PopID();
    return changed;
}

bool definitive_settings_slider_float(
    ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w,
    float& value, float min_value, float max_value,
    const char* format) {
    add_text(draw, layout, x + 18.0f, y + 6.0f, 16.5f,
        rgba(221, 226, 232, 244), label);

    ImGui::SetCursorScreenPos(layout.point(x + w - 206.0f, y + 14.0f));
    ImGui::PushID(id);
    ImGui::SetNextItemWidth(layout.px(186.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, layout.px(2.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, layout.size(7.0f, 5.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,
        surface_color(rgba(14, 20, 27, 245), 0.90f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,
        surface_color(rgba(25, 35, 45, 250), 0.74f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive,
        accent_color(rgba(30, 42, 54, 255), 0.72f));
    ImGui::PushStyleColor(ImGuiCol_SliderGrab,
        accent_color(rgba(167, 201, 230, 235), 0.88f));
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive,
        accent_color(rgba(219, 235, 248, 255), 0.94f));
    ImGui::PushStyleColor(ImGuiCol_Text,
        text_color(rgba(231, 236, 241, 250)));
    ImGui::PushFont(font_for_size(layout.px(16.0f)));
    const bool changed =
        ImGui::SliderFloat("##value", &value, min_value, max_value, format);
    ImGui::PopFont();
    ImGui::PopStyleColor(6);
    ImGui::PopStyleVar(2);
    ImGui::PopID();
    return changed;
}

} // namespace

void App::open_definitive_settings() {
    definitive_detailed_settings_ = false;
    show_settings_ = true;
    definitive_settings_transition_ =
        DefinitiveSettingsTransition::Opening;
    definitive_settings_transition_elapsed_ = 0.0f;
}

void App::close_definitive_settings() {
    if (!show_settings_ ||
        definitive_settings_transition_ ==
            DefinitiveSettingsTransition::Closing) {
        return;
    }

    definitive_settings_transition_ =
        DefinitiveSettingsTransition::Closing;
    definitive_settings_transition_elapsed_ = 0.0f;
}

void App::panel_definitive_settings() {
    if (definitive_settings_transition_ ==
        DefinitiveSettingsTransition::Closed) {
        definitive_settings_transition_ =
            DefinitiveSettingsTransition::Opening;
        definitive_settings_transition_elapsed_ = 0.0f;
    }

    const float dt =
        std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);

    if (definitive_settings_transition_ ==
        DefinitiveSettingsTransition::Opening) {
        definitive_settings_transition_elapsed_ += dt;
        if (definitive_settings_transition_elapsed_ >=
            kDefinitiveSettingsOpenDuration) {
            definitive_settings_transition_elapsed_ =
                kDefinitiveSettingsOpenDuration;
            definitive_settings_transition_ =
                DefinitiveSettingsTransition::Open;
        }
    }
    else if (definitive_settings_transition_ ==
        DefinitiveSettingsTransition::Closing) {
        definitive_settings_transition_elapsed_ += dt;
        if (definitive_settings_transition_elapsed_ >=
            kDefinitiveSettingsCloseDuration) {
            definitive_settings_transition_ =
                DefinitiveSettingsTransition::Closed;
            definitive_settings_transition_elapsed_ = 0.0f;
            show_settings_ = false;
            definitive_detailed_settings_ = false;
            return;
        }
    }

    float transition_visibility = 1.0f;
    if (definitive_settings_transition_ ==
        DefinitiveSettingsTransition::Opening) {
        transition_visibility = smoothstep01(
            definitive_settings_transition_elapsed_ /
            kDefinitiveSettingsOpenDuration);
    }
    else if (definitive_settings_transition_ ==
        DefinitiveSettingsTransition::Closing) {
        transition_visibility = 1.0f - smoothstep01(
            definitive_settings_transition_elapsed_ /
            kDefinitiveSettingsCloseDuration);
    }

    const float transition_scale =
        0.972f + 0.028f * transition_visibility;
    const float transition_y_offset =
        (1.0f - transition_visibility) * 14.0f;

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoScrollWithMouse |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoBackground;

    if (definitive_settings_transition_ !=
        DefinitiveSettingsTransition::Open) {
        flags |= ImGuiWindowFlags_NoInputs;
    }

    ImGui::Begin("##DefinitiveSettingsOverlay", nullptr, flags);
    ImGui::PopStyleVar(3);

    ImDrawList* draw = ImGui::GetWindowDrawList();
    const int animation_vertex_start = draw->VtxBuffer.Size;
    const ImVec2 window_pos = ImGui::GetWindowPos();
    const ImVec2 window_size = ImGui::GetWindowSize();
    const ImVec2 window_end(
        window_pos.x + window_size.x,
        window_pos.y + window_size.y);
    const Layout layout = make_layout(window_pos, window_size);

    draw_settings_background(
        draw,
        window_pos,
        window_size);
    draw->AddRectFilled(
        window_pos, window_end,
        background_color(rgba(0, 2, 6, 148), 0.80f));

    constexpr float panel_x = 96.0f;
    constexpr float panel_y = 46.0f;
    constexpr float panel_w = 1088.0f;
    constexpr float panel_h = 708.0f;

    const ImVec2 panel0 = layout.point(panel_x, panel_y);
    const ImVec2 panel1 = layout.point(panel_x + panel_w, panel_y + panel_h);

    draw->AddRectFilled(
        layout.point(panel_x - 10.0f, panel_y + 10.0f),
        layout.point(panel_x + panel_w + 10.0f, panel_y + panel_h + 10.0f),
        rgba(0, 0, 0, 76), layout.px(8.0f));
    draw->AddRectFilled(
        panel0, panel1,
        surface_color(rgba(5, 9, 14, 245), 0.94f),
        layout.px(5.0f));
    draw->AddRect(
        panel0, panel1,
        accent_color(rgba(111, 132, 151, 218), 0.64f),
        layout.px(5.0f), 0, layout.px(1.0f));

    add_text(draw, layout, panel_x + 38.0f, panel_y + 21.0f, 38.0f,
        rgba(237, 241, 245, 255), "SETTINGS");
    add_text(draw, layout, panel_x + 40.0f, panel_y + 67.0f, 13.5f,
        rgba(143, 154, 165, 228), "DEFINITIVE");

    constexpr std::array<ImU32, 4> settings_accents = {
        IM_COL32(194, 44, 56, 235),
        IM_COL32(52, 128, 125, 235),
        IM_COL32(177, 145, 72, 235),
        IM_COL32(52, 93, 157, 235),
    };
    for (int i = 0; i < 4; ++i) {
        draw->AddRectFilled(
            layout.point(panel_x + 39.0f + i * 23.0f, panel_y + 87.0f),
            layout.point(panel_x + 56.0f + i * 23.0f, panel_y + 92.0f),
            settings_accents[static_cast<size_t>(i)]);
    }

    const ImVec2 close0 =
        layout.point(panel_x + panel_w - 57.0f, panel_y + 26.0f);
    ImGui::SetCursorScreenPos(close0);
    ImGui::PushID("definitive_settings_close");
    const bool close_pressed =
        ImGui::InvisibleButton("##close", layout.size(30.0f, 30.0f));
    const bool close_hovered = ImGui::IsItemHovered();
    ImGui::PopID();

    const ImU32 close_color = text_color(
        close_hovered
            ? rgba(242, 246, 249, 255)
            : rgba(165, 176, 187, 230));
    if (close_hovered) {
        draw->AddRectFilled(
            close0,
            ImVec2(close0.x + layout.px(30.0f),
                close0.y + layout.px(30.0f)),
            surface_color(rgba(36, 49, 61, 160), 0.80f),
            layout.px(3.0f));
    }
    draw->AddLine(
        ImVec2(close0.x + layout.px(8.0f), close0.y + layout.px(8.0f)),
        ImVec2(close0.x + layout.px(22.0f), close0.y + layout.px(22.0f)),
        close_color, layout.px(1.6f));
    draw->AddLine(
        ImVec2(close0.x + layout.px(22.0f), close0.y + layout.px(8.0f)),
        ImVec2(close0.x + layout.px(8.0f), close0.y + layout.px(22.0f)),
        close_color, layout.px(1.6f));

    if ((close_pressed ||
            ImGui::IsKeyPressed(ImGuiKey_Escape, false)) &&
        definitive_settings_transition_ ==
            DefinitiveSettingsTransition::Open) {
        play_ui_close_sound();
        close_definitive_settings();
    }

    draw->AddLine(
        layout.point(panel_x + 28.0f, panel_y + 108.0f),
        layout.point(panel_x + panel_w - 28.0f, panel_y + 108.0f),
        accent_color(rgba(82, 97, 111, 155), 0.52f), layout.px(1.0f));

    // Mirror the legacy Settings page's category names, excluding Logging.
    constexpr std::array<const char*, 7> tab_labels = {
        "Input", "Video", "Audio", "System",
        "Memory Cards", "Experimental", "Customize"
    };
    constexpr float tab_x = 124.0f;
    constexpr float tab_y = 164.0f;
    constexpr float tab_w = 147.0f;

    for (int i = 0; i < static_cast<int>(tab_labels.size()); ++i) {
        if (definitive_settings_tab_button(
            draw, layout,
            tab_labels[static_cast<size_t>(i)],
            tab_labels[static_cast<size_t>(i)],
            tab_x + tab_w * i, tab_y, tab_w,
            definitive_settings_tab_ == i)) {
            definitive_settings_tab_ = i;
        }
    }

    constexpr float left_x = 134.0f;
    constexpr float right_x = 654.0f;
    constexpr float column_w = 492.0f;
    constexpr float content_y = 216.0f;
    constexpr float row_step = 68.0f;

    const auto note = [&](float x, float y, const char* text) {
        definitive_settings_note(draw, layout, x + 18.0f, y + 29.0f, text);
    };

    const auto apply_audio_settings = [&]() {
        save_persistent_config();
        if (system_ == nullptr) {
            return;
        }
        const bool was_running = emu_runner_.is_running();
        if (was_running) {
            emu_runner_.pause_and_wait_idle();
        }
        system_->spu().reinitialize_audio_device();
        if (was_running) {
            emu_runner_.set_running(true);
        }
    };

    switch (definitive_settings_tab_) {
    case 0: { // Input
        draw_settings_section(
            draw, layout, left_x, content_y, column_w, 160.0f, "INPUT");

        if (definitive_settings_action(
            draw, layout, "input_bindings", "Configure Keyboard Bindings",
            left_x + 1.0f, content_y + 43.0f, column_w - 2.0f)) {
            play_ui_open_sound();
            show_bindings_config_ = true;
            show_settings_ = false;
            definitive_detailed_settings_ = false;
            definitive_settings_transition_ =
                DefinitiveSettingsTransition::Closed;
            definitive_settings_transition_elapsed_ = 0.0f;
            ImGui::End();
            return;
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Change the keyboard keys used for PlayStation buttons.");

        draw_settings_section(
            draw, layout, right_x, content_y, column_w, 160.0f, "GAMEPAD");
        const std::string gamepad_title =
            input_ && input_->has_gamepad()
                ? input_->gamepad_name()
                : "No Gamepad Detected";
        add_text(draw, layout, right_x + 18.0f, content_y + 55.0f, 16.5f,
            input_ && input_->has_gamepad()
                ? rgba(178, 221, 190, 245)
                : rgba(188, 195, 202, 235),
            gamepad_title.c_str());
        add_text(draw, layout, right_x + 18.0f, content_y + 84.0f, 13.5f,
            rgba(171, 181, 191, 235),
            input_ && input_->has_gamepad()
                ? "Connected gamepads are mapped automatically."
                : "Connect a controller and VibeStation will auto-map it.");
        break;
    }

    case 1: { // Video
        draw_settings_section(
            draw, layout, left_x, content_y, column_w, 260.0f, "DISPLAY");

        const char* resolution_modes[] = {
            "320x240", "640x480", "1024x768"
        };
        int resolution_index = static_cast<int>(g_output_resolution_mode);
        if (definitive_settings_combo(
            draw, layout, "video_resolution", "Output Resolution",
            left_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            resolution_index, resolution_modes,
            IM_ARRAYSIZE(resolution_modes))) {
            resolution_index = std::clamp(resolution_index, 0, 2);
            g_output_resolution_mode =
                static_cast<OutputResolutionMode>(resolution_index);
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Sets the final framebuffer size shown by VibeStation.");

        const char* deinterlace_modes[] = {
            "Weave", "Bob", "Blend"
        };
        int deinterlace_index = static_cast<int>(g_deinterlace_mode);
        if (definitive_settings_combo(
            draw, layout, "video_deinterlace", "Deinterlace",
            left_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f,
            deinterlace_index, deinterlace_modes,
            IM_ARRAYSIZE(deinterlace_modes))) {
            deinterlace_index = std::clamp(deinterlace_index, 0, 2);
            g_deinterlace_mode =
                static_cast<DeinterlaceMode>(deinterlace_index);
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Chooses how interlaced PS1 video fields are combined.");

        if (definitive_settings_switch(
            draw, layout, "video_filter", "Bilinear Presentation Filter",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, g_bilinear_filtering)) {
            if (renderer_) {
                renderer_->set_bilinear_filtering(g_bilinear_filtering);
            }
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Smooths the final image when scaling instead of keeping hard pixels.");

        draw_settings_section(
            draw, layout, right_x, content_y, column_w, 258.0f, "GPU");

        if (definitive_settings_switch(
            draw, layout, "video_hw_gpu", "Hardware Rasterizer (Experimental)",
            right_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            g_gpu_hardware_rasterizer)) {
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f,
            "Runs accurate PS1 raster work on an OpenGL 4.3 compute backend when supported.");

        if (definitive_settings_switch(
            draw, layout, "video_fast_gpu", "Fast Mode",
            right_x + 1.0f, content_y + 43.0f + row_step, column_w - 2.0f,
            g_gpu_fast_mode)) {
            if (!g_gpu_fast_mode) {
                g_gpu_extreme_fast_mode = false;
            }
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step,
            "Uses optimized software GPU paths for lower CPU usage with some artifact risk.");

        if (definitive_settings_switch(
            draw, layout, "video_extreme_gpu", "Extreme Fast Mode",
            right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, g_gpu_extreme_fast_mode)) {
            if (g_gpu_extreme_fast_mode) {
                g_gpu_fast_mode = true;
            }
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Trades more shading and transparency accuracy for additional speed.");
        break;
    }

    case 2: { // Audio
        draw_settings_section(
            draw, layout, left_x, content_y, column_w, 326.0f, "LATENCY");

        int target_latency = static_cast<int>(g_spu_audio_target_latency_ms);
        if (definitive_settings_slider_int(
            draw, layout, "audio_target", "Target Latency",
            left_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            target_latency, 10, 500, "%d ms")) {
            g_spu_audio_target_latency_ms =
                static_cast<u32>(std::clamp(target_latency, 10, 500));
            g_spu_audio_soft_latency_ms = std::max(
                g_spu_audio_soft_latency_ms, g_spu_audio_target_latency_ms);
            g_spu_audio_max_latency_ms = std::max(
                g_spu_audio_max_latency_ms, g_spu_audio_soft_latency_ms);
            apply_audio_settings();
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Preferred amount of queued audio before playback.");

        int soft_latency = static_cast<int>(g_spu_audio_soft_latency_ms);
        if (definitive_settings_slider_int(
            draw, layout, "audio_soft", "Soft Correction Starts",
            left_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, soft_latency,
            static_cast<int>(g_spu_audio_target_latency_ms),
            750, "%d ms")) {
            g_spu_audio_soft_latency_ms =
                static_cast<u32>(std::clamp(
                    soft_latency,
                    static_cast<int>(g_spu_audio_target_latency_ms), 750));
            g_spu_audio_max_latency_ms = std::max(
                g_spu_audio_max_latency_ms, g_spu_audio_soft_latency_ms);
            apply_audio_settings();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Starts gentle queue correction when latency grows beyond this point.");

        int max_latency = static_cast<int>(g_spu_audio_max_latency_ms);
        if (definitive_settings_slider_int(
            draw, layout, "audio_max", "Maximum Latency",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, max_latency,
            static_cast<int>(g_spu_audio_soft_latency_ms),
            1000, "%d ms")) {
            g_spu_audio_max_latency_ms =
                static_cast<u32>(std::clamp(
                    max_latency,
                    static_cast<int>(g_spu_audio_soft_latency_ms), 1000));
            apply_audio_settings();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Hard ceiling before excess queued audio is trimmed.");

        float xa_buffer = g_spu_xa_buffer_seconds;
        if (definitive_settings_slider_float(
            draw, layout, "audio_xa", "XA Buffer",
            left_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            column_w - 2.0f, xa_buffer, 0.0f, 5.0f, "%.2f sec")) {
            g_spu_xa_buffer_seconds = std::clamp(xa_buffer, 0.0f, 5.0f);
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            "Controls buffering for XA and other streamed CD audio.");

        draw_settings_section(
            draw, layout, right_x, content_y, column_w, 326.0f, "PLAYBACK");

        if (definitive_settings_switch(
            draw, layout, "audio_queue", "Enable Audio Queue",
            right_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            g_spu_enable_audio_queue)) {
            apply_audio_settings();
        }
        note(right_x + 1.0f, content_y + 43.0f,
            "Uses the bounded host queue for steadier playback.");

        if (definitive_settings_switch(
            draw, layout, "audio_trim", "Crossfaded Smooth Trim",
            right_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, g_spu_enable_smooth_trim)) {
            apply_audio_settings();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step,
            "Crossfades queue corrections to make trims less audible.");

        if (definitive_settings_switch(
            draw, layout, "audio_lag_stutter", "Lag Stutter Effect",
            right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, g_spu_enable_lag_stutter)) {
            apply_audio_settings();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Repeats a short audio fragment when emulation falls behind.");

        if (definitive_settings_switch(
            draw, layout, "audio_slow_stutter", "Slowdown Stutter Loop",
            right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            column_w - 2.0f, g_spu_enable_slowdown_stutter)) {
            apply_audio_settings();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            "Loops a small audio segment while slowdown mode is active.");
        break;
    }

    case 3: { // System
        draw_settings_section(
            draw, layout, left_x, content_y, column_w, 326.0f, "CPU & PERFORMANCE");

        const char* cpu_backend_labels[] = {
            "Interpreter", "Recompiler (Experimental)"
        };
        int cpu_backend =
            g_cpu_execution_mode == CpuExecutionMode::Interpreter ? 0 : 1;
        if (definitive_settings_combo(
            draw, layout, "system_cpu", "CPU Backend",
            left_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            cpu_backend, cpu_backend_labels,
            IM_ARRAYSIZE(cpu_backend_labels))) {
            const bool was_running = emu_runner_.is_running();
            if (was_running) {
                emu_runner_.pause_and_wait_idle();
            }
            g_cpu_execution_mode =
                cpu_backend == 0
                    ? CpuExecutionMode::Interpreter
                    : CpuExecutionMode::Recompiler;
            if (system_) {
                system_->cpu().flush_cpu_backend();
            }
            save_persistent_config();
            if (was_running) {
                emu_runner_.set_running(true);
            }
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Selects the interpreter or the experimental unified recompiler.");

        const char* turbo_modes[] = { "200%", "400%", "Unlimited" };
        int turbo_mode = 0;
        if (config_turbo_speed_percent_ <= 0) {
            turbo_mode = 2;
        }
        else if (config_turbo_speed_percent_ >= 400) {
            turbo_mode = 1;
        }
        if (definitive_settings_combo(
            draw, layout, "system_turbo", "Turbo Speed",
            left_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, turbo_mode, turbo_modes,
            IM_ARRAYSIZE(turbo_modes))) {
            config_turbo_speed_percent_ =
                turbo_mode == 2 ? 0 : (turbo_mode == 1 ? 400 : 200);
            apply_speed_override();
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Sets the emulation speed used while holding the turbo hotkey.");

        int slowdown = config_slowdown_speed_percent_;
        if (definitive_settings_slider_int(
            draw, layout, "system_slowdown", "Slowdown Speed",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, slowdown, 10, 100, "%d%%")) {
            config_slowdown_speed_percent_ = std::clamp(slowdown, 10, 100);
            apply_speed_override();
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Sets the emulation speed used while holding the slowdown hotkey.");

        if (definitive_settings_switch(
            draw, layout, "system_low_spec", "Low-spec Mode",
            left_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            column_w - 2.0f, config_low_spec_mode_)) {
            g_low_spec_mode = config_low_spec_mode_;
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            "Reduces internal work for slower PCs with some quality tradeoffs.");

        draw_settings_section(
            draw, layout, right_x, content_y, column_w, 394.0f, "PLAYBACK & SERVICES");

        if (definitive_settings_switch(
            draw, layout, "system_vsync", "VSync Playback",
            right_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            config_vsync_)) {
            SDL_GL_SetSwapInterval(config_vsync_ ? 1 : 0);
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f,
            "Synchronizes presentation with the display refresh rate.");

        if (definitive_settings_switch(
            draw, layout, "system_direct_boot", "Direct Disc Boot",
            right_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, config_direct_disc_boot_)) {
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step,
            "Skips the BIOS intro and starts the inserted game directly.");

        if (definitive_settings_switch(
            draw, layout, "system_rewind", "Enable Rewind",
            right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, config_rewind_enabled_)) {
            emu_runner_.configure_rewind(
                config_rewind_enabled_,
                config_rewind_buffer_seconds_,
                static_cast<int>(system_ ? system_->target_fps() : 60.0));
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Keeps frame snapshots so gameplay can be rewound with Right Ctrl.");

        int rewind_seconds = config_rewind_buffer_seconds_;
        if (definitive_settings_slider_int(
            draw, layout, "system_rewind_seconds", "Rewind Buffer",
            right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            column_w - 2.0f, rewind_seconds, 1, 10, "%d sec")) {
            config_rewind_buffer_seconds_ =
                std::clamp(rewind_seconds, 1, 10);
            emu_runner_.set_rewind_buffer_seconds(
                config_rewind_buffer_seconds_,
                static_cast<int>(system_ ? system_->target_fps() : 60.0));
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            "Controls how many seconds of rewind history are retained.");

        if (definitive_settings_switch(
            draw, layout, "system_discord", "Discord Rich Presence",
            right_x + 1.0f, content_y + 43.0f + row_step * 4.0f,
            column_w - 2.0f, config_discord_rich_presence_)) {
            sync_discord_presence_config();
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 4.0f,
            "Shows the current VibeStation session in the Discord desktop app.");
        break;
    }

    case 4: { // Memory Cards
        draw_settings_section(
            draw, layout, left_x, content_y, 1012.0f, 260.0f, "MEMORY CARDS");

        const char* memory_modes[] = {
            "Generic", "Per-Game", "Disabled"
        };
        int slot1 = std::clamp(config_memory_card_mode_[0], 0, 2);
        if (definitive_settings_combo(
            draw, layout, "memory_slot1", "Slot 1 Mode",
            left_x + 1.0f, content_y + 43.0f, 1010.0f,
            slot1, memory_modes, IM_ARRAYSIZE(memory_modes))) {
            config_memory_card_mode_[0] = slot1;
            apply_memory_card_settings(true);
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Generic shares one card; Per-Game creates a card for each disc.");

        int slot2 = std::clamp(config_memory_card_mode_[1], 0, 2);
        if (definitive_settings_combo(
            draw, layout, "memory_slot2", "Slot 2 Mode",
            left_x + 1.0f, content_y + 43.0f + row_step, 1010.0f,
            slot2, memory_modes, IM_ARRAYSIZE(memory_modes))) {
            config_memory_card_mode_[1] = slot2;
            apply_memory_card_settings(true);
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Controls how the second virtual PlayStation memory card is mounted.");

        if (definitive_settings_action(
            draw, layout, "memory_apply", "Apply Memory Card Settings",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f, 1010.0f)) {
            apply_memory_card_settings(true);
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Refreshes the currently mounted card files using the selected modes.");
        break;
    }

    case 5: { // Experimental
        draw_settings_section(
            draw, layout, left_x, content_y, 1012.0f, 326.0f, "EXPERIMENTAL");

        if (definitive_settings_switch(
            draw, layout, "experimental_bios_size", "Experimental BIOS Size Mode",
            left_x + 1.0f, content_y + 43.0f, 1010.0f,
            g_experimental_bios_size_mode)) {
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Accepts KB-aligned BIOS images outside normal PS1 size checks.");

        if (definitive_settings_switch(
            draw, layout, "experimental_ps2_bios", "Unsafe PS2 BIOS Mode",
            left_x + 1.0f, content_y + 43.0f + row_step, 1010.0f,
            g_unsafe_ps2_bios_mode)) {
            if (g_unsafe_ps2_bios_mode) {
                g_experimental_bios_size_mode = true;
            }
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Maps the full BIOS size for PS2 BIOS experiments; instability is expected.");

        if (definitive_settings_switch(
            draw, layout, "experimental_opcode", "Unhandled Opcode Fallback",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f, 1010.0f,
            g_experimental_unhandled_special_returns_zero)) {
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Lets unknown instructions fall back instead of halting immediately.");

        if (definitive_settings_switch(
            draw, layout, "experimental_dma", "DMA Command Sanitizer",
            left_x + 1.0f, content_y + 43.0f + row_step * 3.0f, 1010.0f,
            g_experimental_dma_command_sanitizer)) {
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            "Coerces malformed DMA channel commands into legal transfer commands.");
        break;
    }

    case 6: { // Customize
        draw_settings_section(
            draw, layout, left_x, content_y, column_w, 260.0f, "THEME");

        const int preset_count = ui_theme::theme_preset_count();
        const int safe_preset_count = std::min(preset_count, 64);
        const char* theme_preset_labels[64] = {};
        for (int i = 0; i < safe_preset_count; ++i) {
            theme_preset_labels[i] =
                ui_theme::theme_preset_by_index(i).label;
        }
        ui_theme::g_selected_theme_preset_index =
            std::clamp(
                ui_theme::g_selected_theme_preset_index,
                0, std::max(0, safe_preset_count - 1));

        int preset_index = ui_theme::g_selected_theme_preset_index;
        if (safe_preset_count > 0 &&
            definitive_settings_combo(
                draw, layout, "customize_preset", "Preset",
                left_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
                preset_index, theme_preset_labels, safe_preset_count)) {
            ui_theme::g_selected_theme_preset_index = preset_index;
            ui_theme::apply_theme_preset_by_index(preset_index);
            ui_theme::apply_theme_style(ImGui::GetStyle());
            ui_theme::mark_theme_settings_dirty();
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Applies a predefined color scheme to the standard and detailed UI.");

        bool simple_theme = ui_theme::g_theme_settings.simple;
        if (definitive_settings_switch(
            draw, layout, "customize_simple", "Simple Customization",
            left_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, simple_theme)) {
            ui_theme::g_theme_settings.simple = simple_theme;
            ui_theme::mark_theme_settings_dirty();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Enables the main background, surface, accent, text and list colors.");

        if (definitive_settings_action(
            draw, layout, "customize_reset", "Reset Theme Colors",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f)) {
            ui_theme::reset_theme_settings();
            ui_theme::apply_theme_style(ImGui::GetStyle());
            ui_theme::mark_theme_settings_dirty();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Restores VibeStation's default theme colors.");

        draw_settings_section(
            draw, layout, right_x, content_y, column_w, 394.0f, "SIMPLE COLORS");

        if (!ui_theme::g_theme_settings.simple) {
            add_text(draw, layout, right_x + 18.0f, content_y + 61.0f, 14.5f,
                rgba(184, 194, 204, 238),
                "Enable Simple Customization to edit these colors.");
        }
        else {
            bool theme_changed = false;

            ImVec4 background = ui_theme::g_theme_settings.background;
            if (definitive_settings_color(
                draw, layout, "customize_background", "Background",
                right_x + 1.0f, content_y + 43.0f,
                column_w - 2.0f, background)) {
                ui_theme::g_theme_settings.background = background;
                theme_changed = true;
            }
            note(right_x + 1.0f, content_y + 43.0f,
                "Base window background color.");

            ImVec4 surface = ui_theme::g_theme_settings.surface;
            if (definitive_settings_color(
                draw, layout, "customize_surface", "Surface",
                right_x + 1.0f, content_y + 43.0f + row_step,
                column_w - 2.0f, surface)) {
                ui_theme::g_theme_settings.surface = surface;
                theme_changed = true;
            }
            note(right_x + 1.0f, content_y + 43.0f + row_step,
                "Panels, controls and raised surfaces.");

            ImVec4 accent = ui_theme::g_theme_settings.accent;
            if (definitive_settings_color(
                draw, layout, "customize_accent", "Accent",
                right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
                column_w - 2.0f, accent)) {
                ui_theme::g_theme_settings.accent = accent;
                theme_changed = true;
            }
            note(right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
                "Highlights, selections and active controls.");

            ImVec4 text_color = ui_theme::g_theme_settings.text;
            if (definitive_settings_color(
                draw, layout, "customize_text", "Text",
                right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
                column_w - 2.0f, text_color)) {
                ui_theme::g_theme_settings.text = text_color;
                theme_changed = true;
            }
            note(right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
                "Primary text color used by the standard UI.");

            ImVec4 lists = ui_theme::g_theme_settings.lists;
            if (definitive_settings_color(
                draw, layout, "customize_lists", "Lists",
                right_x + 1.0f, content_y + 43.0f + row_step * 4.0f,
                column_w - 2.0f, lists)) {
                ui_theme::g_theme_settings.lists = lists;
                theme_changed = true;
            }
            note(right_x + 1.0f, content_y + 43.0f + row_step * 4.0f,
                "Popup and list background color.");

            if (theme_changed) {
                ui_theme::sync_theme_overall_from_basics(
                    ui_theme::g_theme_settings);
                ui_theme::rebuild_theme_colors_from_basics(
                    ui_theme::g_theme_settings);
                ui_theme::apply_theme_style(ImGui::GetStyle());
                ui_theme::mark_theme_settings_dirty();
            }
        }
        break;
    }

    default:
        definitive_settings_tab_ = 0;
        break;
    }

    draw->AddLine(
        layout.point(panel_x + 28.0f, panel_y + panel_h - 92.0f),
        layout.point(panel_x + panel_w - 28.0f, panel_y + panel_h - 92.0f),
        accent_color(rgba(82, 97, 111, 155), 0.52f), layout.px(1.0f));

    ImGui::SetCursorScreenPos(
        layout.point(panel_x + 38.0f, panel_y + panel_h - 66.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, layout.px(2.0f));
    ImGui::PushStyleVar(
        ImGuiStyleVar_FramePadding, layout.size(4.0f, 4.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,
        surface_color(rgba(20, 28, 36, 245), 0.90f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,
        surface_color(rgba(31, 44, 56, 250), 0.74f));
    ImGui::PushStyleColor(ImGuiCol_CheckMark,
        accent_color(rgba(199, 224, 244, 255), 0.92f));
    ImGui::PushStyleColor(ImGuiCol_Text,
        text_color(rgba(220, 226, 232, 245)));

    bool detailed = definitive_detailed_settings_;
    ImGui::PushFont(font_for_size(layout.px(16.0f)));
    const bool detailed_changed =
        ImGui::Checkbox("Detailed Settings", &detailed);
    ImGui::PopFont();

    ImGui::PopStyleColor(4);
    ImGui::PopStyleVar(2);

    add_text(draw, layout,
        panel_x + 205.0f, panel_y + panel_h - 61.0f, 12.5f,
        rgba(166, 177, 187, 230),
        "Shows logging, diagnostics, profiling and other developer-oriented controls.");

    if (detailed_changed) {
        definitive_detailed_settings_ = detailed;
        if (detailed) {
            definitive_settings_transition_ =
                DefinitiveSettingsTransition::Closed;
            definitive_settings_transition_elapsed_ = 0.0f;
        }
    }

    add_text_right(
        draw, layout,
        panel_x + panel_w - 38.0f,
        panel_y + panel_h - 59.0f,
        11.5f, rgba(128, 140, 151, 210),
        vibestation_version_string());

    const ImVec2 transition_center(
        window_pos.x + window_size.x * 0.5f,
        window_pos.y + window_size.y * 0.5f);
    animate_draw_vertices(
        draw,
        animation_vertex_start,
        transition_center,
        transition_scale,
        transition_visibility,
        layout.px(transition_y_offset));

    ImGui::End();
}
