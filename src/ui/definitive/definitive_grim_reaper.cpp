#include "ui/app.h"
#include "ui/definitive/definitive_shared.h"
#include "ui/panels/grim_reaper_panel.h"
#include "vibestation_version.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

using namespace definitive_ui;

constexpr std::array<const char*, 5> kReaperStyleNames = {{
    "BIOS Corruption",
    "Batch BIOS",
    "RAM Reaper",
    "GPU Reaper",
    "Sound Reaper",
}};

constexpr std::array<const char*, 5> kReaperStyleSubtitles = {{
    "Corrupt one BIOS region and reboot",
    "Corrupt multiple BIOS regions together",
    "Real-time RAM, VRAM and SPU RAM corruption",
    "Real-time geometry, texture and display corruption",
    "Real-time pitch, ADSR, reverb and mixer corruption",
}};

const char* short_bios_target_name(int index) {
    switch (index) {
    case 0: return "Intro / Bootmenu";
    case 1: return "Character Sets";
    case 2: return "End";
    default: return "Custom Range";
    }
}

const char* preset_type_for_style(int style) {
    switch (style) {
    case 0: return "grim-single";
    case 1: return "grim-batch";
    case 2: return "ram-reaper";
    case 3: return "gpu-reaper";
    case 4: return "sound-reaper";
    default: return "";
    }
}

void draw_skull_icon(
    ImDrawList* draw,
    const ImVec2& center,
    float scale,
    ImU32 color,
    ImU32 cutout) {
    draw->AddCircleFilled(
        ImVec2(center.x, center.y - 4.0f * scale),
        13.0f * scale,
        color,
        28);
    draw->AddRectFilled(
        ImVec2(center.x - 9.0f * scale, center.y + 3.0f * scale),
        ImVec2(center.x + 9.0f * scale, center.y + 13.0f * scale),
        color,
        2.5f * scale);

    draw->AddCircleFilled(
        ImVec2(center.x - 5.0f * scale, center.y - 5.0f * scale),
        3.1f * scale,
        cutout,
        16);
    draw->AddCircleFilled(
        ImVec2(center.x + 5.0f * scale, center.y - 5.0f * scale),
        3.1f * scale,
        cutout,
        16);
    draw->AddTriangleFilled(
        ImVec2(center.x, center.y - 0.5f * scale),
        ImVec2(center.x - 2.7f * scale, center.y + 4.0f * scale),
        ImVec2(center.x + 2.7f * scale, center.y + 4.0f * scale),
        cutout);

    for (int i = -1; i <= 1; ++i) {
        const float x =
            center.x + static_cast<float>(i) * 5.0f * scale;
        draw->AddLine(
            ImVec2(x, center.y + 7.0f * scale),
            ImVec2(x, center.y + 13.0f * scale),
            cutout,
            std::max(1.0f, 1.4f * scale));
    }
}

void draw_section_box(
    ImDrawList* draw,
    const Layout& layout,
    float x,
    float y,
    float w,
    float h,
    const char* title) {
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 p1 = layout.point(x + w, y + h);

    draw->AddRectFilled(
        p0,
        p1,
        list_color(rgba(4, 8, 12, 214), 0.90f),
        layout.px(3.0f));
    draw->AddRect(
        p0,
        p1,
        accent_color(rgba(105, 123, 141, 175), 0.58f),
        layout.px(3.0f),
        0,
        layout.px(1.0f));

    add_text(
        draw,
        layout,
        x + 18.0f,
        y + 15.0f,
        15.0f,
        rgba(234, 238, 243, 250),
        title);

    draw->AddLine(
        layout.point(x + 16.0f, y + 44.0f),
        layout.point(x + w - 16.0f, y + 44.0f),
        accent_color(rgba(94, 108, 123, 130), 0.50f),
        layout.px(1.0f));
}

struct ReaperRowResult {
    bool clicked = false;
    bool hovered = false;
};

ReaperRowResult reaper_row(
    const Layout& layout,
    ImDrawList* draw,
    int index,
    const char* title,
    const char* subtitle,
    const char* value,
    bool enabled = true,
    bool primary = false) {
    constexpr float kX = 39.0f;
    constexpr float kY = 245.0f;
    constexpr float kW = 525.0f;
    constexpr float kH = 50.0f;
    constexpr float kGap = 7.0f;

    static std::array<float, 7> hover_mix{};
    static std::array<bool, 7> was_hovered{};

    const float y =
        kY + static_cast<float>(index) * (kH + kGap);
    const ImVec2 p = layout.point(kX, y);
    const ImVec2 size = layout.size(kW, kH);

    ImGui::SetCursorScreenPos(p);
    ImGui::PushID(index + 8000);

    if (!enabled) {
        ImGui::BeginDisabled();
    }
    const bool clicked =
        ImGui::InvisibleButton(
            "##reaper_row",
            size);
    const bool hovered =
        enabled && ImGui::IsItemHovered();
    if (!enabled) {
        ImGui::EndDisabled();
    }

    if (hovered &&
        !was_hovered[static_cast<size_t>(index)]) {
        definitive_ui::play_cursor_sound();
    }
    was_hovered[static_cast<size_t>(index)] = hovered;

    const float dt =
        std::clamp(
            ImGui::GetIO().DeltaTime,
            0.0f,
            0.05f);
    float& mix =
        hover_mix[static_cast<size_t>(index)];
    const float target =
        hovered ? 1.0f : 0.0f;
    mix +=
        (target - mix) *
        std::clamp(dt * 14.0f, 0.0f, 1.0f);

    const ImU32 fill =
        primary
            ? accent_color(
                rgba(
                    24, 35, 46,
                    static_cast<int>(
                        155.0f + 58.0f * mix)),
                0.62f)
            : surface_color(
                rgba(
                    8, 12, 17,
                    static_cast<int>(
                        84.0f + 75.0f * mix)),
                0.88f);

    draw->AddRectFilled(
        p,
        ImVec2(p.x + size.x, p.y + size.y),
        fill,
        layout.px(2.0f));

    if (primary || mix > 0.01f) {
        draw->AddRect(
            p,
            ImVec2(p.x + size.x, p.y + size.y),
            accent_color(
                rgba(
                    178, 211, 239,
                    static_cast<int>(
                        primary
                            ? 215.0f
                            : 205.0f * mix)),
                0.86f),
            layout.px(2.0f),
            0,
            layout.px(
                primary ? 1.4f : 1.0f));
    }

    if (primary) {
        draw->AddTriangleFilled(
            layout.point(
                kX + 20.0f,
                y + 13.0f),
            layout.point(
                kX + 20.0f,
                y + 37.0f),
            layout.point(
                kX + 38.0f,
                y + 25.0f),
            text_color(
                rgba(236, 241, 246, 248)));
    }
    else {
        const ImVec2 icon_center =
            layout.point(
                kX + 28.0f,
                y + 25.0f);
        draw->AddCircle(
            icon_center,
            layout.px(9.0f),
            text_color(
                rgba(
                    190, 199, 209,
                    enabled ? 220 : 110)),
            18,
            layout.px(1.5f));
        draw->AddCircleFilled(
            icon_center,
            layout.px(2.0f),
            text_color(
                rgba(
                    220, 226, 232,
                    enabled ? 235 : 120)));
    }

    add_text(
        draw,
        layout,
        kX + 62.0f,
        y + 8.0f,
        primary ? 17.0f : 15.5f,
        enabled
            ? rgba(235, 239, 244, 250)
            : rgba(130, 137, 145, 165),
        title);

    add_text(
        draw,
        layout,
        kX + 62.0f,
        y + 30.0f,
        10.5f,
        enabled
            ? rgba(173, 182, 192, 226)
            : rgba(111, 117, 124, 145),
        subtitle);

    if (value != nullptr &&
        value[0] != '\0') {
        add_text_right(
            draw,
            layout,
            kX + kW - 32.0f,
            y + 17.0f,
            12.5f,
            enabled
                ? rgba(219, 225, 232, 240)
                : rgba(120, 126, 133, 145),
            value);

        const ImVec2 arrow =
            layout.point(
                kX + kW - 17.0f,
                y + 25.0f);
        const ImU32 arrow_color =
            text_color(
                enabled
                    ? rgba(200, 209, 218, 225)
                    : rgba(105, 111, 118, 120));
        draw->AddLine(
            ImVec2(
                arrow.x - layout.px(3.0f),
                arrow.y - layout.px(4.0f)),
            arrow,
            arrow_color,
            layout.px(1.2f));
        draw->AddLine(
            arrow,
            ImVec2(
                arrow.x - layout.px(3.0f),
                arrow.y + layout.px(4.0f)),
            arrow_color,
            layout.px(1.2f));
    }

    ImGui::PopID();
    return {enabled && clicked, hovered};
}

} // namespace

void App::open_definitive_grim_reaper() {
    const bool was_inactive =
        !definitive_grim_reaper_active_;
    definitive_grim_reaper_active_ = true;
    definitive_grim_reaper_closing_ = false;
    if (was_inactive) {
        definitive_grim_reaper_visibility_ = 0.0f;
        definitive_grim_reaper_advanced_ = false;
    }
    show_grim_reaper_ = false;
}

void App::close_definitive_grim_reaper() {
    if (!definitive_grim_reaper_active_) {
        return;
    }
    definitive_grim_reaper_closing_ = true;
}

void App::panel_definitive_grim_reaper() {
    using namespace definitive_ui;

    const float dt =
        std::clamp(
            ImGui::GetIO().DeltaTime,
            0.0f,
            0.05f);
    constexpr float kOpenSeconds = 0.28f;
    constexpr float kCloseSeconds = 0.22f;

    if (definitive_grim_reaper_closing_) {
        definitive_grim_reaper_visibility_ =
            std::max(
                0.0f,
                definitive_grim_reaper_visibility_ -
                    dt / kCloseSeconds);

        if (definitive_grim_reaper_visibility_ <= 0.0f) {
            definitive_grim_reaper_visibility_ = 0.0f;
            definitive_grim_reaper_active_ = false;
            definitive_grim_reaper_closing_ = false;
            definitive_grim_reaper_advanced_ = false;
            return;
        }
    }
    else {
        definitive_grim_reaper_visibility_ =
            std::min(
                1.0f,
                definitive_grim_reaper_visibility_ +
                    dt / kOpenSeconds);
    }

    const float panel_visibility =
        smoothstep01(
            definitive_grim_reaper_visibility_);

    ImGuiViewport* viewport =
        ImGui::GetMainViewport();

    const float panel_width =
        std::clamp(
            viewport->WorkSize.x * 0.34f,
            360.0f,
            470.0f);

    const float visible_x =
        viewport->WorkPos.x +
        viewport->WorkSize.x -
        panel_width;
    const float hidden_x =
        viewport->WorkPos.x +
        viewport->WorkSize.x +
        22.0f;
    const ImVec2 panel_pos(
        hidden_x +
            (visible_x - hidden_x) *
                panel_visibility,
        viewport->WorkPos.y);
    const ImVec2 panel_size(
        panel_width,
        viewport->WorkSize.y);

    ImGui::SetNextWindowPos(
        panel_pos,
        ImGuiCond_Always);
    ImGui::SetNextWindowSize(
        panel_size,
        ImGuiCond_Always);

    ImGui::PushStyleVar(
        ImGuiStyleVar_WindowRounding,
        0.0f);
    ImGui::PushStyleVar(
        ImGuiStyleVar_WindowBorderSize,
        0.0f);
    ImGui::PushStyleVar(
        ImGuiStyleVar_WindowPadding,
        ImVec2(22.0f, 18.0f));
    ImGui::PushStyleVar(
        ImGuiStyleVar_ItemSpacing,
        ImVec2(8.0f, 8.0f));
    ImGui::PushStyleVar(
        ImGuiStyleVar_FrameRounding,
        4.0f);
    ImGui::PushStyleVar(
        ImGuiStyleVar_Alpha,
        panel_visibility);

    ImGui::PushStyleColor(
        ImGuiCol_WindowBg,
        surface_color(
            rgba(5, 8, 12, 246),
            0.94f));
    ImGui::PushStyleColor(
        ImGuiCol_Border,
        accent_color(
            rgba(92, 113, 134, 150),
            0.58f));
    ImGui::PushStyleColor(
        ImGuiCol_FrameBg,
        surface_color(
            rgba(13, 18, 24, 238),
            0.90f));
    ImGui::PushStyleColor(
        ImGuiCol_FrameBgHovered,
        surface_color(
            rgba(24, 34, 44, 248),
            0.76f));
    ImGui::PushStyleColor(
        ImGuiCol_FrameBgActive,
        accent_color(
            rgba(30, 46, 61, 252),
            0.72f));
    ImGui::PushStyleColor(
        ImGuiCol_Button,
        surface_color(
            rgba(18, 24, 31, 235),
            0.88f));
    ImGui::PushStyleColor(
        ImGuiCol_ButtonHovered,
        accent_color(
            rgba(38, 61, 82, 238),
            0.76f));
    ImGui::PushStyleColor(
        ImGuiCol_ButtonActive,
        accent_color(
            rgba(48, 76, 100, 248),
            0.84f));
    ImGui::PushStyleColor(
        ImGuiCol_Header,
        accent_color(
            rgba(36, 61, 84, 210),
            0.72f));
    ImGui::PushStyleColor(
        ImGuiCol_HeaderHovered,
        accent_color(
            rgba(47, 78, 106, 230),
            0.82f));
    ImGui::PushStyleColor(
        ImGuiCol_CheckMark,
        accent_color(
            rgba(205, 230, 249, 255),
            0.94f));
    ImGui::PushStyleColor(
        ImGuiCol_SliderGrab,
        accent_color(
            rgba(164, 202, 232, 240),
            0.90f));
    ImGui::PushStyleColor(
        ImGuiCol_SliderGrabActive,
        accent_color(
            rgba(220, 236, 249, 255),
            0.96f));
    ImGui::PushStyleColor(
        ImGuiCol_Text,
        text_color(
            rgba(230, 235, 240, 250)));
    ImGui::PushStyleColor(
        ImGuiCol_TextDisabled,
        text_color(
            rgba(137, 147, 158, 175)));

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoSavedSettings;
    if (definitive_grim_reaper_closing_) {
        flags |= ImGuiWindowFlags_NoInputs;
    }

    ImGui::Begin(
        "##DefinitiveGrimReaperPanel",
        nullptr,
        flags);

    // Strong left edge separates the controls from live gameplay without
    // hiding or resizing the game surface behind the panel.
    ImDrawList* draw =
        ImGui::GetWindowDrawList();
    const ImVec2 wp =
        ImGui::GetWindowPos();
    const ImVec2 ws =
        ImGui::GetWindowSize();

    draw->AddRectFilledMultiColor(
        ImVec2(wp.x - 18.0f, wp.y),
        ImVec2(wp.x + 12.0f, wp.y + ws.y),
        IM_COL32(0, 0, 0, 0),
        accent_color(rgba(68, 112, 151, 55), 0.72f),
        accent_color(rgba(68, 112, 151, 55), 0.72f),
        IM_COL32(0, 0, 0, 0));

    definitive_grim_reaper_style_ =
        std::clamp(
            definitive_grim_reaper_style_,
            0,
            static_cast<int>(
                kReaperStyleNames.size()) - 1);
    const int style =
        definitive_grim_reaper_style_;

    // Header.
    const ImVec2 header_start =
        ImGui::GetCursorScreenPos();
    const float header_height = 72.0f;

    draw_skull_icon(
        draw,
        ImVec2(
            header_start.x + 24.0f,
            header_start.y + 29.0f),
        1.0f,
        text_color(
            rgba(228, 233, 238, 248)),
        surface_color(
            rgba(7, 10, 14, 255),
            0.96f));

    ImGui::SetCursorPosX(
        ImGui::GetCursorPosX() + 54.0f);
    ImGui::PushFont(
        font_for_size(23.0f));
    ImGui::TextUnformatted(
        "GRIM REAPER");
    ImGui::PopFont();

    ImGui::SetCursorPosX(
        ImGui::GetCursorPosX() + 54.0f);
    ImGui::TextDisabled(
        "Generate cursed PlayStations.");

    ImGui::SetCursorScreenPos(
        ImVec2(
            wp.x + ws.x - 48.0f,
            header_start.y + 7.0f));
    if (ImGui::Button(
            "X##close_reaper",
            ImVec2(30.0f, 30.0f))) {
        play_close_sound();
        close_definitive_grim_reaper();
    }

    ImGui::SetCursorScreenPos(
        ImVec2(
            header_start.x,
            header_start.y +
                header_height));
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::PushFont(
        font_for_size(14.0f));

    // Style is the single high-level selector; all controls below it are
    // contextual so the same concepts are not repeated per Reaper type.
    ImGui::TextUnformatted("STYLE");
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::BeginCombo(
            "##reaper_style",
            kReaperStyleNames[
                static_cast<size_t>(style)])) {
        for (int i = 0;
             i < static_cast<int>(
                 kReaperStyleNames.size());
             ++i) {
            const bool selected =
                style == i;
            if (ImGui::Selectable(
                    kReaperStyleNames[
                        static_cast<size_t>(i)],
                    selected)) {
                definitive_grim_reaper_style_ =
                    i;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::TextDisabled(
        "%s",
        kReaperStyleSubtitles[
            static_cast<size_t>(style)]);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Contextual strength.
    ImGui::TextUnformatted("STRENGTH");

    if (style == 0) {
        const float max_strength =
            grim_reaper_area_index_ == 0
                ? 0.1f
                : 100.0f;
        grim_reaper_random_percent_ =
            std::clamp(
                grim_reaper_random_percent_,
                0.001f,
                max_strength);
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::SliderFloat(
            "##bios_strength",
            &grim_reaper_random_percent_,
            0.001f,
            max_strength,
            "%.3f%%");
    }
    else if (style == 1) {
        ImGui::TextDisabled(
            "Batch mode keeps one strength per selected BIOS range.");
    }
    else if (style == 2) {
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::SliderFloat(
                "##ram_strength",
                &ram_reaper_intensity_percent_,
                0.0f,
                100.0f,
                "%.1f%%") &&
            ram_reaper_enabled_) {
            sync_ram_reaper_config();
        }
    }
    else if (style == 3) {
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::SliderFloat(
                "##gpu_strength",
                &gpu_reaper_intensity_percent_,
                0.0f,
                100.0f,
                "%.1f%%") &&
            gpu_reaper_enabled_) {
            sync_gpu_reaper_config();
        }
    }
    else {
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::SliderFloat(
                "##sound_strength",
                &sound_reaper_intensity_percent_,
                0.0f,
                100.0f,
                "%.1f%%") &&
            sound_reaper_enabled_) {
            sync_sound_reaper_config();
        }
    }

    ImGui::Spacing();
    ImGui::TextUnformatted("TARGETS");

    if (style == 0) {
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo(
                "##bios_target",
                short_bios_target_name(
                    grim_reaper_area_index_))) {
            for (int i = 0;
                 i < kGrimReaperRangeCount;
                 ++i) {
                const bool selected =
                    grim_reaper_area_index_ == i;
                if (ImGui::Selectable(
                        short_bios_target_name(i),
                        selected)) {
                    grim_reaper_area_index_ = i;
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
    }
    else if (style == 1) {
        ImGui::Checkbox(
            "Intro / Bootmenu",
            &grim_batch_intro_enabled_);
        if (grim_batch_intro_enabled_) {
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::SliderFloat(
                "Intro Strike##batch",
                &grim_batch_intro_percent_,
                0.001f,
                0.1f,
                "%.3f%%");
        }

        ImGui::Checkbox(
            "Character Sets",
            &grim_batch_charset_enabled_);
        if (grim_batch_charset_enabled_) {
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::SliderFloat(
                "Charset Strike##batch",
                &grim_batch_charset_percent_,
                0.001f,
                100.0f,
                "%.3f%%");
        }

        ImGui::Checkbox(
            "End",
            &grim_batch_end_enabled_);
        if (grim_batch_end_enabled_) {
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::SliderFloat(
                "End Strike##batch",
                &grim_batch_end_percent_,
                0.001f,
                100.0f,
                "%.3f%%");
        }
    }
    else if (style == 2) {
        bool changed = false;
        changed |= ImGui::Checkbox(
            "Main RAM",
            &ram_reaper_affect_main_ram_);
        changed |= ImGui::Checkbox(
            "VRAM / Visual",
            &ram_reaper_affect_vram_);
        changed |= ImGui::Checkbox(
            "SPU RAM / Audio",
            &ram_reaper_affect_spu_ram_);
        if (changed &&
            ram_reaper_enabled_) {
            sync_ram_reaper_config();
        }
    }
    else if (style == 3) {
        bool changed = false;
        changed |= ImGui::Checkbox(
            "Geometry State",
            &gpu_reaper_affect_geometry_);
        changed |= ImGui::Checkbox(
            "Texture State",
            &gpu_reaper_affect_texture_state_);
        changed |= ImGui::Checkbox(
            "Display State",
            &gpu_reaper_affect_display_state_);
        if (changed &&
            gpu_reaper_enabled_) {
            sync_gpu_reaper_config();
        }
    }
    else {
        bool changed = false;
        changed |= ImGui::Checkbox(
            "Pitch / Semitones",
            &sound_reaper_affect_pitch_);
        changed |= ImGui::Checkbox(
            "Release / ADSR",
            &sound_reaper_affect_envelope_);
        changed |= ImGui::Checkbox(
            "Reverb / Delay",
            &sound_reaper_affect_reverb_);
        changed |= ImGui::Checkbox(
            "Wet / Dry Mixer",
            &sound_reaper_affect_mixer_);
        if (changed &&
            sound_reaper_enabled_) {
            sync_sound_reaper_config();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::TextUnformatted(
            "SLOWED + REVERB");
        draw_spu_diagnostic_mode_controls();
    }

    // Runtime switch only exists for runtime Reapers.
    if (style >= 2) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::TextUnformatted(
            "KEEP IT RUNNING");

        bool enabled =
            style == 2
                ? ram_reaper_enabled_
                : style == 3
                    ? gpu_reaper_enabled_
                    : sound_reaper_enabled_;

        if (ImGui::Checkbox(
                "Continuous corruption",
                &enabled)) {
            if (style == 2) {
                ram_reaper_enabled_ = enabled;
                sync_ram_reaper_config();
            }
            else if (style == 3) {
                gpu_reaper_enabled_ = enabled;
                sync_gpu_reaper_config();
            }
            else {
                sound_reaper_enabled_ = enabled;
                sync_sound_reaper_config();
            }
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    const bool bios_ready =
        system_ != nullptr &&
        system_->bios_loaded() &&
        !bios_path_.empty();
    const bool batch_ready =
        grim_batch_intro_enabled_ ||
        grim_batch_charset_enabled_ ||
        grim_batch_end_enabled_;
    const bool primary_enabled =
        style >= 2
            ? (system_ != nullptr &&
                system_->bios_loaded())
            : (bios_ready &&
                (style != 1 ||
                    batch_ready));

    if (!primary_enabled) {
        ImGui::BeginDisabled();
    }

    const char* primary_label =
        style >= 2
            ? "Apply & Run"
            : "Corrupt & (Re)Start";

    if (ImGui::Button(
            primary_label,
            ImVec2(-1.0f, 42.0f))) {
        play_open_sound();

        bool success = false;
        if (style == 0) {
            success =
                reap_and_reboot_bios();
        }
        else if (style == 1) {
            success =
                reap_and_reboot_bios_batch();
        }
        else {
            // Runtime Reapers are applied around a real emulator restart.
            // Preserve the user's continuous-mode choices exactly: the restart
            // helpers intentionally disable all Reapers while resetting the
            // machine, but Apply & Run must not silently turn them back on.
            const bool restore_ram_reaper =
                ram_reaper_enabled_;
            const bool restore_gpu_reaper =
                gpu_reaper_enabled_;
            const bool restore_sound_reaper =
                sound_reaper_enabled_;

            const bool has_disc =
                system_->disc_loaded() ||
                !game_bin_path_.empty() ||
                !game_cue_path_.empty();

            success =
                has_disc
                    ? boot_disc_from_ui()
                    : start_bios_from_ui();

            if (success) {
                ram_reaper_enabled_ =
                    restore_ram_reaper;
                gpu_reaper_enabled_ =
                    restore_gpu_reaper;
                sound_reaper_enabled_ =
                    restore_sound_reaper;

                sync_ram_reaper_config();
                sync_gpu_reaper_config();
                sync_sound_reaper_config();

                const bool selected_continuous =
                    style == 2
                        ? ram_reaper_enabled_
                        : style == 3
                            ? gpu_reaper_enabled_
                            : sound_reaper_enabled_;

                status_message_ =
                    std::string("Emulation restarted; ") +
                    kReaperStyleNames[
                        static_cast<size_t>(style)] +
                    (selected_continuous
                        ? " continuous mode enabled"
                        : " continuous mode disabled");
            }
        }

        (void)success;
    }

    if (!primary_enabled) {
        ImGui::EndDisabled();
        ImGui::TextDisabled(
            style == 1 &&
                    bios_ready
                ? "Select at least one BIOS range."
                : "Load a BIOS first.");
    }

    ImGui::Spacing();

    if (ImGui::Button(
            definitive_grim_reaper_advanced_
                ? "Hide Advanced"
                : "Advanced",
            ImVec2(-1.0f, 34.0f))) {
        definitive_grim_reaper_advanced_ =
            !definitive_grim_reaper_advanced_;
    }

    if (definitive_grim_reaper_advanced_) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::TextUnformatted(
            "ADVANCED");

        if (style == 0) {
            ImGui::Checkbox(
                "Use Custom Seed",
                &grim_use_custom_seed_);
            if (grim_use_custom_seed_) {
                ImGui::InputScalar(
                    "Seed",
                    ImGuiDataType_U64,
                    &grim_seed_);
            }

            if (grim_reaper_area_index_ ==
                kGrimReaperRangeCount - 1) {
                ImGui::InputText(
                    "Custom Start (hex)",
                    grim_reaper_custom_start_hex_,
                    IM_ARRAYSIZE(
                        grim_reaper_custom_start_hex_));
                ImGui::InputText(
                    "Custom End (hex)",
                    grim_reaper_custom_end_hex_,
                    IM_ARRAYSIZE(
                        grim_reaper_custom_end_hex_));
            }

            if (ImGui::Checkbox(
                    "Keep console logs",
                    &grim_reaper_keep_console_logs_) &&
                grim_reaper_mode_active_) {
                if (grim_reaper_keep_console_logs_ &&
                    grim_reaper_logs_suppressed_) {
                    g_log_category_mask =
                        grim_reaper_saved_log_mask_;
                    g_log_level =
                        grim_reaper_saved_log_level_;
                    grim_reaper_logs_suppressed_ =
                        false;
                }
                else if (!grim_reaper_keep_console_logs_ &&
                    !grim_reaper_logs_suppressed_) {
                    grim_reaper_saved_log_mask_ =
                        g_log_category_mask;
                    grim_reaper_saved_log_level_ =
                        g_log_level;
                    grim_reaper_logs_suppressed_ =
                        true;
                    g_log_category_mask = 0;
                    g_log_level =
                        LogLevel::Error;
                }
            }

            ImGui::InputText(
                "Profile Name",
                grim_preset_name_,
                IM_ARRAYSIZE(
                    grim_preset_name_));
        }
        else if (style == 1) {
            ImGui::Checkbox(
                "Use Custom Seeds Per Range",
                &grim_batch_use_custom_seeds_);

            if (grim_batch_use_custom_seeds_) {
                if (grim_batch_intro_enabled_) {
                    ImGui::InputScalar(
                        "Intro Seed",
                        ImGuiDataType_U64,
                        &grim_batch_intro_seed_);
                }
                if (grim_batch_charset_enabled_) {
                    ImGui::InputScalar(
                        "Charset Seed",
                        ImGuiDataType_U64,
                        &grim_batch_charset_seed_);
                }
                if (grim_batch_end_enabled_) {
                    ImGui::InputScalar(
                        "End Seed",
                        ImGuiDataType_U64,
                        &grim_batch_end_seed_);
                }
            }

            ImGui::InputText(
                "Profile Name",
                batch_preset_name_,
                IM_ARRAYSIZE(
                    batch_preset_name_));
        }
        else if (style == 2) {
            int writes =
                static_cast<int>(
                    std::min<u32>(
                        ram_reaper_writes_per_frame_,
                        5000u));
            if (ImGui::SliderInt(
                    "Writes / Frame",
                    &writes,
                    0,
                    5000)) {
                ram_reaper_writes_per_frame_ =
                    static_cast<u32>(
                        std::max(0, writes));
                if (ram_reaper_enabled_) {
                    sync_ram_reaper_config();
                }
            }

            if (ram_reaper_affect_main_ram_) {
                ImGui::InputScalar(
                    "RAM Start",
                    ImGuiDataType_U32,
                    &ram_reaper_range_start_,
                    nullptr,
                    nullptr,
                    "%06X",
                    ImGuiInputTextFlags_CharsHexadecimal);
                ImGui::InputScalar(
                    "RAM End",
                    ImGuiDataType_U32,
                    &ram_reaper_range_end_,
                    nullptr,
                    nullptr,
                    "%06X",
                    ImGuiInputTextFlags_CharsHexadecimal);

                ram_reaper_range_start_ =
                    std::min(
                        ram_reaper_range_start_,
                        psx::RAM_SIZE - 1u);
                ram_reaper_range_end_ =
                    std::min(
                        ram_reaper_range_end_,
                        psx::RAM_SIZE - 1u);
            }

            ImGui::Checkbox(
                "Use Custom Seed",
                &ram_reaper_use_custom_seed_);
            if (ram_reaper_use_custom_seed_) {
                ImGui::InputScalar(
                    "Seed",
                    ImGuiDataType_U64,
                    &ram_reaper_seed_);
            }

            ImGui::InputText(
                "Profile Name",
                ram_preset_name_,
                IM_ARRAYSIZE(
                    ram_preset_name_));
        }
        else if (style == 3) {
            int writes =
                static_cast<int>(
                    std::min<u32>(
                        gpu_reaper_writes_per_frame_,
                        5000u));
            if (ImGui::SliderInt(
                    "Writes / Frame",
                    &writes,
                    0,
                    5000)) {
                gpu_reaper_writes_per_frame_ =
                    static_cast<u32>(
                        std::max(0, writes));
                if (gpu_reaper_enabled_) {
                    sync_gpu_reaper_config();
                }
            }

            ImGui::Checkbox(
                "Use Custom Seed",
                &gpu_reaper_use_custom_seed_);
            if (gpu_reaper_use_custom_seed_) {
                ImGui::InputScalar(
                    "Seed",
                    ImGuiDataType_U64,
                    &gpu_reaper_seed_);
            }

            ImGui::InputText(
                "Profile Name",
                gpu_preset_name_,
                IM_ARRAYSIZE(
                    gpu_preset_name_));
        }
        else {
            int writes =
                static_cast<int>(
                    std::min<u32>(
                        sound_reaper_writes_per_frame_,
                        5000u));
            if (ImGui::SliderInt(
                    "Writes / Frame",
                    &writes,
                    0,
                    5000)) {
                sound_reaper_writes_per_frame_ =
                    static_cast<u32>(
                        std::max(0, writes));
                if (sound_reaper_enabled_) {
                    sync_sound_reaper_config();
                }
            }

            ImGui::Checkbox(
                "Use Custom Seed",
                &sound_reaper_use_custom_seed_);
            if (sound_reaper_use_custom_seed_) {
                ImGui::InputScalar(
                    "Seed",
                    ImGuiDataType_U64,
                    &sound_reaper_seed_);
            }

            ImGui::Spacing();
            ImGui::TextUnformatted(
                "SPU SAMPLE TOOLS");

            const std::filesystem::path sound_ram_path =
                std::filesystem::current_path() /
                "sound.ram";

            sound_ram_voice_index_ =
                std::clamp(
                    sound_ram_voice_index_,
                    0,
                    23);

            if (sound_ram_multi_voice_export_) {
                ImGui::BeginDisabled();
            }
            ImGui::SliderInt(
                "Sample Voice",
                &sound_ram_voice_index_,
                0,
                23);
            if (sound_ram_multi_voice_export_) {
                ImGui::EndDisabled();
            }

            if (ImGui::Checkbox(
                    "Multi-Voice Export",
                    &sound_ram_multi_voice_export_) &&
                sound_ram_multi_voice_export_) {
                sound_ram_voice_selected_.fill(false);
                sound_ram_voice_selected_[
                    static_cast<size_t>(
                        sound_ram_voice_index_)] = true;
            }

            if (sound_ram_multi_voice_export_) {
                if (ImGui::Button(
                        "Current Voice")) {
                    sound_ram_voice_selected_.fill(false);
                    sound_ram_voice_selected_[
                        static_cast<size_t>(
                            sound_ram_voice_index_)] = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("All Voices")) {
                    sound_ram_voice_selected_.fill(true);
                }
                ImGui::SameLine();
                if (ImGui::Button("Clear")) {
                    sound_ram_voice_selected_.fill(false);
                }

                if (ImGui::BeginTable(
                        "##DefinitiveSoundVoiceSelection",
                        4,
                        ImGuiTableFlags_SizingStretchSame)) {
                    for (int voice = 0;
                         voice < 24;
                         ++voice) {
                        ImGui::TableNextColumn();
                        const std::string label =
                            std::to_string(voice);
                        ImGui::Checkbox(
                            label.c_str(),
                            &sound_ram_voice_selected_[
                                static_cast<size_t>(
                                    voice)]);
                    }
                    ImGui::EndTable();
                }
            }

            const bool replacement_loaded =
                system_->
                    spu_replacement_sample_loaded();

            auto run_spu_sample_action =
                [&](const auto& action) {
                    const bool was_running =
                        emu_runner_.is_running();
                    if (was_running) {
                        emu_runner_.pause_and_wait_idle();
                    }
                    action();
                    if (was_running) {
                        emu_runner_.set_running(true);
                    }
                };

            if (ImGui::Button(
                    sound_ram_multi_voice_export_
                        ? "Save Selected Voices"
                        : "Save Voice")) {
                run_spu_sample_action([&]() {
                    std::string error;

                    if (sound_ram_multi_voice_export_) {
                        std::vector<int> voices;
                        for (int voice = 0;
                             voice <
                                static_cast<int>(
                                    sound_ram_voice_selected_.size());
                             ++voice) {
                            if (sound_ram_voice_selected_[
                                    static_cast<size_t>(
                                        voice)]) {
                                voices.push_back(voice);
                            }
                        }

                        if (system_->
                                save_spu_voice_samples_to_file(
                                    voices,
                                    sound_ram_path.string(),
                                    &error)) {
                            status_message_ =
                                "Saved combined sound.ram.";
                        }
                        else {
                            status_message_ =
                                error.empty()
                                    ? "Failed to save sound.ram."
                                    : error;
                        }
                    }
                    else if (system_->
                            save_spu_voice_sample_to_file(
                                sound_ram_voice_index_,
                                sound_ram_path.string(),
                                &error)) {
                        status_message_ =
                            "Saved sound.ram.";
                    }
                    else {
                        status_message_ =
                            error.empty()
                                ? "Failed to save sound.ram."
                                : error;
                    }
                });
            }

            ImGui::SameLine();
            if (ImGui::Button(
                    "Load sound.ram")) {
                run_spu_sample_action([&]() {
                    std::string error;
                    if (system_->
                            load_spu_replacement_sample_from_file(
                                sound_ram_path.string(),
                                &error)) {
                        status_message_ =
                            "Loaded sound.ram replacement.";
                    }
                    else {
                        status_message_ =
                            error.empty()
                                ? "Failed to load sound.ram."
                                : error;
                    }
                });
            }

            ImGui::SameLine();
            if (!replacement_loaded) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button(
                    "Clear Replacement")) {
                run_spu_sample_action([&]() {
                    system_->
                        clear_spu_replacement_sample();
                    status_message_ =
                        "Cleared SPU replacement sample.";
                });
            }
            if (!replacement_loaded) {
                ImGui::EndDisabled();
            }

            ImGui::InputText(
                "Profile Name",
                sound_preset_name_,
                IM_ARRAYSIZE(
                    sound_preset_name_));
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::TextUnformatted("PROFILES");

    if (ImGui::Button(
            "Save Current",
            ImVec2(
                (ImGui::GetContentRegionAvail().x - 8.0f) * 0.5f,
                34.0f))) {
        switch (style) {
        case 0:
            save_current_grim_preset(false);
            break;
        case 1:
            save_current_grim_preset(true);
            break;
        case 2:
            save_current_ram_preset();
            break;
        case 3:
            save_current_gpu_preset();
            break;
        default:
            save_current_sound_preset();
            break;
        }
    }

    ImGui::SameLine();

    if (ImGui::Button(
            "Browse",
            ImVec2(-1.0f, 34.0f))) {
        refresh_corruption_preset_list();
        ImGui::OpenPopup(
            "##definitive_reaper_presets");
    }

    if (ImGui::BeginPopup(
            "##definitive_reaper_presets")) {
        const char* wanted_type =
            preset_type_for_style(style);
        bool any = false;

        for (const auto& preset :
             corruption_presets_) {
            if (preset.preset_type !=
                wanted_type) {
                continue;
            }

            any = true;
            if (ImGui::MenuItem(
                    preset.display_name.c_str())) {
                load_corruption_preset(
                    preset.path);
            }
        }

        if (!any) {
            ImGui::TextDisabled(
                "No saved profiles for this style.");
        }

        ImGui::EndPopup();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Compact status only; configuration lives above so it is not duplicated.
    const u64 display_seed =
        style == 2
            ? ram_reaper_active_seed_
            : style == 3
                ? gpu_reaper_active_seed_
                : style == 4
                    ? sound_reaper_active_seed_
                    : grim_last_used_seed_;

    const u64 display_mutations =
        style == 2
            ? ram_reaper_total_mutations_
            : style == 3
                ? gpu_reaper_total_mutations_
                : style == 4
                    ? sound_reaper_total_mutations_
                    : static_cast<u64>(
                        grim_reaper_last_mutations_);

    ImGui::TextDisabled(
        "BIOS: %s",
        system_->bios_loaded()
            ? "Loaded"
            : "Not loaded");
    ImGui::TextDisabled(
        "Emulator: %s",
        has_started_emulation_
            ? (emu_runner_.is_running()
                ? "Running"
                : "Paused")
            : "Idle");
    ImGui::TextDisabled(
        "Seed: %llu  |  Mutations: %llu",
        static_cast<unsigned long long>(
            display_seed),
        static_cast<unsigned long long>(
            display_mutations));

    if (!grim_reaper_last_output_path_.empty()) {
        ImGui::TextDisabled(
            "Last BIOS: %s",
            std::filesystem::path(
                grim_reaper_last_output_path_)
                .filename()
                .string()
                .c_str());

        if (ImGui::Button(
                "Replay Last Corrupted BIOS",
                ImVec2(-1.0f, 32.0f))) {
            emu_runner_.pause_and_wait_idle();
            disable_ram_reaper_mode();
            disable_gpu_reaper_mode();
            disable_sound_reaper_mode();
            set_grim_reaper_mode(true);

            if (!system_->load_bios(
                    grim_reaper_last_output_path_)) {
                set_grim_reaper_mode(false);
                status_message_ =
                    "Failed to load last corrupted BIOS copy.";
            }
            else {
                has_started_emulation_ = false;
                system_->reset();
                apply_memory_card_settings(false);
                has_started_emulation_ = true;
                emu_runner_.set_running(true);
                status_message_ =
                    "Corrupted BIOS emulation restarted";
            }
        }
    }

    ImGui::Spacing();
    if (ImGui::Button(
            has_started_emulation_
                ? "Close Panel"
                : "Back",
            ImVec2(-1.0f, 34.0f))) {
        play_close_sound();
        close_definitive_grim_reaper();
    }

    ImGui::PopFont();
    ImGui::End();

    ImGui::PopStyleColor(15);
    ImGui::PopStyleVar(6);
}

