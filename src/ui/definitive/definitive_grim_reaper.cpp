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
    definitive_grim_reaper_active_ = true;
    definitive_grim_reaper_advanced_ = false;
    show_grim_reaper_ = false;
}

void App::close_definitive_grim_reaper() {
    definitive_grim_reaper_active_ = false;
    definitive_grim_reaper_advanced_ = false;
}

void App::panel_definitive_grim_reaper() {
    using namespace definitive_ui;

    ImDrawList* draw =
        ImGui::GetWindowDrawList();
    const ImVec2 window_pos =
        ImGui::GetWindowPos();
    const ImVec2 window_size =
        ImGui::GetWindowSize();
    const Layout layout =
        make_layout(
            window_pos,
            window_size);

    preload_background_assets();
    draw_launcher_background(
        draw,
        window_pos,
        window_size,
        1.0f);
    draw_launcher_readability_shade(
        draw,
        window_pos,
        window_size);

    draw->AddRectFilled(
        window_pos,
        ImVec2(
            window_pos.x + window_size.x,
            window_pos.y + window_size.y),
        background_color(
            rgba(0, 2, 5, 104),
            0.74f));

    definitive_grim_reaper_style_ =
        std::clamp(
            definitive_grim_reaper_style_,
            0,
            static_cast<int>(
                kReaperStyleNames.size()) - 1);
    const int style =
        definitive_grim_reaper_style_;

    // Header / brand.
    add_text(
        draw,
        layout,
        48.0f,
        34.0f,
        44.0f,
        rgba(224, 227, 231, 250),
        "VibeStation");

    constexpr std::array<ImU32, 4> accent_colors = {{
        IM_COL32(194, 44, 56, 255),
        IM_COL32(52, 128, 125, 255),
        IM_COL32(177, 145, 72, 255),
        IM_COL32(52, 93, 157, 255),
    }};
    for (int i = 0; i < 4; ++i) {
        draw->AddRectFilled(
            layout.point(
                50.0f + i * 32.0f,
                110.0f),
            layout.point(
                76.0f + i * 32.0f,
                120.0f),
            accent_colors[
                static_cast<size_t>(i)]);
    }
    add_text(
        draw,
        layout,
        184.0f,
        106.0f,
        13.0f,
        rgba(203, 208, 214, 238),
        "PS1 EMULATOR");

    add_text_right(
        draw,
        layout,
        1232.0f,
        34.0f,
        11.0f,
        rgba(166, 175, 184, 220),
        VIBESTATION_VERSION_STRING);

    const ImVec2 skull_center =
        layout.point(75.0f, 171.0f);
    draw_skull_icon(
        draw,
        skull_center,
        layout.scale * 1.25f,
        text_color(rgba(225, 230, 235, 248)),
        background_color(rgba(8, 11, 15, 255), 0.90f));

    add_text(
        draw,
        layout,
        116.0f,
        145.0f,
        27.0f,
        rgba(234, 237, 241, 252),
        "GRIM REAPER");
    add_text(
        draw,
        layout,
        116.0f,
        181.0f,
        13.0f,
        rgba(185, 194, 204, 232),
        "Generate cursed PlayStations.");

    // Context values for the compact main menu.
    std::string strength_value;
    switch (style) {
    case 0: {
        const float max_strength =
            grim_reaper_area_index_ == 0
                ? 0.1f
                : 100.0f;
        grim_reaper_random_percent_ =
            std::clamp(
                grim_reaper_random_percent_,
                0.001f,
                max_strength);
        char value[48]{};
        std::snprintf(
            value,
            sizeof(value),
            "%.3f%%",
            grim_reaper_random_percent_);
        strength_value = value;
        break;
    }
    case 1:
        strength_value = "Per Range";
        break;
    case 2: {
        char value[48]{};
        std::snprintf(
            value,
            sizeof(value),
            "%.1f%%",
            ram_reaper_intensity_percent_);
        strength_value = value;
        break;
    }
    case 3: {
        char value[48]{};
        std::snprintf(
            value,
            sizeof(value),
            "%.1f%%",
            gpu_reaper_intensity_percent_);
        strength_value = value;
        break;
    }
    default: {
        char value[48]{};
        std::snprintf(
            value,
            sizeof(value),
            "%.1f%%",
            sound_reaper_intensity_percent_);
        strength_value = value;
        break;
    }
    }

    std::string targets_value;
    switch (style) {
    case 0:
        targets_value =
            short_bios_target_name(
                grim_reaper_area_index_);
        break;
    case 1: {
        int count = 0;
        count += grim_batch_intro_enabled_ ? 1 : 0;
        count += grim_batch_charset_enabled_ ? 1 : 0;
        count += grim_batch_end_enabled_ ? 1 : 0;
        targets_value =
            count == 0
                ? "None Selected"
                : std::to_string(count) +
                    (count == 1
                        ? " Range"
                        : " Ranges");
        break;
    }
    case 2: {
        int count = 0;
        count += ram_reaper_affect_main_ram_ ? 1 : 0;
        count += ram_reaper_affect_vram_ ? 1 : 0;
        count += ram_reaper_affect_spu_ram_ ? 1 : 0;
        targets_value =
            std::to_string(count) + "/3";
        break;
    }
    case 3: {
        int count = 0;
        count += gpu_reaper_affect_geometry_ ? 1 : 0;
        count += gpu_reaper_affect_texture_state_ ? 1 : 0;
        count += gpu_reaper_affect_display_state_ ? 1 : 0;
        targets_value =
            std::to_string(count) + "/3";
        break;
    }
    default: {
        int count = 0;
        count += sound_reaper_affect_pitch_ ? 1 : 0;
        count += sound_reaper_affect_envelope_ ? 1 : 0;
        count += sound_reaper_affect_reverb_ ? 1 : 0;
        count += sound_reaper_affect_mixer_ ? 1 : 0;
        targets_value =
            std::to_string(count) + "/4";
        break;
    }
    }

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
                (style != 1 || batch_ready));

    const char* primary_title =
        style >= 2
            ? "Apply & Run"
            : "Corrupt & (Re)Start";
    const char* primary_subtitle =
        !primary_enabled
            ? (style == 1 && bios_ready
                ? "Select at least one target range first"
                : "Load a BIOS first")
            : kReaperStyleSubtitles[
                static_cast<size_t>(style)];

    if (reaper_row(
            layout,
            draw,
            0,
            primary_title,
            primary_subtitle,
            "",
            primary_enabled,
            true).clicked) {
        definitive_ui::play_open_sound();

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
            if (!has_started_emulation_) {
                success =
                    start_bios_from_ui();
            }
            else {
                success = true;
            }

            if (success) {
                if (style == 2) {
                    ram_reaper_enabled_ = true;
                    sync_ram_reaper_config();
                    status_message_ =
                        "RAM Reaper enabled";
                }
                else if (style == 3) {
                    gpu_reaper_enabled_ = true;
                    sync_gpu_reaper_config();
                    status_message_ =
                        "GPU Reaper enabled";
                }
                else {
                    sound_reaper_enabled_ = true;
                    sync_sound_reaper_config();
                    status_message_ =
                        "Sound Reaper enabled";
                }

                if (!emu_runner_.is_running()) {
                    emu_runner_.set_running(true);
                }
            }
        }

        (void)success;
    }

    if (reaper_row(
            layout,
            draw,
            1,
            "Style",
            "Choose the corruption engine",
            kReaperStyleNames[
                static_cast<size_t>(style)]).clicked) {
        definitive_ui::play_open_sound();
        ImGui::OpenPopup(
            "##definitive_reaper_style");
    }

    if (reaper_row(
            layout,
            draw,
            2,
            "Strength",
            style == 1
                ? "Batch uses one strength per selected BIOS range"
                : "Adjust how intense the corruption is",
            strength_value.c_str()).clicked) {
        definitive_ui::play_open_sound();
        ImGui::OpenPopup(
            "##definitive_reaper_strength");
    }

    if (reaper_row(
            layout,
            draw,
            3,
            "Targets",
            "Choose what the Reaper is allowed to touch",
            targets_value.c_str()).clicked) {
        definitive_ui::play_open_sound();
        ImGui::OpenPopup(
            "##definitive_reaper_targets");
    }

    std::string runtime_value =
        style < 2
            ? "One Shot"
            : ((style == 2
                    ? ram_reaper_enabled_
                    : style == 3
                        ? gpu_reaper_enabled_
                        : sound_reaper_enabled_)
                ? "Enabled"
                : "Disabled");

    const bool runtime_row_enabled =
        style >= 2;
    if (reaper_row(
            layout,
            draw,
            4,
            "Keep It Running",
            style < 2
                ? "BIOS corruption is applied once when starting"
                : "Continuously corrupt while emulation is running",
            runtime_value.c_str(),
            runtime_row_enabled).clicked) {
        definitive_ui::play_open_sound();

        if (style == 2) {
            ram_reaper_enabled_ =
                !ram_reaper_enabled_;
            sync_ram_reaper_config();
        }
        else if (style == 3) {
            gpu_reaper_enabled_ =
                !gpu_reaper_enabled_;
            sync_gpu_reaper_config();
        }
        else if (style == 4) {
            sound_reaper_enabled_ =
                !sound_reaper_enabled_;
            sync_sound_reaper_config();
        }
    }

    if (reaper_row(
            layout,
            draw,
            5,
            "Advanced",
            "Seeds, write rates, address ranges and diagnostics",
            definitive_grim_reaper_advanced_
                ? "Open"
                : "",
            true).clicked) {
        definitive_ui::play_open_sound();
        definitive_grim_reaper_advanced_ =
            !definitive_grim_reaper_advanced_;
    }

    if (reaper_row(
            layout,
            draw,
            6,
            "Back",
            has_started_emulation_
                ? "Return to gameplay"
                : "Return to VibeStation",
            "",
            true).clicked ||
        ImGui::IsKeyPressed(
            ImGuiKey_Escape,
            false)) {
        definitive_ui::play_close_sound();
        close_definitive_grim_reaper();
        return;
    }

    // Style selector popup.
    ImGui::SetNextWindowSize(
        layout.size(320.0f, 0.0f));
    if (ImGui::BeginPopup(
            "##definitive_reaper_style")) {
        for (int i = 0;
             i < static_cast<int>(
                 kReaperStyleNames.size());
             ++i) {
            if (ImGui::MenuItem(
                    kReaperStyleNames[
                        static_cast<size_t>(i)],
                    nullptr,
                    style == i)) {
                definitive_grim_reaper_style_ =
                    i;
            }
        }
        ImGui::EndPopup();
    }

    // Strength popup: one visible strength concept, backed by the existing
    // per-engine values.
    ImGui::SetNextWindowSize(
        layout.size(360.0f, 0.0f));
    if (ImGui::BeginPopup(
            "##definitive_reaper_strength")) {
        if (style == 0) {
            const float max_strength =
                grim_reaper_area_index_ == 0
                    ? 0.1f
                    : 100.0f;
            if (ImGui::SliderFloat(
                    "Random Strike (%)",
                    &grim_reaper_random_percent_,
                    0.001f,
                    max_strength,
                    "%.3f%%")) {
                grim_reaper_random_percent_ =
                    std::clamp(
                        grim_reaper_random_percent_,
                        0.001f,
                        max_strength);
            }
        }
        else if (style == 1) {
            ImGui::TextWrapped(
                "Batch BIOS keeps independent strengths for each selected range.");
            if (ImGui::Button(
                    "Open Advanced Range Strengths")) {
                definitive_grim_reaper_advanced_ =
                    true;
                ImGui::CloseCurrentPopup();
            }
        }
        else if (style == 2) {
            if (ImGui::SliderFloat(
                    "RAM Intensity (%)",
                    &ram_reaper_intensity_percent_,
                    0.0f,
                    100.0f,
                    "%.1f%%") &&
                ram_reaper_enabled_) {
                sync_ram_reaper_config();
            }
        }
        else if (style == 3) {
            if (ImGui::SliderFloat(
                    "GPU Chaos (%)",
                    &gpu_reaper_intensity_percent_,
                    0.0f,
                    100.0f,
                    "%.1f%%") &&
                gpu_reaper_enabled_) {
                sync_gpu_reaper_config();
            }
        }
        else {
            if (ImGui::SliderFloat(
                    "Sound Chaos (%)",
                    &sound_reaper_intensity_percent_,
                    0.0f,
                    100.0f,
                    "%.1f%%") &&
                sound_reaper_enabled_) {
                sync_sound_reaper_config();
            }
        }
        ImGui::EndPopup();
    }

    ImGui::SetNextWindowSize(
        layout.size(390.0f, 0.0f));
    if (ImGui::BeginPopup(
            "##definitive_reaper_targets")) {
        if (style == 0) {
            for (int i = 0;
                 i < kGrimReaperRangeCount;
                 ++i) {
                if (ImGui::MenuItem(
                        short_bios_target_name(i),
                        nullptr,
                        grim_reaper_area_index_ == i)) {
                    grim_reaper_area_index_ = i;
                }
            }
        }
        else if (style == 1) {
            ImGui::Checkbox(
                "Intro / Bootmenu",
                &grim_batch_intro_enabled_);
            ImGui::Checkbox(
                "Character Sets",
                &grim_batch_charset_enabled_);
            ImGui::Checkbox(
                "End",
                &grim_batch_end_enabled_);
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
        }
        ImGui::EndPopup();
    }

    // Right-hand profile/status area.
    draw_section_box(
        draw,
        layout,
        610.0f,
        145.0f,
        630.0f,
        475.0f,
        definitive_grim_reaper_advanced_
            ? "ADVANCED PROFILE"
            : "CURRENT PROFILE");

    const float right_x = 632.0f;
    const float right_y = 205.0f;
    const float right_w = 586.0f;
    const float right_h = 392.0f;

    ImGui::SetCursorScreenPos(
        layout.point(
            right_x,
            right_y));
    ImGui::PushStyleVar(
        ImGuiStyleVar_WindowPadding,
        layout.size(10.0f, 8.0f));
    ImGui::PushStyleVar(
        ImGuiStyleVar_ItemSpacing,
        layout.size(8.0f, 8.0f));
    ImGui::PushStyleColor(
        ImGuiCol_ChildBg,
        IM_COL32(0, 0, 0, 0));

    ImGui::BeginChild(
        "##DefinitiveReaperProfile",
        layout.size(
            right_w,
            right_h),
        false,
        ImGuiWindowFlags_NoBackground);

    ImGui::PushFont(
        font_for_size(
            layout.px(13.5f)));

    if (!definitive_grim_reaper_advanced_) {
        ImGui::Text(
            "Style: %s",
            kReaperStyleNames[
                static_cast<size_t>(style)]);
        ImGui::TextWrapped(
            "%s",
            kReaperStyleSubtitles[
                static_cast<size_t>(style)]);
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (style == 0) {
            ImGui::Text(
                "Target: %s",
                short_bios_target_name(
                    grim_reaper_area_index_));
            ImGui::Text(
                "Strike: %.3f%%",
                grim_reaper_random_percent_);
            ImGui::Text(
                "Last seed: %llu",
                static_cast<unsigned long long>(
                    grim_last_used_seed_));
        }
        else if (style == 1) {
            ImGui::Text(
                "Intro: %s",
                grim_batch_intro_enabled_
                    ? "Selected"
                    : "Off");
            ImGui::Text(
                "Character Sets: %s",
                grim_batch_charset_enabled_
                    ? "Selected"
                    : "Off");
            ImGui::Text(
                "End: %s",
                grim_batch_end_enabled_
                    ? "Selected"
                    : "Off");
        }
        else if (style == 2) {
            ImGui::Text(
                "Runtime: %s",
                ram_reaper_enabled_
                    ? "Enabled"
                    : "Disabled");
            ImGui::Text(
                "Writes / frame: %u",
                static_cast<unsigned>(
                    ram_reaper_writes_per_frame_));
            ImGui::Text(
                "Active seed: %llu",
                static_cast<unsigned long long>(
                    ram_reaper_active_seed_));
            ImGui::Text(
                "Mutations: %llu",
                static_cast<unsigned long long>(
                    ram_reaper_total_mutations_));
        }
        else if (style == 3) {
            ImGui::Text(
                "Runtime: %s",
                gpu_reaper_enabled_
                    ? "Enabled"
                    : "Disabled");
            ImGui::Text(
                "Writes / frame: %u",
                static_cast<unsigned>(
                    gpu_reaper_writes_per_frame_));
            ImGui::Text(
                "Active seed: %llu",
                static_cast<unsigned long long>(
                    gpu_reaper_active_seed_));
            ImGui::Text(
                "Mutations: %llu",
                static_cast<unsigned long long>(
                    gpu_reaper_total_mutations_));
        }
        else {
            ImGui::Text(
                "Runtime: %s",
                sound_reaper_enabled_
                    ? "Enabled"
                    : "Disabled");
            ImGui::Text(
                "Writes / frame: %u",
                static_cast<unsigned>(
                    sound_reaper_writes_per_frame_));
            ImGui::Text(
                "Active seed: %llu",
                static_cast<unsigned long long>(
                    sound_reaper_active_seed_));
            ImGui::Text(
                "Mutations: %llu",
                static_cast<unsigned long long>(
                    sound_reaper_total_mutations_));
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (ImGui::Button(
                "Save Current Profile",
                layout.size(190.0f, 34.0f))) {
            bool saved = false;
            switch (style) {
            case 0:
                saved =
                    save_current_grim_preset(false);
                break;
            case 1:
                saved =
                    save_current_grim_preset(true);
                break;
            case 2:
                saved =
                    save_current_ram_preset();
                break;
            case 3:
                saved =
                    save_current_gpu_preset();
                break;
            default:
                saved =
                    save_current_sound_preset();
                break;
            }
            (void)saved;
        }

        ImGui::SameLine();
        if (ImGui::Button(
                "Browse Profiles",
                layout.size(170.0f, 34.0f))) {
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
    }
    else {
        if (style == 0) {
            ImGui::TextUnformatted(
                "BIOS CORRUPTION ADVANCED");
            ImGui::Separator();

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
                    "Keep console logs while corrupting",
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
            ImGui::TextUnformatted(
                "BATCH BIOS ADVANCED");
            ImGui::Separator();

            ImGui::Checkbox(
                "Use Custom Seeds Per Range",
                &grim_batch_use_custom_seeds_);

            if (grim_batch_intro_enabled_) {
                ImGui::SliderFloat(
                    "Intro Strike (%)",
                    &grim_batch_intro_percent_,
                    0.001f,
                    0.1f,
                    "%.3f%%");
                if (grim_batch_use_custom_seeds_) {
                    ImGui::InputScalar(
                        "Intro Seed",
                        ImGuiDataType_U64,
                        &grim_batch_intro_seed_);
                }
            }

            if (grim_batch_charset_enabled_) {
                ImGui::SliderFloat(
                    "Charset Strike (%)",
                    &grim_batch_charset_percent_,
                    0.001f,
                    100.0f,
                    "%.3f%%");
                if (grim_batch_use_custom_seeds_) {
                    ImGui::InputScalar(
                        "Charset Seed",
                        ImGuiDataType_U64,
                        &grim_batch_charset_seed_);
                }
            }

            if (grim_batch_end_enabled_) {
                ImGui::SliderFloat(
                    "End Strike (%)",
                    &grim_batch_end_percent_,
                    0.001f,
                    100.0f,
                    "%.3f%%");
                if (grim_batch_use_custom_seeds_) {
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
            ImGui::TextUnformatted(
                "RAM REAPER ADVANCED");
            ImGui::Separator();

            int writes =
                static_cast<int>(
                    std::min<u32>(
                        ram_reaper_writes_per_frame_,
                        5000u));
            if (ImGui::SliderInt(
                    "Base Writes / Frame",
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
                    "Start (hex)",
                    ImGuiDataType_U32,
                    &ram_reaper_range_start_,
                    nullptr,
                    nullptr,
                    "%06X",
                    ImGuiInputTextFlags_CharsHexadecimal);
                ImGui::InputScalar(
                    "End (hex)",
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
            ImGui::TextUnformatted(
                "GPU REAPER ADVANCED");
            ImGui::Separator();

            int writes =
                static_cast<int>(
                    std::min<u32>(
                        gpu_reaper_writes_per_frame_,
                        5000u));
            if (ImGui::SliderInt(
                    "GPU Writes / Frame",
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
            ImGui::TextUnformatted(
                "SOUND REAPER ADVANCED");
            ImGui::Separator();

            draw_spu_diagnostic_mode_controls();
            ImGui::Separator();

            int writes =
                static_cast<int>(
                    std::min<u32>(
                        sound_reaper_writes_per_frame_,
                        5000u));
            if (ImGui::SliderInt(
                    "Sound Writes / Frame",
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

            ImGui::Separator();
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

            int selected_voice_count = 0;
            if (sound_ram_multi_voice_export_) {
                if (ImGui::Button(
                        "Current Voice Only")) {
                    sound_ram_voice_selected_.fill(false);
                    sound_ram_voice_selected_[
                        static_cast<size_t>(
                            sound_ram_voice_index_)] = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("Select All")) {
                    sound_ram_voice_selected_.fill(true);
                }
                ImGui::SameLine();
                if (ImGui::Button("Clear Selection")) {
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
                            "Voice " +
                            std::to_string(voice);
                        ImGui::Checkbox(
                            label.c_str(),
                            &sound_ram_voice_selected_[
                                static_cast<size_t>(
                                    voice)]);
                        if (sound_ram_voice_selected_[
                                static_cast<size_t>(
                                    voice)]) {
                            ++selected_voice_count;
                        }
                    }
                    ImGui::EndTable();
                }

                ImGui::Text(
                    "Selected Voices: %d",
                    selected_voice_count);
            }

            const bool replacement_loaded =
                system_->spu_replacement_sample_loaded();
            const bool replacement_enabled =
                system_->spu_replacement_sample_enabled();

            ImGui::Text(
                "Replacement: %s / %s / %zu bytes",
                replacement_loaded
                    ? "Loaded"
                    : "Not loaded",
                replacement_enabled
                    ? "Enabled"
                    : "Disabled",
                system_->spu_replacement_sample_bytes());
            ImGui::TextWrapped(
                "sound.ram: %s",
                sound_ram_path.string().c_str());

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
                        : "Save Voice To sound.ram")) {
                run_spu_sample_action([&]() {
                    std::string error;

                    if (sound_ram_multi_voice_export_) {
                        std::vector<int> voices;
                        voices.reserve(
                            sound_ram_voice_selected_.size());

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
                                "Saved combined sound.ram from " +
                                std::to_string(
                                    voices.size()) +
                                " SPU voices.";
                        }
                        else {
                            status_message_ =
                                error.empty()
                                    ? "Failed to save combined sound.ram."
                                    : error;
                        }
                    }
                    else if (system_->
                            save_spu_voice_sample_to_file(
                                sound_ram_voice_index_,
                                sound_ram_path.string(),
                                &error)) {
                        status_message_ =
                            "Saved sound.ram from SPU voice " +
                            std::to_string(
                                sound_ram_voice_index_);
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
            if (ImGui::Button("Load sound.ram")) {
                run_spu_sample_action([&]() {
                    std::string error;
                    if (system_->
                            load_spu_replacement_sample_from_file(
                                sound_ram_path.string(),
                                &error)) {
                        status_message_ =
                            "Loaded sound.ram replacement sample.";
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
            if (ImGui::Button("Clear Replacement")) {
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

            ImGui::Separator();
            ImGui::InputText(
                "Profile Name",
                sound_preset_name_,
                IM_ARRAYSIZE(
                    sound_preset_name_));
        }

        ImGui::Spacing();
        if (ImGui::Button(
                "Done",
                layout.size(120.0f, 32.0f))) {
            definitive_grim_reaper_advanced_ =
                false;
        }
    }

    ImGui::PopFont();
    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);

    // Bottom cards deliberately avoid repeating the configuration controls.
    draw_section_box(
        draw,
        layout,
        39.0f,
        650.0f,
        725.0f,
        118.0f,
        "LAST CORRUPTION");
    draw_section_box(
        draw,
        layout,
        782.0f,
        650.0f,
        458.0f,
        118.0f,
        "SYSTEM / REAPER INFO");

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

    const std::string seed_label =
        "Seed: " +
        std::to_string(display_seed);
    const std::string mutation_label =
        "Mutations: " +
        std::to_string(display_mutations);

    add_text(
        draw,
        layout,
        58.0f,
        704.0f,
        11.0f,
        rgba(196, 204, 213, 232),
        seed_label.c_str());
    add_text(
        draw,
        layout,
        58.0f,
        726.0f,
        11.0f,
        rgba(196, 204, 213, 232),
        mutation_label.c_str());

    if (!grim_reaper_last_output_path_.empty()) {
        const std::string file_label =
            "Last BIOS: " +
            std::filesystem::path(
                grim_reaper_last_output_path_)
                .filename()
                .string();
        add_text(
            draw,
            layout,
            250.0f,
            704.0f,
            10.5f,
            rgba(177, 186, 196, 220),
            file_label.c_str());

        ImGui::SetCursorScreenPos(
            layout.point(
                548.0f,
                711.0f));
        if (ImGui::Button(
                "Replay Last BIOS",
                layout.size(
                    180.0f,
                    32.0f))) {
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

    const std::string bios_label =
        std::string("BIOS: ") +
        (system_->bios_loaded()
            ? "Loaded"
            : "Not loaded");
    const std::string running_label =
        std::string("Emulator: ") +
        (has_started_emulation_
            ? (emu_runner_.is_running()
                ? "Running"
                : "Paused")
            : "Idle");
    const std::string mode_label =
        std::string("Mode: ") +
        kReaperStyleNames[
            static_cast<size_t>(style)];

    add_text(
        draw,
        layout,
        802.0f,
        704.0f,
        10.5f,
        rgba(196, 204, 213, 232),
        bios_label.c_str());
    add_text(
        draw,
        layout,
        802.0f,
        725.0f,
        10.5f,
        rgba(196, 204, 213, 232),
        running_label.c_str());
    add_text(
        draw,
        layout,
        997.0f,
        704.0f,
        10.5f,
        rgba(196, 204, 213, 232),
        mode_label.c_str());

    const std::string games_label =
        "Games Found: " +
        std::to_string(
            game_library_.size());
    add_text(
        draw,
        layout,
        997.0f,
        725.0f,
        10.5f,
        rgba(196, 204, 213, 232),
        games_label.c_str());
}
