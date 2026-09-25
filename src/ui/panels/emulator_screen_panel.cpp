#include "ui/app.h"
#include "ui/definitive/definitive_shared.h"
#include "ui/theme_settings.h"

#include <SDL.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

namespace {
    constexpr float kEmulatorScreenBottomOverscanPixels = 3.0f;

    enum class GameplayToolbarIcon {
        Pause,
        Play,
        Rewind,
        FastForward,
        Skull,
        Camera,
        Folder,
        Restart,
        Exit,
        More
    };

    void draw_gameplay_toolbar_icon(
        ImDrawList* draw,
        GameplayToolbarIcon icon,
        const ImVec2& center,
        float scale,
        ImU32 color,
        ImU32 cutout_color) {
        const float s = scale;
        const float stroke = std::max(1.4f, 2.0f * s);

        switch (icon) {
        case GameplayToolbarIcon::Pause: {
            draw->AddRectFilled(
                ImVec2(center.x - 7.0f * s, center.y - 10.0f * s),
                ImVec2(center.x - 2.0f * s, center.y + 10.0f * s),
                color, 1.5f * s);
            draw->AddRectFilled(
                ImVec2(center.x + 2.0f * s, center.y - 10.0f * s),
                ImVec2(center.x + 7.0f * s, center.y + 10.0f * s),
                color, 1.5f * s);
            break;
        }
        case GameplayToolbarIcon::Play: {
            draw->AddTriangleFilled(
                ImVec2(center.x - 7.0f * s, center.y - 11.0f * s),
                ImVec2(center.x - 7.0f * s, center.y + 11.0f * s),
                ImVec2(center.x + 11.0f * s, center.y),
                color);
            break;
        }
        case GameplayToolbarIcon::Rewind:
        case GameplayToolbarIcon::FastForward: {
            const float dir =
                icon == GameplayToolbarIcon::FastForward ? 1.0f : -1.0f;
            for (int i = 0; i < 2; ++i) {
                const float offset =
                    (static_cast<float>(i) - 0.5f) * 13.0f * s;
                const float cx = center.x + offset;
                draw->AddTriangleFilled(
                    ImVec2(cx - dir * 7.0f * s, center.y),
                    ImVec2(cx + dir * 6.0f * s, center.y - 9.0f * s),
                    ImVec2(cx + dir * 6.0f * s, center.y + 9.0f * s),
                    color);
            }
            break;
        }
        case GameplayToolbarIcon::Skull: {
            draw->AddCircleFilled(
                ImVec2(center.x, center.y - 3.0f * s),
                10.0f * s, color, 24);
            draw->AddRectFilled(
                ImVec2(center.x - 7.0f * s, center.y + 3.0f * s),
                ImVec2(center.x + 7.0f * s, center.y + 10.0f * s),
                color, 2.0f * s);
            draw->AddCircleFilled(
                ImVec2(center.x - 4.0f * s, center.y - 4.0f * s),
                2.4f * s, cutout_color, 12);
            draw->AddCircleFilled(
                ImVec2(center.x + 4.0f * s, center.y - 4.0f * s),
                2.4f * s, cutout_color, 12);
            draw->AddTriangleFilled(
                ImVec2(center.x, center.y - 0.5f * s),
                ImVec2(center.x - 2.0f * s, center.y + 3.0f * s),
                ImVec2(center.x + 2.0f * s, center.y + 3.0f * s),
                cutout_color);
            draw->AddLine(
                ImVec2(center.x - 3.0f * s, center.y + 6.0f * s),
                ImVec2(center.x - 3.0f * s, center.y + 10.0f * s),
                cutout_color, std::max(1.0f, 1.5f * s));
            draw->AddLine(
                ImVec2(center.x + 3.0f * s, center.y + 6.0f * s),
                ImVec2(center.x + 3.0f * s, center.y + 10.0f * s),
                cutout_color, std::max(1.0f, 1.5f * s));
            break;
        }
        case GameplayToolbarIcon::Camera: {
            draw->AddRect(
                ImVec2(center.x - 11.0f * s, center.y - 7.0f * s),
                ImVec2(center.x + 11.0f * s, center.y + 9.0f * s),
                color, 2.5f * s, 0, stroke);
            draw->AddRectFilled(
                ImVec2(center.x - 5.0f * s, center.y - 11.0f * s),
                ImVec2(center.x + 4.0f * s, center.y - 7.0f * s),
                color, 1.5f * s);
            draw->AddCircle(
                ImVec2(center.x, center.y + 1.0f * s),
                5.0f * s, color, 18, stroke);
            break;
        }
        case GameplayToolbarIcon::Folder: {
            draw->AddLine(
                ImVec2(center.x - 11.0f * s, center.y - 7.0f * s),
                ImVec2(center.x - 3.0f * s, center.y - 7.0f * s),
                color, stroke);
            draw->AddLine(
                ImVec2(center.x - 3.0f * s, center.y - 7.0f * s),
                ImVec2(center.x + 1.0f * s, center.y - 3.0f * s),
                color, stroke);
            draw->AddLine(
                ImVec2(center.x + 1.0f * s, center.y - 3.0f * s),
                ImVec2(center.x + 11.0f * s, center.y - 3.0f * s),
                color, stroke);
            draw->AddRect(
                ImVec2(center.x - 11.0f * s, center.y - 3.0f * s),
                ImVec2(center.x + 11.0f * s, center.y + 9.0f * s),
                color, 2.0f * s, 0, stroke);
            break;
        }
        case GameplayToolbarIcon::Restart: {
            constexpr float kPi = 3.14159265358979323846f;
            draw->PathArcTo(
                center,
                9.5f * s,
                -0.25f * kPi,
                1.38f * kPi,
                22);
            draw->PathStroke(color, 0, stroke);
            draw->AddTriangleFilled(
                ImVec2(center.x + 9.8f * s, center.y - 6.5f * s),
                ImVec2(center.x + 4.0f * s, center.y - 7.0f * s),
                ImVec2(center.x + 8.5f * s, center.y - 1.8f * s),
                color);
            break;
        }
        case GameplayToolbarIcon::Exit: {
            draw->AddRect(
                ImVec2(center.x - 11.0f * s, center.y - 10.0f * s),
                ImVec2(center.x - 2.0f * s, center.y + 10.0f * s),
                color, 1.5f * s, 0, stroke);
            draw->AddLine(
                ImVec2(center.x - 5.0f * s, center.y),
                ImVec2(center.x + 10.0f * s, center.y),
                color, stroke);
            draw->AddLine(
                ImVec2(center.x + 10.0f * s, center.y),
                ImVec2(center.x + 5.0f * s, center.y - 5.0f * s),
                color, stroke);
            draw->AddLine(
                ImVec2(center.x + 10.0f * s, center.y),
                ImVec2(center.x + 5.0f * s, center.y + 5.0f * s),
                color, stroke);
            break;
        }
        case GameplayToolbarIcon::More: {
            for (int i = -1; i <= 1; ++i) {
                draw->AddCircleFilled(
                    ImVec2(center.x + static_cast<float>(i) * 8.0f * s,
                        center.y),
                    2.2f * s, color, 12);
            }
            break;
        }
        }
    }
}

void App::draw_gameplay_toolbar(
    const ImVec2& image_pos, const ImVec2& image_size) {
    if (image_size.x < 260.0f || image_size.y < 160.0f) {
        return;
    }

    constexpr int kButtonCount = 9;
    constexpr int kSeparatorCount = 4;

    const float requested_scale =
        std::clamp(image_size.x / 1050.0f, 0.58f, 1.0f);
    const float base_bar_width =
        28.0f + 46.0f * static_cast<float>(kButtonCount) +
        10.0f * static_cast<float>(
            kButtonCount - 1 - kSeparatorCount) +
        18.0f * static_cast<float>(kSeparatorCount);
    const float fit_scale =
        std::max(0.50f, (image_size.x - 24.0f) / base_bar_width);
    const float scale =
        std::min(requested_scale, fit_scale);

    const float button_size = 46.0f * scale;
    const float gap = 10.0f * scale;
    const float padding = 14.0f * scale;
    const float separator_space = 18.0f * scale;
    const float bar_h = 66.0f * scale;
    const float bar_w =
        padding * 2.0f +
        button_size * static_cast<float>(kButtonCount) +
        gap * static_cast<float>(
            kButtonCount - 1 - kSeparatorCount) +
        separator_space * static_cast<float>(kSeparatorCount);

    const float image_bottom = image_pos.y + image_size.y;
    const float visible_y =
        image_bottom - bar_h - 14.0f * scale;
    const float hidden_y =
        image_bottom + 8.0f * scale;
    const float bar_x =
        image_pos.x + (image_size.x - bar_w) * 0.5f;

    const ImGuiIO& io = ImGui::GetIO();
    const float dt =
        std::clamp(io.DeltaTime, 0.0f, 0.05f);

    auto point_in_rect = [](const ImVec2& point,
        const ImVec2& p0, const ImVec2& p1) {
        return point.x >= p0.x && point.x <= p1.x &&
            point.y >= p0.y && point.y <= p1.y;
    };

    const float previous_eased =
        1.0f - std::pow(
            1.0f - std::clamp(
                gameplay_toolbar_visibility_, 0.0f, 1.0f),
            3.0f);
    const float previous_y =
        hidden_y + (visible_y - hidden_y) * previous_eased;

    const ImVec2 trigger0(
        image_pos.x + (image_size.x - bar_w) * 0.5f,
        image_bottom - 34.0f * scale);
    const ImVec2 trigger1(
        trigger0.x + bar_w,
        image_bottom);

    const bool trigger_hovered =
        point_in_rect(io.MousePos, trigger0, trigger1);
    const bool toolbar_hovered =
        point_in_rect(
            io.MousePos,
            ImVec2(bar_x, previous_y),
            ImVec2(bar_x + bar_w, previous_y + bar_h));
    const bool popup_open =
        ImGui::IsPopupOpen("##gameplay_toolbar_more");

    if (trigger_hovered || toolbar_hovered || popup_open) {
        gameplay_toolbar_reveal_hold_ = 0.70f;
    }
    else {
        gameplay_toolbar_reveal_hold_ =
            std::max(
                0.0f,
                gameplay_toolbar_reveal_hold_ - dt);
    }

    const bool should_show =
        trigger_hovered ||
        toolbar_hovered ||
        popup_open ||
        gameplay_toolbar_reveal_hold_ > 0.0f;

    constexpr float kRevealDuration = 0.26f;
    constexpr float kHideDuration = 0.31f;
    if (should_show) {
        gameplay_toolbar_visibility_ = std::min(
            1.0f,
            gameplay_toolbar_visibility_ +
                dt / kRevealDuration);
    }
    else {
        gameplay_toolbar_visibility_ = std::max(
            0.0f,
            gameplay_toolbar_visibility_ -
                dt / kHideDuration);
    }

    const float visibility =
        std::clamp(
            gameplay_toolbar_visibility_, 0.0f, 1.0f);
    const float eased =
        1.0f - std::pow(1.0f - visibility, 3.0f);
    const float bar_y =
        hidden_y + (visible_y - hidden_y) * eased;

    // When hidden, leave a subtle 25%-opacity chevron at bottom center so
    // players know where to hover without permanently covering gameplay.
    if (visibility < 0.34f) {
        const float arrow_fade =
            1.0f - visibility / 0.34f;
        const int arrow_alpha = static_cast<int>(
            64.0f * std::clamp(arrow_fade, 0.0f, 1.0f));
        const ImVec2 arrow_center(
            image_pos.x + image_size.x * 0.5f,
            image_bottom - 10.0f * scale);
        ImDrawList* foreground =
            ImGui::GetForegroundDrawList();
        foreground->AddLine(
            ImVec2(
                arrow_center.x - 8.0f * scale,
                arrow_center.y + 3.0f * scale),
            ImVec2(
                arrow_center.x,
                arrow_center.y - 4.0f * scale),
            IM_COL32(235, 241, 248, arrow_alpha),
            std::max(1.0f, 1.8f * scale));
        foreground->AddLine(
            ImVec2(
                arrow_center.x,
                arrow_center.y - 4.0f * scale),
            ImVec2(
                arrow_center.x + 8.0f * scale,
                arrow_center.y + 3.0f * scale),
            IM_COL32(235, 241, 248, arrow_alpha),
            std::max(1.0f, 1.8f * scale));
    }

    ImGui::SetNextWindowPos(
        ImVec2(bar_x, bar_y),
        ImGuiCond_Always);
    ImGui::SetNextWindowSize(
        ImVec2(bar_w, bar_h),
        ImGuiCond_Always);
    ImGui::PushStyleVar(
        ImGuiStyleVar_WindowPadding,
        ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(
        ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(
        ImGuiStyleVar_WindowBorderSize, 0.0f);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoScrollWithMouse |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoBackground;

    ImGui::Begin(
        "##DefinitiveGameplayToolbar",
        nullptr,
        flags);
    ImGui::PopStyleVar(3);

    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 win_pos = ImGui::GetWindowPos();
    const ImVec2 win_size = ImGui::GetWindowSize();
    const ImVec2 win_end(
        win_pos.x + win_size.x,
        win_pos.y + win_size.y);

    const float rounding = bar_h * 0.50f;
    const int body_alpha =
        static_cast<int>(238.0f * visibility);
    const int border_alpha =
        static_cast<int>(96.0f * visibility);
    const int shadow_alpha =
        static_cast<int>(76.0f * visibility);

    draw->AddRectFilled(
        ImVec2(
            win_pos.x,
            win_pos.y + 6.0f * scale),
        ImVec2(
            win_end.x,
            win_end.y + 6.0f * scale),
        IM_COL32(0, 0, 0, shadow_alpha),
        rounding);
    draw->AddRectFilled(
        win_pos,
        win_end,
        IM_COL32(17, 21, 27, body_alpha),
        rounding);
    draw->AddRect(
        win_pos,
        win_end,
        IM_COL32(89, 101, 116, border_alpha),
        rounding,
        0,
        std::max(1.0f, 1.0f * scale));

    static std::array<float, kButtonCount> hover_mix{};
    static std::array<bool, kButtonCount> was_hovered{};

    struct ButtonResult {
        bool clicked = false;
        bool hovered = false;
        bool active = false;
    };

    auto button = [&](int index,
        const char* id,
        GameplayToolbarIcon icon,
        float x,
        bool enabled,
        bool selected,
        const char* tooltip) -> ButtonResult {
        const ImVec2 local_pos(
            x,
            (bar_h - button_size) * 0.5f);
        const ImVec2 screen_pos(
            win_pos.x + local_pos.x,
            win_pos.y + local_pos.y);

        ImGui::SetCursorPos(local_pos);
        ImGui::PushID(id);
        if (!enabled) {
            ImGui::BeginDisabled();
        }
        const bool clicked =
            ImGui::InvisibleButton(
                "##gameplay_toolbar_button",
                ImVec2(button_size, button_size));
        const bool hovered =
            enabled && ImGui::IsItemHovered();
        const bool active =
            enabled && ImGui::IsItemActive();
        if (!enabled) {
            ImGui::EndDisabled();
        }

        if (hovered &&
            !was_hovered[static_cast<size_t>(index)]) {
            play_ui_cursor_sound();
        }
        was_hovered[static_cast<size_t>(index)] =
            hovered;

        if (hovered || active) {
            gameplay_toolbar_reveal_hold_ =
                std::max(
                    gameplay_toolbar_reveal_hold_,
                    0.55f);
        }

        const float target = hovered ? 1.0f : 0.0f;
        float& mix =
            hover_mix[static_cast<size_t>(index)];
        mix += (target - mix) *
            std::clamp(dt * 14.0f, 0.0f, 1.0f);

        const bool primary = index == 0;
        const bool lit = selected || primary;
        const float highlight =
            std::max(mix, lit ? 0.72f : 0.0f);

        const ImVec2 p0 = screen_pos;
        const ImVec2 p1(
            screen_pos.x + button_size,
            screen_pos.y + button_size);

        if (highlight > 0.01f) {
            const int glow_alpha =
                static_cast<int>(
                    80.0f * highlight * visibility);
            draw->AddRectFilled(
                ImVec2(
                    p0.x - 4.0f * scale,
                    p0.y - 4.0f * scale),
                ImVec2(
                    p1.x + 4.0f * scale,
                    p1.y + 4.0f * scale),
                IM_COL32(
                    26, 114, 255,
                    glow_alpha / 2),
                button_size * 0.42f);
            draw->AddRectFilled(
                p0,
                p1,
                IM_COL32(
                    25, 34, 45,
                    static_cast<int>(
                        (110.0f +
                            75.0f * highlight) *
                        visibility)),
                button_size * 0.42f);
            draw->AddRect(
                p0,
                p1,
                IM_COL32(
                    65, 145, 255,
                    static_cast<int>(
                        (105.0f +
                            125.0f * highlight) *
                        visibility)),
                button_size * 0.42f,
                0,
                std::max(1.0f, 1.6f * scale));
        }

        const int icon_alpha = enabled
            ? static_cast<int>(248.0f * visibility)
            : static_cast<int>(132.0f * visibility);
        const ImU32 icon_color = enabled
            ? IM_COL32(236, 241, 247, icon_alpha)
            : IM_COL32(129, 137, 147, icon_alpha);
        const ImU32 cutout =
            IM_COL32(
                17, 21, 27,
                static_cast<int>(
                    255.0f * visibility));

        draw_gameplay_toolbar_icon(
            draw,
            icon,
            ImVec2(
                screen_pos.x + button_size * 0.5f,
                screen_pos.y + button_size * 0.5f),
            scale,
            icon_color,
            cutout);

        if (hovered && tooltip != nullptr) {
            ImGui::SetTooltip("%s", tooltip);
        }

        ImGui::PopID();
        return ButtonResult{
            clicked, hovered, active};
    };

    auto separator = [&](float x) {
        const float y0 =
            win_pos.y + 14.0f * scale;
        const float y1 =
            win_end.y - 14.0f * scale;
        draw->AddLine(
            ImVec2(win_pos.x + x, y0),
            ImVec2(win_pos.x + x, y1),
            IM_COL32(
                106, 117, 132,
                static_cast<int>(
                    88.0f * visibility)),
            std::max(1.0f, scale));
    };

    float x = padding;

    const bool running = emu_runner_.is_running();
    const ButtonResult pause = button(
        0,
        "pause",
        running
            ? GameplayToolbarIcon::Pause
            : GameplayToolbarIcon::Play,
        x,
        true,
        !running,
        running
            ? "Pause emulation"
            : "Resume emulation");
    if (pause.clicked) {
        if (running) {
            emu_runner_.pause_and_wait_idle();
            status_message_ = "Emulation paused";
        }
        else {
            emu_runner_.set_running(true);
            status_message_ = "Emulation resumed";
        }
    }

    x += button_size + separator_space;
    separator(
        x - separator_space * 0.5f);

    const bool rewind_enabled =
        config_rewind_enabled_ &&
        emu_runner_.is_running();
    const ButtonResult rewind = button(
        1,
        "rewind",
        GameplayToolbarIcon::Rewind,
        x,
        rewind_enabled,
        gameplay_toolbar_rewind_active_,
        rewind_enabled
            ? "Hold to rewind"
            : "Enable Rewind in Settings first");
    const bool rewind_now =
        rewind_enabled && rewind.active;
    if (rewind_now !=
        gameplay_toolbar_rewind_active_) {
        gameplay_toolbar_rewind_active_ =
            rewind_now;
        emu_runner_.set_rewind_active(
            rewind_now);
    }

    x += button_size + gap;
    const ButtonResult turbo = button(
        2,
        "turbo",
        GameplayToolbarIcon::FastForward,
        x,
        emu_runner_.is_running(),
        gameplay_toolbar_turbo_active_ ||
            turbo_hold_active_,
        "Hold for speedup");
    const bool turbo_now =
        emu_runner_.is_running() && turbo.active;
    if (turbo_now !=
        gameplay_toolbar_turbo_active_) {
        gameplay_toolbar_turbo_active_ =
            turbo_now;
        apply_speed_override();
    }

    x += button_size + separator_space;
    separator(
        x - separator_space * 0.5f);

    const ButtonResult grim = button(
        3,
        "grim",
        GameplayToolbarIcon::Skull,
        x,
        true,
        definitive_grim_reaper_active_,
        "Grim Reaper");
    if (grim.clicked) {
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

    x += button_size + gap;
    const ButtonResult snapshot = button(
        4,
        "snapshot",
        GameplayToolbarIcon::Camera,
        x,
        !latest_frame_rgba_.empty(),
        false,
        "Take snapshot");
    if (snapshot.clicked) {
        save_snapshot_png();
    }

    x += button_size + gap;
    const ButtonResult load = button(
        5,
        "load_game",
        GameplayToolbarIcon::Folder,
        x,
        true,
        false,
        "Load game");
    if (load.clicked) {
        play_ui_open_sound();
        std::string path =
            open_file_dialog(
                "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
                "Select PS1 Game");
        play_ui_close_sound();

        if (!path.empty()) {
            std::string bin;
            std::string cue;
            std::string error;
            if (!resolve_disc_paths(
                    path, bin, cue, error)) {
                status_message_ = error;
            }
            else {
                load_disc_from_ui(bin, cue);
            }
        }
    }

    x += button_size + separator_space;
    separator(
        x - separator_space * 0.5f);

    const ButtonResult restart = button(
        6,
        "restart",
        GameplayToolbarIcon::Restart,
        x,
        system_->bios_loaded(),
        false,
        "Restart emulation");
    if (restart.clicked) {
        play_ui_open_sound();

        gameplay_toolbar_rewind_active_ = false;
        emu_runner_.set_rewind_active(false);
        gameplay_toolbar_turbo_active_ = false;
        apply_speed_override();

        const bool has_disc =
            system_->disc_loaded() ||
            !game_bin_path_.empty() ||
            !game_cue_path_.empty();
        if (has_disc) {
            boot_disc_from_ui();
        }
        else {
            start_bios_from_ui();
        }
    }

    x += button_size + gap;
    const ButtonResult exit = button(
        7,
        "exit",
        GameplayToolbarIcon::Exit,
        x,
        true,
        false,
        "Exit to launcher");
    if (exit.clicked) {
        play_ui_close_sound();

        gameplay_toolbar_rewind_active_ = false;
        emu_runner_.set_rewind_active(false);
        gameplay_toolbar_turbo_active_ = false;
        apply_speed_override();

        emu_runner_.pause_and_wait_idle();
        disable_ram_reaper_mode();
        disable_gpu_reaper_mode();
        disable_sound_reaper_mode();
        set_grim_reaper_mode(false);

        show_settings_ = false;
        show_vram_ = false;
        show_about_ = false;
        show_grim_reaper_ = false;
        show_perf_ = false;

        // Fade gameplay to black, switch to the launcher at full black, then
        // reveal the launcher. Keep has_started_emulation_ true until the
        // midpoint so the outgoing gameplay frame remains visible.
        definitive_grim_reaper_active_ = false;
        definitive_grim_reaper_closing_ = false;
        definitive_grim_reaper_advanced_ = false;
        definitive_grim_reaper_visibility_ = 0.0f;
        gameplay_exit_transition_active_ = true;
        gameplay_exit_transition_switched_ = false;
        gameplay_exit_transition_elapsed_ = 0.0f;
        gameplay_toolbar_visibility_ = 0.0f;
        gameplay_toolbar_reveal_hold_ = 0.0f;
        status_message_ = "Returning to launcher...";
    }

    x += button_size + separator_space;
    separator(
        x - separator_space * 0.5f);

    const ButtonResult more = button(
        8,
        "more",
        GameplayToolbarIcon::More,
        x,
        true,
        ImGui::IsPopupOpen(
            "##gameplay_toolbar_more"),
        "More");
    if (more.clicked) {
        play_ui_open_sound();
        ImGui::OpenPopup(
            "##gameplay_toolbar_more");
        gameplay_toolbar_reveal_hold_ = 1.0f;
    }

    ImGui::PushStyleVar(
        ImGuiStyleVar_PopupRounding,
        8.0f * scale);
    ImGui::PushStyleVar(
        ImGuiStyleVar_WindowPadding,
        ImVec2(
            10.0f * scale,
            10.0f * scale));
    ImGui::PushStyleColor(
        ImGuiCol_PopupBg,
        IM_COL32(14, 18, 24, 247));
    ImGui::PushStyleColor(
        ImGuiCol_Border,
        IM_COL32(86, 102, 121, 175));
    ImGui::PushStyleColor(
        ImGuiCol_Header,
        IM_COL32(35, 75, 116, 195));
    ImGui::PushStyleColor(
        ImGuiCol_HeaderHovered,
        IM_COL32(43, 92, 141, 220));
    ImGui::PushStyleColor(
        ImGuiCol_Text,
        IM_COL32(232, 237, 243, 250));

    if (ImGui::BeginPopup(
            "##gameplay_toolbar_more")) {
        gameplay_toolbar_reveal_hold_ = 0.8f;

        if (ImGui::MenuItem(
                "Performance Overlay",
                nullptr,
                show_perf_)) {
            show_perf_ = !show_perf_;
        }
        if (ImGui::MenuItem("Settings")) {
            play_ui_open_sound();
            show_settings_ = true;
        }
        if (ImGui::MenuItem(
                "Show VRAM",
                nullptr,
                show_vram_)) {
            show_vram_ = !show_vram_;
        }
        if (ImGui::MenuItem("About")) {
            show_about_ = true;
        }
        ImGui::EndPopup();
    }

    ImGui::PopStyleColor(5);
    ImGui::PopStyleVar(2);
    ImGui::End();
}

void App::panel_emulator_screen() {
    if (!has_started_emulation_) {
        const bool bios_loaded = system_->bios_loaded();
        const bool disc_loaded = system_->disc_loaded();

        // Show a centered welcome message
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        const char* logo_text = "VibeStation";
        const ImVec4 startup_title_color =
            ui_theme::current_startup_title_color(ui_theme::g_theme_settings);
        const ImVec4 startup_text_color =
            ui_theme::current_startup_text_color(ui_theme::g_theme_settings);
        const ImVec4 startup_text_secondary =
            ui_theme::theme_lerp(startup_text_color, ui_theme::g_theme_settings.background, 0.18f);
        ImGui::PushStyleColor(ImGuiCol_Text, startup_title_color);
        ImGui::SetWindowFontScale(2.0f);
        const ImVec2 logo_size = ImGui::CalcTextSize(logo_text);
        ImGui::SetCursorPos(ImVec2(center.x - (logo_size.x * 0.5f), center.y - 80));
        ImGui::Text("%s", logo_text);
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();

        ImGui::SetCursorPos(ImVec2(center.x - 180, center.y - 20));
        ImGui::TextColored(startup_text_color,
            "Load a BIOS (File > Load BIOS) to get started.");

        ImGui::SetCursorPos(ImVec2(center.x - 180, center.y + 5));
        ImGui::TextColored(startup_text_secondary,
            "Then load a game and use Emulation > Boot Disc.");

        const char* bios_button_label = bios_loaded ? "Change BIOS" : "Load BIOS";
        const ImVec2 button_size(120.0f, 0.0f);

        ImGui::SetCursorPos(ImVec2(center.x - 200, center.y + 42));
        if (ImGui::Button(bios_button_label, button_size)) {
            std::string path = open_file_dialog(
                "BIOS Files (*.bin)\0*.bin\0All Files\0*.*\0", "Select PS1 BIOS");
            if (!path.empty()) {
                emu_runner_.pause_and_wait_idle();
                disable_ram_reaper_mode();
                disable_gpu_reaper_mode();
                disable_sound_reaper_mode();
                if (system_->load_bios(path)) {
                    bios_path_ = path;
                    save_persistent_config();
                    has_started_emulation_ = false;
                    set_grim_reaper_mode(false);
                    status_message_ = "BIOS loaded: " + system_->bios().get_info();
                }
                else {
                    status_message_ = "Failed to load BIOS!";
                }
            }
        }

        ImGui::SameLine();
        if (!bios_loaded) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Load Game", button_size)) {
            std::string path = open_file_dialog(
                "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
                "Select PS1 Game");
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
        if (!bios_loaded) {
            ImGui::EndDisabled();
        }

        ImGui::SameLine();
        if (!bios_loaded) {
            ImGui::BeginDisabled();
        }
        const bool has_selected_game = !game_bin_path_.empty() || system_->disc_loaded();
        const char* start_button_label =
            has_selected_game ? "Boot Selected Game" : "Start Emulation";
        const ImVec2 start_button_size(150.0f, 0.0f);
        if (ImGui::Button(start_button_label, start_button_size)) {
            if (has_selected_game) {
                boot_disc_from_ui();
            }
            else {
                start_bios_from_ui();
            }
        }
        if (!bios_loaded) {
            ImGui::EndDisabled();
        }

        // Eject Disc button (visible when a game is selected and emulation is idle)
        if (bios_loaded && has_selected_game && !emu_runner_.is_running()) {
            ImGui::SameLine();
            const ImVec2 eject_button_size(100.0f, 0.0f);
            if (ImGui::Button("Eject Disc", eject_button_size)) {
                unload_disc_from_ui();
            }
        }

        const bool rom_dir_valid = rom_directory_valid_;
        if (game_library_dirty_ ||
            (rom_dir_valid &&
                (SDL_GetTicks() - game_library_last_scan_ms_ > 15000u))) {
            refresh_game_library();
        }

        ImGui::SetCursorPos(ImVec2(center.x - 300, center.y + 90));
        ImGui::BeginChild("IdleGameLibrary", ImVec2(600, 230), true);
        ImGui::TextColored(ImVec4(0.75f, 0.72f, 0.95f, 1.0f), "Game Library");
        if (rom_dir_valid) {
            ImGui::TextDisabled("ROM Directory: %s", rom_directory_.c_str());
        }
        else {
            ImGui::TextDisabled("ROM Directory: not set");
        }
        if (ImGui::Button("Set ROM Directory", ImVec2(150.0f, 0.0f))) {
            const std::string selected = open_folder_dialog("Select ROM Directory");
            if (!selected.empty()) {
                rom_directory_ = selected;
                game_library_dirty_ = true;
                save_persistent_config();
                refresh_game_library();
                status_message_ = "ROM directory set: " + rom_directory_;
            }
        }
        ImGui::SameLine();
        if (!rom_dir_valid) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Refresh", ImVec2(90.0f, 0.0f))) {
            game_library_dirty_ = true;
            refresh_game_library();
        }
        if (!rom_dir_valid) {
            ImGui::EndDisabled();
        }
        ImGui::Separator();

        if (!rom_dir_valid) {
            ImGui::TextColored(ImVec4(0.88f, 0.45f, 0.45f, 1.0f),
                "No ROM directory configured.");
            ImGui::TextWrapped("Set a ROM directory to scan and list games here.");
        }
        else if (game_library_.empty()) {
            ImGui::TextColored(ImVec4(0.85f, 0.75f, 0.45f, 1.0f),
                "No playable disc images found.");
        }
        else {
            for (size_t i = 0; i < game_library_.size(); ++i) {
                const auto& entry = game_library_[i];
                std::string label = entry.title + "##game_" + std::to_string(i);
                if (ImGui::Selectable(label.c_str(), false)) {
                    load_disc_from_ui(entry.bin_path, entry.cue_path);
                }
            }
        }
        ImGui::EndChild();

        if (!rom_dir_valid) {
            const char* warning =
                "No ROM directory in vibestation_config.ini. Set one to enable the game list.";
            const ImVec2 text_size = ImGui::CalcTextSize(warning);
            const float warning_y = ImGui::GetWindowHeight() - 60.0f;
            ImGui::SetCursorPos(ImVec2((ImGui::GetWindowWidth() - text_size.x) * 0.5f, warning_y));
            ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.45f, 1.0f), "%s", warning);

            const ImVec2 button_size(170.0f, 0.0f);
            ImGui::SetCursorPos(
                ImVec2((ImGui::GetWindowWidth() - button_size.x) * 0.5f, warning_y + 22.0f));
            if (ImGui::Button("Set ROM Directory##warning", button_size)) {
                const std::string selected = open_folder_dialog("Select ROM Directory");
                if (!selected.empty()) {
                    rom_directory_ = selected;
                    game_library_dirty_ = true;
                    save_persistent_config();
                    refresh_game_library();
                    status_message_ = "ROM directory set: " + rom_directory_;
                }
            }
        }
    }
    else {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        const ImVec2 gameplay_area_pos =
            ImGui::GetCursorScreenPos();

        // The gameplay surface is intentionally chrome-free. The only
        // persistent in-client control is the floating toolbar drawn over the
        // game image below.
        const float display_aspect = 4.0f / 3.0f;
        const float dst_aspect =
            (avail.y > 0.0f) ? (avail.x / avail.y) : display_aspect;
        ImVec2 draw_size = avail;
        if (dst_aspect > display_aspect) {
            draw_size.x = avail.y * display_aspect;
        }
        else {
            draw_size.y =
                (display_aspect > 0.0f)
                ? (avail.x / display_aspect)
                : avail.y;
        }

        const float x_pad = (avail.x - draw_size.x) * 0.5f;
        const float y_pad = (avail.y - draw_size.y) * 0.5f;
        ImVec2 cursor = ImGui::GetCursorPos();
        ImGui::SetCursorPos(
            ImVec2(cursor.x + x_pad, cursor.y + y_pad));

        const ImVec2 image_pos = ImGui::GetCursorScreenPos();
        const float overscan_v =
            (latest_frame_height_ > 0)
            ? std::min(
                0.02f,
                kEmulatorScreenBottomOverscanPixels /
                    static_cast<float>(
                        std::max(1, latest_frame_height_)))
            : 0.0f;

        definitive_ui::draw_gameplay_ambient(
            ImGui::GetWindowDrawList(),
            gameplay_area_pos,
            avail,
            image_pos,
            draw_size,
            overscan_v);

        ImGui::Image(
            (ImTextureID)(intptr_t)renderer_->get_texture_id(),
            draw_size,
            ImVec2(0.0f, 0.0f),
            ImVec2(
                1.0f,
                std::max(0.0f, 1.0f - overscan_v)));

        if (show_perf_) {
            draw_performance_overlay(image_pos, draw_size);
        }

        // Draw the floating controls last so they always sit above optional
        // diagnostics rather than being obscured by them.
        draw_gameplay_toolbar(image_pos, draw_size);
    }
}
