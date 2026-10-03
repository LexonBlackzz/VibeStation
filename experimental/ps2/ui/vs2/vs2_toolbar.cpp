// In-game chrome: the floating toolbar and the performance overlay. The
// toolbar matches VibeStation 1's (src/ui/panels/emulator_screen_panel.cpp):
// it rises from the bottom edge of the picture when the mouse comes near.

#include "ui/vs2/vs2_frontend.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace ps2::ui::vs2 {

namespace {

enum class ToolIcon { Pause, Play, FastForward, Skull, Camera, Folder, Tv, Restart, Exit, More };

void draw_tool_icon(ImDrawList* draw, ToolIcon icon, const ImVec2& c, float s, ImU32 color,
                    ImU32 cutout) {
    const float stroke = std::max(1.4f, 2.0f * s);
    switch (icon) {
    case ToolIcon::Pause:
        draw->AddRectFilled(ImVec2(c.x - 7 * s, c.y - 10 * s), ImVec2(c.x - 2 * s, c.y + 10 * s), color, 1.5f * s);
        draw->AddRectFilled(ImVec2(c.x + 2 * s, c.y - 10 * s), ImVec2(c.x + 7 * s, c.y + 10 * s), color, 1.5f * s);
        break;
    case ToolIcon::Play:
        draw->AddTriangleFilled(ImVec2(c.x - 7 * s, c.y - 11 * s), ImVec2(c.x - 7 * s, c.y + 11 * s),
                                ImVec2(c.x + 11 * s, c.y), color);
        break;
    case ToolIcon::FastForward:
        for (int i = 0; i < 2; ++i) {
            const float cx = c.x + (static_cast<float>(i) - 0.5f) * 13 * s;
            draw->AddTriangleFilled(ImVec2(cx + 7 * s, c.y), ImVec2(cx - 6 * s, c.y - 9 * s),
                                    ImVec2(cx - 6 * s, c.y + 9 * s), color);
        }
        break;
    case ToolIcon::Skull:
        draw->AddCircleFilled(ImVec2(c.x, c.y - 3 * s), 10 * s, color, 24);
        draw->AddRectFilled(ImVec2(c.x - 7 * s, c.y + 3 * s), ImVec2(c.x + 7 * s, c.y + 10 * s), color, 2 * s);
        draw->AddCircleFilled(ImVec2(c.x - 4 * s, c.y - 4 * s), 2.4f * s, cutout, 12);
        draw->AddCircleFilled(ImVec2(c.x + 4 * s, c.y - 4 * s), 2.4f * s, cutout, 12);
        draw->AddTriangleFilled(ImVec2(c.x, c.y - 0.5f * s), ImVec2(c.x - 2 * s, c.y + 3 * s),
                                ImVec2(c.x + 2 * s, c.y + 3 * s), cutout);
        draw->AddLine(ImVec2(c.x - 3 * s, c.y + 6 * s), ImVec2(c.x - 3 * s, c.y + 10 * s), cutout,
                      std::max(1.0f, 1.5f * s));
        draw->AddLine(ImVec2(c.x + 3 * s, c.y + 6 * s), ImVec2(c.x + 3 * s, c.y + 10 * s), cutout,
                      std::max(1.0f, 1.5f * s));
        break;
    case ToolIcon::Camera:
        draw->AddRect(ImVec2(c.x - 11 * s, c.y - 7 * s), ImVec2(c.x + 11 * s, c.y + 9 * s), color, 2.5f * s, 0, stroke);
        draw->AddRectFilled(ImVec2(c.x - 5 * s, c.y - 11 * s), ImVec2(c.x + 4 * s, c.y - 7 * s), color, 1.5f * s);
        draw->AddCircle(ImVec2(c.x, c.y + 1 * s), 5 * s, color, 18, stroke);
        break;
    case ToolIcon::Folder:
        draw->AddLine(ImVec2(c.x - 11 * s, c.y - 7 * s), ImVec2(c.x - 3 * s, c.y - 7 * s), color, stroke);
        draw->AddLine(ImVec2(c.x - 3 * s, c.y - 7 * s), ImVec2(c.x + 1 * s, c.y - 3 * s), color, stroke);
        draw->AddLine(ImVec2(c.x + 1 * s, c.y - 3 * s), ImVec2(c.x + 11 * s, c.y - 3 * s), color, stroke);
        draw->AddRect(ImVec2(c.x - 11 * s, c.y - 3 * s), ImVec2(c.x + 11 * s, c.y + 9 * s), color, 2 * s, 0, stroke);
        break;
    case ToolIcon::Tv:
        draw->AddRect(ImVec2(c.x - 11 * s, c.y - 8 * s), ImVec2(c.x + 11 * s, c.y + 8 * s), color, 3 * s, 0, stroke);
        draw->AddLine(ImVec2(c.x - 5 * s, c.y - 8 * s), ImVec2(c.x - 9 * s, c.y - 13 * s), color, stroke);
        draw->AddLine(ImVec2(c.x + 5 * s, c.y - 8 * s), ImVec2(c.x + 9 * s, c.y - 13 * s), color, stroke);
        draw->AddLine(ImVec2(c.x - 5 * s, c.y + 8 * s), ImVec2(c.x - 7 * s, c.y + 12 * s), color, stroke);
        draw->AddLine(ImVec2(c.x + 5 * s, c.y + 8 * s), ImVec2(c.x + 7 * s, c.y + 12 * s), color, stroke);
        break;
    case ToolIcon::Restart: {
        constexpr float kPi = 3.14159265358979323846f;
        draw->PathArcTo(c, 9.5f * s, -0.25f * kPi, 1.38f * kPi, 22);
        draw->PathStroke(color, 0, stroke);
        draw->AddTriangleFilled(ImVec2(c.x + 9.8f * s, c.y - 6.5f * s), ImVec2(c.x + 4 * s, c.y - 7 * s),
                                ImVec2(c.x + 8.5f * s, c.y - 1.8f * s), color);
        break;
    }
    case ToolIcon::Exit:
        draw->AddRect(ImVec2(c.x - 11 * s, c.y - 10 * s), ImVec2(c.x - 2 * s, c.y + 10 * s), color, 1.5f * s, 0, stroke);
        draw->AddLine(ImVec2(c.x - 5 * s, c.y), ImVec2(c.x + 10 * s, c.y), color, stroke);
        draw->AddLine(ImVec2(c.x + 10 * s, c.y), ImVec2(c.x + 5 * s, c.y - 5 * s), color, stroke);
        draw->AddLine(ImVec2(c.x + 10 * s, c.y), ImVec2(c.x + 5 * s, c.y + 5 * s), color, stroke);
        break;
    case ToolIcon::More:
        for (int i = -1; i <= 1; ++i) {
            draw->AddCircleFilled(ImVec2(c.x + static_cast<float>(i) * 8 * s, c.y), 2.2f * s, color, 12);
        }
        break;
    }
}

const char* ee_core_name(EeCore core) {
    switch (core) {
    case EeCore::Jit: return "JIT";
    case EeCore::Dynarec: return "Dynarec";
    default: return "Interpreter";
    }
}

} // namespace

void Frontend::leave_game_to(Screen next) {
    set_turbo_held(false);
    host_.pause_session();
    toolbar_visibility_ = 0.0f;
    toolbar_reveal_hold_ = 0.0f;
    if (next == Screen::Home) {
        home_sel_ = 0;
        home_sel_anim_ = 0.0f;
    }
    play_open_sound();
    go(next, true);
}

void Frontend::set_turbo_held(bool held) {
    if (held == turbo_held_) return;
    turbo_held_ = held;
    host_.set_turbo(held);
}

void Frontend::restart_session() {
    set_turbo_held(false);
    const bool booted = last_boot_disc_.empty() ? host_.start_bios_session()
                                                : host_.boot_disc(last_boot_disc_);
    if (booted && host_.session_active()) {
        edge_light_.reset();
        game_entered_t0_ = now_;
    } else {
        toast(host_.status_message());
        go(Screen::Home, true);
    }
}

void Frontend::draw_toolbar(const ImVec2& image_pos, const ImVec2& image_size, bool ps1) {
    if (image_size.x < 260.0f || image_size.y < 160.0f) return;

    constexpr int kButtons = 9;
    constexpr int kSeparators = 5;
    const float base_width = 28.0f + 46.0f * kButtons + 10.0f * (kButtons - 1 - kSeparators) +
                             18.0f * kSeparators;
    const float scale = std::min(std::clamp(image_size.x / 1050.0f, 0.58f, 1.0f),
                                 std::max(0.50f, (image_size.x - 24.0f) / base_width));
    const float button = 46.0f * scale;
    const float gap = 10.0f * scale;
    const float padding = 14.0f * scale;
    const float separator_space = 18.0f * scale;
    const float bar_h = 66.0f * scale;
    const float bar_w = padding * 2 + button * kButtons + gap * (kButtons - 1 - kSeparators) +
                        separator_space * kSeparators;

    const float image_bottom = image_pos.y + image_size.y;
    const float visible_y = image_bottom - bar_h - 14.0f * scale;
    const float hidden_y = image_bottom + 8.0f * scale;
    const float bar_x = image_pos.x + (image_size.x - bar_w) * 0.5f;

    const ImGuiIO& io = ImGui::GetIO();
    const auto inside = [](const ImVec2& p, const ImVec2& a, const ImVec2& b) {
        return p.x >= a.x && p.x <= b.x && p.y >= a.y && p.y <= b.y;
    };
    const auto ease = [](float v) { return 1.0f - std::pow(1.0f - std::clamp(v, 0.0f, 1.0f), 3.0f); };

    // Show while the mouse is over the bottom strip or the bar, or a popup is
    // open, and linger briefly after it leaves.
    const float previous_y = hidden_y + (visible_y - hidden_y) * ease(toolbar_visibility_);
    const bool trigger = inside(io.MousePos, ImVec2(bar_x, image_bottom - 34.0f * scale),
                                ImVec2(bar_x + bar_w, image_bottom));
    const bool over_bar = inside(io.MousePos, ImVec2(bar_x, previous_y),
                                 ImVec2(bar_x + bar_w, previous_y + bar_h));
    const bool popup_open =
        ImGui::IsPopupOpen("##vs2_toolbar_display") || ImGui::IsPopupOpen("##vs2_toolbar_more");
    if (trigger || over_bar || popup_open) toolbar_reveal_hold_ = 0.70f;
    else toolbar_reveal_hold_ = std::max(0.0f, toolbar_reveal_hold_ - dt_);
    const bool show = trigger || over_bar || popup_open || toolbar_reveal_hold_ > 0.0f;
    toolbar_visibility_ = show ? std::min(1.0f, toolbar_visibility_ + dt_ / 0.26f)
                               : std::max(0.0f, toolbar_visibility_ - dt_ / 0.31f);

    const float vis = toolbar_visibility_;
    const float bar_y = hidden_y + (visible_y - hidden_y) * ease(vis);

    // Hidden: a faint chevron marks where to hover.
    if (vis < 0.34f) {
        const int a = static_cast<int>(64.0f * (1.0f - vis / 0.34f));
        const ImVec2 c(image_pos.x + image_size.x * 0.5f, image_bottom - 10.0f * scale);
        ImDrawList* fg = ImGui::GetForegroundDrawList();
        const float th = std::max(1.0f, 1.8f * scale);
        fg->AddLine(ImVec2(c.x - 8 * scale, c.y + 3 * scale), ImVec2(c.x, c.y - 4 * scale), IM_COL32(235, 241, 248, a), th);
        fg->AddLine(ImVec2(c.x, c.y - 4 * scale), ImVec2(c.x + 8 * scale, c.y + 3 * scale), IM_COL32(235, 241, 248, a), th);
    }

    ImGui::SetNextWindowPos(ImVec2(bar_x, bar_y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(bar_w, bar_h), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::Begin("##vs2_toolbar", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground);
    ImGui::PopStyleVar(3);
    ImGui::PushFont(font(FontRole::Regular, 17.0f));

    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 win = ImGui::GetWindowPos();
    const ImVec2 win_end(win.x + bar_w, win.y + bar_h);
    const float rounding = bar_h * 0.5f;
    draw->AddRectFilled(ImVec2(win.x, win.y + 6 * scale), ImVec2(win_end.x, win_end.y + 6 * scale),
                        IM_COL32(0, 0, 0, static_cast<int>(76 * vis)), rounding);
    draw->AddRectFilled(win, win_end, IM_COL32(17, 21, 27, static_cast<int>(238 * vis)), rounding);
    draw->AddRect(win, win_end, IM_COL32(89, 101, 116, static_cast<int>(96 * vis)), rounding, 0,
                  std::max(1.0f, scale));

    struct Result {
        bool clicked = false;
        bool active = false;
    };
    const auto tool = [&](int index, const char* id, ToolIcon icon, float x, bool enabled,
                          bool selected, const char* tooltip) -> Result {
        const ImVec2 local(x, (bar_h - button) * 0.5f);
        const ImVec2 p0(win.x + local.x, win.y + local.y);
        const ImVec2 p1(p0.x + button, p0.y + button);
        ImGui::SetCursorPos(local);
        ImGui::PushID(id);
        if (!enabled) ImGui::BeginDisabled();
        const bool clicked = ImGui::InvisibleButton("##tool", ImVec2(button, button));
        const bool hovered = enabled && ImGui::IsItemHovered();
        const bool active = enabled && ImGui::IsItemActive();
        if (!enabled) ImGui::EndDisabled();

        const auto i = static_cast<std::size_t>(index);
        if (hovered && !toolbar_was_hovered_[i]) play_highlight_sound();
        toolbar_was_hovered_[i] = hovered;
        if (hovered || active) toolbar_reveal_hold_ = std::max(toolbar_reveal_hold_, 0.55f);

        float& mix = toolbar_hover_mix_[i];
        mix += ((hovered ? 1.0f : 0.0f) - mix) * std::clamp(dt_ * 14.0f, 0.0f, 1.0f);
        const float highlight = std::max(mix, (selected || index == 0) ? 0.72f : 0.0f);
        if (highlight > 0.01f) {
            const float r = button * 0.42f;
            draw->AddRectFilled(ImVec2(p0.x - 4 * scale, p0.y - 4 * scale),
                                ImVec2(p1.x + 4 * scale, p1.y + 4 * scale),
                                IM_COL32(26, 114, 255, static_cast<int>(40 * highlight * vis)), r);
            draw->AddRectFilled(p0, p1, IM_COL32(25, 34, 45, static_cast<int>((110 + 75 * highlight) * vis)), r);
            draw->AddRect(p0, p1, IM_COL32(65, 145, 255, static_cast<int>((105 + 125 * highlight) * vis)), r, 0,
                          std::max(1.0f, 1.6f * scale));
        }
        const ImU32 color = enabled ? IM_COL32(236, 241, 247, static_cast<int>(248 * vis))
                                    : IM_COL32(129, 137, 147, static_cast<int>(132 * vis));
        draw_tool_icon(draw, icon, ImVec2(p0.x + button * 0.5f, p0.y + button * 0.5f), scale, color,
                       IM_COL32(17, 21, 27, static_cast<int>(255 * vis)));
        if (hovered && tooltip != nullptr) ImGui::SetTooltip("%s", tooltip);
        ImGui::PopID();
        return {clicked, active};
    };
    const auto separator = [&](float x) {
        draw->AddLine(ImVec2(win.x + x, win.y + 14 * scale), ImVec2(win.x + x, win_end.y - 14 * scale),
                      IM_COL32(106, 117, 132, static_cast<int>(88 * vis)), std::max(1.0f, scale));
    };

    float x = padding;
    const auto next = [&](bool divider) {
        x += button + (divider ? separator_space : gap);
        if (divider) separator(x - separator_space * 0.5f);
    };

    // The PS1 core inside VibeStation 2 cannot pause or run fast.
    const bool running = host_.session_running();
    const Result pause = tool(0, "pause", running ? ToolIcon::Pause : ToolIcon::Play, x, !ps1,
                              !running, ps1 ? "PS1 games can't be paused here"
                                            : running ? "Pause emulation" : "Resume emulation");
    if (pause.clicked) {
        if (running) host_.pause_session();
        else host_.resume_session();
    }
    next(true);

    const Result turbo = tool(1, "turbo", ToolIcon::FastForward, x, running && !ps1, turbo_held_,
                              "Hold for speedup");
    set_turbo_held(turbo.active && running && !ps1);
    next(true);

    const Result reaper = tool(2, "reaper", ToolIcon::Skull, x, true, false, "Grim Reaper");
    next(false);
    const Result snapshot = tool(3, "snapshot", ToolIcon::Camera, x, true, false, "Take snapshot");
    next(false);
    const Result load = tool(4, "load", ToolIcon::Folder, x, true, false, "Load game");
    next(true);

    const bool display_open = ImGui::IsPopupOpen("##vs2_toolbar_display");
    const Result display = tool(5, "display", ToolIcon::Tv, x, true, display_open,
                                host_.display_filter() == 0 ? "Display: Sharp" : "Display: Smooth");
    if (display.clicked) {
        play_open_sound();
        ImGui::OpenPopup("##vs2_toolbar_display");
        toolbar_reveal_hold_ = 1.0f;
    }
    next(true);

    const Result restart = tool(6, "restart", ToolIcon::Restart, x, true, false, "Restart emulation");
    next(false);
    const Result exit = tool(7, "exit", ToolIcon::Exit, x, true, false, "Exit to menu");
    next(true);

    const bool more_open = ImGui::IsPopupOpen("##vs2_toolbar_more");
    const Result more = tool(8, "more", ToolIcon::More, x, true, more_open, "More");
    if (more.clicked) {
        play_open_sound();
        ImGui::OpenPopup("##vs2_toolbar_more");
        toolbar_reveal_hold_ = 1.0f;
    }

    Screen leave_to = screen_;
    ImGui::PushStyleVar(ImGuiStyleVar_PopupRounding, 8.0f * scale);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f * scale, 10.0f * scale));
    ImGui::PushStyleColor(ImGuiCol_PopupBg, IM_COL32(14, 18, 24, 247));
    ImGui::PushStyleColor(ImGuiCol_Border, IM_COL32(86, 102, 121, 175));
    ImGui::PushStyleColor(ImGuiCol_Header, IM_COL32(35, 75, 116, 195));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, IM_COL32(43, 92, 141, 220));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(232, 237, 243, 250));
    if (ImGui::BeginPopup("##vs2_toolbar_display")) {
        toolbar_reveal_hold_ = 0.8f;
        const int filter = host_.display_filter();
        if (ImGui::MenuItem("Sharp", nullptr, filter == 0)) {
            host_.set_display_filter(0);
            save_settings();
        }
        if (ImGui::MenuItem("Smooth", nullptr, filter == 1)) {
            host_.set_display_filter(1);
            save_settings();
        }
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("##vs2_toolbar_more")) {
        toolbar_reveal_hold_ = 0.8f;
        if (ImGui::MenuItem("Performance Overlay", nullptr, show_perf_)) show_perf_ = !show_perf_;
        if (ImGui::MenuItem("System Configuration")) leave_to = Screen::Config;
        if (ImGui::MenuItem("Version")) leave_to = Screen::Version;
        if (ImGui::MenuItem("Developer View")) host_.open_developer_view();
        ImGui::EndPopup();
    }
    ImGui::PopStyleColor(5);
    ImGui::PopStyleVar(2);

    ImGui::PopFont();
    ImGui::End();

    if (reaper.clicked) leave_to = Screen::Reaper;
    if (load.clicked) leave_to = Screen::Browser;
    if (exit.clicked) leave_to = Screen::Home;
    if (snapshot.clicked) {
        play_select_sound();
        host_.save_snapshot();
        toast(host_.status_message());
    }
    if (restart.clicked) {
        play_select_sound();
        restart_session();
    }
    if (leave_to != screen_) leave_game_to(leave_to);
}

void Frontend::draw_perf_overlay(ImDrawList* draw, const ImVec2& image_pos, const ImVec2& image_size) {
    if (image_size.x < 240.0f || image_size.y < 140.0f) return;
    char lines[3][96];
    std::snprintf(lines[0], sizeof(lines[0]), "%.1f FPS   %.0f%% speed%s", host_.frames_per_second(),
                  host_.speed_percent(), turbo_held_ ? "   TURBO" : "");
    std::snprintf(lines[1], sizeof(lines[1]), "EE: %s", ee_core_name(host_.ee_core()));
    std::snprintf(lines[2], sizeof(lines[2]), "GS: %s", host_.gpu_gs_enabled() ? "GPU (OpenGL)" : "Software");

    const float size = 13.0f;
    float width = 0.0f, height = 0.0f;
    for (const char* line : lines) {
        const ImVec2 ts = text_size(FontRole::Mono, size, line);
        width = std::max(width, ts.x);
        height += ts.y + 2.0f;
    }
    const ImVec2 p0(image_pos.x + 10.0f, image_pos.y + 10.0f);
    const ImVec2 p1(p0.x + width + 20.0f, p0.y + height + 14.0f);
    draw->AddRectFilled(p0, p1, IM_COL32(8, 8, 12, 220), 6.0f);
    draw->AddRect(p0, p1, IM_COL32(140, 140, 170, 250), 6.0f);
    float y = p0.y + 7.0f;
    for (int i = 0; i < 3; ++i) {
        text(draw, FontRole::Mono, size, ImVec2(p0.x + 10.0f, y),
             i == 0 ? IM_COL32(235, 235, 245, 255) : IM_COL32(170, 178, 192, 255), lines[i]);
        y += text_size(FontRole::Mono, size, lines[i]).y + 2.0f;
    }
}

} // namespace ps2::ui::vs2
