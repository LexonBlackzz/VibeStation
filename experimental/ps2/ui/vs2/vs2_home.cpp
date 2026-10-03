#include "ui/vs2/vs2_frontend.h"

#include <array>

namespace ps2::ui::vs2 {

namespace {

constexpr int kItemCount = 6;
constexpr int kReaperItem = 3;
constexpr float kMenuX = 640.0f;
constexpr float kMenuY = 352.0f;
constexpr float kRowStep = 46.0f;
constexpr float kLabelSize = 30.0f;

struct Item {
    const char* label;
    const char* sub;
};

ImU32 lerp_color(ImU32 a, ImU32 b, float t) {
    const ImVec4 x = ImGui::ColorConvertU32ToFloat4(a);
    const ImVec4 y = ImGui::ColorConvertU32ToFloat4(b);
    return ImGui::ColorConvertFloat4ToU32(ImVec4(
        x.x + (y.x - x.x) * t, x.y + (y.y - x.y) * t,
        x.z + (y.z - x.z) * t, x.w + (y.w - x.w) * t));
}

// Soft glow: the label drawn faintly around itself, then on top.
void glow_text(ImDrawList* draw, FontRole role, float size, const ImVec2& p,
               ImU32 col, ImU32 glow, float glow_alpha, float radius, const char* s) {
    if (glow_alpha > 0.01f) {
        for (int i = 0; i < 8; ++i) {
            const float a = static_cast<float>(i) * 0.785398f;
            text(draw, role, size, ImVec2(p.x + std::cos(a) * radius, p.y + std::sin(a) * radius),
                 with_alpha(glow, glow_alpha * 0.16f), s);
        }
    }
    text(draw, role, size, p, col, s);
}

} // namespace

void Frontend::update_home(const Input& in, const Layout& layout) {
    const int before = home_sel_;
    if (in.up) home_sel_ = (home_sel_ + kItemCount - 1) % kItemCount;
    if (in.down) home_sel_ = (home_sel_ + 1) % kItemCount;
    if (in.wheel > 0.0f) home_sel_ = (home_sel_ + kItemCount - 1) % kItemCount;
    if (in.wheel < 0.0f) home_sel_ = (home_sel_ + 1) % kItemCount;

    if (in.clicked) {
        for (int i = 0; i < kItemCount; ++i) {
            const float d = static_cast<float>(i) - home_sel_anim_;
            if (std::fabs(d) > 3.5f) continue;
            const float y = kMenuY + d * kRowStep + 22.0f * saturate(d);
            const ImVec2 p0 = layout.point(kMenuX - 10, y - 6);
            const ImVec2 p1 = layout.point(kMenuX + 420, y + 40);
            if (ImGui::IsMouseHoveringRect(p0, p1, false)) {
                if (i == home_sel_) {
                    activate_home_item();
                    return;
                }
                home_sel_ = i;
                break;
            }
        }
    }
    if (home_sel_ != before) play_highlight_sound();

    if (in.accept) activate_home_item();
    else if (in.triangle) go(Screen::Version);
}

void Frontend::activate_home_item() {
    switch (home_sel_) {
    case 0:
        if (host_.session_active() && !host_.session_running()) {
            play_select_sound();
            host_.resume_session();
            game_entered_t0_ = now_;
            go(Screen::InGame, true);
        } else {
            start_session();
        }
        break;
    case 1:
        go(Screen::Browser);
        break;
    case 2: {
        play_open_sound();
        const std::string path = host_.pick_bios_file();
        if (path.empty()) break;
        if (host_.load_bios(path)) {
            settings_.bios_path = path;
            save_settings();
            play_select_sound();
            toast("BIOS loaded: " + std::filesystem::path(path).filename().string());
        } else {
            toast(host_.status_message());
        }
        break;
    }
    case kReaperItem:
        go(Screen::Reaper);
        break;
    case 4:
        go(Screen::Config);
        break;
    case 5:
        exit_sel_ = 0;
        go(Screen::Exit);
        break;
    default: break;
    }
}

void Frontend::draw_home(ImDrawList* draw, const Layout& layout) {
    home_sel_anim_ = approach(home_sel_anim_, static_cast<float>(home_sel_), 12.0f, dt_);

    // Reveal: the menu slides in, then the corner block, then the hints.
    const float t = static_cast<float>(now_ - reveal_t0_);
    const bool revealed_long_ago = reveal_t0_ < 0.0;
    const float menu_in = revealed_long_ago ? 1.0f : ease_out_cubic((t - 1.4f) / 0.6f);
    const float corner_in = revealed_long_ago ? 1.0f : smoothstep(1.7f, 2.5f, t);
    const float hints_in = revealed_long_ago ? 1.0f : smoothstep(2.0f, 2.8f, t);
    const float alpha = home_alpha_;

    const bool paused = host_.session_active() && !host_.session_running();
    const std::array<Item, kItemCount> items = {{
        paused ? Item{"Resume Game", "Return to the paused game"}
               : Item{"Start Emulation", "Load BIOS and start playing"},
        {"Load Game", "Choose a game from your library"},
        {"Change BIOS", "Manage BIOS files"},
        {"Grim Reaper", "Corrupt EE RAM, VRAM, SPU2 and BIOS"},
        {"Settings", "System Configuration"},
        {"Exit", "Close VibeStation 2"},
    }};

    const float label_size = layout.px(kLabelSize);
    const float sub_size = layout.px(13);
    for (int i = 0; i < kItemCount; ++i) {
        const float d = static_cast<float>(i) - home_sel_anim_;
        const float fade = std::max(0.0f, 1.0f - std::fabs(d) * 0.3f);
        const float a = fade * menu_in * alpha;
        if (a <= 0.01f) continue;
        const float y = kMenuY + d * kRowStep + 22.0f * saturate(d);
        const float x = kMenuX + std::fabs(d) * 8.0f + (1.0f - menu_in) * 120.0f;
        const float selected = saturate(1.0f - std::fabs(d));
        const ImU32 accent = i == kReaperItem ? color::kReaper : color::kSelect;
        const ImU32 col = lerp_color(color::kIdle, accent, selected);

        glow_text(draw, FontRole::Light, label_size, layout.point(x, y),
                  with_alpha(col, a), accent, selected * a, layout.px(2.0f), items[i].label);
        if (selected > 0.05f) {
            text(draw, FontRole::Regular, sub_size, layout.point(x + 2, y + 40),
                 with_alpha(color::kSub, a * selected * selected), items[i].sub);
        }
    }

    // Corner block, as in the PS1 definitive UI.
    if (corner_in * alpha > 0.01f) {
        const float a = corner_in * alpha;
        const float right = 1280.0f - 46.0f;
        const auto right_text = [&](FontRole role, float size, float y, ImU32 col, const char* s) {
            const ImVec2 ts = text_size(role, layout.px(size), s);
            text(draw, role, layout.px(size), ImVec2(layout.point(right, 0).x - ts.x, layout.point(0, y).y),
                 with_alpha(col, a), s);
        };
        right_text(FontRole::Light, 22, 38, IM_COL32(201, 211, 224, 255), "VibeStation 2");
        right_text(FontRole::Mono, 11, 76, IM_COL32(111, 123, 143, 255), kVersionLabel);
        right_text(FontRole::Mono, 11, 96, IM_COL32(111, 123, 143, 255), "128 BITS");
        right_text(FontRole::Mono, 11, 116, IM_COL32(111, 123, 143, 255), "ENDLESS NIGHTS");
        draw->AddLine(layout.point(right - 34, 140), layout.point(right, 140),
                      with_alpha(IM_COL32(70, 82, 106, 255), a), std::max(1.0f, layout.px(1)));
    }

    draw_hints(draw, layout, hints_in * alpha, {{'x', "Enter"}, {'t', "Version"}});
}

} // namespace ps2::ui::vs2
