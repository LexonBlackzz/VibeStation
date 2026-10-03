#include "ui/app.h"
#include "ui/definitive/definitive_shared.h"

#include <SDL.h>
#include <imgui.h>

#include <algorithm>

// Switching from VibeStation 1 (this app) to VibeStation 2 (the PS2 app),
// which runs in the same window: the home screen's "VibeStation 2" button
// opens an experimental warning; on Continue the screen fades to black and
// the host switches apps.

namespace {

using definitive_ui::Layout;
using definitive_ui::rgba;

constexpr float kWarningAnimSeconds = 0.18f;
constexpr float kFadeOutSeconds = 0.5f;
constexpr float kActivationFadeSeconds = 0.6f;

void text_centered(ImDrawList* draw, const Layout& layout, float y, float size,
                   ImU32 color, const char* text) {
    ImFont* font = definitive_ui::font_for_size(layout.px(size));
    const ImVec2 ts = font->CalcTextSizeA(layout.px(size), FLT_MAX, 0.0f, text);
    draw->AddText(font, layout.px(size),
                  ImVec2(layout.point(640.0f, 0.0f).x - ts.x * 0.5f, layout.point(0.0f, y).y),
                  definitive_ui::text_color(color), text);
}

// A definitive-style button: dark fill, light outline when engaged.
bool dialog_button(ImDrawList* draw, const Layout& layout, float x, float y, float w,
                   const char* id, const char* label, bool focused) {
    const ImVec2 p = layout.point(x, y);
    const ImVec2 size = layout.size(w, 40.0f);
    ImGui::SetCursorScreenPos(p);
    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(0, 0, 0, 0));
    const bool pressed = ImGui::Button(id, size);
    ImGui::PopStyleColor(3);
    const bool engaged = focused || ImGui::IsItemHovered();

    draw->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y),
                        definitive_ui::surface_color(rgba(12, 17, 23, engaged ? 230 : 160)));
    draw->AddRect(p, ImVec2(p.x + size.x, p.y + size.y),
                  definitive_ui::accent_color(engaged ? rgba(211, 229, 246, 235) : rgba(120, 132, 148, 150)),
                  0.0f, 0, layout.px(engaged ? 1.35f : 1.0f));
    ImFont* font = definitive_ui::font_for_size(layout.px(15.0f));
    const ImVec2 ts = font->CalcTextSizeA(layout.px(15.0f), FLT_MAX, 0.0f, label);
    draw->AddText(font, layout.px(15.0f),
                  ImVec2(p.x + (size.x - ts.x) * 0.5f, p.y + (size.y - ts.y) * 0.5f),
                  definitive_ui::text_color(rgba(223, 225, 228, 250)), label);
    return pressed;
}

} // namespace

bool App::take_vs2_switch_request() {
    const bool requested = vs2_switch_requested_;
    vs2_switch_requested_ = false;
    return requested;
}

void App::on_deactivated() {
    // VibeStation 2 takes over the window: pause a running game so it is
    // silent and waiting when the user comes back.
    if (emu_runner_.is_running()) {
        emu_runner_.pause_and_wait_idle();
        status_message_ = "Emulation paused";
    }
    definitive_ui::stop_startup_sound();
    vs2_warning_open_ = false;
}

void App::on_activated() {
    ImGui::SetCurrentContext(imgui_context_);
    SDL_GL_SetSwapInterval(config_vsync_ ? 1 : 0);
    SDL_SetWindowTitle(window_, "VibeStation - PS1 Emulator");
    // Keys held during the switch must not stay latched in this context.
    ImGui::GetIO().ClearInputKeys();
    vs2_fade_out_ = -1.0f;
    activation_fade_ = 1.0f;
}

void App::draw_vs2_switch_overlay() {
    const float dt = std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImDrawList* fg = ImGui::GetForegroundDrawList();
    const ImVec2 v0 = viewport->Pos;
    const ImVec2 v1(viewport->Pos.x + viewport->Size.x, viewport->Pos.y + viewport->Size.y);

    // Coming back from VibeStation 2: fade in from black.
    if (activation_fade_ > 0.0f) {
        fg->AddRectFilled(v0, v1, IM_COL32(0, 0, 0, static_cast<int>(255.0f * definitive_ui::smoothstep01(activation_fade_))));
        activation_fade_ = std::max(0.0f, activation_fade_ - dt / kActivationFadeSeconds);
    }

    // Confirmed: fade to black, then hand over to the host.
    if (vs2_fade_out_ >= 0.0f) {
        vs2_fade_out_ += dt;
        const float k = definitive_ui::smoothstep01(vs2_fade_out_ / kFadeOutSeconds);
        fg->AddRectFilled(v0, v1, IM_COL32(0, 0, 0, static_cast<int>(255.0f * k)));
        if (vs2_fade_out_ >= kFadeOutSeconds + 0.1f) {
            vs2_fade_out_ = -1.0f;
            vs2_switch_requested_ = true;
        }
        return;
    }

    vs2_warning_anim_ = std::clamp(
        vs2_warning_anim_ + (vs2_warning_open_ ? dt : -dt) / kWarningAnimSeconds, 0.0f, 1.0f);
    if (vs2_warning_anim_ <= 0.0f) {
        return;
    }
    const float a = definitive_ui::smoothstep01(vs2_warning_anim_);

    // A full-window ImGui window on top, so nothing underneath takes clicks.
    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(viewport->Size);
    if (vs2_warning_open_) ImGui::SetNextWindowFocus();
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::Begin("##vs2_experimental_warning", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBackground |
                     ImGuiWindowFlags_NoNav);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const Layout layout = definitive_ui::make_layout(viewport->Pos, viewport->Size);

    draw->AddRectFilled(v0, v1, IM_COL32(0, 0, 0, static_cast<int>(165.0f * a)));
    const float rise = (1.0f - a) * 12.0f;
    const ImVec2 p0 = layout.point(380.0f, 270.0f + rise);
    const ImVec2 p1 = layout.point(900.0f, 520.0f + rise);
    draw->AddRectFilled(p0, p1, definitive_ui::surface_color(rgba(8, 11, 15, static_cast<int>(245.0f * a))));
    draw->AddRect(p0, p1, definitive_ui::accent_color(rgba(180, 190, 205, static_cast<int>(110.0f * a))),
                  0.0f, 0, layout.px(1.0f));

    const auto with_a = [a](ImU32 c) {
        return (c & ~IM_COL32_A_MASK) | (static_cast<ImU32>(((c >> IM_COL32_A_SHIFT) & 0xFF) * a) << IM_COL32_A_SHIFT);
    };
    text_centered(draw, layout, 300.0f + rise, 24.0f, with_a(rgba(223, 225, 228, 250)),
                  "VibeStation 2 is experimental");
    text_centered(draw, layout, 350.0f + rise, 14.0f, with_a(rgba(176, 183, 191, 240)),
                  "The PS2 emulator is early work in progress. Many games will");
    text_centered(draw, layout, 372.0f + rise, 14.0f, with_a(rgba(176, 183, 191, 240)),
                  "not boot yet, and the ones that do may run slowly or glitch.");
    text_centered(draw, layout, 404.0f + rise, 13.0f, with_a(rgba(140, 148, 160, 230)),
                  "You can switch back to VibeStation 1 from its menu at any time.");

    static int focus = 0; // 0 = Continue, 1 = Cancel
    bool cont = false;
    bool cancel = false;
    if (vs2_warning_open_) {
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow) || ImGui::IsKeyPressed(ImGuiKey_RightArrow) ||
            ImGui::IsKeyPressed(ImGuiKey_GamepadDpadLeft) || ImGui::IsKeyPressed(ImGuiKey_GamepadDpadRight) ||
            ImGui::IsKeyPressed(ImGuiKey_Tab)) {
            focus ^= 1;
            definitive_ui::play_cursor_sound();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_GamepadFaceDown, false)) {
            (focus == 0 ? cont : cancel) = true;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false) || ImGui::IsKeyPressed(ImGuiKey_GamepadFaceRight, false)) {
            cancel = true;
        }
    }
    cont |= dialog_button(draw, layout, 470.0f, 450.0f + rise, 160.0f, "##vs2_continue", "Continue", focus == 0);
    cancel |= dialog_button(draw, layout, 650.0f, 450.0f + rise, 160.0f, "##vs2_cancel", "Cancel", focus == 1);

    ImGui::End();
    ImGui::PopStyleVar(2);

    if (!vs2_warning_open_) return;
    if (cont) {
        play_ui_open_sound();
        definitive_ui::stop_startup_sound();
        vs2_warning_open_ = false;
        vs2_warning_anim_ = 0.0f;
        focus = 0;
        if (hosted_) {
            vs2_fade_out_ = 0.0f;
        } else {
            status_message_ = "VibeStation 2 is not available in this build.";
        }
    } else if (cancel) {
        play_ui_close_sound();
        vs2_warning_open_ = false;
        focus = 0;
    }
}

void App::begin_vs2_switch() {
    if (hosted_ && vs2_fade_out_ < 0.0f) {
        vs2_warning_open_ = false;
        vs2_fade_out_ = 0.0f;
    }
}
