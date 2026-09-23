#include "ui/app.h"
#include "platform/android_bridge.h"
#include "ui/theme_settings.h"

#include <SDL.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <string>

namespace {
struct MobileControlLayout {
    ImVec2 up{};
    ImVec2 right{};
    ImVec2 down{};
    ImVec2 left{};
    ImVec2 triangle{};
    ImVec2 circle{};
    ImVec2 cross{};
    ImVec2 square{};
    ImVec2 l1{};
    ImVec2 r1{};
    ImVec2 select{};
    ImVec2 start{};
    float radius = 1.0f;
    float pill_w = 1.0f;
    float pill_h = 1.0f;
};

MobileControlLayout make_mobile_layout(float width, float height) {
    MobileControlLayout out{};
    const bool landscape = width >= height;
    const float short_side = std::max(1.0f, std::min(width, height));
    out.radius = std::clamp(short_side * (landscape ? 0.065f : 0.060f),
        42.0f, 96.0f);
    out.pill_w = out.radius * 1.85f;
    out.pill_h = out.radius * 0.82f;

    const float dpad_x = width * (landscape ? 0.14f : 0.19f);
    const float face_x = width * (landscape ? 0.86f : 0.81f);
    const float center_y = height * (landscape ? 0.72f : 0.775f);
    const float step = out.radius * 1.25f;

    out.up = ImVec2(dpad_x, center_y - step);
    out.right = ImVec2(dpad_x + step, center_y);
    out.down = ImVec2(dpad_x, center_y + step);
    out.left = ImVec2(dpad_x - step, center_y);

    out.triangle = ImVec2(face_x, center_y - step);
    out.circle = ImVec2(face_x + step, center_y);
    out.cross = ImVec2(face_x, center_y + step);
    out.square = ImVec2(face_x - step, center_y);

    out.l1 = ImVec2(width * 0.16f, height * (landscape ? 0.14f : 0.60f));
    out.r1 = ImVec2(width * 0.84f, height * (landscape ? 0.14f : 0.60f));
    out.select = ImVec2(width * 0.43f, height * (landscape ? 0.90f : 0.91f));
    out.start = ImVec2(width * 0.57f, height * (landscape ? 0.90f : 0.91f));
    return out;
}

bool circle_hit(float x, float y, const ImVec2& center, float radius) {
    const float dx = x - center.x;
    const float dy = y - center.y;
    return (dx * dx + dy * dy) <= radius * radius;
}

bool pill_hit(float x, float y, const ImVec2& center, float width, float height) {
    return std::abs(x - center.x) <= width * 0.5f &&
        std::abs(y - center.y) <= height * 0.5f;
}

u16 hit_mobile_control(float x, float y, float width, float height) {
    const MobileControlLayout c = make_mobile_layout(width, height);
    const struct CircleButton {
        ImVec2 center;
        PsxButton button;
    } circles[] = {
        {c.up, PsxButton::Up},
        {c.right, PsxButton::Right},
        {c.down, PsxButton::Down},
        {c.left, PsxButton::Left},
        {c.triangle, PsxButton::Triangle},
        {c.circle, PsxButton::Circle},
        {c.cross, PsxButton::Cross},
        {c.square, PsxButton::Square},
    };
    for (const auto& item : circles) {
        if (circle_hit(x, y, item.center, c.radius)) {
            return static_cast<u16>(item.button);
        }
    }
    if (pill_hit(x, y, c.l1, c.pill_w, c.pill_h)) {
        return static_cast<u16>(PsxButton::L1);
    }
    if (pill_hit(x, y, c.r1, c.pill_w, c.pill_h)) {
        return static_cast<u16>(PsxButton::R1);
    }
    if (pill_hit(x, y, c.select, c.pill_w, c.pill_h)) {
        return static_cast<u16>(PsxButton::Select);
    }
    if (pill_hit(x, y, c.start, c.pill_w, c.pill_h)) {
        return static_cast<u16>(PsxButton::Start);
    }
    return 0;
}

void draw_centered_text(ImDrawList* draw, const ImVec2& center,
    ImU32 color, const char* text) {
    const ImVec2 size = ImGui::CalcTextSize(text);
    draw->AddText(ImVec2(center.x - size.x * 0.5f,
        center.y - size.y * 0.5f), color, text);
}
}

void App::mobile_top_bar() {
#if defined(__ANDROID__)
    const float bar_h = ImGui::GetFrameHeight() * 1.45f;
    const ImVec4 title_color =
        ui_theme::current_startup_title_color(ui_theme::g_theme_settings);

    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,
        ImVec2(7.0f * mobile_ui_scale_, 4.0f * mobile_ui_scale_));
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(title_color, "VibeStation");

    if (has_started_emulation_) {
        ImGui::SameLine();
        if (emu_runner_.is_running()) {
            if (ImGui::Button("Pause", ImVec2(0.0f, bar_h))) {
                emu_runner_.pause_and_wait_idle();
                status_message_ = "Emulation paused";
            }
        }
        else {
            if (ImGui::Button("Resume", ImVec2(0.0f, bar_h))) {
                emu_runner_.set_running(true);
                status_message_ = "Emulation resumed";
            }
        }

        ImGui::SameLine();
        if (ImGui::Button("Stop", ImVec2(0.0f, bar_h))) {
            emu_runner_.pause_and_wait_idle();
            disable_ram_reaper_mode();
            disable_gpu_reaper_mode();
            disable_sound_reaper_mode();
            has_started_emulation_ = false;
            mobile_touch_buttons_ = 0xFFFF;
            mobile_touch_fingers_.clear();
            status_message_ = "Emulation stopped";
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("Settings", ImVec2(0.0f, bar_h))) {
        show_settings_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reaper", ImVec2(0.0f, bar_h))) {
        show_grim_reaper_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("More", ImVec2(0.0f, bar_h))) {
        ImGui::OpenPopup("MobileMoreMenu");
    }
    if (ImGui::BeginPopup("MobileMoreMenu")) {
        ImGui::MenuItem("Performance overlay", nullptr, &show_perf_);
        if (ImGui::MenuItem("Performance profiler", nullptr,
            show_perf_profiler_)) {
            show_perf_profiler_ = !show_perf_profiler_;
            g_profile_detailed_timing = show_perf_profiler_;
        }
        ImGui::MenuItem("VRAM viewer", nullptr, &show_vram_);
        ImGui::MenuItem("CPU debug", nullptr, &show_debug_cpu_);
        ImGui::MenuItem("Touch controls", nullptr,
            &mobile_touch_controls_enabled_);
        ImGui::Separator();
        ImGui::MenuItem("About", nullptr, &show_about_);
        if (ImGui::MenuItem("Exit")) {
            SDL_Event quit_event{};
            quit_event.type = SDL_QUIT;
            SDL_PushEvent(&quit_event);
        }
        ImGui::EndPopup();
    }
    ImGui::PopStyleVar();
#else
    (void)this;
#endif
}

void App::panel_emulator_screen_mobile() {
#if defined(__ANDROID__)
    const bool bios_loaded = system_ != nullptr && system_->bios_loaded();
    const bool disc_loaded = system_ != nullptr && system_->disc_loaded();

    if (!has_started_emulation_) {
        const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
        const bool landscape = viewport_size.x > viewport_size.y;
        const bool selected_game = !game_bin_path_.empty() || disc_loaded;

        if (game_library_dirty_ ||
            (rom_directory_valid_ &&
                (SDL_GetTicks() - game_library_last_scan_ms_ > 15000u))) {
            refresh_game_library();
        }

        std::string selected_game_name = "None";
        if (selected_game) {
            bool library_match = false;
            for (const auto& entry : game_library_) {
                const bool bin_match =
                    !game_bin_path_.empty() && entry.bin_path == game_bin_path_;
                const bool cue_match =
                    !game_cue_path_.empty() && entry.cue_path == game_cue_path_;
                if (bin_match || cue_match) {
                    selected_game_name = entry.title;
                    library_match = true;
                    break;
                }
            }

            if (!library_match && !game_bin_path_.empty()) {
                selected_game_name =
                    std::filesystem::path(game_bin_path_).filename().string();
            }
            else if (!library_match && !game_cue_path_.empty()) {
                selected_game_name =
                    std::filesystem::path(game_cue_path_).filename().string();
            }
            else if (!library_match) {
                selected_game_name = "Loaded disc";
            }
        }

        const ImVec4 title_color =
            ui_theme::current_startup_title_color(ui_theme::g_theme_settings);
        const ImVec4 text_color =
            ui_theme::current_startup_text_color(ui_theme::g_theme_settings);
        const float touch_gap = 10.0f * mobile_ui_scale_;
        const float action_button_h =
            std::max(ImGui::GetFrameHeight() * 1.65f,
                60.0f * mobile_ui_scale_);
        const float library_row_h =
            std::max(ImGui::GetFrameHeight() * 1.15f,
                46.0f * mobile_ui_scale_);

        auto draw_status_panel = [&](const char* id, float height) {
            ImGui::BeginChild(id, ImVec2(0.0f, height), true);

            ImGui::TextColored(title_color, "BIOS");
            ImGui::PushStyleColor(ImGuiCol_Text,
                bios_loaded
                    ? ImVec4(0.45f, 0.90f, 0.68f, 1.0f)
                    : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
            ImGui::TextWrapped("%s",
                bios_loaded ? system_->bios().get_info().c_str() : "Not loaded");
            ImGui::PopStyleColor();

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::TextColored(title_color, "Game");
            ImGui::PushStyleColor(ImGuiCol_Text,
                selected_game
                    ? text_color
                    : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
            ImGui::TextWrapped("%s", selected_game_name.c_str());
            ImGui::PopStyleColor();

            ImGui::EndChild();
        };

        auto draw_action_buttons = [&]() {
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,
                ImVec2(ImGui::GetStyle().ItemSpacing.x, touch_gap));

            const char* bios_label =
                bios_loaded ? "Change BIOS" : "Load BIOS";
            if (ImGui::Button(bios_label,
                ImVec2(ImGui::GetContentRegionAvail().x, action_button_h))) {
                open_file_dialog(
                    "BIOS Files (*.bin)\0*.bin\0All Files\0*.*\0",
                    "Select PS1 BIOS");
            }

            if (ImGui::Button("Load Game",
                ImVec2(ImGui::GetContentRegionAvail().x, action_button_h))) {
                open_file_dialog(
                    "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
                    "Select PS1 Game");
            }

            ImGui::BeginDisabled(!bios_loaded);
            const char* start_label =
                selected_game ? "Start Emulation" : "Start BIOS";
            if (ImGui::Button(start_label,
                ImVec2(ImGui::GetContentRegionAvail().x, action_button_h))) {
                if (selected_game) {
                    boot_disc_from_ui();
                }
                else {
                    start_bios_from_ui();
                }
            }
            ImGui::EndDisabled();

            if (ImGui::Button("Import ROM Folder",
                ImVec2(ImGui::GetContentRegionAvail().x, action_button_h))) {
                open_folder_dialog("Import ROM Folder");
            }

            ImGui::PopStyleVar();
        };

        auto draw_game_library = [&](const char* id, float height) {
            ImGui::BeginChild(id, ImVec2(0.0f, height), true);

            ImGui::TextColored(title_color, "Game Library");
            const std::string count_text =
                std::to_string(game_library_.size()) +
                (game_library_.size() == 1 ? " game" : " games");
            const float count_x =
                ImGui::GetWindowContentRegionMax().x -
                ImGui::CalcTextSize(count_text.c_str()).x;
            ImGui::SameLine();
            if (count_x > ImGui::GetCursorPosX()) {
                ImGui::SetCursorPosX(count_x);
            }
            ImGui::TextDisabled("%s", count_text.c_str());

            if (rom_directory_valid_) {
                ImGui::BeginDisabled(false);
                const float refresh_h =
                    std::max(ImGui::GetFrameHeight(),
                        38.0f * mobile_ui_scale_);
                if (ImGui::Button("Refresh",
                    ImVec2(120.0f * mobile_ui_scale_, refresh_h))) {
                    game_library_dirty_ = true;
                    refresh_game_library();
                }
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_Text,
                    ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
                ImGui::TextWrapped("%s", rom_directory_.c_str());
                ImGui::PopStyleColor();
            }

            ImGui::Separator();

            ImGui::BeginChild("##GameLibraryRows",
                ImVec2(0.0f, 0.0f), false);

            if (!rom_directory_valid_) {
                ImGui::Dummy(ImVec2(0.0f, touch_gap));
                ImGui::TextDisabled("No ROM directory configured.");
                ImGui::Spacing();
                if (ImGui::Button("Import ROM Folder",
                    ImVec2(std::min(ImGui::GetContentRegionAvail().x,
                                   260.0f * mobile_ui_scale_),
                           action_button_h * 0.85f))) {
                    open_folder_dialog("Import ROM Folder");
                }
            }
            else if (game_library_.empty()) {
                ImGui::Dummy(ImVec2(0.0f, touch_gap));
                ImGui::TextDisabled("No playable disc images found.");
                ImGui::Spacing();
                if (ImGui::Button("Import Another ROM Folder",
                    ImVec2(std::min(ImGui::GetContentRegionAvail().x,
                                   310.0f * mobile_ui_scale_),
                           action_button_h * 0.85f))) {
                    open_folder_dialog("Import ROM Folder");
                }
            }
            else {
                ImGuiListClipper clipper;
                clipper.Begin(
                    static_cast<int>(game_library_.size()),
                    library_row_h);
                while (clipper.Step()) {
                    for (int i = clipper.DisplayStart;
                         i < clipper.DisplayEnd; ++i) {
                        const auto& entry =
                            game_library_[static_cast<size_t>(i)];
                        const std::string label =
                            entry.title + "##mobile_library_" +
                            std::to_string(i);
                        if (ImGui::Selectable(label.c_str(), false,
                            ImGuiSelectableFlags_None,
                            ImVec2(0.0f, library_row_h))) {
                            load_disc_from_ui(
                                entry.bin_path, entry.cue_path);
                        }
                    }
                }
            }

            ImGui::EndChild();
            ImGui::EndChild();
        };

        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const ImVec2 origin = ImGui::GetCursorPos();

        if (landscape) {
            const float outer_margin =
                std::max(16.0f * mobile_ui_scale_, avail.x * 0.025f);
            const float column_gap =
                std::max(14.0f * mobile_ui_scale_, avail.x * 0.018f);
            const float header_h =
                ImGui::GetFrameHeight() * 2.55f;
            const float usable_w =
                std::max(1.0f, avail.x - outer_margin * 2.0f);
            const float columns_w =
                std::max(1.0f, usable_w - column_gap);
            const float left_w = columns_w * 0.38f;
            const float right_w = columns_w - left_w;
            const float body_h =
                std::max(ImGui::GetFrameHeight() * 6.0f,
                    avail.y - header_h - outer_margin);

            ImGui::SetCursorPos(ImVec2(
                origin.x + outer_margin,
                origin.y + outer_margin * 0.45f));
            ImGui::SetWindowFontScale(1.30f);
            ImGui::TextColored(title_color, "VibeStation");
            ImGui::SetWindowFontScale(1.0f);
            ImGui::TextColored(text_color,
                "PlayStation 1 emulator on Android.");

            const float body_y = origin.y + header_h;

            ImGui::SetCursorPos(ImVec2(
                origin.x + outer_margin, body_y));
            ImGui::BeginChild("MobileLandscapeLeft",
                ImVec2(left_w, body_h), false);

            const float status_h =
                std::max(ImGui::GetFrameHeight() * 4.7f,
                    138.0f * mobile_ui_scale_);
            draw_status_panel("LandscapeStatusPanel", status_h);
            ImGui::Dummy(ImVec2(0.0f, touch_gap));
            draw_action_buttons();

            if (!status_message_.empty()) {
                ImGui::Dummy(ImVec2(0.0f, touch_gap * 0.5f));
                ImGui::PushStyleColor(ImGuiCol_Text,
                    ImVec4(0.62f, 0.52f, 0.88f, 1.0f));
                ImGui::TextWrapped("%s", status_message_.c_str());
                ImGui::PopStyleColor();
            }
            ImGui::EndChild();

            ImGui::SetCursorPos(ImVec2(
                origin.x + outer_margin + left_w + column_gap,
                body_y));
            ImGui::BeginChild("MobileLandscapeRight",
                ImVec2(right_w, body_h), false);
            draw_game_library(
                "LandscapeGameLibraryPanel",
                ImGui::GetContentRegionAvail().y);
            ImGui::EndChild();
            return;
        }

        // Portrait keeps the existing stacked VibeStation structure, but
        // uses explicit touch-sized controls rather than globally inflating it.
        const float margin =
            std::max(16.0f * mobile_ui_scale_, avail.x * 0.045f);
        const float content_w =
            std::max(1.0f, avail.x - margin * 2.0f);
        ImGui::SetCursorPos(ImVec2(
            origin.x + margin, origin.y + touch_gap * 0.6f));

        ImGui::BeginChild("MobileHome",
            ImVec2(content_w, 0.0f), false,
            ImGuiWindowFlags_AlwaysUseWindowPadding);

        ImGui::SetWindowFontScale(1.42f);
        ImGui::TextColored(title_color, "VibeStation");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::TextColored(text_color,
            "PlayStation 1 emulator on Android.");
        ImGui::Dummy(ImVec2(0.0f, touch_gap * 0.35f));

        const float portrait_status_h =
            std::max(ImGui::GetFrameHeight() * 4.7f,
                136.0f * mobile_ui_scale_);
        draw_status_panel("PortraitStatusPanel", portrait_status_h);

        ImGui::Dummy(ImVec2(0.0f, touch_gap));
        draw_action_buttons();

        ImGui::Dummy(ImVec2(0.0f, touch_gap));
        const float remaining_h =
            ImGui::GetContentRegionAvail().y;
        const size_t visible_rows =
            std::min<size_t>(
                std::max<size_t>(game_library_.size(), 3u), 6u);
        const float desired_library_h =
            ImGui::GetFrameHeight() * 3.2f +
            library_row_h * static_cast<float>(visible_rows);
        const float portrait_library_h =
            std::max(ImGui::GetFrameHeight() * 4.5f,
                std::min(remaining_h, desired_library_h));
        draw_game_library(
            "PortraitGameLibraryPanel",
            portrait_library_h);

        ImGui::EndChild();
        return;
    }

    ImGui::TextColored(ImVec4(0.72f, 0.68f, 0.92f, 1.0f),
        "%s", status_message_.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("%.0f game / %.0f video FPS", game_fps_, video_fps_);

    ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
    const bool portrait = viewport_size.y > viewport_size.x;
    if (portrait && mobile_touch_controls_enabled_) {
        avail.y = std::max(80.0f, avail.y - viewport_size.y * 0.34f);
    }

    const float display_aspect = 4.0f / 3.0f;
    const float dst_aspect = avail.y > 0.0f ? avail.x / avail.y : display_aspect;
    ImVec2 draw_size = avail;
    if (dst_aspect > display_aspect) {
        draw_size.x = avail.y * display_aspect;
    }
    else {
        draw_size.y = avail.x / display_aspect;
    }
    const float x_pad = std::max(0.0f, (avail.x - draw_size.x) * 0.5f);
    const float y_pad = portrait
        ? 0.0f
        : std::max(0.0f, (avail.y - draw_size.y) * 0.5f);
    ImVec2 cursor = ImGui::GetCursorPos();
    ImGui::SetCursorPos(ImVec2(cursor.x + x_pad, cursor.y + y_pad));
    const ImVec2 image_pos = ImGui::GetCursorScreenPos();

    if (renderer_ != nullptr) {
        ImGui::Image((ImTextureID)(intptr_t)renderer_->get_texture_id(),
            draw_size, ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f));
    }
    draw_performance_overlay(image_pos, draw_size);
    draw_mobile_touch_overlay();
#else
    (void)this;
#endif
}

void App::draw_mobile_touch_overlay() {
#if defined(__ANDROID__)
    if (!mobile_touch_controls_enabled_ || !has_started_emulation_ ||
        show_settings_ || show_grim_reaper_) {
        return;
    }

    int width = 0;
    int height = 0;
    SDL_GetWindowSize(window_, &width, &height);
    if (width <= 0 || height <= 0) {
        return;
    }

    const MobileControlLayout c =
        make_mobile_layout(static_cast<float>(width), static_cast<float>(height));
    ImDrawList* draw = ImGui::GetForegroundDrawList();
    const ImU32 fill = IM_COL32(74, 64, 105, 108);
    const ImU32 outline = IM_COL32(192, 172, 255, 178);
    const ImU32 text = IM_COL32(240, 236, 255, 215);

    // D-pad: rounded directional pads instead of four overlapping circles.
    const struct DrawPad {
        ImVec2 center;
        const char* label;
    } pads[] = {
        {c.up, "^"}, {c.right, ">"}, {c.down, "v"}, {c.left, "<"},
    };
    const float pad_half = c.radius * 0.78f;
    for (const auto& item : pads) {
        const ImVec2 a(item.center.x - pad_half, item.center.y - pad_half);
        const ImVec2 b(item.center.x + pad_half, item.center.y + pad_half);
        draw->AddRectFilled(a, b, fill, c.radius * 0.30f);
        draw->AddRect(a, b, outline, c.radius * 0.30f, 0, 2.0f);
        draw_centered_text(draw, item.center, text, item.label);
    }

    const struct DrawFace {
        ImVec2 center;
        const char* label;
    } faces[] = {
        {c.triangle, "TRI"}, {c.circle, "O"},
        {c.cross, "X"}, {c.square, "SQ"},
    };
    for (const auto& item : faces) {
        draw->AddCircleFilled(item.center, c.radius * 0.90f, fill, 32);
        draw->AddCircle(item.center, c.radius * 0.90f, outline, 32, 2.0f);
        draw_centered_text(draw, item.center, text, item.label);
    }

    const struct DrawPill {
        ImVec2 center;
        const char* label;
    } pills[] = {
        {c.l1, "L1"}, {c.r1, "R1"}, {c.select, "SELECT"}, {c.start, "START"},
    };
    for (const auto& item : pills) {
        const ImVec2 a(item.center.x - c.pill_w * 0.5f,
            item.center.y - c.pill_h * 0.5f);
        const ImVec2 b(item.center.x + c.pill_w * 0.5f,
            item.center.y + c.pill_h * 0.5f);
        draw->AddRectFilled(a, b, fill, c.pill_h * 0.48f);
        draw->AddRect(a, b, outline, c.pill_h * 0.48f, 0, 2.0f);
        draw_centered_text(draw, item.center, text, item.label);
    }
#endif
}

void App::handle_mobile_touch_event(const SDL_Event& event) {
#if defined(__ANDROID__)
    if (!mobile_touch_controls_enabled_ || !has_started_emulation_ ||
        show_settings_ || show_grim_reaper_) {
        if (!mobile_touch_fingers_.empty()) {
            mobile_touch_fingers_.clear();
            mobile_touch_buttons_ = 0xFFFF;
        }
        return;
    }
    if (event.type != SDL_FINGERDOWN &&
        event.type != SDL_FINGERMOTION &&
        event.type != SDL_FINGERUP) {
        return;
    }

    const long long finger = static_cast<long long>(event.tfinger.fingerId);
    auto existing = mobile_touch_fingers_.find(finger);
    if (existing != mobile_touch_fingers_.end()) {
        mobile_touch_buttons_ = static_cast<u16>(
            mobile_touch_buttons_ | existing->second);
        mobile_touch_fingers_.erase(existing);
    }

    if (event.type == SDL_FINGERUP) {
        return;
    }

    int width = 0;
    int height = 0;
    SDL_GetWindowSize(window_, &width, &height);
    const float x = event.tfinger.x * static_cast<float>(width);
    const float y = event.tfinger.y * static_cast<float>(height);
    const u16 button = hit_mobile_control(
        x, y, static_cast<float>(width), static_cast<float>(height));
    if (button != 0) {
        mobile_touch_fingers_[finger] = button;
        mobile_touch_buttons_ = static_cast<u16>(mobile_touch_buttons_ & ~button);
    }
#else
    (void)event;
#endif
}

void App::handle_android_picker_results() {
#if defined(__ANDROID__)
    AndroidPickerResult result{};
    while (android_poll_picker_result(result)) {
        mobile_picker_busy_ = false;
        if (result.cancelled || result.path.empty()) {
            status_message_ = "Selection cancelled.";
            continue;
        }

        if (result.kind == AndroidPickerKind::Bios) {
            emu_runner_.pause_and_wait_idle();
            disable_ram_reaper_mode();
            disable_gpu_reaper_mode();
            disable_sound_reaper_mode();
            if (system_->load_bios(result.path)) {
                bios_path_ = result.path;
                save_persistent_config();
                has_started_emulation_ = false;
                set_grim_reaper_mode(false);
                status_message_ = "BIOS loaded: " + system_->bios().get_info();
            }
            else {
                status_message_ = "Failed to load selected BIOS.";
            }
            continue;
        }

        if (result.kind == AndroidPickerKind::Game) {
            std::string lower = result.display_name;
            std::transform(lower.begin(), lower.end(), lower.begin(),
                [](unsigned char c) {
                    return static_cast<char>(std::tolower(c));
                });
            if (lower.size() >= 4 &&
                lower.substr(lower.size() - 4) == ".cue") {
                status_message_ =
                    "Android direct picker: select the game's main .bin file. Use ROM Folder import for .cue/multitrack games.";
                continue;
            }
            load_disc_from_ui(result.path, "");
            status_message_ = "Game selected: " +
                (result.display_name.empty() ? std::string("disc image")
                                             : result.display_name);
            continue;
        }

        if (result.kind == AndroidPickerKind::RomDirectory) {
            rom_directory_ = result.path;
            game_library_dirty_ = true;
            save_persistent_config();
            refresh_game_library();
            status_message_ = "ROM folder imported: " +
                std::to_string(game_library_.size()) + " game(s)";
        }
    }
#endif
}
