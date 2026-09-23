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
    const float center_y = height * (landscape ? 0.72f : 0.80f);
    const float step = out.radius * 1.25f;

    out.up = ImVec2(dpad_x, center_y - step);
    out.right = ImVec2(dpad_x + step, center_y);
    out.down = ImVec2(dpad_x, center_y + step);
    out.left = ImVec2(dpad_x - step, center_y);

    out.triangle = ImVec2(face_x, center_y - step);
    out.circle = ImVec2(face_x + step, center_y);
    out.cross = ImVec2(face_x, center_y + step);
    out.square = ImVec2(face_x - step, center_y);

    out.l1 = ImVec2(width * 0.16f, height * (landscape ? 0.14f : 0.62f));
    out.r1 = ImVec2(width * 0.84f, height * (landscape ? 0.14f : 0.62f));
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
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const float margin = std::max(18.0f * mobile_ui_scale_, avail.x * 0.045f);
        const float content_w = std::max(1.0f, avail.x - margin * 2.0f);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + margin);

        ImGui::BeginChild("MobileHome", ImVec2(content_w, 0.0f), false,
            ImGuiWindowFlags_AlwaysUseWindowPadding);

        ImGui::Dummy(ImVec2(0.0f, ImGui::GetFrameHeight() * 0.7f));
        ImGui::SetWindowFontScale(1.55f);
        ImGui::TextColored(
            ui_theme::current_startup_title_color(ui_theme::g_theme_settings),
            "VibeStation");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::TextWrapped("PlayStation 1 emulation and corruption tools, now on Android.");
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("BIOS");
        ImGui::SameLine();
        ImGui::TextColored(
            bios_loaded ? ImVec4(0.45f, 0.90f, 0.55f, 1.0f)
                        : ImVec4(0.95f, 0.55f, 0.50f, 1.0f),
            "%s", bios_loaded ? system_->bios().get_info().c_str() : "Not loaded");

        const bool selected_game = !game_bin_path_.empty() || disc_loaded;
        ImGui::Text("Game");
        ImGui::SameLine();
        ImGui::TextColored(
            selected_game ? ImVec4(0.45f, 0.90f, 0.55f, 1.0f)
                          : ImVec4(0.70f, 0.70f, 0.76f, 1.0f),
            "%s", selected_game ? "Selected" : "None");

        ImGui::Spacing();
        const float button_h = ImGui::GetFrameHeight() * 1.65f;
        const char* bios_label = bios_loaded ? "Change BIOS" : "Load BIOS";
        if (ImGui::Button(bios_label, ImVec2(-1.0f, button_h))) {
            open_file_dialog(
                "BIOS Files (*.bin)\0*.bin\0All Files\0*.*\0",
                "Select PS1 BIOS");
        }

        if (ImGui::Button("Load Game (.bin)", ImVec2(-1.0f, button_h))) {
            open_file_dialog(
                "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
                "Select PS1 Game");
        }

        ImGui::BeginDisabled(!bios_loaded);
        const char* start_label = selected_game
            ? "Boot Selected Game"
            : "Start PlayStation BIOS";
        if (ImGui::Button(start_label, ImVec2(-1.0f, button_h))) {
            if (selected_game) {
                boot_disc_from_ui();
            }
            else {
                start_bios_from_ui();
            }
        }
        ImGui::EndDisabled();

        if (selected_game && bios_loaded) {
            if (ImGui::Button("Eject / Clear Selected Game",
                ImVec2(-1.0f, button_h * 0.88f))) {
                unload_disc_from_ui();
                has_started_emulation_ = false;
                emu_runner_.pause_and_wait_idle();
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (game_library_dirty_ ||
            (rom_directory_valid_ &&
                (SDL_GetTicks() - game_library_last_scan_ms_ > 15000u))) {
            refresh_game_library();
        }

        ImGui::TextColored(ImVec4(0.78f, 0.72f, 0.98f, 1.0f),
            "Game Library");
        if (rom_directory_valid_) {
            ImGui::TextWrapped("%zu game%s imported from %s",
                game_library_.size(), game_library_.size() == 1 ? "" : "s",
                rom_directory_.c_str());
        }
        else {
            ImGui::TextWrapped(
                "Import a folder containing .bin/.cue images. Android copies them into VibeStation's private game storage so the emulator can access them reliably.");
        }

        if (ImGui::Button(
            rom_directory_valid_ ? "Import Another ROM Folder" : "Import ROM Folder",
            ImVec2(-1.0f, button_h))) {
            open_folder_dialog("Import ROM Folder");
        }
        if (rom_directory_valid_) {
            if (ImGui::Button("Refresh Library", ImVec2(-1.0f, button_h * 0.86f))) {
                game_library_dirty_ = true;
                refresh_game_library();
            }
        }

        const float library_h = std::max(
            ImGui::GetFrameHeight() * 5.0f,
            ImGui::GetContentRegionAvail().y - ImGui::GetFrameHeight());
        ImGui::BeginChild("MobileGameLibrary", ImVec2(0.0f, library_h), true);
        if (!rom_directory_valid_) {
            ImGui::TextDisabled("No imported ROM folder yet.");
        }
        else if (game_library_.empty()) {
            ImGui::TextDisabled("No .bin/.cue games were found.");
        }
        else {
            for (size_t i = 0; i < game_library_.size(); ++i) {
                const auto& entry = game_library_[i];
                const std::string label =
                    entry.title + "##mobile_game_" + std::to_string(i);
                if (ImGui::Selectable(label.c_str(), false,
                    ImGuiSelectableFlags_None,
                    ImVec2(0.0f, ImGui::GetFrameHeight() * 1.35f))) {
                    load_disc_from_ui(entry.bin_path, entry.cue_path);
                }
            }
        }
        ImGui::EndChild();

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
    const float y_pad = std::max(0.0f, (avail.y - draw_size.y) * 0.5f);
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

    const struct DrawCircle {
        ImVec2 center;
        const char* label;
    } circles[] = {
        {c.up, "U"}, {c.right, "R"}, {c.down, "D"}, {c.left, "L"},
        {c.triangle, "T"}, {c.circle, "O"}, {c.cross, "X"}, {c.square, "S"},
    };
    for (const auto& item : circles) {
        draw->AddCircleFilled(item.center, c.radius, fill, 32);
        draw->AddCircle(item.center, c.radius, outline, 32, 2.0f);
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
