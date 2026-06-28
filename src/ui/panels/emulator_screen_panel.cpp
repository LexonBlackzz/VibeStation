#include "ui/app.h"
#include "ui/theme_settings.h"

#include <SDL.h>
#include <imgui.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

namespace {
    constexpr float kEmulatorScreenBottomOverscanPixels = 3.0f;
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
        if (latest_frame_rgba_.empty()) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Snapshot (F8)", ImVec2(130.0f, 0.0f))) {
            save_snapshot_png();
        }
        if (latest_frame_rgba_.empty()) {
            ImGui::EndDisabled();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%dx%d", std::max(0, latest_frame_width_),
            std::max(0, latest_frame_height_));
        ImGui::Spacing();

        if (show_fast_mode_notice_) {
            ImGui::TextColored(ImVec4(0.95f, 0.3f, 0.3f, 1.0f),
                "%s",
                g_gpu_fast_mode
                ? "Increase in underruns with Fast Mode, disable unnecessary logging!"
                : "Increase in underruns, enable Fast Mode!");
            ImGui::Spacing();
        }
        ImVec2 avail = ImGui::GetContentRegionAvail();
        // Safe presentation baseline for BIOS/logo recovery: fixed 4:3 letterbox.
        const float display_aspect = 4.0f / 3.0f;
        const float dst_aspect =
            (avail.y > 0.0f) ? (avail.x / avail.y) : display_aspect;
        ImVec2 draw_size = avail;
        if (dst_aspect > display_aspect) {
            draw_size.x = avail.y * display_aspect;
        }
        else {
            draw_size.y = (display_aspect > 0.0f) ? (avail.x / display_aspect) : avail.y;
        }
        const float x_pad = (avail.x - draw_size.x) * 0.5f;
        const float y_pad = (avail.y - draw_size.y) * 0.5f;
        ImVec2 cursor = ImGui::GetCursorPos();
        ImGui::SetCursorPos(ImVec2(cursor.x + x_pad, cursor.y + y_pad));
        const ImVec2 image_pos = ImGui::GetCursorScreenPos();
        const float overscan_v =
            (latest_frame_height_ > 0)
            ? std::min(0.02f,
                kEmulatorScreenBottomOverscanPixels /
                static_cast<float>(std::max(1, latest_frame_height_)))
            : 0.0f;
        ImGui::Image((ImTextureID)(intptr_t)renderer_->get_texture_id(), draw_size,
            ImVec2(0.0f, 0.0f), ImVec2(1.0f, std::max(0.0f, 1.0f - overscan_v)));
        draw_performance_overlay(image_pos, draw_size);
    }
}
