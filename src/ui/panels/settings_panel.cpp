#include "ui/app.h"
#include "ui/input_bindings.h"
#include "ui/output_resolution_utils.h"
#include "ui/screenshot_utils.h"
#include "ui/theme_settings.h"
#include <imgui.h>
#include <algorithm>
void App::panel_settings() {
    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Settings", &show_settings_)) {
        if (ImGui::BeginTabBar("SettingsTabs")) {
            if (ImGui::BeginTabItem("Input")) {
                ImGui::TextWrapped("Default: Arrows=D-Pad, Z/X/A/S=Face, "
                    "Q/W/E/R=Shoulders, Enter=Start, Backspace=Select");
                ImGui::Spacing();
                if (pending_bind_index_ >= 0) {
                    ImGui::TextColored(ImVec4(0.95f, 0.8f, 0.3f, 1.0f),
                        "Press a key for %s (Esc to cancel)",
                        kKeyboardBindEntries[pending_bind_index_].label);
                }
                if (ImGui::Button("Configure Bindings")) {
                    show_bindings_config_ = true;
                }
                ImGui::TextDisabled("Opens the keyboard binding editor in a separate window.");
                ImGui::Spacing();
                ImGui::Text("Input Focus: %s", emu_input_focused_ ? "Active" : "Inactive");
                ImGui::Text("Buttons (active-low): 0x%04X", last_button_state_);
                const auto& diag = runtime_snapshot_.boot_diag;
                ImGui::Text("SIO TX/RX: 0x%02X / 0x%02X",
                    static_cast<unsigned>(diag.last_sio_tx),
                    static_cast<unsigned>(diag.last_sio_rx));
                ImGui::Text("SIO tx42/full-poll: %s / %s",
                    diag.saw_tx_cmd42 ? "Yes" : "No",
                    diag.saw_full_pad_poll ? "Yes" : "No");
                ImGui::Spacing();
                ImGui::Text("Gamepad: %s", input_->gamepad_name().c_str());
                if (input_->has_gamepad()) {
                    ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f),
                        "Connected â€” auto-mapped");
                }
                else {
                    ImGui::TextColored(
                        ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                        "No gamepad detected. Connect one and it will auto-bind.");
                }

                ImGui::Separator();
                ImGui::Text("Input Movie");
                InputRecorder::Status movie_status = input_recorder_.status();
                const ImVec4 state_color =
                    movie_status.state == InputRecorder::State::Error
                    ? ImVec4(0.95f, 0.35f, 0.35f, 1.0f)
                    : movie_status.state == InputRecorder::State::Recording
                    ? ImVec4(0.95f, 0.35f, 0.35f, 1.0f)
                    : movie_status.state == InputRecorder::State::Playing
                    ? ImVec4(0.35f, 0.9f, 0.5f, 1.0f)
                    : ImVec4(0.75f, 0.75f, 0.8f, 1.0f);
                ImGui::TextColored(state_color, "State: %s",
                    InputRecorder::state_name(movie_status.state));
                ImGui::TextWrapped("Replay folder: %s",
                    movie_status.replay_folder_path.c_str());

                ImGui::InputText("Record filename/path", input_movie_record_path_,
                    sizeof(input_movie_record_path_));
                if (ImGui::Button("Start Recording")) {
                    start_input_recording_from_ui();
                    movie_status = input_recorder_.status();
                }
                ImGui::SameLine();
                ImGui::BeginDisabled(
                    movie_status.mode != InputRecorder::Mode::Recording);
                if (ImGui::Button("Stop Recording")) {
                    input_recorder_.shutdown();
                    status_message_ = "Input recording stopped.";
                    movie_status = input_recorder_.status();
                }
                ImGui::EndDisabled();

                ImGui::InputText("Playback filename/path", input_movie_playback_path_,
                    sizeof(input_movie_playback_path_));
                if (ImGui::Button("Start Playback")) {
                    start_input_playback_from_ui();
                    movie_status = input_recorder_.status();
                }
                ImGui::SameLine();
                ImGui::BeginDisabled(
                    movie_status.mode != InputRecorder::Mode::Playing);
                if (ImGui::Button("Stop Playback")) {
                    input_recorder_.shutdown();
                    status_message_ = "Input playback stopped.";
                    movie_status = input_recorder_.status();
                }
                ImGui::EndDisabled();

                if (ImGui::Checkbox("Stop playback at EOF",
                    &input_movie_stop_at_eof_)) {
                    if (input_movie_stop_at_eof_) {
                        input_movie_loop_ = false;
                    }
                    input_recorder_.set_end_behavior(input_movie_end_behavior());
                    movie_status = input_recorder_.status();
                }
                if (ImGui::Checkbox("Loop playback", &input_movie_loop_)) {
                    if (input_movie_loop_) {
                        input_movie_stop_at_eof_ = false;
                    }
                    input_recorder_.set_end_behavior(input_movie_end_behavior());
                    movie_status = input_recorder_.status();
                }

                ImGui::Text("Current emulated frame: %llu",
                    static_cast<unsigned long long>(
                        movie_status.current_emulated_frame));
                ImGui::Text("Frames recorded: %llu",
                    static_cast<unsigned long long>(
                        movie_status.recorded_frame_count));
                ImGui::Text("Frames replayed: %llu",
                    static_cast<unsigned long long>(
                        movie_status.playback_frame_count));
                ImGui::Text("Playback index: %llu / %llu",
                    static_cast<unsigned long long>(
                        movie_status.playback_current_index),
                    static_cast<unsigned long long>(
                        movie_status.playback_total_frames));
                ImGui::Text("EOF behavior: %s",
                    InputRecorder::end_behavior_name(movie_status.end_behavior));
                ImGui::TextWrapped("Resolved record path: %s",
                    movie_status.record_path.empty()
                    ? "(none)" : movie_status.record_path.c_str());
                ImGui::TextWrapped("Resolved playback path: %s",
                    movie_status.playback_path.empty()
                    ? "(none)" : movie_status.playback_path.c_str());
                if (!movie_status.last_error.empty()) {
                    ImGui::TextColored(ImVec4(0.95f, 0.4f, 0.4f, 1.0f),
                        "Last error: %s", movie_status.last_error.c_str());
                }
                ImGui::TextWrapped("Status: %s",
                    movie_status.status_message.c_str());

                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Video")) {
                const int frame_w = std::max(1, renderer_->last_frame_width());
                const int frame_h = std::max(1, renderer_->last_frame_height());
                ImGui::Text("Resolution: %dx%d", frame_w, frame_h);
                ImGui::Text("Display Area: %ux%u",
                    static_cast<unsigned>(runtime_snapshot_.boot_diag.display_width),
                    static_cast<unsigned>(runtime_snapshot_.boot_diag.display_height));
                if (ImGui::Checkbox("Fast Mode", &g_gpu_fast_mode)) {
                    if (!g_gpu_fast_mode) {
                        g_gpu_extreme_fast_mode = false;
                    }
                    save_persistent_config();
                }
                ImGui::TextColored(
                    ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                    "Uses optimized GPU paths for lower CPU usage at the cost of possible artifacting.");
                ImGui::BeginDisabled(!g_gpu_fast_mode);
                if (ImGui::Checkbox("Extreme Fast Mode", &g_gpu_extreme_fast_mode)) {
                    save_persistent_config();
                }
                ImGui::EndDisabled();
                ImGui::TextColored(
                    ImVec4(0.7f, 0.7f, 0.5f, 1.0f),
                    "More unstable than Fast Mode and may heavily reduce shading, transparency, and presentation quality.");
                const char* deinterlace_modes[] = { "Weave (Stable)", "Bob (Field)",
                                                   "Blend (Soft)" };
                int deinterlace_index = static_cast<int>(g_deinterlace_mode);
                if (ImGui::Combo("Deinterlace", &deinterlace_index, deinterlace_modes,
                    IM_ARRAYSIZE(deinterlace_modes))) {
                    deinterlace_index = std::max(0, std::min(2, deinterlace_index));
                    g_deinterlace_mode =
                        static_cast<DeinterlaceMode>(deinterlace_index);
                }

                const char* resolution_modes[] = { "320x240", "640x480", "1024x768" };
                int resolution_index = static_cast<int>(g_output_resolution_mode);
                if (ImGui::Combo("Output Resolution", &resolution_index,
                    resolution_modes, IM_ARRAYSIZE(resolution_modes))) {
                    resolution_index = std::max(0, std::min(2, resolution_index));
                    g_output_resolution_mode =
                        static_cast<OutputResolutionMode>(resolution_index);
                    if (!latest_frame_rgba_.empty() && latest_frame_width_ > 0 &&
                        latest_frame_height_ > 0) {
                        int output_width = 320;
                        int output_height = 240;
                        output_resolution_dimensions(g_output_resolution_mode,
                            output_width, output_height);
                        if (latest_frame_width_ != output_width ||
                            latest_frame_height_ != output_height) {
                            resample_rgba_nearest(latest_frame_rgba_, latest_frame_width_,
                                latest_frame_height_, scaled_frame_rgba_,
                                output_width, output_height);
                            renderer_->upload_frame(scaled_frame_rgba_, output_width,
                                output_height);
                            latest_frame_rgba_ = scaled_frame_rgba_;
                            latest_frame_width_ = output_width;
                            latest_frame_height_ = output_height;
                        }
                    }
                }
                if (ImGui::Checkbox("Bilinear Presentation Filter",
                        &g_bilinear_filtering)) {
                    if (renderer_) {
                        renderer_->set_bilinear_filtering(g_bilinear_filtering);
                    }
                    save_persistent_config();
                }
                ImGui::TextColored(
                    ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                    "Smooths the final presentation texture when scaling; disable for crisp nearest-neighbor output.");
                ImGui::Text("Internal Upscaling: 1x (native)");
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Audio")) {
                draw_audio_panel();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("System")) {
                draw_system_panel();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Memory Cards")) {
                draw_memory_card_panel();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Experimental")) {
                draw_experimental_settings_panel();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Customize")) {
                ImGui::Text("Theme Colors");
                ImGui::TextDisabled("Overall changes recolor the full UI. Extra grouped controls and per-element overrides are stored in imgui.ini.");
                ImGui::Separator();

                const int preset_count = ui_theme::theme_preset_count();
                const char* theme_preset_labels[64] = {};
                for (int i = 0; i < preset_count; ++i) {
                    theme_preset_labels[i] = ui_theme::theme_preset_by_index(i).label;
                }
                ui_theme::g_selected_theme_preset_index =
                    std::max(0, std::min(preset_count - 1, ui_theme::g_selected_theme_preset_index));
                ImGui::Combo("Preset", &ui_theme::g_selected_theme_preset_index,
                    theme_preset_labels, preset_count);
                ImGui::SameLine();
                if (ImGui::Button("Apply Preset")) {
                    ui_theme::apply_theme_preset_by_index(ui_theme::g_selected_theme_preset_index);
                    ui_theme::apply_theme_style(ImGui::GetStyle());
                    ui_theme::mark_theme_settings_dirty();
                }
                ImGui::TextDisabled(
                    "Dark Mode and Light Mode use the preset screenshot values plus a dedicated list background color.");

                bool theme_changed = false;
                bool simple_theme_changed = false;

                ImVec4 overall = ui_theme::g_theme_settings.overall;
                if (ImGui::ColorEdit4("Overall", &overall.x,
                    ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_AlphaBar)) {
                    ui_theme::g_theme_settings.overall = overall;
                    ui_theme::rebuild_theme_basics_from_overall(ui_theme::g_theme_settings);
                    theme_changed = true;
                }

                bool simple = ui_theme::g_theme_settings.simple;
                if (ImGui::Checkbox("Simple Customization", &simple)) {
                    ui_theme::g_theme_settings.simple = simple;
                    theme_changed = true;
                }

                if (ui_theme::g_theme_settings.simple) {
                    ImVec4 background = ui_theme::g_theme_settings.background;
                    if (ImGui::ColorEdit4("Background", &background.x,
                        ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_AlphaBar)) {
                        ui_theme::g_theme_settings.background = background;
                        theme_changed = true;
                        simple_theme_changed = true;
                    }

                    ImVec4 surface = ui_theme::g_theme_settings.surface;
                    if (ImGui::ColorEdit4("Surface", &surface.x,
                        ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_AlphaBar)) {
                        ui_theme::g_theme_settings.surface = surface;
                        theme_changed = true;
                        simple_theme_changed = true;
                    }

                    ImVec4 accent = ui_theme::g_theme_settings.accent;
                    if (ImGui::ColorEdit4("Accent", &accent.x,
                        ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_AlphaBar)) {
                        ui_theme::g_theme_settings.accent = accent;
                        theme_changed = true;
                        simple_theme_changed = true;
                    }

                    ImVec4 text = ui_theme::g_theme_settings.text;
                    if (ImGui::ColorEdit4("Text", &text.x,
                        ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_AlphaBar)) {
                        ui_theme::g_theme_settings.text = text;
                        theme_changed = true;
                        simple_theme_changed = true;
                    }

                    ImVec4 lists = ui_theme::g_theme_settings.lists;
                    if (ImGui::ColorEdit4("Lists", &lists.x,
                        ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_AlphaBar)) {
                        ui_theme::g_theme_settings.lists = lists;
                        theme_changed = true;
                        simple_theme_changed = true;
                    }
                }

                bool advanced = ui_theme::g_theme_settings.advanced;
                if (ImGui::Checkbox("Advanced", &advanced)) {
                    if (advanced && !ui_theme::g_theme_settings.advanced) {
                        ui_theme::reset_theme_startup_colors(ui_theme::g_theme_settings);
                    }
                    ui_theme::g_theme_settings.advanced = advanced;
                    theme_changed = true;
                }

                if (theme_changed) {
                    if (simple_theme_changed) {
                        ui_theme::sync_theme_overall_from_basics(ui_theme::g_theme_settings);
                    }
                    ui_theme::rebuild_theme_colors_from_basics(ui_theme::g_theme_settings);
                    ui_theme::apply_theme_style(ImGui::GetStyle());
                    ui_theme::mark_theme_settings_dirty();
                }

                if (ui_theme::g_theme_settings.advanced) {
                    ImGui::Separator();
                    ImGui::TextDisabled("Advanced per-element overrides.");
                    bool advanced_theme_changed = false;
                    ImVec4 startup_title = ui_theme::g_theme_settings.startup_title;
                    if (ImGui::ColorEdit4("Startup Title", &startup_title.x,
                        ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_AlphaBar)) {
                        ui_theme::g_theme_settings.startup_title = startup_title;
                        advanced_theme_changed = true;
                    }
                    ImVec4 startup_text = ui_theme::g_theme_settings.startup_text;
                    if (ImGui::ColorEdit4("Startup Text", &startup_text.x,
                        ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_AlphaBar)) {
                        ui_theme::g_theme_settings.startup_text = startup_text;
                        advanced_theme_changed = true;
                    }
                    for (size_t i = 0; i < ui_theme::kThemeColorSlotCount; ++i) {
                        ImVec4 color = ui_theme::g_theme_settings.colors[i];
                        if (ImGui::ColorEdit4(ui_theme::kThemeColorSlots[i].label, &color.x,
                            ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_AlphaBar)) {
                            ui_theme::g_theme_settings.colors[i] = color;
                            advanced_theme_changed = true;
                        }
                    }
                    if (advanced_theme_changed) {
                        ui_theme::apply_theme_style(ImGui::GetStyle());
                        ui_theme::mark_theme_settings_dirty();
                    }
                }

                if (ImGui::Button("Reset Theme Colors")) {
                    ui_theme::reset_theme_settings();
                    ui_theme::apply_theme_style(ImGui::GetStyle());
                    ui_theme::mark_theme_settings_dirty();
                }
                ImGui::SameLine();
                if (ImGui::Button("Save Theme Now")) {
                    ImGuiIO& io = ImGui::GetIO();
                    if (io.IniFilename != nullptr && io.IniFilename[0] != '\0') {
                        ImGui::SaveIniSettingsToDisk(io.IniFilename);
                    }
                }

                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Logging")) {
                draw_logging_panel();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
    }
    ImGui::End();
}

