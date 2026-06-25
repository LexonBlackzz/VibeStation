#include "ui/app.h"
#include <imgui.h>
#include <algorithm>
void App::draw_system_panel() {
                ImGui::Text("BIOS: %s", bios_path_.empty()
                    ? "Not loaded"
                    : system_->bios().get_info().c_str());
                ImGui::Text("CPU Clock: 33.8688 MHz");
                const char* cpu_backend_labels[] = {
                    "Interpreter",
                    "Decoded Block",
                    "x64 JIT"
                };
                int cpu_backend_index =
                    cpu_execution_mode_to_config_value(g_cpu_execution_mode);
                if (ImGui::Combo("CPU Backend", &cpu_backend_index,
                    cpu_backend_labels, IM_ARRAYSIZE(cpu_backend_labels))) {
                    const bool was_running = emu_runner_.is_running();
                    if (was_running) {
                        emu_runner_.pause_and_wait_idle();
                    }
                    g_cpu_execution_mode =
                        cpu_execution_mode_from_config_value(cpu_backend_index);
                    if (system_) {
                        system_->cpu().flush_cpu_backend();
                    }
                    save_persistent_config();
                    if (was_running) {
                        emu_runner_.set_running(true);
                    }
                }
                if (g_cpu_execution_mode_cli_override) {
                    ImGui::TextDisabled("CLI override active: %s",
                        cpu_execution_mode_name(g_cpu_execution_mode_cli_value));
                }
                const CpuBackendStats backend_stats =
                    system_ ? system_->cpu().cpu_backend_stats()
                            : runtime_snapshot_.cpu_backend_stats;
                if (g_cpu_execution_mode == CpuExecutionMode::X64Jit &&
                    backend_stats.native_blocks == 0) {
                    ImGui::TextDisabled(
                        "x64 JIT currently native-compiles hot ALU/immediate blocks; other blocks use decoded fallback.");
                }
                ImGui::Separator();
                ImGui::Text("Performance");
                ImGui::Text("Emulation pacing: fixed 60 Hz");
                const char* turbo_modes[] = { "200%", "400%", "Unlimited" };
                int turbo_mode_index = 0;
                if (config_turbo_speed_percent_ <= 0) {
                    turbo_mode_index = 2;
                }
                else if (config_turbo_speed_percent_ >= 400) {
                    turbo_mode_index = 1;
                }
                if (ImGui::Combo("Turbo Speed (Hold Backspace)", &turbo_mode_index,
                    turbo_modes, IM_ARRAYSIZE(turbo_modes))) {
                    config_turbo_speed_percent_ =
                        (turbo_mode_index == 2) ? 0 : ((turbo_mode_index == 1) ? 400 : 200);
                    apply_speed_override();
                    save_persistent_config();
                }
                if (config_turbo_speed_percent_ <= 0) {
                    ImGui::TextDisabled(
                        "Unlimited turbo removes frame pacing, skips turbo audio, and drops most display frames while held.");
                }
                int slowdown_speed_percent = config_slowdown_speed_percent_;
                if (ImGui::SliderInt("Slowdown Speed (Hold Right Shift)",
                    &slowdown_speed_percent, 10, 100, "%d%%")) {
                    config_slowdown_speed_percent_ =
                        std::clamp(slowdown_speed_percent, 10, 100);
                    apply_speed_override();
                    save_persistent_config();
                }
                if (ImGui::Checkbox("VSync Playback", &config_vsync_)) {
                    SDL_GL_SetSwapInterval(config_vsync_ ? 1 : 0);
                }
                if (ImGui::Checkbox("Direct Disc Boot (Skip BIOS Intro)",
                    &config_direct_disc_boot_)) {
                    save_persistent_config();
                }
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                    "Applies to Emulation > Boot Disc only.");
                ImGui::Checkbox("Detailed Profiling", &g_profile_detailed_timing);
                if (ImGui::Checkbox("Low-spec Mode", &config_low_spec_mode_)) {
                    g_low_spec_mode = config_low_spec_mode_;
                }
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                    "Reduces audio complexity and internal precision.");
                ImGui::Separator();
                ImGui::Text("Discord Rich Presence");
                if (ImGui::Checkbox("Enable Discord RPC",
                    &config_discord_rich_presence_)) {
                    sync_discord_presence_config();
                    save_persistent_config();
                }
                const char* discord_build_status =
                    (discord_presence_ && discord_presence_->sdk_compiled())
                    ? "Discord Social SDK linked"
                    : "Discord Social SDK not linked in this build";
                ImGui::TextDisabled("%s", discord_build_status);
                ImGui::TextWrapped("Status: %s",
                    discord_presence_ ? discord_presence_->status_text().c_str()
                    : "Unavailable");
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                    "Uses the built-in Discord application configuration and the local Discord desktop client.");
            }