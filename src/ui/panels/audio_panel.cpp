#include "ui/app.h"
#include <imgui.h>


void App::draw_audio_panel() {
                ImGui::Text("SPU emulation: Gaussian + reverb core (stage 2)");
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.3f, 1.0f),
                    "XA/CDDA baseline is live; advanced modulation is still in progress.");

                const auto apply_audio_settings = [&]() {
                    save_persistent_config();
                    if (system_ == nullptr) {
                        return;
                    }
                    const bool was_running = emu_runner_.is_running();
                    if (was_running) {
                        emu_runner_.pause_and_wait_idle();
                    }
                    system_->spu().reinitialize_audio_device();
                    if (was_running) {
                        emu_runner_.set_running(true);
                    }
                };

                int target_latency_ms =
                    static_cast<int>(g_spu_audio_target_latency_ms);
                if (ImGui::InputInt("Target Latency (ms)",
                    &target_latency_ms, 5, 20)) {
                    g_spu_audio_target_latency_ms = static_cast<u32>(
                        std::clamp(target_latency_ms, 10, 500));
                    g_spu_audio_soft_latency_ms = std::max(
                        g_spu_audio_soft_latency_ms,
                        g_spu_audio_target_latency_ms);
                    g_spu_audio_max_latency_ms = std::max(
                        g_spu_audio_max_latency_ms,
                        g_spu_audio_soft_latency_ms);
                    apply_audio_settings();
                }
                int soft_latency_ms =
                    static_cast<int>(g_spu_audio_soft_latency_ms);
                if (ImGui::InputInt("Soft Correction Starts (ms)",
                    &soft_latency_ms, 5, 20)) {
                    g_spu_audio_soft_latency_ms = static_cast<u32>(std::clamp(
                        soft_latency_ms,
                        static_cast<int>(g_spu_audio_target_latency_ms), 750));
                    g_spu_audio_max_latency_ms = std::max(
                        g_spu_audio_max_latency_ms,
                        g_spu_audio_soft_latency_ms);
                    apply_audio_settings();
                }
                int max_latency_ms = static_cast<int>(g_spu_audio_max_latency_ms);
                if (ImGui::InputInt("Maximum Latency (ms)",
                    &max_latency_ms, 5, 20)) {
                    g_spu_audio_max_latency_ms = static_cast<u32>(std::clamp(
                        max_latency_ms,
                        static_cast<int>(g_spu_audio_soft_latency_ms), 1000));
                    apply_audio_settings();
                }
                float xa_buffer_seconds = g_spu_xa_buffer_seconds;
                if (ImGui::InputFloat("XA Buffer (seconds)",
                    &xa_buffer_seconds, 0.01f, 0.1f, "%.3f")) {
                    xa_buffer_seconds = std::max(0.0f, std::min(5.0f, xa_buffer_seconds));
                    g_spu_xa_buffer_seconds = xa_buffer_seconds;
                    save_persistent_config();
                }
                if (ImGui::Checkbox("Enable Audio Queue", &g_spu_enable_audio_queue)) {
                    apply_audio_settings();
                }
                if (ImGui::Checkbox("Enable Crossfaded Smooth Trim",
                    &g_spu_enable_smooth_trim)) {
                    apply_audio_settings();
                }
                ImGui::TextDisabled(
                    "Bounded to four crossfaded correction steps per second.");
                if (ImGui::Checkbox("Lag Stutter Effect", &g_spu_enable_lag_stutter)) {
                    apply_audio_settings();
                }
                if (ImGui::Checkbox("Slowdown Stutter Loop", &g_spu_enable_slowdown_stutter)) {
                    apply_audio_settings();
                }
                if (ImGui::Checkbox("Advanced Sound Status Logging", &g_spu_advanced_sound_status)) {
                    save_persistent_config();
                }
                if (ImGui::Checkbox("Show Audio Queue Stats", &g_spu_show_audio_stats)) {
                    save_persistent_config();
                }
                if (ImGui::Checkbox("Log Audio Queue Stats", &g_spu_audio_stats_log)) {
                    save_persistent_config();
                }

                // ── Ring-buffer audio configuration ──────────────────────────
                ImGui::Separator();
                ImGui::TextColored(ImVec4(0.6f, 0.85f, 0.6f, 1.0f),
                    "Bounded Host Audio Queue");

                if (g_spu_show_audio_stats) {
                    const auto &stats = runtime_snapshot_.audio_queue;
                    const auto frames_to_ms = [&](u64 frame_count) -> double {
                        const u32 rate = stats.obtained_callback_sample_rate != 0u
                            ? stats.obtained_callback_sample_rate
                            : stats.emulated_sample_rate;
                        return rate == 0u ? 0.0
                            : (static_cast<double>(frame_count) * 1000.0) /
                                static_cast<double>(rate);
                    };
                    const float fill = stats.max_stereo_frames == 0u ? 0.0f
                        : static_cast<float>(stats.queue_stereo_frames) /
                            static_cast<float>(stats.max_stereo_frames);
                    char fill_label[128];
                    std::snprintf(fill_label, sizeof(fill_label),
                        "Queue: %u stereo frames / %.1f ms",
                        stats.queue_stereo_frames,
                        frames_to_ms(stats.queue_stereo_frames));
                    ImGui::ProgressBar(std::clamp(fill, 0.0f, 1.0f),
                        ImVec2(-1.0f, 0.0f), fill_label);
                    ImGui::Text("Target / Soft / Hard: %.1f / %.1f / %.1f ms",
                        frames_to_ms(stats.target_stereo_frames),
                        frames_to_ms(stats.soft_latency_stereo_frames),
                        frames_to_ms(stats.max_stereo_frames));
                    ImGui::Text("Max Observed: %u stereo frames / %.1f ms",
                        stats.max_observed_queue_stereo_frames,
                        frames_to_ms(stats.max_observed_queue_stereo_frames));
                    ImGui::Text("Last frame - produced / pushed / consumed: %llu / %llu / %llu stereo frames",
                        static_cast<unsigned long long>(stats.produced_stereo_frames_window),
                        static_cast<unsigned long long>(stats.pushed_stereo_frames_window),
                        static_cast<unsigned long long>(stats.callback_consumed_stereo_frames_window));
                    ImGui::Text("Totals - produced / pushed / consumed: %llu / %llu / %llu stereo frames",
                        static_cast<unsigned long long>(stats.produced_stereo_frames),
                        static_cast<unsigned long long>(stats.pushed_stereo_frames),
                        static_cast<unsigned long long>(stats.callback_consumed_stereo_frames));
                    ImGui::Text("Underruns: %llu  |  Overruns: %llu  |  Dropped: %llu stereo frames",
                        static_cast<unsigned long long>(stats.underrun_count),
                        static_cast<unsigned long long>(stats.overrun_count),
                        static_cast<unsigned long long>(stats.dropped_stereo_frames));
                    ImGui::Text("Smooth Trim [%s]: %llu events / %llu stereo frames (last %u, %.1f -> %.1f ms)",
                        stats.smooth_trim_enabled ? "Enabled" : "Disabled",
                        static_cast<unsigned long long>(stats.smooth_trim_count),
                        static_cast<unsigned long long>(stats.smooth_trim_stereo_frames),
                        stats.smooth_trim_last_stereo_frames,
                        frames_to_ms(stats.smooth_trim_queue_before_stereo_frames),
                        frames_to_ms(stats.smooth_trim_queue_after_stereo_frames));
                    ImGui::Text("Hard Trim: %llu events / %llu stereo frames (last %u)",
                        static_cast<unsigned long long>(stats.hard_trim_count),
                        static_cast<unsigned long long>(stats.hard_trim_stereo_frames),
                        stats.hard_trim_last_stereo_frames);
                    ImGui::Text("Stutter: %s  |  Enter / Exit: %llu / %llu (%.1f / %.1f ms)",
                        stats.stutter_active ? "Active" : "Inactive",
                        static_cast<unsigned long long>(stats.stutter_enter_count),
                        static_cast<unsigned long long>(stats.stutter_exit_count),
                        frames_to_ms(stats.stutter_enter_stereo_frames),
                        frames_to_ms(stats.stutter_exit_stereo_frames));
                    ImGui::Text("Producer / Host Drift (last frame window): %+.3f%% (%+lld stereo frames)",
                        stats.producer_consumer_drift_percent,
                        static_cast<long long>(stats.producer_consumer_drift_stereo_frames_window));
                    ImGui::Text("Silence / Stutter: %llu / %llu stereo frames  |  Callback lock misses: %llu",
                        static_cast<unsigned long long>(stats.callback_silence_stereo_frames),
                        static_cast<unsigned long long>(stats.callback_stutter_stereo_frames),
                        static_cast<unsigned long long>(stats.callback_lock_misses));
                    ImGui::Text("Host Output Peak (pre-clamp): %u  |  Clipped: %llu samples",
                        stats.output_peak_sample_before_clamp,
                        static_cast<unsigned long long>(stats.output_clipped_samples));
                    ImGui::Text("Read Pointer Discontinuities: %llu",
                        static_cast<unsigned long long>(
                            stats.read_pointer_discontinuity_count));
                    ImGui::Text("Requested / Obtained / Preferred Rate: %u / %u / %u Hz",
                        stats.requested_callback_sample_rate,
                        stats.obtained_callback_sample_rate,
                        stats.preferred_device_sample_rate);
                    ImGui::Text("Requested / Obtained Callback: %u / %u stereo frames  |  SDL conversion: %s",
                        stats.requested_callback_buffer_frames,
                        stats.obtained_callback_buffer_frames,
                        stats.sdl_internal_conversion_expected ? "Expected" : "Not expected");
                    ImGui::Text("Queue Resets: %llu  |  Device Reinits: %llu",
                        static_cast<unsigned long long>(stats.queue_reset_count),
                        static_cast<unsigned long long>(stats.device_reinit_count));
                }
                else {
                    ImGui::TextDisabled("Audio queue statistics are hidden.");
                }

                ImGui::Separator();
                ImGui::Text("Sound Status");
                draw_sound_status_content();
            }