#include "ui/app.h"

#include <imgui.h>

#include <algorithm>
#include <cstddef>

void App::draw_spu_diagnostic_mode_controls() {
    if (!config_spu_diagnostic_mode_) {
        if (ImGui::Button("Slowed + Reverb Mode")) {
            config_spu_diagnostic_mode_ = true;
            apply_speed_override();
            save_persistent_config();
        }
    }
    else {
        if (ImGui::Button("Disable Slowed + Reverb Mode")) {
            config_spu_diagnostic_mode_ = false;
            apply_speed_override();
            save_persistent_config();
        }
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(config_spu_diagnostic_mode_ ? "Active" : "Inactive");
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
        "Enables per-sample SPU diagnostics. Voice level meters stay live.");
}

void App::draw_sound_status_content() {
    const auto& diag = runtime_snapshot_.spu_audio;
    const auto& queue_stats = runtime_snapshot_.audio_queue;
    const auto avg_count = [](u64 accum, u64 samples) -> float {
        const u64 denom = std::max<u64>(samples, 1);
        return static_cast<float>(accum) / static_cast<float>(denom);
        };
    const float avg_logical =
        avg_count(diag.logical_voice_accum, diag.logical_voice_samples);
    const float avg_env = avg_count(diag.env_voice_accum, diag.env_voice_samples);
    const float avg_audible =
        avg_count(diag.audible_voice_accum, diag.audible_voice_samples);
    const float queue_kb_raw =
        static_cast<float>(queue_stats.queue_stereo_frames * 2u * sizeof(s16)) /
        1024.0f;
    const float queue_kb =
        (smoothed_audio_queue_kb_ >= 0.0f) ? smoothed_audio_queue_kb_ : queue_kb_raw;
    const float queue_peak_kb =
        static_cast<float>(queue_stats.max_observed_queue_stereo_frames * 2u *
            sizeof(s16)) /
        1024.0f;

    ImGui::Text("Realtime SPU voice monitor (updates every emulated frame).");
    if (ImGui::Button("Reset Sound Stats")) {
        const bool was_running = emu_runner_.is_running();
        if (was_running) {
            emu_runner_.pause_and_wait_idle();
        }
        system_->reset_spu_audio_diag();
        runtime_snapshot_.spu_audio = system_->spu_audio_diag();
        runtime_snapshot_.audio_queue = system_->spu().audio_queue_stats(false);
        status_message_ = "SPU sound stats reset";
        if (was_running) {
            emu_runner_.set_running(true);
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Use after BIOS boot to isolate gameplay peaks.");
    ImGui::Separator();
    ImGui::Text("Running: %s", runtime_snapshot_.running ? "Yes" : "No");
    ImGui::Text("Reverb: %s", diag.reverb_enabled ? "Enabled" : "Disabled");
    ImGui::Text("Generated/Queued Frames: %llu / %llu",
        static_cast<unsigned long long>(diag.generated_frames),
        static_cast<unsigned long long>(diag.queued_frames));
    ImGui::Text("Dropped Stereo Frames: %llu (Overruns: %llu, Underruns: %llu)",
        static_cast<unsigned long long>(queue_stats.dropped_stereo_frames),
        static_cast<unsigned long long>(queue_stats.overrun_count),
        static_cast<unsigned long long>(queue_stats.underrun_count));
    if (g_spu_show_audio_stats) {
        const double queue_latency_ms =
            queue_stats.obtained_callback_sample_rate == 0u
            ? 0.0
            : (static_cast<double>(queue_stats.queue_stereo_frames) * 1000.0) /
                static_cast<double>(queue_stats.obtained_callback_sample_rate);
        ImGui::Text("Audio Queue: %.1f KB (peak %.1f KB), %.1f ms",
            queue_kb, queue_peak_kb, queue_latency_ms);
        ImGui::Text("Produced / Pushed / Callback Consumed: %llu / %llu / %llu stereo frames",
            static_cast<unsigned long long>(queue_stats.produced_stereo_frames),
            static_cast<unsigned long long>(queue_stats.pushed_stereo_frames),
            static_cast<unsigned long long>(queue_stats.callback_consumed_stereo_frames));
    }
    if (!g_spu_advanced_sound_status) {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.85f, 0.75f, 0.35f, 1.0f),
            "Advanced sound status logging is disabled.");
        ImGui::TextDisabled("Enable it in Settings > Audio for voice peaks, ENDX, and detailed SPU counters.");
        return;
    }
    ImGui::Text("Voices Logical (phase!=off): avg %.2f / peak %u",
        avg_logical, static_cast<unsigned>(diag.logical_voice_peak));
    ImGui::Text("Voices Env>0: avg %.2f / peak %u",
        avg_env, static_cast<unsigned>(diag.env_voice_peak));
    ImGui::Text("Voices Audible (nonzero out): avg %.2f / peak %u",
        avg_audible, static_cast<unsigned>(diag.audible_voice_peak));
    ImGui::Text("Logical peak ctx: sample %llu | KON/KOFF %llu/%llu | ENDX 0x%06X",
        static_cast<unsigned long long>(diag.logical_voice_peak_sample),
        static_cast<unsigned long long>(diag.logical_voice_peak_key_on_events),
        static_cast<unsigned long long>(diag.logical_voice_peak_key_off_events),
        static_cast<unsigned>(diag.logical_voice_peak_endx_mask & 0x00FFFFFFu));
    ImGui::Text("Env>0 peak ctx: sample %llu | KON/KOFF %llu/%llu | ENDX 0x%06X",
        static_cast<unsigned long long>(diag.env_voice_peak_sample),
        static_cast<unsigned long long>(diag.env_voice_peak_key_on_events),
        static_cast<unsigned long long>(diag.env_voice_peak_key_off_events),
        static_cast<unsigned>(diag.env_voice_peak_endx_mask & 0x00FFFFFFu));
    ImGui::Text("Audible peak ctx: sample %llu | KON/KOFF %llu/%llu | ENDX 0x%06X",
        static_cast<unsigned long long>(diag.audible_voice_peak_sample),
        static_cast<unsigned long long>(diag.audible_voice_peak_key_on_events),
        static_cast<unsigned long long>(diag.audible_voice_peak_key_off_events),
        static_cast<unsigned>(diag.audible_voice_peak_endx_mask & 0x00FFFFFFu));
    ImGui::Text("KEY ON/OFF Events: %llu / %llu",
        static_cast<unsigned long long>(diag.key_on_events),
        static_cast<unsigned long long>(diag.key_off_events));
    ImGui::Text("Voice offs (END/RELEASE): %llu / %llu",
        static_cast<unsigned long long>(diag.off_due_to_end_flag),
        static_cast<unsigned long long>(diag.release_to_off_events));
    ImGui::Text("Ignored while SPU disabled (KON/KOFF): %llu / %llu",
        static_cast<unsigned long long>(diag.keyon_ignored_while_disabled),
        static_cast<unsigned long long>(diag.keyoff_ignored_while_disabled));
    ImGui::Text("SPUCNT disable events / forced-off voices: %llu / %llu",
        static_cast<unsigned long long>(diag.spucnt_enable_clear_events),
        static_cast<unsigned long long>(diag.spu_disable_forced_off_voices));
    ImGui::Text("KON writes L/H: %llu / %llu | bits collected: %llu | multiwrite windows: %llu",
        static_cast<unsigned long long>(diag.kon_write_events_low),
        static_cast<unsigned long long>(diag.kon_write_events_high),
        static_cast<unsigned long long>(diag.kon_bits_collected),
        static_cast<unsigned long long>(diag.kon_multiwrite_same_sample_events));
    ImGui::Text("KOFF writes L/H: %llu / %llu | bits collected: %llu | multiwrite windows: %llu",
        static_cast<unsigned long long>(diag.koff_write_events_low),
        static_cast<unsigned long long>(diag.koff_write_events_high),
        static_cast<unsigned long long>(diag.koff_bits_collected),
        static_cast<unsigned long long>(diag.koff_multiwrite_same_sample_events));
    ImGui::Text("ENDX Mask: 0x%06X",
        static_cast<unsigned>(runtime_snapshot_.spu_endx_mask & 0x00FFFFFFu));

    ImGui::Spacing();
    ImGui::Text("Voice Levels");
    ImGuiTableFlags table_flags =
        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
        ImGuiTableFlags_SizingStretchSame;
    const float voice_row_height =
        (ImGui::GetTextLineHeight() * 2.0f) + ImGui::GetFrameHeight() +
        (ImGui::GetStyle().ItemSpacing.y * 3.0f);
    if (ImGui::BeginTable("SPUSoundStatusVoices", 4, table_flags)) {
        ImGui::TableSetupColumn("##voice_col_0", ImGuiTableColumnFlags_WidthStretch,
            1.0f);
        ImGui::TableSetupColumn("##voice_col_1", ImGuiTableColumnFlags_WidthStretch,
            1.0f);
        ImGui::TableSetupColumn("##voice_col_2", ImGuiTableColumnFlags_WidthStretch,
            1.0f);
        ImGui::TableSetupColumn("##voice_col_3", ImGuiTableColumnFlags_WidthStretch,
            1.0f);
        for (size_t voice = 0; voice < runtime_snapshot_.spu_voice_level_l.size();
            ++voice) {
            if ((voice % 4u) == 0u) {
                ImGui::TableNextRow(ImGuiTableRowFlags_None, voice_row_height);
            }
            ImGui::TableSetColumnIndex(static_cast<int>(voice % 4u));

            const s16 level_l = runtime_snapshot_.spu_voice_level_l[voice];
            const s16 level_r = runtime_snapshot_.spu_voice_level_r[voice];
            const int abs_l = (level_l < 0) ? -static_cast<int>(level_l)
                : static_cast<int>(level_l);
            const int abs_r = (level_r < 0) ? -static_cast<int>(level_r)
                : static_cast<int>(level_r);
            const int peak = std::max(abs_l, abs_r);
            const float meter = std::min(1.0f, static_cast<float>(peak) / 32767.0f);
            const bool active = runtime_snapshot_.spu_voice_active[voice];

            ImGui::TextColored(active ? ImVec4(0.4f, 0.9f, 0.4f, 1.0f)
                : ImVec4(0.55f, 0.55f, 0.55f, 1.0f),
                "V%02u", static_cast<unsigned>(voice));
            ImGui::SameLine();
            ImGui::TextUnformatted(active ? "ON" : "OFF");
            ImGui::ProgressBar(meter, ImVec2(-1.0f, 0.0f));
            ImGui::Text("L:%6d  R:%6d", static_cast<int>(level_l),
                static_cast<int>(level_r));
        }
        ImGui::EndTable();
    }
}

void App::panel_sound_status() {
    ImGui::SetNextWindowSize(ImVec2(720, 500), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Voice Levels", &show_sound_status_)) {
        ImGui::Text("Voice Levels");
        ImGuiTableFlags table_flags =
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_SizingStretchSame;
        const float voice_row_height =
            (ImGui::GetTextLineHeight() * 2.0f) + ImGui::GetFrameHeight() +
            (ImGui::GetStyle().ItemSpacing.y * 3.0f);
        if (ImGui::BeginTable("SPUVoiceLevelsOnly", 4, table_flags)) {
            ImGui::TableSetupColumn("##voice_col_0", ImGuiTableColumnFlags_WidthStretch,
                1.0f);
            ImGui::TableSetupColumn("##voice_col_1", ImGuiTableColumnFlags_WidthStretch,
                1.0f);
            ImGui::TableSetupColumn("##voice_col_2", ImGuiTableColumnFlags_WidthStretch,
                1.0f);
            ImGui::TableSetupColumn("##voice_col_3", ImGuiTableColumnFlags_WidthStretch,
                1.0f);
            for (size_t voice = 0; voice < runtime_snapshot_.spu_voice_level_l.size();
                ++voice) {
                if ((voice % 4u) == 0u) {
                    ImGui::TableNextRow(ImGuiTableRowFlags_None, voice_row_height);
                }
                ImGui::TableSetColumnIndex(static_cast<int>(voice % 4u));

                const s16 level_l = runtime_snapshot_.spu_voice_level_l[voice];
                const s16 level_r = runtime_snapshot_.spu_voice_level_r[voice];
                const int abs_l = (level_l < 0) ? -static_cast<int>(level_l)
                    : static_cast<int>(level_l);
                const int abs_r = (level_r < 0) ? -static_cast<int>(level_r)
                    : static_cast<int>(level_r);
                const int peak = std::max(abs_l, abs_r);
                const float meter = std::min(1.0f, static_cast<float>(peak) / 32767.0f);
                const bool active = runtime_snapshot_.spu_voice_active[voice];

                ImGui::TextColored(active ? ImVec4(0.4f, 0.9f, 0.4f, 1.0f)
                    : ImVec4(0.55f, 0.55f, 0.55f, 1.0f),
                    "V%02u", static_cast<unsigned>(voice));
                ImGui::SameLine();
                ImGui::TextUnformatted(active ? "ON" : "OFF");
                ImGui::ProgressBar(meter, ImVec2(-1.0f, 0.0f));
                ImGui::Text("L:%6d  R:%6d", static_cast<int>(level_l),
                    static_cast<int>(level_r));
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}
