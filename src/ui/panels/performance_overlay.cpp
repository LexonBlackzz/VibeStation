#include "ui/app.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <vector>

namespace {
    struct GpuDipDiagnostics {
        bool valid = false;
        int dip_count = 0;
        double avg_interval_frames = 0.0;
        double avg_dip_words = 0.0;
        double avg_non_dip_words = 0.0;
        double avg_dip_draws = 0.0;
        double avg_non_dip_draws = 0.0;
        u64 latest_dip_frame = 0;
        u32 latest_dip_pc = 0;
        u32 latest_dip_dma2_words = 0;
        u32 latest_dip_dma2_base = 0;
        bool latest_dip_display_reused = false;
        std::array<u64, 3> recent_frames{};
        int recent_frame_count = 0;
    };

    template <size_t N>
    GpuDipDiagnostics analyze_gpu_dip_pattern(
        const std::array<float, N>& gpu_history,
        const std::array<u64, N>& frame_history,
        const std::array<u32, N>& gpu_words_history,
        const std::array<u32, N>& gpu_draw_history,
        const std::array<u32, N>& cpu_pc_history,
        const std::array<u32, N>& dma2_words_history,
        const std::array<u32, N>& dma2_base_history,
        const std::array<u32, N>& display_hash_history,
        int count,
        int write_index) {
        GpuDipDiagnostics diag{};
        if (count < 8) {
            return diag;
        }

        std::vector<float> samples;
        std::vector<u64> frames;
        std::vector<u32> words;
        std::vector<u32> draws;
        std::vector<u32> pcs;
        std::vector<u32> dma2_words;
        std::vector<u32> dma2_bases;
        std::vector<u32> display_hashes;
        samples.reserve(static_cast<size_t>(count));
        frames.reserve(static_cast<size_t>(count));
        words.reserve(static_cast<size_t>(count));
        draws.reserve(static_cast<size_t>(count));
        pcs.reserve(static_cast<size_t>(count));
        dma2_words.reserve(static_cast<size_t>(count));
        dma2_bases.reserve(static_cast<size_t>(count));
        display_hashes.reserve(static_cast<size_t>(count));
        for (int i = 0; i < count; ++i) {
            const int idx = (write_index - count + i + static_cast<int>(N)) %
                static_cast<int>(N);
            samples.push_back(gpu_history[idx]);
            frames.push_back(frame_history[idx]);
            words.push_back(gpu_words_history[idx]);
            draws.push_back(gpu_draw_history[idx]);
            pcs.push_back(cpu_pc_history[idx]);
            dma2_words.push_back(dma2_words_history[idx]);
            dma2_bases.push_back(dma2_base_history[idx]);
            display_hashes.push_back(display_hash_history[idx]);
        }

        std::vector<float> sorted = samples;
        std::sort(sorted.begin(), sorted.end());
        const float median = sorted[sorted.size() / 2];
        const float peak = *std::max_element(samples.begin(), samples.end());
        if (peak <= 0.0f) {
            return diag;
        }

        const float dip_threshold =
            std::min(median * 0.75f, peak * 0.60f);
        std::vector<u64> dip_frames;
        std::vector<size_t> dip_indices;
        dip_frames.reserve(samples.size() / 8u);
        for (int i = 1; i + 1 < count; ++i) {
            const float cur = samples[static_cast<size_t>(i)];
            if (cur > dip_threshold) {
                continue;
            }
            if (cur >= samples[static_cast<size_t>(i - 1)] ||
                cur > samples[static_cast<size_t>(i + 1)]) {
                continue;
            }
            const u64 frame_id = frames[static_cast<size_t>(i)];
            if (!dip_frames.empty() && frame_id <= dip_frames.back() + 1u) {
                continue;
            }
            dip_frames.push_back(frame_id);
            dip_indices.push_back(static_cast<size_t>(i));
        }

        diag.dip_count = static_cast<int>(dip_frames.size());
        if (!dip_frames.empty()) {
            const size_t recent_count = std::min<size_t>(3, dip_frames.size());
            diag.recent_frame_count = static_cast<int>(recent_count);
            for (size_t i = 0; i < recent_count; ++i) {
                diag.recent_frames[i] =
                    dip_frames[dip_frames.size() - recent_count + i];
            }
            const size_t latest_index = dip_indices.back();
            diag.latest_dip_frame = frames[latest_index];
            diag.latest_dip_pc = pcs[latest_index];
            diag.latest_dip_dma2_words = dma2_words[latest_index];
            diag.latest_dip_dma2_base = dma2_bases[latest_index];
            diag.latest_dip_display_reused =
                (latest_index > 0) &&
                (display_hashes[latest_index] == display_hashes[latest_index - 1]);
        }

        if (dip_frames.size() < 2) {
            if (!dip_indices.empty() && dip_indices.size() < samples.size()) {
                double dip_word_sum = 0.0;
                double dip_draw_sum = 0.0;
                for (size_t dip_index : dip_indices) {
                    dip_word_sum += static_cast<double>(words[dip_index]);
                    dip_draw_sum += static_cast<double>(draws[dip_index]);
                }
                diag.avg_dip_words =
                    dip_word_sum / static_cast<double>(dip_indices.size());
                diag.avg_dip_draws =
                    dip_draw_sum / static_cast<double>(dip_indices.size());

                double non_dip_word_sum = 0.0;
                double non_dip_draw_sum = 0.0;
                size_t non_dip_count = 0;
                for (size_t i = 0; i < samples.size(); ++i) {
                    if (std::find(dip_indices.begin(), dip_indices.end(), i) !=
                        dip_indices.end()) {
                        continue;
                    }
                    non_dip_word_sum += static_cast<double>(words[i]);
                    non_dip_draw_sum += static_cast<double>(draws[i]);
                    ++non_dip_count;
                }
                if (non_dip_count > 0) {
                    diag.avg_non_dip_words =
                        non_dip_word_sum / static_cast<double>(non_dip_count);
                    diag.avg_non_dip_draws =
                        non_dip_draw_sum / static_cast<double>(non_dip_count);
                }
            }
            return diag;
        }

        double interval_sum = 0.0;
        for (size_t i = 1; i < dip_frames.size(); ++i) {
            interval_sum += static_cast<double>(dip_frames[i] - dip_frames[i - 1]);
        }
        diag.avg_interval_frames =
            interval_sum / static_cast<double>(dip_frames.size() - 1);
        double dip_word_sum = 0.0;
        double dip_draw_sum = 0.0;
        for (size_t dip_index : dip_indices) {
            dip_word_sum += static_cast<double>(words[dip_index]);
            dip_draw_sum += static_cast<double>(draws[dip_index]);
        }
        diag.avg_dip_words =
            dip_word_sum / static_cast<double>(std::max<size_t>(1, dip_indices.size()));
        diag.avg_dip_draws =
            dip_draw_sum / static_cast<double>(std::max<size_t>(1, dip_indices.size()));
        double non_dip_word_sum = 0.0;
        double non_dip_draw_sum = 0.0;
        size_t non_dip_count = 0;
        for (size_t i = 0; i < samples.size(); ++i) {
            if (std::find(dip_indices.begin(), dip_indices.end(), i) !=
                dip_indices.end()) {
                continue;
            }
            non_dip_word_sum += static_cast<double>(words[i]);
            non_dip_draw_sum += static_cast<double>(draws[i]);
            ++non_dip_count;
        }
        if (non_dip_count > 0) {
            diag.avg_non_dip_words =
                non_dip_word_sum / static_cast<double>(non_dip_count);
            diag.avg_non_dip_draws =
                non_dip_draw_sum / static_cast<double>(non_dip_count);
        }
        diag.valid = true;
        return diag;
    }
}

void App::push_performance_history_sample() {
    if (!has_started_emulation_) {
        perf_history_write_index_ = 0;
        perf_history_count_ = 0;
        perf_history_last_frame_id_ = 0;
        perf_history_has_last_frame_id_ = false;
        return;
    }

    const u64 frame_id = runtime_snapshot_.frame_id;
    if (perf_history_has_last_frame_id_ &&
        frame_id == perf_history_last_frame_id_) {
        return;
    }

    const int idx = perf_history_write_index_;
    perf_cpu_ms_history_[idx] =
        static_cast<float>(std::max(0.0, runtime_snapshot_.profiling.cpu_ms));
    perf_gpu_ms_history_[idx] =
        static_cast<float>(std::max(0.0, runtime_snapshot_.profiling.gpu_ms));
    perf_core_ms_history_[idx] =
        static_cast<float>(std::max(0.0, runtime_snapshot_.core_frame_ms));
    perf_frame_id_history_[idx] = frame_id;
    if (g_profile_detailed_timing) {
        perf_gpu_words_history_[idx] = runtime_snapshot_.profiling.gpu_gp0_words;
        perf_gpu_draw_commands_history_[idx] =
            runtime_snapshot_.profiling.gpu_draw_commands;
        perf_cpu_pc_history_[idx] = runtime_snapshot_.cpu_pc;
        perf_dma2_words_history_[idx] = runtime_snapshot_.dma2_words;
        perf_dma2_base_history_[idx] = runtime_snapshot_.dma2_base_addr;
        perf_display_hash_history_[idx] = runtime_snapshot_.boot_diag.display_hash;
    }
    else {
        perf_gpu_words_history_[idx] = 0;
        perf_gpu_draw_commands_history_[idx] = 0;
        perf_cpu_pc_history_[idx] = 0;
        perf_dma2_words_history_[idx] = 0;
        perf_dma2_base_history_[idx] = 0;
        perf_display_hash_history_[idx] = 0;
    }

    perf_history_write_index_ = (perf_history_write_index_ + 1) % kPerfHistorySamples;
    if (perf_history_count_ < kPerfHistorySamples) {
        ++perf_history_count_;
    }
    perf_history_last_frame_id_ = frame_id;
    perf_history_has_last_frame_id_ = true;
}

void App::draw_performance_gpu_dip_diagnostics() {
    const GpuDipDiagnostics dip_diag = analyze_gpu_dip_pattern(
        perf_gpu_ms_history_,
        perf_frame_id_history_,
        perf_gpu_words_history_,
        perf_gpu_draw_commands_history_,
        perf_cpu_pc_history_,
        perf_dma2_words_history_,
        perf_dma2_base_history_,
        perf_display_hash_history_,
        perf_history_count_,
        perf_history_write_index_);
    if (dip_diag.valid) {
        ImGui::Text("GPU dip interval: ~%.1f frames (%d dips in history)",
            dip_diag.avg_interval_frames, dip_diag.dip_count);
    }
    else if (dip_diag.dip_count > 0) {
        ImGui::Text("GPU dips seen in history: %d", dip_diag.dip_count);
    }
    if (dip_diag.recent_frame_count > 0) {
        ImGui::Text("Recent dip frames:");
        ImGui::SameLine();
        for (int i = 0; i < dip_diag.recent_frame_count; ++i) {
            if (i > 0) {
                ImGui::SameLine();
                ImGui::TextUnformatted(",");
                ImGui::SameLine();
            }
            ImGui::Text("%llu",
                static_cast<unsigned long long>(dip_diag.recent_frames[i]));
            if (i + 1 < dip_diag.recent_frame_count) {
                ImGui::SameLine();
            }
        }
    }
    if ((dip_diag.valid || dip_diag.dip_count > 0) &&
        dip_diag.avg_non_dip_words > 0.0) {
        ImGui::Text("GP0 at dips: %.0f words / %.0f draws",
            dip_diag.avg_dip_words, dip_diag.avg_dip_draws);
        ImGui::Text("GP0 normal: %.0f words / %.0f draws",
            dip_diag.avg_non_dip_words, dip_diag.avg_non_dip_draws);
    }
}

void App::draw_performance_overlay(const ImVec2& image_pos, const ImVec2& image_size) {
    if (!show_perf_ || !has_started_emulation_ || perf_history_count_ < 2) {
        return;
    }
    if (image_size.x < 240.0f || image_size.y < 140.0f) {
        return;
    }

    const float overlay_w = std::min(455.0f, image_size.x - 20.0f);
    const float overlay_h = std::min(150.0f, image_size.y - 20.0f);
    if (overlay_w < 220.0f || overlay_h < 100.0f) {
        return;
    }

    const ImVec2 p0(image_pos.x + 10.0f, image_pos.y + 10.0f);
    const ImVec2 p1(p0.x + overlay_w, p0.y + overlay_h);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, p1, IM_COL32(8, 8, 12, 220), 6.0f);
    dl->AddRect(p0, p1, IM_COL32(140, 140, 170, 250), 6.0f);

    const auto& stats = runtime_snapshot_.profiling;
    const double slowdown_percent = current_emulation_slowdown_percent();
    const bool unlimited_turbo_active =
        turbo_hold_active_ && config_turbo_speed_percent_ <= 0;
    const double effective_speed_multiplier =
        (measured_emulation_speed_multiplier_ > 0.0)
        ? measured_emulation_speed_multiplier_
        : current_effective_speed_multiplier();
    const GpuDipDiagnostics dip_diag = g_profile_detailed_timing
        ? analyze_gpu_dip_pattern(
            perf_gpu_ms_history_,
            perf_frame_id_history_,
            perf_gpu_words_history_,
            perf_gpu_draw_commands_history_,
            perf_cpu_pc_history_,
            perf_dma2_words_history_,
            perf_dma2_base_history_,
            perf_display_hash_history_,
            perf_history_count_,
            perf_history_write_index_)
        : GpuDipDiagnostics{};
    char header[160];
    std::snprintf(header, sizeof(header),
        "CPU %.2f ms  GPU %.2f ms  Core %.2f ms  Game %.1f  Video %.1f",
        stats.cpu_ms, stats.gpu_ms, runtime_snapshot_.core_frame_ms,
        game_fps_, video_fps_);
    dl->AddText(ImVec2(p0.x + 10.0f, p0.y + 7.0f), IM_COL32(235, 235, 245, 255), header);

    char status_text[64];
    ImU32 status_color = IM_COL32(150, 220, 150, 255);
    if (unlimited_turbo_active) {
        std::snprintf(status_text, sizeof(status_text), "Turbo x%.2f",
            effective_speed_multiplier);
        status_color = IM_COL32(245, 205, 90, 255);
    }
    else {
        std::snprintf(status_text, sizeof(status_text), "Slowdown %.1f%%",
            slowdown_percent);
        status_color =
            (slowdown_percent >= 5.0)
            ? IM_COL32(240, 110, 110, 255)
            : IM_COL32(150, 220, 150, 255);
    }
    dl->AddText(ImVec2(p0.x + 10.0f, p0.y + 23.0f), status_color, status_text);

    if (g_profile_detailed_timing && dip_diag.valid) {
        char dip_text[64];
        std::snprintf(dip_text, sizeof(dip_text), "GPU dips ~%.1f frames",
            dip_diag.avg_interval_frames);
        dl->AddText(ImVec2(p0.x + 150.0f, p0.y + 23.0f),
            IM_COL32(180, 180, 200, 255), dip_text);
    }
    else if (g_profile_detailed_timing && dip_diag.dip_count > 0) {
        char dip_text[48];
        std::snprintf(dip_text, sizeof(dip_text), "GPU dips %d",
            dip_diag.dip_count);
        dl->AddText(ImVec2(p0.x + 150.0f, p0.y + 23.0f),
            IM_COL32(180, 180, 200, 255), dip_text);
    }

    if (g_profile_detailed_timing &&
        (dip_diag.valid || dip_diag.dip_count > 0) &&
        dip_diag.avg_non_dip_words > 0.0) {
        char dip_work_text[80];
        std::snprintf(dip_work_text, sizeof(dip_work_text),
            "GP0@dip %.0fw/%.0fd vs %.0fw/%.0fd",
            dip_diag.avg_dip_words,
            dip_diag.avg_dip_draws,
            dip_diag.avg_non_dip_words,
            dip_diag.avg_non_dip_draws);
        dl->AddText(ImVec2(p0.x + 10.0f, p0.y + 36.0f),
            IM_COL32(170, 170, 185, 255), dip_work_text);
    }
    if (g_profile_detailed_timing && dip_diag.latest_dip_frame != 0) {
        char dip_state_text[128];
        std::snprintf(dip_state_text, sizeof(dip_state_text),
            "Last dip f%llu pc=%08X dma2=%uw @%08X disp=%s",
            static_cast<unsigned long long>(dip_diag.latest_dip_frame),
            dip_diag.latest_dip_pc,
            dip_diag.latest_dip_dma2_words,
            dip_diag.latest_dip_dma2_base,
            dip_diag.latest_dip_display_reused ? "reused" : "changed");
        dl->AddText(ImVec2(p0.x + 10.0f, p0.y + 48.0f),
            IM_COL32(170, 170, 185, 255), dip_state_text);
    }

    const float gx0 = p0.x + 10.0f;
    const float gx1 = p1.x - 10.0f;
    const float gy0 = p0.y + 66.0f;
    const float gy1 = p1.y - 10.0f;
    const float gw = gx1 - gx0;
    const float gh = gy1 - gy0;
    if (gw <= 1.0f || gh <= 1.0f) {
        return;
    }

    float peak = 0.0f;
    for (int i = 0; i < perf_history_count_; ++i) {
        const int idx = (perf_history_write_index_ - perf_history_count_ + i +
            kPerfHistorySamples) %
            kPerfHistorySamples;
        peak = std::max(peak, perf_cpu_ms_history_[idx]);
        peak = std::max(peak, perf_gpu_ms_history_[idx]);
        peak = std::max(peak, perf_core_ms_history_[idx]);
    }
    constexpr float kFrameBudgetMs = 1000.0f / 60.0f;
    const float scale_max = std::max(kFrameBudgetMs, std::max(8.0f, peak * 1.2f));

    const float budget_y = gy1 - (kFrameBudgetMs / scale_max) * gh;
    dl->AddLine(ImVec2(gx0, budget_y), ImVec2(gx1, budget_y),
        IM_COL32(220, 190, 80, 100), 1.0f);

    std::array<ImVec2, kPerfHistorySamples> cpu_pts{};
    std::array<ImVec2, kPerfHistorySamples> gpu_pts{};
    std::array<ImVec2, kPerfHistorySamples> core_pts{};
    const int count = perf_history_count_;
    const float denom = static_cast<float>(std::max(1, count - 1));
    for (int i = 0; i < count; ++i) {
        const int idx = (perf_history_write_index_ - count + i + kPerfHistorySamples) %
            kPerfHistorySamples;
        const float x = gx0 + (static_cast<float>(i) / denom) * gw;
        const float cpu_y = gy1 - std::min(perf_cpu_ms_history_[idx], scale_max) / scale_max * gh;
        const float gpu_y = gy1 - std::min(perf_gpu_ms_history_[idx], scale_max) / scale_max * gh;
        const float core_y = gy1 - std::min(perf_core_ms_history_[idx], scale_max) / scale_max * gh;
        cpu_pts[i] = ImVec2(x, cpu_y);
        gpu_pts[i] = ImVec2(x, gpu_y);
        core_pts[i] = ImVec2(x, core_y);
    }

    dl->AddPolyline(core_pts.data(), count, IM_COL32(190, 190, 210, 200), 0, 1.0f);
    dl->AddPolyline(cpu_pts.data(), count, IM_COL32(90, 240, 90, 255), 0, 2.0f);
    dl->AddPolyline(gpu_pts.data(), count, IM_COL32(255, 110, 110, 255), 0, 2.0f);

    const float legend_y = gy0 + 3.0f;
    dl->AddText(ImVec2(gx0 + 6.0f, legend_y), IM_COL32(90, 240, 90, 255), "CPU");
    dl->AddText(ImVec2(gx0 + 48.0f, legend_y), IM_COL32(255, 110, 110, 255), "GPU");
    dl->AddText(ImVec2(gx0 + 90.0f, legend_y), IM_COL32(190, 190, 210, 255), "Core");
}
