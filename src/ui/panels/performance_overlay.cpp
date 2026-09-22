#include "ui/app.h"
#include "ui/panels/cpu_backend_panel.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace {
    const char* gte_command_name(u32 opcode) {
        switch (opcode & 0x3Fu) {
        case 0x01: return "RTPS";
        case 0x06: return "NCLIP";
        case 0x0C: return "OP";
        case 0x10: return "DPCS";
        case 0x11: return "INTPL";
        case 0x12: return "MVMVA";
        case 0x13: return "NCDS";
        case 0x14: return "CDP";
        case 0x16: return "NCDT";
        case 0x1B: return "NCCS";
        case 0x1C: return "CC";
        case 0x1E: return "NCS";
        case 0x20: return "NCT";
        case 0x28: return "SQR";
        case 0x29: return "DCPL";
        case 0x2A: return "DPCT";
        case 0x2D: return "AVSZ3";
        case 0x2E: return "AVSZ4";
        case 0x30: return "RTPT";
        case 0x3D: return "GPF";
        case 0x3E: return "GPL";
        case 0x3F: return "NCCT";
        default: return "UNKNOWN";
        }
    }

    struct FramePhaseDiagnostics {
        bool valid = false;
        u32 render_frames = 0;
        u32 reuse_frames = 0;
        u32 draw_threshold = 0;
        double render_cpu_ms = 0.0;
        double render_gpu_ms = 0.0;
        double render_core_ms = 0.0;
        double reuse_cpu_ms = 0.0;
        double reuse_gpu_ms = 0.0;
        double reuse_core_ms = 0.0;
    };

    template <size_t N>
    FramePhaseDiagnostics analyze_frame_phases(
        const std::array<float, N>& cpu_history,
        const std::array<float, N>& gpu_history,
        const std::array<float, N>& core_history,
        const std::array<u32, N>& draw_history,
        int count,
        int write_index) {
        FramePhaseDiagnostics out{};
        if (count < 8) {
            return out;
        }

        u32 max_draws = 0;
        for (int i = 0; i < count; ++i) {
            const int idx =
                (write_index - count + i + static_cast<int>(N)) %
                static_cast<int>(N);
            max_draws = std::max(max_draws, draw_history[idx]);
        }
        if (max_draws < 16u) {
            return out;
        }

        out.draw_threshold = std::max<u32>(8u, max_draws / 8u);
        for (int i = 0; i < count; ++i) {
            const int idx =
                (write_index - count + i + static_cast<int>(N)) %
                static_cast<int>(N);
            if (draw_history[idx] >= out.draw_threshold) {
                ++out.render_frames;
                out.render_cpu_ms += cpu_history[idx];
                out.render_gpu_ms += gpu_history[idx];
                out.render_core_ms += core_history[idx];
            }
            else {
                ++out.reuse_frames;
                out.reuse_cpu_ms += cpu_history[idx];
                out.reuse_gpu_ms += gpu_history[idx];
                out.reuse_core_ms += core_history[idx];
            }
        }

        if (out.render_frames == 0 || out.reuse_frames == 0) {
            return out;
        }
        out.render_cpu_ms /= static_cast<double>(out.render_frames);
        out.render_gpu_ms /= static_cast<double>(out.render_frames);
        out.render_core_ms /= static_cast<double>(out.render_frames);
        out.reuse_cpu_ms /= static_cast<double>(out.reuse_frames);
        out.reuse_gpu_ms /= static_cast<double>(out.reuse_frames);
        out.reuse_core_ms /= static_cast<double>(out.reuse_frames);
        out.valid = true;
        return out;
    }

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

void App::panel_performance() {
    ImGui::SetNextWindowSize(ImVec2(400, 300), ImGuiCond_FirstUseEver);
    const bool was_open = show_perf_profiler_;
    if (ImGui::Begin("Performance Profiler", &show_perf_profiler_)) {
        if (!has_started_emulation_) {
            ImGui::Text("Emulation not running.");
            ImGui::End();
            return;
        }

        const auto& stats = runtime_snapshot_.profiling;
        ImGui::Text("Frame Time Breakdown:");
        ImGui::Separator();

        auto row = [](const char* label, double ms, ImVec4 color) {
            ImGui::Text("%-10s:", label);
            ImGui::SameLine(100);
            ImGui::TextColored(color, "%.3f ms", ms);
            };

        if (g_profile_detailed_timing) {
            row("CPU*", stats.cpu_ms, ImVec4(0.4f, 0.8f, 0.4f, 1.0f));
            row("GPU", stats.gpu_ms, ImVec4(0.8f, 0.4f, 0.4f, 1.0f));
            row("SPU", stats.spu_ms, ImVec4(0.4f, 0.4f, 0.8f, 1.0f));
            row("DMA", stats.dma_ms, ImVec4(0.8f, 0.8f, 0.4f, 1.0f));
            row("Timers", stats.timers_ms, ImVec4(0.4f, 0.8f, 0.8f, 1.0f));
            row("CDROM", stats.cdrom_ms, ImVec4(0.8f, 0.4f, 0.8f, 1.0f));
            ImGui::TextDisabled("*CPU excludes time already attributed to GPU.");
            ImGui::TextDisabled(
                "GPU here is CPU-side emulated GPU command time, not host GPU timestamp/present time.");

            ImGui::Separator();
            ImGui::Text("GPU Command Detail:");
            auto gpu_detail_row = [](const char* label, double ms, u32 commands) {
                ImGui::Text("%-18s %7.3f ms  %6u cmds", label, ms, commands);
            };
            gpu_detail_row("Flat polygons", stats.gpu_flat_ms,
                stats.gpu_flat_commands);
            gpu_detail_row("Gouraud polygons", stats.gpu_gouraud_ms,
                stats.gpu_gouraud_commands);
            gpu_detail_row("Textured polygons", stats.gpu_textured_ms,
                stats.gpu_textured_commands);
            gpu_detail_row("Gouraud + texture", stats.gpu_gouraud_textured_ms,
                stats.gpu_gouraud_textured_commands);
            gpu_detail_row("Rectangles", stats.gpu_rect_ms,
                stats.gpu_rect_commands);
            gpu_detail_row("Lines", stats.gpu_line_ms,
                stats.gpu_line_commands);
            gpu_detail_row("VRAM transfers", stats.gpu_transfer_ms,
                stats.gpu_transfer_commands);
            gpu_detail_row("Other GP0", stats.gpu_other_ms,
                stats.gpu_other_commands);

            const double gpu_attributed_ms =
                stats.gpu_flat_ms + stats.gpu_gouraud_ms +
                stats.gpu_textured_ms + stats.gpu_gouraud_textured_ms +
                stats.gpu_rect_ms + stats.gpu_line_ms +
                stats.gpu_transfer_ms + stats.gpu_other_ms;
            const double gpu_unattributed_ms =
                std::max(0.0, stats.gpu_ms - gpu_attributed_ms);
            ImGui::Text("Dispatch attributed: %.3f ms   GP0 overhead/data: %.3f ms",
                gpu_attributed_ms, gpu_unattributed_ms);

            ImGui::Separator();
            ImGui::Text("Textured Raster Work:");
            const double coverage_percent =
                (stats.gpu_candidate_pixels != 0)
                ? (100.0 * static_cast<double>(stats.gpu_covered_pixels) /
                    static_cast<double>(stats.gpu_candidate_pixels))
                : 0.0;
            ImGui::Text(
                "BBox pixels: %llu   covered/sampled: %llu (%.1f%% coverage)",
                static_cast<unsigned long long>(stats.gpu_candidate_pixels),
                static_cast<unsigned long long>(stats.gpu_covered_pixels),
                coverage_percent);
            if (stats.gpu_candidate_pixels != 0) {
                ImGui::Text("Span rejection avoids up to %.1f%% of bbox pixels",
                    100.0 - coverage_percent);
            }
            ImGui::Text("Texels 4/8/15-bit: %llu / %llu / %llu",
                static_cast<unsigned long long>(stats.gpu_texel_samples_4bit),
                static_cast<unsigned long long>(stats.gpu_texel_samples_8bit),
                static_cast<unsigned long long>(stats.gpu_texel_samples_15bit));
            ImGui::Text("Transparent: %llu   semi-transparent: %llu",
                static_cast<unsigned long long>(stats.gpu_transparent_texels),
                static_cast<unsigned long long>(
                    stats.gpu_semitransparent_pixels));
            ImGui::TextDisabled(
                "BBox is the old bounding-box workload; accurate triangle spans now iterate only covered pixels.");

            if (ImGui::Button("Copy GPU profiler snapshot")) {
                char snapshot[2048];
                std::snprintf(
                    snapshot, sizeof(snapshot),
                    "frame=%llu core_ms=%.3f gpu_ms=%.3f\n"
                    "flat=%.3fms/%u gouraud=%.3fms/%u "
                    "textured=%.3fms/%u gouraud_textured=%.3fms/%u\n"
                    "rect=%.3fms/%u line=%.3fms/%u transfer=%.3fms/%u "
                    "other=%.3fms/%u overhead_data=%.3fms\n"
                    "candidates=%llu covered=%llu coverage=%.1f%%\n"
                    "texels_4=%llu texels_8=%llu texels_15=%llu "
                    "transparent=%llu semi=%llu\n"
                    "gp0_words=%u gp0_commands=%u draw_commands=%u",
                    static_cast<unsigned long long>(runtime_snapshot_.frame_id),
                    runtime_snapshot_.core_frame_ms, stats.gpu_ms,
                    stats.gpu_flat_ms, stats.gpu_flat_commands,
                    stats.gpu_gouraud_ms, stats.gpu_gouraud_commands,
                    stats.gpu_textured_ms, stats.gpu_textured_commands,
                    stats.gpu_gouraud_textured_ms,
                    stats.gpu_gouraud_textured_commands,
                    stats.gpu_rect_ms, stats.gpu_rect_commands,
                    stats.gpu_line_ms, stats.gpu_line_commands,
                    stats.gpu_transfer_ms, stats.gpu_transfer_commands,
                    stats.gpu_other_ms, stats.gpu_other_commands,
                    gpu_unattributed_ms,
                    static_cast<unsigned long long>(stats.gpu_candidate_pixels),
                    static_cast<unsigned long long>(stats.gpu_covered_pixels),
                    coverage_percent,
                    static_cast<unsigned long long>(stats.gpu_texel_samples_4bit),
                    static_cast<unsigned long long>(stats.gpu_texel_samples_8bit),
                    static_cast<unsigned long long>(stats.gpu_texel_samples_15bit),
                    static_cast<unsigned long long>(stats.gpu_transparent_texels),
                    static_cast<unsigned long long>(
                        stats.gpu_semitransparent_pixels),
                    stats.gpu_gp0_words, stats.gpu_gp0_commands,
                    stats.gpu_draw_commands);
                ImGui::SetClipboardText(snapshot);
            }

            draw_performance_gpu_dip_diagnostics();

            const FramePhaseDiagnostics phase_diag = analyze_frame_phases(
                perf_cpu_ms_history_, perf_gpu_ms_history_,
                perf_core_ms_history_, perf_gpu_draw_commands_history_,
                perf_history_count_, perf_history_write_index_);
            if (phase_diag.valid) {
                ImGui::Separator();
                ImGui::Text(
                    "Render/reuse phases (draw threshold %u, history %u/%u):",
                    phase_diag.draw_threshold, phase_diag.render_frames,
                    phase_diag.reuse_frames);
                ImGui::Text(
                    "Render-active avg: CPU %.3f  GPU %.3f  Core %.3f ms",
                    phase_diag.render_cpu_ms, phase_diag.render_gpu_ms,
                    phase_diag.render_core_ms);
                ImGui::Text(
                    "Reuse/light avg:  CPU %.3f  GPU %.3f  Core %.3f ms",
                    phase_diag.reuse_cpu_ms, phase_diag.reuse_gpu_ms,
                    phase_diag.reuse_core_ms);
            }

            ImGui::Separator();
            ImGui::Text("GTE Command Detail:");
            if (stats.gte_total_commands == 0) {
                ImGui::TextDisabled("No GTE commands executed in this frame.");
            }
            else {
                struct GteRow {
                    u32 opcode = 0;
                    double ms = 0.0;
                    u32 count = 0;
                };
                std::array<GteRow, 64> rows{};
                u32 row_count = 0;
                for (u32 opcode = 0; opcode < 64; ++opcode) {
                    if (stats.gte_command_counts[opcode] == 0) {
                        continue;
                    }
                    rows[row_count++] = {
                        opcode,
                        stats.gte_command_ms[opcode],
                        stats.gte_command_counts[opcode],
                    };
                }
                std::sort(rows.begin(), rows.begin() + row_count,
                    [](const GteRow& a, const GteRow& b) {
                        return a.ms > b.ms;
                    });

                ImGui::Text(
                    "Total: %.3f ms / %u commands (inside CPU time)",
                    stats.gte_total_ms, stats.gte_total_commands);
                const u32 shown = std::min<u32>(row_count, 12u);
                for (u32 i = 0; i < shown; ++i) {
                    const GteRow& row = rows[i];
                    const double avg_us =
                        row.count == 0
                            ? 0.0
                            : (row.ms * 1000.0) /
                                  static_cast<double>(row.count);
                    ImGui::Text(
                        "%-7s  op=%02X  %7.3f ms  %6u cmds  %6.2f us/cmd",
                        gte_command_name(row.opcode), row.opcode, row.ms,
                        row.count, avg_us);
                }

                if (ImGui::Button("Copy GTE profiler snapshot")) {
                    std::string snapshot;
                    snapshot.reserve(2048);
                    char line[256];
                    std::snprintf(
                        line, sizeof(line),
                        "frame=%llu cpu_ms=%.3f gte_ms=%.3f gte_commands=%u\n",
                        static_cast<unsigned long long>(
                            runtime_snapshot_.frame_id),
                        stats.cpu_ms, stats.gte_total_ms,
                        stats.gte_total_commands);
                    snapshot += line;
                    for (u32 i = 0; i < row_count; ++i) {
                        const GteRow& row = rows[i];
                        const double avg_us =
                            row.count == 0
                                ? 0.0
                                : (row.ms * 1000.0) /
                                      static_cast<double>(row.count);
                        std::snprintf(
                            line, sizeof(line),
                            "%s op=%02X ms=%.3f count=%u avg_us=%.2f\n",
                            gte_command_name(row.opcode), row.opcode, row.ms,
                            row.count, avg_us);
                        snapshot += line;
                    }
                    ImGui::SetClipboardText(snapshot.c_str());
                }
            }
        }
        else {
            ImGui::TextDisabled("Detailed subsystem timings are disabled.");
        }
        ImGui::Separator();
        row("Core", runtime_snapshot_.core_frame_ms, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        row("Render", render_ms_, ImVec4(0.9f, 0.9f, 0.7f, 1.0f));
        row("Swap", swap_ms_, ImVec4(0.7f, 0.9f, 0.9f, 1.0f));
        row("Present", present_ms_, ImVec4(0.7f, 0.9f, 0.9f, 1.0f));
        const CpuBackendStats& backend = runtime_snapshot_.cpu_backend_stats;
        ImGui::Separator();
        draw_cpu_backend_mode_summary(backend, runtime_snapshot_.cpu_backend);
        ImGui::Text("Decoded blocks: %u  Cache: %llu / %llu  Invalidations: %llu  Flushes: %llu",
            backend.block_count,
            static_cast<unsigned long long>(backend.cache_hits),
            static_cast<unsigned long long>(backend.cache_misses),
            static_cast<unsigned long long>(backend.invalidations),
            static_cast<unsigned long long>(backend.flushes));
        ImGui::Text("Invalidation queries: %llu  no-code exits: %llu",
            static_cast<unsigned long long>(backend.invalidation_queries),
            static_cast<unsigned long long>(
                backend.invalidation_fast_no_code_page_exits));
        ImGui::Text("Invalidation blocks: examined %llu  invalidated %llu",
            static_cast<unsigned long long>(backend.invalidation_blocks_examined),
            static_cast<unsigned long long>(
                backend.invalidation_blocks_invalidated));
        ImGui::Text("Decoded/native/fallback instructions: %llu / %llu / %llu",
            static_cast<unsigned long long>(backend.decoded_instructions),
            static_cast<unsigned long long>(backend.native_instructions),
            static_cast<unsigned long long>(backend.fallback_instructions));
        ImGui::Text("JIT V2 native split: inline %llu  helper-backed %llu  helper entries %llu",
            static_cast<unsigned long long>(backend.jit_v2_inline_instructions),
            static_cast<unsigned long long>(backend.jit_v2_helper_instructions),
            static_cast<unsigned long long>(backend.jit_v2_helper_entries));
        if (backend.forced_interpreter_instructions != 0 ||
            backend.forced_interpreter_last_reason !=
                CpuForcedInterpreterReason::None) {
            ImGui::Text("Forced interpreter: %s  slices %llu  instr %llu",
                cpu_forced_interpreter_reason_name(
                    backend.forced_interpreter_last_reason),
                static_cast<unsigned long long>(
                    backend.forced_interpreter_slices),
                static_cast<unsigned long long>(
                    backend.forced_interpreter_instructions));
        }
        ImGui::Text("Native: available %u  attempts %llu  successes %llu  compiled %llu",
            backend.native_available ? 1u : 0u,
            static_cast<unsigned long long>(backend.native_compile_attempts),
            static_cast<unsigned long long>(backend.native_compile_successes),
            static_cast<unsigned long long>(backend.native_blocks_compiled));
        ImGui::Text("Native entries: %llu  cycles %llu  code %zu bytes",
            static_cast<unsigned long long>(backend.native_block_entries),
            static_cast<unsigned long long>(backend.native_cycles),
            backend.native_code_bytes);
        ImGui::Text("Native chains: %llu  transitions %llu  max blocks %llu",
            static_cast<unsigned long long>(backend.native_chain_entries),
            static_cast<unsigned long long>(backend.native_linked_transitions),
            static_cast<unsigned long long>(backend.native_chain_max_blocks));
        ImGui::Text("Native fallback: rejected %llu  compile fail %llu  decoded %llu",
            static_cast<unsigned long long>(
                backend.native_rejected_unsafe_blocks),
            static_cast<unsigned long long>(backend.native_compile_failures),
            static_cast<unsigned long long>(
                backend.native_to_decoded_fallbacks));
        ImGui::Text("Native gating: cold %llu  short %llu",
            static_cast<unsigned long long>(backend.native_hot_threshold_skips),
            static_cast<unsigned long long>(backend.native_short_block_skips));
        ImGui::Text("Native rejects: branch %llu  mem %llu  cop0 %llu  cop2 %llu",
            static_cast<unsigned long long>(backend.native_reject_branch),
            static_cast<unsigned long long>(backend.native_reject_memory),
            static_cast<unsigned long long>(backend.native_reject_cop0),
            static_cast<unsigned long long>(backend.native_reject_cop2));
        ImGui::Text("Native rejects: exc/unk %llu  state %llu  budget %llu  icache %llu",
            static_cast<unsigned long long>(
                backend.native_reject_exception_unknown),
            static_cast<unsigned long long>(backend.native_reject_unsafe_state),
            static_cast<unsigned long long>(backend.native_reject_budget),
            static_cast<unsigned long long>(backend.native_reject_icache));
        ImGui::Text("Native state rejects: pc %llu  branch-delay %llu  load-delay %llu",
            static_cast<unsigned long long>(backend.native_reject_pc_state),
            static_cast<unsigned long long>(
                backend.native_reject_branch_delay_state),
            static_cast<unsigned long long>(
                backend.native_reject_load_delay_state));
        ImGui::Text("Native state rejects: irq %llu  invalidated %llu  other %llu",
            static_cast<unsigned long long>(backend.native_reject_irq_state),
            static_cast<unsigned long long>(
                backend.native_reject_invalidated_state),
            static_cast<unsigned long long>(backend.native_reject_other_state));
        ImGui::Text("Native rejects: mmio %llu  unaligned %llu",
            static_cast<unsigned long long>(backend.native_reject_mmio),
            static_cast<unsigned long long>(backend.native_reject_unaligned));
        ImGui::Text("Block entries: decoded %llu  native %llu  avg %.2f / %.2f instr",
            static_cast<unsigned long long>(backend.decoded_block_entries),
            static_cast<unsigned long long>(backend.native_block_entries),
            backend.decoded_block_entries == 0
                ? 0.0
                : static_cast<double>(backend.decoded_instructions) /
                      static_cast<double>(backend.decoded_block_entries),
            backend.native_block_entries == 0
                ? 0.0
                : static_cast<double>(backend.native_instructions) /
                      static_cast<double>(backend.native_block_entries));
        ImGui::Text("Memory helpers: %llu  MMIO: %llu  Exceptions: %llu",
            static_cast<unsigned long long>(backend.memory_helper_calls),
            static_cast<unsigned long long>(backend.mmio_accesses),
            static_cast<unsigned long long>(backend.exceptions));
        ImGui::Text("Native memory helpers: %llu  fast L/S: %llu / %llu  exception exits: %llu",
            static_cast<unsigned long long>(
                backend.native_memory_helper_calls),
            static_cast<unsigned long long>(
                backend.native_memory_fastpath_loads),
            static_cast<unsigned long long>(
                backend.native_memory_fastpath_stores),
            static_cast<unsigned long long>(
                backend.native_memory_exception_exits));
        ImGui::Text("Helper load-delay: entries %llu  passes %llu  fallbacks %llu",
            static_cast<unsigned long long>(
                backend.native_helper_load_delay_entries),
            static_cast<unsigned long long>(
                backend.native_helper_load_delay_passes),
            static_cast<unsigned long long>(
                backend.native_helper_load_delay_fallbacks));

        ImGui::Separator();
        ImGui::Text("CPU Hot Blocks:");
        if (!g_profile_detailed_timing) {
            ImGui::TextDisabled(
                "Open the full profiler with F12 to collect hot-block data.");
        }
        else if (backend.hot_block_count == 0) {
            ImGui::TextDisabled("No decoded block executions collected yet.");
        }
        else {
            const u64 shown_weight = [&]() {
                u64 total = 0;
                for (u32 i = 0; i < backend.hot_block_count; ++i) {
                    total += backend.hot_blocks[i].estimated_guest_instructions;
                }
                return total;
            }();
            const double shown_share =
                backend.hot_block_total_weight == 0
                    ? 0.0
                    : 100.0 * static_cast<double>(shown_weight) /
                          static_cast<double>(backend.hot_block_total_weight);
            ImGui::Text(
                "This frame: top %u of %u blocks cover %.1f%% of estimated guest instructions",
                backend.hot_block_count, backend.hot_block_total_count,
                shown_share);

            if (ImGui::BeginTable(
                    "cpu_hot_blocks", 8,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                        ImGuiTableFlags_ScrollX | ImGuiTableFlags_SizingFixedFit,
                    ImVec2(-1.0f, 260.0f))) {
                ImGui::TableSetupColumn("PC");
                ImGui::TableSetupColumn("Est instr");
                ImGui::TableSetupColumn("Entries");
                ImGui::TableSetupColumn("Native %");
                ImGui::TableSetupColumn("Shape");
                ImGui::TableSetupColumn("Reject/detail");
                ImGui::TableSetupColumn("Runtime reject");
                ImGui::TableSetupColumn("Ops");
                ImGui::TableHeadersRow();

                for (u32 i = 0; i < backend.hot_block_count; ++i) {
                    const CpuHotBlockStats& hot = backend.hot_blocks[i];
                    const double native_percent =
                        hot.entries == 0
                            ? 0.0
                            : 100.0 * static_cast<double>(hot.native_entries) /
                                  static_cast<double>(hot.entries);
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("0x%08X", hot.start_pc);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%llu",
                        static_cast<unsigned long long>(
                            hot.estimated_guest_instructions));
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%llu",
                        static_cast<unsigned long long>(hot.entries));
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.1f%%", native_percent);
                    ImGui::TableSetColumnIndex(4);
                    ImGui::TextUnformatted(hot.shape.data());
                    ImGui::TableSetColumnIndex(5);
                    ImGui::TextUnformatted(hot.reject_detail.data());
                    ImGui::TableSetColumnIndex(6);
                    if (hot.runtime_rejects == 0) {
                        ImGui::TextDisabled("-");
                    } else {
                        ImGui::Text("%llu %s (%u), %s (%u), mem=%s (%u)",
                            static_cast<unsigned long long>(
                                hot.runtime_rejects),
                            hot.runtime_reject_detail.data(),
                            hot.runtime_reject_dominant_count,
                            hot.runtime_reject_secondary_detail.data(),
                            hot.runtime_reject_secondary_count,
                            hot.runtime_memory_region.data(),
                            hot.runtime_memory_region_count);
                    }
                    ImGui::TableSetColumnIndex(7);
                    ImGui::TextUnformatted(hot.ops.data());
                }
                ImGui::EndTable();
            }

            if (ImGui::Button("Copy CPU hot-block snapshot")) {
                const EmuRunner::RuntimeSnapshot fresh_snapshot =
                    emu_runner_.is_running()
                        ? emu_runner_.runtime_snapshot()
                        : runtime_snapshot_;
                const CpuBackendStats& fresh_backend =
                    fresh_snapshot.cpu_backend_stats;

                u64 fresh_shown_weight = 0;
                for (u32 i = 0; i < fresh_backend.hot_block_count; ++i) {
                    fresh_shown_weight +=
                        fresh_backend.hot_blocks[i].estimated_guest_instructions;
                }
                const double fresh_shown_share =
                    fresh_backend.hot_block_total_weight == 0
                        ? 0.0
                        : 100.0 * static_cast<double>(fresh_shown_weight) /
                              static_cast<double>(
                                  fresh_backend.hot_block_total_weight);

                std::string snapshot;
                snapshot.reserve(8192);
                char line[768];
                std::snprintf(
                    line, sizeof(line),
                    "frame=%llu core_ms=%.3f cpu_ms=%.3f "
                    "gpu_ms=%.3f draws=%u decoded=%llu native=%llu fallback=%llu\n"
                    "hot_blocks=%u/%u shown_share=%.1f%%\n",
                    static_cast<unsigned long long>(fresh_snapshot.frame_id),
                    fresh_snapshot.core_frame_ms,
                    fresh_snapshot.profiling.cpu_ms,
                    fresh_snapshot.profiling.gpu_ms,
                    fresh_snapshot.profiling.gpu_draw_commands,
                    static_cast<unsigned long long>(
                        fresh_backend.decoded_instructions),
                    static_cast<unsigned long long>(
                        fresh_backend.native_instructions),
                    static_cast<unsigned long long>(
                        fresh_backend.fallback_instructions),
                    fresh_backend.hot_block_count,
                    fresh_backend.hot_block_total_count,
                    fresh_shown_share);
                snapshot += line;

                for (u32 i = 0; i < fresh_backend.hot_block_count; ++i) {
                    const CpuHotBlockStats& hot = fresh_backend.hot_blocks[i];
                    const double native_percent =
                        hot.entries == 0
                            ? 0.0
                            : 100.0 * static_cast<double>(hot.native_entries) /
                                  static_cast<double>(hot.entries);
                    std::snprintf(
                        line, sizeof(line),
                        "#%02u pc=%08X weight=%llu entries=%llu instr=%u "
                        "native_entries=%llu native=%.1f%% compiled=%u "
                        "runtime_rejects=%llu runtime_reason=%s "
                        "runtime_reason_count=%u runtime_reason2=%s "
                        "runtime_reason2_count=%u runtime_mem_region=%s "
                        "runtime_mem_region_count=%u "
                        "decoded_only=%u prefix=%u branch=%u mem=%u load=%u "
                        "store=%u fallback=%u shape=%s reject=%s ops=%s\n",
                        i + 1u, hot.start_pc,
                        static_cast<unsigned long long>(
                            hot.estimated_guest_instructions),
                        static_cast<unsigned long long>(hot.entries),
                        hot.instruction_count,
                        static_cast<unsigned long long>(hot.native_entries),
                        native_percent, hot.native_compiled ? 1u : 0u,
                        static_cast<unsigned long long>(hot.runtime_rejects),
                        hot.runtime_reject_detail.data(),
                        hot.runtime_reject_dominant_count,
                        hot.runtime_reject_secondary_detail.data(),
                        hot.runtime_reject_secondary_count,
                        hot.runtime_memory_region.data(),
                        hot.runtime_memory_region_count,
                        hot.native_decoded_only ? 1u : 0u,
                        hot.native_prefix_instruction_count,
                        hot.has_control_flow ? 1u : 0u,
                        hot.has_memory ? 1u : 0u,
                        hot.has_load ? 1u : 0u,
                        hot.has_store ? 1u : 0u,
                        hot.has_fallback ? 1u : 0u,
                        hot.shape.data(), hot.reject_detail.data(),
                        hot.ops.data());
                    snapshot += line;
                }
                ImGui::SetClipboardText(snapshot.c_str());
            }
        }

        if (config_vsync_ && swap_ms_ > 8.0) {
            ImGui::TextDisabled(
                "Swap includes VSync/compositor wait. Disable VSync to profile CPU cost.");
        }

        float budget_ms = 1000.0f / 60.0f;
        float usage = static_cast<float>(runtime_snapshot_.core_frame_ms / budget_ms);
        ImGui::Spacing();
        ImGui::Text("Core Frame Budget Usage (%.1f%%):", usage * 100.0f);
        ImGui::ProgressBar(usage, ImVec2(-1.0f, 0.0f));
    }
    ImGui::End();
    if (was_open && !show_perf_profiler_) {
        g_profile_detailed_timing = false;
    }
}
