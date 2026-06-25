#include "ui/app.h"
#include "core/types.h"
#include <imgui.h>
#include "ui/panels/cpu_backend_panel.h"

void App::draw_logging_panel() {
                bool logging_config_dirty = false;
                const char* levels[] = { "Debug", "Info", "Warn", "Error" };
                int level_index = 1;
                switch (g_log_level) {
                case LogLevel::Debug:
                    level_index = 0;
                    break;
                case LogLevel::Info:
                    level_index = 1;
                    break;
                case LogLevel::Warn:
                    level_index = 2;
                    break;
                case LogLevel::Error:
                    level_index = 3;
                    break;
                }
                if (ImGui::Combo("Level", &level_index, levels, IM_ARRAYSIZE(levels))) {
                    g_log_level = static_cast<LogLevel>(level_index);
                    logging_config_dirty = true;
                }

                if (ImGui::Checkbox("Timestamps", &g_log_timestamp)) {
                    logging_config_dirty = true;
                }
                if (ImGui::Checkbox("Collapse Repeats", &g_log_dedupe)) {
                    logging_config_dirty = true;
                }
                int dedupe_flush = static_cast<int>(g_log_dedupe_flush);
                if (ImGui::InputInt("Repeat Flush", &dedupe_flush)) {
                    g_log_dedupe_flush = static_cast<u32>(std::max(1, dedupe_flush));
                    logging_config_dirty = true;
                }
                ImGui::Separator();
                ImGui::Text("Categories");

                auto category_checkbox = [&](const char* label, LogCategory cat) {
                    bool enabled = log_category_enabled(cat);
                    if (ImGui::Checkbox(label, &enabled)) {
                        if (enabled) {
                            g_log_category_mask |= log_category_bit(cat);
                        }
                        else {
                            g_log_category_mask &= ~log_category_bit(cat);
                        }
                        logging_config_dirty = true;
                    }
                    };
                category_checkbox("General", LogCategory::General);
                ImGui::SameLine();
                category_checkbox("App", LogCategory::App);
                ImGui::SameLine();
                category_checkbox("BIOS", LogCategory::Bios);
                category_checkbox("CPU", LogCategory::Cpu);
                ImGui::SameLine();
                category_checkbox("BUS", LogCategory::Bus);
                ImGui::SameLine();
                category_checkbox("RAM", LogCategory::Ram);
                ImGui::SameLine();
                category_checkbox("IRQ", LogCategory::Irq);
                category_checkbox("DMA", LogCategory::Dma);
                ImGui::SameLine();
                category_checkbox("CDROM", LogCategory::Cdrom);
                ImGui::SameLine();
                category_checkbox("GPU", LogCategory::Gpu);
                category_checkbox("SPU", LogCategory::Spu);
                ImGui::SameLine();
                category_checkbox("SIO", LogCategory::Sio);
                ImGui::SameLine();
                category_checkbox("Timer", LogCategory::Timer);
                category_checkbox("Input", LogCategory::Input);

                if (ImGui::Button("Enable All Categories")) {
                    g_log_category_mask = 0xFFFFFFFFu;
                    logging_config_dirty = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("Disable All Categories")) {
                    g_log_category_mask = 0;
                    logging_config_dirty = true;
                }

                ImGui::Separator();
                ImGui::Text("Trace Channels");
                if (ImGui::Checkbox("DMA Trace", &g_trace_dma)) {
                    logging_config_dirty = true;
                }
                ImGui::SameLine();
                if (ImGui::Checkbox("CDROM Trace", &g_trace_cdrom)) {
                    logging_config_dirty = true;
                }
                if (ImGui::Checkbox("CPU Trace", &g_trace_cpu)) {
                    logging_config_dirty = true;
                }
                ImGui::SameLine();
                if (ImGui::Checkbox("BUS Trace", &g_trace_bus)) {
                    logging_config_dirty = true;
                }
                ImGui::SameLine();
                if (ImGui::Checkbox("RAM Trace", &g_trace_ram)) {
                    logging_config_dirty = true;
                }
                const ImVec4 trace_warning_color =
                    g_trace_cpu ? ImVec4(1.0f, 0.72f, 0.28f, 1.0f)
                                : ImVec4(0.62f, 0.62f, 0.62f, 1.0f);
                ImGui::TextColored(trace_warning_color,
                    "CPU trace disables decoded/JIT execution and forces interpreter.");
                ImGui::TextDisabled(
                    "For benchmarking decoded/JIT, CPU trace must be off.");
                if (ImGui::Checkbox("GPU Trace", &g_trace_gpu)) {
                    logging_config_dirty = true;
                }
                ImGui::SameLine();
                if (ImGui::Checkbox("SPU Trace", &g_trace_spu)) {
                    logging_config_dirty = true;
                }
                if (ImGui::Checkbox("IRQ Trace", &g_trace_irq)) {
                    logging_config_dirty = true;
                }
                ImGui::SameLine();
                if (ImGui::Checkbox("Timer Trace", &g_trace_timer)) {
                    logging_config_dirty = true;
                }
                if (ImGui::Checkbox("SIO Trace", &g_trace_sio)) {
                    logging_config_dirty = true;
                }
                if (ImGui::Checkbox("CPU Deep Diagnostics (slow)",
                        &g_cpu_deep_diagnostics)) {
                    logging_config_dirty = true;
                }
                if (ImGui::Checkbox("Enable FMV Diagnostics",
                        &g_log_fmv_diagnostics)) {
                    if (!g_log_fmv_diagnostics) {
                        show_fmv_diagnostics_ = false;
                        g_mdec_debug_compare_macroblocks = false;
                        g_mdec_debug_upload_probe = false;
                        if (system_) {
                            system_->reset_mdec_debug_compare();
                            system_->reset_mdec_upload_probe();
                        }
                    }
                    logging_config_dirty = true;
                }
                if (g_log_fmv_diagnostics) {
                    if (ImGui::Button("FMV Diagnostics")) {
                        show_fmv_diagnostics_ = true;
                    }
                }

                if (ImGui::Button("Enable All Traces")) {
                    g_trace_dma = true;
                    g_trace_cdrom = true;
                    g_trace_cpu = true;
                    g_trace_bus = true;
                    g_trace_ram = true;
                    g_trace_gpu = true;
                    g_trace_spu = true;
                    g_trace_irq = true;
                    g_trace_timer = true;
                    g_trace_sio = true;
                    logging_config_dirty = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("Disable All Traces")) {
                    g_trace_dma = false;
                    g_trace_cdrom = false;
                    g_trace_cpu = false;
                    g_trace_bus = false;
                    g_trace_ram = false;
                    g_trace_gpu = false;
                    g_trace_spu = false;
                    g_trace_irq = false;
                    g_trace_timer = false;
                    g_trace_sio = false;
                    logging_config_dirty = true;
                }

                if (g_log_fmv_diagnostics) {
                    ImGui::TextDisabled("Open the FMV Diagnostics window for MDEC/GPU analysis tools.");
                }

                ImGui::Separator();
                ImGui::Text("Trace Sampling (burst/stride)");
                auto sample_pair = [&](const char* label, u32& burst, u32& stride) {
                    int b = static_cast<int>(burst);
                    int s = static_cast<int>(stride);
                    std::string burst_label = std::string(label) + " Burst";
                    std::string stride_label = std::string(label) + " Stride";
                    if (ImGui::InputInt(burst_label.c_str(), &b)) {
                        burst = static_cast<u32>(std::max(1, b));
                        logging_config_dirty = true;
                    }
                    if (ImGui::InputInt(stride_label.c_str(), &s)) {
                        stride = static_cast<u32>(std::max(1, s));
                        logging_config_dirty = true;
                    }
                    };
                sample_pair("CPU", g_trace_burst_cpu, g_trace_stride_cpu);
                sample_pair("BUS", g_trace_burst_bus, g_trace_stride_bus);
                sample_pair("RAM", g_trace_burst_ram, g_trace_stride_ram);
                sample_pair("SPU", g_trace_burst_spu, g_trace_stride_spu);
                sample_pair("GPU", g_trace_burst_gpu, g_trace_stride_gpu);
                sample_pair("DMA", g_trace_burst_dma, g_trace_stride_dma);
                sample_pair("CDROM", g_trace_burst_cdrom, g_trace_stride_cdrom);
                sample_pair("IRQ", g_trace_burst_irq, g_trace_stride_irq);
                sample_pair("Timer", g_trace_burst_timer, g_trace_stride_timer);
                sample_pair("SIO", g_trace_burst_sio, g_trace_stride_sio);
                if (ImGui::Button("Reset Trace Sampling Defaults")) {
                    g_trace_burst_cpu = 128;
                    g_trace_stride_cpu = 32768;
                    g_trace_burst_bus = 256;
                    g_trace_stride_bus = 16384;
                    g_trace_burst_ram = 32;
                    g_trace_stride_ram = 131072;
                    g_trace_burst_dma = 64;
                    g_trace_stride_dma = 2048;
                    g_trace_burst_cdrom = 128;
                    g_trace_stride_cdrom = 256;
                    g_trace_burst_gpu = 512;
                    g_trace_stride_gpu = 2048;
                    g_trace_burst_spu = 128;
                    g_trace_stride_spu = 4096;
                    g_trace_burst_irq = 128;
                    g_trace_stride_irq = 2048;
                    g_trace_burst_timer = 64;
                    g_trace_stride_timer = 2048;
                    g_trace_burst_sio = 64;
                    g_trace_stride_sio = 2048;
                    logging_config_dirty = true;
                }

                ImGui::Separator();
                if (ImGui::InputText("Log File", log_path_, IM_ARRAYSIZE(log_path_))) {
                    logging_config_dirty = true;
                }
                if (g_log_file == nullptr) {
                    if (ImGui::Button("Start File Logging")) {
                        g_log_file = std::fopen(log_path_, "w");
                        status_message_ =
                            (g_log_file != nullptr) ? "File logging enabled" : "Failed to open log file";
                        logging_config_dirty = true;
                    }
                }
                else {
                    if (ImGui::Button("Stop File Logging")) {
                        log_flush_repeats();
                        std::fclose(g_log_file);
                        g_log_file = nullptr;
                        status_message_ = "File logging disabled";
                        logging_config_dirty = true;
                    }
                }

                ImGui::Separator();
                if (ImGui::CollapsingHeader("CPU Backend Stats",
                    ImGuiTreeNodeFlags_DefaultOpen)) {
                    CpuBackendStats backend_stats{};
                    CpuExecutionMode backend_mode = effective_cpu_execution_mode();
                    if (emu_runner_.is_running()) {
                        backend_stats = runtime_snapshot_.cpu_backend_stats;
                        backend_mode = runtime_snapshot_.cpu_backend;
                    }
                    else if (system_) {
                        backend_stats = system_->cpu().cpu_backend_stats();
                    }
                    if (draw_cpu_backend_logging_section(
                        backend_stats, backend_mode)) {
                        logging_config_dirty = true;
                    }
                }

                ImGui::Separator();
                ImGui::Text("Runtime Log");
                ImGui::Checkbox("Auto-scroll", &log_auto_scroll_);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(220.0f * ImGui::GetIO().FontGlobalScale);
                ImGui::InputTextWithHint("##LogSearch", "Filter logs", log_search_,
                    IM_ARRAYSIZE(log_search_));
                ImGui::SameLine();
                if (ImGui::Button("Clear Runtime Log")) {
                    log_clear_ui_entries();
                    selected_log_seq_ = 0;
                }

                std::vector<LogUiEntry> log_entries;
                {
                    std::lock_guard<std::mutex> lock(g_log_ui_mutex);
                    log_entries.assign(g_log_ui_entries.begin(), g_log_ui_entries.end());
                }

                std::vector<const LogUiEntry*> filtered_logs;
                filtered_logs.reserve(log_entries.size());
                std::string filter_text = log_search_;
                std::transform(filter_text.begin(), filter_text.end(), filter_text.begin(),
                    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                for (const LogUiEntry& entry : log_entries) {
                    if (!filter_text.empty()) {
                        std::string haystack = entry.timestamp + " " + entry.prefix + " " +
                            log_category_name(entry.category) + " " + entry.message;
                        std::transform(haystack.begin(), haystack.end(), haystack.begin(),
                            [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                        if (haystack.find(filter_text) == std::string::npos) {
                            continue;
                        }
                    }
                    filtered_logs.push_back(&entry);
                }

                if (selected_log_seq_ == 0 && !filtered_logs.empty()) {
                    selected_log_seq_ = filtered_logs.back()->seq;
                }

                const LogUiEntry* selected_entry = nullptr;
                for (const LogUiEntry* entry : filtered_logs) {
                    if (entry->seq == selected_log_seq_) {
                        selected_entry = entry;
                        break;
                    }
                }
                if (selected_entry == nullptr && !filtered_logs.empty()) {
                    selected_entry = filtered_logs.back();
                    selected_log_seq_ = selected_entry->seq;
                }

                auto level_color = [](LogLevel level) {
                    switch (level) {
                    case LogLevel::Debug:
                        return ImVec4(0.55f, 0.55f, 0.55f, 1.0f);
                    case LogLevel::Info:
                        return ImVec4(0.80f, 0.80f, 0.80f, 1.0f);
                    case LogLevel::Warn:
                        return ImVec4(0.90f, 0.72f, 0.30f, 1.0f);
                    case LogLevel::Error:
                        return ImVec4(0.92f, 0.38f, 0.32f, 1.0f);
                    default:
                        return ImVec4(0.80f, 0.80f, 0.80f, 1.0f);
                    }
                };
                auto level_label = [](LogLevel level) {
                    switch (level) {
                    case LogLevel::Debug:
                        return "DBG";
                    case LogLevel::Info:
                        return "INF";
                    case LogLevel::Warn:
                        return "WRN";
                    case LogLevel::Error:
                        return "ERR";
                    default:
                        return "UNK";
                    }
                };

                const float log_region_height = 260.0f * ImGui::GetIO().FontGlobalScale;
                const float detail_width = 320.0f * ImGui::GetIO().FontGlobalScale;
                const float list_width =
                    std::max(120.0f, ImGui::GetContentRegionAvail().x - detail_width - ImGui::GetStyle().ItemSpacing.x);

                ImGui::BeginChild("RuntimeLogList", ImVec2(list_width, log_region_height), true,
                    ImGuiWindowFlags_HorizontalScrollbar);
                ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 2.0f));
                ImGuiListClipper clipper;
                clipper.Begin(static_cast<int>(filtered_logs.size()));
                while (clipper.Step()) {
                    for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                        const LogUiEntry& entry = *filtered_logs[static_cast<size_t>(i)];
                        std::string line;
                        if (!entry.timestamp.empty()) {
                            line += entry.timestamp;
                            line += " ";
                        }
                        line += "[";
                        line += level_label(entry.level);
                        line += "] ";
                        line += log_category_name(entry.category);
                        line += "  ";
                        line += entry.message;
                        const bool selected = (entry.seq == selected_log_seq_);
                        if (ImGui::Selectable((line + "##log_" + std::to_string(entry.seq)).c_str(),
                            selected)) {
                            selected_log_seq_ = entry.seq;
                            selected_entry = &entry;
                        }
                        if (selected) {
                            ImGui::SetItemDefaultFocus();
                        }
                        if (ImGui::IsItemVisible()) {
                            const ImVec2 min = ImGui::GetItemRectMin();
                            ImGui::GetWindowDrawList()->AddRectFilled(
                                ImVec2(min.x + 1.0f, min.y + 3.0f),
                                ImVec2(min.x + 4.0f, min.y + ImGui::GetTextLineHeight() + 3.0f),
                                ImGui::ColorConvertFloat4ToU32(level_color(entry.level)));
                        }
                    }
                }
                if (log_auto_scroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4.0f) {
                    ImGui::SetScrollHereY(1.0f);
                }
                ImGui::PopStyleVar();
                ImGui::EndChild();

                ImGui::SameLine();

                ImGui::BeginChild("RuntimeLogDetail", ImVec2(0.0f, log_region_height), true);
                if (selected_entry != nullptr) {
                    ImGui::TextColored(level_color(selected_entry->level), "%s",
                        level_label(selected_entry->level));
                    ImGui::SameLine();
                    ImGui::Text("%s", log_category_name(selected_entry->category));
                    if (!selected_entry->timestamp.empty()) {
                        ImGui::SameLine();
                        ImGui::TextDisabled("%s", selected_entry->timestamp.c_str());
                    }
                    ImGui::Separator();
                    ImGui::TextWrapped("%s", selected_entry->message.c_str());
                } else {
                    ImGui::TextDisabled("No log entry selected.");
                }
                ImGui::EndChild();

                if (ImGui::Button("Boot Debug Preset")) {
                    g_log_level = LogLevel::Debug;
                    g_log_category_mask = 0xFFFFFFFFu;
                    g_trace_dma = true;
                    g_trace_cdrom = true;
                    g_trace_cpu = true;
                    g_trace_bus = true;
                    g_trace_ram = true;
                    g_trace_gpu = true;
                    g_trace_spu = true;
                    g_trace_irq = true;
                    g_trace_timer = true;
                    g_trace_sio = true;
                    g_trace_burst_cpu = 128;
                    g_trace_stride_cpu = 32768;
                    g_trace_burst_bus = 256;
                    g_trace_stride_bus = 16384;
                    g_trace_burst_ram = 32;
                    g_trace_stride_ram = 131072;
                    g_trace_burst_dma = 64;
                    g_trace_stride_dma = 2048;
                    g_trace_burst_cdrom = 128;
                    g_trace_stride_cdrom = 256;
                    g_trace_burst_gpu = 512;
                    g_trace_stride_gpu = 2048;
                    g_trace_burst_spu = 128;
                    g_trace_stride_spu = 4096;
                    g_trace_burst_irq = 128;
                    g_trace_stride_irq = 2048;
                    g_trace_burst_timer = 64;
                    g_trace_stride_timer = 2048;
                    g_trace_burst_sio = 64;
                    g_trace_stride_sio = 2048;
                    status_message_ = "Logging preset applied";
                    logging_config_dirty = true;
                }

                if (logging_config_dirty) {
                    save_persistent_config();
                }
            }