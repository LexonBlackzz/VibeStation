#include "ui/app.h"
#include "ui/panels/cpu_backend_panel.h"

#include <imgui.h>

void App::panel_debug_cpu() {
    ImGui::SetNextWindowSize(ImVec2(450, 500), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("CPU Debug", &show_debug_cpu_)) {
        const bool running = emu_runner_.is_running();
        const CpuBackendStats backend_stats =
            running ? runtime_snapshot_.cpu_backend_stats
                    : system_->cpu().cpu_backend_stats();
        const CpuExecutionMode requested_backend =
            running ? runtime_snapshot_.cpu_backend
                    : effective_cpu_execution_mode();
        draw_cpu_backend_mode_summary(backend_stats, requested_backend);
        ImGui::Text("Blocks: %u  Decoded entries: %llu  Native entries: %llu",
            backend_stats.block_count,
            static_cast<unsigned long long>(backend_stats.decoded_block_entries),
            static_cast<unsigned long long>(backend_stats.native_block_entries));
        ImGui::Text("Decoded/native/fallback instr: %llu / %llu / %llu",
            static_cast<unsigned long long>(backend_stats.decoded_instructions),
            static_cast<unsigned long long>(backend_stats.native_instructions),
            static_cast<unsigned long long>(backend_stats.fallback_instructions));
        if (backend_stats.forced_interpreter_instructions != 0 ||
            backend_stats.forced_interpreter_last_reason !=
                CpuForcedInterpreterReason::None) {
            ImGui::Text("Forced interpreter: %s  slices %llu  instr %llu",
                cpu_forced_interpreter_reason_name(
                    backend_stats.forced_interpreter_last_reason),
                static_cast<unsigned long long>(
                    backend_stats.forced_interpreter_slices),
                static_cast<unsigned long long>(
                    backend_stats.forced_interpreter_instructions));
        }
        ImGui::Text("Native: available %u  attempts %llu  successes %llu  compiled %llu",
            backend_stats.native_available ? 1u : 0u,
            static_cast<unsigned long long>(
                backend_stats.native_compile_attempts),
            static_cast<unsigned long long>(
                backend_stats.native_compile_successes),
            static_cast<unsigned long long>(
                backend_stats.native_blocks_compiled));
        ImGui::Text("Native entries: %llu  cycles %llu  code %zu bytes",
            static_cast<unsigned long long>(
                backend_stats.native_block_entries),
            static_cast<unsigned long long>(backend_stats.native_cycles),
            backend_stats.native_code_bytes);
        ImGui::Text("Native fallback: rejected %llu  compile fail %llu  decoded %llu",
            static_cast<unsigned long long>(
                backend_stats.native_rejected_unsafe_blocks),
            static_cast<unsigned long long>(
                backend_stats.native_compile_failures),
            static_cast<unsigned long long>(
                backend_stats.native_to_decoded_fallbacks));
        ImGui::Text("Native gating: cold %llu  short %llu",
            static_cast<unsigned long long>(
                backend_stats.native_hot_threshold_skips),
            static_cast<unsigned long long>(
                backend_stats.native_short_block_skips));
        ImGui::Text("Native rejects: branch %llu  mem %llu  cop0 %llu  cop2 %llu",
            static_cast<unsigned long long>(backend_stats.native_reject_branch),
            static_cast<unsigned long long>(backend_stats.native_reject_memory),
            static_cast<unsigned long long>(backend_stats.native_reject_cop0),
            static_cast<unsigned long long>(backend_stats.native_reject_cop2));
        ImGui::Text("Native rejects: exc/unk %llu  state %llu  budget %llu  icache %llu",
            static_cast<unsigned long long>(
                backend_stats.native_reject_exception_unknown),
            static_cast<unsigned long long>(
                backend_stats.native_reject_unsafe_state),
            static_cast<unsigned long long>(backend_stats.native_reject_budget),
            static_cast<unsigned long long>(backend_stats.native_reject_icache));
        ImGui::Text("Native state rejects: pc %llu  branch-delay %llu  load-delay %llu",
            static_cast<unsigned long long>(
                backend_stats.native_reject_pc_state),
            static_cast<unsigned long long>(
                backend_stats.native_reject_branch_delay_state),
            static_cast<unsigned long long>(
                backend_stats.native_reject_load_delay_state));
        ImGui::Text("Native state rejects: irq %llu  invalidated %llu  other %llu",
            static_cast<unsigned long long>(
                backend_stats.native_reject_irq_state),
            static_cast<unsigned long long>(
                backend_stats.native_reject_invalidated_state),
            static_cast<unsigned long long>(
                backend_stats.native_reject_other_state));
        ImGui::Text("Native rejects: mmio %llu  unaligned %llu",
            static_cast<unsigned long long>(backend_stats.native_reject_mmio),
            static_cast<unsigned long long>(
                backend_stats.native_reject_unaligned));
        ImGui::Text("Native memory: helpers %llu  fast L/S %llu / %llu  exception exits %llu",
            static_cast<unsigned long long>(
                backend_stats.native_memory_helper_calls),
            static_cast<unsigned long long>(
                backend_stats.native_memory_fastpath_loads),
            static_cast<unsigned long long>(
                backend_stats.native_memory_fastpath_stores),
            static_cast<unsigned long long>(
                backend_stats.native_memory_exception_exits));
        ImGui::Text("Helper load-delay: entries %llu  passes %llu  fallbacks %llu",
            static_cast<unsigned long long>(
                backend_stats.native_helper_load_delay_entries),
            static_cast<unsigned long long>(
                backend_stats.native_helper_load_delay_passes),
            static_cast<unsigned long long>(
                backend_stats.native_helper_load_delay_fallbacks));
        ImGui::Text("Invalidation: queries %llu  no-code %llu  examined %llu  hit %llu",
            static_cast<unsigned long long>(backend_stats.invalidation_queries),
            static_cast<unsigned long long>(
                backend_stats.invalidation_fast_no_code_page_exits),
            static_cast<unsigned long long>(
                backend_stats.invalidation_blocks_examined),
            static_cast<unsigned long long>(
                backend_stats.invalidation_blocks_invalidated));
        ImGui::Separator();
        if (running) {
            ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.4f, 1.0f),
                "Pause emulation to inspect registers.");
        }
        else {
            ImGui::Text("PC: 0x%08X", system_->cpu().pc());
            ImGui::Text("Cycles: %llu",
                (unsigned long long)system_->cpu().cycle_count());
            ImGui::Separator();

            if (ImGui::BeginTable("Registers", 4,
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                const char* reg_names[] = {
                    "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0", "t1", "t2",
                    "t3",   "t4", "t5", "t6", "t7", "s0", "s1", "s2", "s3", "s4", "s5",
                    "s6",   "s7", "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra" };
                for (int i = 0; i < 32; i++) {
                    ImGui::TableNextColumn();
                    ImGui::TextColored(ImVec4(0.6f, 0.5f, 0.9f, 1.0f), "$%s",
                        reg_names[i]);
                    ImGui::SameLine(55);
                    ImGui::Text("0x%08X", system_->cpu().reg(i));
                }
                ImGui::EndTable();
            }
        }

        ImGui::Spacing();
        if (ImGui::Button("Step") && !running) {
            system_->step();
        }
        ImGui::SameLine();
        if (ImGui::Button(running ? "Pause" : "Run")) {
            if (running) {
                emu_runner_.pause_and_wait_idle();
            }
            else {
                apply_memory_card_settings(false);
                has_started_emulation_ = true;
                emu_runner_.set_running(true);
            }
        }
    }
    ImGui::End();
}
