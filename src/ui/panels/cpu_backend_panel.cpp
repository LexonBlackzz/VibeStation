#include "ui/panels/cpu_backend_panel.h"
#include "core/cpu.h"
#include "core/types.h"
#include <imgui.h>
#include <algorithm>
#include <array>

namespace {
    double percent_of(u64 value, u64 total) {
        if (total == 0) {
            return 0.0;
        }
        return (static_cast<double>(value) * 100.0) /
            static_cast<double>(total);
    }
}

    void draw_cpu_backend_instruction_mix(const CpuBackendStats& stats) {
        const u64 total = stats.decoded_instructions +
            stats.native_instructions + stats.fallback_instructions;
        ImGui::Text("Instruction mix: native %.1f%%  decoded %.1f%%  fallback %.1f%%",
            percent_of(stats.native_instructions, total),
            percent_of(stats.decoded_instructions, total),
            percent_of(stats.fallback_instructions, total));
        ImGui::Text("Instructions: native %llu  decoded %llu  fallback %llu",
            static_cast<unsigned long long>(stats.native_instructions),
            static_cast<unsigned long long>(stats.decoded_instructions),
            static_cast<unsigned long long>(stats.fallback_instructions));
    }

    void draw_cpu_backend_mode_summary(
        const CpuBackendStats& stats, CpuExecutionMode requested_mode) {
        const CpuForcedInterpreterReason forced_reason =
            stats.forced_interpreter_last_reason;
        const bool forced =
            forced_reason != CpuForcedInterpreterReason::None;
        ImGui::Text("Requested backend: %s",
            cpu_execution_mode_name(requested_mode));
        if (forced) {
            ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.28f, 1.0f),
                "Effective backend: Interpreter (forced by %s)",
                cpu_forced_interpreter_reason_name(forced_reason));
            ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.28f, 1.0f),
                "Forced reason: %s",
                cpu_forced_interpreter_reason_name(forced_reason));
        }
        else {
            ImGui::Text("Effective backend: %s",
                cpu_execution_mode_name(requested_mode));
        }
        ImGui::TextDisabled("active %u  native %u",
            stats.active ? 1u : 0u, stats.native_available ? 1u : 0u);
    }

    bool draw_cpu_backend_logging_section(
        const CpuBackendStats& stats, CpuExecutionMode mode) {
        bool dirty = false;
        draw_cpu_backend_mode_summary(stats, mode);

        int hot_threshold =
            static_cast<int>(g_cpu_x64_jit_hot_block_threshold);
        if (ImGui::InputInt("x64 Hot Threshold", &hot_threshold)) {
            g_cpu_x64_jit_hot_block_threshold =
                static_cast<u32>(std::max(0, hot_threshold));
            dirty = true;
        }
        int min_block = static_cast<int>(g_cpu_x64_jit_min_block_instructions);
        if (ImGui::InputInt("x64 Min Block Instructions", &min_block)) {
            g_cpu_x64_jit_min_block_instructions =
                static_cast<u32>(std::max(1, min_block));
            dirty = true;
        }
        ImGui::Checkbox("Force x64 Compile", &g_cpu_x64_jit_force_compile);
        if (ImGui::Checkbox("Enable All Native x64",
                &g_cpu_x64_jit_all_native_enabled)) {
            dirty = true;
        }
        if (ImGui::Checkbox("Enable Native Memory Blocks (Experimental)",
                            &g_cpu_x64_jit_native_memory_enabled)) {
            dirty = true;
        }
        if (ImGui::Checkbox("Enable Native ALU Blocks",
                &g_cpu_x64_jit_native_alu_enabled)) {
            dirty = true;
        }
        if (g_cpu_x64_jit_all_native_cli_override ||
            g_cpu_x64_jit_native_memory_cli_override ||
            g_cpu_x64_jit_native_alu_cli_override) {
            ImGui::TextDisabled("CLI native tiers: all %s  memory %s  ALU %s",
                cpu_x64_jit_all_native_enabled() ? "on" : "off",
                cpu_x64_jit_native_memory_enabled() ? "on" : "off",
                cpu_x64_jit_native_alu_enabled() ? "on" : "off");
        }
        if (ImGui::Checkbox("Enable Native Branch Tails (experimental)",
                &g_cpu_x64_jit_branch_tail_enabled)) {
            dirty = true;
        }
        if (g_cpu_x64_jit_branch_tail_cli_override) {
            ImGui::TextDisabled("CLI override: branch tails %s",
                cpu_x64_jit_branch_tail_enabled() ? "enabled" : "disabled");
        }
        if (ImGui::Checkbox("Log Recent Native Branch Tails",
                &g_cpu_x64_jit_branch_tail_logging)) {
            dirty = true;
        }
        int branch_tail_log_count =
            static_cast<int>(g_cpu_x64_jit_branch_tail_log_count);
        if (ImGui::InputInt("Branch Tail Log Count", &branch_tail_log_count)) {
            g_cpu_x64_jit_branch_tail_log_count =
                static_cast<u32>(std::max(1, branch_tail_log_count));
            dirty = true;
        }
        if (ImGui::Checkbox("Log Hot Native Rejects",
                &g_cpu_backend_rejected_block_logging)) {
            dirty = true;
        }
        if (ImGui::Checkbox("Enable RAM load fastpath (experimental)",
                &g_cpu_x64_jit_ram_load_fastpath_enabled)) {
            dirty = true;
        }
        if (g_cpu_x64_jit_ram_load_fastpath_cli_override) {
            ImGui::TextDisabled("CLI override: RAM load fastpath %s",
                g_cpu_x64_jit_ram_load_fastpath_enabled ? "enabled" : "disabled");
        }
        int rejected_block_log_count =
            static_cast<int>(g_cpu_backend_rejected_block_log_count);
        if (ImGui::InputInt("Hot Reject Log Count",
                &rejected_block_log_count)) {
            g_cpu_backend_rejected_block_log_count =
                static_cast<u32>(std::max(1, rejected_block_log_count));
            dirty = true;
        }

        ImGui::Separator();
        draw_cpu_backend_instruction_mix(stats);
        const bool forced_before_lookup =
            stats.forced_interpreter_instructions != 0 &&
            stats.decoded_block_entries == 0 && stats.native_block_entries == 0 &&
            stats.cache_hits == 0 && stats.cache_misses == 0;
        if (stats.forced_interpreter_last_reason !=
                CpuForcedInterpreterReason::None ||
            forced_before_lookup) {
            ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.28f, 1.0f),
                "Forced interpreter reason: %s",
                cpu_forced_interpreter_reason_name(
                    stats.forced_interpreter_last_reason));
        }
        ImGui::Text("Forced interpreter: slices %llu  instructions %llu",
            static_cast<unsigned long long>(stats.forced_interpreter_slices),
            static_cast<unsigned long long>(
                stats.forced_interpreter_instructions));
        ImGui::Text("Forced reasons: trace %llu  deep %llu  fmv %llu",
            static_cast<unsigned long long>(stats.forced_interpreter_trace),
            static_cast<unsigned long long>(
                stats.forced_interpreter_deep_diagnostics),
            static_cast<unsigned long long>(stats.forced_interpreter_fmv));
        if (stats.forced_interpreter_backend_compare != 0 ||
            stats.forced_interpreter_other != 0) {
            ImGui::Text("Forced reasons: backend/test %llu  other %llu",
                static_cast<unsigned long long>(
                    stats.forced_interpreter_backend_compare),
                static_cast<unsigned long long>(
                    stats.forced_interpreter_other));
        }
        const double avg_decoded =
            stats.decoded_block_entries == 0
                ? 0.0
                : static_cast<double>(stats.decoded_instructions) /
                      static_cast<double>(stats.decoded_block_entries);
        const double avg_native =
            stats.native_block_entries == 0
                ? 0.0
                : static_cast<double>(stats.native_instructions) /
                      static_cast<double>(stats.native_block_entries);
        ImGui::Text("Block entries: decoded %llu  native %llu",
            static_cast<unsigned long long>(stats.decoded_block_entries),
            static_cast<unsigned long long>(stats.native_block_entries));
        ImGui::Text("Average block length: decoded %.2f  native %.2f",
            avg_decoded, avg_native);
        ImGui::Text("Blocks: cached %u  decoded %llu  native %llu  interpreter-only %u",
            stats.block_count,
            static_cast<unsigned long long>(stats.decoded_blocks),
            static_cast<unsigned long long>(stats.native_blocks),
            stats.interpreter_only_blocks);

        ImGui::Separator();
        ImGui::Text("Native compile: attempts %llu  successes %llu  failures %llu",
            static_cast<unsigned long long>(stats.native_compile_attempts),
            static_cast<unsigned long long>(stats.native_compile_successes),
            static_cast<unsigned long long>(stats.native_compile_failures));
        ImGui::Text("Native runtime: entries %llu  cycles %llu  code %zu bytes",
            static_cast<unsigned long long>(stats.native_block_entries),
            static_cast<unsigned long long>(stats.native_cycles),
            stats.native_code_bytes);
        ImGui::Text("Native fallback: decoded %llu  unsafe rejects %llu",
            static_cast<unsigned long long>(stats.native_to_decoded_fallbacks),
            static_cast<unsigned long long>(stats.native_rejected_unsafe_blocks));
        ImGui::Text("Native gating: cold %llu  short %llu",
            static_cast<unsigned long long>(stats.native_hot_threshold_skips),
            static_cast<unsigned long long>(stats.native_short_block_skips));

        struct RejectRow {
            const char* label;
            u64 count;
        };
        std::array<RejectRow, 15> rejects = { {
            { "branch", stats.native_reject_branch },
            { "memory", stats.native_reject_memory },
            { "cop0", stats.native_reject_cop0 },
            { "cop2/gte", stats.native_reject_cop2 },
            { "exception/unknown", stats.native_reject_exception_unknown },
            { "pc state", stats.native_reject_pc_state },
            { "branch delay", stats.native_reject_branch_delay_state },
            { "load delay", stats.native_reject_load_delay_state },
            { "irq state", stats.native_reject_irq_state },
            { "invalidated", stats.native_reject_invalidated_state },
            { "other state", stats.native_reject_other_state },
            { "budget", stats.native_reject_budget },
            { "icache", stats.native_reject_icache },
            { "mmio", stats.native_reject_mmio },
            { "unaligned", stats.native_reject_unaligned },
        } };
        std::sort(rejects.begin(), rejects.end(),
            [](const RejectRow& a, const RejectRow& b) {
                return a.count > b.count;
            });
        if (rejects[0].count == 0) {
            ImGui::Text("Top rejections: none");
        }
        else {
            ImGui::Text("Top rejections: %s %llu  %s %llu  %s %llu",
                rejects[0].label,
                static_cast<unsigned long long>(rejects[0].count),
                rejects[1].label,
                static_cast<unsigned long long>(rejects[1].count),
                rejects[2].label,
                static_cast<unsigned long long>(rejects[2].count));
        }
        ImGui::Text("PC reject detail: pc %llu  next_pc %llu  delay-start %llu  stale %llu",
            static_cast<unsigned long long>(stats.native_reject_pc_mismatch),
            static_cast<unsigned long long>(
                stats.native_reject_next_pc_mismatch),
            static_cast<unsigned long long>(
                stats.native_reject_block_start_after_branch_delay),
            static_cast<unsigned long long>(
                stats.native_reject_stale_invalid_block_state));
        ImGui::Text("Delay reject detail: in %llu  pending %llu  taken %llu  pc %llu",
            static_cast<unsigned long long>(
                stats.native_reject_branch_delay_in_delay_slot),
            static_cast<unsigned long long>(
                stats.native_reject_branch_delay_pending_delay_slot),
            static_cast<unsigned long long>(
                stats.native_reject_branch_delay_pending_branch_taken),
            static_cast<unsigned long long>(
                stats.native_reject_branch_delay_pending_branch_pc));

        ImGui::Separator();
        ImGui::Text("Invalidation: queries %llu  no-code exits %llu",
            static_cast<unsigned long long>(stats.invalidation_queries),
            static_cast<unsigned long long>(
                stats.invalidation_fast_no_code_page_exits));
        ImGui::Text("Invalidation blocks: examined %llu  invalidated %llu",
            static_cast<unsigned long long>(stats.invalidation_blocks_examined),
            static_cast<unsigned long long>(
                stats.invalidation_blocks_invalidated));
        ImGui::Text("Cache: hits %llu  misses %llu  flushes %llu  bytes %zu",
            static_cast<unsigned long long>(stats.cache_hits),
            static_cast<unsigned long long>(stats.cache_misses),
            static_cast<unsigned long long>(stats.flushes),
            stats.code_bytes);
        ImGui::Text("Slow paths: memory helpers %llu  MMIO %llu  exceptions %llu",
            static_cast<unsigned long long>(stats.memory_helper_calls),
            static_cast<unsigned long long>(stats.mmio_accesses),
            static_cast<unsigned long long>(stats.exceptions));
        ImGui::Text("Native memory: helpers %llu  fast L/S %llu / %llu  exception exits %llu",
            static_cast<unsigned long long>(stats.native_memory_helper_calls),
            static_cast<unsigned long long>(
                stats.native_memory_fastpath_loads),
            static_cast<unsigned long long>(
                stats.native_memory_fastpath_stores),
            static_cast<unsigned long long>(
                stats.native_memory_exception_exits));
        ImGui::Text("Helper load-delay: entries %llu  passes %llu  fallbacks %llu",
            static_cast<unsigned long long>(
                stats.native_helper_load_delay_entries),
            static_cast<unsigned long long>(
                stats.native_helper_load_delay_passes),
            static_cast<unsigned long long>(
                stats.native_helper_load_delay_fallbacks));
        const double avg_rejected =
            stats.native_rejected_block_count == 0
                ? 0.0
                : static_cast<double>(stats.native_rejected_block_instructions) /
                      static_cast<double>(stats.native_rejected_block_count);
        ImGui::Text("Rejected blocks: %llu  avg %.2f instr",
            static_cast<unsigned long long>(
                stats.native_rejected_block_count),
            avg_rejected);
        ImGui::Text("Exits: interrupts %llu  budget %llu  fallback %llu  pc mismatch %llu",
            static_cast<unsigned long long>(stats.interrupt_exits),
            static_cast<unsigned long long>(stats.budget_exits),
            static_cast<unsigned long long>(stats.fallback_exits),
            static_cast<unsigned long long>(stats.pc_mismatch_exits));
        return dirty;
    }
