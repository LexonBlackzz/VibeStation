#include "ui/app.h"

#include "version.h"

#include <imgui.h>

void App::panel_about() {
    ImGui::SetNextWindowSize(ImVec2(400, 200), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("About VibeStation", &show_about_,
        ImGuiWindowFlags_NoResize)) {
        ImGui::TextColored(ImVec4(0.6f, 0.4f, 1.0f, 1.0f),
            vibestation_full_version_string());
        ImGui::Separator();
        ImGui::Text("A PlayStation 1 emulator");
        ImGui::Spacing();
        const CpuExecutionMode cpu_mode = effective_cpu_execution_mode();
        const bool native_jit_requested =
            cpu_mode == CpuExecutionMode::X64Jit ||
            cpu_mode == CpuExecutionMode::X64JitV2 ||
            cpu_mode == CpuExecutionMode::X64JitV3;
        const bool native_unavailable =
            native_jit_requested && system_ &&
            !system_->cpu().cpu_backend_stats().native_available;
        ImGui::Text("CPU: MIPS R3000A %s",
            native_unavailable ? "decoded blocks (native JIT unavailable)"
                               : cpu_execution_mode_name(cpu_mode));
        ImGui::Text("GPU: Software rasterizer");
        ImGui::Text("GTE: Fixed-point geometry engine");
        ImGui::Text("SPU: Gaussian + reverb core (stage 2)");
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.6f, 1.0f),
            "Built with SDL2 + Dear ImGui + OpenGL 3.3");
    }
    ImGui::End();
}
