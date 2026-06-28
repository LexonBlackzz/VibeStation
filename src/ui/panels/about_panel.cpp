#include "ui/app.h"

#include <imgui.h>

void App::panel_about() {
    ImGui::SetNextWindowSize(ImVec2(400, 200), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("About VibeStation", &show_about_,
        ImGuiWindowFlags_NoResize)) {
        ImGui::TextColored(ImVec4(0.6f, 0.4f, 1.0f, 1.0f), "VibeStation v0.5.2");
        ImGui::Separator();
        ImGui::Text("A PlayStation 1 emulator");
        ImGui::Spacing();
        ImGui::Text("CPU: MIPS R3000A %s",
            cpu_execution_mode_name(effective_cpu_execution_mode()));
        ImGui::Text("GPU: Software rasterizer");
        ImGui::Text("GTE: Fixed-point geometry engine");
        ImGui::Text("SPU: Gaussian + reverb core (stage 2)");
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.6f, 1.0f),
            "Built with SDL2 + Dear ImGui + OpenGL 3.3");
    }
    ImGui::End();
}
