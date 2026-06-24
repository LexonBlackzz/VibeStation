#include "ui/app.h"
#include "core/types.h"
#include <imgui.h>

void App::draw_experimental_settings_panel() {
    ImGui::TextColored(ImVec4(0.9f, 0.7f, 0.3f, 1.0f),
        "Disabled by default. Enable only for targeted testing.");
    ImGui::Separator();

    ImGui::Checkbox("Experimental BIOS size mode",
        &g_experimental_bios_size_mode);
    ImGui::Checkbox("Unsafe PS2 BIOS mode", &g_unsafe_ps2_bios_mode);
    if (g_unsafe_ps2_bios_mode) {
        g_experimental_bios_size_mode = true;
    }
    ImGui::TextColored(
        ImVec4(0.7f, 0.7f, 0.3f, 1.0f),
        "Experimental mode accepts KB-aligned BIOS files.");
    ImGui::TextColored(
        ImVec4(0.8f, 0.5f, 0.3f, 1.0f),
        "Unsafe PS2 mode maps full BIOS size and is expected to be unstable.");

    ImGui::Separator();
    ImGui::Checkbox("Unhandled opcode fallback",
        &g_experimental_unhandled_special_returns_zero);
    ImGui::TextColored(
        ImVec4(0.8f, 0.5f, 0.3f, 1.0f),
        "When enabled, unknown opcodes act as NOPs; unknown SPECIAL funct values still write 0 to rd.");
    ImGui::Checkbox("DMA command sanitizer",
        &g_experimental_dma_command_sanitizer);
    ImGui::TextColored(
        ImVec4(0.8f, 0.5f, 0.3f, 1.0f),
        "When enabled, malformed DMA channel commands are coerced into legal transfer commands.");
                }