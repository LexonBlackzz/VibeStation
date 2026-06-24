#include "ui/panels/memory_card_panel.h"
#include "ui/app.h"
#include "platform/memory_card_utils.h"

#include <imgui.h>

#include <algorithm>
#include <string>

void App::draw_memory_card_panel() {
    const char* mode_labels[] = { "Generic", "Per-Game", "Disabled" };

    ImGui::TextWrapped(
        "Configure card behavior per slot. Per-Game auto-creates cards by disc name.");
    ImGui::TextDisabled("Card files are stored in ./%s", kMemoryCardDirName);
    ImGui::Separator();

    for (int slot = 0; slot < kMemoryCardSlotCount; ++slot) {
        const std::string header = "Slot " + std::to_string(slot + 1);
        ImGui::TextColored(ImVec4(0.75f, 0.75f, 0.95f, 1.0f), "%s", header.c_str());

        int mode = std::max(0, std::min(2, config_memory_card_mode_[slot]));
        const std::string combo_label = "Mode##memcard_mode_" + std::to_string(slot);
        if (ImGui::Combo(combo_label.c_str(), &mode, mode_labels, IM_ARRAYSIZE(mode_labels))) {
            config_memory_card_mode_[slot] = mode;
            apply_memory_card_settings(true);
        }

        const std::string target = (slot < static_cast<int>(memory_card_target_paths_.size()))
            ? memory_card_target_paths_[slot]
            : std::string();

        if (target.empty()) {
            ImGui::TextDisabled("Target: (no card inserted)");
        }
        else {
            ImGui::TextWrapped("Target: %s", target.c_str());
        }

        bool inserted = runtime_snapshot_.memory_card_inserted[slot];
        bool dirty = runtime_snapshot_.memory_card_dirty[slot];
        std::string live_path = runtime_snapshot_.memory_card_path[slot];

        if ((!has_started_emulation_ || !emu_runner_.is_running()) && system_ != nullptr) {
            inserted = system_->memory_card_inserted(static_cast<u32>(slot));
            dirty = system_->memory_card_dirty(static_cast<u32>(slot));
            live_path = system_->memory_card_path(static_cast<u32>(slot));
        }

        ImGui::Text("Live: %s%s", inserted ? "Inserted" : "Empty", dirty ? " (dirty)" : "");

        if (!live_path.empty()) {
            ImGui::TextWrapped("Mounted: %s", live_path.c_str());
        }

        if (slot + 1 < kMemoryCardSlotCount) {
            ImGui::Separator();
        }
    }

    if (ImGui::Button("Apply Memory Card Settings")) {
        apply_memory_card_settings(true);
    }
}