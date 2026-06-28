#include "ui/app.h"
#include "ui/input_bindings.h"

#include <SDL.h>
#include <imgui.h>

#include <string>

void App::draw_bindings_config_content() {
    if (pending_bind_index_ >= 0) {
        ImGui::TextColored(ImVec4(0.95f, 0.8f, 0.3f, 1.0f),
            "Press a key for %s (Esc to cancel)",
            kKeyboardBindEntries[pending_bind_index_].label);
    }
    else {
        ImGui::TextDisabled("Select a binding to assign a new key.");
    }

    if (ImGui::Button("Reset to Defaults")) {
        input_->set_default_bindings();
        pending_bind_index_ = -1;
        save_persistent_config();
    }

    ImGui::Spacing();
    for (int i = 0; i < kKeyboardBindEntryCount; ++i) {
        const SDL_Scancode scancode =
            input_->key_for_button(kKeyboardBindEntries[i].button);
        const char* key_name =
            (scancode != SDL_SCANCODE_UNKNOWN) ? SDL_GetScancodeName(scancode)
            : "Unbound";
        ImGui::Text("%s", kKeyboardBindEntries[i].label);
        ImGui::SameLine(140.0f);
        std::string assign_label =
            std::string((key_name && key_name[0] != '\0') ? key_name : "Unbound") +
            "##bind_" + kKeyboardBindEntries[i].config_key;
        if (ImGui::Button(assign_label.c_str(), ImVec2(120.0f, 0.0f))) {
            pending_bind_index_ = i;
        }
        ImGui::SameLine();
        std::string clear_label =
            std::string("Clear##") + kKeyboardBindEntries[i].config_key;
        if (ImGui::Button(clear_label.c_str())) {
            input_->clear_key_binding(kKeyboardBindEntries[i].button);
            if (pending_bind_index_ == i) {
                pending_bind_index_ = -1;
            }
            save_persistent_config();
        }
    }
}

void App::panel_bindings_config() {
    ImGui::SetNextWindowSize(ImVec2(460, 520), ImGuiCond_FirstUseEver);
    const bool was_open = show_bindings_config_;
    if (ImGui::Begin("Configure Bindings", &show_bindings_config_)) {
        draw_bindings_config_content();
    }
    ImGui::End();
    if (was_open && !show_bindings_config_ && pending_bind_index_ >= 0) {
        pending_bind_index_ = -1;
        status_message_ = "Keyboard rebinding canceled";
    }
}
