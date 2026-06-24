#include "ui/panels/grim_reaper_panel.h"
#include "ui/app.h"
#include <imgui.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

namespace {
    constexpr const char* kCorruptionPresetDirName = "corruption_presets";

    std::string trim_copy(const std::string& input) {
        const size_t begin = input.find_first_not_of(" \t\r\n");
        if (begin == std::string::npos) {
            return {};
        }
        const size_t end = input.find_last_not_of(" \t\r\n");
        return input.substr(begin, end - begin + 1);
    }

    struct ParsedCorruptionPreset {
        enum class Type {
            Invalid,
            GrimSingle,
            GrimBatch,
            RamReaper,
            GpuReaper,
            SoundReaper,
        };

        Type type = Type::Invalid;
        std::string display_name;
        std::string grim_area;
        float grim_randstrike = 0.0f;
        u64 grim_seed = 1u;
        bool grim_has_seed = false;
        std::string custom_start_hex;
        std::string custom_end_hex;
        bool intro_enabled = false;
        bool charset_enabled = false;
        bool end_enabled = false;
        float intro_randstrike = 0.0f;
        float charset_randstrike = 0.0f;
        float end_randstrike = 0.0f;
        u64 intro_seed = 1u;
        u64 charset_seed = 1u;
        u64 end_seed = 1u;
        bool ram_enabled = false;
        float ram_intensity = 0.0f;
        u32 ram_writes_per_frame = 0u;
        bool ram_target_main = true;
        bool ram_target_vram = true;
        bool ram_target_spu = true;
        u32 ram_range_start = 0u;
        u32 ram_range_end = psx::RAM_SIZE - 1u;
        u64 ram_seed = 1u;
        bool ram_has_seed = false;
        bool gpu_enabled = false;
        float gpu_intensity = 0.0f;
        u32 gpu_writes_per_frame = 0u;
        bool gpu_target_geometry = true;
        bool gpu_target_texture = true;
        bool gpu_target_display = false;
        u64 gpu_seed = 1u;
        bool gpu_has_seed = false;
        bool sound_enabled = false;
        float sound_intensity = 0.0f;
        u32 sound_writes_per_frame = 0u;
        bool sound_target_pitch = true;
        bool sound_target_envelope = true;
        bool sound_target_reverb = true;
        bool sound_target_mixer = true;
        u64 sound_seed = 1u;
        bool sound_has_seed = false;
    };

    bool parse_bool_value(const std::string& value, bool fallback) {
        if (value == "1" || value == "true" || value == "TRUE" || value == "on" ||
            value == "ON") {
            return true;
        }
        if (value == "0" || value == "false" || value == "FALSE" || value == "off" ||
            value == "OFF") {
            return false;
        }
        return fallback;
    }

    bool parse_u64_value(const std::string& value, u64& out) {
        char* end_ptr = nullptr;
        const unsigned long long parsed = std::strtoull(value.c_str(), &end_ptr, 10);
        if (end_ptr == value.c_str()) {
            return false;
        }
        while (*end_ptr != '\0' && std::isspace(static_cast<unsigned char>(*end_ptr))) {
            ++end_ptr;
        }
        if (*end_ptr != '\0') {
            return false;
        }
        out = static_cast<u64>(parsed);
        return true;
    }

    bool parse_u32_value(const std::string& value, u32& out) {
        u64 parsed = 0;
        if (!parse_u64_value(value, parsed)) {
            return false;
        }
        out = static_cast<u32>(std::min<u64>(parsed, std::numeric_limits<u32>::max()));
        return true;
    }

    bool parse_float_value(const std::string& value, float& out) {
        char* end_ptr = nullptr;
        const float parsed = std::strtof(value.c_str(), &end_ptr);
        if (end_ptr == value.c_str()) {
            return false;
        }
        while (*end_ptr != '\0' && std::isspace(static_cast<unsigned char>(*end_ptr))) {
            ++end_ptr;
        }
        if (*end_ptr != '\0') {
            return false;
        }
        out = parsed;
        return true;
    }

    std::string sanitize_preset_file_stem(const char* text, const char* fallback) {
        const std::string raw = trim_copy(text ? std::string(text) : std::string());
        const std::string source = raw.empty() ? std::string(fallback) : raw;
        std::string out;
        out.reserve(source.size());
        for (char ch : source) {
            const unsigned char uch = static_cast<unsigned char>(ch);
            if (std::isalnum(uch)) {
                out.push_back(static_cast<char>(std::tolower(uch)));
            }
            else if (ch == '-' || ch == '_') {
                out.push_back(ch);
            }
            else if (std::isspace(uch)) {
                out.push_back('_');
            }
        }
        if (out.empty()) {
            out = fallback;
        }
        return out;
    }

    std::filesystem::path ensure_corruption_preset_dir() {
        const std::filesystem::path dir = std::filesystem::current_path() /
            kCorruptionPresetDirName;
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        return dir;
    }

    bool parse_corruption_preset_file(const std::filesystem::path& path,
        ParsedCorruptionPreset& out) {
        std::ifstream in(path);
        if (!in.is_open()) {
            return false;
        }

        ParsedCorruptionPreset parsed{};
        std::string section;
        std::string line;
        while (std::getline(in, line)) {
            line = trim_copy(line);
            if (line.empty() || line[0] == '#') {
                continue;
            }
            if (line.back() == '(') {
                section = trim_copy(line.substr(0, line.size() - 1));
                continue;
            }
            if (line == ")") {
                section.clear();
                continue;
            }

            const size_t eq = line.find('=');
            if (eq == std::string::npos) {
                continue;
            }
            const std::string key = trim_copy(line.substr(0, eq));
            const std::string value = trim_copy(line.substr(eq + 1));

            if (section.empty()) {
                if (key == "type") {
                    if (value == "grim_single") {
                        parsed.type = ParsedCorruptionPreset::Type::GrimSingle;
                    }
                    else if (value == "grim_batch") {
                        parsed.type = ParsedCorruptionPreset::Type::GrimBatch;
                    }
                    else if (value == "ram_reaper") {
                        parsed.type = ParsedCorruptionPreset::Type::RamReaper;
                    }
                    else if (value == "gpu_reaper") {
                        parsed.type = ParsedCorruptionPreset::Type::GpuReaper;
                    }
                    else if (value == "sound_reaper") {
                        parsed.type = ParsedCorruptionPreset::Type::SoundReaper;
                    }
                }
                else if (key == "name") {
                    parsed.display_name = value;
                }
                else if (key == "area") {
                    parsed.grim_area = value;
                }
                else if (key == "seed") {
                    parsed.grim_has_seed = parse_u64_value(value, parsed.grim_seed);
                }
                else if (key == "randstrike") {
                    parse_float_value(value, parsed.grim_randstrike);
                }
                else if (key == "custom_start") {
                    parsed.custom_start_hex = value;
                }
                else if (key == "custom_end") {
                    parsed.custom_end_hex = value;
                }
                else if (key == "enabled") {
                    parsed.ram_enabled = parse_bool_value(value, parsed.ram_enabled);
                    parsed.gpu_enabled = parse_bool_value(value, parsed.gpu_enabled);
                    parsed.sound_enabled = parse_bool_value(value, parsed.sound_enabled);
                }
                else if (key == "intensity") {
                    parse_float_value(value, parsed.ram_intensity);
                    parse_float_value(value, parsed.gpu_intensity);
                    parse_float_value(value, parsed.sound_intensity);
                }
                else if (key == "writes_per_frame") {
                    parse_u32_value(value, parsed.ram_writes_per_frame);
                    parse_u32_value(value, parsed.gpu_writes_per_frame);
                    parse_u32_value(value, parsed.sound_writes_per_frame);
                }
                else if (key == "target_main_ram") {
                    parsed.ram_target_main =
                        parse_bool_value(value, parsed.ram_target_main);
                }
                else if (key == "target_vram") {
                    parsed.ram_target_vram =
                        parse_bool_value(value, parsed.ram_target_vram);
                }
                else if (key == "target_spu_ram") {
                    parsed.ram_target_spu =
                        parse_bool_value(value, parsed.ram_target_spu);
                }
                else if (key == "range_start") {
                    parse_u32_value(value, parsed.ram_range_start);
                }
                else if (key == "range_end") {
                    parse_u32_value(value, parsed.ram_range_end);
                }
                else if (key == "target_geometry") {
                    parsed.gpu_target_geometry =
                        parse_bool_value(value, parsed.gpu_target_geometry);
                }
                else if (key == "target_texture_state") {
                    parsed.gpu_target_texture =
                        parse_bool_value(value, parsed.gpu_target_texture);
                }
                else if (key == "target_display_state") {
                    parsed.gpu_target_display =
                        parse_bool_value(value, parsed.gpu_target_display);
                }
                else if (key == "target_pitch") {
                    parsed.sound_target_pitch =
                        parse_bool_value(value, parsed.sound_target_pitch);
                }
                else if (key == "target_envelope") {
                    parsed.sound_target_envelope =
                        parse_bool_value(value, parsed.sound_target_envelope);
                }
                else if (key == "target_reverb") {
                    parsed.sound_target_reverb =
                        parse_bool_value(value, parsed.sound_target_reverb);
                }
                else if (key == "target_mixer") {
                    parsed.sound_target_mixer =
                        parse_bool_value(value, parsed.sound_target_mixer);
                }
                continue;
            }

            if (section == "intro") {
                parsed.intro_enabled = true;
                if (key == "seed") {
                    parse_u64_value(value, parsed.intro_seed);
                }
                else if (key == "randstrike") {
                    parse_float_value(value, parsed.intro_randstrike);
                }
            }
            else if (section == "charset") {
                parsed.charset_enabled = true;
                if (key == "seed") {
                    parse_u64_value(value, parsed.charset_seed);
                }
                else if (key == "randstrike") {
                    parse_float_value(value, parsed.charset_randstrike);
                }
            }
            else if (section == "end") {
                parsed.end_enabled = true;
                if (key == "seed") {
                    parse_u64_value(value, parsed.end_seed);
                }
                else if (key == "randstrike") {
                    parse_float_value(value, parsed.end_randstrike);
                }
            }
            else if (section == "ram_reaper") {
                if (key == "seed") {
                    parsed.ram_has_seed = parse_u64_value(value, parsed.ram_seed);
                }
            }
            else if (section == "gpu_reaper") {
                if (key == "seed") {
                    parsed.gpu_has_seed = parse_u64_value(value, parsed.gpu_seed);
                }
            }
            else if (section == "sound_reaper") {
                if (key == "seed") {
                    parsed.sound_has_seed = parse_u64_value(value, parsed.sound_seed);
                }
            }
        }

        if (parsed.display_name.empty()) {
            parsed.display_name = path.stem().string();
        }
        out = parsed;
        return parsed.type != ParsedCorruptionPreset::Type::Invalid;
    }
}

void App::panel_grim_reaper() {
    ImGui::SetNextWindowSize(ImVec2(640, 460), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Grim Reaper", &show_grim_reaper_)) {
        ImGui::End();
        return;
    }

    ImGui::TextColored(ImVec4(0.9f, 0.4f, 0.4f, 1.0f),
        "Experimental BIOS corruption. Original BIOS is never modified.");
    if (ImGui::BeginTabBar("GrimReaperTabs")) {
        if (ImGui::BeginTabItem("Single BIOS")) {

            grim_reaper_area_index_ =
                std::max(0, std::min(kGrimReaperRangeCount - 1, grim_reaper_area_index_));
            const char* grim_area_labels[kGrimReaperRangeCount] = {};
            for (int i = 0; i < kGrimReaperRangeCount; ++i) {
                grim_area_labels[i] = kGrimReaperRanges[i].label;
            }
            ImGui::Combo("Target Area", &grim_reaper_area_index_, grim_area_labels,
                kGrimReaperRangeCount);

            const bool grim_intro_mode = (grim_reaper_area_index_ == 0);
            const float grim_slider_max = grim_intro_mode ? 0.1f : 100.0f;
            grim_reaper_random_percent_ =
                std::max(0.001f, std::min(grim_slider_max, grim_reaper_random_percent_));
            ImGui::SliderFloat("Random Strike (%)", &grim_reaper_random_percent_, 0.001f,
                grim_slider_max, "%.3f%%");
            if (grim_intro_mode) {
                const float p = grim_reaper_random_percent_;
                if (p >= 0.1f - 1e-6f) {
                    ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f),
                        "Absolute Death - Boot is very likely to fail.");
                }
                else if (p >= 0.05f) {
                    ImGui::TextColored(
                        ImVec4(0.95f, 0.2f, 0.2f, 1.0f),
                        "Danger - Very unstable, may not work, extreme corruption.");
                }
                else if (p >= 0.02f) {
                    ImGui::TextColored(
                        ImVec4(1.0f, 0.55f, 0.1f, 1.0f),
                        "Warning - Unstable, heavy corruptions.");
                }
                else if (p >= 0.01f) {
                    ImGui::TextColored(
                        ImVec4(0.2f, 0.9f, 0.3f, 1.0f),
                        "Recommended - Stable corruptions.");
                }
                else {
                    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                        "Very mild corruption range.");
                }
            }
            if (grim_reaper_area_index_ == (kGrimReaperRangeCount - 1)) {
                ImGui::InputText("Custom Start (hex)", grim_reaper_custom_start_hex_,
                    IM_ARRAYSIZE(grim_reaper_custom_start_hex_));
                ImGui::InputText("Custom End (hex)", grim_reaper_custom_end_hex_,
                    IM_ARRAYSIZE(grim_reaper_custom_end_hex_));
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                    "Set end to 0 for end-of-file.");
            }

            ImGui::Separator();
            ImGui::Text("Corruption Seed");
            ImGui::Checkbox("Use Custom Seed", &grim_use_custom_seed_);
            if (grim_use_custom_seed_) {
                ImGui::InputScalar("Seed", ImGuiDataType_U64, &grim_seed_);
            }
            if (ImGui::Checkbox(
                "Do not suppress console logs when corrupting (may freeze emulator!)",
                &grim_reaper_keep_console_logs_)) {
                if (grim_reaper_mode_active_) {
                    if (grim_reaper_keep_console_logs_ && grim_reaper_logs_suppressed_) {
                        g_log_category_mask = grim_reaper_saved_log_mask_;
                        g_log_level = grim_reaper_saved_log_level_;
                        grim_reaper_logs_suppressed_ = false;
                    }
                    else if (!grim_reaper_keep_console_logs_ &&
                        !grim_reaper_logs_suppressed_) {
                        grim_reaper_saved_log_mask_ = g_log_category_mask;
                        grim_reaper_saved_log_level_ = g_log_level;
                        grim_reaper_logs_suppressed_ = true;
                        g_log_category_mask = 0;
                        g_log_level = LogLevel::Error;
                    }
                }
            }
            ImGui::Text("Last Used Seed: %llu",
                static_cast<unsigned long long>(grim_last_used_seed_));
            if (!system_->bios_loaded() || bios_path_.empty()) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Reap && Reboot BIOS")) {
                reap_and_reboot_bios();
            }
            if (!system_->bios_loaded() || bios_path_.empty()) {
                ImGui::EndDisabled();
                ImGui::TextColored(ImVec4(0.8f, 0.6f, 0.3f, 1.0f), "Load a BIOS first.");
            }

            ImGui::SameLine();
            if (!has_started_emulation_) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Stop")) {
                emu_runner_.pause_and_wait_idle();
                disable_ram_reaper_mode();
                disable_gpu_reaper_mode();
                disable_sound_reaper_mode();
                has_started_emulation_ = false;
                status_message_ = "Emulation stopped";
            }
            if (!has_started_emulation_) {
                ImGui::EndDisabled();
            }

            ImGui::SameLine();
            if (grim_reaper_last_output_path_.empty()) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Restart Corrupted BIOS")) {
                emu_runner_.pause_and_wait_idle();
                disable_ram_reaper_mode();
                disable_gpu_reaper_mode();
                disable_sound_reaper_mode();
                set_grim_reaper_mode(true);
                if (!system_->load_bios(grim_reaper_last_output_path_)) {
                    set_grim_reaper_mode(false);
                    status_message_ = "Failed to load last corrupted BIOS copy.";
                }
                else {
                    has_started_emulation_ = false;
                    system_->reset();
                    apply_memory_card_settings(false);
                    has_started_emulation_ = true;
                    emu_runner_.set_running(true);
                    status_message_ = "Corrupted BIOS emulation restarted";
                }
            }
            if (grim_reaper_last_output_path_.empty()) {
                ImGui::EndDisabled();
            }

            ImGui::Separator();
            ImGui::Text("Preset Files");
            ImGui::InputText("Single Preset Name", grim_preset_name_,
                IM_ARRAYSIZE(grim_preset_name_));
            if (ImGui::Button("Save Single Preset")) {
                save_current_grim_preset(false);
            }
            ImGui::SameLine();
            if (ImGui::Button("Open Preset Browser")) {
                refresh_corruption_preset_list();
                show_corruption_presets_ = true;
            }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("RAM Reaper")) {
            ImGui::Separator();
            ImGui::Text("RAM Reaper");
            ImGui::TextColored(
                ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                "Tilt-style real-time corruption. Enable VRAM/SPU targets for visual/audio glitches.");
            ImGui::Checkbox("Enable RAM Reaper", &ram_reaper_enabled_);
            ImGui::SliderFloat("Tilt Intensity (%)", &ram_reaper_intensity_percent_, 0.0f,
                100.0f, "%.1f%%");
            int writes_per_frame = static_cast<int>(
                std::min<u32>(ram_reaper_writes_per_frame_, 5000u));
            ImGui::SliderInt("Base Writes / Frame", &writes_per_frame, 0, 5000);
            ram_reaper_writes_per_frame_ = static_cast<u32>(std::max(0, writes_per_frame));
            ImGui::Checkbox("Target Main RAM", &ram_reaper_affect_main_ram_);
            ImGui::Checkbox("Target VRAM (Visual)", &ram_reaper_affect_vram_);
            ImGui::Checkbox("Target SPU RAM (Audio)", &ram_reaper_affect_spu_ram_);
            if (ram_reaper_affect_main_ram_) {
                ImGui::InputScalar("Start (hex)", ImGuiDataType_U32, &ram_reaper_range_start_,
                    nullptr, nullptr, "%06X",
                    ImGuiInputTextFlags_CharsHexadecimal);
                ImGui::InputScalar("End (hex)", ImGuiDataType_U32, &ram_reaper_range_end_,
                    nullptr, nullptr, "%06X",
                    ImGuiInputTextFlags_CharsHexadecimal);
            }
            else {
                ImGui::TextColored(ImVec4(0.75f, 0.75f, 0.75f, 1.0f),
                    "Main RAM range controls disabled (target off).");
            }
            ram_reaper_range_start_ = std::min(ram_reaper_range_start_, psx::RAM_SIZE - 1u);
            ram_reaper_range_end_ = std::min(ram_reaper_range_end_, psx::RAM_SIZE - 1u);
            const float expected_writes =
                (static_cast<float>(ram_reaper_writes_per_frame_) *
                    (ram_reaper_intensity_percent_ / 100.0f));
            ImGui::Text("Expected Writes/Frame: %.2f", expected_writes);
            if (!ram_reaper_affect_main_ram_ && !ram_reaper_affect_vram_ &&
                !ram_reaper_affect_spu_ram_) {
                ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.3f, 1.0f),
                    "No targets selected.");
            }
            ImGui::Checkbox("Use Custom Seed##ram", &ram_reaper_use_custom_seed_);
            if (ram_reaper_use_custom_seed_) {
                ImGui::InputScalar("Seed##ram", ImGuiDataType_U64, &ram_reaper_seed_);
            }
            ImGui::Text("Active Seed: %llu",
                static_cast<unsigned long long>(ram_reaper_active_seed_));
            ImGui::Text("Total Mutations: %llu",
                static_cast<unsigned long long>(ram_reaper_total_mutations_));
            ImGui::InputText("RAM Preset Name", ram_preset_name_,
                IM_ARRAYSIZE(ram_preset_name_));
            if (ImGui::Button("Save RAM Preset")) {
                save_current_ram_preset();
            }
            ImGui::SameLine();
            if (ImGui::Button("Browse Presets##ram")) {
                refresh_corruption_preset_list();
                show_corruption_presets_ = true;
            }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("GPU Reaper")) {
            ImGui::Separator();
            ImGui::Text("GPU Reaper");
            ImGui::TextColored(
                ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                "Real-time GPU state corruption for broken polygons, warped textures, and unstable display state.");
            ImGui::Checkbox("Enable GPU Reaper", &gpu_reaper_enabled_);
            ImGui::SliderFloat("GPU Chaos (%)", &gpu_reaper_intensity_percent_, 0.0f,
                100.0f, "%.1f%%");
            int gpu_writes_per_frame = static_cast<int>(
                std::min<u32>(gpu_reaper_writes_per_frame_, 5000u));
            ImGui::SliderInt("GPU Writes / Frame", &gpu_writes_per_frame, 0, 5000);
            gpu_reaper_writes_per_frame_ =
                static_cast<u32>(std::max(0, gpu_writes_per_frame));
            ImGui::Checkbox("Target Geometry State", &gpu_reaper_affect_geometry_);
            ImGui::Checkbox("Target Texture State", &gpu_reaper_affect_texture_state_);
            ImGui::Checkbox("Target Display State", &gpu_reaper_affect_display_state_);
            const float gpu_expected_writes =
                (static_cast<float>(gpu_reaper_writes_per_frame_) *
                    (gpu_reaper_intensity_percent_ / 100.0f));
            ImGui::Text("Expected Writes/Frame: %.2f", gpu_expected_writes);
            if (!gpu_reaper_affect_geometry_ && !gpu_reaper_affect_texture_state_ &&
                !gpu_reaper_affect_display_state_) {
                ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.3f, 1.0f),
                    "No GPU targets selected.");
            }
            ImGui::Checkbox("Use Custom Seed##gpu", &gpu_reaper_use_custom_seed_);
            if (gpu_reaper_use_custom_seed_) {
                ImGui::InputScalar("Seed##gpu", ImGuiDataType_U64, &gpu_reaper_seed_);
            }
            ImGui::Text("Active Seed: %llu",
                static_cast<unsigned long long>(gpu_reaper_active_seed_));
            ImGui::Text("Total Mutations: %llu",
                static_cast<unsigned long long>(gpu_reaper_total_mutations_));
            ImGui::InputText("GPU Preset Name", gpu_preset_name_,
                IM_ARRAYSIZE(gpu_preset_name_));
            if (ImGui::Button("Save GPU Preset")) {
                save_current_gpu_preset();
            }
            ImGui::SameLine();
            if (ImGui::Button("Browse Presets##gpu")) {
                refresh_corruption_preset_list();
                show_corruption_presets_ = true;
            }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Sound Reaper")) {
            ImGui::Separator();
            ImGui::Text("Sound Reaper");
            ImGui::TextColored(
                ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                "Real-time SPU corruption for pitch, reverb, ADSR release, and mixer routing.");
            draw_spu_diagnostic_mode_controls();
            ImGui::Separator();
            ImGui::Checkbox("Enable Sound Reaper", &sound_reaper_enabled_);
            ImGui::SliderFloat("Sound Chaos (%)", &sound_reaper_intensity_percent_, 0.0f,
                100.0f, "%.1f%%");
            int sound_writes_per_frame = static_cast<int>(
                std::min<u32>(sound_reaper_writes_per_frame_, 5000u));
            ImGui::SliderInt("Sound Writes / Frame", &sound_writes_per_frame, 0, 5000);
            sound_reaper_writes_per_frame_ =
                static_cast<u32>(std::max(0, sound_writes_per_frame));
            ImGui::Checkbox("Target Pitch / Semitones", &sound_reaper_affect_pitch_);
            ImGui::Checkbox("Target Release / ADSR", &sound_reaper_affect_envelope_);
            ImGui::Checkbox("Target Reverb / Delay", &sound_reaper_affect_reverb_);
            ImGui::Checkbox("Target Wet / Dry Mixer", &sound_reaper_affect_mixer_);
            const float sound_expected_writes =
                (static_cast<float>(sound_reaper_writes_per_frame_) *
                    (sound_reaper_intensity_percent_ / 100.0f));
            ImGui::Text("Expected Writes/Frame: %.2f", sound_expected_writes);
            if (!sound_reaper_affect_pitch_ && !sound_reaper_affect_envelope_ &&
                !sound_reaper_affect_reverb_ && !sound_reaper_affect_mixer_) {
                ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.3f, 1.0f),
                    "No SPU targets selected.");
            }
            ImGui::Checkbox("Use Custom Seed##sound", &sound_reaper_use_custom_seed_);
            if (sound_reaper_use_custom_seed_) {
                ImGui::InputScalar("Seed##sound", ImGuiDataType_U64, &sound_reaper_seed_);
            }
            ImGui::Text("Active Seed: %llu",
                static_cast<unsigned long long>(sound_reaper_active_seed_));
            ImGui::Text("Total Mutations: %llu",
                static_cast<unsigned long long>(sound_reaper_total_mutations_));
            ImGui::Separator();
            std::filesystem::path sound_ram_path = std::filesystem::current_path() / "sound.ram";
            sound_ram_voice_index_ = std::max(0, std::min(23, sound_ram_voice_index_));
            if (sound_ram_multi_voice_export_) {
                ImGui::BeginDisabled();
            }
            ImGui::SliderInt("Sample Voice", &sound_ram_voice_index_, 0, 23);
            if (sound_ram_multi_voice_export_) {
                ImGui::EndDisabled();
            }
            int selected_voice_count = 0;
            if (ImGui::Checkbox("Multi-Voice Export", &sound_ram_multi_voice_export_) &&
                sound_ram_multi_voice_export_) {
                sound_ram_voice_selected_.fill(false);
                sound_ram_voice_selected_[static_cast<size_t>(sound_ram_voice_index_)] = true;
            }
            if (sound_ram_multi_voice_export_) {
                if (ImGui::Button("Current Voice Only")) {
                    sound_ram_voice_selected_.fill(false);
                    sound_ram_voice_selected_[static_cast<size_t>(sound_ram_voice_index_)] = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("Select All Voices")) {
                    sound_ram_voice_selected_.fill(true);
                }
                ImGui::SameLine();
                if (ImGui::Button("Clear Voice Selection")) {
                    sound_ram_voice_selected_.fill(false);
                }
                if (ImGui::BeginTable("SoundRamVoiceSelection", 4,
                        ImGuiTableFlags_SizingStretchSame)) {
                    for (int voice = 0; voice < 24; ++voice) {
                        ImGui::TableNextColumn();
                        ImGui::Checkbox(
                            ("Voice " + std::to_string(voice)).c_str(),
                            &sound_ram_voice_selected_[static_cast<size_t>(voice)]);
                        if (sound_ram_voice_selected_[static_cast<size_t>(voice)]) {
                            ++selected_voice_count;
                        }
                    }
                    ImGui::EndTable();
                }
                ImGui::Text("Selected Voices: %d", selected_voice_count);
            }
            const bool replacement_loaded = system_->spu_replacement_sample_loaded();
            const bool replacement_enabled = system_->spu_replacement_sample_enabled();
            ImGui::Text("Replacement Sample: %s | %s | %zu bytes",
                replacement_loaded ? "Loaded" : "Not loaded",
                replacement_enabled ? "Enabled" : "Disabled",
                system_->spu_replacement_sample_bytes());
            ImGui::TextWrapped("sound.ram path: %s", sound_ram_path.string().c_str());
            auto run_spu_sample_action = [&](const auto& action) {
                const bool was_running = emu_runner_.is_running();
                if (was_running) {
                    emu_runner_.pause_and_wait_idle();
                }
                action();
                if (was_running) {
                    emu_runner_.set_running(true);
                }
            };
            if (ImGui::Button(sound_ram_multi_voice_export_
                    ? "Save Selected Voices"
                    : "Save Voice To sound.ram")) {
                run_spu_sample_action([&]() {
                    std::string error;
                    if (sound_ram_multi_voice_export_) {
                        std::vector<int> voices;
                        voices.reserve(sound_ram_voice_selected_.size());
                        for (int voice = 0;
                             voice < static_cast<int>(sound_ram_voice_selected_.size());
                             ++voice) {
                            if (sound_ram_voice_selected_[static_cast<size_t>(voice)]) {
                                voices.push_back(voice);
                            }
                        }
                        if (system_->save_spu_voice_samples_to_file(
                                voices, sound_ram_path.string(), &error)) {
                            status_message_ = "Saved combined sound.ram from " +
                                std::to_string(voices.size()) + " SPU voices.";
                        }
                        else {
                            status_message_ = error.empty()
                                ? "Failed to save combined sound.ram."
                                : error;
                        }
                    }
                    else if (system_->save_spu_voice_sample_to_file(
                                 sound_ram_voice_index_, sound_ram_path.string(), &error)) {
                        status_message_ = "Saved sound.ram from SPU voice " +
                            std::to_string(sound_ram_voice_index_);
                    }
                    else {
                        status_message_ = error.empty() ? "Failed to save sound.ram." : error;
                    }
                });
            }
            ImGui::SameLine();
            if (ImGui::Button("Load sound.ram")) {
                run_spu_sample_action([&]() {
                    std::string error;
                    if (system_->load_spu_replacement_sample_from_file(
                        sound_ram_path.string(), &error)) {
                        status_message_ =
                            "Loaded sound.ram. New SPU key-ons will use the replacement sample.";
                    }
                    else {
                        status_message_ = error.empty() ? "Failed to load sound.ram." : error;
                    }
                });
            }
            ImGui::SameLine();
            if (!replacement_loaded) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Clear Replacement")) {
                run_spu_sample_action([&]() {
                    system_->clear_spu_replacement_sample();
                    status_message_ = "Cleared SPU replacement sample.";
                });
            }
            if (!replacement_loaded) {
                ImGui::EndDisabled();
            }
            ImGui::InputText("Sound Preset Name", sound_preset_name_,
                IM_ARRAYSIZE(sound_preset_name_));
            if (ImGui::Button("Save Sound Preset")) {
                save_current_sound_preset();
            }
            ImGui::SameLine();
            if (ImGui::Button("Browse Presets##sound")) {
                refresh_corruption_preset_list();
                show_corruption_presets_ = true;
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Batch BIOS")) {
            ImGui::Separator();
            ImGui::Text("Batch Corruption");
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                "Select multiple ranges and apply random strike per range.");
            ImGui::Checkbox("Use Custom Seeds Per Range", &grim_batch_use_custom_seeds_);

            ImGui::Checkbox("Intro/Bootmenu", &grim_batch_intro_enabled_);
            if (grim_batch_intro_enabled_) {
                grim_batch_intro_percent_ =
                    std::max(0.001f, std::min(0.1f, grim_batch_intro_percent_));
                ImGui::SliderFloat("Intro Strike (%)", &grim_batch_intro_percent_, 0.001f,
                    0.1f, "%.3f%%");

                const float p = grim_batch_intro_percent_;
                if (p >= 0.1f - 1e-6f) {
                    ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f),
                        "Absolute Death - Boot is very likely to fail.");
                }
                else if (p >= 0.05f) {
                    ImGui::TextColored(
                        ImVec4(0.95f, 0.2f, 0.2f, 1.0f),
                        "Danger - Very unstable, may not work, extreme corruption.");
                }
                else if (p >= 0.02f) {
                    ImGui::TextColored(
                        ImVec4(1.0f, 0.55f, 0.1f, 1.0f),
                        "Warning - Unstable, heavy corruptions.");
                }
                else if (p >= 0.01f) {
                    ImGui::TextColored(
                        ImVec4(0.2f, 0.9f, 0.3f, 1.0f),
                        "Recommended - Stable corruptions.");
                }
                if (grim_batch_use_custom_seeds_) {
                    ImGui::InputScalar("Intro Seed", ImGuiDataType_U64, &grim_batch_intro_seed_);
                }
            }

            ImGui::Checkbox("Character Sets", &grim_batch_charset_enabled_);
            if (grim_batch_charset_enabled_) {
                grim_batch_charset_percent_ =
                    std::max(0.001f, std::min(100.0f, grim_batch_charset_percent_));
                ImGui::SliderFloat("Charset Strike (%)", &grim_batch_charset_percent_, 0.001f,
                    100.0f, "%.3f%%");
                if (grim_batch_use_custom_seeds_) {
                    ImGui::InputScalar("Charset Seed", ImGuiDataType_U64,
                        &grim_batch_charset_seed_);
                }
            }

            ImGui::Checkbox("End", &grim_batch_end_enabled_);
            if (grim_batch_end_enabled_) {
                grim_batch_end_percent_ =
                    std::max(0.001f, std::min(100.0f, grim_batch_end_percent_));
                ImGui::SliderFloat("End Strike (%)", &grim_batch_end_percent_, 0.001f,
                    100.0f, "%.3f%%");
                if (grim_batch_use_custom_seeds_) {
                    ImGui::InputScalar("End Seed", ImGuiDataType_U64, &grim_batch_end_seed_);
                }
            }

            const bool batch_has_any_selection =
                grim_batch_intro_enabled_ || grim_batch_charset_enabled_ || grim_batch_end_enabled_;
            ImGui::InputText("Batch Preset Name", batch_preset_name_,
                IM_ARRAYSIZE(batch_preset_name_));
            if (ImGui::Button("Save Batch Preset")) {
                save_current_grim_preset(true);
            }
            ImGui::SameLine();
            if (ImGui::Button("Browse Presets##batch")) {
                refresh_corruption_preset_list();
                show_corruption_presets_ = true;
            }
            if (!system_->bios_loaded() || bios_path_.empty() || !batch_has_any_selection) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Batch Corrupt && Start")) {
                reap_and_reboot_bios_batch();
            }
            if (!system_->bios_loaded() || bios_path_.empty() || !batch_has_any_selection) {
                ImGui::EndDisabled();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    if (!grim_reaper_last_output_path_.empty()) {
        ImGui::Text("Last Corrupted BIOS: %s", grim_reaper_last_output_path_.c_str());
        ImGui::Text("Last Mutations: %u",
            static_cast<unsigned>(grim_reaper_last_mutations_));
    }
    ImGui::TextColored(grim_reaper_mode_active_ ? ImVec4(0.9f, 0.6f, 0.3f, 1.0f)
        : ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
        "Console logging: %s",
        grim_reaper_logs_suppressed_ ? "Suppressed" : "Normal");

    ImGui::End();
}

void App::refresh_corruption_preset_list() {
    corruption_presets_.clear();
    selected_corruption_preset_index_ = -1;

    const std::filesystem::path dir = ensure_corruption_preset_dir();
    std::error_code ec;
    if (!std::filesystem::exists(dir, ec)) {
        return;
    }

    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (ec || !entry.is_regular_file()) {
            continue;
        }
        ParsedCorruptionPreset parsed{};
        if (!parse_corruption_preset_file(entry.path(), parsed)) {
            continue;
        }

        CorruptionPresetListEntry item{};
        item.file_name = entry.path().filename().string();
        item.display_name = parsed.display_name;
        switch (parsed.type) {
        case ParsedCorruptionPreset::Type::GrimSingle:
            item.preset_type = "grim-single";
            break;
        case ParsedCorruptionPreset::Type::GrimBatch:
            item.preset_type = "grim-batch";
            break;
        case ParsedCorruptionPreset::Type::RamReaper:
            item.preset_type = "ram-reaper";
            break;
        case ParsedCorruptionPreset::Type::GpuReaper:
            item.preset_type = "gpu-reaper";
            break;
        case ParsedCorruptionPreset::Type::SoundReaper:
            item.preset_type = "sound-reaper";
            break;
        default:
            item.preset_type = "unknown";
            break;
        }
        item.path = entry.path();
        corruption_presets_.push_back(std::move(item));
    }

    std::sort(corruption_presets_.begin(), corruption_presets_.end(),
        [](const CorruptionPresetListEntry& a,
            const CorruptionPresetListEntry& b) {
                if (a.display_name != b.display_name) {
                    return a.display_name < b.display_name;
                }
                return a.file_name < b.file_name;
        });
}

bool App::save_current_grim_preset(bool batch_mode) {
    const std::filesystem::path dir = ensure_corruption_preset_dir();
    const char* raw_name = batch_mode ? batch_preset_name_ : grim_preset_name_;
    const char* fallback = batch_mode ? "grim_batch" : "grim_preset";
    const std::string stem = sanitize_preset_file_stem(raw_name, fallback);
    const std::filesystem::path path = dir / (stem + ".vibe_preset");

    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        status_message_ = "Failed to create preset file.";
        return false;
    }

    out << std::fixed << std::setprecision(3);
    if (batch_mode) {
        out << "type=grim_batch\n";
        out << "name=" << stem << "\n";
        if (grim_batch_intro_enabled_) {
            out << "intro(\n";
            out << "seed=" << grim_batch_intro_seed_ << "\n";
            out << "randstrike=" << grim_batch_intro_percent_ << "\n";
            out << ")\n";
        }
        if (grim_batch_charset_enabled_) {
            out << "charset(\n";
            out << "seed=" << grim_batch_charset_seed_ << "\n";
            out << "randstrike=" << grim_batch_charset_percent_ << "\n";
            out << ")\n";
        }
        if (grim_batch_end_enabled_) {
            out << "end(\n";
            out << "seed=" << grim_batch_end_seed_ << "\n";
            out << "randstrike=" << grim_batch_end_percent_ << "\n";
            out << ")\n";
        }
    }
    else {
        out << "type=grim_single\n";
        out << "name=" << stem << "\n";
        out << "area=" << kGrimReaperRanges[grim_reaper_area_index_].slug << "\n";
        out << "seed=" << grim_seed_ << "\n";
        out << "randstrike=" << grim_reaper_random_percent_ << "\n";
        if (grim_reaper_area_index_ == (kGrimReaperRangeCount - 1)) {
            out << "custom_start=" << grim_reaper_custom_start_hex_ << "\n";
            out << "custom_end=" << grim_reaper_custom_end_hex_ << "\n";
        }
    }

    if (!out) {
        status_message_ = "Failed writing preset file.";
        return false;
    }

    refresh_corruption_preset_list();
    status_message_ = "Saved preset: " + path.filename().string();
    return true;
}

bool App::save_current_ram_preset() {
    const std::filesystem::path dir = ensure_corruption_preset_dir();
    const std::string stem = sanitize_preset_file_stem(ram_preset_name_, "ram_reaper");
    const std::filesystem::path path = dir / (stem + ".vibe_preset");

    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        status_message_ = "Failed to create preset file.";
        return false;
    }

    out << std::fixed << std::setprecision(3);
    out << "type=ram_reaper\n";
    out << "name=" << stem << "\n";
    out << "enabled=" << (ram_reaper_enabled_ ? 1 : 0) << "\n";
    out << "intensity=" << ram_reaper_intensity_percent_ << "\n";
    out << "writes_per_frame=" << ram_reaper_writes_per_frame_ << "\n";
    out << "target_main_ram=" << (ram_reaper_affect_main_ram_ ? 1 : 0) << "\n";
    out << "target_vram=" << (ram_reaper_affect_vram_ ? 1 : 0) << "\n";
    out << "target_spu_ram=" << (ram_reaper_affect_spu_ram_ ? 1 : 0) << "\n";
    out << "range_start=" << ram_reaper_range_start_ << "\n";
    out << "range_end=" << ram_reaper_range_end_ << "\n";
    out << "ram_reaper(\n";
    out << "seed=" << ram_reaper_seed_ << "\n";
    out << ")\n";

    if (!out) {
        status_message_ = "Failed writing preset file.";
        return false;
    }

    refresh_corruption_preset_list();
    status_message_ = "Saved preset: " + path.filename().string();
    return true;
}

bool App::save_current_gpu_preset() {
    const std::filesystem::path dir = ensure_corruption_preset_dir();
    const std::string stem =
        sanitize_preset_file_stem(gpu_preset_name_, "gpu_reaper");
    const std::filesystem::path path = dir / (stem + ".vibe_preset");

    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        status_message_ = "Failed to create preset file.";
        return false;
    }

    out << std::fixed << std::setprecision(3);
    out << "type=gpu_reaper\n";
    out << "name=" << stem << "\n";
    out << "enabled=" << (gpu_reaper_enabled_ ? 1 : 0) << "\n";
    out << "intensity=" << gpu_reaper_intensity_percent_ << "\n";
    out << "writes_per_frame=" << gpu_reaper_writes_per_frame_ << "\n";
    out << "target_geometry=" << (gpu_reaper_affect_geometry_ ? 1 : 0) << "\n";
    out << "target_texture_state=" << (gpu_reaper_affect_texture_state_ ? 1 : 0)
        << "\n";
    out << "target_display_state=" << (gpu_reaper_affect_display_state_ ? 1 : 0)
        << "\n";
    out << "gpu_reaper(\n";
    out << "seed=" << gpu_reaper_seed_ << "\n";
    out << ")\n";

    if (!out) {
        status_message_ = "Failed writing preset file.";
        return false;
    }

    refresh_corruption_preset_list();
    status_message_ = "Saved preset: " + path.filename().string();
    return true;
}

bool App::save_current_sound_preset() {
    const std::filesystem::path dir = ensure_corruption_preset_dir();
    const std::string stem =
        sanitize_preset_file_stem(sound_preset_name_, "sound_reaper");
    const std::filesystem::path path = dir / (stem + ".vibe_preset");

    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        status_message_ = "Failed to create preset file.";
        return false;
    }

    out << std::fixed << std::setprecision(3);
    out << "type=sound_reaper\n";
    out << "name=" << stem << "\n";
    out << "enabled=" << (sound_reaper_enabled_ ? 1 : 0) << "\n";
    out << "intensity=" << sound_reaper_intensity_percent_ << "\n";
    out << "writes_per_frame=" << sound_reaper_writes_per_frame_ << "\n";
    out << "target_pitch=" << (sound_reaper_affect_pitch_ ? 1 : 0) << "\n";
    out << "target_envelope=" << (sound_reaper_affect_envelope_ ? 1 : 0) << "\n";
    out << "target_reverb=" << (sound_reaper_affect_reverb_ ? 1 : 0) << "\n";
    out << "target_mixer=" << (sound_reaper_affect_mixer_ ? 1 : 0) << "\n";
    out << "sound_reaper(\n";
    out << "seed=" << sound_reaper_seed_ << "\n";
    out << ")\n";

    if (!out) {
        status_message_ = "Failed writing preset file.";
        return false;
    }

    refresh_corruption_preset_list();
    status_message_ = "Saved preset: " + path.filename().string();
    return true;
}

bool App::load_corruption_preset(const std::filesystem::path& path) {
    ParsedCorruptionPreset preset{};
    if (!parse_corruption_preset_file(path, preset)) {
        status_message_ = "Failed to parse preset file.";
        return false;
    }

    if (preset.type == ParsedCorruptionPreset::Type::GrimSingle) {
        show_grim_reaper_ = true;
        if (preset.grim_area == "intro") {
            grim_reaper_area_index_ = 0;
        }
        else if (preset.grim_area == "charset") {
            grim_reaper_area_index_ = 1;
        }
        else if (preset.grim_area == "end") {
            grim_reaper_area_index_ = 2;
        }
        else if (preset.grim_area == "custom") {
            grim_reaper_area_index_ = kGrimReaperRangeCount - 1;
        }
        grim_reaper_random_percent_ = preset.grim_randstrike;
        grim_use_custom_seed_ = preset.grim_has_seed;
        grim_seed_ = preset.grim_seed;
        if (!preset.custom_start_hex.empty()) {
            std::snprintf(grim_reaper_custom_start_hex_,
                IM_ARRAYSIZE(grim_reaper_custom_start_hex_), "%s",
                preset.custom_start_hex.c_str());
        }
        if (!preset.custom_end_hex.empty()) {
            std::snprintf(grim_reaper_custom_end_hex_,
                IM_ARRAYSIZE(grim_reaper_custom_end_hex_), "%s",
                preset.custom_end_hex.c_str());
        }
    }
    else if (preset.type == ParsedCorruptionPreset::Type::GrimBatch) {
        show_grim_reaper_ = true;
        grim_batch_intro_enabled_ = preset.intro_enabled;
        grim_batch_charset_enabled_ = preset.charset_enabled;
        grim_batch_end_enabled_ = preset.end_enabled;
        grim_batch_intro_percent_ = preset.intro_randstrike;
        grim_batch_charset_percent_ = preset.charset_randstrike;
        grim_batch_end_percent_ = preset.end_randstrike;
        grim_batch_intro_seed_ = preset.intro_seed;
        grim_batch_charset_seed_ = preset.charset_seed;
        grim_batch_end_seed_ = preset.end_seed;
        grim_batch_use_custom_seeds_ = preset.intro_enabled || preset.charset_enabled ||
            preset.end_enabled;
    }
    else if (preset.type == ParsedCorruptionPreset::Type::RamReaper) {
        show_grim_reaper_ = true;
        ram_reaper_enabled_ = preset.ram_enabled;
        ram_reaper_intensity_percent_ = preset.ram_intensity;
        ram_reaper_writes_per_frame_ = preset.ram_writes_per_frame;
        ram_reaper_affect_main_ram_ = preset.ram_target_main;
        ram_reaper_affect_vram_ = preset.ram_target_vram;
        ram_reaper_affect_spu_ram_ = preset.ram_target_spu;
        ram_reaper_range_start_ = preset.ram_range_start;
        ram_reaper_range_end_ = preset.ram_range_end;
        ram_reaper_use_custom_seed_ = preset.ram_has_seed;
        ram_reaper_seed_ = preset.ram_seed;
        sync_ram_reaper_config();
    }
    else if (preset.type == ParsedCorruptionPreset::Type::GpuReaper) {
        show_grim_reaper_ = true;
        gpu_reaper_enabled_ = preset.gpu_enabled;
        gpu_reaper_intensity_percent_ = preset.gpu_intensity;
        gpu_reaper_writes_per_frame_ = preset.gpu_writes_per_frame;
        gpu_reaper_affect_geometry_ = preset.gpu_target_geometry;
        gpu_reaper_affect_texture_state_ = preset.gpu_target_texture;
        gpu_reaper_affect_display_state_ = preset.gpu_target_display;
        gpu_reaper_use_custom_seed_ = preset.gpu_has_seed;
        gpu_reaper_seed_ = preset.gpu_seed;
        sync_gpu_reaper_config();
    }
    else if (preset.type == ParsedCorruptionPreset::Type::SoundReaper) {
        show_grim_reaper_ = true;
        sound_reaper_enabled_ = preset.sound_enabled;
        sound_reaper_intensity_percent_ = preset.sound_intensity;
        sound_reaper_writes_per_frame_ = preset.sound_writes_per_frame;
        sound_reaper_affect_pitch_ = preset.sound_target_pitch;
        sound_reaper_affect_envelope_ = preset.sound_target_envelope;
        sound_reaper_affect_reverb_ = preset.sound_target_reverb;
        sound_reaper_affect_mixer_ = preset.sound_target_mixer;
        sound_reaper_use_custom_seed_ = preset.sound_has_seed;
        sound_reaper_seed_ = preset.sound_seed;
        sync_sound_reaper_config();
    }
    else {
        status_message_ = "Unsupported preset type.";
        return false;
    }

    status_message_ = "Loaded preset: " + path.filename().string();
    return true;
}

void App::panel_corruption_presets() {
    ImGui::SetNextWindowSize(ImVec2(520, 420), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Corruption Presets", &show_corruption_presets_)) {
        ImGui::End();
        return;
    }

    if (ImGui::Button("Refresh")) {
        refresh_corruption_preset_list();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Loads presets from ./%s", kCorruptionPresetDirName);

    ImGui::Separator();
    if (corruption_presets_.empty()) {
        ImGui::TextDisabled("No presets found.");
        ImGui::End();
        return;
    }

    if (ImGui::BeginListBox("##corruption_presets", ImVec2(-1.0f, 280.0f))) {
        for (int i = 0; i < static_cast<int>(corruption_presets_.size()); ++i) {
            const auto& preset = corruption_presets_[static_cast<size_t>(i)];
            std::string label =
                preset.display_name + " [" + preset.preset_type + "]##" + preset.file_name;
            const bool selected = (selected_corruption_preset_index_ == i);
            if (ImGui::Selectable(label.c_str(), selected)) {
                selected_corruption_preset_index_ = i;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndListBox();
    }

    if (selected_corruption_preset_index_ >= 0 &&
        selected_corruption_preset_index_ <
        static_cast<int>(corruption_presets_.size())) {
        const auto& preset =
            corruption_presets_[static_cast<size_t>(selected_corruption_preset_index_)];
        ImGui::Text("File: %s", preset.file_name.c_str());
        ImGui::Text("Type: %s", preset.preset_type.c_str());
        if (ImGui::Button("Load Selected")) {
            load_corruption_preset(preset.path);
        }
    }

    ImGui::End();
}
