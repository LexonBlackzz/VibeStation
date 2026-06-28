#include "ui/app.h"
#include <cctype>
#include <filesystem>
#include <string>

namespace {
    constexpr u64 kDiscordApplicationId = 1481706201375838279ull;
}

void App::sync_discord_presence_config() {
    if (!discord_presence_) {
        return;
    }
    discord_presence_->prepare_runtime_dependency();
    discord_presence_->configure(
        config_discord_rich_presence_,
        kDiscordApplicationId);
}

std::string App::current_presence_content_name() const {
    auto prettify_content_name = [](std::string value) {
        for (char& ch : value) {
            if (ch == '_' || ch == '-') {
                ch = ' ';
            }
        }

        std::string compact;
        compact.reserve(value.size());
        bool previous_space = false;
        for (char ch : value) {
            const bool is_space = std::isspace(static_cast<unsigned char>(ch)) != 0;
            if (is_space) {
                if (!compact.empty() && !previous_space) {
                    compact.push_back(' ');
                }
            }
            else {
                compact.push_back(ch);
            }
            previous_space = is_space;
        }
        while (!compact.empty() && compact.back() == ' ') {
            compact.pop_back();
        }
        return compact;
        };

    if (!game_cue_path_.empty()) {
        return prettify_content_name(std::filesystem::path(game_cue_path_).stem().string());
    }
    if (!game_bin_path_.empty()) {
        return prettify_content_name(std::filesystem::path(game_bin_path_).stem().string());
    }
    if (system_ && system_->disc_loaded()) {
        return "Unknown game";
    }
    if (system_ && system_->bios_loaded()) {
        return "PlayStation BIOS";
    }
    return {};
}

void App::update_discord_presence() {
    if (!discord_presence_) {
        return;
    }

    DiscordPresenceActivity activity{};
    activity.emulation_started = has_started_emulation_;
    activity.emulation_running = has_started_emulation_ && emu_runner_.is_running();
    activity.turbo_active = turbo_hold_active_;
    activity.slowdown_active = slowdown_hold_active_;
    activity.bios_loaded = system_ && system_->bios_loaded();
    activity.disc_selected =
        !game_bin_path_.empty() || !game_cue_path_.empty() ||
        (system_ && system_->disc_loaded());
    activity.content_name = current_presence_content_name();
    discord_presence_->update_activity(activity);
}
