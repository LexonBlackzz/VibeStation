#include "discord_presence.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

#if defined(VIBESTATION_HAS_DISCORD_SDK)
#define DISCORDPP_IMPLEMENTATION
#include <discordpp.h>
#endif

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace {

std::string trim_presence_text(const std::string& text, size_t max_len) {
    if (text.size() <= max_len) {
        return text;
    }
    if (max_len <= 3) {
        return text.substr(0, max_len);
    }
    return text.substr(0, max_len - 3) + "...";
}

bool same_activity(const DiscordPresenceActivity& a, const DiscordPresenceActivity& b) {
    return a.emulation_started == b.emulation_started &&
        a.emulation_running == b.emulation_running &&
        a.turbo_active == b.turbo_active &&
        a.slowdown_active == b.slowdown_active &&
        a.bios_loaded == b.bios_loaded &&
        a.disc_selected == b.disc_selected &&
        a.content_name == b.content_name;
}

std::string build_presence_details(const DiscordPresenceActivity& activity) {
    if (!activity.emulation_started) {
        return "In launcher";
    }

    if (activity.disc_selected) {
        return "Playing a game";
    }
    return "In PlayStation BIOS";
}

std::optional<std::string> build_presence_state(const DiscordPresenceActivity& activity) {
    if (!activity.emulation_started) {
        return std::string("Idling");
    }

    if (activity.disc_selected) {
        if (!activity.content_name.empty()) {
            return trim_presence_text(activity.content_name, 128);
        }
        return std::string("Unknown game");
    }

    return std::nullopt;
}

std::uint64_t unix_epoch_millis(std::time_t t) {
    return static_cast<std::uint64_t>(std::max<std::time_t>(0, t)) * 1000ull;
}

}

struct DiscordPresence::Impl {
    bool enabled = false;
    std::uint64_t application_id = 0;
    std::string status_text = "Discord Rich Presence disabled";
    DiscordPresenceActivity requested_activity{};
    bool has_requested_activity = false;
    std::time_t session_started_at = 0;
#if defined(_WIN32) && defined(VIBESTATION_HAS_EMBEDDED_DISCORD_DLL)
    bool runtime_ready = false;
#else
    bool runtime_ready = true;
#endif

#if defined(VIBESTATION_HAS_DISCORD_SDK)
    std::unique_ptr<discordpp::Client> client{};
    bool publish_dirty = false;
    bool publish_in_flight = false;
    std::chrono::steady_clock::time_point next_timer_refresh_at =
        std::chrono::steady_clock::time_point::min();
    std::chrono::steady_clock::time_point next_publish_at =
        std::chrono::steady_clock::time_point::min();

    void reset_client(const std::string& reason) {
        client.reset();
        publish_dirty = false;
        publish_in_flight = false;
        next_timer_refresh_at = std::chrono::steady_clock::time_point::min();
        next_publish_at = std::chrono::steady_clock::time_point::min();
        status_text = reason;
    }

    void ensure_client() {
        if (!enabled) {
            reset_client("Discord Rich Presence disabled");
            return;
        }
        if (!runtime_ready) {
            status_text = "Discord SDK runtime extraction failed";
            return;
        }
        if (application_id == 0) {
            reset_client("Set a Discord Application ID");
            return;
        }
        if (client) {
            return;
        }

        client = std::make_unique<discordpp::Client>();
        client->SetApplicationId(application_id);
        publish_dirty = true;
        next_publish_at = std::chrono::steady_clock::now();
        next_timer_refresh_at = next_publish_at + std::chrono::seconds(10);
        status_text = "Discord Social SDK ready";
    }

    void publish_if_needed() {
        ensure_client();
        if (!client || !has_requested_activity || !publish_dirty || publish_in_flight) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now < next_publish_at) {
            return;
        }

        discordpp::Activity activity;
        activity.SetType(discordpp::ActivityTypes::Playing);
        activity.SetDetails(build_presence_details(requested_activity));
        activity.SetState(build_presence_state(requested_activity));

        if (requested_activity.emulation_started && session_started_at > 0) {
            discordpp::ActivityTimestamps timestamps;
            timestamps.SetStart(unix_epoch_millis(session_started_at));
            activity.SetTimestamps(timestamps);
        }

        publish_in_flight = true;
        client->UpdateRichPresence(activity, [this](discordpp::ClientResult result) {
            publish_in_flight = false;
            next_publish_at = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            if (result.Successful()) {
                publish_dirty = false;
                next_timer_refresh_at =
                    std::chrono::steady_clock::now() + std::chrono::seconds(10);
                status_text = "Discord Rich Presence active";
            }
            else {
                status_text =
                    "Discord Rich Presence error: " + trim_presence_text(result.ToString(), 160);
            }
            });
    }
#endif
};

DiscordPresence::DiscordPresence() : impl_(std::make_unique<Impl>()) {}

DiscordPresence::~DiscordPresence() = default;

bool DiscordPresence::sdk_compiled() const {
#if defined(VIBESTATION_HAS_DISCORD_SDK)
    return true;
#else
    return false;
#endif
}

bool DiscordPresence::prepare_runtime_dependency() {
#if defined(_WIN32) && defined(VIBESTATION_HAS_EMBEDDED_DISCORD_DLL)
    if (impl_->runtime_ready) {
        return true;
    }

    HRSRC resource =
        FindResourceW(nullptr, L"DISCORD_PARTNER_SDK_DLL", MAKEINTRESOURCEW(10));
    if (resource == nullptr) {
        impl_->status_text = "Embedded Discord SDK resource not found";
        return false;
    }

    HGLOBAL loaded_resource = LoadResource(nullptr, resource);
    if (loaded_resource == nullptr) {
        impl_->status_text = "Failed to load embedded Discord SDK resource";
        return false;
    }

    const DWORD resource_size = SizeofResource(nullptr, resource);
    const void* resource_bytes = LockResource(loaded_resource);
    if (resource_bytes == nullptr || resource_size == 0) {
        impl_->status_text = "Embedded Discord SDK resource is invalid";
        return false;
    }

    wchar_t module_path[MAX_PATH] = {};
    const DWORD module_len = GetModuleFileNameW(nullptr, module_path, MAX_PATH);
    if (module_len == 0 || module_len >= MAX_PATH) {
        impl_->status_text = "Failed to resolve executable path for Discord SDK extraction";
        return false;
    }

    const std::filesystem::path output_path =
        std::filesystem::path(module_path).parent_path() / "discord_partner_sdk.dll";

    bool needs_write = true;
    std::error_code ec;
    if (std::filesystem::exists(output_path, ec) &&
        std::filesystem::file_size(output_path, ec) == resource_size) {
        std::ifstream existing(output_path, std::ios::binary);
        if (existing.is_open()) {
            std::vector<char> existing_bytes(static_cast<size_t>(resource_size));
            existing.read(existing_bytes.data(),
                static_cast<std::streamsize>(existing_bytes.size()));
            if (existing.good() || existing.eof()) {
                const size_t bytes_read = static_cast<size_t>(existing.gcount());
                if (bytes_read == existing_bytes.size() &&
                    std::memcmp(existing_bytes.data(), resource_bytes, existing_bytes.size()) == 0) {
                    needs_write = false;
                }
            }
        }
    }

    if (needs_write) {
        std::ofstream out(output_path, std::ios::binary | std::ios::trunc);
        if (!out.is_open()) {
            impl_->status_text = "Failed to write extracted Discord SDK DLL";
            return false;
        }
        out.write(static_cast<const char*>(resource_bytes),
            static_cast<std::streamsize>(resource_size));
        if (!out.good()) {
            impl_->status_text = "Failed to finish writing extracted Discord SDK DLL";
            return false;
        }
    }

    HMODULE loaded_module = LoadLibraryW(output_path.c_str());
    if (loaded_module == nullptr) {
        impl_->status_text = "Failed to load extracted Discord SDK DLL";
        return false;
    }

    impl_->runtime_ready = true;
    if (!impl_->enabled) {
        impl_->status_text = "Discord Rich Presence disabled";
    }
    return true;
#else
    impl_->runtime_ready = true;
    return true;
#endif
}

void DiscordPresence::configure(bool enabled, std::uint64_t application_id) {
    if (impl_->enabled == enabled && impl_->application_id == application_id) {
#if defined(VIBESTATION_HAS_DISCORD_SDK)
        impl_->publish_if_needed();
#endif
        return;
    }

    impl_->enabled = enabled;
    impl_->application_id = application_id;

#if defined(VIBESTATION_HAS_DISCORD_SDK)
    impl_->reset_client(enabled ? "Discord Rich Presence reconfiguring" :
        "Discord Rich Presence disabled");
    if (enabled && application_id != 0) {
        impl_->ensure_client();
    }
    else if (enabled) {
        impl_->status_text = "Set a Discord Application ID";
    }
#else
    if (!enabled) {
        impl_->status_text = "Discord Rich Presence disabled";
    }
    else if (application_id == 0) {
        impl_->status_text = "Set a Discord Application ID";
    }
    else {
        impl_->status_text = "This build was compiled without Discord Social SDK support";
    }
#endif
}

void DiscordPresence::tick() {
#if defined(VIBESTATION_HAS_DISCORD_SDK)
    if (impl_->client) {
        discordpp::RunCallbacks();
        if (impl_->enabled && impl_->has_requested_activity &&
            impl_->requested_activity.emulation_started &&
            !impl_->publish_in_flight &&
            std::chrono::steady_clock::now() >= impl_->next_timer_refresh_at) {
            impl_->publish_dirty = true;
        }
        impl_->publish_if_needed();
    }
#endif
}

void DiscordPresence::update_activity(const DiscordPresenceActivity& activity) {
    if (impl_->has_requested_activity && same_activity(impl_->requested_activity, activity)) {
        return;
    }

    if (activity.emulation_started) {
        if (!impl_->requested_activity.emulation_started || impl_->session_started_at == 0) {
            impl_->session_started_at = std::time(nullptr);
        }
    }
    else {
        impl_->session_started_at = 0;
    }

    impl_->requested_activity = activity;
    impl_->has_requested_activity = true;

#if defined(VIBESTATION_HAS_DISCORD_SDK)
    impl_->publish_dirty = impl_->enabled && (impl_->application_id != 0);
    impl_->publish_if_needed();
#endif
}

std::uint64_t DiscordPresence::application_id() const {
    return impl_->application_id;
}

bool DiscordPresence::enabled() const {
    return impl_->enabled;
}

const std::string& DiscordPresence::status_text() const {
    return impl_->status_text;
}
