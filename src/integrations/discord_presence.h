#pragma once

#include <cstdint>
#include <memory>
#include <string>

struct DiscordPresenceActivity {
    bool emulation_started = false;
    bool emulation_running = false;
    bool turbo_active = false;
    bool slowdown_active = false;
    bool bios_loaded = false;
    bool disc_selected = false;
    std::string content_name;
};

class DiscordPresence {
public:
    DiscordPresence();
    ~DiscordPresence();

    DiscordPresence(const DiscordPresence&) = delete;
    DiscordPresence& operator=(const DiscordPresence&) = delete;

    bool sdk_compiled() const;
    bool prepare_runtime_dependency();
    void configure(bool enabled, std::uint64_t application_id);
    void tick();
    void update_activity(const DiscordPresenceActivity& activity);
    std::uint64_t application_id() const;
    bool enabled() const;
    const std::string& status_text() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
