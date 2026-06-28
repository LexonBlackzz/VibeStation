#include "ui/app.h"
#include "ui/panels/grim_reaper_panel.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

namespace {
    void seed_mt19937(std::mt19937& rng, u64 seed) {
        const u32 lo = static_cast<u32>(seed & 0xFFFFFFFFull);
        const u32 hi = static_cast<u32>((seed >> 32) & 0xFFFFFFFFull);
        std::seed_seq seq{ lo, hi, 0x9E3779B9u, 0x243F6A88u };
        rng.seed(seq);
    }
}

void App::set_grim_reaper_mode(bool enabled) {
    if (enabled) {
        grim_reaper_mode_active_ = true;
        if (grim_reaper_keep_console_logs_) {
            return;
        }
        if (!grim_reaper_logs_suppressed_) {
            grim_reaper_saved_log_mask_ = g_log_category_mask;
            grim_reaper_saved_log_level_ = g_log_level;
            grim_reaper_logs_suppressed_ = true;
        }
        g_log_category_mask = 0;
        g_log_level = LogLevel::Error;
        return;
    }

    grim_reaper_mode_active_ = false;
    if (grim_reaper_logs_suppressed_) {
        g_log_category_mask = grim_reaper_saved_log_mask_;
        g_log_level = grim_reaper_saved_log_level_;
        grim_reaper_logs_suppressed_ = false;
    }
}

bool App::reap_and_reboot_bios() {
    if (!system_ || bios_path_.empty()) {
        status_message_ = "Load a BIOS first.";
        return false;
    }

    std::ifstream in(bios_path_, std::ios::binary);
    if (!in.is_open()) {
        status_message_ = "Failed to open source BIOS file.";
        return false;
    }

    in.seekg(0, std::ios::end);
    const std::streamoff size_off = in.tellg();
    if (size_off <= 0) {
        status_message_ = "Source BIOS is empty or unreadable.";
        return false;
    }
    const size_t bios_size = static_cast<size_t>(size_off);
    in.seekg(0, std::ios::beg);

    std::vector<u8> bios_data(bios_size, 0);
    in.read(reinterpret_cast<char*>(bios_data.data()),
        static_cast<std::streamsize>(bios_data.size()));
    if (!in) {
        status_message_ = "Failed reading source BIOS.";
        return false;
    }

    grim_reaper_area_index_ =
        std::max(0, std::min(kGrimReaperRangeCount - 1, grim_reaper_area_index_));

    auto parse_hex = [](const char* text, size_t& out) -> bool {
        if (!text) {
            return false;
        }
        while (*text != '\0' && std::isspace(static_cast<unsigned char>(*text))) {
            ++text;
        }
        if (*text == '\0') {
            return false;
        }
        char* end_ptr = nullptr;
        const unsigned long long value = std::strtoull(text, &end_ptr, 16);
        if (end_ptr == text) {
            return false;
        }
        while (*end_ptr != '\0' && std::isspace(static_cast<unsigned char>(*end_ptr))) {
            ++end_ptr;
        }
        if (*end_ptr != '\0') {
            return false;
        }
        out = static_cast<size_t>(value);
        return true;
        };

    size_t start = 0;
    size_t end = bios_data.size() - 1u;
    std::string range_slug;

    if (grim_reaper_area_index_ == (kGrimReaperRangeCount - 1)) {
        size_t custom_start = 0;
        if (!parse_hex(grim_reaper_custom_start_hex_, custom_start)) {
            status_message_ = "Invalid custom start hex.";
            return false;
        }

        size_t custom_end = 0;
        bool has_custom_end = parse_hex(grim_reaper_custom_end_hex_, custom_end);
        if (!has_custom_end || custom_end == 0) {
            custom_end = bios_data.size() - 1u;
        }

        start = std::min(custom_start, bios_data.size() - 1u);
        end = std::min(custom_end, bios_data.size() - 1u);
        range_slug = "custom";
    }
    else {
        const GrimReaperRange range = kGrimReaperRanges[grim_reaper_area_index_];
        if (bios_data.size() <= static_cast<size_t>(range.start)) {
            status_message_ = "Selected corruption range is outside BIOS size.";
            return false;
        }
        start = static_cast<size_t>(range.start);
        end = std::min(static_cast<size_t>(range.end), bios_data.size() - 1u);
        range_slug = range.slug;
    }

    if (end < start) {
        status_message_ = "Selected corruption range is invalid.";
        return false;
    }

    const size_t span = end - start + 1u;
    const float pct_max = (grim_reaper_area_index_ == 0) ? 0.1f : 100.0f;
    const float pct = std::max(0.001f, std::min(pct_max, grim_reaper_random_percent_));
    size_t mutations =
        static_cast<size_t>((pct / 100.0f) * static_cast<float>(span));
    if (mutations == 0) {
        mutations = 1;
    }

    const u64 seed =
        grim_use_custom_seed_
        ? grim_seed_
        : ((static_cast<u64>(std::random_device{}()) << 32) ^
            static_cast<u64>(std::random_device{}()));
    grim_last_used_seed_ = seed;
    grim_seed_ = seed;
    std::mt19937 rng;
    seed_mt19937(rng, seed);

    for (size_t i = 0; i < mutations; ++i) {
        const size_t idx = start + (static_cast<size_t>(rng()) % span);
        bios_data[idx] = static_cast<u8>(rng() & 0xFFu);
    }
    const std::filesystem::path src_path = std::filesystem::path(bios_path_);
    const std::filesystem::path out_path =
        src_path.parent_path() /
        (src_path.stem().string() + "_grim_" + range_slug + src_path.extension().string());

    std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        status_message_ = "Failed to create corrupted BIOS copy.";
        return false;
    }
    out.write(reinterpret_cast<const char*>(bios_data.data()),
        static_cast<std::streamsize>(bios_data.size()));
    if (!out) {
        status_message_ = "Failed writing corrupted BIOS copy.";
        return false;
    }

    emu_runner_.pause_and_wait_idle();
    disable_ram_reaper_mode();
    disable_gpu_reaper_mode();
    disable_sound_reaper_mode();
    set_grim_reaper_mode(true);
    if (!system_->load_bios(out_path.string())) {
        set_grim_reaper_mode(false);
        status_message_ = "Failed to load corrupted BIOS copy.";
        return false;
    }

    grim_reaper_last_mutations_ = static_cast<u32>(mutations);
    grim_reaper_last_output_path_ = out_path.string();

    has_started_emulation_ = false;
    system_->reset();
    apply_memory_card_settings(false);
    has_started_emulation_ = true;
    emu_runner_.set_running(true);

    status_message_ = "Reaped BIOS copy loaded (seed " + std::to_string(seed) + "): " +
        out_path.filename().string();
    return true;
}

bool App::reap_and_reboot_bios_batch() {
    if (!system_ || bios_path_.empty()) {
        status_message_ = "Load a BIOS first.";
        return false;
    }

    const bool do_intro = grim_batch_intro_enabled_;
    const bool do_charset = grim_batch_charset_enabled_;
    const bool do_end = grim_batch_end_enabled_;
    if (!do_intro && !do_charset && !do_end) {
        status_message_ = "Select at least one batch corruption range.";
        return false;
    }

    std::ifstream in(bios_path_, std::ios::binary);
    if (!in.is_open()) {
        status_message_ = "Failed to open source BIOS file.";
        return false;
    }

    in.seekg(0, std::ios::end);
    const std::streamoff size_off = in.tellg();
    if (size_off <= 0) {
        status_message_ = "Source BIOS is empty or unreadable.";
        return false;
    }
    const size_t bios_size = static_cast<size_t>(size_off);
    in.seekg(0, std::ios::beg);

    std::vector<u8> bios_data(bios_size, 0);
    in.read(reinterpret_cast<char*>(bios_data.data()),
        static_cast<std::streamsize>(bios_data.size()));
    if (!in) {
        status_message_ = "Failed reading source BIOS.";
        return false;
    }

    size_t total_mutations = 0;
    auto next_random_seed = []() -> u64 {
        return ((static_cast<u64>(std::random_device{}()) << 32) ^
            static_cast<u64>(std::random_device{}()));
        };
    auto apply_range = [&](const GrimReaperRange& range, float percent,
        float max_percent, u64& seed_slot) -> bool {
            if (bios_data.size() <= static_cast<size_t>(range.start)) {
                return false;
            }

            const size_t start = static_cast<size_t>(range.start);
            const size_t end = std::min(static_cast<size_t>(range.end), bios_data.size() - 1u);
            if (end < start) {
                return false;
            }

            const float pct = std::max(0.001f, std::min(max_percent, percent));
            const size_t span = end - start + 1u;
            size_t mutations =
                static_cast<size_t>((pct / 100.0f) * static_cast<float>(span));
            if (mutations == 0) {
                mutations = 1;
            }

            const u64 seed = grim_batch_use_custom_seeds_ ? seed_slot : next_random_seed();
            seed_slot = seed;
            grim_last_used_seed_ = seed;
            std::mt19937 rng;
            seed_mt19937(rng, seed);
            for (size_t i = 0; i < mutations; ++i) {
                const size_t idx = start + (static_cast<size_t>(rng()) % span);
                bios_data[idx] = static_cast<u8>(rng() & 0xFFu);
            }

            total_mutations += mutations;
            return true;
        };

    std::string slug = "batch";
    if (do_intro) {
        if (!apply_range(kGrimReaperRanges[0], grim_batch_intro_percent_, 0.1f,
            grim_batch_intro_seed_)) {
            status_message_ = "Intro range is outside BIOS size.";
            return false;
        }
        slug += "_intro";
    }
    if (do_charset) {
        if (!apply_range(kGrimReaperRanges[1], grim_batch_charset_percent_, 100.0f,
            grim_batch_charset_seed_)) {
            status_message_ = "Character Sets range is outside BIOS size.";
            return false;
        }
        slug += "_charset";
    }
    if (do_end) {
        if (!apply_range(kGrimReaperRanges[2], grim_batch_end_percent_, 100.0f,
            grim_batch_end_seed_)) {
            status_message_ = "End range is outside BIOS size.";
            return false;
        }
        slug += "_end";
    }

    const std::filesystem::path src_path = std::filesystem::path(bios_path_);
    const std::filesystem::path out_path =
        src_path.parent_path() /
        (src_path.stem().string() + "_grim_" + slug + src_path.extension().string());

    std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        status_message_ = "Failed to create corrupted BIOS copy.";
        return false;
    }
    out.write(reinterpret_cast<const char*>(bios_data.data()),
        static_cast<std::streamsize>(bios_data.size()));
    if (!out) {
        status_message_ = "Failed writing corrupted BIOS copy.";
        return false;
    }

    emu_runner_.pause_and_wait_idle();
    disable_ram_reaper_mode();
    disable_gpu_reaper_mode();
    disable_sound_reaper_mode();
    set_grim_reaper_mode(true);
    if (!system_->load_bios(out_path.string())) {
        set_grim_reaper_mode(false);
        status_message_ = "Failed to load corrupted BIOS copy.";
        return false;
    }

    grim_reaper_last_mutations_ = static_cast<u32>(total_mutations);
    grim_reaper_last_output_path_ = out_path.string();

    has_started_emulation_ = false;
    system_->reset();
    apply_memory_card_settings(false);
    has_started_emulation_ = true;
    emu_runner_.set_running(true);

    status_message_ = "Batch reaped BIOS loaded (last seed " +
        std::to_string(grim_last_used_seed_) + "): " +
        out_path.filename().string();
    return true;
}
