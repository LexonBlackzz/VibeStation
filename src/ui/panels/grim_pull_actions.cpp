// Grim Reaper 2.0, Phase 5: the actions behind the New Corruption panel.
// Generation, the death watch and the library are core code (src/core/grim_*);
// this file only wires them to the running emulator.
#include "ui/app.h"

#include "core/bios.h"
#include "core/grim_share.h"
#include "platform/grim_process.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <random>

namespace {
namespace fs = std::filesystem;

u64 random_seed() {
    std::random_device rd;
    return (static_cast<u64>(rd()) << 32) ^ static_cast<u64>(rd());
}

std::string hex_name(u64 hash) {
    char b[24];
    std::snprintf(b, sizeof(b), "%016llX", static_cast<unsigned long long>(hash));
    return b;
}

double now_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

GrimPullEntry make_entry(u64 seed, const GrimGenome& genome, const GrimPullSettings& s) {
    GrimPullEntry e;
    e.seed = seed;
    e.genome = grim_genome_serialize(genome);
    e.genome_hash = grim_genome_hash(genome);
    e.families = s.families;
    e.intensity = s.intensity;
    e.rot = s.rot;
    return e;
}
} // namespace

GrimPullState& App::grim_pull_state() {
    if (!grim_pull_) {
        grim_pull_ = std::make_unique<GrimPullState>();
        const fs::path exe = fs::path(grim_self_exe_path(""));
        grim_pull_->data_dir = (exe.parent_path() / "grim_data").string();
        std::error_code ec;
        fs::create_directories(fs::path(grim_pull_->data_dir) / "maps", ec);
    }
    GrimPullState& s = *grim_pull_;
    if (!s.library_loaded) {
        s.library_loaded = true;
        std::string err;
        if (!s.library.load((fs::path(s.data_dir) / "library.json").string(), err)) {
            s.message = err;
        }
    }
    return s;
}

void App::grim_pull_save_library() {
    GrimPullState& s = grim_pull_state();
    std::string err;
    if (!s.library.save((fs::path(s.data_dir) / "library.json").string(), err)) {
        s.message = "Could not save the library: " + err;
    }
}

void App::grim_pull_start_mapping() {
    GrimPullState& s = grim_pull_state();
    if (s.map_state == GrimPullState::MapState::Running || s.bios_hash == 0 || bios_path_.empty()) {
        return;
    }
    const fs::path maps = fs::path(s.data_dir) / "maps";
    const std::string final_path = (maps / (hex_name(s.bios_hash) + ".json")).string();
    const std::string part_path = (maps / (hex_name(s.bios_hash) + ".partial.json")).string();
    const std::string log_path = (maps / (hex_name(s.bios_hash) + ".log")).string();
    s.map_path = final_path;
    s.map_state = GrimPullState::MapState::Running;
    s.map_error.clear();
    s.map_started = now_seconds();
    auto result = std::make_shared<std::atomic<int>>(-1);
    s.map_result = result;
    if (s.map_thread.joinable()) {
        s.map_thread.join();
    }
    // A child process runs the interpreter-only discovery: the live machine may be on
    // the recompiler, and the CPU mode is process-wide.
    const std::string exe = grim_self_exe_path("");
    const std::string bios = bios_path_;
    s.map_thread = std::thread([=]() {
        std::error_code ec;
        fs::remove(part_path, ec);
        fs::remove(part_path + ".words", ec);
        const GrimChildResult r = grim_run_child(exe, {"--grim-map", bios, "1800", part_path},
                                                 log_path, 600.0);
        bool ok = r.started && !r.timed_out && !r.crashed && r.exit_code == 0 &&
                  fs::exists(part_path, ec) && fs::exists(part_path + ".words", ec);
        if (ok) {
            fs::remove(final_path + ".words", ec);
            fs::rename(part_path + ".words", final_path + ".words", ec);
            if (!ec) {
                fs::remove(final_path, ec);
                fs::rename(part_path, final_path, ec);
            }
            ok = !ec;
        }
        result->store(ok ? 0 : 1, std::memory_order_release);
    });
    // Never block shutdown on a 30 s discovery run: the job owns only copies.
    s.map_thread.detach();
}

void App::grim_pull_update() {
    if (!system_) {
        return;
    }
    // Nothing runs (and no discovery job starts) until the Grim Reaper page was opened once.
    if (!grim_pull_ && !definitive_grim_reaper_active_) {
        return;
    }
    GrimPullState& s = grim_pull_state();

    // Per-BIOS context: sound-bank scan always, boot map when it exists.
    if (system_->bios_loaded() && !bios_path_.empty() && bios_path_ != s.ctx_bios_path) {
        s.ctx_bios_path = bios_path_;
        s.sample.reset();
        s.rom.reset();
        s.bios_hash = 0;
        s.map_state = GrimPullState::MapState::None;
        auto sample = std::make_unique<GrimSampleContext>();
        std::string err;
        if (sample->load(bios_path_, "", err)) {
            s.bios_hash = sample->bios_hash;
            s.bios_name = fs::path(bios_path_).stem().string();
            const std::string map_path =
                (fs::path(s.data_dir) / "maps" / (hex_name(s.bios_hash) + ".json")).string();
            std::error_code ec;
            if (fs::exists(map_path, ec) && fs::exists(map_path + ".words", ec)) {
                auto with_map = std::make_unique<GrimSampleContext>();
                auto rom = std::make_unique<GrimRomContext>();
                if (with_map->load(bios_path_, map_path, err) && rom->load(bios_path_, map_path, err)) {
                    sample = std::move(with_map);
                    s.rom = std::move(rom);
                    s.map_path = map_path;
                    s.map_state = GrimPullState::MapState::Ready;
                }
            }
            s.sample = std::move(sample);
            // No map yet, or one made before maps recorded which RAM ran code (the hardware
            // genes' survival bias needs it). Code genes keep using the old map meanwhile.
            if (s.map_state == GrimPullState::MapState::None ||
                (s.rom && s.rom->map.ram_exec_ranges.empty())) {
                grim_pull_start_mapping();
            }
        } else {
            s.message = "Grim Reaper cannot read this BIOS: " + err;
        }
    }

    // Discovery finished?
    if (s.map_state == GrimPullState::MapState::Running && s.map_result &&
        s.map_result->load(std::memory_order_acquire) >= 0) {
        if (s.map_result->load(std::memory_order_acquire) == 0) {
            auto with_map = std::make_unique<GrimSampleContext>();
            auto rom = std::make_unique<GrimRomContext>();
            std::string err;
            if (with_map->load(bios_path_, s.map_path, err) && rom->load(bios_path_, s.map_path, err)) {
                s.sample = std::move(with_map);
                s.rom = std::move(rom);
                s.map_state = GrimPullState::MapState::Ready;
            } else {
                s.map_state = GrimPullState::MapState::Failed;
                s.map_error = err;
            }
        } else {
            s.map_state = GrimPullState::MapState::Failed;
            s.map_error = "The mapping run did not finish.";
        }
    }

    // Trigger times count emulated frames: describe them in seconds at the rate the
    // machine actually runs (the BIOS switches PAL machines to 50 Hz after reset).
    if (s.has_machine) {
        const u32 fps = static_cast<u32>(system_->target_fps() + 0.5);
        if (fps != s.lines_fps) {
            GrimPullContext ctx;
            ctx.rom = s.rom.get();
            ctx.sample = s.sample.get();
            ctx.bios_hash = s.bios_hash;
            ctx.frame_rate = fps;
            s.lines = grim_pull_describe(s.genome, ctx);
            s.lines_fps = fps;
        }
    }

    // Death watch: read the verdict, record it, and let Mercy replace early deaths.
    if (s.watch) {
        s.status = s.watch->status();
        if (s.status.dead && !s.outcome_saved) {
            s.outcome_saved = true;
            s.library.update(s.pull, true, s.status.death.reason, s.status.death.headline,
                             s.status.death.seconds);
            grim_pull_save_library();
            // At most 25 silent rerolls in a row, so a hopeless setting cannot spin forever.
            if (grim_mercy_should_reroll(s.mercy, s.status) && s.rerolls < 25) {
                ++s.rerolls;
                s.library.mark_mercy(s.pull);
                grim_pull_save_library();
                grim_pull_new();
            } else {
                s.rerolls = 0; // this death is shown
            }
        } else if (!s.status.dead && s.status.frames > kGrimMercyWindowFrames) {
            s.rerolls = 0; // survived the window: shown
        }
    }
}

bool App::grim_pull_boot(const GrimGenome& genome, u64 pull_number) {
    GrimPullState& s = grim_pull_state();
    if (!system_ || !system_->bios_loaded() || bios_path_.empty()) {
        s.message = "Load a BIOS first.";
        return false;
    }
    std::string err;
    if (!grim_pull_compatible(genome, s.bios_hash, err)) {
        s.message = err;
        return false;
    }

    emu_runner_.pause_and_wait_idle();
    grim_pull_release(); // the previous machine, if any
    disable_ram_reaper_mode();
    disable_gpu_reaper_mode();
    disable_sound_reaper_mode();
    set_grim_reaper_mode(true);

    // Stock BIOS first: ROM genes check every original word before patching.
    if (!system_->load_bios(bios_path_)) {
        s.message = "Failed to reload the BIOS.";
        emu_runner_.set_running(true);
        return false;
    }
    auto runtime = std::make_unique<GrimGenomeRuntime>(genome);
    system_->set_grim_genome(runtime.get());
    s.runtime = std::move(runtime); // the old runtime is no longer referenced
    has_started_emulation_ = false;
    session_suspended_ = false;
    system_->reset(); // replays the ROM genes and rewinds the interface genes
    apply_memory_card_settings(false);
    has_started_emulation_ = true;
    if (!system_->grim_rom_error().empty()) {
        s.message = "ROM genes were not applied: " + system_->grim_rom_error();
        system_->set_grim_genome(nullptr);
        s.runtime.reset();
        s.has_machine = false;
        emu_runner_.set_running(true);
        return false;
    }

    s.genome = genome;
    s.pull = pull_number;
    s.machine_id = grim_machine_id(genome);
    GrimPullContext ctx;
    ctx.rom = s.rom.get();
    ctx.sample = s.sample.get();
    ctx.bios_hash = s.bios_hash;
    s.lines = grim_pull_describe(genome, ctx);
    s.lines_fps = 60;
    s.has_machine = true;
    s.outcome_saved = false;
    s.status = GrimLiveStatus{};
    s.message.clear();

    s.watch = std::make_unique<GrimLiveWatch>(&s.genome, grim_live_config(system_->disc_loaded()));
    s.watch->attach(*system_);
    emu_runner_.set_grim_watch(s.watch.get());
    emu_runner_.set_running(true);
    status_message_ = "Pull " + s.machine_id + " booted.";
    return true;
}

bool App::grim_pull_new() {
    GrimPullState& s = grim_pull_state();
    GrimPullContext ctx;
    ctx.rom = s.rom.get();
    ctx.sample = s.sample.get();
    ctx.bios_hash = s.bios_hash;
    if ((s.settings.families & grim_pull_available_families(ctx)) == 0) {
        s.message = s.settings.families == 0
                        ? "Switch on at least one gene family."
                        : "The families you picked are not available yet.";
        return false;
    }
    const u64 seed = random_seed();
    const GrimGenome genome = grim_pull_generate(seed, s.settings, ctx, &s.history);
    if (genome.genes.empty()) {
        s.message = "Nothing came out of that pull. Try again.";
        return false;
    }
    s.history.note(genome);
    const u64 pull = s.library.add(make_entry(seed, genome, s.settings));
    grim_pull_save_library();
    if (!grim_pull_boot(genome, pull)) {
        s.library.update(pull, true, "boot_failed", "Did not boot", 0.0);
        grim_pull_save_library();
        return false;
    }
    return true;
}

bool App::grim_pull_boot_entry(u64 pull_number) {
    GrimPullState& s = grim_pull_state();
    const GrimPullEntry* e = s.library.find(pull_number);
    if (e == nullptr) {
        s.message = "That pull is gone from the history.";
        return false;
    }
    GrimGenome genome;
    std::string err;
    if (!grim_genome_parse(e->genome, genome, err)) {
        s.message = "Cannot read that genome: " + err;
        return false;
    }
    return grim_pull_boot(genome, pull_number);
}

bool App::grim_pull_paste_code(const std::string& text) {
    GrimPullState& s = grim_pull_state();
    GrimGenome genome;
    std::string err;
    if (!grim_share_parse(text, genome, err)) {
        s.message = "Cannot read that code: " + err + ".";
        return false;
    }
    if (genome.genes.empty()) {
        s.message = "That code has no genes.";
        return false;
    }
    // Seed 0 marks a pasted machine: it was not pulled here.
    const u64 pull = s.library.add(make_entry(0, genome, s.settings));
    grim_pull_save_library();
    if (!grim_pull_boot(genome, pull)) {
        s.library.update(pull, true, "boot_failed", "Did not boot", 0.0);
        grim_pull_save_library();
        return false;
    }
    status_message_ = "Pasted machine " + s.machine_id + " booted.";
    return true;
}

void App::grim_pull_keep(u64 pull_number, bool keep) {
    GrimPullState& s = grim_pull_state();
    if (s.library.set_kept(pull_number, keep)) {
        grim_pull_save_library();
    }
}

void App::stop_all_corruption() {
    disable_ram_reaper_mode();
    disable_gpu_reaper_mode();
    disable_sound_reaper_mode();
    grim_pull_release();
}

// Tears down the New Corruption machine (records how it ended, detaches the watch and
// the genome). The next System::reset() restores the stock BIOS image. Call with the
// emulator paused.
void App::grim_pull_release() {
    if (!grim_pull_ || !grim_pull_->has_machine) {
        return;
    }
    GrimPullState& s = *grim_pull_;
    emu_runner_.set_grim_watch(nullptr);
    if (s.watch) {
        if (!s.outcome_saved) {
            const GrimLiveStatus last = s.watch->status();
            s.library.update(s.pull, last.dead, last.death.reason, last.death.headline, last.seconds);
            grim_pull_save_library();
        }
        if (system_) {
            s.watch->detach(*system_);
        }
        s.watch.reset();
    }
    if (system_) {
        system_->set_grim_genome(nullptr);
    }
    s.runtime.reset();
    s.has_machine = false;
    s.status = GrimLiveStatus{};
    s.rerolls = 0;
    set_grim_reaper_mode(false);
}

void App::grim_pull_clean_machine() {
    if (!system_ || !system_->bios_loaded()) {
        return;
    }
    // Both paths call stop_all_corruption(), which releases the pull.
    if (system_->disc_loaded() ? boot_disc_from_ui() : start_bios_from_ui()) {
        status_message_ = "Clean machine booted.";
    }
}

void App::grim_pull_shutdown() {
    if (!grim_pull_) {
        return;
    }
    emu_runner_.pause_and_wait_idle();
    grim_pull_release();
}
