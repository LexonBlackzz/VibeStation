#include "grim_pull_test.h"
#include "grim_eval_runner.h"
#include "core/bios.h"
#include "core/grim_audibility.h"
#include "core/grim_eval.h"
#include "core/grim_genome.h"
#include "core/grim_library.h"
#include "core/grim_live.h"
#include "core/grim_pull.h"
#include "core/grim_rom.h"
#include "core/grim_sample.h"
#include "core/system.h"
#include "platform/grim_process.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <thread>

namespace {
int failures = 0;
void check(bool ok, const std::string &name, const std::string &detail = "") {
  std::printf("GRIM_PULL_TEST %s %s %s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.c_str());
  std::fflush(stdout);
  failures += ok ? 0 : 1;
}

struct Options {
  std::string bios;
  std::string map = "docs/grim-reaper/maps/scph1001_phase4_nodisc_1800.json";
  std::string backend = "interpreter";
  u32 frames = 420;
  u32 pulls = 40;
  u32 threads = 6;
  std::vector<u32> intensities{20, 50, 80};
  u32 families = kGrimFamilyAudio | kGrimFamilyCode | kGrimFamilyInterface;
  std::string out_dir;
};

bool parse_options(std::vector<std::string> args, Options &o) {
  o.bios = grim_take_bios_arg(args);
  for (size_t i = 0; i < args.size(); ++i) {
    const bool has = i + 1 < args.size();
    if (args[i] == "--map" && has) {
      o.map = args[++i];
    } else if (args[i] == "--frames" && has) {
      o.frames = static_cast<u32>(std::max(1, std::atoi(args[++i].c_str())));
    } else if (args[i] == "--backend" && has) {
      o.backend = args[++i];
    } else if (args[i] == "--pulls" && has) {
      o.pulls = static_cast<u32>(std::max(1, std::atoi(args[++i].c_str())));
    } else if (args[i] == "--threads" && has) {
      o.threads = static_cast<u32>(std::max(1, std::atoi(args[++i].c_str())));
    } else if (args[i] == "--intensity" && has) {
      o.intensities.clear();
      const std::string v = args[++i];
      for (size_t p = 0; p < v.size();) {
        o.intensities.push_back(static_cast<u32>(std::atoi(v.c_str() + p)));
        p = v.find(',', p) == std::string::npos ? v.size() : v.find(',', p) + 1u;
      }
    } else if (args[i] == "--out-dir" && has) {
      o.out_dir = args[++i];
    } else if (args[i] == "--families" && has) {
      o.families = 0;
      const std::string v = args[++i];
      if (v.find("audio") != std::string::npos) o.families |= kGrimFamilyAudio;
      if (v.find("code") != std::string::npos) o.families |= kGrimFamilyCode;
      if (v.find("iface") != std::string::npos) o.families |= kGrimFamilyInterface;
    } else {
      std::fprintf(stderr, "GRIM_PULL_ERROR unknown option %s\n", args[i].c_str());
      return false;
    }
  }
  return !o.bios.empty();
}

void set_backend(const std::string &name) {
  g_cpu_execution_mode_cli_override = true;
  g_cpu_execution_mode_cli_value =
      name == "recompiler" ? CpuExecutionMode::Recompiler : CpuExecutionMode::Interpreter;
}

// ---- one machine, driven the way the GUI drives it ---------------------------------------

struct LiveRun {
  std::string rom_error;
  GrimLiveStatus status;
  u32 frames_run = 0;
};

LiveRun run_live(const Options &o, const GrimGenome *genome, u32 frames, bool game_disc = false) {
  LiveRun r;
  auto sys = std::make_unique<System>();
  if (!sys->load_bios(o.bios)) {
    r.rom_error = "bios_load_failed";
    return r;
  }
  std::unique_ptr<GrimGenomeRuntime> rt;
  if (genome != nullptr) {
    rt = std::make_unique<GrimGenomeRuntime>(*genome);
    sys->set_grim_genome(rt.get());
  }
  sys->reset(); // replays the ROM genes onto the stock image, like a GUI reboot
  r.rom_error = sys->grim_rom_error();
  GrimLiveWatch watch(genome, grim_live_config(game_disc));
  watch.attach(*sys);
  for (u32 i = 0; i < frames; ++i) {
    sys->run_frame(false, false);
    watch.on_frame(*sys);
    ++r.frames_run;
    if (watch.status().dead) {
      break;
    }
  }
  r.status = watch.status();
  watch.detach(*sys);
  sys->set_grim_genome(nullptr);
  return r;
}

void print_live(const std::string &name, const Options &o, const LiveRun &r) {
  std::printf("GRIM_PULL_LIVE %s backend=%s dead=%d frame=%u reason=%s seconds=%.3f silent=%d\n",
              name.c_str(), o.backend.c_str(), r.status.dead ? 1 : 0, r.status.death.frame,
              r.status.dead ? r.status.death.reason.c_str() : "alive", r.status.seconds,
              r.status.silent ? 1 : 0);
}

GrimGene rom_gene(const std::vector<u32> &stock, const std::vector<std::pair<u32, u32>> &patches) {
  GrimGene g = grim_default_gene(GrimGeneType::RomCode);
  for (const auto &[offset, word] : patches) {
    GrimRomPatch p;
    p.offset = offset;
    p.original = stock[offset / 4u];
    p.mutated = word;
    g.patches.push_back(p);
  }
  return g;
}

GrimGenome rom_genome(u64 bios_hash, std::vector<GrimGene> genes) {
  GrimGenome g;
  g.version = 2;
  g.bios_hash = bios_hash;
  g.genes = std::move(genes);
  return g;
}

// ---- pure tests --------------------------------------------------------------------------

void plan_tests() {
  const GrimPullPlan low = grim_pull_plan(0), mid = grim_pull_plan(58), high = grim_pull_plan(100);
  check(grim_pull_readout(58) == "4-7 genes \xC2\xB7 risky", "plan_readout_58", grim_pull_readout(58));
  check(low.min_genes == 1 && low.max_genes == 2 && std::string(low.risk_label) == "safe",
        "plan_low");
  check(high.min_genes == 7 && high.max_genes == 11 && std::string(high.risk_label) == "lethal",
        "plan_high");
  bool monotonic = true;
  for (u32 t = 1; t <= 100; ++t) {
    const GrimPullPlan a = grim_pull_plan(t - 1), b = grim_pull_plan(t);
    monotonic = monotonic && b.min_genes >= a.min_genes && b.max_genes >= a.max_genes &&
                b.risk_q10 >= a.risk_q10 && b.rom_early_ms <= a.rom_early_ms &&
                b.rom_curve <= a.rom_curve && b.rom_patches_max >= a.rom_patches_max;
  }
  check(monotonic, "plan_monotonic");
  check(low.rom_curve > high.rom_curve && low.rom_early_ms > high.rom_early_ms && !low.rom_call_swap &&
            high.rom_call_swap && mid.risk_q10 > low.risk_q10,
        "plan_survival_biases_weaken");
  check(grim_pull_plan(1000).max_genes == high.max_genes, "plan_clamps");
}

u32 gene_family(const GrimGene &g) {
  return g.type == GrimGeneType::SpuSample ? kGrimFamilyAudio
         : g.type == GrimGeneType::RomCode ? kGrimFamilyCode
                                           : kGrimFamilyInterface;
}

void generation_tests(const GrimPullContext &ctx, const GrimPullContext &no_rom) {
  const GrimPullSettings base;
  // Determinism, and variety between seeds.
  std::set<u64> hashes;
  bool repeat_ok = true;
  for (u64 seed = 1; seed <= 60; ++seed) {
    const GrimGenome a = grim_pull_generate(seed, base, ctx, nullptr);
    const GrimGenome b = grim_pull_generate(seed, base, ctx, nullptr);
    repeat_ok = repeat_ok && grim_genome_serialize(a) == grim_genome_serialize(b);
    hashes.insert(grim_genome_hash(a));
  }
  check(repeat_ok, "generate_same_seed_same_genome");
  check(hashes.size() >= 55, "generate_seeds_vary", std::to_string(hashes.size()) + "/60 distinct");

  // Family toggles: only enabled families appear; Visual alone makes nothing.
  bool toggles_ok = true;
  std::string toggle_detail;
  for (u32 mask = 1; mask <= 15; ++mask) {
    for (u32 t : {10u, 50u, 90u}) {
      for (u64 seed = 100; seed < 112; ++seed) {
        GrimPullSettings s;
        s.families = mask;
        s.intensity = t;
        const GrimGenome g = grim_pull_generate(seed, s, ctx, nullptr);
        for (const GrimGene &gene : g.genes) {
          if ((gene_family(gene) & mask) == 0) {
            toggles_ok = false;
            toggle_detail = "mask " + std::to_string(mask) + " produced foreign gene";
          }
        }
        if ((mask & ~kGrimFamilyVisual) == 0 && !g.genes.empty()) {
          toggles_ok = false;
          toggle_detail = "visual-only made genes";
        }
      }
    }
  }
  check(toggles_ok, "families_only_enabled_ones", toggle_detail);
  // Without the map there is no Code family, even when asked for.
  bool no_code = true;
  for (u64 seed = 1; seed <= 30; ++seed) {
    GrimPullSettings s;
    s.families = kGrimFamilyCode | kGrimFamilyInterface;
    for (const GrimGene &g : grim_pull_generate(seed, s, no_rom, nullptr).genes) {
      no_code = no_code && g.type != GrimGeneType::RomCode;
    }
  }
  check(no_code, "code_family_needs_map");
  GrimPullSettings code_only;
  code_only.families = kGrimFamilyCode;
  check(grim_pull_generate(5, code_only, no_rom, nullptr).genes.empty(), "unavailable_family_makes_nothing");
  check((grim_pull_available_families(no_rom) & kGrimFamilyCode) == 0 &&
            (grim_pull_available_families(ctx) & kGrimFamilyCode) != 0,
        "available_families");

  // Risk bounds: counts within the plan, every genome valid text that round-trips.
  bool bounds_ok = true, roundtrip_ok = true;
  std::string bounds_detail;
  double mean_low = 0, mean_high = 0;
  for (u32 t : {0u, 25u, 50u, 75u, 100u}) {
    const GrimPullPlan plan = grim_pull_plan(t);
    for (u64 seed = 1; seed <= 40; ++seed) {
      GrimPullSettings s;
      s.intensity = t;
      const GrimGenome g = grim_pull_generate(seed * 7919u + t, s, ctx, nullptr);
      const u32 n = static_cast<u32>(g.genes.size());
      if (n < 1 || n > std::max(plan.max_genes, 1u)) {
        bounds_ok = false;
        bounds_detail = "t=" + std::to_string(t) + " genes=" + std::to_string(n);
      }
      GrimGenome parsed;
      std::string err;
      roundtrip_ok = roundtrip_ok && grim_genome_parse(grim_genome_serialize(g), parsed, err) &&
                     grim_genome_hash(parsed) == grim_genome_hash(g);
      (t == 0 ? mean_low : mean_high) += t == 0 || t == 100 ? n : 0;
    }
  }
  check(bounds_ok, "gene_count_within_plan", bounds_detail);
  check(roundtrip_ok, "generated_genomes_round_trip");
  check(mean_high / 40.0 > mean_low / 40.0 + 3.0, "high_intensity_means_more_genes",
        std::to_string(mean_low / 40.0) + " vs " + std::to_string(mean_high / 40.0));

  // Rot: every interface gene is a rot ramp that starts healthy.
  bool rot_ok = true;
  u32 iface_seen = 0;
  for (u64 seed = 1; seed <= 40; ++seed) {
    GrimPullSettings s;
    s.families = kGrimFamilyInterface;
    s.rot = true;
    for (const GrimGene &g : grim_pull_generate(seed, s, ctx, nullptr).genes) {
      ++iface_seen;
      rot_ok = rot_ok && g.trigger.kind == GrimTriggerKind::Rot && g.trigger.start_frame >= 120 &&
               g.trigger.end_frame > g.trigger.start_frame;
    }
  }
  check(rot_ok && iface_seen > 0, "rot_mode_ramps_every_interface_gene");
  // Without rot the survivable mix of trigger kinds is untouched.
  std::set<int> kinds;
  for (u64 seed = 1; seed <= 40; ++seed) {
    GrimPullSettings s;
    s.families = kGrimFamilyInterface;
    for (const GrimGene &g : grim_pull_generate(seed, s, ctx, nullptr).genes) {
      kinds.insert(static_cast<int>(g.trigger.kind));
    }
  }
  check(kinds.size() >= 3, "no_rot_keeps_trigger_variety");

  // Risk widens interface magnitudes: mean distance from the schema default grows.
  const auto spread = [&](u32 t) {
    double total = 0;
    u32 n = 0;
    for (u64 seed = 1; seed <= 80; ++seed) {
      GrimPullSettings s;
      s.families = kGrimFamilyInterface;
      s.intensity = t;
      for (const GrimGene &g : grim_pull_generate(seed, s, ctx, nullptr).genes) {
        const GrimGene def = grim_default_gene(g.type);
        const auto &schema = grim_gene_schema(g.type);
        for (size_t i = 0; i < schema.size(); ++i) {
          const double range = std::max(1, schema[i].hi - schema[i].lo);
          total += std::abs(g.params[i] - def.params[i]) / range;
          ++n;
        }
      }
    }
    return n ? total / n : 0.0;
  };
  const double low_spread = spread(5), high_spread = spread(100);
  check(high_spread > low_spread * 1.15, "risk_widens_interface_magnitudes",
        std::to_string(low_spread) + " -> " + std::to_string(high_spread));

  // Code genes avoid early init at low intensity (the survival bias) and may reach it at high.
  const auto earliest_ms = [&](u32 t) {
    double earliest = 1e9;
    for (u64 seed = 1; seed <= 60; ++seed) {
      GrimPullSettings s;
      s.families = kGrimFamilyCode;
      s.intensity = t;
      for (const GrimGene &g : grim_pull_generate(seed, s, ctx, nullptr).genes) {
        for (const GrimRomPatch &p : g.patches) {
          const u32 w = p.offset / 4u;
          if (w < ctx.rom->map.words.size() && ctx.rom->map.words[w].first_exec != kGrimNever) {
            earliest = std::min(earliest, grim_cycles_to_ms(ctx.rom->map.words[w].first_exec));
          }
        }
      }
    }
    return earliest;
  };
  const double low_early = earliest_ms(5), high_early = earliest_ms(100);
  check(low_early >= 500.0 && high_early < low_early, "code_genes_respect_early_init_bias",
        std::to_string(low_early) + " ms vs " + std::to_string(high_early) + " ms");

  // Novelty steers which candidate is made, never whether one is shown.
  GrimPullHistory history;
  u64 with_total = 0, without_total = 0;
  for (u64 seed = 1; seed <= 40; ++seed) {
    const GrimGenome plain = grim_pull_generate(seed, base, ctx, nullptr);
    const GrimGenome steered = grim_pull_generate(seed, base, ctx, &history);
    without_total += history.similarity(plain);
    with_total += history.similarity(steered);
    history.note(steered);
    if (steered.genes.empty()) {
      check(false, "novelty_never_drops_a_pull");
    }
  }
  check(with_total <= without_total, "novelty_lowers_overlap_with_recent_pulls",
        std::to_string(with_total) + " vs " + std::to_string(without_total));
  check(history.size() == GrimPullHistory::kRemembered, "novelty_remembers_a_few");
  const GrimGenome again_a = grim_pull_generate(77, base, ctx, &history);
  const GrimGenome again_b = grim_pull_generate(77, base, ctx, &history);
  check(grim_genome_serialize(again_a) == grim_genome_serialize(again_b), "novelty_is_deterministic");

  // Describing a genome gives one readable line per gene.
  const GrimGenome mixed = grim_pull_generate(2024, GrimPullSettings{}, ctx, nullptr);
  const auto lines = grim_pull_describe(mixed, ctx);
  bool lines_ok = lines.size() == mixed.genes.size();
  for (const GrimGeneLine &l : lines) {
    lines_ok = lines_ok && !l.title.empty() && !l.detail.empty();
  }
  check(lines_ok, "describe_one_line_per_gene");
  const std::string id = grim_machine_id(mixed);
  check(id.size() == 5 && id[0] == '#', "machine_id_shape", id);
}

void mercy_and_death_text_tests() {
  GrimLiveStatus early;
  early.dead = true;
  early.death.frame = 100;
  GrimLiveStatus late = early;
  late.death.frame = kGrimMercyWindowFrames + 1;
  GrimLiveStatus alive;
  check(!grim_mercy_should_reroll(false, early), "mercy_off_never_rerolls");
  check(grim_mercy_should_reroll(true, early), "mercy_on_rerolls_early_death");
  check(!grim_mercy_should_reroll(true, late) && !grim_mercy_should_reroll(true, alive),
        "mercy_leaves_late_deaths_and_survivors");

  GrimGenome g;
  g.version = 2;
  GrimGene gene = grim_default_gene(GrimGeneType::RomCode);
  GrimRomPatch p;
  p.offset = 0x2B68;
  gene.patches.push_back(p);
  g.genes.push_back(gene);
  const GrimDeath loop = grim_describe_death("exception_loop", 70, 1.4, 0xBFC02B68u, 10, 0xBFC02B68u, &g);
  check(loop.headline == "Stuck in an exception loop" &&
            loop.detail.find("Reserved-instruction exception at BFC0 2B68") != std::string::npos &&
            loop.culprit_gene == 0,
        "death_text_exception_loop_names_culprit", loop.detail);
  for (const char *reason : {"coverage_stall", "frozen_frame", "dead_audio"}) {
    const GrimDeath d = grim_describe_death(reason, 10, 1.0, 0x80010000u, 0xFFFFFFFFu, 0, nullptr);
    check(!d.headline.empty() && !d.detail.empty() && d.headline != "Dead", std::string("death_text_") + reason,
          d.headline);
  }
}

void library_tests() {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "vibestation_grim_library_test";
  std::error_code ec;
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  const std::string path = (dir / "library.json").string();
  std::string err;

  GrimLibrary lib;
  check(lib.load(path, err) && lib.entries().empty(), "library_missing_file_is_empty");
  GrimPullEntry e;
  e.seed = 0xABCDEF0123456789ull;
  e.genome = "{\"version\":1,\"genes\":[]}\n";
  e.genome_hash = 42;
  e.families = 13;
  e.intensity = 58;
  const u64 first = lib.add(e);
  const u64 second = lib.add(e);
  check(first == 1 && second == 2, "library_pull_numbers_count_up");
  check(lib.update(second, true, "exception_loop", "Stuck in an exception loop", 1.4) &&
            lib.set_kept(second, true) && lib.set_kept(first, true) && lib.set_kept(first, false),
        "library_update_and_keep");
  check(lib.save(path, err) && !fs::exists(path + ".tmp"), "library_save_is_atomic_no_temp_left", err);
  GrimLibrary back;
  check(back.load(path, err) && back.entries().size() == 2, "library_round_trip_count", err);
  const GrimPullEntry *kept = back.find(second);
  check(kept != nullptr && kept->kept && kept->dead && kept->seed == e.seed &&
            kept->headline == "Stuck in an exception loop" && kept->genome == e.genome,
        "library_round_trip_fields");
  check(back.stats().pulls == 2 && back.stats().deaths == 1 && back.stats().kept == 1, "library_stats");
  check(back.add(e) == 3, "library_numbers_continue_after_reload");

  // Trimming drops old un-kept pulls but never kept ones (kept dead machines too).
  GrimLibrary big;
  const u64 keeper = big.add(e);
  big.set_kept(keeper, true);
  for (u32 i = 0; i < GrimLibrary::kMaxHistory + 50; ++i) {
    big.add(e);
  }
  size_t unkept = 0;
  for (const GrimPullEntry &x : big.entries()) {
    unkept += x.kept ? 0u : 1u;
  }
  check(unkept == GrimLibrary::kMaxHistory && big.find(keeper) != nullptr, "library_trims_history_not_kept");

  // A damaged file is set aside and the library still opens.
  {
    std::ofstream bad(path, std::ios::binary | std::ios::trunc);
    bad << "{ this is not json";
  }
  GrimLibrary recovered;
  check(!recovered.load(path, err) && recovered.entries().empty() && fs::exists(path + ".corrupt") &&
            !err.empty(),
        "library_recovers_from_corrupt_file", err);
  fs::remove_all(dir, ec);
}

// A corrupted GPU list that points back at itself used to replay a million packets inside
// one DMA (minutes of host time), freezing the live machine and the death watch with it.
void dma_loop_test(const Options &o) {
  auto sys = std::make_unique<System>();
  if (!sys->load_bios(o.bios)) {
    check(false, "dma_loop_bios_load");
    return;
  }
  sys->reset();
  // Node A (0x1000) -> B (0x1010) -> A, each carrying one NOP word for GP0.
  sys->write32(0x1000u, 0x01001010u);
  sys->write32(0x1004u, 0x00000000u);
  sys->write32(0x1010u, 0x01001000u);
  sys->write32(0x1014u, 0x00000000u);
  sys->write32(0x1F8010F0u, 0x0B000000u | (1u << 11)); // DPCR: channel 2 on
  sys->write32(0x1F8010A0u, 0x1000u);                  // DMA2 MADR
  sys->write32(0x1F8010A8u, 0x01000401u);              // CHCR: linked list, start
  const auto begin = std::chrono::steady_clock::now();
  for (int i = 0; i < 3; ++i) {
    sys->run_frame(false, false);
  }
  const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  check(seconds < 5.0, "dma_linked_list_loop_does_not_hang_the_host", std::to_string(seconds) + " s");
}

// ---- live machine tests --------------------------------------------------------------------

void live_tests(const Options &o, const GrimPullContext &ctx, const std::vector<u32> &stock) {
  const u64 hash = ctx.bios_hash;
  const LiveRun clean = run_live(o, nullptr, o.frames);
  print_live("clean_boot", o, clean);
  check(!clean.status.dead && clean.frames_run == o.frames, "live_clean_boot_stays_alive");

  const GrimGenome spin = rom_genome(hash, {rom_gene(stock, {{0x0u, 0x1000FFFFu}, {0x4u, 0u}})});
  const LiveRun hung = run_live(o, &spin, o.frames);
  print_live("hang", o, hung);
  check(hung.status.dead && hung.status.death.reason == "coverage_stall", "live_hang_is_caught",
        hung.status.death.reason);
  check(hung.status.death.seconds < 4.0, "live_death_is_fast",
        std::to_string(hung.status.death.seconds) + " s");
  check(!hung.status.death.headline.empty() && !hung.status.death.detail.empty(),
        "live_death_has_readable_cause", hung.status.death.headline + " / " + hung.status.death.detail);

  const GrimGenome trap = rom_genome(
      hash, {rom_gene(stock, {{0x000u, 0x0000000Cu}, {0x004u, 0u}, {0x180u, 0x401A7000u},
                              {0x184u, 0u}, {0x188u, 0x03400008u}, {0x18Cu, 0x42000010u}})});
  const LiveRun loop = run_live(o, &trap, o.frames);
  print_live("exception_loop", o, loop);
  check(loop.status.dead && loop.status.death.reason == "exception_loop", "live_exception_loop_is_caught",
        loop.status.death.reason);
  check(loop.status.death.detail.find("Syscall exception") != std::string::npos &&
            loop.status.death.culprit_gene == 0,
        "live_exception_loop_names_cause_and_culprit", loop.status.death.detail);

  // Recovery: the next pull on a fresh machine is unaffected by the dead one.
  const LiveRun again = run_live(o, nullptr, 240);
  check(!again.status.dead, "live_recovers_after_a_dead_pull");

  // Wrong BIOS: refused up front, and the machine itself reports the mismatch.
  GrimGenome foreign = spin;
  foreign.bios_hash ^= 1ull;
  std::string err;
  check(!grim_pull_compatible(foreign, hash, err) && !err.empty() && grim_pull_compatible(spin, hash, err),
        "wrong_bios_is_rejected_before_boot", err);
  const LiveRun refused = run_live(o, &foreign, 5);
  check(!refused.rom_error.empty(), "wrong_bios_machine_reports_mismatch", refused.rom_error);

  // Generated pulls boot, apply cleanly, and give the same verdict on both backends
  // (compare the GRIM_PULL_LIVE lines of the two runs).
  u32 dead = 0, boots = 0;
  for (u64 seed = 1; seed <= 6; ++seed) {
    GrimPullSettings s;
    s.intensity = 50;
    const GrimGenome g = grim_pull_generate(seed, s, ctx, nullptr);
    const LiveRun r = run_live(o, &g, o.frames);
    print_live("pull_seed_" + std::to_string(seed), o, r);
    boots += r.rom_error.empty() ? 1u : 0u;
    dead += r.status.dead ? 1u : 0u;
  }
  check(boots == 6, "generated_pulls_apply_cleanly", std::to_string(boots) + "/6");
  std::printf("GRIM_PULL_TEST INFO generated pulls dead=%u/6\n", dead);
}
} // namespace

int run_grim_pull_test(const std::vector<std::string> &args) {
  grim_prepare_eval_process();
  Options o;
  if (!parse_options(args, o)) {
    std::fprintf(stderr, "usage: --grim-pull-test [bios] [--map file.json] [--frames N] "
                         "[--backend interpreter|recompiler]\n");
    return 1;
  }
  set_backend(o.backend);

  GrimSampleContext sample;
  GrimRomContext rom;
  std::string err;
  if (!sample.load(o.bios, o.map, err)) {
    std::fprintf(stderr, "GRIM_PULL_ERROR %s\n", err.c_str());
    return 1;
  }
  if (!rom.load(o.bios, o.map, err)) {
    std::fprintf(stderr, "GRIM_PULL_ERROR %s\n", err.c_str());
    return 1;
  }
  GrimPullContext ctx;
  ctx.rom = &rom;
  ctx.sample = &sample;
  ctx.bios_hash = rom.bios_hash;
  GrimPullContext no_rom = ctx;
  no_rom.rom = nullptr;

  plan_tests();
  generation_tests(ctx, no_rom);
  mercy_and_death_text_tests();
  library_tests();
  dma_loop_test(o);
  live_tests(o, ctx, rom.words);

  std::printf("GRIM_PULL_TEST %s failures=%d backend=%s\n", failures == 0 ? "ALL_PASS" : "FAILED",
              failures, o.backend.c_str());
  return failures == 0 ? 0 : 1;
}

// ---- yield ---------------------------------------------------------------------------------

int run_grim_pull_yield(const std::vector<std::string> &args) {
  grim_prepare_eval_process();
  Options o;
  o.frames = 900;
  if (!parse_options(args, o)) {
    std::fprintf(stderr, "usage: --grim-pull-yield [bios] [--map file.json] [--pulls N] "
                         "[--frames N] [--intensity a,b,c] [--threads N] [--families audio,code,iface]\n");
    return 1;
  }
  GrimSampleContext sample;
  GrimRomContext rom;
  std::string err;
  if (!sample.load(o.bios, o.map, err) || !rom.load(o.bios, o.map, err)) {
    std::fprintf(stderr, "GRIM_PULL_ERROR %s\n", err.c_str());
    return 1;
  }
  GrimPullContext ctx;
  ctx.rom = &rom;
  ctx.sample = &sample;
  ctx.bios_hash = rom.bios_hash;

  namespace fs = std::filesystem;
  const fs::path work = fs::path(o.out_dir.empty() ? "grim_pull_yield" : o.out_dir);
  std::error_code ec;
  fs::create_directories(work, ec);
  const std::string exe = grim_self_exe_path("");
  const std::string frames = std::to_string(o.frames);
  const std::string clean_wav = (work / "clean.wav").string();
  {
    const GrimChildResult r = grim_run_child(
        exe, {"--grim-eval", o.bios, frames, (work / "clean.jsonl").string(), "--live-gates",
              "--dump-wav", clean_wav},
        (work / "clean.out").string(), 600.0);
    std::printf("GRIM_PULL_YIELD clean exit=%d\n", r.exit_code);
  }
  std::vector<s16> clean_pcm;
  std::string wav_err;
  if (!grim_read_wav(clean_wav, clean_pcm, wav_err)) {
    std::fprintf(stderr, "GRIM_PULL_ERROR clean reference: %s\n", wav_err.c_str());
    return 1;
  }

  struct Row {
    u32 intensity = 0;
    u64 seed = 0;
    bool dead = false, silent = false, audible = false, error = false, hung = false;
    std::string reason;
    u32 genes = 0;
  };
  std::vector<Row> rows;
  for (u32 t : o.intensities) {
    for (u32 i = 0; i < o.pulls; ++i) {
      Row r;
      r.intensity = t;
      r.seed = 5000000ull + t * 1000ull + i;
      rows.push_back(r);
    }
  }
  std::atomic<size_t> next{0};
  const auto worker = [&] {
    for (;;) {
      const size_t k = next.fetch_add(1);
      if (k >= rows.size()) {
        return;
      }
      Row &r = rows[k];
      GrimPullSettings s;
      s.families = o.families;
      s.intensity = r.intensity;
      const GrimGenome g = grim_pull_generate(r.seed, s, ctx, nullptr);
      r.genes = static_cast<u32>(g.genes.size());
      const std::string stem = "pull_" + std::to_string(r.seed);
      const std::string genome_path = (work / (stem + ".json")).string();
      const std::string out_path = (work / (stem + ".out")).string();
      const std::string wav_path = (work / (stem + ".wav")).string();
      {
        std::ofstream gf(genome_path, std::ios::binary | std::ios::trunc);
        gf << grim_genome_serialize(g);
      }
      // A hung or crashing machine must not take the batch down: one child each.
      const GrimChildResult child = grim_run_child(
          exe, {"--grim-eval", o.bios, frames, (work / (stem + ".jsonl")).string(), "--genome",
                genome_path, "--live-gates", "--dump-wav", wav_path},
          out_path, 180.0);
      if (child.timed_out || child.crashed) {
        r.dead = true;
        r.hung = true;
        r.reason = child.timed_out ? "host_hang" : "host_crash";
        continue;
      }
      std::ifstream in(out_path);
      std::string line, result;
      while (std::getline(in, line)) {
        if (line.find("GRIM_EVAL_RESULT") != std::string::npos) result = line;
      }
      if (result.empty() || result.find("status=error") != std::string::npos) {
        r.error = true;
        continue;
      }
      r.dead = result.find("verdict=dead") != std::string::npos;
      r.silent = result.find("silent=1") != std::string::npos;
      const size_t at = result.find("reason=");
      r.reason = at == std::string::npos ? "" : result.substr(at + 7, result.find(' ', at) - at - 7);
      if (!r.dead) {
        std::vector<s16> pcm;
        std::string err;
        if (grim_read_wav(wav_path, pcm, err)) {
          const GrimAudibilityReport a = grim_compare_audio(clean_pcm, pcm);
          r.audible = a.valid && a.audible;
        }
      }
    }
  };
  std::vector<std::thread> pool;
  for (u32 t = 0; t < o.threads; ++t) {
    pool.emplace_back(worker);
  }
  for (std::thread &t : pool) {
    t.join();
  }

  std::printf("GRIM_PULL_YIELD frames=%u pulls_per_level=%u families=0x%X\n", o.frames, o.pulls,
              o.families);
  std::printf("GRIM_PULL_YIELD intensity pulls alive dead survived+audible survived+inaudible "
              "silent errors mean_genes\n");
  for (u32 t : o.intensities) {
    u32 n = 0, alive = 0, dead = 0, audible = 0, inaudible = 0, silent = 0, errors = 0, genes = 0;
    std::map<std::string, u32> reasons;
    for (const Row &r : rows) {
      if (r.intensity != t) continue;
      ++n;
      genes += r.genes;
      if (r.error) { ++errors; continue; }
      if (r.dead) { ++dead; ++reasons[r.reason]; continue; }
      ++alive;
      silent += r.silent ? 1u : 0u;
      (r.audible ? audible : inaudible) += 1u;
    }
    std::printf("GRIM_PULL_YIELD %u %u %u %u %u %u %u %u %.1f", t, n, alive, dead, audible, inaudible,
                silent, errors, n ? static_cast<double>(genes) / n : 0.0);
    for (const auto &[reason, count] : reasons) {
      std::printf(" %s=%u", reason.c_str(), count);
    }
    std::printf("\n");
  }
  return 0;
}
