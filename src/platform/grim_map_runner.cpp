#include "platform/grim_map_runner.h"
#include "core/grim_eval.h"
#include "core/grim_map.h"
#include "core/grim_rom.h"
#include "platform/grim_eval_runner.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>

namespace {
const char *op_name(u32 op) {
  switch (op) {
  case 0x20: return "lb";
  case 0x21: return "lh";
  case 0x22: return "lwl";
  case 0x23: return "lw";
  case 0x24: return "lbu";
  case 0x25: return "lhu";
  case 0x26: return "lwr";
  case 0x28: return "sb";
  case 0x29: return "sh";
  case 0x2A: return "swl";
  case 0x2B: return "sw";
  case 0x2E: return "swr";
  case 0x32: return "lwc2";
  default: return "?";
  }
}

void print_copy_report(const GrimCopyStats &s) {
  std::printf("GRIM_MAP_COPY loads of ROM-origin bytes by opcode:");
  for (u32 op = 0; op < 64; ++op) {
    if (s.tagged_load_bytes_by_op[op] != 0) {
      std::printf(" %s=%llu", op_name(op), static_cast<unsigned long long>(s.tagged_load_bytes_by_op[op]));
    }
  }
  std::printf("\nGRIM_MAP_COPY stores of ROM-origin bytes to RAM by opcode:");
  for (u32 op = 0; op < 64; ++op) {
    if (s.tagged_store_bytes_by_op[op] != 0) {
      std::printf(" %s=%llu", op_name(op), static_cast<unsigned long long>(s.tagged_store_bytes_by_op[op]));
    }
  }
  std::printf("\nGRIM_MAP_COPY tagged stores to I/O=%llu, DMA words with ROM origin=%llu, DMA words "
              "written to RAM=%llu\n",
              static_cast<unsigned long long>(s.tagged_stores_to_io),
              static_cast<unsigned long long>(s.dma_tagged_words),
              static_cast<unsigned long long>(s.dma_words_to_ram));
  // The copy loops: store PCs that moved the most tagged bytes.
  std::vector<std::pair<u64, u32>> pcs;
  for (const auto &[pc, n] : s.tagged_store_pcs) {
    pcs.emplace_back(n, pc);
  }
  std::sort(pcs.begin(), pcs.end(), [](auto &a, auto &b) { return a.first != b.first ? a.first > b.first : a.second < b.second; });
  for (size_t i = 0; i < pcs.size() && i < 8; ++i) {
    std::printf("GRIM_MAP_COPY store_pc=0x%08X tagged_stores=%llu\n", pcs[i].second,
                static_cast<unsigned long long>(pcs[i].first));
  }
  std::vector<std::pair<u64, u32>> lpcs;
  for (const auto &[pc, n] : s.rom_load_pcs) {
    lpcs.emplace_back(n, pc);
  }
  std::sort(lpcs.begin(), lpcs.end(), [](auto &a, auto &b) { return a.first != b.first ? a.first > b.first : a.second < b.second; });
  for (size_t i = 0; i < lpcs.size() && i < 8; ++i) {
    std::printf("GRIM_MAP_COPY rom_load_pc=0x%08X bytes=%llu\n", lpcs[i].second,
                static_cast<unsigned long long>(lpcs[i].first));
  }
  for (const auto &[b, e] : s.unknown_exec_ranges) {
    std::printf("GRIM_MAP_UNKNOWN_EXEC ram=0x%06X-0x%06X (%u bytes executed without ROM origin)\n", b,
                e, e - b);
  }
}
} // namespace

int run_grim_map_cli(const std::vector<std::string> &raw_args) {
  grim_prepare_eval_process();
  std::vector<std::string> args = raw_args;
  const std::string bios = grim_take_bios_arg(args);
  if (bios.empty() || args.size() < 2) {
    std::fprintf(stderr, "usage: --grim-map [bios] <frames> <out.json>\n");
    return 1;
  }
  GrimEvalConfig cfg;
  cfg.bios_path = bios;
  cfg.frames = static_cast<u32>(std::max(1, std::atoi(args[0].c_str())));
  cfg.stop_on_death = false;
  cfg.watchdog_seconds = 0.0;
  for (size_t i = 2; i + 1 < args.size(); ++i) {
    if (args[i] == "--disc") {
      cfg.disc_cue = args[i + 1];
      cfg.map_scenario = "disc";
    }
  }
  GrimBootMap map;
  GrimCopyStats stats;
  cfg.boot_map_out = &map;
  cfg.copy_stats_out = &stats;
  const GrimEvalResult r = run_grim_eval(cfg);
  if (r.end_reason == "bios_load_failed" || r.end_reason == "disc_load_failed") {
    std::printf("GRIM_MAP_RESULT status=error reason=%s\n", r.end_reason.c_str());
    return 1;
  }
  std::string err;
  if (!grim_map_save(map, args[1], err)) {
    std::printf("GRIM_MAP_RESULT status=error reason=%s\n", err.c_str());
    return 1;
  }
  // Frame of the last newly executed word, from the telemetry (help choosing the scenario length).
  u32 last_new_frame = 0;
  for (const GrimFrameTelemetry &f : r.frames) {
    if (f.new_pcs != 0) {
      last_new_frame = f.frame;
    }
  }
  std::printf("GRIM_MAP_RESULT status=ok bios=0x%016llX scenario=%s frames=%u last_new_code_frame=%u "
              "verdict=%s map_hash=0x%016llX code=%u data=%u unused=%u unknown=%u "
              "provenance=%.1f%% (%u/%u) out=%s\n",
              static_cast<unsigned long long>(map.bios_hash), map.scenario.c_str(), map.frames,
              last_new_frame, r.liveness.alive ? "alive" : "dead",
              static_cast<unsigned long long>(map.hash()), map.count(GrimWordClass::Code),
              map.count(GrimWordClass::Data), map.count(GrimWordClass::Unused),
              map.count(GrimWordClass::Unknown), map.provenance_permille() / 10.0,
              map.ram_exec_known, map.ram_exec_words, args[1].c_str());
  print_copy_report(stats);
  return 0;
}

int run_grim_map_merge_cli(const std::vector<std::string> &args) {
  if (args.size() < 3) {
    std::fprintf(stderr, "usage: --grim-map-merge <a.json> <b.json> <out.json>\n");
    return 1;
  }
  GrimBootMap a, b;
  std::string err;
  if (!grim_map_load(args[0], a, err) || !grim_map_load(args[1], b, err)) {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 1;
  }
  if (a.bios_hash != b.bios_hash || a.words.size() != b.words.size()) {
    std::fprintf(stderr, "the maps are for different BIOS images\n");
    return 1;
  }
  const GrimBootMap m = grim_map_merge(a, b);
  if (!grim_map_save(m, args[2], err)) {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 1;
  }
  std::printf("GRIM_MAP_MERGED scenario=%s code=%u data=%u unused=%u unknown=%u out=%s\n",
              m.scenario.c_str(), m.count(GrimWordClass::Code), m.count(GrimWordClass::Data),
              m.count(GrimWordClass::Unused), m.count(GrimWordClass::Unknown), args[2].c_str());
  return 0;
}

int run_grim_map_summary_cli(const std::vector<std::string> &args) {
  if (args.empty()) {
    std::fprintf(stderr, "usage: --grim-map-summary <map.json> [--min-bytes N=128]\n");
    return 1;
  }
  u32 min_bytes = 128;
  for (size_t i = 1; i + 1 < args.size(); ++i) {
    if (args[i] == "--min-bytes") {
      min_bytes = static_cast<u32>(std::atoi(args[i + 1].c_str()));
    }
  }
  GrimBootMap map;
  std::string err;
  if (!grim_map_load(args[0], map, err)) {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 1;
  }
  std::printf("%s", grim_map_summary(map, min_bytes).c_str());
  return 0;
}

int run_grim_describe_genome_cli(const std::vector<std::string> &raw_args) {
  std::vector<std::string> pos;
  std::string bios;
  for (size_t i = 0; i < raw_args.size(); ++i) {
    if (raw_args[i] == "--bios" && i + 1 < raw_args.size()) {
      bios = raw_args[++i];
    } else {
      pos.push_back(raw_args[i]);
    }
  }
  if (pos.empty()) {
    std::fprintf(stderr, "usage: --grim-describe-genome <genome.json> [map.json] [--bios path]\n");
    return 1;
  }
  GrimGenome genome;
  std::string err;
  if (!grim_genome_load(pos[0], genome, err)) {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 1;
  }
  GrimRomContext ctx;
  const GrimRomContext *use = nullptr;
  if (pos.size() > 1) {
    if (bios.empty()) {
      const char *env = std::getenv("VIBESTATION_BIOS");
      bios = env != nullptr ? env : "";
    }
    if (bios.empty() || !ctx.load(bios, pos[1], err)) {
      std::fprintf(stderr, "map not used: %s\n", bios.empty() ? "no BIOS (--bios or VIBESTATION_BIOS)" : err.c_str());
    } else {
      use = &ctx;
    }
  }
  std::printf("%s", grim_describe_genome(genome, use).c_str());
  return 0;
}
