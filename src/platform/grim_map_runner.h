#pragma once

#include <string>
#include <vector>

// Grim Reaper 2.0 Phase 3 command-line modes (see docs/grim-reaper/PROGRESS.md).
//
//   --grim-map [bios] <frames> <out.json>
//     Boots the stock BIOS with no disc under the interpreter, watches it
//     (loads, stores, DMA, executed words, RAM provenance) and writes the map:
//     <out.json> (header + regions) and <out.json>.words (per-word binary).
//     Exit 0 on success. Prints GRIM_MAP_RESULT and the copy-routine report.
//   --grim-map-summary <map.json>
//     Human-readable region table.
//   --grim-describe-genome <genome.json> [map.json] [--bios path]
//     Every gene in readable form. With a map (and the BIOS) the ROM genes
//     show disassembly, region and first-execution time.
//   --grim-map-test [bios]
//     Classification, provenance, determinism, mutation fuzz and ROM gene tests.
int run_grim_map_cli(const std::vector<std::string> &args);
int run_grim_map_summary_cli(const std::vector<std::string> &args);
int run_grim_describe_genome_cli(const std::vector<std::string> &args);
int run_grim_map_test(const std::vector<std::string> &args, const std::string &self_exe);
