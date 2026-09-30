#include "platform/grim_sample_runner.h"
#include "platform/grim_eval_runner.h"
#include "core/grim_sample.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>

int run_grim_samples_cli(const std::vector<std::string> &raw_args) {
  grim_prepare_eval_process();
  std::vector<std::string> args;
  std::string bios;
  std::string map_path;
  for (size_t i = 0; i < raw_args.size(); ++i) {
    if (raw_args[i] == "--map" && i + 1 < raw_args.size()) map_path = raw_args[++i];
    else if (raw_args[i] == "--bios" && i + 1 < raw_args.size()) bios = raw_args[++i];
    else if (raw_args[i].rfind("--", 0) == 0) {
      std::fprintf(stderr, "unknown sample option: %s\n", raw_args[i].c_str()); return 1;
    } else args.push_back(raw_args[i]);
  }
  if (args.size() == 2 && bios.empty()) { bios = args[0]; args.erase(args.begin()); }
  if (bios.empty()) {
    const char *env = std::getenv("VIBESTATION_BIOS");
    bios = env != nullptr ? env : "";
  }
  if (bios.empty() || args.size() != 1) {
    std::fprintf(stderr, "usage: --grim-samples [bios] <out.json> [--map file.json] [--bios path]\n");
    return 1;
  }
  GrimSampleContext c;
  std::string err;
  if (!c.load(bios, map_path, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
  nlohmann::ordered_json j;
  char hash[32];
  std::snprintf(hash, sizeof(hash), "0x%016llX", static_cast<unsigned long long>(c.bios_hash));
  j["version"] = 1;
  j["bios_hash"] = hash;
  j["scanner"] = {{"min_blocks", kGrimAdpcmMinBlocks}, {"alignment_bytes", 8}, {"block_bytes", 16},
                   {"requires_loop_end", true}, {"input", "rom_bytes_only"}};
  j["map"] = map_path;
  if (!map_path.empty()) {
    const auto s = grim_sample_score(c.scanner_samples, c.map);
    j["score"] = {{"true_positive_words", s.true_positive_words}, {"false_positive_words", s.false_positive_words},
                   {"false_negative_words", s.false_negative_words}, {"precision", s.precision()}, {"recall", s.recall()}};
  }
  auto serialize = [](const std::vector<GrimAdpcmSample> &samples) {
    auto a = nlohmann::ordered_json::array();
    for (size_t i = 0; i < samples.size(); ++i) {
      const auto &s = samples[i];
      nlohmann::ordered_json row;
      row["index"] = i;
      row["rom_start"] = s.start_offset;
      row["rom_end"] = s.end_offset;
      row["block_count"] = s.block_count;
      row["loop_start_offsets"] = s.loop_start_offsets;
      row["loop_end_offset"] = s.loop_end_offset;
      row["voice_start_confirmed"] = s.voice_start_confirmed;
      row["spu_words"] = s.spu_words;
      row["voice_mask"] = s.voice_mask;
      row["first_use_cycle"] = s.first_use_cycle == kGrimNever ? nlohmann::ordered_json(nullptr) : nlohmann::ordered_json(s.first_use_cycle);
      row["last_use_cycle"] = s.last_use_cycle;
      row["uses"] = nlohmann::ordered_json::array();
      for (const auto &u : s.uses)
        row["uses"].push_back({{"cycle", u.cycle}, {"rom_offset", u.rom_offset}, {"spu_address", u.spu_address},
                                {"voice", u.voice}, {"kind", grim_spu_sample_use_kind_name(static_cast<GrimSpuSampleUseKind>(u.kind))}});
      a.push_back(std::move(row));
    }
    return a;
  };
  j["scanner_candidates"] = serialize(c.scanner_samples);
  j["samples"] = serialize(c.samples);
  // Retain unresolved events in the report as evidence of coverage limits.
  j["unresolved_uses"] = nlohmann::ordered_json::array();
  for (const auto &u : c.map.spu_sample_uses)
    if (u.rom_offset == kGrimNoRomOffset)
      j["unresolved_uses"].push_back({{"cycle", u.cycle}, {"voice", u.voice}, {"spu_address", u.spu_address},
                                      {"kind", grim_spu_sample_use_kind_name(u.kind)}});
  std::ofstream out(args[0], std::ios::binary | std::ios::trunc);
  out << j.dump(2) << '\n';
  if (!out) { std::fprintf(stderr, "cannot write samples: %s\n", args[0].c_str()); return 1; }
  std::printf("%sGRIM_SAMPLES_RESULT status=ok candidates=%zu samples=%zu out=%s\n",
              grim_sample_summary(c).c_str(), c.scanner_samples.size(), c.samples.size(), args[0].c_str());
  return 0;
}
