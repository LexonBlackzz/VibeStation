// Phase 4: independent scanner fixtures, structural gene invariants, and measured
// clean-boot audio survival AND change. Outputs derive from BIOS and stay local.
#include "platform/grim_sample_runner.h"
#include "platform/grim_eval_runner.h"
#include "core/bios.h"
#include "core/grim_eval.h"
#include "core/grim_sample.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace {
int failures = 0;
void check(bool ok, const std::string &name, const std::string &detail = "") {
  std::printf("GRIM_SAMPLE_TEST %s %s %s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.c_str());
  std::fflush(stdout);
  failures += ok ? 0 : 1;
}
u8 byte(const std::vector<u32> &w, u32 off) {
  return static_cast<u8>(w[off / 4u] >> ((off & 3u) * 8u));
}
void put(std::vector<u32> &w, u32 off, u8 value) {
  const u32 sh = (off & 3u) * 8u;
  w[off / 4u] = (w[off / 4u] & ~(255u << sh)) | (u32{value} << sh);
}
void bank(std::vector<u32> &w, u32 off, u32 blocks, u64 seed) {
  GrimRng rng{seed};
  for (u32 b = 0; b < blocks; ++b) {
    put(w, off + b * 16u, static_cast<u8>((b % 5u) * 16u + b % 13u));
    put(w, off + b * 16u + 1u, static_cast<u8>(b == blocks - 1 ? 3 : b == 2 ? 4 : 0));
    for (u32 p = 2; p < 16; ++p) put(w, off + b * 16u + p, static_cast<u8>(rng.next()));
  }
}
GrimSampleContext fixture() {
  GrimSampleContext c;
  c.words.assign(4096 / 4, 0xFFFFFFFFu);
  c.bios_hash = 1;
  bank(c.words, 512, 18, 11);
  bank(c.words, 2048 + 8, 20, 17); // the second legal eight-byte alignment lane
  c.init();
  return c;
}
void scanner_fixtures() {
  GrimSampleContext c = fixture();
  check(c.scanner_samples.size() == 2 && c.scanner_samples[0].start_offset == 512 &&
        c.scanner_samples[1].start_offset == 2056, "scanner_two_alignment_lanes");
  auto unterminated = c.words;
  put(unterminated, 512 + 17 * 16 + 1, 0);
  const auto absent = grim_scan_adpcm(unterminated);
  check(absent.size() == 1 && absent[0].start_offset == 2056, "scanner_requires_loop_end");
  auto too_short = std::vector<u32>(1024 / 4, 0xFFFFFFFFu);
  bank(too_short, 256, kGrimAdpcmMinBlocks - 1, 29);
  check(grim_scan_adpcm(too_short).empty(), "scanner_rejects_short_random_runs");
  auto broken = c.words;
  put(broken, 512 + 9 * 16, 0x50); // forbidden predictor splits this run
  check(grim_scan_adpcm(broken).size() == 2 &&
        grim_scan_adpcm(broken)[0].start_offset == 512 + 10 * 16,
        "scanner_invalid_header_breaks_run");
  std::vector<u32> silence(1024 / 4, 0);
  put(silence, 1024 - 15, 1);
  check(grim_scan_adpcm(silence).empty(), "scanner_rejects_silent_filler");
  // A sufficiently long structured run is a candidate even without a map.
  check(c.samples.size() == c.scanner_samples.size() && c.map.words.empty(),
        "scanner_needs_no_map");
  auto prefix = std::vector<u32>(1024 / 4, 0xFFFFFFFFu);
  bank(prefix, 256, 8, 93);
  for (u32 off = 256; off < 256 + 64; ++off) put(prefix, off, 0);
  put(prefix, 256, 0x0C); // a real header with silent payload, then zero blocks
  const auto kept = grim_scan_adpcm(prefix);
  check(kept.size() == 1 && kept[0].start_offset == 256 && kept[0].block_count == 8,
        "scanner_trims_only_consecutive_zero_prefix");
}
std::string invariant_error(const GrimSampleContext &c, const GrimGene &g, GrimSampleMut kind) {
  if (g.patches.empty()) return "no effective patches";
  if (g.params[3] < 0 || static_cast<size_t>(g.params[3]) >= c.samples.size()) return "invalid target";
  const auto &s = c.samples[static_cast<size_t>(g.params[3])];
  if (kind == GrimSampleMut::Transplant &&
      (g.params[4] < 0 || static_cast<size_t>(g.params[4]) >= c.samples.size())) return "invalid donor";
  auto changed = c.words;
  u32 previous = 0;
  bool first = true;
  for (const auto &p : g.patches) {
    if ((p.offset & 3u) || p.offset < s.start_offset || p.offset >= s.end_offset ||
        p.original != c.words[p.offset / 4u] || p.original == p.mutated || p.delay_slot ||
        (!first && p.offset <= previous)) return "patch alignment/range/original/order";
    changed[p.offset / 4u] = p.mutated;
    previous = p.offset;
    first = false;
  }
  bool reserved_shift = false;
  for (u32 off = s.start_offset; off < s.end_offset; off += 16) {
    if ((off & 7u) || (off - s.start_offset) % 16u) return "block alignment";
    const u8 h = byte(c.words, off), nh = byte(changed, off);
    const u8 f = byte(c.words, off + 1), nf = byte(changed, off + 1);
    if (!grim_sample_header_gene(kind) && (h != nh || f != nf)) return "payload gene changed header";
    if (kind == GrimSampleMut::FilterSwap && ((h & 0x8Fu) != (nh & 0x8Fu) || f != nf || ((nh >> 4) & 7) > 4))
      return "filter gene changed other fields";
    if (kind == GrimSampleMut::ShiftChange && ((h & 0xF0u) != (nh & 0xF0u) || f != nf))
      return "shift gene changed other fields";
    if (kind >= GrimSampleMut::LoopStartMove && kind <= GrimSampleMut::LoopEndEarly &&
        (h != nh || (nf & 0xF8u) != (f & 0xF8u))) return "loop gene changed nonflag fields";
    reserved_shift = reserved_shift || ((nh & 15u) > 12);
    if (grim_sample_header_gene(kind)) {
      for (u32 p = 2; p < 16; ++p) if (byte(c.words, off + p) != byte(changed, off + p))
        return "header gene changed payload";
    }
    if (kind >= GrimSampleMut::BlockShuffle && kind <= GrimSampleMut::Transplant) {
      bool payload_changed = false;
      for (u32 p = 2; p < 16; ++p) payload_changed = payload_changed || byte(changed, off + p) != byte(c.words, off + p);
      if (!payload_changed) continue;
      const auto &source = kind == GrimSampleMut::Transplant
        ? c.samples[static_cast<size_t>(g.params[4])] : s;
      bool matched = false;
      for (u32 donor = source.start_offset; donor < source.end_offset; donor += 16) {
        bool equal = true;
        for (u32 p = 2; p < 16; ++p) equal = equal && byte(changed, off + p) == byte(c.words, donor + p);
        matched = matched || equal;
      }
      if (!matched) return "block payload not from expected source";
    }
  }
  if (kind == GrimSampleMut::ShiftChange && (g.params[5] != 0) != reserved_shift)
    return "reserved shift tag inaccurate";
  return "";
}
void invariant_fuzz(const GrimSampleContext &c, const std::string &label) {
  for (u32 k = 0; k < static_cast<u32>(GrimSampleMut::Count); ++k) {
    const auto kind = static_cast<GrimSampleMut>(k);
    std::string error;
    u32 tested = 0;
    for (u64 seed = 1; seed <= 256 && error.empty(); ++seed) {
      const u32 target = static_cast<u32>(seed & 1u), donor = 1u - target;
      const auto g = grim_sample_generate(c, seed, kind, 1u + static_cast<u32>(seed % 12),
                                          1u + static_cast<u32>(seed % 15), target, donor);
      error = invariant_error(c, g, kind);
      ++tested;
      const auto again = grim_sample_generate(c, seed, kind, 1u + static_cast<u32>(seed % 12),
                                              1u + static_cast<u32>(seed % 15), target, donor);
      if (grim_genome_hash({2, c.bios_hash, {g}}) != grim_genome_hash({2, c.bios_hash, {again}}))
        error = "not reproducible";
      GrimGenome parsed;
      std::string parse_error;
      if (!grim_genome_parse(grim_genome_serialize({2, c.bios_hash, {g}}), parsed, parse_error))
        error = "generated gene does not parse: " + parse_error;
    }
    check(error.empty() && tested == 256, label + "_" + grim_sample_mut_name(kind),
          error.empty() ? std::to_string(tested) + " deterministic mutations" : error);
  }
}
GrimEvalConfig config(const std::string &bios, u32 frames) {
  GrimEvalConfig c;
  c.bios_path = bios;
  c.frames = frames;
  c.stop_on_death = false;
  c.watchdog_seconds = 120;
  return c;
}
u32 audio_differences(const GrimEvalResult &a, const GrimEvalResult &b) {
  u32 n = 0;
  for (size_t i = 0; i < std::min(a.frames.size(), b.frames.size()); ++i)
    n += a.frames[i].audio_hash != b.frames[i].audio_hash ? 1u : 0u;
  return n;
}
void save(const std::filesystem::path &p, const GrimGenome &g) {
  std::ofstream out(p, std::ios::binary);
  out << grim_genome_serialize(g);
}
void boot_audio(const std::string &bios, const GrimSampleContext &c, const GrimEvalResult &clean,
                u32 frames, u32 seeds, const std::filesystem::path &dir) {
  std::vector<u32> played;
  u32 uploaded_unplayed = kGrimSampleAuto;
  for (u32 i = 0; i < c.samples.size(); ++i)
    if (c.samples[i].voice_start_confirmed && c.samples[i].voice_mask) played.push_back(i);
    else if (c.samples[i].spu_words && c.samples[i].voice_mask == 0) uploaded_unplayed = i;
  check(!played.empty(), "sample_boundaries_resolved_from_voice_starts", std::to_string(played.size()));
  if (played.empty()) return;
  std::ofstream csv;
  if (!dir.empty()) {
    std::filesystem::create_directories(dir);
    csv.open(dir / "audio_survival.csv");
    csv << "kind,runs,survived_changed,survived_identical,audio_failed,dead\n";
  }
  GrimGenome parity;
  for (u32 k = 0; k < static_cast<u32>(GrimSampleMut::Count); ++k) {
    const auto kind = static_cast<GrimSampleMut>(k);
    u32 changed = 0, identical = 0, audio_failed = 0, dead = 0;
    for (u32 j = 0; j < seeds; ++j) {
      const bool contrast = uploaded_unplayed != kGrimSampleAuto && (j % 3u) == 2u;
      const u32 target = contrast ? uploaded_unplayed : played[j % played.size()];
      const auto gene = grim_sample_generate(c, 1000 + k * 97 + j, kind, 1, 1, target);
      check(!gene.patches.empty(), std::string("small_edit_generated_") + grim_sample_mut_name(kind) +
            "_" + std::to_string(j));
      const GrimGenome genome{2, c.bios_hash, {gene}};
      auto cfg = config(bios, frames);
      cfg.use_genome = true;
      cfg.genome = genome;
      if (!dir.empty()) {
        const std::string stem = std::string(grim_sample_mut_name(kind)) + "_" + std::to_string(j);
        save(dir / (stem + ".json"), genome);
        if (j == 0) cfg.dump_wav_path = (dir / (stem + ".wav")).string();
      }
      const auto result = run_grim_eval(cfg);
      if (contrast) check(audio_differences(clean, result) == 0,
                          std::string("uploaded_unplayed_identical_") + grim_sample_mut_name(kind));
      const bool survived = result.end_reason == "frames" && result.frames.size() == frames && result.liveness.alive;
      if (!survived) ++dead;
      else if (result.liveness.silent) ++audio_failed;
      else if (audio_differences(clean, result)) ++changed;
      else ++identical;
      if (j == 0) { parity.version = 2; parity.bios_hash = c.bios_hash; parity.genes.push_back(gene); }
    }
    std::printf("GRIM_SAMPLE_AUDIO kind=%s runs=%u survived_changed=%u survived_identical=%u audio_failed=%u dead=%u\n",
                grim_sample_mut_name(kind), seeds, changed, identical, audio_failed, dead);
    check(dead == 0 && audio_failed == 0 && changed + identical == seeds,
          std::string("small_edits_keep_audio_live_") + grim_sample_mut_name(kind));
    if (csv) csv << grim_sample_mut_name(kind) << ',' << seeds << ',' << changed << ',' << identical << ','
                 << audio_failed << ',' << dead << '\n';
  }
  // Use the same scanner on a synthetic bank installed in untouched ROM. This
  // separates a valid sample edit from one that the boot actually consumes.
  auto unplayed = c;
  u32 unused = kGrimSampleAuto;
  for (u32 off = 0; off + 512 <= c.words.size() * 4u; off += 16) {
    bool untouched = true;
    for (u32 wi = off / 4; wi < (off + 512) / 4; ++wi)
      untouched = untouched && c.map.words[wi].flags == 0;
    if (untouched) { unused = off + 16; break; }
  }
  if (unused != kGrimSampleAuto) {
    for (u32 wi = (unused - 16) / 4; wi < unused / 4; ++wi) unplayed.words[wi] = 0xFFFFFFFFu;
    for (u32 wi = (unused + 256) / 4; wi < (unused + 272) / 4; ++wi) unplayed.words[wi] = 0xFFFFFFFFu;
    bank(unplayed.words, unused, 16, 404);
    const auto temp = std::filesystem::temp_directory_path() /
      ("vibestation_sample_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(temp);
    const auto temp_bios = temp / "unplayed.bin";
    {
      std::ofstream out(temp_bios, std::ios::binary);
      for (u32 w : unplayed.words) {
        const char bytes[4] = {static_cast<char>(w), static_cast<char>(w >> 8),
                               static_cast<char>(w >> 16), static_cast<char>(w >> 24)};
        out.write(bytes, 4);
      }
    }
    std::string error;
    const bool loaded = unplayed.load(temp_bios.string(), "", error);
    check(loaded, "unplayed_fixture_load", error);
    u32 target = kGrimSampleAuto;
    for (u32 i = 0; i < unplayed.samples.size(); ++i)
      if (unplayed.samples[i].start_offset == unused) target = i;
    if (target != kGrimSampleAuto) {
      const auto noise = grim_sample_generate(unplayed, 17, GrimSampleMut::NibbleNoise, 4, 3, target);
      auto cfg = config(temp_bios.string(), frames);
      const auto reference = run_grim_eval(cfg);
      cfg.use_genome = true;
      cfg.genome = {2, unplayed.bios_hash, {noise}};
      const auto result = run_grim_eval(cfg);
      check(!noise.patches.empty() && reference.run_hash == clean.run_hash && result.end_reason == "frames" &&
            result.run_hash == reference.run_hash && audio_differences(reference, result) == 0,
            "edit_unplayed_sample_is_audio_identical");
    } else check(false, "unplayed_fixture_scan");
    std::error_code ec;
    std::filesystem::remove_all(temp, ec);
  } else {
    // An unused raw scanner candidate is enough; no synthetic fixture needed.
    bool tested = false;
    for (u32 i = 0; i < c.samples.size() && !tested; ++i) {
      if (c.samples[i].voice_mask || c.samples[i].spu_words) continue;
      auto cfg = config(bios, frames);
      cfg.use_genome = true;
      cfg.genome = {2, c.bios_hash, {grim_sample_generate(c, 17, GrimSampleMut::NibbleNoise, 1, 1, i)}};
      const auto result = run_grim_eval(cfg);
      check(result.end_reason == "frames" && audio_differences(clean, result) == 0, "edit_unplayed_sample_is_audio_identical");
      tested = true;
    }
    check(tested, "unplayed_sample_available");
  }
  auto cp = config(bios, 300);
  cp.use_genome = true;
  cp.genome = parity;
  cp.native_cpu = true;
  const auto interp = run_grim_eval(cp);
  g_cpu_execution_mode_cli_value = CpuExecutionMode::Recompiler;
  const auto rec = run_grim_eval(cp);
  g_cpu_execution_mode_cli_value = CpuExecutionMode::Interpreter;
  bool equal = interp.frames.size() == 300 && rec.frames.size() == 300 && interp.gene_hits == rec.gene_hits;
  for (size_t i = 0; equal && i < 300; ++i) {
    const auto &a = interp.frames[i], &b = rec.frames[i];
    equal = a.audio_hash == b.audio_hash && a.fb_hash == b.fb_hash && a.cycles == b.cycles &&
            a.spu_key_on == b.spu_key_on && a.spu_voice_mask == b.spu_voice_mask;
  }
  check(equal, "sample_genes_match_between_interpreter_and_recompiler");
}
} // namespace

int run_grim_sample_test(const std::vector<std::string> &raw_args) {
  grim_prepare_eval_process();
  auto args = raw_args;
  std::string bios;
  if (!args.empty() && args[0].rfind("--", 0) != 0) {
    bios = args[0]; args.erase(args.begin());
  } else {
    const char *env = std::getenv("VIBESTATION_BIOS");
    bios = env != nullptr ? env : "";
  }
  u32 frames = 600, seeds = 3;
  std::filesystem::path dir;
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--out-dir" && i + 1 < args.size()) dir = args[++i];
    else if (args[i] == "--frames" && i + 1 < args.size()) frames = static_cast<u32>(std::max(300, std::atoi(args[++i].c_str())));
    else if (args[i] == "--seeds" && i + 1 < args.size()) seeds = static_cast<u32>(std::max(1, std::atoi(args[++i].c_str())));
    else { std::fprintf(stderr, "unknown sample-test option: %s\n", args[i].c_str()); return 1; }
  }
  failures = 0;
  scanner_fixtures();
  invariant_fuzz(fixture(), "synthetic_invariants");
  if (bios.empty()) { std::fprintf(stderr, "--grim-sample-test requires [bios] or VIBESTATION_BIOS\n"); return 1; }
  GrimBootMap map;
  auto cfg = config(bios, frames);
  cfg.boot_map_out = &map;
  const auto clean = run_grim_eval(cfg);
  check(clean.end_reason == "frames" && !clean.liveness.silent, "clean_boot_audio_is_live");
  GrimSampleContext c;
  std::string err;
  check(c.load(bios, "", err), "load_stock_samples", err);
  if (!c.words.empty()) {
    c.map = map;
    c.init();
    const auto score = grim_sample_score(c.scanner_samples, map);
    // 95% precision admits a handful of structural decoys; 99% recall requires
    // practically the whole bank. These are word-level answer-key checks,
    // independent of provenance boundary refinement and mutation selection.
    check(score.precision() >= 0.95 && score.recall() >= 0.99, "scanner_vs_spu_map",
          "precision=" + std::to_string(score.precision()) + " recall=" + std::to_string(score.recall()) +
          " tp=" + std::to_string(score.true_positive_words) + " fp=" + std::to_string(score.false_positive_words) +
          " fn=" + std::to_string(score.false_negative_words));
    std::printf("%s", grim_sample_summary(c).c_str());
    boot_audio(bios, c, clean, frames, seeds, dir);
  }
  std::printf("GRIM_SAMPLE_TEST_RESULT %s failures=%d\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
