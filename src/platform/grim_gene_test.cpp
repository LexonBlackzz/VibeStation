// --grim-gene-test: Grim Reaper 2.0 Phase 2 tests (genome, SPU/GP0 filters,
// triggers, end-to-end on the stock BIOS, determinism, --grim-explore).
#include "core/grim_eval.h"
#include "core/grim_genome.h"
#include "core/grim_fmv.h"
#include "core/bios.h"
#include "core/system.h"
#include "core/types.h"
#include "platform/grim_eval_runner.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>

namespace {

int g_failures = 0;

void check(bool ok, const std::string &name, const std::string &detail = "") {
  std::printf("GRIM_GENE_TEST %s %s %s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.c_str());
  std::fflush(stdout);
  if (!ok) {
    ++g_failures;
  }
}

std::string hex(u64 v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "0x%llX", static_cast<unsigned long long>(v));
  return buf;
}

// ---- helpers ------------------------------------------------------------------

void set_param(GrimGene &g, const char *name, s32 value) {
  const auto &schema = grim_gene_schema(g.type);
  for (size_t i = 0; i < schema.size(); ++i) {
    if (std::string(schema[i].name) == name) {
      g.params[i] = value;
      return;
    }
  }
  std::printf("GRIM_GENE_TEST BUG unknown param %s\n", name);
  ++g_failures;
}

GrimGene make_gene(GrimGeneType type, std::initializer_list<std::pair<const char *, s32>> params,
                   u32 target = 0) {
  GrimGene g = grim_default_gene(type);
  g.seed = 12345;
  if (target != 0) {
    g.target = target;
  }
  for (const auto &p : params) {
    set_param(g, p.first, p.second);
  }
  return g;
}

GrimTrigger window(u32 a, u32 b) {
  GrimTrigger t;
  t.kind = GrimTriggerKind::Window;
  t.start_frame = a;
  t.end_frame = b;
  return t;
}
GrimTrigger rot(u32 a, u32 b) {
  GrimTrigger t;
  t.kind = GrimTriggerKind::Rot;
  t.start_frame = a;
  t.end_frame = b;
  return t;
}
GrimTrigger pattern(u32 period, u32 duty, u32 start = 0, u32 end = 0) {
  GrimTrigger t;
  t.kind = GrimTriggerKind::Intermittent;
  t.period = period;
  t.duty = duty;
  t.start_frame = start;
  t.end_frame = end;
  return t;
}
GrimTrigger chance(u32 permille) {
  GrimTrigger t;
  t.kind = GrimTriggerKind::Intermittent;
  t.probability = permille;
  return t;
}

std::unique_ptr<GrimGenomeRuntime> make_rt(std::vector<GrimGene> genes, u32 frame = 0) {
  GrimGenome g;
  g.genes = std::move(genes);
  auto rt = std::make_unique<GrimGenomeRuntime>(std::move(g));
  rt->begin_frame(frame);
  return rt;
}

std::vector<GrimSpuWrite> spu(GrimGenomeRuntime &rt, u32 off, u32 val, u64 now = 0) {
  GrimSpuWrite out[GrimGenomeRuntime::kMaxSpuOut];
  const size_t n = rt.filter_spu_write(off, static_cast<u16>(val), now, out);
  return std::vector<GrimSpuWrite>(out, out + n);
}

// Single result value, or 0xFFFFFFFF when the write was dropped / multiplied.
u32 one(const std::vector<GrimSpuWrite> &w) { return w.size() == 1 ? w[0].value : 0xFFFFFFFFu; }
bool is_one(const std::vector<GrimSpuWrite> &w, u32 off, u32 val) {
  return w.size() == 1 && w[0].offset == off && w[0].value == val;
}

std::string read_file(const std::filesystem::path &p) {
  std::ifstream in(p, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// ---- rng / genome --------------------------------------------------------------

void test_rng() {
  GrimRng z{0};
  const u64 a = z.next(), b = z.next(), c = z.next();
  check(a == 0xE220A8397B1DCDAFull && b == 0x6E789E6AA1B965F4ull && c == 0x06C45D188009454Full,
        "splitmix64_seed0", hex(a) + " " + hex(b) + " " + hex(c));
  GrimRng k{1234567};
  const u64 expect[5] = {6457827717110365317ull, 3203168211198807973ull, 9817491932198370423ull,
                         4593380528125082431ull, 16408922859458223821ull};
  bool ok = true;
  for (const u64 e : expect) {
    ok = ok && k.next() == e;
  }
  check(ok, "splitmix64_seed1234567");
  GrimRng r{42};
  const u32 want_range[4] = {19, 15, 12, 11};
  ok = true;
  for (const u32 w : want_range) {
    ok = ok && r.range(10, 20) == w;
  }
  ok = ok && r.unit_q10() == 38u && r.unit_q10() == 889u;
  GrimRng s{7};
  const s32 want_s[4] = {-3, -5, -5, -5};
  for (const s32 w : want_s) {
    ok = ok && s.srange(-5, 5) == w;
  }
  check(ok, "rng_helpers_fixed_values");
  check(grim_mix64(0) == 0xE220A8397B1DCDAFull && grim_mix64(1) == 0x910A2DEC89025CC1ull,
        "mix64_fixed_values");
}

// One gene of each type with a non-default trigger, to exercise every field.
GrimGenome all_types_genome() {
  GrimGenome g;
  for (size_t t = 0; t < static_cast<size_t>(GrimGeneType::Count); ++t) {
    GrimGene gene = grim_default_gene(static_cast<GrimGeneType>(t));
    gene.seed = 1000 + t;
    if (grim_gene_is_rom(gene.type)) {
      g.version = 2;
      g.bios_hash = 0x123456789ABCDEF0ull;
    } else switch (t % 4) {
    case 0:
      gene.trigger = window(10, 500);
      break;
    case 1:
      gene.trigger = rot(0, 900);
      break;
    case 2:
      gene.trigger = pattern(30, 10, 5, 0);
      break;
    default:
      gene.trigger = chance(250);
      break;
    }
    g.genes.push_back(gene);
  }
  return g;
}

void test_genome_roundtrip() {
  bool ok = true;
  std::string detail;
  std::vector<GrimGenome> genomes;
  genomes.push_back(all_types_genome());
  genomes.push_back(GrimGenome{});
  for (u64 seed = 0; seed < 60; ++seed) {
    GrimRandomParams p;
    p.max_genes = 6;
    genomes.push_back(grim_random_genome(seed, p));
  }
  for (const GrimGenome &g : genomes) {
    const std::string text = grim_genome_serialize(g);
    GrimGenome back;
    std::string err;
    if (!grim_genome_parse(text, back, err)) {
      ok = false;
      detail = "parse failed: " + err;
      break;
    }
    if (grim_genome_serialize(back) != text || grim_genome_hash(back) != grim_genome_hash(g)) {
      ok = false;
      detail = "bytes or hash differ";
      break;
    }
  }
  check(ok, "genome_json_roundtrip", detail + " genomes=" + std::to_string(genomes.size()));

  // Stable, documented field order.
  GrimGenome one_gene;
  GrimGene g = grim_default_gene(GrimGeneType::SpuPitch);
  g.target = 255;
  g.seed = 5;
  one_gene.genes.push_back(g);
  const std::string want =
      "{\"version\":1,\"genes\":[\n{\"type\":\"spu_pitch\",\"target\":255,\"seed\":5,"
      "\"trigger\":{\"kind\":\"always\"},\"params\":{\"mode\":0,\"mul_q8\":384,\"offset\":256,"
      "\"scale\":2741,\"depth\":64,\"period\":120}}\n]}\n";
  check(grim_genome_serialize(one_gene) == want, "genome_json_canonical_order");

  // A genome changes hash when a parameter changes.
  GrimGenome tweaked = one_gene;
  tweaked.genes[0].params[0] = 1;
  check(grim_genome_hash(tweaked) != grim_genome_hash(one_gene), "genome_hash_sensitive");

  // Generator is a pure function of (seed, params): pin one hash across compilers.
  GrimRandomParams rp;
  const u64 h1 = grim_genome_hash(grim_random_genome(1, rp));
  const u64 h1b = grim_genome_hash(grim_random_genome(1, rp));
  check(h1 == h1b && h1 != grim_genome_hash(grim_random_genome(2, rp)), "random_genome_pure");
  constexpr u64 kSeed1Hash = 0x8D86B610CE07F95Bull; // same on every compiler
  std::printf("GRIM_GENE_TEST INFO random_genome_seed1_hash=%s\n", hex(h1).c_str());
  if (kSeed1Hash != 0) {
    check(h1 == kSeed1Hash, "random_genome_seed1_pinned", hex(h1));
  }
  // Every generated genome is valid and non-empty.
  bool valid = true;
  for (u64 seed = 0; seed < 400 && valid; ++seed) {
    GrimGenome gg = grim_random_genome(seed, rp);
    GrimGenome back;
    std::string err;
    valid = !gg.genes.empty() && gg.genes.size() <= rp.max_genes &&
            grim_genome_parse(grim_genome_serialize(gg), back, err);
  }
  check(valid, "random_genome_always_valid");
}

void test_genome_strict() {
  GrimGenome g;
  GrimGene gene = grim_default_gene(GrimGeneType::SpuPitch);
  gene.target = 255;
  gene.seed = 5;
  g.genes.push_back(gene);
  const std::string base = grim_genome_serialize(g);
  const auto mutate = [&](const std::string &from, const std::string &to) {
    std::string s = base;
    const size_t pos = s.find(from);
    if (pos == std::string::npos) {
      std::printf("GRIM_GENE_TEST BUG mutation anchor missing: %s\n", from.c_str());
      ++g_failures;
      return s;
    }
    s.replace(pos, from.size(), to);
    return s;
  };
  struct Case {
    const char *name;
    std::string json;
  };
  const std::vector<Case> bad = {
      {"unknown_gene_type", mutate("spu_pitch", "spu_bogus")},
      {"param_out_of_range", mutate("\"mode\":0", "\"mode\":9")},
      {"param_float", mutate("\"mode\":0", "\"mode\":0.5")},
      {"param_string", mutate("\"mode\":0", "\"mode\":\"0\"")},
      {"unknown_param", mutate("\"period\":120}", "\"period\":120,\"extra\":1}")},
      {"missing_param", mutate("\"depth\":64,", "")},
      {"unknown_trigger_kind", mutate("\"always\"", "\"sometimes\"")},
      {"target_zero", mutate("\"target\":255", "\"target\":0")},
      {"target_string", mutate("\"target\":255", "\"target\":\"255\"")},
      {"target_outside_mask", mutate("\"target\":255", "\"target\":33554432")},
      {"version_two", mutate("\"version\":1", "\"version\":2")},
      {"negative_seed", mutate("\"seed\":5", "\"seed\":-1")},
      {"unknown_gene_field", mutate("\"seed\":5,", "\"seed\":5,\"note\":1,")},
      {"trailing_garbage", base + "x"},
      {"not_json", "{"},
      {"top_level_array", "[]"},
      {"missing_genes", "{\"version\":1}"},
      {"unknown_top_key", mutate("{\"version\":1,", "{\"version\":1,\"name\":\"x\",")},
      {"window_end_before_start",
       mutate("{\"kind\":\"always\"}", "{\"kind\":\"window\",\"start_frame\":50,\"end_frame\":40}")},
      {"rot_extra_field",
       mutate("{\"kind\":\"always\"}",
              "{\"kind\":\"rot\",\"start_frame\":1,\"end_frame\":40,\"period\":3}")},
      {"intermittent_no_mode",
       mutate("{\"kind\":\"always\"}",
              "{\"kind\":\"intermittent\",\"start_frame\":0,\"end_frame\":0,\"period\":0,"
              "\"duty\":0,\"probability\":0}")},
      {"intermittent_duty_over_period",
       mutate("{\"kind\":\"always\"}",
              "{\"kind\":\"intermittent\",\"start_frame\":0,\"end_frame\":0,\"period\":4,"
              "\"duty\":5,\"probability\":0}")},
  };
  bool ok = true;
  for (const Case &c : bad) {
    GrimGenome out;
    std::string err;
    if (grim_genome_parse(c.json, out, err) || err.empty()) {
      ok = false;
      check(false, std::string("genome_strict_rejects_") + c.name);
    }
  }
  check(ok, "genome_strict_rejects_all", "cases=" + std::to_string(bad.size()));
  GrimGenome out;
  std::string err;
  check(grim_genome_parse(base, out, err), "genome_strict_accepts_valid", err);

  // Both static families use v2's resolved patch transport, with no runtime
  // target/trigger fields. Their strict checks must not regress v1 hashes.
  for (size_t t = 0; t < static_cast<size_t>(GrimGeneType::Count); ++t) {
    const GrimGeneType type = static_cast<GrimGeneType>(t);
    if (!grim_gene_is_rom(type)) {
      continue;
    }
    GrimGenome rg;
    rg.version = 2;
    rg.bios_hash = 0x123456789ABCDEF0ull;
    rg.genes.push_back(grim_default_gene(type));
    rg.genes[0].patches.push_back({32, 0x03021111u, 0x03021121u, false});
    const std::string valid_text = grim_genome_serialize(rg);
    GrimGenome parsed;
    err.clear();
    check(grim_genome_parse(valid_text, parsed, err) &&
              grim_genome_serialize(parsed) == valid_text,
          std::string(grim_gene_type_name(type)) + "_resolved_patches_roundtrip", err);
    const auto reject_change = [&](const char *label, const std::string &from, const std::string &to) {
      std::string text = valid_text;
      const size_t p = text.find(from);
      if (p == std::string::npos) {
        check(false, label, "missing fixture anchor");
        return;
      }
      text.replace(p, from.size(), to);
      GrimGenome rejected;
      std::string why;
      check(!grim_genome_parse(text, rejected, why) && !why.empty(),
            std::string(grim_gene_type_name(type)) + "_rejects_" + label, why);
    };
    reject_change("unaligned", "[[32,", "[[33,");
    reject_change("kind_range", "\"kind\":0", "\"kind\":100");
    reject_change("unknown_param", "\"count\":1", "\"count\":1,\"extra\":0");
    reject_change("runtime_target", "\"seed\":0", "\"target\":1,\"seed\":0");
    reject_change("version_one", "\"version\":2,\"bios_hash\":\"0x123456789ABCDEF0\"", "\"version\":1");
    if (type == GrimGeneType::SpuSample) {
      reject_change("delay_slot", ",0]]", ",1]]");
      reject_change("sample_index", "\"sample\":-1", "\"sample\":-2");
      reject_change("shift_tag", "\"emulator_shift\":0", "\"emulator_shift\":2");
      reject_change("block_phase", "\"block_phase\":0", "\"block_phase\":4");
      reject_change("header_alignment", "[[32,", "[[36,");
      reject_change("payload_header_change", "\"kind\":0", "\"kind\":9");
      reject_change("header_gene_payload", ",50467105,", ",50401569,");
      reject_change("wrong_header_field", "\"kind\":0", "\"kind\":1");
      GrimGenome shift = rg;
      shift.genes[0].params[0] = 1;
      shift.genes[0].params[5] = 1;
      shift.genes[0].patches[0].mutated = 0x0302111Du;
      const std::string tagged = grim_genome_serialize(shift);
      check(grim_genome_parse(tagged, parsed, err), "spu_sample_tagged_reserved_shift_valid", err);
      std::string untagged = tagged;
      const size_t tag = untagged.find("\"emulator_shift\":1");
      untagged.replace(tag, std::string("\"emulator_shift\":1").size(), "\"emulator_shift\":0");
      check(!grim_genome_parse(untagged, parsed, err), "spu_sample_untagged_reserved_shift_rejected", err);
    }
  }
}

// A static gene is verified and applied to the BIOS image. It has no GP0/SPU
// traffic semantics, so give it a real transport test rather than no-op fuzz.
void test_static_gene_transport(GrimGeneType type) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() /
      ("vibestation_grim_static_" + std::to_string(static_cast<u32>(type)) + "_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(dir);
  const fs::path file = dir / "fixture.bin";
  {
    std::ofstream out(file, std::ios::binary);
    const std::vector<char> image(psx::BIOS_SIZE, '\0');
    out.write(image.data(), static_cast<std::streamsize>(image.size()));
  }
  Bios bios;
  const bool loaded = bios.load(file.string());
  const std::string prefix = std::string(grim_gene_type_name(type));
  check(loaded, prefix + "_fixture_loaded");
  if (loaded) {
    GrimGenome genome;
    genome.version = 2;
    genome.bios_hash = bios.image_hash();
    GrimGene gene = grim_default_gene(type);
    const u32 first = type == GrimGeneType::SpuSample ? 0x10u : 0x11223344u;
    const u32 second = type == GrimGeneType::SpuSample ? 0x20u : 0x55667788u;
    gene.patches = {{0x20, 0, first, false}, {0x30, 0, second, false}};
    genome.genes.push_back(gene);
    std::string err;
    GrimGenomeRuntime rt(genome);
    check(rt.has_rom_genes() && rt.apply_rom(bios, err) &&
              bios.read32(0x20) == first && bios.read32(0x30) == second,
          prefix + "_verified_rom_application", err);
    bios.restore_original_image();
    genome.genes[0].patches[1].original = 1;
    GrimGenomeRuntime bad_word(genome);
    check(!bad_word.apply_rom(bios, err) && !err.empty() &&
              bios.read32(0x20) == 0 && bios.read32(0x30) == 0,
          prefix + "_original_mismatch_is_atomic", err);
    genome.genes[0].patches[1].original = 0;
    genome.bios_hash ^= 1;
    GrimGenomeRuntime bad_hash(genome);
    check(!bad_hash.apply_rom(bios, err) && !err.empty() &&
              bios.read32(0x20) == 0 && bios.read32(0x30) == 0,
          prefix + "_bios_mismatch_is_atomic", err);

    // The verification pass spans both families before any gene writes.
    genome.bios_hash = bios.image_hash();
    GrimGene other = grim_default_gene(type == GrimGeneType::RomCode ? GrimGeneType::SpuSample
                                                                   : GrimGeneType::RomCode);
    other.patches = {{0x40, 1, 0xAABBCCDD, false}};
    genome.genes.push_back(other);
    GrimGenomeRuntime mixed_bad(genome);
    check(!mixed_bad.apply_rom(bios, err) && bios.read32(0x20) == 0 &&
              bios.read32(0x30) == 0 && bios.read32(0x40) == 0,
          prefix + "_mixed_families_verify_before_writes", err);
  }
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// ---- triggers -------------------------------------------------------------------

void test_triggers() {
  bool ok = true;
  const GrimTrigger w = window(10, 20);
  ok = ok && grim_trigger_magnitude(w, 9, 0) == 0 && grim_trigger_magnitude(w, 10, 0) == 1024 &&
       grim_trigger_magnitude(w, 19, 0) == 1024 && grim_trigger_magnitude(w, 20, 0) == 0;
  check(ok, "trigger_window");

  const GrimTrigger r = rot(100, 200);
  ok = grim_trigger_magnitude(r, 100, 0) == 0 && grim_trigger_magnitude(r, 150, 0) == 512 &&
       grim_trigger_magnitude(r, 200, 0) == 1024 && grim_trigger_magnitude(r, 5000, 0) == 1024;
  u32 prev = 0;
  for (u32 f = 0; f < 400 && ok; ++f) {
    const u32 m = grim_trigger_magnitude(r, f, 0);
    ok = m >= prev && m <= 1024;
    prev = m;
  }
  check(ok, "trigger_rot_ramp_holds");

  // Rot scales a gene's effect: offset 100 at half ramp gives half.
  {
    GrimGene g = make_gene(GrimGeneType::SpuPitch, {{"mode", 1}, {"offset", 100}}, 0xFFFFFF);
    g.trigger = rot(0, 100);
    auto rt = make_rt({g}, 50);
    const u32 mid = one(spu(*rt, 0x04, 0x1000));
    rt->begin_frame(100);
    const u32 full = one(spu(*rt, 0x04, 0x1000));
    rt->begin_frame(0);
    const u32 start = one(spu(*rt, 0x04, 0x1000));
    check(mid == 0x1032 && full == 0x1064 && start == 0x1000, "trigger_rot_scales_gene",
          hex(start) + " " + hex(mid) + " " + hex(full));
  }

  const GrimTrigger p = pattern(10, 3, 5, 0);
  std::string pat;
  for (u32 f = 0; f < 30; ++f) {
    pat += grim_trigger_magnitude(p, f, 0) != 0 ? '1' : '0';
  }
  check(pat == "000001110000000111000000011100", "trigger_intermittent_pattern", pat);
  const GrimTrigger gated = pattern(10, 3, 5, 20);
  check(grim_trigger_magnitude(gated, 3, 0) == 0 && grim_trigger_magnitude(gated, 5, 0) == 1024 &&
            grim_trigger_magnitude(gated, 20, 0) == 0,
        "trigger_intermittent_window_gate");

  // Probability mode is decided by the gene's RNG draw and is reproducible.
  const GrimTrigger pr = chance(300);
  GrimRng a{99}, b{99};
  u32 on = 0;
  bool same = true;
  for (int i = 0; i < 20000; ++i) {
    const bool x = grim_trigger_magnitude(pr, 0, a.next()) != 0;
    const bool y = grim_trigger_magnitude(pr, 0, b.next()) != 0;
    same = same && x == y;
    on += x ? 1u : 0u;
  }
  check(same && on > 5400 && on < 6600, "trigger_intermittent_probability",
        "on=" + std::to_string(on) + "/20000");
}

// ---- SPU filter -----------------------------------------------------------------

void test_spu_filters() {
  // pitch
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuPitch, {{"mode", 0}, {"mul_q8", 512}}, 1u << 3)});
    check(is_one(spu(*rt, 0x34, 0x1000), 0x34, 0x2000), "spu_pitch_multiply");
    check(is_one(spu(*rt, 0x34, 0x3000), 0x34, 0x3FFF), "spu_pitch_multiply_clamps");
    check(is_one(spu(*rt, 0x44, 0x1000), 0x44, 0x1000), "spu_pitch_other_voice_untouched");
    check(is_one(spu(*rt, 0x30, 0x1000), 0x30, 0x1000), "spu_pitch_other_register_untouched");
  }
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuPitch, {{"mode", 1}, {"offset", 100}})});
    check(is_one(spu(*rt, 0x04, 0x1000), 0x04, 0x1064), "spu_pitch_offset");
    auto rt2 = make_rt({make_gene(GrimGeneType::SpuPitch, {{"mode", 1}, {"offset", -100}})});
    check(is_one(spu(*rt2, 0x04, 10), 0x04, 0), "spu_pitch_offset_floors_at_zero");
  }
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuPitch, {{"mode", 2}, {"scale", 1}})});
    const bool a = is_one(spu(*rt, 0x04, 0x10C0), 0x04, 0x1000);
    const bool b = is_one(spu(*rt, 0x04, 0x1E00), 0x04, 0x2000);
    const bool c = is_one(spu(*rt, 0x04, 0x0900), 0x04, 0x0800);
    auto all = make_rt({make_gene(GrimGeneType::SpuPitch, {{"mode", 2}, {"scale", 0xFFF}})});
    const bool d = is_one(spu(*all, 0x04, 4340), 0x04, 4340); // exactly one semitone up
    check(a && b && c && d, "spu_pitch_quantize", std::to_string(a) + std::to_string(b) +
                                                       std::to_string(c) + std::to_string(d));
  }
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuPitch,
                                 {{"mode", 3}, {"depth", 1024}, {"period", 100}})},
                      25); // triangle peak: +100%
    const bool up = is_one(spu(*rt, 0x04, 0x1000), 0x04, 0x2000);
    rt->begin_frame(0);
    const bool zero = is_one(spu(*rt, 0x04, 0x1000), 0x04, 0x1000);
    rt->begin_frame(75); // trough: -100%
    const bool down = is_one(spu(*rt, 0x04, 0x1000), 0x04, 0x0000);
    check(up && zero && down, "spu_pitch_wobble");
  }
  // adsr
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuAdsr, {{"mode", 0}})});
    const u32 hi = one(spu(*rt, 0x0A, 0x0000));
    const u32 lo = one(spu(*rt, 0x08, 0x0000));
    check(((hi >> 8) & 0x1F) == 0x1F && (hi & 0x4000) != 0 && (hi & 0x8000) == 0 &&
              (lo & 0xF) == 0xF,
          "spu_adsr_infinite_sustain", hex(hi) + " " + hex(lo));
  }
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuAdsr, {{"mode", 1}})});
    const u32 hi = one(spu(*rt, 0x0A, 0x003F));
    check((hi & 0x3F) == 0, "spu_adsr_instant_release", hex(hi));
  }
  {
    bool inside = true, changed = false;
    auto rt = make_rt({make_gene(GrimGeneType::SpuAdsr, {{"mode", 2}})});
    for (int i = 0; i < 100; ++i) {
      const u32 out = one(spu(*rt, 0x08, 0x5555));
      inside = inside && ((out ^ 0x5555) & 0x00FF) == 0;
      changed = changed || out != 0x5555;
    }
    check(inside && changed, "spu_adsr_attack_mangle_stays_in_field");
  }
  // volume
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuVolume, {{"mode", 0}})});
    const bool a = is_one(spu(*rt, 0x30, 0x1234), 0x32, 0x1234);
    const bool b = is_one(spu(*rt, 0x32, 0x0777), 0x30, 0x0777);
    const bool c = is_one(spu(*rt, 0x180, 0x3FFF), 0x180, 0x3FFF); // main vol: not targeted
    auto with_main =
        make_rt({make_gene(GrimGeneType::SpuVolume, {{"mode", 0}}, 0x1FFFFFF)});
    const bool d = is_one(spu(*with_main, 0x180, 0x3FFF), 0x182, 0x3FFF);
    check(a && b && c && d, "spu_volume_swap_lr");
  }
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuVolume, {{"mode", 1}})});
    const bool a = is_one(spu(*rt, 0x30, 0x1000), 0x30, 0x7000); // +0x1000 -> -0x1000
    const bool b = is_one(spu(*rt, 0x30, 0x9000), 0x30, 0x9000); // sweep mode: untouched
    const bool c = is_one(spu(*rt, 0x30, 0x4000), 0x30, 0x3FFF); // -0x4000 -> +0x3FFF
    const bool d = is_one(spu(*rt, 0x30, 0), 0x30, 0);
    check(a && b && c && d, "spu_volume_invert");
  }
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuVolume, {{"mode", 2}, {"clamp", 0x1000}})});
    const bool a = is_one(spu(*rt, 0x30, 0x2000), 0x30, 0x1000);
    const bool b = is_one(spu(*rt, 0x30, 0x6000), 0x30, 0x7000); // -0x2000 -> -0x1000
    const bool c = is_one(spu(*rt, 0x30, 0x0800), 0x30, 0x0800);
    check(a && b && c, "spu_volume_clamp");
  }
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuVolume,
                                 {{"mode", 3}, {"depth", 1024}, {"period", 100}})},
                      25);
    const bool zero = is_one(spu(*rt, 0x30, 0x2000), 0x30, 0);
    rt->begin_frame(75);
    const bool full = is_one(spu(*rt, 0x30, 0x2000), 0x30, 0x2000);
    rt->begin_frame(50);
    const bool half = is_one(spu(*rt, 0x30, 0x2000), 0x30, 0x1000);
    check(zero && full && half, "spu_volume_sweep");
  }
  // address
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuAddress, {{"which", 0}, {"blocks", 16}})});
    const bool a = is_one(spu(*rt, 0x36, 0x1000), 0x36, 0x1010);
    const bool b = is_one(spu(*rt, 0x3E, 0x1000), 0x3E, 0x1000);
    const bool c = is_one(spu(*rt, 0x36, 0xFFF8), 0x36, 0x0008); // stays inside 512 KiB
    auto rep = make_rt({make_gene(GrimGeneType::SpuAddress, {{"which", 1}, {"blocks", -16}})});
    const bool d = is_one(spu(*rep, 0x3E, 0x1000), 0x3E, 0x0FF0);
    const bool e = is_one(spu(*rep, 0x36, 0x1000), 0x36, 0x1000);
    check(a && b && c && d && e, "spu_address_offset");
    auto rnd = make_rt({make_gene(GrimGeneType::SpuAddress,
                                  {{"which", 2}, {"blocks", 50}, {"random", 1}})});
    bool within = true, moved = false;
    for (int i = 0; i < 200; ++i) {
      const u32 v = one(spu(*rnd, 0x36, 0x8000));
      within = within && v >= 0x8000 - 50 && v <= 0x8000 + 50;
      moved = moved || v != 0x8000;
    }
    check(within && moved, "spu_address_random_bounded");
  }
  // key-on
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuKeyOn, {{"mode", 0}, {"prob", 1000}})});
    check(is_one(spu(*rt, 0x188, 0x0003), 0x188, 0), "spu_keyon_drop_all");
    check(is_one(spu(*rt, 0x18C, 0x0003), 0x18C, 0x0003), "spu_keyon_ignores_keyoff_by_default");
    auto one_voice =
        make_rt({make_gene(GrimGeneType::SpuKeyOn, {{"mode", 0}, {"prob", 1000}}, 1u)});
    check(is_one(spu(*one_voice, 0x188, 0x0003), 0x188, 0x0002), "spu_keyon_drop_targeted_voice");
    auto high = make_rt({make_gene(GrimGeneType::SpuKeyOn, {{"mode", 0}, {"prob", 1000}},
                                   1u << 16)});
    const bool a = is_one(spu(*high, 0x18A, 0x0005), 0x18A, 0x0004); // KON high half
    const bool b = is_one(spu(*high, 0x188, 0x0005), 0x188, 0x0005); // low half: not targeted
    check(a && b, "spu_keyon_halves_map_to_voices");
    auto koff = make_rt({make_gene(GrimGeneType::SpuKeyOn,
                                   {{"mode", 0}, {"keys", 1}, {"prob", 1000}})});
    check(is_one(spu(*koff, 0x18C, 0x00FF), 0x18C, 0) && is_one(spu(*koff, 0x188, 0x00FF), 0x188, 0xFF),
          "spu_keyon_keyoff_mode");
  }
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuKeyOn,
                                 {{"mode", 1}, {"prob", 1000}, {"delay", 10}})});
    const bool pass = is_one(spu(*rt, 0x188, 0x0003, 1000), 0x188, 0x0003);
    const bool queued = rt->has_delayed_spu() && rt->next_delayed_due() == 1000u + 10u * 768u;
    const GrimSpuWrite w = rt->pop_delayed_spu();
    check(pass && queued && w.offset == 0x188 && w.value == 0x0003 && !rt->has_delayed_spu(),
          "spu_keyon_duplicate_queues_copy");
  }
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuKeyOn,
                                 {{"mode", 2}, {"prob", 1000}, {"delay", 4}})});
    const bool held = is_one(spu(*rt, 0x18A, 0x0006, 500), 0x18A, 0);
    const bool queued = rt->has_delayed_spu() && rt->next_delayed_due() == 500u + 4u * 768u;
    const GrimSpuWrite w = rt->pop_delayed_spu();
    check(held && queued && w.offset == 0x18A && w.value == 0x0006, "spu_keyon_delay_holds_bits");
    // Queue order: earlier due first, ties keep insertion order.
    auto rt2 = make_rt({make_gene(GrimGeneType::SpuKeyOn,
                                  {{"mode", 2}, {"prob", 1000}, {"delay", 4}})});
    spu(*rt2, 0x188, 1, 2000);
    spu(*rt2, 0x188, 2, 1000);
    spu(*rt2, 0x188, 4, 1000);
    const u16 first = rt2->pop_delayed_spu().value, second = rt2->pop_delayed_spu().value,
              third = rt2->pop_delayed_spu().value;
    check(first == 2 && second == 4 && third == 1, "spu_keyon_delay_queue_order");
  }
  // noise, pmon
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuNoise, {{"piggyback", 0}}, 0x0F)});
    check(is_one(spu(*rt, 0x194, 0x0000), 0x194, 0x000F), "spu_noise_forces_bits");
    auto hi = make_rt({make_gene(GrimGeneType::SpuNoise, {{"piggyback", 0}}, 0x0F0000)});
    check(is_one(spu(*hi, 0x196, 0x0000), 0x196, 0x000F), "spu_noise_high_half");
    auto pig = make_rt({make_gene(GrimGeneType::SpuNoise, {{"piggyback", 1}}, 0xFFFFFF)});
    const auto w1 = spu(*pig, 0x188, 0x0004);
    const auto w2 = spu(*pig, 0x188, 0x0008);
    const bool a = w1.size() == 2 && w1[0].offset == 0x194 && w1[0].value == 0x0004 &&
                   w1[1].offset == 0x188 && w1[1].value == 0x0004;
    const bool b = w2.size() == 2 && w2[0].offset == 0x194 && w2[0].value == 0x000C; // shadowed
    check(a && b, "spu_noise_piggybacks_on_keyon");
    auto pm = make_rt({make_gene(GrimGeneType::SpuPmon, {{"piggyback", 0}}, 0x3)});
    check(is_one(spu(*pm, 0x190, 0x0000), 0x190, 0x0002), "spu_pmon_skips_voice0");
  }
  // reverb
  {
    auto rt = make_rt({make_gene(GrimGeneType::SpuReverb, {{"cfg", 1000}, {"base", 32}, {"eon", 1},
                                                            {"spucnt", 1}}, 0xF)});
    bool cfg_changed = false;
    for (int i = 0; i < 50; ++i) {
      cfg_changed = cfg_changed || one(spu(*rt, 0x1C0, 0x1234)) != 0x1234;
    }
    check(cfg_changed, "spu_reverb_config_mutates");
    check(is_one(spu(*rt, 0x1A2, 0x1000), 0x1A2, 0x1020), "spu_reverb_moves_work_area");
    check(is_one(spu(*rt, 0x198, 0x0000), 0x198, 0x000F), "spu_reverb_forces_eon");
    check(is_one(spu(*rt, 0x1AA, 0x8000), 0x1AA, 0x8080), "spu_reverb_forces_spucnt_bit");
  }
  // Fuzz: any gene, any offset, any value: bounded output, valid offsets.
  {
    bool ok = true;
    GrimRng rng{2024};
    for (int round = 0; round < 400 && ok; ++round) {
      GrimGene g = grim_default_gene(static_cast<GrimGeneType>(rng.range(0, 7)));
      const auto &schema = grim_gene_schema(g.type);
      for (size_t i = 0; i < schema.size(); ++i) {
        g.params[i] = rng.srange(schema[i].lo, schema[i].hi);
      }
      g.seed = rng.next();
      auto rt = make_rt({g}, rng.range(0, 3000));
      for (int i = 0; i < 300 && ok; ++i) {
        const u32 off = rng.range(0, 0x1FF) & ~1u;
        GrimSpuWrite out[GrimGenomeRuntime::kMaxSpuOut];
        const size_t n = rt->filter_spu_write(off, static_cast<u16>(rng.next()),
                                              rng.next() >> 20, out);
        ok = n <= GrimGenomeRuntime::kMaxSpuOut;
        for (size_t k = 0; k < n; ++k) {
          ok = ok && out[k].offset < 0x400 && (out[k].offset & 1u) == 0u;
        }
        if (g.type == GrimGeneType::SpuPitch && off < 0x180 && (off & 0xF) == 4 && n == 1) {
          ok = ok && out[0].value <= 0x3FFF;
        }
      }
    }
    check(ok, "spu_filter_fuzz_bounded");
  }
  // System level: 32-bit writes are split, both halves filtered, readback is
  // the transformed value, and KON halves are both seen.
  {
    auto sys = std::make_unique<System>();
    sys->reset();
    auto rt = make_rt({make_gene(GrimGeneType::SpuPitch, {{"mode", 0}, {"mul_q8", 512}}, 1u << 3),
                       make_gene(GrimGeneType::SpuAddress, {{"which", 0}, {"blocks", 16}}, 1u << 3),
                       make_gene(GrimGeneType::SpuKeyOn, {{"mode", 0}, {"prob", 1000}}, 1u)});
    sys->set_grim_genome(rt.get());
    // Voice 3: +4 pitch (low half), +6 start address (high half), one 32-bit write.
    sys->write32(0x1F801C34, (0x0100u << 16) | 0x1000u);
    const u16 pitch = sys->read16(0x1F801C34);
    const u16 start = sys->read16(0x1F801C36);
    const u64 kon_before = sys->spu_audio_diag().kon_bits_collected;
    sys->write32(0x1F801D88, 0x00050003u); // KON low = 0x0003, high = 0x0005
    const u64 kon_bits = sys->spu_audio_diag().kon_bits_collected - kon_before;
    check(pitch == 0x2000 && start == 0x0110, "spu_split_write32_both_halves_filtered",
          hex(pitch) + " " + hex(start));
    check(kon_bits == 3, "spu_kon_halves_via_write32",
          "bits=" + std::to_string(kon_bits) + " (4 without the drop-voice-0 gene)");
    // Off again: the same writes reach the SPU untouched.
    sys->set_grim_genome(nullptr);
    sys->write32(0x1F801C34, (0x0100u << 16) | 0x1000u);
    check(sys->read16(0x1F801C34) == 0x1000 && sys->read16(0x1F801C36) == 0x0100,
          "spu_genome_off_is_passthrough");
  }
}

// ---- GP0 filter -----------------------------------------------------------------

// Independent description of where things sit in a command (written from the
// GPU documentation, not from grim_genome.cpp).
struct Layout {
  size_t len = 3;
  std::vector<size_t> vert, color, uv;
  bool draw = false, textured = false;
  u32 klass = 0; // 1 poly, 2 line, 4 rect
};

Layout layout_of(u32 op) {
  Layout L;
  if (op >= 0x20 && op <= 0x3F) {
    const size_t tex = (op >> 2) & 1, gou = (op >> 4) & 1, nv = 3 + ((op >> 3) & 1);
    L.draw = true;
    L.klass = 1;
    L.textured = tex != 0;
    L.len = gou ? nv * (2 + tex) : 1 + nv * (1 + tex);
    for (size_t i = 0; i < nv; ++i) {
      const size_t c = gou ? i * (2 + tex) : 0;
      const size_t v = gou ? c + 1 : 1 + i * (1 + tex);
      if (gou || i == 0) {
        L.color.push_back(c);
      }
      L.vert.push_back(v);
      if (tex) {
        L.uv.push_back(v + 1);
      }
    }
  } else if (op >= 0x40 && op <= 0x5F) {
    const bool gou = (op >> 4) & 1;
    L.draw = true;
    L.klass = 2;
    L.len = gou ? 4 : 3;
    L.color = gou ? std::vector<size_t>{0, 2} : std::vector<size_t>{0};
    L.vert = gou ? std::vector<size_t>{1, 3} : std::vector<size_t>{1, 2};
  } else if (op >= 0x60 && op <= 0x7F) {
    const size_t tex = (op >> 2) & 1, size = (op >> 3) & 3;
    L.draw = true;
    L.klass = 4;
    L.textured = tex != 0;
    L.len = 2 + tex + (size == 0 ? 1 : 0);
    L.color = {0};
    L.vert = {1};
    if (tex) {
      L.uv = {2};
    }
  } else if (op == 0x02) {
    L.len = 3;
  } else if (op >= 0xE1 && op <= 0xE6) {
    L.len = 1;
  }
  return L;
}

bool contains(const std::vector<size_t> &v, size_t x) {
  return std::find(v.begin(), v.end(), x) != v.end();
}

// Bits of word `i` a gene of this type is allowed to change.
u32 allowed_mask(GrimGeneType t, u32 op, const Layout &L, size_t i) {
  switch (t) {
  case GrimGeneType::GpuVertex:
    return contains(L.vert, i) ? 0x07FF07FFu : 0u;
  case GrimGeneType::GpuColor:
    return contains(L.color, i) ? 0x00FFFFFFu : 0u;
  case GrimGeneType::GpuFlags:
    return i == 0 && L.draw ? (L.textured ? 0x03000000u : 0x02000000u) : 0u;
  case GrimGeneType::GpuTexParam:
    return contains(L.uv, i) && L.uv.size() > 0 && (i == L.uv[0] || (L.uv.size() > 1 && i == L.uv[1]))
               ? 0xFFFF0000u
               : 0u;
  case GrimGeneType::GpuState:
    return i == 0 && op >= 0xE1 && op <= 0xE6 ? 0x00FFFFFFu : 0u;
  case GrimGeneType::GpuFill:
    return op == 0x02 ? (i == 0 ? 0x00FFFFFFu : 0x01FF03FFu) : 0u;
  default:
    return 0u;
  }
}

std::vector<u32> random_packet(u32 op, const Layout &L, GrimRng &rng) {
  std::vector<u32> w(L.len);
  for (u32 &x : w) {
    x = static_cast<u32>(rng.next());
  }
  w[0] = (op << 24) | (w[0] & 0x00FFFFFFu);
  return w;
}

u32 V(s32 x, s32 y) {
  return (static_cast<u32>(x) & 0x7FFu) | ((static_cast<u32>(y) & 0x7FFu) << 16);
}
s32 vx(u32 w) { return static_cast<s32>((w & 0x7FFu) ^ 0x400u) - 0x400; }
s32 vy(u32 w) { return vx(w >> 16); }

// Runs the filter on a packet with a sentinel behind it (catches overruns).
std::vector<u32> filter(GrimGenomeRuntime &rt, std::vector<u32> w, bool *sentinel_ok = nullptr) {
  const size_t n = w.size();
  w.push_back(0xDEADBEEFu);
  rt.filter_gp0(w.data(), n);
  if (sentinel_ok != nullptr) {
    *sentinel_ok = w[n] == 0xDEADBEEFu;
  }
  w.resize(n);
  return w;
}

void test_gp0_transforms() {
  // vertex
  {
    auto rt = make_rt({make_gene(GrimGeneType::GpuVertex,
                                 {{"mode", 0}, {"dx", 5}, {"dy", -3}, {"wrap", 0}})});
    const auto out = filter(*rt, {0x20112233u, V(10, 20), V(-4, 7), V(100, 100)});
    check(out.size() == 4 && vx(out[1]) == 15 && vy(out[1]) == 17 && vx(out[2]) == 1 &&
              vy(out[2]) == 4 && out[0] == 0x20112233u,
          "gp0_vertex_fixed_jitter");
    auto clamp = make_rt({make_gene(GrimGeneType::GpuVertex,
                                    {{"mode", 0}, {"dx", 100}, {"dy", 0}, {"wrap", 0}})});
    auto wrap = make_rt({make_gene(GrimGeneType::GpuVertex,
                                   {{"mode", 0}, {"dx", 100}, {"dy", 0}, {"wrap", 1}})});
    const auto c = filter(*clamp, {0x20000000u, V(1000, 0), V(0, 0), V(0, 0)});
    const auto w = filter(*wrap, {0x20000000u, V(1000, 0), V(0, 0), V(0, 0)});
    check(vx(c[1]) == 1023 && vx(w[1]) == -948 && vx(c[2]) == 100, "gp0_vertex_clamp_vs_wrap",
          std::to_string(vx(c[1])) + " " + std::to_string(vx(w[1])));
    auto noise = make_rt({make_gene(GrimGeneType::GpuVertex, {{"mode", 1}, {"amp", 8}})});
    bool within = true, moved = false;
    for (int i = 0; i < 100; ++i) {
      const auto n = filter(*noise, {0x20000000u, V(0, 0), V(0, 0), V(0, 0)});
      for (size_t k = 1; k < 4; ++k) {
        within = within && std::abs(vx(n[k])) <= 8 && std::abs(vy(n[k])) <= 8;
        moved = moved || n[k] != 0;
      }
    }
    check(within && moved, "gp0_vertex_noise_bounded");
    auto drift = make_rt({make_gene(GrimGeneType::GpuVertex,
                                    {{"mode", 2}, {"dx", 60}, {"dy", -30}})}, 60);
    const auto d = filter(*drift, {0x20000000u, V(0, 0), V(0, 0), V(0, 0)});
    drift->begin_frame(30);
    const auto d2 = filter(*drift, {0x20000000u, V(0, 0), V(0, 0), V(0, 0)});
    check(vx(d[1]) == 60 && vy(d[1]) == -30 && vx(d2[1]) == 30 && vy(d2[1]) == -15,
          "gp0_vertex_drift_follows_frames");
    auto snap = make_rt({make_gene(GrimGeneType::GpuVertex, {{"mode", 3}, {"grid", 8}})});
    const auto s = filter(*snap, {0x20000000u, V(13, -5), V(11, 3), V(-9, 4)});
    check(vx(s[1]) == 16 && vy(s[1]) == -8 && vx(s[2]) == 8 && vy(s[2]) == 0 && vx(s[3]) == -8,
          "gp0_vertex_snap_to_grid");
    auto swap = make_rt({make_gene(GrimGeneType::GpuVertex, {{"mode", 4}, {"prob", 1000}})});
    const std::vector<u32> tri = {0x30000000u | 0x00112233u, V(1, 2), 0x00445566u, V(3, 4),
                                  0x00778899u, V(5, 6)};
    const auto sw = filter(*swap, tri);
    std::vector<u32> a = {tri[1], tri[3], tri[5]}, b = {sw[1], sw[3], sw[5]};
    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());
    check(a == b && (sw[1] != tri[1] || sw[3] != tri[3] || sw[5] != tri[5]) && sw[0] == tri[0] &&
              sw[2] == tri[2] && sw[4] == tri[4],
          "gp0_vertex_swap_keeps_colors_and_set");
    // rot scales the jitter
    GrimGene g = make_gene(GrimGeneType::GpuVertex, {{"mode", 0}, {"dx", 100}, {"dy", 0}});
    g.trigger = rot(0, 100);
    auto rr = make_rt({g}, 50);
    check(vx(filter(*rr, {0x20000000u, V(0, 0), V(0, 0), V(0, 0)})[1]) == 50, "gp0_vertex_rot_scales");
  }
  // color
  {
    auto inv = make_rt({make_gene(GrimGeneType::GpuColor, {{"mode", 1}})});
    check(filter(*inv, {0x20332211u, 1, 2, 3})[0] == 0x20CCDDEEu, "gp0_color_invert_keeps_opcode");
    auto perm = make_rt({make_gene(GrimGeneType::GpuColor, {{"mode", 0}, {"perm", 5}})});
    check(filter(*perm, {0x20332211u, 1, 2, 3})[0] == 0x20112233u, "gp0_color_channel_swap");
    auto tint = make_rt({make_gene(GrimGeneType::GpuColor, {{"mode", 3}, {"tint", 0x0F0F0F}})});
    check(filter(*tint, {0x20332211u, 1, 2, 3})[0] == 0x203C2D1Eu, "gp0_color_tint");
    auto grad = make_rt({make_gene(GrimGeneType::GpuColor, {{"mode", 2}})});
    const std::vector<u32> tri = {0x30000011u, V(0, 0), 0x00000022u, V(1, 1), 0x00000033u, V(2, 2)};
    const auto g = filter(*grad, tri);
    const bool fwd = g[0] == 0x30000022u && g[2] == 0x33u && g[4] == 0x11u;
    const bool back = g[0] == 0x30000033u && g[2] == 0x11u && g[4] == 0x22u;
    check((fwd || back) && g[1] == tri[1] && g[3] == tri[3] && g[5] == tri[5], "gp0_color_gradient_shuffle");
    auto gou_inv = make_rt({make_gene(GrimGeneType::GpuColor, {{"mode", 1}})});
    const auto gi = filter(*gou_inv, tri);
    check(gi[2] == 0x00FFFFDDu && gi[4] == 0x00FFFFCCu && gi[1] == tri[1], "gp0_color_all_vertex_colors");
  }
  // flags
  {
    auto set = make_rt({make_gene(GrimGeneType::GpuFlags, {{"semi", 1}, {"raw", 0}, {"prob", 1000}})});
    check((filter(*set, {0x20000000u, 0, 0, 0})[0] >> 24) == 0x22, "gp0_flags_set_semi");
    auto clear = make_rt({make_gene(GrimGeneType::GpuFlags, {{"semi", 2}, {"raw", 0}, {"prob", 1000}})});
    check((filter(*clear, {0x22000000u, 0, 0, 0})[0] >> 24) == 0x20, "gp0_flags_clear_semi");
    auto tog = make_rt({make_gene(GrimGeneType::GpuFlags, {{"semi", 3}, {"raw", 3}, {"prob", 1000}})});
    const auto poly = filter(*tog, {0x20000000u, 0, 0, 0});
    const auto tex = filter(*tog, {0x24000000u, 0, 0, 0, 0, 0, 0});
    const auto rect = filter(*tog, {0x65000000u, 0, 0, 0});
    check((poly[0] >> 24) == 0x22 && (tex[0] >> 24) == 0x27 && (rect[0] >> 24) == 0x66,
          "gp0_flags_raw_bit_only_when_textured");
    auto never = make_rt({make_gene(GrimGeneType::GpuFlags, {{"semi", 3}, {"raw", 3}, {"prob", 0}})});
    check(filter(*never, {0x24000000u, 0, 0, 0, 0, 0, 0})[0] == 0x24000000u, "gp0_flags_prob_zero");
  }
  // texparam
  {
    auto rt = make_rt({make_gene(GrimGeneType::GpuTexParam,
                                 {{"which", 2}, {"clut_mask", 0x3F}, {"tp_mask", 0x0F}, {"prob", 1000}})});
    bool clut_ok = true, tp_ok = true, changed = false;
    for (int i = 0; i < 50; ++i) {
      const std::vector<u32> in = {0x24000000u, V(0, 0), 0x12341122u, V(1, 1), 0x56781133u, V(2, 2), 0x00001144u};
      const auto out = filter(*rt, in);
      clut_ok = clut_ok && (out[2] & 0xFFFF) == 0x1122 && ((out[2] ^ in[2]) >> 16) <= 0x3F;
      tp_ok = tp_ok && (out[4] & 0xFFFF) == 0x1133 && ((out[4] ^ in[4]) >> 16) <= 0x0F &&
              out[6] == in[6] && out[1] == in[1];
      changed = changed || out != in;
    }
    check(clut_ok && tp_ok && changed, "gp0_texparam_clut_and_texpage_only");
    const std::vector<u32> flat = {0x20000000u, 1, 2, 3};
    check(filter(*rt, flat) == flat, "gp0_texparam_ignores_untextured");
  }
  // state
  {
    auto rt = make_rt({make_gene(GrimGeneType::GpuState,
                                 {{"amp", 0}, {"drift", 60}, {"e1_mask", 0x1F}, {"e2_mask", 0x1F}, {"e6_mask", 3}},
                                 (1u << 2) | (1u << 4) | (1u << 5))},
                      60);
    // E3 area top-left (10,10) drifts to (70,70); E5 offset (-8, 4) to (52, 64); E6 flips both bits.
    const u32 e3 = filter(*rt, {0xE3000000u | 10 | (10u << 10)})[0];
    const u32 e5 = filter(*rt, {0xE5000000u | (static_cast<u32>(-8) & 0x7FF) | (4u << 11)})[0];
    const u32 e6 = filter(*rt, {0xE6000000u | 0x00000001u})[0];
    const u32 e1 = filter(*rt, {0xE1000000u | 0x00000001u})[0]; // E1 not targeted
    check((e3 & 0x3FF) == 70 && ((e3 >> 10) & 0x1FF) == 70 && (e3 >> 24) == 0xE3, "gp0_state_e3_drift");
    check(vx(e5) == 52 && vx(e5 >> 11) == 64 && (e5 >> 24) == 0xE5, "gp0_state_e5_offset_drift",
          std::to_string(vx(e5)) + "," + std::to_string(vx(e5 >> 11)));
    check((e6 & 3) == 2 && (e6 >> 24) == 0xE6, "gp0_state_e6_mask_bits");
    check(e1 == 0xE1000001u, "gp0_state_respects_target");
    auto xr = make_rt({make_gene(GrimGeneType::GpuState,
                                 {{"amp", 0}, {"drift", 0}, {"e1_mask", 0x3FFF}, {"e2_mask", 0xFFFFF}, {"e6_mask", 3}})});
    bool e2_changed = false, top_ok = true;
    for (int i = 0; i < 50; ++i) {
      const u32 v = filter(*xr, {0xE2000000u})[0];
      e2_changed = e2_changed || (v & 0xFFFFF) != 0;
      top_ok = top_ok && (v >> 24) == 0xE2 && (v & 0x00F00000u) == 0;
    }
    check(e2_changed && top_ok, "gp0_state_e2_texture_window_bits");
  }
  // fill
  {
    auto rt = make_rt({make_gene(GrimGeneType::GpuFill, {{"color_mask", 0xFF}, {"amp", 0}})});
    bool low_only = true, changed = false;
    for (int i = 0; i < 50; ++i) {
      const auto out = filter(*rt, {0x02123456u, 0x00100010u, 0x00200020u});
      low_only = low_only && (out[0] & 0xFFFFFF00u) == 0x02123400u && out[1] == 0x00100010u && out[2] == 0x00200020u;
      changed = changed || out[0] != 0x02123456u;
    }
    check(low_only && changed, "gp0_fill_color_only");
    auto rect = make_rt({make_gene(GrimGeneType::GpuFill, {{"color_mask", 0}, {"amp", 16}})});
    bool moved = false, safe = true;
    for (int i = 0; i < 50; ++i) {
      const auto out = filter(*rect, {0x02123456u, 0x00100010u, 0x00200020u});
      moved = moved || out[1] != 0x00100010u || out[2] != 0x00200020u;
      safe = safe && out[0] == 0x02123456u && (out[1] & 0x3FFu) <= 1023u && ((out[1] >> 16) & 0x1FFu) <= 511u;
    }
    check(moved && safe, "gp0_fill_rect_perturbation");
  }
  // polyline tail words
  {
    auto v = make_rt({make_gene(GrimGeneType::GpuVertex, {{"mode", 0}, {"dx", 7}, {"dy", 0}})});
    const u32 term = 0x55555555u;
    check(v->filter_gp0_polyline_word(term, false, false) == term, "gp0_polyline_terminator_passes");
    check(vx(v->filter_gp0_polyline_word(V(10, 10), false, false)) == 17, "gp0_polyline_vertex_jitter");
    auto c = make_rt({make_gene(GrimGeneType::GpuColor, {{"mode", 1}})});
    check(c->filter_gp0_polyline_word(0x00112233u, true, true) == 0x00EEDDCCu &&
              c->filter_gp0_polyline_word(V(1, 1), true, false) == V(1, 1),
          "gp0_polyline_color_word_only");
    bool ok = true;
    GrimRng rng{77};
    auto fz = make_rt({make_gene(GrimGeneType::GpuVertex, {{"mode", 1}, {"amp", 500}, {"wrap", 1}}),
                       make_gene(GrimGeneType::GpuColor, {{"mode", 1}})});
    for (int i = 0; i < 20000 && ok; ++i) {
      u32 w = static_cast<u32>(rng.next());
      if ((i & 3) == 0) {
        w = (w & ~0xF000F000u) | 0x50005000u; // terminator shape
      }
      const bool was = (w & 0xF000F000u) == 0x50005000u;
      const u32 out = fz->filter_gp0_polyline_word(w, true, (i & 1) != 0);
      const bool now = (out & 0xF000F000u) == 0x50005000u;
      ok = was == now && (!was || out == w);
    }
    check(ok, "gp0_polyline_never_creates_or_eats_terminators");
  }

  // Fuzz: every gene type on every opcode with random parameters.
  {
    bool ok = true;
    std::string detail;
    u64 changed_by_type[static_cast<size_t>(GrimGeneType::Count)] = {};
    GrimRng rng{555};
    for (size_t t = static_cast<size_t>(GrimGeneType::GpuVertex); t < static_cast<size_t>(GrimGeneType::Count) && ok; ++t) {
      const GrimGeneType type = static_cast<GrimGeneType>(t);
      if (grim_gene_is_rom(type)) {
        test_static_gene_transport(type);
        continue;
      }
      if (grim_gene_is_hardware(type)) {
        continue; // memory faults, not GP0 traffic: --grim-pull-test covers them
      }
      if (type == GrimGeneType::MdecFmv) {
        continue; // MDEC traffic, not GP0: test_fmv() covers it
      }
      for (int round = 0; round < 40 && ok; ++round) {
        GrimGene g = grim_default_gene(type);
        const auto &schema = grim_gene_schema(type);
        for (size_t i = 0; i < schema.size(); ++i) {
          g.params[i] = rng.srange(schema[i].lo, schema[i].hi);
        }
        g.seed = rng.next();
        const u32 full = grim_gene_default_target(type);
        g.target = round % 3 == 0 ? full : (static_cast<u32>(rng.next()) & full) | (full & (0u - full));
        auto rt = make_rt({g}, rng.range(0, 5000));
        for (u32 op = 0; op < 256 && ok; ++op) {
          const Layout L = layout_of(op);
          for (int rep = 0; rep < 3 && ok; ++rep) {
            const std::vector<u32> in = random_packet(op, L, rng);
            bool sentinel = false;
            const std::vector<u32> out = filter(*rt, in, &sentinel);
            if (!sentinel || out.size() != in.size()) {
              ok = false;
              detail = "word count / overrun op=" + hex(op);
              break;
            }
            for (size_t i = 0; i < in.size() && ok; ++i) {
              if (((in[i] ^ out[i]) & ~allowed_mask(type, op, L, i)) != 0) {
                ok = false;
                detail = std::string(grim_gene_type_name(type)) + " op=" + hex(op) +
                         " word=" + std::to_string(i) + " " + hex(in[i]) + "->" + hex(out[i]);
              }
              changed_by_type[t] += in[i] != out[i] ? 1u : 0u;
            }
            // Word-count bits (26-31) are never touched; state/fill/transfer opcodes keep the whole byte.
            const u32 keep = L.draw ? 0xFC000000u : 0xFF000000u;
            if (ok && ((in[0] ^ out[0]) & keep) != 0) {
              ok = false;
              detail = "opcode bits changed op=" + hex(op);
            }
            if (ok && L.draw && !L.textured && ((in[0] ^ out[0]) & 0x01000000u) != 0) {
              ok = false;
              detail = "raw-texture bit touched on untextured op=" + hex(op);
            }
          }
        }
      }
    }
    check(ok, "gp0_fuzz_preserves_word_count_and_forbidden_bits", detail);
    bool all_active = true;
    for (size_t t = static_cast<size_t>(GrimGeneType::GpuVertex); t < static_cast<size_t>(GrimGeneType::Count); ++t) {
      if (!grim_gene_is_rom(static_cast<GrimGeneType>(t)) &&
          !grim_gene_is_hardware(static_cast<GrimGeneType>(t)) &&
          static_cast<GrimGeneType>(t) != GrimGeneType::MdecFmv) {
        all_active = all_active && changed_by_type[t] > 0;
      }
    }
    check(all_active, "gp0_fuzz_every_gene_type_changes_something");
  }
}

// ---- FMV (MDEC) ------------------------------------------------------------------------

void test_fmv() {
  // The engine never changes how much data the MDEC consumes: runs (bits 10-15) and
  // the 0xFE00 end/padding code survive any amount of corruption.
  {
    const GrimFmvKnobs k{kGrimFmvAll, 1000u, 1024u};
    GrimRng rng{77};
    bool ok = true;
    u32 changed = 0;
    std::string detail;
    for (int mb = 0; mb < 2000 && ok; ++mb) {
      GrimFmvMacroblock m;
      grim_fmv_begin(k, rng, m);
      for (u32 h = 0; h < 64 && ok; ++h) {
        const u16 in = h == 0 ? 0xFE00u : static_cast<u16>(rng.next());
        const u16 out = grim_fmv_coefficient(k, m, in);
        changed += in != out ? 1u : 0u;
        if ((in == 0xFE00u) != (out == 0xFE00u) || ((in ^ out) & 0xFC00u) != 0u) {
          ok = false;
          detail = hex(in) + "->" + hex(out);
        }
      }
    }
    check(ok && changed > 0, "fmv_coefficients_keep_runs_and_end_codes", detail);
  }
  // Rate 0 or no targets hits nothing, and output/blocks pass through untouched.
  {
    GrimRng rng{5};
    bool clean = true;
    for (const GrimFmvKnobs k : {GrimFmvKnobs{kGrimFmvAll, 0u, 1024u}, GrimFmvKnobs{0u, 1000u, 1024u}}) {
      for (int i = 0; i < 200; ++i) {
        GrimFmvMacroblock m;
        grim_fmv_begin(k, rng, m);
        int blocks[6 * 64];
        for (int &v : blocks) v = static_cast<int>(rng.range(0, 255)) - 128;
        int copy[6 * 64];
        std::copy(std::begin(blocks), std::end(blocks), copy);
        clean = clean && !m.hit && grim_fmv_output(k, m, 0x12345678u) == 0x12345678u &&
                !grim_fmv_blocks(k, m, blocks, 6) && std::equal(std::begin(blocks), std::end(blocks), copy);
      }
    }
    check(clean, "fmv_rate_zero_or_no_targets_is_clean");
  }
  // Quant entries stay usable (1..255) and only change when targeted.
  {
    GrimRng rng{9};
    std::array<u8, 64> q{};
    q.fill(16);
    const size_t none = grim_fmv_quant(GrimFmvKnobs{kGrimFmvCoeffs, 1000u, 1024u}, rng, q.data(), q.size());
    const size_t some = grim_fmv_quant(GrimFmvKnobs{kGrimFmvQuant, 0u, 1024u}, rng, q.data(), q.size());
    const bool valid = std::all_of(q.begin(), q.end(), [](u8 v) { return v >= 1u; });
    check(none == 0 && some > 0 && valid, "fmv_quant_targeted_and_valid");
  }
  // Movie audio: rate 0 (or no audio target) leaves a sector alone; full rate damages
  // every sector without ever reading or writing past it.
  {
    std::vector<s16> sector(2016u * 2u + 2u);
    for (size_t i = 0; i < sector.size(); ++i) sector[i] = static_cast<s16>(i * 37u);
    const std::vector<s16> orig = sector;
    GrimRng rng{3};
    GrimFmvAudioMemory mem;
    const bool quiet =
        !grim_fmv_audio(GrimFmvKnobs{kGrimFmvAll, 0u, 1024u}, rng, sector.data(), 2016u, mem) &&
        !grim_fmv_audio(GrimFmvKnobs{kGrimFmvCoeffs, 1000u, 1024u}, rng, sector.data(), 2016u, mem) &&
        sector == orig;
    u32 hits = 0, changed = 0;
    bool bounded = true;
    for (u32 strength : {1u, 300u, 1024u}) {
      for (int i = 0; i < 60; ++i) {
        std::vector<s16> s = orig;
        hits += grim_fmv_audio(GrimFmvKnobs{kGrimFmvAudio, 1000u, strength}, rng, s.data(), 2016u, mem) ? 1u : 0u;
        changed += s != orig ? 1u : 0u;
        bounded = bounded && s[2016u * 2u] == orig[2016u * 2u] && s[2016u * 2u + 1u] == orig[2016u * 2u + 1u];
      }
    }
    check(quiet && hits == 180u && changed > 150u && bounded, "fmv_audio_hits_sectors_in_bounds",
          std::to_string(hits) + " hits, " + std::to_string(changed) + " changed");
  }
  // The gene: parses, round-trips, and its runtime hits macroblocks under its trigger.
  {
    GrimGene g = make_gene(GrimGeneType::MdecFmv, {{"rate", 1000}, {"strength", 1024}});
    GrimGenome genome;
    genome.genes.push_back(g);
    GrimGenome back;
    std::string err;
    const bool round = grim_genome_parse(grim_genome_serialize(genome), back, err) &&
                       grim_genome_serialize(back) == grim_genome_serialize(genome);
    check(round, "fmv_gene_roundtrip", err);

    auto rt = make_rt({g});
    u32 changed = 0;
    for (int mb = 0; mb < 100; ++mb) {
      rt->fmv_begin_macroblock();
      for (u32 i = 0; i < 32; ++i) {
        changed += rt->fmv_coefficient(static_cast<u16>(0x0400u + i)) != 0x0400u + i ? 1u : 0u;
      }
    }
    check(rt->has_fmv() && changed > 0 && rt->hits()[0] == 100u, "fmv_gene_hits_macroblocks",
          std::to_string(changed));

    GrimGene off = g;
    off.trigger.kind = GrimTriggerKind::Window;
    off.trigger.start_frame = 100;
    off.trigger.end_frame = 200;
    auto idle = make_rt({off}, 10);
    idle->fmv_begin_macroblock();
    check(idle->fmv_coefficient(0x0401u) == 0x0401u && idle->hits()[0] == 0u,
          "fmv_gene_respects_trigger");

    GrimRandomParams rp;
    rp.spu = rp.gpu = false;
    rp.fmv = true;
    rp.min_genes = rp.max_genes = 3;
    const GrimGenome made = grim_random_genome(42, rp);
    bool all_fmv = made.genes.size() == 3;
    for (const GrimGene &mg : made.genes) all_fmv = all_fmv && mg.type == GrimGeneType::MdecFmv;
    check(all_fmv, "fmv_random_genes");
  }
}

// ---- end to end (stock BIOS) ---------------------------------------------------------

GrimEvalConfig eval_cfg(const std::string &bios, u32 frames) {
  GrimEvalConfig c;
  c.bios_path = bios;
  c.frames = frames;
  c.stop_on_death = false;
  c.watchdog_seconds = 0.0;
  return c;
}

void test_end_to_end(const std::string &bios, u32 frames) {
  const GrimEvalResult base = run_grim_eval(eval_cfg(bios, frames));
  check(base.end_reason == "frames" && base.frames.size() == frames, "e2e_baseline_runs");

  const auto compare = [&](const GrimEvalResult &r, bool &fb_differs, bool &audio_differs) {
    fb_differs = audio_differs = false;
    for (size_t i = 0; i < std::min(r.frames.size(), base.frames.size()); ++i) {
      fb_differs = fb_differs || r.frames[i].fb_hash != base.frames[i].fb_hash;
      audio_differs = audio_differs || r.frames[i].audio_hash != base.frames[i].audio_hash;
    }
  };

  {
    GrimEvalConfig c = eval_cfg(bios, frames);
    c.use_genome = true;
    GrimGene g = make_gene(GrimGeneType::SpuPitch, {{"mode", 0}, {"mul_q8", 512}});
    c.genome.genes.push_back(g);
    const GrimEvalResult r = run_grim_eval(c);
    bool fb_diff, audio_diff;
    compare(r, fb_diff, audio_diff);
    check(!fb_diff && audio_diff && !r.gene_hits.empty() && r.gene_hits[0] > 0,
          "e2e_pitch_gene_changes_audio_not_fb",
          "fb_differs=" + std::to_string(fb_diff) + " audio_differs=" + std::to_string(audio_diff) +
              " hits=" + std::to_string(r.gene_hits.empty() ? 0 : r.gene_hits[0]));
    check(r.genome_hash == grim_genome_hash(c.genome) && r.genome_hash != 0, "e2e_genome_hash_reported");
  }
  {
    GrimEvalConfig c = eval_cfg(bios, frames);
    c.use_genome = true;
    c.genome.genes.push_back(make_gene(GrimGeneType::GpuColor, {{"mode", 1}}));
    const GrimEvalResult r = run_grim_eval(c);
    bool fb_diff, audio_diff;
    compare(r, fb_diff, audio_diff);
    check(fb_diff && !audio_diff && !r.gene_hits.empty() && r.gene_hits[0] > 0,
          "e2e_gp0_color_gene_changes_fb_not_audio",
          "fb_differs=" + std::to_string(fb_diff) + " audio_differs=" + std::to_string(audio_diff) +
              " hits=" + std::to_string(r.gene_hits.empty() ? 0 : r.gene_hits[0]));
  }
  // The same genome twice gives identical telemetry (in-process determinism).
  {
    GrimEvalConfig c = eval_cfg(bios, std::min(frames, 400u));
    c.use_genome = true;
    c.genome = all_types_genome();
    Bios stock;
    if (stock.load(bios)) {
      c.genome.bios_hash = stock.image_hash();
    }
    const GrimEvalResult a = run_grim_eval(c);
    const GrimEvalResult b = run_grim_eval(c);
    check(a.run_hash == b.run_hash && a.gene_hits == b.gene_hits, "e2e_same_genome_same_run_hash", hex(a.run_hash));
  }
}

// A genome with several SPU and GP0 genes, rot and intermittent triggers.
GrimGenome mixed_genome() {
  GrimGenome g;
  const auto add = [&](GrimGene gene, GrimTrigger t, u64 seed) {
    gene.trigger = t;
    gene.seed = seed;
    g.genes.push_back(gene);
  };
  add(make_gene(GrimGeneType::SpuPitch, {{"mode", 1}, {"offset", 300}}), rot(50, 300), 11);
  add(make_gene(GrimGeneType::SpuVolume, {{"mode", 1}}), pattern(30, 10), 12);
  add(make_gene(GrimGeneType::SpuKeyOn, {{"mode", 2}, {"prob", 700}, {"delay", 300}}), chance(800), 13);
  add(make_gene(GrimGeneType::SpuReverb, {{"cfg", 500}, {"eon", 1}, {"spucnt", 1}}), rot(0, 400), 14);
  add(make_gene(GrimGeneType::SpuNoise, {{"piggyback", 1}}), chance(300), 15);
  add(make_gene(GrimGeneType::GpuVertex, {{"mode", 1}, {"amp", 6}}), rot(0, 400), 16);
  add(make_gene(GrimGeneType::GpuColor, {{"mode", 0}, {"perm", 6}}), pattern(40, 20), 17);
  add(make_gene(GrimGeneType::GpuFlags, {{"semi", 3}, {"raw", 3}, {"prob", 300}}), window(100, 300), 18);
  add(make_gene(GrimGeneType::GpuState, {{"amp", 3}, {"drift", 20}}), rot(100, 350), 19);
  return g;
}

// The same genome under the interpreter and the recompiler: every guest-visible
// per-frame value the runner records must match (frame hash, audio hash, cycles,
// GPU command counts, key-ons, active voices).
void test_recompiler_parity(const std::string &bios) {
  GrimEvalConfig c = eval_cfg(bios, 400);
  c.use_genome = true;
  c.genome = mixed_genome();
  c.native_cpu = true;
  const bool saved_override = g_cpu_execution_mode_cli_override;
  const CpuExecutionMode saved_value = g_cpu_execution_mode_cli_value;
  g_cpu_execution_mode_cli_override = true;
  g_cpu_execution_mode_cli_value = CpuExecutionMode::Interpreter;
  const GrimEvalResult interp = run_grim_eval(c);
  g_cpu_execution_mode_cli_value = CpuExecutionMode::Recompiler;
  const GrimEvalResult rec = run_grim_eval(c);
  g_cpu_execution_mode_cli_override = saved_override;
  g_cpu_execution_mode_cli_value = saved_value;

  std::string diff;
  for (size_t i = 0; i < std::min(interp.frames.size(), rec.frames.size()) && diff.empty(); ++i) {
    const GrimFrameTelemetry &a = interp.frames[i], &b = rec.frames[i];
    if (a.fb_hash != b.fb_hash) diff = "fb";
    else if (a.audio_hash != b.audio_hash) diff = "audio_hash";
    else if (a.cycles != b.cycles) diff = "cycles";
    else if (a.gp0_polygon != b.gp0_polygon || a.gp0_line != b.gp0_line ||
             a.gp0_rect != b.gp0_rect || a.gp0_fill != b.gp0_fill) diff = "gp0";
    else if (a.spu_key_on != b.spu_key_on) diff = "key_on";
    else if (a.spu_voice_mask != b.spu_voice_mask) diff = "voices";
    if (!diff.empty()) {
      diff += " at frame " + std::to_string(i);
    }
  }
  u64 hits = 0;
  for (const u64 h : rec.gene_hits) {
    hits += h;
  }
  check(diff.empty() && interp.frames.size() == rec.frames.size() && interp.gene_hits == rec.gene_hits &&
            hits > 0,
        "e2e_genes_match_between_interpreter_and_recompiler",
        diff + " frames=" + std::to_string(rec.frames.size()) + " hits=" + std::to_string(hits));
}

void test_determinism(const std::string &bios, const std::string &self_exe) {
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() /
      ("vibestation_grim_gene_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(dir);
  const std::filesystem::path file = dir / "mixed.json";
  {
    std::ofstream out(file, std::ios::binary);
    out << grim_genome_serialize(mixed_genome());
  }
  GrimEvalConfig c = eval_cfg(bios, 400);
  c.use_genome = true;
  c.genome = mixed_genome();
  const GrimEvalResult r = run_grim_eval(c);
  u64 total = 0;
  std::string hits;
  for (const u64 h : r.gene_hits) {
    total += h;
    hits += std::to_string(h) + ",";
  }
  check(total > 0, "determinism_genome_is_active", "hits=" + hits);
  std::printf("GRIM_GENE_TEST INFO determinism_genome_hash=%s run_hash=%s\n",
              hex(r.genome_hash).c_str(), hex(r.run_hash).c_str());
  const int rc = run_grim_determinism_test({bios, "400", "2", "2", "--genome", file.string()}, self_exe);
  check(rc == 0, "determinism_genome_threads_and_processes", "rc=" + std::to_string(rc));
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

void test_explore(const std::string &bios, const std::string &self_exe) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() /
                       ("vibestation_grim_explore_" +
                        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  {
    const int rc = run_grim_explore_cli({"1000", "2", (dir / "ok").string(), "150", "--bios", bios,
                                         "--timeout", "180"},
                                        self_exe);
    const std::string table = read_file(dir / "ok" / "summary.tsv");
    const size_t lines = static_cast<size_t>(std::count(table.begin(), table.end(), '\n'));
    bool survivor_files = true, any_survivor = false;
    for (u64 seed = 1000; seed < 1002; ++seed) {
      const fs::path s = dir / "ok" / "survivors" / ("seed_" + std::to_string(seed));
      if (fs::exists(s)) {
        any_survivor = true;
        const std::string wav = read_file(s / "audio.wav");
        bool bmp = false;
        for (const auto &e : fs::directory_iterator(s / "frames")) {
          bmp = bmp || read_file(e.path()).rfind("BM", 0) == 0;
        }
        GrimGenome g;
        std::string err;
        survivor_files = survivor_files && wav.rfind("RIFF", 0) == 0 && wav.size() > 44 && bmp &&
                         fs::exists(s / "telemetry.jsonl") && grim_genome_load((s / "genome.json").string(), g, err);
      }
    }
    check(rc == 0 && lines == 3 && any_survivor && survivor_files, "explore_small_run_completes",
          "rc=" + std::to_string(rc) + " table_lines=" + std::to_string(lines) +
              " survivors_ok=" + std::to_string(any_survivor && survivor_files));
  }
  {
    const auto t0 = std::chrono::steady_clock::now();
    const int rc = run_grim_explore_cli({"5000", "1", (dir / "hang").string(), "150", "--bios", bios,
                                         "--timeout", "3", "--child-arg", "--test-hang"},
                                        self_exe);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const std::string table = read_file(dir / "hang" / "summary.tsv");
    GrimGenome g;
    std::string err;
    const bool saved = grim_genome_load((dir / "hang" / "crashes" / "seed_5000.json").string(), g, err);
    check(rc == 0 && table.find("timeout") != std::string::npos && saved && secs < 30.0,
          "explore_kills_hanging_child", "seconds=" + std::to_string(secs) + " genome_saved=" + std::to_string(saved));
  }
  std::error_code ec;
  fs::remove_all(dir, ec);
}

} // namespace

int run_grim_gene_test(const std::vector<std::string> &raw_args, const std::string &self_exe) {
  grim_prepare_eval_process();
  std::vector<std::string> args = raw_args;
  const std::string bios = grim_take_bios_arg(args);
  const u32 frames = !args.empty() ? static_cast<u32>(std::max(1, std::atoi(args[0].c_str()))) : 900u;
  g_failures = 0;

  test_rng();
  test_genome_roundtrip();
  test_genome_strict();
  test_triggers();
  test_spu_filters();
  test_gp0_transforms();
  test_fmv();
  if (bios.empty()) {
    std::printf("GRIM_GENE_TEST SKIP bios tests (no BIOS path or VIBESTATION_BIOS)\n");
  } else {
    test_end_to_end(bios, frames);
    test_recompiler_parity(bios);
    test_determinism(bios, self_exe);
    test_explore(bios, self_exe);
  }
  std::printf("GRIM_GENE_TEST_RESULT %s failures=%d\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
  return g_failures == 0 ? 0 : 1;
}
