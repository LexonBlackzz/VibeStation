#include "platform/grim_eval_runner.h"
#include "core/grim_eval.h"
#include "core/grim_genome.h"
#include "core/grim_rom.h"
#include "core/types.h"
#include "platform/grim_process.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <thread>

namespace {

// Evaluation hooks only see the interpreter, and exception-heavy broken
// machines would otherwise flood the log at Warn level.
void prepare_eval_process() {
  g_cpu_execution_mode_cli_override = true;
  g_cpu_execution_mode_cli_value = CpuExecutionMode::Interpreter;
  if (g_log_level == LogLevel::Info) {
    g_log_level = LogLevel::Error;
  }
}

std::vector<std::string> frame_lines(const GrimEvalResult &r) {
  std::vector<std::string> lines;
  lines.reserve(r.frames.size());
  for (const GrimFrameTelemetry &f : r.frames) {
    lines.push_back(grim_frame_json(f));
  }
  return lines;
}

// Name of the JSON field that contains the first differing character.
std::string first_diff_field(const std::string &a, const std::string &b) {
  size_t pos = 0;
  while (pos < a.size() && pos < b.size() && a[pos] == b[pos]) {
    ++pos;
  }
  const size_t key_end = a.rfind("\":", pos);
  if (key_end == std::string::npos) {
    return "?";
  }
  const size_t key_start = a.rfind('"', key_end - 1);
  return a.substr(key_start + 1, key_end - key_start - 1);
}

// Prints and returns false on the first divergence from the reference.
bool compare_runs(const std::string &label, const std::vector<std::string> &ref,
                  const std::vector<std::string> &run) {
  const size_t n = std::min(ref.size(), run.size());
  for (size_t i = 0; i < n; ++i) {
    if (ref[i] != run[i]) {
      std::printf("GRIM_DETERMINISM_FAIL run=%s frame=%zu field=%s\n",
                  label.c_str(), i, first_diff_field(ref[i], run[i]).c_str());
      return false;
    }
  }
  if (ref.size() != run.size()) {
    std::printf("GRIM_DETERMINISM_FAIL run=%s frame=%zu field=frame_count "
                "(ref=%zu run=%zu)\n",
                label.c_str(), n, ref.size(), run.size());
    return false;
  }
  return true;
}

std::vector<std::string> read_frame_lines(const std::string &path) {
  std::vector<std::string> lines;
  std::ifstream in(path, std::ios::binary);
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind("{\"summary\"", 0) != 0) {
      lines.push_back(line);
    }
  }
  return lines;
}

std::string quote_arg(const std::string &s) { return "\"" + s + "\""; }

// The BIOS path is the first argument unless that argument is a number (or
// missing); then it comes from VIBESTATION_BIOS. Removes it from `args`.
std::string take_bios_arg(std::vector<std::string> &args) {
  const bool first_is_path =
      !args.empty() && args[0].find_first_not_of("0123456789") != std::string::npos;
  if (first_is_path) {
    const std::string bios = args[0];
    args.erase(args.begin());
    return bios;
  }
  const char *env = std::getenv("VIBESTATION_BIOS");
  return env != nullptr ? env : "";
}

int g_failures = 0;

void check(bool ok, const std::string &name, const std::string &detail) {
  std::printf("GRIM_TEST %s %s %s\n", ok ? "PASS" : "FAIL", name.c_str(),
              detail.c_str());
  std::fflush(stdout);
  if (!ok) {
    ++g_failures;
  }
}

void expect_liveness(const std::string &name, const GrimLiveness &got,
                     const std::string &reason, s64 death_frame = -2,
                     int silent = -1) {
  const bool ok = got.reason == reason &&
                  (death_frame == -2 || got.death_frame == death_frame) &&
                  (silent < 0 || got.silent == (silent != 0));
  check(ok, name,
        "reason=" + got.reason + " death_frame=" +
            std::to_string(got.death_frame) +
            " silent=" + (got.silent ? "1" : "0") + " (want " + reason +
            (death_frame == -2 ? "" : " at " + std::to_string(death_frame)) +
            (silent < 0 ? "" : silent ? " silent" : " not silent") + ")");
}

std::vector<GrimFrameTelemetry>
synth(u32 count, const std::function<void(u32, GrimFrameTelemetry &)> &fill) {
  std::vector<GrimFrameTelemetry> frames(count);
  for (u32 i = 0; i < count; ++i) {
    frames[i].frame = i;
    frames[i].fb_hash = 0x1234u;
    fill(i, frames[i]);
  }
  return frames;
}

void moving_audio(u32 i, GrimFrameTelemetry &f) {
  f.audio_rms = 1000.0 + (i % 7u) * 150.0;
  f.audio_zcr = 0.05 + (i % 3u) * 0.01;
}

void run_synthetic_gate_tests() {
  const GrimLivenessConfig cfg;
  expect_liveness("gate_alive_animated",
                  grim_evaluate_liveness(synth(400, [](u32 i, GrimFrameTelemetry &f) {
                    f.fb_hash = i;
                    moving_audio(i, f);
                  }), cfg),
                  "alive");
  // VSync-style polling loop: no new code, but the IRQ handler keeps drawing.
  expect_liveness("gate_polling_loop_is_alive",
                  grim_evaluate_liveness(synth(1200, [](u32 i, GrimFrameTelemetry &f) {
                    f.fb_hash = i / 4u;
                    f.gp0_polygon = 12;
                    f.dma_words[2] = 300;
                    moving_audio(i, f);
                  }), cfg),
                  "alive");
  expect_liveness("gate_coverage_stall",
                  grim_evaluate_liveness(synth(400, [](u32, GrimFrameTelemetry &) {}), cfg),
                  "coverage_stall", cfg.coverage_stall_frames - 1);
  // Runaway CPU: keeps executing new "code", nothing is drawn or heard.
  expect_liveness("gate_frozen_frame",
                  grim_evaluate_liveness(synth(1000, [](u32, GrimFrameTelemetry &f) {
                    f.new_pcs = 40;
                  }), cfg),
                  "frozen_frame", cfg.frozen_frame_frames - 1);
  // Idle Shell: after a noisy intro it redraws one static picture forever.
  expect_liveness("gate_idle_redraw_is_alive",
                  grim_evaluate_liveness(synth(1800, [](u32 i, GrimFrameTelemetry &f) {
                    if (i < 100) {
                      f.fb_hash = i;
                      moving_audio(i, f);
                    }
                    f.gp0_polygon = 35;
                    f.dma_words[2] = 2271;
                  }), cfg),
                  "alive");
  expect_liveness("gate_exception_loop",
                  grim_evaluate_liveness(synth(400, [](u32, GrimFrameTelemetry &f) {
                    f.exceptions[8] = 1000;
                    f.exception_repeats = 999;
                  }), cfg),
                  "exception_loop", cfg.exception_loop_frames - 1);
  expect_liveness("gate_varied_exceptions_not_loop",
                  grim_evaluate_liveness(synth(400, [](u32 i, GrimFrameTelemetry &f) {
                    f.exceptions[8] = 1000;
                    f.exception_repeats = 10;
                    f.fb_hash = i;
                    moving_audio(i, f);
                  }), cfg),
                  "alive");
  // Silent machines that never show an image: the GPU keeps working (so no
  // other gate trips) but nothing lit ever reaches the display.
  expect_liveness("gate_dead_audio_silence",
                  grim_evaluate_liveness(synth(400, [](u32, GrimFrameTelemetry &f) {
                    f.gp0_polygon = 12;
                  }), cfg),
                  "dead_audio", -2, 1);
  expect_liveness("gate_dead_audio_dc",
                  grim_evaluate_liveness(synth(400, [](u32 i, GrimFrameTelemetry &f) {
                    f.gp0_polygon = 12;
                    f.audio_rms = 5000.0 + (i % 7u) * 300.0;
                    f.audio_zcr = 0.0;
                  }), cfg),
                  "dead_audio", -2, 1);
  expect_liveness("gate_dead_audio_fixed_tone",
                  grim_evaluate_liveness(synth(400, [](u32 i, GrimFrameTelemetry &f) {
                    f.gp0_polygon = 12;
                    f.audio_rms = 5000.0 + (i % 2u); // within the change ratio
                    f.audio_zcr = 0.02;
                  }), cfg),
                  "dead_audio", -2, 1);
  // A silent machine that draws an image is alive, just flagged silent.
  expect_liveness("gate_silent_but_drawing_is_alive",
                  grim_evaluate_liveness(synth(400, [](u32 i, GrimFrameTelemetry &f) {
                    f.fb_hash = i;
                    f.display_enabled = true;
                    f.fb_lit = 5000;
                  }), cfg),
                  "alive", -2, 1);
}

// Frames to run the stock BIOS for: long enough to pass the intro and sit in
// the Shell for longer than the coverage-stall window.
constexpr u32 kDefaultBiosTestFrames = 1500;

void run_bios_tests(const std::string &bios_path, u32 frames) {
  GrimEvalConfig base;
  base.bios_path = bios_path;
  base.frames = frames;

  const GrimEvalResult stock = run_grim_eval(base);
  check(stock.end_reason == "frames" && stock.liveness.alive, "bios_stock_alive",
        "end=" + stock.end_reason + " reason=" + stock.liveness.reason +
            " death_frame=" + std::to_string(stock.liveness.death_frame) +
            " frames=" + std::to_string(stock.frames.size()));
  std::printf("GRIM_TEST INFO bios_stock_speed emulated_s=%.2f wall_s=%.2f "
              "speed=%.2fx coverage=%u\n",
              stock.emulated_seconds, stock.wall_seconds, stock.speed_factor,
              stock.coverage);

  // Branch-to-self as the very first instruction.
  GrimEvalConfig spin = base;
  spin.bios_patches = {{0x0u, 0x1000FFFFu}, {0x4u, 0x00000000u}};
  expect_liveness("bios_infinite_loop", run_grim_eval(spin).liveness,
                  "coverage_stall");

  // SYSCALL at the reset vector; the BEV handler returns straight to it.
  GrimEvalConfig trap = base;
  trap.bios_patches = {{0x000u, 0x0000000Cu},  // syscall
                       {0x004u, 0x00000000u},
                       {0x180u, 0x401A7000u},  // mfc0 k0, EPC
                       {0x184u, 0x00000000u},
                       {0x188u, 0x03400008u},  // jr k0
                       {0x18Cu, 0x42000010u}}; // rfe
  expect_liveness("bios_exception_loop", run_grim_eval(trap).liveness,
                  "exception_loop");

  GrimEvalConfig muted = base;
  muted.mute_audio = true;
  const GrimEvalResult silent = run_grim_eval(muted);
  expect_liveness("bios_muted_audio_still_alive", silent.liveness, "alive", -2, 1);
  expect_liveness("bios_stock_not_silent", stock.liveness, "alive", -2, 0);
}

} // namespace

int run_grim_eval_cli(const std::vector<std::string> &raw_args) {
  prepare_eval_process();
  std::vector<std::string> args = raw_args;
  const std::string bios = take_bios_arg(args);
  if (bios.empty() || args.size() < 2) {
    std::fprintf(stderr, "usage: --grim-eval [bios] <frames> <out.jsonl> "
                         "[--scenario nodisc] [--max-cycles N] "
                         "[--max-instructions N] [--watchdog-seconds S] "
                         "[--no-stop-on-death] [--genome file.json] "
                         "[--dump-wav out.wav] [--dump-frames dir every_n]\n");
    return 1;
  }
  GrimEvalConfig cfg;
  cfg.bios_path = bios;
  cfg.frames = static_cast<u32>(std::max(1, std::atoi(args[0].c_str())));
  const std::string out_path = args[1];
  for (size_t i = 2; i < args.size(); ++i) {
    const std::string &a = args[i];
    const bool has_value = i + 1 < args.size();
    if (a == "--scenario" && has_value) {
      if (args[++i] != "nodisc") {
        std::fprintf(stderr, "GRIM_EVAL_ERROR unsupported scenario %s\n",
                     args[i].c_str());
        return 1;
      }
    } else if (a == "--max-cycles" && has_value) {
      cfg.max_cycles = std::strtoull(args[++i].c_str(), nullptr, 10);
    } else if (a == "--max-instructions" && has_value) {
      cfg.max_instructions = std::strtoull(args[++i].c_str(), nullptr, 10);
    } else if (a == "--watchdog-seconds" && has_value) {
      cfg.watchdog_seconds = std::atof(args[++i].c_str());
    } else if (a == "--no-stop-on-death") {
      cfg.stop_on_death = false;
    } else if (a == "--genome" && has_value) {
      std::string err;
      if (!grim_genome_load(args[++i], cfg.genome, err)) {
        std::printf("GRIM_EVAL_RESULT status=error reason=bad_genome detail=%s\n", err.c_str());
        return 1;
      }
      cfg.use_genome = true;
    } else if (a == "--dump-wav" && has_value) {
      cfg.dump_wav_path = args[++i];
    } else if (a == "--dump-frames" && i + 2 < args.size()) {
      cfg.dump_frames_dir = args[++i];
      cfg.dump_frames_every = static_cast<u32>(std::max(1, std::atoi(args[++i].c_str())));
    } else if (a == "--test-hang") {
      cfg.test_hang = true; // test only: see GrimEvalConfig::test_hang
    } else {
      std::fprintf(stderr, "GRIM_EVAL_ERROR unknown option %s\n", a.c_str());
      return 1;
    }
  }

  const GrimEvalResult r = run_grim_eval(cfg);
  if (r.end_reason == "rom_gene_mismatch") {
    std::printf("GRIM_EVAL_RESULT status=error reason=rom_gene_mismatch detail=%s\n",
                r.error_detail.c_str());
    return 1;
  }
  if (r.end_reason == "bios_load_failed" ||
      r.end_reason == "unsupported_cpu_mode") {
    std::printf("GRIM_EVAL_RESULT status=error reason=%s\n", r.end_reason.c_str());
    return 1;
  }
  if (!grim_write_telemetry_jsonl(r, out_path)) {
    std::printf("GRIM_EVAL_RESULT status=error reason=write_failed path=%s\n",
                out_path.c_str());
    return 1;
  }
  std::printf("GRIM_EVAL_RESULT verdict=%s reason=%s death_frame=%lld silent=%d "
              "end=%s frames=%zu emulated_s=%.2f wall_s=%.2f speed=%.2fx "
              "run_hash=0x%016llX genome=0x%016llX out=%s\n",
              r.liveness.alive ? "alive" : "dead", r.liveness.reason.c_str(),
              static_cast<long long>(r.liveness.death_frame),
              r.liveness.silent ? 1 : 0,
              r.end_reason.c_str(), r.frames.size(), r.emulated_seconds,
              r.wall_seconds, r.speed_factor,
              static_cast<unsigned long long>(r.run_hash),
              static_cast<unsigned long long>(r.genome_hash), out_path.c_str());
  return r.liveness.alive ? 0 : 2;
}

int run_grim_determinism_test(const std::vector<std::string> &raw_args,
                              const std::string &self_exe) {
  prepare_eval_process();
  std::vector<std::string> args = raw_args;
  GrimEvalConfig cfg;
  std::string genome_path;
  for (size_t i = 0; i + 1 < args.size();) {
    if (args[i] == "--genome") {
      genome_path = args[i + 1];
      args.erase(args.begin() + static_cast<std::ptrdiff_t>(i),
                 args.begin() + static_cast<std::ptrdiff_t>(i) + 2);
    } else {
      ++i;
    }
  }
  cfg.bios_path = take_bios_arg(args);
  if (cfg.bios_path.empty()) {
    std::fprintf(stderr, "usage: --grim-determinism-test [bios] [frames=600] "
                         "[threads=2] [processes=2] [--genome file.json]\n");
    return 1;
  }
  if (!genome_path.empty()) {
    std::string err;
    if (!grim_genome_load(genome_path, cfg.genome, err)) {
      std::printf("GRIM_DETERMINISM_FAIL reason=bad_genome detail=%s\n", err.c_str());
      return 1;
    }
    cfg.use_genome = true;
  }
  cfg.frames = args.size() > 0
                   ? static_cast<u32>(std::max(1, std::atoi(args[0].c_str())))
                   : 600u;
  const int threads = args.size() > 1 ? std::max(0, std::atoi(args[1].c_str())) : 2;
  const int processes =
      args.size() > 2 ? std::max(0, std::atoi(args[2].c_str())) : 2;
  cfg.stop_on_death = false;
  cfg.watchdog_seconds = 0.0;

  // Two back-to-back runs in this thread catch state leaking between runs;
  // the parallel ones catch shared state and host-timing dependence.
  const GrimEvalResult ref_result = run_grim_eval(cfg);
  if (ref_result.frames.empty()) {
    std::printf("GRIM_DETERMINISM_FAIL reason=%s\n", ref_result.end_reason.c_str());
    return 1;
  }
  const std::vector<std::string> ref = frame_lines(ref_result);
  std::vector<std::pair<std::string, std::vector<std::string>>> runs;
  runs.emplace_back("sequential#1", frame_lines(run_grim_eval(cfg)));

  const std::filesystem::path tmp =
      std::filesystem::temp_directory_path() /
      ("vibestation_grim_det_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(tmp);

  std::vector<std::vector<std::string>> thread_lines(threads);
  std::vector<std::string> process_out(processes);
  std::vector<std::thread> workers;
  for (int t = 0; t < threads; ++t) {
    workers.emplace_back([&, t] { thread_lines[t] = frame_lines(run_grim_eval(cfg)); });
  }
  for (int p = 0; p < processes; ++p) {
    process_out[p] = (tmp / ("process_" + std::to_string(p) + ".jsonl")).string();
    std::string cmd = quote_arg(self_exe) + " --grim-eval " + quote_arg(cfg.bios_path) +
                      " " + std::to_string(cfg.frames) + " " +
                      quote_arg(process_out[p]) + " --no-stop-on-death";
    if (!genome_path.empty()) {
      cmd += " --genome " + quote_arg(genome_path);
    }
#ifdef _WIN32
    cmd = "\"" + cmd + " > NUL 2>&1\""; // cmd.exe strips the outer quotes
#else
    cmd += " > /dev/null 2>&1";
#endif
    workers.emplace_back([cmd] { std::system(cmd.c_str()); });
  }
  for (std::thread &w : workers) {
    w.join();
  }
  for (int t = 0; t < threads; ++t) {
    runs.emplace_back("thread#" + std::to_string(t), std::move(thread_lines[t]));
  }
  for (int p = 0; p < processes; ++p) {
    runs.emplace_back("process#" + std::to_string(p), read_frame_lines(process_out[p]));
  }
  std::error_code ec;
  std::filesystem::remove_all(tmp, ec);

  bool ok = true;
  for (const auto &[label, lines] : runs) {
    ok = compare_runs(label, ref, lines) && ok;
  }
  if (!ok) {
    return 1;
  }
  std::printf("GRIM_DETERMINISM_PASS runs=%zu frames=%zu run_hash=0x%016llX "
              "verdict=%s reason=%s\n",
              runs.size() + 1u, ref.size(),
              static_cast<unsigned long long>(ref_result.run_hash),
              ref_result.liveness.alive ? "alive" : "dead",
              ref_result.liveness.reason.c_str());
  return 0;
}

int run_grim_self_test(const std::vector<std::string> &raw_args) {
  prepare_eval_process();
  std::vector<std::string> args = raw_args;
  const std::string bios = take_bios_arg(args);
  g_failures = 0;
  run_synthetic_gate_tests();
  {
    GrimFrameTelemetry a;
    GrimFrameTelemetry b = a;
    b.cpu_hash = 1;
    const std::string field = first_diff_field(grim_frame_json(a), grim_frame_json(b));
    check(field == "cpu_hash", "determinism_reports_field", "field=" + field);
  }
  if (bios.empty()) {
    std::printf("GRIM_TEST SKIP bios_tests (no BIOS path or VIBESTATION_BIOS)\n");
  } else {
    const u32 frames =
        !args.empty() ? static_cast<u32>(std::max(1, std::atoi(args[0].c_str())))
                      : kDefaultBiosTestFrames;
    run_bios_tests(bios, frames);
  }
  std::printf("GRIM_SELF_TEST %s failures=%d\n", g_failures == 0 ? "PASS" : "FAIL",
              g_failures);
  return g_failures == 0 ? 0 : 1;
}

void grim_prepare_eval_process() { prepare_eval_process(); }

std::string grim_take_bios_arg(std::vector<std::string> &args) {
  return take_bios_arg(args);
}

namespace {

// Phase 3: which gene families random genomes draw from, and the ROM options.
struct RomMixOptions {
  std::string mix = "interface"; // interface | rom | both
  std::string map_path, bios;
  u32 patches_max = 6, genes_max = 3, early_ms = 200, curve = 2;
  bool call_swap = false;
  GrimRomContext ctx;
  bool wants_rom() const { return mix == "rom" || mix == "both"; }
};

// Consumes a ROM-related option at args[i]; returns true when it was one.
bool take_rom_option(const std::vector<std::string> &args, size_t &i, RomMixOptions &o) {
  const std::string &a = args[i];
  const bool has_value = i + 1 < args.size();
  auto num = [&](u32 lo, u32 hi) {
    return static_cast<u32>(std::min<long>(hi, std::max<long>(lo, std::atol(args[++i].c_str()))));
  };
  if (a == "--map" && has_value) {
    o.map_path = args[++i];
  } else if (a == "--mix" && has_value) {
    o.mix = args[++i];
  } else if (a == "--rom-patches" && has_value) {
    o.patches_max = num(1, 4096);
  } else if (a == "--rom-genes" && has_value) {
    o.genes_max = num(1, 64);
  } else if (a == "--early-ms" && has_value) {
    o.early_ms = num(0, 60000);
  } else if (a == "--curve" && has_value) {
    o.curve = num(0, 3);
  } else if (a == "--call-swap") {
    o.call_swap = true;
  } else {
    return false;
  }
  return true;
}

// Loads the map + BIOS when ROM genes are wanted. Prints and returns false on error.
bool finish_rom_options(RomMixOptions &o, const std::string &bios_from_env) {
  if (o.mix != "interface" && o.mix != "rom" && o.mix != "both") {
    std::fprintf(stderr, "--mix must be interface, rom or both\n");
    return false;
  }
  if (!o.wants_rom()) {
    return true;
  }
  if (o.bios.empty()) {
    o.bios = bios_from_env;
  }
  std::string err;
  if (o.map_path.empty() || o.bios.empty() || !o.ctx.load(o.bios, o.map_path, err)) {
    std::fprintf(stderr, "ROM genes need --map <file> and a BIOS (--bios or VIBESTATION_BIOS)%s%s\n",
                 err.empty() ? "" : ": ", err.c_str());
    return false;
  }
  return true;
}

void apply_rom_options(GrimRandomParams &p, const RomMixOptions &o) {
  if (o.mix == "rom") {
    p.spu = p.gpu = false;
  }
  if (o.wants_rom()) {
    p.rom = &o.ctx;
    p.rom_genes_min = 1;
    p.rom_genes_max = o.genes_max;
    p.rom_patches_max = o.patches_max;
    p.rom_early_ms = o.early_ms;
    p.rom_curve = o.curve;
    p.rom_call_swap = o.call_swap;
  }
}

} // namespace

int run_grim_random_genome_cli(const std::vector<std::string> &raw_args) {
  RomMixOptions rom;
  std::vector<std::string> args;
  for (size_t i = 0; i < raw_args.size(); ++i) {
    if (raw_args[i] == "--bios" && i + 1 < raw_args.size()) {
      rom.bios = raw_args[++i];
    } else if (!take_rom_option(raw_args, i, rom)) {
      args.push_back(raw_args[i]);
    }
  }
  if (args.size() < 2) {
    std::fprintf(stderr,
                 "usage: --grim-random-genome <seed> <out.json> [gene_count] [--mix "
                 "interface|rom|both --map file --bios path] [--rom-patches N] [--rom-genes N] "
                 "[--early-ms N] [--curve 0-3] [--call-swap]\n");
    return 1;
  }
  const char *env = std::getenv("VIBESTATION_BIOS");
  if (!finish_rom_options(rom, env != nullptr ? env : "")) {
    return 1;
  }
  GrimRandomParams params;
  apply_rom_options(params, rom);
  if (args.size() > 2) {
    params.min_genes = params.max_genes =
        static_cast<u32>(std::max(1, std::atoi(args[2].c_str())));
  }
  const u64 seed = std::strtoull(args[0].c_str(), nullptr, 10);
  const GrimGenome g = grim_random_genome(seed, params);
  std::ofstream out(args[1], std::ios::binary | std::ios::trunc);
  out << grim_genome_serialize(g);
  if (!out) {
    std::fprintf(stderr, "cannot write %s\n", args[1].c_str());
    return 1;
  }
  std::printf("GRIM_GENOME seed=%llu genes=%zu hash=0x%016llX out=%s\n",
              static_cast<unsigned long long>(seed), g.genes.size(),
              static_cast<unsigned long long>(grim_genome_hash(g)), args[1].c_str());
  return 0;
}

namespace {

// Value of `key=` in a "GRIM_EVAL_RESULT key=value ..." line, or "".
std::string result_field(const std::string &line, const std::string &key) {
  const std::string needle = " " + key + "=";
  const size_t pos = line.find(needle);
  if (pos == std::string::npos) {
    return "";
  }
  const size_t start = pos + needle.size();
  return line.substr(start, line.find(' ', start) - start);
}

std::string find_result_line(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  std::string line, found;
  while (std::getline(in, line)) {
    if (line.rfind("GRIM_EVAL_RESULT", 0) == 0) {
      found = line;
    }
  }
  return found;
}

void write_text(const std::filesystem::path &path, const std::string &text) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << text;
}

} // namespace

namespace {

// ---- Phase 3: does late code survive mutation? -------------------------------------------

// Quintile edges (in cycles) of first-execution time over all executed code words.
std::array<u64, 4> exec_time_edges(const GrimRomContext &ctx) {
  std::vector<u64> t;
  for (const GrimMapWord &w : ctx.map.words) {
    if (w.cls == GrimWordClass::Code) {
      t.push_back(w.first_exec);
    }
  }
  std::sort(t.begin(), t.end());
  std::array<u64, 4> edges{};
  for (size_t i = 0; i < 4 && !t.empty(); ++i) {
    edges[i] = t[t.size() * (i + 1) / 5];
  }
  return edges;
}

int time_bucket(u64 cycle, const std::array<u64, 4> &edges) {
  int b = 0;
  while (b < 4 && cycle >= edges[static_cast<size_t>(b)]) {
    ++b;
  }
  return b;
}

// The earliest-executed patch decides the bucket (it is the first to be hit); the
// mutation kind is the kind of the gene that holds it.
void rom_attribution(const GrimGenome &g, const GrimRomContext &ctx,
                     const std::array<u64, 4> &edges, int &bucket, int &kind) {
  u64 best = kGrimNever;
  for (const GrimGene &gene : g.genes) {
    for (const GrimRomPatch &p : gene.patches) {
      const u64 t = p.offset / 4u < ctx.map.words.size() ? ctx.map.words[p.offset / 4u].first_exec
                                                          : kGrimNever;
      if (t < best) {
        best = t;
        kind = gene.params[0];
      }
    }
  }
  if (best != kGrimNever) {
    bucket = time_bucket(best, edges);
  }
}

struct SurvivalRow {
  int bucket, kind;
  std::string outcome; // alive, a death reason, timeout, crash or error
};

void print_survival(const std::vector<SurvivalRow> &rows, const GrimRomContext &ctx,
                    const std::array<u64, 4> &edges, const std::filesystem::path &csv_path) {
  static const char *const kOutcomes[] = {"coverage_stall", "exception_loop", "frozen_frame",
                                          "dead_audio",     "timeout",        "crash", "error"};
  struct Tally {
    u32 n = 0, alive = 0;
    std::array<u32, 7> dead{};
  };
  auto add = [&](Tally &t, const std::string &outcome) {
    ++t.n;
    if (outcome == "alive") {
      ++t.alive;
      return;
    }
    for (size_t i = 0; i < 7; ++i) {
      if (outcome == kOutcomes[i]) {
        ++t.dead[i];
        return;
      }
    }
    ++t.dead[6]; // unknown reasons count as error
  };
  std::array<Tally, 5> by_bucket;
  std::array<Tally, static_cast<size_t>(GrimRomMut::Count)> by_kind;
  for (const SurvivalRow &r : rows) {
    if (r.bucket >= 0) {
      add(by_bucket[static_cast<size_t>(r.bucket)], r.outcome);
    }
    if (r.kind >= 0 && r.kind < static_cast<int>(by_kind.size())) {
      add(by_kind[static_cast<size_t>(r.kind)], r.outcome);
    }
  }
  auto range_text = [&](size_t b) {
    char t[64];
    const double lo = b == 0 ? 0.0 : grim_cycles_to_ms(edges[b - 1]);
    if (b == 4) {
      std::snprintf(t, sizeof(t), ">=%.0fms", lo);
    } else {
      std::snprintf(t, sizeof(t), "%.0f-%.0fms", lo, grim_cycles_to_ms(edges[b]));
    }
    return std::string(t);
  };
  (void)ctx;
  std::string csv = "dimension,bucket,label,n,alive,survival_rate,coverage_stall,exception_loop,"
                    "frozen_frame,dead_audio,timeout,crash,error\n";
  auto emit = [&](const char *dim, size_t idx, const std::string &label, const Tally &t) {
    char line[256];
    std::snprintf(line, sizeof(line), "%s,%zu,%s,%u,%u,%.3f", dim, idx, label.c_str(), t.n, t.alive,
                  t.n ? static_cast<double>(t.alive) / t.n : 0.0);
    csv += line;
    for (u32 d : t.dead) {
      csv += "," + std::to_string(d);
    }
    csv += "\n";
    std::printf("  %-14s %-14s n=%-4u alive=%-4u survival=%5.1f%%  stall=%u exc_loop=%u frozen=%u "
                "audio=%u timeout=%u crash=%u error=%u\n",
                dim, label.c_str(), t.n, t.alive, t.n ? 100.0 * t.alive / t.n : 0.0, t.dead[0],
                t.dead[1], t.dead[2], t.dead[3], t.dead[4], t.dead[5], t.dead[6]);
  };
  std::printf("\nGRIM_SURVIVAL by first-execution time of the earliest patched word (quintiles of "
              "executed code words)\n");
  for (size_t b = 0; b < by_bucket.size(); ++b) {
    emit("first_exec", b, range_text(b), by_bucket[b]);
  }
  std::printf("GRIM_SURVIVAL by mutation kind (kind of the gene holding the earliest patch)\n");
  for (size_t k = 0; k < by_kind.size(); ++k) {
    if (by_kind[k].n != 0) {
      emit("kind", k, grim_rom_mut_name(static_cast<GrimRomMut>(k)), by_kind[k]);
    }
  }
  write_text(csv_path, csv);
  std::printf("GRIM_SURVIVAL_CSV %s\n", csv_path.string().c_str());
}

} // namespace

int run_grim_explore_cli(const std::vector<std::string> &raw_args,
                         const std::string &self_exe) {
  grim_prepare_eval_process();
  std::vector<std::string> pos;
  std::string bios;
  double timeout = 300.0;
  std::vector<std::string> child_extra;
  GrimRandomParams params;
  RomMixOptions rom;
  for (size_t i = 0; i < raw_args.size(); ++i) {
    const std::string &a = raw_args[i];
    const bool has_value = i + 1 < raw_args.size();
    if (take_rom_option(raw_args, i, rom)) {
      continue;
    }
    if (a == "--bios" && has_value) {
      bios = raw_args[++i];
    } else if (a == "--timeout" && has_value) {
      timeout = std::atof(raw_args[++i].c_str());
    } else if (a == "--child-arg" && has_value) {
      child_extra.push_back(raw_args[++i]); // test hook: extra --grim-eval option
    } else if (a == "--families" && has_value) {
      const std::string f = raw_args[++i];
      params.spu = f == "spu" || f == "both";
      params.gpu = f == "gpu" || f == "both";
    } else if (a == "--genes" && has_value) {
      params.max_genes = static_cast<u32>(std::max(1, std::atoi(raw_args[++i].c_str())));
    } else {
      pos.push_back(a);
    }
  }
  if (bios.empty()) {
    const char *env = std::getenv("VIBESTATION_BIOS");
    bios = env != nullptr ? env : "";
  }
  if (pos.size() < 3 || bios.empty()) {
    std::fprintf(stderr,
                 "usage: --grim-explore <seed_start> <count> <out_dir> [frames=900] "
                 "[--bios path] [--timeout S=300] [--families spu|gpu|both] [--genes N]\n"
                 "  Phase 3: [--mix interface|rom|both] [--map file] [--rom-patches N] "
                 "[--rom-genes N] [--early-ms N] [--curve 0-3] [--call-swap]\n");
    return 1;
  }
  rom.bios = bios;
  if (!finish_rom_options(rom, bios)) {
    return 1;
  }
  apply_rom_options(params, rom);
  const u64 seed_start = std::strtoull(pos[0].c_str(), nullptr, 10);
  const u32 count = static_cast<u32>(std::max(1, std::atoi(pos[1].c_str())));
  const std::filesystem::path out_dir = pos[2];
  const u32 frames = pos.size() > 3 ? static_cast<u32>(std::max(1, std::atoi(pos[3].c_str()))) : 900u;
  params.horizon_frames = frames;
  params.min_genes = std::min(params.min_genes, params.max_genes);

  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(out_dir / "work", ec);
  fs::create_directories(out_dir / "survivors", ec);
  fs::create_directories(out_dir / "crashes", ec);
  fs::create_directories(out_dir / "dead", ec);
  const std::string exe = grim_self_exe_path(self_exe);

  struct Row {
    u64 seed;
    std::string verdict, reason, silent, hash;
    size_t genes;
    int bucket = -1, kind = -1; // ROM genes: first-execution quintile and kind of the earliest patch
  };
  std::vector<Row> rows;
  const std::array<u64, 4> time_edges = rom.wants_rom() ? exec_time_edges(rom.ctx)
                                                        : std::array<u64, 4>{};
  u32 alive = 0, dead = 0, timeouts = 0, crashes = 0, errors = 0;

  for (u32 n = 0; n < count; ++n) {
    const u64 seed = seed_start + n;
    const GrimGenome genome = grim_random_genome(seed, params);
    const std::string genome_text = grim_genome_serialize(genome);
    char hash_text[32];
    std::snprintf(hash_text, sizeof(hash_text), "0x%016llX",
                  static_cast<unsigned long long>(grim_genome_hash(genome)));
    const std::string seed_name = "seed_" + std::to_string(seed);
    const fs::path work = out_dir / "work" / seed_name;
    fs::remove_all(work, ec);
    fs::create_directories(work, ec);
    write_text(work / "genome.json", genome_text);

    std::vector<std::string> args = {"--grim-eval",
                                     bios,
                                     std::to_string(frames),
                                     (work / "telemetry.jsonl").string(),
                                     "--genome",
                                     (work / "genome.json").string(),
                                     "--dump-wav",
                                     (work / "audio.wav").string(),
                                     "--dump-frames",
                                     (work / "frames").string(),
                                     std::to_string(std::max(1u, frames / 4u)),
                                     "--no-stop-on-death"};
    args.insert(args.end(), child_extra.begin(), child_extra.end());
    const fs::path log = work / "child.log";
    const GrimChildResult child = grim_run_child(exe, args, log.string(), timeout);
    const std::string line = find_result_line(log.string());

    Row row{seed, "", "", "-", hash_text, genome.genes.size()};
    if (rom.wants_rom()) {
      rom_attribution(genome, rom.ctx, time_edges, row.bucket, row.kind);
    }
    bool is_survivor = false;
    if (child.timed_out) {
      row.verdict = "timeout";
      row.reason = "hang>" + std::to_string(static_cast<int>(timeout)) + "s";
      ++timeouts;
    } else if (child.crashed || !child.started) {
      row.verdict = "crash";
      row.reason = child.started ? "exit=" + std::to_string(child.exit_code) : "spawn_failed";
      ++crashes;
    } else if (line.empty() || line.find("status=error") != std::string::npos) {
      row.verdict = "error";
      row.reason = line.empty() ? "no_result_exit=" + std::to_string(child.exit_code)
                                : result_field(line, "reason");
      ++errors;
    } else {
      row.verdict = result_field(line, "verdict");
      row.reason = result_field(line, "reason");
      row.silent = result_field(line, "silent");
      if (row.verdict == "alive") {
        ++alive;
        is_survivor = true;
      } else {
        ++dead;
      }
    }

    if (is_survivor) {
      fs::remove_all(out_dir / "survivors" / seed_name, ec);
      fs::rename(work, out_dir / "survivors" / seed_name, ec);
    } else if (row.verdict == "dead") {
      write_text(out_dir / "dead" / (seed_name + ".json"), genome_text);
      fs::remove_all(work, ec);
    } else {
      write_text(out_dir / "crashes" / (seed_name + ".json"), genome_text);
      fs::copy_file(log, out_dir / "crashes" / (seed_name + ".log"),
                    fs::copy_options::overwrite_existing, ec);
      fs::remove_all(work, ec);
    }
    std::printf("GRIM_EXPLORE seed=%llu verdict=%s reason=%s silent=%s genome=%s\n",
                static_cast<unsigned long long>(seed), row.verdict.c_str(),
                row.reason.c_str(), row.silent.c_str(), row.hash.c_str());
    std::fflush(stdout);
    rows.push_back(row);
  }

  std::string table = "seed\tverdict\treason\tsilent\tgenome_hash\tgenes\n";
  for (const Row &row : rows) {
    table += std::to_string(row.seed) + "\t" + row.verdict + "\t" + row.reason + "\t" +
             row.silent + "\t" + row.hash + "\t" + std::to_string(row.genes) + "\n";
  }
  write_text(out_dir / "summary.tsv", table);
  std::printf("\n%s", table.c_str());
  std::printf("GRIM_EXPLORE_SUMMARY seeds=%u alive=%u dead=%u timeout=%u crash=%u error=%u "
              "out=%s\n",
              count, alive, dead, timeouts, crashes, errors, out_dir.string().c_str());
  if (rom.wants_rom()) {
    std::vector<SurvivalRow> tally_rows;
    for (const Row &row : rows) {
      tally_rows.push_back({row.bucket, row.kind, row.verdict == "alive" ? "alive"
                                                  : row.verdict == "dead" ? row.reason
                                                                          : row.verdict});
    }
    print_survival(tally_rows, rom.ctx, time_edges, out_dir / "survival.csv");
  }
  fs::remove(out_dir / "work", ec); // empty by now; ignore failure
  return 0;
}
