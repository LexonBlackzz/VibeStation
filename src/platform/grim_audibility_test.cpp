#include "grim_audibility_test.h"
#include "grim_eval_runner.h"
#include "core/grim_audibility.h"
#include "core/grim_eval.h"
#include "core/grim_genome.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace {
int failures = 0;
void check(bool ok, const std::string &name, const std::string &detail = "") {
  std::printf("GRIM_AUDIBILITY_TEST %s %s %s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.c_str());
  std::fflush(stdout);
  failures += ok ? 0 : 1;
}
std::vector<s16> tone(size_t frames, s16 amplitude) {
  std::vector<s16> pcm(frames * 2u);
  for (size_t i = 0; i < frames; ++i)
    pcm[i * 2u] = pcm[i * 2u + 1u] = ((i / 10u) & 1u) ? amplitude : -amplitude;
  return pcm;
}
void add_residual(std::vector<s16> &pcm, size_t first_frame, size_t frames, s16 amplitude,
                   bool left_only = false) {
  for (size_t i = first_frame; i < first_frame + frames; ++i) {
    const s16 difference = ((i / 10u) & 1u) ? amplitude : -amplitude;
    pcm[i * 2u] = static_cast<s16>(pcm[i * 2u] + difference);
    if (!left_only) pcm[i * 2u + 1u] = static_cast<s16>(pcm[i * 2u + 1u] + difference);
  }
}
void wav_reader_fixtures() {
  // A tiny independent RIFF fixture covers signed PCM and a padded unknown
  // chunk. Data precedes fmt, which is legal and exercises chunk traversal.
  std::vector<unsigned char> wav;
  const auto tag = [&](const char *s) { wav.insert(wav.end(), s, s + 4); };
  const auto word = [&](u32 value) {
    for (u32 shift = 0; shift < 32; shift += 8) wav.push_back(static_cast<unsigned char>(value >> shift));
  };
  const auto half = [&](u16 value) {
    wav.push_back(static_cast<unsigned char>(value));
    wav.push_back(static_cast<unsigned char>(value >> 8));
  };
  tag("RIFF"); word(0); tag("WAVE");
  tag("JUNK"); word(3); wav.insert(wav.end(), {1, 2, 3, 0});
  tag("data"); word(8); half(0x8000); half(0xFFFF); half(0); half(0x7FFF);
  tag("fmt ");
  const size_t format_length_offset = wav.size();
  word(16); half(1); half(2);
  const size_t rate_offset = wav.size();
  word(44100); word(176400); half(4); half(16);
  const u32 riff_size = static_cast<u32>(wav.size() - 8);
  for (u32 i = 0; i < 4; ++i) wav[4 + i] = static_cast<unsigned char>(riff_size >> (i * 8u));
  const auto path = std::filesystem::temp_directory_path() /
    ("vibestation_grim_audio_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".wav");
  const auto save = [&]() {
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char *>(wav.data()), static_cast<std::streamsize>(wav.size()));
    return static_cast<bool>(out);
  };
  std::vector<s16> samples;
  std::string error;
  check(save() && grim_read_wav(path.string(), samples, error) &&
        samples == std::vector<s16>({-32768, -1, 0, 32767}), "WAV_chunks_signed_PCM");
  const auto valid = wav;
  wav[format_length_offset] = 20;
  check(save() && !grim_read_wav(path.string(), samples, error) && samples.empty(),
        "WAV_rejects_truncated_extended_fmt_after_data");
  wav = valid;
  const auto set_riff_length = [&](u32 bytes) {
    for (u32 i = 0; i < 4; ++i) wav[4 + i] = static_cast<unsigned char>(bytes >> (i * 8u));
  };
  set_riff_length(riff_size + 4);
  check(save() && !grim_read_wav(path.string(), samples, error) && samples.empty(),
        "WAV_rejects_RIFF_length_beyond_physical_file");
  set_riff_length(riff_size - 4);
  check(save() && !grim_read_wav(path.string(), samples, error) && samples.empty(),
        "WAV_rejects_chunk_beyond_declared_RIFF_length");
  wav = valid;
  wav[format_length_offset] = 17;
  wav.push_back(0); // Extended fmt byte is present, but its alignment pad is absent.
  set_riff_length(riff_size + 1);
  check(save() && !grim_read_wav(path.string(), samples, error) && samples.empty(),
        "WAV_rejects_missing_odd_chunk_padding");
  wav = valid;
  tag("fmt "); word(2); half(1);
  set_riff_length(static_cast<u32>(wav.size() - 8));
  check(save() && !grim_read_wav(path.string(), samples, error) && samples.empty(),
        "WAV_rejects_trailing_short_fmt_after_valid_chunks");
  wav = valid;
  wav[rate_offset] = 0;
  check(save() && !grim_read_wav(path.string(), samples, error) && samples.empty() && !error.empty(),
        "WAV_rejects_wrong_format_without_partial_output");
  wav.resize(20);
  check(save() && !grim_read_wav(path.string(), samples, error) && samples.empty(),
        "WAV_rejects_truncated_chunks");
  std::error_code ec;
  std::filesystem::remove(path, ec);
  check(!ec, "WAV_fixture_cleanup", ec.message());
}
void unit_tests() {
  const GrimAudibilityConfig cfg;
  const auto clean = tone(cfg.window_frames * 5u, 1000);
  const auto identical = grim_compare_audio(clean, clean);
  check(identical.valid && !identical.audible && identical.residual_energy == 0 &&
        identical.peak_residual_db == -120.0, "identical_is_inaudible");
  auto unequal = clean;
  unequal.pop_back();
  check(!grim_compare_audio(clean, unequal).valid &&
        !grim_compare_audio({}, {}).valid, "requires_aligned_nonempty_stereo");
  auto bad_cfg = cfg;
  bad_cfg.window_frames = 4097;
  check(!grim_compare_audio(clean, clean, bad_cfg).valid, "rejects_unsafe_window_size");
  auto changed = clean;
  add_residual(changed, 0, cfg.window_frames * 4u, 100);
  auto r = grim_compare_audio(clean, changed);
  check(r.audible && r.above_threshold_frames == cfg.window_frames * 4u &&
        r.peak_residual_db == -20.0, "integer_ratio_exact_threshold");
  changed = clean;
  add_residual(changed, 0, cfg.window_frames * 4u, 99);
  check(!grim_compare_audio(clean, changed).audible, "below_energy_threshold");
  changed = clean;
  add_residual(changed, 0, cfg.window_frames * 3u, 1000);
  r = grim_compare_audio(clean, changed);
  check(!r.audible && r.above_threshold_frames == cfg.window_frames * 3u &&
        r.peak_residual_db == 0.0, "short_loud_residual_needs_exposure");
  changed = clean;
  add_residual(changed, 0, cfg.window_frames * 2u, 1000);
  add_residual(changed, cfg.window_frames * 3u, cfg.window_frames * 2u, 1000);
  r = grim_compare_audio(clean, changed);
  check(r.audible && r.above_threshold_frames == cfg.window_frames * 4u &&
        r.longest_above_frames == cfg.window_frames * 2u, "cumulative_and_contiguous_exposure");
  const std::vector<s16> silence(clean.size());
  changed = silence;
  add_residual(changed, 0, changed.size() / 2u, 7);
  check(!grim_compare_audio(silence, changed).audible, "near_silence_absolute_residual_floor");
  changed = silence;
  add_residual(changed, 0, changed.size() / 2u, 8);
  check(grim_compare_audio(silence, changed).audible, "near_silence_energy_floor");
  changed = clean;
  add_residual(changed, 0, changed.size() / 2u, 1000, true);
  r = grim_compare_audio(clean, changed);
  check(r.audible && r.channel_peak_db[0] == 0.0 && r.channel_peak_db[1] == -120.0 &&
        r.peak_residual_db == -3.01, "stereo_residual_preserves_channel_evidence");
  std::vector<s16> full_scale(clean.size(), -32768), opposite(clean.size(), 32767);
  r = grim_compare_audio(full_scale, opposite);
  check(r.audible && r.mutated_clipped_samples == clean.size() &&
        r.clean_clipped_samples == clean.size() && r.peak_residual_db == 6.02,
        "full_scale_residual_does_not_overflow");
  auto quiet_difference = full_scale;
  quiet_difference.back() = -32767;
  r = grim_compare_audio(full_scale, quiet_difference);
  check(!r.audible && r.residual_energy == 1 && r.peak_residual_energy == 1 &&
        r.peak_residual_db < -100.0 && r.peak_residual_db != -120.0,
        "sub_ppm_peak_is_not_rounded_to_zero");
  GrimAudibilityConfig partial_cfg;
  partial_cfg.window_frames = 1000;
  partial_cfg.minimum_above_frames = 4401;
  const auto partial_clean = tone(4401, 1000);
  auto partial_mutated = partial_clean;
  add_residual(partial_mutated, 0, 4401, 1000);
  r = grim_compare_audio(partial_clean, partial_mutated, partial_cfg);
  check(r.audible && r.windows == 5 && r.above_threshold_frames == 4401 &&
        r.longest_above_frames == 4401, "partial_final_window_uses_actual_frame_count");
  check(grim_audibility_json(r) == grim_audibility_json(
          grim_compare_audio(partial_clean, partial_mutated, partial_cfg)),
        "metric_report_is_deterministic");
  wav_reader_fixtures();
}

void labelled_fixtures(const std::string &bios, const std::filesystem::path &labels_path) {
  nlohmann::json labels;
  try {
    std::ifstream in(labels_path);
    if (!in) throw std::runtime_error("cannot open labels file");
    in >> labels;
    if (!labels.is_object() || labels.at("version") != 1 || labels.at("scenario") != "nodisc" ||
        !labels.at("frames").is_number_unsigned() || !labels.at("labels").is_array())
      throw std::runtime_error("unsupported label file schema/scenario");
  } catch (const std::exception &e) {
    check(false, "load_listening_labels", e.what()); return;
  }
  const u32 frames = labels.at("frames").get<u32>();
  if (frames < 300 || frames > 3600) { check(false, "label_frame_count"); return; }
  const auto base = labels_path.parent_path() / labels.value("genome_base", ".");
  GrimEvalConfig cfg;
  cfg.bios_path = bios;
  cfg.frames = frames;
  cfg.watchdog_seconds = 300;
  cfg.stop_on_death = false;
  std::vector<s16> clean_audio;
  cfg.audio_out = &clean_audio;
  const auto clean = run_grim_eval(cfg);
  check(clean.end_reason == "frames" && clean.liveness.alive && !clean.liveness.silent,
        "clean_boot_audio_is_live");
  if (clean.end_reason != "frames") return;
  std::vector<s16> rec_clean_audio;
  cfg.audio_out = &rec_clean_audio;
  cfg.native_cpu = true;
  g_cpu_execution_mode_cli_value = CpuExecutionMode::Recompiler;
  const auto rec_clean = run_grim_eval(cfg);
  g_cpu_execution_mode_cli_value = CpuExecutionMode::Interpreter;
  cfg.native_cpu = false;
  check(rec_clean.end_reason == "frames" && rec_clean_audio == clean_audio,
        "clean_PCM_matches_between_backends");
  size_t count = 0;
  for (const auto &label : labels.at("labels")) {
    if (!label.is_object() || !label.contains("genome") || !label.at("genome").is_string() ||
        !label.contains("audible") || (!label.at("audible").is_boolean() && !label.at("audible").is_null())) {
      check(false, "listening_label_schema"); continue;
    }
    if (label.at("audible").is_null()) continue; // pending verdict from Lexon
    std::filesystem::path genome_path = label.at("genome").get<std::string>();
    if (genome_path.is_relative()) genome_path = base / genome_path;
    const std::string name = genome_path.stem().string();
    GrimGenome genome;
    std::string error;
    if (!grim_genome_load(genome_path.string(), genome, error)) {
      check(false, name + "_load", error); continue;
    }
    ++count;
    cfg.use_genome = true;
    cfg.genome = genome;
    std::vector<s16> audio;
    cfg.audio_out = &audio;
    const auto result = run_grim_eval(cfg);
    const auto metric = grim_compare_audio(clean_audio, audio);
    const bool expected = label.at("audible").get<bool>();
    check(result.end_reason == "frames" && result.liveness.alive && !result.liveness.silent &&
          metric.valid && metric.audible == expected, name + "_human_label",
          "expected=" + std::to_string(expected) + " peak_db=" + std::to_string(metric.peak_residual_db) +
          " above_ms=" + std::to_string(metric.above_threshold_ms) +
          " exposure_margin_ms=" + std::to_string(metric.above_threshold_ms - 100.0));
    std::printf("GRIM_AUDIBILITY_FIXTURE %s %s\n", name.c_str(), grim_audibility_json(metric).c_str());
    std::fflush(stdout);
    std::vector<s16> repeated_audio;
    cfg.audio_out = &repeated_audio;
    const auto repeated = run_grim_eval(cfg);
    check(result.run_hash == repeated.run_hash && audio == repeated_audio &&
          grim_audibility_json(metric) == grim_audibility_json(grim_compare_audio(clean_audio, repeated_audio)),
          name + "_repeat_is_exact");
    std::vector<s16> rec_audio;
    cfg.audio_out = &rec_audio;
    cfg.native_cpu = true;
    g_cpu_execution_mode_cli_value = CpuExecutionMode::Recompiler;
    const auto rec = run_grim_eval(cfg);
    g_cpu_execution_mode_cli_value = CpuExecutionMode::Interpreter;
    cfg.native_cpu = false;
    check(rec.end_reason == "frames" && audio == rec_audio &&
          grim_audibility_json(metric) == grim_audibility_json(grim_compare_audio(rec_clean_audio, rec_audio)),
          name + "_metric_matches_between_backends");
  }
  check(count != 0, "listening_labels_are_exercised", "count=" + std::to_string(count));
}
} // namespace

int run_grim_audibility_test(const std::vector<std::string> &raw_args) {
  grim_prepare_eval_process();
  auto args = raw_args;
  std::string bios;
  if (!args.empty() && args[0].rfind("--", 0) != 0) {
    bios = args[0]; args.erase(args.begin());
  } else {
    const char *env = std::getenv("VIBESTATION_BIOS");
    if (env) bios = env;
  }
  bool unit_only = false;
  std::filesystem::path labels = "docs/grim-reaper/audio_labels.json";
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--labels" && i + 1 < args.size()) labels = args[++i];
    else if (args[i] == "--unit-only") unit_only = true;
    else { std::fprintf(stderr, "unknown audibility-test option: %s\n", args[i].c_str()); return 1; }
  }
  failures = 0;
  unit_tests();
  if (!unit_only) {
    if (bios.empty()) {
      std::fprintf(stderr, "--grim-audibility-test requires [bios] or VIBESTATION_BIOS (or --unit-only)\n");
      return 1;
    }
    labelled_fixtures(bios, labels);
  }
  std::printf("GRIM_AUDIBILITY_TEST_RESULT %s failures=%d\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
