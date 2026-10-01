#include "platform/grim_audibility_runner.h"
#include "platform/grim_eval_runner.h"
#include "core/grim_audibility.h"
#include "core/grim_eval.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>

namespace {
bool write_report(const std::string &path, const std::string &report) {
  std::ofstream out(path, std::ios::binary);
  out << report << '\n';
  return static_cast<bool>(out);
}
int error(const std::string &message) {
  std::fprintf(stderr, "GRIM_AUDIBILITY_ERROR %s\n", message.c_str());
  return 1;
}
} // namespace

int run_grim_audio_compare_cli(const std::vector<std::string> &args) {
  if (args.size() < 2 || args.size() > 3) {
    return error("usage: --grim-audio-compare <clean.wav> <mutated.wav> [out.json]");
  }
  std::vector<s16> clean, mutated;
  std::string err;
  if (!grim_read_wav(args[0], clean, err) || !grim_read_wav(args[1], mutated, err)) {
    return error(err);
  }
  if (clean.size() != mutated.size()) return error("PCM lengths differ; compare the same frames");
  const auto report = grim_compare_audio(clean, mutated);
  if (!report.valid) return error(report.error);
  const auto json = grim_audibility_json(report);
  if (args.size() == 3 && !write_report(args[2], json)) return error("cannot write report");
  std::printf("%s\n", json.c_str());
  return 0;
}

int run_grim_audibility_cli(const std::vector<std::string> &args) {
  if (args.size() < 5) {
    return error("usage: --grim-audibility <bios> <frames> <genome|clean> <clean.wav> <out.json> "
                 "[--native-cpu] [--dump-wav path] [--telemetry path]");
  }
  const auto requested_cpu = effective_cpu_execution_mode();
  grim_prepare_eval_process();
  GrimEvalConfig cfg;
  cfg.bios_path = args[0];
  char *end = nullptr;
  const auto frames = std::strtoull(args[1].c_str(), &end, 10);
  if (*end || !frames || frames > std::numeric_limits<u32>::max()) return error("invalid frames");
  cfg.frames = static_cast<u32>(frames);
  std::string err, telemetry_path;
  if (args[2] != "clean") {
    if (!grim_genome_load(args[2], cfg.genome, err)) return error(err);
    cfg.use_genome = true;
  }
  for (size_t i = 5; i < args.size(); ++i) {
    if (args[i] == "--native-cpu") {
      cfg.native_cpu = true;
      g_cpu_execution_mode_cli_value = requested_cpu;
    } else if (args[i] == "--dump-wav" && i + 1 < args.size()) {
      cfg.dump_wav_path = args[++i];
    } else if (args[i] == "--telemetry" && i + 1 < args.size()) {
      telemetry_path = args[++i];
    } else return error("unknown or incomplete option: " + args[i]);
  }
  std::vector<s16> clean, audio;
  if (!grim_read_wav(args[3], clean, err)) return error(err);
  cfg.audio_out = &audio;
  const auto result = run_grim_eval(cfg);
  if (result.frames.empty()) return error(result.end_reason + " " + result.error_detail);
  if (!telemetry_path.empty() && !grim_write_telemetry_jsonl(result, telemetry_path)) {
    return error("cannot write telemetry");
  }
  // Early deaths compare their available prefix, with an explicit completeness
  // field. Liveness remains the gate before audibility is used by search.
  if (audio.size() > clean.size()) return error("reference is shorter than evaluation");
  const bool complete = result.frames.size() == cfg.frames && audio.size() == clean.size();
  clean.resize(audio.size());
  const auto report = grim_compare_audio(clean, audio);
  if (!report.valid) return error(report.error);
  const auto json = std::string("{\"evaluation\":") + grim_summary_json(result) +
                    ",\"complete\":" + (complete ? "true" : "false") +
                    ",\"audibility\":" + grim_audibility_json(report) + "}";
  if (!write_report(args[4], json)) return error("cannot write report");
  std::printf("%s\n", json.c_str());
  if (result.end_reason != "frames" && result.end_reason != "death") {
    return error("evaluation did not complete: " + result.end_reason);
  }
  if (result.liveness.alive && !complete) {
    return error("PCM lengths differ; reference and evaluation must cover the same frames");
  }
  return result.liveness.alive ? 0 : 2;
}
