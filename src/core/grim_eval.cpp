#include "grim_eval.h"
#include "system.h"
#include "platform/disc_path_utils.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <thread>

namespace {
constexpr u64 kFnvOffset = 14695981039346656037ull;
constexpr u64 kFnvPrime = 1099511628211ull;

u64 fnv1a(u64 hash, const void *data, size_t bytes) {
  const u8 *p = static_cast<const u8 *>(data);
  for (size_t i = 0; i < bytes; ++i) {
    hash = (hash ^ p[i]) * kFnvPrime;
  }
  return hash;
}

double relative_change(double a, double b) {
  const double scale = std::max({std::fabs(a), std::fabs(b), 1e-9});
  return std::fabs(a - b) / scale;
}
} // namespace

GrimTelemetry::GrimTelemetry() : exec_bits_((kOtherWord + 64u) / 64u, 0ull) {}

void GrimTelemetry::note_exception(u32 cause, u32 epc) {
  ++exc_[cause & 15u];
  if (cause == static_cast<u32>(Exception::Interrupt)) {
    return;
  }
  if (cause == last_exc_cause_ && epc == last_exc_epc_) {
    ++exc_repeats_;
  }
  last_exc_cause_ = cause;
  last_exc_epc_ = epc;
}

GrimFrameTelemetry GrimTelemetry::end_frame(System &sys,
                                            const std::vector<s16> &audio,
                                            bool mute_audio) {
  GrimFrameTelemetry f;
  f.frame = frame_++;

  const u64 now = sys.cpu().cycle_count();
  f.cycles = now - last_cycle_;
  last_cycle_ = now;
  total_cycles_ += f.cycles;
  f.instructions = instructions_;
  total_instructions_ += instructions_;
  instructions_ = 0;
  f.new_pcs = new_pcs_;
  coverage_ += new_pcs_;
  f.coverage = coverage_;
  new_pcs_ = 0;
  f.exceptions = exc_;
  for (size_t i = 0; i < exc_.size(); ++i) {
    total_exc_[i] += exc_[i];
  }
  exc_.fill(0);
  f.exception_repeats = exc_repeats_;
  total_exc_repeats_ += exc_repeats_;
  exc_repeats_ = 0;

  // Per-frame GPU counters are reset by run_frame() itself.
  const System::ProfilingStats &ps = sys.profiling_stats();
  f.gp0_polygon = ps.gpu_flat_commands + ps.gpu_gouraud_commands +
                  ps.gpu_textured_commands + ps.gpu_gouraud_textured_commands;
  f.gp0_line = ps.gpu_line_commands;
  f.gp0_rect = ps.gpu_rect_commands - ps.gpu_fill_commands;
  f.gp0_fill = ps.gpu_fill_commands;
  f.gp0_transfer = ps.gpu_transfer_commands;
  f.gp0_other = ps.gpu_other_commands;
  f.gp1 = ps.gpu_gp1_commands;

  for (int ch = 0; ch < 7; ++ch) {
    const u64 transfers = sys.dma().debug_completed_transfers(ch);
    const u64 words = sys.dma().debug_moved_words(ch);
    f.dma_transfers[ch] = static_cast<u32>(transfers - last_dma_transfers_[ch]);
    f.dma_words[ch] = static_cast<u32>(words - last_dma_words_[ch]);
    last_dma_transfers_[ch] = transfers;
    last_dma_words_[ch] = words;
  }

  const u64 key_on = sys.spu_audio_diag().key_on_events;
  f.spu_key_on = static_cast<u32>(key_on - last_key_on_);
  last_key_on_ = key_on;
  f.spu_voice_mask = sys.spu().active_voice_mask();

  f.fb_hash = sys.boot_diag().display_hash;
  f.display_enabled = sys.boot_diag().display_enabled != 0;
  f.fb_lit = static_cast<u32>(sys.boot_diag().display_non_black_pixels);

  const size_t n = audio.size() / 2u;
  f.audio_samples = static_cast<u32>(n);
  u64 audio_hash = kFnvOffset;
  double sum_sq = 0.0;
  u32 crossings = 0;
  bool prev_negative = false;
  for (size_t i = 0; i < n; ++i) {
    const s16 l = mute_audio ? s16{0} : audio[i * 2u];
    const s16 r = mute_audio ? s16{0} : audio[i * 2u + 1u];
    audio_hash = fnv1a(audio_hash, &l, sizeof(l));
    audio_hash = fnv1a(audio_hash, &r, sizeof(r));
    sum_sq += static_cast<double>(l) * l + static_cast<double>(r) * r;
    const bool negative = (static_cast<s32>(l) + r) < 0;
    if (i != 0 && negative != prev_negative) {
      ++crossings;
    }
    prev_negative = negative;
  }
  f.audio_hash = audio_hash;
  f.audio_rms = n ? std::sqrt(sum_sq / static_cast<double>(n * 2u)) : 0.0;
  f.audio_zcr = n ? static_cast<double>(crossings) / static_cast<double>(n) : 0.0;

  const CpuDebugState cpu = sys.cpu().debug_state();
  u64 cpu_hash = fnv1a(kFnvOffset, cpu.gpr.data(), sizeof(u32) * cpu.gpr.size());
  const u32 regs[] = {cpu.pc,      cpu.hi,        cpu.lo,
                      cpu.cop0_sr, cpu.cop0_cause, cpu.cop0_epc,
                      cpu.cop0_badvaddr};
  f.cpu_hash = fnv1a(cpu_hash, regs, sizeof(regs));
  return f;
}

bool GrimLivenessTracker::update(const GrimFrameTelemetry &f) {
  if (!verdict_.alive) {
    return true;
  }
  ++frames_;
  const bool fb_changed = have_prev_ && f.fb_hash != prev_fb_;
  const bool audio_moving =
      have_prev_ && f.audio_rms >= cfg_.audio_silence_rms && f.audio_zcr > 0.0 &&
      (relative_change(f.audio_rms, prev_rms_) > cfg_.audio_change_ratio ||
       relative_change(f.audio_zcr, prev_zcr_) > cfg_.audio_change_ratio);
  if (audio_moving) {
    ++audio_moving_frames_;
  }
  if (f.display_enabled && f.fb_lit >= cfg_.image_min_lit_pixels) {
    drew_image_ = true;
  }
  const bool new_code = f.new_pcs != 0;
  u64 gp0 = static_cast<u64>(f.gp0_polygon) + f.gp0_line + f.gp0_rect +
            f.gp0_fill + f.gp0_transfer + f.gp0_other;
  u64 dma_words = 0;
  for (u32 w : f.dma_words) {
    dma_words += w;
  }
  const bool device_work = gp0 != 0 || dma_words != 0 || f.spu_key_on != 0;

  const bool inert = !new_code && !fb_changed && !audio_moving && !device_work;
  stall_run_ = inert ? stall_run_ + 1u : 0u;
  const bool frozen = !fb_changed && !audio_moving && gp0 == 0;
  frozen_run_ = frozen ? frozen_run_ + 1u : 0u;

  u64 non_irq = 0;
  for (size_t cause = 1; cause < f.exceptions.size(); ++cause) {
    non_irq += f.exceptions[cause];
  }
  const bool looping =
      !new_code && non_irq >= cfg_.exception_loop_min_per_frame &&
      static_cast<double>(f.exception_repeats) >=
          cfg_.exception_loop_repeat_ratio * static_cast<double>(non_irq);
  exc_loop_run_ = looping ? exc_loop_run_ + 1u : 0u;

  const char *reason = nullptr;
  if (exc_loop_run_ >= cfg_.exception_loop_frames) {
    reason = "exception_loop";
  } else if (stall_run_ >= cfg_.coverage_stall_frames) {
    reason = "coverage_stall";
  } else if (frozen_run_ >= cfg_.frozen_frame_frames) {
    reason = "frozen_frame";
  }
  if (reason != nullptr) {
    verdict_.alive = false;
    verdict_.reason = reason;
    verdict_.death_frame = f.frame;
  }

  have_prev_ = true;
  prev_fb_ = f.fb_hash;
  prev_rms_ = f.audio_rms;
  prev_zcr_ = f.audio_zcr;
  return !verdict_.alive;
}

GrimLiveness GrimLivenessTracker::finish() const {
  GrimLiveness v = verdict_;
  v.silent = frames_ >= cfg_.audio_min_run_frames &&
             audio_moving_frames_ < cfg_.audio_min_moving_frames;
  if (v.alive && v.silent && !drew_image_) {
    v.alive = false;
    v.reason = "dead_audio";
    v.death_frame = static_cast<s64>(frames_) - 1;
  }
  return v;
}

GrimLiveness grim_evaluate_liveness(const std::vector<GrimFrameTelemetry> &frames,
                                    const GrimLivenessConfig &cfg) {
  GrimLivenessTracker tracker(cfg);
  for (const GrimFrameTelemetry &f : frames) {
    if (tracker.update(f)) {
      break;
    }
  }
  return tracker.finish();
}

GrimEvalResult run_grim_eval(const GrimEvalConfig &cfg) {
  GrimEvalResult r;
  if (!cfg.native_cpu && effective_cpu_execution_mode() != CpuExecutionMode::Interpreter) {
    r.end_reason = "unsupported_cpu_mode";
    return r;
  }
  auto sys = std::make_unique<System>();
  if (!sys->load_bios(cfg.bios_path)) {
    r.end_reason = "bios_load_failed";
    return r;
  }
  sys->reset();
  if (!cfg.disc_cue.empty()) {
    const std::string bin = resolve_first_bin_from_cue(cfg.disc_cue);
    if (bin.empty() || !sys->load_game(bin, cfg.disc_cue)) {
      r.end_reason = "disc_load_failed";
      return r;
    }
  }

  std::unique_ptr<GrimGenomeRuntime> genome;
  if (cfg.use_genome) {
    genome = std::make_unique<GrimGenomeRuntime>(cfg.genome);
    sys->set_grim_genome(genome.get()); // applies ROM genes to the stock image
    r.genome_hash = genome->hash();
    if (!sys->grim_rom_error().empty()) {
      r.end_reason = "rom_gene_mismatch";
      r.error_detail = sys->grim_rom_error();
      sys->set_grim_genome(nullptr);
      return r;
    }
  }
  for (const auto &[offset, word] : cfg.bios_patches) {
    sys->bios_mut().patch32(offset, word);
  }
  if (cfg.test_hang) {
    for (;;) { // test only: never returns, the parent's timeout must kill us
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  std::vector<s16> wav_samples;
  if (!cfg.dump_frames_dir.empty() && cfg.dump_frames_every > 0) {
    std::error_code ec;
    std::filesystem::create_directories(cfg.dump_frames_dir, ec);
  }

  GrimTelemetry telemetry;
  std::unique_ptr<GrimBootMapper> mapper;
  if (!cfg.native_cpu) {
    sys->cpu().set_telemetry(&telemetry);
    if (cfg.boot_map_out != nullptr) {
      mapper = std::make_unique<GrimBootMapper>(sys->bios_mut().image_size());
      sys->set_grim_boot_mapper(mapper.get());
    }
  }
  // The capture buffer is the headless null sink: samples stop there and
  // never reach a host device.
  sys->set_spu_audio_capture(true);
  GrimLivenessTracker liveness(cfg.liveness);
  u64 run_hash = kFnvOffset;
  const auto start = std::chrono::steady_clock::now();
  auto wall_seconds = [&] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
        .count();
  };

  for (u32 i = 0; i < cfg.frames; ++i) {
    sys->run_frame(true, false);
    if (!cfg.dump_wav_path.empty()) {
      const std::vector<s16> &a = sys->spu_audio_capture_samples();
      wav_samples.insert(wav_samples.end(), a.begin(), a.end());
    }
    if (!cfg.dump_frames_dir.empty() && cfg.dump_frames_every > 0 &&
        (i % cfg.dump_frames_every) == 0) {
      std::vector<u32> rgba;
      const DisplaySampleInfo info = sys->gpu().build_display_rgba(rgba, false);
      char name[32];
      std::snprintf(name, sizeof(name), "frame_%05u.bmp", i);
      grim_write_bmp((std::filesystem::path(cfg.dump_frames_dir) / name).string(),
                     info.width, info.height, rgba);
    }
    const GrimFrameTelemetry f =
        telemetry.end_frame(*sys, sys->spu_audio_capture_samples(), cfg.mute_audio);
    sys->clear_spu_audio_capture();
    r.frames.push_back(f);
    r.cd_words += f.dma_words[3];
    const std::string line = grim_frame_json(f);
    run_hash = fnv1a(run_hash, line.data(), line.size());

    if (liveness.update(f) && cfg.stop_on_death) {
      r.end_reason = "death";
      break;
    }
    if (cfg.max_cycles != 0 && telemetry.total_cycles() >= cfg.max_cycles) {
      r.end_reason = "cycle_cap";
      break;
    }
    if (cfg.max_instructions != 0 &&
        telemetry.total_instructions() >= cfg.max_instructions) {
      r.end_reason = "instruction_cap";
      break;
    }
    // ponytail: only checked between frames; a hang inside run_frame needs
    // the parent process to kill this one (see PROGRESS.md).
    if (cfg.watchdog_seconds > 0.0 && wall_seconds() > cfg.watchdog_seconds) {
      r.end_reason = "watchdog";
      break;
    }
  }
  sys->cpu().set_telemetry(nullptr);
  if (mapper != nullptr) {
    sys->set_grim_boot_mapper(nullptr);
    *cfg.boot_map_out = mapper->finish(sys->bios_mut().image_hash(), cfg.map_scenario,
                                       static_cast<u32>(r.frames.size()),
                                       telemetry.total_cycles());
    if (cfg.copy_stats_out != nullptr) {
      *cfg.copy_stats_out = mapper->stats_report();
    }
  }
  if (genome != nullptr) {
    r.gene_hits = genome->hits();
    sys->set_grim_genome(nullptr);
  }
  if (!cfg.dump_wav_path.empty()) {
    grim_write_wav(cfg.dump_wav_path, wav_samples);
  }

  r.liveness = liveness.finish();
  r.cycles = telemetry.total_cycles();
  r.instructions = telemetry.total_instructions();
  r.coverage = telemetry.coverage();
  r.exceptions = telemetry.total_exceptions();
  r.exception_repeats = telemetry.total_exception_repeats();
  r.run_hash = run_hash;
  r.wall_seconds = wall_seconds();
  r.emulated_seconds =
      static_cast<double>(r.cycles) / static_cast<double>(psx::CPU_CLOCK_HZ);
  r.speed_factor = r.wall_seconds > 0.0 ? r.emulated_seconds / r.wall_seconds : 0.0;
  return r;
}

namespace {
template <typename T, size_t N>
std::string json_array(const std::array<T, N> &values) {
  std::string out = "[";
  for (size_t i = 0; i < N; ++i) {
    out += (i ? "," : "") + std::to_string(values[i]);
  }
  return out + "]";
}
} // namespace

std::string grim_frame_json(const GrimFrameTelemetry &f) {
  char head[256];
  std::snprintf(head, sizeof(head),
                "{\"frame\":%u,\"cycles\":%llu,\"instr\":%llu,\"new_pcs\":%u,"
                "\"coverage\":%u,\"exc\":",
                f.frame, static_cast<unsigned long long>(f.cycles),
                static_cast<unsigned long long>(f.instructions), f.new_pcs,
                f.coverage);
  char mid[256];
  std::snprintf(mid, sizeof(mid),
                ",\"exc_repeat\":%u,\"gp0\":{\"poly\":%u,\"line\":%u,\"rect\":%u,"
                "\"fill\":%u,\"xfer\":%u,\"other\":%u},\"gp1\":%u,\"dma_n\":",
                f.exception_repeats, f.gp0_polygon, f.gp0_line, f.gp0_rect,
                f.gp0_fill, f.gp0_transfer, f.gp0_other, f.gp1);
  char tail[384];
  std::snprintf(tail, sizeof(tail),
                ",\"key_on\":%u,\"voices\":\"0x%06X\",\"fb\":\"0x%08X\","
                "\"display\":%u,\"lit\":%u,\"audio_n\":%u,\"audio_hash\":\"0x%016llX\","
                "\"rms\":%.3f,\"zcr\":%.5f,\"cpu_hash\":\"0x%016llX\"}",
                f.spu_key_on, f.spu_voice_mask, f.fb_hash,
                f.display_enabled ? 1u : 0u, f.fb_lit, f.audio_samples,
                static_cast<unsigned long long>(f.audio_hash), f.audio_rms,
                f.audio_zcr, static_cast<unsigned long long>(f.cpu_hash));
  return std::string(head) + json_array(f.exceptions) + mid +
         json_array(f.dma_transfers) + ",\"dma_w\":" + json_array(f.dma_words) +
         tail;
}

std::string grim_summary_json(const GrimEvalResult &r) {
  char head[512];
  std::snprintf(head, sizeof(head),
                "{\"summary\":{\"end_reason\":\"%s\",\"frames\":%zu,"
                "\"cycles\":%llu,\"instructions\":%llu,\"coverage\":%u,"
                "\"exceptions\":",
                r.end_reason.c_str(), r.frames.size(),
                static_cast<unsigned long long>(r.cycles),
                static_cast<unsigned long long>(r.instructions), r.coverage);
  std::string hits = "[";
  for (size_t i = 0; i < r.gene_hits.size(); ++i) {
    hits += (i ? "," : "") + std::to_string(r.gene_hits[i]);
  }
  hits += "]";
  char tail[1024];
  std::snprintf(tail, sizeof(tail),
                ",\"exception_repeats\":%llu,\"verdict\":\"%s\",\"reason\":\"%s\","
                "\"death_frame\":%lld,\"silent\":%s,\"run_hash\":\"0x%016llX\","
                "\"genome_hash\":\"0x%016llX\",\"gene_hits\":%s,"
                "\"emulated_s\":%.3f,\"wall_s\":%.3f,\"speed\":%.2f}}",
                static_cast<unsigned long long>(r.exception_repeats),
                r.liveness.alive ? "alive" : "dead", r.liveness.reason.c_str(),
                static_cast<long long>(r.liveness.death_frame),
                r.liveness.silent ? "true" : "false",
                static_cast<unsigned long long>(r.run_hash),
                static_cast<unsigned long long>(r.genome_hash), hits.c_str(),
                r.emulated_seconds, r.wall_seconds, r.speed_factor);
  return std::string(head) + json_array(r.exceptions) + tail;
}

bool grim_write_telemetry_jsonl(const GrimEvalResult &r, const std::string &path) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    return false;
  }
  for (const GrimFrameTelemetry &f : r.frames) {
    out << grim_frame_json(f) << '\n';
  }
  out << grim_summary_json(r) << '\n';
  return static_cast<bool>(out);
}

bool grim_write_wav(const std::string &path, const std::vector<s16> &samples) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    return false;
  }
  const u32 data_bytes = static_cast<u32>(samples.size() * sizeof(s16));
  const auto put32 = [&](u32 v) {
    const char b[4] = {static_cast<char>(v), static_cast<char>(v >> 8),
                       static_cast<char>(v >> 16), static_cast<char>(v >> 24)};
    out.write(b, 4);
  };
  const auto put16 = [&](u16 v) {
    const char b[2] = {static_cast<char>(v), static_cast<char>(v >> 8)};
    out.write(b, 2);
  };
  constexpr u32 kRate = 44100; // Spu::SAMPLE_RATE
  out.write("RIFF", 4);
  put32(36u + data_bytes);
  out.write("WAVEfmt ", 8);
  put32(16);
  put16(1);  // PCM
  put16(2);  // stereo
  put32(kRate);
  put32(kRate * 4u);
  put16(4);
  put16(16);
  out.write("data", 4);
  put32(data_bytes);
  // Little-endian on every supported host; written as bytes to be explicit.
  for (const s16 s : samples) {
    put16(static_cast<u16>(s));
  }
  return static_cast<bool>(out);
}

bool grim_write_bmp(const std::string &path, int width, int height,
                    const std::vector<u32> &rgba) {
  if (width <= 0 || height <= 0 ||
      rgba.size() < static_cast<size_t>(width) * static_cast<size_t>(height)) {
    return false;
  }
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    return false;
  }
  const u32 row_bytes = (static_cast<u32>(width) * 3u + 3u) & ~3u;
  const u32 image_bytes = row_bytes * static_cast<u32>(height);
  const auto put32 = [&](u32 v) {
    const char b[4] = {static_cast<char>(v), static_cast<char>(v >> 8),
                       static_cast<char>(v >> 16), static_cast<char>(v >> 24)};
    out.write(b, 4);
  };
  const auto put16 = [&](u16 v) {
    const char b[2] = {static_cast<char>(v), static_cast<char>(v >> 8)};
    out.write(b, 2);
  };
  out.write("BM", 2);
  put32(54u + image_bytes);
  put32(0);
  put32(54);
  put32(40);
  put32(static_cast<u32>(width));
  put32(static_cast<u32>(height)); // positive: bottom-up rows
  put16(1);
  put16(24);
  put32(0);
  put32(image_bytes);
  put32(2835);
  put32(2835);
  put32(0);
  put32(0);
  std::vector<char> row(row_bytes, 0);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const u32 px = rgba[static_cast<size_t>(y) * static_cast<size_t>(width) +
                          static_cast<size_t>(x)];
      // The display buffer is R in the low byte (0xAABBGGRR).
      row[static_cast<size_t>(x) * 3u + 0u] = static_cast<char>(px >> 16);
      row[static_cast<size_t>(x) * 3u + 1u] = static_cast<char>(px >> 8);
      row[static_cast<size_t>(x) * 3u + 2u] = static_cast<char>(px);
    }
    out.write(row.data(), static_cast<std::streamsize>(row.size()));
  }
  return static_cast<bool>(out);
}
