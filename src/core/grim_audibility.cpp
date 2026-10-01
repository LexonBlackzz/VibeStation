#include "grim_audibility.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <nlohmann/json.hpp>

namespace {
constexpr u64 kSampleRate = 44100;
double rounded(double value) { return std::round(value * 1000.0) / 1000.0; }
double energy_db(u64 residual, u64 reference) {
  if (!residual) return -120.0;
  return rounded(10.0 * std::log10(static_cast<double>(residual) /
                                 static_cast<double>(reference)));
}
u64 ratio_ppm(u64 residual, u64 reference) {
  // Windows are capped at 4096 stereo frames. Both the reference energy and
  // its remainder times one million fit in u64, including full-scale PCM.
  return (residual / reference) * 1000000u +
         ((residual % reference) * 1000000u) / reference;
}
bool ratio_greater(u64 a, u64 b, u64 c, u64 d) {
  // Compare fractions exactly without overflowing a*d or c*b. Continued
  // fractions reverse the ordering whenever their remainders are inverted.
  bool reverse = false;
  for (;;) {
    const u64 qa = a / b, qc = c / d;
    if (qa != qc) return reverse ? qa < qc : qa > qc;
    const u64 ra = a % b, rc = c % d;
    if (!ra || !rc) {
      if (ra == rc) return false;
      return reverse ? !ra : !rc;
    }
    a = b; b = ra; c = d; d = rc; reverse = !reverse;
  }
}
u16 little16(const unsigned char *b) { return u16{b[0]} | (u16{b[1]} << 8); }
u32 little32(const unsigned char *b) {
  return u32{b[0]} | (u32{b[1]} << 8) | (u32{b[2]} << 16) | (u32{b[3]} << 24);
}
} // namespace

GrimAudibilityReport grim_compare_audio(const std::vector<s16> &clean,
                                        const std::vector<s16> &mutated,
                                        const GrimAudibilityConfig &cfg) {
  GrimAudibilityReport r;
  r.config = cfg;
  if (clean.empty() || clean.size() != mutated.size() || (clean.size() & 1u)) {
    r.error = "PCM inputs must have equal, nonzero stereo frame counts";
    return r;
  }
  if (!cfg.window_frames || cfg.window_frames > 4096 ||
      !cfg.clean_floor_rms || cfg.clean_floor_rms > 32768 ||
      cfg.residual_floor_rms > 65535 || !cfg.threshold_ratio_ppm ||
      !cfg.minimum_above_frames) {
    r.error = "invalid audibility configuration";
    return r;
  }
  r.valid = true;
  r.compared_frames = clean.size() / 2u;
  const u64 clean_floor_square = u64{cfg.clean_floor_rms} * cfg.clean_floor_rms;
  const u64 residual_floor_square = u64{cfg.residual_floor_rms} * cfg.residual_floor_rms;
  u64 longest = 0;
  std::array<u64, 2> channel_peak_residual{}, channel_peak_reference{{1, 1}};
  for (size_t first = 0; first < clean.size(); first += size_t{cfg.window_frames} * 2u) {
    const size_t count = std::min(size_t{cfg.window_frames} * 2u, clean.size() - first);
    std::array<u64, 2> ce{}, re{};
    for (size_t i = 0; i < count; ++i) {
      const s64 a = clean[first + i], b = mutated[first + i], difference = b - a;
      ce[i & 1u] += static_cast<u64>(a * a);
      re[i & 1u] += static_cast<u64>(difference * difference);
      r.clean_clipped_samples += a <= -32767 || a >= 32767;
      r.mutated_clipped_samples += b <= -32767 || b >= 32767;
    }
    const u64 clean_energy = ce[0] + ce[1], residual_energy = re[0] + re[1];
    const u64 reference_energy = std::max(clean_energy, u64{count} * clean_floor_square);
    const u64 ratio = ratio_ppm(residual_energy, reference_energy);
    const u64 frames = count / 2u;
    ++r.windows;
    r.clean_energy += clean_energy;
    r.residual_energy += residual_energy;
    if (ratio_greater(residual_energy, reference_energy,
                      r.peak_residual_energy, r.peak_reference_energy) || r.windows == 1) {
      r.peak_residual_energy = residual_energy;
      r.peak_reference_energy = reference_energy;
    }
    for (size_t channel = 0; channel < 2; ++channel) {
      const u64 reference = std::max(ce[channel], frames * clean_floor_square);
      if (ratio_greater(re[channel], reference, channel_peak_residual[channel],
                        channel_peak_reference[channel]) || r.windows == 1) {
        channel_peak_residual[channel] = re[channel];
        channel_peak_reference[channel] = reference;
      }
    }
    const bool above = ratio >= cfg.threshold_ratio_ppm &&
                       residual_energy >= u64{count} * residual_floor_square;
    if (above) {
      r.above_threshold_frames += frames;
      longest += frames;
      r.longest_above_frames = std::max(longest, r.longest_above_frames);
    } else longest = 0;
  }
  r.audible = r.above_threshold_frames >= cfg.minimum_above_frames;
  r.peak_residual_db = energy_db(r.peak_residual_energy, r.peak_reference_energy);
  for (size_t i = 0; i < 2; ++i)
    r.channel_peak_db[i] = energy_db(channel_peak_residual[i], channel_peak_reference[i]);
  r.above_threshold_ms = rounded(static_cast<double>(r.above_threshold_frames) * 1000.0 / kSampleRate);
  r.longest_above_ms = rounded(static_cast<double>(r.longest_above_frames) * 1000.0 / kSampleRate);
  r.clean_rms = rounded(std::sqrt(static_cast<double>(r.clean_energy) / clean.size()));
  r.residual_rms = rounded(std::sqrt(static_cast<double>(r.residual_energy) / clean.size()));
  return r;
}

std::string grim_audibility_json(const GrimAudibilityReport &r) {
  using json = nlohmann::ordered_json;
  json j;
  j["metric_version"] = 1;
  j["provisional"] = true;
  j["valid"] = r.valid;
  j["audible"] = r.audible;
  if (!r.error.empty()) j["error"] = r.error;
  j["sample_rate"] = kSampleRate;
  j["window_frames"] = r.config.window_frames;
  j["window_ms"] = rounded(r.config.window_frames * 1000.0 / kSampleRate);
  j["clean_floor_rms"] = r.config.clean_floor_rms;
  j["residual_floor_rms"] = r.config.residual_floor_rms;
  j["threshold_ratio_ppm"] = r.config.threshold_ratio_ppm;
  j["threshold_db"] = rounded(10.0 * std::log10(r.config.threshold_ratio_ppm / 1000000.0));
  j["minimum_above_frames"] = r.config.minimum_above_frames;
  j["minimum_above_ms"] = rounded(r.config.minimum_above_frames * 1000.0 / kSampleRate);
  j["compared_frames"] = r.compared_frames;
  j["windows"] = r.windows;
  j["peak_residual_db"] = r.peak_residual_db;
  j["channel_peak_db"] = r.channel_peak_db;
  j["above_threshold_ms"] = r.above_threshold_ms;
  j["longest_above_ms"] = r.longest_above_ms;
  j["clean_rms"] = r.clean_rms;
  j["residual_rms"] = r.residual_rms;
  j["clean_clipped_samples"] = r.clean_clipped_samples;
  j["mutated_clipped_samples"] = r.mutated_clipped_samples;
  j["clean_energy"] = r.clean_energy;
  j["residual_energy"] = r.residual_energy;
  j["peak_residual_energy"] = r.peak_residual_energy;
  j["peak_reference_energy"] = r.peak_reference_energy;
  j["above_threshold_frames"] = r.above_threshold_frames;
  j["longest_above_frames"] = r.longest_above_frames;
  return j.dump();
}

bool grim_read_wav(const std::string &path, std::vector<s16> &samples, std::string &error) {
  samples.clear();
  const auto fail = [&](const std::string &message) {
    samples.clear(); error = message; return false;
  };
  std::ifstream in(path, std::ios::binary);
  if (!in) return fail("cannot open WAV: " + path);
  in.seekg(0, std::ios::end);
  const std::streamoff physical_end = in.tellg();
  if (physical_end < 12) return fail("not a RIFF WAVE file");
  in.seekg(0, std::ios::beg);
  unsigned char header[12];
  if (!in.read(reinterpret_cast<char *>(header), sizeof(header)) ||
      std::memcmp(header, "RIFF", 4) || std::memcmp(header + 8, "WAVE", 4)) {
    return fail("not a RIFF WAVE file");
  }
  const u64 riff_end = 8ull + little32(header + 4);
  if (riff_end < 12 || riff_end > static_cast<u64>(physical_end)) {
    return fail("invalid or truncated RIFF length");
  }
  bool format_ok = false, have_data = false;
  u64 chunk_position = 12;
  while (in && chunk_position < riff_end) {
    if (riff_end - chunk_position < 8) return fail("truncated WAV chunk header");
    unsigned char chunk[8];
    if (!in.read(reinterpret_cast<char *>(chunk), sizeof(chunk))) break;
    const u32 bytes = little32(chunk + 4);
    chunk_position += 8;
    const u64 padded_bytes = u64{bytes} + (bytes & 1u);
    if (padded_bytes > riff_end - chunk_position) {
      return fail("WAV chunk extends beyond RIFF length");
    }
    chunk_position += padded_bytes;
    if (!std::memcmp(chunk, "fmt ", 4)) {
      unsigned char format[16];
      if (bytes < 16 || !in.read(reinterpret_cast<char *>(format), sizeof(format))) {
        return fail("missing or truncated WAV format chunk");
      }
      format_ok = little16(format) == 1 && little16(format + 2) == 2 &&
                  little32(format + 4) == 44100 && little32(format + 8) == 176400 &&
                  little16(format + 12) == 4 && little16(format + 14) == 16;
      if (!format_ok) return fail("WAV must be 44.1 kHz stereo 16-bit PCM");
      in.seekg(static_cast<std::streamoff>(bytes - 16), std::ios::cur);
    } else if (!std::memcmp(chunk, "data", 4)) {
      if (!bytes || (bytes & 3u) || bytes > 1024u * 1024u * 1024u) {
        return fail("invalid WAV data length");
      }
      std::vector<unsigned char> data(bytes);
      if (!in.read(reinterpret_cast<char *>(data.data()), bytes)) break;
      samples.resize(bytes / 2u);
      for (size_t i = 0; i < samples.size(); ++i) {
        const u16 value = little16(data.data() + i * 2u);
        samples[i] = static_cast<s16>(value >= 0x8000u ? static_cast<s32>(value) - 65536 : value);
      }
      have_data = true;
    } else in.seekg(static_cast<std::streamoff>(bytes), std::ios::cur);
    if (bytes & 1u) in.seekg(1, std::ios::cur);
  }
  if (!in || !format_ok || !have_data) return fail("missing or truncated WAV chunks");
  error.clear();
  return true;
}
