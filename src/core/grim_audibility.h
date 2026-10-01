#pragma once
#include "types.h"
#include <array>
#include <string>
#include <vector>

// Observer-only PCM comparison. Integer energy ratios and sample counts decide
// the verdict; logarithms are used only to display the result. The defaults are
// provisional, calibrated against four listening verdicts from Lexon.
struct GrimAudibilityConfig {
  u32 window_frames = 1411;       // stereo frames: approximately 32 ms at 44.1 kHz
  u32 clean_floor_rms = 64;       // s16 units; quiet reference windows have this floor
  u32 residual_floor_rms = 8;     // avoid treating tiny PCM differences as audible
  u32 threshold_ratio_ppm = 10000; // residual energy / reference energy: -20 dB
  u32 minimum_above_frames = 4410; // cumulative exposure: 100 ms at 44.1 kHz
};

struct GrimAudibilityReport {
  bool valid = false;
  bool audible = false;
  std::string error;
  GrimAudibilityConfig config;
  u64 compared_frames = 0;
  u64 windows = 0;
  u64 above_threshold_frames = 0;
  u64 longest_above_frames = 0;
  u64 clean_energy = 0;
  u64 residual_energy = 0;
  u64 peak_residual_energy = 0;
  u64 peak_reference_energy = 1;
  u64 clean_clipped_samples = 0;
  u64 mutated_clipped_samples = 0;
  double peak_residual_db = -120.0; // zero residual is reported as -120 dB
  std::array<double, 2> channel_peak_db{{-120.0, -120.0}};
  double above_threshold_ms = 0.0;
  double longest_above_ms = 0.0;
  double clean_rms = 0.0;
  double residual_rms = 0.0;
};

// Both inputs must be interleaved stereo s16 PCM over the same frames, beginning
// at the same emulated instant. No alignment/time-shift search is performed.
GrimAudibilityReport grim_compare_audio(const std::vector<s16> &clean,
                                        const std::vector<s16> &mutated,
                                        const GrimAudibilityConfig &cfg = {});
std::string grim_audibility_json(const GrimAudibilityReport &report);
// Review helper for CLI comparisons and external listening fixtures. Only
// uncompressed 44.1 kHz, stereo, 16-bit little-endian WAV is accepted.
bool grim_read_wav(const std::string &path, std::vector<s16> &stereo_samples,
                   std::string &error);
