#pragma once
#include "grim_genome.h"
#include "grim_map.h"
#include "types.h"
#include <array>
#include <string>
#include <utility>
#include <vector>

// Grim Reaper 2.0, Phase 1: headless evaluation instrument.
//
// GrimTelemetry records what the emulated machine did each frame (coverage,
// exceptions, GPU/DMA/SPU work, frame/audio/CPU hashes). The liveness gates
// turn that telemetry into "alive" or a death reason. run_grim_eval() runs a
// BIOS headless (interpreter, no window, no audio device, no host pacing).
// Nothing here changes emulation; it only observes. See
// docs/grim-reaper/PROGRESS.md.

class System;

struct GrimFrameTelemetry {
  u32 frame = 0;
  u64 cycles = 0;       // CPU cycles executed this frame
  u64 instructions = 0; // instructions fetched this frame (interpreter)
  u32 new_pcs = 0;      // instruction words executed for the first time
  u32 coverage = 0;     // distinct instruction words executed so far
  std::array<u32, 16> exceptions{}; // by COP0 cause code (0 = interrupt)
  u32 exception_repeats = 0; // non-IRQ exceptions with the same cause+EPC
                             // as the previous non-IRQ exception
  u32 gp0_polygon = 0, gp0_line = 0, gp0_rect = 0, gp0_fill = 0,
      gp0_transfer = 0, gp0_other = 0;
  u32 gp1 = 0;
  std::array<u32, 7> dma_transfers{}; // completed transfers per channel
  std::array<u32, 7> dma_words{};     // words moved per channel
  u32 spu_key_on = 0;
  u32 spu_voice_mask = 0; // voices with a non-Off envelope at frame end
  u32 fb_hash = 0;        // hash of the displayed area (Gpu::build_display_rgba)
  bool display_enabled = false;
  u32 fb_lit = 0;         // non-black displayed pixels
  u32 audio_samples = 0;  // stereo frames produced this frame
  u64 audio_hash = 0;
  double audio_rms = 0.0; // over both channels, s16 units
  double audio_zcr = 0.0; // zero crossings per stereo frame (mid channel)
  u64 cpu_hash = 0;       // GPRs, PC, HI/LO, SR, Cause, EPC, BadVAddr
};

class GrimTelemetry {
public:
  GrimTelemetry();

  // Called by Cpu::run_slice() for every executed instruction while attached.
  void note_exec(u32 pc) {
    ++instructions_;
    const u32 phys = pc & 0x1FFFFFFFu;
    u32 word;
    if (phys < 0x00800000u) {
      word = (phys & (psx::RAM_SIZE - 1u)) >> 2; // RAM and its mirrors
    } else if (phys - psx::BIOS_BASE < psx::BIOS_SIZE) {
      word = kRamWords + ((phys - psx::BIOS_BASE) >> 2);
    } else if (phys - psx::SCRATCHPAD_BASE < 0x1000u) {
      word = kRamWords + kBiosWords + ((phys & (psx::SCRATCHPAD_SIZE - 1u)) >> 2);
    } else {
      word = kOtherWord; // anything else counts as one location
    }
    u64 &bits = exec_bits_[word >> 6];
    const u64 mask = 1ull << (word & 63u);
    if ((bits & mask) == 0) {
      bits |= mask;
      ++new_pcs_;
    }
  }
  // Called by Cpu::exception() with the COP0 cause code and the EPC it set.
  void note_exception(u32 cause, u32 epc);
  u32 interrupts_this_frame() const { return exc_[0]; }

  // Collects the frame that System::run_frame() just finished. `audio` holds
  // the interleaved stereo samples the SPU produced during that frame.
  GrimFrameTelemetry end_frame(System &sys, const std::vector<s16> &audio,
                               bool mute_audio);

  u64 total_cycles() const { return total_cycles_; }
  u64 total_instructions() const { return total_instructions_; }
  u32 coverage() const { return coverage_; }
  const std::array<u64, 16> &total_exceptions() const { return total_exc_; }
  u64 total_exception_repeats() const { return total_exc_repeats_; }

private:
  static constexpr u32 kRamWords = psx::RAM_SIZE / 4u;
  static constexpr u32 kBiosWords = psx::BIOS_SIZE / 4u;
  static constexpr u32 kScratchWords = psx::SCRATCHPAD_SIZE / 4u;
  static constexpr u32 kOtherWord = kRamWords + kBiosWords + kScratchWords;
  std::vector<u64> exec_bits_;

  u64 instructions_ = 0;
  u32 new_pcs_ = 0;
  std::array<u32, 16> exc_{};
  u32 exc_repeats_ = 0;
  u32 last_exc_cause_ = 0xFFFFFFFFu;
  u32 last_exc_epc_ = 0;

  u32 frame_ = 0;
  u32 coverage_ = 0;
  u64 last_cycle_ = 0;
  u64 total_cycles_ = 0;
  u64 total_instructions_ = 0;
  std::array<u64, 16> total_exc_{};
  u64 total_exc_repeats_ = 0;
  std::array<u64, 7> last_dma_transfers_{};
  std::array<u64, 7> last_dma_words_{};
  u64 last_key_on_ = 0;
};

// Liveness gate thresholds (DESIGN.md 3.3). Deliberately conservative: a
// machine is only declared dead when nothing at all has changed for seconds.
// Frame counts are emulated frames (~50/60 per second).
struct GrimLivenessConfig {
  // coverage_stall: this many consecutive frames with no new code, no change
  // in the displayed image, no audio movement and no GPU/DMA/SPU work.
  // Polling loops alone do not trip it, as long as anything else moves.
  u32 coverage_stall_frames = 300;
  // frozen_frame: this many consecutive frames with an unchanged displayed
  // image, no audio movement and no GP0 commands, even if the CPU is still
  // running (possibly new "code", e.g. a runaway CPU executing data). A
  // machine that keeps redrawing the same picture is NOT frozen: the stock
  // Shell does exactly that while it waits for input.
  u32 frozen_frame_frames = 600;
  // exception_loop: this many consecutive frames that each take at least
  // exception_loop_min_per_frame non-interrupt exceptions, of which at least
  // exception_loop_repeat_ratio repeat the previous cause+EPC, with no new
  // code executed.
  u32 exception_loop_frames = 60;
  u32 exception_loop_min_per_frame = 8;
  double exception_loop_repeat_ratio = 0.9;
  // Audio is judged over the whole run. An audio frame "moves" when its RMS
  // is at least audio_silence_rms (s16 units), it crosses zero (so it is not
  // DC), and its RMS or zero-crossing rate differs from the previous frame by
  // more than audio_change_ratio (so it is not one fixed tone). A run with
  // fewer than audio_min_moving_frames such frames is "silent". Silence alone
  // is not death: a silent machine that ever showed an image (at least
  // image_min_lit_pixels non-black pixels) stays alive with silent=true; one
  // that never did is dead_audio. Runs shorter than audio_min_run_frames are
  // not judged on audio.
  double audio_silence_rms = 16.0;
  double audio_change_ratio = 0.05;
  u32 audio_min_moving_frames = 10;
  u32 audio_min_run_frames = 300;
  u32 image_min_lit_pixels = 64;
};

struct GrimLiveness {
  bool alive = true;
  std::string reason = "alive"; // alive, coverage_stall, exception_loop,
                                // frozen_frame, dead_audio
  s64 death_frame = -1;         // frame at which the gate tripped
  bool silent = false;          // no moving audio over the whole run
};

// Streaming form of the gates, so a run can stop as soon as it is dead.
class GrimLivenessTracker {
public:
  explicit GrimLivenessTracker(const GrimLivenessConfig &cfg) : cfg_(cfg) {}
  // Returns true once the machine is dead (the verdict is then fixed).
  bool update(const GrimFrameTelemetry &f);
  // Final verdict, including the whole-run audio gate.
  GrimLiveness finish() const;

private:
  GrimLivenessConfig cfg_;
  GrimLiveness verdict_;
  u32 frames_ = 0;
  u32 stall_run_ = 0;
  u32 frozen_run_ = 0;
  u32 exc_loop_run_ = 0;
  u32 audio_moving_frames_ = 0;
  bool drew_image_ = false;
  bool have_prev_ = false;
  u32 prev_fb_ = 0;
  double prev_rms_ = 0.0;
  double prev_zcr_ = 0.0;
};

GrimLiveness grim_evaluate_liveness(const std::vector<GrimFrameTelemetry> &frames,
                                    const GrimLivenessConfig &cfg);

struct GrimEvalConfig {
  std::string bios_path;
  // (ROM byte offset, word) pairs written into the BIOS image after reset.
  std::vector<std::pair<u32, u32>> bios_patches;
  u32 frames = 600;
  u64 max_cycles = 0;       // 0 = no cap
  u64 max_instructions = 0; // 0 = no cap
  double watchdog_seconds = 120.0; // host wall clock, checked between frames
  bool stop_on_death = true;
  bool mute_audio = false; // test harness only: feed silence to the recorder
  GrimLivenessConfig liveness;

  // Phase 2. With use_genome, the genome is applied from reset (System owns
  // nothing; run_grim_eval keeps the runtime alive for the run).
  bool use_genome = false;
  GrimGenome genome;
  // Review outputs. Empty / 0 = off.
  std::string dump_wav_path;    // captured SPU audio, 16-bit stereo at 44.1 kHz
  std::string dump_frames_dir;  // displayed area as BMP, every dump_frames_every frames
  u32 dump_frames_every = 0;
  // Run under whatever CPU mode is active (the recompiler included) without
  // the executed-PC telemetry hooks, which only see the interpreter. Frame,
  // audio, GPU and cycle telemetry stay valid; instruction counts and coverage
  // read zero. Used to check that genes behave the same under both backends.
  bool native_cpu = false;
  // Test only, never set by a normal command line: spin forever after boot so
  // --grim-gene-test can prove that --grim-explore kills a hung child.
  bool test_hang = false;
  // Phase 3 discovery run: fill *boot_map_out (and *copy_stats_out) from the
  // clean boot. Observation only; the telemetry hash is identical without it.
  GrimBootMap *boot_map_out = nullptr;
  GrimCopyStats *copy_stats_out = nullptr;
  std::string map_scenario = "nodisc";
};

struct GrimEvalResult {
  // frames | death | cycle_cap | instruction_cap | watchdog |
  // bios_load_failed | unsupported_cpu_mode
  std::string end_reason = "frames";
  std::vector<GrimFrameTelemetry> frames;
  GrimLiveness liveness;
  u64 cycles = 0;
  u64 instructions = 0;
  u32 coverage = 0;
  std::array<u64, 16> exceptions{};
  u64 exception_repeats = 0;
  u64 run_hash = 0; // hash over every frame's hashes and counters
  double wall_seconds = 0.0;
  double emulated_seconds = 0.0;
  double speed_factor = 0.0; // emulated / wall
  u64 genome_hash = 0;       // 0 when no genome was applied
  std::vector<u64> gene_hits; // per gene: events it actually changed
  std::string error_detail;   // set with end_reason rom_gene_mismatch
};

// Runs one evaluation on a fresh System. Requires the interpreter
// (effective_cpu_execution_mode() == Interpreter); the telemetry hooks do not
// see recompiled code.
GrimEvalResult run_grim_eval(const GrimEvalConfig &cfg);

// One JSON object per line with a fixed field order.
std::string grim_frame_json(const GrimFrameTelemetry &f);
std::string grim_summary_json(const GrimEvalResult &r);
bool grim_write_telemetry_jsonl(const GrimEvalResult &r, const std::string &path);

// Review helpers (no dependencies beyond the standard library).
bool grim_write_wav(const std::string &path, const std::vector<s16> &stereo_samples);
bool grim_write_bmp(const std::string &path, int width, int height,
                    const std::vector<u32> &rgba);
