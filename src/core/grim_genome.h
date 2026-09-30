#pragma once
#include "types.h"
#include <array>
#include <string>
#include <vector>

// Grim Reaper 2.0, Phase 2: interface genes.
//
// A genome is an ordered list of genes. Each gene corrupts what passes between
// the CPU and the SPU (register writes) or the GPU (whole GP0 commands), is
// triggered by emulated time only, and draws its randomness from its own
// splitmix64 stream. Everything here is integer arithmetic on purpose: the
// same genome must give the same result on MSVC, GCC and clang.
//
// System and Gpu hold a `GrimGenomeRuntime *` (nullptr = off). See
// docs/grim-reaper/PROGRESS.md for the file format and the gene reference.

// ---- randomness ------------------------------------------------------------

// splitmix64 (Vigna). Known outputs for seed 0 are tested in --grim-gene-test.
struct GrimRng {
  u64 state = 0;
  u64 next() {
    state += 0x9E3779B97F4A7C15ull;
    u64 z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  // Inclusive range. Modulo bias is irrelevant here; reproducibility is not.
  u32 range(u32 lo, u32 hi) {
    return hi <= lo ? lo : lo + static_cast<u32>(next() % (u64{hi} - lo + 1u));
  }
  s32 srange(s32 lo, s32 hi) {
    return hi <= lo ? lo
                    : lo + static_cast<s32>(next() % (static_cast<u64>(hi) -
                                                      static_cast<u64>(lo) + 1u));
  }
  // 0..1023 (q10 fraction).
  u32 unit_q10() { return static_cast<u32>(next() >> 54); }
};

// Stateless mix of a value (the splitmix64 finalizer), for per-bit / per-vertex
// decisions derived from one event draw without advancing the stream.
inline u64 grim_mix64(u64 x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

// ---- genome ---------------------------------------------------------------

enum class GrimGeneType : u8 {
  SpuPitch,
  SpuAdsr,
  SpuVolume,
  SpuAddress,
  SpuKeyOn,
  SpuNoise,
  SpuPmon,
  SpuReverb,
  GpuVertex,
  GpuColor,
  GpuFlags,
  GpuTexParam,
  GpuState,
  GpuFill,
  Count
};

enum class GrimTriggerKind : u8 { Always, Window, Rot, Intermittent };

// Trigger, evaluated on the emulated frame counter (System::boot_diag frame
// counter: it counts run_frame() calls since reset and never reads host time).
//   always:       start/end ignored
//   window:       active for start_frame <= frame < end_frame
//   rot:          magnitude ramps 0 -> 1 over [start_frame, end_frame], then holds
//   intermittent: gated to [start_frame, end_frame) (end_frame 0 = forever), and
//                 either a pattern (period > 0: on for the first `duty` frames of
//                 every `period`) or, with period 0, a per-event probability
//                 in permille drawn from the gene's RNG.
struct GrimTrigger {
  GrimTriggerKind kind = GrimTriggerKind::Always;
  u32 start_frame = 0;
  u32 end_frame = 0;
  u32 period = 0;
  u32 duty = 0;
  u32 probability = 0; // permille
};

constexpr size_t kGrimMaxParams = 8;

struct GrimParamSpec {
  const char *name;
  s32 lo, hi, def;
};

struct GrimGene {
  GrimGeneType type = GrimGeneType::SpuPitch;
  // SPU genes: voice mask (bits 0-23; bit 24 also selects the main volume
  // registers for spu_volume). GPU genes: bit 0 polygons, bit 1 lines and
  // polylines, bit 2 rectangles (gpu_state: bit n = E(n+1) command;
  // gpu_fill ignores it).
  u32 target = 0;
  u64 seed = 0;
  GrimTrigger trigger;
  std::array<s32, kGrimMaxParams> params{}; // in schema order
};

struct GrimGenome {
  u32 version = 1;
  std::vector<GrimGene> genes;
};

const char *grim_gene_type_name(GrimGeneType t);
const std::vector<GrimParamSpec> &grim_gene_schema(GrimGeneType t);
u32 grim_gene_default_target(GrimGeneType t);
GrimGene grim_default_gene(GrimGeneType t); // schema defaults, always trigger

// Strict: unknown gene types, unknown or missing fields, out-of-range values
// and wrong JSON types are errors (returns false with a message in `err`).
bool grim_genome_parse(const std::string &json, GrimGenome &out, std::string &err);
bool grim_genome_load(const std::string &path, GrimGenome &out, std::string &err);
// Canonical text: fixed field order, no whitespace beyond one line per gene.
std::string grim_genome_serialize(const GrimGenome &g);
// FNV-1a over the canonical text.
u64 grim_genome_hash(const GrimGenome &g);

struct GrimRandomParams {
  u32 min_genes = 1;
  u32 max_genes = 4;
  u32 horizon_frames = 1800; // window/rot frames are drawn from [0, horizon]
  bool spu = true;
  bool gpu = true;
};
// Reproducible from (seed, params). Biased toward survivable settings: small
// magnitudes, partial targets, ramps and windows more often than "always".
GrimGenome grim_random_genome(u64 seed, const GrimRandomParams &params);

// ---- runtime ----------------------------------------------------------------

struct GrimSpuWrite {
  u32 offset; // from 0x1F801C00
  u16 value;
};

// Magnitude of a trigger at `frame`: 0 (inactive) .. 1024 (full). `rng_word`
// is the event's draw, used only by probability triggers.
u32 grim_trigger_magnitude(const GrimTrigger &t, u32 frame, u64 rng_word);

class GrimGenomeRuntime {
public:
  static constexpr size_t kMaxSpuOut = 8;

  explicit GrimGenomeRuntime(GrimGenome genome);
  const GrimGenome &genome() const { return genome_; }
  u64 hash() const { return hash_; }

  // Rewinds every gene stream, the register shadow and the delay queue.
  // System::reset() calls it, so every boot replays the genome identically.
  void reset();
  void begin_frame(u32 frame) { frame_ = frame; }
  u32 frame() const { return frame_; }

  // ---- SPU ----
  // Filters one 16-bit register write (32-bit writes are split before this).
  // Writes `out[0..n)` in the order they must reach the SPU; n == 0 drops it.
  size_t filter_spu_write(u32 offset, u16 value, u64 now_cycle, GrimSpuWrite *out);
  bool has_delayed_spu() const { return !delayed_.empty(); }
  u64 next_delayed_due() const { return delayed_.front().due; }
  GrimSpuWrite pop_delayed_spu() {
    const GrimSpuWrite w{delayed_.front().offset, delayed_.front().value};
    delayed_.erase(delayed_.begin());
    return w;
  }

  // ---- GPU ----
  // Transforms a whole buffered GP0 command in place. Never changes the word
  // count, and never touches the opcode bits that decide it.
  void filter_gp0(u32 *words, size_t count);
  // Tail words of a polyline. `color_word` is true for the color word of a
  // gouraud polyline. Terminators pass through unchanged.
  u32 filter_gp0_polyline_word(u32 word, bool gouraud, bool color_word);

  // Number of events each gene actually changed (same order as the genome).
  const std::vector<u64> &hits() const { return hits_; }

private:
  struct Delayed {
    u64 due;
    u32 offset;
    u16 value;
  };
  struct WriteList {
    std::array<GrimSpuWrite, kMaxSpuOut> w{};
    size_t n = 0;
    void push(u32 offset, u16 value) {
      if (n < w.size()) {
        w[n++] = GrimSpuWrite{offset, value};
      }
    }
  };
  void queue_delayed(u64 due, u32 offset, u16 value);
  void apply_spu_gene(size_t gi, const GrimSpuWrite &in, WriteList &out, u64 now);
  void apply_gpu_gene(size_t gi, u32 *words, size_t count);
  void apply_gpu_word_gene(size_t gi, u32 &word, bool color_word);

  GrimGenome genome_;
  u64 hash_ = 0;
  u32 frame_ = 0;
  std::vector<GrimRng> rng_;
  std::vector<u64> hits_;
  std::vector<size_t> spu_genes_, gpu_genes_; // indices into genome_.genes
  std::vector<Delayed> delayed_; // sorted by due, stable
  std::array<u16, 0x200> shadow_{}; // last value passed to each SPU register
};

// ---- GUI playback ---------------------------------------------------------------

// `--genome <file>` on the GUI command line: main() loads the file once, and
// App applies the runtime to the System it creates (both CPU modes).
bool grim_gui_genome_load(const std::string &path, std::string &err);
GrimGenomeRuntime *grim_gui_genome(); // nullptr when no genome was loaded
