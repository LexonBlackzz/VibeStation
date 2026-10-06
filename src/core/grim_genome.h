#pragma once
#include "types.h"
#include <array>
#include <string>
#include <vector>

class Bios;
struct GrimRomContext;
struct GrimSampleContext;

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
  // Phase 3: a ROM gene (one kind of structural MIPS mutation applied to the
  // BIOS image). Its resolved list of patches lives in GrimGene::patches.
  RomCode,
  // Phase 4: resolved ADPCM edits on the BIOS sound bank.
  SpuSample,
  // Phase 5.1: Faulty Hardware Simulator. Failing memory cells in main RAM, VRAM and
  // SPU RAM, applied at scanline/frame boundaries (never on the access path).
  HwRam,
  HwVram,
  HwSpuRam,
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

// One word of the BIOS image changed by a ROM gene. `original` is checked
// against the stock image before anything is written.
struct GrimRomPatch {
  u32 offset = 0; // ROM byte offset, word aligned
  u32 original = 0;
  u32 mutated = 0;
  bool delay_slot = false; // the word sits in a branch delay slot
};

// Optional generation metadata for sample windows. Absent/Blocks preserves
// Phase 4's block-count semantics and canonical v2 text exactly. Runtime ROM
// application always uses the resolved patches, never this sizing request.
enum class GrimSampleSizeKind : u8 { Blocks, Milliseconds, FractionPermille };
struct GrimSampleSizing {
  GrimSampleSizeKind kind = GrimSampleSizeKind::Blocks;
  u32 value = 0;
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
  // ROM families: resolved patches (target/trigger are unused).
  // RomCode params: kind, count, early_ms, curve.
  // SpuSample params: kind, count, magnitude, sample, donor, emulator_shift,
  // block_phase (0 or 8: block start modulo 16).
  std::vector<GrimRomPatch> patches;
  GrimSampleSizing sample_sizing;
};

struct GrimGenome {
  // 1: interface genes only. 2: also ROM genes, which pin the BIOS they were
  // made for by hash (a v1 file still parses and serializes exactly as before).
  u32 version = 1;
  u64 bios_hash = 0; // version 2 only
  std::vector<GrimGene> genes;
};
inline bool grim_genome_has_rom(const GrimGenome &g) {
  for (const GrimGene &gene : g.genes) {
    if (gene.type == GrimGeneType::RomCode || gene.type == GrimGeneType::SpuSample) {
      return true;
    }
  }
  return false;
}
inline bool grim_gene_is_rom(GrimGeneType type) {
  return type == GrimGeneType::RomCode || type == GrimGeneType::SpuSample;
}
inline bool grim_gene_is_hardware(GrimGeneType type) {
  return type == GrimGeneType::HwRam || type == GrimGeneType::HwVram ||
         type == GrimGeneType::HwSpuRam;
}

// Critical zones the simulator avoids unless a gene is explicitly marked `critical`
// (about one hardware gene in seventy): the kernel's low 64 KB of RAM (vectors,
// jump tables, kernel code and data), the top 16 KB (stacks), and the first 4 KB of
// SPU RAM (decode buffers).
constexpr u32 kGrimRamKernelBytes = 0x10000u;
constexpr u32 kGrimRamStackBytes = 0x4000u;
constexpr u32 kGrimSpuRamReservedBytes = 0x1000u;

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
  // Phase 3: with `rom` set, between rom_genes_min and rom_genes_max ROM genes
  // of up to rom_patches_max patches each are added (from a separate random
  // stream, so the interface part of a genome does not depend on this).
  const GrimRomContext *rom = nullptr;
  u32 rom_genes_min = 1;
  u32 rom_genes_max = 3;
  u32 rom_patches_max = 6;
  u32 rom_early_ms = 200; // words first executed earlier than this are never patched
  u32 rom_curve = 2;      // 0 uniform .. 3 cubic preference for late code
  bool rom_call_swap = false; // the (usually fatal) call_swap kind, off by default
  // Phase 4: byte-scanned ADPCM samples; the map is optional annotation only.
  const GrimSampleContext *sample = nullptr;
  u32 sample_genes_min = 1;
  u32 sample_genes_max = 3;
  u32 sample_count_max = 2; // requested edit span; permutations require >= 2 blocks
  GrimSampleSizing sample_sizing{GrimSampleSizeKind::Milliseconds, 100};
  s32 sample_kind = -1;    // -1 draws all ten kinds
  u32 sample_magnitude = 1;
  s32 sample_index = -1;   // -1 draws any scanned sample
  s32 sample_donor = -1;   // transplant donor; -1 draws a different sample
  // Phase 5 (live pulls). risk_q10 (0..1024) pushes interface-gene magnitudes toward
  // the full parameter range; 0 keeps the survivable bias and draws exactly as
  // before. rot_only turns every interface gene into a rot ramp (starts healthy,
  // decays) without drawing extra random numbers.
  u32 risk_q10 = 0;
  bool rot_only = false;
  // Phase 5.1: hardware genes (0 max = none, and no random numbers are drawn, so every
  // earlier seed is unchanged). `hw_critical_permille` is the chance that a gene may
  // fail kernel/stack memory; the default is deliberately tiny.
  u32 hw_genes_min = 0;
  u32 hw_genes_max = 0;
  u32 hw_critical_permille = 10;
  // Survival bias for RAM faults: with this chance (permille) a non-critical RAM gene is
  // placed clear of `hw_avoid_ram` (byte ranges that ran code in the clean boot, from the
  // boot map), since one stuck bit in running code is enough to kill. Null = no bias and
  // no extra random numbers.
  const std::vector<std::pair<u32, u32>> *hw_avoid_ram = nullptr;
  u32 hw_avoid_permille = 0;
};
// Reproducible from (seed, params). Biased toward survivable settings: small
// magnitudes, partial targets, ramps and windows more often than "always".
GrimGenome grim_random_genome(u64 seed, const GrimRandomParams &params);

// ---- Faulty Hardware Simulator ------------------------------------------------------

// What the simulator may touch. System implements it (RAM writes also tell the CPU's
// translation cache); tests implement it with plain arrays.
struct GrimHwTarget {
  virtual ~GrimHwTarget() = default;
  // Read-only views of the three memories (2 MiB, 1024*512 words, 512 KiB).
  virtual const u8 *ram_view() const = 0;
  virtual const u16 *vram_view() const = 0;
  virtual const u8 *spu_view() const = 0;
  // Writes go through the owner so it can tell the CPU's translation cache.
  virtual void ram_put(u32 offset, u8 value) = 0;
  virtual void vram_put(u32 index, u16 value) = 0;
  virtual void spu_put(u32 offset, u8 value) = 0;
};

// Reproducible from (seed, params). Appends between hw_genes_min and hw_genes_max hardware
// genes. Mostly small groups of bad cells away from critical memory; high risk grows the
// cell counts, rates and regions.
void grim_add_random_hw_genes(GrimGenome &genome, u64 seed, const GrimRandomParams &params);
// True when the gene's region overlaps memory the simulator normally avoids.
bool grim_hw_gene_is_critical(const GrimGene &gene);

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
  // ROM genes. apply_rom() checks the BIOS hash and every original word first
  // and writes nothing on failure (err says why). It expects the stock image
  // (System::reset() restores it and calls this).
  bool has_rom_genes() const { return grim_genome_has_rom(genome_); }
  bool apply_rom(Bios &bios, std::string &err);
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

  // Faulty Hardware Simulator. System calls this at every frame start (frame_tick) and at
  // every eighth scanline. Stuck cells are re-forced on both ticks; flips and bursts happen
  // on the frame tick only. Cheap when there are no hardware genes: has_hardware() is false.
  bool has_hardware() const { return !hw_genes_.empty(); }
  // bus_load_q10 (0..1024) is how busy the bus was last frame; genes with `load` > 0 fail more when it is high.
  void apply_hardware(GrimHwTarget &target, bool frame_tick, u32 bus_load_q10);

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
  struct HwCell {
    u32 where; // RAM/SPU byte offset, or VRAM word index
    u16 mask;  // bits that fail
  };
  std::vector<size_t> hw_genes_;                // indices into genome_.genes
  std::vector<std::vector<HwCell>> hw_cells_;   // per hardware gene, in activation order
  struct HwRows {                               // decay / hammer bookkeeping, 1 KiB rows
    std::vector<u64> hash;
    std::vector<u16> stable;  // frames the row has been unchanged
    std::vector<u16> hot;     // consecutive frames the row changed
    bool seeded = false;
  };
  std::vector<HwRows> hw_rows_;
  void build_hw_cells();
  std::vector<Delayed> delayed_; // sorted by due, stable
  std::array<u16, 0x200> shadow_{}; // last value passed to each SPU register
};

// ---- GUI playback ---------------------------------------------------------------

// `--genome <file>` on the GUI command line: main() loads the file once, and
// App applies the runtime to the System it creates (both CPU modes).
bool grim_gui_genome_load(const std::string &path, std::string &err);
GrimGenomeRuntime *grim_gui_genome(); // nullptr when no genome was loaded
