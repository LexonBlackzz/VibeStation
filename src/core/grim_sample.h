#pragma once
#include "grim_genome.h"
#include "grim_map.h"
#include "types.h"
#include <string>
#include <vector>

// ADPCM discovery is byte-only. Map provenance is a separate annotation pass:
// it confirms voice starts and repeat points without changing scanner rules.
constexpr u32 kGrimSampleAuto = ~0u;
constexpr u32 kGrimAdpcmMinBlocks = 8;

struct GrimSampleVoiceUse {
  u64 cycle = 0;
  u32 rom_offset = 0;
  u32 spu_address = 0;
  u8 voice = 0;
  u8 kind = 0; // GrimSpuSampleUseKind, kept numeric in sample reports
};

struct GrimAdpcmSample {
  u32 start_offset = 0, end_offset = 0; // ROM bytes, [start,end)
  u32 block_count = 0;
  std::vector<u32> loop_start_offsets;
  u32 loop_end_offset = kGrimSampleAuto;
  u32 voice_mask = 0;
  u64 first_use_cycle = kGrimNever, last_use_cycle = 0;
  u32 spu_words = 0; // map-confirmed words; zero when no map was supplied
  bool voice_start_confirmed = false;
  std::vector<GrimSampleVoiceUse> uses;
};

std::vector<GrimAdpcmSample> grim_scan_adpcm(const std::vector<u32> &words,
                                          u32 min_blocks = kGrimAdpcmMinBlocks);
struct GrimSampleScanScore {
  u32 true_positive_words = 0, false_positive_words = 0, false_negative_words = 0;
  double precision() const;
  double recall() const;
};
GrimSampleScanScore grim_sample_score(const std::vector<GrimAdpcmSample> &samples,
                                    const GrimBootMap &map);
// Resolves boundaries at KeyOnStart addresses and adds loop/voice/time records.
// The raw scanner result remains available separately for honest scoring.
std::vector<GrimAdpcmSample> grim_sample_annotate(const std::vector<u32> &words,
                                               const std::vector<GrimAdpcmSample> &scanned,
                                               const GrimBootMap &map);

enum class GrimSampleMut : u8 {
  FilterSwap, ShiftChange, LoopStartMove, LoopEndRemove, LoopEndEarly,
  BlockShuffle, BlockRepeat, BlockReverse, Transplant, NibbleNoise, Count
};
const char *grim_sample_mut_name(GrimSampleMut kind);
bool grim_sample_header_gene(GrimSampleMut kind);

struct GrimSampleContext {
  std::vector<u32> words;
  u64 bios_hash = 0;
  GrimBootMap map;
  std::vector<GrimAdpcmSample> scanner_samples; // map-independent answer
  std::vector<GrimAdpcmSample> samples; // provenance-annotated boundaries
  // Empty map_path is allowed; generation then uses unconfirmed byte candidates.
  bool load(const std::string &bios_path, const std::string &map_path, std::string &err);
  void init();
};

// count is the number of edited blocks (not resolved words); magnitude controls
// shift delta / noisy nibbles. Auto chooses a suitable sample deterministically.
// Block order/repeat/transplant change payloads only, keeping target headers.
GrimGene grim_sample_generate(const GrimSampleContext &ctx, u64 seed, GrimSampleMut kind,
                             u32 count, u32 magnitude, u32 sample_index = kGrimSampleAuto,
                             u32 donor_index = kGrimSampleAuto);
void grim_add_random_sample_genes(GrimGenome &genome, u64 seed, const GrimRandomParams &rp);
std::string grim_sample_summary(const GrimSampleContext &ctx);
std::string grim_sample_describe_gene(const GrimGene &gene, const GrimSampleContext *ctx);
