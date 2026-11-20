#pragma once
#include "grim_genome.h" // GrimRng, grim_mix64
#include "types.h"
#include <cstddef>
#include <vector>

// Grim Reaper FMV corruption: full-motion video corrupted inside the MDEC as it
// decodes, so a movie on a clean disc comes out broken. The classic FMV Reaper and
// the mdec_fmv gene share this engine; only where their settings come from differs.
//
// Nothing here changes how much data the MDEC consumes or produces (run lengths,
// end-of-block markers and output sizes are never touched), so a game waiting on its
// movie never stalls: the picture breaks, the movie keeps playing.

// What may be hit (bit mask).
enum GrimFmvTarget : u32 {
  kGrimFmvCoeffs = 1, // DCT coefficient levels: smeared, blotchy, ringing blocks
  kGrimFmvQuant = 2,  // quantisation tables as the game uploads them: the whole movie turns
  kGrimFmvBlocks = 4, // whole 8x8 blocks swapped, copied, flattened or inverted
  kGrimFmvPixels = 8, // decoded pixels on their way out: bit rot and colour noise
  kGrimFmvAudio = 16, // the movie's XA audio as it decodes: stutters, skips, crushes, blasts
  kGrimFmvAll = 31,
};

// GrimFmvKnobs and GrimFmvMacroblock live in grim_genome.h (the genome runtime keeps them).

// Decides whether the macroblock about to be decoded is hit (draws one number).
void grim_fmv_begin(const GrimFmvKnobs &k, GrimRng &rng, GrimFmvMacroblock &mb);
// One RLE halfword. The run (bits 10-15) and the 0xFE00 end/padding code are kept.
u16 grim_fmv_coefficient(const GrimFmvKnobs &k, GrimFmvMacroblock &mb, u16 halfword);
// A freshly uploaded quantisation table, in place. Returns entries changed.
size_t grim_fmv_quant(const GrimFmvKnobs &k, GrimRng &rng, u8 *table, size_t n);
// The macroblock's decoded (spatial) 8x8 blocks, `count` of 64 ints each, in MDEC order
// (colour: Cr, Cb, Y1..Y4; monochrome: Y). Returns true if anything changed.
bool grim_fmv_blocks(const GrimFmvKnobs &k, GrimFmvMacroblock &mb, int *blocks, size_t count);
// One output word of the macroblock.
u32 grim_fmv_output(const GrimFmvKnobs &k, GrimFmvMacroblock &mb, u32 word);

// The last clean XA sector, for skips that jump back into it.
struct GrimFmvAudioMemory {
  std::vector<s16> last;
};
// One decoded XA sector, interleaved stereo, in place (the count never changes). Each
// sector is hit with chance `rate`; draws one number when not hit. Returns true if hit.
bool grim_fmv_audio(const GrimFmvKnobs &k, GrimRng &rng, s16 *lr, size_t frames,
                    GrimFmvAudioMemory &memory);
// While no macroblock has been decoded for this many frames, no movie is playing and
// XA audio is left alone (the Disc Reaper covers game music and voices).
constexpr u32 kGrimFmvAudioIdleFrames = 30;

// The MDEC's view of whoever corrupts it (System, combining the FMV Reaper and the
// genome). The MDEC calls these only while it holds a hook.
class GrimFmvHook {
public:
  virtual ~GrimFmvHook() = default;
  virtual void fmv_begin_macroblock() = 0;
  virtual u16 fmv_coefficient(u16 halfword) = 0;
  virtual void fmv_quant_table(u8 *table, size_t n) = 0;
  virtual void fmv_blocks(int *blocks, size_t count) = 0;
  virtual u32 fmv_output_word(u32 word) = 0;
};

// The classic FMV Reaper's settings.
struct GrimFmvReaperConfig {
  bool enabled = false;
  u32 targets = kGrimFmvCoeffs | kGrimFmvBlocks | kGrimFmvAudio;
  float percent = 10.0f;  // share of macroblocks hit
  u32 strength = 384;     // 0..1024
  u64 seed = 1;
};
