#pragma once
#include "grim_genome.h"
#include "grim_map.h"
#include "types.h"
#include <string>
#include <utility>
#include <vector>

struct GrimSampleContext;

// Grim Reaper 2.0, Phase 3: ROM code genes.
//
// Structural MIPS (R3000A) mutations on the words the boot map labels `code`
// (DESIGN.md 2.1). Every mutation keeps the word a valid instruction. A gene
// stores the resolved (offset, original, mutated) patches next to the
// parameters that produced them; applying it verifies `original` first.

// ---- MIPS helpers ------------------------------------------------------------

// True for every R3000A instruction encoding the CPU executes (not reserved).
bool grim_mips_valid(u32 word);
// Branches, jumps and calls: the next word is a delay slot.
bool grim_mips_has_delay_slot(u32 word);
// General register the instruction writes, or -1.
int grim_mips_dest_reg(u32 word);
// Minimal disassembly, e.g. "beq a0,zero,+12". `addr` resolves jump targets.
std::string grim_mips_disasm(u32 word, u32 addr);

// ---- mutations ---------------------------------------------------------------------

enum class GrimRomMut : u8 {
  ImmPerturb,   // small +-, sign flip, or one flipped bit in the immediate
  RegSubst,     // rs / rt / rd -> another register (never $sp $ra $k0 $k1, $gp if used)
  BranchInvert, // BEQ<->BNE, BLEZ<->BGTZ, BLTZ<->BGEZ
  BranchNudge,  // branch offset +-1..4 words, target must be executed code
  AluSubst,     // ADDU/SUBU/XOR/OR/AND, SLL/SRL/SRA, SLLV/SRLV/SRAV
  LsWidth,      // LW/LH/LHU/LB/LBU, SW/SH/SB, offset alignment kept valid
  Nop,          // selective NOP
  LuiOriConst,  // LUI + ORI/ADDIU pair: mutate the constant, re-encode both
  CallSwap,     // JAL -> another traced call target (off by default)
  Count
};
const char *grim_rom_mut_name(GrimRomMut kind);

// The stock image and its map: everything a ROM gene needs to be generated.
struct GrimRomContext {
  std::vector<u32> words; // stock BIOS image, little-endian words
  u64 bios_hash = 0;
  GrimBootMap map;
  bool avoid_gp = false;          // some code uses $gp
  std::vector<u32> jal_targets;   // distinct JAL targets among code words, sorted

  // Loads the BIOS (through Bios, so the hash matches what the runtime checks)
  // and the map, and refuses a map made for another BIOS.
  bool load(const std::string &bios_path, const std::string &map_path, std::string &err);
  // Derives avoid_gp and jal_targets from words + map. Call after filling them.
  void init();
  bool is_code(u32 index) const {
    return index < map.words.size() && map.words[index].cls == GrimWordClass::Code;
  }
  bool in_delay_slot(u32 index) const {
    return index > 0 && index < words.size() && grim_mips_has_delay_slot(words[index - 1]);
  }
};

struct GrimRomEdit {
  u32 index; // word index (byte offset / 4)
  u32 word;
};
bool grim_rom_applicable(GrimRomMut kind, const GrimRomContext &ctx, u32 index);
// One mutation of the word at `index` (LuiOriConst may edit two words). Returns
// false, writing nothing, when this kind cannot be applied to this word.
bool grim_rom_mutate(GrimRomMut kind, GrimRng &rng, const GrimRomContext &ctx, u32 index,
                     std::vector<GrimRomEdit> &out);

// Picks `count` distinct code words that first ran no earlier than early_ms,
// weighted toward later first execution (curve 0 uniform, 1 linear, 2
// quadratic, 3 cubic), mutates them and returns the gene. Reproducible from
// (ctx, seed, kind, count, early_ms, curve). It may hold fewer patches than
// `count`, or none, if the map offers nothing suitable.
GrimGene grim_rom_generate(const GrimRomContext &ctx, u64 seed, GrimRomMut kind, u32 count,
                           u32 early_ms, u32 curve);

// Called by grim_random_genome(): appends ROM genes and pins the BIOS hash.
void grim_add_random_rom_genes(GrimGenome &genome, u64 seed, const GrimRandomParams &rp);

// Readable listing of a genome. With a context, ROM genes show their
// disassembly, region and first-execution time.
std::string grim_describe_genome(const GrimGenome &genome, const GrimRomContext *ctx,
                                 const GrimSampleContext *samples = nullptr);

// Milliseconds of emulated time (33.8688 MHz CPU clock).
inline double grim_cycles_to_ms(u64 cycles) {
  return static_cast<double>(cycles) * 1000.0 / static_cast<double>(psx::CPU_CLOCK_HZ);
}
