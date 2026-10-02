#pragma once
#include "grim_genome.h"
#include "types.h"
#include <deque>
#include <string>
#include <vector>

struct GrimRomContext;
struct GrimSampleContext;

// Grim Reaper 2.0, Phase 5: a "pull" is one fresh random genome, made from the
// gene families the user left switched on at the chosen intensity, then booted
// live. Nothing here evaluates or filters a pull: the outcome is unknown to
// everyone until the machine runs (DESIGN.md, "Every pull is a surprise").
//
// Families name where a gene comes from:
//   Audio     ADPCM sample genes on the BIOS sound bank (ROM)
//   Visual    reserved for the ROM visual genes of Phase 6 (generates nothing yet)
//   Code      structural MIPS mutations of BIOS code (ROM, needs the boot map)
//   Interface SPU register and GP0 filters at runtime (no map needed)
// Each gene still carries its own domain colour in the genome list.

enum GrimFamily : u32 {
  kGrimFamilyAudio = 1,
  kGrimFamilyVisual = 2,
  kGrimFamilyCode = 4,
  kGrimFamilyInterface = 8,
};

struct GrimPullSettings {
  u32 families = kGrimFamilyAudio | kGrimFamilyCode | kGrimFamilyInterface;
  u32 intensity = 50; // 0..100
  bool rot = false;   // interface genes start healthy and decay
};

// What an intensity means. Total gene count scales with it, and so does risk:
// low keeps the survival biases (late first-touch code, small magnitudes), high
// switches them off.
struct GrimPullPlan {
  u32 min_genes = 1, max_genes = 2;
  u32 risk_q10 = 0;       // 0..1024, interface magnitude pushed to the full range
  u32 rom_early_ms = 600; // code genes avoid words first run earlier than this
  u32 rom_curve = 3;      // 3 = strongly prefer late code .. 0 uniform
  u32 rom_patches_max = 2;
  bool rom_call_swap = false;
  u32 sample_ms = 100;    // sample window per gene
  const char *risk_label = "safe"; // safe | mild | risky | lethal
};
GrimPullPlan grim_pull_plan(u32 intensity);
// "4-7 genes · risky" (middle dot as UTF-8).
std::string grim_pull_readout(u32 intensity);

// The pieces a pull may draw on. Any pointer may be null: that family is then
// unavailable and generates nothing.
struct GrimPullContext {
  const GrimRomContext *rom = nullptr;
  const GrimSampleContext *sample = nullptr;
  u64 bios_hash = 0;
};
u32 grim_pull_available_families(const GrimPullContext &ctx);

// Light record of recent pulls so consecutive pulls do not feel samey. It only
// steers generation (which of a few seed-derived candidates is made); it never
// decides whether a pull is shown.
class GrimPullHistory {
public:
  static constexpr size_t kRemembered = 8;
  void note(const GrimGenome &genome);
  // How much a genome overlaps with what was pulled recently (lower = fresher).
  u32 similarity(const GrimGenome &genome) const;
  size_t size() const { return recent_.size(); }
  void clear() { recent_.clear(); }

private:
  std::deque<std::vector<u64>> recent_;
};

// Reproducible from (seed, settings, ctx, history). Never throws away a pull
// because it might die. May return an empty genome if no enabled family is
// available; `family_mask_used` (optional) says which families contributed.
GrimGenome grim_pull_generate(u64 seed, const GrimPullSettings &settings,
                              const GrimPullContext &ctx, const GrimPullHistory *history,
                              u32 *family_mask_used = nullptr);

// A genome with ROM genes only runs on the BIOS it was made for. Refuses (with a
// readable reason) before anything is booted.
bool grim_pull_compatible(const GrimGenome &genome, u64 bios_hash, std::string &err);

// Machine ID shown in the panel, "#0A7F" (from the genome hash).
std::string grim_machine_id(const GrimGenome &genome);

// One readable line per gene for the panel's genome list.
struct GrimGeneLine {
  u32 domain = kGrimFamilyInterface; // colour: Audio blue, Visual gold, Code red
  bool rom = false;                  // "ROM" or "IFACE" tag
  std::string title;
  std::string detail;
};
std::vector<GrimGeneLine> grim_pull_describe(const GrimGenome &genome,
                                             const GrimPullContext &ctx);
