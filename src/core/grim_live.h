#pragma once
#include "grim_eval.h"
#include "grim_genome.h"
#include "types.h"
#include <mutex>
#include <string>
#include <vector>

// Grim Reaper 2.0, Phase 5: the live death watch.
//
// The liveness gates of Phase 1 run on the machine the user is actually playing.
// The watch reads what the emulator already counts (GPU/DMA/SPU work, the
// displayed image, the exceptions the CPU takes) once per frame from the
// emulator thread. It adds no per-instruction hook: the telemetry it attaches
// is the "light" kind (GrimTelemetry::set_tracks_execution(false)), so the
// interpreter keeps its normal loop and the recompiler never sees it.

class System;
class Bios;

// What the panel shows for a dead machine. Plain words first, technical detail
// underneath (DESIGN.md "Death watch").
struct GrimDeath {
  std::string reason;   // gate id: exception_loop, coverage_stall, frozen_frame, dead_audio
  std::string headline; // "Stuck in an exception loop"
  std::string detail;   // "Reserved-instruction exception at BFC0 2B68, repeating ..."
  u32 frame = 0;        // frames since the watch attached
  double seconds = 0.0; // emulated seconds since the watch attached
  u32 pc = 0;
  s32 culprit_gene = -1; // a ROM gene that patched the faulting word, if known
};

struct GrimLiveStatus {
  bool watching = false;
  bool dead = false;
  bool silent = false;   // alive, but no moving audio so far (informational)
  u32 frames = 0;
  double seconds = 0.0;
  GrimDeath death;
};

// Death comes fast on a live machine: stock BIOS boots never idle more than 72
// frames in a row (measured, docs/grim-reaper/PROGRESS.md), so these are well
// above that and still a second or two after the machine stops. A running game
// gets double (a static loading screen with silence is normal there).
GrimLivenessConfig grim_live_config(bool game_disc);

// Plain-words cause of death from a gate id plus what the CPU was doing.
// `exception_cause`/`epc` are the last non-interrupt exception (0xFFFFFFFF = none).
GrimDeath grim_describe_death(const std::string &reason, u32 frame, double seconds, u32 pc,
                              u32 exception_cause, u32 epc, const GrimGenome *genome,
                              const u8 *ram = nullptr, const Bios *bios = nullptr);

// Index of the gene that patched the word at `epc` (or the next word, for a delay
// slot), or -1. ROM addresses match patch offsets directly; RAM addresses match when
// the word and its neighbours equal the patched image (`ram` = 2 MB main RAM, `bios`
// = the loaded, patched BIOS). Either pointer may be null to skip the RAM case.
s32 grim_find_culprit_gene(const GrimGenome &genome, u32 epc, const u8 *ram, const Bios *bios);

// Mercy (off by default): a death inside the kill window is rerolled by the
// caller. Pure function so it can be tested without a machine.
constexpr u32 kGrimMercyWindowFrames = 240; // about 4 s
bool grim_mercy_should_reroll(bool mercy_enabled, const GrimLiveStatus &status);

class GrimLiveWatch {
public:
  explicit GrimLiveWatch(const GrimGenome *genome, GrimLivenessConfig config);
  // Call while the emulator is paused and after System::reset(). Attaches the
  // light telemetry and the audio tap.
  void attach(System &sys);
  void detach(System &sys);
  // Emulator thread, once per completed System::run_frame().
  void on_frame(System &sys);
  GrimLiveStatus status() const;

private:
  GrimLivenessConfig config_;
  GrimGenome genome_; // copy: describing a death must not depend on the caller
  GrimTelemetry telemetry_;
  GrimLivenessTracker tracker_;
  std::vector<s16> tap_;
  bool attached_ = false;
  bool dead_ = false;
  u32 frames_ = 0;
  mutable std::mutex mutex_;
  GrimLiveStatus status_;
};
