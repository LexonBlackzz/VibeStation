#pragma once
// Grim Reaper 2.0, Phase 5: everything the panel needs to keep between frames.
// The core pieces (generator, watch, library) live in src/core/grim_*.
#include "../core/grim_genome.h"
#include "../core/grim_library.h"
#include "../core/grim_live.h"
#include "../core/grim_pull.h"
#include "../core/grim_rom.h"
#include "../core/grim_sample.h"
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

struct GrimPullState {
  // User settings (the panel edits these directly).
  GrimPullSettings settings;
  bool mercy = false;

  // The user's pulls. Loaded lazily, saved after every change.
  GrimLibrary library;
  bool library_loaded = false;
  std::string data_dir;
  GrimPullHistory history;

  // Per-BIOS generation context, rebuilt when the loaded BIOS changes.
  std::string ctx_bios_path;
  std::unique_ptr<GrimSampleContext> sample;
  std::unique_ptr<GrimRomContext> rom;
  u64 bios_hash = 0;
  std::string bios_name; // "SCPH-1001"-style label, or the file name

  // Boot-map discovery (needed by the Code family): one child process per BIOS.
  enum class MapState { None, Running, Ready, Failed };
  MapState map_state = MapState::None;
  std::thread map_thread;
  std::shared_ptr<std::atomic<int>> map_result; // -1 running, 0 ok, 1 failed (shared with the job)
  std::string map_path;
  std::string map_error;
  double map_started = 0.0;

  // The machine on screen.
  bool has_machine = false;
  u64 pull = 0;
  GrimGenome genome;
  std::string machine_id;
  std::vector<GrimGeneLine> lines;
  std::unique_ptr<GrimGenomeRuntime> runtime;
  std::unique_ptr<GrimLiveWatch> watch;
  GrimLiveStatus status;
  bool outcome_saved = false; // the death (or survival) is recorded in the library
  u32 rerolls = 0;            // pulls Mercy replaced silently since the last shown one
  double alive_recorded = 0.0;

  std::string message;
};
