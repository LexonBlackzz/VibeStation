#pragma once
#include "grim_classic.h"
#include "types.h"
#include <functional>
#include <string>
#include <utility>
#include <vector>

// Grim Reaper disc corruption: game data is corrupted as the CD drive reads it, the PS1
// equivalent of corrupting a ROM. A sector is always corrupted the same way (the hits
// depend on the seed and the sector number only), so a corrupted disc behaves like a
// damaged disc image: rereading a sector gives the same bytes.

// What may be hit. The ISO9660 file system itself (volume descriptors, path tables,
// directories) is never touched: without it nothing can be found and nothing is fun.
enum GrimDiscTarget : u32 {
  kGrimDiscFiles = 1,     // ordinary game files (Mode 2 Form 1 / Mode 1 data)
  kGrimDiscMovies = 2,    // STR movie frames (picture data; the frame header is kept)
  kGrimDiscCdAudio = 4,   // CD audio tracks
  kGrimDiscBootCode = 8,  // SYSTEM.CNF's boot executable (often kills the game)
  kGrimDiscXaAudio = 16,  // XA audio (voices, music, effects streamed from disc)
};

struct GrimDiscReaperConfig {
  bool enabled = false;
  GrimByteEngine engine;
  float percent = 0.05f;  // random strike: share of the bytes of each eligible sector
  u32 every = 0;          // N > 0: every Nth byte of the disc's data instead
  u32 targets = kGrimDiscMovies | kGrimDiscXaAudio; // the survivable ones
  u32 start_frame = 0;    // reads before this emulated frame are left alone
  u64 seed = 1;
  // Game files only: leave sectors that look like MIPS code or packed (compressed)
  // data alone. One wrong byte there usually kills the game; textures, models, sound
  // banks and tables survive far more often.
  bool skip_risky = true;
};

// The heuristic behind skip_risky, on the user data of one sector.
bool grim_disc_sector_is_risky(const u8 *data, size_t len);

// The parts of a disc that keep it bootable, found by reading its ISO9660 file system.
struct GrimDiscLayout {
  bool valid = false;                       // an ISO9660 volume was found
  std::vector<std::pair<int, int>> system;  // [lba, end) file system sectors
  std::pair<int, int> boot = {0, 0};        // [lba, end) of the boot executable
  std::string boot_name;                    // e.g. "SLUS_005.94"
  size_t files = 0;
  bool is_system(int lba) const;
  bool is_boot(int lba) const { return lba >= boot.first && lba < boot.second; }
};

// `read_user` fills the 2048 user-data bytes of a data sector; false when it cannot.
GrimDiscLayout grim_disc_scan(const std::function<bool(int lba, u8 *user)> &read_user);

// Command-line form, comma separated: "pct=0.05,targets=files+movies+xa+audio+boot,
// seed=7,engine=xor,value=FF,match=00,offset=256,every=0,start=600" (any subset).
bool grim_disc_parse(const std::string &spec, GrimDiscReaperConfig &out, std::string &err);
// Set by --disc-reaper; every new CdRom starts with it (headless tests).
extern GrimDiscReaperConfig g_grim_disc_cli;

// Corrupts one raw sector in place, leaving sync, header and subheader alone. Returns
// how many bytes changed. `sector_size` is 2352 (raw) or 2048 (user data only).
size_t grim_disc_corrupt_sector(u8 *raw, size_t sector_size, bool audio_track, int lba,
                                const GrimDiscReaperConfig &cfg, const GrimDiscLayout &layout);
