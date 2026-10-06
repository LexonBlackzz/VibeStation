#pragma once
#include "types.h"
#include <cstddef>
#include <string>

// Classic Grim Reaper byte engines, in the spirit of the Vinesauce ROM Corruptor:
// what happens to each byte that gets hit. Shared by the Classic BIOS reaper (where
// the bytes are the BIOS image) and the RAM reaper (main RAM, VRAM, sound RAM).
enum class GrimByteOp : u8 {
  Random,      // a random byte (the original Classic behaviour)
  Add,         // old + n
  Subtract,    // old - n
  Replace,     // bytes equal to `match` become n; others are left alone
  ShiftLeft,   // old << n
  ShiftRight,  // old >> n
  RotateLeft,  // bits wrap around
  RotateRight,
  Xor,         // old ^ n
  And,         // old & n
  Or,          // old | n
  Invert,      // ~old
  Set,         // n
  Count
};

struct GrimByteEngine {
  GrimByteOp op = GrimByteOp::Random;
  u8 value = 1; // n: amount, mask, shift count (1-7) or the replacement byte
  u8 match = 0; // Replace only: the byte that gets replaced
};

// The new value of one hit byte. `random` supplies the Random engine's byte.
u8 grim_byte_apply(const GrimByteEngine &engine, u8 old, u32 random);

// Where hits land inside [start, end] (inclusive byte offsets).
struct GrimByteSweep {
  size_t start = 0;
  size_t end = 0;
  // 0: random strike, `strikes` random positions (repeats allowed). N > 0: every Nth
  // byte from `start` (Vinesauce's "corrupt every N bytes"); `strikes` is ignored.
  size_t every = 0;
  size_t strikes = 1;
};

// Applies the engine to `data` and returns how many bytes changed. Deterministic from
// `seed`. With the Random engine and a random strike it draws exactly what the
// original Classic reaper drew, so old seeds and presets reproduce.
size_t grim_byte_corrupt(u8 *data, size_t size, const GrimByteSweep &sweep,
                         const GrimByteEngine &engine, u64 seed);

const char *grim_byte_op_name(GrimByteOp op);         // "Random", "Add", ...
const char *grim_byte_op_key(GrimByteOp op);          // "random", "add", ... (presets)
const char *grim_byte_op_help(GrimByteOp op);         // one line for the UI
bool grim_byte_op_from_key(const std::string &key, GrimByteOp &out);
// Which parameters the op reads, for the UI.
bool grim_byte_op_uses_value(GrimByteOp op);
bool grim_byte_op_is_shift(GrimByteOp op); // value means 1-7 bits
