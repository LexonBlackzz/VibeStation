#include "grim_classic.h"
#include <random>

namespace {
struct OpInfo {
  const char *name;
  const char *key;
  const char *help;
};
constexpr OpInfo kOps[] = {
    {"Random", "random", "Each hit byte becomes a random value."},
    {"Add", "add", "Adds N to each hit byte (wraps at 255)."},
    {"Subtract", "subtract", "Subtracts N from each hit byte (wraps at 0)."},
    {"Replace", "replace", "Hit bytes equal to the match value become N; others stay."},
    {"Shift left", "shift_left", "Shifts each hit byte left by N bits."},
    {"Shift right", "shift_right", "Shifts each hit byte right by N bits."},
    {"Rotate left", "rotate_left", "Rotates each hit byte left by N bits (bits wrap)."},
    {"Rotate right", "rotate_right", "Rotates each hit byte right by N bits (bits wrap)."},
    {"XOR", "xor", "XORs each hit byte with N: flips the bits set in N."},
    {"AND", "and", "ANDs each hit byte with N: clears the bits not in N."},
    {"OR", "or", "ORs each hit byte with N: sets the bits in N."},
    {"Invert", "invert", "Flips every bit of each hit byte."},
    {"Set", "set", "Every hit byte becomes N."},
    {"Pipe", "pipe", "Copies the byte at hit + offset over each hit byte (data onto data)."},
};
static_assert(sizeof(kOps) / sizeof(kOps[0]) == static_cast<size_t>(GrimByteOp::Count));

// The original Classic reaper's seeding, kept so old seeds reproduce.
void seed_mt19937(std::mt19937 &rng, u64 seed) {
  const u32 lo = static_cast<u32>(seed & 0xFFFFFFFFull);
  const u32 hi = static_cast<u32>((seed >> 32) & 0xFFFFFFFFull);
  std::seed_seq seq{lo, hi, 0x9E3779B9u, 0x243F6A88u};
  rng.seed(seq);
}
} // namespace

u8 grim_byte_apply(const GrimByteEngine &e, u8 old, u32 random, u8 piped) {
  const u32 n = e.value;
  const u32 s = n & 7u;
  switch (e.op) {
  case GrimByteOp::Random: return static_cast<u8>(random & 0xFFu);
  case GrimByteOp::Add: return static_cast<u8>(old + n);
  case GrimByteOp::Subtract: return static_cast<u8>(old - n);
  case GrimByteOp::Replace: return old == e.match ? static_cast<u8>(n) : old;
  case GrimByteOp::ShiftLeft: return static_cast<u8>(old << s);
  case GrimByteOp::ShiftRight: return static_cast<u8>(old >> s);
  case GrimByteOp::RotateLeft: return static_cast<u8>((old << s) | (old >> ((8u - s) & 7u)));
  case GrimByteOp::RotateRight: return static_cast<u8>((old >> s) | (old << ((8u - s) & 7u)));
  case GrimByteOp::Xor: return static_cast<u8>(old ^ n);
  case GrimByteOp::And: return static_cast<u8>(old & n);
  case GrimByteOp::Or: return static_cast<u8>(old | n);
  case GrimByteOp::Invert: return static_cast<u8>(~old);
  case GrimByteOp::Set: return static_cast<u8>(n);
  case GrimByteOp::Pipe: return piped;
  case GrimByteOp::Count: break;
  }
  return old;
}

size_t grim_byte_corrupt(u8 *data, size_t size, const GrimByteSweep &sweep, const GrimByteEngine &engine,
                         u64 seed) {
  if (data == nullptr || size == 0 || sweep.start >= size) {
    return 0;
  }
  const size_t end = sweep.end < size ? sweep.end : size - 1u;
  if (end < sweep.start) {
    return 0;
  }
  std::mt19937 rng;
  seed_mt19937(rng, seed);
  size_t changed = 0;
  const auto hit = [&](size_t i, u32 random) {
    const u8 piped = engine.op == GrimByteOp::Pipe ? data[grim_byte_pipe_source(i, engine.offset, size)] : 0;
    const u8 now = grim_byte_apply(engine, data[i], random, piped);
    changed += now != data[i] ? 1u : 0u;
    data[i] = now;
  };
  if (sweep.every > 0) {
    for (size_t i = sweep.start; i <= end; i += sweep.every) {
      hit(i, engine.op == GrimByteOp::Random ? static_cast<u32>(rng()) : 0u);
      if (end - i < sweep.every) {
        break; // no overflow past the end of size_t
      }
    }
    return changed;
  }
  const size_t span = end - sweep.start + 1u;
  for (size_t k = 0; k < sweep.strikes; ++k) {
    const size_t i = sweep.start + (static_cast<size_t>(rng()) % span);
    hit(i, static_cast<u32>(rng())); // always drawn: positions match the Random engine's
  }
  return changed;
}

const char *grim_byte_op_name(GrimByteOp op) {
  return op < GrimByteOp::Count ? kOps[static_cast<size_t>(op)].name : "?";
}
const char *grim_byte_op_key(GrimByteOp op) {
  return op < GrimByteOp::Count ? kOps[static_cast<size_t>(op)].key : "random";
}
const char *grim_byte_op_help(GrimByteOp op) {
  return op < GrimByteOp::Count ? kOps[static_cast<size_t>(op)].help : "";
}
bool grim_byte_op_from_key(const std::string &key, GrimByteOp &out) {
  for (size_t i = 0; i < static_cast<size_t>(GrimByteOp::Count); ++i) {
    if (key == kOps[i].key) {
      out = static_cast<GrimByteOp>(i);
      return true;
    }
  }
  return false;
}
bool grim_byte_op_uses_value(GrimByteOp op) {
  return op != GrimByteOp::Random && op != GrimByteOp::Invert && op != GrimByteOp::Pipe;
}
bool grim_byte_op_uses_offset(GrimByteOp op) { return op == GrimByteOp::Pipe; }
size_t grim_byte_pipe_source(size_t index, s32 offset, size_t size) {
  if (size == 0) {
    return 0;
  }
  const s64 n = static_cast<s64>(size);
  s64 at = (static_cast<s64>(index) + offset) % n;
  return static_cast<size_t>(at < 0 ? at + n : at);
}
bool grim_byte_op_is_shift(GrimByteOp op) {
  return op == GrimByteOp::ShiftLeft || op == GrimByteOp::ShiftRight || op == GrimByteOp::RotateLeft ||
         op == GrimByteOp::RotateRight;
}
