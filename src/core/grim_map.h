#pragma once
#include "types.h"
#include <array>
#include <map>
#include <string>
#include <vector>

// Grim Reaper 2.0, Phase 3: the clean-boot map of a BIOS.
//
// GrimBootMapper watches one clean boot of the stock BIOS (discovery runs only,
// interpreter only; it is fed from the telemetry loop in Cpu::run_slice and from
// DmaController, never from Cpu::step, the dynarec or normal play) and records
// per ROM word whether it was executed, read as data, when, and where the data
// went. Because the kernel and the Shell run from RAM, every RAM byte carries a
// provenance tag ("this byte came from ROM byte X") that follows the data
// through loads, stores and DMA. See docs/grim-reaper/PROGRESS.md.

enum class GrimWordClass : u8 { Unused = 0, Code, Data, Unknown };
const char *grim_word_class_name(GrimWordClass c);

// Where the data of a ROM word ended up (bit mask; 0 = the CPU only).
constexpr u8 kGrimConsumerSpu = 1;  // SPU registers / FIFO / SPU DMA
constexpr u8 kGrimConsumerGpu = 2;  // GP0 / GP1 / GPU DMA
constexpr u8 kGrimConsumerGte = 4;  // COP2 (lwc2, mtc2, ctc2)
constexpr u8 kGrimConsumerMdec = 8; // MDEC input
std::string grim_consumer_name(u8 mask);

constexpr u64 kGrimNever = ~0ull;
constexpr u32 kGrimNoRomOffset = ~0u;

// Discovery events refer to SPU byte addresses, not the 8-byte units stored in
// voice registers. A missing origin is retained: addresses are often written
// before the bank is uploaded, then resolved again at the key-on write.
enum class GrimSpuSampleUseKind : u8 { StartWrite, RepeatWrite, KeyOnStart, KeyOnRepeat };
const char *grim_spu_sample_use_kind_name(GrimSpuSampleUseKind kind);
struct GrimSpuSampleUse {
  u64 cycle = 0;
  u32 spu_address = 0;
  u32 rom_offset = kGrimNoRomOffset;
  u8 voice = 0;
  GrimSpuSampleUseKind kind = GrimSpuSampleUseKind::StartWrite;
};

// Word flags.
constexpr u8 kGrimExecDirect = 1;  // fetched straight from ROM
constexpr u8 kGrimExecViaRam = 2;  // executed from RAM after being copied
constexpr u8 kGrimReadDirect = 4;  // loaded from ROM by the CPU
constexpr u8 kGrimReadViaRam = 8;  // loaded (or DMA'd) from a RAM copy
constexpr u8 kGrimPartial = 16;    // part of an executed RAM word whose four
                                   // bytes did not come from one ROM word
constexpr u8 kGrimUsed = 32;       // the value was consumed by computation, a branch
                                   // or a peripheral. Moving it (a copy loop's load
                                   // and store, a DMA to RAM) is transport, not use.

struct GrimMapWord {
  u64 first_exec = kGrimNever; // emulated CPU cycle
  u64 first_read = kGrimNever;
  GrimWordClass cls = GrimWordClass::Unused;
  u8 consumer = 0;
  u8 flags = 0;
};

struct GrimBootMap {
  u64 bios_hash = 0;
  std::string scenario = "nodisc";
  u32 frames = 0;
  u64 cycles = 0;
  // Emulated cycle at which the last never-before-executed instruction word
  // ran: the end of the interesting part of the timeline.
  u64 last_new_code_cycle = 0;
  // Provenance coverage: distinct RAM words ever executed, and how many of
  // them had a known ROM origin (the four bytes from one ROM word).
  u32 ram_exec_words = 0;
  u32 ram_exec_known = 0;
  std::vector<GrimMapWord> words;
  std::vector<GrimSpuSampleUse> spu_sample_uses;

  u32 rom_words() const { return static_cast<u32>(words.size()); }
  u32 count(GrimWordClass c) const;
  // Derived subdivision of Unused, preserving the Phase 3 serialized classes.
  u32 dormant_words() const;
  // 0..1000.
  u32 provenance_permille() const {
    return ram_exec_words == 0 ? 1000u : static_cast<u32>(u64{ram_exec_known} * 1000u / ram_exec_words);
  }
  // FNV-1a over everything a map holds, for determinism checks.
  u64 hash() const;
};

// The class rules, from the flags and consumer a word collected.
GrimWordClass grim_classify_word(u8 flags, u8 consumer);
// Union of two maps of the same BIOS (e.g. no-disc and disc scenarios): flags and
// consumers are OR-ed, first-touch times take the minimum, classes are recomputed.
GrimBootMap grim_map_merge(const GrimBootMap &a, const GrimBootMap &b);

struct GrimMapRegion {
  u32 start_word = 0, end_word = 0; // [start, end)
  GrimWordClass cls = GrimWordClass::Unused;
  u8 consumer = 0;
  u64 first_touch = kGrimNever; // min over the region of first_exec / first_read
  u32 via_ram = 0;              // words reached via a RAM copy
};
std::vector<GrimMapRegion> grim_map_regions(const GrimBootMap &m);
std::vector<std::pair<u32, u32>> grim_map_dormant_regions(const GrimBootMap &m);

// Writes <path> (JSON header + region list) and <path>.words (binary, per
// word). Both are byte-for-byte reproducible.
bool grim_map_save(const GrimBootMap &m, const std::string &path, std::string &err);
bool grim_map_load(const std::string &path, GrimBootMap &out, std::string &err);
// Regions smaller than min_bytes are left out of the table (0 = list everything).
std::string grim_map_summary(const GrimBootMap &m, u32 min_bytes = 128);

// Copy-routine measurements, for the report and PROGRESS.md.
struct GrimCopyStats {
  // Bytes of tagged data written to RAM/scratchpad by each store opcode.
  std::array<u64, 64> tagged_store_bytes_by_op{};
  // Bytes of tagged data loaded by each load opcode (ROM or tagged RAM).
  std::array<u64, 64> tagged_load_bytes_by_op{};
  u64 dma_tagged_words = 0;       // RAM->device words with a known ROM origin
  u64 dma_words_to_ram = 0;       // device->RAM words (tags cleared)
  u64 tagged_stores_to_io = 0;    // tagged register stored to an I/O address
  // Tagged stores per store PC (the copy loops), ordered by pc.
  std::map<u32, u64> tagged_store_pcs;
  // Bytes loaded straight from ROM (the CPU reading ROM data), per load PC.
  std::map<u32, u64> rom_load_pcs;
  // Executed RAM words with no ROM origin, as physical RAM byte ranges.
  std::vector<std::pair<u32, u32>> unknown_exec_ranges;
};

class GrimBootMapper {
public:
  explicit GrimBootMapper(u32 rom_bytes);

  // Called around System-visible instruction execution by Cpu::run_slice while
  // attached. `gpr` is the register file before the instruction runs.
  void begin_instruction(u64 cycle, u32 pc, u32 instr, const u32 *gpr, u32 sr);
  // The instruction really ran (it was not replaced by an interrupt entry).
  void commit_instruction();

  // DMA: `words` words starting at RAM address `addr`, moving by `step` bytes
  // per word. from_ram = RAM to device (tags are consumed); otherwise the
  // device writes RAM (tags are cleared).
  void note_dma(int channel, bool from_ram, u32 addr, s32 step, u32 words, u64 cycle);

  GrimBootMap finish(u64 bios_hash, const std::string &scenario, u32 frames,
                     u64 cycles) const;
  const GrimCopyStats &copy_stats() const { return stats_; }
  GrimCopyStats stats_report() const; // stats_ with the unknown ranges filled in

  // Test access: the tag (ROM byte offset + 1, 0 = none) of a physical RAM byte.
  u32 ram_tag(u32 phys) const { return ram_tag_[phys & (kRamBytes - 1u)]; }
  u32 spu_tag(u32 addr) const { return spu_tag_[addr & (kSpuRamBytes - 1u)]; }

private:
  static constexpr u32 kRamBytes = 2u * 1024u * 1024u;
  static constexpr u32 kSpuRamBytes = 512u * 1024u;
  enum Region : u8 { kNone, kRam, kScratch, kRom, kIo };
  Region decode(u32 addr, u32 &idx) const;
  u32 get_tag(Region r, u32 idx) const;
  void set_tag(Region r, u32 idx, u32 tag);
  void note_exec(u32 pc);
  void note_use(const u32 *tags); // four byte tags of a register that was consumed
  // `tags` are byte tags of data that just reached a consumer / was read.
  void note_origin(const u32 *tags, size_t n, u8 consumer, bool via_ram, u64 cycle);
  void load_tags(u32 op, u32 ea, u32 rt, u32 out[4], bool &ok);
  void set_reg_tags(u32 reg, const u32 tags[4]);
  void note_spu_write16(u32 offset, u16 value, const u32 *tags);
  void note_spu_sample_use(u32 voice, u32 addr, GrimSpuSampleUseKind kind);
  u32 spu_rom_origin(u32 addr) const;

  u32 rom_bytes_;
  std::vector<u32> ram_tag_;      // one per RAM byte
  std::vector<u32> spu_tag_;      // one per SPU RAM byte, discovery only
  std::array<u16, 512> spu_regs_{};
  u32 spu_transfer_addr_ = 0;
  std::vector<GrimSpuSampleUse> spu_sample_uses_;
  std::array<u32, 1024> scratch_tag_{};
  std::array<std::array<u32, 4>, 32> reg_tags_{};
  int pend_reg_ = -1;
  std::array<u32, 4> pend_tags_{};

  // Per ROM word.
  std::vector<u64> first_exec_, first_read_;
  std::vector<u8> consumer_, flags_;
  // Executed RAM words: bit 0 seen, bit 1 known.
  std::vector<u8> ram_exec_;
  u32 ram_exec_words_ = 0, ram_exec_known_ = 0;

  // The instruction between begin and commit.
  u64 cur_cycle_ = 0;
  u32 cur_pc_ = 0, cur_instr_ = 0, cur_ea_ = 0, cur_sr_ = 0, cur_value_ = 0;
  GrimCopyStats stats_;
};
