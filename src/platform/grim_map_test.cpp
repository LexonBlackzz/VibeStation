// --grim-map-test: Grim Reaper 2.0 Phase 3 tests (clean-boot map, provenance,
// classification, ROM gene mutations, ROM genes end to end).
#include "core/bios.h"
#include "core/grim_eval.h"
#include "core/grim_genome.h"
#include "core/grim_map.h"
#include "core/grim_rom.h"
#include "core/grim_sample.h"
#include "core/system.h"
#include "core/types.h"
#include "platform/grim_eval_runner.h"
#include "platform/grim_map_runner.h"
#include "platform/grim_process.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <thread>

namespace {

int g_failures = 0;

void check(bool ok, const std::string &name, const std::string &detail = "") {
  std::printf("GRIM_MAP_TEST %s %s %s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.c_str());
  std::fflush(stdout);
  if (!ok) {
    ++g_failures;
  }
}

std::string hex(u64 v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "0x%llX", static_cast<unsigned long long>(v));
  return buf;
}

std::filesystem::path make_temp_dir() {
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() /
      ("vibestation_grim_map_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(dir);
  return dir;
}

std::string read_file(const std::filesystem::path &p) {
  std::ifstream in(p, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

GrimEvalConfig eval_cfg(const std::string &bios, u32 frames) {
  GrimEvalConfig c;
  c.bios_path = bios;
  c.frames = frames;
  c.stop_on_death = false;
  c.watchdog_seconds = 0.0;
  return c;
}

// ---- a tiny assembler for the synthetic BIOS ------------------------------------------------

constexpr u32 kZero = 0, kV0 = 2, kV1 = 3, kA0 = 4, kT0 = 8, kT1 = 9, kT2 = 10, kT3 = 11, kT4 = 12,
              kT5 = 13, kT6 = 14, kT7 = 15, kRa = 31;

u32 I(u32 op, u32 rs, u32 rt, s32 imm) {
  return (op << 26) | (rs << 21) | (rt << 16) | (static_cast<u32>(imm) & 0xFFFFu);
}
u32 R(u32 rs, u32 rt, u32 rd, u32 sh, u32 fn) { return (rs << 21) | (rt << 16) | (rd << 11) | (sh << 6) | fn; }
u32 LUI(u32 rt, u32 imm) { return I(0xF, 0, rt, static_cast<s32>(imm)); }
u32 ORI(u32 rt, u32 rs, u32 imm) { return I(0xD, rs, rt, static_cast<s32>(imm)); }
u32 ADDIU(u32 rt, u32 rs, s32 imm) { return I(9, rs, rt, imm); }
u32 LW(u32 rt, s32 off, u32 base) { return I(0x23, base, rt, off); }
u32 SW(u32 rt, s32 off, u32 base) { return I(0x2B, base, rt, off); }
u32 LBU(u32 rt, s32 off, u32 base) { return I(0x24, base, rt, off); }
u32 SB(u32 rt, s32 off, u32 base) { return I(0x28, base, rt, off); }
u32 SH(u32 rt, s32 off, u32 base) { return I(0x29, base, rt, off); }
u32 LHU(u32 rt, s32 off, u32 base) { return I(0x25, base, rt, off); }
u32 BNE(u32 rs, u32 rt, s32 off) { return I(5, rs, rt, off); }
u32 BEQ(u32 rs, u32 rt, s32 off) { return I(4, rs, rt, off); }
u32 JAL(u32 addr) { return (3u << 26) | ((addr >> 2) & 0x3FFFFFFu); }
u32 JR(u32 rs) { return R(rs, 0, 0, 0, 8); }
u32 JALR(u32 rs) { return R(rs, 0, kRa, 0, 9); }
u32 ADDU(u32 rd, u32 rs, u32 rt) { return R(rs, rt, rd, 0, 0x21); }
u32 MTC0(u32 rt, u32 rd) { return (0x10u << 26) | (4u << 21) | (rt << 16) | (rd << 11); }

struct Asm {
  std::vector<u32> w;
  size_t here() const { return w.size(); }
  void emit(u32 x) { w.push_back(x); }
  // Sets a register to a 32-bit constant.
  void li(u32 rt, u32 value) {
    emit(LUI(rt, value >> 16));
    emit(ORI(rt, rt, value & 0xFFFF));
  }
  // Calls a KSEG0 address from the BIOS (a JAL cannot leave the 0xBxxxxxxx segment).
  void call(u32 addr) {
    li(kT0, addr);
    emit(JALR(kT0));
    emit(0);
  }
  // Word copy loop: `count` words from `src` to `dst`, then the caller continues.
  void copy_words(u32 src, u32 dst, u32 count) {
    li(kT0, src);
    li(kT1, dst);
    emit(ADDIU(kT2, kZero, static_cast<s32>(count)));
    const size_t loop = here();
    emit(LW(kT3, 0, kT0));
    emit(ADDIU(kT0, kT0, 4));
    emit(SW(kT3, 0, kT1));
    emit(ADDIU(kT2, kT2, -1));
    emit(BNE(kT2, kZero, static_cast<s32>(loop) - static_cast<s32>(here() + 1)));
    emit(ADDIU(kT1, kT1, 4));
  }
  void copy_bytes(u32 src, u32 dst, u32 count) {
    li(kT0, src);
    li(kT1, dst);
    emit(ADDIU(kT2, kZero, static_cast<s32>(count)));
    const size_t loop = here();
    emit(LBU(kT3, 0, kT0));
    emit(ADDIU(kT0, kT0, 1));
    emit(SB(kT3, 0, kT1));
    emit(ADDIU(kT2, kT2, -1));
    emit(BNE(kT2, kZero, static_cast<s32>(loop) - static_cast<s32>(here() + 1)));
    emit(ADDIU(kT1, kT1, 1));
  }
};

// Synthetic ROM layout (byte offsets).
constexpr u32 kBlockA = 0x800, kBlockB = 0x900, kBlockR = 0xC80, kTagData = 0xA00,
              kJumpTable = 0xB00, kLiteral = 0xB40, kStub1 = 0xC00, kStub2 = 0xC10,
              kCopyOnly = 0xD00;

std::vector<u32> synthetic_rom() {
  std::vector<u32> rom(psx::BIOS_SIZE / 4u, 0u);
  auto put = [&](u32 off, std::initializer_list<u32> ws) {
    u32 i = off / 4;
    for (u32 x : ws) {
      rom[i++] = x;
    }
  };
  // Three little functions, each with an identical decoy before and after it.
  const u32 fa[4] = {ADDIU(kV0, kV0, 1), ADDIU(kV0, kV0, 2), JR(kRa), 0};
  const u32 fb[4] = {ADDIU(kV1, kV1, 1), ADDIU(kV1, kV1, 2), JR(kRa), 0};
  const u32 fr[4] = {ADDIU(kA0, kA0, 1), ADDIU(kA0, kA0, 2), JR(kRa), 0};
  for (u32 base : {kBlockA - 0x20, kBlockA, kBlockA + 0x10}) {
    put(base, {fa[0], fa[1], fa[2], fa[3]});
  }
  for (u32 base : {kBlockB - 0x20, kBlockB, kBlockB + 0x10}) {
    put(base, {fb[0], fb[1], fb[2], fb[3]});
  }
  for (u32 base : {kBlockR - 0x20, kBlockR, kBlockR + 0x10}) {
    put(base, {fr[0], fr[1], fr[2], fr[3]});
  }
  put(kTagData, {0x11111111u, 0x22222222u});
  put(kJumpTable, {0xBFC00000u + kStub1, 0xBFC00000u + kStub2});
  put(kLiteral, {0x12345678u});
  put(kStub1, {JR(kRa), 0});
  put(kStub2, {JR(kRa), 0});
  for (u32 i = 0; i < 16; ++i) {
    rom[(kCopyOnly / 4) + i] = 0xD0D00000u + i;
  }
  put(0x180, {BEQ(kZero, kZero, -1), 0}); // exception vector: park

  Asm a;
  a.copy_words(0xBFC00000u + kBlockA, 0x80010000u, 4); // ROM -> RAM, words
  a.call(0x80010000u);
  a.copy_bytes(0xBFC00000u + kBlockB, 0x80011000u, 16); // ROM -> RAM, bytes
  a.call(0x80011000u);
  a.copy_words(0xBFC00000u + kBlockR, 0x80014000u, 4); // ROM -> RAM
  a.copy_words(0x80014000u, 0x80015000u, 4);           // RAM -> RAM relocation
  a.call(0x80015000u);
  // Stores under cache isolation must not change tags.
  a.li(kT0, 0xBFC00000u + kTagData);
  a.emit(LW(kT4, 0, kT0));
  a.emit(LW(kT7, 4, kT0));
  a.li(kT5, 0xA0020000u);
  a.emit(0);
  a.emit(SW(kT4, 0, kT5));
  a.emit(SW(kT4, 4, kT5));
  a.emit(LUI(kT6, 0x0041)); // BEV | IsC
  a.emit(MTC0(kT6, 12));
  a.emit(0);
  a.emit(SW(kT7, 4, kT5)); // isolated: goes to the cache
  a.emit(SW(kT7, 8, kT5)); // isolated
  a.emit(LUI(kT6, 0x0040));
  a.emit(MTC0(kT6, 12));
  a.emit(0);
  // A jump table entry (data, used by jalr) and a literal (data, used by addu).
  a.li(kT0, 0xBFC00000u + kJumpTable);
  a.emit(LW(kT1, 0, kT0));
  a.emit(0);
  a.emit(JALR(kT1));
  a.emit(0);
  a.li(kT0, 0xBFC00000u + kLiteral);
  a.emit(LW(kT2, 0, kT0));
  a.emit(0);
  a.emit(ADDU(kT3, kT2, kT2));
  // Copied but never used.
  a.copy_words(0xBFC00000u + kCopyOnly, 0x80016000u, 16);
  const size_t park = a.here();
  a.emit(BEQ(kZero, kZero, -1));
  a.emit(0);
  (void)park;
  for (size_t i = 0; i < a.w.size(); ++i) {
    rom[i] = a.w[i];
  }
  return rom;
}

bool write_rom(const std::filesystem::path &path, const std::vector<u32> &rom) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out.write(reinterpret_cast<const char *>(rom.data()),
            static_cast<std::streamsize>(rom.size() * sizeof(u32)));
  return static_cast<bool>(out);
}

struct SynthRun {
  GrimBootMap map;
  std::unique_ptr<GrimBootMapper> mapper;
  bool ok = false;
};

SynthRun run_synthetic(const std::filesystem::path &bios, u32 frames = 3) {
  SynthRun r;
  auto sys = std::make_unique<System>();
  if (!sys->load_bios(bios.string())) {
    return r;
  }
  sys->reset();
  GrimTelemetry telemetry;
  r.mapper = std::make_unique<GrimBootMapper>(sys->bios_mut().image_size());
  sys->cpu().set_telemetry(&telemetry);
  sys->set_grim_boot_mapper(r.mapper.get());
  sys->set_spu_audio_capture(true);
  for (u32 i = 0; i < frames; ++i) {
    sys->run_frame(true, false);
    sys->clear_spu_audio_capture();
  }
  sys->set_grim_boot_mapper(nullptr);
  sys->cpu().set_telemetry(nullptr);
  r.map = r.mapper->finish(sys->bios_mut().image_hash(), "synthetic", frames, telemetry.total_cycles());
  r.ok = true;
  return r;
}

bool words_are(const GrimBootMap &m, u32 off, u32 count, GrimWordClass cls, u8 must_flags = 0) {
  for (u32 i = 0; i < count; ++i) {
    const GrimMapWord &w = m.words[off / 4 + i];
    if (w.cls != cls || (w.flags & must_flags) != must_flags) {
      return false;
    }
  }
  return true;
}

void test_synthetic(const std::filesystem::path &dir) {
  const std::vector<u32> rom = synthetic_rom();
  const std::filesystem::path path = dir / "synthetic.bin";
  write_rom(path, rom);
  SynthRun r = run_synthetic(path);
  check(r.ok, "synthetic_bios_runs",
        r.ok ? "code words=" + std::to_string(r.map.count(GrimWordClass::Code)) + " cycles=" + std::to_string(r.map.cycles)
             : "");
  if (!r.ok) {
    return;
  }
  const GrimBootMap &m = r.map;
  const u32 exec = kGrimExecViaRam;
  // Word copy, byte copy, and a RAM->RAM relocation: each executed word maps to its
  // own ROM word (the copy loops read it as data first: the copy trap), and the
  // identical decoys next to it stay untouched.
  check(words_are(m, kBlockA, 4, GrimWordClass::Code, exec), "provenance_word_copy_executes_as_code");
  check(words_are(m, kBlockB, 4, GrimWordClass::Code, exec), "provenance_byte_copy_executes_as_code");
  check(words_are(m, kBlockR, 4, GrimWordClass::Code, exec), "provenance_ram_relocation_executes_as_code");
  check(words_are(m, kBlockA - 0x20, 4, GrimWordClass::Unused) && words_are(m, kBlockA + 0x10, 4, GrimWordClass::Unused) &&
            words_are(m, kBlockB - 0x20, 4, GrimWordClass::Unused) && words_are(m, kBlockB + 0x10, 4, GrimWordClass::Unused) &&
            words_are(m, kBlockR - 0x20, 4, GrimWordClass::Unused) && words_are(m, kBlockR + 0x10, 4, GrimWordClass::Unused),
        "provenance_identical_decoys_are_not_marked");
  check(m.ram_exec_words == 12 && m.ram_exec_known == 12, "provenance_coverage_is_complete",
        std::to_string(m.ram_exec_known) + "/" + std::to_string(m.ram_exec_words));
  // Exact byte tags in RAM.
  bool tags_ok = true;
  for (u32 i = 0; i < 16; ++i) {
    tags_ok = tags_ok && r.mapper->ram_tag(0x10000 + i) == kBlockA + i + 1 &&
              r.mapper->ram_tag(0x11000 + i) == kBlockB + i + 1 &&
              r.mapper->ram_tag(0x14000 + i) == kBlockR + i + 1 &&
              r.mapper->ram_tag(0x15000 + i) == kBlockR + i + 1;
  }
  check(tags_ok, "provenance_exact_byte_tags_after_copies");
  // Cache isolation: the isolated stores must not touch the tags.
  bool isc_ok = true;
  for (u32 i = 0; i < 4; ++i) {
    isc_ok = isc_ok && r.mapper->ram_tag(0x20000 + i) == kTagData + i + 1 &&
             r.mapper->ram_tag(0x20004 + i) == kTagData + i + 1 && // from the first, not the isolated store
             r.mapper->ram_tag(0x20008 + i) == 0;
  }
  check(isc_ok, "iscache_stores_do_not_change_tags");
  // Classification.
  check(m.words[kJumpTable / 4].cls == GrimWordClass::Data && m.words[kLiteral / 4].cls == GrimWordClass::Data,
        "classify_jump_table_entry_and_literal_as_data");
  check(m.words[kJumpTable / 4 + 1].cls == GrimWordClass::Unused && m.words[kStub2 / 4].cls == GrimWordClass::Unused &&
            m.words[0xE00 / 4].cls == GrimWordClass::Unused && m.words[0xE00 / 4].flags == 0,
        "classify_untouched_as_unused");
  check(m.words[kStub1 / 4].cls == GrimWordClass::Code && m.words[0].cls == GrimWordClass::Code,
        "classify_direct_rom_code");
  bool copy_only = true;
  for (u32 i = 0; i < 16; ++i) {
    const GrimMapWord &w = m.words[kCopyOnly / 4 + i];
    copy_only = copy_only && w.cls == GrimWordClass::Unused && (w.flags & kGrimReadDirect) != 0;
  }
  check(copy_only, "classify_copy_only_block_is_unused_not_data");
  // Save / load round trip and byte-exact reproducibility.
  std::string err;
  const std::filesystem::path mp = dir / "syn_map.json", mp2 = dir / "syn_map2.json";
  GrimBootMap loaded;
  const bool saved = grim_map_save(m, mp.string(), err) && grim_map_load(mp.string(), loaded, err) &&
                     grim_map_save(loaded, mp2.string(), err);
  check(saved && loaded.hash() == m.hash() && read_file(mp) == read_file(mp2) &&
            read_file(mp.string() + ".words") == read_file(mp2.string() + ".words"),
        "map_file_roundtrip_is_exact", err);
}

// The mapper without a CPU: DMA, isolation and register-tag rules.
void test_mapper_unit() {
  GrimBootMapper mp(psx::BIOS_SIZE);
  u32 gpr[32] = {};
  u64 cycle = 100;
  auto step = [&](u32 instr, u32 sr = 0) {
    mp.begin_instruction(cycle, 0x80000000u, instr, gpr, sr);
    mp.commit_instruction();
    cycle += 2;
  };
  gpr[kT0] = 0xBFC01000u;
  gpr[kT2] = 0x80020000u;
  step(LW(kT1, 0, kT0));
  step(0);
  step(SW(kT1, 0, kT2));
  check(mp.ram_tag(0x20000) == 0x1001 && mp.ram_tag(0x20003) == 0x1004, "unit_load_store_moves_the_tag");
  // A DMA to the SPU consumes it.
  mp.note_dma(4, true, 0x20000, 4, 1, cycle);
  // A DMA that writes RAM clears tags.
  step(SW(kT1, 4, kT2));
  mp.note_dma(3, false, 0x20004, 4, 1, cycle);
  check(mp.ram_tag(0x20004) == 0 && mp.ram_tag(0x20000) == 0x1001, "unit_dma_to_ram_clears_tags");
  // Cache isolation: no tag change.
  step(SW(kT1, 8, kT2), 0x10000u);
  check(mp.ram_tag(0x20008) == 0, "unit_isolated_store_keeps_tags");
  // ALU clears, a move keeps.
  step(ADDIU(kT3, kT1, 4));
  step(SW(kT3, 12, kT2));
  step(ADDU(kT4, kT1, kZero)); // move
  step(SW(kT4, 16, kT2));
  check(mp.ram_tag(0x2000C) == 0 && mp.ram_tag(0x20010) == 0x1001, "unit_alu_clears_move_keeps");
  // A tagged register written to a device register (SPU) is a consumer.
  gpr[kT5] = 0x1F801DA8u;
  step(SW(kT1, 0, kT5));
  const GrimBootMap m = mp.finish(1, "unit", 0, cycle);
  const GrimMapWord &w = m.words[0x1000 / 4];
  check((w.consumer & kGrimConsumerSpu) != 0 && w.cls == GrimWordClass::Data && w.first_read == 100,
        "unit_spu_consumer_marks_data_and_first_read", grim_consumer_name(w.consumer));
}

void test_spu_provenance(const std::filesystem::path &dir) {
  GrimBootMapper mp(psx::BIOS_SIZE);
  u32 gpr[32] = {};
  u64 cycle = 100;
  auto step = [&](u32 instr) {
    mp.begin_instruction(cycle, 0x80000000u, instr, gpr, 0);
    mp.commit_instruction();
    cycle += 2;
  };
  auto write16 = [&](u32 off, u16 value, u32 rt = kT6) {
    gpr[kT5] = 0x1F801C00u + off;
    gpr[rt] = value;
    step(SH(rt, 0, kT5));
  };
  // Import two contiguous blocks via the CPU load-delay/word-copy rules.
  gpr[kT2] = 0x80020000u;
  for (u32 i = 0; i < 32u; i += 4u) {
    gpr[kT0] = 0xBFC01000u + i;
    step(LW(kT1, 0, kT0));
    step(0);
    step(SW(kT1, static_cast<s32>(i), kT2));
  }
  write16(0x6u, 0xFFFFu); // start is written before upload
  write16(0xEu, 0u);     // zero repeat falls back to the start at key-on
  write16(0x4u, 0x2000u);
  write16(0x1A6u, 0xFFFFu);
  mp.note_dma(4, true, 0x20000u, 4, 4, cycle);
  bool wrapped = true;
  for (u32 i = 0; i < 16u; ++i) {
    wrapped = wrapped && mp.spu_tag(0x7FFF8u + i) == 0x1001u + i;
  }
  check(wrapped, "unit_spu_dma_provenance_wraps_at_512kb");
  write16(0x1AAu, 0x8000u);
  write16(0x188u, 1u);
  GrimBootMap m = mp.finish(1, "spu_unit", 0, cycle);
  check(m.spu_sample_uses.size() == 5u &&
            m.spu_sample_uses[0].rom_offset == kGrimNoRomOffset &&
            m.spu_sample_uses[0].kind == GrimSpuSampleUseKind::StartWrite &&
            m.spu_sample_uses[3].kind == GrimSpuSampleUseKind::KeyOnStart &&
            m.spu_sample_uses[3].rom_offset == 0x1000u &&
            m.spu_sample_uses[4].rom_offset == 0x1000u,
        "unit_spu_deferred_start_resolves_on_keyon_and_zero_repeat_falls_back");
  check(m.spu_sample_uses[2].kind == GrimSpuSampleUseKind::PitchWrite &&
            m.spu_sample_uses[2].pitch == 0x2000u &&
            m.spu_sample_uses[3].pitch == 0x2000u && m.spu_sample_uses[4].pitch == 0x2000u,
        "unit_spu_pitch_write_and_keyon_snapshot");
  write16(0x4u, 0x0800u);
  write16(0x188u, 1u);
  m = mp.finish(1, "spu_unit", 0, cycle);
  check(m.spu_sample_uses.back().pitch == 0x0800u && m.spu_sample_uses[3].pitch == 0x2000u,
        "unit_spu_pitch_changes_do_not_rewrite_old_keyons");
  // Both DMA directions move the SPU pointer; incoming DMA retains the Phase 3
  // rule that device writes clear RAM tags.
  mp.note_dma(4, false, 0x20010u, 4, 1, cycle);
  write16(0x1A8u, 0x1234u, kT1);
  check(mp.spu_tag(0xCu) == 0x101Du && mp.spu_tag(0xDu) == 0x101Eu &&
            mp.ram_tag(0x20010u) == 0u,
        "unit_spu_read_dma_advances_transfer_pointer");
  // SW to the IRQ/transfer address pair writes low then high half, just as the bus.
  gpr[kT5] = 0x1F801DA4u;
  gpr[kT6] = 0x00200000u;
  step(SW(kT6, 0, kT5));
  // A load in flight must not tag the immediately following FIFO store with its
  // new origin. Only the subsequent instruction sees the new tags.
  gpr[kT0] = 0xBFC01100u;
  step(LHU(kT1, 0, kT0));
  write16(0x1A8u, 0x5678u, kT1);
  write16(0x1A8u, 0x9ABCu, kT1);
  check(mp.spu_tag(0x100u) == 0x101Du && mp.spu_tag(0x101u) == 0x101Eu &&
            mp.spu_tag(0x102u) == 0x1101u && mp.spu_tag(0x103u) == 0x1102u,
        "unit_spu_fifo_tracks_halfwords_and_load_delay");
  // A byte store is unsupported by System's SPU bus path and must not advance it.
  gpr[kT5] = 0x1F801DA8u;
  step(SB(kT1, 0, kT5));
  write16(0x1A8u, 0x9ABCu, kT1);
  check(mp.spu_tag(0x104u) == 0x1101u && mp.spu_tag(0x106u) == 0u,
        "unit_spu_unsupported_byte_store_does_not_advance_fifo");
  // Repeat points and high-lane key-ons are attributed to the correct voice.
  write16(0x106u, 0xFFFFu);
  write16(0x10Eu, 0xFFFFu);
  write16(0x104u, 0x1234u);
  write16(0x18Au, 1u);
  m = mp.finish(1, "spu_unit", 0, cycle);
  const auto &last = m.spu_sample_uses.back();
  check(last.voice == 16u && last.kind == GrimSpuSampleUseKind::KeyOnRepeat &&
            last.rom_offset == 0x1000u && last.spu_address == 0x7FFF8u && last.pitch == 0x1234u,
        "unit_spu_high_voice_and_repeat_address_resolve");
  // Untagged writes invalidate origin; a partly overwritten block is unresolved.
  write16(0x1A6u, 0xFFFFu);
  write16(0x1A8u, 0u);
  write16(0x188u, 1u);
  m = mp.finish(1, "spu_unit", 0, cycle);
  check(m.spu_sample_uses[m.spu_sample_uses.size() - 2u].rom_offset == kGrimNoRomOffset,
        "unit_spu_partial_overwrite_clears_sample_origin");
  std::string err;
  GrimBootMap loaded;
  const auto p = dir / "spu_events.json", p2 = dir / "spu_events_again.json";
  const bool saved = grim_map_save(m, p.string(), err) && grim_map_load(p.string(), loaded, err) &&
                     grim_map_save(loaded, p2.string(), err);
  check(saved && loaded.hash() == m.hash() && read_file(p) == read_file(p2),
        "unit_spu_event_json_roundtrip_is_exact", err);
  const GrimBootMap merged = grim_map_merge(m, m);
  check(merged.spu_sample_uses.size() == m.spu_sample_uses.size(),
        "unit_map_merge_deduplicates_identical_spu_events");
  GrimBootMap legacy = m;
  legacy.spu_sample_uses.erase(std::remove_if(legacy.spu_sample_uses.begin(), legacy.spu_sample_uses.end(),
      [](const GrimSpuSampleUse &u) { return u.kind == GrimSpuSampleUseKind::PitchWrite; }), legacy.spu_sample_uses.end());
  for (auto &u : legacy.spu_sample_uses) u.pitch = kGrimNoPitch;
  const auto legacy_path = dir / "legacy_spu_events.json", legacy_again = dir / "legacy_spu_events_again.json";
  const bool legacy_saved = grim_map_save(legacy, legacy_path.string(), err) &&
      grim_map_load(legacy_path.string(), loaded, err) && grim_map_save(loaded, legacy_again.string(), err);
  check(legacy_saved && loaded.hash() == legacy.hash() && read_file(legacy_path) == read_file(legacy_again) &&
            read_file(legacy_path).find("\"pitch\"") == std::string::npos,
        "unit_legacy_spu_events_keep_optional_pitch_absent", err);
  GrimBootMap different_pitch = legacy;
  different_pitch.spu_sample_uses.back().pitch = 0;
  check(different_pitch.hash() != legacy.hash(), "unit_zero_pitch_hash_differs_from_absent_pitch");
  GrimBootMap legacy_hash;
  legacy_hash.bios_hash = 0x1234u;
  legacy_hash.scenario = "legacy";
  legacy_hash.frames = 2;
  legacy_hash.cycles = 100;
  legacy_hash.words.resize(1);
  legacy_hash.spu_sample_uses.push_back({9u, 16u, 32u, 3u, GrimSpuSampleUseKind::KeyOnStart});
  check(legacy_hash.hash() == 0x4DEB111AC42A3451ull,
        "unit_legacy_spu_event_hash_keeps_phase4_bytes");
  // Duration is the median of reciprocals, especially for an even key-on
  // count. Pitch/repeat writes must not count as extra playback requests.
  std::vector<u32> adpcm(32u, 0x12345678u);
  for (u32 i = 0; i < 32u; i += 4u) adpcm[i] = 0x1234000Cu;
  adpcm[28u] |= 0x100u;
  const auto candidates = grim_scan_adpcm(adpcm);
  GrimBootMap duration_map;
  duration_map.words.resize(adpcm.size());
  duration_map.spu_sample_uses = {
      {1u, 0u, 0u, 0u, GrimSpuSampleUseKind::KeyOnStart, 0x1000u},
      {2u, 0u, 0u, 0u, GrimSpuSampleUseKind::PitchWrite, 1u},
      {3u, 0u, 0u, 0u, GrimSpuSampleUseKind::KeyOnRepeat, 1u},
      {4u, 0u, 0u, 0u, GrimSpuSampleUseKind::KeyOnStart, 0x4000u},
      {5u, 0u, 0u, 0u, GrimSpuSampleUseKind::KeyOnStart, 0u}};
  auto durations = grim_sample_annotate(adpcm, candidates, duration_map);
  check(durations.size() == 1u && durations[0].pitch_key_on_count == 2u &&
            durations[0].zero_pitch_key_on_count == 1u &&
            durations[0].median_key_on_pitch == 10240.0 &&
            durations[0].block_duration_numerator == 25u && durations[0].block_duration_denominator == 63u &&
            durations[0].loop_start_offsets.size() == 1u,
        "unit_sample_duration_uses_even_median_and_ignores_pitch_repeat_writes");
  duration_map.spu_sample_uses.push_back({6u, 0u, 0u, 0u, GrimSpuSampleUseKind::KeyOnStart, 0x2000u});
  durations = grim_sample_annotate(adpcm, candidates, duration_map);
  check(durations.size() == 1u && durations[0].pitch_key_on_count == 3u &&
            durations[0].median_key_on_pitch == 8192.0 &&
            durations[0].block_duration_numerator == 20u && durations[0].block_duration_denominator == 63u,
        "unit_sample_duration_uses_odd_median");
  for (auto &u : duration_map.spu_sample_uses) u.pitch = kGrimNoPitch;
  durations = grim_sample_annotate(adpcm, candidates, duration_map);
  const auto unplayed = grim_sample_annotate(adpcm, candidates, GrimBootMap{});
  check(durations.size() == 1u && unplayed.size() == 1u &&
            durations[0].pitch_key_on_count == 0u && durations[0].median_key_on_pitch == 4096.0 &&
            durations[0].block_duration_numerator == 40u && durations[0].block_duration_denominator == 63u &&
            unplayed[0].block_duration_numerator == 40u && unplayed[0].block_duration_denominator == 63u,
        "unit_sample_duration_legacy_and_unplayed_use_base_pitch");
  GrimBootMap dormant;
  dormant.words.resize(8);
  dormant.words[2].flags = dormant.words[3].flags = kGrimReadDirect;
  dormant.words[6].flags = kGrimReadViaRam;
  const auto ranges = grim_map_dormant_regions(dormant);
  check(dormant.dormant_words() == 3u && ranges.size() == 2u &&
            ranges[0] == std::make_pair(8u, 16u) && ranges[1] == std::make_pair(24u, 28u),
        "unit_map_dormant_is_derived_without_changing_classes");

  // Execute a real telemetry-loop boot as well: register values must come from
  // the pre-instruction GPR snapshot, and FIFO tags from delayed LHU results.
  std::vector<u32> rom(psx::BIOS_SIZE / 4u, 0u);
  rom[0x1000u / 4u] = 0x9876000Cu;
  rom[0x1004u / 4u] = 0x76543210u;
  rom[0x1008u / 4u] = 0xFEDCBA98u;
  rom[0x100Cu / 4u] = 0x12345678u;
  Asm a;
  auto sh_constant = [&](u32 off, u32 value) {
    a.li(kT5, 0x1F801C00u + off);
    a.li(kT6, value);
    a.emit(SH(kT6, 0, kT5));
  };
  sh_constant(0x6u, 0x200u);
  sh_constant(0xEu, 0u);
  sh_constant(0x4u, 0x1000u);
  sh_constant(0x1A6u, 0x200u);
  a.li(kT0, 0xBFC01000u);
  a.li(kT5, 0x1F801DA8u);
  for (u32 i = 0; i < 16u; i += 2u) {
    a.emit(LHU(kT1, static_cast<s32>(i), kT0));
    a.emit(0);
    a.emit(SH(kT1, 0, kT5));
  }
  sh_constant(0x1AAu, 0x8000u);
  for (u32 i = 0; i < 64u; ++i) a.emit(0);
  sh_constant(0x188u, 1u);
  a.emit(BEQ(kZero, kZero, -1));
  a.emit(0);
  std::copy(a.w.begin(), a.w.end(), rom.begin());
  const auto bios = dir / "spu_provenance_synthetic.bin";
  const bool written = write_rom(bios, rom);
  SynthRun boot;
  if (written) boot = run_synthetic(bios);
  bool resolved = false;
  if (boot.ok) {
    for (const GrimSpuSampleUse &u : boot.map.spu_sample_uses) {
      resolved = resolved || (u.voice == 0u && u.kind == GrimSpuSampleUseKind::KeyOnStart &&
                              u.spu_address == 0x1000u && u.rom_offset == 0x1000u && u.pitch == 0x1000u);
    }
  }
  check(resolved, "synthetic_spu_fifo_boot_resolves_voice_to_rom");
  GrimEvalConfig input_cfg = eval_cfg(bios.string(), 3);
  const GrimEvalResult no_input = run_grim_eval(input_cfg);
  input_cfg.scripted_buttons = {{1u, 0xFFFFu}, {2u, 0xFFFFu}};
  const GrimEvalResult released = run_grim_eval(input_cfg);
  check(no_input.run_hash == released.run_hash && no_input.frames.size() == released.frames.size(),
        "synthetic_released_scripted_input_keeps_clean_telemetry");
  input_cfg.scripted_buttons = {{2u, 0xFFFFu}, {1u, 0xFFFFu}};
  check(run_grim_eval(input_cfg).end_reason == "invalid_scripted_buttons",
        "unit_scripted_input_rejects_unsorted_frames");
}

// ---- real BIOS ------------------------------------------------------------------------------------

struct StockMap {
  GrimRomContext ctx;
  GrimBootMap map;
  bool ok = false;
};

void build_context(const std::string &bios, const GrimBootMap &map, GrimRomContext &ctx) {
  Bios b;
  b.load(bios);
  ctx.map = map;
  ctx.bios_hash = b.image_hash();
  ctx.words.assign(map.words.size(), 0);
  for (u32 i = 0; i < ctx.words.size(); ++i) {
    b.original_word(i * 4u, ctx.words[i]);
  }
  ctx.init();
}

GrimBootMap run_map(const std::string &bios, u32 frames, u64 *run_hash = nullptr) {
  GrimEvalConfig c = eval_cfg(bios, frames);
  GrimBootMap map;
  c.boot_map_out = &map;
  const GrimEvalResult r = run_grim_eval(c);
  if (run_hash != nullptr) {
    *run_hash = r.run_hash;
  }
  return map;
}

void test_map_determinism(const std::string &bios, const std::string &self_exe,
                          const std::filesystem::path &dir, const GrimBootMap &reference,
                          u32 frames) {
  // Threads.
  GrimBootMap t1, t2;
  std::thread a([&] { t1 = run_map(bios, frames); });
  std::thread b([&] { t2 = run_map(bios, frames); });
  a.join();
  b.join();
  check(t1.hash() == reference.hash() && t2.hash() == reference.hash(), "map_deterministic_across_threads",
        hex(reference.hash()));
  // Processes: the files must be byte-identical to the in-process map.
  std::string err;
  grim_map_save(reference, (dir / "ref.json").string(), err);
  const std::string exe = grim_self_exe_path(self_exe);
  GrimChildResult ca, cb;
  std::thread pa([&] {
    ca = grim_run_child(exe, {"--grim-map", bios, std::to_string(frames), (dir / "pa.json").string()},
                        (dir / "pa.log").string(), 600.0);
  });
  std::thread pb([&] {
    cb = grim_run_child(exe, {"--grim-map", bios, std::to_string(frames), (dir / "pb.json").string()},
                        (dir / "pb.log").string(), 600.0);
  });
  pa.join();
  pb.join();
  const bool same = ca.exit_code == 0 && cb.exit_code == 0 &&
                    read_file(dir / "pa.json") == read_file(dir / "ref.json") &&
                    read_file(dir / "pb.json") == read_file(dir / "ref.json") &&
                    read_file(dir / "pa.json.words") == read_file(dir / "ref.json.words") &&
                    read_file(dir / "pb.json.words") == read_file(dir / "ref.json.words");
  check(same, "map_deterministic_across_processes");
}

void test_map_sanity(const GrimBootMap &m) {
  check(m.provenance_permille() >= 900, "stock_map_provenance_coverage",
        std::to_string(m.provenance_permille() / 10.0) + "% of " + std::to_string(m.ram_exec_words) +
            " executed RAM words");
  check(m.count(GrimWordClass::Code) > 10000 && m.count(GrimWordClass::Data) > 1000 &&
            m.count(GrimWordClass::Unused) > 10000,
        "stock_map_has_code_data_and_unused",
        "code=" + std::to_string(m.count(GrimWordClass::Code)) +
            " data=" + std::to_string(m.count(GrimWordClass::Data)) +
            " unused=" + std::to_string(m.count(GrimWordClass::Unused)));
  u32 spu_words = 0;
  for (const GrimMapWord &w : m.words) {
    spu_words += (w.consumer & kGrimConsumerSpu) != 0 ? 1u : 0u;
  }
  check(spu_words > 1000, "stock_map_finds_words_that_reach_the_spu", std::to_string(spu_words));
}

// ---- mutation fuzz ------------------------------------------------------------------------------

int reg_field_diff_count(u32 a, u32 b) {
  int n = 0;
  for (u32 sh : {11u, 16u, 21u}) {
    n += ((a >> sh) & 31u) != ((b >> sh) & 31u) ? 1 : 0;
  }
  return n;
}

bool in_set(u32 v, std::initializer_list<u32> s) {
  return std::find(s.begin(), s.end(), v) != s.end();
}

void test_mutation_fuzz(const GrimRomContext &ctx) {
  std::vector<u32> code;
  for (u32 i = 0; i < ctx.words.size(); ++i) {
    if (ctx.is_code(i)) {
      code.push_back(i);
    }
  }
  for (u32 k = 0; k < static_cast<u32>(GrimRomMut::Count); ++k) {
    const GrimRomMut kind = static_cast<GrimRomMut>(k);
    GrimRng rng{1000 + k};
    u32 done = 0, tries = 0;
    std::string fail;
    while (done < 3000 && tries < 400000 && fail.empty()) {
      ++tries;
      const u32 idx = code[rng.range(0, static_cast<u32>(code.size()) - 1u)];
      std::vector<GrimRomEdit> edits;
      const bool applicable = grim_rom_applicable(kind, ctx, idx);
      if (!grim_rom_mutate(kind, rng, ctx, idx, edits)) {
        continue;
      }
      if (!applicable) {
        fail = "mutated a word that was not applicable";
      }
      ++done;
      const u32 w = ctx.words[idx];
      const u32 n = edits.empty() ? 0 : edits[0].word;
      if (edits.empty() || (kind != GrimRomMut::LuiOriConst && edits[0].index != idx)) {
        fail = "no edit for the requested word";
        break;
      }
      for (const GrimRomEdit &e : edits) {
        if (!grim_mips_valid(e.word) || e.word == ctx.words[e.index] || !ctx.is_code(e.index)) {
          fail = "invalid, unchanged or non-code word " + hex(e.word) + " from " + hex(ctx.words[e.index]);
        }
      }
      if (!fail.empty()) {
        break;
      }
      const u32 op = w >> 26, nop_ = n >> 26;
      switch (kind) {
      case GrimRomMut::ImmPerturb:
        if ((w & 0xFFFF0000u) != (n & 0xFFFF0000u)) fail = "imm: upper half changed";
        break;
      case GrimRomMut::RegSubst: {
        const u32 changed = w ^ n;
        u32 shift = 99;
        for (u32 sh : {11u, 16u, 21u}) {
          if (((changed >> sh) & 31u) != 0) shift = sh;
        }
        const u32 from = (w >> shift) & 31u, to = (n >> shift) & 31u;
        auto bad = [&](u32 r) { return r == 26 || r == 27 || r == 29 || r == 31 || (r == 28 && ctx.avoid_gp); };
        if (shift == 99 || reg_field_diff_count(w, n) != 1 || (changed & ~(31u << shift)) != 0 || bad(from) || bad(to)) {
          fail = "reg: more than one field, other bits, or an avoided register";
        }
        break;
      }
      case GrimRomMut::BranchInvert: {
        const bool pair = (op == 4 && nop_ == 5) || (op == 5 && nop_ == 4) || (op == 6 && nop_ == 7) ||
                          (op == 7 && nop_ == 6);
        const bool regimm = op == 1 && nop_ == 1 && ((w ^ n) == (1u << 16));
        if (!(pair && (w & 0x03FFFFFFu) == (n & 0x03FFFFFFu)) && !regimm) fail = "branch_invert: not an inverse pair";
        break;
      }
      case GrimRomMut::BranchNudge: {
        const s32 d = static_cast<s16>(n & 0xFFFFu) - static_cast<s16>(w & 0xFFFFu);
        const s64 t = static_cast<s64>(idx) + 1 + static_cast<s16>(n & 0xFFFFu);
        if ((w & 0xFFFF0000u) != (n & 0xFFFF0000u) || d == 0 || d < -4 || d > 4 || t < 0 ||
            t >= static_cast<s64>(ctx.words.size()) || !ctx.is_code(static_cast<u32>(t)) || t == idx) {
          fail = "branch_nudge: target not aligned executed code, or delta out of range";
        }
        break;
      }
      case GrimRomMut::AluSubst: {
        const u32 f = w & 63u, nf = n & 63u;
        const bool same_set = (in_set(f, {0, 2, 3}) && in_set(nf, {0, 2, 3})) ||
                              (in_set(f, {4, 6, 7}) && in_set(nf, {4, 6, 7})) ||
                              (in_set(f, {0x21, 0x23, 0x24, 0x25, 0x26}) &&
                               in_set(nf, {0x21, 0x23, 0x24, 0x25, 0x26}));
        if (!same_set || (w & ~63u) != (n & ~63u)) fail = "alu: left its set or changed other bits";
        break;
      }
      case GrimRomMut::LsWidth: {
        auto width = [](u32 o) { return in_set(o, {0x20, 0x24, 0x28}) ? 1 : in_set(o, {0x21, 0x25, 0x29}) ? 2 : 4; };
        const bool loads = in_set(op, {0x20, 0x21, 0x23, 0x24, 0x25}) && in_set(nop_, {0x20, 0x21, 0x23, 0x24, 0x25});
        const bool stores = in_set(op, {0x28, 0x29, 0x2B}) && in_set(nop_, {0x28, 0x29, 0x2B});
        const s32 off = static_cast<s16>(n & 0xFFFFu);
        if (!(loads || stores) || (w & 0x03FFFFFFu) != (n & 0x03FFFFFFu) || (off % width(nop_)) != 0) {
          fail = "ls_width: wrong family, other bits, or misaligned offset for the new width";
        }
        break;
      }
      case GrimRomMut::Nop:
        if (n != 0) fail = "nop: not zero";
        break;
      case GrimRomMut::LuiOriConst: {
        // Only changed words are listed (the LUI and/or its ORI/ADDIU partner); each keeps
        // its opcode, rs and rt.
        bool pair_ok = edits.size() <= 2;
        for (const GrimRomEdit &e : edits) {
          const u32 orig = ctx.words[e.index];
          pair_ok = pair_ok && (e.word >> 16) == (orig >> 16);
        }
        if (!pair_ok) {
          fail = "lui_ori: pair not re-encoded consistently";
        }
        break;
      }
      case GrimRomMut::CallSwap:
        if (op != 3 || nop_ != 3 ||
            !std::binary_search(ctx.jal_targets.begin(), ctx.jal_targets.end(), n & 0x03FFFFFFu)) {
          fail = "call_swap: target is not a traced call target";
        }
        break;
      default:
        break;
      }
    }
    check(fail.empty() && done >= 300, std::string("fuzz_") + grim_rom_mut_name(kind),
          fail.empty() ? std::to_string(done) + " mutations" : fail);
  }
}

// Does the generator keep to its rules, and is it reproducible?
void test_generator(const GrimRomContext &ctx) {
  const u32 early_ms = 300;
  const GrimGene a = grim_rom_generate(ctx, 42, GrimRomMut::ImmPerturb, 200, early_ms, 2);
  const GrimGene b = grim_rom_generate(ctx, 42, GrimRomMut::ImmPerturb, 200, early_ms, 2);
  check(a.patches.size() == 200 && grim_genome_hash(GrimGenome{2, ctx.bios_hash, {a}}) ==
                                       grim_genome_hash(GrimGenome{2, ctx.bios_hash, {b}}),
        "generator_reproducible", std::to_string(a.patches.size()) + " patches");
  const u64 early = u64{early_ms} * (psx::CPU_CLOCK_HZ / 1000u);
  bool rules = true;
  for (const GrimRomPatch &p : a.patches) {
    const u32 i = p.offset / 4u;
    rules = rules && ctx.is_code(i) && ctx.map.words[i].first_exec >= early && p.original == ctx.words[i] &&
            p.delay_slot == ctx.in_delay_slot(i) && grim_mips_valid(p.mutated);
  }
  check(rules, "generator_targets_code_after_the_early_window_with_true_originals");
  auto mean_exec = [&](u32 curve) {
    const GrimGene g = grim_rom_generate(ctx, 7, GrimRomMut::ImmPerturb, 400, 0, curve);
    double sum = 0;
    for (const GrimRomPatch &p : g.patches) {
      sum += static_cast<double>(ctx.map.words[p.offset / 4u].first_exec);
    }
    return g.patches.empty() ? 0.0 : sum / static_cast<double>(g.patches.size());
  };
  const double m0 = mean_exec(0), m3 = mean_exec(3);
  check(m3 > m0 * 1.15, "generator_curve_prefers_later_code",
        "mean first exec uniform=" + std::to_string(grim_cycles_to_ms(static_cast<u64>(m0))) +
            "ms cubic=" + std::to_string(grim_cycles_to_ms(static_cast<u64>(m3))) + "ms");
  // A delay-slot patch is marked.
  bool any_delay = false;
  const GrimGene many = grim_rom_generate(ctx, 9, GrimRomMut::Nop, 3000, 0, 0);
  for (const GrimRomPatch &p : many.patches) {
    any_delay = any_delay || p.delay_slot;
  }
  check(any_delay, "generator_marks_delay_slot_patches");
}

void test_disassembler() {
  check(grim_mips_disasm(BEQ(kA0, kZero, 3), 0xBFC00000u) == "beq a0,zero,+12" &&
            grim_mips_disasm(I(5, kA0, kZero, 3), 0) == "bne a0,zero,+12" &&
            grim_mips_disasm(LW(kT0, 32, 29), 0) == "lw t0,32(sp)" && grim_mips_disasm(0, 0) == "nop" &&
            grim_mips_disasm(JR(kRa), 0) == "jr ra",
        "disassembler_basics");
}

// ---- genome format ------------------------------------------------------------------------------

GrimGenome sample_rom_genome(u64 bios_hash) {
  GrimGenome g;
  g.version = 2;
  g.bios_hash = bios_hash;
  GrimGene gene;
  gene.type = GrimGeneType::RomCode;
  gene.target = 1;
  gene.seed = 99;
  gene.params = {static_cast<s32>(GrimRomMut::BranchInvert), 2, 200, 2, 0, 0, 0, 0};
  gene.patches.push_back({0x1000, 0x10800003u, 0x14800003u, false});
  gene.patches.push_back({0x1010, 0x00000000u, 0x24020001u, true});
  g.genes.push_back(gene);
  GrimGene iface = grim_default_gene(GrimGeneType::SpuPitch);
  iface.seed = 5;
  g.genes.push_back(iface);
  return g;
}

void test_genome_format(const std::filesystem::path &dir) {
  const GrimGenome g = sample_rom_genome(0x32B1A0FA4DB70C8Full);
  const std::string text = grim_genome_serialize(g);
  GrimGenome parsed;
  std::string err;
  const bool parsed_ok = grim_genome_parse(text, parsed, err);
  check(parsed_ok && grim_genome_serialize(parsed) == text && grim_genome_hash(parsed) == grim_genome_hash(g) &&
            parsed.genes[0].patches.size() == 2 && parsed.genes[0].patches[1].delay_slot &&
            text.find("\"version\":2,\"bios_hash\":\"0x32B1A0FA4DB70C8F\"") != std::string::npos,
        "genome_v2_roundtrip_and_stable_hash", err);
  // Version 1 files still parse and serialize exactly as before.
  GrimGenome v1;
  const std::string v1_text =
      "{\"version\":1,\"genes\":[\n{\"type\":\"spu_pitch\",\"target\":16777215,\"seed\":101,\"trigger\":{\"kind\":\"always\"},"
      "\"params\":{\"mode\":1,\"mul_q8\":384,\"offset\":-700,\"scale\":2741,\"depth\":64,\"period\":120}}\n]}\n";
  check(grim_genome_parse(v1_text, v1, err) && v1.version == 1 && grim_genome_serialize(v1) == v1_text,
        "genome_v1_still_parses_and_serializes_unchanged", err);
  const std::filesystem::path sample = "docs/grim-reaper/genomes/mix_everything.json";
  if (std::filesystem::exists(sample)) {
    GrimGenome mix;
    check(grim_genome_load(sample.string(), mix, err) && grim_genome_hash(mix) == 0xCEAE5586B9E3907Full,
          "phase2_sample_genome_hash_is_unchanged", err);
  }
  // Strictness.
  auto rejects = [&](const std::string &name, std::string bad) {
    GrimGenome out;
    std::string e;
    check(!grim_genome_parse(bad, out, e), name, e);
  };
  std::string v1_with_rom = text;
  v1_with_rom.replace(v1_with_rom.find("\"version\":2,\"bios_hash\":\"0x32B1A0FA4DB70C8F\""), 45, "\"version\":1");
  rejects("genome_rejects_rom_gene_in_v1", v1_with_rom);
  std::string no_hash = text;
  no_hash.replace(no_hash.find(",\"bios_hash\":\"0x32B1A0FA4DB70C8F\""), 33, "");
  rejects("genome_v2_requires_bios_hash", no_hash);
  std::string odd = text;
  odd.replace(odd.find("[4096,"), 6, "[4098,");
  rejects("genome_rejects_unaligned_patch_offset", odd);
  std::string range = text;
  range.replace(range.find("\"kind\":2"), 8, "\"kind\":9");
  rejects("genome_rejects_out_of_range_rom_param", range);
  std::string extra = text;
  extra.replace(extra.find("\"patches\""), 9, "\"patchez\"");
  rejects("genome_rejects_unknown_rom_field", extra);
  (void)dir;
}

// ---- ROM genes end to end ------------------------------------------------------------------------

GrimGene manual_rom_gene(const GrimRomContext &ctx, GrimRomMut kind, const std::vector<GrimRomEdit> &edits) {
  GrimGene g;
  g.type = GrimGeneType::RomCode;
  g.target = 1;
  g.seed = 1;
  g.params = {static_cast<s32>(kind), static_cast<s32>(edits.size()), 0, 0, 0, 0, 0, 0};
  for (const GrimRomEdit &e : edits) {
    g.patches.push_back({e.index * 4u, ctx.words[e.index], e.word, ctx.in_delay_slot(e.index)});
  }
  return g;
}

void test_rom_genes_end_to_end(const std::string &bios, const std::string &self_exe,
                               const std::filesystem::path &dir, const GrimRomContext &ctx) {
  // A large block of early init code NOPed out: the machine dies.
  std::vector<GrimRomEdit> early;
  for (u32 i = 0; i < ctx.words.size(); ++i) {
    if (ctx.is_code(i) && i * 4u < 0x45C && ctx.words[i] != 0) {
      early.push_back({i, 0});
    }
  }
  GrimGenome dead_genome;
  dead_genome.version = 2;
  dead_genome.bios_hash = ctx.bios_hash;
  dead_genome.genes.push_back(manual_rom_gene(ctx, GrimRomMut::Nop, early));
  GrimEvalConfig c = eval_cfg(bios, 900);
  c.stop_on_death = true;
  c.use_genome = true;
  c.genome = dead_genome;
  const GrimEvalResult dead = run_grim_eval(c);
  check(!dead.liveness.alive, "e2e_nop_early_init_block_is_dead",
        std::to_string(early.size()) + " words -> " + dead.liveness.reason + " at frame " +
            std::to_string(dead.liveness.death_frame));

  // Small immediate perturbations (+1) in late Shell code (first executed after 9 s): every
  // one stays alive, and at least one changes the telemetry (so the patch really acts).
  std::vector<u32> late_words;
  for (u32 i = 0; i < ctx.words.size(); ++i) {
    const u32 w = ctx.words[i];
    if (ctx.is_code(i) && ctx.map.words[i].first_exec > 9000ull * (psx::CPU_CLOCK_HZ / 1000u) &&
        (w >> 26) == 9 && ((w >> 16) & 31u) != 29 && ((w >> 21) & 31u) != 29 && (w & 0xFFFFu) != 0 &&
        (w & 0xFFFFu) < 0x7000u) {
      late_words.push_back(i);
    }
  }
  check(late_words.size() >= 6, "e2e_found_late_addiu_words", std::to_string(late_words.size()));
  const GrimEvalResult base = run_grim_eval(eval_cfg(bios, 900));
  GrimGenome late_genome;
  late_genome.version = 2;
  late_genome.bios_hash = ctx.bios_hash;
  u32 alive_n = 0, changed_n = 0, tried = 0;
  std::string detail;
  const size_t stride = std::max<size_t>(1, late_words.size() / 6);
  for (size_t k = 0; k < late_words.size() && tried < 6; k += stride) {
    const u32 late = late_words[k];
    GrimGenome g;
    g.version = 2;
    g.bios_hash = ctx.bios_hash;
    g.genes.push_back(manual_rom_gene(ctx, GrimRomMut::ImmPerturb, {{late, ctx.words[late] + 1}}));
    c.genome = g;
    const GrimEvalResult r = run_grim_eval(c);
    ++tried;
    const bool alive = r.liveness.alive && r.frames.size() == 900 && r.gene_hits.size() == 1 && r.gene_hits[0] == 1;
    const bool changed = r.run_hash != base.run_hash;
    alive_n += alive ? 1u : 0u;
    changed_n += changed ? 1u : 0u;
    if (late_genome.genes.empty() && alive && changed) {
      late_genome = g;
    }
    detail += " " + hex(late * 4u) + (alive ? ":alive" : ":" + r.liveness.reason) + (changed ? "+changed" : "+same");
  }
  check(tried == 6 && alive_n * 2 > tried && changed_n >= 1 && !late_genome.genes.empty(),
        "e2e_small_late_perturbations_mostly_stay_alive",
        std::to_string(alive_n) + "/" + std::to_string(tried) + " alive, " + std::to_string(changed_n) +
            " changed the run;" + detail);
  if (late_genome.genes.empty()) {
    const u32 late = late_words.front();
    late_genome.genes.push_back(manual_rom_gene(ctx, GrimRomMut::ImmPerturb, {{late, ctx.words[late] + 1}}));
  }
  c.genome = late_genome;

  // The patches are in the image from reset on, and survive a reboot.
  {
    auto sys = std::make_unique<System>();
    sys->load_bios(bios);
    sys->reset();
    GrimGenomeRuntime rt(late_genome);
    const GrimRomPatch &p = late_genome.genes[0].patches[0];
    sys->set_grim_genome(&rt);
    const bool applied = sys->bios_mut().read32(p.offset) == p.mutated && sys->grim_rom_error().empty();
    sys->reset();
    const bool after_reboot = sys->bios_mut().read32(p.offset) == p.mutated;
    sys->set_grim_genome(nullptr);
    sys->reset();
    const bool stock_again = sys->bios_mut().read32(p.offset) == p.original;
    check(applied && after_reboot && stock_again, "rom_genes_are_in_the_image_from_reset_and_after_reboot");
  }

  // Wrong original word: refused, nothing patched.
  GrimGenome wrong = late_genome;
  if (!wrong.genes.empty()) {
    wrong.genes[0].patches[0].original ^= 0x00010000u;
    c.genome = wrong;
    const GrimEvalResult r = run_grim_eval(c);
    check(r.end_reason == "rom_gene_mismatch" && r.error_detail.find("expected original word") != std::string::npos,
          "rom_gene_refuses_wrong_original_word", r.error_detail);
  }
  // A modified BIOS file (one byte changed): refused by the BIOS hash.
  std::filesystem::path modified = dir / "modified.bin";
  {
    std::ifstream in(bios, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    data[0x20000] = static_cast<char>(data[0x20000] ^ 0x01);
    std::ofstream out(modified, std::ios::binary | std::ios::trunc);
    out << data;
  }
  GrimEvalConfig cm = eval_cfg(modified.string(), 50);
  cm.use_genome = true;
  cm.genome = late_genome;
  const GrimEvalResult rm = run_grim_eval(cm);
  check(rm.end_reason == "rom_gene_mismatch" && rm.error_detail.find("BIOS 0x") != std::string::npos,
        "rom_gene_refuses_a_different_bios", rm.error_detail);

  // Composition: interface + ROM genes in one genome.
  GrimGenome mix = late_genome;
  GrimGene pitch = grim_default_gene(GrimGeneType::SpuPitch);
  pitch.seed = 77;
  GrimGene vert = grim_default_gene(GrimGeneType::GpuVertex);
  vert.seed = 78;
  mix.genes.insert(mix.genes.begin(), pitch);
  mix.genes.push_back(vert);
  const std::string text = grim_genome_serialize(mix);
  GrimGenome round;
  std::string err;
  check(grim_genome_parse(text, round, err) && grim_genome_serialize(round) == text &&
            grim_genome_hash(round) == grim_genome_hash(mix),
        "composition_roundtrip_and_stable_hash", err);
  const std::filesystem::path gpath = dir / "mix.json";
  {
    std::ofstream out(gpath, std::ios::binary);
    out << text;
  }
  GrimEvalConfig cx = eval_cfg(bios, 400);
  cx.use_genome = true;
  cx.genome = mix;
  const GrimEvalResult rx = run_grim_eval(cx);
  const bool hits_ok = rx.gene_hits.size() == 3 && rx.gene_hits[0] > 0 && rx.gene_hits[1] == 1 && rx.gene_hits[2] > 0;
  check(rx.end_reason == "frames" && hits_ok && rx.liveness.alive, "composition_rom_and_interface_genes_act_together",
        std::to_string(rx.gene_hits.size()) + " genes, reason=" + rx.liveness.reason);
  // Across processes.
  const std::string exe = grim_self_exe_path(self_exe);
  GrimChildResult ca, cb;
  std::thread pa([&] {
    ca = grim_run_child(exe, {"--grim-eval", bios, "400", (dir / "a.jsonl").string(), "--genome", gpath.string(), "--no-stop-on-death"},
                        (dir / "a.log").string(), 300.0);
  });
  std::thread pb([&] {
    cb = grim_run_child(exe, {"--grim-eval", bios, "400", (dir / "b.jsonl").string(), "--genome", gpath.string(), "--no-stop-on-death"},
                        (dir / "b.log").string(), 300.0);
  });
  pa.join();
  pb.join();
  auto run_hash_of = [](const std::string &log) {
    const size_t p = log.find("run_hash=");
    return p == std::string::npos ? std::string() : log.substr(p + 9, 18);
  };
  const std::string ha = run_hash_of(read_file(dir / "a.log")), hb = run_hash_of(read_file(dir / "b.log"));
  char self_hash[32];
  std::snprintf(self_hash, sizeof(self_hash), "0x%016llX", static_cast<unsigned long long>(rx.run_hash));
  auto frames_only = [](std::string text) { // the summary line carries wall-clock fields
    return text.substr(0, text.find("{\"summary\""));
  };
  check(!ha.empty() && ha == hb && ha == self_hash &&
            frames_only(read_file(dir / "a.jsonl")) == frames_only(read_file(dir / "b.jsonl")),
        "composition_deterministic_across_processes", ha + " " + hb + " in-process " + self_hash);

  // Interpreter and recompiler agree on a machine with ROM genes.
  GrimEvalConfig cp = eval_cfg(bios, 300);
  cp.use_genome = true;
  cp.genome = mix;
  cp.native_cpu = true;
  const bool saved_override = g_cpu_execution_mode_cli_override;
  const CpuExecutionMode saved_value = g_cpu_execution_mode_cli_value;
  g_cpu_execution_mode_cli_override = true;
  g_cpu_execution_mode_cli_value = CpuExecutionMode::Interpreter;
  const GrimEvalResult interp = run_grim_eval(cp);
  g_cpu_execution_mode_cli_value = CpuExecutionMode::Recompiler;
  const GrimEvalResult rec = run_grim_eval(cp);
  g_cpu_execution_mode_cli_override = saved_override;
  g_cpu_execution_mode_cli_value = saved_value;
  std::string diff;
  for (size_t i = 0; i < std::min(interp.frames.size(), rec.frames.size()) && diff.empty(); ++i) {
    const GrimFrameTelemetry &a = interp.frames[i], &b = rec.frames[i];
    if (a.fb_hash != b.fb_hash) diff = "fb";
    else if (a.audio_hash != b.audio_hash) diff = "audio_hash";
    else if (a.cycles != b.cycles) diff = "cycles";
    else if (a.spu_key_on != b.spu_key_on) diff = "key_on";
    if (!diff.empty()) diff += " at frame " + std::to_string(i);
  }
  check(diff.empty() && interp.frames.size() == 300 && rec.frames.size() == 300 && rec.gene_hits == interp.gene_hits,
        "rom_genes_match_between_interpreter_and_recompiler", diff);

  // A random mixed genome from the generator applies and runs.
  GrimRandomParams rp;
  rp.rom = &ctx;
  rp.horizon_frames = 300;
  const GrimGenome random = grim_random_genome(3, rp);
  check(grim_genome_has_rom(random) && random.version == 2 && random.bios_hash == ctx.bios_hash &&
            random.genes.size() > 1,
        "random_genome_mixes_both_families", std::to_string(random.genes.size()) + " genes");
  GrimRandomParams no_rom;
  no_rom.horizon_frames = 300;
  const GrimGenome plain_a = grim_random_genome(3, no_rom), plain_b = grim_random_genome(3, rp);
  bool prefix = plain_b.genes.size() >= plain_a.genes.size();
  for (size_t i = 0; prefix && i < plain_a.genes.size(); ++i) {
    prefix = grim_genome_serialize(GrimGenome{1, 0, {plain_a.genes[i]}}) ==
             grim_genome_serialize(GrimGenome{1, 0, {plain_b.genes[i]}});
  }
  check(prefix, "adding_rom_genes_does_not_change_the_interface_part_of_a_seed");
}

} // namespace

int run_grim_map_test(const std::vector<std::string> &raw_args, const std::string &self_exe) {
  grim_prepare_eval_process();
  std::vector<std::string> args = raw_args;
  const std::string bios = grim_take_bios_arg(args);
  const u32 frames = !args.empty() ? static_cast<u32>(std::max(1, std::atoi(args[0].c_str()))) : 900u;
  g_failures = 0;
  const std::filesystem::path dir = make_temp_dir();

  test_disassembler();
  test_genome_format(dir);
  test_mapper_unit();
  test_spu_provenance(dir);
  test_synthetic(dir);
  if (bios.empty()) {
    std::printf("GRIM_MAP_TEST SKIP bios tests (no BIOS path or VIBESTATION_BIOS)\n");
  } else {
    // Observation must not change emulation: identical telemetry with and without the mapper,
    // and the stock no-genome hash is the Phase 1 / Phase 2 value.
    u64 plain_hash = 0, mapped_hash = 0;
    const GrimEvalResult plain = run_grim_eval(eval_cfg(bios, 1800));
    plain_hash = plain.run_hash;
    check(plain_hash == 0x432E585CF1535F9Cull, "stock_run_hash_unchanged", hex(plain_hash));
    GrimBootMap ref = run_map(bios, frames, &mapped_hash);
    const GrimEvalResult plain_short = run_grim_eval(eval_cfg(bios, frames));
    check(mapped_hash == plain_short.run_hash, "mapper_does_not_change_emulation", hex(mapped_hash));
    test_map_sanity(ref);
    test_map_determinism(bios, self_exe, dir, ref, frames);
    GrimRomContext ctx;
    build_context(bios, ref, ctx);
    test_mutation_fuzz(ctx);
    test_generator(ctx);
    test_rom_genes_end_to_end(bios, self_exe, dir, ctx);
  }
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  std::printf("GRIM_MAP_TEST_RESULT %s failures=%d\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
  return g_failures == 0 ? 0 : 1;
}
