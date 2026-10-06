#include "grim_rom.h"
#include "grim_sample.h"
#include "bios.h"
#include <algorithm>
#include <cstdio>

// ---- decoding ---------------------------------------------------------------------

namespace {
inline u32 op_of(u32 w) { return w >> 26; }
inline u32 rs_of(u32 w) { return (w >> 21) & 31u; }
inline u32 rt_of(u32 w) { return (w >> 16) & 31u; }
inline u32 rd_of(u32 w) { return (w >> 11) & 31u; }
inline u32 funct_of(u32 w) { return w & 63u; }
inline s32 simm_of(u32 w) { return static_cast<s16>(w & 0xFFFFu); }

bool is_cond_branch(u32 w) {
  const u32 op = op_of(w);
  return (op >= 4 && op <= 7) || (op == 1 && (rt_of(w) == 0 || rt_of(w) == 1 || rt_of(w) == 0x10 ||
                                               rt_of(w) == 0x11));
}
bool is_load_op(u32 op) { return op >= 0x20 && op <= 0x26 && op != 0x27; }
bool is_store_op(u32 op) { return op == 0x28 || op == 0x29 || op == 0x2A || op == 0x2B || op == 0x2E; }

const char *const kRegNames[32] = {"zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
                                   "t0",   "t1", "t2", "t3", "t4", "t5", "t6", "t7",
                                   "s0",   "s1", "s2", "s3", "s4", "s5", "s6", "s7",
                                   "t8",   "t9", "k0", "k1", "gp", "sp", "fp", "ra"};

bool avoided_reg(u32 r, const GrimRomContext &ctx) {
  return r == 26 || r == 27 || r == 29 || r == 31 || (r == 28 && ctx.avoid_gp);
}
} // namespace

bool grim_mips_valid(u32 w) {
  const u32 op = op_of(w);
  switch (op) {
  case 0:
    switch (funct_of(w)) {
    case 0: case 2: case 3: case 4: case 6: case 7: case 8: case 9: case 0xC: case 0xD:
    case 0x10: case 0x11: case 0x12: case 0x13: case 0x18: case 0x19: case 0x1A: case 0x1B:
    case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
    case 0x2A: case 0x2B:
      return true;
    default:
      return false;
    }
  case 1:
    return rt_of(w) == 0 || rt_of(w) == 1 || rt_of(w) == 0x10 || rt_of(w) == 0x11;
  case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9: case 0xA: case 0xB:
  case 0xC: case 0xD: case 0xE: case 0xF:
  case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26:
  case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2E: case 0x32: case 0x3A:
    return true;
  case 0x10: // COP0: MFC0 / MTC0 / RFE
    return rs_of(w) == 0 || rs_of(w) == 4 || ((w & (1u << 25)) != 0 && funct_of(w) == 0x10);
  case 0x12: // COP2 (GTE)
    return true;
  default:
    return false;
  }
}

bool grim_mips_has_delay_slot(u32 w) {
  const u32 op = op_of(w);
  if (op == 2 || op == 3) {
    return true;
  }
  if (op == 0) {
    return funct_of(w) == 8 || funct_of(w) == 9;
  }
  return is_cond_branch(w);
}

int grim_mips_dest_reg(u32 w) {
  const u32 op = op_of(w);
  switch (op) {
  case 0:
    switch (funct_of(w)) {
    case 0: case 2: case 3: case 4: case 6: case 7: case 9: case 0x10: case 0x12:
    case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
    case 0x2A: case 0x2B:
      return static_cast<int>(rd_of(w));
    default:
      return -1;
    }
  case 1:
    return (rt_of(w) & 0x1E) == 0x10 ? 31 : -1;
  case 3:
    return 31;
  case 8: case 9: case 0xA: case 0xB: case 0xC: case 0xD: case 0xE: case 0xF:
  case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26:
    return static_cast<int>(rt_of(w));
  case 0x10: case 0x12:
    return rs_of(w) == 0 || (op == 0x12 && rs_of(w) == 2) ? static_cast<int>(rt_of(w)) : -1;
  default:
    return -1;
  }
}

std::string grim_mips_disasm(u32 w, u32 addr) {
  char b[96];
  const u32 op = op_of(w);
  const char *rs = kRegNames[rs_of(w)], *rt = kRegNames[rt_of(w)], *rd = kRegNames[rd_of(w)];
  const s32 imm = simm_of(w);
  const u32 uimm = w & 0xFFFFu;
  auto branch = [&](const char *name, bool two) {
    if (two) {
      std::snprintf(b, sizeof(b), "%s %s,%s,%+d", name, rs, rt, imm * 4);
    } else {
      std::snprintf(b, sizeof(b), "%s %s,%+d", name, rs, imm * 4);
    }
    return std::string(b);
  };
  if (w == 0) {
    return "nop";
  }
  switch (op) {
  case 0: {
    const u32 f = funct_of(w);
    const u32 sh = (w >> 6) & 31u;
    switch (f) {
    case 0: case 2: case 3: {
      const char *n = f == 0 ? "sll" : f == 2 ? "srl" : "sra";
      std::snprintf(b, sizeof(b), "%s %s,%s,%u", n, rd, rt, sh);
      return b;
    }
    case 4: case 6: case 7:
      std::snprintf(b, sizeof(b), "%s %s,%s,%s", f == 4 ? "sllv" : f == 6 ? "srlv" : "srav", rd,
                    rt, rs);
      return b;
    case 8: std::snprintf(b, sizeof(b), "jr %s", rs); return b;
    case 9: std::snprintf(b, sizeof(b), "jalr %s,%s", rd, rs); return b;
    case 0xC: return "syscall";
    case 0xD: return "break";
    case 0x10: std::snprintf(b, sizeof(b), "mfhi %s", rd); return b;
    case 0x11: std::snprintf(b, sizeof(b), "mthi %s", rs); return b;
    case 0x12: std::snprintf(b, sizeof(b), "mflo %s", rd); return b;
    case 0x13: std::snprintf(b, sizeof(b), "mtlo %s", rs); return b;
    case 0x18: case 0x19: case 0x1A: case 0x1B:
      std::snprintf(b, sizeof(b), "%s %s,%s",
                    f == 0x18 ? "mult" : f == 0x19 ? "multu" : f == 0x1A ? "div" : "divu", rs, rt);
      return b;
    case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
    case 0x2A: case 0x2B: {
      const char *n = f == 0x20 ? "add" : f == 0x21 ? "addu" : f == 0x22 ? "sub" :
                      f == 0x23 ? "subu" : f == 0x24 ? "and" : f == 0x25 ? "or" :
                      f == 0x26 ? "xor" : f == 0x27 ? "nor" : f == 0x2A ? "slt" : "sltu";
      std::snprintf(b, sizeof(b), "%s %s,%s,%s", n, rd, rs, rt);
      return b;
    }
    default: break;
    }
    break;
  }
  case 1: {
    const u32 r = rt_of(w);
    const char *n = r == 0 ? "bltz" : r == 1 ? "bgez" : r == 0x10 ? "bltzal" : r == 0x11 ? "bgezal" : nullptr;
    if (n != nullptr) {
      return branch(n, false);
    }
    break;
  }
  case 2: case 3:
    std::snprintf(b, sizeof(b), "%s 0x%08X", op == 2 ? "j" : "jal",
                  ((addr + 4u) & 0xF0000000u) | ((w & 0x03FFFFFFu) << 2));
    return b;
  case 4: return branch("beq", true);
  case 5: return branch("bne", true);
  case 6: return branch("blez", false);
  case 7: return branch("bgtz", false);
  case 8: case 9: case 0xA: case 0xB:
    std::snprintf(b, sizeof(b), "%s %s,%s,%d",
                  op == 8 ? "addi" : op == 9 ? "addiu" : op == 0xA ? "slti" : "sltiu", rt, rs, imm);
    return b;
  case 0xC: case 0xD: case 0xE:
    std::snprintf(b, sizeof(b), "%s %s,%s,0x%X", op == 0xC ? "andi" : op == 0xD ? "ori" : "xori",
                  rt, rs, uimm);
    return b;
  case 0xF: std::snprintf(b, sizeof(b), "lui %s,0x%X", rt, uimm); return b;
  case 0x10:
    if (rs_of(w) == 0 || rs_of(w) == 4) {
      std::snprintf(b, sizeof(b), "%s %s,$%u", rs_of(w) == 0 ? "mfc0" : "mtc0", rt, rd_of(w));
      return b;
    }
    if ((w & (1u << 25)) && funct_of(w) == 0x10) {
      return "rfe";
    }
    break;
  case 0x12:
    if (rs_of(w) == 0 || rs_of(w) == 2 || rs_of(w) == 4 || rs_of(w) == 6) {
      std::snprintf(b, sizeof(b), "%s %s,$%u",
                    rs_of(w) == 0 ? "mfc2" : rs_of(w) == 2 ? "cfc2" : rs_of(w) == 4 ? "mtc2" : "ctc2",
                    rt, rd_of(w));
      return b;
    }
    std::snprintf(b, sizeof(b), "cop2 0x%07X", w & 0x1FFFFFFu);
    return b;
  case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26:
  case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2E: case 0x32: case 0x3A: {
    const char *n = op == 0x20 ? "lb" : op == 0x21 ? "lh" : op == 0x22 ? "lwl" : op == 0x23 ? "lw" :
                    op == 0x24 ? "lbu" : op == 0x25 ? "lhu" : op == 0x26 ? "lwr" : op == 0x28 ? "sb" :
                    op == 0x29 ? "sh" : op == 0x2A ? "swl" : op == 0x2B ? "sw" : op == 0x2E ? "swr" :
                    op == 0x32 ? "lwc2" : "swc2";
    if (op == 0x32 || op == 0x3A) {
      std::snprintf(b, sizeof(b), "%s $%u,%d(%s)", n, rt_of(w), imm, rs);
    } else {
      std::snprintf(b, sizeof(b), "%s %s,%d(%s)", n, rt, imm, rs);
    }
    return b;
  }
  default:
    break;
  }
  std::snprintf(b, sizeof(b), ".word 0x%08X", w);
  return b;
}

// ---- context ---------------------------------------------------------------------------

bool GrimRomContext::load(const std::string &bios_path, const std::string &map_path,
                          std::string &err) {
  Bios bios;
  if (!bios.load(bios_path)) {
    err = "cannot load BIOS " + bios_path;
    return false;
  }
  if (!grim_map_load(map_path, map, err)) {
    return false;
  }
  bios_hash = bios.image_hash();
  if (map.bios_hash != bios_hash) {
    char msg[200];
    std::snprintf(msg, sizeof(msg),
                  "the map was made for BIOS 0x%016llX but %s is 0x%016llX",
                  static_cast<unsigned long long>(map.bios_hash), bios_path.c_str(),
                  static_cast<unsigned long long>(bios_hash));
    err = msg;
    return false;
  }
  words.assign(map.words.size(), 0);
  for (u32 i = 0; i < words.size(); ++i) {
    bios.original_word(i * 4u, words[i]);
  }
  init();
  return true;
}

void GrimRomContext::init() {
  avoid_gp = false;
  jal_targets.clear();
  for (u32 i = 0; i < words.size(); ++i) {
    if (!is_code(i)) {
      continue;
    }
    const u32 w = words[i];
    const u32 op = op_of(w);
    if ((is_load_op(op) || is_store_op(op)) && rs_of(w) == 28) {
      avoid_gp = true;
    }
    if (op == 3) {
      jal_targets.push_back(w & 0x03FFFFFFu);
    }
  }
  std::sort(jal_targets.begin(), jal_targets.end());
  jal_targets.erase(std::unique(jal_targets.begin(), jal_targets.end()), jal_targets.end());
}

// ---- mutations ---------------------------------------------------------------------------

const char *grim_rom_mut_name(GrimRomMut kind) {
  static const char *const kNames[] = {"imm_perturb", "reg_subst", "branch_invert",
                                       "branch_nudge", "alu_subst", "ls_width",
                                       "nop",          "lui_ori_const", "call_swap"};
  return static_cast<size_t>(kind) < sizeof(kNames) / sizeof(kNames[0]) ? kNames[static_cast<size_t>(kind)]
                                                                       : "?";
}

namespace {
// Fields of `w` that a register substitution may change: shifts of 11, 16, 21.
size_t subst_fields(u32 w, const GrimRomContext &ctx, u32 shifts[3]) {
  if (w == 0) {
    return 0;
  }
  const u32 op = op_of(w);
  u32 cand[3];
  size_t n = 0;
  auto add = [&](u32 shift) { cand[n++] = shift; };
  if (op == 0) {
    switch (funct_of(w)) {
    case 0: case 2: case 3: add(16); add(11); break;
    case 4: case 6: case 7: case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25:
    case 0x26: case 0x27: case 0x2A: case 0x2B: add(21); add(16); add(11); break;
    case 0x18: case 0x19: case 0x1A: case 0x1B: add(21); add(16); break;
    case 0x10: case 0x12: add(11); break;
    case 0x11: case 0x13: add(21); break;
    default: break;
    }
  } else if (op == 1 && (rt_of(w) == 0 || rt_of(w) == 1)) {
    add(21);
  } else if (op == 4 || op == 5) {
    add(21); add(16);
  } else if (op == 6 || op == 7) {
    add(21);
  } else if ((op >= 8 && op <= 0xE) || is_load_op(op) || is_store_op(op)) {
    add(21); add(16);
  } else if (op == 0xF) {
    add(16);
  }
  size_t out = 0;
  for (size_t i = 0; i < n; ++i) {
    if (!avoided_reg((w >> cand[i]) & 31u, ctx)) {
      shifts[out++] = cand[i];
    }
  }
  return out;
}

int width_of_mem_op(u32 op) {
  switch (op) {
  case 0x20: case 0x24: case 0x28: return 1;
  case 0x21: case 0x25: case 0x29: return 2;
  case 0x23: case 0x2B: return 4;
  default: return 0;
  }
}

// The LUI at `index` and its ORI/ADDIU partner within three words, if the
// register is not touched in between and no branch intervenes.
bool find_lui_pair(const GrimRomContext &ctx, u32 index, u32 &partner) {
  const u32 lui = ctx.words[index];
  if (op_of(lui) != 0xF || !ctx.is_code(index)) {
    return false;
  }
  const u32 r = rt_of(lui);
  if (r == 0) {
    return false;
  }
  for (u32 k = 1; k <= 3 && index + k < ctx.words.size(); ++k) {
    const u32 w = ctx.words[index + k];
    if (!ctx.is_code(index + k)) {
      return false;
    }
    if ((op_of(w) == 0xD || op_of(w) == 9) && rs_of(w) == r && rt_of(w) == r) {
      partner = index + k;
      return true;
    }
    if (grim_mips_has_delay_slot(w) || grim_mips_dest_reg(w) == static_cast<int>(r)) {
      return false;
    }
  }
  return false;
}
} // namespace

bool grim_rom_applicable(GrimRomMut kind, const GrimRomContext &ctx, u32 index) {
  if (!ctx.is_code(index) || index >= ctx.words.size()) {
    return false;
  }
  const u32 w = ctx.words[index];
  const u32 op = op_of(w);
  switch (kind) {
  case GrimRomMut::ImmPerturb:
    return op >= 8 && op <= 0xE && rt_of(w) != 29 && rs_of(w) != 29;
  case GrimRomMut::RegSubst: {
    u32 f[3];
    return subst_fields(w, ctx, f) != 0;
  }
  case GrimRomMut::BranchInvert:
    return (op >= 4 && op <= 7) || (op == 1 && (rt_of(w) == 0 || rt_of(w) == 1));
  case GrimRomMut::BranchNudge:
    return is_cond_branch(w);
  case GrimRomMut::AluSubst: {
    if (w == 0 || op != 0) {
      return false;
    }
    const u32 f = funct_of(w);
    return f == 0 || f == 2 || f == 3 || f == 4 || f == 6 || f == 7 || f == 0x21 || f == 0x23 ||
           f == 0x24 || f == 0x25 || f == 0x26;
  }
  case GrimRomMut::LsWidth:
    return width_of_mem_op(op) != 0;
  case GrimRomMut::Nop:
    return w != 0;
  case GrimRomMut::LuiOriConst: {
    u32 partner;
    return find_lui_pair(ctx, index, partner);
  }
  case GrimRomMut::CallSwap:
    return op == 3 && ctx.jal_targets.size() > 1;
  default:
    return false;
  }
}

bool grim_rom_mutate(GrimRomMut kind, GrimRng &rng, const GrimRomContext &ctx, u32 index,
                     std::vector<GrimRomEdit> &out) {
  if (!grim_rom_applicable(kind, ctx, index)) {
    return false;
  }
  const u32 w = ctx.words[index];
  const u32 op = op_of(w);
  u32 nw = w;
  switch (kind) {
  case GrimRomMut::ImmPerturb: {
    const u32 imm = w & 0xFFFFu;
    u32 ni = imm;
    const u32 how = rng.range(0, 2);
    if (how == 0) {
      s32 d = rng.srange(-8, 7);
      d += d >= 0 ? 1 : 0; // -8..-1, 1..8
      ni = (imm + static_cast<u32>(d)) & 0xFFFFu;
    } else if (how == 1) {
      ni = op <= 0xB ? (0u - imm) & 0xFFFFu : imm ^ 0x8000u; // arithmetic: negate
    }
    if (how == 2 || ni == imm) {
      ni = imm ^ (1u << rng.range(0, 15));
    }
    nw = (w & 0xFFFF0000u) | ni;
    break;
  }
  case GrimRomMut::RegSubst: {
    u32 shifts[3];
    const size_t n = subst_fields(w, ctx, shifts);
    const u32 shift = shifts[rng.range(0, static_cast<u32>(n) - 1u)];
    const u32 cur = (w >> shift) & 31u;
    u32 pool[32];
    size_t np = 0;
    for (u32 r = 0; r < 32; ++r) {
      if (r != cur && !avoided_reg(r, ctx)) {
        pool[np++] = r;
      }
    }
    nw = (w & ~(31u << shift)) | (pool[rng.range(0, static_cast<u32>(np) - 1u)] << shift);
    break;
  }
  case GrimRomMut::BranchInvert:
    if (op == 4 || op == 6) {
      nw = w + (1u << 26);
    } else if (op == 5 || op == 7) {
      nw = w - (1u << 26);
    } else {
      nw = w ^ (1u << 16); // BLTZ <-> BGEZ (rt 0 <-> 1)
    }
    break;
  case GrimRomMut::BranchNudge: {
    static const s32 kDeltas[8] = {-4, -3, -2, -1, 1, 2, 3, 4};
    const u32 start = rng.range(0, 7);
    bool found = false;
    for (u32 k = 0; k < 8 && !found; ++k) {
      const s32 ns = simm_of(w) + kDeltas[(start + k) & 7u];
      const s64 target = static_cast<s64>(index) + 1 + ns;
      if (ns < -32768 || ns > 32767 || target < 0 || target >= static_cast<s64>(ctx.words.size()) ||
          target == static_cast<s64>(index) || !ctx.is_code(static_cast<u32>(target))) {
        continue;
      }
      nw = (w & 0xFFFF0000u) | (static_cast<u32>(ns) & 0xFFFFu);
      found = true;
    }
    if (!found) {
      return false;
    }
    break;
  }
  case GrimRomMut::AluSubst: {
    static const u32 kArith[] = {0x21, 0x23, 0x26, 0x25, 0x24};
    static const u32 kShiftImm[] = {0, 2, 3};
    static const u32 kShiftVar[] = {4, 6, 7};
    const u32 f = funct_of(w);
    const u32 *set = f == 0 || f == 2 || f == 3 ? kShiftImm : f >= 4 && f <= 7 ? kShiftVar : kArith;
    const size_t n = set == kArith ? 5 : 3;
    u32 pool[5];
    size_t np = 0;
    for (size_t i = 0; i < n; ++i) {
      if (set[i] != f) {
        pool[np++] = set[i];
      }
    }
    nw = (w & ~63u) | pool[rng.range(0, static_cast<u32>(np) - 1u)];
    break;
  }
  case GrimRomMut::LsWidth: {
    static const u32 kLoads[] = {0x23, 0x21, 0x25, 0x20, 0x24};
    static const u32 kStores[] = {0x2B, 0x29, 0x28};
    const bool load = is_load_op(op);
    const u32 *set = load ? kLoads : kStores;
    const size_t n = load ? 5 : 3;
    const s32 off = simm_of(w);
    u32 pool[5];
    size_t np = 0;
    for (size_t i = 0; i < n; ++i) {
      const int width = width_of_mem_op(set[i]);
      if (set[i] != op && (off % width) == 0) {
        pool[np++] = set[i];
      }
    }
    if (np == 0) {
      return false;
    }
    nw = (w & 0x03FFFFFFu) | (pool[rng.range(0, static_cast<u32>(np) - 1u)] << 26);
    break;
  }
  case GrimRomMut::Nop:
    nw = 0;
    break;
  case GrimRomMut::LuiOriConst: {
    u32 partner = 0;
    find_lui_pair(ctx, index, partner);
    const u32 second = ctx.words[partner];
    const bool ori = op_of(second) == 0xD;
    u32 c = (w << 16) + (ori ? (second & 0xFFFFu)
                             : static_cast<u32>(static_cast<s32>(simm_of(second))));
    const u32 how = rng.range(0, 2);
    if (how == 0) {
      s32 d = rng.srange(-256, 255);
      d += d >= 0 ? 1 : 0;
      c += static_cast<u32>(d);
    } else if (how == 1) {
      c ^= 1u << rng.range(0, 23); // keep the top byte (the address segment)
    } else {
      s32 k = rng.srange(-4, 3);
      k += k >= 0 ? 1 : 0;
      c += static_cast<u32>(k) << 16;
    }
    const u32 lo = c & 0xFFFFu;
    const u32 hi = ori ? c >> 16
                       : ((c - static_cast<u32>(static_cast<s32>(static_cast<s16>(lo)))) >> 16);
    const u32 new_lui = (w & 0xFFFF0000u) | (hi & 0xFFFFu);
    const u32 new_second = (second & 0xFFFF0000u) | lo;
    if (new_lui == w && new_second == second) {
      return false;
    }
    if (new_lui != w) {
      out.push_back({index, new_lui});
    }
    if (new_second != second) {
      out.push_back({partner, new_second});
    }
    return true;
  }
  case GrimRomMut::CallSwap: {
    u32 target = w & 0x03FFFFFFu;
    while (target == (w & 0x03FFFFFFu)) {
      target = ctx.jal_targets[rng.range(0, static_cast<u32>(ctx.jal_targets.size()) - 1u)];
    }
    nw = (w & 0xFC000000u) | target;
    break;
  }
  default:
    return false;
  }
  if (nw == w || !grim_mips_valid(nw)) {
    return false;
  }
  out.push_back({index, nw});
  return true;
}

GrimGene grim_rom_generate(const GrimRomContext &ctx, u64 seed, GrimRomMut kind, u32 count,
                           u32 early_ms, u32 curve) {
  GrimGene gene;
  gene.type = GrimGeneType::RomCode;
  gene.target = 1;
  gene.seed = seed;
  gene.params[0] = static_cast<s32>(kind);
  gene.params[1] = static_cast<s32>(count);
  gene.params[2] = static_cast<s32>(early_ms);
  gene.params[3] = static_cast<s32>(curve);

  const u64 early = u64{early_ms} * (psx::CPU_CLOCK_HZ / 1000u);
  const u64 t_end = std::max<u64>(ctx.map.last_new_code_cycle, early + 1u);
  std::vector<u32> cand;
  for (u32 i = 0; i < ctx.words.size(); ++i) {
    if (ctx.is_code(i) && ctx.map.words[i].first_exec >= early &&
        grim_rom_applicable(kind, ctx, i)) {
      cand.push_back(i);
    }
  }
  if (cand.empty()) {
    return gene;
  }
  auto weight = [&](u32 i) -> u32 {
    const u64 t = std::min<u64>(ctx.map.words[i].first_exec, t_end) - early;
    const u32 q = static_cast<u32>(t * 1024u / (t_end - early)); // 0..1024
    u32 w = 1024;
    if (curve == 1) {
      w = q;
    } else if (curve == 2) {
      w = (q * q) >> 10;
    } else if (curve == 3) {
      w = (q * q >> 10) * q >> 10;
    }
    return 16u + w; // late code is preferred, early code is never impossible
  };
  GrimRng rng{seed};
  std::vector<u32> used; // word indices already patched
  const u64 max_tries = u64{count} * 400u + 200u;
  for (u64 tries = 0; tries < max_tries && gene.patches.size() < count; ++tries) {
    const u32 i = cand[rng.range(0, static_cast<u32>(cand.size()) - 1u)];
    if (rng.range(0, 1039) >= weight(i)) {
      continue;
    }
    std::vector<GrimRomEdit> edits;
    if (!grim_rom_mutate(kind, rng, ctx, i, edits)) {
      continue;
    }
    bool clash = false;
    for (const GrimRomEdit &e : edits) {
      clash = clash || std::find(used.begin(), used.end(), e.index) != used.end();
    }
    if (clash) {
      continue;
    }
    for (const GrimRomEdit &e : edits) {
      used.push_back(e.index);
      GrimRomPatch p;
      p.offset = e.index * 4u;
      p.original = ctx.words[e.index];
      p.mutated = e.word;
      p.delay_slot = ctx.in_delay_slot(e.index);
      gene.patches.push_back(p);
    }
  }
  return gene;
}

void grim_add_random_rom_genes(GrimGenome &genome, u64 seed, const GrimRandomParams &rp) {
  GrimRng rng{seed ^ 0x524F4D47454E4531ull}; // "ROMGENE1": separate from the interface stream
  const GrimRomContext &ctx = *rp.rom;
  const u32 n = rng.range(rp.rom_genes_min, std::max(rp.rom_genes_min, rp.rom_genes_max));
  const u32 kinds = rp.rom_call_swap ? static_cast<u32>(GrimRomMut::Count)
                                     : static_cast<u32>(GrimRomMut::CallSwap);
  for (u32 g = 0; g < n; ++g) {
    const GrimRomMut kind = static_cast<GrimRomMut>(rng.range(0, kinds - 1u));
    const u32 count = rng.range(1, std::max(1u, rp.rom_patches_max));
    GrimGene gene = grim_rom_generate(ctx, rng.next(), kind, count, rp.rom_early_ms, rp.rom_curve);
    if (!gene.patches.empty()) {
      genome.genes.push_back(std::move(gene));
    }
  }
  if (grim_genome_has_rom(genome)) {
    genome.version = 2;
    genome.bios_hash = ctx.bios_hash;
  }
}

// ---- describe -------------------------------------------------------------------------------

namespace {
std::string trigger_text(const GrimTrigger &t) {
  char b[128];
  switch (t.kind) {
  case GrimTriggerKind::Always: return "always";
  case GrimTriggerKind::Window:
    std::snprintf(b, sizeof(b), "window frames %u-%u", t.start_frame, t.end_frame);
    return b;
  case GrimTriggerKind::Rot:
    std::snprintf(b, sizeof(b), "rot frames %u-%u", t.start_frame, t.end_frame);
    return b;
  default:
    if (t.period > 0) {
      std::snprintf(b, sizeof(b), "intermittent %u on / %u frames from %u", t.duty, t.period,
                    t.start_frame);
    } else {
      std::snprintf(b, sizeof(b), "intermittent p=%u permille from %u", t.probability, t.start_frame);
    }
    return b;
  }
}
} // namespace

std::string grim_describe_genome(const GrimGenome &genome, const GrimRomContext *ctx,
                               const GrimSampleContext *samples) {
  std::string out;
  char b[256];
  std::vector<GrimMapRegion> regions;
  if (ctx != nullptr) {
    regions = grim_map_regions(ctx->map);
  }
  std::snprintf(b, sizeof(b), "genome version=%u genes=%zu hash=0x%016llX", genome.version,
                genome.genes.size(), static_cast<unsigned long long>(grim_genome_hash(genome)));
  out += b;
  if (genome.version >= 2) {
    std::snprintf(b, sizeof(b), " bios=0x%016llX", static_cast<unsigned long long>(genome.bios_hash));
    out += b;
  }
  out += "\n";
  for (size_t gi = 0; gi < genome.genes.size(); ++gi) {
    const GrimGene &g = genome.genes[gi];
    if (g.type == GrimGeneType::SpuSample) {
      out += "gene " + std::to_string(gi) + ": " + grim_sample_describe_gene(g, samples);
      continue;
    }
    if (g.type == GrimGeneType::RomCode) {
      std::snprintf(b, sizeof(b),
                    "gene %zu: rom_code kind=%s patches=%zu (generated: count=%d early_ms=%d "
                    "curve=%d seed=%llu)\n",
                    gi, grim_rom_mut_name(static_cast<GrimRomMut>(g.params[0])), g.patches.size(),
                    g.params[1], g.params[2], g.params[3],
                    static_cast<unsigned long long>(g.seed));
      out += b;
      for (const GrimRomPatch &p : g.patches) {
        std::snprintf(b, sizeof(b), "  0xBFC%05X: %s -> %s", p.offset,
                      grim_mips_disasm(p.original, 0xBFC00000u + p.offset).c_str(),
                      grim_mips_disasm(p.mutated, 0xBFC00000u + p.offset).c_str());
        out += b;
        if (ctx != nullptr && p.offset / 4u < ctx->map.words.size()) {
          const u32 wi = p.offset / 4u;
          const GrimMapWord &mw = ctx->map.words[wi];
          for (const GrimMapRegion &r : regions) {
            if (wi >= r.start_word && wi < r.end_word) {
              std::snprintf(b, sizeof(b), "  [%s region 0x%05X-0x%05X, first exec %.2f ms%s%s]",
                            grim_word_class_name(r.cls), r.start_word * 4u, r.end_word * 4u,
                            mw.first_exec == kGrimNever ? -1.0 : grim_cycles_to_ms(mw.first_exec),
                            (mw.flags & kGrimExecViaRam) ? ", runs from RAM" : "",
                            p.delay_slot ? ", delay slot" : "");
              out += b;
              break;
            }
          }
        } else if (p.delay_slot) {
          out += "  [delay slot]";
        }
        out += "\n";
      }
      continue;
    }
    std::snprintf(b, sizeof(b), "gene %zu: %s target=0x%X seed=%llu trigger=%s\n", gi,
                  grim_gene_type_name(g.type), g.target, static_cast<unsigned long long>(g.seed),
                  trigger_text(g.trigger).c_str());
    out += b;
    const auto &schema = grim_gene_schema(g.type);
    out += "  ";
    for (size_t k = 0; k < schema.size(); ++k) {
      out += (k ? " " : "") + std::string(schema[k].name) + "=" + std::to_string(g.params[k]);
    }
    out += "\n";
  }
  return out;
}
