#include "cpu_jit_v4.h"

#include "jit_code_page_bitmap.h"
#include "system.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#if defined(VIBESTATION_ENABLE_X64_JIT) && \
    (defined(_M_X64) || defined(__x86_64__))
#include <xbyak/xbyak.h>
#define VIBESTATION_JIT_V4_X64 1
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#else
#include <sys/mman.h>
#endif
#else
#define VIBESTATION_JIT_V4_X64 0
#endif

namespace {

constexpr u32 kV4MaxBlockInstructions = 32u;
constexpr size_t kV4CodeArenaBytes = 32u * 1024u * 1024u;
constexpr size_t kV4MaxBlocks = 65536u;
constexpr u32 kV4PhysPageShift = 12u;
constexpr size_t kV4PhysPageCount = size_t{1} << (29u - kV4PhysPageShift);
constexpr size_t kV4DispatchTopCount = size_t{1} << 20u;
constexpr size_t kV4DispatchEntriesPerPage = size_t{1} << 10u;

enum class V4AluOp : u8 {
  Nop,
  Sll,
  Srl,
  Sra,
  Sllv,
  Srlv,
  Srav,
  Addu,
  Subu,
  And,
  Or,
  Xor,
  Nor,
  Slt,
  Sltu,
  Addiu,
  Slti,
  Sltiu,
  Andi,
  Ori,
  Xori,
  Lui,
};

struct V4DecodedInstruction {
  V4AluOp op = V4AluOp::Nop;
  u8 rs = 0;
  u8 rt = 0;
  u8 rd = 0;
  u8 shamt = 0;
  u16 imm = 0;
  s32 simm = 0;
};

enum class V4ControlOp : u8 {
  None,
  J,
  Jal,
  Jr,
  Jalr,
  Beq,
  Bne,
  Blez,
  Bgtz,
  Bltz,
  Bgez,
  Bltzal,
  Bgezal,
};

struct V4DecodedControl {
  V4ControlOp op = V4ControlOp::None;
  u8 rs = 0;
  u8 rt = 0;
  u8 rd = 0;
  s32 simm = 0;
  u32 imm26 = 0;
};

enum class V4LoadOp : u8 {
  Lb,
  Lh,
  Lw,
  Lbu,
  Lhu,
};

struct V4DecodedLoad {
  V4LoadOp op = V4LoadOp::Lw;
  u8 rs = 0;
  u8 rt = 0;
  s32 simm = 0;
};

bool decode_v4_load(u32 bits, V4DecodedLoad &out) {
  switch ((bits >> 26) & 0x3Fu) {
  case 0x20: out.op = V4LoadOp::Lb; break;
  case 0x21: out.op = V4LoadOp::Lh; break;
  case 0x23: out.op = V4LoadOp::Lw; break;
  case 0x24: out.op = V4LoadOp::Lbu; break;
  case 0x25: out.op = V4LoadOp::Lhu; break;
  default: return false;
  }
  out.rs = static_cast<u8>((bits >> 21) & 0x1Fu);
  out.rt = static_cast<u8>((bits >> 16) & 0x1Fu);
  out.simm = static_cast<s32>(static_cast<s16>(bits & 0xFFFFu));
  return true;
}

bool decode_v4_control(u32 bits, V4DecodedControl &out) {
  out = {};
  out.rs = static_cast<u8>((bits >> 21) & 0x1Fu);
  out.rt = static_cast<u8>((bits >> 16) & 0x1Fu);
  out.rd = static_cast<u8>((bits >> 11) & 0x1Fu);
  out.simm = static_cast<s32>(static_cast<s16>(bits & 0xFFFFu));
  out.imm26 = bits & 0x03FFFFFFu;
  const u32 primary = (bits >> 26) & 0x3Fu;
  if (primary == 0u) {
    switch (bits & 0x3Fu) {
    case 0x08: out.op = V4ControlOp::Jr; return true;
    case 0x09: out.op = V4ControlOp::Jalr; return true;
    default: return false;
    }
  }
  switch (primary) {
  case 0x01:
    switch (out.rt) {
    case 0x00: out.op = V4ControlOp::Bltz; return true;
    case 0x01: out.op = V4ControlOp::Bgez; return true;
    case 0x10: out.op = V4ControlOp::Bltzal; return true;
    case 0x11: out.op = V4ControlOp::Bgezal; return true;
    default: return false; // branch-likely/legacy REGIMM stays on the oracle
    }
  case 0x02: out.op = V4ControlOp::J; return true;
  case 0x03: out.op = V4ControlOp::Jal; return true;
  case 0x04: out.op = V4ControlOp::Beq; return true;
  case 0x05: out.op = V4ControlOp::Bne; return true;
  case 0x06: out.op = V4ControlOp::Blez; return true;
  case 0x07: out.op = V4ControlOp::Bgtz; return true;
  default: return false;
  }
}

bool decode_v4_alu(u32 bits, V4DecodedInstruction &out) {
  out = {};
  out.rs = static_cast<u8>((bits >> 21) & 0x1Fu);
  out.rt = static_cast<u8>((bits >> 16) & 0x1Fu);
  out.rd = static_cast<u8>((bits >> 11) & 0x1Fu);
  out.shamt = static_cast<u8>((bits >> 6) & 0x1Fu);
  out.imm = static_cast<u16>(bits & 0xFFFFu);
  out.simm = static_cast<s32>(static_cast<s16>(out.imm));

  const u32 primary = (bits >> 26) & 0x3Fu;
  if (primary == 0u) {
    if (bits == 0u) {
      out.op = V4AluOp::Nop;
      return true;
    }
    switch (bits & 0x3Fu) {
    case 0x00: out.op = V4AluOp::Sll; return true;
    case 0x02: out.op = V4AluOp::Srl; return true;
    case 0x03: out.op = V4AluOp::Sra; return true;
    case 0x04: out.op = V4AluOp::Sllv; return true;
    case 0x06: out.op = V4AluOp::Srlv; return true;
    case 0x07: out.op = V4AluOp::Srav; return true;
    case 0x21: out.op = V4AluOp::Addu; return true;
    case 0x23: out.op = V4AluOp::Subu; return true;
    case 0x24: out.op = V4AluOp::And; return true;
    case 0x25: out.op = V4AluOp::Or; return true;
    case 0x26: out.op = V4AluOp::Xor; return true;
    case 0x27: out.op = V4AluOp::Nor; return true;
    case 0x2A: out.op = V4AluOp::Slt; return true;
    case 0x2B: out.op = V4AluOp::Sltu; return true;
    default: return false;
    }
  }

  switch (primary) {
  case 0x09: out.op = V4AluOp::Addiu; return true;
  case 0x0A: out.op = V4AluOp::Slti; return true;
  case 0x0B: out.op = V4AluOp::Sltiu; return true;
  case 0x0C: out.op = V4AluOp::Andi; return true;
  case 0x0D: out.op = V4AluOp::Ori; return true;
  case 0x0E: out.op = V4AluOp::Xori; return true;
  case 0x0F: out.op = V4AluOp::Lui; return true;
  default: return false;
  }
}

#if VIBESTATION_JIT_V4_X64

struct V4DispatchPage;

struct V4NativeState {
  u32 *gpr = nullptr;
  V4DispatchPage **dispatch_top = nullptr;
  const u32 *icache_generations = nullptr;
  const u32 *code_page_generations = nullptr;
  void *block_return = nullptr;
  u8 *main_ram = nullptr;
  u8 *scratchpad = nullptr;
  u32 mapped_main_ram_size = 0;
  u32 memory_fastpath_allowed = 0;
  u32 block_bail = 0;
  u32 memory_entries = 0;
  u32 cache_epoch = 0;
  u32 pc = 0;
  u32 last_pc = 0;
  u32 last_in_delay_slot = 0;
  u32 active_branch_pc = 0;
  u32 pending_load_reg = 0;
  u32 pending_load_value = 0;
  u32 cycles = 0;
  u32 instructions = 0;
  u32 cycle_budget = 0;
  u32 instruction_budget = 0;
  u32 block_entries = 0;
};

using V4NativeFn = void (*)(V4NativeState *);
using V4ResidentDispatchFn = void (*)(V4NativeState *);

class V4CodeArena {
public:
  V4CodeArena() = default;

  ~V4CodeArena() {
    if (base_ == nullptr) {
      return;
    }
#if defined(_WIN32)
    VirtualFree(base_, 0, MEM_RELEASE);
#else
    munmap(base_, kV4CodeArenaBytes);
#endif
  }

  V4CodeArena(const V4CodeArena &) = delete;
  V4CodeArena &operator=(const V4CodeArena &) = delete;

  bool ensure_available() {
    if (base_ != nullptr) {
      return true;
    }
#if defined(_WIN32)
    base_ = static_cast<u8 *>(VirtualAlloc(nullptr, kV4CodeArenaBytes,
                                           MEM_RESERVE | MEM_COMMIT,
                                           PAGE_EXECUTE_READWRITE));
#else
    void *ptr = mmap(nullptr, kV4CodeArenaBytes,
                     PROT_READ | PROT_WRITE | PROT_EXEC,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    base_ = ptr == MAP_FAILED ? nullptr : static_cast<u8 *>(ptr);
#endif
    return base_ != nullptr;
  }

  bool available() const { return base_ != nullptr; }
  size_t bytes_used() const { return used_; }

  void reset_to(size_t offset) {
    used_ = std::min(offset, kV4CodeArenaBytes);
  }

  void *begin_emit(size_t capacity) {
    if (base_ == nullptr || capacity == 0u) {
      return nullptr;
    }
    const size_t offset = (used_ + 15u) & ~size_t{15u};
    if (offset > kV4CodeArenaBytes ||
        capacity > kV4CodeArenaBytes - offset) {
      return nullptr;
    }
    return base_ + offset;
  }

  bool commit_emit(void *code, size_t size) {
    if (base_ == nullptr || code == nullptr || size == 0u) {
      return false;
    }
    const size_t offset = (used_ + 15u) & ~size_t{15u};
    if (code != base_ + offset || offset > kV4CodeArenaBytes ||
        size > kV4CodeArenaBytes - offset) {
      return false;
    }
    used_ = offset + size;
    return true;
  }

private:
  u8 *base_ = nullptr;
  size_t used_ = 0u;
};

struct V4Block {
  u32 start_pc = 0;
  u32 instruction_count = 0;
  u32 max_cycles = 0;
  u32 code_size = 0;
  u32 icache_generation = 0;
  u32 code_page_generation = 0;
  u32 phys_page = 0;
  u16 icache_index = 0;
  V4NativeFn fn = nullptr;
  bool interpreter_only = false;
  bool has_control = false;
  bool has_memory = false;
  bool cacheable = false;
};

struct V4DispatchEntry {
  void *code = nullptr;
  V4Block *block = nullptr;
  u32 cache_epoch = 0;
};

static_assert(sizeof(V4DispatchEntry) == 24u);

struct V4DispatchPage {
  std::array<V4DispatchEntry, kV4DispatchEntriesPerPage> entries{};
};

void emit_v4_block_return(Xbyak::CodeGenerator &code) {
  code.jmp(code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, block_return))]);
}

void emit_read_guest(Xbyak::CodeGenerator &code, const Xbyak::Reg32 &dst,
                     u8 guest_reg) {
  if (guest_reg == 0u) {
    code.xor_(dst, dst);
    return;
  }
  code.mov(dst, code.dword[code.r10 + static_cast<int>(guest_reg) * 4]);
}

void emit_write_guest(Xbyak::CodeGenerator &code, u8 guest_reg,
                      const Xbyak::Reg32 &src) {
  if (guest_reg == 0u) {
    return;
  }
  code.mov(code.dword[code.r10 + static_cast<int>(guest_reg) * 4], src);
}

u8 v4_alu_write_reg(const V4DecodedInstruction &inst) {
  switch (inst.op) {
  case V4AluOp::Nop:
    return 0u;
  case V4AluOp::Sll:
  case V4AluOp::Srl:
  case V4AluOp::Sra:
  case V4AluOp::Sllv:
  case V4AluOp::Srlv:
  case V4AluOp::Srav:
  case V4AluOp::Addu:
  case V4AluOp::Subu:
  case V4AluOp::And:
  case V4AluOp::Or:
  case V4AluOp::Xor:
  case V4AluOp::Nor:
  case V4AluOp::Slt:
  case V4AluOp::Sltu:
    return inst.rd;
  case V4AluOp::Addiu:
  case V4AluOp::Slti:
  case V4AluOp::Sltiu:
  case V4AluOp::Andi:
  case V4AluOp::Ori:
  case V4AluOp::Xori:
  case V4AluOp::Lui:
    return inst.rt;
  }
  return 0u;
}

u8 v4_control_write_reg(const V4DecodedControl &control) {
  if (control.op == V4ControlOp::Jal) {
    return 31u;
  }
  if (control.op == V4ControlOp::Jalr) {
    return control.rd;
  }
  if (control.op == V4ControlOp::Bltzal ||
      control.op == V4ControlOp::Bgezal) {
    return 31u;
  }
  return 0u;
}

// R3000A load delay ordering:
//   1. the current instruction captures operands from the old GPR value;
//   2. a current write to the same GPR cancels the pending load;
//   3. otherwise the previous load commits before the next instruction.
//
// V4 keeps this tiny state in its native context so linked blocks do not need to
// bounce through Cpu::advance_load_delay().
void emit_retire_incoming_load(Xbyak::CodeGenerator &code, u8 cancel_reg) {
  using namespace Xbyak;
  Label clear, done;
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_load_reg))]);
  code.test(code.eax, code.eax);
  code.jz(done);
  if (cancel_reg != 0u) {
    code.cmp(code.eax, static_cast<u32>(cancel_reg));
    code.je(clear);
  }
  code.mov(code.ecx, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_load_value))]);
  code.mov(code.dword[code.r10 + code.rax * 4], code.ecx);
  code.L(clear);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_load_reg))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_load_value))],
      0u);
  code.L(done);
}

void emit_v4_alu_instruction(Xbyak::CodeGenerator &code,
                             const V4DecodedInstruction &inst) {
  switch (inst.op) {
    case V4AluOp::Nop:
      break;

    case V4AluOp::Sll:
      emit_read_guest(code, code.eax, inst.rt);
      code.shl(code.eax, inst.shamt);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Srl:
      emit_read_guest(code, code.eax, inst.rt);
      code.shr(code.eax, inst.shamt);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Sra:
      emit_read_guest(code, code.eax, inst.rt);
      code.sar(code.eax, inst.shamt);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Sllv:
      emit_read_guest(code, code.eax, inst.rt);
      emit_read_guest(code, code.ecx, inst.rs);
      code.shl(code.eax, code.cl);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Srlv:
      emit_read_guest(code, code.eax, inst.rt);
      emit_read_guest(code, code.ecx, inst.rs);
      code.shr(code.eax, code.cl);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Srav:
      emit_read_guest(code, code.eax, inst.rt);
      emit_read_guest(code, code.ecx, inst.rs);
      code.sar(code.eax, code.cl);
      emit_write_guest(code, inst.rd, code.eax);
      break;

    case V4AluOp::Addu:
      emit_read_guest(code, code.eax, inst.rs);
      emit_read_guest(code, code.ecx, inst.rt);
      code.add(code.eax, code.ecx);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Subu:
      emit_read_guest(code, code.eax, inst.rs);
      emit_read_guest(code, code.ecx, inst.rt);
      code.sub(code.eax, code.ecx);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::And:
      emit_read_guest(code, code.eax, inst.rs);
      emit_read_guest(code, code.ecx, inst.rt);
      code.and_(code.eax, code.ecx);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Or:
      emit_read_guest(code, code.eax, inst.rs);
      emit_read_guest(code, code.ecx, inst.rt);
      code.or_(code.eax, code.ecx);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Xor:
      emit_read_guest(code, code.eax, inst.rs);
      emit_read_guest(code, code.ecx, inst.rt);
      code.xor_(code.eax, code.ecx);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Nor:
      emit_read_guest(code, code.eax, inst.rs);
      emit_read_guest(code, code.ecx, inst.rt);
      code.or_(code.eax, code.ecx);
      code.not_(code.eax);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Slt:
      emit_read_guest(code, code.eax, inst.rs);
      emit_read_guest(code, code.ecx, inst.rt);
      code.cmp(code.eax, code.ecx);
      code.setl(code.al);
      code.movzx(code.eax, code.al);
      emit_write_guest(code, inst.rd, code.eax);
      break;
    case V4AluOp::Sltu:
      emit_read_guest(code, code.eax, inst.rs);
      emit_read_guest(code, code.ecx, inst.rt);
      code.cmp(code.eax, code.ecx);
      code.setb(code.al);
      code.movzx(code.eax, code.al);
      emit_write_guest(code, inst.rd, code.eax);
      break;

    case V4AluOp::Addiu:
      emit_read_guest(code, code.eax, inst.rs);
      code.add(code.eax, static_cast<u32>(inst.simm));
      emit_write_guest(code, inst.rt, code.eax);
      break;
    case V4AluOp::Slti:
      emit_read_guest(code, code.eax, inst.rs);
      code.cmp(code.eax, static_cast<u32>(inst.simm));
      code.setl(code.al);
      code.movzx(code.eax, code.al);
      emit_write_guest(code, inst.rt, code.eax);
      break;
    case V4AluOp::Sltiu:
      emit_read_guest(code, code.eax, inst.rs);
      code.cmp(code.eax, static_cast<u32>(inst.simm));
      code.setb(code.al);
      code.movzx(code.eax, code.al);
      emit_write_guest(code, inst.rt, code.eax);
      break;
    case V4AluOp::Andi:
      emit_read_guest(code, code.eax, inst.rs);
      code.and_(code.eax, static_cast<u32>(inst.imm));
      emit_write_guest(code, inst.rt, code.eax);
      break;
    case V4AluOp::Ori:
      emit_read_guest(code, code.eax, inst.rs);
      code.or_(code.eax, static_cast<u32>(inst.imm));
      emit_write_guest(code, inst.rt, code.eax);
      break;
    case V4AluOp::Xori:
      emit_read_guest(code, code.eax, inst.rs);
      code.xor_(code.eax, static_cast<u32>(inst.imm));
      emit_write_guest(code, inst.rt, code.eax);
      break;
    case V4AluOp::Lui:
      code.mov(code.eax, static_cast<u32>(inst.imm) << 16u);
      emit_write_guest(code, inst.rt, code.eax);
      break;
    }
}

V4NativeFn compile_v4_alu(
    V4CodeArena &arena,
    const std::array<V4DecodedInstruction, kV4MaxBlockInstructions> &decoded,
    u32 count, u32 start_pc, u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 4096u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);

  for (u32 i = 0; i < count; ++i) {
    emit_v4_alu_instruction(code, decoded[i]);
    if (i == 0u) {
      emit_retire_incoming_load(code, v4_alu_write_reg(decoded[i]));
    }
  }

  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      start_pc + (count - 1u) * 4u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      0u);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      count * 4u);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cycles))], count);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, instructions))],
      count);
  emit_v4_block_return(code);
  code.ready();

  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}

V4NativeFn compile_v4_branch(
    V4CodeArena &arena,
    const std::array<V4DecodedInstruction, kV4MaxBlockInstructions> &prefix,
    u32 prefix_count, const V4DecodedControl &control,
    const V4DecodedInstruction &delay, u32 branch_pc, u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 2048u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);

  for (u32 i = 0; i < prefix_count; ++i) {
    emit_v4_alu_instruction(code, prefix[i]);
    if (i == 0u) {
      emit_retire_incoming_load(code, v4_alu_write_reg(prefix[i]));
    }
  }

  const u32 fallthrough = branch_pc + 8u;
  const u32 jump_target =
      ((branch_pc + 4u) & 0xF0000000u) | (control.imm26 << 2u);
  const u32 branch_target =
      branch_pc + 4u + static_cast<u32>(control.simm * 4);

  if (control.op == V4ControlOp::Beq ||
      control.op == V4ControlOp::Bne) {
    emit_read_guest(code, code.eax, control.rs);
    emit_read_guest(code, code.ecx, control.rt);
    code.cmp(code.eax, code.ecx);
    if (control.op == V4ControlOp::Beq) {
      code.sete(code.dl);
    } else {
      code.setne(code.dl);
    }
    code.movzx(code.edx, code.dl);
  } else if (control.op == V4ControlOp::Blez ||
             control.op == V4ControlOp::Bgtz ||
             control.op == V4ControlOp::Bltz ||
             control.op == V4ControlOp::Bgez ||
             control.op == V4ControlOp::Bltzal ||
             control.op == V4ControlOp::Bgezal) {
    emit_read_guest(code, code.eax, control.rs);
    code.cmp(code.eax, 0);
    switch (control.op) {
    case V4ControlOp::Blez: code.setle(code.dl); break;
    case V4ControlOp::Bgtz: code.setg(code.dl); break;
    case V4ControlOp::Bltz:
    case V4ControlOp::Bltzal: code.setl(code.dl); break;
    case V4ControlOp::Bgez:
    case V4ControlOp::Bgezal: code.setge(code.dl); break;
    default: break;
    }
    code.movzx(code.edx, code.dl);
    // Match the interpreter's broad REGIMM behavior: link variants write RA
    // regardless of whether the branch is taken, before the delay slot.
    if (control.op == V4ControlOp::Bltzal ||
        control.op == V4ControlOp::Bgezal) {
      code.mov(code.eax, branch_pc + 8u);
      emit_write_guest(code, 31u, code.eax);
    }
  } else if (control.op == V4ControlOp::Jr ||
             control.op == V4ControlOp::Jalr) {
    // Capture the dynamic target before either the link write or the delay slot.
    // This is observable when rs == rd or the delay slot rewrites rs.
    emit_read_guest(code, code.r8d, control.rs);
    if (control.op == V4ControlOp::Jalr) {
      code.mov(code.eax, branch_pc + 8u);
      emit_write_guest(code, control.rd, code.eax);
    }
  } else if (control.op == V4ControlOp::Jal) {
    code.mov(code.eax, branch_pc + 8u);
    emit_write_guest(code, 31u, code.eax);
  }

  // If the branch starts the block, it captured operands before the incoming
  // load retires. With a prefix, the first ALU instruction already retired it.
  if (prefix_count == 0u) {
    emit_retire_incoming_load(code, v4_control_write_reg(control));
  }

  // JAL/JALR's link is visible to the delay slot.
  emit_v4_alu_instruction(code, delay);

  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      branch_pc + 4u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      1u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      branch_pc);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, instructions))],
      prefix_count + 2u);

  if (control.op == V4ControlOp::J ||
      control.op == V4ControlOp::Jal) {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        jump_target);
    code.add(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cycles))],
        prefix_count + 3u);
  } else if (control.op == V4ControlOp::Jr ||
             control.op == V4ControlOp::Jalr) {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        code.r8d);
    code.add(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cycles))],
        prefix_count + 3u);
  } else {
    Label not_taken, selected;
    code.test(code.edx, code.edx);
    code.jz(not_taken);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        branch_target);
    code.add(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cycles))],
        prefix_count + 3u);
    code.jmp(selected);

    code.L(not_taken);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        fallthrough);
    code.add(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cycles))],
        prefix_count + 2u);
    code.L(selected);
  }

  emit_v4_block_return(code);
  code.ready();

  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}


V4NativeFn compile_v4_load(
    V4CodeArena &arena, const V4DecodedLoad &load,
    const std::array<V4DecodedInstruction, kV4MaxBlockInstructions> &tail,
    u32 tail_count, u32 start_pc, u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 2048u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  code.setDefaultJmpNEAR(true);
  Label ram, scratch, loaded, bail;

  // Form the address before retiring an incoming load. If rs is the delayed
  // destination, R3000A semantics require this LW to see the old rs value.
  emit_read_guest(code, code.eax, load.rs);
  code.add(code.eax, static_cast<u32>(load.simm));
  if (load.op == V4LoadOp::Lh || load.op == V4LoadOp::Lhu) {
    code.test(code.eax, 1u);
    code.jnz(bail);
  } else if (load.op == V4LoadOp::Lw) {
    code.test(code.eax, 3u);
    code.jnz(bail);
  }

  auto emit_memory_read = [&]() {
    switch (load.op) {
    case V4LoadOp::Lb:
      code.movzx(code.r8d, code.byte[code.rcx + code.rdx]);
      code.shl(code.r8d, 24);
      code.sar(code.r8d, 24);
      break;
    case V4LoadOp::Lh:
      code.movzx(code.r8d, code.word[code.rcx + code.rdx]);
      code.shl(code.r8d, 16);
      code.sar(code.r8d, 16);
      break;
    case V4LoadOp::Lw:
      code.mov(code.r8d, code.dword[code.rcx + code.rdx]);
      break;
    case V4LoadOp::Lbu:
      code.movzx(code.r8d, code.byte[code.rcx + code.rdx]);
      break;
    case V4LoadOp::Lhu:
      code.movzx(code.r8d, code.word[code.rcx + code.rdx]);
      break;
    }
  };

  code.mov(code.edx, code.eax);
  code.and_(code.edx, 0x1FFFFFFFu);
  code.cmp(code.edx, code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, mapped_main_ram_size))]);
  code.jb(ram);
  code.cmp(code.edx, 0x1F800000u);
  code.jb(bail);
  code.cmp(code.edx, 0x1F801000u);
  code.jae(bail);
  code.L(scratch);
  code.sub(code.edx, 0x1F800000u);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, scratchpad))]);
  code.test(code.rcx, code.rcx);
  code.jz(bail);
  emit_memory_read();
  code.xor_(code.r9d, code.r9d);
  code.jmp(loaded);

  code.L(ram);
  code.and_(code.edx, psx::RAM_SIZE - 1u);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, main_ram))]);
  code.test(code.rcx, code.rcx);
  code.jz(bail);
  emit_memory_read();
  code.mov(code.r9d, 4u);

  code.L(loaded);
  // schedule_load(rt, value) cancels an older delayed write to the same
  // register, then advance_load_delay() retires the older load.
  emit_retire_incoming_load(code, load.rt);
  if (load.rt != 0u) {
    code.mov(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, pending_load_reg))],
        static_cast<u32>(load.rt));
    code.mov(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, pending_load_value))],
        code.r8d);
  }

  // Keep executing through a safe ALU tail instead of turning every load into
  // a one-instruction native island. The first following instruction captures
  // operands before the newly scheduled load retires, matching R3000A delay
  // semantics; a write to the same register cancels that load.
  for (u32 i = 0; i < tail_count; ++i) {
    emit_v4_alu_instruction(code, tail[i]);
    if (i == 0u) {
      emit_retire_incoming_load(code, v4_alu_write_reg(tail[i]));
    }
  }

  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      start_pc + tail_count * 4u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      0u);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      (tail_count + 1u) * 4u);
  code.add(code.r9d, tail_count + 2u);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cycles))],
      code.r9d);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, instructions))],
      tail_count + 1u);
  emit_v4_block_return(code);

  code.L(bail);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, block_bail))], 1u);
  emit_v4_block_return(code);
  code.ready();

  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}

V4ResidentDispatchFn install_v4_resident_dispatch(
    V4CodeArena &arena, void *&block_return) {
  using namespace Xbyak;
  constexpr size_t kReservation = 1024u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  code.setDefaultJmpNEAR(true);
  Label loop, after_block, done;

  code.push(code.rbx);
  code.push(code.r12);
  code.push(code.r13);
  code.push(code.r14);
  code.push(code.r15);
#if defined(_WIN32)
  code.sub(code.rsp, 32);
  code.mov(code.rbx, code.rcx);
#else
  code.mov(code.rbx, code.rdi);
#endif

  // V4 blocks are internal fragments, not ABI-callable functions. Keep the
  // resident state and GPR base pinned for the whole dispatcher lifetime.
  code.mov(code.r11, code.rbx);
  code.mov(code.r10, code.ptr[
      code.rbx + static_cast<int>(offsetof(V4NativeState, gpr))]);

  code.mov(code.r12, code.ptr[
      code.rbx + static_cast<int>(offsetof(V4NativeState, dispatch_top))]);
  code.mov(code.r13d, code.dword[
      code.rbx + static_cast<int>(offsetof(V4NativeState, cache_epoch))]);

  code.L(loop);
  code.mov(code.eax, code.dword[
      code.rbx + static_cast<int>(offsetof(V4NativeState, pc))]);
  code.mov(code.edx, code.eax);
  code.shr(code.eax, 12);
  code.mov(code.r14, code.ptr[code.r12 + code.rax * 8]);
  code.test(code.r14, code.r14);
  code.jz(done);

  code.mov(code.eax, code.edx);
  code.shr(code.eax, 2);
  code.and_(code.eax, 0x3FFu);
  code.imul(code.eax, code.eax,
             static_cast<int>(sizeof(V4DispatchEntry)));
  code.lea(code.r15, code.ptr[code.r14 + code.rax]);
  code.cmp(code.dword[
                code.r15 +
                static_cast<int>(offsetof(V4DispatchEntry, cache_epoch))],
            code.r13d);
  code.jne(done);

  code.mov(code.r14, code.ptr[
      code.r15 + static_cast<int>(offsetof(V4DispatchEntry, block))]);
  code.test(code.r14, code.r14);
  code.jz(done);

  {
    Label memory_ok;
    code.cmp(code.byte[
        code.r14 + static_cast<int>(offsetof(V4Block, has_memory))], 0u);
    code.je(memory_ok);
    code.cmp(code.dword[
        code.rbx +
        static_cast<int>(offsetof(V4NativeState, memory_fastpath_allowed))],
        0u);
    code.je(done);
    code.L(memory_ok);
  }

  {
    Label uncached, validity_ok;
    code.cmp(code.byte[
        code.r14 + static_cast<int>(offsetof(V4Block, cacheable))], 0u);
    code.je(uncached);

    // Cached code: exact guest I-cache line generation is authoritative.
    code.mov(code.rax, code.ptr[
        code.rbx +
        static_cast<int>(offsetof(V4NativeState, icache_generations))]);
    code.test(code.rax, code.rax);
    code.jz(done);
    code.movzx(code.ecx, code.word[
        code.r14 + static_cast<int>(offsetof(V4Block, icache_index))]);
    code.mov(code.edx, code.dword[code.rax + code.rcx * 4]);
    code.cmp(code.edx, code.dword[
        code.r14 + static_cast<int>(offsetof(V4Block, icache_generation))]);
    code.jne(done);
    code.jmp(validity_ok);

    // Uncached code: RAM writes are observed immediately, so retain the
    // physical-page generation guard.
    code.L(uncached);
    code.mov(code.rax, code.ptr[
        code.rbx +
        static_cast<int>(offsetof(V4NativeState, code_page_generations))]);
    code.test(code.rax, code.rax);
    code.jz(done);
    code.mov(code.ecx, code.dword[
        code.r14 + static_cast<int>(offsetof(V4Block, phys_page))]);
    code.mov(code.edx, code.dword[code.rax + code.rcx * 4]);
    code.cmp(code.edx, code.dword[
        code.r14 +
        static_cast<int>(offsetof(V4Block, code_page_generation))]);
    code.jne(done);
    code.L(validity_ok);
  }

  code.mov(code.eax, code.dword[
      code.rbx + static_cast<int>(offsetof(V4NativeState, instructions))]);
  code.add(code.eax, code.dword[
      code.r14 + static_cast<int>(offsetof(V4Block, instruction_count))]);
  code.cmp(code.eax, code.dword[
      code.rbx + static_cast<int>(offsetof(V4NativeState, instruction_budget))]);
  code.ja(done);

  {
    Label cycle_ok, strict_cycle_budget;
    code.cmp(code.dword[
        code.r14 + static_cast<int>(offsetof(V4Block, instruction_count))], 1u);
    code.jne(strict_cycle_budget);

    // Cpu::run_slice() / Cpu::step() already allow one architectural
    // instruction to overshoot the remaining cycle budget. Preserve that
    // contract for a one-instruction native block instead of rejecting it only
    // to execute the same instruction through the interpreter.
    code.mov(code.eax, code.dword[
        code.rbx + static_cast<int>(offsetof(V4NativeState, cycles))]);
    code.cmp(code.eax, code.dword[
        code.rbx + static_cast<int>(offsetof(V4NativeState, cycle_budget))]);
    code.jb(cycle_ok);
    code.jmp(done);

    code.L(strict_cycle_budget);
    code.mov(code.eax, code.dword[
        code.rbx + static_cast<int>(offsetof(V4NativeState, cycles))]);
    code.add(code.eax, code.dword[
        code.r14 + static_cast<int>(offsetof(V4Block, max_cycles))]);
    code.cmp(code.eax, code.dword[
        code.rbx + static_cast<int>(offsetof(V4NativeState, cycle_budget))]);
    code.ja(done);
    code.L(cycle_ok);
  }

  code.mov(code.rax, code.ptr[
      code.r15 + static_cast<int>(offsetof(V4DispatchEntry, code))]);
  code.test(code.rax, code.rax);
  code.jz(done);

  // Tail-jump into translated code. The block returns here via the pointer
  // pinned in V4NativeState, eliminating call/ret and per-block ABI setup.
  code.jmp(code.rax);

  code.L(after_block);
  block_return = const_cast<u8 *>(code.getCurr());
  code.cmp(code.dword[
      code.rbx + static_cast<int>(offsetof(V4NativeState, block_bail))], 0u);
  code.jne(done);
  code.inc(code.dword[
      code.rbx + static_cast<int>(offsetof(V4NativeState, block_entries))]);
  {
    Label not_memory;
    code.cmp(code.byte[
        code.r14 + static_cast<int>(offsetof(V4Block, has_memory))], 0u);
    code.je(not_memory);
    code.inc(code.dword[
        code.rbx + static_cast<int>(offsetof(V4NativeState, memory_entries))]);
    code.L(not_memory);
  }
  code.jmp(loop);

  code.L(done);
#if defined(_WIN32)
  code.add(code.rsp, 32);
#endif
  code.pop(code.r15);
  code.pop(code.r14);
  code.pop(code.r13);
  code.pop(code.r12);
  code.pop(code.rbx);
  code.ret();
  code.ready();
  const size_t code_size = code.getSize();
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4ResidentDispatchFn>(buffer);
}

#endif // VIBESTATION_JIT_V4_X64

} // namespace

struct CpuJitV4Backend::Impl {
#if VIBESTATION_JIT_V4_X64
  V4CodeArena arena;
  std::unique_ptr<V4DispatchPage *[]> dispatch_top;
  std::vector<std::unique_ptr<V4DispatchPage>> dispatch_pages;
  std::unique_ptr<V4Block[]> blocks;
  size_t block_count = 0u;
  u32 cache_epoch = 1u;
  JitCodePageBitmap<29u, 12u> code_pages;
  std::array<u32, kV4PhysPageCount> page_generations{};
  V4ResidentDispatchFn resident_dispatch = nullptr;
  void *resident_block_return = nullptr;
  size_t permanent_code_bytes = 0u;
  bool initialization_attempted = false;
  bool initialized = false;

  bool ensure_initialized() {
    if (initialized) {
      return true;
    }
    if (initialization_attempted) {
      return false;
    }
    initialization_attempted = true;
    if (!arena.ensure_available()) {
      return false;
    }
    try {
      dispatch_top =
          std::make_unique<V4DispatchPage *[]>(kV4DispatchTopCount);
      blocks = std::make_unique<V4Block[]>(kV4MaxBlocks);
    } catch (...) {
      return false;
    }
    page_generations.fill(1u);
    resident_dispatch =
        install_v4_resident_dispatch(arena, resident_block_return);
    if (resident_dispatch == nullptr || resident_block_return == nullptr) {
      return false;
    }
    permanent_code_bytes = arena.bytes_used();
    initialized = true;
    return true;
  }

  bool native_available() const {
    return !initialization_attempted || initialized;
  }

  V4DispatchEntry *dispatch_entry(u32 pc, bool create) {
    const size_t top_index = static_cast<size_t>(pc >> 12u);
    V4DispatchPage *page = dispatch_top[top_index];
    if (page == nullptr && create) {
      auto owned = std::make_unique<V4DispatchPage>();
      page = owned.get();
      dispatch_pages.push_back(std::move(owned));
      dispatch_top[top_index] = page;
    }
    if (page == nullptr) {
      return nullptr;
    }
    return &page->entries[(pc >> 2u) & 0x3FFu];
  }

  V4Block *lookup(u32 pc, bool cacheable, u32 icache_generation) {
    V4DispatchEntry *entry = dispatch_entry(pc, false);
    if (entry == nullptr || entry->cache_epoch != cache_epoch ||
        entry->block == nullptr) {
      return nullptr;
    }
    V4Block *block = entry->block;
    if (block->cacheable != cacheable) {
      return nullptr;
    }
    if (cacheable) {
      // Cached translations are tied to the exact 16-byte guest-visible
      // I-cache snapshot. A write elsewhere in the same 4 KiB RAM page must
      // not evict them.
      if (block->icache_generation != icache_generation) {
        return nullptr;
      }
    } else if (block->phys_page >= kV4PhysPageCount ||
               block->code_page_generation != page_generations[block->phys_page]) {
      // Uncached/KSEG1 execution observes RAM directly, so page generations
      // remain the conservative SMC guard for those blocks.
      return nullptr;
    }
    return block;
  }

  void install(u32 pc, V4Block *block) {
    V4DispatchEntry *entry = dispatch_entry(pc, true);
    if (entry == nullptr) {
      return;
    }
    entry->code = block != nullptr ? reinterpret_cast<void *>(block->fn) : nullptr;
    entry->block = block;
    entry->cache_epoch = cache_epoch;
  }

  void reset_translations() {
    if (!initialized) {
      return;
    }
    arena.reset_to(permanent_code_bytes);
    block_count = 0u;
    code_pages.clear();
    ++cache_epoch;
    if (cache_epoch == 0u) {
      // Epoch wrap is practically unreachable. If it ever happens, clear only
      // the allocated sparse pages rather than the entire 4 GiB PC table.
      for (auto &page : dispatch_pages) {
        page->entries = {};
      }
      cache_epoch = 1u;
    }
  }

  V4Block *allocate_block() {
    if (block_count >= kV4MaxBlocks) {
      return nullptr;
    }
    V4Block *block = &blocks[block_count++];
    *block = {};
    return block;
  }

  V4Block *compile_block(Cpu &cpu, CpuBackendStats &stats, u32 start_pc,
                         bool cacheable, u32 icache_generation) {
    V4Block *block = allocate_block();
    if (block == nullptr) {
      return nullptr;
    }

    block->start_pc = start_pc;
    block->cacheable = cacheable;
    block->icache_index = static_cast<u16>((start_pc >> 4) & 0xFFu);
    block->icache_generation = cacheable ? icache_generation : 0u;
    const u32 start_phys = psx::mask_address(start_pc);
    block->phys_page = start_phys >> kV4PhysPageShift;
    block->code_page_generation = page_generations[block->phys_page];

    // A cached native block never crosses a 16-byte guest I-cache line.
    // One generation tag therefore identifies the complete instruction snapshot
    // used to compile it, and we never speculatively refill a future line.
    const u32 line_instructions =
        cacheable ? ((16u - (start_pc & 0x0Fu)) >> 2u)
                  : kV4MaxBlockInstructions;
    const u32 page_instructions =
        (0x1000u - (start_phys & 0x0FFFu)) >> 2u;
    const u32 decode_limit =
        std::min({kV4MaxBlockInstructions, line_instructions,
                  page_instructions});

    auto read_visible = [&](u32 addr, u32 &bits) {
      return cpu.read_visible_instruction_for_backend(addr, bits);
    };

    std::array<V4DecodedInstruction, kV4MaxBlockInstructions> decoded{};
    u32 count = 0u;
    for (; count < decode_limit; ++count) {
      V4DecodedInstruction inst{};
      u32 bits = 0u;
      if (!read_visible(start_pc + count * 4u, bits) ||
          !decode_v4_alu(bits, inst)) {
        break;
      }
      decoded[count] = inst;
    }

    V4DecodedControl control{};
    V4DecodedInstruction delay{};
    u32 control_bits = 0u;
    u32 delay_bits = 0u;
    const u32 branch_pc = start_pc + count * 4u;
    const bool control_pair_within_decode_limit = count + 2u <= decode_limit;
    const bool simple_control =
        control_pair_within_decode_limit &&
        read_visible(branch_pc, control_bits) &&
        read_visible(branch_pc + 4u, delay_bits) &&
        decode_v4_control(control_bits, control) &&
        decode_v4_alu(delay_bits, delay);

    V4DecodedLoad load{};
    u32 load_bits = 0u;
    const bool simple_load =
        count == 0u && read_visible(start_pc, load_bits) &&
        decode_v4_load(load_bits, load);
    std::array<V4DecodedInstruction, kV4MaxBlockInstructions> load_tail{};
    u32 load_tail_count = 0u;
    if (simple_load) {
      for (u32 i = 1u; i < decode_limit; ++i) {
        V4DecodedInstruction inst{};
        u32 bits = 0u;
        if (!read_visible(start_pc + i * 4u, bits) ||
            !decode_v4_alu(bits, inst)) {
          break;
        }
        load_tail[load_tail_count++] = inst;
      }
    }

    if (count == 0u && !simple_control && !simple_load) {
      block->interpreter_only = true;
      install(start_pc, block);
      ++stats.interpreter_only_blocks;
      stats.block_count = static_cast<u32>(block_count);
      return block;
    }

    ++stats.native_compile_attempts;
    try {
      V4NativeFn entry = nullptr;
      if (simple_control) {
        entry = compile_v4_branch(
            arena, decoded, count, control, delay, branch_pc,
            block->code_size);
      } else if (simple_load) {
        entry = compile_v4_load(arena, load, load_tail, load_tail_count,
                                start_pc, block->code_size);
      } else {
        entry = compile_v4_alu(
            arena, decoded, count, start_pc, block->code_size);
      }
      if (entry == nullptr) {
        --block_count;
        ++stats.native_compile_failures;
        return nullptr;
      }
      block->fn = entry;
      block->instruction_count =
          simple_control ? count + 2u
                         : (simple_load ? load_tail_count + 1u : count);
      block->max_cycles =
          simple_control ? count + 3u
                         : (simple_load ? load_tail_count + 6u : count);
      block->has_control = simple_control;
      block->has_memory = simple_load;
    } catch (...) {
      --block_count;
      ++stats.native_compile_failures;
      return nullptr;
    }

    const u32 translated_count =
        simple_control ? count + 2u
                       : (simple_load ? load_tail_count + 1u : count);
    for (u32 i = 0; i < translated_count; ++i) {
      code_pages.mark_address(psx::mask_address(start_pc + i * 4u));
    }
    install(start_pc, block);

    ++stats.native_compile_successes;
    ++stats.native_blocks_compiled;
    if (simple_control) {
      ++stats.native_branch_tail_blocks_compiled;
    } else if (simple_load) {
      ++stats.native_memory_blocks_compiled;
    } else {
      ++stats.native_alu_blocks_compiled;
    }
    ++stats.native_blocks;
    stats.block_count = static_cast<u32>(block_count);
    stats.native_code_bytes = arena.bytes_used();
    stats.code_bytes = arena.bytes_used();
    return block;
  }
#else
  bool native_available() const { return false; }
  void reset_translations() {}
#endif
};

CpuJitV4Backend::CpuJitV4Backend(Cpu &cpu)
    : cpu_(cpu), impl_(std::make_unique<Impl>()) {
  stats_.available = true;
  stats_.native_available = impl_->native_available();
}

CpuJitV4Backend::~CpuJitV4Backend() = default;

CpuRunSliceResult CpuJitV4Backend::run_slice(u32 max_cycles,
                                             u32 max_instructions) {
  CpuRunSliceResult result{};
  if (max_cycles == 0u || max_instructions == 0u) {
    return result;
  }

  stats_.active = true;
  stats_.native_available = impl_->native_available();

  const auto fallback_one = [&]() {
    const u32 consumed = cpu_.step();
    result.cycles += consumed;
    ++result.instructions;
    ++stats_.fallback_instructions;
    ++stats_.interpreter_fallback_steps;
    ++stats_.fallback_exits;
  };

#if !VIBESTATION_JIT_V4_X64
  while (result.cycles < max_cycles &&
         result.instructions < max_instructions) {
    fallback_one();
  }
  return result;
#else
  if (!impl_->ensure_initialized()) {
    stats_.native_available = false;
    while (result.cycles < max_cycles &&
           result.instructions < max_instructions) {
      fallback_one();
    }
    return result;
  }

  // Keep diagnostic modes on the interpreter during bring-up. Their callbacks
  // observe instruction-by-instruction state and are deliberately not a V4 hot
  // path concern.
  if (g_trace_cpu || g_cpu_deep_diagnostics || g_log_fmv_diagnostics ||
      g_cpu_backend_compare_test_force_interpreter ||
      g_cpu_backend_compare_irq_on_branch) {
    while (result.cycles < max_cycles &&
           result.instructions < max_instructions) {
      fallback_one();
    }
    return result;
  }

  while (result.cycles < max_cycles &&
         result.instructions < max_instructions) {
    // Phase 2 will make branch/load delay state resident in V4. Until then,
    // never enter a native block while those architectural states are live.
    bool unsafe_state = false;
    if ((cpu_.pc_ & 3u) != 0u || cpu_.next_pc_ != cpu_.pc_ + 4u) {
      ++stats_.native_reject_pc_state;
      unsafe_state = true;
    }
    if (cpu_.pending_delay_slot_ || cpu_.pending_branch_taken_ ||
        cpu_.pending_branch_pc_ != 0u) {
      ++stats_.native_reject_branch_delay_state;
      unsafe_state = true;
    }
    if (cpu_.next_load_.reg != 0u) {
      ++stats_.native_reject_load_delay_state;
      unsafe_state = true;
    }
    if (unsafe_state) {
      ++stats_.native_reject_unsafe_state;
      fallback_one();
      continue;
    }

    // Match Cpu::step()'s hardware IRQ line synchronization before deciding
    // whether native execution may cross the next instruction boundary.
    if (cpu_.sys_->irq_pending()) {
      cpu_.cop0_cause_ |= (1u << 10);
    } else {
      cpu_.cop0_cause_ &= ~(1u << 10);
    }
    if (cpu_.check_irq()) {
      ++stats_.native_reject_irq_state;
      fallback_one();
      continue;
    }

    const bool cacheable = cpu_.instruction_cacheable(cpu_.pc_);
    u32 icache_generation = 0u;
    if (cacheable) {
      // Refill only the line containing the instruction about to execute.
      // The JIT compiler consumes that guest-visible snapshot rather than RAM.
      if (cpu_.prepare_instruction_cache_line_for_backend(cpu_.pc_)) {
        constexpr u32 kRefillCycles = 4u;
        cpu_.cycles_ += kRefillCycles;
        result.cycles += kRefillCycles;
        stats_.executed_cycles += kRefillCycles;
      }
      icache_generation =
          cpu_.instruction_cache_generation_for_backend(cpu_.pc_);
    }

    V4Block *block =
        impl_->lookup(cpu_.pc_, cacheable, icache_generation);
    if (block != nullptr) {
      ++stats_.cache_hits;
    } else {
      ++stats_.cache_misses;
      block = impl_->compile_block(cpu_, stats_, cpu_.pc_, cacheable,
                                   icache_generation);
      if (block == nullptr) {
        // A full arena/metadata slab is recycled in bulk; no per-block
        // executable allocations or frees are needed.
        impl_->reset_translations();
        stats_.native_blocks = 0u;
        stats_.block_count = 0u;
        stats_.interpreter_only_blocks = 0u;
        ++stats_.flushes;
        block = impl_->compile_block(cpu_, stats_, cpu_.pc_, cacheable,
                                   icache_generation);
      }
    }

    if (block == nullptr || block->interpreter_only || block->fn == nullptr) {
      ++stats_.native_reject_unsupported_instruction;
      fallback_one();
      continue;
    }

    const u32 remaining_cycles = max_cycles - result.cycles;
    const u32 remaining_instructions = max_instructions - result.instructions;

    const u32 start_pc = cpu_.pc_;
    V4NativeState native{};
    native.gpr = cpu_.gpr_;
    native.dispatch_top = impl_->dispatch_top.get();
    native.icache_generations = cpu_.icache_generation_.data();
    native.code_page_generations = impl_->page_generations.data();
    native.block_return = impl_->resident_block_return;
    native.main_ram = cpu_.sys_->jit_main_ram_data_mut();
    native.scratchpad = cpu_.sys_->jit_scratchpad_data_mut();
    native.mapped_main_ram_size = cpu_.sys_->jit_mapped_main_ram_size();
    native.memory_fastpath_allowed =
        (!g_trace_ram && !g_trace_bus && !g_ram_watch_diagnostics) ? 1u : 0u;
    native.cache_epoch = impl_->cache_epoch;
    native.pc = start_pc;
    native.pending_load_reg = cpu_.load_.reg;
    native.pending_load_value = cpu_.load_.value;
    native.cycle_budget = remaining_cycles;
    native.instruction_budget = remaining_instructions;
    impl_->resident_dispatch(&native);

    if (native.instructions == 0u || native.block_entries == 0u) {
      ++stats_.native_reject_budget;
      ++stats_.budget_exits;
      fallback_one();
      continue;
    }

    cpu_.current_pc_ = native.last_pc;
    cpu_.pc_ = native.pc;
    cpu_.next_pc_ = native.pc + 4u;
    cpu_.in_delay_slot_ = native.last_in_delay_slot != 0u;
    cpu_.active_branch_pc_ = native.active_branch_pc;
    cpu_.pending_delay_slot_ = false;
    cpu_.pending_branch_taken_ = false;
    cpu_.pending_branch_pc_ = 0u;
    cpu_.load_ = {native.pending_load_reg, native.pending_load_value};
    cpu_.next_load_ = {0u, 0u};
    cpu_.exception_raised_ = false;
    cpu_.cycle_penalty_ = 0u;
    cpu_.executing_step_ = false;
    cpu_.gpr_[0] = 0u;
    cpu_.cycles_ += native.cycles;

    result.cycles += native.cycles;
    result.instructions += native.instructions;
    stats_.native_block_entries += native.block_entries;
    stats_.native_memory_fastpath_loads += native.memory_entries;
    if (block->has_control) {
      ++stats_.native_branch_tail_entries;
    } else {
      ++stats_.native_alu_block_entries;
    }
    if (native.block_entries > 1u) {
      ++stats_.native_chain_entries;
      stats_.native_linked_transitions += native.block_entries - 1u;
      stats_.native_chain_max_blocks =
          std::max<u64>(stats_.native_chain_max_blocks, native.block_entries);
    }
    stats_.native_instructions += native.instructions;
    stats_.native_cycles += native.cycles;
    stats_.optimized_instructions += native.instructions;
    stats_.executed_cycles += native.cycles;
  }
  return result;
#endif
}

void CpuJitV4Backend::invalidate_range(u32 phys_or_normalized_addr,
                                       u32 size_bytes) {
  ++stats_.invalidation_queries;
  if (size_bytes == 0u) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

#if VIBESTATION_JIT_V4_X64
  if (!impl_->initialized || impl_->block_count == 0u) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  u32 remaining = size_bytes;
  u32 address = phys_or_normalized_addr;
  bool touches_code = false;
  while (remaining != 0u) {
    const u32 phys = psx::mask_address(address);
    const u32 page = phys >> kV4PhysPageShift;
    if (impl_->code_pages.test_page(page)) {
      touches_code = true;
      u32 &generation = impl_->page_generations[page];
      if (++generation == 0u) {
        generation = 1u;
      }
    }
    const u32 bytes_to_page = 0x1000u - (phys & 0xFFFu);
    const u32 advance = std::min(remaining, bytes_to_page);
    remaining -= advance;
    address += advance;
  }

  if (!touches_code) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  // Keep the arena and every unrelated translation alive. Stale blocks on the
  // written physical page fail their generation guard and are lazily replaced
  // only if execution returns to them.
  ++stats_.invalidations;
#else
  (void)phys_or_normalized_addr;
#endif
}

void CpuJitV4Backend::begin_frame(u32 frame_index) {
  current_frame_ = frame_index;
  stats_.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV4;
}

void CpuJitV4Backend::flush() {
  impl_->reset_translations();
  ++stats_.flushes;
  stats_.native_blocks = 0u;
  stats_.block_count = 0u;
  stats_.interpreter_only_blocks = 0u;
  stats_.native_code_bytes = 0u;
  stats_.code_bytes = 0u;
}

CpuBackendStats CpuJitV4Backend::stats() const {
  CpuBackendStats out = stats_;
  out.available = true;
  out.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV4;
  out.native_available = impl_->native_available();
  return out;
}
