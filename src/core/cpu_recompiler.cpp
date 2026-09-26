#include "cpu_recompiler.h"

#include "jit_code_page_bitmap.h"
#include "system.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <unordered_map>
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

u32 v4_normalize_code_phys(u32 phys) {
  // The RAM_SIZE register can expose an 8 MiB window, but the machine still
  // has 2 MiB of backing RAM. Code-page ownership must follow the backing
  // bytes so writes through one mirror invalidate translations through another.
  if (phys < psx::RAM_MAX_SIZE) {
    return phys & (psx::RAM_SIZE - 1u);
  }
  return phys;
}

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

enum class V4OverflowAluOp : u8 {
  Add,
  Sub,
  Addi,
};

struct V4DecodedOverflowAlu {
  V4OverflowAluOp op = V4OverflowAluOp::Add;
  u8 rs = 0;
  u8 rt = 0;
  u8 rd = 0;
  s32 simm = 0;
};

bool decode_v4_overflow_alu(u32 bits, V4DecodedOverflowAlu &out) {
  out = {};
  out.rs = static_cast<u8>((bits >> 21) & 0x1Fu);
  out.rt = static_cast<u8>((bits >> 16) & 0x1Fu);
  out.rd = static_cast<u8>((bits >> 11) & 0x1Fu);
  out.simm = static_cast<s32>(static_cast<s16>(bits & 0xFFFFu));
  const u32 primary = (bits >> 26) & 0x3Fu;
  if (primary == 0u) {
    switch (bits & 0x3Fu) {
    case 0x20: out.op = V4OverflowAluOp::Add; return true;
    case 0x22: out.op = V4OverflowAluOp::Sub; return true;
    default: return false;
    }
  }
  if (primary == 0x08u) {
    out.op = V4OverflowAluOp::Addi;
    return true;
  }
  return false;
}

enum class V4HiLoOp : u8 {
  Mfhi,
  Mthi,
  Mflo,
  Mtlo,
};

struct V4DecodedHiLo {
  V4HiLoOp op = V4HiLoOp::Mfhi;
  u8 rs = 0;
  u8 rd = 0;
};

bool decode_v4_hilo(u32 bits, V4DecodedHiLo &out) {
  if (((bits >> 26) & 0x3Fu) != 0u) {
    return false;
  }
  out = {};
  out.rs = static_cast<u8>((bits >> 21) & 0x1Fu);
  out.rd = static_cast<u8>((bits >> 11) & 0x1Fu);
  switch (bits & 0x3Fu) {
  case 0x10: out.op = V4HiLoOp::Mfhi; return true;
  case 0x11: out.op = V4HiLoOp::Mthi; return true;
  case 0x12: out.op = V4HiLoOp::Mflo; return true;
  case 0x13: out.op = V4HiLoOp::Mtlo; return true;
  default: return false;
  }
}

enum class V4MulDivOp : u8 {
  Mult,
  Multu,
  Div,
  Divu,
};

struct V4DecodedMulDiv {
  V4MulDivOp op = V4MulDivOp::Mult;
  u8 rs = 0;
  u8 rt = 0;
};

bool decode_v4_muldiv(u32 bits, V4DecodedMulDiv &out) {
  if (((bits >> 26) & 0x3Fu) != 0u) {
    return false;
  }
  out = {};
  out.rs = static_cast<u8>((bits >> 21) & 0x1Fu);
  out.rt = static_cast<u8>((bits >> 16) & 0x1Fu);
  switch (bits & 0x3Fu) {
  case 0x18: out.op = V4MulDivOp::Mult; return true;
  case 0x19: out.op = V4MulDivOp::Multu; return true;
  case 0x1A: out.op = V4MulDivOp::Div; return true;
  case 0x1B: out.op = V4MulDivOp::Divu; return true;
  default: return false;
  }
}

enum class V4ExceptionOp : u8 {
  Syscall,
  Break,
};

struct V4DecodedException {
  V4ExceptionOp op = V4ExceptionOp::Syscall;
};

bool decode_v4_exception(u32 bits, V4DecodedException &out) {
  if (((bits >> 26) & 0x3Fu) != 0u) {
    return false;
  }
  switch (bits & 0x3Fu) {
  case 0x0C:
    out.op = V4ExceptionOp::Syscall;
    return true;
  case 0x0D:
    out.op = V4ExceptionOp::Break;
    return true;
  default:
    return false;
  }
}

enum class V4Cop0Op : u8 {
  Mfc0,
  Mtc0,
  Rfe,
};

struct V4DecodedCop0 {
  V4Cop0Op op = V4Cop0Op::Mfc0;
  u8 rt = 0;
  u8 rd = 0;
};

bool decode_v4_cop0(u32 bits, V4DecodedCop0 &out) {
  if (((bits >> 26) & 0x3Fu) != 0x10u) {
    return false;
  }
  out = {};
  out.rt = static_cast<u8>((bits >> 16) & 0x1Fu);
  out.rd = static_cast<u8>((bits >> 11) & 0x1Fu);
  const u32 sub = (bits >> 21) & 0x1Fu;
  if (sub == 0x00u) {
    out.op = V4Cop0Op::Mfc0;
    return true;
  }
  if (sub == 0x04u) {
    out.op = V4Cop0Op::Mtc0;
    return true;
  }
  if (sub == 0x10u && (bits & 0x3Fu) == 0x10u) {
    out.op = V4Cop0Op::Rfe;
    return true;
  }
  return false;
}

enum class V4LoadOp : u8 {
  Lb,
  Lh,
  Lwl,
  Lw,
  Lbu,
  Lhu,
  Lwr,
};

struct V4DecodedLoad {
  V4LoadOp op = V4LoadOp::Lw;
  u8 rs = 0;
  u8 rt = 0;
  s32 simm = 0;
};

enum class V4StoreOp : u8 {
  Sb,
  Sh,
  Swl,
  Sw,
  Swr,
};

struct V4DecodedStore {
  V4StoreOp op = V4StoreOp::Sw;
  u8 rs = 0;
  u8 rt = 0;
  s32 simm = 0;
};

bool decode_v4_store(u32 bits, V4DecodedStore &out) {
  switch ((bits >> 26) & 0x3Fu) {
  case 0x28: out.op = V4StoreOp::Sb; break;
  case 0x29: out.op = V4StoreOp::Sh; break;
  case 0x2A: out.op = V4StoreOp::Swl; break;
  case 0x2B: out.op = V4StoreOp::Sw; break;
  case 0x2E: out.op = V4StoreOp::Swr; break;
  default: return false;
  }
  out.rs = static_cast<u8>((bits >> 21) & 0x1Fu);
  out.rt = static_cast<u8>((bits >> 16) & 0x1Fu);
  out.simm = static_cast<s32>(static_cast<s16>(bits & 0xFFFFu));
  return true;
}

bool decode_v4_load(u32 bits, V4DecodedLoad &out) {
  switch ((bits >> 26) & 0x3Fu) {
  case 0x20: out.op = V4LoadOp::Lb; break;
  case 0x21: out.op = V4LoadOp::Lh; break;
  case 0x22: out.op = V4LoadOp::Lwl; break;
  case 0x23: out.op = V4LoadOp::Lw; break;
  case 0x24: out.op = V4LoadOp::Lbu; break;
  case 0x25: out.op = V4LoadOp::Lhu; break;
  case 0x26: out.op = V4LoadOp::Lwr; break;
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

u32 v4_hot_mmio_read16(System *sys, u32 phys) {
  return sys != nullptr ? sys->jit_read16_hot_mmio(phys) : 0x10000u;
}

struct V4DispatchPage;

struct V4NativeState {
  u32 *gpr = nullptr;
  Cpu *cpu = nullptr;
  System *system = nullptr;
  V4DispatchPage **dispatch_top = nullptr;
  u32 *icache_generations = nullptr;
  u32 *icache_tags = nullptr;
  u32 *icache_words = nullptr;
  bool *icache_valid = nullptr;
  u32 icache_line_stride = 0;
  const u32 *code_page_generations = nullptr;
  const u64 *code_page_bits = nullptr;
  const u64 *code_line_bits = nullptr;
  void *block_return = nullptr;
  void *pending_delay_fn = nullptr;
  u8 *main_ram = nullptr;
  u8 *scratchpad = nullptr;
  u32 mapped_main_ram_size = 0;
  u32 memory_fastpath_allowed = 0;
  u32 block_bail = 0;
  u32 memory_entries = 0;
  u32 store_entries = 0;
  u32 store_phys = 0;
  u32 store_byte_offset = 0;
  u32 *cop0_regs = nullptr;
  u32 cop0_jumpdest = 0;
  u32 cop0_badvaddr = 0;
  u32 cop0_sr = 0;
  u32 cop0_cause = 0;
  u32 cop0_epc = 0;
  u32 hi = 0;
  u32 lo = 0;
  u64 muldiv_result_ready_cycle = 0;
  u64 cpu_cycle_base = 0;
  u32 cache_epoch = 0;
  u32 pc = 0;
  u32 next_pc = 0;
  u32 last_pc = 0;
  u32 last_in_delay_slot = 0;
  u32 active_branch_pc = 0;
  u32 pending_delay_slot = 0;
  u32 pending_branch_taken = 0;
  u32 pending_branch_pc = 0;
  u32 scheduler_yield = 0;
  u32 pending_load_reg = 0;
  u32 pending_load_value = 0;
  u32 exception_raised = 0;
  u32 exception_return_sr = 0;
  u32 exception_return_bd = 0;
  u32 cycles = 0;
  u32 instructions = 0;
  u32 cycle_budget = 0;
  u32 instruction_budget = 0;
  u32 block_entries = 0;
  u32 direct_links = 0;
  u32 missing_exits = 0;
  u32 epoch_exits = 0;
  u32 memory_exits = 0;
  u32 generation_exits = 0;
  u32 budget_exits = 0;
  u32 bail_exits = 0;
  u32 icache_refills = 0;
  u32 revalidate_attempts = 0;
  u32 revalidate_successes = 0;
};

using V4NativeFn = void (*)(V4NativeState *);
using V4ResidentDispatchFn = void (*)(V4NativeState *);
using V4HelperFn = u32 (*)(Cpu *);

enum class V4HelperReason : u8 {
  Irq,
  UnalignedPc,
  UnsafeState,
  Opcode,
  CompileFailure,
  Budget,
};

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

enum class V4RejectKind : u8 {
  Other,
  Branch,
  Memory,
  Cop0,
  Cop2,
  Exception,
};

V4RejectKind classify_v4_reject(u32 bits) {
  const u32 primary = (bits >> 26) & 0x3Fu;
  if (primary == 0u) {
    switch (bits & 0x3Fu) {
    case 0x0C: // SYSCALL
    case 0x0D: // BREAK
    case 0x20: // ADD
    case 0x22: // SUB
      return V4RejectKind::Exception;
    default:
      return V4RejectKind::Other;
    }
  }
  if (primary == 0x01u || (primary >= 0x14u && primary <= 0x17u)) {
    return V4RejectKind::Branch;
  }
  if (primary == 0x10u) {
    return V4RejectKind::Cop0;
  }
  if (primary == 0x12u || primary == 0x32u || primary == 0x3Au) {
    return V4RejectKind::Cop2;
  }
  if ((primary >= 0x20u && primary <= 0x2Eu) ||
      (primary >= 0x30u && primary <= 0x3Bu)) {
    return V4RejectKind::Memory;
  }
  if (primary == 0x08u) { // ADDI can overflow.
    return V4RejectKind::Exception;
  }
  return V4RejectKind::Other;
}

struct V4Block {
  u32 start_pc = 0;
  u32 instruction_count = 0;
  u32 max_cycles = 0;
  u32 code_size = 0;
  u32 cache_epoch = 0;
  u32 icache_generation = 0;
  u32 second_icache_generation = 0;
  u32 code_page_generation = 0;
  u32 phys_page = 0;
  u16 icache_index = 0;
  u16 second_icache_index = 0;
  std::array<u32, kV4MaxBlockInstructions> guest_bits{};
  V4NativeFn fn = nullptr;
  // Native scheduler-tail prefix. Branch-only prefixes are restricted to a
  // fresh dispatcher entry so they cannot cross a System scheduling boundary
  // that an earlier native chain would otherwise have returned at.
  V4NativeFn budget_fn = nullptr;
  V4HelperFn helper_fn = nullptr;
  bool budget_requires_empty_chain = false;
  V4RejectKind reject_kind = V4RejectKind::Other;
  bool interpreter_only = false;
  bool has_control = false;
  bool has_memory = false;
  bool cacheable = false;
  bool second_icache_line = false;
  bool retry_second_line = false;
};

// Cached-code validation is deliberately performed inside the resident x64
// dispatcher. A generation mismatch is an architectural I-cache event, not a
// reason to bounce through C++ or an interpreter opcode path.
struct V4DispatchEntry {
  V4Block *block = nullptr;
};

static_assert(sizeof(V4DispatchEntry) == 8u);

struct V4DispatchPage {
  std::array<V4DispatchEntry, kV4DispatchEntriesPerPage> entries{};
};

V4HelperFn compile_v4_helper(V4CodeArena &arena, u32 instruction) {
  using namespace Xbyak;
  constexpr size_t kReservation = 64u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
#if defined(_WIN32)
  code.mov(code.edx, instruction);
  code.sub(code.rsp, 40);
#else
  code.mov(code.esi, instruction);
  code.sub(code.rsp, 8);
#endif
  code.mov(code.rax,
           reinterpret_cast<size_t>(Cpu::compiled_opcode_fn(instruction)));
  code.call(code.rax);
#if defined(_WIN32)
  code.add(code.rsp, 40);
#else
  code.add(code.rsp, 8);
#endif
  code.ret();
  code.ready();
  if (!arena.commit_emit(buffer, code.getSize())) {
    return nullptr;
  }
  return reinterpret_cast<V4HelperFn>(buffer);
}

// Dispatch pages survive arena resets. An edge embeds the address of a cell,
// never a V4Block or code pointer, so recompilation updates every incoming
// edge through the cell and the shared guard checks the new translation.
struct V4LinkTargets {
  V4DispatchEntry *fallthrough = nullptr;
  V4DispatchEntry *taken = nullptr;
  V4DispatchEntry *after_store = nullptr;
  void *entry = nullptr;
};

void emit_v4_block_return(Xbyak::CodeGenerator &code) {
  code.jmp(code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, block_return))]);
}

void emit_v4_selected_link(Xbyak::CodeGenerator &code,
                           const V4LinkTargets &links) {
  if (links.entry == nullptr) {
    emit_v4_block_return(code);
    return;
  }
  code.mov(code.r14, code.ptr[code.r14]);
  // The resident linked entry performs the same epoch, generation and budget
  // checks as ordinary dispatch, without resolving the guest PC again.
  code.mov(code.rax, reinterpret_cast<size_t>(links.entry));
  code.jmp(code.rax);
}

void emit_v4_link(Xbyak::CodeGenerator &code, V4DispatchEntry *cell,
                  const V4LinkTargets &links) {
  if (cell == nullptr) {
    emit_v4_block_return(code);
    return;
  }
  code.mov(code.r14, reinterpret_cast<size_t>(cell));
  emit_v4_selected_link(code, links);
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

void emit_v4_exception_no_delay(Xbyak::CodeGenerator &code, Exception cause,
                                u32 current_pc) {
  using namespace Xbyak;

  // R3000A exceptions commit the incoming load before entering the handler.
  emit_retire_incoming_load(code, 0u);

  // Preserve the pre-exception SR for diagnostics/exception-return tracing.
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))]);
  code.mov(code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, exception_return_sr))],
      code.eax);
  code.mov(code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, exception_return_bd))],
      0u);

  // Push IEc/KUc into the previous-mode stack and enter kernel/interrupt-off.
  code.mov(code.ecx, code.eax);
  code.and_(code.ecx, 0x3Fu);
  code.and_(code.eax, ~0x3Fu);
  code.shl(code.ecx, 2);
  code.and_(code.ecx, 0x3Fu);
  code.or_(code.eax, code.ecx);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))],
      code.eax);

  // CE is meaningless for these exceptions. Replace ExcCode and clear BD.
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_cause))]);
  code.and_(code.eax, ~((0x3u << 28) | 0x7Cu | (1u << 31)));
  code.or_(code.eax, static_cast<u32>(cause) << 2);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_cause))],
      code.eax);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_epc))],
      current_pc);

  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      current_pc);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, pending_branch_taken))],
      0u);
  code.mov(code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, pending_branch_pc))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, exception_raised))],
      1u);

  Label low_vector, vector_ready;
  code.test(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))],
      1u << 22);
  code.jz(low_vector);
  code.mov(code.eax, 0xBFC00180u);
  code.jmp(vector_ready);
  code.L(low_vector);
  code.mov(code.eax, 0x80000080u);
  code.L(vector_ready);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      code.eax);
  code.add(code.eax, 4u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, next_pc))],
      code.eax);

  // Cpu::instruction_cycles() charges two cycles for a faulting instruction.
  code.add(code.ebx, 2u);
  code.dec(code.r12d);
  emit_v4_block_return(code);
}


void emit_v4_exception_pending_delay(Xbyak::CodeGenerator &code,
                                     Exception cause) {
  using namespace Xbyak;

  // The faulting instruction is the already-pending branch delay slot.
  // Commit an older delayed load, then build EPC/BD directly from the resident
  // branch state without re-executing the opcode through Cpu::step().
  emit_retire_incoming_load(code, 0u);

  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))]);
  code.mov(code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, exception_return_sr))],
      code.eax);
  code.mov(code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, exception_return_bd))],
      1u);

  code.mov(code.ecx, code.eax);
  code.and_(code.ecx, 0x3Fu);
  code.and_(code.eax, ~0x3Fu);
  code.shl(code.ecx, 2);
  code.and_(code.ecx, 0x3Fu);
  code.or_(code.eax, code.ecx);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))],
      code.eax);

  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_cause))]);
  code.and_(code.eax, ~((0x3u << 28) | 0x7Cu));
  code.or_(code.eax, (static_cast<u32>(cause) << 2) | (1u << 31));
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_cause))],
      code.eax);

  // EPC identifies the branch; TAR/JumpDest identifies the instruction after
  // the faulting delay slot, matching Cpu::exception().
  code.mov(code.eax, code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, pending_branch_pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_epc))],
      code.eax);
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      code.eax);
  code.add(code.eax, 4u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_jumpdest))],
      code.eax);

  // Cpu::exception() clears active delay state before the handler begins.
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, pending_branch_taken))],
      0u);
  code.mov(code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, pending_branch_pc))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, exception_raised))],
      1u);

  Label low_vector, vector_ready;
  code.test(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))],
      1u << 22);
  code.jz(low_vector);
  code.mov(code.eax, 0xBFC00180u);
  code.jmp(vector_ready);
  code.L(low_vector);
  code.mov(code.eax, 0x80000080u);
  code.L(vector_ready);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      code.eax);
  code.add(code.eax, 4u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, next_pc))],
      code.eax);

  code.add(code.ebx, 2u);
  code.dec(code.r12d);
  emit_v4_block_return(code);
}

V4NativeFn compile_v4_exception(V4CodeArena &arena,
                                const V4DecodedException &inst,
                                u32 start_pc, u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 512u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  const Exception cause =
      inst.op == V4ExceptionOp::Syscall ? Exception::Syscall : Exception::Break;
  emit_v4_exception_no_delay(code, cause, start_pc);
  code.ready();
  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}

V4NativeFn compile_v4_overflow_alu(V4CodeArena &arena,
                                    const V4DecodedOverflowAlu &inst,
                                    u32 start_pc,
                                    const V4LinkTargets &links,
                                    u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 512u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  // The overflow slow path sits beyond the normal success epilogue and direct
  // link sequence, so force a near conditional branch instead of relying on
  // Xbyak's short forward-jump form.
  code.setDefaultJmpNEAR(true);
  Label overflow;

  emit_read_guest(code, code.eax, inst.rs);
  if (inst.op == V4OverflowAluOp::Add) {
    emit_read_guest(code, code.ecx, inst.rt);
    code.add(code.eax, code.ecx);
  } else if (inst.op == V4OverflowAluOp::Sub) {
    emit_read_guest(code, code.ecx, inst.rt);
    code.sub(code.eax, code.ecx);
  } else {
    code.add(code.eax, static_cast<u32>(inst.simm));
  }
  code.jo(overflow);

  const u8 dest =
      inst.op == V4OverflowAluOp::Addi ? inst.rt : inst.rd;
  // Commit the arithmetic result before retiring the previous load. The
  // retire helper intentionally clobbers EAX/ECX while preserving the write
  // via its cancel register semantics.
  emit_write_guest(code, dest, code.eax);
  emit_retire_incoming_load(code, dest);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      start_pc);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      0u);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))], 4u);
  code.inc(code.ebx);
  code.dec(code.r12d);
  emit_v4_link(code, links.fallthrough, links);

  code.L(overflow);
  emit_v4_exception_no_delay(code, Exception::Overflow, start_pc);

  code.ready();
  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}

V4NativeFn compile_v4_hilo(V4CodeArena &arena,
                               const V4DecodedHiLo &inst,
                               u32 start_pc,
                               const V4LinkTargets &links,
                               u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 768u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);

  // Keep HI/LO transfers as one-instruction blocks so EBX is the exact cycle
  // offset at which the operation issues. This matches the interpreter's
  // multiply/divide scoreboard without forcing a helper transition.
  Label ready;
  code.mov(code.rax, code.qword[
      code.r11 + static_cast<int>(offsetof(V4NativeState,
                                           muldiv_result_ready_cycle))]);
  code.mov(code.rcx, code.qword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cpu_cycle_base))]);
  code.add(code.rcx, code.rbx);
  code.inc(code.rcx);
  code.cmp(code.rax, code.rcx);
  code.jbe(ready);
  code.sub(code.rax, code.rcx);
  code.add(code.ebx, code.eax);
  code.L(ready);

  u8 cancel_reg = 0u;
  switch (inst.op) {
  case V4HiLoOp::Mfhi:
    code.mov(code.eax, code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, hi))]);
    emit_write_guest(code, inst.rd, code.eax);
    cancel_reg = inst.rd;
    break;
  case V4HiLoOp::Mflo:
    code.mov(code.eax, code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, lo))]);
    emit_write_guest(code, inst.rd, code.eax);
    cancel_reg = inst.rd;
    break;
  case V4HiLoOp::Mthi:
    emit_read_guest(code, code.eax, inst.rs);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, hi))], code.eax);
    code.mov(code.rax, code.qword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cpu_cycle_base))]);
    code.add(code.rax, code.rbx);
    code.inc(code.rax);
    code.mov(code.qword[
        code.r11 + static_cast<int>(offsetof(V4NativeState,
                                             muldiv_result_ready_cycle))],
        code.rax);
    break;
  case V4HiLoOp::Mtlo:
    emit_read_guest(code, code.eax, inst.rs);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, lo))], code.eax);
    code.mov(code.rax, code.qword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cpu_cycle_base))]);
    code.add(code.rax, code.rbx);
    code.inc(code.rax);
    code.mov(code.qword[
        code.r11 + static_cast<int>(offsetof(V4NativeState,
                                             muldiv_result_ready_cycle))],
        code.rax);
    break;
  }

  emit_retire_incoming_load(code, cancel_reg);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      start_pc);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      0u);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))], 4u);
  code.inc(code.ebx);
  code.dec(code.r12d);
  emit_v4_link(code, links.fallthrough, links);
  code.ready();

  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}


V4NativeFn compile_v4_muldiv(V4CodeArena &arena,
                             const V4DecodedMulDiv &inst,
                             u32 start_pc,
                             const V4LinkTargets &links,
                             u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 1024u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  code.setDefaultJmpNEAR(true);

  // MULT/DIV share the same asynchronous HI/LO scoreboard as MFHI/MFLO.
  // Keep the complete operation in generated x64 so a normal guest opcode
  // never needs Cpu::run_compiled_opcode() just to preserve timing.
  Label muldiv_ready;
  code.mov(code.rax, code.qword[
      code.r11 + static_cast<int>(offsetof(V4NativeState,
                                           muldiv_result_ready_cycle))]);
  code.mov(code.rcx, code.qword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cpu_cycle_base))]);
  code.add(code.rcx, code.rbx);
  code.inc(code.rcx);
  code.cmp(code.rax, code.rcx);
  code.jbe(muldiv_ready);
  code.sub(code.rax, code.rcx);
  code.add(code.ebx, code.eax);
  code.L(muldiv_ready);

  emit_read_guest(code, code.eax, inst.rs);
  emit_read_guest(code, code.ecx, inst.rt);

  Label result_ready, div_zero, div_overflow;
  u32 fixed_ticks = 0u;
  switch (inst.op) {
  case V4MulDivOp::Mult: {
    code.mov(code.r8d, code.eax);
    code.imul(code.ecx);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, lo))], code.eax);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, hi))], code.edx);

    Label negative, ticks6, ticks9, ticks13, ticks_done;
    code.test(code.r8d, code.r8d);
    code.js(negative);
    code.cmp(code.r8d, 0x800u);
    code.jb(ticks6);
    code.cmp(code.r8d, 0x100000u);
    code.jb(ticks9);
    code.jmp(ticks13);
    code.L(negative);
    code.cmp(code.r8d, static_cast<u32>(-2048));
    code.jae(ticks6);
    code.cmp(code.r8d, static_cast<u32>(-1048576));
    code.jae(ticks9);
    code.L(ticks13);
    code.mov(code.edx, 13u);
    code.jmp(ticks_done);
    code.L(ticks9);
    code.mov(code.edx, 9u);
    code.jmp(ticks_done);
    code.L(ticks6);
    code.mov(code.edx, 6u);
    code.L(ticks_done);
    break;
  }
  case V4MulDivOp::Multu: {
    code.mov(code.r8d, code.eax);
    code.mul(code.ecx);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, lo))], code.eax);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, hi))], code.edx);

    Label ticks6, ticks9, ticks_done;
    code.cmp(code.r8d, 0x800u);
    code.jb(ticks6);
    code.cmp(code.r8d, 0x100000u);
    code.jb(ticks9);
    code.mov(code.edx, 13u);
    code.jmp(ticks_done);
    code.L(ticks9);
    code.mov(code.edx, 9u);
    code.jmp(ticks_done);
    code.L(ticks6);
    code.mov(code.edx, 6u);
    code.L(ticks_done);
    break;
  }
  case V4MulDivOp::Div:
    fixed_ticks = 36u;
    code.test(code.ecx, code.ecx);
    code.jz(div_zero);
    code.cmp(code.eax, 0x80000000u);
    code.jne(div_overflow);
    code.cmp(code.ecx, 0xFFFFFFFFu);
    code.je(result_ready);
    code.L(div_overflow);
    code.cdq();
    code.idiv(code.ecx);
    code.jmp(result_ready);
    code.L(div_zero);
    {
      Label negative_dividend, div_zero_done;
      code.mov(code.edx, code.eax);
      code.test(code.eax, code.eax);
      code.js(negative_dividend);
      code.mov(code.eax, 0xFFFFFFFFu);
      code.jmp(div_zero_done);
      code.L(negative_dividend);
      code.mov(code.eax, 1u);
      code.L(div_zero_done);
    }
    code.jmp(result_ready);
    // Signed overflow: quotient = INT_MIN, remainder = 0.
    code.L(result_ready);
    // If divisor was -1 with INT_MIN dividend EAX already contains INT_MIN;
    // EDX is made zero below before committing the architectural result.
    break;
  case V4MulDivOp::Divu:
    fixed_ticks = 36u;
    code.test(code.ecx, code.ecx);
    code.jz(div_zero);
    code.xor_(code.edx, code.edx);
    code.div(code.ecx);
    code.jmp(result_ready);
    code.L(div_zero);
    code.mov(code.edx, code.eax);
    code.mov(code.eax, 0xFFFFFFFFu);
    code.L(result_ready);
    break;
  }

  if (inst.op == V4MulDivOp::Div) {
    // Repair the INT_MIN / -1 special case without relying on host #DE.
    Label not_overflow_commit;
    emit_read_guest(code, code.r8d, inst.rs);
    emit_read_guest(code, code.r9d, inst.rt);
    code.cmp(code.r8d, 0x80000000u);
    code.jne(not_overflow_commit);
    code.cmp(code.r9d, 0xFFFFFFFFu);
    code.jne(not_overflow_commit);
    code.mov(code.eax, 0x80000000u);
    code.xor_(code.edx, code.edx);
    code.L(not_overflow_commit);
  }

  if (inst.op == V4MulDivOp::Div || inst.op == V4MulDivOp::Divu) {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, lo))], code.eax);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, hi))], code.edx);
    code.mov(code.edx, fixed_ticks);
  }

  // ready_cycle = current issue cycle (including an older HI/LO stall) + latency.
  code.mov(code.rax, code.qword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cpu_cycle_base))]);
  code.add(code.rax, code.rbx);
  code.add(code.rax, code.rdx);
  code.mov(code.qword[
      code.r11 + static_cast<int>(offsetof(V4NativeState,
                                           muldiv_result_ready_cycle))],
      code.rax);

  emit_retire_incoming_load(code, 0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      start_pc);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      0u);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))], 4u);
  code.inc(code.ebx);
  code.dec(code.r12d);
  emit_v4_link(code, links.fallthrough, links);
  code.ready();

  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}


V4NativeFn compile_v4_cop0(V4CodeArena &arena,
                           const V4DecodedCop0 &inst,
                           u32 start_pc,
                           const V4LinkTargets &links,
                           u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 1024u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  code.setDefaultJmpNEAR(true);

  switch (inst.op) {
  case V4Cop0Op::Mfc0:
    switch (inst.rd) {
    case 6:
      code.mov(code.r8d, code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_jumpdest))]);
      break;
    case 8:
      code.mov(code.r8d, code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_badvaddr))]);
      break;
    case 12:
      code.mov(code.r8d, code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))]);
      break;
    case 13:
      code.mov(code.r8d, code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_cause))]);
      break;
    case 14:
      code.mov(code.r8d, code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_epc))]);
      break;
    case 15:
      code.mov(code.r8d, 0x00000002u);
      break;
    default:
      code.mov(code.rax, code.ptr[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_regs))]);
      code.mov(code.r8d, code.dword[code.rax + static_cast<int>(inst.rd) * 4]);
      break;
    }
    emit_retire_incoming_load(code, inst.rt);
    if (inst.rt != 0u) {
      code.mov(code.dword[
          code.r11 +
          static_cast<int>(offsetof(V4NativeState, pending_load_reg))],
          static_cast<u32>(inst.rt));
      code.mov(code.dword[
          code.r11 +
          static_cast<int>(offsetof(V4NativeState, pending_load_value))],
          code.r8d);
    }
    break;

  case V4Cop0Op::Mtc0:
    emit_read_guest(code, code.r8d, inst.rt);
    emit_retire_incoming_load(code, 0u);
    switch (inst.rd) {
    case 6:
    case 8:
    case 14:
      // JumpDest, BadVAddr and EPC are not guest-writable on this R3000A model.
      break;
    case 12: {
      constexpr u32 kSRWriteMask =
          0b1111'0010'0111'1111'1111'1111'0011'1111u;
      code.mov(code.eax, code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))]);
      code.and_(code.eax, ~kSRWriteMask);
      code.and_(code.r8d, kSRWriteMask);
      code.or_(code.eax, code.r8d);
      code.mov(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))],
          code.eax);
      break;
    }
    case 13:
      code.mov(code.eax, code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_cause))]);
      code.and_(code.eax, ~0x300u);
      code.and_(code.r8d, 0x300u);
      code.or_(code.eax, code.r8d);
      code.mov(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_cause))],
          code.eax);
      break;
    default:
      code.mov(code.rax, code.ptr[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_regs))]);
      code.mov(code.dword[code.rax + static_cast<int>(inst.rd) * 4], code.r8d);
      break;
    }
    // An SR/Cause write can make a pending IRQ immediately eligible.
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, scheduler_yield))],
        1u);
    break;

  case V4Cop0Op::Rfe:
    emit_retire_incoming_load(code, 0u);
    code.mov(code.eax, code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))]);
    code.mov(code.ecx, code.eax);
    code.and_(code.ecx, 0x3Fu);
    code.and_(code.eax, ~0x3Fu);
    code.shr(code.ecx, 2);
    code.or_(code.eax, code.ecx);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))],
        code.eax);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, scheduler_yield))],
        1u);
    break;
  }

  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      start_pc);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      0u);
  code.add(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))], 4u);
  // COP0 instructions use the core's 2-cycle issue cost.
  code.add(code.ebx, 2u);
  code.dec(code.r12d);
  emit_v4_link(code, links.fallthrough, links);
  code.ready();

  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
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
    u32 count, u32 start_pc, const V4LinkTargets &links, u32 &code_size) {
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
  code.add(code.ebx, count);
  code.sub(code.r12d, count);
  emit_v4_link(code, links.fallthrough, links);
  code.ready();

  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}

V4NativeFn compile_v4_pending_delay_alu(
    V4CodeArena &arena, const V4DecodedInstruction &inst,
    u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 512u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);

  // The branch decision and target were captured by the previous scheduler
  // slice. Execute only the already-pending delay-slot instruction here, then
  // resume at native.next_pc exactly like Cpu::step().
  emit_v4_alu_instruction(code, inst);
  emit_retire_incoming_load(code, v4_alu_write_reg(inst));

  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      code.eax);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      1u);
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_branch_pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      code.eax);
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, next_pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      code.eax);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_branch_taken))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_branch_pc))],
      0u);
  code.inc(code.ebx);
  code.dec(code.r12d);
  // Preserve the architectural interrupt sampling point immediately after a
  // delay slot. A later execution-flow phase can move that check into the
  // resident dispatcher; until then, yield without using the opcode helper.
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, scheduler_yield))],
      1u);
  emit_v4_block_return(code);
  code.ready();

  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}

V4NativeFn compile_v4_pending_delay_overflow_alu(
    V4CodeArena &arena, const V4DecodedOverflowAlu &inst,
    u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 768u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  // The overflow bailout is past the normal success epilogue, so force near
  // conditional branches rather than relying on Xbyak's short forward form.
  code.setDefaultJmpNEAR(true);
  Label overflow;

  // Evaluate the pending delay-slot arithmetic without committing any guest
  // state. If it overflows, the ordinary helper re-executes this instruction
  // with the branch-delay state still intact, preserving EPC/BD semantics.
  emit_read_guest(code, code.eax, inst.rs);
  if (inst.op == V4OverflowAluOp::Add) {
    emit_read_guest(code, code.ecx, inst.rt);
    code.add(code.eax, code.ecx);
  } else if (inst.op == V4OverflowAluOp::Sub) {
    emit_read_guest(code, code.ecx, inst.rt);
    code.sub(code.eax, code.ecx);
  } else {
    code.add(code.eax, static_cast<u32>(inst.simm));
  }
  code.jo(overflow);

  const u8 dest =
      inst.op == V4OverflowAluOp::Addi ? inst.rt : inst.rd;
  emit_write_guest(code, dest, code.eax);
  emit_retire_incoming_load(code, dest);

  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      code.eax);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      1u);
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_branch_pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      code.eax);
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, next_pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      code.eax);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_branch_taken))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_branch_pc))],
      0u);
  code.inc(code.ebx);
  code.dec(code.r12d);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, scheduler_yield))],
      1u);
  emit_v4_block_return(code);

  code.L(overflow);
  emit_v4_exception_pending_delay(code, Exception::Overflow);
  code.ready();

  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}

V4NativeFn compile_v4_pending_delay_store(
    V4CodeArena &arena, const V4DecodedStore &store, u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 1536u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  code.setDefaultJmpNEAR(true);
  Label ram, stored, bail;

  // Pending delay-slot stores may stay native only for ordinary RAM and
  // scratchpad accesses. MMIO, cache isolation, misalignment and writes into
  // translated code fall back before any architectural side effect so the
  // regular Cpu::step() path can preserve precise exception/BD semantics.
  code.cmp(code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, memory_fastpath_allowed))],
      0u);
  code.je(bail);
  code.test(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))],
      1u << 16);
  code.jnz(bail);

  // Capture both store operands before retiring any incoming delayed load.
  emit_read_guest(code, code.eax, store.rs);
  code.add(code.eax, static_cast<u32>(store.simm));
  emit_read_guest(code, code.r8d, store.rt);
  if (store.op == V4StoreOp::Sh) {
    code.test(code.eax, 1u);
    code.jnz(bail);
  } else if (store.op == V4StoreOp::Sw) {
    code.test(code.eax, 3u);
    code.jnz(bail);
  }

  code.mov(code.edx, code.eax);
  code.and_(code.edx, 0x1FFFFFFFu);
  if (store.op == V4StoreOp::Swl || store.op == V4StoreOp::Swr) {
    code.mov(code.ecx, code.edx);
    code.and_(code.ecx, 3u);
    code.mov(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, store_byte_offset))],
        code.ecx);
    code.and_(code.edx, ~3u);
  }
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_phys))],
      code.edx);

  // Do not fast-store into a translated 16-byte code line. Normalize RAM
  // mirrors to the same 2 MiB backing address before consulting the bitmap.
  code.mov(code.r9d, code.edx);
  {
    Label line_key_ready;
    code.cmp(code.r9d, code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, mapped_main_ram_size))]);
    code.jae(line_key_ready);
    code.and_(code.r9d, psx::RAM_SIZE - 1u);
    code.L(line_key_ready);
  }
  code.shr(code.r9d, 4u);
  code.mov(code.ecx, code.r9d);
  code.shr(code.r9d, 6u);
  code.and_(code.ecx, 63u);
  code.mov(code.rax, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, code_line_bits))]);
  code.test(code.rax, code.rax);
  code.jz(bail);
  code.mov(code.rax, code.qword[code.rax + code.r9 * 8]);
  code.shr(code.rax, code.cl);
  code.test(code.al, 1u);
  code.jnz(bail);

  code.cmp(code.edx, code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, mapped_main_ram_size))]);
  code.jb(ram);
  code.cmp(code.edx, 0x1F800000u);
  code.jb(bail);
  code.cmp(code.edx, 0x1F801000u);
  code.jae(bail);

  auto emit_unaligned_store = [&]() {
    Label off0, off1, off2, merged;
    code.test(code.r9d, code.r9d);
    code.jz(off0);
    code.cmp(code.r9d, 1u);
    code.je(off1);
    code.cmp(code.r9d, 2u);
    code.je(off2);

    if (store.op == V4StoreOp::Swl) {
      code.mov(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off0);
      code.and_(code.eax, 0xFFFFFF00u);
      code.shr(code.r8d, 24);
      code.or_(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off1);
      code.and_(code.eax, 0xFFFF0000u);
      code.shr(code.r8d, 16);
      code.or_(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off2);
      code.and_(code.eax, 0xFF000000u);
      code.shr(code.r8d, 8);
      code.or_(code.eax, code.r8d);
    } else {
      code.and_(code.eax, 0x00FFFFFFu);
      code.shl(code.r8d, 24);
      code.or_(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off0);
      code.mov(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off1);
      code.and_(code.eax, 0x000000FFu);
      code.shl(code.r8d, 8);
      code.or_(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off2);
      code.and_(code.eax, 0x0000FFFFu);
      code.shl(code.r8d, 16);
      code.or_(code.eax, code.r8d);
    }
    code.L(merged);
  };

  code.sub(code.edx, 0x1F800000u);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, scratchpad))]);
  code.test(code.rcx, code.rcx);
  code.jz(bail);
  if (store.op == V4StoreOp::Swl || store.op == V4StoreOp::Swr) {
    code.mov(code.eax, code.dword[code.rcx + code.rdx]);
    code.mov(code.r9d, code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, store_byte_offset))]);
    emit_unaligned_store();
    code.mov(code.dword[code.rcx + code.rdx], code.eax);
    code.mov(code.r9d, 5u);
  } else {
    switch (store.op) {
    case V4StoreOp::Sb:
      code.mov(code.byte[code.rcx + code.rdx], code.r8b);
      break;
    case V4StoreOp::Sh:
      code.mov(code.word[code.rcx + code.rdx], code.r8w);
      break;
    case V4StoreOp::Sw:
      code.mov(code.dword[code.rcx + code.rdx], code.r8d);
      break;
    default:
      break;
    }
  }
  code.xor_(code.r9d, code.r9d);
  code.jmp(stored);

  code.L(ram);
  code.and_(code.edx, psx::RAM_SIZE - 1u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_phys))],
      code.edx);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, main_ram))]);
  code.test(code.rcx, code.rcx);
  code.jz(bail);
  if (store.op == V4StoreOp::Swl || store.op == V4StoreOp::Swr) {
    code.mov(code.eax, code.dword[code.rcx + code.rdx]);
    code.mov(code.r9d, code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, store_byte_offset))]);
    emit_unaligned_store();
    code.mov(code.dword[code.rcx + code.rdx], code.eax);
  } else {
    switch (store.op) {
    case V4StoreOp::Sb:
      code.mov(code.byte[code.rcx + code.rdx], code.r8b);
      break;
    case V4StoreOp::Sh:
      code.mov(code.word[code.rcx + code.rdx], code.r8w);
      break;
    case V4StoreOp::Sw:
      code.mov(code.dword[code.rcx + code.rdx], code.r8d);
      break;
    default:
      break;
    }
    code.mov(code.r9d, 1u);
  }

  code.L(stored);
  emit_retire_incoming_load(code, 0u);
  code.add(code.ebx, 2u);
  code.add(code.ebx, code.r9d);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_entries))]);

  // Match native store invalidation of the direct-mapped guest I-cache slot.
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_phys))]);
  code.shr(code.eax, 4u);
  code.and_(code.eax, 0xFFu);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, icache_valid))]);
  code.mov(code.edx, code.eax);
  code.imul(code.edx, code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, icache_line_stride))]);
  code.mov(code.byte[code.rcx + code.rdx], 0u);
  code.mov(code.rcx, code.ptr[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, icache_generations))]);
  code.inc(code.dword[code.rcx + code.rax * 4]);
  {
    Label generation_ok;
    code.cmp(code.dword[code.rcx + code.rax * 4], 0u);
    code.jne(generation_ok);
    code.mov(code.dword[code.rcx + code.rax * 4], 1u);
    code.L(generation_ok);
  }

  // Resolve the already-captured branch destination after its delay slot.
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      code.eax);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      1u);
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_branch_pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      code.eax);
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, next_pc))]);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      code.eax);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_branch_taken))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_branch_pc))],
      0u);
  code.dec(code.r12d);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, scheduler_yield))],
      1u);
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

V4NativeFn compile_v4_budget_branch(
    V4CodeArena &arena, const V4DecodedControl &control, u32 branch_pc,
    u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 1024u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  Label not_taken, selected;

  const u32 fallthrough = branch_pc + 8u;
  const u32 jump_target =
      ((branch_pc + 4u) & 0xF0000000u) | (control.imm26 << 2u);
  const u32 branch_target =
      branch_pc + 4u + static_cast<u32>(control.simm * 4);

  // Capture operands before the incoming load retires, exactly like the
  // interpreter. Leave the delay slot pending so System keeps the same
  // device-service boundary it had when this branch used the opcode helper.
  if (control.op == V4ControlOp::Beq || control.op == V4ControlOp::Bne) {
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
    if (control.op == V4ControlOp::Bltzal ||
        control.op == V4ControlOp::Bgezal) {
      code.mov(code.eax, branch_pc + 8u);
      emit_write_guest(code, 31u, code.eax);
    }
  } else if (control.op == V4ControlOp::Jr ||
             control.op == V4ControlOp::Jalr) {
    emit_read_guest(code, code.r8d, control.rs);
    if (control.op == V4ControlOp::Jalr) {
      code.mov(code.eax, branch_pc + 8u);
      emit_write_guest(code, control.rd, code.eax);
    }
  } else if (control.op == V4ControlOp::Jal) {
    code.mov(code.eax, branch_pc + 8u);
    emit_write_guest(code, 31u, code.eax);
  }

  emit_retire_incoming_load(code, v4_control_write_reg(control));

  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      branch_pc);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      branch_pc + 4u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_delay_slot))],
      1u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_branch_pc))],
      branch_pc);
  code.dec(code.r12d);

  if (control.op == V4ControlOp::J || control.op == V4ControlOp::Jal) {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, next_pc))],
        jump_target);
    code.mov(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, pending_branch_taken))],
        1u);
    code.add(code.ebx, 2u);
  } else if (control.op == V4ControlOp::Jr ||
             control.op == V4ControlOp::Jalr) {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, next_pc))],
        code.r8d);
    code.mov(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, pending_branch_taken))],
        1u);
    code.add(code.ebx, 2u);
  } else {
    code.test(code.edx, code.edx);
    code.jz(not_taken);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, next_pc))],
        branch_target);
    code.mov(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, pending_branch_taken))],
        1u);
    code.add(code.ebx, 2u);
    code.jmp(selected);

    code.L(not_taken);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, next_pc))],
        fallthrough);
    code.mov(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, pending_branch_taken))],
        0u);
    code.inc(code.ebx);
    code.L(selected);
  }

  // Yield after the branch only. The pending delay slot remains at the same
  // outer scheduler boundary as the previous helper implementation.
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, scheduler_yield))],
      1u);
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
    const V4DecodedInstruction &delay, u32 branch_pc,
    const V4LinkTargets &links, u32 &code_size) {
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
  code.sub(code.r12d, prefix_count + 2u);

  if (control.op == V4ControlOp::J ||
      control.op == V4ControlOp::Jal) {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        jump_target);
    code.add(code.ebx, prefix_count + 3u);
    code.mov(code.r14, reinterpret_cast<size_t>(links.taken));
  } else if (control.op == V4ControlOp::Jr ||
             control.op == V4ControlOp::Jalr) {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        code.r8d);
    code.add(code.ebx, prefix_count + 3u);
  } else {
    Label not_taken, selected;
    code.test(code.edx, code.edx);
    code.jz(not_taken);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        branch_target);
    code.add(code.ebx, prefix_count + 3u);
    code.mov(code.r14, reinterpret_cast<size_t>(links.taken));
    code.jmp(selected);

    code.L(not_taken);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        fallthrough);
    code.add(code.ebx, prefix_count + 2u);
    code.mov(code.r14, reinterpret_cast<size_t>(links.fallthrough));
    code.L(selected);
  }

  if (control.op == V4ControlOp::Jr || control.op == V4ControlOp::Jalr) {
    emit_v4_block_return(code);
  } else {
    emit_v4_selected_link(code, links);
  }
  code.ready();

  code_size = static_cast<u32>(code.getSize());
  if (!arena.commit_emit(buffer, code_size)) {
    return nullptr;
  }
  return reinterpret_cast<V4NativeFn>(buffer);
}


bool v4_nonlink_conditional(const V4DecodedControl &control) {
  switch (control.op) {
  case V4ControlOp::Beq:
  case V4ControlOp::Bne:
  case V4ControlOp::Blez:
  case V4ControlOp::Bgtz:
  case V4ControlOp::Bltz:
  case V4ControlOp::Bgez:
    return true;
  default:
    return false;
  }
}

V4NativeFn compile_v4_guarded_delay_branch(
    V4CodeArena &arena, const V4DecodedControl &control,
    const V4DecodedOverflowAlu &delay, u32 branch_pc,
    const V4LinkTargets &links, u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 1024u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  code.setDefaultJmpNEAR(true);
  Label bail, not_taken, selected;

  // A branch retires the incoming delayed load before its delay-slot
  // instruction observes registers. Rather than materializing that state on a
  // guarded bailout, keep this first implementation conservative and let the
  // precise helper path handle branch entries with a pending load.
  code.cmp(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_load_reg))],
      0u);
  code.jne(bail);

  // Capture the branch condition before executing the delay slot.
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
  } else {
    emit_read_guest(code, code.eax, control.rs);
    code.cmp(code.eax, 0);
    switch (control.op) {
    case V4ControlOp::Blez: code.setle(code.dl); break;
    case V4ControlOp::Bgtz: code.setg(code.dl); break;
    case V4ControlOp::Bltz: code.setl(code.dl); break;
    case V4ControlOp::Bgez: code.setge(code.dl); break;
    default: code.jmp(bail); break;
    }
    code.movzx(code.edx, code.dl);
  }

  // Guard the signed delay-slot arithmetic before committing any architectural
  // state. On actual overflow, the block exits untouched and the existing
  // helper executes the branch + delay slot with precise EPC/BD semantics.
  emit_read_guest(code, code.eax, delay.rs);
  if (delay.op == V4OverflowAluOp::Add) {
    emit_read_guest(code, code.ecx, delay.rt);
    code.add(code.eax, code.ecx);
  } else if (delay.op == V4OverflowAluOp::Sub) {
    emit_read_guest(code, code.ecx, delay.rt);
    code.sub(code.eax, code.ecx);
  } else {
    code.add(code.eax, static_cast<u32>(delay.simm));
  }
  code.jo(bail);

  const u8 delay_dest =
      delay.op == V4OverflowAluOp::Addi ? delay.rt : delay.rd;
  emit_write_guest(code, delay_dest, code.eax);

  const u32 fallthrough = branch_pc + 8u;
  const u32 branch_target =
      branch_pc + 4u + static_cast<u32>(control.simm * 4);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      branch_pc + 4u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      1u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      branch_pc);
  code.sub(code.r12d, 2u);

  code.test(code.edx, code.edx);
  code.jz(not_taken);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      branch_target);
  code.add(code.ebx, 3u);
  code.mov(code.r14, reinterpret_cast<size_t>(links.taken));
  code.jmp(selected);

  code.L(not_taken);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))], fallthrough);
  code.add(code.ebx, 2u);
  code.mov(code.r14, reinterpret_cast<size_t>(links.fallthrough));
  code.L(selected);
  emit_v4_selected_link(code, links);

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

V4NativeFn compile_v4_store_delay_branch(
    V4CodeArena &arena, const V4DecodedControl &control,
    const V4DecodedStore &store, u32 branch_pc,
    const V4LinkTargets &links, u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 2048u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  code.setDefaultJmpNEAR(true);
  Label ram, stored, bail, not_taken, selected;

  // Keep the guarded form side-effect-free until all delay-slot store checks
  // have passed. Incoming loads would retire between the branch and its delay
  // slot, so leave those entries on the precise helper path for now.
  code.cmp(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pending_load_reg))],
      0u);
  code.jne(bail);
  code.test(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))],
      1u << 16);
  code.jnz(bail);

  emit_read_guest(code, code.eax, store.rs);
  code.add(code.eax, static_cast<u32>(store.simm));
  emit_read_guest(code, code.r8d, store.rt);
  if (store.op == V4StoreOp::Sh) {
    code.test(code.eax, 1u);
    code.jnz(bail);
  } else if (store.op == V4StoreOp::Sw) {
    code.test(code.eax, 3u);
    code.jnz(bail);
  }

  code.mov(code.edx, code.eax);
  code.and_(code.edx, 0x1FFFFFFFu);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_phys))],
      code.edx);

  // Never fast-store into a line containing translated code. The helper path
  // owns self-modifying-code invalidation and exception/MMIO semantics.
  code.mov(code.r9d, code.edx);
  {
    Label line_key_ready;
    code.cmp(code.r9d, code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, mapped_main_ram_size))]);
    code.jae(line_key_ready);
    code.and_(code.r9d, psx::RAM_SIZE - 1u);
    code.L(line_key_ready);
  }
  code.shr(code.r9d, 4u);
  code.mov(code.ecx, code.r9d);
  code.shr(code.r9d, 6u);
  code.and_(code.ecx, 63u);
  code.mov(code.rax, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, code_line_bits))]);
  code.test(code.rax, code.rax);
  code.jz(bail);
  code.mov(code.rax, code.qword[code.rax + code.r9 * 8]);
  code.shr(code.rax, code.cl);
  code.test(code.al, 1u);
  code.jnz(bail);

  code.cmp(code.edx, code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, mapped_main_ram_size))]);
  code.jb(ram);
  code.cmp(code.edx, 0x1F800000u);
  code.jb(bail);
  code.cmp(code.edx, 0x1F801000u);
  code.jae(bail);

  // Scratchpad store. r9d becomes the RAM penalty (zero here).
  code.sub(code.edx, 0x1F800000u);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, scratchpad))]);
  code.test(code.rcx, code.rcx);
  code.jz(bail);
  switch (store.op) {
  case V4StoreOp::Sb: code.mov(code.byte[code.rcx + code.rdx], code.r8b); break;
  case V4StoreOp::Sh: code.mov(code.word[code.rcx + code.rdx], code.r8w); break;
  case V4StoreOp::Sw: code.mov(code.dword[code.rcx + code.rdx], code.r8d); break;
  }
  code.xor_(code.r9d, code.r9d);
  code.jmp(stored);

  code.L(ram);
  code.and_(code.edx, psx::RAM_SIZE - 1u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_phys))],
      code.edx);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, main_ram))]);
  code.test(code.rcx, code.rcx);
  code.jz(bail);
  switch (store.op) {
  case V4StoreOp::Sb: code.mov(code.byte[code.rcx + code.rdx], code.r8b); break;
  case V4StoreOp::Sh: code.mov(code.word[code.rcx + code.rdx], code.r8w); break;
  case V4StoreOp::Sw: code.mov(code.dword[code.rcx + code.rdx], code.r8d); break;
  }
  code.mov(code.r9d, 1u);

  code.L(stored);
  // The store has committed. From this point onward there are no guarded
  // exits. Account its 2-cycle baseline plus the 1-cycle main-RAM penalty.
  code.add(code.ebx, 2u);
  code.add(code.ebx, code.r9d);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_entries))]);

  // Match Cpu::notify_code_write(): invalidate the direct-mapped guest I-cache
  // slot and advance its generation after the successful store.
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_phys))]);
  code.shr(code.eax, 4u);
  code.and_(code.eax, 0xFFu);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, icache_valid))]);
  code.mov(code.edx, code.eax);
  code.imul(code.edx, code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, icache_line_stride))]);
  code.mov(code.byte[code.rcx + code.rdx], 0u);
  code.mov(code.rcx, code.ptr[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, icache_generations))]);
  code.inc(code.dword[code.rcx + code.rax * 4]);
  {
    Label generation_ok;
    code.cmp(code.dword[code.rcx + code.rax * 4], 0u);
    code.jne(generation_ok);
    code.mov(code.dword[code.rcx + code.rax * 4], 1u);
    code.L(generation_ok);
  }

  // The delay-slot store cannot alter GPRs, so evaluating the branch condition
  // after the store is equivalent to capturing it before the delay slot.
  if (control.op == V4ControlOp::Beq || control.op == V4ControlOp::Bne) {
    emit_read_guest(code, code.eax, control.rs);
    emit_read_guest(code, code.ecx, control.rt);
    code.cmp(code.eax, code.ecx);
    if (control.op == V4ControlOp::Beq) {
      code.sete(code.dl);
    } else {
      code.setne(code.dl);
    }
    code.movzx(code.edx, code.dl);
  } else {
    emit_read_guest(code, code.eax, control.rs);
    code.cmp(code.eax, 0);
    switch (control.op) {
    case V4ControlOp::Blez: code.setle(code.dl); break;
    case V4ControlOp::Bgtz: code.setg(code.dl); break;
    case V4ControlOp::Bltz: code.setl(code.dl); break;
    case V4ControlOp::Bgez: code.setge(code.dl); break;
    default: break;
    }
    code.movzx(code.edx, code.dl);
  }

  const u32 fallthrough = branch_pc + 8u;
  const u32 branch_target =
      branch_pc + 4u + static_cast<u32>(control.simm * 4);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      branch_pc + 4u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      1u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      branch_pc);
  code.sub(code.r12d, 2u);

  code.test(code.edx, code.edx);
  code.jz(not_taken);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      branch_target);
  code.add(code.ebx, 2u);
  code.mov(code.r14, reinterpret_cast<size_t>(links.taken));
  code.jmp(selected);
  code.L(not_taken);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))], fallthrough);
  code.inc(code.ebx);
  code.mov(code.r14, reinterpret_cast<size_t>(links.fallthrough));
  code.L(selected);
  emit_v4_selected_link(code, links);

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

V4NativeFn compile_v4_load(
    V4CodeArena &arena,
    const std::array<V4DecodedInstruction, kV4MaxBlockInstructions> &prefix,
    u32 prefix_count, const V4DecodedLoad &load,
    const std::array<V4DecodedInstruction, kV4MaxBlockInstructions> &tail,
    u32 tail_count, const V4DecodedControl *control,
    const V4DecodedInstruction *delay, u32 branch_pc, u32 start_pc,
    const V4LinkTargets &links, u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 2048u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  code.setDefaultJmpNEAR(true);
  Label ram, scratch, hot_mmio16, loaded, unaligned, slow_after_prefix, bail;
  Label &slow_exit = prefix_count != 0u ? slow_after_prefix : bail;

  for (u32 i = 0; i < prefix_count; ++i) {
    emit_v4_alu_instruction(code, prefix[i]);
    if (i == 0u) {
      emit_retire_incoming_load(code, v4_alu_write_reg(prefix[i]));
    }
  }

  // Without a prefix the load itself is the load-delay instruction and must
  // capture its source before the incoming delayed load retires.
  emit_read_guest(code, code.eax, load.rs);
  code.add(code.eax, static_cast<u32>(load.simm));
  if (load.op == V4LoadOp::Lh || load.op == V4LoadOp::Lhu) {
    code.test(code.eax, 1u);
    code.jnz(unaligned);
  } else if (load.op == V4LoadOp::Lw) {
    code.test(code.eax, 3u);
    code.jnz(unaligned);
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
    case V4LoadOp::Lwl:
    case V4LoadOp::Lw:
    case V4LoadOp::Lwr:
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
  if (load.op == V4LoadOp::Lwl || load.op == V4LoadOp::Lwr) {
    code.and_(code.edx, ~3u);
  }
  code.cmp(code.edx, code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, mapped_main_ram_size))]);
  code.jb(ram);
  code.cmp(code.edx, 0x1F800000u);
  code.jb(slow_exit);
  code.cmp(code.edx, 0x1F801000u);
  code.jb(scratch);
  if (load.op == V4LoadOp::Lh || load.op == V4LoadOp::Lhu) {
    code.cmp(code.edx, 0x1F801070u);
    code.jb(slow_exit);
    code.cmp(code.edx, 0x1F801078u);
    code.jb(hot_mmio16);
    code.cmp(code.edx, 0x1F801100u);
    code.jb(slow_exit);
    code.cmp(code.edx, 0x1F801130u);
    code.jb(hot_mmio16);
  }
  code.jmp(slow_exit);
  code.L(scratch);
  code.sub(code.edx, 0x1F800000u);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, scratchpad))]);
  code.test(code.rcx, code.rcx);
  code.jz(slow_exit);
  emit_memory_read();
  code.xor_(code.r9d, code.r9d);
  code.jmp(loaded);

  code.L(ram);
  code.and_(code.edx, psx::RAM_SIZE - 1u);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, main_ram))]);
  code.test(code.rcx, code.rcx);
  code.jz(slow_exit);
  emit_memory_read();
  code.mov(code.r9d, 4u);
  code.jmp(loaded);

  code.L(hot_mmio16);
  // Call only the narrow timer/IRQ bridge. Keep the resident state and GPR
  // pointers intact across the host ABI call so execution can remain native.
  code.push(code.r10);
  code.push(code.r11);
#if defined(_WIN32)
  code.sub(code.rsp, 32);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, system))]);
  // EDX already carries the physical address.
#else
  code.mov(code.rdi, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, system))]);
  code.mov(code.esi, code.edx);
#endif
  code.mov(code.rax, reinterpret_cast<size_t>(&v4_hot_mmio_read16));
  code.call(code.rax);
#if defined(_WIN32)
  code.add(code.rsp, 32);
#endif
  code.pop(code.r11);
  code.pop(code.r10);
  code.cmp(code.eax, 0x10000u);
  code.jae(slow_exit);
  code.mov(code.r8d, code.eax);
  if (load.op == V4LoadOp::Lh) {
    code.shl(code.r8d, 16);
    code.sar(code.r8d, 16);
  }
  code.xor_(code.r9d, code.r9d);

  code.L(loaded);
  if (load.op == V4LoadOp::Lwl || load.op == V4LoadOp::Lwr) {
    // LWL/LWR merge against the architecturally visible target value. If the
    // previous instruction scheduled a load to the same register, that pending
    // value is the merge source even though it has not retired yet.
    emit_read_guest(code, code.ecx, load.rt);
    {
      Label use_gpr_value;
      code.cmp(code.dword[
          code.r11 +
          static_cast<int>(offsetof(V4NativeState, pending_load_reg))],
          static_cast<u32>(load.rt));
      code.jne(use_gpr_value);
      code.mov(code.ecx, code.dword[
          code.r11 +
          static_cast<int>(offsetof(V4NativeState, pending_load_value))]);
      code.L(use_gpr_value);
    }

    Label merge0, merge1, merge2, merge_done;
    code.mov(code.edx, code.eax);
    code.and_(code.edx, 3u);
    code.test(code.edx, code.edx);
    code.jz(merge0);
    code.cmp(code.edx, 1u);
    code.je(merge1);
    code.cmp(code.edx, 2u);
    code.je(merge2);

    if (load.op == V4LoadOp::Lwl) {
      // offset 3: entire aligned word becomes visible.
      code.jmp(merge_done);
      code.L(merge0);
      code.and_(code.ecx, 0x00FFFFFFu);
      code.shl(code.r8d, 24);
      code.or_(code.r8d, code.ecx);
      code.jmp(merge_done);
      code.L(merge1);
      code.and_(code.ecx, 0x0000FFFFu);
      code.shl(code.r8d, 16);
      code.or_(code.r8d, code.ecx);
      code.jmp(merge_done);
      code.L(merge2);
      code.and_(code.ecx, 0x000000FFu);
      code.shl(code.r8d, 8);
      code.or_(code.r8d, code.ecx);
    } else {
      // LWR offset 3 preserves the high 24 bits and imports val >> 24.
      code.shr(code.r8d, 24);
      code.and_(code.ecx, 0xFFFFFF00u);
      code.or_(code.r8d, code.ecx);
      code.jmp(merge_done);
      code.L(merge0);
      // offset 0: entire aligned word becomes visible.
      code.jmp(merge_done);
      code.L(merge1);
      code.shr(code.r8d, 8);
      code.and_(code.ecx, 0xFF000000u);
      code.or_(code.r8d, code.ecx);
      code.jmp(merge_done);
      code.L(merge2);
      code.shr(code.r8d, 16);
      code.and_(code.ecx, 0xFFFF0000u);
      code.or_(code.r8d, code.ecx);
    }
    code.L(merge_done);
  }
  // A prefix already retired the incoming load. Otherwise the load retires or
  // cancels it now, after address operands were captured.
  if (prefix_count == 0u) {
    emit_retire_incoming_load(code, load.rt);
  }
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

  if (control != nullptr && delay != nullptr) {
    const V4DecodedControl &branch = *control;
    const u32 fallthrough = branch_pc + 8u;
    const u32 jump_target =
        ((branch_pc + 4u) & 0xF0000000u) | (branch.imm26 << 2u);
    const u32 branch_target =
        branch_pc + 4u + static_cast<u32>(branch.simm * 4);

    // The load and any ALU tail have consumed their cycles. Keep the RAM
    // penalty in r9d until the branch direction selects its exact cost.
    code.add(code.r9d, prefix_count + tail_count + 2u);

    // Capture the branch operands before retiring the new load when the branch
    // is the immediate load-delay instruction.
    if (branch.op == V4ControlOp::Beq ||
        branch.op == V4ControlOp::Bne) {
      emit_read_guest(code, code.eax, branch.rs);
      emit_read_guest(code, code.ecx, branch.rt);
      code.cmp(code.eax, code.ecx);
      if (branch.op == V4ControlOp::Beq) {
        code.sete(code.dl);
      } else {
        code.setne(code.dl);
      }
      code.movzx(code.edx, code.dl);
    } else if (branch.op == V4ControlOp::Blez ||
               branch.op == V4ControlOp::Bgtz ||
               branch.op == V4ControlOp::Bltz ||
               branch.op == V4ControlOp::Bgez ||
               branch.op == V4ControlOp::Bltzal ||
               branch.op == V4ControlOp::Bgezal) {
      emit_read_guest(code, code.eax, branch.rs);
      code.cmp(code.eax, 0);
      switch (branch.op) {
      case V4ControlOp::Blez: code.setle(code.dl); break;
      case V4ControlOp::Bgtz: code.setg(code.dl); break;
      case V4ControlOp::Bltz:
      case V4ControlOp::Bltzal: code.setl(code.dl); break;
      case V4ControlOp::Bgez:
      case V4ControlOp::Bgezal: code.setge(code.dl); break;
      default: break;
      }
      code.movzx(code.edx, code.dl);
      if (branch.op == V4ControlOp::Bltzal ||
          branch.op == V4ControlOp::Bgezal) {
        code.mov(code.eax, branch_pc + 8u);
        emit_write_guest(code, 31u, code.eax);
      }
    } else if (branch.op == V4ControlOp::Jr ||
               branch.op == V4ControlOp::Jalr) {
      emit_read_guest(code, code.r8d, branch.rs);
      if (branch.op == V4ControlOp::Jalr) {
        code.mov(code.eax, branch_pc + 8u);
        emit_write_guest(code, branch.rd, code.eax);
      }
    } else if (branch.op == V4ControlOp::Jal) {
      code.mov(code.eax, branch_pc + 8u);
      emit_write_guest(code, 31u, code.eax);
    }

    if (tail_count == 0u) {
      emit_retire_incoming_load(code, v4_control_write_reg(branch));
    }

    emit_v4_alu_instruction(code, *delay);

    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
        branch_pc + 4u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
        1u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
        branch_pc);
    code.sub(code.r12d, prefix_count + tail_count + 3u);

    if (branch.op == V4ControlOp::J ||
        branch.op == V4ControlOp::Jal) {
      code.mov(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
          jump_target);
      code.add(code.r9d, 3u);
      code.mov(code.r14, reinterpret_cast<size_t>(links.taken));
    } else if (branch.op == V4ControlOp::Jr ||
               branch.op == V4ControlOp::Jalr) {
      code.mov(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
          code.r8d);
      code.add(code.r9d, 3u);
    } else {
      Label not_taken, selected;
      code.test(code.edx, code.edx);
      code.jz(not_taken);
      code.mov(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
          branch_target);
      code.add(code.r9d, 3u);
      code.mov(code.r14, reinterpret_cast<size_t>(links.taken));
      code.jmp(selected);

      code.L(not_taken);
      code.mov(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
          fallthrough);
      code.add(code.r9d, 2u);
      code.mov(code.r14, reinterpret_cast<size_t>(links.fallthrough));
      code.L(selected);
    }

    code.add(code.ebx, code.r9d);
  } else {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
        start_pc + (prefix_count + tail_count) * 4u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
        0u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
        0u);
    code.add(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        (prefix_count + tail_count + 1u) * 4u);
    code.add(code.r9d, prefix_count + tail_count + 2u);
    code.add(code.ebx, code.r9d);
    code.sub(code.r12d, prefix_count + tail_count + 1u);
  }

  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, memory_entries))]);
  if (control == nullptr) {
    emit_v4_link(code, links.fallthrough, links);
  } else if (control->op == V4ControlOp::Jr ||
             control->op == V4ControlOp::Jalr) {
    emit_v4_block_return(code);
  } else {
    emit_v4_selected_link(code, links);
  }

  code.L(unaligned);
  // EAX still carries the virtual effective address from the faulting load.
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_badvaddr))],
      code.eax);
  if (prefix_count != 0u) {
    code.add(code.ebx, prefix_count);
    code.sub(code.r12d, prefix_count);
  }
  emit_v4_exception_no_delay(code, Exception::AddrLoadErr,
                             start_pc + prefix_count * 4u);

  code.L(slow_after_prefix);
  if (prefix_count != 0u) {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
        start_pc + (prefix_count - 1u) * 4u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
        0u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
        0u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        start_pc + prefix_count * 4u);
    code.add(code.ebx, prefix_count);
    code.sub(code.r12d, prefix_count);
    emit_v4_block_return(code);
  }

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


V4NativeFn compile_v4_store(
    V4CodeArena &arena,
    const std::array<V4DecodedInstruction, kV4MaxBlockInstructions> &prefix,
    u32 prefix_count, const V4DecodedStore &store,
    const std::array<V4DecodedInstruction, kV4MaxBlockInstructions> &tail,
    u32 tail_count, const V4DecodedControl *control,
    const V4DecodedInstruction *delay, u32 branch_pc, u32 start_pc,
    bool cacheable, const V4LinkTargets &links, u32 &code_size) {
  using namespace Xbyak;
  constexpr size_t kReservation = 2048u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  code.setDefaultJmpNEAR(true);
  Label ram, scratch, stored, stop_after_store, unaligned, slow_after_prefix, bail;
  Label &guard_exit = prefix_count != 0u ? slow_after_prefix : bail;

  for (u32 i = 0; i < prefix_count; ++i) {
    emit_v4_alu_instruction(code, prefix[i]);
    if (i == 0u) {
      emit_retire_incoming_load(code, v4_alu_write_reg(prefix[i]));
    }
  }
  if (prefix_count != 0u) {
    code.add(code.ebx, prefix_count);
    code.sub(code.r12d, prefix_count);
  }


  // Cache-isolated stores target the guest I-cache rather than RAM.
  code.test(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))],
      1u << 16);
  code.jnz(guard_exit);


  // Capture both operands before retiring an incoming delayed load.
  emit_read_guest(code, code.eax, store.rs);
  code.add(code.eax, static_cast<u32>(store.simm));
  emit_read_guest(code, code.r8d, store.rt);

  if (store.op == V4StoreOp::Sh) {
    code.test(code.eax, 1u);
    code.jnz(unaligned);
  } else if (store.op == V4StoreOp::Sw) {
    code.test(code.eax, 3u);
    code.jnz(unaligned);
  }

  code.mov(code.edx, code.eax);
  code.and_(code.edx, 0x1FFFFFFFu);
  if (store.op == V4StoreOp::Swl || store.op == V4StoreOp::Swr) {
    code.mov(code.ecx, code.edx);
    code.and_(code.ecx, 3u);
    code.mov(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, store_byte_offset))],
        code.ecx);
    code.and_(code.edx, ~3u);
  }
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_phys))],
      code.edx);


  // Never directly write a translated 16-byte code line. A different line on
  // the same 4 KiB page is safe for cached code and should stay on fastmem.
  // Normalize RAM mirrors to the same 2 MiB backing address first.
  code.mov(code.r9d, code.edx);
  {
    Label line_key_ready;
    code.cmp(code.r9d, code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, mapped_main_ram_size))]);
    code.jae(line_key_ready);
    code.and_(code.r9d, psx::RAM_SIZE - 1u);
    code.L(line_key_ready);
  }
  code.shr(code.r9d, 4u);
  code.mov(code.ecx, code.r9d);
  code.shr(code.r9d, 6u);
  code.and_(code.ecx, 63u);
  code.mov(code.rax, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, code_line_bits))]);
  code.test(code.rax, code.rax);
  code.jz(guard_exit);
  code.mov(code.rax, code.qword[code.rax + code.r9 * 8]);
  code.shr(code.rax, code.cl);
  code.test(code.al, 1u);
  code.jnz(guard_exit);

  code.cmp(code.edx, code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, mapped_main_ram_size))]);
  code.jb(ram);
  code.cmp(code.edx, 0x1F800000u);
  code.jb(guard_exit);
  code.cmp(code.edx, 0x1F801000u);
  code.jae(guard_exit);

  auto emit_unaligned_store = [&]() {
    // EAX = old aligned memory word, R8D = guest register value,
    // R9D = byte offset. Return merged dword in EAX.
    Label off0, off1, off2, merged;
    code.test(code.r9d, code.r9d);
    code.jz(off0);
    code.cmp(code.r9d, 1u);
    code.je(off1);
    code.cmp(code.r9d, 2u);
    code.je(off2);

    if (store.op == V4StoreOp::Swl) {
      // offset 3
      code.mov(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off0);
      code.and_(code.eax, 0xFFFFFF00u);
      code.shr(code.r8d, 24);
      code.or_(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off1);
      code.and_(code.eax, 0xFFFF0000u);
      code.shr(code.r8d, 16);
      code.or_(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off2);
      code.and_(code.eax, 0xFF000000u);
      code.shr(code.r8d, 8);
      code.or_(code.eax, code.r8d);
    } else {
      // SWR offset 3
      code.and_(code.eax, 0x00FFFFFFu);
      code.shl(code.r8d, 24);
      code.or_(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off0);
      code.mov(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off1);
      code.and_(code.eax, 0x000000FFu);
      code.shl(code.r8d, 8);
      code.or_(code.eax, code.r8d);
      code.jmp(merged);
      code.L(off2);
      code.and_(code.eax, 0x0000FFFFu);
      code.shl(code.r8d, 16);
      code.or_(code.eax, code.r8d);
    }
    code.L(merged);
  };

  code.L(scratch);
  code.sub(code.edx, 0x1F800000u);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, scratchpad))]);
  code.test(code.rcx, code.rcx);
  code.jz(guard_exit);
  if (store.op == V4StoreOp::Swl || store.op == V4StoreOp::Swr) {
    code.mov(code.eax, code.dword[code.rcx + code.rdx]);
    code.mov(code.r9d, code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, store_byte_offset))]);
    emit_unaligned_store();
    code.mov(code.dword[code.rcx + code.rdx], code.eax);
  } else {
    switch (store.op) {
    case V4StoreOp::Sb:
      code.mov(code.byte[code.rcx + code.rdx], code.r8b);
      break;
    case V4StoreOp::Sh:
      code.mov(code.word[code.rcx + code.rdx], code.r8w);
      break;
    case V4StoreOp::Sw:
      code.mov(code.dword[code.rcx + code.rdx], code.r8d);
      break;
    default:
      break;
    }
  }
  code.xor_(code.r9d, code.r9d);
  code.jmp(stored);

  code.L(ram);
  code.and_(code.edx, psx::RAM_SIZE - 1u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_phys))],
      code.edx);
  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, main_ram))]);
  code.test(code.rcx, code.rcx);
  code.jz(guard_exit);
  if (store.op == V4StoreOp::Swl || store.op == V4StoreOp::Swr) {
    code.mov(code.eax, code.dword[code.rcx + code.rdx]);
    code.mov(code.r9d, code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, store_byte_offset))]);
    emit_unaligned_store();
    code.mov(code.dword[code.rcx + code.rdx], code.eax);
    // SWL/SWR are RAM read-modify-write operations: 4 read wait cycles + 1 write.
    code.mov(code.r9d, 5u);
  } else {
    switch (store.op) {
    case V4StoreOp::Sb:
      code.mov(code.byte[code.rcx + code.rdx], code.r8b);
      break;
    case V4StoreOp::Sh:
      code.mov(code.word[code.rcx + code.rdx], code.r8w);
      break;
    case V4StoreOp::Sw:
      code.mov(code.dword[code.rcx + code.rdx], code.r8d);
      break;
    default:
      break;
    }
    code.mov(code.r9d, 1u);
  }

  code.L(stored);

  // Account timing while the RAM/scratchpad penalty is still in r9d. The C++
  // invalidation helper below may clobber all caller-saved registers.
  code.add(code.ebx, 2u);
  code.add(code.ebx, code.r9d);

  // The translated-line guard above proves this store cannot modify any
  // currently translated instruction bytes. Preserve the emulator's guest
  // I-cache side effect in native code, avoiding a C++ helper on every store.
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_phys))]);
  code.shr(code.eax, 4u);
  code.and_(code.eax, 0xFFu);

  code.mov(code.rcx, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, icache_valid))]);
  code.mov(code.edx, code.eax);
  code.imul(code.edx, code.dword[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, icache_line_stride))]);
  code.mov(code.byte[code.rcx + code.rdx], 0u);

  code.mov(code.rcx, code.ptr[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, icache_generations))]);
  code.inc(code.dword[code.rcx + code.rax * 4]);
  {
    Label generation_ok;
    code.cmp(code.dword[code.rcx + code.rax * 4], 0u);
    code.jne(generation_ok);
    code.mov(code.dword[code.rcx + code.rax * 4], 1u);
    code.L(generation_ok);
  }

  // Cached execution is direct-mapped. If this store invalidated the exact
  // I-cache slot containing the fused code, the architectural next instruction
  // must refetch/refill before it executes. End this native fragment after the
  // store and let the normal dispatcher path perform that refill. Uncached code
  // has no guest I-cache dependency and can keep running through the tail.
  if (cacheable && (tail_count != 0u || control != nullptr)) {
    code.cmp(code.eax, (start_pc >> 4u) & 0xFFu);
    code.je(stop_after_store);
  }


  // The store retires the incoming delayed load. Any fused ALU tail therefore
  // observes the committed value, exactly like successive Cpu::step() calls.
  emit_retire_incoming_load(code, 0u);
  for (u32 i = 0; i < tail_count; ++i) {
    emit_v4_alu_instruction(code, tail[i]);
  }

  if (control != nullptr && delay != nullptr) {
    const V4DecodedControl &branch = *control;
    const u32 fallthrough = branch_pc + 8u;
    const u32 jump_target =
        ((branch_pc + 4u) & 0xF0000000u) | (branch.imm26 << 2u);
    const u32 branch_target =
        branch_pc + 4u + static_cast<u32>(branch.simm * 4);

    if (branch.op == V4ControlOp::Beq ||
        branch.op == V4ControlOp::Bne) {
      emit_read_guest(code, code.eax, branch.rs);
      emit_read_guest(code, code.ecx, branch.rt);
      code.cmp(code.eax, code.ecx);
      if (branch.op == V4ControlOp::Beq) {
        code.sete(code.dl);
      } else {
        code.setne(code.dl);
      }
      code.movzx(code.edx, code.dl);
    } else if (branch.op == V4ControlOp::Blez ||
               branch.op == V4ControlOp::Bgtz ||
               branch.op == V4ControlOp::Bltz ||
               branch.op == V4ControlOp::Bgez ||
               branch.op == V4ControlOp::Bltzal ||
               branch.op == V4ControlOp::Bgezal) {
      emit_read_guest(code, code.eax, branch.rs);
      code.cmp(code.eax, 0);
      switch (branch.op) {
      case V4ControlOp::Blez: code.setle(code.dl); break;
      case V4ControlOp::Bgtz: code.setg(code.dl); break;
      case V4ControlOp::Bltz:
      case V4ControlOp::Bltzal: code.setl(code.dl); break;
      case V4ControlOp::Bgez:
      case V4ControlOp::Bgezal: code.setge(code.dl); break;
      default: break;
      }
      code.movzx(code.edx, code.dl);
      if (branch.op == V4ControlOp::Bltzal ||
          branch.op == V4ControlOp::Bgezal) {
        code.mov(code.eax, branch_pc + 8u);
        emit_write_guest(code, 31u, code.eax);
      }
    } else if (branch.op == V4ControlOp::Jr ||
               branch.op == V4ControlOp::Jalr) {
      emit_read_guest(code, code.r8d, branch.rs);
      if (branch.op == V4ControlOp::Jalr) {
        code.mov(code.eax, branch_pc + 8u);
        emit_write_guest(code, branch.rd, code.eax);
      }
    } else if (branch.op == V4ControlOp::Jal) {
      code.mov(code.eax, branch_pc + 8u);
      emit_write_guest(code, 31u, code.eax);
    }

    emit_v4_alu_instruction(code, *delay);

    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
        branch_pc + 4u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
        1u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
        branch_pc);
    code.sub(code.r12d, tail_count + 3u);

    if (branch.op == V4ControlOp::J ||
        branch.op == V4ControlOp::Jal) {
      code.mov(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
          jump_target);
      code.add(code.ebx, tail_count + 3u);
      code.mov(code.r14, reinterpret_cast<size_t>(links.taken));
    } else if (branch.op == V4ControlOp::Jr ||
               branch.op == V4ControlOp::Jalr) {
      code.mov(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
          code.r8d);
      code.add(code.ebx, tail_count + 3u);
    } else {
      Label not_taken, selected;
      code.test(code.edx, code.edx);
      code.jz(not_taken);
      code.mov(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
          branch_target);
      code.add(code.ebx, tail_count + 3u);
      code.mov(code.r14, reinterpret_cast<size_t>(links.taken));
      code.jmp(selected);

      code.L(not_taken);
      code.mov(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
          fallthrough);
      code.add(code.ebx, tail_count + 2u);
      code.mov(code.r14, reinterpret_cast<size_t>(links.fallthrough));
      code.L(selected);
    }
  } else {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
        start_pc + (prefix_count + tail_count) * 4u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
        0u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
        0u);
    code.add(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        (prefix_count + tail_count + 1u) * 4u);
    if (tail_count != 0u) {
      code.add(code.ebx, tail_count);
    }
    code.sub(code.r12d, tail_count + 1u);
  }
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_entries))]);
  if (control == nullptr) {
    emit_v4_link(code, links.fallthrough, links);
  } else if (control->op == V4ControlOp::Jr ||
             control->op == V4ControlOp::Jalr) {
    emit_v4_block_return(code);
  } else {
    emit_v4_selected_link(code, links);
  }

  code.L(stop_after_store);
  emit_retire_incoming_load(code, 0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
      start_pc + prefix_count * 4u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
      0u);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      start_pc + (prefix_count + 1u) * 4u);
  code.dec(code.r12d);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, store_entries))]);
  emit_v4_link(code, links.after_store, links);

  code.L(unaligned);
  // EAX still carries the virtual effective address from the faulting store.
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_badvaddr))],
      code.eax);
  emit_v4_exception_no_delay(code, Exception::AddrStoreErr,
                             start_pc + prefix_count * 4u);

  code.L(slow_after_prefix);
  if (prefix_count != 0u) {
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_pc))],
        start_pc + (prefix_count - 1u) * 4u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, last_in_delay_slot))],
        0u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, active_branch_pc))],
        0u);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pc))],
        start_pc + prefix_count * 4u);
    emit_v4_block_return(code);
  }

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
    V4CodeArena &arena, V4NativeState *bound_state,
    void *&block_return, void *&linked_entry) {
  using namespace Xbyak;
  constexpr size_t kReservation = 2048u;
  void *buffer = arena.begin_emit(kReservation);
  if (buffer == nullptr) {
    return nullptr;
  }
  CodeGenerator code(kReservation, buffer);
  code.setDefaultJmpNEAR(true);
  Label loop, check_block, after_block, done;
  Label missing, stale_epoch, blocked_memory, stale_generation, budget_exit;
  Label revalidate_cached, bail_exit;

  code.push(code.rbx);
  code.push(code.r12);
  code.push(code.r13);
  code.push(code.r14);
  code.push(code.r15);
#if defined(_WIN32)
  code.sub(code.rsp, 32);
  code.mov(code.r11, code.rcx);
#else
  code.mov(code.r11, code.rdi);
#endif

  // V4 blocks are internal fragments, not ABI-callable functions. Keep the
  // resident state and GPR base pinned for the whole dispatcher lifetime.
  code.mov(code.r10, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, gpr))]);

  // Keep hot scheduler counters resident across the native chain. r12d is the
  // instruction downcounter and ebx mirrors V4NativeState::cycles.
  code.mov(code.r12d, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, instruction_budget))]);
  code.xor_(code.ebx, code.ebx);
  code.mov(code.r13d, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cache_epoch))]);
  code.mov(code.r15, code.ptr[
      code.r11 +
      static_cast<int>(offsetof(V4NativeState, icache_generations))]);

  code.L(loop);
  {
    Label normal_dispatch;
    code.cmp(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pending_delay_slot))],
        0u);
    code.je(normal_dispatch);
    code.test(code.r12d, code.r12d);
    code.jz(budget_exit);
    code.cmp(code.ebx, code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cycle_budget))]);
    code.jae(budget_exit);
    code.mov(code.rax, code.ptr[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pending_delay_fn))]);
    code.test(code.rax, code.rax);
    code.jz(missing);
    code.jmp(code.rax);
    code.L(normal_dispatch);
  }
  code.xor_(code.r9d, code.r9d);
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, pc))]);
  code.mov(code.edx, code.eax);
  code.shr(code.eax, 12);
  code.mov(code.r14, code.ptr[
      code.r11 + static_cast<int>(offsetof(V4NativeState, dispatch_top))]);
  code.mov(code.r14, code.ptr[code.r14 + code.rax * 8]);
  code.test(code.r14, code.r14);
  code.jz(missing);

  code.mov(code.eax, code.edx);
  code.shr(code.eax, 2);
  code.and_(code.eax, 0x3FFu);
  code.mov(code.r14, code.ptr[code.r14 + code.rax * 8]);
  code.L(check_block);
  code.test(code.r14, code.r14);
  code.jz(missing);
  code.cmp(code.dword[
               code.r14 + static_cast<int>(offsetof(V4Block, cache_epoch))],
           code.r13d);
  code.jne(stale_epoch);

  {
    Label memory_ok;
    code.cmp(code.byte[
        code.r14 + static_cast<int>(offsetof(V4Block, has_memory))], 0u);
    code.je(memory_ok);
    code.cmp(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, memory_fastpath_allowed))],
        0u);
    code.je(blocked_memory);
    code.L(memory_ok);
  }

  {
    Label uncached, validity_ok;
    code.cmp(code.byte[
        code.r14 + static_cast<int>(offsetof(V4Block, cacheable))], 0u);
    code.je(uncached);

    // Cached code: exact guest I-cache line generation is authoritative.
    code.movzx(code.ecx, code.word[
        code.r14 + static_cast<int>(offsetof(V4Block, icache_index))]);
    code.mov(code.edx, code.dword[code.r15 + code.rcx * 4]);
    code.cmp(code.edx, code.dword[
        code.r14 + static_cast<int>(offsetof(V4Block, icache_generation))]);
    code.jne(revalidate_cached);
    {
      Label one_line;
      code.cmp(code.byte[
          code.r14 + static_cast<int>(offsetof(V4Block, second_icache_line))],
          0u);
      code.je(one_line);
      code.movzx(code.ecx, code.word[
          code.r14 + static_cast<int>(offsetof(V4Block, second_icache_index))]);
      code.mov(code.edx, code.dword[code.r15 + code.rcx * 4]);
      code.cmp(code.edx, code.dword[
          code.r14 +
          static_cast<int>(offsetof(V4Block, second_icache_generation))]);
      code.jne(revalidate_cached);
      code.L(one_line);
    }
    code.jmp(validity_ok);

    // Generation mismatches are common with the PS1's direct-mapped I-cache.
    // Handle the complete line-refill + byte-validation path in resident x64.
    // This is a fundamental native-only invariant: a hot cached block never
    // calls C++ merely to prove that its guest-visible instruction bytes match.
    code.L(revalidate_cached);
    code.test(code.r12d, code.r12d);
    code.jz(budget_exit);
    code.cmp(code.ebx, code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cycle_budget))]);
    code.jae(budget_exit);
    code.cmp(code.byte[
        code.r14 + static_cast<int>(offsetof(V4Block, retry_second_line))], 0u);
    code.jne(stale_generation);
    code.cmp(code.byte[
        code.r14 +
        static_cast<int>(offsetof(V4Block, budget_requires_empty_chain))], 0u);
    code.jne(stale_generation);
    code.inc(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, revalidate_attempts))]);

    {
      Label compare_loop, refill_line, refill_ram, refill_scratch;
      Label refill_source_ready, refill_generation_ok, line_ready;
      Label revalidate_done, no_second_generation, refill_invalid_region;

      // r9d is the translated instruction index. r8/rax/rcx/rdx are volatile
      // scratch registers; the resident bases in r10-r15/rbx remain untouched.
      code.xor_(code.r9d, code.r9d);
      code.L(compare_loop);
      code.cmp(code.r9d, code.dword[
          code.r14 + static_cast<int>(offsetof(V4Block, instruction_count))]);
      code.jae(revalidate_done);

      // edx = guest PC for this translated instruction.
      code.mov(code.edx, code.dword[
          code.r14 + static_cast<int>(offsetof(V4Block, start_pc))]);
      code.mov(code.eax, code.r9d);
      code.shl(code.eax, 2);
      code.add(code.edx, code.eax);

      // eax/rax = byte offset of this direct-mapped I-cache line.
      code.mov(code.ecx, code.edx);
      code.shr(code.ecx, 4);
      code.and_(code.ecx, 0xFFu);
      code.mov(code.eax, code.ecx);
      code.imul(code.eax, code.dword[
          code.r11 +
          static_cast<int>(offsetof(V4NativeState, icache_line_stride))]);
      code.movsxd(code.rax, code.eax);

      code.mov(code.r8, code.ptr[
          code.r11 + static_cast<int>(offsetof(V4NativeState, icache_valid))]);
      code.cmp(code.byte[code.r8 + code.rax], 0u);
      code.je(refill_line);
      code.mov(code.r8, code.ptr[
          code.r11 + static_cast<int>(offsetof(V4NativeState, icache_tags))]);
      code.mov(code.ecx, code.edx);
      code.and_(code.ecx, 0x1FFFFFF0u);
      code.cmp(code.dword[code.r8 + code.rax], code.ecx);
      code.je(line_ready);

      code.L(refill_line);
      // Recompute line index/offset and convert EDX to its physical line tag.
      code.mov(code.ecx, code.edx);
      code.shr(code.ecx, 4);
      code.and_(code.ecx, 0xFFu);
      code.mov(code.eax, code.ecx);
      code.imul(code.eax, code.dword[
          code.r11 +
          static_cast<int>(offsetof(V4NativeState, icache_line_stride))]);
      code.movsxd(code.rax, code.eax);
      code.and_(code.edx, 0x1FFFFFF0u);

      // Publish the architectural tag/valid state and generation exactly once
      // for the refill. Source bytes come directly from RAM or scratchpad; those
      // are the only regions Cpu::instruction_cacheable() admits.
      code.mov(code.r8, code.ptr[
          code.r11 + static_cast<int>(offsetof(V4NativeState, icache_tags))]);
      code.mov(code.dword[code.r8 + code.rax], code.edx);
      code.mov(code.r8, code.ptr[
          code.r11 + static_cast<int>(offsetof(V4NativeState, icache_valid))]);
      code.mov(code.byte[code.r8 + code.rax], 1u);
      code.inc(code.dword[code.r15 + code.rcx * 4]);
      code.jnz(refill_generation_ok);
      code.mov(code.dword[code.r15 + code.rcx * 4], 1u);
      code.L(refill_generation_ok);

      // rcx = destination word array for the selected line.
      code.mov(code.rcx, code.ptr[
          code.r11 + static_cast<int>(offsetof(V4NativeState, icache_words))]);
      code.add(code.rcx, code.rax);

      code.cmp(code.edx, 0x00800000u);
      code.jb(refill_ram);
      code.cmp(code.edx, 0x1F800000u);
      code.jb(refill_invalid_region);
      code.cmp(code.edx, 0x1F801000u);
      code.jae(refill_invalid_region);
      code.L(refill_scratch);
      code.mov(code.r8, code.ptr[
          code.r11 + static_cast<int>(offsetof(V4NativeState, scratchpad))]);
      code.mov(code.eax, code.edx);
      code.sub(code.eax, 0x1F800000u);
      code.add(code.r8, code.rax);
      code.jmp(refill_source_ready);

      code.L(refill_ram);
      code.mov(code.r8, code.ptr[
          code.r11 + static_cast<int>(offsetof(V4NativeState, main_ram))]);
      code.mov(code.eax, code.edx);
      code.and_(code.eax, 0x001FFFFFu);
      code.add(code.r8, code.rax);

      code.L(refill_source_ready);
      code.mov(code.eax, code.dword[code.r8 + 0]);
      code.mov(code.dword[code.rcx + 0], code.eax);
      code.mov(code.eax, code.dword[code.r8 + 4]);
      code.mov(code.dword[code.rcx + 4], code.eax);
      code.mov(code.eax, code.dword[code.r8 + 8]);
      code.mov(code.dword[code.rcx + 8], code.eax);
      code.mov(code.eax, code.dword[code.r8 + 12]);
      code.mov(code.dword[code.rcx + 12], code.eax);
      code.add(code.ebx, 4u);
      code.inc(code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, icache_refills))]);

      code.L(line_ready);
      // Reconstruct the guest PC, locate the guest-visible word, and compare it
      // to the byte snapshot used to compile the block.
      code.mov(code.edx, code.dword[
          code.r14 + static_cast<int>(offsetof(V4Block, start_pc))]);
      code.mov(code.eax, code.r9d);
      code.shl(code.eax, 2);
      code.add(code.edx, code.eax);
      code.mov(code.ecx, code.edx);
      code.shr(code.ecx, 4);
      code.and_(code.ecx, 0xFFu);
      code.mov(code.eax, code.ecx);
      code.imul(code.eax, code.dword[
          code.r11 +
          static_cast<int>(offsetof(V4NativeState, icache_line_stride))]);
      code.movsxd(code.rax, code.eax);
      code.mov(code.r8, code.ptr[
          code.r11 + static_cast<int>(offsetof(V4NativeState, icache_words))]);
      code.add(code.r8, code.rax);
      code.mov(code.eax, code.edx);
      code.shr(code.eax, 2);
      code.and_(code.eax, 3u);
      code.mov(code.ecx, code.dword[code.r8 + code.rax * 4]);
      code.cmp(code.ecx, code.dword[
          code.r14 + code.r9 * 4 +
          static_cast<int>(offsetof(V4Block, guest_bits))]);
      code.jne(stale_generation);
      code.inc(code.r9d);
      code.jmp(compare_loop);

      code.L(refill_invalid_region);
      code.jmp(stale_generation);

      code.L(revalidate_done);
      code.movzx(code.ecx, code.word[
          code.r14 + static_cast<int>(offsetof(V4Block, icache_index))]);
      code.mov(code.eax, code.dword[code.r15 + code.rcx * 4]);
      code.mov(code.dword[
          code.r14 + static_cast<int>(offsetof(V4Block, icache_generation))],
          code.eax);
      code.cmp(code.byte[
          code.r14 + static_cast<int>(offsetof(V4Block, second_icache_line))],
          0u);
      code.je(no_second_generation);
      code.movzx(code.ecx, code.word[
          code.r14 + static_cast<int>(offsetof(V4Block, second_icache_index))]);
      code.mov(code.eax, code.dword[code.r15 + code.rcx * 4]);
      code.mov(code.dword[
          code.r14 +
          static_cast<int>(offsetof(V4Block, second_icache_generation))],
          code.eax);
      code.L(no_second_generation);
    }

    code.inc(code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, revalidate_successes))]);
    // Cpu::step() begins the instruction while the pre-fetch cycle count is
    // still inside the slice. If the architectural 4-cycle I-cache refill then
    // reaches or crosses the deadline, that already-started instruction still
    // retires. Reproduce that rule with the one-instruction native budget
    // fragment instead of returning to C++ for the boundary case.
    {
      Label refill_inside_budget, refill_fragment_ready;
      code.cmp(code.ebx, code.dword[
          code.r11 + static_cast<int>(offsetof(V4NativeState, cycle_budget))]);
      code.jb(refill_inside_budget);
      code.test(code.r12d, code.r12d);
      code.jz(budget_exit);
      code.mov(code.rax, code.ptr[
          code.r14 + static_cast<int>(offsetof(V4Block, budget_fn))]);
      code.test(code.rax, code.rax);
      code.jz(budget_exit);
      code.cmp(code.byte[
          code.r14 +
          static_cast<int>(offsetof(V4Block, budget_requires_empty_chain))], 0u);
      code.je(refill_fragment_ready);
      // A branch-only budget fragment must start from a fresh chain. Yielding
      // here keeps the same boundary without executing guest semantics in C++.
      code.jmp(budget_exit);
      code.L(refill_fragment_ready);
      code.jmp(code.rax);
      code.L(refill_inside_budget);
    }
    code.jmp(validity_ok);

    // Uncached code: RAM writes are observed immediately, so retain the
    // physical-page generation guard.
    code.L(uncached);
    code.mov(code.rax, code.ptr[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, code_page_generations))]);
    code.mov(code.ecx, code.dword[
        code.r14 + static_cast<int>(offsetof(V4Block, phys_page))]);
    code.mov(code.edx, code.dword[code.rax + code.rcx * 4]);
    code.cmp(code.edx, code.dword[
        code.r14 +
        static_cast<int>(offsetof(V4Block, code_page_generation))]);
    code.jne(stale_generation);
    code.L(validity_ok);
  }

  Label full_instruction_budget, full_cycle_budget, try_budget_fragment;
  code.cmp(code.r12d, code.dword[
      code.r14 + static_cast<int>(offsetof(V4Block, instruction_count))]);
  code.jae(full_instruction_budget);
  code.jmp(try_budget_fragment);

  code.L(full_instruction_budget);
  {
    Label cycle_ok, strict_cycle_budget;
    code.cmp(code.dword[
        code.r14 + static_cast<int>(offsetof(V4Block, instruction_count))], 1u);
    code.jne(strict_cycle_budget);

    // Cpu::run_slice() / Cpu::step() already allow one architectural
    // instruction to overshoot the remaining cycle budget. Preserve that
    // contract for a one-instruction native block instead of rejecting it only
    // to execute the same instruction through the interpreter.
    code.cmp(code.ebx, code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cycle_budget))]);
    code.jb(cycle_ok);
    code.jmp(try_budget_fragment);

    code.L(strict_cycle_budget);
    code.mov(code.eax, code.ebx);
    code.add(code.eax, code.dword[
        code.r14 + static_cast<int>(offsetof(V4Block, max_cycles))]);
    code.cmp(code.eax, code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cycle_budget))]);
    code.ja(try_budget_fragment);
    code.L(cycle_ok);
  }
  code.jmp(full_cycle_budget);

  // A translated block can be larger than the remaining scheduler budget. If
  // its first instruction is independently compilable, execute that exact one
  // instruction natively and return through the normal resident trampoline.
  // This keeps the historical scheduler boundary while avoiding a helper call.
  code.L(try_budget_fragment);
  code.test(code.r12d, code.r12d);
  code.jz(budget_exit);
  code.mov(code.rax, code.ptr[
      code.r14 + static_cast<int>(offsetof(V4Block, budget_fn))]);
  code.test(code.rax, code.rax);
  code.jz(budget_exit);
  {
    Label budget_chain_ok;
    code.cmp(code.byte[
        code.r14 +
        static_cast<int>(offsetof(V4Block, budget_requires_empty_chain))], 0u);
    code.je(budget_chain_ok);
    code.cmp(code.r12d, code.dword[
        code.r11 +
        static_cast<int>(offsetof(V4NativeState, instruction_budget))]);
    code.jne(budget_exit);
    code.L(budget_chain_ok);
  }
  code.cmp(code.ebx, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cycle_budget))]);
  code.jae(budget_exit);
  code.jmp(code.rax);

  code.L(full_cycle_budget);
  code.mov(code.rax, code.ptr[
      code.r14 + static_cast<int>(offsetof(V4Block, fn))]);
  code.test(code.rax, code.rax);
  code.jz(missing);
  {
    Label ordinary_entry;
    code.test(code.r9d, code.r9d);
    code.jz(ordinary_entry);
    code.inc(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, direct_links))]);
    code.L(ordinary_entry);
  }

  // Tail-jump into translated code. The block returns here via the pointer
  // pinned in V4NativeState, eliminating call/ret and per-block ABI setup.
  code.jmp(code.rax);

  code.L(after_block);
  block_return = const_cast<u8 *>(code.getCurr());
  code.cmp(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, block_bail))], 0u);
  code.jne(bail_exit);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, block_entries))]);
  {
    Label no_scheduler_yield, post_delay_irq_clear;
    code.cmp(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, scheduler_yield))],
        0u);
    code.je(no_scheduler_yield);

    // Split branches still yield before their delay slot to preserve the
    // scheduler boundary. Once the delay slot itself has retired, the only
    // required boundary here is interrupt sampling. Check the already-synced
    // COP0 state in native code and keep the resident chain alive when no IRQ
    // is eligible.
    code.cmp(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, pending_delay_slot))],
        0u);
    code.jne(done);
    code.mov(code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, scheduler_yield))],
        0u);
    code.mov(code.eax, code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_sr))]);
    code.test(code.eax, 1u);
    code.jz(post_delay_irq_clear);
    code.mov(code.ecx, code.dword[
        code.r11 + static_cast<int>(offsetof(V4NativeState, cop0_cause))]);
    code.and_(code.ecx, code.eax);
    code.test(code.ecx, 0xFF00u);
    code.jnz(done);
    code.L(post_delay_irq_clear);
    code.jmp(loop);

    code.L(no_scheduler_yield);
    code.jmp(loop);
  }

  linked_entry = const_cast<u8 *>(code.getCurr());
  code.cmp(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, block_bail))], 0u);
  code.jne(bail_exit);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, block_entries))]);
  code.mov(code.r9d, 1u);
  code.jmp(check_block);

  code.L(missing);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, missing_exits))]);
  code.jmp(done);
  code.L(stale_epoch);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, epoch_exits))]);
  code.jmp(done);
  code.L(blocked_memory);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, memory_exits))]);
  code.jmp(done);
  code.L(stale_generation);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, generation_exits))]);
  code.jmp(done);
  code.L(budget_exit);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, budget_exits))]);
  code.jmp(done);
  code.L(bail_exit);
  code.inc(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, bail_exits))]);

  code.L(done);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, cycles))], code.ebx);
  code.mov(code.eax, code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, instruction_budget))]);
  code.sub(code.eax, code.r12d);
  code.mov(code.dword[
      code.r11 + static_cast<int>(offsetof(V4NativeState, instructions))],
      code.eax);
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

struct CpuRecompilerBackend::Impl {
#if VIBESTATION_JIT_V4_X64
  V4CodeArena arena;
  std::unique_ptr<V4DispatchPage *[]> dispatch_top;
  std::vector<std::unique_ptr<V4DispatchPage>> dispatch_pages;
  std::unique_ptr<V4Block[]> blocks;
  size_t block_count = 0u;
  u32 cache_epoch = 1u;
  JitCodePageBitmap<29u, 12u> code_pages;
  JitCodePageBitmap<29u, 4u> code_lines;
  std::array<u32, kV4PhysPageCount> page_generations{};
  V4ResidentDispatchFn resident_dispatch = nullptr;
  V4NativeState native_state{};
  bool native_state_bound = false;
  std::unordered_map<u32, V4NativeFn> pending_delay_alu_cache;
  std::unordered_map<u32, V4HelperFn> helper_cache;
  void *resident_block_return = nullptr;
  void *resident_linked_entry = nullptr;
  size_t permanent_code_bytes = 0u;
  bool direct_links_enabled = true;
  bool crossline_branch_enabled = true;
  bool helper_profile_enabled = false;
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
    // Diagnostic A/B switch; sampled only when this backend is initialized.
    const char *disable_links = std::getenv("VIBESTATION_V4_DISABLE_DIRECT_LINKS");
    direct_links_enabled =
        disable_links == nullptr || disable_links[0] != '1';
    const char *disable_crossline =
        std::getenv("VIBESTATION_V4_DISABLE_CROSSLINE_BRANCH");
    crossline_branch_enabled =
        disable_crossline == nullptr || disable_crossline[0] != '1';
    const char *profile_helpers =
        std::getenv("VIBESTATION_V4_PROFILE_HELPERS");
    helper_profile_enabled =
        profile_helpers != nullptr && profile_helpers[0] == '1';
    resident_dispatch =
        install_v4_resident_dispatch(arena, &native_state,
                                     resident_block_return,
                                     resident_linked_entry);
    if (resident_dispatch == nullptr || resident_block_return == nullptr ||
        resident_linked_entry == nullptr) {
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

  V4Block *lookup_candidate(u32 pc, bool cacheable) {
    V4DispatchEntry *entry = dispatch_entry(pc, false);
    if (entry == nullptr || entry->block == nullptr) {
      return nullptr;
    }
    V4Block *block = entry->block;
    if (block->cache_epoch != cache_epoch || block->cacheable != cacheable) {
      return nullptr;
    }

    if (cacheable) {
      // Do not inspect guest I-cache generations here. The resident x64
      // dispatcher owns refill + byte validation, so a hot block never crosses
      // back into C++ merely because an alias changed its direct-mapped line.
      return block;
    }

    // KSEG1/uncached code observes RAM directly. A page-generation mismatch
    // means the old translation is stale; compile a replacement rather than
    // byte-comparing it in C++.
    if (block->phys_page >= kV4PhysPageCount ||
        block->code_page_generation != page_generations[block->phys_page]) {
      return nullptr;
    }
    return block;
  }

  bool crossline_waiting_for_refill(Cpu &cpu, u32 pc, bool cacheable) {
    if (!cacheable) {
      return false;
    }
    V4DispatchEntry *entry = dispatch_entry(pc, false);
    if (entry == nullptr || entry->block == nullptr) {
      return false;
    }
    const V4Block *block = entry->block;
    if (block->cache_epoch != cache_epoch || block->start_pc != pc ||
        !block->second_icache_line ||
        block->instruction_count < 2u) {
      return false;
    }
    for (u32 i = 0u; i + 1u < block->instruction_count; ++i) {
      u32 visible = 0u;
      if (!cpu.read_visible_instruction_for_backend(pc + i * 4u, visible) ||
          visible != block->guest_bits[i]) {
        return false;
      }
    }
    u32 delay_bits = 0u;
    return !cpu.read_visible_instruction_for_backend(
        pc + (block->instruction_count - 1u) * 4u, delay_bits);
  }

  V4Block *try_revalidate(Cpu &cpu, u32 pc, bool cacheable,
                          u32 icache_generation) {
    V4DispatchEntry *entry = dispatch_entry(pc, false);
    if (entry == nullptr || entry->block == nullptr) {
      return nullptr;
    }
    V4Block *block = entry->block;
    if (block->cache_epoch != cache_epoch || block->cacheable != cacheable ||
        block->instruction_count == 0u ||
        block->instruction_count > kV4MaxBlockInstructions) {
      return nullptr;
    }
    if (block->retry_second_line) {
      u32 delay_bits = 0u;
      if (cpu.read_visible_instruction_for_backend(pc + 4u, delay_bits)) {
        return nullptr;
      }
    }

    for (u32 i = 0; i < block->instruction_count; ++i) {
      u32 visible = 0u;
      if (!cpu.read_visible_instruction_for_backend(pc + i * 4u, visible) ||
          visible != block->guest_bits[i]) {
        return nullptr;
      }
    }

    if (cacheable) {
      block->icache_generation = icache_generation;
      if (block->second_icache_line) {
        block->second_icache_generation =
            cpu.instruction_cache_generation_for_backend(
                static_cast<u32>(block->second_icache_index) << 4u);
      }
    } else {
      if (block->phys_page >= kV4PhysPageCount) {
        return nullptr;
      }
      block->code_page_generation = page_generations[block->phys_page];
    }
    return block;
  }

  void install(u32 pc, V4Block *block) {
    V4DispatchEntry *entry = dispatch_entry(pc, true);
    if (entry == nullptr) {
      return;
    }
    entry->block = block;
  }

  void reset_translations() {
    if (!initialized) {
      return;
    }
    arena.reset_to(permanent_code_bytes);
    helper_cache.clear();
    pending_delay_alu_cache.clear();
    block_count = 0u;
    code_pages.clear();
    code_lines.clear();
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

  V4NativeFn pending_delay_alu_for(u32 instruction) {
    const auto found = pending_delay_alu_cache.find(instruction);
    if (found != pending_delay_alu_cache.end()) {
      return found->second;
    }
    u32 code_size = 0u;
    V4NativeFn fn = nullptr;
    V4DecodedInstruction decoded{};
    if (decode_v4_alu(instruction, decoded)) {
      fn = compile_v4_pending_delay_alu(arena, decoded, code_size);
    } else {
      V4DecodedOverflowAlu overflow{};
      if (decode_v4_overflow_alu(instruction, overflow)) {
        fn = compile_v4_pending_delay_overflow_alu(
            arena, overflow, code_size);
      } else {
        V4DecodedStore store{};
        if (decode_v4_store(instruction, store)) {
          fn = compile_v4_pending_delay_store(arena, store, code_size);
        }
      }
    }
    if (fn != nullptr) {
      pending_delay_alu_cache.emplace(instruction, fn);
    }
    return fn;
  }

  V4HelperFn helper_for(u32 instruction) {
    const auto found = helper_cache.find(instruction);
    if (found != helper_cache.end()) {
      return found->second;
    }
    V4HelperFn fn = compile_v4_helper(arena, instruction);
    if (fn != nullptr) {
      helper_cache.emplace(instruction, fn);
    }
    return fn;
  }

  V4Block *acquire_block(u32 pc, bool &reused) {
    V4DispatchEntry *entry = dispatch_entry(pc, false);
    if (entry != nullptr && entry->block != nullptr &&
        entry->block->cache_epoch == cache_epoch &&
        entry->block->start_pc == pc) {
      reused = true;
      V4Block *block = entry->block;
      *block = {};
      return block;
    }

    reused = false;
    if (block_count >= kV4MaxBlocks) {
      return nullptr;
    }
    V4Block *block = &blocks[block_count++];
    *block = {};
    return block;
  }

  V4Block *compile_block(Cpu &cpu, CpuBackendStats &stats, u32 start_pc,
                         bool cacheable, u32 icache_generation) {
    bool reused_block = false;
    V4Block *block = acquire_block(start_pc, reused_block);
    if (block == nullptr) {
      return nullptr;
    }

    block->start_pc = start_pc;
    block->cache_epoch = cache_epoch;
    block->cacheable = cacheable;
    block->icache_index = static_cast<u16>((start_pc >> 4) & 0xFFu);
    block->icache_generation = cacheable ? icache_generation : 0u;
    const u32 start_phys =
        v4_normalize_code_phys(psx::mask_address(start_pc));
    block->phys_page = start_phys >> kV4PhysPageShift;
    block->code_page_generation = page_generations[block->phys_page];

    // Straight-line cached blocks stay within one guest I-cache line. A
    // branch may include its delay slot from the next line only when that
    // line is already guest-visible, with a separate generation guard.
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
    const bool control_pair_cross_line =
        crossline_branch_enabled && cacheable && count < decode_limit &&
        count + 1u == decode_limit &&
        (branch_pc & 0x0Fu) == 0x0Cu;
    const bool control_candidate =
        (control_pair_within_decode_limit || control_pair_cross_line) &&
        read_visible(branch_pc, control_bits) &&
        decode_v4_control(control_bits, control);
    const bool delay_visible =
        control_candidate && read_visible(branch_pc + 4u, delay_bits);
    const bool split_control =
        count == 0u && control_pair_cross_line && control_candidate &&
        !delay_visible;
    const bool simple_control =
        delay_visible && decode_v4_alu(delay_bits, delay);
    V4DecodedOverflowAlu guarded_control_delay{};
    const bool guarded_control =
        count == 0u && delay_visible && v4_nonlink_conditional(control) &&
        decode_v4_overflow_alu(delay_bits, guarded_control_delay);
    V4DecodedStore guarded_store_delay{};
    const bool guarded_store_control =
        count == 0u && delay_visible && v4_nonlink_conditional(control) &&
        decode_v4_store(delay_bits, guarded_store_delay);
    if ((simple_control || guarded_control || guarded_store_control) &&
        control_pair_cross_line) {
      block->second_icache_line = true;
      block->second_icache_index =
          static_cast<u16>(((branch_pc + 4u) >> 4u) & 0xFFu);
      block->second_icache_generation =
          cpu.instruction_cache_generation_for_backend(branch_pc + 4u);
    }

    const u32 memory_pc = start_pc + count * 4u;
    V4DecodedLoad load{};
    u32 load_bits = 0u;
    const bool simple_load =
        count < decode_limit && read_visible(memory_pc, load_bits) &&
        decode_v4_load(load_bits, load);
    V4DecodedStore store{};
    u32 store_bits = 0u;
    const bool simple_store =
        count < decode_limit && read_visible(memory_pc, store_bits) &&
        decode_v4_store(store_bits, store);
    V4DecodedOverflowAlu overflow_alu{};
    u32 overflow_bits = 0u;
    const bool simple_overflow_alu =
        count == 0u && read_visible(start_pc, overflow_bits) &&
        decode_v4_overflow_alu(overflow_bits, overflow_alu);
    V4DecodedHiLo hilo{};
    u32 hilo_bits = 0u;
    const bool simple_hilo =
        count == 0u && read_visible(start_pc, hilo_bits) &&
        decode_v4_hilo(hilo_bits, hilo);
    V4DecodedMulDiv muldiv{};
    u32 muldiv_bits = 0u;
    const bool simple_muldiv =
        count == 0u && read_visible(start_pc, muldiv_bits) &&
        decode_v4_muldiv(muldiv_bits, muldiv);
    V4DecodedCop0 cop0{};
    u32 cop0_bits = 0u;
    const bool simple_cop0 =
        count == 0u && read_visible(start_pc, cop0_bits) &&
        decode_v4_cop0(cop0_bits, cop0);
    V4DecodedException exception_inst{};
    u32 exception_bits = 0u;
    const bool simple_exception =
        count == 0u && read_visible(start_pc, exception_bits) &&
        decode_v4_exception(exception_bits, exception_inst);
    std::array<V4DecodedInstruction, kV4MaxBlockInstructions> store_tail{};
    u32 store_tail_count = 0u;
    if (simple_store) {
      for (u32 i = count + 1u; i < decode_limit; ++i) {
        V4DecodedInstruction inst{};
        u32 bits = 0u;
        if (!read_visible(start_pc + i * 4u, bits) ||
            !decode_v4_alu(bits, inst)) {
          break;
        }
        store_tail[store_tail_count++] = inst;
      }
    }
    V4DecodedControl store_control{};
    V4DecodedInstruction store_delay{};
    u32 store_control_bits = 0u;
    u32 store_delay_bits = 0u;
    const u32 store_branch_pc =
        start_pc + (count + store_tail_count + 1u) * 4u;
    const bool store_control_pair_within_decode_limit =
        simple_store && count + store_tail_count + 3u <= decode_limit;
    const bool store_has_control =
        store_control_pair_within_decode_limit &&
        read_visible(store_branch_pc, store_control_bits) &&
        read_visible(store_branch_pc + 4u, store_delay_bits) &&
        decode_v4_control(store_control_bits, store_control) &&
        decode_v4_alu(store_delay_bits, store_delay);

    std::array<V4DecodedInstruction, kV4MaxBlockInstructions> load_tail{};
    u32 load_tail_count = 0u;
    if (simple_load) {
      for (u32 i = count + 1u; i < decode_limit; ++i) {
        V4DecodedInstruction inst{};
        u32 bits = 0u;
        if (!read_visible(start_pc + i * 4u, bits) ||
            !decode_v4_alu(bits, inst)) {
          break;
        }
        load_tail[load_tail_count++] = inst;
      }
    }

    V4DecodedControl load_control{};
    V4DecodedInstruction load_delay{};
    u32 load_control_bits = 0u;
    u32 load_delay_bits = 0u;
    const u32 load_branch_pc =
        start_pc + (count + load_tail_count + 1u) * 4u;
    const bool load_control_pair_within_decode_limit =
        simple_load && count + load_tail_count + 3u <= decode_limit;
    const bool load_has_control =
        load_control_pair_within_decode_limit &&
        read_visible(load_branch_pc, load_control_bits) &&
        read_visible(load_branch_pc + 4u, load_delay_bits) &&
        decode_v4_control(load_control_bits, load_control) &&
        decode_v4_alu(load_delay_bits, load_delay);

    if (count == 0u && !split_control && !simple_control && !guarded_control &&
        !guarded_store_control && !simple_load && !simple_store &&
        !simple_overflow_alu && !simple_hilo && !simple_muldiv && !simple_cop0 &&
        !simple_exception) {
      ++stats.native_compile_attempts;
      u32 rejected_bits = 0u;
      (void)read_visible(start_pc, rejected_bits);
      block->reject_kind = classify_v4_reject(rejected_bits);
      block->instruction_count = 1u;
      block->guest_bits[0] = rejected_bits;
      block->retry_second_line =
          control_pair_cross_line && control_candidate && !delay_visible;
      block->helper_fn = helper_for(rejected_bits);
      if (block->helper_fn == nullptr) {
        if (!reused_block) {
          --block_count;
        }
        ++stats.native_compile_failures;
        return nullptr;
      }
      code_pages.mark_address(start_phys);
      code_lines.mark_address(start_phys);
      install(start_pc, block);
      ++stats.native_compile_successes;
      ++stats.native_blocks_compiled;
      ++stats.native_compiled_block_size_histogram[1];
      ++stats.native_blocks;
      stats.block_count = static_cast<u32>(block_count);
      stats.native_code_bytes = arena.bytes_used();
      stats.code_bytes = arena.bytes_used();
      return block;
    }

    ++stats.native_compile_attempts;
    try {
      V4LinkTargets links{};
      links.entry = direct_links_enabled ? resident_linked_entry : nullptr;
      const auto link_control = [&](const V4DecodedControl &branch,
                                    u32 pc) {
        if (branch.op == V4ControlOp::Jr ||
            branch.op == V4ControlOp::Jalr) {
          return;
        }
        if (branch.op == V4ControlOp::J ||
            branch.op == V4ControlOp::Jal) {
          const u32 target =
              ((pc + 4u) & 0xF0000000u) | (branch.imm26 << 2u);
          links.taken = dispatch_entry(target, true);
        } else {
          const u32 target =
              pc + 4u + static_cast<u32>(branch.simm * 4);
          links.taken = dispatch_entry(target, true);
          links.fallthrough = dispatch_entry(pc + 8u, true);
        }
      };
      if (direct_links_enabled) {
        if (simple_control || guarded_control || guarded_store_control) {
          link_control(control, branch_pc);
        } else if (simple_load && load_has_control) {
          link_control(load_control, load_branch_pc);
        } else if (simple_store && store_has_control) {
          link_control(store_control, store_branch_pc);
        } else {
          const u32 translated_count =
              (simple_overflow_alu || simple_hilo || simple_muldiv || simple_cop0 || simple_exception)
                  ? 1u
                  : (simple_load ? count + load_tail_count + 1u
                                 : (simple_store
                                        ? count + store_tail_count + 1u
                                        : count));
          links.fallthrough =
              dispatch_entry(start_pc + translated_count * 4u, true);
        }
        if (simple_store && cacheable &&
            (store_tail_count != 0u || store_has_control)) {
          links.after_store =
              dispatch_entry(start_pc + (count + 1u) * 4u, true);
        }
      }

      V4NativeFn entry = nullptr;
      if (split_control) {
        entry = compile_v4_budget_branch(
            arena, control, branch_pc, block->code_size);
      } else if (simple_exception) {
        entry = compile_v4_exception(
            arena, exception_inst, start_pc, block->code_size);
      } else if (simple_cop0) {
        entry = compile_v4_cop0(
            arena, cop0, start_pc, links, block->code_size);
      } else if (simple_muldiv) {
        entry = compile_v4_muldiv(
            arena, muldiv, start_pc, links, block->code_size);
      } else if (simple_hilo) {
        entry = compile_v4_hilo(
            arena, hilo, start_pc, links, block->code_size);
      } else if (simple_overflow_alu) {
        entry = compile_v4_overflow_alu(
            arena, overflow_alu, start_pc, links, block->code_size);
      } else if (guarded_control) {
        entry = compile_v4_guarded_delay_branch(
            arena, control, guarded_control_delay, branch_pc, links,
            block->code_size);
      } else if (guarded_store_control) {
        entry = compile_v4_store_delay_branch(
            arena, control, guarded_store_delay, branch_pc, links,
            block->code_size);
      } else if (simple_control) {
        entry = compile_v4_branch(
            arena, decoded, count, control, delay, branch_pc,
            links, block->code_size);
      } else if (simple_load) {
        entry = compile_v4_load(
            arena, decoded, count, load, load_tail, load_tail_count,
            load_has_control ? &load_control : nullptr,
            load_has_control ? &load_delay : nullptr,
            load_branch_pc, start_pc, links, block->code_size);
      } else if (simple_store) {
        entry = compile_v4_store(
            arena, decoded, count, store, store_tail, store_tail_count,
            store_has_control ? &store_control : nullptr,
            store_has_control ? &store_delay : nullptr,
            store_branch_pc, start_pc, cacheable, links,
            block->code_size);
      } else {
        entry = compile_v4_alu(
            arena, decoded, count, start_pc, links, block->code_size);
      }
      if (entry == nullptr) {
        if (!reused_block) {
          --block_count;
        }
        ++stats.native_compile_failures;
        return nullptr;
      }
      block->fn = entry;

      // Compile a scheduler-tail fragment. Straight-line work uses a one-op
      // native prefix. A branch at the start of the block may also execute
      // natively by materializing the pending delay state, but only from a
      // fresh dispatcher entry so an earlier chain cannot cross a System
      // scheduling boundary.
      V4LinkTargets budget_links{};
      u32 budget_code_size = 0u;
      if (split_control) {
        block->budget_fn = entry;
        block->budget_requires_empty_chain = true;
      } else if (count != 0u) {
        block->budget_fn = compile_v4_alu(
            arena, decoded, 1u, start_pc, budget_links, budget_code_size);
      } else if (simple_exception) {
        block->budget_fn = compile_v4_exception(
            arena, exception_inst, start_pc, budget_code_size);
      } else if (simple_cop0) {
        block->budget_fn = compile_v4_cop0(
            arena, cop0, start_pc, budget_links, budget_code_size);
      } else if (simple_muldiv) {
        block->budget_fn = compile_v4_muldiv(
            arena, muldiv, start_pc, budget_links, budget_code_size);
      } else if (simple_hilo) {
        block->budget_fn = compile_v4_hilo(
            arena, hilo, start_pc, budget_links, budget_code_size);
      } else if (simple_overflow_alu) {
        block->budget_fn = compile_v4_overflow_alu(
            arena, overflow_alu, start_pc, budget_links, budget_code_size);
      } else if (simple_control || guarded_control || guarded_store_control) {
        block->budget_fn = compile_v4_budget_branch(
            arena, control, branch_pc, budget_code_size);
        block->budget_requires_empty_chain = block->budget_fn != nullptr;
      } else if (simple_load) {
        block->budget_fn = compile_v4_load(
            arena, decoded, 0u, load, load_tail, 0u, nullptr, nullptr,
            start_pc, start_pc, budget_links, budget_code_size);
      } else if (simple_store) {
        block->budget_fn = compile_v4_store(
            arena, decoded, 0u, store, store_tail, 0u, nullptr, nullptr,
            start_pc, start_pc, cacheable, budget_links, budget_code_size);
      }
      block->instruction_count =
          split_control
              ? 1u
              : ((simple_overflow_alu || simple_hilo || simple_muldiv ||
                  simple_cop0 || simple_exception)
                     ? 1u
                     : ((simple_control || guarded_control ||
                         guarded_store_control)
                            ? count + 2u
                            : (simple_load
                                   ? count + load_tail_count +
                                         (load_has_control ? 3u : 1u)
                                   : (simple_store
                                          ? count + store_tail_count +
                                                (store_has_control ? 3u : 1u)
                                          : count))));
      block->max_cycles =
          split_control
              ? 2u
              : ((simple_overflow_alu || simple_hilo || simple_muldiv ||
                  simple_cop0 || simple_exception)
                     ? 40u
                     : ((simple_control || guarded_control ||
                         guarded_store_control)
                            ? (guarded_store_control ? 5u : count + 3u)
                            : (simple_load
                                   ? count + load_tail_count +
                                         (load_has_control ? 9u : 6u)
                                   : (simple_store
                                          ? count + store_tail_count +
                                                (store_has_control ? 6u : 3u)
                                          : count))));
      block->has_control =
          split_control || simple_control || guarded_control ||
          guarded_store_control || load_has_control || store_has_control;
      block->has_memory =
          simple_load || simple_store || guarded_store_control;
    } catch (...) {
      if (!reused_block) {
        --block_count;
      }
      ++stats.native_compile_failures;
      return nullptr;
    }

    const u32 translated_count =
        split_control
            ? 1u
            : ((simple_overflow_alu || simple_hilo || simple_muldiv ||
                simple_cop0 || simple_exception)
                   ? 1u
                   : ((simple_control || guarded_control ||
                       guarded_store_control)
                          ? count + 2u
                          : (simple_load
                                 ? count + load_tail_count +
                                       (load_has_control ? 3u : 1u)
                                 : (simple_store
                                        ? count + store_tail_count +
                                              (store_has_control ? 3u : 1u)
                                        : count))));
    for (u32 i = 0; i < translated_count; ++i) {
      u32 translated_bits = 0u;
      if (!read_visible(start_pc + i * 4u, translated_bits)) {
        translated_bits = cpu.read_instruction_for_backend(start_pc + i * 4u);
      }
      block->guest_bits[i] = translated_bits;
      const u32 code_phys =
          v4_normalize_code_phys(psx::mask_address(start_pc + i * 4u));
      code_pages.mark_address(code_phys);
      code_lines.mark_address(code_phys);
    }
    install(start_pc, block);

    ++stats.native_compile_successes;
    ++stats.native_blocks_compiled;
    ++stats.native_compiled_block_size_histogram[block->instruction_count];
    if (split_control || simple_control || guarded_control ||
        guarded_store_control) {
      ++stats.native_branch_tail_blocks_compiled;
      if (guarded_store_control) {
        ++stats.native_memory_blocks_compiled;
      }
    } else if (simple_overflow_alu || simple_hilo || simple_muldiv || simple_cop0 || simple_exception) {
      ++stats.native_alu_blocks_compiled;
    } else if (simple_load || simple_store) {
      ++stats.native_memory_blocks_compiled;
      if (load_has_control || store_has_control) {
        ++stats.native_branch_tail_blocks_compiled;
      }
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

CpuRecompilerBackend::CpuRecompilerBackend(Cpu &cpu)
    : cpu_(cpu), impl_(std::make_unique<Impl>()) {
  stats_.available = true;
  stats_.native_available = impl_->native_available();
}

CpuRecompilerBackend::~CpuRecompilerBackend() = default;

CpuRunSliceResult CpuRecompilerBackend::run_slice(u32 max_cycles,
                                             u32 max_instructions) {
  CpuRunSliceResult result{};
  if (max_cycles == 0u || max_instructions == 0u) {
    return result;
  }

  stats_.active = true;
  stats_.native_available = impl_->native_available();
  ++stats_.recompiler_frame_run_slice_calls;

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

  const auto run_helper = [&](u32 instruction, V4HelperReason reason) {
    ++stats_.recompiler_frame_helper_reasons[static_cast<size_t>(reason)];
    V4HelperFn fn = impl_->helper_for(instruction);
    if (fn == nullptr) {
      impl_->reset_translations();
      ++stats_.flushes;
      ++stats_.recompiler_frame_flushes;
      fn = impl_->helper_for(instruction);
    }
    const bool profile = impl_->helper_profile_enabled;
    const u32 primary = (instruction >> 26) & 0x3Fu;
    if (profile) {
      const size_t reason_index = static_cast<size_t>(reason);
      ++stats_.jit_v4_helper_reasons[reason_index];
      ++stats_.jit_v4_helper_primary_counts[primary];
      ++stats_.jit_v4_helper_primary_by_reason[reason_index][primary];
      if (primary == 0u) {
        const u32 special = instruction & 0x3Fu;
        ++stats_.jit_v4_helper_special_counts[special];
        ++stats_.jit_v4_helper_special_by_reason[reason_index][special];
      }
    }
    const bool sample =
        profile && (stats_.jit_v4_helper_instructions & 511u) == 0u;
    const auto sample_start = sample ? std::chrono::steady_clock::now()
                                     : std::chrono::steady_clock::time_point{};
    const u32 consumed = fn != nullptr
                             ? fn(&cpu_)
                             : Cpu::compiled_opcode_fn(instruction)(
                                   &cpu_, instruction);
    if (sample) {
      const auto elapsed = std::chrono::steady_clock::now() - sample_start;
      ++stats_.jit_v4_helper_primary_samples[primary];
      stats_.jit_v4_helper_primary_sample_ns[primary] +=
          static_cast<u64>(std::chrono::duration_cast<
                               std::chrono::nanoseconds>(elapsed)
                               .count());
    }
    result.cycles += consumed;
    ++result.instructions;
    ++stats_.native_instructions;
    ++stats_.jit_v4_helper_instructions;
    ++stats_.recompiler_frame_helper_instructions;
    stats_.native_cycles += consumed;
    ++stats_.optimized_instructions;
    stats_.executed_cycles += consumed;
  };

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
    const bool pending_branch_delay =
        cpu_.pending_delay_slot_ || cpu_.pending_branch_taken_ ||
        cpu_.pending_branch_pc_ != 0u;
    if ((cpu_.pc_ & 3u) != 0u) {
      ++stats_.native_reject_pc_state;
      unsafe_state = true;
    } else if (!pending_branch_delay && cpu_.next_pc_ != cpu_.pc_ + 4u) {
      ++stats_.native_reject_pc_state;
      unsafe_state = true;
    }
    if (cpu_.next_load_.reg != 0u) {
      ++stats_.native_reject_load_delay_state;
      unsafe_state = true;
    }

    // Match Cpu::step()'s hardware IRQ line synchronization before deciding
    // whether native execution may cross the next instruction boundary.
    if (cpu_.sys_->irq_pending()) {
      cpu_.cop0_cause_ |= (1u << 10);
    } else {
      cpu_.cop0_cause_ &= ~(1u << 10);
    }
    const bool irq_entry = !cpu_.pending_delay_slot_ && cpu_.check_irq();
    if (irq_entry) {
      ++stats_.native_reject_irq_state;
      run_helper(0u, V4HelperReason::Irq);
      continue;
    }

    if ((cpu_.pc_ & 3u) != 0u) {
      run_helper(0u, V4HelperReason::UnalignedPc);
      continue;
    }

    const bool cacheable = cpu_.instruction_cacheable(cpu_.pc_);
    u32 icache_generation =
        cacheable ? cpu_.instruction_cache_generation_for_backend(cpu_.pc_) : 0u;

    // First ask only whether a translation exists. Existing cached blocks enter
    // resident x64 immediately; generation/tag/refill validation happens there.
    V4Block *block = impl_->lookup_candidate(cpu_.pc_, cacheable);

    V4NativeFn pending_delay_fn = nullptr;
    if (pending_branch_delay && !unsafe_state && cpu_.pending_delay_slot_ &&
        cpu_.pending_branch_pc_ != 0u) {
      u32 delay_instruction = 0u;
      bool delay_visible =
          cpu_.read_visible_instruction_for_backend(cpu_.pc_,
                                                    delay_instruction);
      if (!delay_visible && cacheable) {
        // The branch already executed natively. Fetching its not-yet-visible
        // delay-slot line here is an architectural cold fetch, not a semantic
        // fallback or validation probe.
        if (cpu_.prepare_instruction_cache_line_for_backend(cpu_.pc_)) {
          ++stats_.recompiler_frame_icache_refills;
          constexpr u32 kRefillCycles = 4u;
          cpu_.cycles_ += kRefillCycles;
          result.cycles += kRefillCycles;
          stats_.executed_cycles += kRefillCycles;
        }
        delay_visible =
            cpu_.read_visible_instruction_for_backend(cpu_.pc_,
                                                      delay_instruction);
      }
      if (delay_visible) {
        pending_delay_fn = impl_->pending_delay_alu_for(delay_instruction);
      }
    }
    if (pending_branch_delay && pending_delay_fn == nullptr) {
      ++stats_.native_reject_branch_delay_state;
      unsafe_state = true;
    }
    if (unsafe_state) {
      ++stats_.native_reject_unsafe_state;
    }

    if (unsafe_state) {
      u32 instruction = 0u;
      if (!cpu_.read_visible_instruction_for_backend(cpu_.pc_,
                                                     instruction)) {
        instruction = cpu_.read_instruction_for_backend(cpu_.pc_);
      }
      run_helper(instruction, V4HelperReason::UnsafeState);
      continue;
    }

    if (block != nullptr) {
      ++stats_.cache_hits;
    } else {
      ++stats_.cache_misses;
      ++stats_.recompiler_frame_cache_misses;

      if (cacheable) {
        // Compilation needs an architectural guest-visible snapshot. This is
        // the cold compile path only; hot validation stays entirely resident.
        if (cpu_.prepare_instruction_cache_line_for_backend(cpu_.pc_)) {
          ++stats_.recompiler_frame_icache_refills;
          constexpr u32 kRefillCycles = 4u;
          cpu_.cycles_ += kRefillCycles;
          result.cycles += kRefillCycles;
          stats_.executed_cycles += kRefillCycles;
        }
        icache_generation =
            cpu_.instruction_cache_generation_for_backend(cpu_.pc_);
      }

      if (impl_->crossline_waiting_for_refill(
              cpu_, cpu_.pc_, cacheable)) {
        u32 instruction = 0u;
        (void)cpu_.read_visible_instruction_for_backend(cpu_.pc_,
                                                        instruction);
        run_helper(instruction, V4HelperReason::Opcode);
        continue;
      }

      const auto compile_start = std::chrono::steady_clock::now();
      block = impl_->compile_block(cpu_, stats_, cpu_.pc_, cacheable,
                                   icache_generation);
      const auto compile_elapsed =
          std::chrono::steady_clock::now() - compile_start;
      const u64 compile_ns =
          static_cast<u64>(std::chrono::duration_cast<
                               std::chrono::nanoseconds>(compile_elapsed)
                               .count());
      stats_.recompiler_frame_compile_ns += compile_ns;
      stats_.recompiler_frame_compile_max_ns =
          std::max(stats_.recompiler_frame_compile_max_ns, compile_ns);
      ++stats_.recompiler_frame_compile_blocks;
      if (block == nullptr) {
        ++stats_.recompiler_frame_compile_failures;
      }

      if (block == nullptr) {
        // A full arena/metadata slab is recycled in bulk; no per-block
        // executable allocations or frees are needed.
        impl_->reset_translations();
        stats_.native_blocks = 0u;
        stats_.block_count = 0u;
        stats_.interpreter_only_blocks = 0u;
        ++stats_.flushes;
        ++stats_.recompiler_frame_flushes;
        const auto retry_compile_start = std::chrono::steady_clock::now();
        block = impl_->compile_block(cpu_, stats_, cpu_.pc_, cacheable,
                                    icache_generation);
        const auto retry_compile_elapsed =
            std::chrono::steady_clock::now() - retry_compile_start;
        const u64 retry_compile_ns =
            static_cast<u64>(std::chrono::duration_cast<
                                 std::chrono::nanoseconds>(retry_compile_elapsed)
                                 .count());
        stats_.recompiler_frame_compile_ns += retry_compile_ns;
        stats_.recompiler_frame_compile_max_ns =
            std::max(stats_.recompiler_frame_compile_max_ns, retry_compile_ns);
        ++stats_.recompiler_frame_compile_blocks;
        if (block == nullptr) {
          ++stats_.recompiler_frame_compile_failures;
        }
      }
    }

    if (block != nullptr && block->helper_fn != nullptr) {
      run_helper(block->guest_bits[0], V4HelperReason::Opcode);
      continue;
    }
    if (block == nullptr || block->fn == nullptr) {
      ++stats_.native_reject_unsupported_instruction;
      if (block != nullptr && block->interpreter_only) {
        switch (block->reject_kind) {
        case V4RejectKind::Branch: ++stats_.native_reject_branch; break;
        case V4RejectKind::Memory: ++stats_.native_reject_memory; break;
        case V4RejectKind::Cop0: ++stats_.native_reject_cop0; break;
        case V4RejectKind::Cop2: ++stats_.native_reject_cop2; break;
        case V4RejectKind::Exception:
          ++stats_.native_reject_exception_unknown;
          break;
        case V4RejectKind::Other: break;
        }
      }
      u32 instruction = 0u;
      if (!cpu_.read_visible_instruction_for_backend(cpu_.pc_,
                                                     instruction)) {
        instruction = cpu_.read_instruction_for_backend(cpu_.pc_);
      }
      run_helper(instruction, V4HelperReason::CompileFailure);
      continue;
    }

    const u32 remaining_cycles = max_cycles - result.cycles;
    const u32 remaining_instructions = max_instructions - result.instructions;

    const u32 start_pc = cpu_.pc_;
    V4NativeState &native = impl_->native_state;
    if (!impl_->native_state_bound) {
      native.gpr = cpu_.gpr_;
      native.cpu = &cpu_;
      native.system = cpu_.sys_;
      native.dispatch_top = impl_->dispatch_top.get();
      native.cop0_regs = cpu_.cop0_regs_;
      native.icache_generations = cpu_.icache_generation_.data();
      native.icache_tags = &cpu_.icache_[0].tag;
      native.icache_words = cpu_.icache_[0].words.data();
      native.icache_valid = &cpu_.icache_[0].valid;
      native.icache_line_stride = sizeof(cpu_.icache_[0]);
      native.code_page_generations = impl_->page_generations.data();
      native.code_page_bits = impl_->code_pages.data();
      native.code_line_bits = impl_->code_lines.data();
      native.block_return = impl_->resident_block_return;
      native.main_ram = cpu_.sys_->jit_main_ram_data_mut();
      native.scratchpad = cpu_.sys_->jit_scratchpad_data_mut();
      impl_->native_state_bound = true;
    }

    // Keep the execution context resident across dispatcher entries. Only
    // architectural values, budgets and per-entry counters need refreshing.
    native.mapped_main_ram_size = cpu_.sys_->jit_mapped_main_ram_size();
    native.memory_fastpath_allowed =
        (!g_trace_ram && !g_trace_bus && !g_ram_watch_diagnostics) ? 1u : 0u;
    native.block_bail = 0u;
    native.memory_entries = 0u;
    native.store_entries = 0u;
    native.store_phys = 0u;
    native.cop0_jumpdest = cpu_.cop0_jumpdest_;
    native.cop0_badvaddr = cpu_.cop0_badvaddr_;
    native.cop0_sr = cpu_.cop0_sr_;
    native.cop0_cause = cpu_.cop0_cause_;
    native.cop0_epc = cpu_.cop0_epc_;
    native.hi = cpu_.hi_;
    native.lo = cpu_.lo_;
    native.muldiv_result_ready_cycle = cpu_.muldiv_result_ready_cycle_;
    native.cpu_cycle_base = cpu_.cycles_;
    native.cache_epoch = impl_->cache_epoch;
    native.pc = start_pc;
    native.next_pc =
        pending_delay_fn != nullptr ? cpu_.next_pc_ : start_pc + 4u;
    native.last_pc = 0u;
    native.last_in_delay_slot = 0u;
    native.active_branch_pc = 0u;
    native.pending_delay_slot = pending_delay_fn != nullptr ? 1u : 0u;
    native.pending_branch_taken =
        pending_delay_fn != nullptr && cpu_.pending_branch_taken_ ? 1u : 0u;
    native.pending_branch_pc =
        pending_delay_fn != nullptr ? cpu_.pending_branch_pc_ : 0u;
    native.pending_delay_fn = reinterpret_cast<void *>(pending_delay_fn);
    native.scheduler_yield = 0u;
    native.pending_load_reg = cpu_.load_.reg;
    native.pending_load_value = cpu_.load_.value;
    native.exception_raised = 0u;
    native.exception_return_sr = 0u;
    native.exception_return_bd = 0u;
    native.cycles = 0u;
    native.instructions = 0u;
    native.cycle_budget = remaining_cycles;
    native.instruction_budget = remaining_instructions;
    native.block_entries = 0u;
    native.direct_links = 0u;
    native.missing_exits = 0u;
    native.epoch_exits = 0u;
    native.memory_exits = 0u;
    native.generation_exits = 0u;
    native.budget_exits = 0u;
    native.bail_exits = 0u;
    native.icache_refills = 0u;
    native.revalidate_attempts = 0u;
    native.revalidate_successes = 0u;
    impl_->resident_dispatch(&native);
    ++stats_.native_chain_invocations;
    ++stats_.recompiler_frame_native_dispatches;
    stats_.native_direct_link_transitions += native.direct_links;
    stats_.recompiler_frame_direct_links += native.direct_links;
    stats_.native_dispatch_missing_exits += native.missing_exits;
    stats_.native_dispatch_epoch_exits += native.epoch_exits;
    stats_.native_dispatch_memory_exits += native.memory_exits;
    stats_.native_dispatch_generation_exits += native.generation_exits;
    stats_.native_dispatch_budget_exits += native.budget_exits;
    stats_.native_dispatch_bail_exits += native.bail_exits;
    stats_.recompiler_frame_dispatch_missing_exits += native.missing_exits;
    stats_.recompiler_frame_dispatch_epoch_exits += native.epoch_exits;
    stats_.recompiler_frame_dispatch_memory_exits += native.memory_exits;
    stats_.recompiler_frame_dispatch_generation_exits += native.generation_exits;
    stats_.recompiler_frame_dispatch_budget_exits += native.budget_exits;
    stats_.recompiler_frame_dispatch_bail_exits += native.bail_exits;
    stats_.recompiler_frame_icache_refills += native.icache_refills;
    stats_.recompiler_frame_revalidate_attempts += native.revalidate_attempts;
    stats_.recompiler_frame_revalidate_successes += native.revalidate_successes;
    stats_.cache_hits += native.revalidate_successes;

    // Resident I-cache revalidation may refill a line before discovering that
    // a translation is genuinely stale. Commit those cycles even if no guest
    // instruction retired.
    constexpr u32 kIcacheRefillCycles = 4u;
    const u32 native_refill_cycles =
        native.icache_refills * kIcacheRefillCycles;
    cpu_.cycles_ += native.cycles;
    result.cycles += native.cycles;
    stats_.native_cycles +=
        native.cycles >= native_refill_cycles
            ? native.cycles - native_refill_cycles
            : 0u;
    stats_.executed_cycles += native.cycles;

    if (native.instructions == 0u) {
      ++stats_.native_reject_budget;
      ++stats_.budget_exits;
      run_helper(block->guest_bits[0], V4HelperReason::Budget);
      continue;
    }

    cpu_.current_pc_ = native.last_pc;
    cpu_.pc_ = native.pc;
    cpu_.next_pc_ = native.pending_delay_slot != 0u
                        ? native.next_pc
                        : native.pc + 4u;
    cpu_.in_delay_slot_ = native.last_in_delay_slot != 0u;
    cpu_.active_branch_pc_ = native.active_branch_pc;
    cpu_.pending_delay_slot_ = native.pending_delay_slot != 0u;
    cpu_.pending_branch_taken_ = native.pending_branch_taken != 0u;
    cpu_.pending_branch_pc_ = native.pending_branch_pc;
    cpu_.load_ = {native.pending_load_reg, native.pending_load_value};
    cpu_.next_load_ = {0u, 0u};
    cpu_.exception_raised_ = native.exception_raised != 0u;
    cpu_.cycle_penalty_ = 0u;
    cpu_.executing_step_ = false;
    cpu_.gpr_[0] = 0u;
    cpu_.hi_ = native.hi;
    cpu_.lo_ = native.lo;
    cpu_.muldiv_result_ready_cycle_ = native.muldiv_result_ready_cycle;
    cpu_.cop0_jumpdest_ = native.cop0_jumpdest;
    cpu_.cop0_badvaddr_ = native.cop0_badvaddr;
    cpu_.cop0_sr_ = native.cop0_sr;
    cpu_.cop0_cause_ = native.cop0_cause;
    cpu_.cop0_epc_ = native.cop0_epc;
    if (native.exception_raised != 0u) {
      std::memcpy(cpu_.exception_return_regs_, cpu_.gpr_,
                  sizeof(cpu_.exception_return_regs_));
      cpu_.exception_return_hi_ = native.hi;
      cpu_.exception_return_lo_ = native.lo;
      cpu_.exception_return_epc_ = native.cop0_epc;
      cpu_.exception_return_sr_ = native.exception_return_sr;
      cpu_.exception_return_bd_ = native.exception_return_bd != 0u;
      cpu_.exception_return_valid_ = true;
      cpu_.gte_result_ready_cycle_ = cpu_.cycles_;
      cpu_.gte_input_ready_cycle_ = cpu_.cycles_;
    }

    result.instructions += native.instructions;
    stats_.native_block_entries += native.block_entries;
    stats_.native_memory_fastpath_loads += native.memory_entries;
    stats_.native_memory_fastpath_stores += native.store_entries;
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
    stats_.optimized_instructions += native.instructions;
  }
  return result;
#endif
}

void CpuRecompilerBackend::invalidate_range(u32 phys_or_normalized_addr,
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
    const u32 phys =
        v4_normalize_code_phys(psx::mask_address(address));
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
  ++stats_.recompiler_frame_invalidations;
#else
  (void)phys_or_normalized_addr;
#endif
}

void CpuRecompilerBackend::begin_frame(u32 frame_index) {
  current_frame_ = frame_index;
  stats_.active =
      effective_cpu_execution_mode() == CpuExecutionMode::Recompiler;

  stats_.recompiler_frame_compile_ns = 0;
  stats_.recompiler_frame_compile_max_ns = 0;
  stats_.recompiler_frame_compile_blocks = 0;
  stats_.recompiler_frame_compile_failures = 0;
  stats_.recompiler_frame_revalidate_ns = 0;
  stats_.recompiler_frame_revalidate_attempts = 0;
  stats_.recompiler_frame_revalidate_successes = 0;
  stats_.recompiler_frame_cache_misses = 0;
  stats_.recompiler_frame_icache_refills = 0;
  stats_.recompiler_frame_helper_instructions = 0;
  stats_.recompiler_frame_helper_reasons.fill(0);
  stats_.recompiler_frame_run_slice_calls = 0;
  stats_.recompiler_frame_native_dispatches = 0;
  stats_.recompiler_frame_direct_links = 0;
  stats_.recompiler_frame_invalidations = 0;
  stats_.recompiler_frame_flushes = 0;
  stats_.recompiler_frame_dispatch_missing_exits = 0;
  stats_.recompiler_frame_dispatch_epoch_exits = 0;
  stats_.recompiler_frame_dispatch_memory_exits = 0;
  stats_.recompiler_frame_dispatch_generation_exits = 0;
  stats_.recompiler_frame_dispatch_budget_exits = 0;
  stats_.recompiler_frame_dispatch_bail_exits = 0;
}

void CpuRecompilerBackend::flush() {
  impl_->reset_translations();
  ++stats_.flushes;
  stats_.native_blocks = 0u;
  stats_.block_count = 0u;
  stats_.interpreter_only_blocks = 0u;
  stats_.native_code_bytes = 0u;
  stats_.code_bytes = 0u;
}

CpuBackendStats CpuRecompilerBackend::stats() const {
  CpuBackendStats out = stats_;
  out.available = true;
  out.active =
      effective_cpu_execution_mode() == CpuExecutionMode::Recompiler;
  out.native_available = impl_->native_available();
  return out;
}
