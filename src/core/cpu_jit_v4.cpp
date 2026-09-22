#include "cpu_jit_v4.h"

#include "jit_code_page_bitmap.h"
#include "system.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
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
constexpr size_t kV4DispatchTopCount = size_t{1} << 20u;
constexpr size_t kV4DispatchEntriesPerPage = size_t{1} << 10u;

enum class V4AluOp : u8 {
  Nop,
  Sll,
  Srl,
  Sra,
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

struct V4NativeState {
  u32 *gpr = nullptr;
  u32 pc = 0;
  u32 cycles = 0;
  u32 instructions = 0;
};

using V4NativeFn = void (*)(V4NativeState *);

class V4CodeArena {
public:
  V4CodeArena() {
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
  }

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

  bool available() const { return base_ != nullptr; }
  size_t bytes_used() const { return used_; }

  void reset() { used_ = 0u; }

  void *copy_code(const void *source, size_t size) {
    if (base_ == nullptr || source == nullptr || size == 0u) {
      return nullptr;
    }
    const size_t offset = (used_ + 15u) & ~size_t{15u};
    if (offset > kV4CodeArenaBytes || size > kV4CodeArenaBytes - offset) {
      return nullptr;
    }
    u8 *dst = base_ + offset;
    std::memcpy(dst, source, size);
    used_ = offset + size;

    // x86/x64 has coherent I/D caches; no FlushInstructionCache syscall is
    // needed for freshly written code in this RWX bring-up arena.
    return dst;
  }

private:
  u8 *base_ = nullptr;
  size_t used_ = 0u;
};

struct V4Block {
  u32 start_pc = 0;
  u32 instruction_count = 0;
  u32 code_size = 0;
  V4NativeFn fn = nullptr;
  bool interpreter_only = false;
};

struct V4DispatchEntry {
  V4Block *block = nullptr;
  u32 cache_epoch = 0;
};

struct V4DispatchPage {
  std::array<V4DispatchEntry, kV4DispatchEntriesPerPage> entries{};
};

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

std::unique_ptr<Xbyak::CodeGenerator> compile_v4_alu(
    const std::array<V4DecodedInstruction, kV4MaxBlockInstructions> &decoded,
    u32 count) {
  using namespace Xbyak;
  auto code = std::make_unique<CodeGenerator>(4096);
#if defined(_WIN32)
  code->mov(code->r11, code->rcx);
#else
  code->mov(code->r11, code->rdi);
#endif
  code->mov(code->r10, code->ptr[
      code->r11 + static_cast<int>(offsetof(V4NativeState, gpr))]);

  for (u32 i = 0; i < count; ++i) {
    const V4DecodedInstruction &inst = decoded[i];
    switch (inst.op) {
    case V4AluOp::Nop:
      break;

    case V4AluOp::Sll:
      emit_read_guest(*code, code->eax, inst.rt);
      code->shl(code->eax, inst.shamt);
      emit_write_guest(*code, inst.rd, code->eax);
      break;
    case V4AluOp::Srl:
      emit_read_guest(*code, code->eax, inst.rt);
      code->shr(code->eax, inst.shamt);
      emit_write_guest(*code, inst.rd, code->eax);
      break;
    case V4AluOp::Sra:
      emit_read_guest(*code, code->eax, inst.rt);
      code->sar(code->eax, inst.shamt);
      emit_write_guest(*code, inst.rd, code->eax);
      break;

    case V4AluOp::Addu:
      emit_read_guest(*code, code->eax, inst.rs);
      emit_read_guest(*code, code->ecx, inst.rt);
      code->add(code->eax, code->ecx);
      emit_write_guest(*code, inst.rd, code->eax);
      break;
    case V4AluOp::Subu:
      emit_read_guest(*code, code->eax, inst.rs);
      emit_read_guest(*code, code->ecx, inst.rt);
      code->sub(code->eax, code->ecx);
      emit_write_guest(*code, inst.rd, code->eax);
      break;
    case V4AluOp::And:
      emit_read_guest(*code, code->eax, inst.rs);
      emit_read_guest(*code, code->ecx, inst.rt);
      code->and_(code->eax, code->ecx);
      emit_write_guest(*code, inst.rd, code->eax);
      break;
    case V4AluOp::Or:
      emit_read_guest(*code, code->eax, inst.rs);
      emit_read_guest(*code, code->ecx, inst.rt);
      code->or_(code->eax, code->ecx);
      emit_write_guest(*code, inst.rd, code->eax);
      break;
    case V4AluOp::Xor:
      emit_read_guest(*code, code->eax, inst.rs);
      emit_read_guest(*code, code->ecx, inst.rt);
      code->xor_(code->eax, code->ecx);
      emit_write_guest(*code, inst.rd, code->eax);
      break;
    case V4AluOp::Nor:
      emit_read_guest(*code, code->eax, inst.rs);
      emit_read_guest(*code, code->ecx, inst.rt);
      code->or_(code->eax, code->ecx);
      code->not_(code->eax);
      emit_write_guest(*code, inst.rd, code->eax);
      break;
    case V4AluOp::Slt:
      emit_read_guest(*code, code->eax, inst.rs);
      emit_read_guest(*code, code->ecx, inst.rt);
      code->cmp(code->eax, code->ecx);
      code->setl(code->al);
      code->movzx(code->eax, code->al);
      emit_write_guest(*code, inst.rd, code->eax);
      break;
    case V4AluOp::Sltu:
      emit_read_guest(*code, code->eax, inst.rs);
      emit_read_guest(*code, code->ecx, inst.rt);
      code->cmp(code->eax, code->ecx);
      code->setb(code->al);
      code->movzx(code->eax, code->al);
      emit_write_guest(*code, inst.rd, code->eax);
      break;

    case V4AluOp::Addiu:
      emit_read_guest(*code, code->eax, inst.rs);
      code->add(code->eax, static_cast<u32>(inst.simm));
      emit_write_guest(*code, inst.rt, code->eax);
      break;
    case V4AluOp::Slti:
      emit_read_guest(*code, code->eax, inst.rs);
      code->cmp(code->eax, static_cast<u32>(inst.simm));
      code->setl(code->al);
      code->movzx(code->eax, code->al);
      emit_write_guest(*code, inst.rt, code->eax);
      break;
    case V4AluOp::Sltiu:
      emit_read_guest(*code, code->eax, inst.rs);
      code->cmp(code->eax, static_cast<u32>(inst.simm));
      code->setb(code->al);
      code->movzx(code->eax, code->al);
      emit_write_guest(*code, inst.rt, code->eax);
      break;
    case V4AluOp::Andi:
      emit_read_guest(*code, code->eax, inst.rs);
      code->and_(code->eax, static_cast<u32>(inst.imm));
      emit_write_guest(*code, inst.rt, code->eax);
      break;
    case V4AluOp::Ori:
      emit_read_guest(*code, code->eax, inst.rs);
      code->or_(code->eax, static_cast<u32>(inst.imm));
      emit_write_guest(*code, inst.rt, code->eax);
      break;
    case V4AluOp::Xori:
      emit_read_guest(*code, code->eax, inst.rs);
      code->xor_(code->eax, static_cast<u32>(inst.imm));
      emit_write_guest(*code, inst.rt, code->eax);
      break;
    case V4AluOp::Lui:
      code->mov(code->eax, static_cast<u32>(inst.imm) << 16u);
      emit_write_guest(*code, inst.rt, code->eax);
      break;
    }
  }

  code->add(code->dword[
      code->r11 + static_cast<int>(offsetof(V4NativeState, pc))],
      count * 4u);
  code->add(code->dword[
      code->r11 + static_cast<int>(offsetof(V4NativeState, cycles))], count);
  code->add(code->dword[
      code->r11 + static_cast<int>(offsetof(V4NativeState, instructions))],
      count);
  code->ret();
  code->ready();
  return code;
}

#endif // VIBESTATION_JIT_V4_X64

} // namespace

struct CpuJitV4Backend::Impl {
#if VIBESTATION_JIT_V4_X64
  V4CodeArena arena;
  std::unique_ptr<V4DispatchPage *[]> dispatch_top =
      std::make_unique<V4DispatchPage *[]>(kV4DispatchTopCount);
  std::vector<std::unique_ptr<V4DispatchPage>> dispatch_pages;
  std::unique_ptr<V4Block[]> blocks = std::make_unique<V4Block[]>(kV4MaxBlocks);
  size_t block_count = 0u;
  u32 cache_epoch = 1u;
  JitCodePageBitmap<29u, 12u> code_pages;

  bool native_available() const { return arena.available(); }

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

  V4Block *lookup(u32 pc) {
    V4DispatchEntry *entry = dispatch_entry(pc, false);
    if (entry == nullptr || entry->cache_epoch != cache_epoch) {
      return nullptr;
    }
    return entry->block;
  }

  void install(u32 pc, V4Block *block) {
    V4DispatchEntry *entry = dispatch_entry(pc, true);
    if (entry == nullptr) {
      return;
    }
    entry->block = block;
    entry->cache_epoch = cache_epoch;
  }

  void reset_translations() {
    arena.reset();
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

  V4Block *compile_block(Cpu &cpu, CpuBackendStats &stats, u32 start_pc) {
    V4Block *block = allocate_block();
    if (block == nullptr) {
      return nullptr;
    }

    block->start_pc = start_pc;
    std::array<V4DecodedInstruction, kV4MaxBlockInstructions> decoded{};
    u32 count = 0u;
    for (; count < kV4MaxBlockInstructions; ++count) {
      V4DecodedInstruction inst{};
      const u32 bits = cpu.read_instruction_for_backend(start_pc + count * 4u);
      if (!decode_v4_alu(bits, inst)) {
        break;
      }
      decoded[count] = inst;
    }

    if (count == 0u) {
      block->interpreter_only = true;
      install(start_pc, block);
      ++stats.interpreter_only_blocks;
      stats.block_count = static_cast<u32>(block_count);
      return block;
    }

    ++stats.native_compile_attempts;
    try {
      std::unique_ptr<Xbyak::CodeGenerator> generated =
          compile_v4_alu(decoded, count);
      block->code_size = static_cast<u32>(generated->getSize());
      void *entry = arena.copy_code(generated->getCode(), block->code_size);
      if (entry == nullptr) {
        --block_count;
        ++stats.native_compile_failures;
        return nullptr;
      }
      block->fn = reinterpret_cast<V4NativeFn>(entry);
      block->instruction_count = count;
    } catch (...) {
      --block_count;
      ++stats.native_compile_failures;
      return nullptr;
    }

    for (u32 i = 0; i < count; ++i) {
      code_pages.mark_address(psx::mask_address(start_pc + i * 4u));
    }
    install(start_pc, block);

    ++stats.native_compile_successes;
    ++stats.native_blocks_compiled;
    ++stats.native_alu_blocks_compiled;
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
  if (!impl_->native_available()) {
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
      g_cpu_backend_compare_test_force_interpreter) {
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
    if (cpu_.pending_delay_slot_ || cpu_.in_delay_slot_ ||
        cpu_.pending_branch_taken_ || cpu_.pending_branch_pc_ != 0u ||
        cpu_.load_.reg != 0u || cpu_.next_load_.reg != 0u ||
        cpu_.next_pc_ != cpu_.pc_ + 4u) {
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

    // Guest I-cache visibility is deliberately a later V4 phase. For now,
    // native compilation is restricted to uncached KSEG1 execution, including
    // the BIOS, so raw backend instruction reads cannot observe bytes that the
    // guest I-cache would still hide.
    if (cpu_.instruction_cacheable(cpu_.pc_)) {
      ++stats_.native_reject_icache;
      fallback_one();
      continue;
    }

    V4Block *block = impl_->lookup(cpu_.pc_);
    if (block != nullptr) {
      ++stats_.cache_hits;
    } else {
      ++stats_.cache_misses;
      block = impl_->compile_block(cpu_, stats_, cpu_.pc_);
      if (block == nullptr) {
        // A full arena/metadata slab is recycled in bulk; no per-block
        // executable allocations or frees are needed.
        impl_->reset_translations();
        stats_.native_blocks = 0u;
        stats_.block_count = 0u;
        stats_.interpreter_only_blocks = 0u;
        ++stats_.flushes;
        block = impl_->compile_block(cpu_, stats_, cpu_.pc_);
      }
    }

    if (block == nullptr || block->interpreter_only || block->fn == nullptr) {
      ++stats_.native_reject_unsupported_instruction;
      fallback_one();
      continue;
    }

    const u32 remaining_cycles = max_cycles - result.cycles;
    const u32 remaining_instructions = max_instructions - result.instructions;
    if (block->instruction_count > remaining_cycles ||
        block->instruction_count > remaining_instructions) {
      ++stats_.native_reject_budget;
      ++stats_.budget_exits;
      fallback_one();
      continue;
    }

    const u32 start_pc = cpu_.pc_;
    V4NativeState native{};
    native.gpr = cpu_.gpr_;
    native.pc = start_pc;
    block->fn(&native);

    if (native.instructions != block->instruction_count ||
        native.cycles != block->instruction_count ||
        native.pc != start_pc + block->instruction_count * 4u) {
      ++stats_.pc_mismatch_exits;
      fallback_one();
      continue;
    }

    cpu_.current_pc_ =
        start_pc + (block->instruction_count - 1u) * 4u;
    cpu_.pc_ = native.pc;
    cpu_.next_pc_ = native.pc + 4u;
    cpu_.in_delay_slot_ = false;
    cpu_.active_branch_pc_ = 0u;
    cpu_.pending_delay_slot_ = false;
    cpu_.pending_branch_taken_ = false;
    cpu_.pending_branch_pc_ = 0u;
    cpu_.exception_raised_ = false;
    cpu_.cycle_penalty_ = 0u;
    cpu_.executing_step_ = false;
    cpu_.gpr_[0] = 0u;
    cpu_.cycles_ += native.cycles;

    result.cycles += native.cycles;
    result.instructions += native.instructions;
    ++stats_.native_block_entries;
    ++stats_.native_alu_block_entries;
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
  u32 remaining = size_bytes;
  u32 address = phys_or_normalized_addr;
  bool touches_code = false;
  while (remaining != 0u) {
    const u32 phys = psx::mask_address(address);
    if (impl_->code_pages.test_address(phys)) {
      touches_code = true;
      break;
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

  // Phase 4 will replace this conservative whole-arena epoch bump with
  // per-physical-page generations. Even this bring-up path is O(touched pages)
  // and never scans all translated blocks.
  impl_->reset_translations();
  ++stats_.invalidations;
  ++stats_.flushes;
  stats_.native_blocks = 0u;
  stats_.block_count = 0u;
  stats_.interpreter_only_blocks = 0u;
  stats_.native_code_bytes = 0u;
  stats_.code_bytes = 0u;
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
