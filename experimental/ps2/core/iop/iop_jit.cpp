#include "core/iop/iop_jit.h"

#include "core/iop/iop_bus.h"
#include "core/iop/iop_cpu.h"
#include "core/iop/iop_ram.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#if defined(_M_X64) || defined(__x86_64__)
#define VIBESTATION_IOP_JIT_X64 1
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#else
#include <sys/mman.h>
#endif
#endif

namespace ps2 {
namespace {

#if defined(VIBESTATION_IOP_JIT_X64)

constexpr std::size_t kCodePageSize = 64u * 1024u;
constexpr std::size_t kMaxCodePages = 64u;

#ifdef _WIN32
constexpr u8 kStateArgumentRegister = 1u; // RCX
constexpr u8 kRamArgumentRegister = 2u;   // RDX
constexpr u8 kGenerationArgumentRegister = 8u; // R8
#else
constexpr u8 kStateArgumentRegister = 7u; // RDI
constexpr u8 kRamArgumentRegister = 6u;   // RSI
constexpr u8 kGenerationArgumentRegister = 2u; // RDX
#endif

struct Emitter {
    std::vector<u8> bytes;

    void emit(u8 value) { bytes.push_back(value); }
    void emit32(u32 value) {
        for (u32 i = 0; i < 4u; ++i) {
            emit(static_cast<u8>(value >> (i * 8u)));
        }
    }

    void memory(u8 opcode, u8 reg, u32 offset) {
        emit(opcode);
        emit(static_cast<u8>(
            0x80u | ((reg & 7u) << 3u) |
            (kStateArgumentRegister & 7u)));
        emit32(offset);
    }

    void load_eax(u32 reg) {
        memory(
            0x8Bu,
            0u,
            static_cast<u32>(
                offsetof(IopCpuState, gpr) + reg * sizeof(u32)));
    }

    void load_edx(u32 reg) {
        memory(
            0x8Bu,
            2u,
            static_cast<u32>(
                offsetof(IopCpuState, gpr) + reg * sizeof(u32)));
    }

    void store_eax(u32 reg) {
        if (reg == 0u) return;
        memory(
            0x89u,
            0u,
            static_cast<u32>(
                offsetof(IopCpuState, gpr) + reg * sizeof(u32)));
    }

    void load_state_eax(u32 offset) {
        memory(0x8Bu, 0u, offset);
    }

    void store_state_eax(u32 offset) {
        memory(0x89u, 0u, offset);
    }

    void store_state_edx(u32 offset) {
        memory(0x89u, 2u, offset);
    }

    void store_state_imm32(u32 offset, u32 value) {
        emit(0xC7u);
        emit(static_cast<u8>(
            0x80u | (kStateArgumentRegister & 7u)));
        emit32(offset);
        emit32(value);
    }

    void store_gpr_imm32(u32 reg, u32 value) {
        if (reg == 0u) return;
        store_state_imm32(
            static_cast<u32>(
                offsetof(IopCpuState, gpr) + reg * sizeof(u32)),
            value);
    }

    std::size_t jcc32(u8 condition) {
        emit(0x0Fu);
        emit(condition);
        const std::size_t displacement = bytes.size();
        emit32(0u);
        return displacement;
    }

    std::size_t jmp32() {
        emit(0xE9u);
        const std::size_t displacement = bytes.size();
        emit32(0u);
        return displacement;
    }

    void patch_rel32(std::size_t displacement, std::size_t target) {
        const s64 rel =
            static_cast<s64>(target) -
            static_cast<s64>(displacement + 4u);
        const u32 encoded =
            static_cast<u32>(static_cast<s32>(rel));
        for (u32 i = 0; i < 4u; ++i) {
            bytes[displacement + i] =
                static_cast<u8>(encoded >> (i * 8u));
        }
    }

    void preserve_ram_base() {
        emit(0x49u);
        emit(0x89u);
        emit(static_cast<u8>(
            0xC0u |
            ((kRamArgumentRegister & 7u) << 3u) |
            3u)); // MOV R11, RAM arg
    }

    void preserve_generation_base() {
        const u8 rex = static_cast<u8>(
            0x49u |
            ((kGenerationArgumentRegister & 8u) ? 0x04u : 0u));
        emit(rex);
        emit(0x89u);
        emit(static_cast<u8>(
            0xC0u |
            ((kGenerationArgumentRegister & 7u) << 3u) |
            2u)); // MOV R10, generation arg
    }
};

void* allocate_code_page() {
#ifdef _WIN32
    return VirtualAlloc(
        nullptr,
        kCodePageSize,
        MEM_COMMIT | MEM_RESERVE,
        PAGE_READWRITE);
#else
    void* page = mmap(
        nullptr,
        kCodePageSize,
        PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS,
        -1,
        0);
    return page == MAP_FAILED ? nullptr : page;
#endif
}

bool protect_code_page(void* page, bool executable) {
#ifdef _WIN32
    DWORD old_protection = 0;
    return VirtualProtect(
               page,
               kCodePageSize,
               executable ? PAGE_EXECUTE_READ : PAGE_READWRITE,
               &old_protection) != 0;
#else
    return mprotect(
               page,
               kCodePageSize,
               executable
                   ? PROT_READ | PROT_EXEC
                   : PROT_READ | PROT_WRITE) == 0;
#endif
}

void release_code_page(void* page) {
#ifdef _WIN32
    VirtualFree(page, 0, MEM_RELEASE);
#else
    munmap(page, kCodePageSize);
#endif
}

void flush_code(void* code, std::size_t size) {
#ifdef _WIN32
    FlushInstructionCache(GetCurrentProcess(), code, size);
#else
    auto* first = static_cast<char*>(code);
    __builtin___clear_cache(first, first + size);
#endif
}

bool emit_body(u32 instruction, Emitter& out) {
    const u32 opcode = instruction >> 26;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;
    const u32 sa = (instruction >> 6) & 31u;
    const u32 immediate = instruction & 0xFFFFu;
    u32 destination = rt;

    if (opcode == 0u) {
        const u32 funct = instruction & 63u;
        destination = rd;
        switch (funct) {
        case 0x00u: // SLL / NOP
        case 0x02u: // SRL
        case 0x03u: // SRA
            if (rs != 0u) return false;
            if (destination != 0u) {
                out.load_eax(rt);
                out.emit(0xC1u);
                out.emit(
                    funct == 0x00u ? 0xE0u :
                    funct == 0x02u ? 0xE8u : 0xF8u);
                out.emit(static_cast<u8>(sa));
            }
            break;
        case 0x04u: // SLLV
        case 0x06u: // SRLV
        case 0x07u: // SRAV
            if (destination != 0u) {
                out.load_edx(rs);
                out.load_eax(rt);
                if constexpr (kStateArgumentRegister == 1u) {
                    out.emit(0x51u); // PUSH RCX
                }
                out.emit(0x89u); out.emit(0xD1u); // MOV ECX,EDX
                out.emit(0xD3u);
                out.emit(
                    funct == 0x04u ? 0xE0u :
                    funct == 0x06u ? 0xE8u : 0xF8u);
                if constexpr (kStateArgumentRegister == 1u) {
                    out.emit(0x59u); // POP RCX
                }
            }
            break;
        case 0x10u: // MFHI
            if (destination != 0u) {
                out.load_state_eax(
                    static_cast<u32>(offsetof(IopCpuState, hi)));
            }
            break;
        case 0x11u: // MTHI
            out.load_eax(rs);
            out.store_state_eax(
                static_cast<u32>(offsetof(IopCpuState, hi)));
            destination = 0u;
            break;
        case 0x12u: // MFLO
            if (destination != 0u) {
                out.load_state_eax(
                    static_cast<u32>(offsetof(IopCpuState, lo)));
            }
            break;
        case 0x13u: // MTLO
            out.load_eax(rs);
            out.store_state_eax(
                static_cast<u32>(offsetof(IopCpuState, lo)));
            destination = 0u;
            break;
        case 0x18u: // MULT
        case 0x19u: // MULTU
            out.load_eax(rs);
            out.load_edx(rt);
            out.emit(0xF7u);
            out.emit(funct == 0x18u ? 0xEAu : 0xE2u);
            out.store_state_eax(
                static_cast<u32>(offsetof(IopCpuState, lo)));
            out.store_state_edx(
                static_cast<u32>(offsetof(IopCpuState, hi)));
            destination = 0u;
            break;
        case 0x21u: // ADDU
        case 0x23u: // SUBU
        case 0x24u: // AND
        case 0x25u: // OR
        case 0x26u: // XOR
        case 0x27u: // NOR
        case 0x2Au: // SLT
        case 0x2Bu: // SLTU
            if (destination != 0u) {
                out.load_eax(rs);
                out.load_edx(rt);
                if (funct == 0x2Au || funct == 0x2Bu) {
                    out.emit(0x39u); out.emit(0xD0u); // CMP EAX,EDX
                    out.emit(0x0Fu);
                    out.emit(funct == 0x2Au ? 0x9Cu : 0x92u);
                    out.emit(0xC0u);
                    out.emit(0x0Fu); out.emit(0xB6u); out.emit(0xC0u);
                } else {
                    out.emit(
                        funct == 0x23u ? 0x29u :
                        funct == 0x24u ? 0x21u :
                        funct == 0x25u || funct == 0x27u ? 0x09u :
                        funct == 0x26u ? 0x31u :
                        0x01u);
                    out.emit(0xD0u);
                    if (funct == 0x27u) {
                        out.emit(0xF7u); out.emit(0xD0u);
                    }
                }
            }
            break;
        default:
            return false;
        }
    } else {
        switch (opcode) {
        case 0x09u: // ADDIU
            if (destination != 0u) {
                out.load_eax(rs);
                out.emit(0x05u);
                out.emit32(static_cast<u32>(
                    static_cast<s32>(
                        static_cast<s16>(immediate))));
            }
            break;
        case 0x0Au: // SLTI
        case 0x0Bu: // SLTIU
            if (destination != 0u) {
                out.load_eax(rs);
                out.emit(0x3Du);
                out.emit32(static_cast<u32>(
                    static_cast<s32>(
                        static_cast<s16>(immediate))));
                out.emit(0x0Fu);
                out.emit(opcode == 0x0Au ? 0x9Cu : 0x92u);
                out.emit(0xC0u);
                out.emit(0x0Fu); out.emit(0xB6u); out.emit(0xC0u);
            }
            break;
        case 0x0Cu: // ANDI
        case 0x0Du: // ORI
        case 0x0Eu: // XORI
            if (destination != 0u) {
                out.load_eax(rs);
                out.emit(
                    opcode == 0x0Cu ? 0x25u :
                    opcode == 0x0Du ? 0x0Du : 0x35u);
                out.emit32(immediate);
            }
            break;
        case 0x0Fu: // LUI
            if (destination != 0u) {
                out.emit(0xB8u);
                out.emit32(immediate << 16u);
            }
            break;
        default:
            return false;
        }
    }

    if (destination != 0u) {
        out.store_eax(destination);
    }
    return true;
}

bool emit_store(
    u32 instruction,
    u32 retired_before,
    u32 code_page,
    Emitter& out) {
    const u32 opcode = instruction >> 26;
    if (opcode != 0x28u && opcode != 0x29u && opcode != 0x2Bu) {
        return false;
    }

    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const s16 immediate =
        static_cast<s16>(instruction & 0xFFFFu);

    out.load_eax(rs);
    out.emit(0x05u);
    out.emit32(static_cast<u32>(static_cast<s32>(immediate)));

    // KSEG0/KSEG1 map through the 29-bit physical window. Other segments
    // retain their address and will fail the direct-RAM guard below.
    out.emit(0x3Du); out.emit32(0x80000000u);
    const std::size_t below_kseg = out.jcc32(0x82u); // JB
    out.emit(0x3Du); out.emit32(0xC0000000u);
    const std::size_t above_kseg = out.jcc32(0x83u); // JAE
    out.emit(0x25u); out.emit32(0x1FFFFFFFu);
    const std::size_t mapped_kseg = out.jmp32();

    const std::size_t physical_label = out.bytes.size();
    out.patch_rel32(below_kseg, physical_label);
    out.patch_rel32(above_kseg, physical_label);
    out.patch_rel32(mapped_kseg, physical_label);

    std::vector<std::size_t> fail_jumps;
    out.emit(0x3Du); out.emit32(0x00800000u);
    fail_jumps.push_back(out.jcc32(0x83u)); // JAE

    // The physical 8 MiB IOP window mirrors 2 MiB of backing RAM.
    out.emit(0x25u); out.emit32(0x001FFFFFu);
    out.emit(0x41u); out.emit(0x89u); out.emit(0xC0u); // MOV R8D,EAX

    // Cache-isolated RAM stores are architecturally swallowed by the current
    // IOP model. Keep that runtime behavior without leaving native execution.
    out.load_state_eax(
        static_cast<u32>(
            offsetof(IopCpuState, cop0) + 12u * sizeof(u32)));
    out.emit(0xA9u); out.emit32(0x00010000u);
    const std::size_t isolated = out.jcc32(0x85u); // JNZ

    out.load_edx(rt);
    if (opcode == 0x28u) {
        out.emit(0x43u); out.emit(0x88u);
        out.emit(0x14u); out.emit(0x03u); // [R11+R8] = DL
    } else if (opcode == 0x29u) {
        out.emit(0x66u); out.emit(0x43u); out.emit(0x89u);
        out.emit(0x14u); out.emit(0x03u); // [R11+R8] = DX
    } else {
        out.emit(0x43u); out.emit(0x89u);
        out.emit(0x14u); out.emit(0x03u); // [R11+R8] = EDX
    }

    // Conservative native write barrier. Page generations are host-only state,
    // so incrementing an untracked data page is harmless and lets direct stores
    // invalidate code without calling back through the C++ bus.
    out.emit(0x41u); out.emit(0xC1u); out.emit(0xE8u); out.emit(0x0Cu);
    out.emit(0x43u); out.emit(0x83u); out.emit(0x04u); out.emit(0x82u);
    out.emit(0x01u);

    if (code_page != 0xFFFFFFFFu) {
        out.emit(0x41u); out.emit(0x81u); out.emit(0xF8u);
        out.emit32(code_page);
        const std::size_t not_code = out.jcc32(0x85u);
        out.emit(0xB8u);
        out.emit32(retired_before + 1u);
        out.emit(0xC3u);
        out.patch_rel32(not_code, out.bytes.size());
    }

    const std::size_t continued = out.jmp32();
    const std::size_t isolated_label = out.bytes.size();
    out.patch_rel32(isolated, isolated_label);
    const std::size_t after_store = out.bytes.size();
    out.patch_rel32(continued, after_store);

    const std::size_t done = out.jmp32();
    const std::size_t fail_label = out.bytes.size();
    out.emit(0xB8u);
    out.emit32(retired_before);
    out.emit(0xC3u);
    const std::size_t end = out.bytes.size();
    for (const std::size_t jump : fail_jumps) {
        out.patch_rel32(jump, fail_label);
    }
    out.patch_rel32(done, end);
    return true;
}

bool emit_branch_and_delay(
    u32 pc,
    u32 branch,
    u32 delay,
    Emitter& out) {
    const std::size_t before = out.bytes.size();
    if (!emit_body(delay, out)) {
        return false;
    }
    // Delay code has to execute after the branch decision, not before it.
    out.bytes.resize(before);

    const u32 opcode = branch >> 26;
    const u32 rs = (branch >> 21) & 31u;
    const u32 rt = (branch >> 16) & 31u;
    const s16 imm = static_cast<s16>(branch & 0xFFFFu);
    const u32 pc_offset = static_cast<u32>(offsetof(IopCpuState, pc));
    const u32 next_pc_offset =
        static_cast<u32>(offsetof(IopCpuState, next_pc));
    const u32 fallthrough = pc + 8u;
    const u32 target =
        pc + 4u + static_cast<u32>(static_cast<s32>(imm) * 4);

    if (opcode == 0u) {
        const u32 funct = branch & 63u;
        if (funct == 0x08u || funct == 0x09u) {
            const u32 rd = (branch >> 11) & 31u;
            out.load_eax(rs);
            out.store_state_eax(pc_offset);
            if (funct == 0x09u && rd != 0u) {
                out.store_gpr_imm32(rd, pc + 8u);
            }
            if (!emit_body(delay, out)) {
                out.bytes.resize(before);
                return false;
            }
            out.load_state_eax(pc_offset);
            out.emit(0x05u); out.emit32(4u);
            out.store_state_eax(next_pc_offset);
            return true;
        }
    }

    if (opcode == 0x01u) {
        const u32 variant = rt;
        const bool bltz = variant == 0x00u || variant == 0x10u;
        const bool bgez = variant == 0x01u || variant == 0x11u;
        if (!bltz && !bgez) {
            out.bytes.resize(before);
            return false;
        }

        out.store_state_imm32(pc_offset, fallthrough);
        out.load_eax(rs);
        out.emit(0x85u); out.emit(0xC0u); // TEST EAX,EAX
        if (variant == 0x10u || variant == 0x11u) {
            out.store_gpr_imm32(31u, pc + 8u);
        }
        const std::size_t skip =
            out.jcc32(bltz ? 0x89u : 0x88u); // JNS / JS
        out.store_state_imm32(pc_offset, target);
        out.patch_rel32(skip, out.bytes.size());

        if (!emit_body(delay, out)) {
            out.bytes.resize(before);
            return false;
        }
        out.load_state_eax(pc_offset);
        out.emit(0x05u); out.emit32(4u);
        out.store_state_eax(next_pc_offset);
        return true;
    }

    switch (opcode) {
    case 0x02u: // J
    case 0x03u: { // JAL
        const u32 jump =
            ((pc + 4u) & 0xF0000000u) |
            ((branch & 0x03FFFFFFu) << 2u);
        if (opcode == 0x03u) {
            out.store_gpr_imm32(31u, pc + 8u);
        }
        out.store_state_imm32(pc_offset, jump);
        break;
    }
    case 0x04u:
    case 0x05u:
        out.store_state_imm32(pc_offset, fallthrough);
        out.load_eax(rs);
        out.load_edx(rt);
        out.emit(0x39u); out.emit(0xD0u); // CMP EAX,EDX
        {
            const std::size_t skip =
                out.jcc32(opcode == 0x04u ? 0x85u : 0x84u);
            out.store_state_imm32(pc_offset, target);
            out.patch_rel32(skip, out.bytes.size());
        }
        break;
    case 0x06u:
    case 0x07u:
        out.store_state_imm32(pc_offset, fallthrough);
        out.load_eax(rs);
        out.emit(0x85u); out.emit(0xC0u);
        {
            const std::size_t skip =
                out.jcc32(opcode == 0x06u ? 0x8Fu : 0x8Eu);
            out.store_state_imm32(pc_offset, target);
            out.patch_rel32(skip, out.bytes.size());
        }
        break;
    default:
        out.bytes.resize(before);
        return false;
    }

    if (!emit_body(delay, out)) {
        out.bytes.resize(before);
        return false;
    }
    out.load_state_eax(pc_offset);
    out.emit(0x05u); out.emit32(4u);
    out.store_state_eax(next_pc_offset);
    return true;
}

bool emit_block(
    u32 pc,
    u32 code_page,
    const u32* words,
    u32 word_count,
    Emitter& out,
    u32& compiled,
    bool& control_flow,
    bool& uses_ram,
    u32& store_mask) {
    compiled = 0u;
    control_flow = false;
    uses_ram = false;
    store_mask = 0u;
    out.preserve_ram_base();
    out.preserve_generation_base();

    for (u32 i = 0u; i < word_count; ++i) {
        const std::size_t before = out.bytes.size();
        if (emit_body(words[i], out)) {
            ++compiled;
            continue;
        }

        out.bytes.resize(before);
        if (emit_store(words[i], compiled, code_page, out)) {
            store_mask |= 1u << i;
            uses_ram = true;
            ++compiled;
            continue;
        }

        out.bytes.resize(before);
        if (i + 1u < word_count &&
            emit_branch_and_delay(
                pc + i * 4u,
                words[i],
                words[i + 1u],
                out)) {
            compiled += 2u;
            control_flow = true;
        }
        break;
    }

    if (compiled == 0u) return false;
    out.emit(0xB8u);
    out.emit32(compiled);
    out.emit(0xC3u);
    return true;
}

#endif

} // namespace

IopJit::~IopJit() {
    release_code_cache();
}

void IopJit::release_code_cache() {
#if defined(VIBESTATION_IOP_JIT_X64)
    for (const Page& page : pages_) {
        release_code_page(page.address);
    }
#endif
    pages_.clear();
    std::fill(entries_.begin(), entries_.end(), BlockEntry{});
}

void IopJit::clear() {
    release_code_cache();
    block_compiled_count_ = 0u;
    block_executed_count_ = 0u;
    instruction_count_ = 0u;
    chain_count_ = 0u;
    guard_exit_count_ = 0u;
    code_store_exit_count_ = 0u;
    cache_flush_count_ = 0u;
}

IopJit::BlockFunction IopJit::compile_block(
    u32 pc,
    u32 code_page,
    const u32* words,
    u32 word_count,
    u32& compiled_instructions,
    bool& control_flow,
    bool& uses_ram,
    u32& ram_store_mask) {
#if defined(VIBESTATION_IOP_JIT_X64)
    Emitter emitter;
    if (!emit_block(
            pc,
            code_page,
            words,
            word_count,
            emitter,
            compiled_instructions,
            control_flow,
            uses_ram,
            ram_store_mask)) {
        return nullptr;
    }

    if (pages_.empty() ||
        pages_.back().used + emitter.bytes.size() > kCodePageSize) {
        if (pages_.size() >= kMaxCodePages) {
            release_code_cache();
            ++cache_flush_count_;
        }
        void* address = allocate_code_page();
        if (address == nullptr) return nullptr;
        pages_.push_back(Page{address, 0u});
    }

    Page& page = pages_.back();
    if (page.used != 0u &&
        !protect_code_page(page.address, false)) {
        return nullptr;
    }
    auto* code = static_cast<u8*>(page.address) + page.used;
    std::memcpy(code, emitter.bytes.data(), emitter.bytes.size());
    if (!protect_code_page(page.address, true)) return nullptr;
    flush_code(code, emitter.bytes.size());
    page.used += emitter.bytes.size();
    ++block_compiled_count_;
    return reinterpret_cast<BlockFunction>(code);
#else
    (void)pc;
    (void)code_page;
    (void)words;
    (void)word_count;
    compiled_instructions = 0u;
    control_flow = false;
    uses_ram = false;
    ram_store_mask = 0u;
    return nullptr;
#endif
}

u32 IopJit::run(IopCpu& cpu, u32 maximum_instructions) {
#if defined(VIBESTATION_IOP_JIT_X64)
    if (maximum_instructions == 0u ||
        cpu.halted_ ||
        cpu.pending_load_.valid ||
        cpu.next_load_.valid ||
        cpu.next_is_delay_slot_ ||
        cpu.bus_.interrupt_pending()) {
        return 0u;
    }

    auto code_domain = [&](u32 pc,
                           u32& generation,
                           u32& code_page,
                           bool& ram_code) -> u32 {
        const u32 physical = IopBus::to_physical(pc);
        ram_code = physical < 0x00800000u;
        if (ram_code) {
            const u32 offset =
                physical & static_cast<u32>(IopRam::kSize - 1u);
            cpu.bus_.jit_track_code_page(pc);
            generation = cpu.bus_.jit_page_generation(pc);
            code_page = offset / IopRam::kPageSize;
            return code_page;
        }
        generation = 0u;
        code_page = 0xFFFFFFFFu;
        return 0x80000000u | (physical >> 12u);
    };

    u32 initial_generation = 0u;
    u32 initial_code_page = 0xFFFFFFFFu;
    bool initial_ram = false;
    const u32 domain = code_domain(
        cpu.state_.pc,
        initial_generation,
        initial_code_page,
        initial_ram);

    u32 retired_total = 0u;
    u32 current_pc = cpu.state_.pc;
    u32 generation = initial_generation;
    u32 code_page = initial_code_page;
    bool ram_code = initial_ram;
    u32 chained_blocks = 0u;

    while (retired_total < maximum_instructions) {
        const u32 hash =
            (current_pc >> 2u) * 2654435761u ^
            generation * 2246822519u;
        const std::size_t index =
            (static_cast<std::size_t>(hash) * entries_.size()) >> 32u;
        BlockEntry& entry = entries_[index];

        if (!entry.known ||
            entry.pc != current_pc ||
            entry.generation != generation) {
            u32 fetched[32]{};
            const u32 page_offset = current_pc & 0xFFFu;
            const u32 words_to_page =
                (0x1000u - page_offset) / 4u;
            const u32 remaining =
                maximum_instructions - retired_total;
            const u32 fetch_count =
                std::min<u32>(32u, std::min(words_to_page, remaining));
            u32 valid = 0u;
            for (; valid < fetch_count; ++valid) {
                if (!cpu.bus_.read32(
                        current_pc + valid * 4u,
                        fetched[valid])) {
                    break;
                }
            }
            if (valid == 0u) break;

            u32 compiled = 0u;
            bool control = false;
            bool uses_ram = false;
            u32 store_mask = 0u;
            BlockFunction function = compile_block(
                current_pc,
                code_page,
                fetched,
                valid,
                compiled,
                control,
                uses_ram,
                store_mask);

            entry = {};
            entry.pc = current_pc;
            entry.generation = generation;
            entry.instruction_count =
                static_cast<u8>(compiled);
            entry.function = function;
            entry.control_flow = control;
            entry.uses_ram = uses_ram;
            entry.ram_store_mask = store_mask;
            for (u32 i = 0u; i < compiled && i < 32u; ++i) {
                entry.words[i] = fetched[i];
            }
            entry.known = true;
        }

        if (entry.function == nullptr ||
            entry.instruction_count == 0u ||
            entry.instruction_count >
                maximum_instructions - retired_total) {
            break;
        }

        const u32 retired = entry.function(
            &cpu.state_,
            cpu.bus_.jit_ram_data(),
            cpu.bus_.jit_page_generations());
        if (retired == 0u ||
            retired > entry.instruction_count) {
            if (entry.uses_ram) ++guard_exit_count_;
            break;
        }

        ++block_executed_count_;
        instruction_count_ += retired;
        ++chained_blocks;

        const bool full =
            retired == entry.instruction_count;
        const bool control =
            entry.control_flow && full;

        if (!full) {
            const bool code_store =
                (entry.ram_store_mask &
                 (1u << (retired - 1u))) != 0u;
            if (code_store) ++code_store_exit_count_;
            else ++guard_exit_count_;
        }

        cpu.state_.last_pc =
            current_pc + (retired - 1u) * 4u;
        cpu.state_.last_instruction =
            entry.words[retired - 1u];

        if (!control) {
            cpu.state_.pc =
                current_pc + retired * 4u;
            cpu.state_.next_pc =
                cpu.state_.pc + 4u;
        }

        retired_total += retired;
        if (!full || retired_total >= maximum_instructions) break;

        if (ram_code &&
            cpu.bus_.jit_page_generation(current_pc) != generation) {
            break;
        }

        const u32 next_pc = cpu.state_.pc;
        u32 next_generation = 0u;
        u32 next_code_page = 0xFFFFFFFFu;
        bool next_ram = false;
        const u32 next_domain = code_domain(
            next_pc,
            next_generation,
            next_code_page,
            next_ram);
        if (next_domain != domain) break;

        current_pc = next_pc;
        generation = next_generation;
        code_page = next_code_page;
        ram_code = next_ram;
    }

    if (chained_blocks > 1u) ++chain_count_;
    cpu.state_.instructions_executed += retired_total;
    cpu.state_.gpr[0] = 0u;
    return retired_total;
#else
    (void)cpu;
    (void)maximum_instructions;
    return 0u;
#endif
}

} // namespace ps2
