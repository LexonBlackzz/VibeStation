#include "core/ee/ee_jit.h"
#include "core/ee/ee_cpu.h"
#include "core/memory/ee_ram.h"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>

#if defined(_M_X64) || defined(__x86_64__)
#define VIBESTATION_EE_JIT_X64 1
#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/mman.h>
#endif
#endif

namespace ps2 {
namespace {

#if defined(VIBESTATION_EE_JIT_X64)
constexpr std::size_t kPageSize = 64u * 1024u;
constexpr std::size_t kMaxPages = 256u;
#ifdef _WIN32
constexpr u8 kArgumentRegister = 1u; // RCX
constexpr u8 kRamArgumentRegister = 2u; // RDX
constexpr u8 kGenerationArgumentRegister = 8u; // R8
constexpr u8 kScratchArgumentRegister = 9u; // R9
#else
constexpr u8 kArgumentRegister = 7u; // RDI
constexpr u8 kRamArgumentRegister = 6u; // RSI
constexpr u8 kGenerationArgumentRegister = 2u; // RDX
constexpr u8 kScratchArgumentRegister = 1u; // RCX
#endif
constexpr u32 kEeRamSize = 32u * 1024u * 1024u;
constexpr u32 kEeScratchBase = 0x70000000u;
constexpr u32 kEeScratchSize = 16u * 1024u;

void ee_jit_cop1_register_helper(
    EeCpuState* state,
    u32 instruction);
void ee_jit_scratch_memory_helper(
    EeCpuState* state,
    u8* scratch,
    u32 instruction);

struct Emitter {
    std::vector<u8> bytes;

    void emit(u8 value) { bytes.push_back(value); }
    void emit32(u32 value) {
        for (u32 i = 0; i < 4; ++i) emit(static_cast<u8>(value >> (i * 8)));
    }
    void emit64(u64 value) {
        for (u32 i = 0; i < 8; ++i) emit(static_cast<u8>(value >> (i * 8)));
    }
    void memory(u8 rex, u8 opcode, u8 reg, u32 offset) {
        if (rex != 0) emit(rex);
        emit(opcode);
        emit(static_cast<u8>(0x80u | (reg << 3) | kArgumentRegister));
        emit32(offset);
    }
    void load_rax(u32 reg, bool word) {
        memory(word ? 0u : 0x48u, 0x8Bu, 0u,
               static_cast<u32>(offsetof(EeCpuState, gpr) +
                                reg * sizeof(EeGpr)));
    }
    void load_rdx(u32 reg, bool word) {
        memory(word ? 0u : 0x48u, 0x8Bu, 2u,
               static_cast<u32>(offsetof(EeCpuState, gpr) +
                                reg * sizeof(EeGpr)));
    }
    void store_rax(u32 reg) {
        memory(0x48u, 0x89u, 0u,
               static_cast<u32>(offsetof(EeCpuState, gpr) +
                                reg * sizeof(EeGpr)));
    }
    void store_rax_hi(u32 reg) {
        memory(0x48u, 0x89u, 0u,
               static_cast<u32>(offsetof(EeCpuState, gpr) +
                                reg * sizeof(EeGpr) + sizeof(u64)));
    }
    void load_rdx_hi(u32 reg) {
        memory(0x48u, 0x8Bu, 2u,
               static_cast<u32>(offsetof(EeCpuState, gpr) +
                                reg * sizeof(EeGpr) + sizeof(u64)));
    }
    void store_gpr_imm64(u32 reg, u64 value) {
        emit(0x48u);
        emit(0xB8u);
        emit64(value);
        store_rax(reg);
    }
    void store_state_imm32(u32 offset, u32 value) {
        emit(0xC7u);
        emit(static_cast<u8>(0x80u | kArgumentRegister));
        emit32(offset);
        emit32(value);
    }
    void load_state_eax(u32 offset) {
        memory(0u, 0x8Bu, 0u, offset);
    }
    void load_state_rax(u32 offset) {
        memory(0x48u, 0x8Bu, 0u, offset);
    }
    void store_state_eax(u32 offset) {
        memory(0u, 0x89u, 0u, offset);
    }
    void store_state_rax(u32 offset) {
        memory(0x48u, 0x89u, 0u, offset);
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
    void preserve_ram_base() {
        emit(0x49u);
        emit(0x89u);
        emit(static_cast<u8>(0xC0u |
            ((kRamArgumentRegister & 7u) << 3) | 3u)); // MOV R11, RAM arg
    }
    void preserve_generation_base() {
        const u8 rex = static_cast<u8>(
            0x49u | ((kGenerationArgumentRegister & 8u) ? 0x04u : 0u));
        emit(rex);
        emit(0x89u);
        emit(static_cast<u8>(
            0xC0u |
            ((kGenerationArgumentRegister & 7u) << 3) |
            2u)); // MOV R10, generation arg
    }
    void preserve_scratch_base() {
#ifdef _WIN32
        // The fourth Win64 argument already arrives in R9.
        static_assert(kScratchArgumentRegister == 9u);
#else
        // SysV's fourth argument is RCX; keep it in caller-saved R9 so the
        // memory fast path has one stable base on both ABIs.
        static_assert(kScratchArgumentRegister == 1u);
        emit(0x49u); emit(0x89u); emit(0xC9u); // MOV R9,RCX
#endif
    }
    void patch_rel32(std::size_t displacement, std::size_t target) {
        const s64 rel = static_cast<s64>(target) -
            static_cast<s64>(displacement + 4u);
        const u32 encoded = static_cast<u32>(static_cast<s32>(rel));
        for (u32 i = 0; i < 4u; ++i) {
            bytes[displacement + i] =
                static_cast<u8>(encoded >> (i * 8u));
        }
    }
    void sign_extend_word() { emit(0x48u); emit(0x98u); } // CDQE

    void call_state_instruction_helper(
        void (*helper)(EeCpuState*, u32),
        u32 instruction) {
#ifdef _WIN32
        // Preserve state plus all three resident pointer registers. Five
        // stack slots (4 pushes + 40 bytes) leave CALL 16-byte aligned while
        // also providing Win64's mandatory 32-byte shadow space.
        emit(0x51u);                         // PUSH RCX
        emit(0x41u); emit(0x51u);           // PUSH R9
        emit(0x41u); emit(0x52u);           // PUSH R10
        emit(0x41u); emit(0x53u);           // PUSH R11
        emit(0x48u); emit(0x83u); emit(0xECu); emit(0x28u); // SUB RSP,40
        emit(0xBAu); emit32(instruction);    // MOV EDX,imm32
        emit(0x48u); emit(0xB8u);
        emit64(reinterpret_cast<u64>(helper));
        emit(0xFFu); emit(0xD0u);            // CALL RAX
        emit(0x48u); emit(0x83u); emit(0xC4u); emit(0x28u); // ADD RSP,40
        emit(0x41u); emit(0x5Bu);            // POP R11
        emit(0x41u); emit(0x5Au);            // POP R10
        emit(0x41u); emit(0x59u);            // POP R9
        emit(0x59u);                         // POP RCX
#else
        // Four pushes retain state + R9/R10/R11; the extra 8-byte adjustment
        // restores the SysV 16-byte call-site alignment.
        emit(0x57u);                         // PUSH RDI
        emit(0x41u); emit(0x51u);           // PUSH R9
        emit(0x41u); emit(0x52u);           // PUSH R10
        emit(0x41u); emit(0x53u);           // PUSH R11
        emit(0x48u); emit(0x83u); emit(0xECu); emit(0x08u); // SUB RSP,8
        emit(0xBEu); emit32(instruction);    // MOV ESI,imm32
        emit(0x48u); emit(0xB8u);
        emit64(reinterpret_cast<u64>(helper));
        emit(0xFFu); emit(0xD0u);            // CALL RAX
        emit(0x48u); emit(0x83u); emit(0xC4u); emit(0x08u); // ADD RSP,8
        emit(0x41u); emit(0x5Bu);            // POP R11
        emit(0x41u); emit(0x5Au);            // POP R10
        emit(0x41u); emit(0x59u);            // POP R9
        emit(0x5Fu);                         // POP RDI
#endif
    }

    void call_state_scratch_instruction_helper(
        void (*helper)(EeCpuState*, u8*, u32),
        u32 instruction) {
#ifdef _WIN32
        emit(0x51u);                         // PUSH RCX
        emit(0x41u); emit(0x51u);           // PUSH R9
        emit(0x41u); emit(0x52u);           // PUSH R10
        emit(0x41u); emit(0x53u);           // PUSH R11
        emit(0x48u); emit(0x83u); emit(0xECu); emit(0x28u);
        emit(0x4Cu); emit(0x89u); emit(0xCAu); // MOV RDX,R9
        emit(0x41u); emit(0xB8u); emit32(instruction); // MOV R8D,imm32
        emit(0x48u); emit(0xB8u); emit64(reinterpret_cast<u64>(helper));
        emit(0xFFu); emit(0xD0u);
        emit(0x48u); emit(0x83u); emit(0xC4u); emit(0x28u);
        emit(0x41u); emit(0x5Bu);
        emit(0x41u); emit(0x5Au);
        emit(0x41u); emit(0x59u);
        emit(0x59u);
#else
        emit(0x57u);                         // PUSH RDI
        emit(0x41u); emit(0x51u);           // PUSH R9
        emit(0x41u); emit(0x52u);           // PUSH R10
        emit(0x41u); emit(0x53u);           // PUSH R11
        emit(0x48u); emit(0x83u); emit(0xECu); emit(0x08u);
        emit(0x4Cu); emit(0x89u); emit(0xCEu); // MOV RSI,R9
        emit(0xBAu); emit32(instruction);    // MOV EDX,imm32
        emit(0x48u); emit(0xB8u); emit64(reinterpret_cast<u64>(helper));
        emit(0xFFu); emit(0xD0u);
        emit(0x48u); emit(0x83u); emit(0xC4u); emit(0x08u);
        emit(0x41u); emit(0x5Bu);
        emit(0x41u); emit(0x5Au);
        emit(0x41u); emit(0x59u);
        emit(0x5Fu);
#endif
    }
};

void* allocate_page() {
#ifdef _WIN32
    return VirtualAlloc(nullptr, kPageSize, MEM_COMMIT | MEM_RESERVE,
                        PAGE_READWRITE);
#else
    void* page = mmap(nullptr, kPageSize, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    return page == MAP_FAILED ? nullptr : page;
#endif
}

bool protect_page(void* page, bool executable) {
#ifdef _WIN32
    DWORD old_protection = 0;
    return VirtualProtect(page, kPageSize,
                          executable ? PAGE_EXECUTE_READ : PAGE_READWRITE,
                          &old_protection) != 0;
#else
    return mprotect(page, kPageSize,
                    executable ? PROT_READ | PROT_EXEC
                               : PROT_READ | PROT_WRITE) == 0;
#endif
}

void release_page(void* page) {
#ifdef _WIN32
    VirtualFree(page, 0, MEM_RELEASE);
#else
    munmap(page, kPageSize);
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

float ee_jit_ps2_fpu_input(u32 bits) {
    const u32 exponent = bits & 0x7F800000u;
    if (exponent == 0u) {
        bits &= 0x80000000u;
    } else if (exponent == 0x7F800000u) {
        bits = (bits & 0x80000000u) | 0x7F7FFFFFu;
    }
    return std::bit_cast<float>(bits);
}

u32 ee_jit_ps2_fpu_result(float value) {
    u32 bits = std::bit_cast<u32>(value);
    const u32 exponent = bits & 0x7F800000u;
    if (exponent == 0u) return bits & 0x80000000u;
    if (exponent == 0x7F800000u) {
        return (bits & 0x80000000u) | 0x7F7FFFFFu;
    }
    return bits;
}

bool ee_jit_cop1_register_helper_supported(u32 instruction) {
    if ((instruction >> 26) != 0x11u) return false;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 funct = instruction & 63u;
    if (rs == 0x14u) {
        return funct == 0x20u; // CVT.S.W
    }
    if (rs != 0x10u) return false;
    switch (funct) {
    case 0x00u: // ADD.S
    case 0x01u: // SUB.S
    case 0x02u: // MUL.S
    case 0x03u: // DIV.S
    case 0x04u: // SQRT.S
    case 0x16u: // RSQRT.S
    case 0x18u: // ADDA.S
    case 0x19u: // SUBA.S
    case 0x1Au: // MULA.S
    case 0x1Cu: // MADD.S
    case 0x1Du: // MSUB.S
    case 0x1Eu: // MADDA.S
    case 0x1Fu: // MSUBA.S
    case 0x24u: // CVT.W.S
    case 0x28u: // MAX.S
    case 0x29u: // MIN.S
    case 0x30u: // C.F.S
    case 0x32u: // C.EQ.S
    case 0x34u: // C.LT.S
    case 0x36u: // C.LE.S
        return true;
    default:
        return false;
    }
}

void ee_jit_cop1_register_helper(
    EeCpuState* state,
    u32 instruction) {
    if (state == nullptr) return;

    const u32 rs = (instruction >> 21) & 31u;
    const u32 ft = (instruction >> 16) & 31u;
    const u32 fs = (instruction >> 11) & 31u;
    const u32 fd = (instruction >> 6) & 31u;
    const u32 funct = instruction & 63u;
    constexpr u32 kCond = 0x00800000u;

    if (rs == 0x14u) { // COP1.W: CVT.S.W
        const s32 value = static_cast<s32>(state->fpr[fs]);
        state->fpr[fd] =
            ee_jit_ps2_fpu_result(static_cast<float>(value));
        return;
    }

    const float a = ee_jit_ps2_fpu_input(state->fpr[fs]);
    const float b = ee_jit_ps2_fpu_input(state->fpr[ft]);
    const float acc = ee_jit_ps2_fpu_input(state->fpu_acc);
    auto set_fd = [&](float value) {
        state->fpr[fd] = ee_jit_ps2_fpu_result(value);
    };
    auto set_acc = [&](float value) {
        state->fpu_acc = ee_jit_ps2_fpu_result(value);
    };
    auto set_cond = [&](bool value) {
        if (value) state->fcr[31] |= kCond;
        else state->fcr[31] &= ~kCond;
    };

    switch (funct) {
    case 0x00u: set_fd(a + b); break;
    case 0x01u: set_fd(a - b); break;
    case 0x02u: set_fd(a * b); break;
    case 0x03u:
        if ((state->fpr[ft] & 0x7FFFFFFFu) == 0u) {
            const u32 sign =
                (state->fpr[fs] ^ state->fpr[ft]) & 0x80000000u;
            state->fpr[fd] = sign | 0x7F7FFFFFu;
        } else {
            set_fd(a / b);
        }
        break;
    case 0x04u: set_fd(std::sqrt(std::fabs(b))); break;
    case 0x16u:
        if ((state->fpr[ft] & 0x7FFFFFFFu) == 0u) {
            const u32 sign =
                (state->fpr[fs] ^ state->fpr[ft]) & 0x80000000u;
            state->fpr[fd] = sign | 0x7F7FFFFFu;
        } else {
            set_fd(a / std::sqrt(std::fabs(b)));
        }
        break;
    case 0x18u: set_acc(a + b); break;
    case 0x19u: set_acc(a - b); break;
    case 0x1Au: set_acc(a * b); break;
    case 0x1Cu: set_fd(acc + (a * b)); break;
    case 0x1Du: set_fd(acc - (a * b)); break;
    case 0x1Eu: set_acc(acc + (a * b)); break;
    case 0x1Fu: set_acc(acc - (a * b)); break;
    case 0x24u:
        if ((state->fpr[fs] & 0x7F800000u) <= 0x4E800000u) {
            const double value = static_cast<double>(a);
            if (value > 2147483647.0) state->fpr[fd] = 0x7FFFFFFFu;
            else if (value < -2147483648.0) state->fpr[fd] = 0x80000000u;
            else state->fpr[fd] =
                static_cast<u32>(static_cast<s32>(value));
        } else {
            state->fpr[fd] =
                (state->fpr[fs] & 0x80000000u)
                    ? 0x80000000u
                    : 0x7FFFFFFFu;
        }
        break;
    case 0x28u:
        state->fpr[fd] =
            (a >= b) ? state->fpr[fs] : state->fpr[ft];
        break;
    case 0x29u:
        state->fpr[fd] =
            (a <= b) ? state->fpr[fs] : state->fpr[ft];
        break;
    case 0x30u: set_cond(false); break;
    case 0x32u: set_cond(a == b); break;
    case 0x34u: set_cond(a < b); break;
    case 0x36u: set_cond(a <= b); break;
    default: break;
    }
}

bool emit_instruction_body(u32 instruction, Emitter& out) {
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
                out.load_rax(rt, true);
                out.emit(0xC1u);
                out.emit(funct == 0x00u ? 0xE0u :
                         funct == 0x02u ? 0xE8u : 0xF8u);
                out.emit(static_cast<u8>(sa));
                out.sign_extend_word();
            }
            break;
        case 0x04u: // SLLV
        case 0x06u: // SRLV
        case 0x07u: // SRAV
            if (sa != 0u) return false;
            if (destination != 0u) {
                out.load_rdx(rs, true);
                out.load_rax(rt, true);
                // x86 variable shifts use CL. On Win64 RCX holds the state
                // pointer, so preserve it across the shift; SysV uses RDI.
                if constexpr (kArgumentRegister == 1u) {
                    out.emit(0x51u); // PUSH RCX
                }
                out.emit(0x89u); out.emit(0xD1u); // MOV ECX,EDX
                out.emit(0xD3u);
                out.emit(
                    funct == 0x04u ? 0xE0u :
                    funct == 0x06u ? 0xE8u : 0xF8u);
                out.sign_extend_word();
                if constexpr (kArgumentRegister == 1u) {
                    out.emit(0x59u); // POP RCX
                }
            }
            break;
        case 0x0Au: // MOVZ
        case 0x0Bu: // MOVN
            if (sa != 0u) return false;
            if (destination != 0u) {
                out.load_rax(rs, false);
                out.load_rdx(rt, false);
                out.emit(0x48u); out.emit(0x85u); out.emit(0xD2u);
                const std::size_t skip =
                    out.jcc32(funct == 0x0Au ? 0x85u : 0x84u);
                out.store_rax(destination);
                out.patch_rel32(skip, out.bytes.size());
            }
            destination = 0u;
            break;
        case 0x0Fu: // SYNC
            destination = 0u;
            break;
        case 0x10u: // MFHI
            if (destination != 0u) {
                out.load_state_rax(
                    static_cast<u32>(offsetof(EeCpuState, hi)));
            }
            break;
        case 0x11u: // MTHI
            out.load_rax(rs, false);
            out.store_state_rax(
                static_cast<u32>(offsetof(EeCpuState, hi)));
            destination = 0u;
            break;
        case 0x12u: // MFLO
            if (destination != 0u) {
                out.load_state_rax(
                    static_cast<u32>(offsetof(EeCpuState, lo)));
            }
            break;
        case 0x13u: // MTLO
            out.load_rax(rs, false);
            out.store_state_rax(
                static_cast<u32>(offsetof(EeCpuState, lo)));
            destination = 0u;
            break;
        case 0x14u: // DSLLV
        case 0x16u: // DSRLV
        case 0x17u: // DSRAV
            if (sa != 0u) return false;
            if (destination != 0u) {
                out.load_rdx(rs, true);
                out.load_rax(rt, false);
                if constexpr (kArgumentRegister == 1u) {
                    out.emit(0x51u); // PUSH RCX
                }
                out.emit(0x89u); out.emit(0xD1u); // MOV ECX,EDX
                out.emit(0x48u); out.emit(0xD3u);
                out.emit(
                    funct == 0x14u ? 0xE0u :
                    funct == 0x16u ? 0xE8u : 0xF8u);
                if constexpr (kArgumentRegister == 1u) {
                    out.emit(0x59u); // POP RCX
                }
            }
            break;
        case 0x18u: // MULT
        case 0x19u: // MULTU
            if (sa != 0u) return false;
            out.load_rax(rs, true);
            out.load_rdx(rt, true);
            out.emit(0xF7u);
            out.emit(funct == 0x18u ? 0xEAu : 0xE2u); // IMUL/MUL EDX
            out.sign_extend_word();
            out.store_state_rax(
                static_cast<u32>(offsetof(EeCpuState, lo)));
            if (destination != 0u) {
                out.store_rax(destination);
            }
            out.emit(0x89u); out.emit(0xD0u); // MOV EAX,EDX
            out.sign_extend_word();
            out.store_state_rax(
                static_cast<u32>(offsetof(EeCpuState, hi)));
            destination = 0u;
            break;
        case 0x28u: // MFSA
            if (destination != 0u) {
                out.load_state_eax(
                    static_cast<u32>(offsetof(EeCpuState, sa)));
            }
            break;
        case 0x29u: // MTSA
            out.load_rax(rs, true);
            out.store_state_eax(
                static_cast<u32>(offsetof(EeCpuState, sa)));
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
        case 0x2Du: // DADDU
        case 0x2Fu: // DSUBU
            if (sa != 0u) return false;
            if (destination != 0u) {
                const bool word = funct == 0x21u || funct == 0x23u;
                out.load_rax(rs, word);
                out.load_rdx(rt, word);
                if (funct == 0x2Au || funct == 0x2Bu) {
                    out.emit(0x48u);
                    out.emit(0x39u);
                    out.emit(0xD0u); // CMP RAX,RDX
                    out.emit(0x0Fu);
                    out.emit(funct == 0x2Au ? 0x9Cu : 0x92u);
                    out.emit(0xC0u); // SETL / SETB AL
                    out.emit(0x0Fu);
                    out.emit(0xB6u);
                    out.emit(0xC0u); // MOVZX EAX,AL
                } else {
                    if (!word) out.emit(0x48u);
                    out.emit(
                        funct == 0x23u || funct == 0x2Fu ? 0x29u :
                        funct == 0x24u ? 0x21u :
                        funct == 0x25u || funct == 0x27u ? 0x09u :
                        funct == 0x26u ? 0x31u :
                        0x01u);
                    out.emit(0xD0u); // operation RAX, RDX
                    if (funct == 0x27u) {
                        out.emit(0x48u);
                        out.emit(0xF7u);
                        out.emit(0xD0u); // NOT RAX
                    }
                    if (word) out.sign_extend_word();
                }
            }
            break;
        case 0x38u: // DSLL
        case 0x3Au: // DSRL
        case 0x3Bu: // DSRA
        case 0x3Cu: // DSLL32
        case 0x3Eu: // DSRL32
        case 0x3Fu: // DSRA32
            if (rs != 0u) return false;
            if (destination != 0u) {
                out.load_rax(rt, false);
                out.emit(0x48u);
                out.emit(0xC1u);
                out.emit(
                    funct == 0x38u || funct == 0x3Cu ? 0xE0u :
                    funct == 0x3Au || funct == 0x3Eu ? 0xE8u :
                    0xF8u);
                const u32 shift =
                    sa + ((funct & 0x04u) != 0u ? 32u : 0u);
                out.emit(static_cast<u8>(shift));
            }
            break;
        default:
            return false;
        }
    } else {
        switch (opcode) {
        case 0x09u: // ADDIU
            if (destination != 0u) {
                out.load_rax(rs, true);
                out.emit(0x05u); // ADD EAX, imm32
                out.emit32(static_cast<u32>(static_cast<s32>(
                    static_cast<s16>(immediate))));
                out.sign_extend_word();
            }
            break;
        case 0x0Au: // SLTI
        case 0x0Bu: // SLTIU
            if (destination != 0u) {
                out.load_rax(rs, false);
                out.emit(0x48u);
                out.emit(0x3Du); // CMP RAX, sign-extended imm32
                out.emit32(static_cast<u32>(static_cast<s32>(
                    static_cast<s16>(immediate))));
                out.emit(0x0Fu);
                out.emit(opcode == 0x0Au ? 0x9Cu : 0x92u); // SETL / SETB
                out.emit(0xC0u);
                out.emit(0x0Fu);
                out.emit(0xB6u);
                out.emit(0xC0u); // MOVZX EAX,AL
            }
            break;
        case 0x0Cu: // ANDI
        case 0x0Du: // ORI
        case 0x0Eu: // XORI
            if (destination != 0u) {
                out.load_rax(rs, false);
                out.emit(0x48u);
                out.emit(opcode == 0x0Cu ? 0x25u :
                         opcode == 0x0Du ? 0x0Du : 0x35u);
                out.emit32(immediate);
            }
            break;
        case 0x0Fu: // LUI
            if (destination != 0u) {
                out.emit(0xB8u); // MOV EAX, imm32
                out.emit32(immediate << 16);
                out.sign_extend_word();
            }
            break;
        case 0x19u: // DADDIU
            if (destination != 0u) {
                out.load_rax(rs, false);
                out.emit(0x48u);
                out.emit(0x05u); // ADD RAX, sign-extended imm32
                out.emit32(static_cast<u32>(static_cast<s32>(
                    static_cast<s16>(immediate))));
            }
            break;
        case 0x01u: { // non-branch REGIMM helpers
            const u32 variant = rt;
            if (variant != 0x18u && variant != 0x19u) return false;
            out.load_rax(rs, true);
            out.emit(0x25u); // AND EAX, mask
            out.emit32(variant == 0x18u ? 0xFu : 0x7u);
            out.emit(0x35u); // XOR EAX, immediate low bits
            out.emit32(
                static_cast<u32>(static_cast<u16>(immediate)) &
                (variant == 0x18u ? 0xFu : 0x7u));
            if (variant == 0x19u) {
                out.emit(0xD1u); out.emit(0xE0u); // SHL EAX,1
            }
            out.store_state_eax(
                static_cast<u32>(offsetof(EeCpuState, sa)));
            destination = 0u;
            break;
        }
        case 0x11u: { // COP1
            const u32 cop_rs = rs;
            const u32 fs = (instruction >> 11) & 31u;
            const u32 fd = (instruction >> 6) & 31u;
            const u32 cop_funct = instruction & 63u;
            if (cop_rs == 0x00u) { // MFC1
                if (rt != 0u) {
                    out.load_state_eax(
                        static_cast<u32>(
                            offsetof(EeCpuState, fpr) +
                            fs * sizeof(u32)));
                    out.sign_extend_word();
                    out.store_rax(rt);
                }
            } else if (cop_rs == 0x02u) { // CFC1
                if (rt != 0u) {
                    if (fs == 0u) {
                        out.emit(0xB8u);
                        out.emit32(0x00002E00u);
                    } else if (fs == 31u) {
                        out.load_state_eax(
                            static_cast<u32>(
                                offsetof(EeCpuState, fcr) +
                                31u * sizeof(u32)));
                    } else {
                        out.emit(0x31u); out.emit(0xC0u); // XOR EAX,EAX
                    }
                    out.sign_extend_word();
                    out.store_rax(rt);
                }
            } else if (cop_rs == 0x04u) { // MTC1
                out.load_rax(rt, true);
                out.store_state_eax(
                    static_cast<u32>(
                        offsetof(EeCpuState, fpr) +
                        fs * sizeof(u32)));
            } else if (cop_rs == 0x06u) { // CTC1
                if (fs == 31u) {
                    out.load_rax(rt, true);
                    out.store_state_eax(
                        static_cast<u32>(
                            offsetof(EeCpuState, fcr) +
                            31u * sizeof(u32)));
                }
            } else if (cop_rs == 0x10u &&
                       (cop_funct == 0x05u ||
                        cop_funct == 0x06u ||
                        cop_funct == 0x07u)) {
                out.load_state_eax(
                    static_cast<u32>(
                        offsetof(EeCpuState, fpr) +
                        fs * sizeof(u32)));
                if (cop_funct == 0x05u) {
                    out.emit(0x25u); out.emit32(0x7FFFFFFFu); // AND EAX
                } else if (cop_funct == 0x07u) {
                    out.emit(0x35u); out.emit32(0x80000000u); // XOR EAX
                }
                out.store_state_eax(
                    static_cast<u32>(
                        offsetof(EeCpuState, fpr) +
                        fd * sizeof(u32)));
            } else if (
                ee_jit_cop1_register_helper_supported(instruction)) {
                out.call_state_instruction_helper(
                    &ee_jit_cop1_register_helper,
                    instruction);
            } else {
                return false;
            }
            destination = 0u;
            break;
        }
        case 0x2Fu: // CACHE
        case 0x33u: // PREF
            destination = 0u;
            break;
        default:
            return false;
        }
    }

    if (destination != 0u) out.store_rax(destination);
    return true;
}

bool emit_instruction(u32 instruction, Emitter& out) {
    if (!emit_instruction_body(instruction, out)) return false;
    out.emit(0xC3u); // RET
    return true;
}

void ee_jit_scratch_memory_helper(
    EeCpuState* state,
    u8* scratch,
    u32 instruction) {
    if (state == nullptr || scratch == nullptr) return;

    const u32 opcode = instruction >> 26;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const s16 imm = static_cast<s16>(instruction & 0xFFFFu);
    u32 address =
        static_cast<u32>(state->gpr[rs].lo) +
        static_cast<u32>(static_cast<s32>(imm));
    if (opcode == 0x1Eu || opcode == 0x1Fu) {
        address &= ~0x0Fu;
    }
    const u32 offset = address - kEeScratchBase;

    auto load16 = [&](u32 at) {
        u16 value = 0u;
        std::memcpy(&value, scratch + at, sizeof(value));
        return value;
    };
    auto load32 = [&](u32 at) {
        u32 value = 0u;
        std::memcpy(&value, scratch + at, sizeof(value));
        return value;
    };
    auto load64 = [&](u32 at) {
        u64 value = 0u;
        std::memcpy(&value, scratch + at, sizeof(value));
        return value;
    };
    auto store16 = [&](u32 at, u16 value) {
        std::memcpy(scratch + at, &value, sizeof(value));
    };
    auto store32 = [&](u32 at, u32 value) {
        std::memcpy(scratch + at, &value, sizeof(value));
    };
    auto store64 = [&](u32 at, u64 value) {
        std::memcpy(scratch + at, &value, sizeof(value));
    };
    auto write_word = [&](u32 reg, u32 value) {
        if (reg != 0u) {
            state->gpr[reg].lo = static_cast<u64>(
                static_cast<s64>(static_cast<s32>(value)));
        }
    };
    auto write64 = [&](u32 reg, u64 value) {
        if (reg != 0u) state->gpr[reg].lo = value;
    };

    switch (opcode) {
    case 0x1Eu: // LQ
        if (rt != 0u) {
            state->gpr[rt].lo = load64(offset);
            state->gpr[rt].hi = load64(offset + 8u);
        }
        break;
    case 0x1Fu: // SQ
        store64(offset, state->gpr[rt].lo);
        store64(offset + 8u, state->gpr[rt].hi);
        break;
    case 0x20u: // LB
        write64(
            rt,
            static_cast<u64>(
                static_cast<s64>(
                    static_cast<s8>(scratch[offset]))));
        break;
    case 0x21u: // LH
        write64(
            rt,
            static_cast<u64>(
                static_cast<s64>(
                    static_cast<s16>(load16(offset)))));
        break;
    case 0x23u: // LW
    case 0x30u: // LL
        write_word(rt, load32(offset));
        break;
    case 0x24u: // LBU
        write64(rt, scratch[offset]);
        break;
    case 0x25u: // LHU
        write64(rt, load16(offset));
        break;
    case 0x27u: // LWU
        write64(rt, load32(offset));
        break;
    case 0x31u: // LWC1
        state->fpr[rt] = load32(offset);
        break;
    case 0x34u: // LLD
    case 0x37u: // LD
        write64(rt, load64(offset));
        break;
    case 0x28u: // SB
        scratch[offset] = static_cast<u8>(state->gpr[rt].lo);
        break;
    case 0x29u: // SH
        store16(offset, static_cast<u16>(state->gpr[rt].lo));
        break;
    case 0x2Bu: // SW
        store32(offset, static_cast<u32>(state->gpr[rt].lo));
        break;
    case 0x38u: // SC
        store32(offset, static_cast<u32>(state->gpr[rt].lo));
        write_word(rt, 1u);
        break;
    case 0x39u: // SWC1
        store32(offset, state->fpr[rt]);
        break;
    case 0x3Cu: // SCD
        store64(offset, state->gpr[rt].lo);
        write64(rt, 1u);
        break;
    case 0x3Fu: // SD
        store64(offset, state->gpr[rt].lo);
        break;
    default:
        break;
    }
}

bool emit_guarded_ram_load(
    u32 instruction,
    u32 retired_before,
    Emitter& out,
    bool rollback_branch = false,
    u32 rollback_pc = 0u) {
    const u32 opcode = instruction >> 26;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const s16 immediate =
        static_cast<s16>(instruction & 0xFFFFu);

    u32 width = 0;
    switch (opcode) {
    case 0x20u: // LB
    case 0x24u: // LBU
        width = 1u;
        break;
    case 0x21u: // LH
    case 0x25u: // LHU
        width = 2u;
        break;
    case 0x23u: // LW
    case 0x27u: // LWU
    case 0x30u: // LL
    case 0x31u: // LWC1
        width = 4u;
        break;
    case 0x34u: // LLD
    case 0x37u: // LD
        width = 8u;
        break;
    case 0x1Eu: // LQ
        width = 16u;
        break;
    default:
        return false;
    }

    // EE effective addresses use the low 32 bits. The system's quiet block
    // guard only permits main-RAM aliases, so reproduce EeBus::to_physical()
    // here and return to the interpreter before the load if the address is
    // outside that set.
    out.load_rax(rs, true);
    out.emit(0x05u); // ADD EAX, imm32
    out.emit32(static_cast<u32>(static_cast<s32>(immediate)));

    std::vector<std::size_t> fail_jumps;
    std::vector<std::size_t> direct_jumps;
    std::vector<std::size_t> alias_jumps;
    std::vector<std::size_t> scratch_jumps;

    // Scratchpad is not part of EeBus::to_physical main RAM. Keep it native
    // through a tiny bound helper instead of treating it as an MMIO guard
    // failure and abandoning the entire block.
    out.emit(0x3Du); out.emit32(kEeScratchBase);
    const std::size_t below_scratch = out.jcc32(0x82u); // JB
    out.emit(0x3Du);
    out.emit32(
        kEeScratchBase + kEeScratchSize -
        (opcode == 0x1Eu ? 1u : width));
    scratch_jumps.push_back(out.jcc32(0x86u)); // JBE
    const std::size_t main_mapping_label = out.bytes.size();
    out.patch_rel32(below_scratch, main_mapping_label);

    out.emit(0x3Du); // CMP EAX, 0xC0000000
    out.emit32(0xC0000000u);
    fail_jumps.push_back(out.jcc32(0x83u)); // JAE

    out.emit(0x3Du); // CMP EAX, 0x20000000
    out.emit32(0x20000000u);
    direct_jumps.push_back(out.jcc32(0x82u)); // JB

    out.emit(0x3Du); // CMP EAX, 0x22000000
    out.emit32(0x22000000u);
    alias_jumps.push_back(out.jcc32(0x82u)); // JB

    out.emit(0x3Du); // CMP EAX, 0x30000000
    out.emit32(0x30000000u);
    direct_jumps.push_back(out.jcc32(0x82u)); // JB

    out.emit(0x3Du); // CMP EAX, 0x32000000
    out.emit32(0x32000000u);
    alias_jumps.push_back(out.jcc32(0x82u)); // JB

    out.emit(0x3Du); // CMP EAX, 0x80000000
    out.emit32(0x80000000u);
    direct_jumps.push_back(out.jcc32(0x82u)); // JB

    // KSEG0/KSEG1.
    out.emit(0x25u); // AND EAX, imm32
    out.emit32(0x1FFFFFFFu);
    direct_jumps.push_back(out.jmp32());

    const std::size_t alias_label = out.bytes.size();
    out.emit(0x25u); // 0x2/0x3 accelerated aliases use low 25 bits.
    out.emit32(0x01FFFFFFu);

    const std::size_t mapped_label = out.bytes.size();
    for (const std::size_t jump : direct_jumps) {
        out.patch_rel32(jump, mapped_label);
    }
    for (const std::size_t jump : alias_jumps) {
        out.patch_rel32(jump, alias_label);
    }

    if (opcode == 0x1Eu) {
        out.emit(0x25u); // LQ aligns the effective address down to 16 bytes.
        out.emit32(0xFFFFFFF0u);
    }

    out.emit(0x3Du); // CMP EAX, last legal starting byte
    out.emit32(kEeRamSize - width);
    fail_jumps.push_back(out.jcc32(0x87u)); // JA

    if (opcode == 0x31u) {
        out.emit(0x41u); out.emit(0x8Bu); // MOV EAX,dword [R11+RAX]
        out.emit(0x04u); out.emit(0x03u);
        out.store_state_eax(
            static_cast<u32>(
                offsetof(EeCpuState, fpr) +
                rt * sizeof(u32)));
    } else if (opcode == 0x1Eu) {
        if (rt != 0u) {
            out.emit(0x49u); out.emit(0x8Bu); // MOV RAX,[R11+RAX]
            out.emit(0x04u); out.emit(0x03u);
            out.store_rax(rt);
            out.load_rax(rs, true);
            out.emit(0x05u);
            out.emit32(static_cast<u32>(static_cast<s32>(immediate)));
            // Recreate and align the address after RAX was used for data.
            out.emit(0x25u);
            out.emit32(0xFFFFFFF0u);
            // Address aliases were already proven by the guard. Reapply the
            // direct physical mapping used above.
            out.emit(0x25u);
            out.emit32(0x01FFFFFFu);
            out.emit(0x49u); out.emit(0x8Bu);
            out.emit(0x44u); out.emit(0x03u); out.emit(0x08u);
            out.store_rax_hi(rt);
        }
    } else if (rt != 0u) {
        switch (opcode) {
        case 0x20u: // MOVSX RAX, byte [R11+RAX]
            out.emit(0x49u); out.emit(0x0Fu); out.emit(0xBEu);
            out.emit(0x04u); out.emit(0x03u);
            break;
        case 0x24u: // MOVZX EAX, byte [R11+RAX]
            out.emit(0x41u); out.emit(0x0Fu); out.emit(0xB6u);
            out.emit(0x04u); out.emit(0x03u);
            break;
        case 0x21u: // MOVSX RAX, word [R11+RAX]
            out.emit(0x49u); out.emit(0x0Fu); out.emit(0xBFu);
            out.emit(0x04u); out.emit(0x03u);
            break;
        case 0x25u: // MOVZX EAX, word [R11+RAX]
            out.emit(0x41u); out.emit(0x0Fu); out.emit(0xB7u);
            out.emit(0x04u); out.emit(0x03u);
            break;
        case 0x23u: // MOVSXD RAX, dword [R11+RAX]
        case 0x30u:
            out.emit(0x49u); out.emit(0x63u);
            out.emit(0x04u); out.emit(0x03u);
            break;
        case 0x27u: // MOV EAX, dword [R11+RAX]
            out.emit(0x41u); out.emit(0x8Bu);
            out.emit(0x04u); out.emit(0x03u);
            break;
        case 0x34u: // MOV RAX, qword [R11+RAX]
        case 0x37u:
            out.emit(0x49u); out.emit(0x8Bu);
            out.emit(0x04u); out.emit(0x03u);
            break;
        default:
            return false;
        }
        out.store_rax(rt);
    }

    const std::size_t done_jump = out.jmp32();

    const std::size_t scratch_label = out.bytes.size();
    for (const std::size_t jump : scratch_jumps) {
        out.patch_rel32(jump, scratch_label);
    }
    out.emit(0x4Du); out.emit(0x85u); out.emit(0xC9u); // TEST R9,R9
    const std::size_t scratch_missing = out.jcc32(0x84u); // JZ
    out.call_state_scratch_instruction_helper(
        &ee_jit_scratch_memory_helper,
        instruction);
    const std::size_t scratch_done = out.jmp32();

    const std::size_t fail_label = out.bytes.size();
    if (rollback_branch) {
        out.store_state_imm32(
            static_cast<u32>(offsetof(EeCpuState, pc)),
            rollback_pc);
        out.store_state_imm32(
            static_cast<u32>(offsetof(EeCpuState, next_pc)),
            rollback_pc + 4u);
    }
    out.emit(0xB8u); // MOV EAX, retired_before
    out.emit32(retired_before);
    out.emit(0xC3u); // RET
    const std::size_t done_label = out.bytes.size();

    for (const std::size_t jump : fail_jumps) {
        out.patch_rel32(jump, fail_label);
    }
    out.patch_rel32(scratch_missing, fail_label);
    out.patch_rel32(done_jump, done_label);
    out.patch_rel32(scratch_done, done_label);
    return true;
}

u32 ram_physical_address(u32 address) {
    if (address < 0x20000000u) return address;
    if (address < 0x22000000u) return address & 0x01FFFFFFu;
    if (address < 0x30000000u) return address;
    if (address < 0x32000000u) return address & 0x01FFFFFFu;
    if (address < 0x80000000u) return address;
    return address & 0x1FFFFFFFu;
}

bool emit_guarded_ram_store(
    u32 instruction,
    u32 retired_before,
    u32 code_page,
    Emitter& out) {
    const u32 opcode = instruction >> 26;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const s16 immediate =
        static_cast<s16>(instruction & 0xFFFFu);

    u32 width = 0;
    switch (opcode) {
    case 0x28u: width = 1u; break; // SB
    case 0x29u: width = 2u; break; // SH
    case 0x2Bu: width = 4u; break; // SW
    case 0x38u: width = 4u; break; // SC
    case 0x39u: width = 4u; break; // SWC1
    case 0x3Cu: width = 8u; break; // SCD
    case 0x3Fu: width = 8u; break; // SD
    case 0x1Fu: width = 16u; break; // SQ
    default: return false;
    }

    out.load_rax(rs, true);
    out.emit(0x05u); // ADD EAX, imm32
    out.emit32(static_cast<u32>(static_cast<s32>(immediate)));

    std::vector<std::size_t> fail_jumps;
    std::vector<std::size_t> direct_jumps;
    std::vector<std::size_t> alias_jumps;
    std::vector<std::size_t> scratch_jumps;

    out.emit(0x3Du); out.emit32(kEeScratchBase);
    const std::size_t below_scratch = out.jcc32(0x82u); // JB
    out.emit(0x3Du);
    out.emit32(
        kEeScratchBase + kEeScratchSize -
        (opcode == 0x1Fu ? 1u : width));
    scratch_jumps.push_back(out.jcc32(0x86u)); // JBE
    const std::size_t main_mapping_label = out.bytes.size();
    out.patch_rel32(below_scratch, main_mapping_label);

    out.emit(0x3Du);
    out.emit32(0xC0000000u);
    fail_jumps.push_back(out.jcc32(0x83u)); // JAE

    out.emit(0x3Du);
    out.emit32(0x20000000u);
    direct_jumps.push_back(out.jcc32(0x82u)); // JB

    out.emit(0x3Du);
    out.emit32(0x22000000u);
    alias_jumps.push_back(out.jcc32(0x82u)); // JB

    out.emit(0x3Du);
    out.emit32(0x30000000u);
    direct_jumps.push_back(out.jcc32(0x82u)); // JB

    out.emit(0x3Du);
    out.emit32(0x32000000u);
    alias_jumps.push_back(out.jcc32(0x82u)); // JB

    out.emit(0x3Du);
    out.emit32(0x80000000u);
    direct_jumps.push_back(out.jcc32(0x82u)); // JB

    out.emit(0x25u); // AND EAX, KSEG physical mask
    out.emit32(0x1FFFFFFFu);
    direct_jumps.push_back(out.jmp32());

    const std::size_t alias_label = out.bytes.size();
    out.emit(0x25u);
    out.emit32(0x01FFFFFFu);

    const std::size_t mapped_label = out.bytes.size();
    for (const std::size_t jump : direct_jumps) {
        out.patch_rel32(jump, mapped_label);
    }
    for (const std::size_t jump : alias_jumps) {
        out.patch_rel32(jump, alias_label);
    }

    out.emit(0x3Du);
    out.emit32(kEeRamSize - width);
    fail_jumps.push_back(out.jcc32(0x87u)); // JA

    // SQ/LQ semantics align down; scalar stores retain alignment checks.
    if (opcode == 0x1Fu) {
        out.emit(0x25u);
        out.emit32(0xFFFFFFF0u);
    } else if (width > 1u) {
        out.emit(0xA9u); // TEST EAX, imm32
        out.emit32(width - 1u);
        fail_jumps.push_back(out.jcc32(0x85u)); // JNZ
    }

    out.emit(0x41u); out.emit(0x89u); out.emit(0xC0u); // MOV R8D,EAX
    if (opcode == 0x39u) {
        out.load_state_eax(
            static_cast<u32>(
                offsetof(EeCpuState, fpr) +
                rt * sizeof(u32)));
        out.emit(0x43u); out.emit(0x89u);
        out.emit(0x04u); out.emit(0x03u); // MOV [R11+R8],EAX
    } else if (opcode == 0x1Fu) {
        out.load_rdx(rt, false);
        out.emit(0x4Bu); out.emit(0x89u);
        out.emit(0x14u); out.emit(0x03u); // MOV [R11+R8],RDX
        out.load_rdx_hi(rt);
        out.emit(0x4Bu); out.emit(0x89u);
        out.emit(0x54u); out.emit(0x03u); out.emit(0x08u);
    } else {
        out.load_rdx(rt, width != 8u);
        switch (width) {
        case 1u:
            out.emit(0x43u); out.emit(0x88u);
            out.emit(0x14u); out.emit(0x03u); // MOV [R11+R8],DL
            break;
        case 2u:
            out.emit(0x66u); out.emit(0x43u); out.emit(0x89u);
            out.emit(0x14u); out.emit(0x03u); // MOV [R11+R8],DX
            break;
        case 4u:
            out.emit(0x43u); out.emit(0x89u);
            out.emit(0x14u); out.emit(0x03u); // MOV [R11+R8],EDX
            break;
        case 8u:
            out.emit(0x4Bu); out.emit(0x89u);
            out.emit(0x14u); out.emit(0x03u); // MOV [R11+R8],RDX
            break;
        default:
            return false;
        }
    }

    if ((opcode == 0x38u || opcode == 0x3Cu) && rt != 0u) {
        out.store_gpr_imm64(rt, 1u);
    }

    // Fine-grained self-modifying-code barrier. EeRam packs an 8-bit
    // translated-region mask into the high byte of each 4 KiB page metadata
    // word. Only stores touching one of those 512-byte regions advance the
    // generation; ordinary data/stack writes on the same page remain native.
    //
    // R8D still contains the physical byte address. Recreate it in EAX even
    // for SWC1, whose data load used EAX above.
    out.emit(0x44u); out.emit(0x89u); out.emit(0xC0u); // MOV EAX,R8D
    out.emit(0xC1u); out.emit(0xE8u);
    out.emit(static_cast<u8>(EeRam::kCodeRegionShift)); // SHR EAX,9
    out.emit(0x83u); out.emit(0xE0u);
    out.emit(static_cast<u8>(EeRam::kCodeRegionCount - 1u)); // AND EAX,7
    out.emit(0x83u); out.emit(0xC0u);
    out.emit(static_cast<u8>(EeRam::kCodeMaskShift)); // ADD EAX,24

    out.emit(0x41u); out.emit(0xC1u); out.emit(0xE8u); out.emit(0x0Cu);
    out.emit(0x43u); out.emit(0x8Bu); out.emit(0x14u);
    out.emit(0x82u); // MOV EDX,[R10+R8*4]
    out.emit(0x0Fu); out.emit(0xA3u); out.emit(0xC2u); // BT EDX,EAX
    const std::size_t not_tracked_region = out.jcc32(0x83u); // JNC

    out.emit(0x43u); out.emit(0x83u); out.emit(0x04u);
    out.emit(0x82u);
    out.emit(opcode == 0x1Fu ? 0x02u : 0x01u);
    // SQ mirrors the interpreter's two write64 calls, hence +2 generation.

    // Only a write which actually overlaps translated code on this page can
    // require an immediate exit from the current translation.
    out.emit(0x41u); out.emit(0x81u); out.emit(0xF8u);
    out.emit32(code_page);
    const std::size_t not_code_page = out.jcc32(0x85u); // JNE
    out.emit(0xB8u);
    out.emit32(retired_before + 1u);
    out.emit(0xC3u);
    const std::size_t barrier_done = out.bytes.size();
    out.patch_rel32(not_code_page, barrier_done);
    out.patch_rel32(not_tracked_region, barrier_done);

    const std::size_t continue_jump = out.jmp32();

    const std::size_t scratch_label = out.bytes.size();
    for (const std::size_t jump : scratch_jumps) {
        out.patch_rel32(jump, scratch_label);
    }
    out.emit(0x4Du); out.emit(0x85u); out.emit(0xC9u); // TEST R9,R9
    const std::size_t scratch_missing = out.jcc32(0x84u);
    out.call_state_scratch_instruction_helper(
        &ee_jit_scratch_memory_helper,
        instruction);
    const std::size_t scratch_done = out.jmp32();

    const std::size_t fail_label = out.bytes.size();
    out.emit(0xB8u);
    out.emit32(retired_before);
    out.emit(0xC3u);
    const std::size_t continue_label = out.bytes.size();

    for (const std::size_t jump : fail_jumps) {
        out.patch_rel32(jump, fail_label);
    }
    out.patch_rel32(scratch_missing, fail_label);
    out.patch_rel32(continue_jump, continue_label);
    out.patch_rel32(scratch_done, continue_label);
    return true;
}

bool branch_likely_instruction(u32 instruction) {
    const u32 opcode = instruction >> 26;
    if (opcode >= 0x14u && opcode <= 0x17u) return true;
    if (opcode == 0x01u) {
        const u32 variant = (instruction >> 16) & 31u;
        return variant == 0x02u || variant == 0x03u ||
               variant == 0x12u || variant == 0x13u;
    }
    if (opcode == 0x11u &&
        ((instruction >> 21) & 31u) == 0x08u) {
        return (((instruction >> 16) & 3u) & 2u) != 0u;
    }
    return false;
}

bool emit_branch_and_delay(
    u32 branch_pc,
    u32 retired_before,
    u32 branch_instruction,
    u32 delay_instruction,
    Emitter& out) {
    const std::size_t before = out.bytes.size();

    Emitter delay_probe;
    const bool delay_is_body =
        emit_instruction_body(delay_instruction, delay_probe);
    if (!delay_is_body) {
        delay_probe.bytes.clear();
    }
    const bool delay_is_ram_load =
        !delay_is_body &&
        emit_guarded_ram_load(
            delay_instruction,
            retired_before,
            delay_probe,
            true,
            branch_pc);
    if (!delay_is_body && !delay_is_ram_load) {
        return false;
    }

    const auto emit_delay = [&]() -> bool {
        if (delay_is_body) {
            return emit_instruction_body(delay_instruction, out);
        }
        return emit_guarded_ram_load(
            delay_instruction,
            retired_before,
            out,
            true,
            branch_pc);
    };

    const u32 opcode = branch_instruction >> 26;
    const u32 rs = (branch_instruction >> 21) & 31u;
    const u32 rt = (branch_instruction >> 16) & 31u;
    const s16 imm = static_cast<s16>(branch_instruction & 0xFFFFu);
    const u32 pc_offset = static_cast<u32>(offsetof(EeCpuState, pc));
    const u32 next_pc_offset =
        static_cast<u32>(offsetof(EeCpuState, next_pc));
    const u32 fallthrough = branch_pc + 8u;
    const u32 target = branch_pc + 4u +
        static_cast<u32>(static_cast<s32>(imm) * 4);

    if (opcode == 0x00u) {
        const u32 funct = branch_instruction & 63u;
        if (funct == 0x08u || funct == 0x09u) { // JR / JALR
            const u32 rd = (branch_instruction >> 11) & 31u;
            if (delay_is_ram_load && funct == 0x09u) {
                out.bytes.resize(before);
                return false;
            }
            out.load_rax(rs, true);
            out.store_state_eax(pc_offset);
            if (funct == 0x09u && rd != 0u) {
                const u64 link = static_cast<u64>(static_cast<s64>(
                    static_cast<s32>(branch_pc + 8u)));
                out.store_gpr_imm64(rd, link);
            }
            if (!emit_delay()) {
                out.bytes.resize(before);
                return false;
            }
            out.load_state_eax(pc_offset);
            out.emit(0x05u);
            out.emit32(4u);
            out.store_state_eax(next_pc_offset);
            return true;
        }
    }

    if (opcode == 0x01u) {
        const u32 variant = rt;
        const bool bltz =
            variant == 0x00u || variant == 0x02u ||
            variant == 0x10u || variant == 0x12u;
        const bool bgez =
            variant == 0x01u || variant == 0x03u ||
            variant == 0x11u || variant == 0x13u;
        const bool likely =
            variant == 0x02u || variant == 0x03u ||
            variant == 0x12u || variant == 0x13u;
        const bool link =
            variant == 0x10u || variant == 0x11u ||
            variant == 0x12u || variant == 0x13u;
        if (!bltz && !bgez) {
            out.bytes.resize(before);
            return false;
        }

        out.load_rax(rs, false);
        out.emit(0x48u);
        out.emit(0x85u);
        out.emit(0xC0u); // TEST RAX,RAX

        // Link variants write r31 even when a likely branch is annulled,
        // matching EeCpu::execute_regimm().
        if (link && delay_is_ram_load) {
            out.bytes.resize(before);
            return false;
        }
        if (link) {
            const u64 link_value = static_cast<u64>(static_cast<s64>(
                static_cast<s32>(branch_pc + 8u)));
            out.store_gpr_imm64(31u, link_value);
        }

        if (likely) {
            const std::size_t not_taken =
                out.jcc32(bltz ? 0x89u : 0x88u); // JNS / JS
            out.store_state_imm32(pc_offset, target);
            if (!emit_delay()) {
                out.bytes.resize(before);
                return false;
            }
            out.load_state_eax(pc_offset);
            out.emit(0x05u);
            out.emit32(4u);
            out.store_state_eax(next_pc_offset);
            const std::size_t done = out.jmp32();

            const std::size_t not_taken_label = out.bytes.size();
            out.patch_rel32(not_taken, not_taken_label);
            out.store_state_imm32(pc_offset, fallthrough);
            out.store_state_imm32(next_pc_offset, fallthrough + 4u);
            out.emit(0xB8u); // MOV EAX,1: annulled delay did not retire.
            out.emit32(retired_before + 1u);
            out.emit(0xC3u);
            out.patch_rel32(done, out.bytes.size());
            return true;
        }

        out.store_state_imm32(pc_offset, fallthrough);
        const std::size_t skip_target =
            out.jcc32(bltz ? 0x89u : 0x88u); // JNS / JS
        out.store_state_imm32(pc_offset, target);
        out.patch_rel32(skip_target, out.bytes.size());

        if (!emit_delay()) {
            out.bytes.resize(before);
            return false;
        }
        out.load_state_eax(pc_offset);
        out.emit(0x05u);
        out.emit32(4u);
        out.store_state_eax(next_pc_offset);
        return true;
    }

    if (opcode == 0x11u && rs == 0x08u) { // BC1F/T/FL/TL
        const u32 variant = rt & 3u;
        const bool likely = (variant & 2u) != 0u;
        const bool take_when_set = (variant & 1u) != 0u;
        out.load_state_eax(
            static_cast<u32>(
                offsetof(EeCpuState, fcr) + 31u * sizeof(u32)));
        out.emit(0xA9u); // TEST EAX, FPU condition bit
        out.emit32(0x00800000u);

        if (likely) {
            const std::size_t not_taken =
                out.jcc32(take_when_set ? 0x84u : 0x85u);
            out.store_state_imm32(pc_offset, target);
            if (!emit_delay()) {
                out.bytes.resize(before);
                return false;
            }
            out.load_state_eax(pc_offset);
            out.emit(0x05u); out.emit32(4u);
            out.store_state_eax(next_pc_offset);
            const std::size_t done = out.jmp32();
            const std::size_t not_taken_label = out.bytes.size();
            out.patch_rel32(not_taken, not_taken_label);
            out.store_state_imm32(pc_offset, fallthrough);
            out.store_state_imm32(next_pc_offset, fallthrough + 4u);
            out.emit(0xB8u); out.emit32(retired_before + 1u); out.emit(0xC3u);
            out.patch_rel32(done, out.bytes.size());
            return true;
        }

        out.store_state_imm32(pc_offset, fallthrough);
        const std::size_t skip_target =
            out.jcc32(take_when_set ? 0x84u : 0x85u);
        out.store_state_imm32(pc_offset, target);
        out.patch_rel32(skip_target, out.bytes.size());
        if (!emit_delay()) {
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
        const u32 jump_target =
            ((branch_pc + 4u) & 0xF0000000u) |
            ((branch_instruction & 0x03FFFFFFu) << 2);
        if (opcode == 0x03u) {
            const u64 link = static_cast<u64>(static_cast<s64>(
                static_cast<s32>(branch_pc + 8u)));
            out.store_gpr_imm64(31u, link);
        }
        out.store_state_imm32(pc_offset, jump_target);
        break;
    }
    case 0x14u: // BEQL
    case 0x15u: { // BNEL
        out.load_rax(rs, false);
        out.load_rdx(rt, false);
        out.emit(0x48u); out.emit(0x39u); out.emit(0xD0u);
        const std::size_t not_taken =
            out.jcc32(opcode == 0x14u ? 0x85u : 0x84u);
        out.store_state_imm32(pc_offset, target);
        if (!emit_delay()) {
            out.bytes.resize(before);
            return false;
        }
        out.load_state_eax(pc_offset);
        out.emit(0x05u); out.emit32(4u);
        out.store_state_eax(next_pc_offset);
        const std::size_t done = out.jmp32();
        const std::size_t not_taken_label = out.bytes.size();
        out.patch_rel32(not_taken, not_taken_label);
        out.store_state_imm32(pc_offset, fallthrough);
        out.store_state_imm32(next_pc_offset, fallthrough + 4u);
        out.emit(0xB8u); out.emit32(retired_before + 1u); out.emit(0xC3u);
        out.patch_rel32(done, out.bytes.size());
        return true;
    }
    case 0x16u: // BLEZL
    case 0x17u: { // BGTZL
        out.load_rax(rs, false);
        out.emit(0x48u); out.emit(0x85u); out.emit(0xC0u);
        const std::size_t not_taken =
            out.jcc32(opcode == 0x16u ? 0x8Fu : 0x8Eu); // JG / JLE
        out.store_state_imm32(pc_offset, target);
        if (!emit_delay()) {
            out.bytes.resize(before);
            return false;
        }
        out.load_state_eax(pc_offset);
        out.emit(0x05u); out.emit32(4u);
        out.store_state_eax(next_pc_offset);
        const std::size_t done = out.jmp32();
        const std::size_t not_taken_label = out.bytes.size();
        out.patch_rel32(not_taken, not_taken_label);
        out.store_state_imm32(pc_offset, fallthrough);
        out.store_state_imm32(next_pc_offset, fallthrough + 4u);
        out.emit(0xB8u); out.emit32(retired_before + 1u); out.emit(0xC3u);
        out.patch_rel32(done, out.bytes.size());
        return true;
    }
    case 0x04u: // BEQ
    case 0x05u: { // BNE
        out.store_state_imm32(pc_offset, fallthrough);
        out.load_rax(rs, false);
        out.load_rdx(rt, false);
        out.emit(0x48u);
        out.emit(0x39u);
        out.emit(0xD0u); // CMP RAX,RDX
        const std::size_t skip_target =
            out.jcc32(opcode == 0x04u ? 0x85u : 0x84u);
        out.store_state_imm32(pc_offset, target);
        out.patch_rel32(skip_target, out.bytes.size());
        break;
    }
    case 0x06u: // BLEZ
    case 0x07u: { // BGTZ
        out.store_state_imm32(pc_offset, fallthrough);
        out.load_rax(rs, false);
        out.emit(0x48u);
        out.emit(0x85u);
        out.emit(0xC0u); // TEST RAX,RAX
        const std::size_t skip_target =
            out.jcc32(opcode == 0x06u ? 0x8Fu : 0x8Eu);
        out.store_state_imm32(pc_offset, target);
        out.patch_rel32(skip_target, out.bytes.size());
        break;
    }
    default:
        out.bytes.resize(before);
        return false;
    }

    if (!emit_delay()) {
        out.bytes.resize(before);
        return false;
    }

    out.load_state_eax(pc_offset);
    out.emit(0x05u); // ADD EAX, imm32
    out.emit32(4u);
    out.store_state_eax(next_pc_offset);
    return true;
}

bool emit_block(
    u32 pc,
    const u32* instructions,
    u32 instruction_count,
    Emitter& out,
    u32& compiled_instructions,
    bool& control_flow,
    bool& uses_ram) {
    compiled_instructions = 0;
    control_flow = false;
    uses_ram = false;
    out.preserve_ram_base();
    out.preserve_generation_base();
    out.preserve_scratch_base();
    for (u32 i = 0; i < instruction_count; ++i) {
        const std::size_t before = out.bytes.size();
        if (emit_instruction_body(instructions[i], out)) {
            ++compiled_instructions;
            continue;
        }

        out.bytes.resize(before);
        if (emit_guarded_ram_load(
                instructions[i], compiled_instructions, out)) {
            ++compiled_instructions;
            uses_ram = true;
            continue;
        }

        out.bytes.resize(before);
        const u32 code_page =
            ram_physical_address(pc) / 4096u;
        if (emit_guarded_ram_store(
                instructions[i],
                compiled_instructions,
                code_page,
                out)) {
            ++compiled_instructions;
            uses_ram = true;
            continue;
        }

        out.bytes.resize(before);
        if (i + 1u < instruction_count &&
            emit_branch_and_delay(
                pc + i * 4u,
                i,
                instructions[i],
                instructions[i + 1u],
                out)) {
            compiled_instructions += 2u;
            control_flow = true;
        }
        break;
    }
    if (compiled_instructions == 0u) return false;
    out.emit(0xB8u); // MOV EAX, compiled instruction count
    out.emit32(compiled_instructions);
    out.emit(0xC3u); // RET
    return true;
}
#endif

} // namespace

EeJit::~EeJit() { release_code_cache(); }

void EeJit::release_code_cache() {
#if defined(VIBESTATION_EE_JIT_X64)
    for (const Page& page : pages_) release_page(page.address);
#endif
    pages_.clear();
    entries_ = {};
    std::fill(block_entries_.begin(), block_entries_.end(), BlockEntry{});
}

void EeJit::clear() {
    release_code_cache();
    compiled_count_ = 0;
    executed_count_ = 0;
    block_compiled_count_ = 0;
    block_executed_count_ = 0;
    block_instruction_count_ = 0;
    block_fastmem_load_count_ = 0;
    block_guard_bailout_count_ = 0;
    block_fastmem_store_count_ = 0;
    block_code_store_exit_count_ = 0;
    cache_flush_count_ = 0;
}

EeJit::Function EeJit::compile(u32 instruction) {
#if defined(VIBESTATION_EE_JIT_X64)
    Emitter emitter;
    if (!emit_instruction(instruction, emitter)) return nullptr;
    if (pages_.empty() ||
        pages_.back().used + emitter.bytes.size() > kPageSize) {
        if (pages_.size() >= kMaxPages) { release_code_cache(); ++cache_flush_count_; }
        void* address = allocate_page();
        if (address == nullptr) return nullptr;
        pages_.push_back(Page{address, 0});
    }
    Page& page = pages_.back();
    if (page.used != 0 && !protect_page(page.address, false)) return nullptr;
    auto* code = static_cast<u8*>(page.address) + page.used;
    std::memcpy(code, emitter.bytes.data(), emitter.bytes.size());
    if (!protect_page(page.address, true)) return nullptr;
    flush_code(code, emitter.bytes.size());
    page.used += emitter.bytes.size();
    ++compiled_count_;
    return reinterpret_cast<Function>(code);
#else
    (void)instruction;
    return nullptr;
#endif
}

EeJit::BlockFunction EeJit::compile_block(
    u32 pc,
    const u32* instructions,
    u32 instruction_count,
    u32& compiled_instructions,
    bool& control_flow,
    bool& uses_ram) {
#if defined(VIBESTATION_EE_JIT_X64)
    Emitter emitter;
    if (!emit_block(
            pc,
            instructions,
            instruction_count,
            emitter,
            compiled_instructions,
            control_flow,
            uses_ram)) {
        return nullptr;
    }

    if (pages_.empty() ||
        pages_.back().used + emitter.bytes.size() > kPageSize) {
        if (pages_.size() >= kMaxPages) { release_code_cache(); ++cache_flush_count_; }
        void* address = allocate_page();
        if (address == nullptr) return nullptr;
        pages_.push_back(Page{address, 0});
    }

    Page& page = pages_.back();
    if (page.used != 0 && !protect_page(page.address, false)) return nullptr;
    auto* code = static_cast<u8*>(page.address) + page.used;
    std::memcpy(code, emitter.bytes.data(), emitter.bytes.size());
    if (!protect_page(page.address, true)) return nullptr;
    flush_code(code, emitter.bytes.size());
    page.used += emitter.bytes.size();
    ++block_compiled_count_;
    return reinterpret_cast<BlockFunction>(code);
#else
    (void)pc;
    (void)instructions;
    (void)instruction_count;
    compiled_instructions = 0;
    control_flow = false;
    uses_ram = false;
    return nullptr;
#endif
}

u32 EeJit::execute_block(
    EeCpuState& state,
    u32 pc,
    u32 page_generation,
    const u32* instructions,
    u32 instruction_count,
    u32 maximum_instructions,
    const u8* ram_data,
    u32* page_generations,
    bool& control_flow,
    u32 yield_pc,
    u8* scratchpad_data) {
#if defined(VIBESTATION_EE_JIT_X64)
    if (instructions == nullptr ||
        instruction_count == 0u ||
        maximum_instructions == 0u) {
        control_flow = false;
        return 0;
    }

    auto block_entry = [&](u32 block_pc,
                           u32 generation,
                           const u32* words,
                           u32 word_count) -> BlockEntry* {
        const u32 hash =
            (block_pc >> 2) * 2654435761u ^
            generation * 2246822519u;
        const std::size_t index =
            (static_cast<std::size_t>(hash) *
             block_entries_.size()) >> 32;
        BlockEntry& entry = block_entries_[index];

        if (!entry.known ||
            entry.pc != block_pc ||
            entry.page_generation != generation) {
            u32 compiled_instructions = 0;
            bool compiled_control_flow = false;
            bool compiled_uses_ram = false;
            BlockFunction function = compile_block(
                block_pc,
                words,
                word_count,
                compiled_instructions,
                compiled_control_flow,
                compiled_uses_ram);
            if (page_generations != nullptr &&
                compiled_instructions != 0u) {
                const u32 block_physical =
                    ram_physical_address(block_pc);
                if (block_physical < kEeRamSize) {
                    EeRam::track_jit_code(
                        page_generations,
                        block_physical,
                        static_cast<std::size_t>(
                            compiled_instructions) * sizeof(u32));
                }
            }
            entry.pc = block_pc;
            entry.page_generation = generation;
            entry.instruction_count =
                static_cast<u8>(compiled_instructions);
            entry.function = function;
            entry.control_flow = compiled_control_flow;
            entry.annul_capable =
                compiled_control_flow &&
                compiled_instructions >= 2u &&
                branch_likely_instruction(
                    words[compiled_instructions - 2u]);
            entry.uses_ram = compiled_uses_ram;
            entry.ram_load_mask = 0u;
            entry.ram_store_mask = 0u;
            entry.guard_bail_streak = 0u;
            entry.guard_skip_remaining = 0u;
            for (u32 i = 0;
                 i < compiled_instructions && i < 32u;
                 ++i) {
                switch (words[i] >> 26) {
                case 0x1Eu:
                case 0x20u:
                case 0x21u:
                case 0x23u:
                case 0x24u:
                case 0x25u:
                case 0x27u:
                case 0x30u:
                case 0x31u:
                case 0x34u:
                case 0x37u:
                    entry.ram_load_mask |= 1u << i;
                    break;
                case 0x1Fu:
                case 0x28u:
                case 0x29u:
                case 0x2Bu:
                case 0x38u:
                case 0x39u:
                case 0x3Cu:
                case 0x3Fu:
                    entry.ram_store_mask |= 1u << i;
                    break;
                default:
                    break;
                }
            }
            entry.known = true;
        }
        return &entry;
    };

    // The system layer has already reduced maximum_instructions to the next
    // observable EE boundary (scheduler event, timer/Compare edge, IOP step,
    // video transition, DMA completion, or caller budget). Keep execution
    // resident in the JIT across basic blocks until that deadline or until an
    // unsupported/MMIO/self-modifying-code boundary forces a real exit.
    //
    // Chaining is deliberately confined to the current tracked 4 KiB code
    // page: EeRam's generation tracking makes that page a safe coherency
    // domain without widening the invalidation contract.
    const u32 origin_physical = ram_physical_address(pc);
    u32 current_code_page = origin_physical >> 12;
    u32 current_page_generation = page_generation;
    u32 total_retired = 0u;
    u32 current_pc = pc;
    const u32* current_words = instructions;
    u32 current_count = instruction_count;
    u32 fetched_words[32]{};
    bool final_control_flow = false;

    while (total_retired < maximum_instructions) {
        if (total_retired != 0u &&
            yield_pc != 0u &&
            current_pc == yield_pc) {
            break;
        }

        BlockEntry* entry = block_entry(
            current_pc,
            current_page_generation,
            current_words,
            current_count);
        if (entry == nullptr ||
            entry->function == nullptr ||
            entry->instruction_count == 0u ||
            entry->instruction_count >
                maximum_instructions - total_retired ||
            (entry->uses_ram && ram_data == nullptr) ||
            (entry->ram_store_mask != 0u &&
             page_generations == nullptr)) {
            break;
        }

        // A hot block which repeatedly proves that its dynamic memory
        // operand is MMIO/non-RAM is temporarily kept on the interpreter.
        // This is the same basic adaptive idea used by the mature PS1 V4
        // backend: do not pay a native entry + guard + exit on every visit
        // when the guard has already failed several times in a row.
        if (entry->guard_skip_remaining != 0u) {
            --entry->guard_skip_remaining;
            break;
        }

        const u32 retired =
            entry->function(
                &state,
                ram_data,
                page_generations,
                scratchpad_data);
        if (retired > entry->instruction_count) {
            break;
        }

        if (retired == 0u) {
            const bool first_is_guarded_memory =
                ((entry->ram_load_mask | entry->ram_store_mask) & 1u) != 0u;
            if (first_is_guarded_memory) {
                ++block_guard_bailout_count_;
                if (++entry->guard_bail_streak >= 4u) {
                    entry->guard_bail_streak = 0u;
                    entry->guard_skip_remaining = 32u;
                }
            }
            break;
        }

        const bool full_block =
            retired == entry->instruction_count;
        const bool annulled_control =
            entry->control_flow &&
            entry->annul_capable &&
            !full_block &&
            retired + 1u == entry->instruction_count;
        final_control_flow =
            entry->control_flow && (full_block || annulled_control);

        ++block_executed_count_;
        block_instruction_count_ += retired;
        if (full_block || annulled_control) {
            entry->guard_bail_streak = 0u;
        } else {
            const bool guard_failure =
                retired < 32u &&
                (((entry->ram_load_mask | entry->ram_store_mask) >>
                  retired) & 1u) != 0u;
            if (guard_failure) {
                ++block_guard_bailout_count_;
                if (++entry->guard_bail_streak >= 4u) {
                    entry->guard_bail_streak = 0u;
                    entry->guard_skip_remaining = 32u;
                }
            } else {
                entry->guard_bail_streak = 0u;
            }
            if ((entry->ram_store_mask &
                 (1u << (retired - 1u))) != 0u) {
                ++block_code_store_exit_count_;
            }
        }
        const u32 retired_mask =
            retired >= 32u
                ? 0xFFFFFFFFu
                : ((1u << retired) - 1u);
        block_fastmem_load_count_ +=
            std::popcount(
                entry->ram_load_mask & retired_mask);
        block_fastmem_store_count_ +=
            std::popcount(
                entry->ram_store_mask & retired_mask);

        state.last_pc =
            current_pc + (retired - 1u) * 4u;
        state.last_instruction =
            current_words[retired - 1u];

        if (!final_control_flow) {
            state.pc = current_pc + retired * 4u;
            state.next_pc = state.pc + 4u;
        }

        total_retired += retired;

        if ((!full_block && !annulled_control) ||
            total_retired >= maximum_instructions) {
            break;
        }

        // A native store to this code page increments its generation and
        // returns from generated code. Never execute another cached block
        // from the old generation in the same host dispatch.
        if (page_generations != nullptr &&
            EeRam::generation_from_metadata(
                page_generations[current_code_page]) !=
                current_page_generation) {
            break;
        }

        const u32 next_pc = state.pc;
        if (ram_data == nullptr ||
            next_pc >= 0xC0000000u ||
            (next_pc & 3u) != 0u) {
            break;
        }
        const u32 next_physical =
            ram_physical_address(next_pc);
        if (next_physical >= kEeRamSize) {
            break;
        }

        const u32 next_code_page = next_physical >> 12;
        if (next_code_page != current_code_page) {
            // Cross-page chaining is safe when page-generation metadata is
            // available: each compiled block is keyed by its own generation
            // and track_jit_code() marks only the translated regions on that
            // page. This removes an arbitrary 4 KiB dispatcher boundary.
            if (page_generations == nullptr) break;
            current_code_page = next_code_page;
            current_page_generation =
                EeRam::generation_from_metadata(
                    page_generations[current_code_page]);
        }

        const u32 words_to_page_end =
            (4096u - (next_physical & 4095u)) / 4u;
        const u32 remaining =
            maximum_instructions - total_retired;
        const u32 page_limited =
            words_to_page_end < remaining
                ? words_to_page_end
                : remaining;
        current_count =
            page_limited < 32u ? page_limited : 32u;
        if (current_count == 0u) break;

        for (u32 i = 0; i < current_count; ++i) {
            std::memcpy(
                &fetched_words[i],
                ram_data + next_physical + i * 4u,
                sizeof(u32));
        }
        current_pc = next_pc;
        current_words = fetched_words;
    }

    control_flow = final_control_flow;
    return total_retired;
#else
    (void)state;
    (void)pc;
    (void)page_generation;
    (void)instructions;
    (void)instruction_count;
    (void)maximum_instructions;
    (void)ram_data;
    (void)page_generations;
    (void)yield_pc;
    control_flow = false;
    return 0;
#endif
}

bool EeJit::execute(EeCpuState& state, u32 instruction) {
#if defined(VIBESTATION_EE_JIT_X64)
    const u32 hash = instruction * 2654435761u;
    const std::size_t index = hash >> 20;
    Entry& entry = entries_[index];
    if (!entry.known || entry.instruction != instruction) {
        entry.instruction = instruction;
        entry.function = compile(instruction);
        entry.known = true;
    }
    if (entry.function == nullptr) return false;
    entry.function(&state);
    ++executed_count_;
    return true;
#else
    (void)state;
    (void)instruction;
    return false;
#endif
}

} // namespace ps2
