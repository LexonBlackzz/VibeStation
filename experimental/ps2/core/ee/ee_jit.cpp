#include "core/ee/ee_jit.h"
#include "core/ee/ee_cpu.h"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
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
#else
constexpr u8 kArgumentRegister = 7u; // RDI
constexpr u8 kRamArgumentRegister = 6u; // RSI
constexpr u8 kGenerationArgumentRegister = 2u; // RDX
#endif
constexpr u32 kEeRamSize = 32u * 1024u * 1024u;

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

bool emit_guarded_ram_load(
    u32 instruction,
    u32 retired_before,
    Emitter& out) {
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
    const std::size_t fail_label = out.bytes.size();
    out.emit(0xB8u); // MOV EAX, retired_before
    out.emit32(retired_before);
    out.emit(0xC3u); // RET
    const std::size_t done_label = out.bytes.size();

    for (const std::size_t jump : fail_jumps) {
        out.patch_rel32(jump, fail_label);
    }
    out.patch_rel32(done_jump, done_label);
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

    // Conservative write barrier: every native RAM store advances the page
    // generation. Cached code on that page will be recompiled on next entry.
    out.emit(0x41u); out.emit(0xC1u); out.emit(0xE8u); out.emit(0x0Cu);
    out.emit(0x43u); out.emit(0x83u); out.emit(0x04u);
    out.emit(0x82u);
    out.emit(opcode == 0x1Fu ? 0x02u : 0x01u);
    // SQ mirrors the interpreter's two write64 calls, so a tracked code page
    // advances twice. Scalar stores advance it once.

    // A self-modifying store must end this block immediately. The caller will
    // observe the bumped generation before fetching any later cached word.
    out.emit(0x41u); out.emit(0x81u); out.emit(0xF8u);
    out.emit32(code_page);
    const std::size_t not_code_page = out.jcc32(0x85u); // JNE
    out.emit(0xB8u);
    out.emit32(retired_before + 1u);
    out.emit(0xC3u);
    const std::size_t done_label = out.bytes.size();
    out.patch_rel32(not_code_page, done_label);

    const std::size_t continue_jump = out.jmp32();
    const std::size_t fail_label = out.bytes.size();
    out.emit(0xB8u);
    out.emit32(retired_before);
    out.emit(0xC3u);
    const std::size_t continue_label = out.bytes.size();

    for (const std::size_t jump : fail_jumps) {
        out.patch_rel32(jump, fail_label);
    }
    out.patch_rel32(continue_jump, continue_label);
    return true;
}

bool emit_branch_and_delay(
    u32 branch_pc,
    u32 branch_instruction,
    u32 delay_instruction,
    Emitter& out) {
    const std::size_t before = out.bytes.size();
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
            out.load_rax(rs, true);
            out.store_state_eax(pc_offset);
            if (funct == 0x09u && rd != 0u) {
                const u64 link = static_cast<u64>(static_cast<s64>(
                    static_cast<s32>(branch_pc + 8u)));
                out.store_gpr_imm64(rd, link);
            }
            if (!emit_instruction_body(delay_instruction, out)) {
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

    if (!emit_instruction_body(delay_instruction, out)) {
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

EeJit::~EeJit() { clear(); }

void EeJit::clear() {
#if defined(VIBESTATION_EE_JIT_X64)
    for (const Page& page : pages_) release_page(page.address);
#endif
    pages_.clear();
    entries_ = {};
    std::fill(block_entries_.begin(), block_entries_.end(), BlockEntry{});
    compiled_count_ = 0;
    executed_count_ = 0;
    block_compiled_count_ = 0;
    block_executed_count_ = 0;
    block_instruction_count_ = 0;
    block_fastmem_load_count_ = 0;
    block_guard_bailout_count_ = 0;
    block_fastmem_store_count_ = 0;
    block_code_store_exit_count_ = 0;
}

EeJit::Function EeJit::compile(u32 instruction) {
#if defined(VIBESTATION_EE_JIT_X64)
    Emitter emitter;
    if (!emit_instruction(instruction, emitter)) return nullptr;
    if (pages_.empty() ||
        pages_.back().used + emitter.bytes.size() > kPageSize) {
        if (pages_.size() >= kMaxPages) clear();
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
        if (pages_.size() >= kMaxPages) clear();
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
    bool& control_flow) {
#if defined(VIBESTATION_EE_JIT_X64)
    if (instructions == nullptr ||
        instruction_count == 0u ||
        maximum_instructions == 0u) {
        control_flow = false;
        return 0;
    }

    const u32 hash =
        (pc >> 2) * 2654435761u ^
        page_generation * 2246822519u;
    const std::size_t index =
        (static_cast<std::size_t>(hash) *
         block_entries_.size()) >> 32;
    BlockEntry& entry = block_entries_[index];

    if (!entry.known ||
        entry.pc != pc ||
        entry.page_generation != page_generation) {
        u32 compiled_instructions = 0;
        bool compiled_control_flow = false;
        bool compiled_uses_ram = false;
        BlockFunction function = compile_block(
            pc,
            instructions,
            instruction_count,
            compiled_instructions,
            compiled_control_flow,
            compiled_uses_ram);
        entry.pc = pc;
        entry.page_generation = page_generation;
        entry.instruction_count =
            static_cast<u8>(compiled_instructions);
        entry.function = function;
        entry.control_flow = compiled_control_flow;
        entry.uses_ram = compiled_uses_ram;
        entry.ram_load_mask = 0u;
        entry.ram_store_mask = 0u;
        for (u32 i = 0; i < compiled_instructions && i < 32u; ++i) {
            switch (instructions[i] >> 26) {
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

    if (entry.function == nullptr ||
        entry.instruction_count == 0u ||
        entry.instruction_count > maximum_instructions ||
        (entry.uses_ram && ram_data == nullptr) ||
        (entry.ram_store_mask != 0u && page_generations == nullptr)) {
        control_flow = false;
        return 0;
    }

    const u32 retired =
        entry.function(&state, ram_data, page_generations);
    if (retired == 0u || retired > entry.instruction_count) {
        control_flow = false;
        return 0u;
    }
    control_flow =
        entry.control_flow && retired == entry.instruction_count;
    ++block_executed_count_;
    block_instruction_count_ += retired;
    if (retired < entry.instruction_count) {
        ++block_guard_bailout_count_;
        if (retired != 0u &&
            (entry.ram_store_mask &
             (1u << (retired - 1u))) != 0u) {
            ++block_code_store_exit_count_;
        }
    }
    const u32 retired_mask =
        retired >= 32u ? 0xFFFFFFFFu : ((1u << retired) - 1u);
    block_fastmem_load_count_ +=
        std::popcount(entry.ram_load_mask & retired_mask);
    block_fastmem_store_count_ +=
        std::popcount(entry.ram_store_mask & retired_mask);
    return retired;
#else
    (void)state;
    (void)pc;
    (void)page_generation;
    (void)instructions;
    (void)instruction_count;
    (void)maximum_instructions;
    (void)ram_data;
    (void)page_generations;
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
