#include "core/ee/ee_dynarec.h"
#include "core/ee/ee_dynarec_ir.h"

#include "core/ee/ee_cpu.h"
#include "core/memory/ee_ram.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#if defined(_M_X64) || defined(__x86_64__)
#define VIBESTATION_EE_DYNAREC_X64 1
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

constexpr u32 kRamSize = static_cast<u32>(EeRam::kSize);
constexpr u32 kPageSize = EeRam::kPageSize;
constexpr u32 kMaxBlockInstructions = 64u;
constexpr std::size_t kCodePageSize = 64u * 1024u;
constexpr std::size_t kMaxCodePages = 1024u;
constexpr std::size_t kBlockCacheEntries = 65536u;

u32 compile_budget_for_limit(u32 limit) {
    limit = (std::min)(limit, kMaxBlockInstructions);
    if (limit <= 16u) return limit;
    if (limit < 32u) return 16u;
    if (limit < 64u) return 32u;
    return 64u;
}

u32 ram_physical(u32 address, bool& valid) {
    u32 physical = address;
    if (address >= 0x20000000u && address < 0x22000000u) {
        physical = address - 0x20000000u;
    } else if (address >= 0x30000000u && address < 0x32000000u) {
        physical = address - 0x30000000u;
    } else if (address >= 0x80000000u && address < 0xC0000000u) {
        physical = address & 0x1FFFFFFFu;
    }
    valid = physical < kRamSize;
    return physical;
}

u32 read_word(const u8* ram, u32 physical) {
    return static_cast<u32>(ram[physical + 0u]) |
           (static_cast<u32>(ram[physical + 1u]) << 8u) |
           (static_cast<u32>(ram[physical + 2u]) << 16u) |
           (static_cast<u32>(ram[physical + 3u]) << 24u);
}

bool is_store(u32 instruction) {
    switch (instruction >> 26) {
    case 0x1Fu: // SQ
    case 0x28u: // SB
    case 0x29u: // SH
    case 0x2Bu: // SW
    case 0x38u: // SC
    case 0x39u: // SWC1
    case 0x3Cu: // SCD
    case 0x3Eu: // SQC2
    case 0x3Fu: // SD
        return true;
    default:
        return false;
    }
}

bool is_load(u32 instruction) {
    switch (instruction >> 26) {
    case 0x1Eu: // LQ
    case 0x20u: // LB
    case 0x21u: // LH
    case 0x23u: // LW
    case 0x24u: // LBU
    case 0x25u: // LHU
    case 0x27u: // LWU
    case 0x30u: // LL
    case 0x31u: // LWC1
    case 0x34u: // LLD
    case 0x36u: // LQC2
    case 0x37u: // LD
        return true;
    default:
        return false;
    }
}

bool is_mfc0(u32 instruction) {
    return (instruction >> 26) == 0x10u &&
           ((instruction >> 21) & 31u) == 0u &&
           (instruction & 7u) == 0u;
}

bool is_mtc0(u32 instruction) {
    return (instruction >> 26) == 0x10u &&
           ((instruction >> 21) & 31u) == 4u &&
           (instruction & 7u) == 0u;
}

bool is_cop0_ei_di(u32 instruction) {
    if ((instruction >> 26) != 0x10u ||
        ((instruction >> 21) & 31u) != 0x10u) {
        return false;
    }
    const u32 funct = instruction & 63u;
    return funct == 0x38u || funct == 0x39u;
}

bool is_cop1_move(u32 instruction) {
    if ((instruction >> 26) != 0x11u) return false;
    const u32 rs = (instruction >> 21) & 31u;
    return rs == 0x00u || rs == 0x02u ||
           rs == 0x04u || rs == 0x06u;
}

bool is_cop1_bitwise_unary(u32 instruction) {
    if ((instruction >> 26) != 0x11u ||
        ((instruction >> 21) & 31u) != 0x10u) {
        return false;
    }
    const u32 funct = instruction & 63u;
    return funct == 0x05u || // ABS.S
           funct == 0x06u || // MOV.S
           funct == 0x07u;   // NEG.S
}

bool is_cop1_basic_arithmetic(u32 instruction) {
    if ((instruction >> 26) != 0x11u ||
        ((instruction >> 21) & 31u) != 0x10u) {
        return false;
    }
    const u32 funct = instruction & 63u;
    return funct == 0x00u || // ADD.S
           funct == 0x01u || // SUB.S
           funct == 0x02u;   // MUL.S
}

bool is_cop1_acc_arithmetic(u32 instruction) {
    if ((instruction >> 26) != 0x11u ||
        ((instruction >> 21) & 31u) != 0x10u) {
        return false;
    }
    switch (instruction & 63u) {
    case 0x18u: // ADDA.S
    case 0x19u: // SUBA.S
    case 0x1Au: // MULA.S
    case 0x1Cu: // MADD.S
    case 0x1Du: // MSUB.S
    case 0x1Eu: // MADDA.S
    case 0x1Fu: // MSUBA.S
        return true;
    default:
        return false;
    }
}

bool is_cop1_minmax(u32 instruction) {
    if ((instruction >> 26) != 0x11u ||
        ((instruction >> 21) & 31u) != 0x10u) {
        return false;
    }
    const u32 funct = instruction & 63u;
    return funct == 0x28u || funct == 0x29u;
}

bool is_cop1_compare(u32 instruction) {
    if ((instruction >> 26) != 0x11u ||
        ((instruction >> 21) & 31u) != 0x10u) {
        return false;
    }
    const u32 funct = instruction & 63u;
    return funct == 0x30u || funct == 0x32u ||
           funct == 0x34u || funct == 0x36u;
}

bool is_cop1_cvt_s_w(u32 instruction) {
    return (instruction >> 26) == 0x11u &&
           ((instruction >> 21) & 31u) == 0x14u &&
           (instruction & 63u) == 0x20u;
}

bool supported_special(u32 instruction) {
    const u32 funct = instruction & 63u;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 sa = (instruction >> 6) & 31u;
    switch (funct) {
    case 0x00u: // SLL
    case 0x02u: // SRL
    case 0x03u: // SRA
    case 0x38u: // DSLL
    case 0x3Au: // DSRL
    case 0x3Bu: // DSRA
    case 0x3Cu: // DSLL32
    case 0x3Eu: // DSRL32
    case 0x3Fu: // DSRA32
        return rs == 0u;
    case 0x04u: // SLLV
    case 0x06u: // SRLV
    case 0x07u: // SRAV
    case 0x0Au: // MOVZ
    case 0x0Bu: // MOVN
    case 0x0Fu: // SYNC
    case 0x10u: // MFHI
    case 0x11u: // MTHI
    case 0x12u: // MFLO
    case 0x13u: // MTLO
    case 0x14u: // DSLLV
    case 0x16u: // DSRLV
    case 0x17u: // DSRAV
    case 0x18u: // MULT
    case 0x19u: // MULTU
    case 0x1Au: // DIV
    case 0x1Bu: // DIVU
    case 0x28u: // MFSA
    case 0x29u: // MTSA
        return true;
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
        return sa == 0u;
    default:
        return false;
    }
}

bool supported_noncontrol(u32 instruction) {
    if (instruction == 0u) return true;
    const u32 opcode = instruction >> 26;
    if (opcode == 0u) return supported_special(instruction);
    if (opcode == 0x01u) {
        const u32 rt = (instruction >> 16) & 31u;
        return rt == 0x18u || rt == 0x19u; // MTSAB / MTSAH
    }
    switch (opcode) {
    case 0x09u: // ADDIU
    case 0x0Au: // SLTI
    case 0x0Bu: // SLTIU
    case 0x0Cu: // ANDI
    case 0x0Du: // ORI
    case 0x0Eu: // XORI
    case 0x0Fu: // LUI
    case 0x19u: // DADDIU
    case 0x2Fu: // CACHE
    case 0x33u: // PREF
        return true;
    default:
        break;
    }
    if (is_load(instruction) || is_store(instruction)) return true;
    if (is_cop1_move(instruction) ||
        is_cop1_bitwise_unary(instruction) ||
        is_cop1_basic_arithmetic(instruction) ||
        is_cop1_acc_arithmetic(instruction) ||
        is_cop1_minmax(instruction) ||
        is_cop1_compare(instruction) ||
        is_cop1_cvt_s_w(instruction)) return true;
    if (is_mfc0(instruction)) return true;
    if (is_cop0_ei_di(instruction)) return true;
    if (is_mtc0(instruction)) {
        // All select-0 MTC0 writes are native precise exits. Count is formed
        // as a one-instruction block so its write precedes exactly one Count
        // retirement increment, matching step_internal().
        return true;
    }
    return false;
}

enum class ControlKind {
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
    Beql,
    Bnel,
    Blezl,
    Bgtzl,
    Bltzl,
    Bgezl,
    Bltzall,
    Bgezall,
    Bc1f,
    Bc1t,
    Bc1fl,
    Bc1tl,
};

ControlKind control_kind(u32 instruction) {
    const u32 opcode = instruction >> 26;
    switch (opcode) {
    case 0x02u: return ControlKind::J;
    case 0x03u: return ControlKind::Jal;
    case 0x04u: return ControlKind::Beq;
    case 0x05u: return ControlKind::Bne;
    case 0x06u: return ControlKind::Blez;
    case 0x07u: return ControlKind::Bgtz;
    case 0x14u: return ControlKind::Beql;
    case 0x15u: return ControlKind::Bnel;
    case 0x16u: return ControlKind::Blezl;
    case 0x17u: return ControlKind::Bgtzl;
    case 0x01u: {
        const u32 rt = (instruction >> 16) & 31u;
        switch (rt) {
        case 0x00u: return ControlKind::Bltz;
        case 0x01u: return ControlKind::Bgez;
        case 0x02u: return ControlKind::Bltzl;
        case 0x03u: return ControlKind::Bgezl;
        case 0x10u: return ControlKind::Bltzal;
        case 0x11u: return ControlKind::Bgezal;
        case 0x12u: return ControlKind::Bltzall;
        case 0x13u: return ControlKind::Bgezall;
        default: return ControlKind::None;
        }
    }
    case 0x00u:
        if ((instruction & 63u) == 0x08u) return ControlKind::Jr;
        if ((instruction & 63u) == 0x09u) return ControlKind::Jalr;
        return ControlKind::None;
    case 0x11u:
        if (((instruction >> 21) & 31u) == 0x08u) {
            switch ((instruction >> 16) & 3u) {
            case 0u: return ControlKind::Bc1f;
            case 1u: return ControlKind::Bc1t;
            case 2u: return ControlKind::Bc1fl;
            case 3u: return ControlKind::Bc1tl;
            default: break;
            }
        }
        return ControlKind::None;
    default:
        return ControlKind::None;
    }
}

bool is_control(u32 instruction) {
    return control_kind(instruction) != ControlKind::None;
}

bool is_fusable_conditional(ControlKind control) {
    switch (control) {
    case ControlKind::Beq:
    case ControlKind::Bne:
    case ControlKind::Blez:
    case ControlKind::Bgtz:
    case ControlKind::Bltz:
    case ControlKind::Bgez:
        return true;
    default:
        return false;
    }
}

u8 branch_take_cc(ControlKind control) {
    switch (control) {
    case ControlKind::Beq: return 0x4u;  // JE
    case ControlKind::Bne: return 0x5u;  // JNE
    case ControlKind::Blez: return 0xEu; // JLE
    case ControlKind::Bgtz: return 0xFu; // JG
    case ControlKind::Bltz: return 0xCu; // JL
    case ControlKind::Bgez: return 0xDu; // JGE
    default: return 0x4u;
    }
}

u32 branch_target(u32 pc, u32 instruction) {
    const s16 imm = static_cast<s16>(instruction & 0xFFFFu);
    return pc + 4u +
           static_cast<u32>(static_cast<s32>(imm) * 4);
}

bool writes_gpr(u32 instruction, u32 reg) {
    if (reg == 0u) return false;
    const u32 opcode = instruction >> 26;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;
    if (opcode == 0u) {
        const u32 funct = instruction & 63u;
        switch (funct) {
        case 0x11u:
        case 0x13u:
        case 0x08u:
            return false;
        case 0x09u:
            return rd == reg;
        default:
            return rd == reg;
        }
    }
    if (opcode == 0x03u) return reg == 31u;
    if (opcode == 0x01u) {
        const u32 variant = rt;
        return (variant == 0x10u || variant == 0x11u ||
                variant == 0x12u || variant == 0x13u) &&
               reg == 31u;
    }
    if (is_store(instruction)) {
        return (opcode == 0x38u || opcode == 0x3Cu) && rt == reg;
    }
    if (opcode == 0x31u || opcode == 0x36u) return false;
    if (is_cop1_bitwise_unary(instruction)) return false;

    if (is_cop1_move(instruction)) {
        const u32 cop_rs = (instruction >> 21) & 31u;
        return (cop_rs == 0x00u || cop_rs == 0x02u) && rt == reg;
    }
    if (is_mtc0(instruction) || is_cop0_ei_di(instruction)) return false;
    if (is_mfc0(instruction)) return rt == reg;
    switch (opcode) {
    case 0x09u:
    case 0x0Au:
    case 0x0Bu:
    case 0x0Cu:
    case 0x0Du:
    case 0x0Eu:
    case 0x0Fu:
    case 0x19u:
    case 0x1Eu:
    case 0x20u:
    case 0x21u:
    case 0x23u:
    case 0x24u:
    case 0x25u:
    case 0x27u:
    case 0x30u:
    case 0x34u:
    case 0x37u:
        return rt == reg;
    default:
        return false;
    }
}

void score_registers(
    u32 instruction,
    std::array<u32, 32>& scores) {
    const u32 opcode = instruction >> 26;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;
    auto use = [&](u32 reg, u32 weight = 1u) {
        if (reg != 0u) scores[reg] += weight;
    };

    if (opcode == 0u) {
        const u32 funct = instruction & 63u;
        switch (funct) {
        case 0x00u:
        case 0x02u:
        case 0x03u:
        case 0x38u:
        case 0x3Au:
        case 0x3Bu:
        case 0x3Cu:
        case 0x3Eu:
        case 0x3Fu:
            use(rt); use(rd, 2u); return;
        case 0x10u:
        case 0x12u:
            use(rd, 2u); return;
        case 0x11u:
        case 0x13u:
        case 0x08u:
            use(rs); return;
        case 0x09u:
            use(rs); use(rd, 2u); return;
        default:
            use(rs); use(rt); use(rd, 2u); return;
        }
    }

    if (opcode == 0x02u) return;
    if (opcode == 0x03u) { use(31u, 2u); return; }
    if (opcode == 0x01u || (opcode >= 0x04u && opcode <= 0x07u)) {
        use(rs);
        if (opcode == 0x04u || opcode == 0x05u) use(rt);
        if (opcode == 0x01u) {
            const u32 variant = rt;
            if (variant == 0x10u || variant == 0x11u ||
                variant == 0x12u || variant == 0x13u) {
                use(31u, 2u);
            }
        }
        return;
    }

    if (is_store(instruction)) {
        use(rs, 2u);
        if (opcode != 0x39u && opcode != 0x3Eu) use(rt);
        return;
    }
    if (is_load(instruction)) {
        use(rs, 2u);
        if (opcode != 0x31u && opcode != 0x36u) use(rt, 2u);
        return;
    }
    if (is_cop1_move(instruction)) {
        const u32 cop_rs = (instruction >> 21) & 31u;
        if (cop_rs == 0x00u || cop_rs == 0x02u) use(rt, 2u);
        else use(rt);
        return;
    }
    if (is_mfc0(instruction)) {
        use(rt, 2u); return;
    }
    if (is_mtc0(instruction)) {
        use(rt); return;
    }
    if (is_cop0_ei_di(instruction)) return;

    use(rs);
    use(rt, 2u);
}

#if defined(VIBESTATION_EE_DYNAREC_X64)

enum Reg : u8 {
    RAX = 0, RCX = 1, RDX = 2, RBX = 3,
    RSP = 4, RBP = 5, RSI = 6, RDI = 7,
    R8 = 8, R9 = 9, R10 = 10, R11 = 11,
    R12 = 12, R13 = 13, R14 = 14, R15 = 15,
};

struct Emitter {
    std::vector<u8> bytes;
    std::array<u32, 4> guest_cache{
        0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
    std::array<bool, 4> dirty{};
    bool cache_hi = false;
    bool cache_lo = false;
    bool hi_dirty = false;
    bool lo_dirty = false;
    u32 register_cache_uses = 0u;

    void emit(u8 v) { bytes.push_back(v); }
    void emit32(u32 v) {
        for (u32 i = 0; i < 4u; ++i)
            emit(static_cast<u8>(v >> (i * 8u)));
    }
    void emit64(u64 v) {
        for (u32 i = 0; i < 8u; ++i)
            emit(static_cast<u8>(v >> (i * 8u)));
    }

    void rex(bool w, int r = -1, int x = -1, int b = -1) {
        u8 value = 0x40u;
        if (w) value |= 0x08u;
        if (r >= 8) value |= 0x04u;
        if (x >= 8) value |= 0x02u;
        if (b >= 8) value |= 0x01u;
        if (value != 0x40u) emit(value);
    }

    void modrm(u8 mod, u8 reg, u8 rm) {
        emit(static_cast<u8>((mod << 6u) |
                             ((reg & 7u) << 3u) |
                             (rm & 7u)));
    }

    void sib(u8 scale, u8 index, u8 base) {
        emit(static_cast<u8>((scale << 6u) |
                             ((index & 7u) << 3u) |
                             (base & 7u)));
    }

    void push(Reg reg) {
        if (reg >= R8) emit(0x41u);
        emit(static_cast<u8>(0x50u + (reg & 7u)));
    }
    void pop(Reg reg) {
        if (reg >= R8) emit(0x41u);
        emit(static_cast<u8>(0x58u + (reg & 7u)));
    }

    void mov_rr64(Reg dst, Reg src) {
        rex(true, src, -1, dst);
        emit(0x89u);
        modrm(3u, src, dst);
    }
    void mov_rr32(Reg dst, Reg src) {
        rex(false, src, -1, dst);
        emit(0x89u);
        modrm(3u, src, dst);
    }

    void mov_r64_imm(Reg dst, u64 value) {
        rex(true, -1, -1, dst);
        emit(static_cast<u8>(0xB8u + (dst & 7u)));
        emit64(value);
    }
    void mov_r32_imm(Reg dst, u32 value) {
        rex(false, -1, -1, dst);
        emit(static_cast<u8>(0xB8u + (dst & 7u)));
        emit32(value);
    }

    void mem_disp_prefix(
        bool w, Reg reg, Reg base, u8 opcode) {
        rex(w, reg, -1, base);
        emit(opcode);
        modrm(2u, reg, base);
        if ((base & 7u) == RSP) sib(0u, RSP, base);
    }

    void load64(Reg dst, Reg base, u32 disp) {
        mem_disp_prefix(true, dst, base, 0x8Bu);
        emit32(disp);
    }
    void load32(Reg dst, Reg base, u32 disp) {
        mem_disp_prefix(false, dst, base, 0x8Bu);
        emit32(disp);
    }
    void store64(Reg base, u32 disp, Reg src) {
        mem_disp_prefix(true, src, base, 0x89u);
        emit32(disp);
    }
    void store32(Reg base, u32 disp, Reg src) {
        mem_disp_prefix(false, src, base, 0x89u);
        emit32(disp);
    }
    void store32_imm(Reg base, u32 disp, u32 value) {
        rex(false, -1, -1, base);
        emit(0xC7u);
        modrm(2u, 0u, base);
        if ((base & 7u) == RSP) sib(0u, RSP, base);
        emit32(disp);
        emit32(value);
    }

    void load_indexed(
        Reg dst, Reg base, Reg index, u32 width, bool sign,
        u32 displacement = 0u) {
        if (width == 1u) {
            rex(sign, dst, index, base);
            emit(0x0Fu); emit(sign ? 0xBEu : 0xB6u);
        } else if (width == 2u) {
            rex(sign, dst, index, base);
            emit(0x0Fu); emit(sign ? 0xBFu : 0xB7u);
        } else if (width == 4u && sign) {
            rex(true, dst, index, base);
            emit(0x63u);
        } else {
            rex(width == 8u, dst, index, base);
            emit(0x8Bu);
        }
        modrm(2u, dst, RSP);
        sib(0u, index, base);
        emit32(displacement);
    }

    void store_indexed(
        Reg base, Reg index, Reg src, u32 width,
        u32 displacement = 0u) {
        if (width == 1u) {
            rex(false, src, index, base);
            emit(0x88u);
        } else if (width == 2u) {
            emit(0x66u);
            rex(false, src, index, base);
            emit(0x89u);
        } else {
            rex(width == 8u, src, index, base);
            emit(0x89u);
        }
        modrm(2u, src, RSP);
        sib(0u, index, base);
        emit32(displacement);
    }

    void loadzx8_indexed(Reg dst, Reg base, Reg index) {
        rex(false, dst, index, base);
        emit(0x0Fu); emit(0xB6u);
        modrm(0u, dst, RSP);
        sib(0u, index, base);
    }

    void add_mem32_index_imm8(
        Reg base, Reg index, u8 scale, u8 value) {
        rex(false, 0, index, base);
        emit(0x83u);
        modrm(0u, 0u, RSP);
        sib(scale, index, base);
        emit(value);
    }

    void add_r64_imm32(Reg reg, u32 value) {
        rex(true, -1, -1, reg);
        emit(0x81u); modrm(3u, 0u, reg); emit32(value);
    }
    void add_r32_imm32(Reg reg, u32 value) {
        rex(false, -1, -1, reg);
        emit(0x81u); modrm(3u, 0u, reg); emit32(value);
    }
    void and_r64_imm32(Reg reg, u32 value) {
        rex(true, -1, -1, reg);
        emit(0x81u); modrm(3u, 4u, reg); emit32(value);
    }
    void and_r32_imm32(Reg reg, u32 value) {
        rex(false, -1, -1, reg);
        emit(0x81u); modrm(3u, 4u, reg); emit32(value);
    }
    void or_r64_imm32(Reg reg, u32 value) {
        rex(true, -1, -1, reg);
        emit(0x81u); modrm(3u, 1u, reg); emit32(value);
    }
    void or_r32_imm32(Reg reg, u32 value) {
        rex(false, -1, -1, reg);
        emit(0x81u); modrm(3u, 1u, reg); emit32(value);
    }
    void xor_r64_imm32(Reg reg, u32 value) {
        rex(true, -1, -1, reg);
        emit(0x81u); modrm(3u, 6u, reg); emit32(value);
    }
    void xor_r32_imm32(Reg reg, u32 value) {
        rex(false, -1, -1, reg);
        emit(0x81u); modrm(3u, 6u, reg); emit32(value);
    }

    void add_rr64(Reg dst, Reg src) {
        rex(true, src, -1, dst); emit(0x01u); modrm(3u, src, dst);
    }
    void add_rr32(Reg dst, Reg src) {
        rex(false, src, -1, dst); emit(0x01u); modrm(3u, src, dst);
    }
    void sub_rr64(Reg dst, Reg src) {
        rex(true, src, -1, dst); emit(0x29u); modrm(3u, src, dst);
    }
    void sub_rr32(Reg dst, Reg src) {
        rex(false, src, -1, dst); emit(0x29u); modrm(3u, src, dst);
    }
    void and_rr64(Reg dst, Reg src) {
        rex(true, src, -1, dst); emit(0x21u); modrm(3u, src, dst);
    }
    void or_rr64(Reg dst, Reg src) {
        rex(true, src, -1, dst); emit(0x09u); modrm(3u, src, dst);
    }
    void xor_rr64(Reg dst, Reg src) {
        rex(true, src, -1, dst); emit(0x31u); modrm(3u, src, dst);
    }
    void not_r64(Reg reg) {
        rex(true, -1, -1, reg); emit(0xF7u); modrm(3u, 2u, reg);
    }

    void shift_imm64(Reg reg, u8 subop, u8 amount) {
        rex(true, -1, -1, reg);
        emit(0xC1u); modrm(3u, subop, reg); emit(amount);
    }
    void shift_imm32(Reg reg, u8 subop, u8 amount) {
        rex(false, -1, -1, reg);
        emit(0xC1u); modrm(3u, subop, reg); emit(amount);
    }
    void shift_cl64(Reg reg, u8 subop) {
        rex(true, -1, -1, reg);
        emit(0xD3u); modrm(3u, subop, reg);
    }
    void shift_cl32(Reg reg, u8 subop) {
        rex(false, -1, -1, reg);
        emit(0xD3u); modrm(3u, subop, reg);
    }
    void sign_extend_eax() { emit(0x48u); emit(0x98u); }
    void movsxd_rr(Reg dst, Reg src) {
        rex(true, dst, -1, src);
        emit(0x63u);
        modrm(3u, dst, src);
    }
    void mul_r32(Reg src, bool is_signed) {
        rex(false, -1, -1, src);
        emit(0xF7u);
        modrm(3u, is_signed ? 5u : 4u, src);
    }
    void div_r32(Reg src, bool is_signed) {
        rex(false, -1, -1, src);
        emit(0xF7u);
        modrm(3u, is_signed ? 7u : 6u, src);
    }
    void cdq() { emit(0x99u); }

    void cmp_rr64(Reg lhs, Reg rhs) {
        rex(true, rhs, -1, lhs); emit(0x39u); modrm(3u, rhs, lhs);
    }
    void cmp_r64_imm32(Reg reg, u32 value) {
        rex(true, -1, -1, reg);
        emit(0x81u); modrm(3u, 7u, reg); emit32(value);
    }
    void cmp_r32_imm32(Reg reg, u32 value) {
        rex(false, -1, -1, reg);
        emit(0x81u); modrm(3u, 7u, reg); emit32(value);
    }
    void test_rr64(Reg lhs, Reg rhs) {
        rex(true, rhs, -1, lhs); emit(0x85u); modrm(3u, rhs, lhs);
    }

    void setcc_al(u8 cc) {
        emit(0x0Fu); emit(static_cast<u8>(0x90u + cc)); emit(0xC0u);
    }
    void movzx_eax_al() {
        emit(0x0Fu); emit(0xB6u); emit(0xC0u);
    }

    void movd_xmm_r32(u8 xmm, Reg src) {
        emit(0x66u);
        rex(false, xmm, -1, src);
        emit(0x0Fu); emit(0x6Eu);
        modrm(3u, xmm, src);
    }
    void movd_r32_xmm(Reg dst, u8 xmm) {
        emit(0x66u);
        rex(false, xmm, -1, dst);
        emit(0x0Fu); emit(0x7Eu);
        modrm(3u, xmm, dst);
    }
    void scalar_ss(u8 opcode, u8 dst_xmm, u8 src_xmm) {
        emit(0xF3u);
        rex(false, dst_xmm, -1, src_xmm);
        emit(0x0Fu); emit(opcode);
        modrm(3u, dst_xmm, src_xmm);
    }
    void cvtsi2ss_xmm_r32(u8 xmm, Reg src) {
        emit(0xF3u);
        rex(false, xmm, -1, src);
        emit(0x0Fu); emit(0x2Au);
        modrm(3u, xmm, src);
    }
    void ucomiss(u8 lhs_xmm, u8 rhs_xmm) {
        rex(false, lhs_xmm, -1, rhs_xmm);
        emit(0x0Fu); emit(0x2Eu);
        modrm(3u, lhs_xmm, rhs_xmm);
    }

    std::size_t jcc32(u8 cc) {
        emit(0x0Fu); emit(static_cast<u8>(0x80u + cc));
        const std::size_t at = bytes.size();
        emit32(0u);
        return at;
    }
    std::size_t jmp32() {
        emit(0xE9u);
        const std::size_t at = bytes.size();
        emit32(0u);
        return at;
    }
    void patch(std::size_t at, std::size_t target) {
        const s64 rel =
            static_cast<s64>(target) -
            static_cast<s64>(at + 4u);
        const u32 encoded = static_cast<u32>(static_cast<s32>(rel));
        for (u32 i = 0; i < 4u; ++i)
            bytes[at + i] =
                static_cast<u8>(encoded >> (i * 8u));
    }

    int cache_slot(u32 guest) const {
        for (int i = 0; i < 4; ++i)
            if (guest_cache[static_cast<std::size_t>(i)] == guest) return i;
        return -1;
    }
    static Reg cache_host(int slot) {
        return static_cast<Reg>(R12 + slot);
    }

    void load_guest(Reg dst, u32 guest, bool word = false) {
        if (guest == 0u) {
            mov_r32_imm(dst, 0u);
            return;
        }
        const int slot = cache_slot(guest);
        if (slot >= 0) {
            ++register_cache_uses;
            if (dst != cache_host(slot)) mov_rr64(dst, cache_host(slot));
            if (word) {
                // A word consumer only observes the low 32 bits.
                mov_rr32(dst, dst);
            }
            return;
        }
        const u32 off = static_cast<u32>(
            offsetof(EeCpuState, gpr) +
            guest * sizeof(EeGpr));
        if (word) load32(dst, RBX, off);
        else load64(dst, RBX, off);
    }

    void store_guest(u32 guest, Reg src) {
        if (guest == 0u) return;
        const int slot = cache_slot(guest);
        if (slot >= 0) {
            ++register_cache_uses;
            if (cache_host(slot) != src) mov_rr64(cache_host(slot), src);
            dirty[static_cast<std::size_t>(slot)] = true;
            return;
        }
        store64(
            RBX,
            static_cast<u32>(
                offsetof(EeCpuState, gpr) +
                guest * sizeof(EeGpr)),
            src);
    }

    void flush_cache() {
        for (int i = 0; i < 4; ++i) {
            if (!dirty[static_cast<std::size_t>(i)]) continue;
            const u32 guest =
                guest_cache[static_cast<std::size_t>(i)];
            store64(
                RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, gpr) +
                    guest * sizeof(EeGpr)),
                cache_host(i));
        }
        if (hi_dirty) {
            store64(
                RBX,
                static_cast<u32>(offsetof(EeCpuState, hi)),
                R8);
        }
        if (lo_dirty) {
            store64(
                RBX,
                static_cast<u32>(offsetof(EeCpuState, lo)),
                R9);
        }
    }

    void prologue() {
        push(RBX); push(RBP); push(R12);
        push(R13); push(R14); push(R15);
#ifdef _WIN32
        mov_rr64(RBX, RCX);
        mov_rr64(RBP, RDX);
        mov_rr64(R10, R8);
        mov_rr64(R11, R9);
#else
        mov_rr64(RBX, RDI);
        mov_rr64(RBP, RSI);
        mov_rr64(R10, RDX);
        mov_rr64(R11, RCX);
#endif
        for (int i = 0; i < 4; ++i) {
            const u32 guest =
                guest_cache[static_cast<std::size_t>(i)];
            if (guest == 0xFFFFFFFFu) continue;
            load64(
                cache_host(i),
                RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, gpr) +
                    guest * sizeof(EeGpr)));
        }
        if (cache_hi) {
            load64(R8, RBX, static_cast<u32>(offsetof(EeCpuState, hi)));
        }
        if (cache_lo) {
            load64(R9, RBX, static_cast<u32>(offsetof(EeCpuState, lo)));
        }
    }

    void epilogue_return(u32 retired) {
        flush_cache();
        mov_r32_imm(RAX, retired);
        pop(R15); pop(R14); pop(R13);
        pop(R12); pop(RBP); pop(RBX);
        emit(0xC3u);
    }
};

void emit_ps2_float_normalize(
    Emitter& out,
    Reg value,
    Reg scratch) {
    out.mov_rr32(scratch, value);
    out.and_r32_imm32(scratch, 0x7F800000u);

    out.cmp_r32_imm32(scratch, 0u);
    const std::size_t not_denormal = out.jcc32(0x5u); // JNE
    out.and_r32_imm32(value, 0x80000000u);
    const std::size_t done_denormal = out.jmp32();

    const std::size_t nonzero_exp = out.bytes.size();
    out.patch(not_denormal, nonzero_exp);
    out.cmp_r32_imm32(scratch, 0x7F800000u);
    const std::size_t normal = out.jcc32(0x5u); // JNE
    out.and_r32_imm32(value, 0x80000000u);
    out.or_r32_imm32(value, 0x7F7FFFFFu);

    const std::size_t done = out.bytes.size();
    out.patch(normal, done);
    out.patch(done_denormal, done);
}

void* allocate_page() {
#ifdef _WIN32
    return VirtualAlloc(
        nullptr, kCodePageSize,
        MEM_COMMIT | MEM_RESERVE,
        PAGE_READWRITE);
#else
    void* page = mmap(
        nullptr, kCodePageSize,
        PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS,
        -1, 0);
    return page == MAP_FAILED ? nullptr : page;
#endif
}

bool protect_page(void* page, bool executable) {
#ifdef _WIN32
    DWORD old_protection = 0;
    return VirtualProtect(
               page, kCodePageSize,
               executable
                   ? PAGE_EXECUTE_READ
                   : PAGE_READWRITE,
               &old_protection) != 0;
#else
    return mprotect(
               page, kCodePageSize,
               executable
                   ? PROT_READ | PROT_EXEC
                   : PROT_READ | PROT_WRITE) == 0;
#endif
}

void release_page(void* page) {
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

void emit_add_state64(Emitter& out, u32 offset, u32 value) {
    out.rex(true, -1, -1, RBX);
    out.emit(0x81u);
    out.modrm(2u, 0u, RBX);
    out.emit32(offset);
    out.emit32(value);
}

void emit_add_state32(Emitter& out, u32 offset, u32 value) {
    out.rex(false, -1, -1, RBX);
    out.emit(0x81u);
    out.modrm(2u, 0u, RBX);
    out.emit32(offset);
    out.emit32(value);
}

void emit_or_state32(Emitter& out, u32 offset, u32 value) {
    out.rex(false, -1, -1, RBX);
    out.emit(0x81u);
    out.modrm(2u, 1u, RBX);
    out.emit32(offset);
    out.emit32(value);
}

void emit_and_state32(Emitter& out, u32 offset, u32 value) {
    out.rex(false, -1, -1, RBX);
    out.emit(0x81u);
    out.modrm(2u, 4u, RBX);
    out.emit32(offset);
    out.emit32(value);
}

void emit_commit_common(
    Emitter& out,
    u32 retired,
    u32 last_pc,
    u32 last_instruction) {
    if (retired != 0u) {
        out.store32_imm(
            RBX,
            static_cast<u32>(offsetof(EeCpuState, last_pc)),
            last_pc);
        out.store32_imm(
            RBX,
            static_cast<u32>(offsetof(EeCpuState, last_instruction)),
            last_instruction);
        emit_add_state64(
            out,
            static_cast<u32>(
                offsetof(EeCpuState, instructions_executed)),
            retired);
        emit_add_state32(
            out,
            static_cast<u32>(
                offsetof(EeCpuState, cop0) + 9u * sizeof(u32)),
            retired);

        out.load32(
            RAX, RBX,
            static_cast<u32>(
                offsetof(EeCpuState, cop0) + 9u * sizeof(u32)));
        out.load32(
            RDX, RBX,
            static_cast<u32>(
                offsetof(EeCpuState, cop0) + 11u * sizeof(u32)));
        out.rex(false, RDX, -1, RAX);
        out.emit(0x39u);
        out.modrm(3u, RDX, RAX);
        const std::size_t not_equal = out.jcc32(0x5u); // JNE
        emit_or_state32(
            out,
            static_cast<u32>(
                offsetof(EeCpuState, cop0) + 13u * sizeof(u32)),
            0x00008000u);
        out.patch(not_equal, out.bytes.size());
    }
}

void emit_commit_sequential(
    Emitter& out,
    u32 retired,
    u32 next_pc,
    u32 last_pc,
    u32 last_instruction) {
    emit_commit_common(
        out, retired, last_pc, last_instruction);
    out.store32_imm(
        RBX,
        static_cast<u32>(offsetof(EeCpuState, pc)),
        next_pc);
    out.store32_imm(
        RBX,
        static_cast<u32>(offsetof(EeCpuState, next_pc)),
        next_pc + 4u);
    out.epilogue_return(retired);
}

void emit_commit_dynamic_pc(
    Emitter& out,
    u32 retired,
    Reg pc_reg,
    u32 last_pc,
    u32 last_instruction) {
    emit_commit_common(
        out, retired, last_pc, last_instruction);
    out.store32(
        RBX,
        static_cast<u32>(offsetof(EeCpuState, pc)),
        pc_reg);
    out.add_r32_imm32(pc_reg, 4u);
    out.store32(
        RBX,
        static_cast<u32>(offsetof(EeCpuState, next_pc)),
        pc_reg);
    out.epilogue_return(retired);
}

void emit_fastmem_address(
    Emitter& out,
    u32 rs,
    s16 immediate,
    u32 width,
    std::vector<std::size_t>& fail_jumps,
    bool require_single_page = false,
    u32 alignment_mask = 0u) {
    out.load_guest(RAX, rs, true);
    out.add_r32_imm32(
        RAX,
        static_cast<u32>(static_cast<s32>(immediate)));

    std::vector<std::size_t> direct;
    std::vector<std::size_t> alias2;
    std::vector<std::size_t> alias3;

    out.cmp_r32_imm32(RAX, 0x20000000u);
    direct.push_back(out.jcc32(0x2u)); // JB

    out.cmp_r32_imm32(RAX, 0x22000000u);
    alias2.push_back(out.jcc32(0x2u));

    out.cmp_r32_imm32(RAX, 0x30000000u);
    direct.push_back(out.jcc32(0x2u));

    out.cmp_r32_imm32(RAX, 0x32000000u);
    alias3.push_back(out.jcc32(0x2u));

    out.cmp_r32_imm32(RAX, 0x80000000u);
    direct.push_back(out.jcc32(0x2u));

    out.cmp_r32_imm32(RAX, 0xC0000000u);
    fail_jumps.push_back(out.jcc32(0x3u)); // JAE

    out.and_r32_imm32(RAX, 0x1FFFFFFFu);
    direct.push_back(out.jmp32());

    const std::size_t alias2_label = out.bytes.size();
    out.add_r32_imm32(RAX, 0xE0000000u);

    const std::size_t alias3_jump = out.jmp32();

    const std::size_t alias3_label = out.bytes.size();
    out.add_r32_imm32(RAX, 0xD0000000u);

    const std::size_t mapped = out.bytes.size();
    for (const auto jump : direct) out.patch(jump, mapped);
    for (const auto jump : alias2) out.patch(jump, alias2_label);
    for (const auto jump : alias3) out.patch(jump, alias3_label);
    out.patch(alias3_jump, mapped);

    if (alignment_mask != 0u) {
        out.and_r32_imm32(RAX, ~alignment_mask);
    }

    out.cmp_r32_imm32(RAX, kRamSize - width);
    fail_jumps.push_back(out.jcc32(0x7u)); // JA

    if (require_single_page && width > 1u) {
        out.mov_rr32(RCX, RAX);
        out.and_r32_imm32(RCX, kPageSize - 1u);
        out.cmp_r32_imm32(RCX, kPageSize - width);
        fail_jumps.push_back(out.jcc32(0x7u)); // JA
    }
}

void emit_store_generation_barrier(
    Emitter& out,
    const std::array<u32, 4>& code_pages,
    u32 code_page_count,
    std::vector<std::size_t>& selfmod_jumps,
    u8 generation_increment = 1u) {
    out.mov_rr32(RCX, RAX);
    out.shift_imm32(RCX, 5u, 12u); // SHR ECX,12

    out.loadzx8_indexed(RDX, R11, RCX);
    out.test_rr64(RDX, RDX);
    const std::size_t untracked = out.jcc32(0x4u); // JE

    out.add_mem32_index_imm8(
        R10, RCX, 2u, generation_increment);
    for (u32 i = 0u; i < code_page_count; ++i) {
        out.cmp_r32_imm32(RCX, code_pages[i]);
        selfmod_jumps.push_back(out.jcc32(0x4u)); // JE
    }

    out.patch(untracked, out.bytes.size());
}

struct FusedConditionalEdge {
    u32 branch_index = 0;
    u32 delay_index = 0;
    ControlKind control = ControlKind::None;
    u32 taken_pc = 0;
    u32 fallthrough_pc = 0;
    u32 side_exit_pc = 0;
    bool predicted_taken = false;
};

struct CompileState {
    Emitter out;
    u32 block_pc = 0;
    u32 code_page = 0;
    std::array<u32, 4> code_pages{};
    u32 code_page_count = 0;
    std::array<u32, kMaxBlockInstructions> words{};
    std::array<u32, kMaxBlockInstructions> pcs{};
    std::array<EeDynarecIrInstruction, kMaxBlockInstructions> ir{};
    u32 count = 0;
    ControlKind control = ControlKind::None;
    u32 control_index = 0;
    u32 trace_next_pc = 0;
    u32 fused_static_jumps = 0;
    std::array<FusedConditionalEdge, 8> fused_conditionals{};
    u32 fused_conditional_count = 0;
    bool ends_cop0_write = false;
    u32 fastmem_loads = 0;
    u32 fastmem_stores = 0;
};

bool emit_fused_conditional_side_exit(
    CompileState& cs,
    const FusedConditionalEdge& edge) {
    Emitter& out = cs.out;
    const EeDynarecIrInstruction& branch =
        cs.ir[edge.branch_index];

    out.load_guest(RAX, branch.rs);
    if (edge.control == ControlKind::Beq ||
        edge.control == ControlKind::Bne) {
        out.load_guest(RDX, branch.rt);
        out.cmp_rr64(RAX, RDX);
    } else {
        out.test_rr64(RAX, RAX);
    }

    const std::size_t take =
        out.jcc32(branch_take_cc(edge.control));
    const u32 retired = edge.delay_index + 1u;
    const u32 last_pc = cs.pcs[edge.delay_index];
    const u32 last_instruction = cs.words[edge.delay_index];

    if (edge.predicted_taken) {
        // The taken edge is the hot path. A false condition falls into the
        // precise fallthrough side exit; a true condition skips over it.
        out.mov_r32_imm(RCX, edge.side_exit_pc);
        emit_commit_dynamic_pc(
            out, retired, RCX, last_pc, last_instruction);
        out.patch(take, out.bytes.size());
    } else {
        // Fallthrough is hot. A taken branch jumps into the side exit while
        // the predicted not-taken path skips around the return sequence.
        const std::size_t hot = out.jmp32();
        const std::size_t side = out.bytes.size();
        out.patch(take, side);
        out.mov_r32_imm(RCX, edge.side_exit_pc);
        emit_commit_dynamic_pc(
            out, retired, RCX, last_pc, last_instruction);
        out.patch(hot, out.bytes.size());
    }
    return true;
}

bool emit_body(
    CompileState& cs,
    u32 index,
    std::vector<std::pair<std::size_t, u32>>& exits) {
    Emitter& out = cs.out;
    const EeDynarecIrInstruction& ir = cs.ir[index];
    const u32 instruction = ir.word;
    const u32 opcode = ir.opcode;
    const u32 rs = ir.rs;
    const u32 rt = ir.rt;
    const u32 rd = ir.rd;
    const u32 sa = ir.sa;
    const s16 imm = ir.immediate;

    if (instruction == 0u) return true;

    // Static J/JAL instructions may be fused into a same-page native trace.
    // The block builder has already placed their architecturally executed
    // delay slot immediately after them and redirected the following trace PC.
    if (opcode == 0x02u) { // J
        return true;
    }
    if (opcode == 0x03u) { // JAL
        out.mov_r64_imm(
            RAX,
            static_cast<u64>(
                static_cast<s64>(
                    static_cast<s32>(cs.pcs[index] + 8u))));
        out.store_guest(31u, RAX);
        return true;
    }

    if (opcode == 0u) {
        const u32 funct = instruction & 63u;
        switch (funct) {
        case 0x00u:
        case 0x02u:
        case 0x03u: {
            out.load_guest(RAX, rt, true);
            out.shift_imm32(
                RAX,
                funct == 0x00u ? 4u :
                funct == 0x02u ? 5u : 7u,
                static_cast<u8>(sa));
            out.sign_extend_eax();
            out.store_guest(rd, RAX);
            return true;
        }
        case 0x04u: // SLLV
        case 0x06u: // SRLV
        case 0x07u: { // SRAV
            out.load_guest(RAX, rt, true);
            out.load_guest(RCX, rs, true);
            out.shift_cl32(
                RAX,
                funct == 0x04u ? 4u :
                funct == 0x06u ? 5u : 7u);
            out.sign_extend_eax();
            out.store_guest(rd, RAX);
            return true;
        }
        case 0x0Au: // MOVZ
        case 0x0Bu: { // MOVN
            out.load_guest(RDX, rt);
            out.test_rr64(RDX, RDX);
            const std::size_t skip = out.jcc32(
                funct == 0x0Au ? 0x5u : 0x4u); // MOVZ: JNE, MOVN: JE
            out.load_guest(RAX, rs);
            out.store_guest(rd, RAX);
            out.patch(skip, out.bytes.size());
            return true;
        }
        case 0x0Fu: // SYNC
            return true;
        case 0x10u: // MFHI
            if (rd != 0u) {
                if (out.cache_hi) out.mov_rr64(RAX, R8);
                else out.load64(
                    RAX, RBX,
                    static_cast<u32>(offsetof(EeCpuState, hi)));
                out.store_guest(rd, RAX);
            }
            return true;
        case 0x11u: // MTHI
            out.load_guest(RAX, rs);
            if (out.cache_hi) {
                out.mov_rr64(R8, RAX);
                out.hi_dirty = true;
            } else {
                out.store64(
                    RBX,
                    static_cast<u32>(offsetof(EeCpuState, hi)),
                    RAX);
            }
            return true;
        case 0x12u: // MFLO
            if (rd != 0u) {
                if (out.cache_lo) out.mov_rr64(RAX, R9);
                else out.load64(
                    RAX, RBX,
                    static_cast<u32>(offsetof(EeCpuState, lo)));
                out.store_guest(rd, RAX);
            }
            return true;
        case 0x13u: // MTLO
            out.load_guest(RAX, rs);
            if (out.cache_lo) {
                out.mov_rr64(R9, RAX);
                out.lo_dirty = true;
            } else {
                out.store64(
                    RBX,
                    static_cast<u32>(offsetof(EeCpuState, lo)),
                    RAX);
            }
            return true;
        case 0x14u: // DSLLV
        case 0x16u: // DSRLV
        case 0x17u: { // DSRAV
            out.load_guest(RAX, rt);
            out.load_guest(RCX, rs, true);
            out.shift_cl64(
                RAX,
                funct == 0x14u ? 4u :
                funct == 0x16u ? 5u : 7u);
            out.store_guest(rd, RAX);
            return true;
        }
        case 0x18u: // MULT
        case 0x19u: { // MULTU
            const bool signed_multiply = funct == 0x18u;
            out.load_guest(RAX, rs, true);
            out.load_guest(RCX, rt, true);
            out.mul_r32(RCX, signed_multiply);
            out.movsxd_rr(RAX, RAX);
            out.movsxd_rr(RDX, RDX);

            if (out.cache_lo) {
                out.mov_rr64(R9, RAX);
                out.lo_dirty = true;
            } else {
                out.store64(
                    RBX,
                    static_cast<u32>(offsetof(EeCpuState, lo)),
                    RAX);
            }
            if (out.cache_hi) {
                out.mov_rr64(R8, RDX);
                out.hi_dirty = true;
            } else {
                out.store64(
                    RBX,
                    static_cast<u32>(offsetof(EeCpuState, hi)),
                    RDX);
            }
            out.store_guest(rd, RAX);
            return true;
        }
        case 0x1Au: // DIV
        case 0x1Bu: { // DIVU
            const bool signed_divide = funct == 0x1Au;
            out.load_guest(RAX, rs, true);
            out.load_guest(RCX, rt, true);

            out.cmp_r32_imm32(RCX, 0u);
            const std::size_t divisor_nonzero =
                out.jcc32(0x5u); // JNE

            // R5900 divide-by-zero result. HI receives sign_extend_32(lhs).
            out.movsxd_rr(RDX, RAX);
            if (signed_divide) {
                out.cmp_r32_imm32(RAX, 0u);
                const std::size_t lhs_nonnegative =
                    out.jcc32(0xDu); // JGE
                out.mov_r32_imm(RAX, 1u);
                const std::size_t zero_done = out.jmp32();
                const std::size_t nonnegative_label = out.bytes.size();
                out.patch(lhs_nonnegative, nonnegative_label);
                out.mov_r32_imm(RAX, 0xFFFFFFFFu);
                const std::size_t zero_join = out.bytes.size();
                out.patch(zero_done, zero_join);
                out.movsxd_rr(RAX, RAX);
            } else {
                out.mov_r32_imm(RAX, 0xFFFFFFFFu);
                out.movsxd_rr(RAX, RAX);
            }
            const std::size_t result_done_from_zero =
                out.jmp32();

            const std::size_t nonzero_label = out.bytes.size();
            out.patch(divisor_nonzero, nonzero_label);

            std::size_t result_done_from_overflow = 0u;
            bool has_overflow_path = false;
            if (signed_divide) {
                out.cmp_r32_imm32(RAX, 0x80000000u);
                const std::size_t normal_lhs =
                    out.jcc32(0x5u); // JNE
                out.cmp_r32_imm32(RCX, 0xFFFFFFFFu);
                const std::size_t normal_rhs =
                    out.jcc32(0x5u); // JNE

                out.mov_r32_imm(RAX, 0x80000000u);
                out.movsxd_rr(RAX, RAX);
                out.mov_r32_imm(RDX, 0u);
                result_done_from_overflow = out.jmp32();
                has_overflow_path = true;

                const std::size_t normal_label = out.bytes.size();
                out.patch(normal_lhs, normal_label);
                out.patch(normal_rhs, normal_label);
                out.cdq();
                out.div_r32(RCX, true);
            } else {
                out.mov_r32_imm(RDX, 0u);
                out.div_r32(RCX, false);
            }

            out.movsxd_rr(RAX, RAX);
            out.movsxd_rr(RDX, RDX);

            const std::size_t result_join = out.bytes.size();
            out.patch(result_done_from_zero, result_join);
            if (has_overflow_path) {
                out.patch(result_done_from_overflow, result_join);
            }

            if (out.cache_lo) {
                out.mov_rr64(R9, RAX);
                out.lo_dirty = true;
            } else {
                out.store64(
                    RBX,
                    static_cast<u32>(offsetof(EeCpuState, lo)),
                    RAX);
            }
            if (out.cache_hi) {
                out.mov_rr64(R8, RDX);
                out.hi_dirty = true;
            } else {
                out.store64(
                    RBX,
                    static_cast<u32>(offsetof(EeCpuState, hi)),
                    RDX);
            }
            return true;
        }
        case 0x28u: // MFSA
            out.load32(
                RAX, RBX,
                static_cast<u32>(offsetof(EeCpuState, sa)));
            out.store_guest(rd, RAX);
            return true;
        case 0x29u: // MTSA
            out.load_guest(RAX, rs, true);
            out.store32(
                RBX,
                static_cast<u32>(offsetof(EeCpuState, sa)),
                RAX);
            return true;
        case 0x21u:
        case 0x23u: {
            out.load_guest(RAX, rs, true);
            out.load_guest(RDX, rt, true);
            if (funct == 0x21u) out.add_rr32(RAX, RDX);
            else out.sub_rr32(RAX, RDX);
            out.sign_extend_eax();
            out.store_guest(rd, RAX);
            return true;
        }
        case 0x24u:
        case 0x25u:
        case 0x26u:
        case 0x27u:
        case 0x2Du:
        case 0x2Fu: {
            out.load_guest(RAX, rs);
            out.load_guest(RDX, rt);
            if (funct == 0x24u) out.and_rr64(RAX, RDX);
            else if (funct == 0x25u) out.or_rr64(RAX, RDX);
            else if (funct == 0x26u || funct == 0x27u)
                out.xor_rr64(RAX, RDX);
            else if (funct == 0x2Du) out.add_rr64(RAX, RDX);
            else out.sub_rr64(RAX, RDX);
            if (funct == 0x27u) {
                // NOR is OR followed by NOT, not XNOR.
                out.load_guest(RAX, rs);
                out.or_rr64(RAX, RDX);
                out.not_r64(RAX);
            }
            out.store_guest(rd, RAX);
            return true;
        }
        case 0x2Au:
        case 0x2Bu: {
            out.load_guest(RAX, rs);
            out.load_guest(RDX, rt);
            out.cmp_rr64(RAX, RDX);
            out.setcc_al(funct == 0x2Au ? 0xCu : 0x2u);
            out.movzx_eax_al();
            out.store_guest(rd, RAX);
            return true;
        }
        case 0x38u:
        case 0x3Au:
        case 0x3Bu:
        case 0x3Cu:
        case 0x3Eu:
        case 0x3Fu: {
            out.load_guest(RAX, rt);
            const u8 amount = static_cast<u8>(
                sa + ((funct == 0x3Cu ||
                       funct == 0x3Eu ||
                       funct == 0x3Fu) ? 32u : 0u));
            out.shift_imm64(
                RAX,
                (funct == 0x38u || funct == 0x3Cu) ? 4u :
                (funct == 0x3Au || funct == 0x3Eu) ? 5u : 7u,
                amount);
            out.store_guest(rd, RAX);
            return true;
        }
        default:
            return false;
        }
    }

    if (opcode == 0x01u) {
        const u32 variant = rt;
        if (variant == 0x18u || variant == 0x19u) {
            out.load_guest(RAX, rs, true);
            out.and_r32_imm32(
                RAX, variant == 0x18u ? 0xFu : 0x7u);
            out.xor_r64_imm32(
                RAX,
                static_cast<u32>(imm) &
                    (variant == 0x18u ? 0xFu : 0x7u));
            if (variant == 0x19u) {
                out.shift_imm32(RAX, 4u, 1u);
            }
            out.store32(
                RBX,
                static_cast<u32>(offsetof(EeCpuState, sa)),
                RAX);
            return true;
        }
    }

    switch (opcode) {
    case 0x09u: // ADDIU
        out.load_guest(RAX, rs, true);
        out.add_r32_imm32(
            RAX, static_cast<u32>(static_cast<s32>(imm)));
        out.sign_extend_eax();
        out.store_guest(rt, RAX);
        return true;
    case 0x19u: // DADDIU
        out.load_guest(RAX, rs);
        out.add_r64_imm32(
            RAX, static_cast<u32>(static_cast<s32>(imm)));
        out.store_guest(rt, RAX);
        return true;
    case 0x0Cu: // ANDI
        out.load_guest(RAX, rs);
        out.and_r64_imm32(RAX, instruction & 0xFFFFu);
        out.store_guest(rt, RAX);
        return true;
    case 0x0Du: // ORI
        out.load_guest(RAX, rs);
        out.or_r64_imm32(RAX, instruction & 0xFFFFu);
        out.store_guest(rt, RAX);
        return true;
    case 0x0Eu: // XORI
        out.load_guest(RAX, rs);
        out.xor_r64_imm32(RAX, instruction & 0xFFFFu);
        out.store_guest(rt, RAX);
        return true;
    case 0x0Fu: { // LUI
        out.mov_r32_imm(RAX, (instruction & 0xFFFFu) << 16u);
        out.sign_extend_eax();
        out.store_guest(rt, RAX);
        return true;
    }
    case 0x0Au:
    case 0x0Bu: {
        out.load_guest(RAX, rs);
        out.cmp_r64_imm32(
            RAX, static_cast<u32>(static_cast<s32>(imm)));
        out.setcc_al(opcode == 0x0Au ? 0xCu : 0x2u);
        out.movzx_eax_al();
        out.store_guest(rt, RAX);
        return true;
    }
    case 0x2Fu: // CACHE
    case 0x33u: // PREF
        return true;
    default:
        break;
    }

    if (is_cop1_cvt_s_w(instruction)) {
        const u32 fs = rd;
        const u32 fd = (instruction >> 6) & 31u;
        out.load32(
            RAX, RBX,
            static_cast<u32>(
                offsetof(EeCpuState, fpr) + fs * sizeof(u32)));
        out.cvtsi2ss_xmm_r32(0u, RAX);
        out.movd_r32_xmm(RAX, 0u);
        emit_ps2_float_normalize(out, RAX, RCX);
        out.store32(
            RBX,
            static_cast<u32>(
                offsetof(EeCpuState, fpr) + fd * sizeof(u32)),
            RAX);
        return true;
    }

    if (is_cop1_acc_arithmetic(instruction)) {
        const u32 ft = rt;
        const u32 fs = rd;
        const u32 fd = (instruction >> 6) & 31u;
        const u32 funct = instruction & 63u;
        auto load_normalized = [&](u32 reg, u8 xmm) {
            out.load32(
                RAX, RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, fpr) + reg * sizeof(u32)));
            emit_ps2_float_normalize(out, RAX, RCX);
            out.movd_xmm_r32(xmm, RAX);
        };
        auto store_normalized = [&](u8 xmm, u32 offset) {
            out.movd_r32_xmm(RAX, xmm);
            emit_ps2_float_normalize(out, RAX, RCX);
            out.store32(RBX, offset, RAX);
        };

        load_normalized(fs, 0u);
        load_normalized(ft, 1u);

        if (funct == 0x18u || funct == 0x19u || funct == 0x1Au) {
            out.scalar_ss(
                funct == 0x18u ? 0x58u :
                funct == 0x19u ? 0x5Cu :
                                 0x59u,
                0u, 1u);
            store_normalized(
                0u,
                static_cast<u32>(offsetof(EeCpuState, fpu_acc)));
            return true;
        }

        // MADD/MSUB variants evaluate the product first, matching
        // acc +/- (a*b) rather than a fused host operation.
        out.scalar_ss(0x59u, 0u, 1u); // MULSS product
        out.load32(
            RAX, RBX,
            static_cast<u32>(offsetof(EeCpuState, fpu_acc)));
        emit_ps2_float_normalize(out, RAX, RCX);
        out.movd_xmm_r32(1u, RAX); // XMM1 = ACC
        out.scalar_ss(
            (funct == 0x1Cu || funct == 0x1Eu) ? 0x58u : 0x5Cu,
            1u, 0u);

        const bool write_acc =
            funct == 0x1Eu || funct == 0x1Fu;
        store_normalized(
            1u,
            write_acc
                ? static_cast<u32>(offsetof(EeCpuState, fpu_acc))
                : static_cast<u32>(
                    offsetof(EeCpuState, fpr) + fd * sizeof(u32)));
        return true;
    }

    if (is_cop1_minmax(instruction)) {
        const u32 ft = rt;
        const u32 fs = rd;
        const u32 fd = (instruction >> 6) & 31u;
        const u32 funct = instruction & 63u;
        auto load_normalized = [&](u32 reg, u8 xmm) {
            out.load32(
                RAX, RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, fpr) + reg * sizeof(u32)));
            emit_ps2_float_normalize(out, RAX, RCX);
            out.movd_xmm_r32(xmm, RAX);
        };
        load_normalized(fs, 0u);
        load_normalized(ft, 1u);
        out.ucomiss(0u, 1u);
        const std::size_t choose_fs =
            out.jcc32(funct == 0x28u ? 0x3u : 0x6u); // JAE / JBE
        out.load32(
            RAX, RBX,
            static_cast<u32>(
                offsetof(EeCpuState, fpr) + ft * sizeof(u32)));
        const std::size_t selected = out.jmp32();
        const std::size_t fs_label = out.bytes.size();
        out.patch(choose_fs, fs_label);
        out.load32(
            RAX, RBX,
            static_cast<u32>(
                offsetof(EeCpuState, fpr) + fs * sizeof(u32)));
        const std::size_t selected_label = out.bytes.size();
        out.patch(selected, selected_label);
        out.store32(
            RBX,
            static_cast<u32>(
                offsetof(EeCpuState, fpr) + fd * sizeof(u32)),
            RAX);
        return true;
    }

    if (is_cop1_compare(instruction)) {
        constexpr u32 kCond = 0x00800000u;
        const u32 funct = instruction & 63u;
        const u32 fcr31 = static_cast<u32>(
            offsetof(EeCpuState, fcr) + 31u * sizeof(u32));
        emit_and_state32(out, fcr31, ~kCond);
        if (funct == 0x30u) { // C.F.S
            return true;
        }

        const u32 ft = rt;
        const u32 fs = rd;
        auto load_normalized = [&](u32 reg, u8 xmm) {
            out.load32(
                RAX, RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, fpr) + reg * sizeof(u32)));
            emit_ps2_float_normalize(out, RAX, RCX);
            out.movd_xmm_r32(xmm, RAX);
        };
        load_normalized(fs, 0u);
        load_normalized(ft, 1u);
        out.ucomiss(0u, 1u);

        // Skip setting the condition bit when the requested predicate fails.
        const std::size_t false_jump =
            out.jcc32(
                funct == 0x32u ? 0x5u : // EQ: JNE
                funct == 0x34u ? 0x3u : // LT: JAE
                                 0x7u);  // LE: JA
        emit_or_state32(out, fcr31, kCond);
        out.patch(false_jump, out.bytes.size());
        return true;
    }

    if (is_cop1_basic_arithmetic(instruction)) {
        const u32 ft = rt;
        const u32 fs = rd;
        const u32 fd = (instruction >> 6) & 31u;
        const u32 funct = instruction & 63u;

        out.load32(
            RAX, RBX,
            static_cast<u32>(
                offsetof(EeCpuState, fpr) + fs * sizeof(u32)));
        emit_ps2_float_normalize(out, RAX, RCX);
        out.movd_xmm_r32(0u, RAX);

        out.load32(
            RAX, RBX,
            static_cast<u32>(
                offsetof(EeCpuState, fpr) + ft * sizeof(u32)));
        emit_ps2_float_normalize(out, RAX, RCX);
        out.movd_xmm_r32(1u, RAX);

        out.scalar_ss(
            funct == 0x00u ? 0x58u : // ADDSS
            funct == 0x01u ? 0x5Cu : // SUBSS
                             0x59u,  // MULSS
            0u, 1u);
        out.movd_r32_xmm(RAX, 0u);
        emit_ps2_float_normalize(out, RAX, RCX);
        out.store32(
            RBX,
            static_cast<u32>(
                offsetof(EeCpuState, fpr) + fd * sizeof(u32)),
            RAX);
        return true;
    }

    if (is_cop1_bitwise_unary(instruction)) {
        const u32 fs = rd;
        const u32 fd = (instruction >> 6) & 31u;
        const u32 funct = instruction & 63u;
        out.load32(
            RAX, RBX,
            static_cast<u32>(
                offsetof(EeCpuState, fpr) + fs * sizeof(u32)));
        if (funct == 0x05u) { // ABS.S
            out.and_r32_imm32(RAX, 0x7FFFFFFFu);
        } else if (funct == 0x07u) { // NEG.S
            out.xor_r32_imm32(RAX, 0x80000000u);
        }
        out.store32(
            RBX,
            static_cast<u32>(
                offsetof(EeCpuState, fpr) + fd * sizeof(u32)),
            RAX);
        return true;
    }

    if (is_cop1_move(instruction)) {
        const u32 cop_rs = (instruction >> 21) & 31u;
        const u32 fs = rd;
        if (cop_rs == 0x00u) { // MFC1
            out.load32(
                RAX, RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, fpr) + fs * sizeof(u32)));
            out.sign_extend_eax();
            out.store_guest(rt, RAX);
        } else if (cop_rs == 0x02u) { // CFC1
            if (fs == 0u) {
                out.mov_r32_imm(RAX, 0x00002E00u);
            } else if (fs == 31u) {
                out.load32(
                    RAX, RBX,
                    static_cast<u32>(
                        offsetof(EeCpuState, fcr) +
                        31u * sizeof(u32)));
            } else {
                out.mov_r32_imm(RAX, 0u);
            }
            out.sign_extend_eax();
            out.store_guest(rt, RAX);
        } else if (cop_rs == 0x04u) { // MTC1
            out.load_guest(RAX, rt, true);
            out.store32(
                RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, fpr) + fs * sizeof(u32)),
                RAX);
        } else { // CTC1
            if (fs == 31u) {
                out.load_guest(RAX, rt, true);
                out.store32(
                    RBX,
                    static_cast<u32>(
                        offsetof(EeCpuState, fcr) +
                        31u * sizeof(u32)),
                    RAX);
            }
        }
        return true;
    }

    if (is_mfc0(instruction)) {
        const u32 cop = rd;
        out.load32(
            RAX, RBX,
            static_cast<u32>(
                offsetof(EeCpuState, cop0) + cop * sizeof(u32)));
        if (cop == 9u && index != 0u) {
            out.add_r32_imm32(RAX, index);
        }
        out.sign_extend_eax();
        out.store_guest(rt, RAX);
        return true;
    }

    if (is_cop0_ei_di(instruction)) {
        const bool enable = (instruction & 63u) == 0x38u;
        const u32 status_offset = static_cast<u32>(
            offsetof(EeCpuState, cop0) + 12u * sizeof(u32));
        out.load32(RAX, RBX, status_offset);

        // EI/DI is allowed when EDI is set, EXL/ERL is active, or KSU is
        // kernel. Match execute_cop0() exactly, then force a precise exit so
        // the system immediately resamples pending interrupts.
        out.mov_rr32(RCX, RAX);
        out.and_r32_imm32(RCX, 0x00020006u);
        out.test_rr64(RCX, RCX);
        const std::size_t allowed_a = out.jcc32(0x5u); // JNE

        out.mov_rr32(RCX, RAX);
        out.and_r32_imm32(RCX, 0x00000018u);
        out.test_rr64(RCX, RCX);
        const std::size_t allowed_b = out.jcc32(0x4u); // JE
        const std::size_t done_jump = out.jmp32();

        const std::size_t allowed = out.bytes.size();
        out.patch(allowed_a, allowed);
        out.patch(allowed_b, allowed);
        if (enable) {
            emit_or_state32(out, status_offset, 0x00010000u);
        } else {
            emit_and_state32(out, status_offset, ~0x00010000u);
        }
        out.patch(done_jump, out.bytes.size());
        return true;
    }

    if (is_mtc0(instruction)) {
        if (rd != 15u) {
            out.load_guest(RAX, rt, true);
            out.store32(
                RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, cop0) + rd * sizeof(u32)),
                RAX);
            if (rd == 11u) {
                emit_and_state32(
                    out,
                    static_cast<u32>(
                        offsetof(EeCpuState, cop0) +
                        13u * sizeof(u32)),
                    ~0x00008000u);
            }
        }
        return true;
    }

    if (is_load(instruction)) {
        const u32 width = ir.memory_width;
        const u32 alignment_mask = ir.alignment_mask;
        const bool sign =
            opcode == 0x20u || opcode == 0x21u ||
            opcode == 0x23u || opcode == 0x30u;
        if (width == 0u) return false;

        std::vector<std::size_t> fail;
        emit_fastmem_address(
            out, rs, imm, width, fail, false, alignment_mask);

        if (opcode == 0x31u) { // LWC1
            out.load_indexed(RDX, RBP, RAX, 4u, false);
            out.store32(
                RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, fpr) + rt * sizeof(u32)),
                RDX);
        } else if (opcode == 0x36u) { // LQC2
            if (rt != 0u) {
                out.load_indexed(RDX, RBP, RAX, 8u, false);
                out.store64(
                    RBX,
                    static_cast<u32>(
                        offsetof(EeCpuState, vu_vf) +
                        rt * sizeof(EeGpr)),
                    RDX);
                out.load_indexed(RDX, RBP, RAX, 8u, false, 8u);
                out.store64(
                    RBX,
                    static_cast<u32>(
                        offsetof(EeCpuState, vu_vf) +
                        rt * sizeof(EeGpr) + sizeof(u64)),
                    RDX);
            }
        } else if (opcode == 0x1Eu) { // LQ
            if (rt != 0u) {
                out.load_indexed(RDX, RBP, RAX, 8u, false);
                out.store_guest(rt, RDX);
                out.load_indexed(RDX, RBP, RAX, 8u, false, 8u);
                out.store64(
                    RBX,
                    static_cast<u32>(
                        offsetof(EeCpuState, gpr) +
                        rt * sizeof(EeGpr) + sizeof(u64)),
                    RDX);
            }
        } else if (rt != 0u) {
            out.load_indexed(RAX, RBP, RAX, width, sign);
            if (width == 4u && sign) {
                // MOVSXD already sign-extended.
            } else if (width < 4u && sign) {
                // MOVSX with REX.W already sign-extended.
            } else if (width == 4u) {
                out.mov_rr32(RAX, RAX);
            }
            out.store_guest(rt, RAX);
        }

        const std::size_t resume = out.bytes.size();
        const std::size_t skip_fail = out.jmp32();
        const std::size_t fail_label = out.bytes.size();
        for (const auto jump : fail) out.patch(jump, fail_label);

        const u32 retired = index;
        const u32 last_pc =
            retired == 0u ? cs.block_pc : cs.pcs[retired - 1u];
        const u32 last_op =
            retired == 0u ? 0u : cs.words[retired - 1u];
        if (retired == 0u) {
            out.epilogue_return(0u);
        } else {
            emit_commit_sequential(
                out, retired,
                cs.pcs[index],
                last_pc, last_op);
        }
        const std::size_t continue_label = out.bytes.size();
        out.patch(skip_fail, continue_label);
        (void)resume;
        ++cs.fastmem_loads;
        return true;
    }

    if (is_store(instruction)) {
        const u32 width = ir.memory_width;
        const u32 alignment_mask = ir.alignment_mask;
        if (width == 0u) return false;

        std::vector<std::size_t> fail;
        emit_fastmem_address(
            out, rs, imm, width, fail, true, alignment_mask);

        if (opcode == 0x39u) { // SWC1
            out.load32(
                RDX,
                RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, fpr) + rt * sizeof(u32)));
            out.store_indexed(RBP, RAX, RDX, 4u);
        } else if (opcode == 0x3Eu) { // SQC2
            out.load64(
                RDX,
                RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, vu_vf) +
                    rt * sizeof(EeGpr)));
            out.store_indexed(RBP, RAX, RDX, 8u);
            out.load64(
                RDX,
                RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, vu_vf) +
                    rt * sizeof(EeGpr) + sizeof(u64)));
            out.store_indexed(RBP, RAX, RDX, 8u, 8u);
        } else if (opcode == 0x1Fu) { // SQ
            out.load_guest(RDX, rt);
            out.store_indexed(RBP, RAX, RDX, 8u);
            out.load64(
                RDX,
                RBX,
                static_cast<u32>(
                    offsetof(EeCpuState, gpr) +
                    rt * sizeof(EeGpr) + sizeof(u64)));
            out.store_indexed(RBP, RAX, RDX, 8u, 8u);
        } else {
            out.load_guest(RDX, rt, width != 8u);
            out.store_indexed(RBP, RAX, RDX, width);
        }

        if ((opcode == 0x38u || opcode == 0x3Cu) && rt != 0u) {
            out.mov_r64_imm(RDX, 1u);
            out.store_guest(rt, RDX);
        }

        std::vector<std::size_t> selfmod;
        emit_store_generation_barrier(
            out,
            cs.code_pages,
            cs.code_page_count,
            selfmod,
            (opcode == 0x1Fu || opcode == 0x3Eu) ? 2u : 1u);

        const std::size_t skip_fail = out.jmp32();
        const std::size_t fail_label = out.bytes.size();
        for (const auto jump : fail) out.patch(jump, fail_label);
        const u32 prior = index;
        if (prior == 0u) {
            out.epilogue_return(0u);
        } else {
            emit_commit_sequential(
                out, prior,
                cs.pcs[index],
                cs.pcs[prior - 1u],
                cs.words[prior - 1u]);
        }

        const std::size_t selfmod_label = out.bytes.size();
        for (const auto jump : selfmod)
            out.patch(jump, selfmod_label);
        emit_commit_sequential(
            out, index + 1u,
            cs.pcs[index] + 4u,
            cs.pcs[index],
            instruction);

        const std::size_t continue_label = out.bytes.size();
        out.patch(skip_fail, continue_label);

        ++cs.fastmem_stores;
        return true;
    }

    return false;
}

#endif // VIBESTATION_EE_DYNAREC_X64

} // namespace

EeDynarec::EeDynarec()
    : blocks_(kBlockCacheEntries) {}

EeDynarec::~EeDynarec() {
    release_code_cache();
}

void EeDynarec::release_code_cache() {
#if defined(VIBESTATION_EE_DYNAREC_X64)
    for (const auto& page : code_pages_) {
        if (page.address != nullptr) release_page(page.address);
    }
#endif
    code_pages_.clear();
    std::fill(blocks_.begin(), blocks_.end(), Block{});
}

void EeDynarec::clear() {
    release_code_cache();
    compiled_blocks_ = 0;
    executed_blocks_ = 0;
    executed_instructions_ = 0;
    link_hits_ = 0;
    link_misses_ = 0;
    guard_exits_ = 0;
    code_invalidation_exits_ = 0;
    cop0_write_exits_ = 0;
    fastmem_loads_ = 0;
    fastmem_stores_ = 0;
    register_cache_hits_ = 0;
    register_cache_flushes_ = 0;
    cache_flushes_ = 0;
    fused_static_jumps_ = 0;
    fused_conditional_branches_ = 0;
    conditional_side_exits_ = 0;
    dispatch_calls_ = 0;
    deadline_exits_ = 0;
    unsupported_exits_ = 0;
    unsupported_opcodes_.fill(0u);
}

EeDynarec::Block* EeDynarec::lookup_or_compile(
    u32 pc,
    u32 compile_limit,
    u8* ram_data,
    u32* page_generations,
    u8* code_page_tracked) {
#if !defined(VIBESTATION_EE_DYNAREC_X64)
    (void)pc;
    (void)compile_limit;
    (void)ram_data;
    (void)page_generations;
    (void)code_page_tracked;
    return nullptr;
#else
    if (compile_limit == 0u ||
        ram_data == nullptr ||
        page_generations == nullptr ||
        code_page_tracked == nullptr) {
        return nullptr;
    }

    bool valid = false;
    const u32 physical = ram_physical(pc, valid);
    if (!valid || (physical & 3u) != 0u) return nullptr;

    const u32 page = physical / kPageSize;
    code_page_tracked[page] = 1u;
    const u32 generation = page_generations[page];
    const u32 compile_budget =
        compile_budget_for_limit(compile_limit);

    // A PC can execute under very different event deadlines. Keep the
    // 1-16 instruction exact-timing variants separate from the 16/32/64
    // throughput variants so an 8:1 IOP deadline cannot permanently poison
    // the cache with a tiny block.
    const u64 key =
        (static_cast<u64>(pc >> 2u) * 11400714819323198485ull) ^
        (static_cast<u64>(compile_budget) * 0x9E3779B97F4A7C15ull);
    const std::size_t index =
        static_cast<std::size_t>(key ^ (key >> 32u)) &
        (blocks_.size() - 1u);
    Block& cached = blocks_[index];
    bool cached_pages_valid =
        cached.function != nullptr &&
        cached.pc == pc &&
        cached.page_generation == generation &&
        cached.compile_budget == compile_budget;
    if (cached_pages_valid) {
        for (u32 i = 0u; i < cached.source_page_count; ++i) {
            if (page_generations[cached.source_pages[i]] !=
                cached.source_generations[i]) {
                cached_pages_valid = false;
                break;
            }
        }
    }
    if (cached_pages_valid) {
        return &cached;
    }

    CompileState cs;
    cs.block_pc = pc;
    cs.code_page = page;
    cs.code_pages[0] = page;
    cs.code_page_count = 1u;

    const u32 available =
        (std::min)(kMaxBlockInstructions, compile_budget);
    u32 fetch_pc = pc;

    auto already_in_trace = [&](u32 candidate) {
        for (u32 i = 0u; i < cs.count; ++i) {
            if (cs.pcs[i] == candidate) return true;
        }
        return false;
    };
    auto append = [&](u32 guest_pc, u32 instruction) {
        cs.pcs[cs.count] = guest_pc;
        cs.words[cs.count] = instruction;
        cs.ir[cs.count] =
            decode_ee_dynarec_ir(guest_pc, instruction);
        ++cs.count;
    };
    auto ensure_code_page = [&](u32 physical_address) {
        const u32 candidate = physical_address / kPageSize;
        for (u32 i = 0u; i < cs.code_page_count; ++i) {
            if (cs.code_pages[i] == candidate) return true;
        }
        if (cs.code_page_count >= cs.code_pages.size()) {
            return false;
        }
        cs.code_pages[cs.code_page_count++] = candidate;
        code_page_tracked[candidate] = 1u;
        return true;
    };

    while (cs.count < available) {
        bool fetch_valid = false;
        const u32 fetch_physical =
            ram_physical(fetch_pc, fetch_valid);
        if (!fetch_valid || (fetch_physical & 3u) != 0u ||
            !ensure_code_page(fetch_physical)) {
            break;
        }

        const u32 instruction =
            read_word(ram_data, fetch_physical);
        const ControlKind control =
            control_kind(instruction);
        if (control != ControlKind::None) {
            if (cs.count + 2u > available) break;

            const u32 delay_pc = fetch_pc + 4u;
            bool delay_valid = false;
            const u32 delay_physical =
                ram_physical(delay_pc, delay_valid);
            if (!delay_valid ||
                !ensure_code_page(delay_physical)) {
                break;
            }
            const u32 delay =
                read_word(ram_data, delay_physical);
            const EeDynarecIrInstruction delay_ir =
                decode_ee_dynarec_ir(delay_pc, delay);
            if (!supported_noncontrol(delay) ||
                delay_ir.has(EeIrPreciseExit) ||
                delay_ir.has(EeIrReadsMemory) ||
                delay_ir.has(EeIrWritesMemory)) {
                break;
            }

            // Static J/JAL edges can be fused directly into this native
            // trace while the source-page set remains small. Every source
            // page is generation-tracked, so cross-page fusion retains the
            // same self-modifying-code guarantees as a one-page block.
            if (control == ControlKind::J ||
                control == ControlKind::Jal) {
                const u32 target =
                    ((fetch_pc + 4u) & 0xF0000000u) |
                    ((instruction & 0x03FFFFFFu) << 2u);
                bool target_valid = false;
                const u32 target_physical =
                    ram_physical(target, target_valid);
                const bool creates_cycle =
                    target == fetch_pc ||
                    target == delay_pc ||
                    already_in_trace(target);
                if (target_valid &&
                    !creates_cycle &&
                    ensure_code_page(target_physical)) {
                    append(fetch_pc, instruction);
                    append(delay_pc, delay);
                    ++cs.fused_static_jumps;
                    fetch_pc = target;
                    continue;
                }
            }

            const u32 rs = (instruction >> 21) & 31u;
            const u32 rt = (instruction >> 16) & 31u;
            const bool cop1_branch =
                control == ControlKind::Bc1f ||
                control == ControlKind::Bc1t ||
                control == ControlKind::Bc1fl ||
                control == ControlKind::Bc1tl;
            if ((!cop1_branch && writes_gpr(delay, rs)) ||
                ((control == ControlKind::Beq ||
                  control == ControlKind::Bne ||
                  control == ControlKind::Beql ||
                  control == ControlKind::Bnel) &&
                 writes_gpr(delay, rt))) {
                break;
            }

            // Keep the predicted direction of safe non-likely integer
            // branches inside this native trace. Backward edges are normally
            // loop back-edges and are predicted taken; forward edges are
            // predicted fallthrough. The compile budget bounds unrolling, so
            // repeated guest PCs are safe and retain the exact event deadline.
            if (is_fusable_conditional(control) &&
                cs.fused_conditional_count <
                    cs.fused_conditionals.size() &&
                cs.count + 3u <= available) {
                const u32 target =
                    branch_target(fetch_pc, instruction);
                const u32 fallthrough = fetch_pc + 8u;
                const bool predicted_taken =
                    target <= fetch_pc;
                const u32 predicted_pc =
                    predicted_taken ? target : fallthrough;
                const u32 side_exit_pc =
                    predicted_taken ? fallthrough : target;

                bool predicted_valid = false;
                const u32 predicted_physical =
                    ram_physical(predicted_pc, predicted_valid);
                if (predicted_valid &&
                    predicted_pc != delay_pc &&
                    ensure_code_page(predicted_physical)) {
                    FusedConditionalEdge& edge =
                        cs.fused_conditionals[
                            cs.fused_conditional_count++];
                    edge.branch_index = cs.count;
                    edge.delay_index = cs.count + 1u;
                    edge.control = control;
                    edge.taken_pc = target;
                    edge.fallthrough_pc = fallthrough;
                    edge.side_exit_pc = side_exit_pc;
                    edge.predicted_taken = predicted_taken;

                    append(fetch_pc, instruction);
                    append(delay_pc, delay);
                    fetch_pc = predicted_pc;
                    continue;
                }
            }

            cs.control_index = cs.count;
            append(fetch_pc, instruction);
            append(delay_pc, delay);
            cs.control = control;
            break;
        }

        if (!supported_noncontrol(instruction)) break;
        if (is_mtc0(instruction) &&
            ((instruction >> 11) & 31u) == 9u &&
            cs.count != 0u) {
            // Prior instructions must retire before MTC0 Count overwrites
            // Count. Compile the write as the next block instead.
            break;
        }

        append(fetch_pc, instruction);
        fetch_pc += 4u;
        if (is_mtc0(instruction) ||
            is_cop0_ei_di(instruction)) {
            cs.ends_cop0_write = true;
            break;
        }
    }

    if (cs.count == 0u) return nullptr;
    cs.trace_next_pc = fetch_pc;

    std::array<u32, 32> scores{};
    bool uses_hi = false;
    bool uses_lo = false;
    for (u32 i = 0u; i < cs.count; ++i) {
        score_registers(cs.words[i], scores);
        if ((cs.words[i] >> 26) == 0u) {
            const u32 funct = cs.words[i] & 63u;
            uses_hi =
                uses_hi || funct == 0x10u || funct == 0x11u ||
                (funct >= 0x18u && funct <= 0x1Bu);
            uses_lo =
                uses_lo || funct == 0x12u || funct == 0x13u ||
                (funct >= 0x18u && funct <= 0x1Bu);
        }
    }

    for (u32 slot = 0u; slot < 4u; ++slot) {
        u32 best_reg = 0u;
        u32 best_score = 0u;
        for (u32 reg = 1u; reg < 32u; ++reg) {
            bool already = false;
            for (u32 prior = 0u; prior < slot; ++prior) {
                if (cs.out.guest_cache[prior] == reg) {
                    already = true;
                    break;
                }
            }
            if (!already && scores[reg] > best_score) {
                best_score = scores[reg];
                best_reg = reg;
            }
        }
        if (best_score == 0u) break;
        cs.out.guest_cache[slot] = best_reg;
    }
    cs.out.cache_hi = uses_hi;
    cs.out.cache_lo = uses_lo;
    cs.out.prologue();

    std::vector<std::pair<std::size_t, u32>> exits;
    const bool has_control = cs.control != ControlKind::None;
    const u32 body_count =
        has_control ? cs.count - 2u : cs.count;

    for (u32 i = 0u; i < body_count; ++i) {
        bool fused_branch = false;
        const FusedConditionalEdge* delay_edge = nullptr;
        for (u32 edge_index = 0u;
             edge_index < cs.fused_conditional_count;
             ++edge_index) {
            const auto& edge =
                cs.fused_conditionals[edge_index];
            if (edge.branch_index == i) {
                fused_branch = true;
                break;
            }
            if (edge.delay_index == i) {
                delay_edge = &edge;
            }
        }

        if (!fused_branch &&
            !emit_body(cs, i, exits)) {
            return nullptr;
        }
        if (delay_edge != nullptr &&
            !emit_fused_conditional_side_exit(
                cs, *delay_edge)) {
            return nullptr;
        }
    }

    u32 taken_pc = 0u;
    u32 fallthrough_pc = 0u;
    bool conditional = false;
    bool likely = false;

    if (has_control) {
        const u32 branch_index = cs.count - 2u;
        const u32 delay_index = cs.count - 1u;
        const EeDynarecIrInstruction& branch_ir =
            cs.ir[branch_index];
        const u32 instruction = branch_ir.word;
        const u32 delay = cs.ir[delay_index].word;
        const u32 branch_pc = branch_ir.pc;
        const u32 rs = branch_ir.rs;
        const u32 rt = branch_ir.rt;

        bool preserved_dynamic_target = false;
        bool preserved_link_branch_source = false;
        if (cs.control == ControlKind::Jalr) {
            // JALR may legally use rd == rs. Preserve the target before the
            // link write so the architectural source value wins.
            cs.out.load_guest(R10, rs, true);
            preserved_dynamic_target = true;
        }
        if (cs.control == ControlKind::Bltzal ||
            cs.control == ControlKind::Bgezal ||
            cs.control == ControlKind::Bltzall ||
            cs.control == ControlKind::Bgezall) {
            // Likewise BLTZAL/BGEZAL may test r31 while also writing r31.
            cs.out.load_guest(R10, rs);
            preserved_link_branch_source = true;
        }

        if (cs.control == ControlKind::Jal) {
            cs.out.mov_r64_imm(
                RAX,
                static_cast<u64>(
                    static_cast<s64>(
                        static_cast<s32>(branch_pc + 8u))));
            cs.out.store_guest(31u, RAX);
        } else if (cs.control == ControlKind::Jalr) {
            const u32 rd = (instruction >> 11) & 31u;
            cs.out.mov_r64_imm(
                RAX,
                static_cast<u64>(
                    static_cast<s64>(
                        static_cast<s32>(branch_pc + 8u))));
            cs.out.store_guest(rd, RAX);
        } else if (
            cs.control == ControlKind::Bltzal ||
            cs.control == ControlKind::Bgezal ||
            cs.control == ControlKind::Bltzall ||
            cs.control == ControlKind::Bgezall) {
            cs.out.mov_r64_imm(
                RAX,
                static_cast<u64>(
                    static_cast<s64>(
                        static_cast<s32>(branch_pc + 8u))));
            cs.out.store_guest(31u, RAX);
        }

        fallthrough_pc = branch_pc + 8u;

        const bool is_likely =
            cs.control == ControlKind::Beql ||
            cs.control == ControlKind::Bnel ||
            cs.control == ControlKind::Blezl ||
            cs.control == ControlKind::Bgtzl ||
            cs.control == ControlKind::Bltzl ||
            cs.control == ControlKind::Bgezl ||
            cs.control == ControlKind::Bltzall ||
            cs.control == ControlKind::Bgezall ||
            cs.control == ControlKind::Bc1fl ||
            cs.control == ControlKind::Bc1tl;

        if (is_likely) {
            conditional = true;
            likely = true;
            taken_pc = branch_target(branch_pc, instruction);

            const bool cop1_likely =
                cs.control == ControlKind::Bc1fl ||
                cs.control == ControlKind::Bc1tl;
            if (cop1_likely) {
                cs.out.load32(
                    RAX, RBX,
                    static_cast<u32>(
                        offsetof(EeCpuState, fcr) +
                        31u * sizeof(u32)));
                cs.out.and_r32_imm32(RAX, 0x00800000u);
                cs.out.test_rr64(RAX, RAX);
            } else {
                if (preserved_link_branch_source) {
                    cs.out.mov_rr64(RAX, R10);
                } else {
                    cs.out.load_guest(RAX, rs);
                }
                if (cs.control == ControlKind::Beql ||
                    cs.control == ControlKind::Bnel) {
                    cs.out.load_guest(RDX, rt);
                    cs.out.cmp_rr64(RAX, RDX);
                } else {
                    cs.out.test_rr64(RAX, RAX);
                }
            }

            u8 take_cc = 0x4u;
            switch (cs.control) {
            case ControlKind::Beql: take_cc = 0x4u; break;
            case ControlKind::Bnel: take_cc = 0x5u; break;
            case ControlKind::Blezl: take_cc = 0xEu; break;
            case ControlKind::Bgtzl: take_cc = 0xFu; break;
            case ControlKind::Bltzl:
            case ControlKind::Bltzall: take_cc = 0xCu; break;
            case ControlKind::Bgezl:
            case ControlKind::Bgezall: take_cc = 0xDu; break;
            case ControlKind::Bc1fl: take_cc = 0x4u; break; // condition false
            case ControlKind::Bc1tl: take_cc = 0x5u; break; // condition true
            default: break;
            }

            const std::size_t take = cs.out.jcc32(take_cc);

            // Not taken: retire the branch but annul its delay slot.
            emit_commit_sequential(
                cs.out,
                cs.count - 1u,
                fallthrough_pc,
                branch_pc,
                instruction);

            const std::size_t taken_label = cs.out.bytes.size();
            cs.out.patch(take, taken_label);
            if (!emit_body(cs, delay_index, exits)) return nullptr;
            cs.out.mov_r32_imm(RCX, taken_pc);
            emit_commit_dynamic_pc(
                cs.out,
                cs.count,
                RCX,
                branch_pc + 4u,
                delay);
        } else {
            if (!emit_body(cs, delay_index, exits)) return nullptr;
        }

        if (!is_likely) switch (cs.control) {
        case ControlKind::J:
        case ControlKind::Jal:
            taken_pc =
                ((branch_pc + 4u) & 0xF0000000u) |
                ((instruction & 0x03FFFFFFu) << 2u);
            cs.out.mov_r32_imm(RCX, taken_pc);
            emit_commit_dynamic_pc(
                cs.out, cs.count, RCX,
                branch_pc + 4u, delay);
            break;
        case ControlKind::Jr:
        case ControlKind::Jalr:
            if (!preserved_dynamic_target) {
                cs.out.load_guest(R10, rs, true);
            }
            emit_commit_dynamic_pc(
                cs.out, cs.count, R10,
                branch_pc + 4u, delay);
            break;
        case ControlKind::Beq:
        case ControlKind::Bne:
        case ControlKind::Blez:
        case ControlKind::Bgtz:
        case ControlKind::Bltz:
        case ControlKind::Bgez:
        case ControlKind::Bltzal:
        case ControlKind::Bgezal:
        case ControlKind::Bc1f:
        case ControlKind::Bc1t: {
            conditional = true;
            taken_pc = branch_target(branch_pc, instruction);
            const bool cop1_branch =
                cs.control == ControlKind::Bc1f ||
                cs.control == ControlKind::Bc1t;
            if (cop1_branch) {
                cs.out.load32(
                    RAX, RBX,
                    static_cast<u32>(
                        offsetof(EeCpuState, fcr) +
                        31u * sizeof(u32)));
                cs.out.and_r32_imm32(RAX, 0x00800000u);
                cs.out.test_rr64(RAX, RAX);
            } else {
                if (preserved_link_branch_source) {
                    cs.out.mov_rr64(RAX, R10);
                } else {
                    cs.out.load_guest(RAX, rs);
                }
                if (cs.control == ControlKind::Beq ||
                    cs.control == ControlKind::Bne) {
                    cs.out.load_guest(RDX, rt);
                    cs.out.cmp_rr64(RAX, RDX);
                } else {
                    cs.out.test_rr64(RAX, RAX);
                }
            }

            u8 take_cc = 0x4u; // JE
            switch (cs.control) {
            case ControlKind::Beq: take_cc = 0x4u; break;
            case ControlKind::Bne: take_cc = 0x5u; break;
            case ControlKind::Blez: take_cc = 0xEu; break; // JLE
            case ControlKind::Bgtz: take_cc = 0xFu; break; // JG
            case ControlKind::Bltz:
            case ControlKind::Bltzal: take_cc = 0xCu; break; // JL
            case ControlKind::Bgez:
            case ControlKind::Bgezal: take_cc = 0xDu; break; // JGE
            case ControlKind::Bc1f: take_cc = 0x4u; break;
            case ControlKind::Bc1t: take_cc = 0x5u; break;
            default: break;
            }

            const std::size_t take =
                cs.out.jcc32(take_cc);
            cs.out.mov_r32_imm(RCX, fallthrough_pc);
            const std::size_t commit_jump =
                cs.out.jmp32();
            const std::size_t taken_label =
                cs.out.bytes.size();
            cs.out.patch(take, taken_label);
            cs.out.mov_r32_imm(RCX, taken_pc);
            const std::size_t commit =
                cs.out.bytes.size();
            cs.out.patch(commit_jump, commit);
            emit_commit_dynamic_pc(
                cs.out, cs.count, RCX,
                branch_pc + 4u, delay);
            break;
        }
        default:
            return nullptr;
        }
    } else {
        emit_commit_sequential(
            cs.out,
            cs.count,
            cs.trace_next_pc,
            cs.pcs[cs.count - 1u],
            cs.words[cs.count - 1u]);
    }

    if (cs.out.bytes.empty() ||
        cs.out.bytes.size() > kCodePageSize / 2u) {
        return nullptr;
    }

    if (code_pages_.empty() ||
        code_pages_.back().used + cs.out.bytes.size() >
            kCodePageSize) {
        if (code_pages_.size() >= kMaxCodePages) {
            release_code_cache();
            ++cache_flushes_;
        }
        void* page_memory = allocate_page();
        if (page_memory == nullptr) return nullptr;
        code_pages_.push_back({page_memory, 0u});
    }

    CodePage& code_page = code_pages_.back();
    if (!protect_page(code_page.address, false)) return nullptr;
    auto* destination =
        static_cast<u8*>(code_page.address) + code_page.used;
    std::memcpy(
        destination,
        cs.out.bytes.data(),
        cs.out.bytes.size());
    flush_code(destination, cs.out.bytes.size());
    if (!protect_page(code_page.address, true)) return nullptr;
    code_page.used +=
        (cs.out.bytes.size() + 15u) & ~std::size_t{15u};

    cached = {};
    cached.pc = pc;
    cached.page_generation = generation;
    cached.instruction_count = cs.count;
    cached.sequential_pc =
        has_control ? 0u : cs.trace_next_pc;
    cached.taken_pc = taken_pc;
    cached.fallthrough_pc = fallthrough_pc;
    cached.code_page = page;
    cached.compile_budget = compile_budget;
    cached.source_page_count =
        static_cast<u8>(cs.code_page_count);
    for (u32 i = 0u; i < cs.code_page_count; ++i) {
        cached.source_pages[i] = cs.code_pages[i];
        cached.source_generations[i] =
            page_generations[cs.code_pages[i]];
    }
    cached.side_exit_count =
        static_cast<u8>(cs.fused_conditional_count);
    for (u32 i = 0u; i < cs.fused_conditional_count; ++i) {
        cached.side_exits[i].retired =
            cs.fused_conditionals[i].delay_index + 1u;
        cached.side_exits[i].target_pc =
            cs.fused_conditionals[i].side_exit_pc;
    }
    cached.fastmem_loads = cs.fastmem_loads;
    cached.fastmem_stores = cs.fastmem_stores;
    cached.cached_register_uses = cs.out.register_cache_uses;
    cached.conditional_branch = conditional;
    cached.branch_likely = likely;
    cached.control_flow = has_control;
    cached.ends_with_cop0_write = cs.ends_cop0_write;
    cached.function =
        reinterpret_cast<NativeFunction>(destination);
    cached.code_page_owner = code_page.address;

    fused_static_jumps_ += cs.fused_static_jumps;
    fused_conditional_branches_ += cs.fused_conditional_count;
    ++compiled_blocks_;
    return &cached;
#endif
}

EeDynarec::Block* EeDynarec::resolve_link(
    Block& source,
    u32 target_pc,
    u32 compile_limit,
    Block*& slot,
    u32& generation_slot,
    u8* ram_data,
    u32* page_generations,
    u8* code_page_tracked) {
    bool valid = false;
    const u32 physical = ram_physical(target_pc, valid);
    if (!valid) return nullptr;
    const u32 generation =
        page_generations[physical / kPageSize];

    const u32 requested_budget =
        compile_budget_for_limit(compile_limit);
    bool linked_pages_valid =
        slot != nullptr &&
        slot->pc == target_pc &&
        slot->page_generation == generation &&
        generation_slot == generation &&
        slot->compile_budget == requested_budget &&
        slot->function != nullptr;
    if (linked_pages_valid) {
        for (u32 i = 0u; i < slot->source_page_count; ++i) {
            if (page_generations[slot->source_pages[i]] !=
                slot->source_generations[i]) {
                linked_pages_valid = false;
                break;
            }
        }
    }
    if (linked_pages_valid) {
        ++link_hits_;
        return slot;
    }

    ++link_misses_;
    slot = lookup_or_compile(
        target_pc,
        compile_limit,
        ram_data,
        page_generations,
        code_page_tracked);
    generation_slot =
        slot != nullptr ? slot->page_generation : 0u;
    (void)source;
    return slot;
}

EeDynarec::RunResult EeDynarec::execute(
    EeCpuState& state,
    u32 maximum_instructions,
    u8* ram_data,
    u32* page_generations,
    u8* code_page_tracked) {
    RunResult result{};
    ++dispatch_calls_;
    if (maximum_instructions == 0u ||
        ram_data == nullptr ||
        page_generations == nullptr ||
        code_page_tracked == nullptr) {
        return result;
    }

#if !defined(VIBESTATION_EE_DYNAREC_X64)
    (void)state;
    return result;
#else
    auto block_code_valid = [&](const Block& candidate) {
        if (candidate.source_page_count == 0u) return false;
        for (u32 i = 0u; i < candidate.source_page_count; ++i) {
            if (page_generations[candidate.source_pages[i]] !=
                candidate.source_generations[i]) {
                return false;
            }
        }
        return true;
    };

    auto record_unsupported = [&](u32 pc) {
        bool valid = false;
        const u32 physical = ram_physical(pc, valid);
        if (!valid || physical > kRamSize - 4u) return;
        const u32 instruction = read_word(ram_data, physical);
        ++unsupported_opcodes_[instruction >> 26u];
    };

    Block* block = lookup_or_compile(
        state.pc,
        maximum_instructions,
        ram_data,
        page_generations,
        code_page_tracked);
    if (block == nullptr) {
        record_unsupported(state.pc);
        ++unsupported_exits_;
        return result;
    }

    u32 remaining = maximum_instructions;
    while (block != nullptr) {
        if (block->instruction_count == 0u ||
            block->instruction_count > remaining) {
            result.reason = ExitReason::Deadline;
            ++deadline_exits_;
            return result;
        }

        if (!block_code_valid(*block)) {
            result.reason = ExitReason::CodeInvalidated;
            ++code_invalidation_exits_;
            return result;
        }

        const u32 retired = block->function(
            &state,
            ram_data,
            page_generations,
            code_page_tracked);
        ++executed_blocks_;
        executed_instructions_ += retired;
        ++register_cache_flushes_;

        result.retired += retired;

        SideExit* fused_side_exit = nullptr;
        if (retired < block->instruction_count) {
            for (u32 i = 0u; i < block->side_exit_count; ++i) {
                SideExit& candidate = block->side_exits[i];
                if (candidate.retired == retired &&
                    candidate.target_pc == state.pc) {
                    fused_side_exit = &candidate;
                    break;
                }
            }
        }

        if (fused_side_exit != nullptr) {
            if (!block_code_valid(*block)) {
                result.reason = ExitReason::CodeInvalidated;
                ++code_invalidation_exits_;
                return result;
            }

            ++conditional_side_exits_;
            remaining -= retired;
            if (remaining == 0u) {
                result.reason = ExitReason::Deadline;
                ++deadline_exits_;
                return result;
            }

            Block* next = resolve_link(
                *block,
                fused_side_exit->target_pc,
                remaining,
                fused_side_exit->link,
                fused_side_exit->link_generation,
                ram_data,
                page_generations,
                code_page_tracked);
            if (next == nullptr) {
                record_unsupported(fused_side_exit->target_pc);
                result.reason = ExitReason::Unsupported;
                ++unsupported_exits_;
                return result;
            }
            block = next;
            continue;
        }

        const bool likely_annul =
            block->branch_likely &&
            retired + 1u == block->instruction_count &&
            state.pc == block->fallthrough_pc;
        if (retired < block->instruction_count && !likely_annul) {
            if (!block_code_valid(*block)) {
                result.reason = ExitReason::CodeInvalidated;
                ++code_invalidation_exits_;
            } else {
                result.reason = ExitReason::Guard;
                ++guard_exits_;
            }
            return result;
        }

        fastmem_loads_ += block->fastmem_loads;
        fastmem_stores_ += block->fastmem_stores;
        register_cache_hits_ += block->cached_register_uses;

        remaining -= retired;
        if (block->ends_with_cop0_write) {
            result.reason = ExitReason::Cop0Write;
            ++cop0_write_exits_;
            return result;
        }
        if (remaining == 0u) {
            result.reason = ExitReason::Deadline;
            ++deadline_exits_;
            return result;
        }

        const u32 next_pc = state.pc;
        Block* next = nullptr;
        if (block->conditional_branch) {
            if (next_pc == block->taken_pc) {
                next = resolve_link(
                    *block, next_pc, remaining,
                    block->taken_link,
                    block->taken_link_generation,
                    ram_data, page_generations,
                    code_page_tracked);
            } else if (next_pc == block->fallthrough_pc) {
                next = resolve_link(
                    *block, next_pc, remaining,
                    block->fallthrough_link,
                    block->fallthrough_link_generation,
                    ram_data, page_generations,
                    code_page_tracked);
            }
        } else if (block->control_flow) {
            if (block->taken_pc != 0u &&
                next_pc == block->taken_pc) {
                next = resolve_link(
                    *block, next_pc, remaining,
                    block->taken_link,
                    block->taken_link_generation,
                    ram_data, page_generations,
                    code_page_tracked);
            } else {
                // JR/JALR targets are dynamic. Cache the most recent target
                // in the sequential link slot; a target change simply misses.
                next = resolve_link(
                    *block, next_pc, remaining,
                    block->sequential_link,
                    block->sequential_link_generation,
                    ram_data, page_generations,
                    code_page_tracked);
            }
        } else {
            next = resolve_link(
                *block, next_pc, remaining,
                block->sequential_link,
                block->sequential_link_generation,
                ram_data, page_generations,
                code_page_tracked);
        }

        if (next == nullptr) {
            record_unsupported(next_pc);
            result.reason = ExitReason::Unsupported;
            ++unsupported_exits_;
            return result;
        }
        block = next;
    }

    result.reason = ExitReason::Unsupported;
    ++unsupported_exits_;
    return result;
#endif
}

} // namespace ps2
