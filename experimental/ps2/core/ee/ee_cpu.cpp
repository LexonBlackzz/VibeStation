#include "core/ee/ee_cpu.h"

#include "core/memory/ee_bus.h"

#include <iomanip>
#include <limits>
#include <sstream>

namespace ps2 {
namespace {

std::string hex32(u32 value) {
    std::ostringstream out;
    out << "0x" << std::uppercase << std::hex
        << std::setw(8) << std::setfill('0') << value;
    return out.str();
}

u64 sign_extend_32(u32 value) {
    return static_cast<u64>(static_cast<s64>(static_cast<s32>(value)));
}

void divide_signed32(u32 lhs, u32 rhs, u64& lo, u64& hi) {
    const s32 a = static_cast<s32>(lhs);
    const s32 b = static_cast<s32>(rhs);

    if (lhs == 0x80000000u && rhs == 0xFFFFFFFFu) {
        lo = sign_extend_32(0x80000000u);
        hi = 0;
        return;
    }

    if (b != 0) {
        lo = sign_extend_32(static_cast<u32>(a / b));
        hi = sign_extend_32(static_cast<u32>(a % b));
    } else {
        lo = sign_extend_32(static_cast<u32>(a < 0 ? 1 : -1));
        hi = sign_extend_32(static_cast<u32>(a));
    }
}

void divide_unsigned32(u32 lhs, u32 rhs, u64& lo, u64& hi) {
    if (rhs != 0) {
        lo = sign_extend_32(lhs / rhs);
        hi = sign_extend_32(lhs % rhs);
    } else {
        lo = sign_extend_32(0xFFFFFFFFu);
        hi = sign_extend_32(lhs);
    }
}

void multiply_signed32(u32 lhs, u32 rhs, u64& lo, u64& hi) {
    const s64 result =
        static_cast<s64>(static_cast<s32>(lhs)) *
        static_cast<s64>(static_cast<s32>(rhs));
    lo = sign_extend_32(static_cast<u32>(result));
    hi = sign_extend_32(static_cast<u32>(static_cast<u64>(result) >> 32));
}

void multiply_unsigned32(u32 lhs, u32 rhs, u64& lo, u64& hi) {
    const u64 result = static_cast<u64>(lhs) * static_cast<u64>(rhs);
    lo = sign_extend_32(static_cast<u32>(result));
    hi = sign_extend_32(static_cast<u32>(result >> 32));
}

} // namespace

void EeCpu::reset(u32 entry_point) {
    state_ = {};
    state_.pc = entry_point;
    state_.next_pc = entry_point + 4;
    state_.cop0[1] = 47;
    state_.cop0[12] = 0x70400004;
    state_.cop0[15] = 0x00002E20;
    state_.cop0[16] = 0x00000440;
    state_.fcr[0] = 0x00002E30;
    state_.fcr[31] = 0x01000001;
    halted_ = false;
    halt_reason_.clear();
}

void EeCpu::clear_halt() {
    halted_ = false;
    halt_reason_.clear();
}

s16 EeCpu::immediate(u32 instruction) {
    return static_cast<s16>(instruction & 0xFFFFu);
}

u32 EeCpu::branch_target(u32 pc, s16 imm) {
    return pc + 4u + static_cast<u32>(static_cast<s32>(imm) * 4);
}

u64 EeCpu::sign_extend_word(u32 value) {
    return sign_extend_32(value);
}

u64 EeCpu::gpr_u64(u32 index) const {
    return state_.gpr[index & 31u].lo;
}

s64 EeCpu::gpr_s64(u32 index) const {
    return static_cast<s64>(gpr_u64(index));
}

void EeCpu::write_gpr64(u32 index, u64 value) {
    if ((index & 31u) != 0) {
        state_.gpr[index & 31u].lo = value;
    }
}

void EeCpu::write_gpr_word(u32 index, u32 value) {
    write_gpr64(index, sign_extend_word(value));
}

void EeCpu::branch_likely_not_taken(u32 pc) {
    state_.pc = pc + 8u;
    state_.next_pc = pc + 12u;
}

bool EeCpu::fail(
    u32 pc,
    u32 instruction,
    const std::string& reason,
    std::string& error) {
    halted_ = true;
    halt_reason_ =
        reason + " at PC " + hex32(pc) +
        " (instruction " + hex32(instruction) + ")";
    error = halt_reason_;
    return false;
}

bool EeCpu::execute_special(
    u32 pc,
    u32 instruction,
    std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;
    const u32 sa = (instruction >> 6) & 31u;
    const u32 funct = instruction & 63u;

    switch (funct) {
    case 0x00: // SLL / NOP
        write_gpr_word(rd, static_cast<u32>(gpr_u64(rt)) << sa);
        return true;
    case 0x02: // SRL
        write_gpr_word(rd, static_cast<u32>(gpr_u64(rt)) >> sa);
        return true;
    case 0x03: // SRA
        write_gpr_word(
            rd,
            static_cast<u32>(
                static_cast<s32>(static_cast<u32>(gpr_u64(rt))) >> sa));
        return true;
    case 0x04: // SLLV
        write_gpr_word(
            rd,
            static_cast<u32>(gpr_u64(rt)) <<
                (static_cast<u32>(gpr_u64(rs)) & 31u));
        return true;
    case 0x06: // SRLV
        write_gpr_word(
            rd,
            static_cast<u32>(gpr_u64(rt)) >>
                (static_cast<u32>(gpr_u64(rs)) & 31u));
        return true;
    case 0x07: // SRAV
        write_gpr_word(
            rd,
            static_cast<u32>(
                static_cast<s32>(static_cast<u32>(gpr_u64(rt))) >>
                (static_cast<u32>(gpr_u64(rs)) & 31u)));
        return true;
    case 0x08: // JR
        state_.next_pc = static_cast<u32>(gpr_u64(rs));
        return true;
    case 0x09: // JALR
        write_gpr_word(rd, pc + 8u);
        state_.next_pc = static_cast<u32>(gpr_u64(rs));
        return true;
    case 0x0A: // MOVZ
        if (gpr_u64(rt) == 0) {
            write_gpr64(rd, gpr_u64(rs));
        }
        return true;
    case 0x0B: // MOVN
        if (gpr_u64(rt) != 0) {
            write_gpr64(rd, gpr_u64(rs));
        }
        return true;
    case 0x0D: // BREAK
        return fail(pc, instruction, "BREAK instruction", error);
    case 0x0F: // SYNC
        return true;
    case 0x10: // MFHI
        write_gpr64(rd, state_.hi);
        return true;
    case 0x11: // MTHI
        state_.hi = gpr_u64(rs);
        return true;
    case 0x12: // MFLO
        write_gpr64(rd, state_.lo);
        return true;
    case 0x13: // MTLO
        state_.lo = gpr_u64(rs);
        return true;
    case 0x14: // DSLLV
        write_gpr64(rd, gpr_u64(rt) << (gpr_u64(rs) & 63u));
        return true;
    case 0x16: // DSRLV
        write_gpr64(rd, gpr_u64(rt) >> (gpr_u64(rs) & 63u));
        return true;
    case 0x17: // DSRAV
        write_gpr64(
            rd,
            static_cast<u64>(
                static_cast<s64>(gpr_u64(rt)) >> (gpr_u64(rs) & 63u)));
        return true;
    case 0x18: // MULT
        multiply_signed32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo,
            state_.hi);
        write_gpr64(rd, state_.lo);
        return true;
    case 0x19: // MULTU
        multiply_unsigned32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo,
            state_.hi);
        write_gpr64(rd, state_.lo);
        return true;
    case 0x1A: // DIV
        divide_signed32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo,
            state_.hi);
        return true;
    case 0x1B: // DIVU
        divide_unsigned32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo,
            state_.hi);
        return true;
    case 0x21: // ADDU
        write_gpr_word(
            rd,
            static_cast<u32>(gpr_u64(rs)) +
                static_cast<u32>(gpr_u64(rt)));
        return true;
    case 0x23: // SUBU
        write_gpr_word(
            rd,
            static_cast<u32>(gpr_u64(rs)) -
                static_cast<u32>(gpr_u64(rt)));
        return true;
    case 0x24: // AND
        write_gpr64(rd, gpr_u64(rs) & gpr_u64(rt));
        return true;
    case 0x25: // OR
        write_gpr64(rd, gpr_u64(rs) | gpr_u64(rt));
        return true;
    case 0x26: // XOR
        write_gpr64(rd, gpr_u64(rs) ^ gpr_u64(rt));
        return true;
    case 0x27: // NOR
        write_gpr64(rd, ~(gpr_u64(rs) | gpr_u64(rt)));
        return true;
    case 0x2A: // SLT
        write_gpr64(rd, gpr_s64(rs) < gpr_s64(rt) ? 1u : 0u);
        return true;
    case 0x2B: // SLTU
        write_gpr64(rd, gpr_u64(rs) < gpr_u64(rt) ? 1u : 0u);
        return true;
    case 0x2D: // DADDU
        write_gpr64(rd, gpr_u64(rs) + gpr_u64(rt));
        return true;
    case 0x2F: // DSUBU
        write_gpr64(rd, gpr_u64(rs) - gpr_u64(rt));
        return true;
    case 0x38: // DSLL
        write_gpr64(rd, gpr_u64(rt) << sa);
        return true;
    case 0x3A: // DSRL
        write_gpr64(rd, gpr_u64(rt) >> sa);
        return true;
    case 0x3B: // DSRA
        write_gpr64(rd, static_cast<u64>(static_cast<s64>(gpr_u64(rt)) >> sa));
        return true;
    case 0x3C: // DSLL32
        write_gpr64(rd, gpr_u64(rt) << (sa + 32u));
        return true;
    case 0x3E: // DSRL32
        write_gpr64(rd, gpr_u64(rt) >> (sa + 32u));
        return true;
    case 0x3F: // DSRA32
        write_gpr64(
            rd,
            static_cast<u64>(
                static_cast<s64>(gpr_u64(rt)) >> (sa + 32u)));
        return true;
    default:
        return fail(
            pc,
            instruction,
            "Unsupported SPECIAL function " + hex32(funct),
            error);
    }
}

bool EeCpu::execute_regimm(
    u32 pc,
    u32 instruction,
    std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;

    bool taken = false;
    bool likely = false;
    bool link = false;

    switch (rt) {
    case 0x00: taken = gpr_s64(rs) < 0; break;       // BLTZ
    case 0x01: taken = gpr_s64(rs) >= 0; break;      // BGEZ
    case 0x02: taken = gpr_s64(rs) < 0; likely = true; break;  // BLTZL
    case 0x03: taken = gpr_s64(rs) >= 0; likely = true; break; // BGEZL
    case 0x10: taken = gpr_s64(rs) < 0; link = true; break;    // BLTZAL
    case 0x11: taken = gpr_s64(rs) >= 0; link = true; break;   // BGEZAL
    case 0x12:
        taken = gpr_s64(rs) < 0;
        link = true;
        likely = true;
        break;
    case 0x13:
        taken = gpr_s64(rs) >= 0;
        link = true;
        likely = true;
        break;
    default:
        return fail(
            pc,
            instruction,
            "Unsupported REGIMM variant " + hex32(rt),
            error);
    }

    if (link) {
        write_gpr_word(31, pc + 8u);
    }

    if (taken) {
        state_.next_pc = branch_target(pc, immediate(instruction));
    } else if (likely) {
        branch_likely_not_taken(pc);
    }

    return true;
}

bool EeCpu::execute_cop0(
    u32 pc,
    u32 instruction,
    std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;
    const u32 sel = instruction & 7u;
    const u32 funct = instruction & 63u;

    if (rs == 0x00) { // MFC0
        if (sel != 0) {
            return fail(pc, instruction, "Unsupported COP0 select", error);
        }
        write_gpr_word(rt, state_.cop0[rd]);
        return true;
    }

    if (rs == 0x04) { // MTC0
        if (sel != 0) {
            return fail(pc, instruction, "Unsupported COP0 select", error);
        }
        if (rd != 15) {
            state_.cop0[rd] = static_cast<u32>(gpr_u64(rt));
        }
        return true;
    }

    if (rs == 0x10) {
        switch (funct) {
        case 0x01: // TLBR
            return true;
        case 0x02: { // TLBWI
            const u32 index = state_.cop0[0] & 0x3Fu;
            if (index < state_.tlb.size()) {
                auto& entry = state_.tlb[index];
                entry.page_mask = state_.cop0[5];
                entry.entry_hi = state_.cop0[10];
                entry.entry_lo0 = state_.cop0[2];
                entry.entry_lo1 = state_.cop0[3];
            }
            return true;
        }
        case 0x06: // TLBWR
        case 0x08: // TLBP
            return true;
        case 0x18: // ERET
            state_.pc = state_.cop0[14];
            state_.next_pc = state_.pc + 4;
            state_.cop0[12] &= ~0x2u;
            return true;
        case 0x38: // EI/DI subset used during startup
            state_.cop0[12] &= ~0x00010000u;
            return true;
        case 0x39:
            state_.cop0[12] |= 0x00010000u;
            return true;
        default:
            break;
        }
    }

    return fail(pc, instruction, "Unsupported COP0 operation", error);
}

bool EeCpu::execute_cop1(
    u32 pc,
    u32 instruction,
    std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 fs = (instruction >> 11) & 31u;

    switch (rs) {
    case 0x00: // MFC1
        write_gpr_word(rt, state_.fpr[fs]);
        return true;
    case 0x02: // CFC1
        write_gpr_word(rt, state_.fcr[fs]);
        return true;
    case 0x04: // MTC1
        state_.fpr[fs] = static_cast<u32>(gpr_u64(rt));
        return true;
    case 0x06: // CTC1
        state_.fcr[fs] = static_cast<u32>(gpr_u64(rt));
        return true;
    default:
        return fail(pc, instruction, "Unsupported COP1 operation", error);
    }
}

bool EeCpu::execute_mmi(
    u32 pc,
    u32 instruction,
    std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;
    const u32 funct = instruction & 63u;

    switch (funct) {
    case 0x10: // MFHI1
        write_gpr64(rd, state_.hi1);
        return true;
    case 0x11: // MTHI1
        state_.hi1 = gpr_u64(rs);
        return true;
    case 0x12: // MFLO1
        write_gpr64(rd, state_.lo1);
        return true;
    case 0x13: // MTLO1
        state_.lo1 = gpr_u64(rs);
        return true;
    case 0x18: // MULT1
        multiply_signed32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo1,
            state_.hi1);
        write_gpr64(rd, state_.lo1);
        return true;
    case 0x19: // MULTU1
        multiply_unsigned32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo1,
            state_.hi1);
        write_gpr64(rd, state_.lo1);
        return true;
    case 0x1A: // DIV1
        divide_signed32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo1,
            state_.hi1);
        return true;
    case 0x1B: // DIVU1
        divide_unsigned32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo1,
            state_.hi1);
        return true;
    default:
        return fail(
            pc,
            instruction,
            "Unsupported MMI function " + hex32(funct),
            error);
    }
}

bool EeCpu::step(std::string& error) {
    error.clear();

    if (halted_) {
        error = halt_reason_;
        return false;
    }

    const u32 pc = state_.pc;
    const u32 old_next_pc = state_.next_pc;

    u32 instruction = 0;
    if (!bus_.read32(pc, instruction)) {
        return fail(
            pc,
            0,
            "Instruction fetch fault from " + hex32(pc),
            error);
    }

    state_.last_pc = pc;
    state_.last_instruction = instruction;
    state_.pc = old_next_pc;
    state_.next_pc = old_next_pc + 4u;

    const u32 opcode = instruction >> 26;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const s16 imm = immediate(instruction);

    const auto effective_address = [&]() {
        return static_cast<u32>(
            gpr_u64(rs) + static_cast<u64>(static_cast<s64>(imm)));
    };

    const auto load_fault = [&](const char* kind, u32 address) {
        std::string reason = std::string(kind) + " fault from " + hex32(address);
        if (EeBus::is_iop_ram_physical(EeBus::to_physical(address))) {
            reason += " (IOP RAM not implemented)";
        }
        return fail(pc, instruction, reason, error);
    };

    bool ok = true;

    switch (opcode) {
    case 0x00:
        ok = execute_special(pc, instruction, error);
        break;
    case 0x01:
        ok = execute_regimm(pc, instruction, error);
        break;
    case 0x02: // J
        state_.next_pc =
            ((pc + 4u) & 0xF0000000u) |
            ((instruction & 0x03FFFFFFu) << 2);
        break;
    case 0x03: // JAL
        write_gpr_word(31, pc + 8u);
        state_.next_pc =
            ((pc + 4u) & 0xF0000000u) |
            ((instruction & 0x03FFFFFFu) << 2);
        break;
    case 0x04: // BEQ
        if (gpr_u64(rs) == gpr_u64(rt)) {
            state_.next_pc = branch_target(pc, imm);
        }
        break;
    case 0x05: // BNE
        if (gpr_u64(rs) != gpr_u64(rt)) {
            state_.next_pc = branch_target(pc, imm);
        }
        break;
    case 0x06: // BLEZ
        if (gpr_s64(rs) <= 0) {
            state_.next_pc = branch_target(pc, imm);
        }
        break;
    case 0x07: // BGTZ
        if (gpr_s64(rs) > 0) {
            state_.next_pc = branch_target(pc, imm);
        }
        break;
    case 0x08: { // ADDI
        const s32 lhs = static_cast<s32>(static_cast<u32>(gpr_u64(rs)));
        const s64 result = static_cast<s64>(lhs) + static_cast<s64>(imm);
        if (result < std::numeric_limits<s32>::min() ||
            result > std::numeric_limits<s32>::max()) {
            ok = fail(pc, instruction, "ADDI overflow", error);
        } else {
            write_gpr_word(rt, static_cast<u32>(static_cast<s32>(result)));
        }
        break;
    }
    case 0x09: // ADDIU
        write_gpr_word(
            rt,
            static_cast<u32>(gpr_u64(rs)) +
                static_cast<u32>(static_cast<s32>(imm)));
        break;
    case 0x0A: // SLTI
        write_gpr64(rt, gpr_s64(rs) < static_cast<s64>(imm) ? 1u : 0u);
        break;
    case 0x0B: // SLTIU
        write_gpr64(
            rt,
            gpr_u64(rs) < static_cast<u64>(static_cast<s64>(imm)) ? 1u : 0u);
        break;
    case 0x0C: // ANDI
        write_gpr64(rt, gpr_u64(rs) & static_cast<u64>(instruction & 0xFFFFu));
        break;
    case 0x0D: // ORI
        write_gpr64(rt, gpr_u64(rs) | static_cast<u64>(instruction & 0xFFFFu));
        break;
    case 0x0E: // XORI
        write_gpr64(rt, gpr_u64(rs) ^ static_cast<u64>(instruction & 0xFFFFu));
        break;
    case 0x0F: // LUI
        write_gpr_word(rt, (instruction & 0xFFFFu) << 16);
        break;
    case 0x10:
        ok = execute_cop0(pc, instruction, error);
        break;
    case 0x11:
        ok = execute_cop1(pc, instruction, error);
        break;
    case 0x14: // BEQL
        if (gpr_u64(rs) == gpr_u64(rt)) {
            state_.next_pc = branch_target(pc, imm);
        } else {
            branch_likely_not_taken(pc);
        }
        break;
    case 0x15: // BNEL
        if (gpr_u64(rs) != gpr_u64(rt)) {
            state_.next_pc = branch_target(pc, imm);
        } else {
            branch_likely_not_taken(pc);
        }
        break;
    case 0x16: // BLEZL
        if (gpr_s64(rs) <= 0) {
            state_.next_pc = branch_target(pc, imm);
        } else {
            branch_likely_not_taken(pc);
        }
        break;
    case 0x17: // BGTZL
        if (gpr_s64(rs) > 0) {
            state_.next_pc = branch_target(pc, imm);
        } else {
            branch_likely_not_taken(pc);
        }
        break;
    case 0x19: // DADDIU
        write_gpr64(rt, gpr_u64(rs) + static_cast<u64>(static_cast<s64>(imm)));
        break;
    case 0x1C:
        ok = execute_mmi(pc, instruction, error);
        break;
    case 0x20: { // LB
        const u32 address = effective_address();
        u8 value = 0;
        if (!bus_.read8(address, value)) {
            ok = load_fault("Load byte", address);
        } else {
            write_gpr64(rt, static_cast<u64>(static_cast<s64>(static_cast<s8>(value))));
        }
        break;
    }
    case 0x21: { // LH
        const u32 address = effective_address();
        u16 value = 0;
        if (!bus_.read16(address, value)) {
            ok = load_fault("Load halfword", address);
        } else {
            write_gpr64(rt, static_cast<u64>(static_cast<s64>(static_cast<s16>(value))));
        }
        break;
    }
    case 0x23: { // LW
        const u32 address = effective_address();
        u32 value = 0;
        if (!bus_.read32(address, value)) {
            ok = load_fault("Load word", address);
        } else {
            write_gpr_word(rt, value);
        }
        break;
    }
    case 0x24: { // LBU
        const u32 address = effective_address();
        u8 value = 0;
        if (!bus_.read8(address, value)) {
            ok = load_fault("Load byte", address);
        } else {
            write_gpr64(rt, value);
        }
        break;
    }
    case 0x25: { // LHU
        const u32 address = effective_address();
        u16 value = 0;
        if (!bus_.read16(address, value)) {
            ok = load_fault("Load halfword", address);
        } else {
            write_gpr64(rt, value);
        }
        break;
    }
    case 0x27: { // LWU
        const u32 address = effective_address();
        u32 value = 0;
        if (!bus_.read32(address, value)) {
            ok = load_fault("Load word", address);
        } else {
            write_gpr64(rt, value);
        }
        break;
    }
    case 0x28: { // SB
        const u32 address = effective_address();
        if (!bus_.write8(address, static_cast<u8>(gpr_u64(rt)))) {
            ok = fail(pc, instruction, "Store byte fault to " + hex32(address), error);
        }
        break;
    }
    case 0x29: { // SH
        const u32 address = effective_address();
        if (!bus_.write16(address, static_cast<u16>(gpr_u64(rt)))) {
            ok = fail(pc, instruction, "Store halfword fault to " + hex32(address), error);
        }
        break;
    }
    case 0x2B: { // SW
        const u32 address = effective_address();
        if (!bus_.write32(address, static_cast<u32>(gpr_u64(rt)))) {
            std::string reason = "Store word fault to " + hex32(address);
            if (EeBus::is_iop_ram_physical(EeBus::to_physical(address))) {
                reason += " (IOP RAM not implemented)";
            }
            ok = fail(pc, instruction, reason, error);
        }
        break;
    }
    case 0x2F: // CACHE
    case 0x33: // PREF
        break;
    case 0x31: { // LWC1
        const u32 address = effective_address();
        u32 value = 0;
        if (!bus_.read32(address, value)) {
            ok = load_fault("LWC1", address);
        } else {
            state_.fpr[rt] = value;
        }
        break;
    }
    case 0x37: { // LD
        const u32 address = effective_address();
        u64 value = 0;
        if (!bus_.read64(address, value)) {
            ok = load_fault("Load doubleword", address);
        } else {
            write_gpr64(rt, value);
        }
        break;
    }
    case 0x39: { // SWC1
        const u32 address = effective_address();
        if (!bus_.write32(address, state_.fpr[rt])) {
            ok = fail(pc, instruction, "SWC1 fault to " + hex32(address), error);
        }
        break;
    }
    case 0x3F: { // SD
        const u32 address = effective_address();
        if (!bus_.write64(address, gpr_u64(rt))) {
            std::string reason = "Store doubleword fault to " + hex32(address);
            if (EeBus::is_iop_ram_physical(EeBus::to_physical(address))) {
                reason += " (IOP RAM not implemented)";
            }
            ok = fail(pc, instruction, reason, error);
        }
        break;
    }
    default:
        ok = fail(
            pc,
            instruction,
            "Unsupported opcode " + hex32(opcode),
            error);
        break;
    }

    if (!ok) {
        state_.pc = pc;
        state_.next_pc = old_next_pc;
        return false;
    }

    state_.gpr[0] = {};
    ++state_.instructions_executed;
    ++state_.cop0[9];
    bus_.tick(1);
    return true;
}

u64 EeCpu::run(u64 instruction_budget, std::string& error) {
    error.clear();
    u64 executed = 0;
    while (executed < instruction_budget && !halted_) {
        if (!step(error)) {
            break;
        }
        ++executed;
    }
    return executed;
}

} // namespace ps2
