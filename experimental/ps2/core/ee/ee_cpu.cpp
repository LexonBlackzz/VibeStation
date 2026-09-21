#include "core/ee/ee_cpu.h"

#include "core/memory/ee_bus.h"

#include <bit>
#include <cmath>
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

float ps2_fpu_input(u32 bits) {
    const u32 exponent = bits & 0x7F800000u;
    if (exponent == 0) bits &= 0x80000000u;
    else if (exponent == 0x7F800000u) bits = (bits & 0x80000000u) | 0x7F7FFFFFu;
    return std::bit_cast<float>(bits);
}

u32 ps2_fpu_result(float value) {
    u32 bits = std::bit_cast<u32>(value);
    const u32 exponent = bits & 0x7F800000u;
    if (exponent == 0) return bits & 0x80000000u;
    if (exponent == 0x7F800000u) return (bits & 0x80000000u) | 0x7F7FFFFFu;
    return bits;
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
    state_.vu_vf[0].lo = 0;
    state_.vu_vf[0].hi = 0x3F80000000000000ull;
    state_.vu_vi[20] = 0x3F800000u;
    halted_ = false;
    next_is_delay_slot_ = false;
    current_is_delay_slot_ = false;
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
    next_is_delay_slot_ = false;
}

void EeCpu::raise_exception(u32 code, u32 pc, bool in_delay_slot) {
    u32& status = state_.cop0[12];
    u32& cause = state_.cop0[13];

    cause = (cause & ~0x8000007Cu) | ((code << 2) & 0x7Cu);

    u32 offset = (code == 0) ? 0x200u : 0x180u;
    if ((status & 0x2u) == 0) {
        status |= 0x2u;
        if (in_delay_slot) {
            state_.cop0[14] = pc - 4u;
            cause |= 0x80000000u;
        } else {
            state_.cop0[14] = pc;
            cause &= ~0x80000000u;
        }
    } else {
        offset = 0x180u;
    }

    const u32 base = (status & 0x00400000u) != 0 ? 0xBFC00200u : 0x80000000u;
    state_.pc = base + offset;
    state_.next_pc = state_.pc + 4u;
    next_is_delay_slot_ = false;
    current_is_delay_slot_ = false;
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
        next_is_delay_slot_ = true;
        return true;
    case 0x09: // JALR
        write_gpr_word(rd, pc + 8u);
        state_.next_pc = static_cast<u32>(gpr_u64(rs));
        next_is_delay_slot_ = true;
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
    case 0x0C: // SYSCALL
        raise_exception(8u, pc, current_is_delay_slot_);
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
        write_gpr_word(rd, static_cast<u32>(gpr_u64(rs)) + static_cast<u32>(gpr_u64(rt)));
        return true;
    case 0x23: // SUBU
        write_gpr_word(rd, static_cast<u32>(gpr_u64(rs)) - static_cast<u32>(gpr_u64(rt)));
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
        write_gpr64(rd, static_cast<u64>(static_cast<s64>(gpr_u64(rt)) >> (sa + 32u)));
        return true;
    default:
        return fail(pc, instruction, "Unsupported SPECIAL function " + hex32(funct), error);
    }
}

bool EeCpu::execute_regimm(u32 pc, u32 instruction, std::string& error) {
    const u32 rs=(instruction>>21)&31u; const u32 rt=(instruction>>16)&31u;
    bool taken=false, likely=false, link=false;
    switch(rt){
    case 0x00: taken=gpr_s64(rs)<0; break;
    case 0x01: taken=gpr_s64(rs)>=0; break;
    case 0x02: taken=gpr_s64(rs)<0; likely=true; break;
    case 0x03: taken=gpr_s64(rs)>=0; likely=true; break;
    case 0x10: taken=gpr_s64(rs)<0; link=true; break;
    case 0x11: taken=gpr_s64(rs)>=0; link=true; break;
    case 0x12: taken=gpr_s64(rs)<0; link=true; likely=true; break;
    case 0x13: taken=gpr_s64(rs)>=0; link=true; likely=true; break;
    default: return fail(pc,instruction,"Unsupported REGIMM variant "+hex32(rt),error);
    }
    if(link) write_gpr_word(31,pc+8u);
    if(taken) {
        state_.next_pc=branch_target(pc,immediate(instruction));
        next_is_delay_slot_=true;
    } else if(likely) {
        branch_likely_not_taken(pc);
    } else {
        next_is_delay_slot_=true;
    }
    return true;
}

bool EeCpu::execute_cop0(u32 pc,u32 instruction,std::string& error){
    const u32 rs=(instruction>>21)&31u, rt=(instruction>>16)&31u, rd=(instruction>>11)&31u, sel=instruction&7u, funct=instruction&63u;
    if(rs==0x00){ if(sel!=0) return fail(pc,instruction,"Unsupported COP0 select",error); write_gpr_word(rt,state_.cop0[rd]); return true; }
    if(rs==0x04){
        if(sel!=0) return fail(pc,instruction,"Unsupported COP0 select",error);
        if(rd!=15) {
            state_.cop0[rd]=static_cast<u32>(gpr_u64(rt));
            // MIPS Count/Compare timer: writing Compare acknowledges IP7.
            if(rd==11) state_.cop0[13]&=~0x00008000u;
        }
        return true;
    }
    if(rs==0x10){ switch(funct){
        case 0x01: return true;
        case 0x02:{ const u32 index=state_.cop0[0]&0x3Fu; if(index<state_.tlb.size()){auto& e=state_.tlb[index];e.page_mask=state_.cop0[5];e.entry_hi=state_.cop0[10];e.entry_lo0=state_.cop0[2];e.entry_lo1=state_.cop0[3];} return true;}
        case 0x06: case 0x08: return true;
        case 0x18:
            if ((state_.cop0[12] & 0x4u) != 0) {
                state_.pc=state_.cop0[30];
                state_.cop0[12]&=~0x4u;
            } else {
                state_.pc=state_.cop0[14];
                state_.cop0[12]&=~0x2u;
            }
            state_.next_pc=state_.pc+4;
            next_is_delay_slot_=false;
            current_is_delay_slot_=false;
            return true;
        case 0x38: {
            const u32 status=state_.cop0[12];
            if ((status & 0x00020000u) != 0 || (status & 0x6u) != 0 || (status & 0x18u) == 0)
                state_.cop0[12]&=~0x00010000u;
            return true;
        }
        case 0x39: {
            const u32 status=state_.cop0[12];
            if ((status & 0x00020000u) != 0 || (status & 0x6u) != 0 || (status & 0x18u) == 0)
                state_.cop0[12]|=0x00010000u;
            return true;
        }
        default: break;
    }}
    return fail(pc,instruction,"Unsupported COP0 operation",error);
}

bool EeCpu::execute_cop1(u32 pc, u32 instruction, std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 fs = (instruction >> 11) & 31u;
    const u32 fd = (instruction >> 6) & 31u;
    const u32 funct = instruction & 63u;
    constexpr u32 kCond = 0x00800000u;

    switch (rs) {
    case 0x00: // MFC1
        write_gpr_word(rt, state_.fpr[fs]);
        return true;
    case 0x02: // CFC1
        if (fs == 0) write_gpr_word(rt, 0x00002E00u);
        else if (fs == 31) write_gpr_word(rt, state_.fcr[31]);
        else write_gpr_word(rt, 0);
        return true;
    case 0x04: // MTC1
        state_.fpr[fs] = static_cast<u32>(gpr_u64(rt));
        return true;
    case 0x06: // CTC1
        if (fs == 31) state_.fcr[31] = static_cast<u32>(gpr_u64(rt));
        return true;
    case 0x08: { // BC1
        const bool cond = (state_.fcr[31] & kCond) != 0;
        const u32 variant = rt & 3u;
        const bool taken = (variant & 1u) != 0 ? cond : !cond;
        const bool likely = (variant & 2u) != 0;
        if (taken) {
            state_.next_pc = branch_target(pc, immediate(instruction));
            next_is_delay_slot_ = true;
        } else if (likely) {
            branch_likely_not_taken(pc);
        } else {
            next_is_delay_slot_ = true;
        }
        return true;
    }
    default:
        break;
    }

    if (rs == 0x14u) { // COP1.W
        if (funct == 0x20u) { // CVT.S.W
            const s32 value = static_cast<s32>(state_.fpr[fs]);
            state_.fpr[fd] = ps2_fpu_result(static_cast<float>(value));
            return true;
        }
        return fail(pc, instruction, "Unsupported COP1.W operation", error);
    }

    if (rs != 0x10u) return fail(pc, instruction, "Unsupported COP1 operation", error);

    const u32 ft = rt;
    const float a = ps2_fpu_input(state_.fpr[fs]);
    const float b = ps2_fpu_input(state_.fpr[ft]);
    const float acc = ps2_fpu_input(state_.fpu_acc);
    auto set_fd = [&](float v) { state_.fpr[fd] = ps2_fpu_result(v); };
    auto set_acc = [&](float v) { state_.fpu_acc = ps2_fpu_result(v); };
    auto set_cond = [&](bool v) {
        if (v) state_.fcr[31] |= kCond;
        else state_.fcr[31] &= ~kCond;
    };

    switch (funct) {
    case 0x00: set_fd(a + b); return true; // ADD.S
    case 0x01: set_fd(a - b); return true; // SUB.S
    case 0x02: set_fd(a * b); return true; // MUL.S
    case 0x03: // DIV.S
        if ((state_.fpr[ft] & 0x7FFFFFFFu) == 0) {
            const u32 sign = (state_.fpr[fs] ^ state_.fpr[ft]) & 0x80000000u;
            state_.fpr[fd] = sign | 0x7F7FFFFFu;
        } else set_fd(a / b);
        return true;
    case 0x04: // SQRT.S
        set_fd(std::sqrt(std::fabs(b)));
        return true;
    case 0x05: state_.fpr[fd] = state_.fpr[fs] & 0x7FFFFFFFu; return true; // ABS.S
    case 0x06: state_.fpr[fd] = state_.fpr[fs]; return true; // MOV.S
    case 0x07: state_.fpr[fd] = state_.fpr[fs] ^ 0x80000000u; return true; // NEG.S
    case 0x16: { // RSQRT.S
        if ((state_.fpr[ft] & 0x7FFFFFFFu) == 0) {
            const u32 sign = (state_.fpr[fs] ^ state_.fpr[ft]) & 0x80000000u;
            state_.fpr[fd] = sign | 0x7F7FFFFFu;
        } else set_fd(a / std::sqrt(std::fabs(b)));
        return true;
    }
    case 0x18: set_acc(a + b); return true; // ADDA.S
    case 0x19: set_acc(a - b); return true; // SUBA.S
    case 0x1A: set_acc(a * b); return true; // MULA.S
    case 0x1C: set_fd(acc + (a * b)); return true; // MADD.S
    case 0x1D: set_fd(acc - (a * b)); return true; // MSUB.S
    case 0x1E: set_acc(acc + (a * b)); return true; // MADDA.S
    case 0x1F: set_acc(acc - (a * b)); return true; // MSUBA.S
    case 0x24: { // CVT.W.S (round toward zero)
        if ((state_.fpr[fs] & 0x7F800000u) <= 0x4E800000u) {
            const double d = static_cast<double>(a);
            if (d > 2147483647.0) state_.fpr[fd] = 0x7FFFFFFFu;
            else if (d < -2147483648.0) state_.fpr[fd] = 0x80000000u;
            else state_.fpr[fd] = static_cast<u32>(static_cast<s32>(d));
        } else state_.fpr[fd] = (state_.fpr[fs] & 0x80000000u) ? 0x80000000u : 0x7FFFFFFFu;
        return true;
    }
    case 0x28: state_.fpr[fd] = (a >= b) ? state_.fpr[fs] : state_.fpr[ft]; return true; // MAX.S
    case 0x29: state_.fpr[fd] = (a <= b) ? state_.fpr[fs] : state_.fpr[ft]; return true; // MIN.S
    case 0x30: set_cond(false); return true; // C.F.S
    case 0x32: set_cond(a == b); return true; // C.EQ.S
    case 0x34: set_cond(a < b); return true; // C.LT.S
    case 0x36: set_cond(a <= b); return true; // C.LE.S
    default:
        return fail(pc, instruction, "Unsupported COP1.S function " + hex32(funct), error);
    }
}

bool EeCpu::execute_cop2(u32 pc, u32 instruction, std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 fs = (instruction >> 11) & 31u;

    auto lane_read = [&](u32 reg, u32 lane) -> u32 {
        const EeGpr& v = state_.vu_vf[reg & 31u];
        const u64 half = lane < 2 ? v.lo : v.hi;
        return static_cast<u32>(half >> ((lane & 1u) * 32u));
    };
    auto lane_write = [&](u32 reg, u32 lane, u32 value) {
        if ((reg & 31u) == 0) return;
        EeGpr& v = state_.vu_vf[reg & 31u];
        u64& half = lane < 2 ? v.lo : v.hi;
        const u32 shift = (lane & 1u) * 32u;
        half = (half & ~(0xFFFFFFFFull << shift)) | (static_cast<u64>(value) << shift);
    };
    auto selected = [&](u32 lane) {
        static constexpr u32 bits[4] = {24u, 23u, 22u, 21u};
        return ((instruction >> bits[lane]) & 1u) != 0;
    };

    switch (rs) {
    case 0x01: // QMFC2
        if (rt != 0) state_.gpr[rt] = state_.vu_vf[fs];
        return true;
    case 0x02: { // CFC2
        if (rt == 0) return true;
        u32 value = state_.vu_vi[fs];
        if (fs == 20u) value &= 0x007FFFFFu;
        write_gpr_word(rt, value);
        return true;
    }
    case 0x05: // QMTC2
        if (fs != 0) state_.vu_vf[fs] = state_.gpr[rt];
        return true;
    case 0x06: { // CTC2
        if (fs == 0u || fs == 17u || fs == 26u || fs == 29u) return true;
        const u32 value = static_cast<u32>(gpr_u64(rt));
        if (fs == 20u) {
            state_.vu_vi[20] = (value & 0x007FFFFFu) | 0x3F800000u;
            return true;
        }
        if (fs == 28u) {
            state_.vu_vi[28] = value & 0x00000C0Cu;
            if ((value & 0x2u) != 0) {
                for (u32 i = 1; i < 32; ++i) state_.vu_vf[i] = {};
                for (u32 i = 1; i < 16; ++i) state_.vu_vi[i] = 0;
                state_.vu_vi[29] &= ~0xFFu;
            }
            if ((value & 0x200u) != 0) state_.vu_vi[29] &= ~0xFF00u;
            return true;
        }
        state_.vu_vi[fs] = value;
        return true;
    }
    default:
        break;
    }

    if (rs < 0x10u) return fail(pc, instruction, "Unsupported COP2 operation", error);

    const u32 ft = (instruction >> 16) & 31u;
    const u32 fd = (instruction >> 6) & 31u;
    const u32 funct = instruction & 63u;

    if (funct == 0x2Cu) { // VSUB
        for (u32 lane = 0; lane < 4; ++lane) {
            if (!selected(lane)) continue;
            const float a = std::bit_cast<float>(lane_read(fs, lane));
            const float b = std::bit_cast<float>(lane_read(ft, lane));
            lane_write(fd, lane, std::bit_cast<u32>(a - b));
        }
        return true;
    }
    if (funct == 0x30u) { // VIADD
        const u32 it = ft & 0xFu;
        const u32 is = fs & 0xFu;
        const u32 id = fd & 0xFu;
        if (id != 0) {
            const s16 result = static_cast<s16>(state_.vu_vi[is]) + static_cast<s16>(state_.vu_vi[it]);
            state_.vu_vi[id] = static_cast<u16>(result);
        }
        return true;
    }
    if (funct >= 0x3Cu) {
        const u32 sub = (instruction & 3u) | ((instruction >> 4) & 0x7Cu);
        const u32 it = ft & 0xFu;
        const u32 is = fs & 0xFu;
        if (sub == 0x3Fu) { // VISWR
            const u32 base = 0x11004000u + ((state_.vu_vi[is] & 0xFFFFu) * 16u & 0xFFFu);
            for (u32 lane = 0; lane < 4; ++lane) {
                if (selected(lane) && !bus_.write32(base + lane * 4u, state_.vu_vi[it] & 0xFFFFu))
                    return fail(pc, instruction, "VISWR VU0 memory fault", error);
            }
            return true;
        }
        if (sub == 0x35u) { // VSQI
            const u32 base = 0x11004000u + ((state_.vu_vi[it] & 0xFFFFu) * 16u & 0xFFFu);
            for (u32 lane = 0; lane < 4; ++lane) {
                if (selected(lane) && !bus_.write32(base + lane * 4u, lane_read(fs, lane)))
                    return fail(pc, instruction, "VSQI VU0 memory fault", error);
            }
            if (ft != 0) state_.vu_vi[it] = static_cast<u16>(state_.vu_vi[it] + 1u);
            return true;
        }
        return fail(pc, instruction, "Unsupported COP2 SPECIAL2 function " + hex32(sub), error);
    }

    return fail(pc, instruction, "Unsupported COP2 macro function " + hex32(funct), error);
}

bool EeCpu::execute_mmi(u32 pc,u32 instruction,std::string& error){
    const u32 rs=(instruction>>21)&31u, rt=(instruction>>16)&31u, rd=(instruction>>11)&31u, funct=instruction&63u;
    switch(funct){
    case 0x10: write_gpr64(rd,state_.hi1); return true;
    case 0x11: state_.hi1=gpr_u64(rs); return true;
    case 0x12: write_gpr64(rd,state_.lo1); return true;
    case 0x13: state_.lo1=gpr_u64(rs); return true;
    case 0x18: multiply_signed32(static_cast<u32>(gpr_u64(rs)),static_cast<u32>(gpr_u64(rt)),state_.lo1,state_.hi1); write_gpr64(rd,state_.lo1); return true;
    case 0x19: multiply_unsigned32(static_cast<u32>(gpr_u64(rs)),static_cast<u32>(gpr_u64(rt)),state_.lo1,state_.hi1); write_gpr64(rd,state_.lo1); return true;
    case 0x1A: divide_signed32(static_cast<u32>(gpr_u64(rs)),static_cast<u32>(gpr_u64(rt)),state_.lo1,state_.hi1); return true;
    case 0x1B: divide_unsigned32(static_cast<u32>(gpr_u64(rs)),static_cast<u32>(gpr_u64(rt)),state_.lo1,state_.hi1); return true;
    case 0x28: { // MMI1
        const u32 sub = (instruction >> 6) & 31u;
        if (sub == 0x10u) { // PADDUW
            if (rd != 0) {
                for (u32 lane = 0; lane < 4; ++lane) {
                    const u64 a_half = lane < 2 ? state_.gpr[rs].lo : state_.gpr[rs].hi;
                    const u64 b_half = lane < 2 ? state_.gpr[rt].lo : state_.gpr[rt].hi;
                    const u32 shift = (lane & 1u) * 32u;
                    const u64 sum = static_cast<u64>(static_cast<u32>(a_half >> shift)) +
                                    static_cast<u64>(static_cast<u32>(b_half >> shift));
                    const u32 value = sum > 0xFFFFFFFFull ? 0xFFFFFFFFu : static_cast<u32>(sum);
                    u64& out_half = lane < 2 ? state_.gpr[rd].lo : state_.gpr[rd].hi;
                    out_half = (out_half & ~(0xFFFFFFFFull << shift)) | (static_cast<u64>(value) << shift);
                }
            }
            return true;
        }
        return fail(pc,instruction,"Unsupported MMI1 function "+hex32(sub),error);
    }
    case 0x29: { // MMI3
        const u32 sub = (instruction >> 6) & 31u;
        if (sub == 0x12u) { // POR
            if (rd != 0) {
                state_.gpr[rd].lo = state_.gpr[rs].lo | state_.gpr[rt].lo;
                state_.gpr[rd].hi = state_.gpr[rs].hi | state_.gpr[rt].hi;
            }
            return true;
        }
        return fail(pc,instruction,"Unsupported MMI3 function "+hex32(sub),error);
    }
    default: return fail(pc,instruction,"Unsupported MMI function "+hex32(funct),error);
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
    current_is_delay_slot_ = next_is_delay_slot_;
    next_is_delay_slot_ = false;

    if (bus_.intc_pending()) state_.cop0[13] |= 0x00000400u;
    else state_.cop0[13] &= ~0x00000400u;
    if (bus_.dmac_pending()) state_.cop0[13] |= 0x00000800u;
    else state_.cop0[13] &= ~0x00000800u;

    const u32 status = state_.cop0[12];
    if ((state_.cop0[13] & status & 0x0000FF00u) != 0 &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0) {
        raise_exception(0u, pc, current_is_delay_slot_);
        ++state_.instructions_executed;
        ++state_.cop0[9];
        if (state_.cop0[9] == state_.cop0[11]) {
            state_.cop0[13] |= 0x00008000u;
        }
        bus_.tick(1);
        return true;
    }

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
        const std::string reason =
            std::string(kind) + " fault from " + hex32(address);
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
        next_is_delay_slot_ = true;
        break;
    case 0x03: // JAL
        write_gpr_word(31, pc + 8u);
        state_.next_pc =
            ((pc + 4u) & 0xF0000000u) |
            ((instruction & 0x03FFFFFFu) << 2);
        next_is_delay_slot_ = true;
        break;
    case 0x04: // BEQ
        if (gpr_u64(rs) == gpr_u64(rt)) {
            state_.next_pc = branch_target(pc, imm);
        }
        next_is_delay_slot_ = true;
        break;
    case 0x05: // BND
        if (gpr_u64(rs) != gpr_u64(rt)) {
            state_.next_pc = branch_target(pc, imm);
        }
        next_is_delay_slot_ = true;
        break;
    case 0x06: // BLEZ
        if (gpr_s64(rs) <= 0) {
            state_.next_pc = branch_target(pc, imm);
        }
        next_is_delay_slot_ = true;
        break;
    case 0x07: // BGTZ
        if (gpr_s64(rs) > 0) {
            state_.next_pc = branch_target(pc, imm);
        }
        next_is_delay_slot_ = true;
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
    case 0x12:
        ok = execute_cop2(pc, instruction, error);
        break;
    case 0x14: // BEQL
        if (gpr_u64(rs) == gpr_u64(rt)) {
            state_.next_pc = branch_target(pc, imm);
            next_is_delay_slot_ = true;
        } else {
            branch_likely_not_taken(pc);
        }
        break;
    case 0x15: // BNEL
        if (gpr_u64(rs) != gpr_u64(rt)) {
            state_.next_pc = branch_target(pc, imm);
            next_is_delay_slot_ = true;
        } else {
            branch_likely_not_taken(pc);
        }
        break;
    case 0x16: // BLEZL
        if (gpr_s64(rs) <= 0) {
            state_.next_pc = branch_target(pc, imm);
            next_is_delay_slot_ = true;
        } else {
            branch_likely_not_taken(pc);
        }
        break;
    case 0x17: // BGTZL
        if (gpr_s64(rs) > 0) {
            state_.next_pc = branch_target(pc, imm);
            next_is_delay_slot_ = true;
        } else {
            branch_likely_not_taken(pc);
        }
        break;
    case 0x19: // DADDIU
        write_gpr64(rt, gpr_u64(rs) + static_cast<u64>(static_cast<s64>(imm)));
        break;
    case 0x1A: { // LDL
        static constexpr u64 masks[8] = {
            0x00FFFFFFFFFFFFFFull, 0x0000FFFFFFFFFFFFull, 0x000000FFFFFFFFFFull, 0x00000000FFFFFFFFull,
            0x0000000000FFFFFFull, 0x000000000000FFFFull, 0x00000000000000FFull, 0x0000000000000000ull};
        static constexpr u8 shifts[8] = {56,48,40,32,24,16,8,0};
        const u32 address = effective_address();
        const u32 shift = address & 7u;
        u64 mem = 0;
        if (!bus_.read64(address & ~7u, mem)) ok = load_fault("LDL", address);
        else if (rt != 0) state_.gpr[rt].lo = (state_.gpr[rt].lo & masks[shift]) | (mem << shifts[shift]);
        break;
    }
    case 0x1B: { // LDR
        static constexpr u64 masks[8] = {
            0x0000000000000000ull, 0xFF00000000000000ull, 0xFFFF000000000000ull, 0xFFFFFF0000000000ull,
            0xFFFFFFFF00000000ull, 0xFFFFFFFFFF000000ull, 0xFFFFFFFFFFFF0000ull, 0xFFFFFFFFFFFFFF00ull};
        static constexpr u8 shifts[8] = {0,8,16,24,32,40,48,56};
        const u32 address = effective_address();
        const u32 shift = address & 7u;
        u64 mem = 0;
        if (!bus_.read64(address & ~7u, mem)) ok = load_fault("LDR", address);
        else if (rt != 0) state_.gpr[rt].lo = (state_.gpr[rt].lo & masks[shift]) | (mem >> shifts[shift]);
        break;
    }
    case 0x1C:
        ok = execute_mmi(pc, instruction, error);
        break;
    case 0x1E: { // LQ
        const u32 address = effective_address() & ~0x0Fu;
        u64 lo = 0;
        u64 hi = 0;
        if (!bus_.read64(address, lo) ||
            !bus_.read64(address + 8u, hi)) {
            ok = load_fault("Load quadword", address);
        } else if (rt != 0) {
            state_.gpr[rt].lo = lo;
            state_.gpr[rt].hi = hi;
        }
        break;
    }
    case 0x1F: { // SQ
        const u32 address = effective_address() & ~0x0Fu;
        if (!bus_.write64(address, state_.gpr[rt].lo) ||
            !bus_.write64(address + 8u, state_.gpr[rt].hi)) {
            ok = fail(
                pc,
                instruction,
                "Store quadword fault to " + hex32(address),
                error);
        }
        break;
    }
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
            ok = fail(pc, instruction, "Store word fault to " + hex32(address), error);
        }
        break;
    }
    case 0x2C: { // SDL
        static constexpr u64 masks[8] = {
            0xFFFFFFFFFFFFFF00ull,0xFFFFFFFFFFFF0000ull,0xFFFFFFFFFF000000ull,0xFFFFFFFF00000000ull,
            0xFFFFFF0000000000ull,0xFFFF000000000000ull,0xFF00000000000000ull,0x0000000000000000ull};
        static constexpr u8 shifts[8] = {56,48,40,32,24,16,8,0};
        const u32 address = effective_address(); const u32 shift = address & 7u;
        u64 mem = 0;
        if (!bus_.read64(address & ~7u, mem) || !bus_.write64(address & ~7u, (gpr_u64(rt) >> shifts[shift]) | (mem & masks[shift])))
            ok = fail(pc,instruction,"SDL fault at "+hex32(address),error);
        break;
    }
    case 0x2D: { // SDR
        static constexpr u64 masks[8] = {
            0x0000000000000000ull,0x00000000000000FFull,0x000000000000FFFFull,0x0000000000FFFFFFull,
            0x00000000FFFFFFFFull,0x000000FFFFFFFFFFull,0x0000FFFFFFFFFFFFull,0x00FFFFFFFFFFFFFFull};
        static constexpr u8 shifts[8] = {0,8,16,24,32,40,48,56};
        const u32 address = effective_address(); const u32 shift = address & 7u;
        u64 mem = 0;
        if (!bus_.read64(address & ~7u, mem) || !bus_.write64(address & ~7u, (gpr_u64(rt) << shifts[shift]) | (mem & masks[shift])))
            ok = fail(pc,instruction,"SDR fault at "+hex32(address),error);
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
            ok = fail(
                pc,
                instruction,
                "Store doubleword fault to " + hex32(address),
                error);
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
    if (state_.cop0[9] == state_.cop0[11]) {
        state_.cop0[13] |= 0x00008000u;
    }
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
