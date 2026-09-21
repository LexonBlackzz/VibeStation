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
    memory_exception_pending_ = false;
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

void EeCpu::raise_exception(
    u32 code,
    u32 pc,
    bool in_delay_slot,
    bool tlb_refill) {
    u32& status = state_.cop0[12];
    u32& cause = state_.cop0[13];

    cause = (cause & ~0x8000007Cu) | ((code << 2) & 0x7Cu);

    u32 offset =
        code == 0 ? 0x200u :
        tlb_refill ? 0x000u :
        0x180u;
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

    const u32 base =
        (status & 0x00400000u) != 0
            ? 0xBFC00200u
            : 0x80000000u;
    state_.pc = base + offset;
    state_.next_pc = state_.pc + 4u;
    next_is_delay_slot_ = false;
    current_is_delay_slot_ = false;
}

bool EeCpu::translate_address(
    u32 virtual_address,
    bool store,
    u32 fault_pc,
    bool in_delay_slot,
    u32& translated) {
    // Keep the existing bootstrap-friendly direct mappings for kuseg and
    // KSEG0/KSEG1.  Kernel mapped segments use the R5900's software-managed
    // TLB and are where the retail BIOS begins relying on wired entries.
    if (virtual_address < 0xC0000000u) {
        translated = virtual_address;
        return true;
    }

    const u32 asid = state_.cop0[10] & 0xFFu;
    for (const auto& entry : state_.tlb) {
        const u32 pair_mask =
            (entry.page_mask & 0x01FFE000u) | 0x1FFFu;
        const u32 vpn_mask = ~pair_mask;
        if ((entry.entry_hi & vpn_mask) !=
            (virtual_address & vpn_mask)) {
            continue;
        }

        const bool global =
            (entry.entry_lo0 & 1u) != 0 &&
            (entry.entry_lo1 & 1u) != 0;
        if (!global && (entry.entry_hi & 0xFFu) != asid) {
            continue;
        }

        const u32 page_size = (pair_mask + 1u) >> 1;
        const bool odd_page =
            (virtual_address & page_size) != 0;
        const u32 entry_lo =
            odd_page ? entry.entry_lo1 : entry.entry_lo0;

        state_.cop0[8] = virtual_address;
        state_.cop0[4] =
            (state_.cop0[4] & 0xFF800000u) |
            ((virtual_address >> 9) & 0x007FFFF0u);
        state_.cop0[10] =
            (virtual_address & vpn_mask) | asid;

        if ((entry_lo & 0x2u) == 0) {
            memory_exception_pending_ = true;
            raise_exception(
                store ? 3u : 2u,
                fault_pc,
                in_delay_slot);
            return false;
        }
        if (store && (entry_lo & 0x4u) == 0) {
            memory_exception_pending_ = true;
            raise_exception(1u, fault_pc, in_delay_slot);
            return false;
        }

        const u32 pfn = (entry_lo >> 6) & 0x000FFFFFu;
        const u32 page_offset_mask = page_size - 1u;
        const u32 physical_base =
            (pfn << 12) & ~page_offset_mask;
        translated =
            physical_base |
            (virtual_address & page_offset_mask);
        return true;
    }

    state_.cop0[8] = virtual_address;
    state_.cop0[4] =
        (state_.cop0[4] & 0xFF800000u) |
        ((virtual_address >> 9) & 0x007FFFF0u);
    state_.cop0[10] =
        (virtual_address & 0xFFFFE000u) | asid;
    memory_exception_pending_ = true;
    raise_exception(
        store ? 3u : 2u,
        fault_pc,
        in_delay_slot,
        true);
    return false;
}

bool EeCpu::fail(
    u32 pc,
    u32 instruction,
    const std::string& reason,
    std::string& error) {
    if (memory_exception_pending_) {
        memory_exception_pending_ = false;
        error.clear();
        return true;
    }

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
        raise_exception(9u, pc, current_is_delay_slot_);
        return true;
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
    case 0x20: { // ADD
        const s64 result =
            static_cast<s64>(static_cast<s32>(static_cast<u32>(gpr_u64(rs)))) +
            static_cast<s64>(static_cast<s32>(static_cast<u32>(gpr_u64(rt))));
        if (result < std::numeric_limits<s32>::min() ||
            result > std::numeric_limits<s32>::max()) {
            raise_exception(12u, pc, current_is_delay_slot_);
        } else {
            write_gpr_word(rd, static_cast<u32>(static_cast<s32>(result)));
        }
        return true;
    }
    case 0x21: // ADDU
        write_gpr_word(rd, static_cast<u32>(gpr_u64(rs)) + static_cast<u32>(gpr_u64(rt)));
        return true;
    case 0x22: { // SUB
        const s64 result =
            static_cast<s64>(static_cast<s32>(static_cast<u32>(gpr_u64(rs)))) -
            static_cast<s64>(static_cast<s32>(static_cast<u32>(gpr_u64(rt))));
        if (result < std::numeric_limits<s32>::min() ||
            result > std::numeric_limits<s32>::max()) {
            raise_exception(12u, pc, current_is_delay_slot_);
        } else {
            write_gpr_word(rd, static_cast<u32>(static_cast<s32>(result)));
        }
        return true;
    }
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
    case 0x28: // MFSA
        write_gpr64(rd, state_.sa);
        return true;
    case 0x29: // MTSA
        state_.sa = static_cast<u32>(gpr_u64(rs));
        return true;
    case 0x2A: // SLT
        write_gpr64(rd, gpr_s64(rs) < gpr_s64(rt) ? 1u : 0u);
        return true;
    case 0x2B: // SLTU
        write_gpr64(rd, gpr_u64(rs) < gpr_u64(rt) ? 1u : 0u);
        return true;
    case 0x2C: { // DADD
        const s64 lhs = static_cast<s64>(gpr_u64(rs));
        const s64 rhs = static_cast<s64>(gpr_u64(rt));
        const s64 result = static_cast<s64>(
            static_cast<u64>(lhs) + static_cast<u64>(rhs));
        const bool overflow =
            (rhs > 0 && lhs > std::numeric_limits<s64>::max() - rhs) ||
            (rhs < 0 && lhs < std::numeric_limits<s64>::min() - rhs);
        if (overflow) {
            raise_exception(12u, pc, current_is_delay_slot_);
        } else {
            write_gpr64(rd, static_cast<u64>(result));
        }
        return true;
    }
    case 0x2D: // DADDU
        write_gpr64(rd, gpr_u64(rs) + gpr_u64(rt));
        return true;
    case 0x2E: { // DSUB
        const s64 lhs = static_cast<s64>(gpr_u64(rs));
        const s64 rhs = static_cast<s64>(gpr_u64(rt));
        const bool overflow =
            (rhs < 0 && lhs > std::numeric_limits<s64>::max() + rhs) ||
            (rhs > 0 && lhs < std::numeric_limits<s64>::min() + rhs);
        if (overflow) {
            raise_exception(12u, pc, current_is_delay_slot_);
        } else {
            write_gpr64(rd, static_cast<u64>(lhs - rhs));
        }
        return true;
    }
    case 0x2F: // DSUBU
        write_gpr64(rd, gpr_u64(rs) - gpr_u64(rt));
        return true;
    case 0x30: // TGE
        if (gpr_s64(rs) >= gpr_s64(rt)) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
    case 0x31: // TGEU
        if (gpr_u64(rs) >= gpr_u64(rt)) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
    case 0x32: // TLT
        if (gpr_s64(rs) < gpr_s64(rt)) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
    case 0x33: // TLTU
        if (gpr_u64(rs) < gpr_u64(rt)) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
    case 0x34: // TEQ
        if (gpr_u64(rs) == gpr_u64(rt)) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
    case 0x36: // TNE
        if (gpr_u64(rs) != gpr_u64(rt)) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
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
    case 0x18: // MTSAB
        state_.sa =
            (static_cast<u32>(gpr_u64(rs)) & 0xFu) ^
            (static_cast<u32>(immediate(instruction)) & 0xFu);
        return true;
    case 0x19: // MTSAH
        state_.sa =
            ((static_cast<u32>(gpr_u64(rs)) & 0x7u) ^
             (static_cast<u32>(immediate(instruction)) & 0x7u)) << 1u;
        return true;
    case 0x08: // TGEI
        if (gpr_s64(rs) >= static_cast<s64>(immediate(instruction))) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
    case 0x09: // TGEIU
        if (gpr_u64(rs) >= static_cast<u64>(
                static_cast<s64>(immediate(instruction)))) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
    case 0x0A: // TLTI
        if (gpr_s64(rs) < static_cast<s64>(immediate(instruction))) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
    case 0x0B: // TLTIU
        if (gpr_u64(rs) < static_cast<u64>(
                static_cast<s64>(immediate(instruction)))) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
    case 0x0C: // TEQI
        if (gpr_u64(rs) == static_cast<u64>(
                static_cast<s64>(immediate(instruction)))) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
    case 0x0E: // TNEI
        if (gpr_u64(rs) != static_cast<u64>(
                static_cast<s64>(immediate(instruction)))) {
            raise_exception(13u, pc, current_is_delay_slot_);
        }
        return true;
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
        case 0x01: { // TLBR
            const u32 index = state_.cop0[0] & 0x3Fu;
            if (index < state_.tlb.size()) {
                const auto& e = state_.tlb[index];
                state_.cop0[5] = e.page_mask;
                state_.cop0[10] = e.entry_hi;
                state_.cop0[2] = e.entry_lo0;
                state_.cop0[3] = e.entry_lo1;
            }
            return true;
        }
        case 0x02: { // TLBWI
            const u32 index = state_.cop0[0] & 0x3Fu;
            if(index<state_.tlb.size()){
                auto& e=state_.tlb[index];
                e.page_mask=state_.cop0[5];
                e.entry_hi=state_.cop0[10];
                e.entry_lo0=state_.cop0[2];
                e.entry_lo1=state_.cop0[3];
            }
            return true;
        }
        case 0x06: { // TLBWR
            const u32 index = state_.cop0[1] % state_.tlb.size();
            auto& e = state_.tlb[index];
            e.page_mask = state_.cop0[5];
            e.entry_hi = state_.cop0[10];
            e.entry_lo0 = state_.cop0[2];
            e.entry_lo1 = state_.cop0[3];
            return true;
        }
        case 0x08: { // TLBP
            const u32 probe = state_.cop0[10];
            state_.cop0[0] = 0x80000000u;
            for (u32 index = 0; index < state_.tlb.size(); ++index) {
                const auto& e = state_.tlb[index];
                const u32 vpn_mask =
                    ~(e.page_mask | 0x1FFFu);
                const bool vpn_match =
                    (e.entry_hi & vpn_mask) ==
                    (probe & vpn_mask);
                const bool global =
                    (e.entry_lo0 & 1u) != 0 &&
                    (e.entry_lo1 & 1u) != 0;
                const bool asid_match =
                    global ||
                    ((e.entry_hi & 0xFFu) ==
                     (probe & 0xFFu));
                if (vpn_match && asid_match) {
                    state_.cop0[0] = index;
                    break;
                }
            }
            return true;
        }
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

    auto vu_float = [&](u32 reg, u32 lane) {
        return ps2_fpu_input(lane_read(reg, lane));
    };
    auto vu_write_float = [&](u32 reg, u32 lane, float value) {
        lane_write(reg, lane, ps2_fpu_result(value));
    };
    auto vector_binary = [&](auto op) {
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            vu_write_float(
                fd,
                lane,
                op(vu_float(fs, lane), vu_float(ft, lane)));
        }
    };
    auto broadcast_binary = [&](u32 source_lane, auto op) {
        const float scalar = vu_float(ft, source_lane & 3u);
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            vu_write_float(
                fd,
                lane,
                op(vu_float(fs, lane), scalar));
        }
    };

    const auto add = [](float x, float y) { return x + y; };
    const auto sub = [](float x, float y) { return x - y; };
    const auto mul = [](float x, float y) { return x * y; };
    const auto vmax = [](float x, float y) { return std::fmax(x, y); };
    const auto vmin = [](float x, float y) { return std::fmin(x, y); };

    if (funct <= 0x03u) { // VADDx/y/z/w
        broadcast_binary(funct, add);
        return true;
    }
    if (funct >= 0x04u && funct <= 0x07u) { // VSUBx/y/z/w
        broadcast_binary(funct & 3u, sub);
        return true;
    }
    if (funct >= 0x10u && funct <= 0x13u) { // VMAXx/y/z/w
        broadcast_binary(funct & 3u, vmax);
        return true;
    }
    if (funct >= 0x14u && funct <= 0x17u) { // VMINIx/y/z/w
        broadcast_binary(funct & 3u, vmin);
        return true;
    }
    if (funct >= 0x18u && funct <= 0x1Bu) { // VMULx/y/z/w
        broadcast_binary(funct & 3u, mul);
        return true;
    }

    switch (funct) {
    case 0x28: // VADD
        vector_binary(add);
        return true;
    case 0x2A: // VMUL
        vector_binary(mul);
        return true;
    case 0x2B: // VMAX
        vector_binary(vmax);
        return true;
    case 0x2C: // VSUB
        vector_binary(sub);
        return true;
    case 0x2F: // VMINI
        vector_binary(vmin);
        return true;
    default:
        break;
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

bool EeCpu::execute_mmi(
    u32 pc,
    u32 instruction,
    std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;
    const u32 sa = (instruction >> 6) & 31u;
    const u32 funct = instruction & 63u;

    const EeGpr a = state_.gpr[rs];
    const EeGpr b = state_.gpr[rt];

    auto get8 = [](const EeGpr& value, u32 lane) -> u8 {
        const u64 half = lane < 8u ? value.lo : value.hi;
        return static_cast<u8>(half >> ((lane & 7u) * 8u));
    };
    auto get16 = [](const EeGpr& value, u32 lane) -> u16 {
        const u64 half = lane < 4u ? value.lo : value.hi;
        return static_cast<u16>(half >> ((lane & 3u) * 16u));
    };
    auto get32 = [](const EeGpr& value, u32 lane) -> u32 {
        const u64 half = lane < 2u ? value.lo : value.hi;
        return static_cast<u32>(half >> ((lane & 1u) * 32u));
    };
    auto set8 = [](EeGpr& value, u32 lane, u8 v) {
        u64& half = lane < 8u ? value.lo : value.hi;
        const u32 shift = (lane & 7u) * 8u;
        half =
            (half & ~(0xFFull << shift)) |
            (static_cast<u64>(v) << shift);
    };
    auto set16 = [](EeGpr& value, u32 lane, u16 v) {
        u64& half = lane < 4u ? value.lo : value.hi;
        const u32 shift = (lane & 3u) * 16u;
        half =
            (half & ~(0xFFFFull << shift)) |
            (static_cast<u64>(v) << shift);
    };
    auto set32 = [](EeGpr& value, u32 lane, u32 v) {
        u64& half = lane < 2u ? value.lo : value.hi;
        const u32 shift = (lane & 1u) * 32u;
        half =
            (half & ~(0xFFFFFFFFull << shift)) |
            (static_cast<u64>(v) << shift);
    };
    auto store = [&](const EeGpr& value) {
        if (rd != 0u) state_.gpr[rd] = value;
    };

    auto mmi0 = [&](u32 sub) -> bool {
        EeGpr out{};
        switch (sub) {
        case 0x00: // PADDW
        case 0x01: // PSUBW
        case 0x02: // PCGTW
        case 0x03: // PMAXW
            for (u32 i = 0; i < 4u; ++i) {
                const u32 av = get32(a, i);
                const u32 bv = get32(b, i);
                u32 v = 0;
                if (sub == 0x00u) v = av + bv;
                else if (sub == 0x01u) v = av - bv;
                else if (sub == 0x02u) {
                    v = static_cast<s32>(av) > static_cast<s32>(bv)
                        ? 0xFFFFFFFFu : 0u;
                } else {
                    v = static_cast<s32>(av) > static_cast<s32>(bv)
                        ? av : bv;
                }
                set32(out, i, v);
            }
            store(out);
            return true;

        case 0x04: // PADDH
        case 0x05: // PSUBH
        case 0x06: // PCGTH
        case 0x07: // PMAXH
            for (u32 i = 0; i < 8u; ++i) {
                const u16 av = get16(a, i);
                const u16 bv = get16(b, i);
                u16 v = 0;
                if (sub == 0x04u) v = static_cast<u16>(av + bv);
                else if (sub == 0x05u) v = static_cast<u16>(av - bv);
                else if (sub == 0x06u) {
                    v = static_cast<s16>(av) > static_cast<s16>(bv)
                        ? 0xFFFFu : 0u;
                } else {
                    v = static_cast<s16>(av) > static_cast<s16>(bv)
                        ? av : bv;
                }
                set16(out, i, v);
            }
            store(out);
            return true;

        case 0x08: // PADDB
        case 0x09: // PSUBB
        case 0x0A: // PCGTB
            for (u32 i = 0; i < 16u; ++i) {
                const u8 av = get8(a, i);
                const u8 bv = get8(b, i);
                u8 v = 0;
                if (sub == 0x08u) v = static_cast<u8>(av + bv);
                else if (sub == 0x09u) v = static_cast<u8>(av - bv);
                else {
                    v = static_cast<s8>(av) > static_cast<s8>(bv)
                        ? 0xFFu : 0u;
                }
                set8(out, i, v);
            }
            store(out);
            return true;

        case 0x12: // PEXTLW
            for (u32 i = 0; i < 2u; ++i) {
                set32(out, i * 2u, get32(b, i));
                set32(out, i * 2u + 1u, get32(a, i));
            }
            store(out);
            return true;
        case 0x13: // PPACW
            set32(out, 0u, get32(b, 0u));
            set32(out, 1u, get32(b, 2u));
            set32(out, 2u, get32(a, 0u));
            set32(out, 3u, get32(a, 2u));
            store(out);
            return true;
        case 0x16: // PEXTLH
            for (u32 i = 0; i < 4u; ++i) {
                set16(out, i * 2u, get16(b, i));
                set16(out, i * 2u + 1u, get16(a, i));
            }
            store(out);
            return true;
        case 0x17: // PPACH
            for (u32 i = 0; i < 4u; ++i) {
                set16(out, i, get16(b, i * 2u));
                set16(out, i + 4u, get16(a, i * 2u));
            }
            store(out);
            return true;
        case 0x1A: // PEXTLB
            for (u32 i = 0; i < 8u; ++i) {
                set8(out, i * 2u, get8(b, i));
                set8(out, i * 2u + 1u, get8(a, i));
            }
            store(out);
            return true;
        case 0x1B: // PPACB
            for (u32 i = 0; i < 8u; ++i) {
                set8(out, i, get8(b, i * 2u));
                set8(out, i + 8u, get8(a, i * 2u));
            }
            store(out);
            return true;
        case 0x1E: // PEXT5
            for (u32 i = 0; i < 4u; ++i) {
                const u32 v = get32(b, i);
                set32(
                    out,
                    i,
                    ((v & 0x001Fu) << 3) |
                    ((v & 0x03E0u) << 6) |
                    ((v & 0x7C00u) << 9) |
                    ((v & 0x8000u) << 16));
            }
            store(out);
            return true;
        case 0x1F: // PPAC5
            for (u32 i = 0; i < 4u; ++i) {
                const u32 v = get32(b, i);
                set32(
                    out,
                    i,
                    ((v >> 3) & 0x001Fu) |
                    ((v >> 6) & 0x03E0u) |
                    ((v >> 9) & 0x7C00u) |
                    ((v >> 16) & 0x8000u));
            }
            store(out);
            return true;
        default:
            return false;
        }
    };

    auto mmi1 = [&](u32 sub) -> bool {
        EeGpr out{};
        switch (sub) {
        case 0x01: // PABSW
            for (u32 i = 0; i < 4u; ++i) {
                const s32 v = static_cast<s32>(get32(b, i));
                set32(
                    out,
                    i,
                    v == std::numeric_limits<s32>::min()
                        ? 0x7FFFFFFFu
                        : static_cast<u32>(v < 0 ? -v : v));
            }
            store(out);
            return true;
        case 0x02: // PCEQW
        case 0x03: // PMINW
            for (u32 i = 0; i < 4u; ++i) {
                const u32 av = get32(a, i);
                const u32 bv = get32(b, i);
                set32(
                    out,
                    i,
                    sub == 0x02u
                        ? (av == bv ? 0xFFFFFFFFu : 0u)
                        : (static_cast<s32>(av) < static_cast<s32>(bv)
                            ? av : bv));
            }
            store(out);
            return true;
        case 0x05: // PABSH
            for (u32 i = 0; i < 8u; ++i) {
                const s16 v = static_cast<s16>(get16(b, i));
                set16(
                    out,
                    i,
                    v == std::numeric_limits<s16>::min()
                        ? 0x7FFFu
                        : static_cast<u16>(v < 0 ? -v : v));
            }
            store(out);
            return true;
        case 0x06: // PCEQH
        case 0x07: // PMINH
            for (u32 i = 0; i < 8u; ++i) {
                const u16 av = get16(a, i);
                const u16 bv = get16(b, i);
                set16(
                    out,
                    i,
                    sub == 0x06u
                        ? (av == bv ? 0xFFFFu : 0u)
                        : (static_cast<s16>(av) < static_cast<s16>(bv)
                            ? av : bv));
            }
            store(out);
            return true;
        case 0x0A: // PCEQB
            for (u32 i = 0; i < 16u; ++i) {
                set8(
                    out,
                    i,
                    get8(a, i) == get8(b, i) ? 0xFFu : 0u);
            }
            store(out);
            return true;
        case 0x10: // PADDUW
        case 0x11: // PSUBUW
            for (u32 i = 0; i < 4u; ++i) {
                const u64 av = get32(a, i);
                const u64 bv = get32(b, i);
                u32 v = 0;
                if (sub == 0x10u) {
                    const u64 sum = av + bv;
                    v = sum > 0xFFFFFFFFull
                        ? 0xFFFFFFFFu
                        : static_cast<u32>(sum);
                } else {
                    v = av <= bv
                        ? 0u
                        : static_cast<u32>(av - bv);
                }
                set32(out, i, v);
            }
            store(out);
            return true;
        case 0x12: // PEXTUW
            for (u32 i = 0; i < 2u; ++i) {
                set32(out, i * 2u, get32(b, i + 2u));
                set32(out, i * 2u + 1u, get32(a, i + 2u));
            }
            store(out);
            return true;
        case 0x14: // PADDUH
        case 0x15: // PSUBUH
            for (u32 i = 0; i < 8u; ++i) {
                const u32 av = get16(a, i);
                const u32 bv = get16(b, i);
                u16 v = 0;
                if (sub == 0x14u) {
                    const u32 sum = av + bv;
                    v = static_cast<u16>(
                        sum > 0xFFFFu ? 0xFFFFu : sum);
                } else {
                    v = av <= bv
                        ? 0u
                        : static_cast<u16>(av - bv);
                }
                set16(out, i, v);
            }
            store(out);
            return true;
        case 0x16: // PEXTUH
            for (u32 i = 0; i < 4u; ++i) {
                set16(out, i * 2u, get16(b, i + 4u));
                set16(out, i * 2u + 1u, get16(a, i + 4u));
            }
            store(out);
            return true;
        case 0x18: // PADDUB
        case 0x19: // PSUBUB
            for (u32 i = 0; i < 16u; ++i) {
                const u32 av = get8(a, i);
                const u32 bv = get8(b, i);
                u8 v = 0;
                if (sub == 0x18u) {
                    const u32 sum = av + bv;
                    v = static_cast<u8>(
                        sum > 0xFFu ? 0xFFu : sum);
                } else {
                    v = av <= bv
                        ? 0u
                        : static_cast<u8>(av - bv);
                }
                set8(out, i, v);
            }
            store(out);
            return true;
        case 0x1A: // PEXTUB
            for (u32 i = 0; i < 8u; ++i) {
                set8(out, i * 2u, get8(b, i + 8u));
                set8(out, i * 2u + 1u, get8(a, i + 8u));
            }
            store(out);
            return true;
        case 0x1B: { // QFSRV
            const u32 shift = (state_.sa & 0xFu) << 3u;
            if (shift == 0u) {
                out = b;
            } else if (shift < 64u) {
                out.lo =
                    (b.lo >> shift) |
                    (b.hi << (64u - shift));
                out.hi =
                    (b.hi >> shift) |
                    (a.lo << (64u - shift));
            } else {
                const u32 s = shift - 64u;
                out.lo = b.hi >> s;
                out.hi = a.lo >> s;
                if (s != 0u) {
                    out.lo |= a.lo << (64u - s);
                    out.hi |= a.hi << (64u - s);
                }
            }
            store(out);
            return true;
        }
        default:
            return false;
        }
    };

    auto mmi2 = [&](u32 sub) -> bool {
        EeGpr out{};
        switch (sub) {
        case 0x0E: // PCPYLD
            out.lo = b.lo;
            out.hi = a.lo;
            store(out);
            return true;
        case 0x12: // PAND
            out.lo = a.lo & b.lo;
            out.hi = a.hi & b.hi;
            store(out);
            return true;
        case 0x13: // PXOR
            out.lo = a.lo ^ b.lo;
            out.hi = a.hi ^ b.hi;
            store(out);
            return true;
        case 0x1A: // PEXEH
            for (u32 i = 0; i < 8u; ++i) {
                static constexpr u32 order[8] =
                    {0u, 2u, 1u, 3u, 4u, 6u, 5u, 7u};
                set16(out, i, get16(b, order[i]));
            }
            store(out);
            return true;
        case 0x1B: // PREVH
            for (u32 i = 0; i < 8u; ++i) {
                static constexpr u32 order[8] =
                    {2u, 1u, 0u, 3u, 6u, 5u, 4u, 7u};
                set16(out, i, get16(b, order[i]));
            }
            store(out);
            return true;
        case 0x1E: // PEXEW
            set32(out, 0u, get32(b, 0u));
            set32(out, 1u, get32(b, 2u));
            set32(out, 2u, get32(b, 1u));
            set32(out, 3u, get32(b, 3u));
            store(out);
            return true;
        case 0x1F: // PROT3W
            set32(out, 0u, get32(b, 1u));
            set32(out, 1u, get32(b, 2u));
            set32(out, 2u, get32(b, 0u));
            set32(out, 3u, get32(b, 3u));
            store(out);
            return true;
        default:
            return false;
        }
    };

    auto mmi3 = [&](u32 sub) -> bool {
        EeGpr out{};
        switch (sub) {
        case 0x0E: // PCPYUD
            out.lo = a.hi;
            out.hi = b.hi;
            store(out);
            return true;
        case 0x12: // POR
            out.lo = a.lo | b.lo;
            out.hi = a.hi | b.hi;
            store(out);
            return true;
        case 0x13: // PNOR
            out.lo = ~(a.lo | b.lo);
            out.hi = ~(a.hi | b.hi);
            store(out);
            return true;
        case 0x1A: // PEXCH
            for (u32 i = 0; i < 8u; ++i) {
                static constexpr u32 order[8] =
                    {0u, 2u, 1u, 3u, 4u, 6u, 5u, 7u};
                set16(out, i, get16(b, order[i]));
            }
            store(out);
            return true;
        case 0x1B: // PCPYH
            for (u32 i = 0; i < 4u; ++i) {
                set16(out, i, get16(b, 0u));
                set16(out, i + 4u, get16(b, 4u));
            }
            store(out);
            return true;
        case 0x1E: // PEXCW
            set32(out, 0u, get32(b, 0u));
            set32(out, 1u, get32(b, 2u));
            set32(out, 2u, get32(b, 1u));
            set32(out, 3u, get32(b, 3u));
            store(out);
            return true;
        default:
            return false;
        }
    };

    switch (funct) {
    case 0x08:
        if (mmi0(sa)) return true;
        return fail(
            pc,
            instruction,
            "Unsupported MMI0 function " + hex32(sa),
            error);
    case 0x09:
        if (mmi2(sa)) return true;
        return fail(
            pc,
            instruction,
            "Unsupported MMI2 function " + hex32(sa),
            error);
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
    case 0x28:
        if (mmi1(sa)) return true;
        return fail(
            pc,
            instruction,
            "Unsupported MMI1 function " + hex32(sa),
            error);
    case 0x29:
        if (mmi3(sa)) return true;
        return fail(
            pc,
            instruction,
            "Unsupported MMI3 function " + hex32(sa),
            error);
    case 0x34: { // PSLLH
        EeGpr out{};
        const u32 shift = sa & 0xFu;
        for (u32 i = 0; i < 8u; ++i) {
            set16(out, i, static_cast<u16>(get16(b, i) << shift));
        }
        store(out);
        return true;
    }
    case 0x36: { // PSRLH
        EeGpr out{};
        const u32 shift = sa & 0xFu;
        for (u32 i = 0; i < 8u; ++i) {
            set16(out, i, static_cast<u16>(get16(b, i) >> shift));
        }
        store(out);
        return true;
    }
    case 0x37: { // PSRAH
        EeGpr out{};
        const u32 shift = sa & 0xFu;
        for (u32 i = 0; i < 8u; ++i) {
            set16(
                out,
                i,
                static_cast<u16>(
                    static_cast<s16>(get16(b, i)) >> shift));
        }
        store(out);
        return true;
    }
    case 0x3C: { // PSLLW
        EeGpr out{};
        for (u32 i = 0; i < 4u; ++i) {
            set32(out, i, get32(b, i) << sa);
        }
        store(out);
        return true;
    }
    case 0x3E: { // PSRLW
        EeGpr out{};
        for (u32 i = 0; i < 4u; ++i) {
            set32(out, i, get32(b, i) >> sa);
        }
        store(out);
        return true;
    }
    case 0x3F: { // PSRAW
        EeGpr out{};
        for (u32 i = 0; i < 4u; ++i) {
            set32(
                out,
                i,
                static_cast<u32>(
                    static_cast<s32>(get32(b, i)) >> sa));
        }
        store(out);
        return true;
    }
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
    u32 fetch_address = 0;
    if (!translate_address(
            pc,
            false,
            pc,
            current_is_delay_slot_,
            fetch_address)) {
        memory_exception_pending_ = false;
        ++state_.instructions_executed;
        ++state_.cop0[9];
        if (state_.cop0[9] == state_.cop0[11]) {
            state_.cop0[13] |= 0x00008000u;
        }
        bus_.tick(1);
        return true;
    }
    if (!bus_.read32(fetch_address, instruction)) {
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

    const auto translate_memory = [&](u32 address, bool store, u32& out) {
        return translate_address(
            address,
            store,
            pc,
            current_is_delay_slot_,
            out);
    };
    const auto read8_mem = [&](u32 address, u8& value) {
        u32 translated = 0;
        return translate_memory(address, false, translated) &&
               bus_.read8(translated, value);
    };
    const auto read16_mem = [&](u32 address, u16& value) {
        u32 translated = 0;
        return translate_memory(address, false, translated) &&
               bus_.read16(translated, value);
    };
    const auto read32_mem = [&](u32 address, u32& value) {
        u32 translated = 0;
        return translate_memory(address, false, translated) &&
               bus_.read32(translated, value);
    };
    const auto read64_mem = [&](u32 address, u64& value) {
        u32 translated = 0;
        return translate_memory(address, false, translated) &&
               bus_.read64(translated, value);
    };
    const auto write8_mem = [&](u32 address, u8 value) {
        u32 translated = 0;
        return translate_memory(address, true, translated) &&
               bus_.write8(translated, value);
    };
    const auto write16_mem = [&](u32 address, u16 value) {
        u32 translated = 0;
        return translate_memory(address, true, translated) &&
               bus_.write16(translated, value);
    };
    const auto write32_mem = [&](u32 address, u32 value) {
        u32 translated = 0;
        return translate_memory(address, true, translated) &&
               bus_.write32(translated, value);
    };
    const auto write64_mem = [&](u32 address, u64 value) {
        u32 translated = 0;
        return translate_memory(address, true, translated) &&
               bus_.write64(translated, value);
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
            raise_exception(12u, pc, current_is_delay_slot_);
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
    case 0x18: { // DADDI
        const s64 lhs = static_cast<s64>(gpr_u64(rs));
        const s64 rhs = static_cast<s64>(imm);
        const bool overflow =
            (rhs > 0 && lhs > std::numeric_limits<s64>::max() - rhs) ||
            (rhs < 0 && lhs < std::numeric_limits<s64>::min() - rhs);
        if (overflow) {
            raise_exception(12u, pc, current_is_delay_slot_);
        } else {
            write_gpr64(rt, static_cast<u64>(lhs + rhs));
        }
        break;
    }
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
        if (!read64_mem(address & ~7u, mem)) ok = load_fault("LDL", address);
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
        if (!read64_mem(address & ~7u, mem)) ok = load_fault("LDR", address);
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
        if (!read64_mem(address, lo) ||
            !read64_mem(address + 8u, hi)) {
            ok = load_fault("Load quadword", address);
        } else if (rt != 0) {
            state_.gpr[rt].lo = lo;
            state_.gpr[rt].hi = hi;
        }
        break;
    }
    case 0x1F: { // SQ
        const u32 address = effective_address() & ~0x0Fu;
        if (!write64_mem(address, state_.gpr[rt].lo) ||
            !write64_mem(address + 8u, state_.gpr[rt].hi)) {
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
        if (!read8_mem(address, value)) {
            ok = load_fault("Load byte", address);
        } else {
            write_gpr64(rt, static_cast<u64>(static_cast<s64>(static_cast<s8>(value))));
        }
        break;
    }
    case 0x21: { // LH
        const u32 address = effective_address();
        u16 value = 0;
        if (!read16_mem(address, value)) {
            ok = load_fault("Load halfword", address);
        } else {
            write_gpr64(rt, static_cast<u64>(static_cast<s64>(static_cast<s16>(value))));
        }
        break;
    }
    case 0x22: { // LWL
        const u32 address = effective_address();
        u32 memory = 0;
        if (!read32_mem(address & ~3u, memory)) {
            ok = load_fault("LWL", address);
        } else {
            const u32 shift = (address & 3u) * 8u;
            const u32 old = static_cast<u32>(gpr_u64(rt));
            const u32 value =
                (old & (0x00FFFFFFu >> shift)) |
                (memory << (24u - shift));
            write_gpr_word(rt, value);
        }
        break;
    }
    case 0x23: { // LW
        const u32 address = effective_address();
        u32 value = 0;
        if (!read32_mem(address, value)) {
            ok = load_fault("Load word", address);
        } else {
            write_gpr_word(rt, value);
        }
        break;
    }
    case 0x24: { // LBU
        const u32 address = effective_address();
        u8 value = 0;
        if (!read8_mem(address, value)) {
            ok = load_fault("Load byte", address);
        } else {
            write_gpr64(rt, value);
        }
        break;
    }
    case 0x25: { // LHU
        const u32 address = effective_address();
        u16 value = 0;
        if (!read16_mem(address, value)) {
            ok = load_fault("Load halfword", address);
        } else {
            write_gpr64(rt, value);
        }
        break;
    }
    case 0x26: { // LWR
        const u32 address = effective_address();
        u32 memory = 0;
        if (!read32_mem(address & ~3u, memory)) {
            ok = load_fault("LWR", address);
        } else {
            const u32 shift = (address & 3u) * 8u;
            const u32 old = static_cast<u32>(gpr_u64(rt));
            const u32 value =
                (old & (0xFFFFFF00u << (24u - shift))) |
                (memory >> shift);
            write_gpr_word(rt, value);
        }
        break;
    }
    case 0x27: { // LWU
        const u32 address = effective_address();
        u32 value = 0;
        if (!read32_mem(address, value)) {
            ok = load_fault("Load word", address);
        } else {
            write_gpr64(rt, value);
        }
        break;
    }
    case 0x28: { // SB
        const u32 address = effective_address();
        if (!write8_mem(address, static_cast<u8>(gpr_u64(rt)))) {
            ok = fail(pc, instruction, "Store byte fault to " + hex32(address), error);
        }
        break;
    }
    case 0x29: { // SH
        const u32 address = effective_address();
        if (!write16_mem(address, static_cast<u16>(gpr_u64(rt)))) {
            ok = fail(pc, instruction, "Store halfword fault to " + hex32(address), error);
        }
        break;
    }
    case 0x2A: { // SWL
        const u32 address = effective_address();
        const u32 aligned = address & ~3u;
        u32 memory = 0;
        if (!read32_mem(aligned, memory)) {
            ok = load_fault("SWL read", address);
        } else {
            const u32 shift = (address & 3u) * 8u;
            const u32 value =
                (static_cast<u32>(gpr_u64(rt)) >> (24u - shift)) |
                (memory & (0xFFFFFF00u << shift));
            if (!write32_mem(aligned, value)) {
                ok = fail(pc, instruction, "SWL fault to " + hex32(address), error);
            }
        }
        break;
    }
    case 0x2B: { // SW
        const u32 address = effective_address();
        if (!write32_mem(address, static_cast<u32>(gpr_u64(rt)))) {
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
        if (!read64_mem(address & ~7u, mem) || !write64_mem(address & ~7u, (gpr_u64(rt) >> shifts[shift]) | (mem & masks[shift])))
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
        if (!read64_mem(address & ~7u, mem) || !write64_mem(address & ~7u, (gpr_u64(rt) << shifts[shift]) | (mem & masks[shift])))
            ok = fail(pc,instruction,"SDR fault at "+hex32(address),error);
        break;
    }
    case 0x2E: { // SWR
        const u32 address = effective_address();
        const u32 aligned = address & ~3u;
        u32 memory = 0;
        if (!read32_mem(aligned, memory)) {
            ok = load_fault("SWR read", address);
        } else {
            const u32 shift = (address & 3u) * 8u;
            const u32 value =
                (static_cast<u32>(gpr_u64(rt)) << shift) |
                (memory & (0x00FFFFFFu >> (24u - shift)));
            if (!write32_mem(aligned, value)) {
                ok = fail(pc, instruction, "SWR fault to " + hex32(address), error);
            }
        }
        break;
    }
    case 0x2F: // CACHE
    case 0x33: // PREF
        break;
    case 0x30: { // LL
        const u32 address = effective_address();
        u32 value = 0;
        if (!read32_mem(address, value)) {
            ok = load_fault("LL", address);
        } else {
            // Bootstrap is single-threaded; no competing agent can invalidate
            // the reservation between LL and SC yet.
            write_gpr_word(rt, value);
        }
        break;
    }
    case 0x34: { // LLD
        const u32 address = effective_address();
        u64 value = 0;
        if (!read64_mem(address, value)) {
            ok = load_fault("LLD", address);
        } else {
            write_gpr64(rt, value);
        }
        break;
    }
    case 0x31: { // LWC1
        const u32 address = effective_address();
        u32 value = 0;
        if (!read32_mem(address, value)) {
            ok = load_fault("LWC1", address);
        } else {
            state_.fpr[rt] = value;
        }
        break;
    }
    case 0x37: { // LD
        const u32 address = effective_address();
        u64 value = 0;
        if (!read64_mem(address, value)) {
            ok = load_fault("Load doubleword", address);
        } else {
            write_gpr64(rt, value);
        }
        break;
    }
    case 0x38: { // SC
        const u32 address = effective_address();
        const u32 value = static_cast<u32>(gpr_u64(rt));
        if (!write32_mem(address, value)) {
            ok = fail(pc, instruction, "SC fault to " + hex32(address), error);
        } else {
            write_gpr_word(rt, 1u);
        }
        break;
    }
    case 0x39: { // SWC1
        const u32 address = effective_address();
        if (!write32_mem(address, state_.fpr[rt])) {
            ok = fail(pc, instruction, "SWC1 fault to " + hex32(address), error);
        }
        break;
    }
    case 0x3C: { // SCD
        const u32 address = effective_address();
        const u64 value = gpr_u64(rt);
        if (!write64_mem(address, value)) {
            ok = fail(pc, instruction, "SCD fault to " + hex32(address), error);
        } else {
            write_gpr64(rt, 1u);
        }
        break;
    }
    case 0x3F: { // SD
        const u32 address = effective_address();
        if (!write64_mem(address, gpr_u64(rt))) {
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
