#include "core/ee/ee_cpu.h"

#include "core/memory/ee_bus.h"
#include "core/vu/vu1.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
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

void madd_signed32(
    u32 lhs,
    u32 rhs,
    u64& lo,
    u64& hi) {
    const u64 accumulator =
        static_cast<u64>(static_cast<u32>(lo)) |
        (static_cast<u64>(static_cast<u32>(hi)) << 32);
    const s64 product =
        static_cast<s64>(static_cast<s32>(lhs)) *
        static_cast<s64>(static_cast<s32>(rhs));
    const u64 result =
        accumulator + static_cast<u64>(product);
    lo = sign_extend_32(static_cast<u32>(result));
    hi = sign_extend_32(static_cast<u32>(result >> 32));
}

void madd_unsigned32(
    u32 lhs,
    u32 rhs,
    u64& lo,
    u64& hi) {
    const u64 accumulator =
        static_cast<u64>(static_cast<u32>(lo)) |
        (static_cast<u64>(static_cast<u32>(hi)) << 32);
    const u64 result =
        accumulator +
        static_cast<u64>(lhs) * static_cast<u64>(rhs);
    lo = sign_extend_32(static_cast<u32>(result));
    hi = sign_extend_32(static_cast<u32>(result >> 32));
}

u32 leading_sign_bits_excluding_sign(u32 value) {
    if ((value & 0x80000000u) != 0) {
        value = ~value;
    }
    return std::countl_zero(value) - 1u;
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
    state_.vu_vi[22] = 0x3F800000u;
    halted_ = false;
    next_is_delay_slot_ = false;
    current_is_delay_slot_ = false;
    memory_exception_pending_ = false;
    hot_sif_getreg_diag_inflight_ = false;
    hot_sif_getreg_diag_start_ = 0u;
    hot_sif_getreg_read_offset_ = 0u;
    fast_prefix_fallback_opcodes_->fill(0u);
    fast_prefix_cop2_fallback_words_->fill(0u);
    fast_prefix_cop2_fallback_counts_->fill(0u);
    fast_prefix_cop2_fallback_count_ = 0u;
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
    if (code < state_.exception_counts.size()) {
        ++state_.exception_counts[code];
    }

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
    case 0x0C: { // SYSCALL
        if (pc == 0x0024DE74u &&
            static_cast<u32>(gpr_u64(3)) == 0x7Au &&
            static_cast<u32>(gpr_u64(4)) == 4u &&
            hot_sif_getreg_read_offset_ == 0u) {
            hot_sif_getreg_diag_inflight_ = true;
            hot_sif_getreg_diag_start_ =
                state_.instructions_executed;
        }

        auto& record =
            state_.recent_syscalls[state_.recent_syscall_next];
        record.instruction = state_.instructions_executed;
        record.pc = pc;
        record.number = static_cast<u32>(gpr_u64(3));
        for (u32 i = 0; i < record.args.size(); ++i) {
            record.args[i] = gpr_u64(4u + i);
        }
        state_.recent_syscall_next =
            (state_.recent_syscall_next + 1u) %
            static_cast<u32>(state_.recent_syscalls.size());
        state_.recent_syscall_count = std::min(
            state_.recent_syscall_count + 1u,
            static_cast<u32>(state_.recent_syscalls.size()));
        raise_exception(8u, pc, current_is_delay_slot_);
        return true;
    }
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
    if(rs==0x08){ // BC0F / BC0T / BC0FL / BC0TL
        if(rt>3u) return fail(pc,instruction,"Unsupported BC0 condition branch",error);
        u32 dmac_stat=0, dmac_pcr=0;
        if(!bus_.read32(0x1000E010u,dmac_stat) ||
           !bus_.read32(0x1000E020u,dmac_pcr))
            return fail(pc,instruction,"BC0 DMAC condition read fault",error);
        const bool condition=
            (((dmac_stat | ~dmac_pcr) & 0x3FFu) == 0x3FFu);
        const bool branch_on_true=(rt & 1u)!=0;
        const bool likely=(rt & 2u)!=0;
        const bool take=condition==branch_on_true;
        if(take){
            state_.next_pc=branch_target(pc,immediate(instruction));
            next_is_delay_slot_=true;
        } else if(likely){
            branch_likely_not_taken(pc);
        } else {
            next_is_delay_slot_=true;
        }
        return true;
    }
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
        case 0x38: { // EI
            const u32 status=state_.cop0[12];
            if ((status & 0x00020000u) != 0 || (status & 0x6u) != 0 || (status & 0x18u) == 0)
                state_.cop0[12]|=0x00010000u;
            return true;
        }
        case 0x39: { // DI
            const u32 status=state_.cop0[12];
            if ((status & 0x00020000u) != 0 || (status & 0x6u) != 0 || (status & 0x18u) == 0)
                state_.cop0[12]&=~0x00010000u;
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

void EeCpu::sync_vu0_to_micro() {
    if (vu0_micro_ == nullptr) return;

    auto vf_lane = [](const EeGpr& value, u32 lane) -> u32 {
        const u64 half = lane < 2u ? value.lo : value.hi;
        return static_cast<u32>(
            half >> ((lane & 1u) * 32u));
    };

    for (u32 reg = 1; reg < 32u; ++reg) {
        for (u32 lane = 0; lane < 4u; ++lane) {
            vu0_micro_->set_vf(
                reg, lane, vf_lane(state_.vu_vf[reg], lane));
        }
    }
    for (u32 reg = 1; reg < 16u; ++reg) {
        vu0_micro_->set_vi(
            reg, static_cast<u16>(state_.vu_vi[reg]));
    }
    vu0_micro_->set_acc(0u, static_cast<u32>(state_.vu_acc.lo));
    vu0_micro_->set_acc(1u, static_cast<u32>(state_.vu_acc.lo >> 32));
    vu0_micro_->set_acc(2u, static_cast<u32>(state_.vu_acc.hi));
    vu0_micro_->set_acc(3u, static_cast<u32>(state_.vu_acc.hi >> 32));
    vu0_micro_->set_status(state_.vu_vi[16]);
    vu0_micro_->set_mac(state_.vu_vi[17]);
    vu0_micro_->set_clip(state_.vu_vi[18]);
    vu0_micro_->set_random(state_.vu_vi[20]);
    vu0_micro_->set_immediate(state_.vu_vi[21]);
    vu0_micro_->set_q(state_.vu_vi[22]);
    vu0_micro_->set_p(state_.vu_vi[23]);
}

void EeCpu::sync_vu0_from_micro() {
    if (vu0_micro_ == nullptr) return;

    for (u32 reg = 1; reg < 32u; ++reg) {
        state_.vu_vf[reg].lo =
            static_cast<u64>(vu0_micro_->vf(reg, 0u)) |
            (static_cast<u64>(vu0_micro_->vf(reg, 1u)) << 32);
        state_.vu_vf[reg].hi =
            static_cast<u64>(vu0_micro_->vf(reg, 2u)) |
            (static_cast<u64>(vu0_micro_->vf(reg, 3u)) << 32);
    }
    for (u32 reg = 1; reg < 16u; ++reg) {
        state_.vu_vi[reg] = vu0_micro_->vi(reg);
    }
    state_.vu_acc.lo =
        static_cast<u64>(vu0_micro_->acc(0u)) |
        (static_cast<u64>(vu0_micro_->acc(1u)) << 32);
    state_.vu_acc.hi =
        static_cast<u64>(vu0_micro_->acc(2u)) |
        (static_cast<u64>(vu0_micro_->acc(3u)) << 32);
    state_.vu_vi[16] = vu0_micro_->status();
    state_.vu_vi[17] = vu0_micro_->mac();
    state_.vu_vi[18] = vu0_micro_->clip();
    state_.vu_vi[20] = vu0_micro_->random();
    state_.vu_vi[21] = vu0_micro_->immediate();
    state_.vu_vi[22] = vu0_micro_->q();
    state_.vu_vi[23] = vu0_micro_->p();
    state_.vu_vi[26] = (vu0_micro_->pc() / 8u) & 0x1FFu;
}

void EeCpu::set_vu0_micro_running(bool running) {
    if (running) state_.vu_vi[29] |= 1u;
    else state_.vu_vi[29] &= ~1u;
}

bool EeCpu::run_vu0_micro(
    u32 start_address,
    std::string& error) {
    error.clear();
    if (vu0_micro_ == nullptr) {
        error = "VU0 micro interpreter is not attached";
        return false;
    }

    sync_vu0_to_micro();
    set_vu0_micro_running(true);
    vu0_micro_->start(start_address);

    std::string vu_error;
    constexpr u64 kBootstrapMicroBudget = 262144u;
    vu0_micro_->run(kBootstrapMicroBudget, vu_error);
    set_vu0_micro_running(false);

    if (!vu_error.empty()) {
        error = vu_error;
        return false;
    }
    if (vu0_micro_->running()) {
        error = "VU0 microprogram exceeded bootstrap instruction budget";
        return false;
    }

    sync_vu0_from_micro();
    return true;
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
        half =
            (half & ~(0xFFFFFFFFull << shift)) |
            (static_cast<u64>(value) << shift);
    };
    auto acc_read = [&](u32 lane) -> u32 {
        const u64 half = lane < 2u ? state_.vu_acc.lo : state_.vu_acc.hi;
        return static_cast<u32>(half >> ((lane & 1u) * 32u));
    };
    auto acc_write = [&](u32 lane, u32 value) {
        u64& half = lane < 2u ? state_.vu_acc.lo : state_.vu_acc.hi;
        const u32 shift = (lane & 1u) * 32u;
        half =
            (half & ~(0xFFFFFFFFull << shift)) |
            (static_cast<u64>(value) << shift);
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
    case 0x08: { // BC2F / BC2T / BC2FL / BC2TL
        const bool condition = (state_.vu_vi[29] & 0x100u) != 0;
        const bool branch_on_true = (rt & 1u) != 0;
        const bool likely = (rt & 2u) != 0;
        if (rt > 3u) {
            return fail(
                pc,
                instruction,
                "Unsupported BC2 condition branch",
                error);
        }
        const bool take = condition == branch_on_true;
        if (take) {
            state_.next_pc =
                branch_target(pc, immediate(instruction));
            next_is_delay_slot_ = true;
        } else if (likely) {
            branch_likely_not_taken(pc);
        } else {
            next_is_delay_slot_ = true;
        }
        return true;
    }
    case 0x06: { // CTC2
        if (fs == 0u || fs == 17u || fs == 26u || fs == 29u) return true;
        const u32 value = static_cast<u32>(gpr_u64(rt));
        if (fs < 16u) {
            state_.vu_vi[fs] = static_cast<u16>(value);
            return true;
        }
        if (fs == 20u) {
            state_.vu_vi[20] =
                (value & 0x007FFFFFu) | 0x3F800000u;
            return true;
        }
        if (fs == 28u) {
            state_.vu_vi[28] = value & 0x00000C0Cu;
            if ((value & 0x2u) != 0) {
                for (u32 i = 1; i < 32; ++i) state_.vu_vf[i] = {};
                for (u32 i = 1; i < 16; ++i) state_.vu_vi[i] = 0;
                state_.vu_vi[29] &= ~0xFFu;
            }
            if ((value & 0x200u) != 0) {
                state_.vu_vi[29] &= ~0xFF00u;
            }
            return true;
        }
        state_.vu_vi[fs] = value;
        return true;
    }
    default:
        break;
    }

    if (rs < 0x10u) {
        return fail(
            pc, instruction, "Unsupported COP2 operation", error);
    }

    const u32 ft = (instruction >> 16) & 31u;
    const u32 fd = (instruction >> 6) & 31u;
    const u32 funct = instruction & 63u;
    const u32 fsf = (instruction >> 21) & 3u;
    const u32 ftf = (instruction >> 23) & 3u;

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
    auto scalar_control_binary = [&](u32 control, auto op) {
        const float scalar = ps2_fpu_input(state_.vu_vi[control]);
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            vu_write_float(fd, lane, op(vu_float(fs, lane), scalar));
        }
    };
    auto acc_float = [&](u32 lane) {
        return ps2_fpu_input(acc_read(lane));
    };
    auto acc_write_float = [&](u32 lane, float value) {
        acc_write(lane, ps2_fpu_result(value));
    };
    auto broadcast_madd = [&](u32 source_lane, bool subtract_product) {
        const float scalar = vu_float(ft, source_lane & 3u);
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            const float product = vu_float(fs, lane) * scalar;
            vu_write_float(
                fd,
                lane,
                acc_float(lane) +
                    (subtract_product ? -product : product));
        }
    };
    auto vector_madd = [&](bool subtract_product) {
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            const float product =
                vu_float(fs, lane) * vu_float(ft, lane);
            vu_write_float(
                fd,
                lane,
                acc_float(lane) +
                    (subtract_product ? -product : product));
        }
    };
    auto control_madd = [&](u32 control, bool subtract_product) {
        const float scalar = ps2_fpu_input(state_.vu_vi[control]);
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            const float product = vu_float(fs, lane) * scalar;
            vu_write_float(
                fd,
                lane,
                acc_float(lane) +
                    (subtract_product ? -product : product));
        }
    };
    auto acc_broadcast_binary = [&](u32 source_lane, auto op) {
        const float scalar = vu_float(ft, source_lane & 3u);
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (selected(lane)) {
                acc_write_float(
                    lane,
                    op(vu_float(fs, lane), scalar));
            }
        }
    };
    auto acc_vector_binary = [&](auto op) {
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (selected(lane)) {
                acc_write_float(
                    lane,
                    op(vu_float(fs, lane), vu_float(ft, lane)));
            }
        }
    };
    auto acc_control_binary = [&](u32 control, auto op) {
        const float scalar = ps2_fpu_input(state_.vu_vi[control]);
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (selected(lane)) {
                acc_write_float(
                    lane,
                    op(vu_float(fs, lane), scalar));
            }
        }
    };
    auto acc_broadcast_madd = [&](u32 source_lane, bool subtract_product) {
        const float scalar = vu_float(ft, source_lane & 3u);
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            const float product = vu_float(fs, lane) * scalar;
            acc_write_float(
                lane,
                acc_float(lane) +
                    (subtract_product ? -product : product));
        }
    };
    auto acc_vector_madd = [&](bool subtract_product) {
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            const float product =
                vu_float(fs, lane) * vu_float(ft, lane);
            acc_write_float(
                lane,
                acc_float(lane) +
                    (subtract_product ? -product : product));
        }
    };
    auto acc_control_madd = [&](u32 control, bool subtract_product) {
        const float scalar = ps2_fpu_input(state_.vu_vi[control]);
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            const float product = vu_float(fs, lane) * scalar;
            acc_write_float(
                lane,
                acc_float(lane) +
                    (subtract_product ? -product : product));
        }
    };
    auto vu0_address = [&](u32 vi) {
        return 0x11004000u +
            ((static_cast<u32>(vi) * 16u) & 0xFFFu);
    };
    auto read_vector = [&](u32 reg, u32 qword) -> bool {
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            u32 value = 0;
            if (!bus_.read32(vu0_address(qword) + lane * 4u, value)) {
                return false;
            }
            lane_write(reg, lane, value);
        }
        return true;
    };
    auto write_vector = [&](u32 reg, u32 qword) -> bool {
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!selected(lane)) continue;
            if (!bus_.write32(
                    vu0_address(qword) + lane * 4u,
                    lane_read(reg, lane))) {
                return false;
            }
        }
        return true;
    };
    auto write_vi = [&](u32 reg, u32 value) {
        reg &= 0xFu;
        if (reg != 0u) state_.vu_vi[reg] = static_cast<u16>(value);
    };
    auto saturating_float_to_int = [](float value) -> s32 {
        if (std::isnan(value)) return 0;
        if (value >=
            static_cast<float>(std::numeric_limits<s32>::max())) {
            return std::numeric_limits<s32>::max();
        }
        if (value <=
            static_cast<float>(std::numeric_limits<s32>::min())) {
            return std::numeric_limits<s32>::min();
        }
        return static_cast<s32>(value);
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
    if (funct >= 0x08u && funct <= 0x0Bu) { // VMADDx/y/z/w
        broadcast_madd(funct & 3u, false);
        return true;
    }
    if (funct >= 0x0Cu && funct <= 0x0Fu) { // VMSUBx/y/z/w
        broadcast_madd(funct & 3u, true);
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
    case 0x1C: // VMULq
        scalar_control_binary(22u, mul);
        return true;
    case 0x1D: // VMAXi
        scalar_control_binary(21u, vmax);
        return true;
    case 0x1E: // VMULi
        scalar_control_binary(21u, mul);
        return true;
    case 0x1F: // VMINIi
        scalar_control_binary(21u, vmin);
        return true;
    case 0x20: // VADDq
        scalar_control_binary(22u, add);
        return true;
    case 0x21: // VMADDq
        control_madd(22u, false);
        return true;
    case 0x22: // VADDi
        scalar_control_binary(21u, add);
        return true;
    case 0x23: // VMADDi
        control_madd(21u, false);
        return true;
    case 0x24: // VSUBq
        scalar_control_binary(22u, sub);
        return true;
    case 0x25: // VMSUBq
        control_madd(22u, true);
        return true;
    case 0x26: // VSUBi
        scalar_control_binary(21u, sub);
        return true;
    case 0x27: // VMSUBi
        control_madd(21u, true);
        return true;
    case 0x28: // VADD
        vector_binary(add);
        return true;
    case 0x29: // VMADD
        vector_madd(false);
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
    case 0x2D: // VMSUB
        vector_madd(true);
        return true;
    case 0x2E: { // VOPMSUB
        const float result[3] = {
            acc_float(0u) -
                vu_float(fs, 1u) * vu_float(ft, 2u),
            acc_float(1u) -
                vu_float(fs, 2u) * vu_float(ft, 0u),
            acc_float(2u) -
                vu_float(fs, 0u) * vu_float(ft, 1u),
        };
        for (u32 lane = 0; lane < 3u; ++lane) {
            if (selected(lane)) {
                vu_write_float(fd, lane, result[lane]);
            }
        }
        return true;
    }
    case 0x2F: // VMINI
        vector_binary(vmin);
        return true;
    case 0x30: // VIADD
        write_vi(
            fd,
            static_cast<s16>(state_.vu_vi[fs & 0xFu]) +
            static_cast<s16>(state_.vu_vi[ft & 0xFu]));
        return true;
    case 0x31: // VISUB
        write_vi(
            fd,
            static_cast<s16>(state_.vu_vi[fs & 0xFu]) -
            static_cast<s16>(state_.vu_vi[ft & 0xFu]));
        return true;
    case 0x32: { // VIADDI
        s16 imm = static_cast<s16>((instruction >> 6) & 0x1Fu);
        if ((imm & 0x10) != 0) imm |= static_cast<s16>(0xFFF0);
        write_vi(
            ft,
            static_cast<s16>(state_.vu_vi[fs & 0xFu]) + imm);
        return true;
    }
    case 0x34: // VIAND
        write_vi(
            fd,
            state_.vu_vi[fs & 0xFu] & state_.vu_vi[ft & 0xFu]);
        return true;
    case 0x35: // VIOR
        write_vi(
            fd,
            state_.vu_vi[fs & 0xFu] | state_.vu_vi[ft & 0xFu]);
        return true;
    case 0x38: { // VCALLMS
        std::string vu_error;
        if (!run_vu0_micro((instruction >> 6) & 0x7FFFu, vu_error)) {
            return fail(
                pc,
                instruction,
                "VU0 VCALLMS failed: " + vu_error,
                error);
        }
        return true;
    }
    case 0x39: { // VCALLMSR
        std::string vu_error;
        if (!run_vu0_micro(state_.vu_vi[27] & 0xFFFFu, vu_error)) {
            return fail(
                pc,
                instruction,
                "VU0 VCALLMSR failed: " + vu_error,
                error);
        }
        return true;
    }
    default:
        break;
    }

    if (funct >= 0x3Cu) {
        const u32 special =
            (instruction & 3u) | ((instruction >> 4) & 0x7Cu);
        const u32 it = ft & 0xFu;
        const u32 is = fs & 0xFu;

        if (special <= 0x03u) { // VADDAx/y/z/w
            acc_broadcast_binary(special, add);
            return true;
        }
        if (special >= 0x04u && special <= 0x07u) { // VSUBAx/y/z/w
            acc_broadcast_binary(special & 3u, sub);
            return true;
        }
        if (special >= 0x08u && special <= 0x0Bu) { // VMADDAx/y/z/w
            acc_broadcast_madd(special & 3u, false);
            return true;
        }
        if (special >= 0x0Cu && special <= 0x0Fu) { // VMSUBAx/y/z/w
            acc_broadcast_madd(special & 3u, true);
            return true;
        }
        if (special >= 0x10u && special <= 0x13u) { // VITOF0/4/12/15
            static constexpr u32 shifts[4] = {0u, 4u, 12u, 15u};
            const u32 shift = shifts[special - 0x10u];
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (!selected(lane)) continue;
                const float value =
                    static_cast<float>(
                        static_cast<s32>(lane_read(fs, lane))) /
                    static_cast<float>(1u << shift);
                vu_write_float(ft, lane, value);
            }
            return true;
        }
        if (special >= 0x14u && special <= 0x17u) { // VFTOI0/4/12/15
            static constexpr u32 shifts[4] = {0u, 4u, 12u, 15u};
            const u32 shift = shifts[special - 0x14u];
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (!selected(lane)) continue;
                const float scaled =
                    vu_float(fs, lane) *
                    static_cast<float>(1u << shift);
                lane_write(
                    ft,
                    lane,
                    static_cast<u32>(
                        saturating_float_to_int(scaled)));
            }
            return true;
        }
        if (special >= 0x18u && special <= 0x1Bu) { // VMULAx/y/z/w
            acc_broadcast_binary(special & 3u, mul);
            return true;
        }
        if (special == 0x1Cu) { // VMULAq
            acc_control_binary(22u, mul);
            return true;
        }
        if (special == 0x1Du) { // VABS
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (selected(lane)) {
                    lane_write(
                        ft,
                        lane,
                        lane_read(fs, lane) & 0x7FFFFFFFu);
                }
            }
            return true;
        }
        if (special == 0x1Eu) { // VMULAi
            acc_control_binary(21u, mul);
            return true;
        }
        if (special == 0x1Fu) { // VCLIPw
            const float w = std::fabs(vu_float(ft, 3u));
            u32 clip = (state_.vu_vi[18] << 6) & 0xFFFFFFu;
            for (u32 lane = 0; lane < 3u; ++lane) {
                const float value = vu_float(fs, lane);
                if (value > w) clip |= 1u << (lane * 2u);
                if (value < -w) clip |= 1u << (lane * 2u + 1u);
            }
            state_.vu_vi[18] = clip;
            return true;
        }
        if (special == 0x20u) { // VADDAq
            acc_control_binary(22u, add);
            return true;
        }
        if (special == 0x21u) { // VMADDAq
            acc_control_madd(22u, false);
            return true;
        }
        if (special == 0x22u) { // VADDAi
            acc_control_binary(21u, add);
            return true;
        }
        if (special == 0x23u) { // VMADDAi
            acc_control_madd(21u, false);
            return true;
        }
        if (special == 0x24u) { // VSUBAq
            acc_control_binary(22u, sub);
            return true;
        }
        if (special == 0x25u) { // VMSUBAq
            acc_control_madd(22u, true);
            return true;
        }
        if (special == 0x26u) { // VSUBAi
            acc_control_binary(21u, sub);
            return true;
        }
        if (special == 0x27u) { // VMSUBAi
            acc_control_madd(21u, true);
            return true;
        }
        if (special == 0x28u) { // VADDA
            acc_vector_binary(add);
            return true;
        }
        if (special == 0x29u) { // VMADDA
            acc_vector_madd(false);
            return true;
        }
        if (special == 0x2Au) { // VMULA
            acc_vector_binary(mul);
            return true;
        }
        if (special == 0x2Cu) { // VSUBA
            acc_vector_binary(sub);
            return true;
        }
        if (special == 0x2Du) { // VMSUBA
            acc_vector_madd(true);
            return true;
        }
        if (special == 0x2Eu) { // VOPMULA
            const float value[3] = {
                vu_float(fs, 1u) * vu_float(ft, 2u),
                vu_float(fs, 2u) * vu_float(ft, 0u),
                vu_float(fs, 0u) * vu_float(ft, 1u),
            };
            for (u32 lane = 0; lane < 3u; ++lane) {
                if (selected(lane)) {
                    acc_write_float(lane, value[lane]);
                }
            }
            return true;
        }
        if (special == 0x2Fu) { // VNOP
            return true;
        }
        if (special == 0x30u) { // VMOVE
            const EeGpr source = state_.vu_vf[fs];
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (!selected(lane)) continue;
                const u64 half = lane < 2u ? source.lo : source.hi;
                lane_write(
                    ft,
                    lane,
                    static_cast<u32>(
                        half >> ((lane & 1u) * 32u)));
            }
            return true;
        }
        if (special == 0x31u) { // VMR32
            const EeGpr source = state_.vu_vf[fs];
            auto source_lane = [&](u32 lane) {
                const u32 rotated = (lane + 1u) & 3u;
                const u64 half =
                    rotated < 2u ? source.lo : source.hi;
                return static_cast<u32>(
                    half >> ((rotated & 1u) * 32u));
            };
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (selected(lane)) {
                    lane_write(ft, lane, source_lane(lane));
                }
            }
            return true;
        }
        if (special == 0x34u) { // VLQI
            const u16 address =
                static_cast<u16>(state_.vu_vi[is]);
            if (!read_vector(ft, address)) {
                return fail(
                    pc, instruction, "VLQI VU0 memory fault", error);
            }
            write_vi(is, static_cast<u16>(address + 1u));
            return true;
        }
        if (special == 0x35u) { // VSQI
            const u16 address =
                static_cast<u16>(state_.vu_vi[it]);
            if (!write_vector(fs, address)) {
                return fail(
                    pc, instruction, "VSQI VU0 memory fault", error);
            }
            write_vi(it, static_cast<u16>(address + 1u));
            return true;
        }
        if (special == 0x36u) { // VLQD
            const u16 address =
                static_cast<u16>(state_.vu_vi[is] - 1u);
            write_vi(is, address);
            if (!read_vector(ft, address)) {
                return fail(
                    pc, instruction, "VLQD VU0 memory fault", error);
            }
            return true;
        }
        if (special == 0x37u) { // VSQD
            const u16 address =
                static_cast<u16>(state_.vu_vi[it] - 1u);
            write_vi(it, address);
            if (!write_vector(fs, address)) {
                return fail(
                    pc, instruction, "VSQD VU0 memory fault", error);
            }
            return true;
        }
        if (special == 0x38u) { // VDIV
            const float numerator = vu_float(fs, fsf);
            const float denominator = vu_float(ft, ftf);
            if (denominator == 0.0f) {
                state_.vu_vi[22] = ps2_fpu_result(
                    std::copysign(
                        std::numeric_limits<float>::max(),
                        numerator * denominator));
            } else {
                state_.vu_vi[22] =
                    ps2_fpu_result(numerator / denominator);
            }
            return true;
        }
        if (special == 0x39u) { // VSQRT
            state_.vu_vi[22] = ps2_fpu_result(
                std::sqrt(std::fabs(vu_float(ft, ftf))));
            return true;
        }
        if (special == 0x3Au) { // VRSQRT
            const float numerator = vu_float(fs, fsf);
            const float denominator =
                std::sqrt(std::fabs(vu_float(ft, ftf)));
            if (denominator == 0.0f) {
                state_.vu_vi[22] = ps2_fpu_result(
                    std::copysign(
                        std::numeric_limits<float>::max(),
                        numerator));
            } else {
                state_.vu_vi[22] =
                    ps2_fpu_result(numerator / denominator);
            }
            return true;
        }
        if (special == 0x3Bu) { // VWAITQ
            return true;
        }
        if (special == 0x3Cu) { // VMTIR
            write_vi(it, lane_read(fs, fsf));
            return true;
        }
        if (special == 0x3Du) { // VMFIR
            const u32 value = static_cast<u32>(
                static_cast<s32>(
                    static_cast<s16>(state_.vu_vi[is])));
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (selected(lane)) lane_write(ft, lane, value);
            }
            return true;
        }
        if (special == 0x3Eu) { // VILWR
            const u32 base =
                vu0_address(state_.vu_vi[is]);
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (!selected(lane)) continue;
                u32 value = 0;
                if (!bus_.read32(base + lane * 4u, value)) {
                    return fail(
                        pc,
                        instruction,
                        "VILWR VU0 memory fault",
                        error);
                }
                write_vi(it, value);
            }
            return true;
        }
        if (special == 0x3Fu) { // VISWR
            const u32 base =
                vu0_address(state_.vu_vi[is]);
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (selected(lane) &&
                    !bus_.write32(
                        base + lane * 4u,
                        state_.vu_vi[it] & 0xFFFFu)) {
                    return fail(
                        pc,
                        instruction,
                        "VISWR VU0 memory fault",
                        error);
                }
            }
            return true;
        }
        if (special == 0x40u) { // VRNEXT
            const u32 x = (state_.vu_vi[20] >> 4) & 1u;
            const u32 y = (state_.vu_vi[20] >> 22) & 1u;
            state_.vu_vi[20] <<= 1u;
            state_.vu_vi[20] ^= x ^ y;
            state_.vu_vi[20] =
                (state_.vu_vi[20] & 0x007FFFFFu) | 0x3F800000u;
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (selected(lane)) {
                    lane_write(ft, lane, state_.vu_vi[20]);
                }
            }
            return true;
        }
        if (special == 0x41u) { // VRGET
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (selected(lane)) {
                    lane_write(ft, lane, state_.vu_vi[20]);
                }
            }
            return true;
        }
        if (special == 0x42u) { // VRINIT
            state_.vu_vi[20] =
                0x3F800000u |
                (lane_read(fs, fsf) & 0x007FFFFFu);
            return true;
        }
        if (special == 0x43u) { // VRXOR
            state_.vu_vi[20] =
                0x3F800000u |
                ((state_.vu_vi[20] ^ lane_read(fs, fsf)) &
                 0x007FFFFFu);
            return true;
        }

        return fail(
            pc,
            instruction,
            "Unsupported COP2 SPECIAL2 function " +
                hex32(special),
            error);
    }

    return fail(
        pc,
        instruction,
        "Unsupported COP2 macro function " + hex32(funct),
        error);
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
    auto get_acc_slot = [&](u32 lane) -> u32 {
        const u64 value =
            lane < 2u ? state_.lo :
            lane < 4u ? state_.hi :
            lane < 6u ? state_.lo1 :
                        state_.hi1;
        return static_cast<u32>(
            value >> ((lane & 1u) * 32u));
    };
    auto set_acc_slot = [&](u32 lane, u32 value) {
        u64* target =
            lane < 2u ? &state_.lo :
            lane < 4u ? &state_.hi :
            lane < 6u ? &state_.lo1 :
                        &state_.hi1;
        const u32 shift = (lane & 1u) * 32u;
        *target =
            (*target & ~(0xFFFFFFFFull << shift)) |
            (static_cast<u64>(value) << shift);
    };
    auto get_lo_word = [&](u32 lane) -> u32 {
        const u64 value = lane < 2u ? state_.lo : state_.lo1;
        return static_cast<u32>(
            value >> ((lane & 1u) * 32u));
    };
    auto set_lo_word = [&](u32 lane, u32 value) {
        u64& target = lane < 2u ? state_.lo : state_.lo1;
        const u32 shift = (lane & 1u) * 32u;
        target =
            (target & ~(0xFFFFFFFFull << shift)) |
            (static_cast<u64>(value) << shift);
    };
    auto get_hi_word = [&](u32 lane) -> u32 {
        const u64 value = lane < 2u ? state_.hi : state_.hi1;
        return static_cast<u32>(
            value >> ((lane & 1u) * 32u));
    };
    auto set_hi_word = [&](u32 lane, u32 value) {
        u64& target = lane < 2u ? state_.hi : state_.hi1;
        const u32 shift = (lane & 1u) * 32u;
        target =
            (target & ~(0xFFFFFFFFull << shift)) |
            (static_cast<u64>(value) << shift);
    };
    auto finish_half_acc = [&](EeGpr& out) {
        set32(out, 0u, get_acc_slot(0u));
        set32(out, 1u, get_acc_slot(2u));
        set32(out, 2u, get_acc_slot(4u));
        set32(out, 3u, get_acc_slot(6u));
    };
    auto packed_word_accumulate = [&](
        bool subtract_product,
        bool unsigned_product,
        EeGpr& out) {
        for (u32 lane = 0; lane < 2u; ++lane) {
            const u32 word = lane * 2u;
            u64 accumulator =
                static_cast<u64>(
                    lane == 0u
                        ? static_cast<u32>(state_.lo)
                        : static_cast<u32>(state_.lo1)) |
                (static_cast<u64>(
                    lane == 0u
                        ? static_cast<u32>(state_.hi)
                        : static_cast<u32>(state_.hi1)) << 32);
            u64 product = 0;
            if (unsigned_product) {
                product =
                    static_cast<u64>(get32(a, word)) *
                    static_cast<u64>(get32(b, word));
            } else {
                product = static_cast<u64>(
                    static_cast<s64>(
                        static_cast<s32>(get32(a, word))) *
                    static_cast<s64>(
                        static_cast<s32>(get32(b, word))));
            }
            const u64 result =
                subtract_product
                    ? accumulator - product
                    : accumulator + product;
            u64& lo = lane == 0u ? state_.lo : state_.lo1;
            u64& hi = lane == 0u ? state_.hi : state_.hi1;
            lo = sign_extend_32(static_cast<u32>(result));
            hi = sign_extend_32(
                static_cast<u32>(result >> 32));
            if (lane == 0u) out.lo = result;
            else out.hi = result;
        }
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

        case 0x10: // PADDSW
        case 0x11: // PSUBSW
            for (u32 i = 0; i < 4u; ++i) {
                const s64 av = static_cast<s32>(get32(a, i));
                const s64 bv = static_cast<s32>(get32(b, i));
                const s64 result =
                    sub == 0x10u ? av + bv : av - bv;
                const s64 clamped =
                    result > std::numeric_limits<s32>::max()
                        ? std::numeric_limits<s32>::max()
                        : result < std::numeric_limits<s32>::min()
                            ? std::numeric_limits<s32>::min()
                            : result;
                set32(out, i, static_cast<u32>(
                    static_cast<s32>(clamped)));
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
        case 0x14: // PADDSH
        case 0x15: // PSUBSH
            for (u32 i = 0; i < 8u; ++i) {
                const s32 av = static_cast<s16>(get16(a, i));
                const s32 bv = static_cast<s16>(get16(b, i));
                const s32 result =
                    sub == 0x14u ? av + bv : av - bv;
                const s32 clamped =
                    result > std::numeric_limits<s16>::max()
                        ? std::numeric_limits<s16>::max()
                        : result < std::numeric_limits<s16>::min()
                            ? std::numeric_limits<s16>::min()
                            : result;
                set16(out, i, static_cast<u16>(
                    static_cast<s16>(clamped)));
            }
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
        case 0x18: // PADDSB
        case 0x19: // PSUBSB
            for (u32 i = 0; i < 16u; ++i) {
                const s16 av = static_cast<s8>(get8(a, i));
                const s16 bv = static_cast<s8>(get8(b, i));
                const s16 result =
                    sub == 0x18u ? av + bv : av - bv;
                const s16 clamped =
                    result > std::numeric_limits<s8>::max()
                        ? std::numeric_limits<s8>::max()
                        : result < std::numeric_limits<s8>::min()
                            ? std::numeric_limits<s8>::min()
                            : result;
                set8(out, i, static_cast<u8>(
                    static_cast<s8>(clamped)));
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
        case 0x04: // PADSBH
            for (u32 i = 0; i < 8u; ++i) {
                const u16 av = get16(a, i);
                const u16 bv = get16(b, i);
                set16(
                    out,
                    i,
                    i < 4u
                        ? static_cast<u16>(av - bv)
                        : static_cast<u16>(av + bv));
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
        case 0x00: // PMADDW
            packed_word_accumulate(false, false, out);
            store(out);
            return true;
        case 0x04: // PMSUBW
            packed_word_accumulate(true, false, out);
            store(out);
            return true;
        case 0x02: // PSLLVW
            out.lo = sign_extend_32(
                get32(b, 0u) << (get32(a, 0u) & 31u));
            out.hi = sign_extend_32(
                get32(b, 2u) << (get32(a, 2u) & 31u));
            store(out);
            return true;
        case 0x03: // PSRLVW
            out.lo = sign_extend_32(
                get32(b, 0u) >> (get32(a, 0u) & 31u));
            out.hi = sign_extend_32(
                get32(b, 2u) >> (get32(a, 2u) & 31u));
            store(out);
            return true;
        case 0x08: // PMFHI
            out.lo = state_.hi;
            out.hi = state_.hi1;
            store(out);
            return true;
        case 0x09: // PMFLO
            out.lo = state_.lo;
            out.hi = state_.lo1;
            store(out);
            return true;
        case 0x0A: // PINTH
            for (u32 i = 0; i < 4u; ++i) {
                set16(out, i * 2u, get16(b, i));
                set16(out, i * 2u + 1u, get16(a, i + 4u));
            }
            store(out);
            return true;
        case 0x0C: { // PMULTW
            const s64 p0 =
                static_cast<s64>(static_cast<s32>(get32(a, 0u))) *
                static_cast<s64>(static_cast<s32>(get32(b, 0u)));
            const s64 p1 =
                static_cast<s64>(static_cast<s32>(get32(a, 2u))) *
                static_cast<s64>(static_cast<s32>(get32(b, 2u)));
            multiply_signed32(
                get32(a, 0u), get32(b, 0u),
                state_.lo, state_.hi);
            multiply_signed32(
                get32(a, 2u), get32(b, 2u),
                state_.lo1, state_.hi1);
            out.lo = static_cast<u64>(p0);
            out.hi = static_cast<u64>(p1);
            store(out);
            return true;
        }
        case 0x0D: // PDIVW
            divide_signed32(
                get32(a, 0u), get32(b, 0u),
                state_.lo, state_.hi);
            divide_signed32(
                get32(a, 2u), get32(b, 2u),
                state_.lo1, state_.hi1);
            return true;
        case 0x0E: // PCPYLD
            out.lo = b.lo;
            out.hi = a.lo;
            store(out);
            return true;
        case 0x10: // PMADDH
        case 0x14: // PMSUBH
            for (u32 i = 0; i < 8u; ++i) {
                const s32 product =
                    static_cast<s32>(
                        static_cast<s16>(get16(a, i))) *
                    static_cast<s32>(
                        static_cast<s16>(get16(b, i)));
                const u32 old = get_acc_slot(i);
                const u32 value =
                    sub == 0x10u
                        ? old + static_cast<u32>(product)
                        : old - static_cast<u32>(product);
                set_acc_slot(i, value);
            }
            finish_half_acc(out);
            store(out);
            return true;
        case 0x11: // PHMADH
        case 0x15: // PHMSBH
            for (u32 pair = 0; pair < 4u; ++pair) {
                const u32 even = pair * 2u;
                const u32 odd = even + 1u;
                const s32 first =
                    static_cast<s32>(
                        static_cast<s16>(get16(a, odd))) *
                    static_cast<s32>(
                        static_cast<s16>(get16(b, odd)));
                const s32 second =
                    static_cast<s32>(
                        static_cast<s16>(get16(a, even))) *
                    static_cast<s32>(
                        static_cast<s16>(get16(b, even)));
                set_acc_slot(
                    even,
                    static_cast<u32>(
                        sub == 0x11u
                            ? first + second
                            : first - second));
                set_acc_slot(
                    odd,
                    sub == 0x11u
                        ? static_cast<u32>(first)
                        : ~static_cast<u32>(first));
            }
            finish_half_acc(out);
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
        case 0x1C: // PMULTH
            for (u32 i = 0; i < 8u; ++i) {
                const s32 product =
                    static_cast<s32>(
                        static_cast<s16>(get16(a, i))) *
                    static_cast<s32>(
                        static_cast<s16>(get16(b, i)));
                set_acc_slot(i, static_cast<u32>(product));
            }
            finish_half_acc(out);
            store(out);
            return true;
        case 0x1D: { // PDIVBW
            const s16 divisor =
                static_cast<s16>(get16(b, 0u));
            for (u32 i = 0; i < 4u; ++i) {
                const s32 dividend =
                    static_cast<s32>(get32(a, i));
                s32 quotient = 0;
                s32 remainder = 0;
                if (dividend == std::numeric_limits<s32>::min() &&
                    divisor == -1) {
                    quotient = std::numeric_limits<s32>::min();
                } else if (divisor != 0) {
                    quotient = dividend / divisor;
                    remainder = dividend % divisor;
                } else {
                    quotient = dividend < 0 ? 1 : -1;
                    remainder = dividend;
                }
                set_lo_word(i, static_cast<u32>(quotient));
                set_hi_word(i, static_cast<u32>(remainder));
            }
            return true;
        }
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
        case 0x00: // PMADDUW
            packed_word_accumulate(false, true, out);
            store(out);
            return true;
        case 0x03: // PSRAVW
            out.lo = sign_extend_32(static_cast<u32>(
                static_cast<s32>(get32(b, 0u)) >>
                (get32(a, 0u) & 31u)));
            out.hi = sign_extend_32(static_cast<u32>(
                static_cast<s32>(get32(b, 2u)) >>
                (get32(a, 2u) & 31u)));
            store(out);
            return true;
        case 0x08: // PMTHI
            state_.hi = a.lo;
            state_.hi1 = a.hi;
            return true;
        case 0x09: // PMTLO
            state_.lo = a.lo;
            state_.lo1 = a.hi;
            return true;
        case 0x0A: // PINTEH
            for (u32 i = 0; i < 4u; ++i) {
                set16(out, i * 2u, get16(b, i * 2u));
                set16(out, i * 2u + 1u, get16(a, i * 2u));
            }
            store(out);
            return true;
        case 0x0C: { // PMULTUW
            const u64 p0 =
                static_cast<u64>(get32(a, 0u)) *
                static_cast<u64>(get32(b, 0u));
            const u64 p1 =
                static_cast<u64>(get32(a, 2u)) *
                static_cast<u64>(get32(b, 2u));
            multiply_unsigned32(
                get32(a, 0u), get32(b, 0u),
                state_.lo, state_.hi);
            multiply_unsigned32(
                get32(a, 2u), get32(b, 2u),
                state_.lo1, state_.hi1);
            out.lo = p0;
            out.hi = p1;
            store(out);
            return true;
        }
        case 0x0D: // PDIVUW
            divide_unsigned32(
                get32(a, 0u), get32(b, 0u),
                state_.lo, state_.hi);
            divide_unsigned32(
                get32(a, 2u), get32(b, 2u),
                state_.lo1, state_.hi1);
            return true;
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
    case 0x00: // MADD
        madd_signed32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo,
            state_.hi);
        write_gpr64(rd, state_.lo);
        return true;
    case 0x01: // MADDU
        madd_unsigned32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo,
            state_.hi);
        write_gpr64(rd, state_.lo);
        return true;
    case 0x04: { // PLZCW
        if (rd != 0u) {
            EeGpr out = state_.gpr[rd];
            const u32 word0 =
                leading_sign_bits_excluding_sign(
                    static_cast<u32>(gpr_u64(rs)));
            const u32 word1 =
                leading_sign_bits_excluding_sign(
                    static_cast<u32>(gpr_u64(rs) >> 32));
            out.lo =
                static_cast<u64>(word0) |
                (static_cast<u64>(word1) << 32);
            state_.gpr[rd] = out;
        }
        return true;
    }
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
    case 0x20: // MADD1
        madd_signed32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo1,
            state_.hi1);
        write_gpr64(rd, state_.lo1);
        return true;
    case 0x21: // MADDU1
        madd_unsigned32(
            static_cast<u32>(gpr_u64(rs)),
            static_cast<u32>(gpr_u64(rt)),
            state_.lo1,
            state_.hi1);
        write_gpr64(rd, state_.lo1);
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
    case 0x30: { // PMFHL
        if (rd == 0u) return true;
        EeGpr out{};

        const auto low32 = [](u64 value) {
            return static_cast<u32>(value);
        };
        const auto high32 = [](u64 value) {
            return static_cast<u32>(value >> 32);
        };
        const auto clamp_s32 = [](s64 value) -> u64 {
            if (value > std::numeric_limits<s32>::max()) {
                return sign_extend_32(0x7FFFFFFFu);
            }
            if (value < std::numeric_limits<s32>::min()) {
                return sign_extend_32(0x80000000u);
            }
            return sign_extend_32(
                static_cast<u32>(static_cast<s32>(value)));
        };
        const auto clamp_s16 = [](u32 value) -> u16 {
            const s32 signed_value = static_cast<s32>(value);
            if (signed_value > std::numeric_limits<s16>::max()) {
                return 0x7FFFu;
            }
            if (signed_value < std::numeric_limits<s16>::min()) {
                return 0x8000u;
            }
            return static_cast<u16>(
                static_cast<s16>(signed_value));
        };

        switch (sa) {
        case 0x00: // LW
            set32(out, 0u, low32(state_.lo));
            set32(out, 1u, low32(state_.hi));
            set32(out, 2u, low32(state_.lo1));
            set32(out, 3u, low32(state_.hi1));
            break;
        case 0x01: // UW
            set32(out, 0u, high32(state_.lo));
            set32(out, 1u, high32(state_.hi));
            set32(out, 2u, high32(state_.lo1));
            set32(out, 3u, high32(state_.hi1));
            break;
        case 0x02: { // SLW
            const s64 primary = static_cast<s64>(
                static_cast<u64>(low32(state_.lo)) |
                (static_cast<u64>(low32(state_.hi)) << 32));
            const s64 secondary = static_cast<s64>(
                static_cast<u64>(low32(state_.lo1)) |
                (static_cast<u64>(low32(state_.hi1)) << 32));
            out.lo = clamp_s32(primary);
            out.hi = clamp_s32(secondary);
            break;
        }
        case 0x03: // LH
            set16(out, 0u, static_cast<u16>(state_.lo));
            set16(out, 1u, static_cast<u16>(state_.lo >> 32));
            set16(out, 2u, static_cast<u16>(state_.hi));
            set16(out, 3u, static_cast<u16>(state_.hi >> 32));
            set16(out, 4u, static_cast<u16>(state_.lo1));
            set16(out, 5u, static_cast<u16>(state_.lo1 >> 32));
            set16(out, 6u, static_cast<u16>(state_.hi1));
            set16(out, 7u, static_cast<u16>(state_.hi1 >> 32));
            break;
        case 0x04: // SH
            set16(out, 0u, clamp_s16(low32(state_.lo)));
            set16(out, 1u, clamp_s16(high32(state_.lo)));
            set16(out, 2u, clamp_s16(low32(state_.hi)));
            set16(out, 3u, clamp_s16(high32(state_.hi)));
            set16(out, 4u, clamp_s16(low32(state_.lo1)));
            set16(out, 5u, clamp_s16(high32(state_.lo1)));
            set16(out, 6u, clamp_s16(low32(state_.hi1)));
            set16(out, 7u, clamp_s16(high32(state_.hi1)));
            break;
        default:
            return true;
        }

        state_.gpr[rd] = out;
        return true;
    }
    case 0x31: // PMTHL
        if (sa == 0u) {
            const auto replace_low32 = [](u64 original, u32 value) {
                return (original & 0xFFFFFFFF00000000ull) |
                       static_cast<u64>(value);
            };
            state_.lo =
                replace_low32(state_.lo, get32(a, 0u));
            state_.hi =
                replace_low32(state_.hi, get32(a, 1u));
            state_.lo1 =
                replace_low32(state_.lo1, get32(a, 2u));
            state_.hi1 =
                replace_low32(state_.hi1, get32(a, 3u));
        }
        return true;
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

bool EeCpu::skip_bios_idle_iteration() {
    return skip_bios_idle_iterations(1u);
}

bool EeCpu::skip_bios_idle_iterations(u32 iterations) {
    if (iterations == 0u || iterations > 0x1FFFFFFFu) {
        return false;
    }
    return skip_bios_idle_instructions(iterations * 8u);
}

bool EeCpu::skip_bios_idle_instructions(u32 instructions) {
    constexpr u32 kIdlePc = 0x00081FC0u;
    constexpr std::array<u32, 8> kIdleCode = {
        0u, 0u, 0u, 0u, 0u, 0u, 0x1000FFF9u, 0u};

    if (instructions == 0u || halted_) return false;
    if (!bus_.matches_code(kIdlePc, kIdleCode)) return false;

    u32 phase = 0u;
    if (state_.pc >= kIdlePc &&
        state_.pc <= kIdlePc + 0x18u &&
        ((state_.pc - kIdlePc) & 3u) == 0u &&
        state_.next_pc == state_.pc + 4u &&
        !next_is_delay_slot_) {
        phase = (state_.pc - kIdlePc) >> 2u;
    } else if (
        state_.pc == kIdlePc + 0x1Cu &&
        state_.next_pc == kIdlePc &&
        next_is_delay_slot_) {
        phase = 7u;
    } else {
        return false;
    }

    // COP0 Count is advanced by every retired instruction. Do not cross a
    // Compare match: that interrupt must be observed at its exact cycle.
    const u32 count = state_.cop0[9];
    const u32 distance_to_compare = state_.cop0[11] - count;
    if (distance_to_compare != 0u &&
        distance_to_compare <= instructions) {
        return false;
    }

    const u32 final_phase =
        static_cast<u32>(
            (static_cast<u64>(phase) + instructions) & 7u);
    const u32 last_phase =
        static_cast<u32>(
            (static_cast<u64>(phase) + instructions - 1u) & 7u);

    auto phase_pc = [](u32 value) {
        return kIdlePc + value * 4u;
    };

    state_.cop0[13] &= ~0x00000C00u;
    state_.cop0[9] += instructions;
    state_.instructions_executed += instructions;
    state_.last_pc = phase_pc(last_phase);
    state_.last_instruction =
        last_phase == 6u ? kIdleCode[6] : 0u;
    current_is_delay_slot_ = last_phase == 7u;

    if (final_phase == 7u) {
        state_.pc = kIdlePc + 0x1Cu;
        state_.next_pc = kIdlePc;
        next_is_delay_slot_ = true;
    } else {
        state_.pc = phase_pc(final_phase);
        state_.next_pc = state_.pc + 4u;
        next_is_delay_slot_ = false;
    }

    state_.gpr[0] = {};
    bus_.tick(instructions);
    return true;
}

bool EeCpu::skip_bios_zero_loop(u32 iterations) {
    constexpr u32 kLoopPc = 0x8000E3C8u;
    constexpr std::array<u32, 7> kLoopCode = {
        0x7E020000u, 0x26100010u, 0x0204102Bu, 0u, 0u,
        0x1440FFFAu, 0x700014A9u};
    if (iterations == 0u || halted_ || state_.pc != kLoopPc ||
        state_.next_pc != kLoopPc + 4u || next_is_delay_slot_ ||
        state_.gpr[2].lo != 0u || state_.gpr[2].hi != 0u) {
        return false;
    }
    if (!bus_.matches_code(kLoopPc, kLoopCode)) return false;

    const u64 address = state_.gpr[16].lo;
    const u64 end = address + 16ull * iterations;
    if (address >= 0x02000000u || end >= state_.gpr[4].lo ||
        end > 0x02000000u || (address & 15u) != 0u) return false;
    const u32 cycles = 7u * iterations;
    const u32 distance = state_.cop0[11] - state_.cop0[9];
    if (distance != 0u && distance <= cycles) return false;
    if (!bus_.fill_ram_zero(static_cast<u32>(address), 16u * iterations))
        return false;

    state_.gpr[16].lo = end;
    state_.gpr[2] = {};
    state_.gpr[0] = {};
    state_.last_pc = kLoopPc + 24u;
    state_.last_instruction = kLoopCode[6];
    state_.cop0[13] &= ~0x00000C00u;
    state_.cop0[9] += cycles;
    state_.instructions_executed += cycles;
    current_is_delay_slot_ = true;
    next_is_delay_slot_ = false;
    bus_.tick(cycles);
    return true;
}

bool EeCpu::skip_bios_nibble_loop(u32 iterations) {
    constexpr u32 kLoopPc = 0x0020A0E8u;
    constexpr std::array<u32, 9> kLoopCode = {
        0x90A20000u, 0x24C6FFFFu, 0x3043000Fu,
        0x00021102u, 0x00031900u, 0x00431021u,
        0xA0A20000u, 0x04C1FFF8u, 0x24A50001u};
    if (iterations == 0u || halted_ || state_.pc != kLoopPc ||
        state_.next_pc != kLoopPc + 4u || next_is_delay_slot_) {
        return false;
    }
    if (!bus_.matches_code(kLoopPc, kLoopCode)) return false;

    const u64 address = state_.gpr[5].lo;
    const u64 remaining = state_.gpr[6].lo;
    if (remaining < iterations || address >= 0x02000000u ||
        address + iterations > 0x02000000u) return false;
    const u32 cycles = 9u * iterations;
    const u32 distance = state_.cop0[11] - state_.cop0[9];
    if (distance != 0u && distance <= cycles) return false;
    u8 last_original = 0;
    if (!bus_.nibble_swap_ram(static_cast<u32>(address), iterations,
                              last_original)) return false;

    const u32 low_nibble = last_original & 0x0Fu;
    state_.gpr[2].lo = static_cast<u8>((last_original >> 4) |
                                        (last_original << 4));
    state_.gpr[3].lo = low_nibble << 4;
    state_.gpr[5].lo = static_cast<u32>(address + iterations);
    state_.gpr[6].lo = static_cast<u32>(remaining - iterations);
    state_.gpr[0] = {};
    state_.last_pc = kLoopPc + 32u;
    state_.last_instruction = kLoopCode[8];
    state_.cop0[13] &= ~0x00000C00u;
    state_.cop0[9] += cycles;
    state_.instructions_executed += cycles;
    current_is_delay_slot_ = true;
    next_is_delay_slot_ = false;
    bus_.tick(cycles);
    return true;
}

u32 EeCpu::skip_bios_count_wait(u32 max_iterations) {
    constexpr u32 kLoopPc = 0x9FC42930u;
    constexpr std::array<u32, 7> kLoopCode = {
        0x40024800u, 0x00431023u, 0x0044102Bu,
        0u, 0u, 0x1440FFFAu, 0u};
    if (max_iterations == 0u || halted_ || state_.pc != kLoopPc ||
        state_.next_pc != kLoopPc + 4u || next_is_delay_slot_) return 0;
    if (!bus_.matches_code(kLoopPc, kLoopCode)) return 0;

    const u32 count = state_.cop0[9];
    const u32 base = static_cast<u32>(state_.gpr[3].lo);
    const u64 target = state_.gpr[4].lo;
    u32 iterations = 0;
    while (iterations < max_iterations) {
        const u32 current_count = count + 7u * iterations;
        const u32 difference = current_count - base;
        const u64 signed_difference = static_cast<u64>(
            static_cast<s64>(static_cast<s32>(difference)));
        if (signed_difference >= target) break;
        ++iterations;
    }
    if (iterations == 0u) return 0;
    const u32 cycles = 7u * iterations;
    const u32 distance = state_.cop0[11] - count;
    if (distance != 0u && distance <= cycles) return 0;

    state_.gpr[2].lo = 1u;
    state_.gpr[0] = {};
    state_.last_pc = kLoopPc + 24u;
    state_.last_instruction = 0u;
    state_.cop0[13] &= ~0x00000C00u;
    state_.cop0[9] += cycles;
    state_.instructions_executed += cycles;
    current_is_delay_slot_ = true;
    next_is_delay_slot_ = false;
    bus_.tick(cycles);
    return iterations;
}

u32 EeCpu::skip_bios_countdown_wait(u32 max_iterations) {
    const u32 pc = state_.pc;
    const bool first_loop = pc == 0x000826B0u;
    if ((!first_loop && pc != 0x00252758u &&
         pc != 0x00252DE8u) ||
        max_iterations == 0u || halted_ ||
        state_.next_pc != pc + 4u || next_is_delay_slot_) return 0;
    const std::array<u32, 7> code = {
        0u, 0u, 0u, 0u, 0u,
        first_loop ? 0x1443FFFAu : 0x1440FFFAu,
        0x2442FFFFu};
    if (!bus_.matches_code(pc, code)) return 0;

    const u32 initial = static_cast<u32>(state_.gpr[2].lo);
    const u32 target = first_loop ?
        static_cast<u32>(state_.gpr[3].lo) : 0u;
    const u32 distance_to_exit = initial - target;
    const u32 iterations = std::min(max_iterations, distance_to_exit);
    if (iterations == 0u) return 0;
    const u32 cycles = iterations * 7u;
    const u32 distance_to_compare = state_.cop0[11] - state_.cop0[9];
    if (distance_to_compare != 0u &&
        distance_to_compare <= cycles) return 0;

    state_.gpr[2].lo = sign_extend_word(initial - iterations);
    state_.gpr[0] = {};
    state_.last_pc = pc + 24u;
    state_.last_instruction = code[6];
    state_.cop0[13] &= ~0x00000C00u;
    state_.cop0[9] += cycles;
    state_.instructions_executed += cycles;
    current_is_delay_slot_ = true;
    next_is_delay_slot_ = false;
    bus_.tick(cycles);
    return iterations;
}

bool EeCpu::skip_bios_copy_iteration() {
    return skip_bios_copy_iterations(1u);
}

bool EeCpu::skip_bios_copy_iterations(u32 iterations) {
    const u32 pc = state_.pc;
    constexpr std::array<u32, 7> kLoopCode = {
        0x90A20000u, 0x2484FFFFu, 0x24A50001u,
        0xA2020000u, 0x26100001u, 0x1480FFFAu, 0u};
    if (iterations == 0u || halted_ ||
        (pc != 0x00200DE8u && pc != 0x00100BE0u) ||
        state_.next_pc != pc + 4u || next_is_delay_slot_ ||
        state_.gpr[4].lo < iterations ||
        state_.gpr[4].lo > 0x7FFFFFFFu) return false;
    if (!bus_.matches_code(pc, kLoopCode)) return false;
    const u32 distance = state_.cop0[11] - state_.cop0[9];
    const u32 cycles = 7u * iterations;
    if (distance != 0u && distance <= cycles) return false;

    if (state_.gpr[5].lo >= 0x02000000u ||
        state_.gpr[16].lo >= 0x02000000u ||
        state_.gpr[5].lo + iterations > 0x02000000u ||
        state_.gpr[16].lo + iterations > 0x02000000u) return false;
    const u32 src = static_cast<u32>(state_.gpr[5].lo);
    const u32 dst = static_cast<u32>(state_.gpr[16].lo);
    if (dst < pc + 28u && dst + iterations > pc) return false;
    u8 value = 0;
    if (!bus_.copy_ram_forward(dst, src, iterations, value)) return false;

    state_.gpr[2].lo = value;
    const bool final_iteration = state_.gpr[4].lo == iterations;
    state_.gpr[4].lo = sign_extend_word(
        static_cast<u32>(state_.gpr[4].lo) - iterations);
    state_.gpr[5].lo = sign_extend_word(src + iterations);
    state_.gpr[16].lo = sign_extend_word(dst + iterations);
    state_.gpr[0] = {};
    state_.last_pc = pc + 24u;
    state_.last_instruction = 0u;
    if (final_iteration) {
        state_.pc = pc + 28u;
        state_.next_pc = pc + 32u;
    }
    state_.cop0[13] &= ~0x00000C00u;
    state_.cop0[9] += cycles;
    state_.instructions_executed += cycles;
    current_is_delay_slot_ = true;
    next_is_delay_slot_ = false;
    bus_.tick(cycles);
    return true;
}

bool EeCpu::skip_bios_mmio_poll_iteration() {
    const u32 pc = state_.pc;
    const bool intc_poll = pc == 0x8000DAD0u;
    const bool bios_intc_poll = pc == 0x00266118u;
    if ((!intc_poll && !bios_intc_poll && pc != 0x00082180u) || halted_ ||
        state_.next_pc != pc + 4u || next_is_delay_slot_) return false;
    const std::array<u32, 7> code = {
        intc_poll ? 0x8C820000u : 0x8C620000u,
        intc_poll || bios_intc_poll ? 0x30420004u : 0x00441024u,
        0u, 0u, 0u, 0x1040FFFAu,
        bios_intc_poll ? 0x24020004u : 0x3C021000u};
    if (!bus_.matches_code(pc, code)) return false;
    const u32 address_reg = intc_poll ? 4u : 3u;
    const u32 address = intc_poll || bios_intc_poll ?
        0x1000F000u : 0x1000F230u;
    if (state_.gpr[address_reg].lo != address ||
        (!intc_poll && !bios_intc_poll &&
         state_.gpr[4].lo != 0x00040000u)) return false;
    u32 value = 0;
    if (!bus_.read32(address, value) ||
        (value & (intc_poll || bios_intc_poll ?
                  4u : 0x00040000u)) != 0u) return false;
    const u32 distance = state_.cop0[11] - state_.cop0[9];
    if (distance != 0u && distance <= 7u) return false;

    state_.gpr[2].lo = bios_intc_poll ? 4u : 0x10000000u;
    state_.gpr[0] = {};
    state_.last_pc = pc + 24u;
    state_.last_instruction = code[6];
    state_.cop0[13] &= ~0x00000C00u;
    state_.cop0[9] += 7u;
    state_.instructions_executed += 7u;
    current_is_delay_slot_ = true;
    next_is_delay_slot_ = false;
    bus_.tick(7u);
    return true;
}

u32 EeCpu::skip_bios_mmio_poll_iterations(u32 max_iterations) {
    if (max_iterations == 0u || !skip_bios_mmio_poll_iteration())
        return 0u;
    // The caller permits more than one iteration only while the polled
    // interrupt source and code cannot change. Every iteration returns to
    // the same PC with the same register result and branch-delay state.
    if (max_iterations > 1u) {
        const u32 extra_cycles = (max_iterations - 1u) * 7u;
        state_.cop0[9] += extra_cycles;
        state_.instructions_executed += extra_cycles;
        bus_.tick(extra_cycles);
    }
    return max_iterations;
}

bool EeCpu::skip_bios_literal_iteration() {
    return skip_bios_literal_iteration_impl(true);
}

bool EeCpu::skip_bios_literal_iteration_impl(bool verify_code) {
    constexpr u32 kPc = 0x00200D70u;
    constexpr std::array<u32, 2> kEntry = {
        0x16200004u, 0x268781C8u};
    constexpr std::array<u32, 7> kInput = {
        0x8CE50014u, 0x8CE20004u, 0x90A60000u,
        0x24A50001u, 0x00551024u, 0x1040001Cu,
        0xACE50014u};
    constexpr std::array<u32, 13> kOutput = {
        0xA2060000u, 0x26100001u, 0x8E6381C8u,
        0x02121023u, 0x10430008u, 0x266481C8u,
        0x0062102Bu, 0x14400005u, 0x2631FFFFu,
        0x8C820004u, 0x00021040u, 0x1000FFCDu,
        0xAC820004u};
    if (halted_ || state_.pc != kPc ||
        state_.next_pc != kPc + 4u || next_is_delay_slot_ ||
        state_.gpr[17].lo == 0u ||
        (verify_code &&
         (!bus_.matches_code(kPc, kEntry) ||
          !bus_.matches_code(0x00200D84u, kInput) ||
          !bus_.matches_code(0x00200E0Cu, kOutput)))) return false;
    const u32 distance = state_.cop0[11] - state_.cop0[9];
    if (distance != 0u && distance <= 22u) return false;

    const u32 context = static_cast<u32>(state_.gpr[20].lo) - 32312u;
    const u32 input_slot = context + 20u;
    const u32 mask_slot = context + 4u;
    const u32 output = static_cast<u32>(state_.gpr[16].lo);
    const u32 output_next = output + 1u;
    const u32 output_context =
        static_cast<u32>(state_.gpr[19].lo) - 32312u;
    if (context >= 0x02000000u - 24u ||
        output >= 0x02000000u ||
        output_context >= 0x02000000u - 8u) return false;

    u32 cursor = 0, mask_word = 0;
    if (!bus_.read32(input_slot, cursor) ||
        !bus_.read32(mask_slot, mask_word) ||
        cursor >= 0x02000000u) return false;
    u8 byte = 0;
    if (!bus_.read8(cursor, byte)) return false;
    const u64 masked = sign_extend_word(mask_word) &
                       state_.gpr[21].lo;
    if (masked != 0u) return false;

    u32 output_count = 0;
    if (!bus_.read32(output_context, output_count)) return false;
    const u64 count_value = sign_extend_word(output_count);
    const u64 produced = sign_extend_word(
        output_next - static_cast<u32>(state_.gpr[18].lo));
    if (produced == count_value || count_value < produced) return false;
    const u32 counter_slot = output_context + 4u;
    // These writes precede the later counter reads in the guest routine.
    // Restrict this path to disjoint RAM so pre-reading them is equivalent.
    auto overlaps = [](u32 a, u32 a_size, u32 b, u32 b_size) {
        return a < b + b_size && b < a + a_size;
    };
    if (overlaps(input_slot, 4u, output_context, 4u) ||
        overlaps(input_slot, 4u, counter_slot, 4u) ||
        overlaps(input_slot, 4u, kPc, 0xD0u) ||
        overlaps(counter_slot, 4u, kPc, 0xD0u) ||
        overlaps(output, 1u, output_context, 4u) ||
        overlaps(output, 1u, counter_slot, 4u) ||
        overlaps(output, 1u, kPc, 0xD0u)) return false;
    u32 counter = 0;
    if (!bus_.read32(counter_slot, counter)) return false;
    if (!bus_.write32(input_slot, cursor + 1u) ||
        !bus_.write8(output, byte) ||
        !bus_.write32(counter_slot, counter << 1u)) return false;

    state_.gpr[2].lo = sign_extend_word(counter << 1u);
    state_.gpr[3].lo = count_value;
    state_.gpr[4].lo = sign_extend_word(output_context);
    state_.gpr[5].lo = sign_extend_word(cursor + 1u);
    state_.gpr[6].lo = byte;
    state_.gpr[7].lo = sign_extend_word(context);
    state_.gpr[16].lo = sign_extend_word(output_next);
    state_.gpr[17].lo = sign_extend_word(
        static_cast<u32>(state_.gpr[17].lo) - 1u);
    state_.gpr[0] = {};
    state_.last_pc = 0x00200E3Cu;
    state_.last_instruction = kOutput[12];
    state_.cop0[13] &= ~0x00000C00u;
    state_.cop0[9] += 22u;
    state_.instructions_executed += 22u;
    current_is_delay_slot_ = true;
    next_is_delay_slot_ = false;
    bus_.tick(22u);
    return true;
}

u32 EeCpu::skip_bios_literal_iterations(u32 max_iterations) {
    u32 completed = 0;
    // All three guest write ranges are kept disjoint from this code above.
    // With no external writer during a system-approved batch, one validation
    // is enough; each later iteration still checks its data/exit conditions.
    while (completed < max_iterations &&
           skip_bios_literal_iteration_impl(completed == 0u)) {
        ++completed;
    }
    return completed;
}

bool EeCpu::can_skip_hot_sif_getreg() const {
    constexpr u32 kPc = 0x0024DE74u;
    constexpr u32 kReturnPc = 0x00263338u;
    constexpr std::array<u32, 4> kWrapper = {
        0x2403007Au, // addiu v1,zero,0x7A
        0x0000000Cu, // syscall
        0x03E00008u, // jr ra
        0x00000000u, // nop
    };

    return !halted_ &&
        state_.pc == kPc &&
        state_.next_pc == kPc + 4u &&
        !next_is_delay_slot_ &&
        static_cast<u32>(state_.gpr[3].lo) == 0x7Au &&
        static_cast<u32>(state_.gpr[4].lo) == 4u &&
        static_cast<u32>(state_.gpr[31].lo) == kReturnPc &&
        bus_.matches_code(0x0024DE70u, kWrapper);
}

bool EeCpu::skip_hot_sif_getreg(u32 value) {
    constexpr u32 kPc = 0x0024DE74u;
    constexpr u32 kReturnPc = 0x00263338u;
    constexpr u32 kCycles = 106u;

    if (!can_skip_hot_sif_getreg()) {
        return false;
    }

    // Preserve the diagnostic state produced by the real SYSCALL path.
    auto& record =
        state_.recent_syscalls[state_.recent_syscall_next];
    record.instruction = state_.instructions_executed;
    record.pc = kPc;
    record.number = 0x7Au;
    for (u32 i = 0; i < record.args.size(); ++i) {
        record.args[i] = gpr_u64(4u + i);
    }
    state_.recent_syscall_next =
        (state_.recent_syscall_next + 1u) %
        static_cast<u32>(state_.recent_syscalls.size());
    state_.recent_syscall_count = std::min(
        state_.recent_syscall_count + 1u,
        static_cast<u32>(state_.recent_syscalls.size()));
    ++state_.exception_counts[8];

    write_gpr_word(2u, value);
    state_.gpr[0] = {};

    // SYSCALL records exception code 8, the retail handler advances EPC by
    // one instruction, returns through ERET, then the wrapper executes JR RA
    // + its NOP delay slot. Interrupt-pending bits are preserved.
    state_.cop0[13] =
        (state_.cop0[13] & ~0x8000007Cu) | 0x20u;
    state_.cop0[14] = kPc + 4u;

    state_.last_pc = kPc + 8u;
    state_.last_instruction = 0u;
    state_.pc = kReturnPc;
    state_.next_pc = kReturnPc + 4u;
    current_is_delay_slot_ = true;
    next_is_delay_slot_ = false;

    state_.instructions_executed += kCycles;
    state_.cop0[9] += kCycles;
    bus_.tick(kCycles);
    return true;
}

bool EeCpu::step(std::string& error) {
    return step_internal(error, false, nullptr, false);
}

bool EeCpu::step_predecoded(
    u32 instruction, std::string& error) {
    return step_internal(error, false, &instruction, false);
}

bool EeCpu::step_quiet(std::string& error) {
    return step_internal(error, true, nullptr, false);
}

bool EeCpu::step_quiet_predecoded(
    u32 instruction, std::string& error) {
    return step_internal(error, true, &instruction, false);
}

bool EeCpu::step_quiet_unchecked_predecoded(
    u32 instruction, std::string& error) {
    return step_internal(error, true, &instruction, true);
}

u32 EeCpu::run_quiet_fast_prefix(
    u32 block_pc,
    const u32* instructions,
    u32 instruction_count,
    u32 maximum_instructions,
    bool* store_executed,
    const u8* instruction_ram,
    u8* direct_ram,
    u32* page_generations) {
    const bool direct_trace =
        instructions == nullptr && instruction_ram != nullptr;
    if (halted_ ||
        maximum_instructions == 0u ||
        (!direct_trace &&
         (instructions == nullptr ||
          instruction_count == 0u ||
          state_.pc != block_pc))) {
        return 0u;
    }

    if (store_executed != nullptr) {
        *store_executed = false;
    }

    u32 limit = direct_trace
        ? maximum_instructions
        : std::min(instruction_count, maximum_instructions);
    const u32 compare_distance =
        state_.cop0[11] - state_.cop0[9];
    if (compare_distance != 0u) {
        limit = std::min(limit, compare_distance);
    }
    u32 retired = 0u;

    constexpr u32 kMainRamSize = 32u * 1024u * 1024u;
    const bool direct_main_ram =
        direct_trace &&
        direct_ram != nullptr &&
        std::endian::native == std::endian::little;

    auto mark_direct_write = [&](u32 physical, u32 width) {
        if (page_generations == nullptr || width == 0u) return;
        const u32 first = physical >> 12;
        const u32 last = (physical + width - 1u) >> 12;
        for (u32 page = first; page <= last; ++page) {
            ++page_generations[page];
        }
    };
    auto read_ram8 = [&](u32 address, u32 physical, u8& value) {
        if (direct_main_ram && physical < kMainRamSize) {
            value = direct_ram[physical];
            return true;
        }
        return bus_.read8(address, value);
    };
    auto read_ram16 = [&](u32 address, u32 physical, u16& value) {
        if (direct_main_ram &&
            physical <= kMainRamSize - sizeof(value)) {
            std::memcpy(&value, direct_ram + physical, sizeof(value));
            return true;
        }
        return bus_.read16(address, value);
    };
    auto read_ram32 = [&](u32 address, u32 physical, u32& value) {
        if (direct_main_ram &&
            physical <= kMainRamSize - sizeof(value)) {
            std::memcpy(&value, direct_ram + physical, sizeof(value));
            return true;
        }
        return bus_.read32(address, value);
    };
    auto read_ram64 = [&](u32 address, u32 physical, u64& value) {
        if (direct_main_ram &&
            physical <= kMainRamSize - sizeof(value)) {
            std::memcpy(&value, direct_ram + physical, sizeof(value));
            return true;
        }
        return bus_.read64(address, value);
    };
    auto write_ram8 = [&](u32 address, u32 physical, u8 value) {
        if (direct_main_ram && physical < kMainRamSize) {
            direct_ram[physical] = value;
            mark_direct_write(physical, 1u);
            return true;
        }
        return bus_.write8(address, value);
    };
    auto write_ram16 = [&](u32 address, u32 physical, u16 value) {
        if (direct_main_ram &&
            physical <= kMainRamSize - sizeof(value)) {
            std::memcpy(direct_ram + physical, &value, sizeof(value));
            mark_direct_write(physical, sizeof(value));
            return true;
        }
        return bus_.write16(address, value);
    };
    auto write_ram32 = [&](u32 address, u32 physical, u32 value) {
        if (direct_main_ram &&
            physical <= kMainRamSize - sizeof(value)) {
            std::memcpy(direct_ram + physical, &value, sizeof(value));
            mark_direct_write(physical, sizeof(value));
            return true;
        }
        return bus_.write32(address, value);
    };
    auto write_ram64 = [&](u32 address, u32 physical, u64 value) {
        if (direct_main_ram &&
            physical <= kMainRamSize - sizeof(value)) {
            std::memcpy(direct_ram + physical, &value, sizeof(value));
            mark_direct_write(physical, sizeof(value));
            return true;
        }
        return bus_.write64(address, value);
    };

    auto quiet_data_span = [&](u32 virtual_address,
                               u32 width,
                               u32 alignment_mask = 0u) {
        const u32 aligned = virtual_address & ~alignment_mask;
        const u32 physical = EeBus::to_physical(aligned);
        constexpr u32 kMainRamSize = 32u * 1024u * 1024u;
        if (virtual_address < 0xC0000000u &&
            physical < kMainRamSize &&
            width <= kMainRamSize - physical) {
            return true;
        }
        if (direct_trace &&
            aligned >= 0x70000000u &&
            aligned - 0x70000000u <= 16u * 1024u &&
            width <= 16u * 1024u - (aligned - 0x70000000u)) {
            return true;
        }
        return false;
    };

    for (; retired < limit; ++retired) {
        const u32 expected_pc = direct_trace
            ? state_.pc
            : block_pc + retired * 4u;
        if (!direct_trace && state_.pc != expected_pc) break;

        u32 instruction = 0u;
        if (direct_trace) {
            const u32 physical = EeBus::to_physical(expected_pc);
            constexpr u32 kMainRamSize = 32u * 1024u * 1024u;
            if ((expected_pc & 3u) != 0u ||
                physical > kMainRamSize - sizeof(u32)) {
                break;
            }
            std::memcpy(
                &instruction,
                instruction_ram + physical,
                sizeof(instruction));
        } else {
            instruction = instructions[retired];
        }

        if (instruction == 0u &&
            !next_is_delay_slot_ &&
            state_.next_pc == expected_pc + 4u) {
            u32 run = 1u;
            if (direct_trace) {
                constexpr u32 kMainRamSize =
                    32u * 1024u * 1024u;
                while (retired + run < limit) {
                    const u32 next_pc =
                        expected_pc + run * 4u;
                    const u32 next_physical =
                        EeBus::to_physical(next_pc);
                    if (next_physical >
                        kMainRamSize - sizeof(u32)) {
                        break;
                    }
                    u32 next_instruction = 0u;
                    std::memcpy(
                        &next_instruction,
                        instruction_ram + next_physical,
                        sizeof(next_instruction));
                    if (next_instruction != 0u) break;
                    ++run;
                }
            } else {
                while (retired + run < limit &&
                       instructions[retired + run] == 0u) {
                    ++run;
                }
            }

            state_.last_pc =
                expected_pc + (run - 1u) * 4u;
            state_.last_instruction = 0u;
            state_.pc = expected_pc + run * 4u;
            state_.next_pc = state_.pc + 4u;
            current_is_delay_slot_ = false;
            state_.gpr[0] = {};
            state_.instructions_executed += run;
            state_.cop0[9] += run;
            if (state_.cop0[9] == state_.cop0[11]) {
                state_.cop0[13] |= 0x00008000u;
            }

            retired += run - 1u;
            continue;
        }

        const u32 opcode = instruction >> 26;
        const u32 rs = (instruction >> 21) & 31u;
        const u32 rt = (instruction >> 16) & 31u;
        const u32 rd = (instruction >> 11) & 31u;
        const u32 sa = (instruction >> 6) & 31u;
        const u32 funct = instruction & 63u;
        const s16 imm = immediate(instruction);

        bool handled = true;
        bool stop_after_instruction = false;
        const u32 old_next_pc = state_.next_pc;
        const bool was_delay_slot = next_is_delay_slot_;
        next_is_delay_slot_ = false;

        // Only commit architectural PC/delay state after proving this opcode
        // belongs to the no-MMIO/no-exception linear fast subset.
        if (instruction == 0u) {
            // NOP
        } else if (opcode == 0x02u) { // J
            state_.next_pc =
                ((expected_pc + 4u) & 0xF0000000u) |
                ((instruction & 0x03FFFFFFu) << 2);
            next_is_delay_slot_ = true;
        } else if (opcode == 0x03u) { // JAL
            write_gpr_word(31u, expected_pc + 8u);
            state_.next_pc =
                ((expected_pc + 4u) & 0xF0000000u) |
                ((instruction & 0x03FFFFFFu) << 2);
            next_is_delay_slot_ = true;
        } else if (opcode >= 0x04u && opcode <= 0x07u) {
            bool take = false;
            switch (opcode) {
            case 0x04u: take = gpr_u64(rs) == gpr_u64(rt); break;
            case 0x05u: take = gpr_u64(rs) != gpr_u64(rt); break;
            case 0x06u: take = gpr_s64(rs) <= 0; break;
            case 0x07u: take = gpr_s64(rs) > 0; break;
            default: break;
            }
            // step_internal() normally advances next_pc to the instruction
            // after the delay slot before branch decode. Reproduce that
            // pipeline state here even when the branch is not taken.
            state_.next_pc = expected_pc + 8u;
            if (take) state_.next_pc = branch_target(expected_pc, imm);
            next_is_delay_slot_ = true;
        } else if (opcode >= 0x14u && opcode <= 0x17u) {
            bool take = false;
            switch (opcode) {
            case 0x14u: take = gpr_u64(rs) == gpr_u64(rt); break;
            case 0x15u: take = gpr_u64(rs) != gpr_u64(rt); break;
            case 0x16u: take = gpr_s64(rs) <= 0; break;
            case 0x17u: take = gpr_s64(rs) > 0; break;
            default: break;
            }
            if (take) {
                state_.next_pc = branch_target(expected_pc, imm);
                next_is_delay_slot_ = true;
            } else {
                state_.pc = expected_pc + 8u;
                state_.next_pc = expected_pc + 12u;
                next_is_delay_slot_ = false;
            }
        } else if (opcode == 0x08u) { // ADDI
            const s32 lhs =
                static_cast<s32>(
                    static_cast<u32>(gpr_u64(rs)));
            const s32 rhs = static_cast<s32>(imm);
            const s64 sum =
                static_cast<s64>(lhs) +
                static_cast<s64>(rhs);
            if (sum <
                    static_cast<s64>(
                        std::numeric_limits<s32>::min()) ||
                sum >
                    static_cast<s64>(
                        std::numeric_limits<s32>::max())) {
                // Rare architected overflow: leave PC/state untouched and
                // let the full interpreter raise the exact exception.
                handled = false;
            } else {
                write_gpr_word(
                    rt,
                    static_cast<u32>(
                        static_cast<s32>(sum)));
            }
        } else if (opcode == 0x09u) {
            write_gpr_word(
                rt,
                static_cast<u32>(gpr_u64(rs)) +
                    static_cast<u32>(static_cast<s32>(imm)));
        } else if (opcode == 0x0Au) {
            write_gpr64(
                rt,
                gpr_s64(rs) < static_cast<s64>(imm) ? 1u : 0u);
        } else if (opcode == 0x0Bu) {
            write_gpr64(
                rt,
                gpr_u64(rs) <
                        static_cast<u64>(static_cast<s64>(imm))
                    ? 1u
                    : 0u);
        } else if (opcode == 0x0Cu) {
            write_gpr64(
                rt,
                gpr_u64(rs) &
                    static_cast<u64>(instruction & 0xFFFFu));
        } else if (opcode == 0x0Du) {
            write_gpr64(
                rt,
                gpr_u64(rs) |
                    static_cast<u64>(instruction & 0xFFFFu));
        } else if (opcode == 0x0Eu) {
            write_gpr64(
                rt,
                gpr_u64(rs) ^
                    static_cast<u64>(instruction & 0xFFFFu));
        } else if (opcode == 0x0Fu) {
            write_gpr_word(rt, (instruction & 0xFFFFu) << 16);
        } else if (opcode == 0x12u) { // COP2/VU0 register-only fast subset
            const u32 cop_rs = rs;
            const u32 ft = rt;
            const u32 fs = rd;
            const u32 fd = sa;
            const u32 cop_funct = funct;

            auto vu_lane_read = [&](u32 reg, u32 lane) -> u32 {
                const EeGpr& value = state_.vu_vf[reg & 31u];
                const u64 half = lane < 2u ? value.lo : value.hi;
                return static_cast<u32>(
                    half >> ((lane & 1u) * 32u));
            };
            auto vu_lane_write =
                [&](u32 reg, u32 lane, u32 value) {
                    if ((reg & 31u) == 0u) return;
                    EeGpr& target = state_.vu_vf[reg & 31u];
                    u64& half = lane < 2u ? target.lo : target.hi;
                    const u32 shift = (lane & 1u) * 32u;
                    half =
                        (half & ~(0xFFFFFFFFull << shift)) |
                        (static_cast<u64>(value) << shift);
                };
            auto vu_acc_read = [&](u32 lane) -> u32 {
                const u64 half =
                    lane < 2u ? state_.vu_acc.lo : state_.vu_acc.hi;
                return static_cast<u32>(
                    half >> ((lane & 1u) * 32u));
            };
            auto vu_acc_write = [&](u32 lane, u32 value) {
                u64& half =
                    lane < 2u ? state_.vu_acc.lo : state_.vu_acc.hi;
                const u32 shift = (lane & 1u) * 32u;
                half =
                    (half & ~(0xFFFFFFFFull << shift)) |
                    (static_cast<u64>(value) << shift);
            };
            auto vu_selected = [&](u32 lane) {
                static constexpr u32 bits[4] = {
                    24u, 23u, 22u, 21u};
                return ((instruction >> bits[lane]) & 1u) != 0u;
            };
            auto vu_float = [&](u32 reg, u32 lane) {
                return ps2_fpu_input(vu_lane_read(reg, lane));
            };
            auto vu_write_float =
                [&](u32 reg, u32 lane, float value) {
                    vu_lane_write(
                        reg, lane, ps2_fpu_result(value));
                };
            auto acc_float = [&](u32 lane) {
                return ps2_fpu_input(vu_acc_read(lane));
            };
            auto acc_write_float =
                [&](u32 lane, float value) {
                    vu_acc_write(
                        lane, ps2_fpu_result(value));
                };
            auto broadcast_binary =
                [&](u32 source_lane, auto op) {
                    const float scalar =
                        vu_float(ft, source_lane & 3u);
                    for (u32 lane = 0u; lane < 4u; ++lane) {
                        if (!vu_selected(lane)) continue;
                        vu_write_float(
                            fd,
                            lane,
                            op(vu_float(fs, lane), scalar));
                    }
                };
            auto broadcast_madd =
                [&](u32 source_lane) {
                    const float scalar =
                        vu_float(ft, source_lane & 3u);
                    for (u32 lane = 0u; lane < 4u; ++lane) {
                        if (!vu_selected(lane)) continue;
                        const float product =
                            vu_float(fs, lane) * scalar;
                        vu_write_float(
                            fd,
                            lane,
                            acc_float(lane) + product);
                    }
                };
            auto acc_broadcast_binary =
                [&](u32 source_lane, auto op) {
                    const float scalar =
                        vu_float(ft, source_lane & 3u);
                    for (u32 lane = 0u; lane < 4u; ++lane) {
                        if (!vu_selected(lane)) continue;
                        acc_write_float(
                            lane,
                            op(vu_float(fs, lane), scalar));
                    }
                };
            auto acc_broadcast_madd =
                [&](u32 source_lane) {
                    const float scalar =
                        vu_float(ft, source_lane & 3u);
                    for (u32 lane = 0u; lane < 4u; ++lane) {
                        if (!vu_selected(lane)) continue;
                        const float product =
                            vu_float(fs, lane) * scalar;
                        acc_write_float(
                            lane,
                            acc_float(lane) + product);
                    }
                };
            auto vector_binary = [&](auto op) {
                for (u32 lane = 0u; lane < 4u; ++lane) {
                    if (!vu_selected(lane)) continue;
                    vu_write_float(
                        fd,
                        lane,
                        op(vu_float(fs, lane), vu_float(ft, lane)));
                }
            };
            auto vector_madd = [&](bool subtract_product) {
                for (u32 lane = 0u; lane < 4u; ++lane) {
                    if (!vu_selected(lane)) continue;
                    const float product =
                        vu_float(fs, lane) * vu_float(ft, lane);
                    vu_write_float(
                        fd,
                        lane,
                        acc_float(lane) +
                            (subtract_product ? -product : product));
                }
            };
            auto scalar_control_binary =
                [&](u32 control, auto op) {
                    const float scalar =
                        ps2_fpu_input(state_.vu_vi[control]);
                    for (u32 lane = 0u; lane < 4u; ++lane) {
                        if (!vu_selected(lane)) continue;
                        vu_write_float(
                            fd,
                            lane,
                            op(vu_float(fs, lane), scalar));
                    }
                };
            auto control_madd =
                [&](u32 control, bool subtract_product) {
                    const float scalar =
                        ps2_fpu_input(state_.vu_vi[control]);
                    for (u32 lane = 0u; lane < 4u; ++lane) {
                        if (!vu_selected(lane)) continue;
                        const float product =
                            vu_float(fs, lane) * scalar;
                        vu_write_float(
                            fd,
                            lane,
                            acc_float(lane) +
                                (subtract_product ? -product : product));
                    }
                };
            auto acc_vector_binary = [&](auto op) {
                for (u32 lane = 0u; lane < 4u; ++lane) {
                    if (!vu_selected(lane)) continue;
                    acc_write_float(
                        lane,
                        op(vu_float(fs, lane), vu_float(ft, lane)));
                }
            };
            auto acc_vector_madd = [&](bool subtract_product) {
                for (u32 lane = 0u; lane < 4u; ++lane) {
                    if (!vu_selected(lane)) continue;
                    const float product =
                        vu_float(fs, lane) * vu_float(ft, lane);
                    acc_write_float(
                        lane,
                        acc_float(lane) +
                            (subtract_product ? -product : product));
                }
            };
            auto acc_control_binary =
                [&](u32 control, auto op) {
                    const float scalar =
                        ps2_fpu_input(state_.vu_vi[control]);
                    for (u32 lane = 0u; lane < 4u; ++lane) {
                        if (!vu_selected(lane)) continue;
                        acc_write_float(
                            lane,
                            op(vu_float(fs, lane), scalar));
                    }
                };
            auto acc_control_madd =
                [&](u32 control, bool subtract_product) {
                    const float scalar =
                        ps2_fpu_input(state_.vu_vi[control]);
                    for (u32 lane = 0u; lane < 4u; ++lane) {
                        if (!vu_selected(lane)) continue;
                        const float product =
                            vu_float(fs, lane) * scalar;
                        acc_write_float(
                            lane,
                            acc_float(lane) +
                                (subtract_product ? -product : product));
                    }
                };
            auto saturating_float_to_int = [](float value) -> s32 {
                if (std::isnan(value)) return 0;
                if (value >=
                    static_cast<float>(
                        std::numeric_limits<s32>::max())) {
                    return std::numeric_limits<s32>::max();
                }
                if (value <=
                    static_cast<float>(
                        std::numeric_limits<s32>::min())) {
                    return std::numeric_limits<s32>::min();
                }
                return static_cast<s32>(value);
            };
            auto write_vi = [&](u32 reg, u32 value) {
                reg &= 0xFu;
                if (reg != 0u) {
                    state_.vu_vi[reg] = static_cast<u16>(value);
                }
            };

            if (cop_rs == 0x01u) { // QMFC2
                if (rt != 0u) state_.gpr[rt] = state_.vu_vf[fs];
            } else if (cop_rs == 0x02u) { // CFC2
                if (rt != 0u) {
                    u32 value = state_.vu_vi[fs];
                    if (fs == 20u) value &= 0x007FFFFFu;
                    write_gpr_word(rt, value);
                }
            } else if (cop_rs == 0x05u) { // QMTC2
                if (fs != 0u) state_.vu_vf[fs] = state_.gpr[rt];
            } else if (cop_rs == 0x06u) { // CTC2
                if (fs == 0u || fs == 17u ||
                    fs == 26u || fs == 29u) {
                    // Read-only / ignored control registers.
                } else {
                    const u32 value =
                        static_cast<u32>(gpr_u64(rt));
                    if (fs < 16u) {
                        state_.vu_vi[fs] =
                            static_cast<u16>(value);
                    } else if (fs == 20u) {
                        state_.vu_vi[20] =
                            (value & 0x007FFFFFu) |
                            0x3F800000u;
                    } else if (fs == 28u) {
                        state_.vu_vi[28] =
                            value & 0x00000C0Cu;
                        if ((value & 0x2u) != 0u) {
                            for (u32 i = 1u; i < 32u; ++i) {
                                state_.vu_vf[i] = {};
                            }
                            for (u32 i = 1u; i < 16u; ++i) {
                                state_.vu_vi[i] = 0u;
                            }
                            state_.vu_vi[29] &= ~0xFFu;
                        }
                        if ((value & 0x200u) != 0u) {
                            state_.vu_vi[29] &= ~0xFF00u;
                        }
                    } else {
                        state_.vu_vi[fs] = value;
                    }
                }
            } else if (cop_rs >= 0x10u) {
                const auto add =
                    [](float lhs, float rhs) {
                        return lhs + rhs;
                    };
                const auto mul =
                    [](float lhs, float rhs) {
                        return lhs * rhs;
                    };

                if (cop_funct <= 0x03u) { // VADDx/y/z/w
                    broadcast_binary(cop_funct, add);
                } else if (cop_funct >= 0x04u &&
                           cop_funct <= 0x07u) { // VSUBx/y/z/w
                    broadcast_binary(
                        cop_funct & 3u,
                        [](float lhs, float rhs) {
                            return lhs - rhs;
                        });
                } else if (cop_funct >= 0x08u &&
                           cop_funct <= 0x0Bu) { // VMADDx/y/z/w
                    broadcast_madd(cop_funct & 3u);
                } else if (cop_funct >= 0x0Cu &&
                           cop_funct <= 0x0Fu) { // VMSUBx/y/z/w
                    const float scalar =
                        vu_float(ft, cop_funct & 3u);
                    for (u32 lane = 0u; lane < 4u; ++lane) {
                        if (!vu_selected(lane)) continue;
                        const float product =
                            vu_float(fs, lane) * scalar;
                        vu_write_float(
                            fd,
                            lane,
                            acc_float(lane) - product);
                    }
                } else if (cop_funct >= 0x10u &&
                           cop_funct <= 0x13u) { // VMAXx/y/z/w
                    broadcast_binary(
                        cop_funct & 3u,
                        [](float lhs, float rhs) {
                            return std::fmax(lhs, rhs);
                        });
                } else if (cop_funct >= 0x14u &&
                           cop_funct <= 0x17u) { // VMINIx/y/z/w
                    broadcast_binary(
                        cop_funct & 3u,
                        [](float lhs, float rhs) {
                            return std::fmin(lhs, rhs);
                        });
                } else if (cop_funct >= 0x18u &&
                           cop_funct <= 0x1Bu) { // VMULx/y/z/w
                    broadcast_binary(
                        cop_funct & 3u, mul);
                } else if (cop_funct == 0x1Cu) { // VMULq
                    scalar_control_binary(22u, mul);
                } else if (cop_funct == 0x1Du) { // VMAXi
                    scalar_control_binary(
                        21u,
                        [](float lhs, float rhs) {
                            return std::fmax(lhs, rhs);
                        });
                } else if (cop_funct == 0x1Eu) { // VMULi
                    scalar_control_binary(21u, mul);
                } else if (cop_funct == 0x1Fu) { // VMINIi
                    scalar_control_binary(
                        21u,
                        [](float lhs, float rhs) {
                            return std::fmin(lhs, rhs);
                        });
                } else if (cop_funct == 0x20u) { // VADDq
                    scalar_control_binary(
                        22u,
                        [](float lhs, float rhs) {
                            return lhs + rhs;
                        });
                } else if (cop_funct == 0x21u) { // VMADDq
                    control_madd(22u, false);
                } else if (cop_funct == 0x22u) { // VADDi
                    scalar_control_binary(
                        21u,
                        [](float lhs, float rhs) {
                            return lhs + rhs;
                        });
                } else if (cop_funct == 0x23u) { // VMADDi
                    control_madd(21u, false);
                } else if (cop_funct == 0x24u) { // VSUBq
                    scalar_control_binary(
                        22u,
                        [](float lhs, float rhs) {
                            return lhs - rhs;
                        });
                } else if (cop_funct == 0x25u) { // VMSUBq
                    control_madd(22u, true);
                } else if (cop_funct == 0x26u) { // VSUBi
                    scalar_control_binary(
                        21u,
                        [](float lhs, float rhs) {
                            return lhs - rhs;
                        });
                } else if (cop_funct == 0x27u) { // VMSUBi
                    control_madd(21u, true);
                } else if (cop_funct == 0x28u) { // VADD
                    vector_binary(
                        [](float lhs, float rhs) {
                            return lhs + rhs;
                        });
                } else if (cop_funct == 0x29u) { // VMADD
                    vector_madd(false);
                } else if (cop_funct == 0x2Au) { // VMUL
                    vector_binary(mul);
                } else if (cop_funct == 0x2Bu) { // VMAX
                    vector_binary(
                        [](float lhs, float rhs) {
                            return std::fmax(lhs, rhs);
                        });
                } else if (cop_funct == 0x2Cu) { // VSUB
                    vector_binary(
                        [](float lhs, float rhs) {
                            return lhs - rhs;
                        });
                } else if (cop_funct == 0x2Du) { // VMSUB
                    vector_madd(true);
                } else if (cop_funct == 0x2Eu) { // VOPMSUB
                    const float result[3] = {
                        acc_float(0u) -
                            vu_float(fs, 1u) * vu_float(ft, 2u),
                        acc_float(1u) -
                            vu_float(fs, 2u) * vu_float(ft, 0u),
                        acc_float(2u) -
                            vu_float(fs, 0u) * vu_float(ft, 1u),
                    };
                    for (u32 lane = 0u; lane < 3u; ++lane) {
                        if (vu_selected(lane)) {
                            vu_write_float(fd, lane, result[lane]);
                        }
                    }
                } else if (cop_funct == 0x2Fu) { // VMINI
                    vector_binary(
                        [](float lhs, float rhs) {
                            return std::fmin(lhs, rhs);
                        });
                } else if (cop_funct >= 0x3Cu) {
                    const u32 special =
                        (instruction & 3u) |
                        ((instruction >> 4) & 0x7Cu);
                    const u32 it = ft & 0xFu;
                    const u32 is = fs & 0xFu;
                    const u32 fsf =
                        (instruction >> 21) & 3u;
                    const u32 ftf =
                        (instruction >> 23) & 3u;

                    if (special <= 0x03u) { // VADDAx/y/z/w
                        acc_broadcast_binary(
                            special,
                            [](float lhs, float rhs) {
                                return lhs + rhs;
                            });
                    } else if (special >= 0x04u &&
                               special <= 0x07u) { // VSUBAx/y/z/w
                        acc_broadcast_binary(
                            special & 3u,
                            [](float lhs, float rhs) {
                                return lhs - rhs;
                            });
                    } else if (special >= 0x08u &&
                               special <= 0x0Bu) { // VMADDAx/y/z/w
                        acc_broadcast_madd(special & 3u);
                    } else if (special >= 0x0Cu &&
                               special <= 0x0Fu) { // VMSUBAx/y/z/w
                        const float scalar =
                            vu_float(ft, special & 3u);
                        for (u32 lane = 0u; lane < 4u; ++lane) {
                            if (!vu_selected(lane)) continue;
                            const float product =
                                vu_float(fs, lane) * scalar;
                            acc_write_float(
                                lane,
                                acc_float(lane) - product);
                        }
                    } else if (special >= 0x10u &&
                               special <= 0x13u) { // VITOF0/4/12/15
                        static constexpr u32 shifts[4] = {
                            0u, 4u, 12u, 15u};
                        const u32 shift = shifts[special - 0x10u];
                        for (u32 lane = 0u; lane < 4u; ++lane) {
                            if (!vu_selected(lane)) continue;
                            const float value =
                                static_cast<float>(
                                    static_cast<s32>(
                                        vu_lane_read(fs, lane))) /
                                static_cast<float>(1u << shift);
                            vu_write_float(ft, lane, value);
                        }
                    } else if (special >= 0x14u &&
                               special <= 0x17u) { // VFTOI0/4/12/15
                        static constexpr u32 shifts[4] = {
                            0u, 4u, 12u, 15u};
                        const u32 shift = shifts[special - 0x14u];
                        for (u32 lane = 0u; lane < 4u; ++lane) {
                            if (!vu_selected(lane)) continue;
                            const float scaled =
                                vu_float(fs, lane) *
                                static_cast<float>(1u << shift);
                            vu_lane_write(
                                ft,
                                lane,
                                static_cast<u32>(
                                    saturating_float_to_int(scaled)));
                        }
                    } else if (special >= 0x18u &&
                               special <= 0x1Bu) { // VMULAx/y/z/w
                        acc_broadcast_binary(
                            special & 3u, mul);
                    } else if (special == 0x1Cu) { // VMULAq
                        acc_control_binary(22u, mul);
                    } else if (special == 0x1Du) { // VABS
                        for (u32 lane = 0u; lane < 4u; ++lane) {
                            if (vu_selected(lane)) {
                                vu_lane_write(
                                    ft,
                                    lane,
                                    vu_lane_read(fs, lane) &
                                        0x7FFFFFFFu);
                            }
                        }
                    } else if (special == 0x1Eu) { // VMULAi
                        acc_control_binary(21u, mul);
                    } else if (special == 0x1Fu) { // VCLIPw
                        const float w =
                            std::fabs(vu_float(ft, 3u));
                        u32 clip =
                            (state_.vu_vi[18] << 6) & 0xFFFFFFu;
                        for (u32 lane = 0u; lane < 3u; ++lane) {
                            const float value =
                                vu_float(fs, lane);
                            if (value > w) {
                                clip |= 1u << (lane * 2u);
                            }
                            if (value < -w) {
                                clip |= 1u << (lane * 2u + 1u);
                            }
                        }
                        state_.vu_vi[18] = clip;
                    } else if (special == 0x20u) { // VADDAq
                        acc_control_binary(
                            22u,
                            [](float lhs, float rhs) {
                                return lhs + rhs;
                            });
                    } else if (special == 0x21u) { // VMADDAq
                        acc_control_madd(22u, false);
                    } else if (special == 0x22u) { // VADDAi
                        acc_control_binary(
                            21u,
                            [](float lhs, float rhs) {
                                return lhs + rhs;
                            });
                    } else if (special == 0x23u) { // VMADDAi
                        acc_control_madd(21u, false);
                    } else if (special == 0x24u) { // VSUBAq
                        acc_control_binary(
                            22u,
                            [](float lhs, float rhs) {
                                return lhs - rhs;
                            });
                    } else if (special == 0x25u) { // VMSUBAq
                        acc_control_madd(22u, true);
                    } else if (special == 0x26u) { // VSUBAi
                        acc_control_binary(
                            21u,
                            [](float lhs, float rhs) {
                                return lhs - rhs;
                            });
                    } else if (special == 0x27u) { // VMSUBAi
                        acc_control_madd(21u, true);
                    } else if (special == 0x28u) { // VADDA
                        acc_vector_binary(
                            [](float lhs, float rhs) {
                                return lhs + rhs;
                            });
                    } else if (special == 0x29u) { // VMADDA
                        acc_vector_madd(false);
                    } else if (special == 0x2Au) { // VMULA
                        acc_vector_binary(mul);
                    } else if (special == 0x2Cu) { // VSUBA
                        acc_vector_binary(
                            [](float lhs, float rhs) {
                                return lhs - rhs;
                            });
                    } else if (special == 0x2Du) { // VMSUBA
                        acc_vector_madd(true);
                    } else if (special == 0x2Eu) { // VOPMULA
                        const float values[3] = {
                            vu_float(fs, 1u) * vu_float(ft, 2u),
                            vu_float(fs, 2u) * vu_float(ft, 0u),
                            vu_float(fs, 0u) * vu_float(ft, 1u),
                        };
                        for (u32 lane = 0u; lane < 3u; ++lane) {
                            if (vu_selected(lane)) {
                                acc_write_float(lane, values[lane]);
                            }
                        }
                    } else if (special == 0x2Fu) { // VNOP
                        // Architectural no-op.
                    } else if (special == 0x30u) { // VMOVE
                        const EeGpr source = state_.vu_vf[fs];
                        auto source_lane = [&](u32 lane) {
                            const u64 half =
                                lane < 2u ? source.lo : source.hi;
                            return static_cast<u32>(
                                half >> ((lane & 1u) * 32u));
                        };
                        for (u32 lane = 0u; lane < 4u; ++lane) {
                            if (vu_selected(lane)) {
                                vu_lane_write(
                                    ft,
                                    lane,
                                    source_lane(lane));
                            }
                        }
                    } else if (special == 0x31u) { // VMR32
                        const EeGpr source = state_.vu_vf[fs];
                        auto source_lane = [&](u32 lane) {
                            const u32 rotated = (lane + 1u) & 3u;
                            const u64 half =
                                rotated < 2u ? source.lo : source.hi;
                            return static_cast<u32>(
                                half >> ((rotated & 1u) * 32u));
                        };
                        for (u32 lane = 0u; lane < 4u; ++lane) {
                            if (vu_selected(lane)) {
                                vu_lane_write(
                                    ft,
                                    lane,
                                    source_lane(lane));
                            }
                        }
                    } else if (special == 0x38u) { // VDIV
                        const float numerator =
                            vu_float(fs, fsf);
                        const float denominator =
                            vu_float(ft, ftf);
                        if (denominator == 0.0f) {
                            state_.vu_vi[22] =
                                ps2_fpu_result(
                                    std::copysign(
                                        std::numeric_limits<float>::max(),
                                        numerator * denominator));
                        } else {
                            state_.vu_vi[22] =
                                ps2_fpu_result(
                                    numerator / denominator);
                        }
                    } else if (special == 0x39u) { // VSQRT
                        state_.vu_vi[22] =
                            ps2_fpu_result(
                                std::sqrt(
                                    std::fabs(vu_float(ft, ftf))));
                    } else if (special == 0x3Au) { // VRSQRT
                        const float numerator =
                            vu_float(fs, fsf);
                        const float denominator =
                            std::sqrt(
                                std::fabs(vu_float(ft, ftf)));
                        if (denominator == 0.0f) {
                            state_.vu_vi[22] =
                                ps2_fpu_result(
                                    std::copysign(
                                        std::numeric_limits<float>::max(),
                                        numerator));
                        } else {
                            state_.vu_vi[22] =
                                ps2_fpu_result(
                                    numerator / denominator);
                        }
                    } else if (special == 0x3Bu) { // VWAITQ
                        // Architectural no-op in this synchronous model.
                    } else if (special == 0x3Cu) { // VMTIR
                        write_vi(
                            it,
                            vu_lane_read(fs, fsf));
                    } else if (special == 0x3Du) { // VMFIR
                        const u32 value =
                            static_cast<u32>(
                                static_cast<s32>(
                                    static_cast<s16>(
                                        state_.vu_vi[is])));
                        for (u32 lane = 0u; lane < 4u; ++lane) {
                            if (vu_selected(lane)) {
                                vu_lane_write(ft, lane, value);
                            }
                        }
                    } else {
                        handled = false;
                    }
                } else {
                    handled = false;
                }
            } else {
                handled = false;
            }
        } else if (opcode == 0x19u) {
            write_gpr64(
                rt,
                gpr_u64(rs) +
                    static_cast<u64>(static_cast<s64>(imm)));
        } else if (opcode == 0x1Cu) { // MMI hot integer multiply/divide subset
            switch (funct) {
            case 0x18u: // MULT1
                multiply_signed32(
                    static_cast<u32>(gpr_u64(rs)),
                    static_cast<u32>(gpr_u64(rt)),
                    state_.lo1,
                    state_.hi1);
                write_gpr64(rd, state_.lo1);
                break;
            case 0x19u: // MULTU1
                multiply_unsigned32(
                    static_cast<u32>(gpr_u64(rs)),
                    static_cast<u32>(gpr_u64(rt)),
                    state_.lo1,
                    state_.hi1);
                write_gpr64(rd, state_.lo1);
                break;
            case 0x1Au: // DIV1
                divide_signed32(
                    static_cast<u32>(gpr_u64(rs)),
                    static_cast<u32>(gpr_u64(rt)),
                    state_.lo1,
                    state_.hi1);
                break;
            case 0x1Bu: // DIVU1
                divide_unsigned32(
                    static_cast<u32>(gpr_u64(rs)),
                    static_cast<u32>(gpr_u64(rt)),
                    state_.lo1,
                    state_.hi1);
                break;
            default:
                handled = false;
                break;
            }
        } else if (
            opcode == 0x1Eu ||
            opcode == 0x20u || opcode == 0x21u ||
            opcode == 0x23u || opcode == 0x24u ||
            opcode == 0x25u || opcode == 0x27u ||
            opcode == 0x30u || opcode == 0x31u ||
            opcode == 0x34u || opcode == 0x36u ||
            opcode == 0x37u) {
            const u32 address = static_cast<u32>(
                gpr_u64(rs) +
                static_cast<u64>(static_cast<s64>(imm)));
            const u32 aligned =
                (opcode == 0x1Eu || opcode == 0x36u)
                    ? (address & ~0x0Fu)
                    : address;
            const u32 width =
                (opcode == 0x20u || opcode == 0x24u) ? 1u :
                (opcode == 0x21u || opcode == 0x25u) ? 2u :
                (opcode == 0x23u || opcode == 0x27u ||
                 opcode == 0x30u || opcode == 0x31u) ? 4u :
                (opcode == 0x34u || opcode == 0x37u) ? 8u :
                16u;
            const u32 physical = EeBus::to_physical(aligned);
            if (!quiet_data_span(
                    address,
                    width,
                    (opcode == 0x1Eu || opcode == 0x36u) ? 15u : 0u)) {
                handled = false;
            } else {
                bool access_ok = true;
                switch (opcode) {
                case 0x1Eu: { // LQ
                    u64 lo = 0;
                    u64 hi = 0;
                    access_ok =
                        read_ram64(aligned, physical, lo) &&
                        read_ram64(
                            aligned + 8u, physical + 8u, hi);
                    if (access_ok && rt != 0u) {
                        state_.gpr[rt].lo = lo;
                        state_.gpr[rt].hi = hi;
                    }
                    break;
                }
                case 0x20u: { // LB
                    u8 value = 0;
                    access_ok = read_ram8(address, physical, value);
                    if (access_ok) {
                        write_gpr64(
                            rt,
                            static_cast<u64>(static_cast<s64>(
                                static_cast<s8>(value))));
                    }
                    break;
                }
                case 0x21u: { // LH
                    u16 value = 0;
                    access_ok = read_ram16(address, physical, value);
                    if (access_ok) {
                        write_gpr64(
                            rt,
                            static_cast<u64>(static_cast<s64>(
                                static_cast<s16>(value))));
                    }
                    break;
                }
                case 0x23u:
                case 0x30u: { // LW / LL
                    u32 value = 0;
                    access_ok = read_ram32(address, physical, value);
                    if (access_ok) write_gpr_word(rt, value);
                    break;
                }
                case 0x24u: { // LBU
                    u8 value = 0;
                    access_ok = read_ram8(address, physical, value);
                    if (access_ok) write_gpr64(rt, value);
                    break;
                }
                case 0x25u: { // LHU
                    u16 value = 0;
                    access_ok = read_ram16(address, physical, value);
                    if (access_ok) write_gpr64(rt, value);
                    break;
                }
                case 0x27u: { // LWU
                    u32 value = 0;
                    access_ok = read_ram32(address, physical, value);
                    if (access_ok) write_gpr64(rt, value);
                    break;
                }
                case 0x31u: { // LWC1
                    u32 value = 0;
                    access_ok = read_ram32(address, physical, value);
                    if (access_ok) state_.fpr[rt] = value;
                    break;
                }
                case 0x34u:
                case 0x37u: { // LLD / LD
                    u64 value = 0;
                    access_ok = read_ram64(address, physical, value);
                    if (access_ok) write_gpr64(rt, value);
                    break;
                }
                case 0x36u: { // LQC2
                    u64 lo = 0;
                    u64 hi = 0;
                    access_ok =
                        read_ram64(aligned, physical, lo) &&
                        read_ram64(
                            aligned + 8u, physical + 8u, hi);
                    if (access_ok && rt != 0u) {
                        state_.vu_vf[rt].lo = lo;
                        state_.vu_vf[rt].hi = hi;
                    }
                    break;
                }
                default:
                    access_ok = false;
                    break;
                }
                if (!access_ok) handled = false;
            }
        } else if (
            opcode == 0x1Fu ||
            opcode == 0x28u || opcode == 0x29u ||
            opcode == 0x2Bu || opcode == 0x38u ||
            opcode == 0x39u || opcode == 0x3Cu ||
            opcode == 0x3Eu || opcode == 0x3Fu) {
            const u32 address = static_cast<u32>(
                gpr_u64(rs) +
                static_cast<u64>(static_cast<s64>(imm)));
            const u32 aligned =
                (opcode == 0x1Fu || opcode == 0x3Eu)
                    ? (address & ~0x0Fu)
                    : address;
            const u32 width =
                opcode == 0x28u ? 1u :
                opcode == 0x29u ? 2u :
                (opcode == 0x2Bu || opcode == 0x38u ||
                 opcode == 0x39u) ? 4u :
                (opcode == 0x3Cu || opcode == 0x3Fu) ? 8u :
                16u;
            const u32 physical = EeBus::to_physical(aligned);
            if (!quiet_data_span(
                    address,
                    width,
                    (opcode == 0x1Fu || opcode == 0x3Eu) ? 15u : 0u)) {
                handled = false;
            } else {
                bool access_ok = true;
                switch (opcode) {
                case 0x1Fu: // SQ
                    access_ok =
                        write_ram64(
                            aligned, physical, state_.gpr[rt].lo) &&
                        write_ram64(
                            aligned + 8u,
                            physical + 8u,
                            state_.gpr[rt].hi);
                    break;
                case 0x28u: // SB
                    access_ok = write_ram8(
                        address,
                        physical,
                        static_cast<u8>(gpr_u64(rt)));
                    break;
                case 0x29u: // SH
                    access_ok = write_ram16(
                        address,
                        physical,
                        static_cast<u16>(gpr_u64(rt)));
                    break;
                case 0x2Bu: // SW
                    access_ok = write_ram32(
                        address,
                        physical,
                        static_cast<u32>(gpr_u64(rt)));
                    break;
                case 0x38u: // SC
                    access_ok = write_ram32(
                        address,
                        physical,
                        static_cast<u32>(gpr_u64(rt)));
                    if (access_ok) write_gpr_word(rt, 1u);
                    break;
                case 0x39u: // SWC1
                    access_ok = write_ram32(
                        address, physical, state_.fpr[rt]);
                    break;
                case 0x3Cu: // SCD
                    access_ok = write_ram64(
                        address, physical, gpr_u64(rt));
                    if (access_ok) write_gpr64(rt, 1u);
                    break;
                case 0x3Eu: // SQC2
                    access_ok =
                        write_ram64(
                            aligned, physical, state_.vu_vf[rt].lo) &&
                        write_ram64(
                            aligned + 8u,
                            physical + 8u,
                            state_.vu_vf[rt].hi);
                    break;
                case 0x3Fu: // SD
                    access_ok = write_ram64(
                        address, physical, gpr_u64(rt));
                    break;
                default:
                    access_ok = false;
                    break;
                }
                if (!access_ok) {
                    handled = false;
                } else {
                    // Cached prefixes must revalidate after a store because
                    // the write may have touched their code page. Direct RAM
                    // traces fetch the next instruction fresh, so they can
                    // safely continue through ordinary and self-modifying
                    // stores without returning to the block cache.
                    stop_after_instruction = !direct_trace;
                    if (stop_after_instruction &&
                        store_executed != nullptr) {
                        *store_executed = true;
                    }
                }
            }
        } else if (opcode == 0x11u) { // COP1
            const u32 cop_rs = rs;
            const u32 fs = (instruction >> 11) & 31u;
            const u32 fd = (instruction >> 6) & 31u;
            const u32 cop_funct = instruction & 63u;
            constexpr u32 kFpuCond = 0x00800000u;
            bool cop1_handled = true;

            if (cop_rs == 0x00u) { // MFC1
                write_gpr_word(rt, state_.fpr[fs]);
            } else if (cop_rs == 0x02u) { // CFC1
                if (fs == 0u) write_gpr_word(rt, 0x00002E00u);
                else if (fs == 31u) write_gpr_word(rt, state_.fcr[31]);
                else write_gpr_word(rt, 0u);
            } else if (cop_rs == 0x04u) { // MTC1
                state_.fpr[fs] = static_cast<u32>(gpr_u64(rt));
            } else if (cop_rs == 0x06u) { // CTC1
                if (fs == 31u) {
                    state_.fcr[31] = static_cast<u32>(gpr_u64(rt));
                }
            } else if (cop_rs == 0x08u) { // BC1
                const bool cond =
                    (state_.fcr[31] & kFpuCond) != 0u;
                const u32 variant = rt & 3u;
                const bool take =
                    (variant & 1u) != 0u ? cond : !cond;
                const bool likely = (variant & 2u) != 0u;
                if (take) {
                    state_.next_pc =
                        branch_target(expected_pc, imm);
                    next_is_delay_slot_ = true;
                } else if (likely) {
                    state_.pc = expected_pc + 8u;
                    state_.next_pc = expected_pc + 12u;
                    next_is_delay_slot_ = false;
                } else {
                    state_.next_pc = expected_pc + 8u;
                    next_is_delay_slot_ = true;
                }
            } else if (cop_rs == 0x14u) { // COP1.W
                if (cop_funct == 0x20u) { // CVT.S.W
                    const s32 value =
                        static_cast<s32>(state_.fpr[fs]);
                    state_.fpr[fd] =
                        ps2_fpu_result(static_cast<float>(value));
                } else {
                    cop1_handled = false;
                }
            } else if (cop_rs == 0x10u) { // COP1.S
                const u32 ft = rt;
                const float a = ps2_fpu_input(state_.fpr[fs]);
                const float b = ps2_fpu_input(state_.fpr[ft]);
                const float acc = ps2_fpu_input(state_.fpu_acc);
                auto set_fd = [&](float value) {
                    state_.fpr[fd] = ps2_fpu_result(value);
                };
                auto set_acc = [&](float value) {
                    state_.fpu_acc = ps2_fpu_result(value);
                };
                auto set_cond = [&](bool value) {
                    if (value) state_.fcr[31] |= kFpuCond;
                    else state_.fcr[31] &= ~kFpuCond;
                };
                switch (cop_funct) {
                case 0x00u: set_fd(a + b); break;
                case 0x01u: set_fd(a - b); break;
                case 0x02u: set_fd(a * b); break;
                case 0x03u:
                    if ((state_.fpr[ft] & 0x7FFFFFFFu) == 0u) {
                        const u32 sign =
                            (state_.fpr[fs] ^ state_.fpr[ft]) &
                            0x80000000u;
                        state_.fpr[fd] = sign | 0x7F7FFFFFu;
                    } else {
                        set_fd(a / b);
                    }
                    break;
                case 0x04u:
                    set_fd(std::sqrt(std::fabs(b)));
                    break;
                case 0x05u:
                    state_.fpr[fd] =
                        state_.fpr[fs] & 0x7FFFFFFFu;
                    break;
                case 0x06u:
                    state_.fpr[fd] = state_.fpr[fs];
                    break;
                case 0x07u:
                    state_.fpr[fd] =
                        state_.fpr[fs] ^ 0x80000000u;
                    break;
                case 0x16u:
                    if ((state_.fpr[ft] & 0x7FFFFFFFu) == 0u) {
                        const u32 sign =
                            (state_.fpr[fs] ^ state_.fpr[ft]) &
                            0x80000000u;
                        state_.fpr[fd] = sign | 0x7F7FFFFFu;
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
                case 0x24u: {
                    if ((state_.fpr[fs] & 0x7F800000u) <=
                        0x4E800000u) {
                        const double value =
                            static_cast<double>(a);
                        if (value > 2147483647.0) {
                            state_.fpr[fd] = 0x7FFFFFFFu;
                        } else if (value < -2147483648.0) {
                            state_.fpr[fd] = 0x80000000u;
                        } else {
                            state_.fpr[fd] = static_cast<u32>(
                                static_cast<s32>(value));
                        }
                    } else {
                        state_.fpr[fd] =
                            (state_.fpr[fs] & 0x80000000u)
                                ? 0x80000000u
                                : 0x7FFFFFFFu;
                    }
                    break;
                }
                case 0x28u:
                    state_.fpr[fd] =
                        (a >= b) ? state_.fpr[fs] : state_.fpr[ft];
                    break;
                case 0x29u:
                    state_.fpr[fd] =
                        (a <= b) ? state_.fpr[fs] : state_.fpr[ft];
                    break;
                case 0x30u: set_cond(false); break;
                case 0x32u: set_cond(a == b); break;
                case 0x34u: set_cond(a < b); break;
                case 0x36u: set_cond(a <= b); break;
                default:
                    cop1_handled = false;
                    break;
                }
            } else {
                cop1_handled = false;
            }
            if (!cop1_handled) handled = false;
        } else if (opcode == 0x2Fu || opcode == 0x33u) {
            // CACHE / PREF are bootstrap no-ops.
        } else if (opcode == 0x01u) {
            if (rt <= 0x03u || (rt >= 0x10u && rt <= 0x13u)) {
                const bool less_zero = gpr_s64(rs) < 0;
                const bool take =
                    (rt == 0x00u || rt == 0x02u ||
                     rt == 0x10u || rt == 0x12u)
                        ? less_zero
                        : !less_zero;
                const bool likely = rt == 0x02u || rt == 0x03u ||
                                    rt == 0x12u || rt == 0x13u;
                const bool link = rt >= 0x10u && rt <= 0x13u;

                if (link) {
                    write_gpr_word(31u, expected_pc + 8u);
                }
                if (take) {
                    state_.next_pc = branch_target(expected_pc, imm);
                    next_is_delay_slot_ = true;
                } else if (likely) {
                    state_.pc = expected_pc + 8u;
                    state_.next_pc = expected_pc + 12u;
                    next_is_delay_slot_ = false;
                } else {
                    state_.next_pc = expected_pc + 8u;
                    next_is_delay_slot_ = true;
                }
            } else if (rt == 0x18u) { // MTSAB
                state_.sa =
                    (static_cast<u32>(gpr_u64(rs)) & 0xFu) ^
                    (static_cast<u32>(imm) & 0xFu);
            } else if (rt == 0x19u) { // MTSAH
                state_.sa =
                    ((static_cast<u32>(gpr_u64(rs)) & 0x7u) ^
                     (static_cast<u32>(imm) & 0x7u)) << 1u;
            } else {
                handled = false;
            }
        } else if (opcode == 0x00u) {
            switch (funct) {
            case 0x00u:
                write_gpr_word(
                    rd, static_cast<u32>(gpr_u64(rt)) << sa);
                break;
            case 0x02u:
                write_gpr_word(
                    rd, static_cast<u32>(gpr_u64(rt)) >> sa);
                break;
            case 0x03u:
                write_gpr_word(
                    rd,
                    static_cast<u32>(
                        static_cast<s32>(
                            static_cast<u32>(gpr_u64(rt))) >> sa));
                break;
            case 0x04u:
                write_gpr_word(
                    rd,
                    static_cast<u32>(gpr_u64(rt)) <<
                        (static_cast<u32>(gpr_u64(rs)) & 31u));
                break;
            case 0x06u:
                write_gpr_word(
                    rd,
                    static_cast<u32>(gpr_u64(rt)) >>
                        (static_cast<u32>(gpr_u64(rs)) & 31u));
                break;
            case 0x07u:
                write_gpr_word(
                    rd,
                    static_cast<u32>(
                        static_cast<s32>(
                            static_cast<u32>(gpr_u64(rt))) >>
                        (static_cast<u32>(gpr_u64(rs)) & 31u)));
                break;
            case 0x08u: // JR
                state_.next_pc = static_cast<u32>(gpr_u64(rs));
                next_is_delay_slot_ = true;
                break;
            case 0x09u: { // JALR
                const u32 target = static_cast<u32>(gpr_u64(rs));
                write_gpr_word(rd, expected_pc + 8u);
                state_.next_pc = target;
                next_is_delay_slot_ = true;
                break;
            }
            case 0x0Au:
                if (gpr_u64(rt) == 0u) write_gpr64(rd, gpr_u64(rs));
                break;
            case 0x0Bu:
                if (gpr_u64(rt) != 0u) write_gpr64(rd, gpr_u64(rs));
                break;
            case 0x0Fu:
                break; // SYNC
            case 0x10u:
                write_gpr64(rd, state_.hi);
                break;
            case 0x11u:
                state_.hi = gpr_u64(rs);
                break;
            case 0x12u:
                write_gpr64(rd, state_.lo);
                break;
            case 0x13u:
                state_.lo = gpr_u64(rs);
                break;
            case 0x14u:
                write_gpr64(
                    rd, gpr_u64(rt) << (gpr_u64(rs) & 63u));
                break;
            case 0x16u:
                write_gpr64(
                    rd, gpr_u64(rt) >> (gpr_u64(rs) & 63u));
                break;
            case 0x17u:
                write_gpr64(
                    rd,
                    static_cast<u64>(
                        static_cast<s64>(gpr_u64(rt)) >>
                        (gpr_u64(rs) & 63u)));
                break;
            case 0x18u: // MULT
                multiply_signed32(
                    static_cast<u32>(gpr_u64(rs)),
                    static_cast<u32>(gpr_u64(rt)),
                    state_.lo,
                    state_.hi);
                // R5900 MULT/MULTU also write the low result to rd.
                write_gpr64(rd, state_.lo);
                break;
            case 0x19u: // MULTU
                multiply_unsigned32(
                    static_cast<u32>(gpr_u64(rs)),
                    static_cast<u32>(gpr_u64(rt)),
                    state_.lo,
                    state_.hi);
                write_gpr64(rd, state_.lo);
                break;
            case 0x1Au: // DIV
                divide_signed32(
                    static_cast<u32>(gpr_u64(rs)),
                    static_cast<u32>(gpr_u64(rt)),
                    state_.lo,
                    state_.hi);
                break;
            case 0x1Bu: // DIVU
                divide_unsigned32(
                    static_cast<u32>(gpr_u64(rs)),
                    static_cast<u32>(gpr_u64(rt)),
                    state_.lo,
                    state_.hi);
                break;
            case 0x21u:
                write_gpr_word(
                    rd,
                    static_cast<u32>(gpr_u64(rs)) +
                        static_cast<u32>(gpr_u64(rt)));
                break;
            case 0x23u:
                write_gpr_word(
                    rd,
                    static_cast<u32>(gpr_u64(rs)) -
                        static_cast<u32>(gpr_u64(rt)));
                break;
            case 0x24u:
                write_gpr64(rd, gpr_u64(rs) & gpr_u64(rt));
                break;
            case 0x25u:
                write_gpr64(rd, gpr_u64(rs) | gpr_u64(rt));
                break;
            case 0x26u:
                write_gpr64(rd, gpr_u64(rs) ^ gpr_u64(rt));
                break;
            case 0x27u:
                write_gpr64(rd, ~(gpr_u64(rs) | gpr_u64(rt)));
                break;
            case 0x28u:
                write_gpr64(rd, state_.sa);
                break;
            case 0x29u:
                state_.sa = static_cast<u32>(gpr_u64(rs));
                break;
            case 0x2Au:
                write_gpr64(
                    rd, gpr_s64(rs) < gpr_s64(rt) ? 1u : 0u);
                break;
            case 0x2Bu:
                write_gpr64(
                    rd, gpr_u64(rs) < gpr_u64(rt) ? 1u : 0u);
                break;
            case 0x2Du:
                write_gpr64(rd, gpr_u64(rs) + gpr_u64(rt));
                break;
            case 0x2Fu:
                write_gpr64(rd, gpr_u64(rs) - gpr_u64(rt));
                break;
            case 0x38u:
                write_gpr64(rd, gpr_u64(rt) << sa);
                break;
            case 0x3Au:
                write_gpr64(rd, gpr_u64(rt) >> sa);
                break;
            case 0x3Bu:
                write_gpr64(
                    rd,
                    static_cast<u64>(
                        static_cast<s64>(gpr_u64(rt)) >> sa));
                break;
            case 0x3Cu:
                write_gpr64(rd, gpr_u64(rt) << (sa + 32u));
                break;
            case 0x3Eu:
                write_gpr64(rd, gpr_u64(rt) >> (sa + 32u));
                break;
            case 0x3Fu:
                write_gpr64(
                    rd,
                    static_cast<u64>(
                        static_cast<s64>(gpr_u64(rt)) >>
                        (sa + 32u)));
                break;
            default:
                handled = false;
                break;
            }
        } else {
            handled = false;
        }

        if (!handled) {
            ++(*fast_prefix_fallback_opcodes_)[opcode & 63u];
            if (opcode == 0x12u) {
                bool found = false;
                for (u32 i = 0u;
                     i < fast_prefix_cop2_fallback_count_;
                     ++i) {
                    if ((*fast_prefix_cop2_fallback_words_)[i] ==
                        instruction) {
                        ++(*fast_prefix_cop2_fallback_counts_)[i];
                        found = true;
                        break;
                    }
                }
                if (!found &&
                    fast_prefix_cop2_fallback_count_ <
                        fast_prefix_cop2_fallback_words_->size()) {
                    const u32 index =
                        fast_prefix_cop2_fallback_count_++;
                    (*fast_prefix_cop2_fallback_words_)[index] =
                        instruction;
                    (*fast_prefix_cop2_fallback_counts_)[index] = 1u;
                }
            }
            break;
        }

        current_is_delay_slot_ = was_delay_slot;
        state_.last_pc = expected_pc;
        state_.last_instruction = instruction;

        const bool redirected_pc =
            state_.pc != expected_pc;
        if (!redirected_pc) {
            state_.pc = old_next_pc;
            if (!next_is_delay_slot_) {
                state_.next_pc = old_next_pc + 4u;
            }
        }
        state_.gpr[0] = {};
        ++state_.instructions_executed;
        ++state_.cop0[9];
        if (state_.cop0[9] == state_.cop0[11]) {
            state_.cop0[13] |= 0x00008000u;
        }
        if (stop_after_instruction) {
            ++retired;
            break;
        }
    }

    return retired;
}

u32 EeCpu::run_native_block(
    u32 pc,
    u32 page_generation,
    const u32* instructions,
    u32 instruction_count,
    u32 maximum_instructions,
    const u8* ram_data,
    u32* page_generations) {
    if (halted_ || next_is_delay_slot_ ||
        state_.pc != pc ||
        instructions == nullptr ||
        instruction_count == 0u ||
        maximum_instructions == 0u) {
        return 0u;
    }

    bool control_flow = false;
    const u32 retired = jit_.execute_block(
        state_,
        pc,
        page_generation,
        instructions,
        instruction_count,
        maximum_instructions,
        ram_data,
        page_generations,
        control_flow);
    if (retired == 0u) return 0u;

    state_.last_pc = pc + (retired - 1u) * 4u;
    state_.last_instruction = instructions[retired - 1u];
    if (!control_flow) {
        state_.pc = pc + retired * 4u;
        state_.next_pc = state_.pc + 4u;
    }
    state_.gpr[0] = {};
    state_.instructions_executed += retired;
    state_.cop0[9] += retired;
    if (state_.cop0[9] == state_.cop0[11]) {
        state_.cop0[13] |= 0x00008000u;
    }
    current_is_delay_slot_ = false;
    next_is_delay_slot_ = false;
    return retired;
}

bool EeCpu::step_internal(
    std::string& error,
    bool quiet,
    const u32* prefetched_instruction,
    bool skip_interrupt_check) {
    error.clear();

    if (halted_) {
        error = halt_reason_;
        return false;
    }

    const u32 pc = state_.pc;
    const u32 old_next_pc = state_.next_pc;
    current_is_delay_slot_ = next_is_delay_slot_;
    next_is_delay_slot_ = false;

    if (!quiet) {
        if (bus_.intc_pending()) state_.cop0[13] |= 0x00000400u;
        else state_.cop0[13] &= ~0x00000400u;
        if (bus_.dmac_pending()) state_.cop0[13] |= 0x00000800u;
        else state_.cop0[13] &= ~0x00000800u;
    }

    const u32 status = state_.cop0[12];
    if (!skip_interrupt_check &&
        (state_.cop0[13] & status & 0x0000FF00u) != 0 &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0) {
        raise_exception(0u, pc, current_is_delay_slot_);
        ++state_.instructions_executed;
        ++state_.cop0[9];
        if (state_.cop0[9] == state_.cop0[11]) {
            state_.cop0[13] |= 0x00008000u;
        }
        if (!quiet) bus_.tick(1);
        return true;
    }

    u32 instruction = 0;
    if (prefetched_instruction != nullptr) {
        instruction = *prefetched_instruction;
    } else {
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
            if (!quiet) bus_.tick(1);
            return true;
        }
        if (!bus_.fetch32(fetch_address, instruction)) {
            return fail(
                pc,
                0,
                "Instruction fetch fault from " + hex32(pc),
                error);
        }
    }
    state_.last_pc = pc;
    state_.last_instruction = instruction;
    state_.pc = old_next_pc;
    state_.next_pc = old_next_pc + 4u;

    // Firmware spends long stretches in wait loops padded with real NOPs.
    // Complete those instructions before building the memory-operation
    // helpers and entering the full decoder.
    if (instruction == 0u) {
        ++state_.instructions_executed;
        ++state_.cop0[9];
        if (state_.cop0[9] == state_.cop0[11]) {
            state_.cop0[13] |= 0x00008000u;
        }
        if (!quiet) bus_.tick(1);
        return true;
    }

    const u32 opcode = instruction >> 26;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const s16 imm = immediate(instruction);

    // These BIOS staples do not access memory or the coprocessors. Retire
    // them before constructing the full decoder's memory/fault helpers.
    if (!jit_enabled_ && (
            opcode == 0x02u || opcode == 0x03u ||
            opcode == 0x04u || opcode == 0x05u ||
            opcode == 0x06u || opcode == 0x07u ||
            opcode == 0x09u || opcode == 0x0Au ||
            opcode == 0x0Bu || opcode == 0x0Cu ||
            opcode == 0x0Du || opcode == 0x0Eu ||
            opcode == 0x0Fu || opcode == 0x14u ||
            opcode == 0x15u || opcode == 0x16u ||
            opcode == 0x17u || opcode == 0x19u ||
            opcode == 0x2Fu || opcode == 0x33u)) {
        switch (opcode) {
        case 0x02u: // J
            state_.next_pc =
                ((pc + 4u) & 0xF0000000u) |
                ((instruction & 0x03FFFFFFu) << 2);
            next_is_delay_slot_ = true;
            break;
        case 0x03u: // JAL
            write_gpr_word(31u, pc + 8u);
            state_.next_pc =
                ((pc + 4u) & 0xF0000000u) |
                ((instruction & 0x03FFFFFFu) << 2);
            next_is_delay_slot_ = true;
            break;
        case 0x04u:
        case 0x05u:
            if ((gpr_u64(rs) == gpr_u64(rt)) == (opcode == 0x04u)) {
                state_.next_pc = branch_target(pc, imm);
            }
            next_is_delay_slot_ = true;
            break;
        case 0x06u:
            if (gpr_s64(rs) <= 0) state_.next_pc = branch_target(pc, imm);
            next_is_delay_slot_ = true;
            break;
        case 0x07u:
            if (gpr_s64(rs) > 0) state_.next_pc = branch_target(pc, imm);
            next_is_delay_slot_ = true;
            break;
        case 0x09u:
            write_gpr_word(rt, static_cast<u32>(gpr_u64(rs)) +
                               static_cast<u32>(static_cast<s32>(imm)));
            break;
        case 0x0Au:
            write_gpr64(rt, gpr_s64(rs) < static_cast<s64>(imm) ? 1u : 0u);
            break;
        case 0x0Bu:
            write_gpr64(
                rt,
                gpr_u64(rs) < static_cast<u64>(static_cast<s64>(imm))
                    ? 1u : 0u);
            break;
        case 0x0Cu:
            write_gpr64(rt, gpr_u64(rs) &
                                static_cast<u64>(instruction & 0xFFFFu));
            break;
        case 0x0Du:
            write_gpr64(rt, gpr_u64(rs) |
                                static_cast<u64>(instruction & 0xFFFFu));
            break;
        case 0x0Eu:
            write_gpr64(rt, gpr_u64(rs) ^
                                static_cast<u64>(instruction & 0xFFFFu));
            break;
        case 0x0Fu:
            write_gpr_word(rt, (instruction & 0xFFFFu) << 16);
            break;
        case 0x14u:
        case 0x15u: {
            const bool taken =
                (gpr_u64(rs) == gpr_u64(rt)) == (opcode == 0x14u);
            if (taken) {
                state_.next_pc = branch_target(pc, imm);
                next_is_delay_slot_ = true;
            } else {
                branch_likely_not_taken(pc);
            }
            break;
        }
        case 0x16u:
        case 0x17u: {
            const bool taken =
                opcode == 0x16u ? gpr_s64(rs) <= 0 : gpr_s64(rs) > 0;
            if (taken) {
                state_.next_pc = branch_target(pc, imm);
                next_is_delay_slot_ = true;
            } else {
                branch_likely_not_taken(pc);
            }
            break;
        }
        case 0x19u:
            write_gpr64(
                rt,
                gpr_u64(rs) + static_cast<u64>(static_cast<s64>(imm)));
            break;
        case 0x2Fu: // CACHE
        case 0x33u: // PREF
            break;
        }
        state_.gpr[0] = {};
        ++state_.instructions_executed;
        ++state_.cop0[9];
        if (state_.cop0[9] == state_.cop0[11]) {
            state_.cop0[13] |= 0x00008000u;
        }
        if (!quiet) bus_.tick(1);
        return true;
    }

    // Common register-only SPECIAL instructions need none of the generic
    // memory/fault decoder's setup. Keep this path instruction-exact.
    if (!jit_enabled_ && opcode == 0u) {
        const u32 funct = instruction & 63u;
        const u32 rd = (instruction >> 11) & 31u;
        const u32 sa = (instruction >> 6) & 31u;
        switch (funct) {
        case 0x00u:
            write_gpr_word(rd, static_cast<u32>(gpr_u64(rt)) << sa);
            break;
        case 0x02u:
            write_gpr_word(rd, static_cast<u32>(gpr_u64(rt)) >> sa);
            break;
        case 0x03u:
            write_gpr_word(rd, static_cast<u32>(
                static_cast<s32>(static_cast<u32>(gpr_u64(rt))) >> sa));
            break;
        case 0x04u:
            write_gpr_word(
                rd,
                static_cast<u32>(gpr_u64(rt)) <<
                    (static_cast<u32>(gpr_u64(rs)) & 31u));
            break;
        case 0x06u:
            write_gpr_word(
                rd,
                static_cast<u32>(gpr_u64(rt)) >>
                    (static_cast<u32>(gpr_u64(rs)) & 31u));
            break;
        case 0x07u:
            write_gpr_word(
                rd,
                static_cast<u32>(
                    static_cast<s32>(static_cast<u32>(gpr_u64(rt))) >>
                    (static_cast<u32>(gpr_u64(rs)) & 31u)));
            break;
        case 0x08u:
            state_.next_pc = static_cast<u32>(gpr_u64(rs));
            next_is_delay_slot_ = true;
            break;
        case 0x09u:
            write_gpr_word(rd, pc + 8u);
            state_.next_pc = static_cast<u32>(gpr_u64(rs));
            next_is_delay_slot_ = true;
            break;
        case 0x0Au:
            if (gpr_u64(rt) == 0u) write_gpr64(rd, gpr_u64(rs));
            break;
        case 0x0Bu:
            if (gpr_u64(rt) != 0u) write_gpr64(rd, gpr_u64(rs));
            break;
        case 0x0Fu:
            break;
        case 0x10u: write_gpr64(rd, state_.hi); break;
        case 0x11u: state_.hi = gpr_u64(rs); break;
        case 0x12u: write_gpr64(rd, state_.lo); break;
        case 0x13u: state_.lo = gpr_u64(rs); break;
        case 0x14u:
            write_gpr64(rd, gpr_u64(rt) << (gpr_u64(rs) & 63u));
            break;
        case 0x16u:
            write_gpr64(rd, gpr_u64(rt) >> (gpr_u64(rs) & 63u));
            break;
        case 0x17u:
            write_gpr64(
                rd,
                static_cast<u64>(
                    static_cast<s64>(gpr_u64(rt)) >>
                    (gpr_u64(rs) & 63u)));
            break;
        case 0x21u:
            write_gpr_word(
                rd,
                static_cast<u32>(gpr_u64(rs)) +
                    static_cast<u32>(gpr_u64(rt)));
            break;
        case 0x23u:
            write_gpr_word(
                rd,
                static_cast<u32>(gpr_u64(rs)) -
                    static_cast<u32>(gpr_u64(rt)));
            break;
        case 0x24u: write_gpr64(rd, gpr_u64(rs) & gpr_u64(rt)); break;
        case 0x25u: write_gpr64(rd, gpr_u64(rs) | gpr_u64(rt)); break;
        case 0x26u: write_gpr64(rd, gpr_u64(rs) ^ gpr_u64(rt)); break;
        case 0x27u: write_gpr64(rd, ~(gpr_u64(rs) | gpr_u64(rt))); break;
        case 0x28u: write_gpr64(rd, state_.sa); break;
        case 0x29u: state_.sa = static_cast<u32>(gpr_u64(rs)); break;
        case 0x2Au:
            write_gpr64(rd, gpr_s64(rs) < gpr_s64(rt) ? 1u : 0u);
            break;
        case 0x2Bu:
            write_gpr64(rd, gpr_u64(rs) < gpr_u64(rt) ? 1u : 0u);
            break;
        case 0x2Du: write_gpr64(rd, gpr_u64(rs) + gpr_u64(rt)); break;
        case 0x2Fu: write_gpr64(rd, gpr_u64(rs) - gpr_u64(rt)); break;
        case 0x38u: write_gpr64(rd, gpr_u64(rt) << sa); break;
        case 0x3Au: write_gpr64(rd, gpr_u64(rt) >> sa); break;
        case 0x3Bu:
            write_gpr64(
                rd,
                static_cast<u64>(static_cast<s64>(gpr_u64(rt)) >> sa));
            break;
        case 0x3Cu: write_gpr64(rd, gpr_u64(rt) << (sa + 32u)); break;
        case 0x3Eu: write_gpr64(rd, gpr_u64(rt) >> (sa + 32u)); break;
        case 0x3Fu:
            write_gpr64(
                rd,
                static_cast<u64>(
                    static_cast<s64>(gpr_u64(rt)) >> (sa + 32u)));
            break;
        default:
            goto generic_decode;
        }
        state_.gpr[0] = {};
        ++state_.instructions_executed;
        ++state_.cop0[9];
        if (state_.cop0[9] == state_.cop0[11]) state_.cop0[13] |= 0x00008000u;
        if (!quiet) bus_.tick(1);
        return true;
    }

    // REGIMM branches and shift-address helpers are common in BIOS/OSDSYS.
    // Dispatch them before constructing the generic memory helper stack.
    if (!jit_enabled_ && opcode == 0x01u) {
        if (execute_regimm(pc, instruction, error)) {
            state_.gpr[0] = {};
            ++state_.instructions_executed;
            ++state_.cop0[9];
            if (state_.cop0[9] == state_.cop0[11]) {
                state_.cop0[13] |= 0x00008000u;
            }
            if (!quiet) bus_.tick(1);
            return true;
        }
        goto generic_decode;
    }

    // The system's predecoded quiet path has already proven that these
    // accesses land entirely in EE main RAM. Handle the common scalar memory
    // operations here instead of constructing the generic translation/fault
    // helper stack for every load/store.
    if (prefetched_instruction != nullptr && !jit_enabled_) {
        const u32 address = static_cast<u32>(
            gpr_u64(rs) + static_cast<u64>(static_cast<s64>(imm)));
        bool handled = true;
        bool access_ok = true;
        switch (opcode) {
        case 0x1Eu: { // LQ
            const u32 aligned = address & ~0x0Fu;
            u64 lo = 0;
            u64 hi = 0;
            access_ok =
                bus_.read64(aligned, lo) &&
                bus_.read64(aligned + 8u, hi);
            if (access_ok && rt != 0u) {
                state_.gpr[rt].lo = lo;
                state_.gpr[rt].hi = hi;
            }
            break;
        }
        case 0x1Fu: { // SQ
            const u32 aligned = address & ~0x0Fu;
            access_ok =
                bus_.write64(aligned, state_.gpr[rt].lo) &&
                bus_.write64(aligned + 8u, state_.gpr[rt].hi);
            break;
        }
        case 0x20u: { // LB
            u8 value = 0;
            access_ok = bus_.read8(address, value);
            if (access_ok) write_gpr64(
                rt, static_cast<u64>(static_cast<s64>(
                    static_cast<s8>(value))));
            break;
        }
        case 0x21u: { // LH
            u16 value = 0;
            access_ok = bus_.read16(address, value);
            if (access_ok) write_gpr64(
                rt, static_cast<u64>(static_cast<s64>(
                    static_cast<s16>(value))));
            break;
        }
        case 0x23u: { // LW
            u32 value = 0;
            if (hot_sif_getreg_diag_inflight_ &&
                hot_sif_getreg_read_offset_ == 0u &&
                EeBus::to_physical(address) == 0x1000F230u) {
                hot_sif_getreg_read_offset_ =
                    static_cast<u32>(
                        state_.instructions_executed -
                        hot_sif_getreg_diag_start_);
                hot_sif_getreg_diag_inflight_ = false;
            }
            access_ok = bus_.read32(address, value);
            if (access_ok) write_gpr_word(rt, value);
            break;
        }
        case 0x24u: { // LBU
            u8 value = 0;
            access_ok = bus_.read8(address, value);
            if (access_ok) write_gpr64(rt, value);
            break;
        }
        case 0x25u: { // LHU
            u16 value = 0;
            access_ok = bus_.read16(address, value);
            if (access_ok) write_gpr64(rt, value);
            break;
        }
        case 0x27u: { // LWU
            u32 value = 0;
            access_ok = bus_.read32(address, value);
            if (access_ok) write_gpr64(rt, value);
            break;
        }
        case 0x28u: // SB
            access_ok = bus_.write8(address, static_cast<u8>(gpr_u64(rt)));
            break;
        case 0x29u: // SH
            access_ok = bus_.write16(address, static_cast<u16>(gpr_u64(rt)));
            break;
        case 0x2Bu: // SW
            access_ok = bus_.write32(address, static_cast<u32>(gpr_u64(rt)));
            break;
        case 0x30u: { // LL
            u32 value = 0;
            access_ok = bus_.read32(address, value);
            if (access_ok) write_gpr_word(rt, value);
            break;
        }
        case 0x31u: { // LWC1
            u32 value = 0;
            access_ok = bus_.read32(address, value);
            if (access_ok) state_.fpr[rt] = value;
            break;
        }
        case 0x36u: { // LQC2
            const u32 aligned = address & ~0x0Fu;
            u64 lo = 0;
            u64 hi = 0;
            access_ok =
                bus_.read64(aligned, lo) &&
                bus_.read64(aligned + 8u, hi);
            if (access_ok && rt != 0u) {
                state_.vu_vf[rt].lo = lo;
                state_.vu_vf[rt].hi = hi;
            }
            break;
        }
        case 0x34u: // LLD
        case 0x37u: { // LD
            u64 value = 0;
            access_ok = bus_.read64(address, value);
            if (access_ok) write_gpr64(rt, value);
            break;
        }
        case 0x38u: // SC
            access_ok = bus_.write32(address, static_cast<u32>(gpr_u64(rt)));
            if (access_ok) write_gpr_word(rt, 1u);
            break;
        case 0x39u: // SWC1
            access_ok = bus_.write32(address, state_.fpr[rt]);
            break;
        case 0x3Cu: // SCD
            access_ok = bus_.write64(address, gpr_u64(rt));
            if (access_ok) write_gpr64(rt, 1u);
            break;
        case 0x3Eu: { // SQC2
            const u32 aligned = address & ~0x0Fu;
            access_ok =
                bus_.write64(aligned, state_.vu_vf[rt].lo) &&
                bus_.write64(aligned + 8u, state_.vu_vf[rt].hi);
            break;
        }
        case 0x3Fu: // SD
            access_ok = bus_.write64(address, gpr_u64(rt));
            break;
        default:
            handled = false;
            break;
        }

        if (handled) {
            if (!access_ok) {
                state_.pc = pc;
                state_.next_pc = old_next_pc;
                return fail(
                    pc,
                    instruction,
                    "Predecoded EE RAM access failed",
                    error);
            }
            state_.gpr[0] = {};
            ++state_.instructions_executed;
            ++state_.cop0[9];
            if (state_.cop0[9] == state_.cop0[11]) {
                state_.cop0[13] |= 0x00008000u;
            }
            if (!quiet) bus_.tick(1);
            return true;
        }
    }

generic_decode:
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

    if (!jit_enabled_ || !jit_.execute(state_, instruction)) {
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
        if (hot_sif_getreg_diag_inflight_ &&
            hot_sif_getreg_read_offset_ == 0u &&
            EeBus::to_physical(address) == 0x1000F230u) {
            hot_sif_getreg_read_offset_ =
                static_cast<u32>(
                    state_.instructions_executed -
                    hot_sif_getreg_diag_start_);
            hot_sif_getreg_diag_inflight_ = false;
        }
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
    case 0x36: { // LQC2
        const u32 address = effective_address() & ~0x0Fu;
        u64 lo = 0;
        u64 hi = 0;
        if (!read64_mem(address, lo) ||
            !read64_mem(address + 8u, hi)) {
            ok = load_fault("LQC2", address);
        } else if (rt != 0u) {
            state_.vu_vf[rt].lo = lo;
            state_.vu_vf[rt].hi = hi;
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
    case 0x3E: { // SQC2
        const u32 address = effective_address() & ~0x0Fu;
        if (!write64_mem(address, state_.vu_vf[rt].lo) ||
            !write64_mem(address + 8u, state_.vu_vf[rt].hi)) {
            ok = fail(
                pc,
                instruction,
                "SQC2 fault to " + hex32(address),
                error);
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
    if (!quiet) bus_.tick(1);
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
