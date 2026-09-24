#include "core/iop/iop_cpu.h"

#include "core/iop/iop_bus.h"

#include <algorithm>
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

bool add_overflow(s32 lhs, s32 rhs, s32& result) {
    const s64 wide = static_cast<s64>(lhs) + static_cast<s64>(rhs);
    result = static_cast<s32>(wide);
    return wide < std::numeric_limits<s32>::min() ||
           wide > std::numeric_limits<s32>::max();
}

bool sub_overflow(s32 lhs, s32 rhs, s32& result) {
    const s64 wide = static_cast<s64>(lhs) - static_cast<s64>(rhs);
    result = static_cast<s32>(wide);
    return wide < std::numeric_limits<s32>::min() ||
           wide > std::numeric_limits<s32>::max();
}

} // namespace

void IopCpu::reset(u32 entry_point) {
    state_ = {};
    state_.pc = entry_point;
    state_.next_pc = entry_point + 4;
    state_.cop0[12] = 0x00400000u; // BEV
    state_.cop0[15] = 0x0000001Fu; // IOP R3000A PRId

    pending_load_ = {};
    next_load_ = {};
    direct_write_mask_ = 0;
    next_is_delay_slot_ = false;
    halted_ = false;
    halt_reason_.clear();
}

void IopCpu::clear_halt() {
    halted_ = false;
    halt_reason_.clear();
}

s16 IopCpu::immediate(u32 instruction) {
    return static_cast<s16>(instruction & 0xFFFFu);
}

u32 IopCpu::branch_target(u32 pc, s16 imm) {
    return pc + 4u + static_cast<u32>(static_cast<s32>(imm) * 4);
}

void IopCpu::write_gpr(u32 index, u32 value) {
    index &= 31u;
    if (index == 0) {
        return;
    }
    state_.gpr[index] = value;
    direct_write_mask_ |= (1u << index);
}

void IopCpu::schedule_load(u32 index, u32 value) {
    index &= 31u;
    if (index == 0) {
        return;
    }
    next_load_ = PendingLoad{true, index, value};
}

u32 IopCpu::load_merge_base(u32 index) const {
    index &= 31u;
    if (pending_load_.valid && pending_load_.reg == index) {
        return pending_load_.value;
    }
    return state_.gpr[index];
}

bool IopCpu::fail(
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

void IopCpu::raise_exception(
    u32 code,
    u32 pc,
    bool in_delay_slot) {
    const u32 exception = (code >> 2) & 31u;
    ++state_.exception_counts[exception];

    state_.cop0[13] &= ~0x8000007Fu;
    state_.cop0[13] |= code & 0x7Cu;

    if (in_delay_slot) {
        state_.cop0[13] |= 0x80000000u;
        state_.cop0[14] = pc - 4u;
    } else {
        state_.cop0[14] = pc;
    }

    const u32 status = state_.cop0[12];
    state_.cop0[12] =
        (status & ~0x3Fu) | ((status & 0x0Fu) << 2);

    const u32 vector =
        (status & 0x00400000u) != 0
            ? 0xBFC00180u
            : 0x80000080u;

    state_.pc = vector;
    state_.next_pc = vector + 4u;
    next_is_delay_slot_ = false;
}

bool IopCpu::execute_special(
    u32 pc,
    u32 instruction,
    bool in_delay_slot,
    std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;
    const u32 sa = (instruction >> 6) & 31u;
    const u32 funct = instruction & 63u;

    switch (funct) {
    case 0x00: // SLL / NOP
        write_gpr(rd, state_.gpr[rt] << sa);
        return true;
    case 0x02: // SRL
        write_gpr(rd, state_.gpr[rt] >> sa);
        return true;
    case 0x03: // SRA
        write_gpr(
            rd,
            static_cast<u32>(
                static_cast<s32>(state_.gpr[rt]) >> sa));
        return true;
    case 0x04: // SLLV
        write_gpr(rd, state_.gpr[rt] << (state_.gpr[rs] & 31u));
        return true;
    case 0x06: // SRLV
        write_gpr(rd, state_.gpr[rt] >> (state_.gpr[rs] & 31u));
        return true;
    case 0x07: // SRAV
        write_gpr(
            rd,
            static_cast<u32>(
                static_cast<s32>(state_.gpr[rt]) >>
                (state_.gpr[rs] & 31u)));
        return true;
    case 0x08: // JR
        state_.next_pc = state_.gpr[rs];
        next_is_delay_slot_ = true;
        return true;
    case 0x09: // JALR
        write_gpr(rd, pc + 8u);
        state_.next_pc = state_.gpr[rs];
        next_is_delay_slot_ = true;
        return true;
    case 0x0C: { // SYSCALL
        auto& record =
            state_.recent_syscalls[state_.recent_syscall_next];
        record.instruction = state_.instructions_executed;
        record.pc = pc;
        record.encoded = (instruction >> 6) & 0xFFFFFu;
        record.v0 = state_.gpr[2];
        for (u32 i = 0; i < record.args.size(); ++i) {
            record.args[i] = state_.gpr[4u + i];
        }
        state_.recent_syscall_next =
            (state_.recent_syscall_next + 1u) %
            static_cast<u32>(state_.recent_syscalls.size());
        state_.recent_syscall_count = std::min(
            state_.recent_syscall_count + 1u,
            static_cast<u32>(state_.recent_syscalls.size()));
        raise_exception(0x20u, pc, in_delay_slot);
        return true;
    }
    case 0x0D: // BREAK
        raise_exception(0x24u, pc, in_delay_slot);
        return true;
    case 0x10: // MFHI
        write_gpr(rd, state_.hi);
        return true;
    case 0x11: // MTHI
        state_.hi = state_.gpr[rs];
        return true;
    case 0x12: // MFLO
        write_gpr(rd, state_.lo);
        return true;
    case 0x13: // MTLO
        state_.lo = state_.gpr[rs];
        return true;
    case 0x18: { // MULT
        const s64 result =
            static_cast<s64>(static_cast<s32>(state_.gpr[rs])) *
            static_cast<s64>(static_cast<s32>(state_.gpr[rt]));
        state_.lo = static_cast<u32>(result);
        state_.hi = static_cast<u32>(static_cast<u64>(result) >> 32);
        return true;
    }
    case 0x19: { // MULTU
        const u64 result =
            static_cast<u64>(state_.gpr[rs]) *
            static_cast<u64>(state_.gpr[rt]);
        state_.lo = static_cast<u32>(result);
        state_.hi = static_cast<u32>(result >> 32);
        return true;
    }
    case 0x1A: { // DIV
        const s32 lhs = static_cast<s32>(state_.gpr[rs]);
        const s32 rhs = static_cast<s32>(state_.gpr[rt]);
        if (state_.gpr[rs] == 0x80000000u &&
            state_.gpr[rt] == 0xFFFFFFFFu) {
            state_.lo = 0x80000000u;
            state_.hi = 0;
        } else if (rhs != 0) {
            state_.lo = static_cast<u32>(lhs / rhs);
            state_.hi = static_cast<u32>(lhs % rhs);
        } else {
            state_.lo = lhs < 0 ? 1u : 0xFFFFFFFFu;
            state_.hi = static_cast<u32>(lhs);
        }
        return true;
    }
    case 0x1B: // DIVU
        if (state_.gpr[rt] != 0) {
            state_.lo = state_.gpr[rs] / state_.gpr[rt];
            state_.hi = state_.gpr[rs] % state_.gpr[rt];
        } else {
            state_.lo = 0xFFFFFFFFu;
            state_.hi = state_.gpr[rs];
        }
        return true;
    case 0x20: { // ADD
        s32 result = 0;
        if (add_overflow(
                static_cast<s32>(state_.gpr[rs]),
                static_cast<s32>(state_.gpr[rt]),
                result)) {
            raise_exception(0x30u, pc, in_delay_slot);
        } else {
            write_gpr(rd, static_cast<u32>(result));
        }
        return true;
    }
    case 0x21: // ADDU
        write_gpr(rd, state_.gpr[rs] + state_.gpr[rt]);
        return true;
    case 0x22: { // SUB
        s32 result = 0;
        if (sub_overflow(
                static_cast<s32>(state_.gpr[rs]),
                static_cast<s32>(state_.gpr[rt]),
                result)) {
            raise_exception(0x30u, pc, in_delay_slot);
        } else {
            write_gpr(rd, static_cast<u32>(result));
        }
        return true;
    }
    case 0x23: // SUBU
        write_gpr(rd, state_.gpr[rs] - state_.gpr[rt]);
        return true;
    case 0x24: // AND
        write_gpr(rd, state_.gpr[rs] & state_.gpr[rt]);
        return true;
    case 0x25: // OR
        write_gpr(rd, state_.gpr[rs] | state_.gpr[rt]);
        return true;
    case 0x26: // XOR
        write_gpr(rd, state_.gpr[rs] ^ state_.gpr[rt]);
        return true;
    case 0x27: // NOR
        write_gpr(rd, ~(state_.gpr[rs] | state_.gpr[rt]));
        return true;
    case 0x2A: // SLT
        write_gpr(
            rd,
            static_cast<s32>(state_.gpr[rs]) <
                    static_cast<s32>(state_.gpr[rt])
                ? 1u
                : 0u);
        return true;
    case 0x2B: // SLTU
        write_gpr(
            rd,
            state_.gpr[rs] < state_.gpr[rt] ? 1u : 0u);
        return true;
    default:
        return fail(
            pc,
            instruction,
            "Unsupported IOP SPECIAL function " + hex32(funct),
            error);
    }
}

bool IopCpu::execute_regimm(
    u32 pc,
    u32 instruction,
    std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    bool taken = false;

    switch (rt) {
    case 0x00: // BLTZ
        taken = static_cast<s32>(state_.gpr[rs]) < 0;
        break;
    case 0x01: // BGEZ
        taken = static_cast<s32>(state_.gpr[rs]) >= 0;
        break;
    case 0x10: // BLTZAL
        write_gpr(31, pc + 8u);
        taken = static_cast<s32>(state_.gpr[rs]) < 0;
        break;
    case 0x11: // BGEZAL
        write_gpr(31, pc + 8u);
        taken = static_cast<s32>(state_.gpr[rs]) >= 0;
        break;
    default:
        return fail(
            pc,
            instruction,
            "Unsupported IOP REGIMM variant " + hex32(rt),
            error);
    }

    if (taken) {
        state_.next_pc = branch_target(pc, immediate(instruction));
    }
    next_is_delay_slot_ = true;
    return true;
}

bool IopCpu::execute_cop0(
    u32 pc,
    u32 instruction,
    std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;
    const u32 funct = instruction & 63u;

    switch (rs) {
    case 0x00: // MFC0
    case 0x02: // CFC0
        schedule_load(rt, state_.cop0[rd]);
        return true;
    case 0x04: // MTC0
    case 0x06: // CTC0
        state_.cop0[rd] = state_.gpr[rt];
        return true;
    case 0x10: // RFE
        if (funct != 0x10u) {
            return fail(
                pc,
                instruction,
                "Unsupported IOP COP0 function " + hex32(funct),
                error);
        }
        state_.cop0[12] =
            (state_.cop0[12] & 0xFFFFFFF0u) |
            ((state_.cop0[12] & 0x3Cu) >> 2);
        return true;
    default:
        return fail(
            pc,
            instruction,
            "Unsupported IOP COP0 rs " + hex32(rs),
            error);
    }
}

bool IopCpu::execute_cop2(
    u32 pc,
    u32 instruction,
    std::string& error) {
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const u32 rd = (instruction >> 11) & 31u;

    switch (rs) {
    case 0x00: // MFC2
        schedule_load(rt, state_.gte_data[rd]);
        return true;
    case 0x02: // CFC2
        schedule_load(rt, state_.gte_ctrl[rd]);
        return true;
    case 0x04: // MTC2
        state_.gte_data[rd] = state_.gpr[rt];
        return true;
    case 0x06: // CTC2
        state_.gte_ctrl[rd] = state_.gpr[rt];
        return true;
    default:
        break;
    }

    if (rs >= 0x10u) {
        // The PS2-mode BIOS only needs the IOP's legacy GTE to survive
        // capability/init probes before the visible startup path. Geometry
        // execution is a PS1-compatibility concern, so retire commands here
        // without inventing geometry results; FLAG remains clear.
        state_.gte_ctrl[31] = 0;
        return true;
    }

    return fail(
        pc,
        instruction,
        "Unsupported IOP COP2 rs " + hex32(rs),
        error);
}

bool IopCpu::in_osdsys_idle_loop() const {
    if (halted_ || pending_load_.valid || next_load_.valid ||
        bus_.interrupt_pending()) {
        return false;
    }

    const bool at_branch =
        state_.pc == 0x0000AE94u &&
        state_.next_pc == 0x0000AE98u &&
        !next_is_delay_slot_;
    const bool at_delay =
        state_.pc == 0x0000AE98u &&
        state_.next_pc == 0x0000AE94u &&
        next_is_delay_slot_;
    if (!at_branch && !at_delay) return false;

    u32 branch = 0;
    u32 delay = 0;
    return bus_.read32(0x0000AE94u, branch) &&
           bus_.read32(0x0000AE98u, delay) &&
           branch == 0x08002BA5u &&
           delay == 0u;
}

bool IopCpu::skip_osdsys_idle_pair() {
    if (halted_ || state_.pc != 0x0000AE94u ||
        state_.next_pc != 0x0000AE98u || next_is_delay_slot_ ||
        pending_load_.valid || next_load_.valid ||
        bus_.interrupt_pending()) return false;
    u32 branch = 0, delay = 0;
    if (!bus_.read32(0x0000AE94u, branch) ||
        !bus_.read32(0x0000AE98u, delay) ||
        branch != 0x08002BA5u || delay != 0u) return false;

    state_.cop0[13] &= ~0x00000400u;
    bus_.tick(1u);
    if (bus_.interrupt_pending()) state_.cop0[13] |= 0x00000400u;
    else state_.cop0[13] &= ~0x00000400u;
    bus_.tick(1u);

    state_.last_pc = 0x0000AE98u;
    state_.last_instruction = 0u;
    state_.pc = 0x0000AE94u;
    state_.next_pc = 0x0000AE98u;
    state_.gpr[0] = 0u;
    state_.instructions_executed += 2u;
    direct_write_mask_ = 0u;
    return true;
}

u64 IopCpu::skip_osdsys_idle_pairs(u64 max_pairs) {
    if (max_pairs == 0u || halted_ ||
        state_.pc != 0x0000AE94u ||
        state_.next_pc != 0x0000AE98u || next_is_delay_slot_ ||
        pending_load_.valid || next_load_.valid ||
        bus_.interrupt_pending()) return 0;
    u32 branch = 0, delay = 0;
    if (!bus_.read32(0x0000AE94u, branch) ||
        !bus_.read32(0x0000AE98u, delay) ||
        branch != 0x08002BA5u || delay != 0u) return 0;

    u64 pairs = std::min<u64>(max_pairs, 65536u);
    while (pairs != 0u && !bus_.tick_event_free(pairs * 2u)) {
        pairs >>= 1u;
    }
    if (pairs == 0u) return 0;

    state_.cop0[13] &= ~0x00000400u;
    state_.last_pc = 0x0000AE98u;
    state_.last_instruction = 0u;
    state_.pc = 0x0000AE94u;
    state_.next_pc = 0x0000AE98u;
    state_.gpr[0] = 0u;
    state_.instructions_executed += pairs * 2u;
    direct_write_mask_ = 0u;
    return pairs;
}

bool IopCpu::step(std::string& error) {
    error.clear();

    if (halted_) {
        error = halt_reason_;
        return false;
    }

    // The IOP INTC is wired to the R3000A external interrupt input.
    // Do not interrupt a branch delay slot; PCSX2 also tests INTC after the
    // branch+delay-slot pair has completed.
    const bool external_irq = bus_.interrupt_pending();
    if (external_irq) {
        state_.cop0[13] |= 0x00000400u;
    } else {
        state_.cop0[13] &= ~0x00000400u;
    }

    const bool cop0_interrupt_enabled =
        (state_.cop0[12] & 0x00000001u) != 0 &&
        (state_.cop0[12] & state_.cop0[13] & 0x0000FF00u) != 0;

    if (!next_is_delay_slot_ && cop0_interrupt_enabled) {
        if (pending_load_.valid && pending_load_.reg != 0) {
            state_.gpr[pending_load_.reg] = pending_load_.value;
        }
        pending_load_ = {};
        next_load_ = {};
        raise_exception(0, state_.pc, false);
    }

    const u32 pc = state_.pc;
    const u32 old_next_pc = state_.next_pc;
    const bool in_delay_slot = next_is_delay_slot_;
    next_is_delay_slot_ = false;

    u32 instruction = 0;
    if (!bus_.read32(pc, instruction)) {
        return fail(
            pc,
            0,
            "IOP instruction fetch fault from " + hex32(pc),
            error);
    }

    state_.last_pc = pc;
    state_.last_instruction = instruction;
    state_.pc = old_next_pc;
    state_.next_pc = old_next_pc + 4u;

    direct_write_mask_ = 0;
    next_load_ = {};

    // Retail ROM OSDSYS spends most of bootstrap in this J/NOP scheduler
    // idle pair. Keep the ordinary interrupt check above and load-delay
    // retirement below, but avoid the full decoder for these exact words.
    if ((pc == 0x0000AE94u && instruction == 0x08002BA5u) ||
        (pc == 0x0000AE98u && instruction == 0u)) {
        if (pc == 0x0000AE94u) {
            state_.next_pc = 0x0000AE94u;
            next_is_delay_slot_ = true;
        }
        if (pending_load_.valid && pending_load_.reg != 0) {
            state_.gpr[pending_load_.reg] = pending_load_.value;
        }
        pending_load_ = {};
        state_.gpr[0] = 0;
        ++state_.instructions_executed;
        bus_.tick(1);
        return true;
    }

    const u32 opcode = instruction >> 26;
    const u32 rs = (instruction >> 21) & 31u;
    const u32 rt = (instruction >> 16) & 31u;
    const s16 imm = immediate(instruction);

    const auto address = [&]() {
        return state_.gpr[rs] +
               static_cast<u32>(static_cast<s32>(imm));
    };

    const auto cache_isolated_ram = [&](u32 addr) {
        return (state_.cop0[12] & 0x00010000u) != 0 &&
               IopBus::is_ram_address(addr);
    };

    const auto write8 = [&](u32 addr, u8 value) {
        return cache_isolated_ram(addr) || bus_.write8(addr, value);
    };
    const auto write16 = [&](u32 addr, u16 value) {
        return cache_isolated_ram(addr) || bus_.write16(addr, value);
    };
    const auto write32 = [&](u32 addr, u32 value) {
        return cache_isolated_ram(addr) || bus_.write32(addr, value);
    };

    const auto read_fault = [&](const char* kind, u32 addr) {
        return fail(
            pc,
            instruction,
            std::string("IOP ") + kind + " fault from " + hex32(addr),
            error);
    };

    bool ok = true;

    switch (opcode) {
    case 0x00:
        ok = execute_special(pc, instruction, in_delay_slot, error);
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
        write_gpr(31, pc + 8u);
        state_.next_pc =
            ((pc + 4u) & 0xF0000000u) |
            ((instruction & 0x03FFFFFFu) << 2);
        next_is_delay_slot_ = true;
        break;
    case 0x04: // BEQ
        if (state_.gpr[rs] == state_.gpr[rt]) {
            state_.next_pc = branch_target(pc, imm);
        }
        next_is_delay_slot_ = true;
        break;
    case 0x05: // BNE
        if (state_.gpr[rs] != state_.gpr[rt]) {
            state_.next_pc = branch_target(pc, imm);
        }
        next_is_delay_slot_ = true;
        break;
    case 0x06: // BLEZ
        if (static_cast<s32>(state_.gpr[rs]) <= 0) {
            state_.next_pc = branch_target(pc, imm);
        }
        next_is_delay_slot_ = true;
        break;
    case 0x07: // BGTZ
        if (static_cast<s32>(state_.gpr[rs]) > 0) {
            state_.next_pc = branch_target(pc, imm);
        }
        next_is_delay_slot_ = true;
        break;
    case 0x08: { // ADDI
        s32 result = 0;
        if (add_overflow(
                static_cast<s32>(state_.gpr[rs]),
                static_cast<s32>(imm),
                result)) {
            raise_exception(0x30u, pc, in_delay_slot);
        } else {
            write_gpr(rt, static_cast<u32>(result));
        }
        break;
    }
    case 0x09: // ADDIU
        write_gpr(
            rt,
            state_.gpr[rs] +
                static_cast<u32>(static_cast<s32>(imm)));
        break;
    case 0x0A: // SLTI
        write_gpr(
            rt,
            static_cast<s32>(state_.gpr[rs]) <
                    static_cast<s32>(imm)
                ? 1u
                : 0u);
        break;
    case 0x0B: // SLTIU
        write_gpr(
            rt,
            state_.gpr[rs] <
                    static_cast<u32>(static_cast<s32>(imm))
                ? 1u
                : 0u);
        break;
    case 0x0C: // ANDI
        write_gpr(rt, state_.gpr[rs] & (instruction & 0xFFFFu));
        break;
    case 0x0D: // ORI
        write_gpr(rt, state_.gpr[rs] | (instruction & 0xFFFFu));
        break;
    case 0x0E: // XORI
        write_gpr(rt, state_.gpr[rs] ^ (instruction & 0xFFFFu));
        break;
    case 0x0F: // LUI
        write_gpr(rt, (instruction & 0xFFFFu) << 16);
        break;
    case 0x10:
        ok = execute_cop0(pc, instruction, error);
        break;
    case 0x12:
        ok = execute_cop2(pc, instruction, error);
        break;
    case 0x32: { // LWC2
        const u32 addr = address();
        u32 value = 0;
        if (!bus_.read32(addr, value)) {
            ok = read_fault("LWC2", addr);
        } else {
            state_.gte_data[rt] = value;
        }
        break;
    }
    case 0x20: { // LB
        const u32 addr = address();
        u8 value = 0;
        if (!bus_.read8(addr, value)) {
            ok = read_fault("load byte", addr);
        } else {
            schedule_load(
                rt,
                static_cast<u32>(
                    static_cast<s32>(static_cast<s8>(value))));
        }
        break;
    }
    case 0x21: { // LH
        const u32 addr = address();
        u16 value = 0;
        if (!bus_.read16(addr, value)) {
            ok = read_fault("load halfword", addr);
        } else {
            schedule_load(
                rt,
                static_cast<u32>(
                    static_cast<s32>(static_cast<s16>(value))));
        }
        break;
    }
    case 0x22: { // LWL
        const u32 addr = address();
        u32 memory = 0;
        if (!bus_.read32(addr & ~3u, memory)) {
            ok = read_fault("LWL", addr);
        } else {
            const u32 shift = (addr & 3u) * 8u;
            const u32 old = load_merge_base(rt);
            const u32 value =
                (old & (0x00FFFFFFu >> shift)) |
                (memory << (24u - shift));
            schedule_load(rt, value);
        }
        break;
    }
    case 0x23: { // LW
        const u32 addr = address();
        u32 value = 0;
        if (!bus_.read32(addr, value)) {
            ok = read_fault("load word", addr);
        } else {
            schedule_load(rt, value);
        }
        break;
    }
    case 0x24: { // LBU
        const u32 addr = address();
        u8 value = 0;
        if (!bus_.read8(addr, value)) {
            ok = read_fault("load byte", addr);
        } else {
            schedule_load(rt, value);
        }
        break;
    }
    case 0x25: { // LHU
        const u32 addr = address();
        u16 value = 0;
        if (!bus_.read16(addr, value)) {
            ok = read_fault("load halfword", addr);
        } else {
            schedule_load(rt, value);
        }
        break;
    }
    case 0x26: { // LWR
        const u32 addr = address();
        u32 memory = 0;
        if (!bus_.read32(addr & ~3u, memory)) {
            ok = read_fault("LWR", addr);
        } else {
            const u32 shift = (addr & 3u) * 8u;
            const u32 old = load_merge_base(rt);
            const u32 value =
                (old & (0xFFFFFF00u << (24u - shift))) |
                (memory >> shift);
            schedule_load(rt, value);
        }
        break;
    }
    case 0x28: // SB
        ok = write8(address(), static_cast<u8>(state_.gpr[rt]));
        if (!ok) {
            ok = fail(
                pc,
                instruction,
                "IOP store byte fault to " + hex32(address()),
                error);
        }
        break;
    case 0x29: // SH
        ok = write16(address(), static_cast<u16>(state_.gpr[rt]));
        if (!ok) {
            ok = fail(
                pc,
                instruction,
                "IOP store halfword fault to " + hex32(address()),
                error);
        }
        break;
    case 0x2A: { // SWL
        const u32 addr = address();
        const u32 aligned = addr & ~3u;
        u32 memory = 0;
        if (!bus_.read32(aligned, memory)) {
            ok = read_fault("SWL read", addr);
        } else {
            const u32 shift = (addr & 3u) * 8u;
            const u32 value =
                (state_.gpr[rt] >> (24u - shift)) |
                (memory & (0xFFFFFF00u << shift));
            ok = write32(aligned, value);
            if (!ok) {
                ok = fail(
                    pc,
                    instruction,
                    "IOP SWL fault to " + hex32(addr),
                    error);
            }
        }
        break;
    }
    case 0x2B: { // SW
        const u32 addr = address();
        if (!write32(addr, state_.gpr[rt])) {
            ok = fail(
                pc,
                instruction,
                "IOP store word fault to " + hex32(addr),
                error);
        }
        break;
    }
    case 0x2E: { // SWR
        const u32 addr = address();
        const u32 aligned = addr & ~3u;
        u32 memory = 0;
        if (!bus_.read32(aligned, memory)) {
            ok = read_fault("SWR read", addr);
        } else {
            const u32 shift = (addr & 3u) * 8u;
            const u32 value =
                (state_.gpr[rt] << shift) |
                (memory & (0x00FFFFFFu >> (24u - shift)));
            ok = bus_.write32(aligned, value);
            if (!ok) {
                ok = fail(
                    pc,
                    instruction,
                    "IOP SWR fault to " + hex32(addr),
                    error);
            }
        }
        break;
    }
    case 0x3A: { // SWC2
        const u32 addr = address();
        if (!write32(addr, state_.gte_data[rt])) {
            ok = fail(
                pc,
                instruction,
                "IOP SWC2 fault to " + hex32(addr),
                error);
        }
        break;
    }
    default:
        ok = fail(
            pc,
            instruction,
            "Unsupported IOP opcode " + hex32(opcode),
            error);
        break;
    }

    if (!ok) {
        state_.pc = pc;
        state_.next_pc = old_next_pc;
        next_is_delay_slot_ = in_delay_slot;
        return false;
    }

    if (pending_load_.valid &&
        pending_load_.reg != 0 &&
        (direct_write_mask_ & (1u << pending_load_.reg)) == 0) {
        state_.gpr[pending_load_.reg] = pending_load_.value;
    }

    pending_load_ = next_load_;
    state_.gpr[0] = 0;
    ++state_.instructions_executed;
    bus_.tick(1);
    return true;
}

u64 IopCpu::run(
    u64 instruction_budget,
    std::string& error) {
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
