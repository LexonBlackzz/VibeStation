#include "core/ee/ee_cpu.h"

#include "core/memory/ee_bus.h"

#include <iomanip>
#include <sstream>

namespace ps2 {
namespace {

std::string hex32(u32 value) {
    std::ostringstream out;
    out << "0x" << std::uppercase << std::hex
        << std::setw(8) << std::setfill('0') << value;
    return out.str();
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
    return pc + 4u + static_cast<u32>(static_cast<s32>(imm) << 2);
}

u64 EeCpu::sign_extend_word(u32 value) {
    return static_cast<u64>(static_cast<s64>(static_cast<s32>(value)));
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
        write_gpr_word(
            rd,
            static_cast<u32>(gpr_u64(rt)) << sa);
        return true;
    case 0x02: // SRL
        write_gpr_word(
            rd,
            static_cast<u32>(gpr_u64(rt)) >> sa);
        return true;
    case 0x03: // SRA
        write_gpr_word(
            rd,
            static_cast<u32>(
                static_cast<s32>(static_cast<u32>(gpr_u64(rt))) >> sa));
        return true;
    case 0x08: // JR
        state_.next_pc = static_cast<u32>(gpr_u64(rs));
        return true;
    case 0x09: // JALR
        write_gpr64(rd, static_cast<u64>(pc + 8u));
        state_.next_pc = static_cast<u32>(gpr_u64(rs));
        return true;
    case 0x0F: // SYNC
        return true;
    case 0x21: // ADDU
        write_gpr_word(
            rd,
            static_cast<u32>(gpr_u64(rs)) +
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

    switch (rt) {
    case 0x00: // BLTZ
        taken = gpr_s64(rs) < 0;
        break;
    case 0x01: // BGEZ
        taken = gpr_s64(rs) >= 0;
        break;
    default:
        return fail(
            pc,
            instruction,
            "Unsupported REGIMM variant " + hex32(rt),
            error);
    }

    if (taken) {
        state_.next_pc = branch_target(pc, immediate(instruction));
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

    switch (rs) {
    case 0x00: // MFC0
        if (sel != 0) {
            return fail(pc, instruction, "Unsupported COP0 select", error);
        }
        write_gpr_word(rt, state_.cop0[rd]);
        return true;

    case 0x04: // MTC0
        if (sel != 0) {
            return fail(pc, instruction, "Unsupported COP0 select", error);
        }
        if (rd != 15) {
            state_.cop0[rd] = static_cast<u32>(gpr_u64(rt));
        }
        return true;

    case 0x10:
        if (funct == 0x02) { // TLBWI
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
        return fail(
            pc,
            instruction,
            "Unsupported COP0 function " + hex32(funct),
            error);

    default:
        return fail(
            pc,
            instruction,
            "Unsupported COP0 rs " + hex32(rs),
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
        write_gpr64(31, static_cast<u64>(pc + 8u));
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
    case 0x09: // ADDIU
        write_gpr64(
            rt,
            gpr_u64(rs) +
                static_cast<u64>(static_cast<s64>(imm)));
        break;
    case 0x0A: // SLTI
        write_gpr64(
            rt,
            gpr_s64(rs) < static_cast<s64>(imm) ? 1u : 0u);
        break;
    case 0x0C: // ANDI
        write_gpr64(
            rt,
            gpr_u64(rs) &
                static_cast<u64>(instruction & 0xFFFFu));
        break;
    case 0x0D: // ORI
        write_gpr64(
            rt,
            gpr_u64(rs) |
                static_cast<u64>(instruction & 0xFFFFu));
        break;
    case 0x0F: // LUI
        write_gpr_word(rt, (instruction & 0xFFFFu) << 16);
        break;
    case 0x10:
        ok = execute_cop0(pc, instruction, error);
        break;
    case 0x23: { // LW
        const u32 address = static_cast<u32>(
            gpr_u64(rs) + static_cast<u64>(static_cast<s64>(imm)));
        u32 value = 0;
        if (!bus_.read32(address, value)) {
            ok = fail(
                pc,
                instruction,
                "Load word fault from " + hex32(address),
                error);
        } else {
            write_gpr_word(rt, value);
        }
        break;
    }
    case 0x24: { // LBU
        const u32 address = static_cast<u32>(
            gpr_u64(rs) + static_cast<u64>(static_cast<s64>(imm)));
        u8 value = 0;
        if (!bus_.read8(address, value)) {
            ok = fail(
                pc,
                instruction,
                "Load byte fault from " + hex32(address),
                error);
        } else {
            write_gpr64(rt, value);
        }
        break;
    }
    case 0x2B: { // SW
        const u32 address = static_cast<u32>(
            gpr_u64(rs) + static_cast<u64>(static_cast<s64>(imm)));
        if (!bus_.write32(address, static_cast<u32>(gpr_u64(rt)))) {
            ok = fail(
                pc,
                instruction,
                "Store word fault to " + hex32(address),
                error);
        }
        break;
    }
    case 0x3F: { // SD
        const u32 address = static_cast<u32>(
            gpr_u64(rs) + static_cast<u64>(static_cast<s64>(imm)));
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
    state_.cop0[9] += 1u;
    bus_.tick(1);
    return true;
}

u64 EeCpu::run(
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
