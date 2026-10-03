#pragma once

#include "common/types.h"

namespace ps2 {

enum class EeDynarecIrKind : u8 {
    Integer,
    MemoryLoad,
    MemoryStore,
    ControlFlow,
    Cop0,
    Cop1,
    Mmi,
    CacheHint,
    Unknown,
};

enum EeDynarecIrFlag : u16 {
    EeIrNone = 0u,
    EeIrReadsMemory = 1u << 0,
    EeIrWritesMemory = 1u << 1,
    EeIrHasDelaySlot = 1u << 2,
    EeIrBranchLikely = 1u << 3,
    EeIrWritesLink = 1u << 4,
    EeIrPreciseExit = 1u << 5,
    EeIrCoprocessor = 1u << 6,
};

struct EeDynarecIrInstruction {
    u32 pc = 0;
    u32 word = 0;
    s16 immediate = 0;
    u8 opcode = 0;
    u8 rs = 0;
    u8 rt = 0;
    u8 rd = 0;
    u8 sa = 0;
    u8 funct = 0;
    u8 memory_width = 0;
    u8 alignment_mask = 0;
    EeDynarecIrKind kind = EeDynarecIrKind::Unknown;
    u16 flags = EeIrNone;

    [[nodiscard]] bool has(EeDynarecIrFlag flag) const {
        return (flags & static_cast<u16>(flag)) != 0u;
    }
};

inline EeDynarecIrInstruction decode_ee_dynarec_ir(
    u32 pc,
    u32 word) {
    EeDynarecIrInstruction ir{};
    ir.pc = pc;
    ir.word = word;
    ir.immediate = static_cast<s16>(word & 0xFFFFu);
    ir.opcode = static_cast<u8>(word >> 26);
    ir.rs = static_cast<u8>((word >> 21) & 31u);
    ir.rt = static_cast<u8>((word >> 16) & 31u);
    ir.rd = static_cast<u8>((word >> 11) & 31u);
    ir.sa = static_cast<u8>((word >> 6) & 31u);
    ir.funct = static_cast<u8>(word & 63u);

    auto memory = [&](bool store, u8 width, u8 alignment_mask = 0u) {
        ir.kind = store
            ? EeDynarecIrKind::MemoryStore
            : EeDynarecIrKind::MemoryLoad;
        ir.memory_width = width;
        ir.alignment_mask = alignment_mask;
        ir.flags |= store ? EeIrWritesMemory : EeIrReadsMemory;
    };

    if (word == 0u) {
        ir.kind = EeDynarecIrKind::Integer;
        return ir;
    }

    switch (ir.opcode) {
    case 0x00u:
        ir.kind = EeDynarecIrKind::Integer;
        if (ir.funct == 0x08u || ir.funct == 0x09u ||
            ir.funct == 0x0Cu || ir.funct == 0x0Du) {
            ir.kind = EeDynarecIrKind::ControlFlow;
            ir.flags |= EeIrHasDelaySlot;
            if (ir.funct == 0x09u) ir.flags |= EeIrWritesLink;
            if (ir.funct == 0x0Cu || ir.funct == 0x0Du) {
                ir.flags |= EeIrPreciseExit;
            }
        }
        return ir;

    case 0x01u:
        ir.kind = EeDynarecIrKind::Integer;
        if (ir.rt <= 0x03u ||
            (ir.rt >= 0x10u && ir.rt <= 0x13u)) {
            ir.kind = EeDynarecIrKind::ControlFlow;
            ir.flags |= EeIrHasDelaySlot;
            if (ir.rt == 0x02u || ir.rt == 0x03u ||
                ir.rt == 0x12u || ir.rt == 0x13u) {
                ir.flags |= EeIrBranchLikely;
            }
            if (ir.rt >= 0x10u && ir.rt <= 0x13u) {
                ir.flags |= EeIrWritesLink;
            }
        }
        return ir;

    case 0x02u:
    case 0x03u:
        ir.kind = EeDynarecIrKind::ControlFlow;
        ir.flags |= EeIrHasDelaySlot;
        if (ir.opcode == 0x03u) ir.flags |= EeIrWritesLink;
        return ir;

    case 0x04u:
    case 0x05u:
    case 0x06u:
    case 0x07u:
        ir.kind = EeDynarecIrKind::ControlFlow;
        ir.flags |= EeIrHasDelaySlot;
        return ir;

    case 0x14u:
    case 0x15u:
    case 0x16u:
    case 0x17u:
        ir.kind = EeDynarecIrKind::ControlFlow;
        ir.flags |= EeIrHasDelaySlot | EeIrBranchLikely;
        return ir;

    case 0x10u:
        ir.kind = EeDynarecIrKind::Cop0;
        ir.flags |= EeIrCoprocessor;
        if (ir.rs == 0x04u ||
            (ir.rs == 0x10u &&
             (ir.funct == 0x38u || ir.funct == 0x39u))) {
            ir.flags |= EeIrPreciseExit;
        }
        return ir;

    case 0x11u:
        ir.kind = EeDynarecIrKind::Cop1;
        ir.flags |= EeIrCoprocessor;
        if (ir.rs == 0x08u) {
            ir.kind = EeDynarecIrKind::ControlFlow;
            ir.flags |= EeIrCoprocessor | EeIrHasDelaySlot;
            if ((ir.rt & 2u) != 0u) ir.flags |= EeIrBranchLikely;
        }
        return ir;

    case 0x1Cu:
        ir.kind = EeDynarecIrKind::Mmi;
        return ir;

    case 0x2Fu:
    case 0x33u:
        ir.kind = EeDynarecIrKind::CacheHint;
        return ir;

    case 0x1Eu: memory(false, 16u, 0x0Fu); return ir; // LQ
    case 0x20u: memory(false, 1u); return ir;
    case 0x21u: memory(false, 2u); return ir;
    case 0x23u: memory(false, 4u); return ir;
    case 0x24u: memory(false, 1u); return ir;
    case 0x25u: memory(false, 2u); return ir;
    case 0x27u: memory(false, 4u); return ir;
    case 0x30u: memory(false, 4u); return ir;
    case 0x31u: memory(false, 4u); return ir;
    case 0x34u: memory(false, 8u); return ir;
    case 0x36u: memory(false, 16u, 0x0Fu); return ir;
    case 0x37u: memory(false, 8u); return ir;

    case 0x1Fu: memory(true, 16u, 0x0Fu); return ir; // SQ
    case 0x28u: memory(true, 1u); return ir;
    case 0x29u: memory(true, 2u); return ir;
    case 0x2Bu: memory(true, 4u); return ir;
    case 0x38u: memory(true, 4u); return ir;
    case 0x39u: memory(true, 4u); return ir;
    case 0x3Cu: memory(true, 8u); return ir;
    case 0x3Eu: memory(true, 16u, 0x0Fu); return ir;
    case 0x3Fu: memory(true, 8u); return ir;

    default:
        ir.kind = EeDynarecIrKind::Integer;
        return ir;
    }
}

} // namespace ps2
