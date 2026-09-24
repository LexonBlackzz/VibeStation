#include "core/ps2_system.h"

#include <bit>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

bool step_at(ps2::Ps2System& system, ps2::u32 pc, ps2::u32 instruction) {
    if (!system.bus().write32(pc, instruction)) return false;
    system.ee().reset(pc);
    std::string error;
    return system.ee().step(error);
}

bool test_system_stack_footprint() {
    return expect(
        sizeof(ps2::Ps2System) < 512u * 1024u,
        "Ps2System stack footprint is too large for the default Windows stack");
}

bool test_mmi_por_128() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000;
    const ps2::u32 op = (0x1Cu << 26) | (1u << 21) | (2u << 16) |
                        (3u << 11) | (0x12u << 6) | 0x29u;
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {0x00FF00FF00FF00FFull, 0xAAAAAAAA55555555ull};
    system.ee().state().gpr[2] = {0xFF00FF00FF00FF00ull, 0x55555555AAAAAAAAull};
    bool ok = system.bus().write32(pc, op);
    std::string error;
    ok = expect(ok && system.ee().step(error), "POR execution failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0xFFFFFFFFFFFFFFFFull,
                "POR low half mismatch") && ok;
    ok = expect(system.ee().state().gpr[3].hi == 0xFFFFFFFFFFFFFFFFull,
                "POR high half mismatch") && ok;
    return ok;
}

bool test_mmi_padduw() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000;
    const ps2::u32 op = (0x1Cu << 26) | (1u << 21) | (2u << 16) |
                        (3u << 11) | (0x10u << 6) | 0x28u;
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {0xFFFFFFFF00000001ull, 0x800000007FFFFFFFull};
    system.ee().state().gpr[2] = {0x0000000200000002ull, 0x8000000000000001ull};
    bool ok = system.bus().write32(pc, op);
    std::string error;
    ok = expect(ok && system.ee().step(error), "PADDUW execution failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0xFFFFFFFF00000003ull,
                "PADDUW low lanes mismatch") && ok;
    ok = expect(system.ee().state().gpr[3].hi == 0xFFFFFFFF80000000ull,
                "PADDUW high lanes mismatch") && ok;
    return ok;
}

bool test_mmi_madd_and_plzcw() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2100u;
    std::string error;
    bool ok = true;

    // MADD r3,r1,r2 accumulates into the primary LO/HI pair.
    constexpr ps2::u32 madd =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        0x00u;
    system.ee().reset(pc);
    system.ee().state().lo = 2u;
    system.ee().state().hi = 0u;
    system.ee().state().gpr[1].lo = 3u;
    system.ee().state().gpr[2].lo = 0xFFFFFFFCu;
    ok = expect(system.bus().write32(pc, madd),
                "MADD test opcode write failed") && ok;
    ok = expect(system.ee().step(error),
                "MADD execution failed") && ok;
    ok = expect(system.ee().state().lo == 0xFFFFFFFFFFFFFFF6ull &&
                    system.ee().state().hi == 0xFFFFFFFFFFFFFFFFull,
                "MADD accumulator mismatch") && ok;
    ok = expect(system.ee().state().gpr[3].lo ==
                    0xFFFFFFFFFFFFFFF6ull,
                "MADD destination mismatch") && ok;

    // MADDU1 uses the secondary accumulator pair.
    constexpr ps2::u32 maddu1 =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        0x21u;
    system.ee().reset(pc);
    system.ee().state().lo1 = 5u;
    system.ee().state().hi1 = 0u;
    system.ee().state().gpr[1].lo = 0xFFFFFFFFu;
    system.ee().state().gpr[2].lo = 2u;
    ok = expect(system.bus().write32(pc, maddu1),
                "MADDU1 test opcode write failed") && ok;
    error.clear();
    ok = expect(system.ee().step(error),
                "MADDU1 execution failed") && ok;
    ok = expect(system.ee().state().lo1 == 3u &&
                    system.ee().state().hi1 == 2u,
                "MADDU1 accumulator mismatch") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 3u,
                "MADDU1 destination mismatch") && ok;

    // PLZCW counts the leading sign run, excluding the sign bit itself,
    // independently for the two low words of rs.
    constexpr ps2::u32 plzcw =
        (0x1Cu << 26) |
        (1u << 21) | (3u << 11) |
        0x04u;
    system.ee().reset(pc);
    system.ee().state().gpr[1].lo =
        (static_cast<ps2::u64>(0xFFFFFFF0u) << 32) | 1u;
    system.ee().state().gpr[3].hi =
        0xAABBCCDDEEFF0011ull;
    ok = expect(system.bus().write32(pc, plzcw),
                "PLZCW test opcode write failed") && ok;
    error.clear();
    ok = expect(system.ee().step(error),
                "PLZCW execution failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo ==
                    ((static_cast<ps2::u64>(27u) << 32) | 30u),
                "PLZCW count mismatch") && ok;
    ok = expect(system.ee().state().gpr[3].hi ==
                    0xAABBCCDDEEFF0011ull,
                "PLZCW incorrectly modified upper 64 bits") && ok;

    return ok;
}

bool test_mmi_pmfhl_pmthl() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2180u;
    std::string error;
    bool ok = true;

    // PMTHL.LW copies the four source words into the low words of
    // LO/HI/LO1/HI1, preserving their upper 32 bits.
    constexpr ps2::u32 pmthl =
        (0x1Cu << 26) |
        (1u << 21) |
        (0u << 6) |
        0x31u;
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0x2222222211111111ull,
        0x4444444433333333ull,
    };
    system.ee().state().lo = 0xAAAAAAAA00000000ull;
    system.ee().state().hi = 0xBBBBBBBB00000000ull;
    system.ee().state().lo1 = 0xCCCCCCCC00000000ull;
    system.ee().state().hi1 = 0xDDDDDDDD00000000ull;
    ok = expect(system.bus().write32(pc, pmthl),
                "PMTHL test opcode write failed") && ok;
    ok = expect(system.ee().step(error),
                "PMTHL execution failed") && ok;
    ok = expect(system.ee().state().lo ==
                    0xAAAAAAAA11111111ull &&
                system.ee().state().hi ==
                    0xBBBBBBBB22222222ull &&
                system.ee().state().lo1 ==
                    0xCCCCCCCC33333333ull &&
                system.ee().state().hi1 ==
                    0xDDDDDDDD44444444ull,
                "PMTHL accumulator mapping mismatch") && ok;

    // PMFHL.LW reconstructs the four low accumulator words.
    constexpr ps2::u32 pmfhl_lw =
        (0x1Cu << 26) |
        (2u << 11) |
        (0u << 6) |
        0x30u;
    ok = expect(system.bus().write32(pc + 4u, pmfhl_lw),
                "PMFHL.LW test opcode write failed") && ok;
    system.ee().state().pc = pc + 4u;
    system.ee().state().next_pc = pc + 8u;
    error.clear();
    ok = expect(system.ee().step(error),
                "PMFHL.LW execution failed") && ok;
    ok = expect(system.ee().state().gpr[2].lo ==
                    0x2222222211111111ull &&
                system.ee().state().gpr[2].hi ==
                    0x4444444433333333ull,
                "PMFHL.LW result mismatch") && ok;

    // PMFHL.SH saturates each signed accumulator word to a halfword.
    constexpr ps2::u32 pmfhl_sh =
        (0x1Cu << 26) |
        (3u << 11) |
        (4u << 6) |
        0x30u;
    system.ee().state().lo =
        0xFFFF7FFF00008000ull;
    system.ee().state().hi =
        0xFFFFFFD60000002Aull;
    system.ee().state().lo1 =
        0x00007FFF00000001ull;
    system.ee().state().hi1 =
        0xFFFF8000FFFFFFFFull;
    ok = expect(system.bus().write32(pc + 8u, pmfhl_sh),
                "PMFHL.SH test opcode write failed") && ok;
    system.ee().state().pc = pc + 8u;
    system.ee().state().next_pc = pc + 12u;
    error.clear();
    ok = expect(system.ee().step(error),
                "PMFHL.SH execution failed") && ok;

    const ps2::u64 expected_lo =
        0xFFD6002A80007FFFull;
    const ps2::u64 expected_hi =
        0x8000FFFF7FFF0001ull;
    ok = expect(system.ee().state().gpr[3].lo == expected_lo &&
                    system.ee().state().gpr[3].hi == expected_hi,
                "PMFHL.SH saturation mismatch") && ok;

    return ok;
}

bool test_mmi_packed_accumulator_moves() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x21C0u;
    std::string error;
    bool ok = true;

    auto run = [&](ps2::u32 instruction, const char* message) {
        ok = expect(system.bus().write32(pc, instruction), message) && ok;
        system.ee().state().pc = pc;
        system.ee().state().next_pc = pc + 4u;
        error.clear();
        ok = expect(system.ee().step(error), message) && ok;
    };

    constexpr ps2::u32 pmfhi =
        (0x1Cu << 26) | (2u << 11) | (0x08u << 6) | 0x09u;
    constexpr ps2::u32 pmflo =
        (0x1Cu << 26) | (3u << 11) | (0x09u << 6) | 0x09u;
    constexpr ps2::u32 pmthi =
        (0x1Cu << 26) | (4u << 21) | (0x08u << 6) | 0x29u;
    constexpr ps2::u32 pmtlo =
        (0x1Cu << 26) | (5u << 21) | (0x09u << 6) | 0x29u;

    system.ee().reset(pc);
    system.ee().state().hi = 0x0123456789ABCDEFull;
    system.ee().state().hi1 = 0xFEDCBA9876543210ull;
    system.ee().state().lo = 0x1111222233334444ull;
    system.ee().state().lo1 = 0xAAAABBBBCCCCDDDDull;
    run(pmfhi, "PMFHI execution failed");
    run(pmflo, "PMFLO execution failed");
    ok = expect(system.ee().state().gpr[2].lo == 0x0123456789ABCDEFull &&
                    system.ee().state().gpr[2].hi == 0xFEDCBA9876543210ull,
                "PMFHI packed result mismatch") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0x1111222233334444ull &&
                    system.ee().state().gpr[3].hi == 0xAAAABBBBCCCCDDDDull,
                "PMFLO packed result mismatch") && ok;

    system.ee().state().gpr[4] = {
        0x8877665544332211ull, 0x1020304050607080ull};
    system.ee().state().gpr[5] = {
        0xCAFEBABEDEADBEEFull, 0x0F1E2D3C4B5A6978ull};
    run(pmthi, "PMTHI execution failed");
    run(pmtlo, "PMTLO execution failed");
    ok = expect(system.ee().state().hi == 0x8877665544332211ull &&
                    system.ee().state().hi1 == 0x1020304050607080ull,
                "PMTHI packed source mismatch") && ok;
    ok = expect(system.ee().state().lo == 0xCAFEBABEDEADBEEFull &&
                    system.ee().state().lo1 == 0x0F1E2D3C4B5A6978ull,
                "PMTLO packed source mismatch") && ok;

    return ok;
}

bool test_mmi_bootstrap_packed_ops() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2200u;
    std::string error;
    bool ok = true;

    auto run = [&](ps2::u32 instruction) {
        ok = expect(
                 system.bus().write32(pc, instruction),
                 "failed to install MMI bootstrap opcode") && ok;
        system.ee().state().pc = pc;
        system.ee().state().next_pc = pc + 4u;
        error.clear();
        ok = expect(
                 system.ee().step(error),
                 "MMI bootstrap opcode execution failed") && ok;
    };

    // PEXTLW r1,r1,r2 exercises both interleave semantics and rd==rs aliasing.
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0x2222222211111111ull,
        0x4444444433333333ull,
    };
    system.ee().state().gpr[2] = {
        0xBBBBBBBBAAAAAAAAull,
        0xDDDDDDDDCCCCCCCCull,
    };
    const ps2::u32 pextlw =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (1u << 11) |
        (0x12u << 6) | 0x08u;
    run(pextlw);
    ok = expect(
             system.ee().state().gpr[1].lo ==
                 0x11111111AAAAAAAAull &&
             system.ee().state().gpr[1].hi ==
                 0x22222222BBBBBBBBull,
             "PEXLW interleave/alias mismatch") && ok;

    // PAND r3,r1,r2.
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0xFF00FF00AA55AA55ull,
        0x0F0FF0F012345678ull,
    };
    system.ee().state().gpr[2] = {
        0x0FF00FF0FFFF0000ull,
        0x3333CCCCFFFFFFFFull,
    };
    const ps2::u32 pand =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        (0x12u << 6) | 0x09u;
    run(pand);
    ok = expect(
             system.ee().state().gpr[3].lo ==
                 (0xFF00FF00AA55AA55ull &
                  0x0FF00FF0FFFF0000ull) &&
             system.ee().state().gpr[3].hi ==
                 (0x0F0FF0F012345678ull &
                  0x3333CCCCFFFFFFFFull),
             "PAND result mismatch") && ok;

    // PCPYLD r3,r1,r2 => low = rt.low, high = rs.low.
    const ps2::u32 pcpyld =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        (0x0Eu << 6) | 0x09u;
    run(pcpyld);
    ok = expect(
             system.ee().state().gpr[3].lo ==
                 0x0FF00FF0FFFF0000ull &&
             system.ee().state().gpr[3].hi ==
                 0xFF00FF00AA55AA55ull,
             "PCPYLD result mismatch") && ok;

    // PADDUH saturates each halfword independently.
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0xFFFF000180007FFFull,
        0x0001000200030004ull,
    };
    system.ee().state().gpr[2] = {
        0x00010001FFFF0001ull,
        0xFFFF000100020001ull,
    };
    const ps2::u32 padduh =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        (0x14u << 6) | 0x28u;
    run(padduh);
    ok = expect(
             system.ee().state().gpr[3].lo ==
                 0xFFFF0002FFFF8000ull,
             "PADDUH saturation mismatch") && ok;

    // PSLLW keeps four independent 32-bit lanes.
    system.ee().reset(pc);
    system.ee().state().gpr[2] = {
        0x0000000200000001ull,
        0x8000000040000000ull,
    };
    const ps2::u32 psllw =
        (2u << 16) | (3u << 11) |
        (3u << 6) | 0x3Cu |
        (0x1Cu << 26);
    run(psllw);
    ok = expect(
             system.ee().state().gpr[3].lo ==
                 0x0000001000000008ull &&
             system.ee().state().gpr[3].hi ==
                 0x0000000000000000ull,
             "PSLLW lane shift mismatch") && ok;

    return ok;
}

bool test_mmi_bios_instruction_expansion() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2280u;
    std::string error;
    bool ok = true;

    auto run = [&](ps2::u32 instruction) {
        ok = expect(
                 system.bus().write32(pc, instruction),
                 "failed to install expanded MMI opcode") && ok;
        system.ee().state().pc = pc;
        system.ee().state().next_pc = pc + 4u;
        error.clear();
        ok = expect(
                 system.ee().step(error),
                 "expanded MMI opcode execution failed") && ok;
    };

    // PADDSW: signed word saturation is used by ROM-side packed math.
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0x800000007FFFFFFFull,
        0x00000001FFFFFFFFull,
    };
    system.ee().state().gpr[2] = {
        0xFFFFFFFF00000001ull,
        0xFFFFFFFF00000002ull,
    };
    const ps2::u32 paddsw =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        (0x10u << 6) | 0x08u;
    run(paddsw);
    ok = expect(
             system.ee().state().gpr[3].lo ==
                 0x800000007FFFFFFFull &&
             system.ee().state().gpr[3].hi ==
                 0x0000000000000001ull,
             "PADDSW saturation mismatch") && ok;

    // PSLLVW/PSRAVW operate on word 0 and word 2 and sign-extend each
    // result into a 64-bit destination half.
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0x0000000000000001ull,
        0x0000000000000004ull,
    };
    system.ee().state().gpr[2] = {
        0x0000000040000000ull,
        0x00000000F0000000ull,
    };
    const ps2::u32 psllvw =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        (0x02u << 6) | 0x09u;
    run(psllvw);
    ok = expect(
             system.ee().state().gpr[3].lo ==
                 0xFFFFFFFF80000000ull &&
             system.ee().state().gpr[3].hi == 0,
             "PSLLVW lane/sign extension mismatch") && ok;

    const ps2::u32 psravw =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        (0x03u << 6) | 0x29u;
    run(psravw);
    ok = expect(
             system.ee().state().gpr[3].lo ==
                 0x0000000020000000ull &&
             system.ee().state().gpr[3].hi ==
                 0xFFFFFFFFFF000000ull,
             "PSRAVW lane/sign extension mismatch") && ok;

    // PMULTW updates both accumulator lanes and returns both full products.
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0x00000000FFFFFFFEull,
        0x0000000000010000ull,
    };
    system.ee().state().gpr[2] = {
        0x0000000000000003ull,
        0x0000000000010000ull,
    };
    const ps2::u32 pmultw =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        (0x0Cu << 6) | 0x09u;
    run(pmultw);
    ok = expect(
             system.ee().state().gpr[3].lo ==
                 0xFFFFFFFFFFFFFFFAull &&
             system.ee().state().gpr[3].hi ==
                 0x0000000100000000ull,
             "PMULTW destination mismatch") && ok;
    ok = expect(
             system.ee().state().lo ==
                 0xFFFFFFFFFFFFFFFAull &&
             system.ee().state().hi ==
                 0xFFFFFFFFFFFFFFFFull &&
             system.ee().state().lo1 == 0 &&
             system.ee().state().hi1 == 1,
             "PMULTW accumulator mismatch") && ok;

    // PADSBH is the last defined MMI1 subgroup operation: subtract
    // lower halfwords and add upper halfwords.
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0x0004000300020001ull,
        0x0008000700060005ull,
    };
    system.ee().state().gpr[2] = {
        0x0001000100010001ull,
        0x0001000100010001ull,
    };
    const ps2::u32 padsbh =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        (0x04u << 6) | 0x28u;
    run(padsbh);
    ok = expect(
             system.ee().state().gpr[3].lo ==
                 0x0003000200010000ull &&
             system.ee().state().gpr[3].hi ==
                 0x0009000800070006ull,
             "PADSBH mixed add/subtract mismatch") && ok;

    // PMADDUW accumulates unsigned products into both packed HI/LO pairs.
    system.ee().reset(pc);
    system.ee().state().lo = 10u;
    system.ee().state().hi = 0u;
    system.ee().state().lo1 = 20u;
    system.ee().state().hi1 = 0u;
    system.ee().state().gpr[1] = {3u, 4u};
    system.ee().state().gpr[2] = {5u, 6u};
    const ps2::u32 pmadduw =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        (0x00u << 6) | 0x29u;
    run(pmadduw);
    ok = expect(
             system.ee().state().gpr[3].lo == 25u &&
             system.ee().state().gpr[3].hi == 44u &&
             system.ee().state().lo == 25u &&
             system.ee().state().lo1 == 44u,
             "PMADDUW accumulator mismatch") && ok;

    // PMULTH fills all eight 32-bit packed accumulator slots and exposes
    // even slots through the destination register.
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0x0004000300020001ull,
        0x0008000700060005ull,
    };
    system.ee().state().gpr[2] = {
        0x0002000200020002ull,
        0x0002000200020002ull,
    };
    const ps2::u32 pmulth =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) | (3u << 11) |
        (0x1Cu << 6) | 0x09u;
    run(pmulth);
    ok = expect(
             system.ee().state().lo ==
                 0x0000000400000002ull &&
             system.ee().state().hi ==
                 0x0000000800000006ull &&
             system.ee().state().lo1 ==
                 0x0000000C0000000Aull &&
             system.ee().state().hi1 ==
                 0x000000100000000Eull &&
             system.ee().state().gpr[3].lo ==
                 0x0000000600000002ull &&
             system.ee().state().gpr[3].hi ==
                 0x0000000E0000000Aull,
             "PMULTH accumulator layout mismatch") && ok;

    // PDIVBW divides all four signed words by the first signed halfword.
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0x000000140000000Aull,
        0x00000008FFFFFFF7ull,
    };
    system.ee().state().gpr[2].lo = 3u;
    const ps2::u32 pdivbw =
        (0x1Cu << 26) |
        (1u << 21) | (2u << 16) |
        (0x1Du << 6) | 0x09u;
    run(pdivbw);
    ok = expect(
             system.ee().state().lo ==
                 0x0000000600000003ull &&
             system.ee().state().hi ==
                 0x0000000200000001ull &&
             system.ee().state().lo1 ==
                 0x00000002FFFFFFFDull &&
             system.ee().state().hi1 ==
                 0x0000000200000000ull,
             "PDIVBW quotient/remainder layout mismatch") && ok;

    // PMTHI/PMFHI cover the packed 128-bit HI transfer path.
    system.ee().reset(pc);
    system.ee().state().gpr[1] = {
        0x1122334455667788ull,
        0x99AABBCCDDEEFF00ull,
    };
    const ps2::u32 pmthi =
        (0x1Cu << 26) |
        (1u << 21) |
        (0x08u << 6) | 0x29u;
    run(pmthi);
    const ps2::u32 pmfhi =
        (0x1Cu << 26) |
        (3u << 11) |
        (0x08u << 6) | 0x09u;
    run(pmfhi);
    ok = expect(
             system.ee().state().gpr[3].lo ==
                 0x1122334455667788ull &&
             system.ee().state().gpr[3].hi ==
                 0x99AABBCCDDEEFF00ull,
             "PMTHI/PMFHI packed transfer mismatch") && ok;

    return ok;
}

bool test_cop2_bios_macro_expansion() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2300u;
    std::string error;
    bool ok = true;

    auto special2 = [](
        ps2::u32 sub,
        ps2::u32 ft,
        ps2::u32 fs,
        ps2::u32 selector = 0xFu) {
        return
            (0x12u << 26) |
            ((0x10u | (selector & 0xFu)) << 21) |
            ((ft & 31u) << 16) |
            ((fs & 31u) << 11) |
            (((sub >> 2) & 31u) << 6) |
            (0x3Cu | (sub & 3u));
    };
    auto run = [&](ps2::u32 instruction) {
        ok = expect(
                 system.bus().write32(pc, instruction),
                 "failed to install COP2 macro opcode") && ok;
        system.ee().state().pc = pc;
        system.ee().state().next_pc = pc + 4u;
        error.clear();
        ok = expect(
                 system.ee().step(error),
                 "COP2 BIOS macro execution failed") && ok;
    };

    // VMOVE copies selected VU0 vector lanes without alias corruption.
    system.ee().reset(pc);
    system.ee().state().vu_vf[1] = {
        0x2222222211111111ull,
        0x4444444433333333ull,
    };
    run(special2(0x30u, 2u, 1u));
    ok = expect(
             system.ee().state().vu_vf[2].lo ==
                 0x2222222211111111ull &&
             system.ee().state().vu_vf[2].hi ==
                 0x4444444433333333ull,
             "VU0 VMOVE result mismatch") && ok;

    // VLQI reads VU0 data memory and post-increments its VI address.
    system.ee().reset(pc);
    system.ee().state().vu_vi[1] = 4u;
    ok = expect(
             system.bus().write32(0x11004040u, 0x11111111u) &&
             system.bus().write32(0x11004044u, 0x22222222u) &&
             system.bus().write32(0x11004048u, 0x33333333u) &&
             system.bus().write32(0x1100404Cu, 0x44444444u),
             "VU0 VLQI data setup failed") && ok;
    run(special2(0x34u, 2u, 1u));
    ok = expect(
             system.ee().state().vu_vf[2].lo ==
                 0x2222222211111111ull &&
             system.ee().state().vu_vf[2].hi ==
                 0x4444444433333333ull &&
             system.ee().state().vu_vi[1] == 5u,
             "VU0 VLQI/post-increment mismatch") && ok;

    // VDIV writes the VU0 Q control register.  Selector 0xF chooses W/W.
    system.ee().reset(pc);
    system.ee().state().vu_vf[1].hi =
        static_cast<ps2::u64>(std::bit_cast<ps2::u32>(6.0f)) << 32;
    system.ee().state().vu_vf[2].hi =
        static_cast<ps2::u64>(std::bit_cast<ps2::u32>(2.0f)) << 32;
    run(special2(0x38u, 2u, 1u, 0xFu));
    ok = expect(
             system.ee().state().vu_vi[22] ==
                 std::bit_cast<ps2::u32>(3.0f),
             "VU0 VDIV Q result mismatch") && ok;

    // VMULA + VMADD exercise the VU0 ACC path used by macro-mode
    // vector setup code.
    system.ee().reset(pc);
    const auto fbits = [](float value) {
        return std::bit_cast<ps2::u32>(value);
    };
    system.ee().state().vu_vf[1] = {
        static_cast<ps2::u64>(fbits(1.0f)) |
            (static_cast<ps2::u64>(fbits(2.0f)) << 32),
        static_cast<ps2::u64>(fbits(3.0f)) |
            (static_cast<ps2::u64>(fbits(4.0f)) << 32),
    };
    system.ee().state().vu_vf[2] = {
        static_cast<ps2::u64>(fbits(5.0f)) |
            (static_cast<ps2::u64>(fbits(6.0f)) << 32),
        static_cast<ps2::u64>(fbits(7.0f)) |
            (static_cast<ps2::u64>(fbits(8.0f)) << 32),
    };
    run(special2(0x2Au, 2u, 1u));
    ok = expect(
             system.ee().state().vu_acc.lo ==
                 (static_cast<ps2::u64>(fbits(5.0f)) |
                  (static_cast<ps2::u64>(fbits(12.0f)) << 32)) &&
             system.ee().state().vu_acc.hi ==
                 (static_cast<ps2::u64>(fbits(21.0f)) |
                  (static_cast<ps2::u64>(fbits(32.0f)) << 32)),
             "VU0 VMULA accumulator mismatch") && ok;

    const ps2::u32 vmadd =
        (0x12u << 26) |
        (0x1Fu << 21) |
        (2u << 16) |
        (1u << 11) |
        (3u << 6) |
        0x29u;
    run(vmadd);
    ok = expect(
             system.ee().state().vu_vf[3].lo ==
                 (static_cast<ps2::u64>(fbits(10.0f)) |
                  (static_cast<ps2::u64>(fbits(24.0f)) << 32)) &&
             system.ee().state().vu_vf[3].hi ==
                 (static_cast<ps2::u64>(fbits(42.0f)) |
                  (static_cast<ps2::u64>(fbits(64.0f)) << 32)),
             "VU0 VMADD accumulator result mismatch") && ok;

    // VMTIR/VMFIR round-trip signed 16-bit integer data.
    system.ee().reset(pc);
    system.ee().state().vu_vf[1].hi =
        static_cast<ps2::u64>(0x0000FF80u) << 32;
    run(special2(0x3Cu, 2u, 1u, 0xFu));
    ok = expect(
             system.ee().state().vu_vi[2] == 0xFF80u,
             "VU0 VMTIR result mismatch") && ok;
    run(special2(0x3Du, 3u, 2u, 0xFu));
    ok = expect(
             system.ee().state().vu_vf[3].lo ==
                 0xFFFFFF80FFFFFF80ull &&
             system.ee().state().vu_vf[3].hi ==
                 0xFFFFFF80FFFFFF80ull,
             "VU0 VMFIR sign extension mismatch") && ok;

    // VI arithmetic no longer falls into the generic COP2 hard stop.
    system.ee().reset(pc);
    system.ee().state().vu_vi[1] = 7u;
    system.ee().state().vu_vi[2] = 5u;
    const ps2::u32 visub =
        (0x12u << 26) |
        (0x10u << 21) |
        (2u << 16) |
        (1u << 11) |
        (3u << 6) |
        0x31u;
    run(visub);
    ok = expect(
             system.ee().state().vu_vi[3] == 2u,
             "VU0 VISUB result mismatch") && ok;

    return ok;
}

bool test_unaligned_doubleword_merges() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000;
    constexpr ps2::u32 base = 0x1000;
    constexpr ps2::u64 mem = 0x8877665544332211ull;
    bool ok = system.bus().write64(base, mem);
    std::string error;

    ps2::u32 ldl = (0x1Au << 26) | (2u << 21) | (3u << 16);
    system.bus().write32(pc, ldl);
    system.ee().reset(pc);
    system.ee().state().gpr[2].lo = base;
    system.ee().state().gpr[3].lo = 0x0123456789ABCDEFull;
    ok = expect(system.ee().step(error), "LDL failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0x1123456789ABCDEFull,
                "LDL merge mismatch") && ok;

    ps2::u32 ldr = (0x1Bu << 26) | (2u << 21) | (3u << 16) | 7u;
    system.bus().write32(pc, ldr);
    system.ee().reset(pc);
    system.ee().state().gpr[2].lo = base;
    system.ee().state().gpr[3].lo = 0x0123456789ABCDEFull;
    ok = expect(system.ee().step(error), "LDR failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0x0123456789ABCD88ull,
                "LDR merge mismatch") && ok;
    return ok;
}

bool test_unaligned_word_and_atomic_memory_ops() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000u;
    constexpr ps2::u32 base = 0x1000u;
    bool ok = true;
    std::string error;

    auto run_mem = [&](ps2::u32 opcode, ps2::u16 imm, ps2::u64 rt_value) {
        const ps2::u32 instruction =
            (opcode << 26) | (2u << 21) | (3u << 16) | imm;
        ok = expect(system.bus().write32(pc, instruction),
                    "failed to install EE memory opcode") && ok;
        system.ee().reset(pc);
        system.ee().state().gpr[2].lo = base;
        system.ee().state().gpr[3].lo = rt_value;
        error.clear();
        ok = expect(system.ee().step(error),
                    "EE memory opcode execution failed") && ok;
    };

    ok = expect(system.bus().write32(base, 0x44332211u),
                "failed to seed unaligned word memory") && ok;

    run_mem(0x22u, 1u, 0xAABBCCDDu); // LWL
    ok = expect(
             system.ee().state().gpr[3].lo == 0x000000002211CCDDull,
             "LWL merge mismatch") && ok;

    run_mem(0x26u, 2u, 0xAABBCCDDu); // LWR
    ok = expect(
             system.ee().state().gpr[3].lo == 0xFFFFFFFFAABB4433ull,
             "LWR merge mismatch") && ok;

    ok = expect(system.bus().write32(base, 0x44332211u),
                "failed to reseed SWL memory") && ok;
    run_mem(0x2Au, 1u, 0xA1B2C3D4u); // SWL
    ps2::u32 word = 0;
    ok = expect(system.bus().read32(base, word) &&
                    word == 0x4433A1B2u,
                "SWL merge mismatch") && ok;

    ok = expect(system.bus().write32(base, 0x44332211u),
                "failed to reseed SWR memory") && ok;
    run_mem(0x2Eu, 2u, 0xA1B2C3D4u); // SWR
    ok = expect(system.bus().read32(base, word) &&
                    word == 0xC3D42211u,
                "SWR merge mismatch") && ok;

    ok = expect(system.bus().write32(base, 0x89ABCDEFu),
                "failed to seed LL memory") && ok;
    run_mem(0x30u, 0u, 0u); // LL
    ok = expect(
             system.ee().state().gpr[3].lo == 0xFFFFFFFF89ABCDEFull,
             "LL load/sign extension mismatch") && ok;

    run_mem(0x38u, 0u, 0x12345678u); // SC
    ok = expect(system.bus().read32(base, word) &&
                    word == 0x12345678u,
                "SC store mismatch") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 1u,
                "SC success result mismatch") && ok;

    constexpr ps2::u64 wide = 0x1122334455667788ull;
    ok = expect(system.bus().write64(base, wide),
                "failed to seed LLD memory") && ok;
    run_mem(0x34u, 0u, 0u); // LLD
    ok = expect(system.ee().state().gpr[3].lo == wide,
                "LLD load mismatch") && ok;

    constexpr ps2::u64 replacement = 0x8877665544332211ull;
    run_mem(0x3Cu, 0u, replacement); // SCD
    ps2::u64 wide_read = 0;
    ok = expect(system.bus().read64(base, wide_read) &&
                    wide_read == replacement,
                "SCD store mismatch") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 1u,
                "SCD success result mismatch") && ok;

    return ok;
}

bool test_bootstrap_mmio() {
    ps2::Ps2System system;
    bool ok = true;

    ok = expect(system.bus().write16(0xBA000008u, 0x1234u), "DVE write failed") && ok;
    ps2::u16 h = 0;
    ok = expect(system.bus().read16(0xBA000008u, h) && h == 0x1234u,
                "DVE readback mismatch") && ok;

    ps2::u32 value = 0;
    ok = expect(system.bus().write32(0x10003000u, 9u), "GIF CTRL write failed") && ok;
    ok = expect(system.bus().read32(0x10003000u, value) && value == 9u,
                "GIF CTRL readback mismatch") && ok;
    ok = expect(system.bus().write64(0x10006000u, 0x1122334455667788ull) &&
                system.bus().write64(0x10006008u, 0x99AABBCCDDEEFF00ull),
                "GIF FIFO write failed") && ok;

    for (ps2::u32 i = 0; i < 4; ++i) {
        const ps2::u32 base = 0x10000000u + i * 0x800u;
        ok = expect(system.bus().write32(base + 0x20u, 0xBEEFu), "timer COMP write failed") && ok;
        ok = expect(system.bus().read32(base + 0x20u, value) && value == 0xBEEFu,
                    "timer COMP read mismatch") && ok;
        ok = expect(system.bus().write32(base + 0x30u, 0x1234u), "timer HOLD write failed") && ok;
    }

    ok = expect(system.bus().write32(0x10008080u, 0xFFFFFFFFu), "DMAC SADR write failed") && ok;
    ok = expect(system.bus().read32(0x10008080u, value) && value == 0x3FF0u,
                "DMAC SADR mask mismatch") && ok;
    ok = expect(system.bus().write32(0x1000E010u, 0x00010000u), "DMAC STAT mask toggle failed") && ok;
    ok = expect(system.bus().read32(0x1000E010u, value) && value == 0x00010000u,
                "DMAC STAT mask did not toggle on") && ok;
    ok = expect(system.bus().write32(0x1000E010u, 0x00010000u) &&
                system.bus().read32(0x1000E010u, value) && value == 0,
                "DMAC STAT mask did not toggle off") && ok;

    ok = expect(system.bus().write32(0x10003C10u, 1u), "VIF1 reset failed") && ok;
    ok = expect(system.bus().write32(0x10003C20u, 2u), "VIF1 ERR write failed") && ok;
    ok = expect(system.bus().read32(0x10003C20u, value) && value == 2u,
                "VIF1 ERR readback mismatch") && ok;
    ok = expect(system.bus().write64(0x10005000u, 0x0123456789ABCDEFull),
                "VIF1 FIFO write failed") && ok;

    ok = expect(system.bus().write32(0x10002010u, 0x40000000u), "IPU reset write failed") && ok;
    ok = expect(system.bus().read32(0x10002010u, value) && value == 0,
                "IPU reset did not clear CTRL") && ok;
    ok = expect(system.bus().write64(0x10007010u, 0xCAFEBABEDEADBEEFull),
                "IPU input FIFO write failed") && ok;

    return ok;
}

bool test_ee_timer_events() {
    ps2::Ps2System system;
    constexpr ps2::u32 count = 0x10000000u;
    constexpr ps2::u32 mode = 0x10000010u;
    constexpr ps2::u32 comp = 0x10000020u;
    constexpr ps2::u32 intc_stat = 0x1000F000u;
    constexpr ps2::u32 intc_mask = 0x1000F010u;

    bool ok = true;
    ps2::u32 value = 0;

    // BUSCLK timer, zero-return, enabled, compare IRQ enabled.
    constexpr ps2::u32 compare_mode =
        (1u << 6) | (1u << 7) | (1u << 8);
    ok = expect(system.bus().write32(intc_mask, 1u << 9),
                "timer INTC mask enable failed") && ok;
    ok = expect(system.bus().write32(count, 0u) &&
                system.bus().write32(comp, 3u) &&
                system.bus().write32(mode, compare_mode),
                "timer compare setup failed") && ok;

    system.bus().tick(6u); // BUSCLK is EE clock / 2 => three timer ticks.
    ok = expect(system.bus().read32(count, value) && value == 0u,
                "timer zero-return compare mismatch") && ok;
    ok = expect(system.bus().read32(mode, value) &&
                (value & (1u << 10)) != 0,
                "timer compare flag missing") && ok;
    ok = expect(system.bus().read32(intc_stat, value) &&
                (value & (1u << 9)) != 0,
                "timer compare IRQ missing") && ok;

    // Clear both the INTC cause and sticky compare event, then ensure a later
    // compare edge can raise the source again.
    ok = expect(system.bus().write32(intc_stat, 1u << 9) &&
                system.bus().write32(mode, compare_mode | (1u << 10)),
                "timer compare acknowledge failed") && ok;
    system.bus().tick(6u);
    ok = expect(system.bus().read32(intc_stat, value) &&
                (value & (1u << 9)) != 0,
                "timer compare IRQ did not retrigger after flag clear") && ok;

    // Overflow has its own sticky flag/enable but shares the same timer INTC
    // source.
    ok = expect(system.bus().write32(intc_stat, 1u << 9) &&
                system.bus().write32(mode, (1u << 7) | (1u << 9) |
                                           (1u << 10) | (1u << 11)) &&
                system.bus().write32(count, 0xFFFFu),
                "timer overflow setup failed") && ok;
    system.bus().tick(2u);
    ok = expect(system.bus().read32(count, value) && value == 0u,
                "timer overflow count mismatch") && ok;
    ok = expect(system.bus().read32(mode, value) &&
                (value & (1u << 11)) != 0,
                "timer overflow flag missing") && ok;
    ok = expect(system.bus().read32(intc_stat, value) &&
                (value & (1u << 9)) != 0,
                "timer overflow IRQ missing") && ok;
    return ok;
}

bool test_ee_timer_bulk_tick_matches_scalar() {
    constexpr ps2::u32 count = 0x10000000u;
    constexpr ps2::u32 mode = 0x10000010u;
    constexpr ps2::u32 comp = 0x10000020u;
    constexpr ps2::u32 intc_stat = 0x1000F000u;
    constexpr ps2::u32 intc_mask = 0x1000F010u;

    auto run_case = [&](ps2::u32 initial_count,
                        ps2::u32 compare,
                        ps2::u32 timer_mode,
                        ps2::u32 cycles) {
        ps2::EeHw bulk;
        ps2::EeHw scalar;
        bulk.reset();
        scalar.reset();

        bool ok = expect(
            bulk.write32(intc_mask, 1u << 9) &&
            scalar.write32(intc_mask, 1u << 9) &&
            bulk.write32(count, initial_count) &&
            scalar.write32(count, initial_count) &&
            bulk.write32(comp, compare) &&
            scalar.write32(comp, compare) &&
            bulk.write32(mode, timer_mode) &&
            scalar.write32(mode, timer_mode),
            "EE bulk timer test setup failed");

        bulk.tick(cycles);
        for (ps2::u32 i = 0; i < cycles; ++i) {
            scalar.tick(1u);
        }

        ps2::u32 bulk_count = 0;
        ps2::u32 scalar_count = 0;
        ps2::u32 bulk_mode = 0;
        ps2::u32 scalar_mode = 0;
        ps2::u32 bulk_stat = 0;
        ps2::u32 scalar_stat = 0;
        ok = expect(
            bulk.read32(count, bulk_count) &&
            scalar.read32(count, scalar_count) &&
            bulk.read32(mode, bulk_mode) &&
            scalar.read32(mode, scalar_mode) &&
            bulk.read32(intc_stat, bulk_stat) &&
            scalar.read32(intc_stat, scalar_stat),
            "EE bulk timer state read failed") && ok;
        ok = expect(
            bulk_count == scalar_count &&
            bulk_mode == scalar_mode &&
            bulk_stat == scalar_stat &&
            bulk.cycles() == scalar.cycles(),
            "EE bulk timer advance diverged from scalar ticking") && ok;
        return ok;
    };

    bool ok = true;
    ok = run_case(
        0u, 3u,
        (1u << 6) | (1u << 7) | (1u << 8),
        123u) && ok;
    ok = run_case(
        0xFFF0u, 0x3456u,
        (1u << 7) | (1u << 9),
        100u) && ok;
    ok = run_case(
        0xFFFEu, 3u,
        (1u << 6) | (1u << 7) |
        (1u << 8) | (1u << 9),
        40u) && ok;
    ok = run_case(
        0x1200u, 0x1220u,
        1u | (1u << 7) | (1u << 8),
        1001u) && ok;
    return ok;
}

bool test_ee_timer_irq_distance() {
    ps2::EeHw hw;
    hw.reset();

    constexpr ps2::u32 count = 0x10000000u;
    constexpr ps2::u32 mode = 0x10000010u;
    constexpr ps2::u32 comp = 0x10000020u;
    constexpr ps2::u32 intc_mask = 0x1000F010u;
    constexpr ps2::u32 compare_mode =
        (1u << 6) | (1u << 7) | (1u << 8);

    bool ok = expect(
        hw.write32(intc_mask, 1u << 9) &&
        hw.write32(count, 0u) &&
        hw.write32(comp, 3u) &&
        hw.write32(mode, compare_mode),
        "EE timer distance setup failed");
    ok = expect(
        hw.cycles_to_timer_irq() == 6u,
        "EE timer distance did not match compare edge") && ok;

    hw.tick(5u);
    ok = expect(
        hw.cycles_to_timer_irq() == 1u &&
        !hw.intc_pending(),
        "EE timer distance did not preserve partial phase") && ok;

    hw.tick(1u);
    ok = expect(
        hw.intc_pending() &&
        hw.cycles_to_timer_irq() == ~ps2::u64{0},
        "EE timer distance did not retire the sticky IRQ edge") && ok;

    hw.reset();
    ok = expect(
        hw.write32(intc_mask, 1u << 9) &&
        hw.write32(comp, 3u) &&
        hw.write32(mode, (1u << 8)),
        "EE disabled timer distance setup failed") && ok;
    ok = expect(
        hw.cycles_to_timer_irq() == ~ps2::u64{0},
        "disabled EE timer incorrectly blocked batching") && ok;
    return ok;
}

bool test_ee_intc_register_semantics() {
    ps2::Ps2System system;
    ps2::u32 value = 0;
    bool ok = expect(system.bus().write32(0x1000F010u, 0x0003u), "INTC_MASK initial toggle failed");
    ok = expect(system.bus().read32(0x1000F010u, value) && value == 0x0003u,
                "INTC_MASK initial value mismatch") && ok;
    ok = expect(system.bus().write32(0x1000F010u, 0x0001u) &&
                system.bus().read32(0x1000F010u, value) && value == 0x0002u,
                "INTC_MASK XOR semantics mismatch") && ok;

    ok = expect(system.iop_bus().write32(0x1F801450u, 0x2u), "IOP SBUS INTC raise failed") && ok;
    ok = expect(system.bus().read32(0x1000F000u, value) && value == 0x0002u,
                "INTC_STAT raise mismatch") && ok;
    ok = expect(system.bus().write32(0x1000F000u, 0x0002u) &&
                system.bus().read32(0x1000F000u, value) && value == 0,
                "INTC_STAT write-one-to-clear mismatch") && ok;
    ok = expect(system.iop_bus().write32(0x1F801450u, 0x2u), "IOP SBUS IRQ write failed") && ok;
    ok = expect(system.bus().read32(0x1000F000u, value) && (value & 0x2u) != 0,
                "IOP SBUS write did not raise EE INTC bit 1") && ok;
    return ok;
}

bool test_vu_mapping_and_cop2() {
    ps2::Ps2System system;
    auto acc_lane = [&system](ps2::u32 lane) -> ps2::u32 {
        const auto& acc = system.ee().state().vu_acc;
        const ps2::u64 half = lane < 2u ? acc.lo : acc.hi;
        return static_cast<ps2::u32>(half >> ((lane & 1u) * 32u));
    };
    bool ok = system.bus().write32(0x11004000u, 0xAABBCCDDu);
    ps2::u32 value = 0;
    ok = expect(ok && system.bus().read32(0x11005000u, value) && value == 0xAABBCCDDu,
                "VU0 data mirror mismatch") && ok;

    constexpr ps2::u32 pc = 0x2000;
    // CTC2 r2, FBRST then CFC2 r3, FBRST.
    const ps2::u32 ctc2 = (0x12u << 26) | (0x06u << 21) | (2u << 16) | (28u << 11);
    system.bus().write32(pc, ctc2);
    system.ee().reset(pc);
    system.ee().state().gpr[2].lo = 0x202u;
    std::string error;
    ok = expect(system.ee().step(error), "CTC2 FBRST failed") && ok;
    ok = expect(system.ee().state().vu_vi[28] == 0, "FBRST writable mask mismatch") && ok;

    const ps2::u32 cfc2 = (0x12u << 26) | (0x02u << 21) | (3u << 16) | (28u << 11);
    system.bus().write32(pc, cfc2);
    system.ee().reset(pc);
    ok = expect(system.ee().step(error), "CFC2 FBRST failed") && ok;
    ok = expect(system.ee().state().gpr[3].lo == 0, "CFC2 FBRST result mismatch") && ok;

    auto pack2 = [](float x, float y) -> ps2::u64 {
        return static_cast<ps2::u64>(std::bit_cast<ps2::u32>(x)) |
               (static_cast<ps2::u64>(std::bit_cast<ps2::u32>(y)) << 32);
    };

    // VADD.xyzw vf3,vf1,vf2.
    const ps2::u32 vadd =
        (0x12u << 26) |
        (0x1Fu << 21) |
        (2u << 16) |
        (1u << 11) |
        (3u << 6) |
        0x28u;
    ok = expect(system.bus().write32(pc, vadd),
                "VU0 VADD test write failed") && ok;
    system.ee().reset(pc);
    system.ee().state().vu_vf[1] = {
        pack2(1.0f, 2.0f),
        pack2(3.0f, 4.0f),
    };
    system.ee().state().vu_vf[2] = {
        pack2(10.0f, 20.0f),
        pack2(30.0f, 40.0f),
    };
    ok = expect(system.ee().step(error),
                "VU0 VADD macro execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vf[3].lo ==
                 pack2(11.0f, 22.0f) &&
             system.ee().state().vu_vf[3].hi ==
                 pack2(33.0f, 44.0f),
             "VU0 VADD macro result mismatch") && ok;

    // VMULx.xz vf4,vf1,vf2: only X and Z lanes update, scalar is vf2.x.
    const ps2::u32 vmulx_xz =
        (0x12u << 26) |
        (0x1Au << 21) |
        (2u << 16) |
        (1u << 11) |
        (4u << 6) |
        0x18u;
    ok = expect(system.bus().write32(pc, vmulx_xz),
                "VU0 VMULx test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vf[4] = {
        pack2(99.0f, 98.0f),
        pack2(97.0f, 96.0f),
    };
    ok = expect(system.ee().step(error),
                "VU0 VMULx macro execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vf[4].lo ==
                 pack2(10.0f, 98.0f) &&
             system.ee().state().vu_vf[4].hi ==
                 pack2(30.0f, 96.0f),
             "VU0 VMULx destination mask mismatch") && ok;

    // VMR32.xyzw vf5,vf4 rotates Y/Z/W/X into the destination lanes.
    const ps2::u32 vmr32 =
        (0x12u << 26) |
        (0x1Fu << 21) |
        (5u << 16) |
        (4u << 11) |
        (0x0Cu << 6) |
        0x3Du;
    ok = expect(system.bus().write32(pc, vmr32),
                "VU0 VMR32 test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vf[4] = {
        0x2222222211111111ull,
        0x4444444433333333ull,
    };
    ok = expect(system.ee().step(error),
                "VU0 VMR32 macro execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vf[5].lo ==
                 0x3333333322222222ull &&
             system.ee().state().vu_vf[5].hi ==
                 0x1111111144444444ull,
             "VU0 VMR32 rotation mismatch") && ok;

    // VMFIR.xz vf4, vi5 sign-extends VI and preserves masked-off lanes.
    const ps2::u32 vmfir_xz =
        (0x12u << 26) |
        (0x1Au << 21) |
        (4u << 16) |
        (5u << 11) |
        (0x0Fu << 6) |
        0x3Du;
    ok = expect(system.bus().write32(pc, vmfir_xz),
                "VU0 VMFIR test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vi[5] = 0x8001u;
    system.ee().state().vu_vf[4] = {
        0x2222222211111111ull,
        0x4444444433333333ull,
    };
    ok = expect(system.ee().step(error),
                "VU0 VMFIR macro execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vf[4].lo ==
                 0x22222222FFFF8001ull &&
             system.ee().state().vu_vf[4].hi ==
                 0x44444444FFFF8001ull,
             "VU0 VMFIR sign extension or destination mask mismatch") && ok;

    // VSQRT/VDIV feed Q; WAITQ is synchronous in the interpreter.
    ok = expect(
             system.bus().write32(pc, 0x4A2503BDu) &&
                 system.bus().write32(pc + 4u, 0x4A0003BFu) &&
                 system.bus().write32(pc + 8u, 0x4A6503BCu),
             "VU0 Q arithmetic instructions write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vf[5].lo = pack2(16.0f, 0.0f);
    ok = expect(system.ee().step(error),
                "VU0 VSQRT macro execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vi[22] ==
                 std::bit_cast<ps2::u32>(4.0f),
             "VU0 VSQRT Q result mismatch") && ok;
    ok = expect(system.ee().step(error),
                "VU0 VWAITQ macro execution failed") && ok;
    ok = expect(system.ee().step(error),
                "VU0 VDIV macro execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vi[22] ==
                 std::bit_cast<ps2::u32>(0.0625f),
             "VU0 VDIV Q result mismatch") && ok;

    // VADDq.xz uses the Q scalar, leaving Y/W untouched.
    const ps2::u32 vaddq_xz =
        (0x12u << 26) |
        (0x1Au << 21) |
        (1u << 11) |
        (3u << 6) |
        0x20u;
    ok = expect(system.bus().write32(pc, vaddq_xz),
                "VU0 VADDq test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vi[22] = std::bit_cast<ps2::u32>(2.5f);
    system.ee().state().vu_vf[1] = {
        pack2(1.0f, 2.0f),
        pack2(3.0f, 4.0f),
    };
    system.ee().state().vu_vf[3] = {
        pack2(90.0f, 91.0f),
        pack2(92.0f, 93.0f),
    };
    ok = expect(system.ee().step(error),
                "VU0 VADDq macro execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vf[3].lo ==
                 pack2(3.5f, 91.0f) &&
             system.ee().state().vu_vf[3].hi ==
                 pack2(5.5f, 93.0f),
             "VU0 VADDq scalar or destination mask mismatch") && ok;

    // VMTIR vi5, vf4.z copies only the low 16 bits of the selected lane.
    const ps2::u32 vmtir_z =
        (0x12u << 26) |
        (0x12u << 21) |
        (5u << 16) |
        (4u << 11) |
        (0x0Fu << 6) |
        0x3Cu;
    ok = expect(system.bus().write32(pc, vmtir_z),
                "VU0 VMTIR test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vf[4].hi =
        0x11111111DEADBEEFull;
    ok = expect(system.ee().step(error),
                "VU0 VMTIR macro execution failed") && ok;
    ok = expect(system.ee().state().vu_vi[5] == 0xBEEFu,
                "VU0 VMTIR source lane or halfword mismatch") && ok;

    // VMOVE.yw vf6, vf4 copies only the selected lanes.
    const ps2::u32 vmove_yw =
        (0x12u << 26) |
        (0x15u << 21) |
        (6u << 16) |
        (4u << 11) |
        (0x0Cu << 6) |
        0x3Cu;
    ok = expect(system.bus().write32(pc, vmove_yw),
                "VU0 VMOVE test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vf[4] = {
        0x2222222211111111ull,
        0x4444444433333333ull};
    system.ee().state().vu_vf[6] = {
        0xBBBBBBBBAAAAAAAAull,
        0xDDDDDDDDCCCCCCCCull};
    ok = expect(system.ee().step(error),
                "VU0 VMOVE macro execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vf[6].lo ==
                 0x22222222AAAAAAAAull &&
             system.ee().state().vu_vf[6].hi ==
                 0x44444444CCCCCCCCull,
             "VU0 VMOVE destination mask mismatch") && ok;

    // OPMULA followed by OPMSUB with exchanged operands forms a cross product.
    const ps2::u32 vopmula =
        (0x12u << 26) |
        (0x1Fu << 21) |
        (2u << 16) |
        (1u << 11) |
        (0x0Bu << 6) |
        0x3Eu;
    const ps2::u32 vopmsub =
        (0x12u << 26) |
        (0x1Fu << 21) |
        (1u << 16) |
        (2u << 11) |
        (3u << 6) |
        0x2Eu;
    ok = expect(
             system.bus().write32(pc, vopmula) &&
                 system.bus().write32(pc + 4u, vopmsub),
             "VU0 outer-product instructions write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vf[1] = {
        pack2(1.0f, 2.0f), pack2(3.0f, 4.0f)};
    system.ee().state().vu_vf[2] = {
        pack2(4.0f, 5.0f), pack2(6.0f, 7.0f)};
    system.ee().state().vu_vf[3] = {
        pack2(90.0f, 91.0f), pack2(92.0f, 93.0f)};
    ok = expect(
             system.ee().step(error) && system.ee().step(error),
             "VU0 outer-product execution failed") && ok;
    ok = expect(
             acc_lane(0u) ==
                 std::bit_cast<ps2::u32>(12.0f) &&
             acc_lane(1u) ==
                 std::bit_cast<ps2::u32>(12.0f) &&
             acc_lane(2u) ==
                 std::bit_cast<ps2::u32>(5.0f) &&
             system.ee().state().vu_vf[3].lo ==
                 pack2(-3.0f, 6.0f) &&
             system.ee().state().vu_vf[3].hi ==
                 pack2(-3.0f, 93.0f),
             "VU0 outer-product accumulator or result mismatch") && ok;

    // VMULAx.xz and VMADDA.xz update only selected ACC lanes.
    const ps2::u32 vmulax_xz =
        (0x12u << 26) |
        (0x1Au << 21) |
        (2u << 16) |
        (1u << 11) |
        (0x06u << 6) |
        0x3Cu;
    const ps2::u32 vmadda_xz =
        (0x12u << 26) |
        (0x1Au << 21) |
        (2u << 16) |
        (1u << 11) |
        (0x0Au << 6) |
        0x3Du;
    ok = expect(
             system.bus().write32(pc, vmulax_xz) &&
                 system.bus().write32(pc + 4u, vmadda_xz),
             "VU0 accumulator macro instructions write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vf[1] = {
        pack2(1.0f, 2.0f), pack2(3.0f, 4.0f)};
    system.ee().state().vu_vf[2] = {
        pack2(4.0f, 5.0f), pack2(6.0f, 7.0f)};
    system.ee().state().vu_acc = {
        pack2(90.0f, 91.0f),
        pack2(92.0f, 93.0f)};
    ok = expect(
             system.ee().step(error) && system.ee().step(error),
             "VU0 accumulator macro execution failed") && ok;
    ok = expect(
             acc_lane(0u) ==
                 std::bit_cast<ps2::u32>(8.0f) &&
             acc_lane(1u) ==
                 std::bit_cast<ps2::u32>(91.0f) &&
             acc_lane(2u) ==
                 std::bit_cast<ps2::u32>(30.0f) &&
             acc_lane(3u) ==
                 std::bit_cast<ps2::u32>(93.0f),
             "VU0 accumulator arithmetic or mask mismatch") && ok;

    // VMADDz.xz reads ACC and broadcasts VF[ft].z into selected lanes.
    const ps2::u32 vmaddz_xz =
        (0x12u << 26) |
        (0x1Au << 21) |
        (2u << 16) |
        (1u << 11) |
        (3u << 6) |
        0x0Au;
    ok = expect(system.bus().write32(pc, vmaddz_xz),
                "VU0 VMADDz test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vf[3] = {
        pack2(90.0f, 91.0f), pack2(92.0f, 93.0f)};
    ok = expect(system.ee().step(error),
                "VU0 VMADDz macro execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vf[3].lo ==
                 pack2(14.0f, 91.0f) &&
             system.ee().state().vu_vf[3].hi ==
                 pack2(48.0f, 93.0f),
             "VU0 VMADDz scalar, accumulator, or mask mismatch") && ok;

    ok = expect(system.bus().write32(pc, 0x4A0002FFu),
                "VU0 VNOP test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    ok = expect(system.ee().step(error),
                "VU0 VNOP macro execution failed") && ok;

    // VFTOI4.xz and VITOF4.xz convert between float and signed 12:4.
    const ps2::u32 vftoi4_xz =
        (0x12u << 26) |
        (0x1Au << 21) |
        (6u << 16) |
        (4u << 11) |
        (0x05u << 6) |
        0x3Du;
    const ps2::u32 vitof4_xz =
        (0x12u << 26) |
        (0x1Au << 21) |
        (7u << 16) |
        (6u << 11) |
        (0x04u << 6) |
        0x3Du;
    ok = expect(
             system.bus().write32(pc, vftoi4_xz) &&
                 system.bus().write32(pc + 4u, vitof4_xz),
             "VU0 fixed-point conversion instructions write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vf[4] = {
        pack2(12.5f, 20.0f), pack2(-1.5f, 30.0f)};
    system.ee().state().vu_vf[6] = {
        0xAAAAAAAAAAAAAAAAull, 0xBBBBBBBBBBBBBBBBull};
    ok = expect(
             system.ee().step(error) && system.ee().step(error),
             "VU0 fixed-point conversion execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vf[6].lo ==
                 0xAAAAAAAA000000C8ull &&
             system.ee().state().vu_vf[6].hi ==
                 0xBBBBBBBBFFFFFFE8ull &&
             system.ee().state().vu_vf[7].lo ==
                 pack2(12.5f, 0.0f) &&
             static_cast<ps2::u32>(
                 system.ee().state().vu_vf[7].hi) ==
                 std::bit_cast<ps2::u32>(-1.5f),
             "VU0 fixed-point conversion or mask mismatch") && ok;

    // VISWR.x vi3, (vi2) stores the low integer register halfword.
    const ps2::u32 viswr_x =
        (0x12u << 26) |
        (0x18u << 21) |
        (3u << 16) |
        (2u << 11) |
        (0x0Fu << 6) |
        0x3Fu;
    ok = expect(system.bus().write32(pc, viswr_x),
                "VU0 VISWR test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vi[2] = 1u;
    system.ee().state().vu_vi[3] = 0x1234u;
    ok = expect(system.ee().step(error),
                "VU0 VISWR macro execution failed") && ok;
    ok = expect(system.bus().read32(0x11004010u, value) && value == 0x1234u,
                "VU0 VISWR integer memory store mismatch") && ok;

    // This exact retail-BIOS encoding is VISWR.x vi0,(vi1), not padding.
    system.ee().state().vu_vi[1] = 1u;
    ok = expect(system.bus().write32(pc, 0x4B000BFFu),
                "VU0 BIOS VISWR test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    ok = expect(system.ee().step(error),
                "VU0 BIOS VISWR macro execution failed") && ok;
    ok = expect(system.bus().read32(0x11004010u, value) && value == 0u,
                "VU0 BIOS VISWR did not clear VU memory") && ok;

    // VILWR.x vi4,(vi2) loads the low halfword from a selected VU lane.
    const ps2::u32 vilwr_x =
        (0x12u << 26) |
        (0x18u << 21) |
        (4u << 16) |
        (2u << 11) |
        (0x0Fu << 6) |
        0x3Eu;
    ok = expect(
             system.bus().write32(0x11004020u, 0xDEADBEEFu) &&
                 system.bus().write32(pc, vilwr_x),
             "VU0 VILWR test setup failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().vu_vi[2] = 2u;
    ok = expect(system.ee().step(error),
                "VU0 VILWR macro execution failed") && ok;
    ok = expect(system.ee().state().vu_vi[4] == 0xBEEFu,
                "VU0 VILWR integer load mismatch") && ok;

    // SQC2/LQC2 round-trip a vector through EE memory.
    const ps2::u32 sqc2 =
        (0x3Eu << 26) | (6u << 21) | (5u << 16) | 0x30u;
    const ps2::u32 lqc2 =
        (0x36u << 26) | (6u << 21) | (7u << 16) | 0x30u;
    ok = expect(
             system.bus().write32(pc, sqc2) &&
             system.bus().write32(pc + 4u, lqc2),
             "VU0 quadword memory test write failed") && ok;
    system.ee().state().pc = pc;
    system.ee().state().next_pc = pc + 4u;
    system.ee().state().gpr[6].lo = 0x3000u;
    system.ee().state().vu_vf[5] = {
        0x0123456789ABCDEFull,
        0xFFEEDDCCBBAA9988ull,
    };
    ok = expect(system.ee().step(error) && system.ee().step(error),
                "VU0 SQC2/LQC2 execution failed") && ok;
    ok = expect(
             system.ee().state().vu_vf[7].lo ==
                 0x0123456789ABCDEFull &&
             system.ee().state().vu_vf[7].hi ==
                 0xFFEEDDCCBBAA9988ull,
             "VU0 SQC2/LQC2 round-trip mismatch") && ok;

    return ok;
}

bool test_ee_intc_cpu_exception() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2800;
    bool ok = expect(system.bus().write32(pc, 0), "INTC test NOP write failed");
    ok = expect(system.bus().write32(0x1000F010u, 1u << 1), "INTC test mask enable failed") && ok;
    ok = expect(system.iop_bus().write32(0x1F801450u, 0x2u), "INTC CPU source raise failed") && ok;
    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0x00010401u; // EIE | IP2 mask | IE
    std::string error;
    ok = expect(system.ee().step(error), "INTC CPU exception failed") && ok;
    ok = expect(system.ee().state().pc == 0x80000200u, "INTC vector mismatch") && ok;
    ok = expect(system.ee().state().cop0[14] == pc, "INTC EPC mismatch") && ok;
    ok = expect((system.ee().state().cop0[13] & 0x0000047Cu) == 0x00000400u,
                "INTC Cause mismatch") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x2u) != 0, "INTC did not set EXL") && ok;
    return ok;
}

bool test_ee_cop0_count_compare_irq() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2A00u;
    bool ok = expect(
        system.bus().write32(pc, 0u) &&
        system.bus().write32(pc + 4u, 0u),
        "COP0 timer NOP setup failed");

    system.ee().reset(pc);
    system.ee().state().cop0[9] = 5u;
    system.ee().state().cop0[11] = 6u;
    system.ee().state().cop0[12] =
        0x00010001u | 0x00008000u; // EIE | IE | IP7 mask.

    std::string error;
    ok = expect(system.ee().step(error),
                "COP0 timer compare-producing step failed") && ok;
    ok = expect((system.ee().state().cop0[13] & 0x00008000u) != 0,
                "COP0 Count==Compare did not assert IP7") && ok;

    ok = expect(system.ee().step(error),
                "COP0 timer interrupt exception failed") && ok;
    ok = expect(system.ee().state().pc == 0x80000200u,
                "COP0 timer interrupt vector mismatch") && ok;
    ok = expect(system.ee().state().cop0[14] == pc + 4u,
                "COP0 timer EPC mismatch") && ok;

    // MTC0 Compare must acknowledge the pending timer line.
    constexpr ps2::u32 mtc0_compare =
        (0x10u << 26) | (0x04u << 21) | (2u << 16) | (11u << 11);
    ok = expect(system.bus().write32(pc, mtc0_compare),
                "MTC0 Compare setup failed") && ok;
    system.ee().reset(pc);
    system.ee().state().cop0[13] = 0x00008000u;
    system.ee().state().gpr[2].lo = 0x12345678u;
    ok = expect(system.ee().step(error),
                "MTC0 Compare execution failed") && ok;
    ok = expect(system.ee().state().cop0[11] == 0x12345678u &&
                (system.ee().state().cop0[13] & 0x00008000u) == 0,
                "MTC0 Compare did not clear IP7") && ok;
    return ok;
}

bool test_ee_bc0_dmac_condition_branches() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2B00u;
    std::string error;
    bool ok = true;

    // BC0F with every DMAC channel enabled and none complete: CPCOND0=false.
    const ps2::u32 bc0f =
        (0x10u << 26) |
        (0x08u << 21) |
        (0u << 16) |
        2u;
    ok = expect(
             system.bus().write32(pc, bc0f) &&
             system.bus().write32(0x1000E020u, 0x3FFu),
             "BC0F setup failed") && ok;
    system.ee().reset(pc);
    ok = expect(system.ee().step(error), "BC0F execution failed") && ok;
    ok = expect(
             system.ee().state().pc == pc + 4u &&
             system.ee().state().next_pc == pc + 12u,
             "BC0F did not branch on incomplete DMAC condition") && ok;

    // BC0TL is taken when no channels participate: (~CPC) satisfies condition.
    const ps2::u32 bc0tl =
        (0x10u << 26) |
        (0x08u << 21) |
        (3u << 16) |
        2u;
    ok = expect(
             system.bus().write32(pc, bc0tl) &&
             system.bus().write32(0x1000E020u, 0u),
             "BC0TL setup failed") && ok;
    system.ee().reset(pc);
    error.clear();
    ok = expect(system.ee().step(error), "BC0TL execution failed") && ok;
    ok = expect(
             system.ee().state().pc == pc + 4u &&
             system.ee().state().next_pc == pc + 12u,
             "BC0TL did not branch on satisfied DMAC condition") && ok;

    // BC0FL must skip its delay slot when the false condition is not met.
    const ps2::u32 bc0fl =
        (0x10u << 26) |
        (0x08u << 21) |
        (2u << 16) |
        2u;
    ok = expect(
             system.bus().write32(pc, bc0fl),
             "BC0FL setup failed") && ok;
    system.ee().reset(pc);
    error.clear();
    ok = expect(system.ee().step(error), "BC0FL execution failed") && ok;
    ok = expect(
             system.ee().state().pc == pc + 8u &&
             system.ee().state().next_pc == pc + 12u,
             "BC0FL likely-not-taken skip mismatch") && ok;

    return ok;
}

bool test_ee_di_ei_privilege_gate() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2C00;
    // EI / DI COP0 functions.
    const ps2::u32 ei = 0x42000038u;
    const ps2::u32 di = 0x42000039u;
    std::string error;
    bool ok = expect(system.bus().write32(pc, ei) && system.bus().write32(pc + 4u, di),
                     "EI/DI test code write failed");

    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0x10u; // user KSU, _EDI=EXL=ERL=0
    ok = expect(system.ee().step(error), "gated EI execution failed") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x10000u) == 0,
                "EI ignored privilege gate") && ok;

    system.ee().reset(pc + 4u);
    system.ee().state().cop0[12] = 0x10010u; // user KSU, EIE=1, _EDI=0
    ok = expect(system.ee().step(error), "gated DI execution failed") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x10000u) != 0,
                "DI ignored privilege gate") && ok;

    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0; // kernel KSU
    ok = expect(system.ee().step(error), "kernel EI execution failed") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x10000u) != 0,
                "kernel EI did not set EIE") && ok;

    system.ee().reset(pc + 4u);
    system.ee().state().cop0[12] = 0x10000u; // kernel KSU, EIE=1
    ok = expect(system.ee().step(error), "kernel DI execution failed") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x10000u) == 0,
                "kernel DI did not clear EIE") && ok;
    return ok;
}

bool test_ee_break_exception_and_tlb_ops() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x3200u;
    std::string error;
    bool ok = true;

    ok = expect(
             system.bus().write32(pc, 0x0000000Du),
             "BREAK test write failed") && ok;
    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0u;
    ok = expect(
             system.ee().step(error),
             "BREAK exception execution failed") && ok;
    ok = expect(
             !system.ee().halted() &&
             system.ee().state().pc == 0x80000180u &&
             system.ee().state().cop0[14] == pc &&
             (system.ee().state().cop0[13] & 0x7Cu) == 0x24u,
             "BREAK did not raise architectural exception") && ok;

    constexpr ps2::u32 tlbwi = 0x42000002u;
    constexpr ps2::u32 tlbp  = 0x42000008u;
    constexpr ps2::u32 tlbr  = 0x42000001u;
    ok = expect(
             system.bus().write32(pc + 0u, tlbwi) &&
             system.bus().write32(pc + 4u, tlbp) &&
             system.bus().write32(pc + 8u, tlbr),
             "TLB test code write failed") && ok;

    system.ee().reset(pc);
    system.ee().state().cop0[0] = 7u;
    system.ee().state().cop0[5] = 0x00006000u;
    system.ee().state().cop0[10] = 0x1234402Au;
    system.ee().state().cop0[2] = 0x0012341Fu;
    system.ee().state().cop0[3] = 0x0056781Fu;

    ok = expect(system.ee().step(error),
                "TLBWI execution failed") && ok;

    // Probe the same VPN/ASID after clobbering the index.
    system.ee().state().cop0[0] = 0x80000000u;
    system.ee().state().cop0[10] = 0x1234402Au;
    ok = expect(system.ee().step(error),
                "TLBP execution failed") && ok;
    ok = expect(
             system.ee().state().cop0[0] == 7u,
             "TLBP did not find written entry") && ok;

    system.ee().state().cop0[5] = 0u;
    system.ee().state().cop0[10] = 0u;
    system.ee().state().cop0[2] = 0u;
    system.ee().state().cop0[3] = 0u;
    ok = expect(system.ee().step(error),
                "TLBR execution failed") && ok;
    ok = expect(
             system.ee().state().cop0[5] == 0x00006000u &&
             system.ee().state().cop0[10] == 0x1234402Au &&
             system.ee().state().cop0[2] == 0x0012341Fu &&
             system.ee().state().cop0[3] == 0x0056781Fu,
             "TLBR state round-trip mismatch") && ok;

    return ok;
}

bool test_ee_trap_instructions() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x3800u;

    // TEQ r1,r2 and TNEI r1,7.
    constexpr ps2::u32 teq =
        (1u << 21) | (2u << 16) | 0x34u;
    constexpr ps2::u32 tnei =
        (0x01u << 26) | (1u << 21) | (0x0Eu << 16) | 7u;

    bool ok = expect(
        system.bus().write32(pc, teq) &&
        system.bus().write32(pc + 4u, tnei),
        "trap test code write failed");

    system.ee().reset(pc);
    auto& state = system.ee().state();
    state.cop0[12] = 0u;
    state.gpr[1].lo = 5u;
    state.gpr[2].lo = 5u;

    std::string error;
    ok = expect(system.ee().step(error),
                "TEQ execution failed") && ok;
    ok = expect(!system.ee().halted() &&
                    state.pc == 0x80000180u &&
                    (state.cop0[13] & 0x7Cu) == 0x34u,
                "TEQ did not raise Trap exception") && ok;

    system.ee().reset(pc + 4u);
    state.cop0[12] = 0u;
    state.gpr[1].lo = 5u;
    error.clear();
    ok = expect(system.ee().step(error),
                "TNEI execution failed") && ok;
    ok = expect(!system.ee().halted() &&
                    state.pc == 0x80000180u &&
                    (state.cop0[13] & 0x7Cu) == 0x34u,
                "TNEI did not raise Trap exception") && ok;

    return ok;
}

bool test_ee_sa_and_qfsrv() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x3600u;

    // MTSA r1; MFSA r2.
    constexpr ps2::u32 mtsa =
        (1u << 21) | 0x29u;
    constexpr ps2::u32 mfsa =
        (2u << 11) | 0x28u;
    // QFSRV r5,r3,r4 is MMI1 subfunction 0x1B.
    constexpr ps2::u32 qfsrv =
        (0x1Cu << 26) |
        (3u << 21) |
        (4u << 16) |
        (5u << 11) |
        (0x1Bu << 6) |
        0x28u;
    // MTSAB r1,3 and MTSAH r1,2.
    constexpr ps2::u32 mtsab =
        (0x01u << 26) | (1u << 21) | (0x18u << 16) | 3u;
    constexpr ps2::u32 mtsah =
        (0x01u << 26) | (1u << 21) | (0x19u << 16) | 2u;

    bool ok = expect(
        system.bus().write32(pc + 0u, mtsa) &&
        system.bus().write32(pc + 4u, mfsa) &&
        system.bus().write32(pc + 8u, qfsrv) &&
        system.bus().write32(pc + 12u, mtsab) &&
        system.bus().write32(pc + 16u, mtsah),
        "SA/QFSRV test code write failed");

    system.ee().reset(pc);
    auto& state = system.ee().state();
    state.gpr[1].lo = 8u; // QFSRV uses SA*8 => 64 bits.
    state.gpr[3] = {
        0xFFEEDDCCBBAA9988ull,
        0x7766554433221100ull,
    };
    state.gpr[4] = {
        0x0123456789ABCDEFull,
        0xFEDCBA9876543210ull,
    };

    std::string error;
    ok = expect(system.ee().step(error), "MTSA execution failed") && ok;
    ok = expect(state.sa == 8u, "MTSA did not update SA") && ok;
    ok = expect(system.ee().step(error), "MFSA execution failed") && ok;
    ok = expect(state.gpr[2].lo == 8u, "MFSA result mismatch") && ok;

    ok = expect(system.ee().step(error), "QFSRV execution failed") && ok;
    ok = expect(
        state.gpr[5].lo == 0xFEDCBA9876543210ull &&
        state.gpr[5].hi == 0xFFEEDDCCBBAA9988ull,
        "QFSRV 64-bit boundary result mismatch") && ok;

    state.gpr[1].lo = 0xAu;
    ok = expect(system.ee().step(error), "MTSAB execution failed") && ok;
    ok = expect(state.sa == 9u, "MTSAB SA result mismatch") && ok;

    ok = expect(system.ee().step(error), "MTSAH execution failed") && ok;
    ok = expect(state.sa == 0u, "MTSAH SA result mismatch") && ok;
    return ok;
}

bool test_ee_tlb_mapped_memory_and_refill() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x3400u;
    constexpr ps2::u32 virtual_base = 0xC0004000u;
    constexpr ps2::u32 physical_base = 0x00002000u;

    // LW r3,0(r2); SW r4,4(r2)
    constexpr ps2::u32 lw =
        (0x23u << 26) | (2u << 21) | (3u << 16);
    constexpr ps2::u32 sw =
        (0x2Bu << 26) | (2u << 21) | (4u << 16) | 4u;

    bool ok = expect(
        system.bus().write32(pc, lw) &&
        system.bus().write32(pc + 4u, sw) &&
        system.bus().write32(physical_base, 0x89ABCDEFu),
        "TLB mapped-memory test setup failed");

    system.ee().reset(pc);
    auto& state = system.ee().state();
    state.gpr[2].lo = virtual_base;
    state.gpr[4].lo = 0x12345678u;

    auto& entry = state.tlb[0];
    entry.page_mask = 0u;
    entry.entry_hi = virtual_base; // ASID 0.
    entry.entry_lo0 =
        ((physical_base >> 12) << 6) | 0x7u; // G|V|D.
    entry.entry_lo1 =
        (((physical_base + 0x1000u) >> 12) << 6) | 0x7u;

    std::string error;
    ok = expect(system.ee().step(error),
                "TLB-mapped LW execution failed") && ok;
    ok = expect(state.gpr[3].lo == 0xFFFFFFFF89ABCDEFull,
                "TLB-mapped LW value mismatch") && ok;

    ok = expect(system.ee().step(error),
                "TLB-mapped SW execution failed") && ok;
    ps2::u32 stored = 0;
    ok = expect(
        system.bus().read32(physical_base + 4u, stored) &&
            stored == 0x12345678u,
        "TLB-mapped SW did not reach physical RAM") && ok;

    // A missing mapped address should enter the refill vector rather than
    // halting the interpreter.
    system.ee().reset(pc);
    state.gpr[2].lo = 0xC1000000u;
    state.cop0[12] = 0u; // BEV=EXL=0.
    error.clear();
    ok = expect(system.ee().step(error),
                "TLB refill exception step failed") && ok;
    ok = expect(!system.ee().halted(),
                "TLB miss incorrectly halted EE") && ok;
    ok = expect(state.pc == 0x80000000u,
                "TLB miss did not use refill vector") && ok;
    ok = expect((state.cop0[13] & 0x7Cu) == 0x08u,
                "TLB miss Cause mismatch") && ok;
    ok = expect(state.cop0[8] == 0xC1000000u,
                "TLB miss BadVAddr mismatch") && ok;
    ok = expect((state.cop0[10] & 0xFFFFE000u) == 0xC1000000u,
                "TLB miss EntryHi VPN2 mismatch") && ok;

    return ok;
}

bool test_ee_integer_overflow_exception() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x3000u;
    constexpr ps2::u32 add =
        (1u << 21) | (2u << 16) | (3u << 11) | 0x20u;
    bool ok = expect(system.bus().write32(pc, add),
                     "ADD overflow test write failed");

    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0;
    system.ee().state().gpr[1].lo = 0x7FFFFFFFu;
    system.ee().state().gpr[2].lo = 1u;
    std::string error;
    ok = expect(system.ee().step(error),
                "ADD overflow exception execution failed") && ok;
    ok = expect(!system.ee().halted(),
                "ADD overflow incorrectly halted EE") && ok;
    ok = expect(system.ee().state().pc == 0x80000180u,
                "ADD overflow vector mismatch") && ok;
    ok = expect(system.ee().state().cop0[14] == pc,
                "ADD overflow EPC mismatch") && ok;
    ok = expect((system.ee().state().cop0[13] & 0x7Cu) == 0x30u,
                "ADD overflow Cause mismatch") && ok;

    constexpr ps2::u32 daddi =
        (0x18u << 26) | (1u << 21) | (2u << 16) | 1u;
    ok = expect(system.bus().write32(pc, daddi),
                "DADDI test write failed") && ok;
    system.ee().reset(pc);
    system.ee().state().gpr[1].lo = 0x0000000100000000ull;
    error.clear();
    ok = expect(system.ee().step(error),
                "DADDI execution failed") && ok;
    ok = expect(system.ee().state().gpr[2].lo ==
                    0x0000000100000001ull,
                "DADDI result mismatch") && ok;

    return ok;
}

bool test_syscall_exception() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000;
    bool ok = expect(system.bus().write32(pc, 0x0000000Cu), "SYSCALL write failed");
    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0; // BEV=ERL=EXL=0
    std::string error;
    ok = expect(system.ee().step(error), "SYSCALL execution failed") && ok;
    ok = expect(system.ee().state().pc == 0x80000180u, "SYSCALL vector mismatch") && ok;
    ok = expect(system.ee().state().cop0[14] == pc, "SYSCALL EPC mismatch") && ok;
    ok = expect((system.ee().state().cop0[13] & 0x7Cu) == 0x20u, "SYSCALL cause mismatch") && ok;
    ok = expect((system.ee().state().cop0[13] & 0x80000000u) == 0, "SYSCALL BD set unexpectedly") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x2u) != 0, "SYSCALL did not set EXL") && ok;

    ok = expect(system.bus().write32(0x80000180u, 0x42000018u), "ERET write failed") && ok;
    ok = expect(system.ee().step(error), "ERET execution failed") && ok;
    ok = expect(system.ee().state().pc == pc, "ERET return PC mismatch") && ok;
    ok = expect((system.ee().state().cop0[12] & 0x2u) == 0, "ERET did not clear EXL") && ok;
    return ok;
}

bool test_syscall_delay_slot_exception() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2400;
    // BEQ r0,r0,+1 followed by SYSCALL in the mandatory delay slot.
    bool ok = expect(system.bus().write32(pc, 0x10000001u), "delay branch write failed");
    ok = expect(system.bus().write32(pc + 4u, 0x0000000Cu), "delay SYSCALL write failed") && ok;
    system.ee().reset(pc);
    system.ee().state().cop0[12] = 0;
    std::string error;
    ok = expect(system.ee().step(error), "delay branch execution failed") && ok;
    ok = expect(system.ee().step(error), "delay-slot SYSCALL failed") && ok;
    ok = expect(system.ee().state().pc == 0x80000180u, "delay SYSCALL vector mismatch") && ok;
    ok = expect(system.ee().state().cop0[14] == pc, "delay SYSCALL EPC mismatch") && ok;
    ok = expect((system.ee().state().cop0[13] & 0x80000000u) != 0, "delay SYSCALL BD missing") && ok;
    return ok;
}


bool test_video_timing_vblank_irqs() {
    ps2::VideoTiming timing;
    ps2::EeHw ee_hw;
    ps2::IopIntc iop_intc;
    timing.reset();
    ee_hw.reset();
    iop_intc.reset();

    bool ok = true;
    ps2::u32 value = 0;
    timing.tick(ps2::VideoTiming::kNtscRenderCycles - 1u, ee_hw, iop_intc);
    ok = expect(ee_hw.read32(0x1000F000u, value) && value == 0,
                "VBlank start fired early") && ok;
    ok = expect(iop_intc.status() == 0, "IOP VBlank start fired early") && ok;

    timing.tick(1, ee_hw, iop_intc);
    ok = expect(ee_hw.read32(0x1000F000u, value) && (value & (1u << 2)) != 0,
                "EE VBlank-start INTC source missing") && ok;
    ok = expect((iop_intc.status() & (1u << 0)) != 0,
                "IOP VBlank-start interrupt missing") && ok;
    ok = expect(timing.phase() == ps2::VideoTiming::Phase::VBlank,
                "video timing did not enter VBlank") && ok;

    timing.tick(ps2::VideoTiming::kNtscVBlankCycles, ee_hw, iop_intc);
    ok = expect(ee_hw.read32(0x1000F000u, value) && (value & (1u << 3)) != 0,
                "EE VBlank-end INTC source missing") && ok;
    ok = expect((iop_intc.status() & (1u << 11)) != 0,
                "IOP VBlank-end interrupt missing") && ok;
    ok = expect(timing.phase() == ps2::VideoTiming::Phase::Render,
                "video timing did not return to render") && ok;
    ok = expect(timing.fields_started() == 1,
                "video timing field counter mismatch") && ok;
    return ok;
}


bool test_gif_packet_decode() {
    ps2::Ps2System system;
    bool ok = true;

    constexpr ps2::u32 fifo = ps2::GsCore::kGifFifoBase;

    // PACKED A+D packet: PRIM then FRAME_1.
    const ps2::u64 packed_tag_lo =
        2ull | (1ull << 15) | (1ull << 60);
    ok = expect(system.bus().write64(fifo, packed_tag_lo) &&
                system.bus().write64(fifo + 8u, 0xEull),
                "GIF packed tag write failed") && ok;
    ok = expect(system.bus().write64(fifo, 6ull) &&
                system.bus().write64(fifo + 8u, 0x00ull),
                "GIF PRIM A+D write failed") && ok;
    ok = expect(system.bus().write64(fifo, 0x1122334455667788ull) &&
                system.bus().write64(fifo + 8u, 0x4Cull),
                "GIF FRAME_1 A+D write failed") && ok;
    ok = expect(system.gs_core().register_value(0x00) == 6ull,
                "GIF PRIM register mismatch") && ok;
    ok = expect(system.gs_core().register_value(0x4C) == 0x1122334455667788ull,
                "GIF FRAME_1 register mismatch") && ok;

    // REGLIST packet with PRIM + XYZ2 in a single qword.
    const ps2::u64 reglist_tag_lo =
        1ull | (1ull << 15) | (1ull << 58) | (2ull << 60);
    ok = expect(system.bus().write64(fifo, reglist_tag_lo) &&
                system.bus().write64(fifo + 8u, 0x50ull),
                "GIF reglist tag write failed") && ok;
    const ps2::u64 xyz = 0x001234560078009Aull;
    ok = expect(system.bus().write64(fifo, 3ull) &&
                system.bus().write64(fifo + 8u, xyz),
                "GIF reglist payload write failed") && ok;
    ok = expect(system.gs_core().register_value(0x00) == 3ull,
                "GIF reglist PRIM mismatch") && ok;
    ok = expect(system.gs_core().register_value(0x05) == xyz,
                "GIF reglist XYZ2 mismatch") && ok;

    // IMAGE packet accounting.
    const ps2::u64 image_tag_lo =
        1ull | (1ull << 15) | (2ull << 58);
    ok = expect(system.bus().write64(fifo, image_tag_lo) &&
                system.bus().write64(fifo + 8u, 0),
                "GIF image tag write failed") && ok;
    ok = expect(system.bus().write64(fifo, 0x0123456789ABCDEFull) &&
                system.bus().write64(fifo + 8u, 0xFEDCBA9876543210ull),
                "GIF image payload write failed") && ok;

    const auto& stats = system.gs_core().stats();
    ok = expect(stats.gif_tags == 3, "GIF tag count mismatch") && ok;
    ok = expect(stats.packed_writes == 2, "GIF packed write count mismatch") && ok;
    ok = expect(stats.reglist_writes == 2, "GIF reglist write count mismatch") && ok;
    ok = expect(stats.image_qwords == 1, "GIF image qword count mismatch") && ok;
    ok = expect(stats.vertices == 1, "GIF vertex kick count mismatch") && ok;
    ok = expect(stats.eop_packets == 3, "GIF EOP count mismatch") && ok;
    return ok;
}


bool test_gif_dma_engine() {
    ps2::Ps2System system;
    ps2::GifDma dma;
    dma.reset();
    std::string error;
    bool ok = true;

    constexpr ps2::u32 dmac_ctrl = 0x1000E000u;
    constexpr ps2::u32 dmac_stat = 0x1000E010u;
    constexpr ps2::u32 gif_chcr = 0x1000A000u;
    constexpr ps2::u32 gif_madr = 0x1000A010u;
    constexpr ps2::u32 gif_qwc = 0x1000A020u;
    constexpr ps2::u32 gif_tadr = 0x1000A030u;

    const ps2::u64 gif_tag =
        1ull | (1ull << 15) | (1ull << 60);
    constexpr ps2::u64 ad_descriptor = 0xEull;

    // Normal mode: two qwords from RAM become one GIF tag and one A+D write.
    ok = expect(system.bus().write64(0x4000u, gif_tag) &&
                system.bus().write64(0x4008u, ad_descriptor) &&
                system.bus().write64(0x4010u, 6ull) &&
                system.bus().write64(0x4018u, 0x00ull),
                "GIF DMA normal payload setup failed") && ok;
    ok = expect(system.bus().write32(dmac_ctrl, 1u) &&
                system.bus().write32(dmac_stat, 1u << 18) &&
                system.bus().write32(gif_madr, 0x4000u) &&
                system.bus().write32(gif_qwc, 2u) &&
                system.bus().write32(gif_chcr, 0x101u),
                "GIF DMA normal register setup failed") && ok;
    ok = expect(dma.service(system.bus(), system.gs_core(), error),
                "GIF DMA normal first service failed") && ok;
    ok = expect(dma.service(system.bus(), system.gs_core(), error),
                "GIF DMA normal second service failed") && ok;

    ps2::u32 value = 0;
    ok = expect(system.gs_core().register_value(0x00) == 6ull,
                "GIF DMA normal did not reach GS") && ok;
    ok = expect(system.bus().read32(gif_qwc, value) && value == 0,
                "GIF DMA normal QWC did not reach zero") && ok;
    ok = expect(system.bus().read32(gif_chcr, value) && (value & 0x100u) == 0,
                "GIF DMA normal STR did not clear") && ok;
    ok = expect(system.bus().read32(dmac_stat, value) && (value & (1u << 2)) != 0,
                "GIF DMA normal completion cause missing") && ok;
    ok = expect(system.bus().dmac_pending(),
                "GIF DMA normal completion did not assert DMAC pending") && ok;

    // Clear the completion cause, then run an END source-chain tag.
    ok = expect(system.bus().write32(dmac_stat, 1u << 2),
                "GIF DMA status acknowledge failed") && ok;
    const ps2::u32 dma_tag0 = 2u | (7u << 28);
    const ps2::u64 dma_tag_lo = static_cast<ps2::u64>(dma_tag0);
    ok = expect(system.bus().write64(0x5000u, dma_tag_lo) &&
                system.bus().write64(0x5008u, 0) &&
                system.bus().write64(0x5010u, gif_tag) &&
                system.bus().write64(0x5018u, ad_descriptor) &&
                system.bus().write64(0x5020u, 0xA5A5ull) &&
                system.bus().write64(0x5028u, 0x4Cull),
                "GIF DMA chain payload setup failed") && ok;
    ok = expect(system.bus().write32(gif_tadr, 0x5000u) &&
                system.bus().write32(gif_qwc, 0u) &&
                system.bus().write32(gif_chcr, 0x105u),
                "GIF DMA chain register setup failed") && ok;
    ok = expect(dma.service(system.bus(), system.gs_core(), error),
                "GIF DMA chain first service failed") && ok;
    ok = expect(dma.service(system.bus(), system.gs_core(), error),
                "GIF DMA chain second service failed") && ok;
    ok = expect(system.gs_core().register_value(0x4C) == 0xA5A5ull,
                "GIF DMA END chain did not reach GS") && ok;
    ok = expect(system.bus().read32(gif_chcr, value) && (value & 0x100u) == 0,
                "GIF DMA END chain STR did not clear") && ok;
    ok = expect(system.bus().dmac_pending(),
                "GIF DMA END chain did not assert DMAC pending") && ok;
    return ok;
}


bool test_gs_vram_swizzle_addresses() {
    ps2::GsVram vram;
    bool ok = true;

    ok = expect(vram.write_pixel(0, 8, 0, 0, 1, 0x44332211u),
                "PSMCT32 swizzle write failed") && ok;
    ok = expect(vram.byte_at(256) == 0x11u &&
                vram.byte_at(257) == 0x22u &&
                vram.byte_at(258) == 0x33u &&
                vram.byte_at(259) == 0x44u,
                "PSMCT32 x=8 block address mismatch") && ok;

    vram.reset();
    ok = expect(vram.write_pixel(2, 16, 0, 0, 1, 0xBEEFu),
                "PSMCT16 swizzle write failed") && ok;
    ok = expect(vram.byte_at(512) == 0xEFu &&
                vram.byte_at(513) == 0xBEu,
                "PSMCT16 x=16 block address mismatch") && ok;

    vram.reset();
    ok = expect(vram.write_pixel(10, 32, 0, 0, 1, 0x1234u),
                "PSMCT16S swizzle write failed") && ok;
    ok = expect(vram.byte_at(4096) == 0x34u &&
                vram.byte_at(4097) == 0x12u,
                "PSMCT16S x=32 block address mismatch") && ok;

    return ok;
}

bool test_gs_host_to_local_image_transfer() {
    ps2::GsCore gs;
    gs.reset();

    auto ad_packet = [&](ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag =
            1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };

    bool ok = true;

    // 32-bit upload: 4 pixels, one IMAGE qword.
    const ps2::u64 blit32 =
        (static_cast<ps2::u64>(1u) << 48); // DBP=0, DBW=1, DPSM=PSMCT32
    const ps2::u64 pos32 =
        (static_cast<ps2::u64>(8u) << 32); // DSAX=8, DSAY=0
    const ps2::u64 reg32 =
        4ull | (1ull << 32); // 4x1
    ad_packet(0x50, blit32);
    ad_packet(0x51, pos32);
    ad_packet(0x52, reg32);
    ad_packet(0x53, 0);

    const ps2::u64 image_tag32 =
        1ull | (1ull << 15) | (2ull << 58);
    gs.write_gif_qword(image_tag32, 0);
    gs.write_gif_qword(
        0x2222222211111111ull,
        0x4444444433333333ull);

    ok = expect(!gs.transfer_active(),
                "PSMCT32 transfer did not complete") && ok;
    ok = expect(gs.vram().read_pixel(0, 8, 0, 0, 1) == 0x11111111u &&
                gs.vram().read_pixel(0, 9, 0, 0, 1) == 0x22222222u &&
                gs.vram().read_pixel(0, 10, 0, 0, 1) == 0x33333333u &&
                gs.vram().read_pixel(0, 11, 0, 0, 1) == 0x44444444u,
                "PSMCT32 IMAGE upload pixel mismatch") && ok;

    // 24-bit upload: six tightly-packed pixels span two GIF qwords.
    gs.reset();
    const ps2::u64 blit24 =
        (static_cast<ps2::u64>(1u) << 48) |
        (static_cast<ps2::u64>(1u) << 56);
    const ps2::u64 reg24 = 6ull | (1ull << 32);
    ad_packet(0x50, blit24);
    ad_packet(0x51, 0);
    ad_packet(0x52, reg24);
    ad_packet(0x53, 0);

    const ps2::u64 image_tag24 =
        2ull | (1ull << 15) | (2ull << 58);
    gs.write_gif_qword(image_tag24, 0);

    // Pixel byte stream:
    // 030201 060504 090807 0C0B0A 0F0E0D 121110, then qword padding.
    gs.write_gif_qword(
        0x0807060504030201ull,
        0x100F0E0D0C0B0A09ull);
    gs.write_gif_qword(
        0x0000000000001211ull,
        0);

    ok = expect(!gs.transfer_active(),
                "PSMCT24 transfer did not complete") && ok;
    ok = expect(gs.vram().read_pixel(1, 0, 0, 0, 1) == 0x030201u &&
                gs.vram().read_pixel(1, 1, 0, 0, 1) == 0x060504u &&
                gs.vram().read_pixel(1, 2, 0, 0, 1) == 0x090807u &&
                gs.vram().read_pixel(1, 3, 0, 0, 1) == 0x0C0B0Au &&
                gs.vram().read_pixel(1, 4, 0, 0, 1) == 0x0F0E0Du &&
                gs.vram().read_pixel(1, 5, 0, 0, 1) == 0x121110u,
                "PSMCT24 qword-carry upload mismatch") && ok;

    const auto& stats = gs.stats();
    ok = expect(stats.host_to_local_transfers == 1 &&
                stats.host_to_local_pixels == 6 &&
                stats.image_qwords == 2,
                "host-to-local transfer statistics mismatch") && ok;
    ok = expect((gs.register_value(0x53) & 0x3u) == 3u,
                "completed transfer did not deactivate TRXDIR") && ok;

    return ok;
}


bool test_gs_untextured_rasterization() {
    auto ad = [](ps2::GsCore& gs, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag =
            1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };

    auto xyz = [](ps2::u32 x_fp, ps2::u32 y_fp, ps2::u32 z = 0) {
        return static_cast<ps2::u64>(x_fp & 0xFFFFu) |
               (static_cast<ps2::u64>(y_fp & 0xFFFFu) << 16) |
               (static_cast<ps2::u64>(z) << 32);
    };

    const ps2::u64 frame =
        static_cast<ps2::u64>(1u) << 16; // FBP=0, FBW=1, PSMCT32, FBMSK=0.
    const ps2::u64 scissor =
        (static_cast<ps2::u64>(31u) << 16) |
        (static_cast<ps2::u64>(31u) << 48);

    bool ok = true;

    // Point coordinates use the GS nearest-pixel convention after XYOFFSET.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 0u); // point
        ad(gs, 0x01, 0xA0403020u);
        ad(gs, 0x05, xyz(16, 16));

        ok = expect(gs.vram().read_pixel(0, 1, 1, 0, 1) == 0xA0403020u,
                    "GS point raster pixel mismatch") && ok;
        ok = expect(gs.stats().raster_draws == 1 &&
                    gs.stats().raster_pixels == 1 &&
                    gs.stats().skipped_raster_draws == 0,
                    "GS point raster statistics mismatch") && ok;
    }

    // AA1 is accepted during bootstrap and rendered without edge coverage.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 0u | (1u << 7)); // point + AA1
        ad(gs, 0x01, 0xB0605040u);
        ad(gs, 0x05, xyz(16, 16));

        ok = expect(
                 gs.vram().read_pixel(0, 1, 1, 0, 1) == 0xB0605040u,
                 "GS AA1 bootstrap point was dropped") && ok;
        ok = expect(
                 gs.stats().raster_draws == 1 &&
                 gs.stats().skipped_raster_draws == 0,
                 "GS AA1 bootstrap draw statistics mismatch") && ok;
    }

    // A center-to-center line excludes the terminal pixel according to the
    // GS diamond-exit rule, avoiding duplicate endpoints in connected strips.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 1u); // line list
        ad(gs, 0x01, 0xC0556677u);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(48, 0));

        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0xC0556677u &&
                    gs.vram().read_pixel(0, 1, 0, 0, 1) == 0xC0556677u &&
                    gs.vram().read_pixel(0, 2, 0, 0, 1) == 0xC0556677u,
                    "GS line raster coverage mismatch") && ok;
        ok = expect(gs.vram().read_pixel(0, 3, 0, 0, 1) == 0,
                    "GS line raster included terminal endpoint") && ok;
        ok = expect(gs.stats().raster_draws == 1 &&
                    gs.stats().raster_pixels == 3,
                    "GS line raster statistics mismatch") && ok;
    }

    // In a line strip the shared vertex is emitted by the following segment,
    // not by both segments.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 2u); // line strip
        ad(gs, 0x01, 0xE0112233u);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(32, 0));
        ad(gs, 0x05, xyz(32, 32));

        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0xE0112233u &&
                    gs.vram().read_pixel(0, 1, 0, 0, 1) == 0xE0112233u &&
                    gs.vram().read_pixel(0, 2, 0, 0, 1) == 0xE0112233u &&
                    gs.vram().read_pixel(0, 2, 1, 0, 1) == 0xE0112233u,
                    "GS line-strip shared endpoint mismatch") && ok;
        ok = expect(gs.vram().read_pixel(0, 2, 2, 0, 1) == 0,
                    "GS line strip included final endpoint") && ok;
        ok = expect(gs.stats().raster_draws == 2 &&
                    gs.stats().raster_pixels == 4,
                    "GS line-strip raster statistics mismatch") && ok;
    }

    // Flat untextured 2x2 sprite.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u); // PRMODECONT.AC = use PRIM attributes.
        ad(gs, 0x18, 0u); // XYOFFSET_1
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u); // TEST_1 disabled
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 6u); // sprite
        ad(gs, 0x01, 0x44332211u);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(32, 32));

        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0x44332211u &&
                    gs.vram().read_pixel(0, 1, 0, 0, 1) == 0x44332211u &&
                    gs.vram().read_pixel(0, 0, 1, 0, 1) == 0x44332211u &&
                    gs.vram().read_pixel(0, 1, 1, 0, 1) == 0x44332211u,
                    "GS sprite raster pixels mismatch") && ok;
        ok = expect(gs.vram().read_pixel(0, 2, 2, 0, 1) == 0,
                    "GS sprite raster overran rectangle") && ok;
        ok = expect(gs.stats().raster_draws == 1 &&
                    gs.stats().raster_pixels == 4 &&
                    gs.stats().skipped_raster_draws == 0,
                    "GS sprite raster statistics mismatch") && ok;
    }

    // Flat untextured triangle list.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 3u); // triangle list
        ad(gs, 0x01, 0x88776655u);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(48, 0));
        ad(gs, 0x05, xyz(0, 48));

        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0x88776655u,
                    "GS triangle failed to cover interior pixel") && ok;
        ok = expect(gs.vram().read_pixel(0, 2, 2, 0, 1) == 0,
                    "GS triangle covered exterior pixel") && ok;
        ok = expect(gs.stats().raster_draws == 1 &&
                    gs.stats().raster_pixels != 0,
                    "GS triangle raster statistics mismatch") && ok;
    }

    // Textured draw must be observable as skipped, not silently approximated.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 6u | (1u << 4)); // sprite + TME
        ad(gs, 0x01, 0xFFFFFFFFu);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(32, 32));

        ok = expect(gs.stats().raster_draws == 0 &&
                    gs.stats().skipped_raster_draws == 1,
                    "unsupported textured draw was not skipped") && ok;
        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0,
                    "unsupported textured draw modified VRAM") && ok;
    }

    return ok;
}

bool test_gs_async_raster_ordering() {
    auto ad = [](ps2::GsCore& gs, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag = 1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };
    auto xy = [](ps2::u32 x, ps2::u32 y) {
        return static_cast<ps2::u64>(x * 16u) |
               (static_cast<ps2::u64>(y * 16u) << 16);
    };
    auto run = [&](bool threaded) {
        ps2::GsCore gs;
        gs.set_async_rasterization(threaded);
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, (63ull << 16) | (63ull << 48));
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, 1ull << 16);
        ad(gs, 0x00, 6u);
        ad(gs, 0x01, 0x11223344u);
        ad(gs, 0x05, xy(0, 0));
        ad(gs, 0x05, xy(64, 64));
        ad(gs, 0x01, 0x55667788u);
        ad(gs, 0x05, xy(8, 0));
        ad(gs, 0x05, xy(16, 8));

        // A host transfer must wait for both earlier draws before writing.
        ad(gs, 0x50, 1ull << 48);
        ad(gs, 0x51, 8ull << 32);
        ad(gs, 0x52, 4ull | (1ull << 32));
        ad(gs, 0x53, 0u);
        gs.write_gif_qword(1ull | (1ull << 15) | (2ull << 58), 0);
        gs.write_gif_qword(0xAABBCCDD12345678ull,
                           0x31415926DEADBEEFull);

        const auto& vram = gs.vram();
        const auto& stats = gs.stats();
        return std::array<ps2::u64, 7>{
            vram.read_pixel(0, 0, 0, 0, 1),
            vram.read_pixel(0, 8, 0, 0, 1),
            vram.read_pixel(0, 9, 0, 0, 1),
            vram.read_pixel(0, 12, 0, 0, 1),
            vram.read_pixel(0, 20, 20, 0, 1),
            stats.raster_draws,
            stats.raster_pixels,
        };
    };

    const auto expected = run(false);
    bool ok = expect(expected[0] == 0x11223344u &&
                     expected[1] == 0x12345678u &&
                     expected[2] == 0xAABBCCDDu &&
                     expected[3] == 0x55667788u &&
                     expected[4] == 0x11223344u &&
                     expected[5] == 2 && expected[6] == 4160,
                     "GS synchronous draw/transfer fixture mismatch");
    for (int i = 0; i < 4; ++i) {
        ok = expect(run(true) == expected,
                    "GS threaded raster draw/transfer ordering mismatch") && ok;
    }
    return ok;
}


bool test_gs_display_extraction() {
    ps2::Ps2System system;
    bool ok = true;

    // Display circuit 1: 4x2 pixels, framebuffer starts at DBX=2, DBY=3.
    constexpr ps2::u64 pmode = 1u;
    constexpr ps2::u64 dispfb =
        (static_cast<ps2::u64>(1u) << 9) |
        (static_cast<ps2::u64>(2u) << 32) |
        (static_cast<ps2::u64>(3u) << 43);
    constexpr ps2::u64 display =
        (static_cast<ps2::u64>(3u) << 32) |
        (static_cast<ps2::u64>(1u) << 44);

    ok = expect(system.gs_privileged().write64(0x12000000u, pmode) &&
                system.gs_privileged().write64(0x12000070u, dispfb) &&
                system.gs_privileged().write64(0x12000080u, display),
                "GS display register setup failed") && ok;

    system.gs_display().update(
        system.gs_privileged(), system.gs_core().vram());
    ok = expect(
        system.gs_display().valid() &&
        !system.gs_display().has_visible_pixels() &&
        system.gs_display().nonzero_pixel_count() == 0,
        "valid black scanout was treated as visible output") && ok;

    const ps2::u32 colors[8] = {
        0xFF000011u, 0xFF002200u, 0xFF330000u, 0xFF443322u,
        0xFF556677u, 0xFF778899u, 0xFFABCDEFu, 0xFF102030u,
    };
    for (ps2::u32 y = 0; y < 2; ++y) {
        for (ps2::u32 x = 0; x < 4; ++x) {
            ok = expect(
                system.gs_core().vram().write_pixel(
                    0, 2u + x, 3u + y, 0, 1, colors[y * 4u + x]),
                "GS display VRAM setup failed") && ok;
        }
    }

    system.gs_display().update(
        system.gs_privileged(), system.gs_core().vram());

    const auto& out = system.gs_display();
    ok = expect(out.valid(), "GS display surface not valid") && ok;
    ok = expect(out.width() == 4 && out.height() == 2,
                "GS display dimensions mismatch") && ok;
    ok = expect(out.circuit() == 1 && out.psm() == 0,
                "GS display metadata mismatch") && ok;
    ok = expect(out.rgba8().size() == 8,
                "GS display pixel count mismatch") && ok;
    ok = expect(out.has_visible_pixels() &&
                out.nonzero_pixel_count() == 8,
                "GS visible-pixel milestone mismatch") && ok;
    for (std::size_t i = 0; i < 8 && i < out.rgba8().size(); ++i) {
        ok = expect(out.rgba8()[i] == colors[i],
                    "GS display extracted pixel mismatch") && ok;
    }

    // Both circuits can be enabled while only circuit 2 has valid scanout
    // state. The presenter must not blank merely because circuit 1 is first.
    system.gs_privileged().reset();
    system.gs_display().reset();
    constexpr ps2::u64 pmode_both = 3u;
    constexpr ps2::u64 dispfb2 =
        static_cast<ps2::u64>(1u) << 9;
    constexpr ps2::u64 display2 =
        (static_cast<ps2::u64>(1u) << 32) |
        (static_cast<ps2::u64>(0u) << 44);
    ok = expect(
        system.gs_privileged().write64(0x12000000u, pmode_both) &&
        system.gs_privileged().write64(0x12000090u, dispfb2) &&
        system.gs_privileged().write64(0x120000A0u, display2),
        "GS circuit-2 fallback register setup failed") && ok;
    ok = expect(
        system.gs_core().vram().write_pixel(
            0, 0u, 0u, 0u, 1u, 0x00AABBCCu),
        "GS circuit-2 fallback VRAM setup failed") && ok;

    system.gs_display().update(
        system.gs_privileged(), system.gs_core().vram());

    ok = expect(
        system.gs_display().valid() &&
        system.gs_display().circuit() == 2u &&
        system.gs_display().width() == 2u &&
        system.gs_display().height() == 1u,
        "GS display did not fall back to circuit 2") && ok;
    ok = expect(
        !system.gs_display().rgba8().empty() &&
        system.gs_display().rgba8()[0] == 0xFFAABBCCu,
        "GS circuit-2 fallback pixel mismatch") && ok;

    // With both circuits valid, circuit 1 is blended over circuit 2. A fully
    // transparent circuit-1 pixel must not hide visible circuit-2 BIOS output.
    system.gs_privileged().reset();
    system.gs_display().reset();
    constexpr ps2::u64 blend_dispfb1 =
        (static_cast<ps2::u64>(1u) << 9) |
        (static_cast<ps2::u64>(1u) << 43);
    constexpr ps2::u64 blend_dispfb2 =
        static_cast<ps2::u64>(1u) << 9;
    constexpr ps2::u64 blend_display =
        static_cast<ps2::u64>(0u) << 32;
    ok = expect(
        system.gs_privileged().write64(0x12000000u, 3u) &&
        system.gs_privileged().write64(0x12000070u, blend_dispfb1) &&
        system.gs_privileged().write64(0x12000080u, blend_display) &&
        system.gs_privileged().write64(0x12000090u, blend_dispfb2) &&
        system.gs_privileged().write64(0x120000A0u, blend_display),
        "GS dual-circuit register setup failed") && ok;
    ok = expect(
        system.gs_core().vram().write_pixel(
            0u, 0u, 1u, 0u, 1u, 0x000000FFu) &&
        system.gs_core().vram().write_pixel(
            0u, 0u, 0u, 0u, 1u, 0x00FF0000u),
        "GS dual-circuit VRAM setup failed") && ok;

    system.gs_display().update(
        system.gs_privileged(), system.gs_core().vram());
    ok = expect(
        system.gs_display().valid() &&
        system.gs_display().circuit() == 3u &&
        !system.gs_display().rgba8().empty() &&
        system.gs_display().rgba8()[0] == 0xFFFF0000u,
        "transparent circuit 1 incorrectly hid circuit 2") && ok;

    // MMOD=1 selects PMODE.ALP. ALP=0x40 is one-half in GS 1.7 alpha.
    constexpr ps2::u64 half_alpha_pmode =
        3u | (1u << 5) | (static_cast<ps2::u64>(0x40u) << 8);
    ok = expect(
        system.gs_privileged().write64(
            0x12000000u,
            half_alpha_pmode),
        "GS constant-alpha PMODE setup failed") && ok;
    ok = expect(
        system.gs_core().vram().write_pixel(
            0u, 0u, 1u, 0u, 1u, 0xFF0000FFu),
        "GS constant-alpha source setup failed") && ok;

    system.gs_display().update(
        system.gs_privileged(), system.gs_core().vram());
    ok = expect(
        !system.gs_display().rgba8().empty() &&
        system.gs_display().rgba8()[0] == 0xFF800080u,
        "GS constant-alpha dual-circuit merge mismatch") && ok;

    // Interlaced field mode (SMODE2.INT + FFMD) scans half as many
    // framebuffer lines as the full DISPLAY height. The presenter should
    // bob those field lines to the full host surface rather than reading
    // into the following field/image and showing two vertically stacked
    // pictures.
    system.gs_privileged().reset();
    system.gs_display().reset();
    constexpr ps2::u64 interlaced_pmode = 1u;
    constexpr ps2::u64 interlaced_smode2 = 3u; // INT=1, FFMD=1
    constexpr ps2::u64 interlaced_dispfb =
        static_cast<ps2::u64>(1u) << 9; // FBW=1
    constexpr ps2::u64 interlaced_display =
        (static_cast<ps2::u64>(1u) << 32) | // width 2
        (static_cast<ps2::u64>(3u) << 44);  // display height 4
    ok = expect(
        system.gs_privileged().write64(0x12000000u, interlaced_pmode) &&
        system.gs_privileged().write64(0x12000020u, interlaced_smode2) &&
        system.gs_privileged().write64(0x12000070u, interlaced_dispfb) &&
        system.gs_privileged().write64(0x12000080u, interlaced_display),
        "GS interlaced field-mode register setup failed") && ok;

    constexpr ps2::u32 field_row0 = 0xFF112233u;
    constexpr ps2::u32 field_row1 = 0xFF445566u;
    constexpr ps2::u32 bogus_row2 = 0xFFAA0000u;
    constexpr ps2::u32 bogus_row3 = 0xFF00AA00u;
    for (ps2::u32 x = 0; x < 2u; ++x) {
        ok = expect(
            system.gs_core().vram().write_pixel(
                0u, x, 0u, 0u, 1u, field_row0) &&
            system.gs_core().vram().write_pixel(
                0u, x, 1u, 0u, 1u, field_row1) &&
            system.gs_core().vram().write_pixel(
                0u, x, 2u, 0u, 1u, bogus_row2) &&
            system.gs_core().vram().write_pixel(
                0u, x, 3u, 0u, 1u, bogus_row3),
            "GS interlaced field-mode VRAM setup failed") && ok;
    }

    system.gs_display().update(
        system.gs_privileged(), system.gs_core().vram());
    const auto& interlaced_out = system.gs_display();
    ok = expect(
        interlaced_out.valid() &&
        interlaced_out.width() == 2u &&
        interlaced_out.height() == 4u &&
        interlaced_out.rgba8().size() == 8u,
        "GS interlaced field-mode dimensions mismatch") && ok;
    if (interlaced_out.rgba8().size() == 8u) {
        for (ps2::u32 x = 0; x < 2u; ++x) {
            ok = expect(
                interlaced_out.rgba8()[x] == field_row0 &&
                interlaced_out.rgba8()[2u + x] == field_row0 &&
                interlaced_out.rgba8()[4u + x] == field_row1 &&
                interlaced_out.rgba8()[6u + x] == field_row1,
                "GS interlaced field-mode bob scanout mismatch") && ok;
        }
    }

    return ok;
}


bool test_gs_fst_direct_color_texturing() {
    auto ad = [](ps2::GsCore& gs, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag =
            1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };
    auto xyz = [](ps2::u32 x_fp, ps2::u32 y_fp) {
        return static_cast<ps2::u64>(x_fp & 0xFFFFu) |
               (static_cast<ps2::u64>(y_fp & 0xFFFFu) << 16);
    };
    auto uv = [](ps2::u32 u_fp, ps2::u32 v_fp) {
        return static_cast<ps2::u64>(u_fp & 0x3FFFu) |
               (static_cast<ps2::u64>(v_fp & 0x3FFFu) << 16);
    };
    auto tex0 = [](ps2::u32 bp, ps2::u32 bw, ps2::u32 psm,
                   ps2::u32 tw, ps2::u32 th, bool tcc, ps2::u32 tfx) {
        return static_cast<ps2::u64>(bp & 0x3FFFu) |
               (static_cast<ps2::u64>(bw & 0x3Fu) << 14) |
               (static_cast<ps2::u64>(psm & 0x3Fu) << 20) |
               (static_cast<ps2::u64>(tw & 0xFu) << 26) |
               (static_cast<ps2::u64>(th & 0xFu) << 30) |
               (static_cast<ps2::u64>(tcc ? 1u : 0u) << 34) |
               (static_cast<ps2::u64>(tfx & 0x3u) << 35);
    };

    constexpr ps2::u32 texture_bp = 32;
    const ps2::u64 frame =
        static_cast<ps2::u64>(1u) << 16; // FBP=0, FBW=1, PSMCT32.
    const ps2::u64 scissor =
        (static_cast<ps2::u64>(31u) << 16) |
        (static_cast<ps2::u64>(31u) << 48);
    bool ok = true;

    // Exact 2x2 PSMCT32 DECAL sprite.
    {
        ps2::GsCore gs;
        gs.reset();
        const ps2::u32 colors[4] = {
            0x11223344u, 0x55667788u,
            0x99AABBCCu, 0xDDEEFF10u,
        };
        for (ps2::u32 y = 0; y < 2; ++y) {
            for (ps2::u32 x = 0; x < 2; ++x) {
                ok = expect(
                    gs.vram().write_pixel(
                        0, x, y, texture_bp, 1, colors[y * 2u + x]),
                    "textured sprite source setup failed") && ok;
            }
        }

        ad(gs, 0x1A, 1u); // PRMODECONT
        ad(gs, 0x18, 0u); // XYOFFSET_1
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u); // TEST_1 disabled
        ad(gs, 0x4C, frame);
        ad(gs, 0x06, tex0(texture_bp, 1, 0, 1, 1, true, 1)); // DECAL
        ad(gs, 0x08, 0u); // repeat U/V
        ad(gs, 0x00, 6u | (1u << 4) | (1u << 8)); // sprite, TME, FST
        ad(gs, 0x01, 0x80808080u);
        ad(gs, 0x03, uv(0, 0));
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x03, uv(32, 32));
        ad(gs, 0x05, xyz(32, 32));

        ok = expect(
            gs.vram().read_pixel(0, 0, 0, 0, 1) == colors[0] &&
            gs.vram().read_pixel(0, 1, 0, 0, 1) == colors[1] &&
            gs.vram().read_pixel(0, 0, 1, 0, 1) == colors[2] &&
            gs.vram().read_pixel(0, 1, 1, 0, 1) == colors[3],
            "PSMCT32 DECAL sprite texels mismatch") && ok;
        ok = expect(
            gs.stats().textured_raster_draws == 1 &&
            gs.stats().texture_samples == 4,
            "textured sprite statistics mismatch") && ok;
    }

    // Repeat wrapping with MODULATE identity color (vertex channel 128).
    {
        ps2::GsCore gs;
        gs.reset();
        ok = expect(
            gs.vram().write_pixel(0, 0, 0, texture_bp, 1, 0xFF204080u) &&
            gs.vram().write_pixel(0, 1, 0, texture_bp, 1, 0xFF80A0C0u),
            "repeat texture source setup failed") && ok;

        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x06, tex0(texture_bp, 1, 0, 1, 0, false, 0)); // MODULATE
        ad(gs, 0x08, 0u); // REPEAT
        ad(gs, 0x00, 6u | (1u << 4) | (1u << 8));
        ad(gs, 0x01, 0x80808080u);
        ad(gs, 0x03, uv(32, 0)); // Starts at texel 2 -> repeats to 0.
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x03, uv(64, 16));
        ad(gs, 0x05, xyz(32, 16));

        ok = expect(
            gs.vram().read_pixel(0, 0, 0, 0, 1) == 0x80204080u &&
            gs.vram().read_pixel(0, 1, 0, 0, 1) == 0x8080A0C0u,
            "FST repeat/MODULATE texture mismatch") && ok;
    }

    // HIGHLIGHT and HIGHLIGHT2 share RGB math but differ in their
    // TCC-enabled alpha result.
    for (ps2::u32 tfx : {2u, 3u}) {
        ps2::GsCore gs;
        gs.reset();
        ok = expect(
            gs.vram().write_pixel(
                0, 0, 0, texture_bp, 1, 0x40204080u),
            "highlight texture source setup failed") && ok;

        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x06, tex0(texture_bp, 1, 0, 0, 0, true, tfx));
        ad(gs, 0x08, 0u);
        ad(gs, 0x00, 6u | (1u << 4) | (1u << 8));
        ad(gs, 0x01, 0x20104080u);
        ad(gs, 0x03, uv(0, 0));
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x03, uv(16, 16));
        ad(gs, 0x05, xyz(16, 16));

        const ps2::u32 expected =
            tfx == 2u ? 0x602440A0u : 0x402440A0u;
        ok = expect(
            gs.vram().read_pixel(0, 0, 0, 0, 1) == expected,
            tfx == 2u
                ? "GS HIGHLIGHT texture-function mismatch"
                : "GS HIGHLIGHT2 texture-function mismatch") && ok;
        ok = expect(
            gs.stats().textured_raster_draws == 1 &&
            gs.stats().skipped_raster_draws == 0,
            "GS highlight texture draw was skipped") && ok;
    }

    // Affine FST triangle: XY and UV use the same fixed-point coordinates.
    {
        ps2::GsCore gs;
        gs.reset();
        for (ps2::u32 y = 0; y < 4; ++y) {
            for (ps2::u32 x = 0; x < 4; ++x) {
                const ps2::u32 color =
                    0xFF000000u | (x + 1u) | ((y + 1u) << 8);
                ok = expect(
                    gs.vram().write_pixel(
                        0, x, y, texture_bp, 1, color),
                    "triangle texture source setup failed") && ok;
            }
        }

        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x06, tex0(texture_bp, 1, 0, 2, 2, true, 1)); // 4x4 DECAL
        ad(gs, 0x08, 0u);
        ad(gs, 0x00, 3u | (1u << 4) | (1u << 8)); // triangle, TME, FST
        ad(gs, 0x01, 0xFFFFFFFFu);

        ad(gs, 0x03, uv(0, 0));
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x03, uv(48, 0));
        ad(gs, 0x05, xyz(48, 0));
        ad(gs, 0x03, uv(0, 48));
        ad(gs, 0x05, xyz(0, 48));

        ok = expect(
            gs.vram().read_pixel(0, 0, 0, 0, 1) == 0xFF000101u &&
            gs.vram().read_pixel(0, 1, 0, 0, 1) == 0xFF000102u &&
            gs.vram().read_pixel(0, 0, 1, 0, 1) == 0xFF000201u,
            "affine FST triangle sampling mismatch") && ok;
        ok = expect(gs.stats().textured_raster_draws == 1,
                    "textured triangle draw count mismatch") && ok;
    }

    // FST clear is now a supported STQ path. Default S/T/Q produces a
    // deterministic origin sample rather than an explicit skip.
    {
        ps2::GsCore gs;
        gs.reset();
        ok = expect(
            gs.vram().write_pixel(0, 0, 0, texture_bp, 1, 0xFF123456u),
            "STQ origin texture setup failed") && ok;
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x06, tex0(texture_bp, 1, 0, 1, 1, true, 1));
        ad(gs, 0x01, 0xFFFFFFFFu);
        ad(gs, 0x00, 6u | (1u << 4)); // TME, STQ.
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(32, 32));
        ok = expect(
            gs.stats().raster_draws == 1 &&
            gs.stats().skipped_raster_draws == 0,
            "STQ textured draw was skipped") && ok;
    }

    return ok;
}


bool test_gs_depth_layout_and_pixel_pipeline() {
    auto ad = [](ps2::GsCore& gs, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag = 1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };
    auto xyz = [](ps2::u32 x_fp, ps2::u32 y_fp, ps2::u32 z = 0) {
        return static_cast<ps2::u64>(x_fp & 0xFFFFu) |
               (static_cast<ps2::u64>(y_fp & 0xFFFFu) << 16) |
               (static_cast<ps2::u64>(z) << 32);
    };

    bool ok = true;

    // Depth formats use the GS Z swizzle, not the color swizzle.
    {
        ps2::GsVram vram;
        const ps2::u32 color_address =
            ps2::GsVram::pixel_address_bytes(0, 0, 0, 0, 1);
        const ps2::u32 depth_address =
            ps2::GsVram::depth_address_bytes(48, 0, 0, 0, 1);
        ok = expect(
            depth_address == (((color_address >> 2) ^ 0x600u) << 2),
            "PSMZ32 swizzle XOR mismatch") && ok;
        ok = expect(vram.write_depth(48, 0, 0, 0, 1, 0x12345678u) &&
                    vram.read_depth(48, 0, 0, 0, 1) == 0x12345678u,
                    "PSMZ32 read/write mismatch") && ok;
        ok = expect(vram.write_depth(49, 1, 0, 0, 1, 0xAABBCCDDu) &&
                    vram.read_depth(49, 1, 0, 0, 1) == 0x00BBCCDDu,
                    "PSMZ24 masking mismatch") && ok;
    }

    const ps2::u64 frame =
        static_cast<ps2::u64>(1u) << 16; // FBP=0, FBW=1, PSMCT32.
    const ps2::u64 scissor =
        (static_cast<ps2::u64>(7u) << 16) |
        (static_cast<ps2::u64>(7u) << 48);

    // FRAME.FBMSK is specified in RGBA32 bit positions even when the
    // framebuffer is PSMCT16. Red/blue mask bits must be packed to RGB5A1.
    {
        ps2::GsCore gs;
        gs.reset();
        const ps2::u64 frame16 =
            (1ull << 16) |
            (2ull << 24) |
            (0x00F800F8ull << 32);
        ok = expect(
            gs.vram().write_pixel(2, 0, 0, 0, 1, 0x7FFFu),
            "PSMCT16 FBMASK destination setup failed") && ok;

        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame16);
        ad(gs, 0x00, 0u); // point
        ad(gs, 0x01, 0x00000000u);
        ad(gs, 0x05, xyz(0, 0));

        ok = expect(
            gs.vram().read_pixel(2, 0, 0, 0, 1) == 0x7C1Fu,
            "PSMCT16 FBMASK packing mismatch") && ok;
        ok = expect(
            gs.stats().raster_draws == 1 &&
            gs.stats().skipped_raster_draws == 0,
            "PSMCT16 FBMASK draw was skipped") && ok;
    }

    // Alpha-only masking must preserve the RGB5A1 destination alpha bit.
    {
        ps2::GsCore gs;
        gs.reset();
        const ps2::u64 frame16_alpha =
            (1ull << 16) |
            (2ull << 24) |
            (0x80000000ull << 32);
        ok = expect(
            gs.vram().write_pixel(2, 0, 0, 0, 1, 0x8000u),
            "PSMCT16 alpha-mask destination setup failed") && ok;

        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame16_alpha);
        ad(gs, 0x00, 0u);
        ad(gs, 0x01, 0x00FFFFFFu);
        ad(gs, 0x05, xyz(0, 0));

        ok = expect(
            gs.vram().read_pixel(2, 0, 0, 0, 1) == 0xFFFFu,
            "PSMCT16 alpha FBMASK mismatch") && ok;
    }

    // Alpha test KEEP and RGB_ONLY.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x46, 1u); // COLCLAMP
        ad(gs, 0x4C, frame);

        // ATE=1, ATST=GREATER, AREF=0x80, AFAIL=KEEP.
        const ps2::u64 test_keep =
            1ull | (6ull << 1) | (0x80ull << 4);
        ad(gs, 0x47, test_keep);
        ad(gs, 0x00, 6u);
        ad(gs, 0x01, 0x40223344u);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(16, 16));
        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0,
                    "GS alpha KEEP failure wrote framebuffer") && ok;

        ad(gs, 0x00, 6u);
        ad(gs, 0x01, 0xC0223344u);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(16, 16));
        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0xC0223344u,
                    "GS alpha GREATER pass mismatch") && ok;

        // Failed alpha with RGB_ONLY updates RGB while preserving destination A.
        gs.vram().write_pixel(0, 1, 0, 0, 1, 0xAA010203u);
        const ps2::u64 test_rgb =
            test_keep | (3ull << 12);
        ad(gs, 0x47, test_rgb);
        ad(gs, 0x00, 6u);
        ad(gs, 0x01, 0x40112233u);
        ad(gs, 0x05, xyz(16, 0));
        ad(gs, 0x05, xyz(32, 16));
        ok = expect(gs.vram().read_pixel(0, 1, 0, 0, 1) == 0xAA112233u,
                    "GS alpha RGB_ONLY did not preserve destination alpha") && ok;
    }

    // ZTST=GEQUAL, Z write, and ZMSK.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x46, 1u);
        ad(gs, 0x4C, frame);

        constexpr ps2::u32 zbp_blocks = 128u;
        const ps2::u64 zbuf =
            static_cast<ps2::u64>(zbp_blocks >> 5) |
            (static_cast<ps2::u64>(48u) << 24);
        ad(gs, 0x4E, zbuf);
        ad(gs, 0x47, (1ull << 16) | (2ull << 17)); // ZTE + GEQUAL.
        ok = expect(gs.vram().write_depth(48, 0, 0, zbp_blocks, 1, 100u),
                    "GS Z source setup failed") && ok;

        ad(gs, 0x00, 6u);
        ad(gs, 0x01, 0xFF102030u);
        ad(gs, 0x05, xyz(0, 0, 50u));
        ad(gs, 0x05, xyz(16, 16, 50u));
        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0,
                    "GS GEQUAL accepted smaller Z") && ok;
        ok = expect(gs.vram().read_depth(48, 0, 0, zbp_blocks, 1) == 100u,
                    "GS rejected Z changed depth buffer") && ok;

        ad(gs, 0x00, 6u);
        ad(gs, 0x01, 0xFF405060u);
        ad(gs, 0x05, xyz(0, 0, 150u));
        ad(gs, 0x05, xyz(16, 16, 150u));
        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0xFF405060u,
                    "GS GEQUAL accepted draw color mismatch") && ok;
        ok = expect(gs.vram().read_depth(48, 0, 0, zbp_blocks, 1) == 150u,
                    "GS Z write mismatch") && ok;

        // Set ZMSK and verify color still writes but depth does not.
        ad(gs, 0x4E, zbuf | (1ull << 32));
        ad(gs, 0x00, 6u);
        ad(gs, 0x01, 0xFF708090u);
        ad(gs, 0x05, xyz(0, 0, 200u));
        ad(gs, 0x05, xyz(16, 16, 200u));
        ok = expect(gs.vram().read_depth(48, 0, 0, zbp_blocks, 1) == 150u,
                    "GS ZMSK did not block depth write") && ok;

        // ZBUF.PSM=0 is the BIOS's short encoding for PSMZ32, distinct
        // from the 0x30 format code used by GS local-memory transfers.
        ad(gs, 0x4E, static_cast<ps2::u64>(zbp_blocks >> 5));
        ad(gs, 0x00, 6u);
        ad(gs, 0x01, 0xFF90A0B0u);
        ad(gs, 0x05, xyz(0, 0, 250u));
        ad(gs, 0x05, xyz(16, 16, 250u));
        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0xFF90A0B0u &&
                    gs.vram().read_depth(48, 0, 0, zbp_blocks, 1) == 250u,
                    "GS short ZBUF.PSM encoding did not render/write Z32") && ok;
    }

    // Alpha blend equation, PABE bypass, and FBA.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x46, 1u);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);

        // (Cs - Cd) * FIX/128 + Cd with FIX=0x40 => exact half blend.
        const ps2::u64 alpha =
            0ull | (1ull << 2) | (2ull << 4) | (1ull << 6) |
            (0x40ull << 32);
        ad(gs, 0x42, alpha);
        gs.vram().write_pixel(0, 0, 0, 0, 1, 0x80204060u);
        ad(gs, 0x00, 6u | (1u << 6)); // sprite + ABE
        ad(gs, 0x01, 0x80A08040u);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(16, 16));
        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0x80606050u,
                    "GS alpha blend equation mismatch") && ok;

        // PABE bypasses blending when source alpha MSB is clear.
        ad(gs, 0x49, 1u);
        gs.vram().write_pixel(0, 1, 0, 0, 1, 0x80FFFFFFu);
        ad(gs, 0x00, 6u | (1u << 6));
        ad(gs, 0x01, 0x40112233u);
        ad(gs, 0x05, xyz(16, 0));
        ad(gs, 0x05, xyz(32, 16));
        ok = expect(gs.vram().read_pixel(0, 1, 0, 0, 1) == 0x40112233u,
                    "GS PABE did not bypass blending") && ok;

        // FBA forces the framebuffer alpha MSB on normal writes.
        ad(gs, 0x49, 0u);
        ad(gs, 0x4A, 1u);
        ad(gs, 0x00, 6u);
        ad(gs, 0x01, 0x00123456u);
        ad(gs, 0x05, xyz(32, 0));
        ad(gs, 0x05, xyz(48, 16));
        ok = expect(gs.vram().read_pixel(0, 2, 0, 0, 1) == 0x80123456u,
                    "GS FBA did not force alpha MSB") && ok;
    }

    // IIP/Gouraud barycentric color interpolation.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x46, 1u);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x00, 3u | (1u << 3)); // triangle + IIP

        ad(gs, 0x01, 0x800000FFu);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x01, 0x8000FF00u);
        ad(gs, 0x05, xyz(48, 0));
        ad(gs, 0x01, 0x80FF0000u);
        ad(gs, 0x05, xyz(0, 48));

        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0x802A2AAAu,
                    "GS Gouraud interpolation mismatch") && ok;
        ok = expect(gs.stats().raster_draws == 1 &&
                    gs.stats().skipped_raster_draws == 0,
                    "GS Gouraud draw was skipped") && ok;
    }

    return ok;
}


bool test_gs_stq_perspective_texturing() {
    auto ad = [](ps2::GsCore& gs, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag = 1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };
    auto xyz = [](ps2::u32 x_fp, ps2::u32 y_fp) {
        return static_cast<ps2::u64>(x_fp & 0xFFFFu) |
               (static_cast<ps2::u64>(y_fp & 0xFFFFu) << 16);
    };
    auto rgbaq = [](ps2::u32 rgba, float q) {
        return static_cast<ps2::u64>(rgba) |
               (static_cast<ps2::u64>(std::bit_cast<ps2::u32>(q)) << 32);
    };
    auto st = [](float s, float t) {
        return static_cast<ps2::u64>(std::bit_cast<ps2::u32>(s)) |
               (static_cast<ps2::u64>(std::bit_cast<ps2::u32>(t)) << 32);
    };

    ps2::GsCore gs;
    gs.reset();
    bool ok = true;

    constexpr ps2::u32 texture_bp = 0x100u;
    ok = expect(gs.vram().write_pixel(0, 0, 0, texture_bp, 1, 0xFF0000FFu) &&
                gs.vram().write_pixel(0, 1, 0, texture_bp, 1, 0xFF00FF00u) &&
                gs.vram().write_pixel(0, 0, 1, texture_bp, 1, 0xFFFF0000u) &&
                gs.vram().write_pixel(0, 1, 1, texture_bp, 1, 0xFFFFFFFFu),
                "STQ texture setup failed") && ok;

    ad(gs, 0x1A, 1u);
    ad(gs, 0x18, 0u);
    ad(gs, 0x40,
       (static_cast<ps2::u64>(3u) << 16) |
       (static_cast<ps2::u64>(3u) << 48));
    ad(gs, 0x46, 1u);
    ad(gs, 0x4C, static_cast<ps2::u64>(1u) << 16);

    const ps2::u64 tex0 =
        static_cast<ps2::u64>(texture_bp) |
        (1ull << 14) |          // TBW=1
        (1ull << 26) |          // TW=1 => 2 texels
        (1ull << 30) |          // TH=1
        (1ull << 35);           // TFX=DECAL
    ad(gs, 0x06, tex0);

    // Triangle, TME=1, FST=0.  S and Q vary together on the right vertex,
    // making S/Q perspective-correct rather than a plain affine S.
    ad(gs, 0x00, 3u | (1u << 4));
    ad(gs, 0x01, rgbaq(0xFF000000u, 1.0f));
    ad(gs, 0x02, st(0.0f, 0.0f));
    ad(gs, 0x05, xyz(0, 0));

    ad(gs, 0x01, rgbaq(0xFF000000u, 2.0f));
    ad(gs, 0x02, st(2.0f, 0.0f));
    ad(gs, 0x05, xyz(32, 0));

    ad(gs, 0x01, rgbaq(0xFF000000u, 1.0f));
    ad(gs, 0x02, st(0.0f, 1.0f));
    ad(gs, 0x05, xyz(0, 32));

    // At pixel center (8,8), interpolated S=0.5 and Q=1.25, so
    // S/Q*2 = 0.8 and the nearest integer texel is still x=0.
    ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0xFF0000FFu,
                "GS STQ perspective sample mismatch") && ok;
    ok = expect(gs.stats().textured_raster_draws == 1 &&
                gs.stats().skipped_raster_draws == 0,
                "GS STQ draw was skipped") && ok;
    return ok;
}


bool test_gs_indexed_textures_and_texa() {
    auto ad = [](ps2::GsCore& gs, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag = 1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };
    auto xyz = [](ps2::u32 x_fp, ps2::u32 y_fp) {
        return static_cast<ps2::u64>(x_fp & 0xFFFFu) |
               (static_cast<ps2::u64>(y_fp & 0xFFFFu) << 16);
    };
    auto uv = [](ps2::u32 u_fp, ps2::u32 v_fp) {
        return static_cast<ps2::u64>(u_fp & 0x3FFFu) |
               (static_cast<ps2::u64>(v_fp & 0x3FFFu) << 16);
    };

    bool ok = true;

    // Indexed swizzle read/write, including the formats embedded in the high
    // bits of PSMCT32 storage.
    {
        ps2::GsVram vram;
        constexpr ps2::u32 bp = 32;
        ok = expect(vram.write_index(19, 0, 0, bp, 2, 0x12u) &&
                    vram.write_index(19, 1, 0, bp, 2, 0x34u),
                    "PSMT8 write failed") && ok;
        ok = expect(vram.read_index(19, 0, 0, bp, 2) == 0x12u &&
                    vram.read_index(19, 1, 0, bp, 2) == 0x34u,
                    "PSMT8 readback mismatch") && ok;
        ok = expect(vram.byte_at((bp << 8) + 0u) == 0x12u &&
                    vram.byte_at((bp << 8) + 4u) == 0x34u,
                    "PSMT8 column swizzle mismatch") && ok;

        constexpr ps2::u32 bp4 = 48;
        ok = expect(vram.write_index(20, 0, 0, bp4, 2, 0xAu) &&
                    vram.write_index(20, 1, 0, bp4, 2, 0xBu),
                    "PSMT4 write failed") && ok;
        ok = expect(vram.read_index(20, 0, 0, bp4, 2) == 0xAu &&
                    vram.read_index(20, 1, 0, bp4, 2) == 0xBu,
                    "PSMT4 readback mismatch") && ok;
        ok = expect((vram.byte_at((bp4 << 8) + 0u) & 0x0Fu) == 0xAu &&
                    (vram.byte_at((bp4 << 8) + 4u) & 0x0Fu) == 0xBu,
                    "PSMT4 column swizzle mismatch") && ok;

        constexpr ps2::u32 bph = 64;
        ok = expect(vram.write_pixel(0, 0, 0, bph, 1, 0x11223344u) &&
                    vram.write_index(27, 0, 0, bph, 1, 0xAAu),
                    "PSMT8H write failed") && ok;
        ok = expect(vram.read_pixel(0, 0, 0, bph, 1) == 0xAA223344u,
                    "PSMT8H high-byte placement mismatch") && ok;
        ok = expect(vram.write_index(36, 0, 0, bph, 1, 0x5u) &&
                    vram.read_pixel(0, 0, 0, bph, 1) == 0xA5223344u,
                    "PSMT4HL placement mismatch") && ok;
        ok = expect(vram.write_index(44, 0, 0, bph, 1, 0xCu) &&
                    vram.read_pixel(0, 0, 0, bph, 1) == 0xC5223344u,
                    "PSMT4HH placement mismatch") && ok;
    }

    // CSM1 CLUT permutations and TEXA expansion.
    {
        ps2::GsVram vram;
        constexpr ps2::u32 cbp = 96;

        // Logical PSMT4 index 2 maps to raw T32 word 4.
        vram.write_linear32(cbp, 4, 0x7F112233u);
        ok = expect(
            vram.read_clut_color(
                20, 2, cbp, 0, false, 0, 0, 0, 0, 0, 0, false) ==
                0x7F112233u,
            "PSMT4 CSM1 32-bit CLUT permutation mismatch") && ok;

        // Logical PSMT8 index 16 begins at the CSM1 source-column 64.
        vram.write_linear32(cbp, 64, 0xCC445566u);
        ok = expect(
            vram.read_clut_color(
                19, 16, cbp, 0, false, 0, 0, 0, 0, 0, 0, false) ==
                0xCC445566u,
            "PSMT8 CSM1 32-bit CLUT permutation mismatch") && ok;

        constexpr ps2::u32 cbp16 = 104;
        // Logical 4-bit index 2 maps to raw 16-bit halfword 8.
        vram.write_linear16(cbp16, 8, 0x001Fu);
        ok = expect(
            vram.read_clut_color(
                20, 2, cbp16, 2, false, 0, 0, 0, 0,
                0x40u, 0xE0u, false) == 0x400000F8u,
            "16-bit CLUT TEXA.TA0 expansion mismatch") && ok;
        vram.write_linear16(cbp16, 8, 0x801Fu);
        ok = expect(
            vram.read_clut_color(
                20, 2, cbp16, 2, false, 0, 0, 0, 0,
                0x40u, 0xE0u, false) == 0xE00000F8u,
            "16-bit CLUT TEXA.TA1 expansion mismatch") && ok;
        vram.write_linear16(cbp16, 8, 0x0000u);
        ok = expect(
            vram.read_clut_color(
                20, 2, cbp16, 2, false, 0, 0, 0, 0,
                0x40u, 0xE0u, true) == 0,
            "16-bit CLUT TEXA.AEM zero handling mismatch") && ok;
    }

    // Host-to-local IMAGE streams for indexed formats.
    {
        ps2::GsCore gs;
        gs.reset();

        const ps2::u64 blit8 =
            (static_cast<ps2::u64>(2u) << 48) |
            (static_cast<ps2::u64>(19u) << 56);
        ad(gs, 0x50, blit8);
        ad(gs, 0x51, 0);
        ad(gs, 0x52, 4ull | (1ull << 32));
        ad(gs, 0x53, 0);
        gs.write_gif_qword(
            1ull | (1ull << 15) | (2ull << 58), 0);
        gs.write_gif_qword(0x0000000004030201ull, 0);

        ok = expect(!gs.transfer_active() &&
                    gs.vram().read_index(19, 0, 0, 0, 2) == 1u &&
                    gs.vram().read_index(19, 1, 0, 0, 2) == 2u &&
                    gs.vram().read_index(19, 2, 0, 0, 2) == 3u &&
                    gs.vram().read_index(19, 3, 0, 0, 2) == 4u,
                    "PSMT8 IMAGE upload mismatch") && ok;

        gs.reset();
        const ps2::u64 blit4 =
            (static_cast<ps2::u64>(2u) << 48) |
            (static_cast<ps2::u64>(20u) << 56);
        ad(gs, 0x50, blit4);
        ad(gs, 0x51, 0);
        ad(gs, 0x52, 4ull | (1ull << 32));
        ad(gs, 0x53, 0);
        gs.write_gif_qword(
            1ull | (1ull << 15) | (2ull << 58), 0);
        gs.write_gif_qword(0x0000000000004321ull, 0);

        ok = expect(!gs.transfer_active() &&
                    gs.vram().read_index(20, 0, 0, 0, 2) == 1u &&
                    gs.vram().read_index(20, 1, 0, 0, 2) == 2u &&
                    gs.vram().read_index(20, 2, 0, 0, 2) == 3u &&
                    gs.vram().read_index(20, 3, 0, 0, 2) == 4u,
                    "PSMT4 IMAGE nibble upload mismatch") && ok;
    }

    // End-to-end PSMT8 + CSM1 palette textured sprite.
    {
        ps2::GsCore gs;
        gs.reset();
        constexpr ps2::u32 texture_bp = 128;
        constexpr ps2::u32 palette_bp = 160;

        ok = expect(
            gs.vram().write_index(19, 0, 0, texture_bp, 2, 2u) &&
            gs.vram().write_index(19, 1, 0, texture_bp, 2, 3u),
            "indexed sprite texture setup failed") && ok;
        // CSM1 index 2/3 map to source words 4/5.
        gs.vram().write_linear32(palette_bp, 4, 0xFF112233u);
        gs.vram().write_linear32(palette_bp, 5, 0xFF445566u);

        const ps2::u64 frame = static_cast<ps2::u64>(1u) << 16;
        const ps2::u64 scissor =
            (static_cast<ps2::u64>(3u) << 16) |
            (static_cast<ps2::u64>(3u) << 48);
        const ps2::u64 tex0 =
            static_cast<ps2::u64>(texture_bp) |
            (2ull << 14) |
            (19ull << 20) |
            (1ull << 26) |
            (1ull << 34) |
            (1ull << 35) |
            (static_cast<ps2::u64>(palette_bp) << 37);

        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x46, 1u);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame);
        ad(gs, 0x06, tex0);
        ad(gs, 0x08, 0u);
        ad(gs, 0x00, 6u | (1u << 4) | (1u << 8));
        ad(gs, 0x01, 0xFFFFFFFFu);
        ad(gs, 0x03, uv(0, 0));
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x03, uv(32, 16));
        ad(gs, 0x05, xyz(32, 16));

        ok = expect(
            gs.vram().read_pixel(0, 0, 0, 0, 1) == 0xFF112233u &&
            gs.vram().read_pixel(0, 1, 0, 0, 1) == 0xFF445566u,
            "PSMT8 palette textured sprite mismatch") && ok;
        ok = expect(gs.stats().textured_raster_draws == 1,
                    "indexed textured draw was skipped") && ok;
    }

    return ok;
}


bool test_gs_local_copy_and_depth_transfer() {
    auto ad = [](ps2::GsCore& gs, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag = 1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };

    bool ok = true;

    // Overlapping same-buffer copy must honor DIRX so source data is not
    // destroyed before it is read.
    {
        ps2::GsCore gs;
        gs.reset();
        ok = expect(
            gs.vram().write_pixel(0, 0, 0, 0, 1, 0x11111111u) &&
            gs.vram().write_pixel(0, 1, 0, 0, 1, 0x22222222u) &&
            gs.vram().write_pixel(0, 2, 0, 0, 1, 0x33333333u),
            "local-copy overlap source setup failed") && ok;

        const ps2::u64 blit =
            (static_cast<ps2::u64>(1u) << 16) |
            (static_cast<ps2::u64>(1u) << 48);
        const ps2::u64 pos =
            (static_cast<ps2::u64>(1u) << 32) |
            (1ull << 60); // SSAX=0, DSAX=1, DIRX=1.
        ad(gs, 0x50, blit);
        ad(gs, 0x51, pos);
        ad(gs, 0x52, 3ull | (1ull << 32));
        ad(gs, 0x53, 2u); // local -> local

        ok = expect(
            gs.vram().read_pixel(0, 0, 0, 0, 1) == 0x11111111u &&
            gs.vram().read_pixel(0, 1, 0, 0, 1) == 0x11111111u &&
            gs.vram().read_pixel(0, 2, 0, 0, 1) == 0x22222222u &&
            gs.vram().read_pixel(0, 3, 0, 0, 1) == 0x33333333u,
            "DIRX overlap-safe local copy mismatch") && ok;
        ok = expect(
            gs.stats().local_to_local_transfers == 1 &&
            gs.stats().local_to_local_pixels == 3 &&
            (gs.register_value(0x53) & 3u) == 3u,
            "local-copy completion/statistics mismatch") && ok;
    }

    // Indexed formats use the same transfer engine and preserve index values.
    {
        ps2::GsCore gs;
        gs.reset();
        constexpr ps2::u32 src_bp = 64;
        constexpr ps2::u32 dst_bp = 96;
        for (ps2::u32 x = 0; x < 4; ++x) {
            ok = expect(
                gs.vram().write_index(19, x, 0, src_bp, 2, 0x20u + x),
                "indexed local-copy source setup failed") && ok;
        }

        const ps2::u64 blit =
            static_cast<ps2::u64>(src_bp) |
            (2ull << 16) |
            (19ull << 24) |
            (static_cast<ps2::u64>(dst_bp) << 32) |
            (2ull << 48) |
            (19ull << 56);
        ad(gs, 0x50, blit);
        ad(gs, 0x51, 0);
        ad(gs, 0x52, 4ull | (1ull << 32));
        ad(gs, 0x53, 2u);

        ok = expect(
            gs.vram().read_index(19, 0, 0, dst_bp, 2) == 0x20u &&
            gs.vram().read_index(19, 1, 0, dst_bp, 2) == 0x21u &&
            gs.vram().read_index(19, 2, 0, dst_bp, 2) == 0x22u &&
            gs.vram().read_index(19, 3, 0, dst_bp, 2) == 0x23u,
            "PSMT8 local-to-local copy mismatch") && ok;
    }

    // Host IMAGE transfers can initialize Z buffers using their transfer bpp.
    {
        ps2::GsCore gs;
        gs.reset();
        constexpr ps2::u32 zbp = 128;
        const ps2::u64 blit =
            (static_cast<ps2::u64>(zbp) << 32) |
            (1ull << 48) |
            (48ull << 56);
        ad(gs, 0x50, blit);
        ad(gs, 0x51, 0);
        ad(gs, 0x52, 2ull | (1ull << 32));
        ad(gs, 0x53, 0u);

        gs.write_gif_qword(
            1ull | (1ull << 15) | (2ull << 58), 0);
        gs.write_gif_qword(
            0x5566778811223344ull, 0);

        ok = expect(
            !gs.transfer_active() &&
            gs.vram().read_depth(48, 0, 0, zbp, 1) == 0x11223344u &&
            gs.vram().read_depth(48, 1, 0, zbp, 1) == 0x55667788u,
            "PSMZ32 host IMAGE upload mismatch") && ok;
    }

    return ok;
}


bool test_gs_signal_finish_label_and_imr() {
    ps2::Ps2System system;
    auto& gs = system.gs_core();
    auto& priv = system.gs_privileged();

    auto ad = [](ps2::GsCore& core, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag = 1ull | (1ull << 15) | (1ull << 60);
        core.write_gif_qword(tag, 0xEull);
        core.write_gif_qword(value, address);
    };

    bool ok = true;
    ps2::u32 value = 0;

    ok = expect(priv.read32(0x12001000u, value) &&
                (value & 0xFFFFC000u) == 0x551B4000u,
                "GS CSR reset identity/FIFO mismatch") && ok;
    ok = expect(priv.read32(0x12001010u, value) &&
                value == 0x00007F00u,
                "GS IMR reset mask mismatch") && ok;

    // Unmask SIGNAL only. IMR uses one=masked.
    ok = expect(priv.write32(0x12001010u, 0x00001E00u),
                "GS IMR SIGNAL unmask failed") && ok;

    ad(gs, 0x60, 0xFFFFFFFF12345678ull);
    ok = expect((priv.csr() & 1u) != 0 &&
                priv.signal_id() == 0x12345678u,
                "GS SIGNAL state/SIGID mismatch") && ok;
    ok = expect(priv.irq_pending(),
                "unmasked GS SIGNAL did not become IRQ-pending") && ok;

    // A second SIGNAL stalls/queues behind the first on hardware. Keep the
    // first ID visible until CSR.SIGNAL is acknowledged, then promote it.
    ad(gs, 0x60, 0x00FF00FFAA55AA55ull);
    ok = expect(priv.signal_id() == 0x12345678u,
                "queued SIGNAL replaced active SIGID too early") && ok;
    ok = expect(priv.write32(0x12001000u, 1u),
                "GS CSR SIGNAL acknowledge failed") && ok;
    const ps2::u32 expected_queued =
        (0x12345678u & ~0x00FF00FFu) |
        (0xAA55AA55u & 0x00FF00FFu);
    ok = expect((priv.csr() & 1u) != 0 &&
                priv.signal_id() == expected_queued,
                "queued GS SIGNAL promotion mismatch") && ok;
    ok = expect(priv.write32(0x12001000u, 1u) &&
                (priv.csr() & 1u) == 0,
                "second GS SIGNAL acknowledge failed") && ok;

    // LABEL updates LBLID with IDMSK but does not create an interrupt event.
    ad(gs, 0x62, 0xFFFF000089ABCDEFu);
    ok = expect(priv.label_id() == 0x89AB0000u,
                "GS LABEL masked update mismatch") && ok;

    // FINISH is independently maskable.
    ok = expect(priv.write32(0x12001010u, 0x00001D00u),
                "GS IMR FINISH unmask failed") && ok;
    ad(gs, 0x61, 0);
    ok = expect((priv.csr() & 2u) != 0 && priv.irq_pending(),
                "GS FINISH did not set pending event") && ok;
    ok = expect(priv.write32(0x12001000u, 2u) &&
                (priv.csr() & 2u) == 0,
                "GS FINISH acknowledge failed") && ok;

    // VSINT participates in the same mask/event circuit.
    ok = expect(priv.write32(0x12001010u, 0x00001700u),
                "GS IMR VSINT unmask failed") && ok;
    priv.raise_vsync();
    ok = expect((priv.csr() & (1u << 3)) != 0 && priv.irq_pending(),
                "GS VSINT pending state mismatch") && ok;
    ok = expect((priv.csr() & (1u << 13)) != 0,
                "GS CSR FIELD did not advance to odd") && ok;
    ok = expect(priv.write32(0x12001000u, 1u << 3) &&
                (priv.csr() & (1u << 3)) == 0,
                "GS VSINT acknowledge failed") && ok;

    priv.raise_vsync();
    ok = expect((priv.csr() & (1u << 13)) == 0,
                "GS CSR FIELD did not advance back to even") && ok;
    ok = expect(priv.write32(0x12001000u, 1u << 3),
                "second GS VSINT acknowledge failed") && ok;

    // CSR.RESET restores the hardware-visible interrupt masks/identity state.
    ok = expect(priv.write32(0x12001000u, 1u << 9),
                "GS CSR soft reset failed") && ok;
    ok = expect(priv.imr() == 0x00007F00u &&
                (priv.csr() & 0xFFFFC01Fu) == 0x551B4000u &&
                priv.signal_id() == 0 && priv.label_id() == 0,
                "GS CSR soft reset state mismatch") && ok;

    const auto& stats = gs.stats();
    ok = expect(stats.signal_events == 2 &&
                stats.finish_events == 1 &&
                stats.label_events == 1,
                "GS synchronization event statistics mismatch") && ok;

    return ok;
}


bool test_gs_fog_dither_scanmask_and_context2() {
    auto ad = [](ps2::GsCore& gs, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag =
            1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xEull);
        gs.write_gif_qword(value, address);
    };
    auto xyz = [](ps2::u32 x_fp, ps2::u32 y_fp, ps2::u32 z = 0) {
        return static_cast<ps2::u64>(x_fp & 0xFFFFu) |
               (static_cast<ps2::u64>(y_fp & 0xFFFFu) << 16) |
               (static_cast<ps2::u64>(z) << 32);
    };

    const ps2::u64 frame32 = static_cast<ps2::u64>(1u) << 16;
    const ps2::u64 scissor =
        (static_cast<ps2::u64>(31u) << 16) |
        (static_cast<ps2::u64>(31u) << 48);
    bool ok = true;

    // Packed FOG uses bits 100..107, not the low 64-bit payload.
    {
        ps2::GsCore gs;
        gs.reset();
        const ps2::u64 tag =
            1ull | (1ull << 15) | (1ull << 60);
        gs.write_gif_qword(tag, 0xAull);
        gs.write_gif_qword(0, static_cast<ps2::u64>(0x5Au) << 36);
        ok = expect(
            gs.register_value(0x0A) ==
                (static_cast<ps2::u64>(0x5Au) << 56),
            "packed FOG field decode mismatch") && ok;
    }

    // Packed texture-state descriptors carry their native 64-bit register
    // payload in the low half of the GIF qword.
    {
        ps2::GsCore gs;
        gs.reset();
        const ps2::u64 tex0_1 = 0x0123456789ABCDEFull;
        const ps2::u64 tex0_2 = 0x0011223344556677ull;
        const ps2::u64 clamp_1 = 0x000003FFABC01234ull;
        const ps2::u64 clamp_2 = 0x000000123456789Aull;

        auto packed = [&](ps2::u32 descriptor, ps2::u64 value) {
            const ps2::u64 tag =
                1ull | (1ull << 15) | (1ull << 60);
            gs.write_gif_qword(tag, descriptor);
            gs.write_gif_qword(value, 0);
        };

        packed(0x06, tex0_1);
        packed(0x07, tex0_2);
        packed(0x08, clamp_1);
        packed(0x09, clamp_2);

        ok = expect(
            gs.register_value(0x06) == tex0_1 &&
            gs.register_value(0x07) == tex0_2 &&
            gs.register_value(0x08) == clamp_1 &&
            gs.register_value(0x09) == clamp_2,
            "packed TEX0/CLAMP descriptor decode mismatch") && ok;
    }

    // FGE blends RGB toward FOGCOL using the per-vertex fog coefficient.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame32);
        ad(gs, 0x3D, 0x00FF0000u); // blue fog
        ad(gs, 0x0A, static_cast<ps2::u64>(128u) << 56);
        ad(gs, 0x00, 1u << 5); // point + FGE
        ad(gs, 0x01, 0x800000FFu); // red
        ad(gs, 0x05, xyz(0, 0));

        ok = expect(
            gs.vram().read_pixel(0, 0, 0, 0, 1) == 0x807F007Full,
            "GS fog blend mismatch") && ok;
        ok = expect(gs.stats().skipped_raster_draws == 0,
                    "FGE draw was still rejected") && ok;
    }

    // DTHE/DIMX only affect 16-bit color quantization.
    {
        ps2::GsCore gs;
        gs.reset();
        const ps2::u64 frame16 =
            (static_cast<ps2::u64>(1u) << 16) |
            (static_cast<ps2::u64>(2u) << 24);
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame16);
        ad(gs, 0x44, 3u); // DIMX[0][0] = +3
        ad(gs, 0x45, 1u); // DTHE
        ad(gs, 0x00, 0u);
        ad(gs, 0x01, 0x80050505u);
        ad(gs, 0x05, xyz(0, 0));

        ok = expect(
            gs.vram().read_pixel(2, 0, 0, 0, 1) == 0x8421u,
            "GS 16-bit dithering mismatch") && ok;
    }

    // SCANMSK=2 prohibits even scanlines while leaving odd scanlines active.
    {
        ps2::GsCore gs;
        gs.reset();
        ad(gs, 0x1A, 1u);
        ad(gs, 0x18, 0u);
        ad(gs, 0x40, scissor);
        ad(gs, 0x47, 0u);
        ad(gs, 0x4C, frame32);
        ad(gs, 0x22, 2u);
        ad(gs, 0x00, 6u); // sprite
        ad(gs, 0x01, 0xA0112233u);
        ad(gs, 0x05, xyz(0, 0));
        ad(gs, 0x05, xyz(16, 32));

        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0,
                    "SCANMSK drew prohibited even line") && ok;
        ok = expect(gs.vram().read_pixel(0, 0, 1, 0, 1) == 0xA0112233u,
                    "SCANMSK suppressed allowed odd line") && ok;
    }

    // CTXT=1 must use the second XYOFFSET/SCISSOR/TEST/FRAME register bank.
    {
        ps2::GsCore gs;
        gs.reset();
        const ps2::u64 frame2 =
            1ull |
            (static_cast<ps2::u64>(1u) << 16); // FBP=1 -> BP 32
        ad(gs, 0x1A, 1u);
        ad(gs, 0x19, 0u);
        ad(gs, 0x41, scissor);
        ad(gs, 0x48, 0u);
        ad(gs, 0x4D, frame2);
        ad(gs, 0x00, 1ull << 9); // point + CTXT=1
        ad(gs, 0x01, 0xCC445566u);
        ad(gs, 0x05, xyz(0, 0));

        ok = expect(
            gs.vram().read_pixel(0, 0, 0, 32, 1) == 0xCC445566u,
            "GS context-2 FRAME target mismatch") && ok;
        ok = expect(gs.vram().read_pixel(0, 0, 0, 0, 1) == 0,
                    "GS context-2 draw leaked into context-1 target") && ok;
    }

    return ok;
}


bool test_gs_local_to_host_transfer() {
    ps2::GsCore gs;
    gs.reset();

    auto ad = [](ps2::GsCore& core, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag = 1ull | (1ull << 15) | (1ull << 60);
        core.write_gif_qword(tag, 0xEull);
        core.write_gif_qword(value, address);
    };

    bool ok = true;
    const ps2::u32 pixels[6] = {
        0x030201u, 0x060504u, 0x090807u,
        0x0C0B0Au, 0x0F0E0Du, 0x121110u,
    };
    for (ps2::u32 x = 0; x < 6; ++x) {
        ok = expect(gs.vram().write_pixel(1, x, 0, 0, 1, pixels[x]),
                    "local-to-host source setup failed") && ok;
    }

    // Source side of BITBLTBUF: SBP=0, SBW=1, SPSM=PSMCT24.
    const ps2::u64 blit =
        (1ull << 16) |
        (1ull << 24);
    ad(gs, 0x50, blit);
    ad(gs, 0x51, 0);
    ad(gs, 0x52, 6ull | (1ull << 32));
    ad(gs, 0x53, 1u);

    ps2::u64 lo = 0;
    ps2::u64 hi = 0;
    ok = expect(gs.transfer_active(),
                "local-to-host transfer did not arm") && ok;
    ok = expect(gs.read_local_to_host_qword(lo, hi),
                "local-to-host first qword missing") && ok;
    ok = expect(lo == 0x0807060504030201ull &&
                hi == 0x100F0E0D0C0B0A09ull,
                "local-to-host first 24-bit qword mismatch") && ok;
    ok = expect(gs.transfer_active(),
                "24-bit readback lost cross-qword remainder") && ok;

    lo = hi = 0;
    ok = expect(gs.read_local_to_host_qword(lo, hi),
                "local-to-host second qword missing") && ok;
    ok = expect(lo == 0x1211ull && hi == 0,
                "local-to-host 24-bit tail mismatch") && ok;
    ok = expect(!gs.transfer_active() &&
                (gs.register_value(0x53) & 3u) == 3u,
                "local-to-host transfer did not complete") && ok;

    const auto& stats = gs.stats();
    ok = expect(stats.local_to_host_transfers == 1 &&
                stats.local_to_host_pixels == 6 &&
                stats.local_to_host_qwords == 2 &&
                stats.local_to_host_bytes == 18,
                "local-to-host statistics mismatch") && ok;
    return ok;
}

bool test_vif1_reverse_dma() {
    ps2::Ps2System system;
    ps2::Vif1Dma dma;
    dma.reset();

    auto ad = [](ps2::GsCore& core, ps2::u32 address, ps2::u64 value) {
        const ps2::u64 tag = 1ull | (1ull << 15) | (1ull << 60);
        core.write_gif_qword(tag, 0xEull);
        core.write_gif_qword(value, address);
    };

    bool ok = true;
    std::string error;

    for (ps2::u32 x = 0; x < 4; ++x) {
        ok = expect(
            system.gs_core().vram().write_pixel(
                0, x, 0, 0, 1, 0x11111111u * (x + 1u)),
            "VIF1 reverse source setup failed") && ok;
    }

    const ps2::u64 blit = 1ull << 16; // SBP=0, SBW=1, SPSM=PSMCT32.
    ad(system.gs_core(), 0x50, blit);
    ad(system.gs_core(), 0x51, 0);
    ad(system.gs_core(), 0x52, 4ull | (1ull << 32));
    ad(system.gs_core(), 0x53, 1u);

    constexpr ps2::u32 dmac_ctrl = 0x1000E000u;
    constexpr ps2::u32 dmac_stat = 0x1000E010u;
    constexpr ps2::u32 vif1_chcr = 0x10009000u;
    constexpr ps2::u32 vif1_madr = 0x10009010u;
    constexpr ps2::u32 vif1_qwc = 0x10009020u;
    constexpr ps2::u32 vif1_stat = 0x10003C00u;
    constexpr ps2::u32 gs_busdir = 0x12001040u;

    ok = expect(system.bus().write64(gs_busdir, 1u),
                "GS BUSDIR reverse setup failed") && ok;
    ok = expect(system.bus().write32(vif1_stat, 1u << 23),
                "VIF1 FDR setup failed") && ok;
    ok = expect(system.bus().write32(dmac_ctrl, 1u) &&
                system.bus().write32(dmac_stat, 1u << 17) &&
                system.bus().write32(vif1_madr, 0x6000u) &&
                system.bus().write32(vif1_qwc, 1u) &&
                system.bus().write32(vif1_chcr, 0x100u),
                "VIF1 reverse DMA register setup failed") && ok;

    ok = expect(dma.service(
                    system.bus(),
                    system.gs_core(),
                    system.gs_privileged(),
                    error),
                "VIF1 reverse DMA service failed") && ok;

    ps2::u64 lo = 0;
    ps2::u64 hi = 0;
    ps2::u32 value = 0;
    ok = expect(system.bus().read64(0x6000u, lo) &&
                system.bus().read64(0x6008u, hi) &&
                lo == 0x2222222211111111ull &&
                hi == 0x4444444433333333ull,
                "VIF1 reverse DMA RAM payload mismatch") && ok;
    ok = expect(system.bus().read32(vif1_madr, value) &&
                value == 0x6010u,
                "VIF1 reverse MADR advance mismatch") && ok;
    ok = expect(system.bus().read32(vif1_qwc, value) && value == 0,
                "VIF1 reverse QWC did not reach zero") && ok;
    ok = expect(system.bus().read32(vif1_chcr, value) &&
                (value & 0x100u) == 0,
                "VIF1 reverse STR did not clear") && ok;
    ok = expect(system.bus().read32(dmac_stat, value) &&
                (value & (1u << 1)) != 0,
                "VIF1 reverse completion cause missing") && ok;
    ok = expect(system.bus().dmac_pending(),
                "VIF1 reverse completion did not assert DMAC pending") && ok;
    ok = expect(!system.gs_core().transfer_active(),
                "GS local-to-host transfer remained active") && ok;
    return ok;
}

bool test_ee_native_linear_block() {
    constexpr ps2::u32 pc = 0x5000u;
    const std::array<ps2::u32, 4> code = {
        (0x09u << 26) | (1u << 16) | 7u,
        (0x0Du << 26) | (1u << 21) | (2u << 16) | 0x100u,
        (0u << 26) | (1u << 21) | (2u << 16) | (3u << 11) | 0x2Du,
        (0u << 26) | (3u << 16) | (4u << 11) | (2u << 6) | 0x00u,
    };

    ps2::Ps2System exact;
    ps2::Ps2System native;
    exact.ee().reset(pc);
    native.ee().reset(pc);

    std::string error;
    bool ok = true;
    for (ps2::u32 instruction : code) {
        ok = expect(exact.ee().step_predecoded(instruction, error),
                    "EE native reference step failed") && ok;
    }

    const ps2::u32 retired = native.ee().run_native_block(
        pc, 0u, code.data(),
        static_cast<ps2::u32>(code.size()),
        static_cast<ps2::u32>(code.size()));

#if defined(_M_X64) || defined(__x86_64__)
    ok = expect(retired == code.size(),
                "EE native block did not retire the full block") && ok;
    const auto& a = exact.ee().state();
    const auto& b = native.ee().state();
    ok = expect(
        a.pc == b.pc &&
        a.next_pc == b.next_pc &&
        a.instructions_executed == b.instructions_executed &&
        a.cop0[9] == b.cop0[9] &&
        a.gpr[1].lo == b.gpr[1].lo &&
        a.gpr[2].lo == b.gpr[2].lo &&
        a.gpr[3].lo == b.gpr[3].lo &&
        a.gpr[4].lo == b.gpr[4].lo,
        "EE native block architectural state diverged") && ok;
#else
    ok = expect(retired == 0u,
                "EE native block unexpectedly ran on non-x64 host") && ok;
#endif
    return ok;
}

bool test_ee_native_extended_integer_block() {
    constexpr ps2::u32 pc = 0x5400u;
    const std::array<ps2::u32, 9> code = {
        (0x09u << 26) | (1u << 16) | 0x1234u, // ADDIU r1,r0,0x1234
        (1u << 21) | 0x11u,                   // MTHI r1
        (2u << 11) | 0x10u,                   // MFHI r2
        (0x09u << 26) | (3u << 16) | 0x00FFu, // ADDIU r3,r0,0xff
        (2u << 21) | (3u << 16) | (4u << 11) | 0x27u, // NOR
        (2u << 21) | (3u << 16) | (5u << 11) | 0x2Fu, // DSUBU
        (2u << 16) | (6u << 11) | (4u << 6) | 0x3Cu,  // DSLL32
        (0x2Fu << 26) | (1u << 21),            // CACHE
        (0x33u << 26) | (1u << 21),            // PREF
    };

    ps2::Ps2System exact;
    ps2::Ps2System native;
    exact.ee().reset(pc);
    native.ee().reset(pc);

    std::string error;
    bool ok = true;
    for (const ps2::u32 instruction : code) {
        ok = expect(
            exact.ee().step_predecoded(instruction, error),
            "EE extended native reference step failed") && ok;
    }

    const ps2::u32 retired = native.ee().run_native_block(
        pc, 0u, code.data(),
        static_cast<ps2::u32>(code.size()),
        static_cast<ps2::u32>(code.size()));

#if defined(_M_X64) || defined(__x86_64__)
    ok = expect(
        retired == code.size(),
        "EE extended native block did not retire fully") && ok;
    const auto& a = exact.ee().state();
    const auto& b = native.ee().state();
    ok = expect(
        a.pc == b.pc &&
        a.next_pc == b.next_pc &&
        a.hi == b.hi &&
        a.gpr[1].lo == b.gpr[1].lo &&
        a.gpr[2].lo == b.gpr[2].lo &&
        a.gpr[3].lo == b.gpr[3].lo &&
        a.gpr[4].lo == b.gpr[4].lo &&
        a.gpr[5].lo == b.gpr[5].lo &&
        a.gpr[6].lo == b.gpr[6].lo,
        "EE extended native integer state diverged") && ok;
#else
    ok = expect(
        retired == 0u,
        "EE extended native block unexpectedly ran on non-x64") && ok;
#endif
    return ok;
}

bool test_ee_native_ram_loads() {
    constexpr ps2::u32 pc = 0x5C00u;
    const std::array<ps2::u32, 7> code = {
        (0x20u << 26) | (1u << 21) | (2u << 16) | 0u, // LB
        (0x24u << 26) | (1u << 21) | (3u << 16) | 7u, // LBU
        (0x21u << 26) | (1u << 21) | (4u << 16) | 6u, // LH
        (0x25u << 26) | (1u << 21) | (5u << 16) | 6u, // LHU
        (0x23u << 26) | (1u << 21) | (6u << 16) | 4u, // LW
        (0x27u << 26) | (1u << 21) | (7u << 16) | 4u, // LWU
        (0x37u << 26) | (1u << 21) | (8u << 16) | 0u, // LD
    };
    constexpr std::array<ps2::u32, 5> aliases = {
        0x00006000u,
        0x20006000u,
        0x30006000u,
        0x80006000u,
        0xA0006000u,
    };

    bool ok = true;
    for (const ps2::u32 base : aliases) {
        ps2::Ps2System exact;
        ps2::Ps2System native;
        ok = expect(
            exact.bus().write64(0x6000u, 0xFEDCBA9876543210ull) &&
            native.bus().write64(0x6000u, 0xFEDCBA9876543210ull),
            "EE fastmem load data setup failed") && ok;
        exact.ee().reset(pc);
        native.ee().reset(pc);
        exact.ee().state().gpr[1].lo = base;
        native.ee().state().gpr[1].lo = base;

        std::string error;
        for (const ps2::u32 instruction : code) {
            ok = expect(exact.ee().step_predecoded(instruction, error),
                        "EE fastmem reference load failed") && ok;
        }

        const ps2::u32 retired = native.ee().run_native_block(
            pc, 0u, code.data(),
            static_cast<ps2::u32>(code.size()),
            static_cast<ps2::u32>(code.size()),
            native.ram().data());
#if defined(_M_X64) || defined(__x86_64__)
        ok = expect(retired == code.size(),
                    "EE fastmem block did not retire all loads") && ok;
        const auto& a = exact.ee().state();
        const auto& b = native.ee().state();
        ok = expect(
            a.pc == b.pc &&
            a.next_pc == b.next_pc &&
            a.instructions_executed == b.instructions_executed &&
            a.cop0[9] == b.cop0[9],
            "EE fastmem load control state diverged") && ok;
        for (ps2::u32 reg = 2u; reg <= 8u; ++reg) {
            ok = expect(
                a.gpr[reg].lo == b.gpr[reg].lo,
                "EE fastmem loaded register diverged") && ok;
        }
#else
        ok = expect(retired == 0u,
                    "EE fastmem unexpectedly ran on non-x64") && ok;
#endif
    }

#if defined(_M_X64) || defined(__x86_64__)
    ps2::Ps2System guarded;
    guarded.ee().reset(pc);
    guarded.ee().state().gpr[1].lo = 0x10000000u;
    const ps2::u32 retired = guarded.ee().run_native_block(
        pc, 0u, code.data(),
        static_cast<ps2::u32>(code.size()),
        static_cast<ps2::u32>(code.size()),
        guarded.ram().data());
    ok = expect(
        retired == 0u &&
        guarded.ee().state().pc == pc &&
        guarded.ee().state().instructions_executed == 0u,
        "EE fastmem guard did not reject MMIO address") && ok;
#endif
    return ok;
}

bool test_ee_native_ram_stores() {
    constexpr ps2::u32 pc = 0x6800u;
    constexpr ps2::u32 data = 0x9000u;
    const std::array<ps2::u32, 5> code = {
        (0x28u << 26) | (1u << 21) | (2u << 16) | 0u, // SB
        (0x29u << 26) | (1u << 21) | (2u << 16) | 2u, // SH
        (0x2Bu << 26) | (1u << 21) | (2u << 16) | 4u, // SW
        (0x3Fu << 26) | (1u << 21) | (2u << 16) | 8u, // SD
        (0x09u << 26) | (3u << 16) | 9u,              // ADDIU r3,r0,9
    };

    ps2::Ps2System exact;
    ps2::Ps2System native;
    exact.ee().reset(pc);
    native.ee().reset(pc);
    exact.ee().state().gpr[1].lo = data;
    native.ee().state().gpr[1].lo = data;
    exact.ee().state().gpr[2].lo = 0xFEDCBA9876543210ull;
    native.ee().state().gpr[2].lo = 0xFEDCBA9876543210ull;
    exact.ram().track_code_page(data);
    native.ram().track_code_page(data);

    std::string error;
    bool ok = true;
    for (const ps2::u32 instruction : code) {
        ok = expect(
            exact.ee().step_predecoded(instruction, error),
            "EE native store reference step failed") && ok;
    }

    const ps2::u32 retired = native.ee().run_native_block(
        pc, 0u, code.data(),
        static_cast<ps2::u32>(code.size()),
        static_cast<ps2::u32>(code.size()),
        native.ram().data(),
        native.ram().page_generation_data());

#if defined(_M_X64) || defined(__x86_64__)
    ok = expect(retired == code.size(),
                "EE native store block did not retire fully") && ok;
    for (ps2::u32 offset : {0u, 2u, 4u}) {
        ps2::u32 a = 0;
        ps2::u32 b = 0;
        const ps2::u32 width = offset == 0u ? 1u :
                               offset == 2u ? 2u : 4u;
        if (width == 1u) {
            ps2::u8 av = 0;
            ps2::u8 bv = 0;
            ok = expect(exact.ram().read8(data + offset, av) &&
                        native.ram().read8(data + offset, bv) &&
                        av == bv,
                        "EE native byte store diverged") && ok;
        } else if (width == 2u) {
            ps2::u16 av = 0;
            ps2::u16 bv = 0;
            ok = expect(exact.ram().read16(data + offset, av) &&
                        native.ram().read16(data + offset, bv) &&
                        av == bv,
                        "EE native halfword store diverged") && ok;
        } else {
            ok = expect(exact.ram().read32(data + offset, a) &&
                        native.ram().read32(data + offset, b) &&
                        a == b,
                        "EE native word store diverged") && ok;
        }
    }
    ps2::u64 exact64 = 0;
    ps2::u64 native64 = 0;
    ok = expect(exact.ram().read64(data + 8u, exact64) &&
                native.ram().read64(data + 8u, native64) &&
                exact64 == native64,
                "EE native doubleword store diverged") && ok;
    ok = expect(
        exact.ram().page_generation(data) ==
            native.ram().page_generation(data),
        "EE native store write barrier generation diverged") && ok;
    ok = expect(
        exact.ee().state().gpr[3].lo ==
            native.ee().state().gpr[3].lo,
        "EE native store block did not continue after data stores") && ok;

    ps2::Ps2System selfmod;
    selfmod.ee().reset(pc);
    selfmod.ee().state().gpr[1].lo = pc + 0x100u;
    selfmod.ee().state().gpr[2].lo = 0x12345678u;
    selfmod.ram().track_code_page(pc);
    const ps2::u32 generation =
        selfmod.ram().page_generation(pc);
    const ps2::u32 selfmod_code[2] = {
        (0x2Bu << 26) | (1u << 21) | (2u << 16),
        (0x09u << 26) | (3u << 16) | 1u,
    };
    const ps2::u32 selfmod_retired =
        selfmod.ee().run_native_block(
            pc, generation, selfmod_code, 2u, 2u,
            selfmod.ram().data(),
            selfmod.ram().page_generation_data());
    ok = expect(
        selfmod_retired == 1u &&
        selfmod.ee().state().pc == pc + 4u &&
        selfmod.ram().page_generation(pc) == generation + 1u,
        "EE self-modifying native store did not exit at barrier") && ok;

    ps2::Ps2System guarded;
    guarded.ee().reset(pc);
    guarded.ee().state().gpr[1].lo = 0x10000000u;
    guarded.ee().state().gpr[2].lo = 0x12345678u;
    const ps2::u32 guarded_retired =
        guarded.ee().run_native_block(
            pc, 0u, selfmod_code, 2u, 2u,
            guarded.ram().data(),
            guarded.ram().page_generation_data());
    ok = expect(
        guarded_retired == 0u &&
        guarded.ee().state().pc == pc,
        "EE native store guard accepted MMIO address") && ok;
#else
    ok = expect(retired == 0u,
                "EE native stores unexpectedly ran on non-x64") && ok;
#endif
    return ok;
}

bool test_ee_native_quadword_fastmem() {
    constexpr ps2::u32 pc = 0x6A00u;
    constexpr ps2::u32 source = 0xA008u;
    constexpr ps2::u32 destination = 0xB00Cu;
    const std::array<ps2::u32, 2> code = {
        (0x1Eu << 26) | (1u << 21) | (2u << 16), // LQ r2,0(r1)
        (0x1Fu << 26) | (3u << 21) | (2u << 16), // SQ r2,0(r3)
    };

    ps2::Ps2System exact;
    ps2::Ps2System native;
    constexpr ps2::u64 lo = 0x0123456789ABCDEFull;
    constexpr ps2::u64 hi = 0xFEDCBA9876543210ull;
    bool ok = expect(
        exact.bus().write64(source & ~0xFu, lo) &&
        exact.bus().write64((source & ~0xFu) + 8u, hi) &&
        native.bus().write64(source & ~0xFu, lo) &&
        native.bus().write64((source & ~0xFu) + 8u, hi),
        "EE native LQ/SQ setup failed");
    exact.ee().reset(pc);
    native.ee().reset(pc);
    exact.ee().state().gpr[1].lo = source;
    native.ee().state().gpr[1].lo = source;
    exact.ee().state().gpr[3].lo = destination;
    native.ee().state().gpr[3].lo = destination;
    exact.ram().track_code_page(destination & ~0xFu);
    native.ram().track_code_page(destination & ~0xFu);

    std::string error;
    for (const ps2::u32 instruction : code) {
        ok = expect(
            exact.ee().step_predecoded(instruction, error),
            "EE native LQ/SQ reference step failed") && ok;
    }

    const ps2::u32 retired = native.ee().run_native_block(
        pc, 0u, code.data(),
        static_cast<ps2::u32>(code.size()),
        static_cast<ps2::u32>(code.size()),
        native.ram().data(),
        native.ram().page_generation_data());

#if defined(_M_X64) || defined(__x86_64__)
    ok = expect(
        retired == code.size(),
        "EE native LQ/SQ block did not retire fully") && ok;
    ok = expect(
        exact.ee().state().gpr[2].lo ==
            native.ee().state().gpr[2].lo &&
        exact.ee().state().gpr[2].hi ==
            native.ee().state().gpr[2].hi,
        "EE native LQ register result diverged") && ok;
    ps2::u64 exact_lo = 0;
    ps2::u64 exact_hi = 0;
    ps2::u64 native_lo = 0;
    ps2::u64 native_hi = 0;
    const ps2::u32 aligned_destination =
        destination & ~0xFu;
    ok = expect(
        exact.ram().read64(aligned_destination, exact_lo) &&
        exact.ram().read64(aligned_destination + 8u, exact_hi) &&
        native.ram().read64(aligned_destination, native_lo) &&
        native.ram().read64(aligned_destination + 8u, native_hi) &&
        exact_lo == native_lo &&
        exact_hi == native_hi &&
        exact_lo == lo &&
        exact_hi == hi,
        "EE native SQ RAM result diverged") && ok;
    ok = expect(
        exact.ram().page_generation(aligned_destination) ==
            native.ram().page_generation(aligned_destination),
        "EE native SQ write barrier generation diverged") && ok;
#else
    ok = expect(
        retired == 0u,
        "EE native LQ/SQ unexpectedly ran on non-x64") && ok;
#endif
    return ok;
}

bool test_ee_native_fpu_and_sc_fastmem() {
    constexpr ps2::u32 pc = 0x6C00u;
    constexpr ps2::u32 data = 0x9800u;
    const std::array<ps2::u32, 4> code = {
        (0x31u << 26) | (1u << 21) | (4u << 16) | 0u,  // LWC1 f4,0(r1)
        (0x39u << 26) | (1u << 21) | (3u << 16) | 4u,  // SWC1 f3,4(r1)
        (0x38u << 26) | (1u << 21) | (2u << 16) | 8u,  // SC r2,8(r1)
        (0x3Cu << 26) | (1u << 21) | (5u << 16) | 16u, // SCD r5,16(r1)
    };

    ps2::Ps2System exact;
    ps2::Ps2System native;
    bool ok = expect(
        exact.bus().write32(data, 0xDEADBEEFu) &&
        native.bus().write32(data, 0xDEADBEEFu),
        "EE native FPU fastmem setup failed");
    exact.ee().reset(pc);
    native.ee().reset(pc);
    exact.ee().state().gpr[1].lo = data;
    native.ee().state().gpr[1].lo = data;
    exact.ee().state().gpr[2].lo = 0x1122334455667788ull;
    native.ee().state().gpr[2].lo = 0x1122334455667788ull;
    exact.ee().state().gpr[5].lo = 0x8877665544332211ull;
    native.ee().state().gpr[5].lo = 0x8877665544332211ull;
    exact.ee().state().fpr[3] = 0xAABBCCDDu;
    native.ee().state().fpr[3] = 0xAABBCCDDu;
    exact.ram().track_code_page(data);
    native.ram().track_code_page(data);

    std::string error;
    for (const ps2::u32 instruction : code) {
        ok = expect(
            exact.ee().step_predecoded(instruction, error),
            "EE native FPU/SC reference step failed") && ok;
    }

    const ps2::u32 retired = native.ee().run_native_block(
        pc, 0u, code.data(),
        static_cast<ps2::u32>(code.size()),
        static_cast<ps2::u32>(code.size()),
        native.ram().data(),
        native.ram().page_generation_data());

#if defined(_M_X64) || defined(__x86_64__)
    ok = expect(
        retired == code.size(),
        "EE native FPU/SC block did not retire fully") && ok;
    ok = expect(
        exact.ee().state().fpr[4] == native.ee().state().fpr[4] &&
        exact.ee().state().gpr[2].lo == native.ee().state().gpr[2].lo &&
        exact.ee().state().gpr[5].lo == native.ee().state().gpr[5].lo,
        "EE native FPU/SC register state diverged") && ok;

    ps2::u32 exact32 = 0;
    ps2::u32 native32 = 0;
    ps2::u64 exact64 = 0;
    ps2::u64 native64 = 0;
    ok = expect(
        exact.ram().read32(data + 4u, exact32) &&
        native.ram().read32(data + 4u, native32) &&
        exact32 == native32 &&
        exact32 == 0xAABBCCDDu,
        "EE native SWC1 result diverged") && ok;
    ok = expect(
        exact.ram().read32(data + 8u, exact32) &&
        native.ram().read32(data + 8u, native32) &&
        exact32 == native32,
        "EE native SC result diverged") && ok;
    ok = expect(
        exact.ram().read64(data + 16u, exact64) &&
        native.ram().read64(data + 16u, native64) &&
        exact64 == native64,
        "EE native SCD result diverged") && ok;
    ok = expect(
        exact.ram().page_generation(data) ==
            native.ram().page_generation(data),
        "EE native FPU/SC write barrier diverged") && ok;
#else
    ok = expect(
        retired == 0u,
        "EE native FPU/SC unexpectedly ran on non-x64") && ok;
#endif
    return ok;
}

bool test_ee_native_regimm() {
    constexpr ps2::u32 pc = 0x5700u;
    bool ok = true;

    auto compare_branch = [&](ps2::u32 variant,
                              ps2::u64 source,
                              const char* label) {
        const ps2::u32 code[2] = {
            (0x01u << 26) | (1u << 21) |
                (variant << 16) | 2u,
            (0x09u << 26) | (2u << 16) | 7u,
        };
        ps2::Ps2System exact;
        ps2::Ps2System native;
        exact.ee().reset(pc);
        native.ee().reset(pc);
        exact.ee().state().gpr[1].lo = source;
        native.ee().state().gpr[1].lo = source;

        std::string error;
        ok = expect(
            exact.ee().step_predecoded(code[0], error) &&
            exact.ee().step_predecoded(code[1], error),
            label) && ok;

        const ps2::u32 retired = native.ee().run_native_block(
            pc, 0u, code, 2u, 2u);
#if defined(_M_X64) || defined(__x86_64__)
        const auto& a = exact.ee().state();
        const auto& b = native.ee().state();
        ok = expect(
            retired == 2u &&
            a.pc == b.pc &&
            a.next_pc == b.next_pc &&
            a.gpr[2].lo == b.gpr[2].lo &&
            a.gpr[31].lo == b.gpr[31].lo,
            label) && ok;
#else
        ok = expect(
            retired == 0u,
            "EE native REGIMM branch ran on non-x64") && ok;
#endif
    };

    compare_branch(
        0x10u,
        0xFFFFFFFFFFFFFFFFull,
        "EE native BLTZAL state diverged");
    compare_branch(
        0x01u,
        0xFFFFFFFFFFFFFFFFull,
        "EE native BGEZ not-taken state diverged");

    {
        const ps2::u32 code[2] = {
            (0x01u << 26) | (31u << 21) |
                (0x10u << 16) | 2u,
            (0x09u << 26) | (2u << 16) | 9u,
        };
        ps2::Ps2System exact;
        ps2::Ps2System native;
        exact.ee().reset(pc);
        native.ee().reset(pc);
        exact.ee().state().gpr[31].lo =
            0xFFFFFFFFFFFFFFFFull;
        native.ee().state().gpr[31].lo =
            0xFFFFFFFFFFFFFFFFull;
        std::string error;
        ok = expect(
            exact.ee().step_predecoded(code[0], error) &&
            exact.ee().step_predecoded(code[1], error),
            "EE REGIMM link-source reference failed") && ok;
        const ps2::u32 retired =
            native.ee().run_native_block(pc, 0u, code, 2u, 2u);
#if defined(_M_X64) || defined(__x86_64__)
        ok = expect(
            retired == 2u &&
            exact.ee().state().pc == native.ee().state().pc &&
            exact.ee().state().gpr[31].lo ==
                native.ee().state().gpr[31].lo,
            "EE native REGIMM rs=r31 ordering diverged") && ok;
#else
        ok = expect(
            retired == 0u,
            "EE native REGIMM rs=r31 ran on non-x64") && ok;
#endif
    }

    const ps2::u32 sa_code[2] = {
        (0x01u << 26) | (3u << 21) | (0x18u << 16) | 0x000Bu,
        (0x01u << 26) | (3u << 21) | (0x19u << 16) | 0x0005u,
    };
    ps2::Ps2System exact_sa;
    ps2::Ps2System native_sa;
    exact_sa.ee().reset(pc);
    native_sa.ee().reset(pc);
    exact_sa.ee().state().gpr[3].lo = 0xAu;
    native_sa.ee().state().gpr[3].lo = 0xAu;

    std::string error;
    ok = expect(
        exact_sa.ee().step_predecoded(sa_code[0], error) &&
        exact_sa.ee().step_predecoded(sa_code[1], error),
        "EE REGIMM SA reference failed") && ok;
    const ps2::u32 sa_retired = native_sa.ee().run_native_block(
        pc, 0u, sa_code, 2u, 2u);
#if defined(_M_X64) || defined(__x86_64__)
    ok = expect(
        sa_retired == 2u &&
        exact_sa.ee().state().sa == native_sa.ee().state().sa &&
        exact_sa.ee().state().pc == native_sa.ee().state().pc,
        "EE native MTSAB/MTSAH state diverged") && ok;
#else
    ok = expect(
        sa_retired == 0u,
        "EE native SA helpers ran on non-x64") && ok;
#endif

    return ok;
}

bool test_ee_native_branch_delay() {
    constexpr ps2::u32 pc = 0x5800u;
    const ps2::u32 code[2] = {
        0x1000FFFFu,
        (0x09u << 26) | (1u << 16) | 7u,
    };
    ps2::Ps2System exact;
    ps2::Ps2System compiled;
    exact.ee().reset(pc);
    compiled.ee().reset(pc);
    std::string error;
    bool ok = expect(exact.ee().step_predecoded(code[0], error) &&
                     exact.ee().step_predecoded(code[1], error),
                     "EE branch reference failed");
    const ps2::u32 retired =
        compiled.ee().run_native_block(pc, 0u, code, 2u, 2u);
#if defined(_M_X64) || defined(__x86_64__)
    ok = expect(retired == 2u &&
                compiled.ee().state().pc == exact.ee().state().pc &&
                compiled.ee().state().next_pc == exact.ee().state().next_pc &&
                compiled.ee().state().gpr[1].lo == exact.ee().state().gpr[1].lo,
                "EE native branch-delay state diverged") && ok;
#else
    ok = expect(retired == 0u, "EE native branch ran on non-x64") && ok;
#endif
    return ok;
}

bool test_ee_quiet_fast_prefix() {
    constexpr ps2::u32 pc = 0x5A00u;
    const std::array<ps2::u32, 7> code = {
        (0x09u << 26) | (1u << 16) | 0x1234u, // ADDIU r1,r0,0x1234
        (0x0Du << 26) | (1u << 21) | (2u << 16) | 0x00FFu, // ORI
        (1u << 21) | 0x11u, // MTHI r1
        (3u << 11) | 0x10u, // MFHI r3
        (2u << 21) | (3u << 16) | (4u << 11) | 0x27u, // NOR
        (0x19u << 26) | (4u << 21) | (5u << 16), // DADDIU r5,r4,0
        (0x2Au << 26) | (6u << 16), // SWL r6,0(r0): unsupported fast store
    };

    ps2::Ps2System exact;
    ps2::Ps2System fast;
    exact.ee().reset(pc);
    fast.ee().reset(pc);

    std::string error;
    bool ok = true;
    constexpr ps2::u32 expected = 6u;
    for (ps2::u32 i = 0u; i < expected; ++i) {
        ok = expect(
            exact.ee().step_quiet_predecoded(code[i], error),
            "EE fast-prefix reference step failed") && ok;
    }

    const ps2::u32 retired = fast.ee().run_quiet_fast_prefix(
        pc,
        code.data(),
        static_cast<ps2::u32>(code.size()),
        static_cast<ps2::u32>(code.size()));

    const auto& a = exact.ee().state();
    const auto& b = fast.ee().state();
    ok = expect(
        retired == expected,
        "EE fast-prefix did not stop before unsupported store") && ok;
    ok = expect(
        a.pc == b.pc &&
        a.next_pc == b.next_pc &&
        a.instructions_executed == b.instructions_executed &&
        a.cop0[9] == b.cop0[9] &&
        a.hi == b.hi &&
        a.gpr[1].lo == b.gpr[1].lo &&
        a.gpr[2].lo == b.gpr[2].lo &&
        a.gpr[3].lo == b.gpr[3].lo &&
        a.gpr[4].lo == b.gpr[4].lo &&
        a.gpr[5].lo == b.gpr[5].lo,
        "EE fast-prefix architectural state diverged") && ok;

    {
        const std::array<ps2::u32, 6> nops{};
        ps2::Ps2System exact_nops;
        ps2::Ps2System fast_nops;
        exact_nops.ee().reset(pc);
        fast_nops.ee().reset(pc);
        exact_nops.ee().state().cop0[11] = 6u;
        fast_nops.ee().state().cop0[11] = 6u;
        std::string nop_error;
        for (ps2::u32 i = 0u; i < nops.size(); ++i) {
            ok = expect(
                exact_nops.ee().step_quiet_predecoded(0u, nop_error),
                "EE bulk-NOP reference step failed") && ok;
        }
        const ps2::u32 nop_retired =
            fast_nops.ee().run_quiet_fast_prefix(
                pc,
                nops.data(),
                static_cast<ps2::u32>(nops.size()),
                static_cast<ps2::u32>(nops.size()));
        const auto& na = exact_nops.ee().state();
        const auto& nb = fast_nops.ee().state();
        ok = expect(
            nop_retired == nops.size() &&
            na.pc == nb.pc &&
            na.next_pc == nb.next_pc &&
            na.instructions_executed == nb.instructions_executed &&
            na.cop0[9] == nb.cop0[9] &&
            na.cop0[13] == nb.cop0[13],
            "EE bulk-NOP retirement diverged") && ok;
    }
    return ok;
}

bool test_ee_quiet_fast_ram_store_barrier() {
    constexpr ps2::u32 pc = 0x5A40u;
    const std::array<ps2::u32, 3> code = {
        (0x09u << 26) | (1u << 16) | 0x5A40u, // ADDIU r1,r0,pc
        (0x2Bu << 26) | (1u << 21) | (2u << 16), // SW r2,0(r1)
        (0x09u << 26) | (3u << 16) | 9u, // must not execute in same prefix
    };

    ps2::Ps2System system;
    system.ee().reset(pc);
    system.ee().state().gpr[2].lo = 0x12345678u;
    system.ram().track_code_page(pc);
    const ps2::u32 generation =
        system.ram().page_generation(pc);

    bool store_executed = false;
    const ps2::u32 retired = system.ee().run_quiet_fast_prefix(
        pc,
        code.data(),
        static_cast<ps2::u32>(code.size()),
        static_cast<ps2::u32>(code.size()),
        &store_executed);

    ps2::u32 stored = 0u;
    bool ok = expect(
        system.ram().read32(pc, stored),
        "EE tight store result could not be read");
    ok = expect(
        retired == 2u &&
        store_executed &&
        stored == 0x12345678u &&
        system.ee().state().pc == pc + 8u &&
        system.ee().state().gpr[3].lo == 0u &&
        system.ram().page_generation(pc) == generation + 1u,
        "EE tight interpreter store barrier diverged") && ok;
    return ok;
}

bool test_ee_quiet_fast_ram_loads() {
    constexpr ps2::u32 pc = 0x5A80u;
    constexpr ps2::u32 data = 0x6400u;
    const std::array<ps2::u32, 6> code = {
        (0x23u << 26) | (1u << 21) | (2u << 16),      // LW
        (0x27u << 26) | (1u << 21) | (3u << 16),      // LWU
        (0x37u << 26) | (1u << 21) | (4u << 16),      // LD
        (0x31u << 26) | (1u << 21) | (5u << 16) | 4u, // LWC1
        (0x09u << 26) | (6u << 16) | 7u,              // ADDIU
        (0x2Au << 26) | (1u << 21) | (6u << 16) | 8u, // SWL: fallback
    };

    ps2::Ps2System exact;
    ps2::Ps2System fast;
    bool ok = expect(
        exact.bus().write64(data, 0xFEDCBA9876543210ull) &&
        fast.bus().write64(data, 0xFEDCBA9876543210ull),
        "EE tight RAM-load setup failed");
    exact.ee().reset(pc);
    fast.ee().reset(pc);
    exact.ee().state().gpr[1].lo = data;
    fast.ee().state().gpr[1].lo = data;

    std::string error;
    constexpr ps2::u32 expected = 5u;
    for (ps2::u32 i = 0u; i < expected; ++i) {
        ok = expect(
            exact.ee().step_quiet_predecoded(code[i], error),
            "EE tight RAM-load reference step failed") && ok;
    }

    const ps2::u32 retired = fast.ee().run_quiet_fast_prefix(
        pc,
        code.data(),
        static_cast<ps2::u32>(code.size()),
        static_cast<ps2::u32>(code.size()));

    const auto& a = exact.ee().state();
    const auto& b = fast.ee().state();
    ok = expect(
        retired == expected &&
        a.pc == b.pc &&
        a.next_pc == b.next_pc &&
        a.instructions_executed == b.instructions_executed &&
        a.cop0[9] == b.cop0[9] &&
        a.gpr[2].lo == b.gpr[2].lo &&
        a.gpr[3].lo == b.gpr[3].lo &&
        a.gpr[4].lo == b.gpr[4].lo &&
        a.fpr[5] == b.fpr[5] &&
        a.gpr[6].lo == b.gpr[6].lo,
        "EE tight interpreter RAM-load state diverged") && ok;
    return ok;
}

bool test_ee_quiet_fast_branch_block() {
    constexpr ps2::u32 pc = 0x5B00u;
    bool ok = true;

    {
        const std::array<ps2::u32, 3> code = {
            (0x09u << 26) | (1u << 16) | 1u, // ADDIU r1,r0,1
            (0x04u << 26) | (1u << 21) | (1u << 16) | 2u, // BEQ taken
            (0x09u << 26) | (2u << 16) | 7u, // delay slot
        };
        ps2::Ps2System exact;
        ps2::Ps2System fast;
        exact.ee().reset(pc);
        fast.ee().reset(pc);
        std::string error;
        for (const ps2::u32 instruction : code) {
            ok = expect(
                exact.ee().step_quiet_predecoded(instruction, error),
                "EE fast branch reference step failed") && ok;
        }
        const ps2::u32 retired = fast.ee().run_quiet_fast_prefix(
            pc,
            code.data(),
            static_cast<ps2::u32>(code.size()),
            static_cast<ps2::u32>(code.size()));
        const auto& a = exact.ee().state();
        const auto& b = fast.ee().state();
        ok = expect(
            retired == code.size() &&
            a.pc == b.pc &&
            a.next_pc == b.next_pc &&
            a.instructions_executed == b.instructions_executed &&
            a.cop0[9] == b.cop0[9] &&
            a.gpr[1].lo == b.gpr[1].lo &&
            a.gpr[2].lo == b.gpr[2].lo,
            "EE tight interpreter taken branch diverged") && ok;
    }

    {
        const std::array<ps2::u32, 2> code = {
            (0x14u << 26) | (1u << 21) | (2u << 16) | 2u, // BEQL not taken
            (0x09u << 26) | (3u << 16) | 9u, // annulled delay slot
        };
        ps2::Ps2System exact;
        ps2::Ps2System fast;
        exact.ee().reset(pc);
        fast.ee().reset(pc);
        exact.ee().state().gpr[1].lo = 1u;
        exact.ee().state().gpr[2].lo = 2u;
        fast.ee().state().gpr[1].lo = 1u;
        fast.ee().state().gpr[2].lo = 2u;
        std::string error;
        ok = expect(
            exact.ee().step_quiet_predecoded(code[0], error),
            "EE fast likely-branch reference step failed") && ok;
        const ps2::u32 retired = fast.ee().run_quiet_fast_prefix(
            pc,
            code.data(),
            static_cast<ps2::u32>(code.size()),
            static_cast<ps2::u32>(code.size()));
        const auto& a = exact.ee().state();
        const auto& b = fast.ee().state();
        ok = expect(
            retired == 1u &&
            a.pc == b.pc &&
            a.next_pc == b.next_pc &&
            a.instructions_executed == b.instructions_executed &&
            a.cop0[9] == b.cop0[9] &&
            b.gpr[3].lo == 0u,
            "EE tight interpreter likely branch annul diverged") && ok;
    }

    return ok;
}

bool test_ee_quiet_step_matches_exact_execution() {
    ps2::Ps2System exact;
    ps2::Ps2System quiet;
    constexpr ps2::u32 pc = 0x3000u;
    const ps2::u32 code[] = {
        (0x09u << 26) | (1u << 16) | 0x4000u,                 // addiu r1,r0,0x4000
        (0x09u << 26) | (2u << 16) | 0x1234u,                 // addiu r2,r0,0x1234
        (0x2Bu << 26) | (1u << 21) | (2u << 16),              // sw r2,0(r1)
        (0x23u << 26) | (1u << 21) | (3u << 16),              // lw r3,0(r1)
    };

    bool ok = true;
    for (ps2::u32 i = 0; i < 4u; ++i) {
        ok = expect(exact.bus().write32(pc + i * 4u, code[i]) &&
                    quiet.bus().write32(pc + i * 4u, code[i]),
                    "EE quiet-step code setup failed") && ok;
    }
    exact.ee().reset(pc);
    quiet.ee().reset(pc);
    exact.ee().state().cop0[11] = 4u;
    quiet.ee().state().cop0[11] = 4u;

    std::string exact_error;
    std::string quiet_error;
    for (ps2::u32 i = 0; i < 4u; ++i) {
        ok = expect(exact.ee().step(exact_error),
                    "EE exact reference step failed") && ok;
        ok = expect(quiet.ee().step_quiet_predecoded(
                        code[i], quiet_error),
                    "EE predecoded quiet step failed") && ok;
    }
    quiet.bus().tick(4u);

    const auto& a = exact.ee().state();
    const auto& b = quiet.ee().state();
    ok = expect(a.pc == b.pc && a.next_pc == b.next_pc &&
                    a.instructions_executed == b.instructions_executed &&
                    a.cop0[9] == b.cop0[9] &&
                    a.cop0[13] == b.cop0[13] &&
                    a.gpr[1].lo == b.gpr[1].lo &&
                    a.gpr[2].lo == b.gpr[2].lo &&
                    a.gpr[3].lo == b.gpr[3].lo,
                "EE quiet-step architectural state diverged") && ok;

    ps2::u32 exact_word = 0;
    ps2::u32 quiet_word = 0;
    ok = expect(exact.bus().read32(0x4000u, exact_word) &&
                    quiet.bus().read32(0x4000u, quiet_word) &&
                    exact_word == quiet_word &&
                    exact_word == 0x1234u,
                "EE quiet-step RAM result diverged") && ok;
    return ok;
}

bool test_ee_ram_page_generation() {
    ps2::Ps2System system;
    constexpr ps2::u32 page0 = 0x1000u;
    constexpr ps2::u32 page1 = 0x2000u;

    system.ram().track_code_page(page0);
    system.ram().track_code_page(page1);
    const ps2::u32 g0 = system.ram().page_generation(page0);
    const ps2::u32 g1 = system.ram().page_generation(page1);

    bool ok = expect(
        system.bus().write32(page0 + 0x20u, 0x12345678u),
        "EE RAM generation write setup failed");
    ok = expect(
        system.ram().page_generation(page0) == g0 + 1u &&
        system.ram().page_generation(page1) == g1,
        "EE RAM page generation changed the wrong page") && ok;

    const ps2::u32 g0_cross = system.ram().page_generation(page0);
    const ps2::u32 g1_cross = system.ram().page_generation(page1);
    ok = expect(
        system.bus().write64(page1 - 4u, 0x1122334455667788ull),
        "EE RAM cross-page generation write failed") && ok;
    ok = expect(
        system.ram().page_generation(page0) == g0_cross + 1u &&
        system.ram().page_generation(page1) == g1_cross + 1u,
        "EE RAM cross-page write did not invalidate both pages") && ok;
    return ok;
}

bool test_iop_osdsys_idle_detection() {
    ps2::Ps2System system;
    constexpr ps2::u32 branch_pc = 0x0000AE94u;
    constexpr ps2::u32 delay_pc = 0x0000AE98u;
    constexpr ps2::u32 idle_branch = 0x08002BA5u;

    bool ok = expect(
        system.iop_bus().write32(branch_pc, idle_branch) &&
        system.iop_bus().write32(delay_pc, 0u),
        "IOP idle-loop test setup failed");
    system.iop().reset(branch_pc);

    ok = expect(system.iop().in_osdsys_idle_loop(),
                "IOP idle loop was not recognized at branch") && ok;

    std::string error;
    ok = expect(system.iop().step(error),
                "IOP idle branch step failed") && ok;
    ok = expect(system.iop().in_osdsys_idle_loop(),
                "IOP idle loop was not recognized in delay slot") && ok;

    error.clear();
    ok = expect(system.iop().step(error),
                "IOP idle delay-slot step failed") && ok;
    ok = expect(system.iop().in_osdsys_idle_loop(),
                "IOP idle loop was not recognized after one pair") && ok;

    ok = expect(system.iop_bus().write32(delay_pc, 1u),
                "IOP idle-loop mutation failed") && ok;
    ok = expect(!system.iop().in_osdsys_idle_loop(),
                "IOP idle detector accepted mutated code") && ok;
    return ok;
}

bool test_iop_halt_is_nonfatal_to_ee_bootstrap() {
    ps2::Ps2System system;
    // COP1 is not part of the PS1-derived IOP/R3000A ISA and deliberately
    // forces the IOP interpreter into its diagnostic halt state.
    constexpr ps2::u32 unsupported_iop = 0x44000000u;
    bool ok = expect(
        system.iop_bus().write32(0x1000u, unsupported_iop),
        "IOP nonfatal-halt test setup failed");
    system.iop().reset(0x1000u);

    std::string error;
    ok = expect(!system.iop().step(error) && system.iop_halted(),
                "IOP did not enter diagnostic halt state") && ok;
    ok = expect(!system.halted(),
                "IOP diagnostic halt incorrectly stopped EE bootstrap") && ok;
    return ok;
}

bool test_fpu_accumulator() {
    ps2::Ps2System system;
    constexpr ps2::u32 pc = 0x2000;
    const ps2::u32 adda = (0x11u << 26) | (0x10u << 21) | (1u << 16) | 0x18u;
    system.bus().write32(pc, adda);
    system.ee().reset(pc);
    system.ee().state().fpr[0] = std::bit_cast<ps2::u32>(1.5f);
    system.ee().state().fpr[1] = std::bit_cast<ps2::u32>(2.25f);
    std::string error;
    bool ok = expect(system.ee().step(error), "ADDA.S failed");
    ok = expect(std::bit_cast<float>(system.ee().state().fpu_acc) == 3.75f,
                "ADDA.S accumulator mismatch") && ok;
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok = test_system_stack_footprint() && ok;
    ok = test_mmi_por_128() && ok;
    ok = test_mmi_padduw() && ok;
    ok = test_mmi_madd_and_plzcw() && ok;
    ok = test_mmi_pmfhl_pmthl() && ok;
    ok = test_mmi_packed_accumulator_moves() && ok;
    ok = test_mmi_bootstrap_packed_ops() && ok;
    ok = test_mmi_bios_instruction_expansion() && ok;
    ok = test_cop2_bios_macro_expansion() && ok;
    ok = test_unaligned_doubleword_merges() && ok;
    ok = test_unaligned_word_and_atomic_memory_ops() && ok;
    ok = test_bootstrap_mmio() && ok;
    ok = test_ee_timer_events() && ok;
    ok = test_ee_timer_bulk_tick_matches_scalar() && ok;
    ok = test_ee_timer_irq_distance() && ok;
    ok = test_ee_intc_register_semantics() && ok;
    ok = test_vu_mapping_and_cop2() && ok;
    ok = test_ee_intc_cpu_exception() && ok;
    ok = test_ee_cop0_count_compare_irq() && ok;
    ok = test_ee_bc0_dmac_condition_branches() && ok;
    ok = test_ee_di_ei_privilege_gate() && ok;
    ok = test_ee_break_exception_and_tlb_ops() && ok;
    ok = test_ee_trap_instructions() && ok;
    ok = test_ee_sa_and_qfsrv() && ok;
    ok = test_ee_tlb_mapped_memory_and_refill() && ok;
    ok = test_ee_integer_overflow_exception() && ok;
    ok = test_syscall_exception() && ok;
    ok = test_syscall_delay_slot_exception() && ok;
    ok = test_video_timing_vblank_irqs() && ok;
    ok = test_gif_packet_decode() && ok;
    ok = test_gs_fog_dither_scanmask_and_context2() && ok;
    ok = test_gif_dma_engine() && ok;
    ok = test_gs_vram_swizzle_addresses() && ok;
    ok = test_gs_host_to_local_image_transfer() && ok;
    ok = test_gs_untextured_rasterization() && ok;
    ok = test_gs_async_raster_ordering() && ok;
    ok = test_gs_display_extraction() && ok;
    ok = test_gs_fst_direct_color_texturing() && ok;
    ok = test_gs_depth_layout_and_pixel_pipeline() && ok;
    ok = test_gs_stq_perspective_texturing() && ok;
    ok = test_gs_indexed_textures_and_texa() && ok;
    ok = test_gs_local_copy_and_depth_transfer() && ok;
    ok = test_gs_signal_finish_label_and_imr() && ok;
    ok = test_gs_local_to_host_transfer() && ok;
    ok = test_vif1_reverse_dma() && ok;
    ok = test_ee_native_linear_block() && ok;
    ok = test_ee_native_extended_integer_block() && ok;
    ok = test_ee_native_ram_loads() && ok;
    ok = test_ee_native_ram_stores() && ok;
    ok = test_ee_native_quadword_fastmem() && ok;
    ok = test_ee_native_fpu_and_sc_fastmem() && ok;
    ok = test_ee_native_regimm() && ok;
    ok = test_ee_native_branch_delay() && ok;
    ok = test_ee_quiet_fast_prefix() && ok;
    ok = test_ee_quiet_fast_ram_store_barrier() && ok;
    ok = test_ee_quiet_fast_ram_loads() && ok;
    ok = test_ee_quiet_fast_branch_block() && ok;
    ok = test_ee_quiet_step_matches_exact_execution() && ok;
    ok = test_ee_ram_page_generation() && ok;
    ok = test_iop_osdsys_idle_detection() && ok;
    ok = test_iop_halt_is_nonfatal_to_ee_bootstrap() && ok;
    ok = test_fpu_accumulator() && ok;
    if (!ok) return EXIT_FAILURE;
    std::cout << "VibeStation PS2 bootstrap tests passed.\n";
    return EXIT_SUCCESS;
}
