#include "core/ps2_system.h"
#include "core/gs/gs_rasterizer.h"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <vector>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool test_ram_little_endian() {
    ps2::Ps2System system;

    constexpr ps2::u32 address = 0x00001000u;
    if (!expect(system.bus().write32(address, 0x44332211u), "write32 failed")) {
        return false;
    }

    ps2::u8 byte = 0;
    return expect(system.bus().read8(address + 0, byte) && byte == 0x11u,
                  "byte 0 mismatch") &&
           expect(system.bus().read8(address + 1, byte) && byte == 0x22u,
                  "byte 1 mismatch") &&
           expect(system.bus().read8(address + 2, byte) && byte == 0x33u,
                  "byte 2 mismatch") &&
           expect(system.bus().read8(address + 3, byte) && byte == 0x44u,
                  "byte 3 mismatch");
}

bool test_ram_aliases() {
    ps2::Ps2System system;

    constexpr ps2::u32 physical = 0x00123450u;
    constexpr ps2::u32 uncached = 0x20123450u;
    constexpr ps2::u32 accelerated = 0x30123450u;
    constexpr ps2::u32 kseg0 = 0x80123450u;
    constexpr ps2::u32 kseg1 = 0xA0123450u;
    constexpr ps2::u32 value = 0xCAFEBABEu;

    if (!expect(system.bus().write32(kseg0, value), "KSEG0 write failed")) {
        return false;
    }

    ps2::u32 readback = 0;
    return expect(system.bus().read32(physical, readback) && readback == value,
                  "physical alias mismatch") &&
           expect(system.bus().fetch32(kseg0, readback) && readback == value,
                  "instruction fetch RAM alias mismatch") &&
           expect(system.bus().read32(uncached, readback) && readback == value,
                  "uncached RAM alias mismatch") &&
           expect(system.bus().read32(accelerated, readback) && readback == value,
                  "accelerated RAM alias mismatch") &&
           expect(system.bus().read32(kseg1, readback) && readback == value,
                  "KSEG1 alias mismatch");
}

bool test_ram_bounds() {
    ps2::Ps2System system;

    ps2::u32 value = 0;
    return expect(
               !system.bus().read32(
                   static_cast<ps2::u32>(ps2::EeRam::kSize), value),
               "out-of-range read unexpectedly succeeded") &&
           expect(
               !system.bus().write32(
                   static_cast<ps2::u32>(ps2::EeRam::kSize - 2),
                   0xFFFFFFFFu),
               "cross-boundary write unexpectedly succeeded");
}

bool test_bios_idle_iteration_matches_ee_steps() {
    constexpr ps2::u32 kIdlePc = 0x00081FC0u;
    constexpr std::array<ps2::u32, 8> kInstructions = {
        0u, 0u, 0u, 0u, 0u, 0u, 0x1000FFF9u, 0u};
    ps2::Ps2System fast;
    ps2::Ps2System reference;
    for (ps2::u32 i = 0; i < kInstructions.size(); ++i) {
        if (!expect(fast.bus().write32(kIdlePc + i * 4u, kInstructions[i]) &&
                    reference.bus().write32(kIdlePc + i * 4u, kInstructions[i]),
                    "idle loop setup failed")) return false;
    }
    fast.ee().reset(kIdlePc);
    reference.ee().reset(kIdlePc);
    fast.ee().state().cop0[11] = 100u;
    reference.ee().state().cop0[11] = 100u;
    if (!expect(fast.bus().write32(0x10000010u, 0x80u) &&
                reference.bus().write32(0x10000010u, 0x80u),
                "idle timer setup failed")) return false;

    if (!expect(fast.ee().skip_bios_idle_iteration(),
                "verified BIOS idle loop was not skipped")) return false;
    std::string error;
    for (int i = 0; i < 8; ++i) {
        if (!expect(reference.ee().step(error),
                    "reference BIOS idle instruction failed")) return false;
    }
    const auto& skipped = fast.ee().state();
    const auto& stepped = reference.ee().state();
    bool ok = expect(skipped.pc == stepped.pc &&
                     skipped.next_pc == stepped.next_pc &&
                     skipped.last_pc == stepped.last_pc &&
                     skipped.last_instruction == stepped.last_instruction &&
                     skipped.instructions_executed == stepped.instructions_executed &&
                     skipped.cop0[9] == stepped.cop0[9],
                     "skipped BIOS idle state diverged from eight EE steps");
    ps2::u32 fast_count = 0, reference_count = 0;
    ok = expect(fast.bus().read32(0x10000000u, fast_count) &&
                reference.bus().read32(0x10000000u, reference_count) &&
                fast_count == reference_count,
                "skipped BIOS idle timer diverged") && ok;
    ok = expect(fast.ee().step(error) && reference.ee().step(error) &&
                fast.ee().state().pc == reference.ee().state().pc,
                "EE branch-delay state diverged after idle skip") && ok;

    fast.ee().reset(kIdlePc);
    reference.ee().reset(kIdlePc);
    fast.ee().state().cop0[11] = 100u;
    reference.ee().state().cop0[11] = 100u;
    ok = expect(fast.ee().skip_bios_idle_iterations(3u),
                "verified BIOS idle batch was not skipped") && ok;
    for (int i = 0; i < 24; ++i) {
        if (!expect(reference.ee().step(error),
                    "reference BIOS idle batch instruction failed")) return false;
    }
    ok = expect(fast.ee().state().pc == reference.ee().state().pc &&
                fast.ee().state().next_pc == reference.ee().state().next_pc &&
                fast.ee().state().last_pc == reference.ee().state().last_pc &&
                fast.ee().state().instructions_executed ==
                    reference.ee().state().instructions_executed &&
                fast.ee().state().cop0[9] == reference.ee().state().cop0[9],
                "batched BIOS idle state diverged") && ok;
    ok = expect(fast.bus().read32(0x10000000u, fast_count) &&
                reference.bus().read32(0x10000000u, reference_count) &&
                fast_count == reference_count,
                "batched BIOS idle timer diverged") && ok;

    fast.ee().reset(kIdlePc);
    fast.ee().state().cop0[11] = 4u;
    ok = expect(!fast.ee().skip_bios_idle_iteration(),
                "idle skip crossed COP0 Compare event") && ok;
    fast.ee().state().cop0[11] = 100u;
    ok = expect(fast.bus().write32(kIdlePc + 4u, 0x24020001u) &&
                !fast.ee().skip_bios_idle_iteration(),
                "idle skip accepted changed code") && ok;
    return ok;
}

bool test_bios_loop_fast_paths_match_ee_steps() {
    auto compare = [](
        const char* label, ps2::u32 pc,
        std::span<const ps2::u32> code,
        auto setup, auto skip, ps2::u32 retired,
        ps2::u32 memory, ps2::u32 bytes) {
        ps2::Ps2System fast;
        ps2::Ps2System reference;
        for (ps2::u32 i = 0; i < code.size(); ++i) {
            if (!expect(fast.bus().write32(pc + 4u * i, code[i]) &&
                        reference.bus().write32(pc + 4u * i, code[i]),
                        "BIOS loop code setup failed")) return false;
        }
        fast.ee().reset(pc);
        reference.ee().reset(pc);
        fast.ee().state().cop0[11] = 0x10000000u;
        reference.ee().state().cop0[11] = 0x10000000u;
        setup(fast);
        setup(reference);
        if (!expect(skip(fast), label)) return false;
        std::string error;
        for (ps2::u32 i = 0; i < retired; ++i) {
            if (!expect(reference.ee().step(error),
                        "BIOS loop reference step failed")) return false;
        }
        auto equal_state = [&]() {
            const auto& a = fast.ee().state();
            const auto& b = reference.ee().state();
            if (a.pc != b.pc || a.next_pc != b.next_pc ||
                a.last_pc != b.last_pc ||
                a.last_instruction != b.last_instruction ||
                a.instructions_executed != b.instructions_executed ||
                a.cop0[9] != b.cop0[9] || a.cop0[13] != b.cop0[13])
                return false;
            for (ps2::u32 i = 0; i < a.gpr.size(); ++i) {
                if (a.gpr[i].lo != b.gpr[i].lo ||
                    a.gpr[i].hi != b.gpr[i].hi) return false;
            }
            return true;
        };
        if (!expect(equal_state(), "BIOS loop CPU state diverged"))
            return false;
        for (ps2::u32 i = 0; i < bytes; ++i) {
            ps2::u8 a = 0, b = 0;
            if (!expect(fast.bus().read8(memory + i, a) &&
                        reference.bus().read8(memory + i, b) && a == b,
                        "BIOS loop RAM diverged")) return false;
        }
        if (!expect(fast.ee().step(error) && reference.ee().step(error) &&
                    equal_state(), "BIOS loop follow-up step diverged"))
            return false;
        return true;
    };

    constexpr std::array<ps2::u32, 7> zero = {
        0x7E020000u, 0x26100010u, 0x0204102Bu, 0u, 0u,
        0x1440FFFAu, 0x700014A9u};
    constexpr std::array<ps2::u32, 9> nibble = {
        0x90A20000u, 0x24C6FFFFu, 0x3043000Fu,
        0x00021102u, 0x00031900u, 0x00431021u,
        0xA0A20000u, 0x04C1FFF8u, 0x24A50001u};
    constexpr std::array<ps2::u32, 7> countdown = {
        0u, 0u, 0u, 0u, 0u, 0x1443FFFAu, 0x2442FFFFu};
    constexpr std::array<ps2::u32, 7> copy = {
        0x90A20000u, 0x2484FFFFu, 0x24A50001u,
        0xA2020000u, 0x26100001u, 0x1480FFFAu, 0u};
    constexpr std::array<ps2::u32, 7> poll = {
        0x8C620000u, 0x00441024u, 0u, 0u, 0u,
        0x1040FFFAu, 0x3C021000u};
    bool ok = compare("zero-loop shortcut rejected valid code",
        0x8000E3C8u, zero,
        [](auto& s) {
            s.ee().state().gpr[2] = {};
            s.ee().state().gpr[16].lo = 0x1000u;
            s.ee().state().gpr[4].lo = 0x2000u;
        },
        [](auto& s) { return s.ee().skip_bios_zero_loop(3u); },
        21u, 0x1000u, 48u);
    ok = compare("nibble-loop shortcut rejected valid code",
        0x0020A0E8u, nibble,
        [](auto& s) {
            s.ee().state().gpr[5].lo = 0x1B00100u;
            s.ee().state().gpr[6].lo = 5u;
            (void)s.bus().write32(0x1B00100u, 0x78563412u);
        },
        [](auto& s) { return s.ee().skip_bios_nibble_loop(3u); },
        27u, 0x1B00100u, 4u) && ok;
    ok = compare("countdown shortcut rejected valid code",
        0x000826B0u, countdown,
        [](auto& s) {
            s.ee().state().gpr[2].lo = 5u;
            s.ee().state().gpr[3].lo = 0xFFFFFFFFFFFFFFFFull;
        },
        [](auto& s) { return s.ee().skip_bios_countdown_wait(3u) == 3u; },
        21u, 0u, 0u) && ok;
    ok = compare("copy shortcut rejected valid code",
        0x00200DE8u, copy,
        [](auto& s) {
            s.ee().state().gpr[4].lo = 1u;
            s.ee().state().gpr[5].lo = 0x1200u;
            s.ee().state().gpr[16].lo = 0x1300u;
            (void)s.bus().write8(0x1200u, 0xA5u);
        },
        [](auto& s) { return s.ee().skip_bios_copy_iteration(); },
        7u, 0x1300u, 1u) && ok;
    ok = compare("MMIO poll shortcut rejected valid code",
        0x00082180u, poll,
        [](auto& s) {
            s.ee().state().gpr[3].lo = 0x1000F230u;
            s.ee().state().gpr[4].lo = 0x00040000u;
        },
        [](auto& s) { return s.ee().skip_bios_mmio_poll_iteration(); },
        7u, 0u, 0u) && ok;
    constexpr std::array<ps2::u32, 7> bios_poll = {
        0x8C620000u, 0x30420004u, 0u, 0u, 0u,
        0x1040FFFAu, 0x24020004u};
    ok = compare("BIOS MMIO poll shortcut rejected valid code",
        0x00266118u, bios_poll,
        [](auto& s) {
            s.ee().state().gpr[3].lo = 0x1000F000u;
        },
        [](auto& s) { return s.ee().skip_bios_mmio_poll_iteration(); },
        7u, 0u, 0u) && ok;
    std::array<ps2::u32, (0x00200E40u - 0x00200D70u) / 4u> literal{};
    auto put_literal = [&](ps2::u32 address, ps2::u32 instruction) {
        literal[(address - 0x00200D70u) / 4u] = instruction;
    };
    put_literal(0x00200D70u, 0x16200004u);
    put_literal(0x00200D74u, 0x268781C8u);
    constexpr std::array<ps2::u32, 7> literal_input = {
        0x8CE50014u, 0x8CE20004u, 0x90A60000u,
        0x24A50001u, 0x00551024u, 0x1040001Cu,
        0xACE50014u};
    for (ps2::u32 i = 0; i < literal_input.size(); ++i)
        put_literal(0x00200D84u + i * 4u, literal_input[i]);
    constexpr std::array<ps2::u32, 13> literal_output = {
        0xA2060000u, 0x26100001u, 0x8E6381C8u,
        0x02121023u, 0x10430008u, 0x266481C8u,
        0x0062102Bu, 0x14400005u, 0x2631FFFFu,
        0x8C820004u, 0x00021040u, 0x1000FFCDu,
        0xAC820004u};
    for (ps2::u32 i = 0; i < literal_output.size(); ++i)
        put_literal(0x00200E0Cu + i * 4u, literal_output[i]);
    ok = compare("literal decoder shortcut rejected valid code",
        0x00200D70u, literal,
        [](auto& s) {
            auto& gpr = s.ee().state().gpr;
            gpr[16].lo = 0x5000u;
            gpr[17].lo = 3u;
            gpr[18].lo = 0x5000u;
            gpr[19].lo = 0x3000u + 32312u;
            gpr[20].lo = 0x2000u + 32312u;
            gpr[21].lo = 1u;
            (void)s.bus().write32(0x2004u, 0u);
            (void)s.bus().write32(0x2014u, 0x4000u);
            (void)s.bus().write8(0x4000u, 0xA5u);
            (void)s.bus().write32(0x3000u, 100u);
            (void)s.bus().write32(0x3004u, 1u);
        },
        [](auto& s) { return s.ee().skip_bios_literal_iteration(); },
        22u, 0x2000u, 0x3010u) && ok;
    return ok;
}

bool test_scanout_skips_unchanged_vram() {
    ps2::GsPrivileged regs;
    ps2::GsVram vram;
    ps2::GsDisplay display;
    regs.reset();
    vram.reset();
    display.reset();

    constexpr ps2::u32 kPmode = 0x12000000u;
    constexpr ps2::u32 kDispfb1 = 0x12000070u;
    constexpr ps2::u32 kDisplay1 = 0x12000080u;
    if (!expect(regs.write64(kPmode, 1u), "PMODE setup failed") ||
        !expect(regs.write64(kDispfb1, 1u << 9), "DISPFB setup failed") ||
        !expect(regs.write64(kDisplay1, (1ull << 32) | (1ull << 44)),
                "DISPLAY setup failed")) {
        return false;
    }

    display.update(regs, vram);
    const ps2::u64 first = display.generation();
    if (!expect(display.valid(), "scanout did not become valid")) return false;
    display.update(regs, vram);
    if (!expect(display.generation() == first,
                "unchanged scanout was recomposed")) return false;

    if (!expect(vram.write_pixel(0, 0, 0, 0, 1, 0xFF123456u),
                "VRAM setup failed")) return false;
    display.update(regs, vram);
    if (!expect(display.generation() == first + 1 &&
                display.has_visible_pixels(),
                "VRAM write did not invalidate scanout")) return false;

    const ps2::u64 second = display.generation();
    if (!expect(regs.write64(kDisplay1, (2ull << 32) | (1ull << 44)),
                "DISPLAY change failed")) return false;
    display.update(regs, vram);
    return expect(display.generation() == second + 1 && display.width() == 3,
                  "display register write did not invalidate scanout");
}

bool test_gs_sprite_blend_reuses_destination() {
    ps2::GsVram vram;
    ps2::GsRasterContext ctx{};
    ctx.fbw = 1;
    ctx.scax1 = 3;
    ctx.scay1 = 3;
    ctx.alpha_blend = true;
    ctx.alpha_a = 0; // Source
    ctx.alpha_b = 1; // Destination
    ctx.alpha_c = 2; // FIX
    ctx.alpha_d = 1; // Destination
    ctx.alpha_fix = 64;
    ctx.fbmask = 0x00FF0000u; // Preserve destination blue.

    ps2::GsRasterVertex a{};
    ps2::GsRasterVertex b{};
    b.x = 32;
    b.y = 32;
    b.rgba = 0x80406080u;

    for (ps2::u32 psm : {0u, 1u}) {
        vram.reset();
        ctx.psm = psm;
        if (!expect(vram.write_pixel(psm, 0, 0, 0, 1, 0x80102030u),
                    "GS destination setup failed")) return false;
        if (!expect(ps2::GsRasterizer::draw_sprite(vram, ctx, a, b) == 4,
                    "GS sprite pixel count mismatch")) return false;
        const ps2::u32 expected = 0x00104058u;
        if (!expect(vram.read_pixel(psm, 0, 0, 0, 1) ==
                    (psm == 0u ? expected | 0x80000000u : expected),
                    "GS blended framebuffer mismatch")) return false;
    }
    return true;
}

bool test_gs_untextured_triangle_without_depth() {
    ps2::GsVram vram;
    ps2::GsRasterContext ctx{};
    ctx.fbw = 1;
    ctx.scax1 = 3;
    ctx.scay1 = 3;
    ps2::GsRasterVertex a{};
    ps2::GsRasterVertex b{};
    ps2::GsRasterVertex c{};
    b.x = 64;
    c.y = 64;
    c.rgba = 0x80406080u;
    a.z = 0xFFFFFFFFu;
    b.z = 0xFFFFFFFFu;
    c.z = 0xFFFFFFFFu;
    return expect(ps2::GsRasterizer::draw_triangle(vram, ctx, a, b, c) != 0,
                  "untextured triangle drew no pixels") &&
           expect(vram.read_pixel(0, 0, 0, 0, 1) == c.rgba,
                  "untextured triangle color mismatch");
}

bool test_dmac_running_mask() {
    ps2::EeHw hw;
    hw.reset();
    bool ok = expect(!hw.dmac_enabled() && hw.dmac_running_mask() == 0,
                     "DMAC should reset idle");
    constexpr std::array<ps2::u32, 10> chcr_addresses = {
        0x10008000u, 0x10009000u, 0x1000A000u, 0x1000B000u,
        0x1000B400u, 0x1000C000u, 0x1000C400u, 0x1000C800u,
        0x1000D000u, 0x1000D400u,
    };
    for (ps2::u32 channel = 0; channel < chcr_addresses.size(); ++channel) {
        ok = expect(hw.write32(chcr_addresses[channel], 1u << 8),
                    "DMAC CHCR start write failed") && ok;
        ok = expect((hw.dmac_running_mask() & (1u << channel)) != 0,
                    "DMAC running channel was not tracked") && ok;
    }
    ok = expect(hw.dmac_running_mask() == 0x3FFu,
                "DMAC running mask lost a channel") && ok;
    ok = expect(hw.write32(0x1000E000u, 1u) && hw.dmac_enabled(),
                "DMAC CTRL enable was not tracked") && ok;
    for (ps2::u32 channel = 0; channel < chcr_addresses.size(); ++channel) {
        ok = expect(hw.write32(chcr_addresses[channel], 0),
                    "DMAC CHCR completion write failed") && ok;
    }
    ok = expect(hw.dmac_running_mask() == 0,
                "DMAC completion left a channel active") && ok;
    hw.reset();
    return expect(!hw.dmac_enabled() && hw.dmac_running_mask() == 0,
                  "DMAC reset left the fast-path mask active") && ok;
}

bool test_ee_jit_matches_interpreter() {
    ps2::Ps2System jit_system;
    ps2::Ps2System interpreter_system;
    jit_system.ee().set_jit_enabled(true);
    interpreter_system.ee().set_jit_enabled(false);

    const auto i_type = [](ps2::u32 opcode, ps2::u32 rs,
                           ps2::u32 rt, ps2::u32 imm) {
        return (opcode << 26) | (rs << 21) | (rt << 16) | imm;
    };
    const auto r_type = [](ps2::u32 rs, ps2::u32 rt, ps2::u32 rd,
                           ps2::u32 sa, ps2::u32 funct) {
        return (rs << 21) | (rt << 16) | (rd << 11) |
               (sa << 6) | funct;
    };
    const std::array<ps2::u32, 18> instructions = {
        i_type(0x09, 1, 3, 0xFFFF), // ADDIU
        i_type(0x0C, 1, 3, 0x00FF), // ANDI
        i_type(0x0D, 1, 3, 0x8001), // ORI
        i_type(0x0E, 1, 3, 0x8001), // XORI
        i_type(0x0F, 0, 3, 0x8000), // LUI
        i_type(0x19, 1, 3, 0xFFFE), // DADDIU
        i_type(0x09, 3, 3, 1),      // source/destination alias
        r_type(1, 2, 3, 0, 0x21),   // ADDU
        r_type(1, 2, 3, 0, 0x23),   // SUBU
        r_type(1, 2, 3, 0, 0x24),   // AND
        r_type(1, 2, 3, 0, 0x25),   // OR
        r_type(1, 2, 3, 0, 0x26),   // XOR
        r_type(1, 2, 3, 0, 0x2D),   // DADDU
        r_type(0, 2, 3, 3, 0x00),   // SLL
        r_type(0, 2, 3, 4, 0x02),   // SRL
        r_type(0, 2, 3, 5, 0x03),   // SRA
        r_type(1, 2, 1, 0, 0x25),   // destination/source alias
        0u,                          // NOP
    };

    constexpr ps2::u32 code = 0x00001000u;
    bool ok = true;
    for (ps2::u32 round = 0; round < 4u; ++round) {
        for (const ps2::u32 instruction : instructions) {
            jit_system.ee().reset(code);
            interpreter_system.ee().reset(code);
            ok = expect(jit_system.bus().write32(code, instruction) &&
                            interpreter_system.bus().write32(code, instruction),
                        "failed to install EE JIT differential instruction") && ok;
            for (ps2::u32 reg = 1; reg < 32u; ++reg) {
                const ps2::u64 lo =
                    (static_cast<ps2::u64>(round) << 48) |
                    (0x80000000ull + reg * 0x01010101ull);
                const ps2::u64 hi = 0xA5A5A5A500000000ull | reg;
                jit_system.ee().state().gpr[reg] = {lo, hi};
                interpreter_system.ee().state().gpr[reg] = {lo, hi};
            }
            std::string jit_error;
            std::string interpreter_error;
            const bool jit_ok = jit_system.ee().step(jit_error);
            const bool interpreter_ok =
                interpreter_system.ee().step(interpreter_error);
            ok = expect(jit_ok == interpreter_ok &&
                            jit_error == interpreter_error,
                        "EE JIT and interpreter step results differ") && ok;
            const auto& jit_state = jit_system.ee().state();
            const auto& interpreter_state = interpreter_system.ee().state();
            ok = expect(jit_state.pc == interpreter_state.pc &&
                            jit_state.next_pc == interpreter_state.next_pc &&
                            jit_state.instructions_executed ==
                                interpreter_state.instructions_executed &&
                            jit_state.cop0 == interpreter_state.cop0,
                        "EE JIT and interpreter control state differ") && ok;
            for (ps2::u32 reg = 0; reg < 32u; ++reg) {
                ok = expect(jit_state.gpr[reg].lo ==
                                interpreter_state.gpr[reg].lo &&
                                jit_state.gpr[reg].hi ==
                                interpreter_state.gpr[reg].hi,
                            "EE JIT and interpreter registers differ") && ok;
            }
        }
    }
#if defined(_M_X64) || defined(__x86_64__)
    ok = expect(jit_system.ee().jit().compiled_count() != 0 &&
                    jit_system.ee().jit().executed_count() != 0,
                "EE JIT did not execute native code") && ok;
#endif
    return ok;
}

bool test_scheduler_ordering() {
    ps2::Scheduler scheduler;
    std::vector<ps2::EventType> fired;

    scheduler.schedule(ps2::EventType::Gs, 20);
    scheduler.schedule(ps2::EventType::Vif0, 10);
    scheduler.schedule(ps2::EventType::Gif, 10);

    scheduler.run_until(20, [&](const ps2::Scheduler::Event& event) {
        fired.push_back(event.type);
    });

    return expect(fired.size() == 3, "wrong scheduler event count") &&
           expect(
               fired[0] == ps2::EventType::Vif0,
               "first same-timestamp event lost insertion order") &&
           expect(
               fired[1] == ps2::EventType::Gif,
               "second same-timestamp event lost insertion order") &&
           expect(
               fired[2] == ps2::EventType::Gs,
               "later event fired out of order") &&
           expect(scheduler.now() == 20, "scheduler time mismatch");
}

bool test_scheduler_single_step() {
    ps2::Scheduler scheduler;
    scheduler.advance_one();
    scheduler.advance_one();
    if (!expect(scheduler.now() == 2 && scheduler.empty(),
                "empty scheduler step mismatch")) return false;
    scheduler.schedule(ps2::EventType::Gs, 1);
    scheduler.advance_one();
    return expect(scheduler.now() == 3 && scheduler.empty(),
                  "queued scheduler step mismatch");
}

bool test_scheduler_cancel() {
    ps2::Scheduler scheduler;
    int fired = 0;

    const auto cancelled =
        scheduler.schedule(ps2::EventType::EeDmac, 4);
    scheduler.schedule(ps2::EventType::Gs, 8);
    scheduler.cancel(cancelled);

    scheduler.run_until(8, [&](const ps2::Scheduler::Event&) {
        ++fired;
    });

    return expect(fired == 1, "cancelled scheduler event fired");
}

bool test_ee_reset_state() {
    ps2::Ps2System system;
    system.reset(0x00100000u);

    const auto& state = system.ee().state();
    return expect(state.pc == 0x00100000u, "EE reset PC mismatch") &&
           expect(
               state.next_pc == 0x00100004u,
               "EE reset next PC mismatch") &&
           expect(
               state.gpr[0].lo == 0 && state.gpr[0].hi == 0,
               "EE r0 reset state mismatch");
}

std::filesystem::path create_test_bios() {
    std::vector<ps2::u8> data(ps2::Bios::kSize, 0);

    const auto write32 =
        [&](std::size_t offset, ps2::u32 value) {
            for (ps2::u32 i = 0; i < 4; ++i) {
                data[offset + i] =
                    static_cast<ps2::u8>(value >> (i * 8));
            }
        };

    const auto write_entry =
        [&](std::size_t offset, const char* name,
            ps2::u16 ext_size, ps2::u32 file_size) {
            for (std::size_t i = 0;
                 i < 10 && name[i] != '\0'; ++i) {
                data[offset + i] =
                    static_cast<ps2::u8>(name[i]);
            }
            data[offset + 10] =
                static_cast<ps2::u8>(ext_size);
            data[offset + 11] =
                static_cast<ps2::u8>(ext_size >> 8);
            write32(offset + 12, file_size);
        };

    const std::array<ps2::u32, 9> reset_code = {
        0x401A7800u, 0x00000000u, 0x2B410059u,
        0x14200005u, 0x00000000u, 0x3C1ABFC0u,
        0x375A0800u, 0x03400008u, 0x00000000u,
    };
    for (std::size_t i = 0; i < reset_code.size(); ++i) {
        write32(i * 4, reset_code[i]);
    }

    const std::array<ps2::u32, 5> iop_stage = {
        0x3C081234u, // LUI  t0, 0x1234
        0x35085678u, // ORI  t0, t0, 0x5678
        0xAC080100u, // SW   t0, 0x100(zero)
        0x1000FFFFu, // BEQ  zero, zero, -1
        0x00000000u, // delay slot
    };
    for (std::size_t i = 0; i < iop_stage.size(); ++i) {
        write32(0x24 + (i * 4), iop_stage[i]);
    }

    const std::array<ps2::u32, 4> stage_two = {
        0x3C1A9FC4u, 0x375A1000u, 0x03400008u, 0x00000000u,
    };
    for (std::size_t i = 0; i < stage_two.size(); ++i) {
        write32(0x800 + (i * 4), stage_two[i]);
    }

    write32(0x41000, 0x24021234u);
    // Deliberately unsupported MMI funct 0x02. MADD (funct 0x00) is a real
    // R5900 instruction and can no longer serve as the synthetic halt marker.
    write32(0x41004, 0x70000002u);

    constexpr std::size_t romdir = 0x1000;
    write_entry(romdir + 0x00, "RESET", 0, 0x1000);
    write_entry(romdir + 0x10, "ROMDIR", 0, 0x40);
    write_entry(romdir + 0x20, "EXTINFO", 0, 0x20);
    write_entry(romdir + 0x30, "ROMVER", 0, 0x10);

    constexpr char romver[] = "TESTAC20260920\n";
    for (std::size_t i = 0; i < sizeof(romver) - 1; ++i) {
        data[0x1060 + i] =
            static_cast<ps2::u8>(romver[i]);
    }

    const auto path =
        std::filesystem::temp_directory_path() /
        "vibestation_ps2_test_bios.bin";
    std::ofstream file(
        path, std::ios::binary | std::ios::trunc);
    file.write(
        reinterpret_cast<const char*>(data.data()),
        static_cast<std::streamsize>(data.size()));
    return path;
}

bool test_bios_mapping_and_startup() {
    const auto path = create_test_bios();

    ps2::Ps2System system;
    std::string error;
    bool ok =
        expect(
            system.load_bios(path.string(), error),
            "synthetic BIOS failed to load") &&
        expect(
            system.bios().loaded(),
            "BIOS loaded flag not set") &&
        expect(
            system.bios().romver() == "TESTAC20260920",
            "ROMVER parsing failed");

    ps2::u32 physical = 0;
    ps2::u32 cached = 0;
    ps2::u32 uncached = 0;

    ok =
        expect(
            system.bus().read32(
                ps2::Bios::kPhysicalBase, physical) &&
                physical == 0x401A7800u,
            "physical BIOS mapping failed") &&
        ok;
    ok =
        expect(
            system.bus().read32(
                ps2::Bios::kCachedBase, cached) &&
                cached == physical,
            "cached BIOS alias failed") &&
        ok;
    ok =
        expect(
            system.bus().read32(
                ps2::Bios::kUncachedBase, uncached) &&
                uncached == physical,
            "uncached BIOS alias failed") &&
        ok;
    ok =
        expect(
            !system.bus().write32(
                ps2::Bios::kResetVector, 0),
            "BIOS unexpectedly accepted a write") &&
        ok;

    ok = expect(system.boot_bios(error), "BIOS startup failed") && ok;
    ok =
        expect(
            system.ee().state().pc == ps2::Bios::kResetVector,
            "EE reset vector mismatch") &&
        ok;
    ok =
        expect(
            system.ee().state().next_pc ==
                ps2::Bios::kResetVector + 4,
            "EE reset next-PC mismatch") &&
        ok;
    ok =
        expect(
            system.reset_instruction() == 0x401A7800u,
            "BIOS reset instruction mismatch") &&
        ok;

    std::string run_error;
    const ps2::u64 ran = system.run_ee(64, run_error);
    ok =
        expect(ran == 14, "unexpected synthetic BIOS instruction count") &&
        ok;
    ok =
        expect(system.ee().state().pc == 0x9FC41004u,
               "synthetic BIOS did not follow reset jumps") &&
        ok;
    ok =
        expect(system.ee().state().gpr[2].lo == 0x1234u,
               "synthetic BIOS ADDIU result mismatch") &&
        ok;
    ok =
        expect(system.ee().halted(),
               "unsupported instruction did not halt interpreter") &&
        ok;

    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);
    return ok;
}

bool test_iop_reset_and_shared_ram() {
    const auto path = create_test_bios();

    ps2::Ps2System system;
    std::string error;
    bool ok =
        expect(system.load_bios(path.string(), error),
               "IOP test BIOS failed to load") &&
        expect(system.boot_bios(error),
               "IOP test BIOS failed to start");

    const auto& reset = system.iop().state();
    ok =
        expect(reset.pc == ps2::Bios::kResetVector,
               "IOP reset PC mismatch") &&
        ok;
    ok =
        expect(reset.cop0[12] == 0x00400000u,
               "IOP reset Status mismatch") &&
        ok;
    ok =
        expect(reset.cop0[15] == 0x0000001Fu,
               "IOP PRId mismatch") &&
        ok;
    ok =
        expect(system.iop_reset_instruction() == 0x401A7800u,
               "IOP reset instruction mismatch") &&
        ok;

    const ps2::u64 executed = system.iop().run(8, error);
    ok =
        expect(executed == 8,
               "IOP synthetic reset path instruction count mismatch") &&
        ok;
    ok =
        expect(!system.iop().halted(),
               "IOP halted during synthetic reset path") &&
        ok;

    ps2::u32 value = 0;
    ok =
        expect(system.iop_ram().read32(0x100u, value) &&
                   value == 0x12345678u,
               "IOP CPU did not write IOP RAM") &&
        ok;

    value = 0;
    ok =
        expect(system.bus().read32(0xBC000100u, value) &&
                   value == 0x12345678u,
               "EE could not read the IOP RAM window") &&
        ok;

    ok =
        expect(system.iop_bus().write32(0x00200100u, 0xCAFEBABEu),
               "IOP RAM mirror write failed") &&
        ok;
    value = 0;
    ok =
        expect(system.bus().read32(0xBC000100u, value) &&
                   value == 0xCAFEBABEu,
               "IOP RAM mirror did not alias the 2 MiB RAM") &&
        ok;

    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);
    return ok;
}

bool test_iop_ram_mirror_boundary() {
    ps2::Ps2System system;

    bool ok =
        expect(
            system.iop_bus().write32(0x001FFFFEu, 0x44332211u),
            "IOP mirrored boundary write failed");

    ps2::u8 byte = 0;
    ok =
        expect(system.iop_ram().read8(0x001FFFFEu, byte) &&
                   byte == 0x11u,
               "IOP mirror byte 0 mismatch") &&
        ok;
    ok =
        expect(system.iop_ram().read8(0x001FFFFFu, byte) &&
                   byte == 0x22u,
               "IOP mirror byte 1 mismatch") &&
        ok;
    ok =
        expect(system.iop_ram().read8(0x00000000u, byte) &&
                   byte == 0x33u,
               "IOP mirror wrapped byte 2 mismatch") &&
        ok;
    ok =
        expect(system.iop_ram().read8(0x00000001u, byte) &&
                   byte == 0x44u,
               "IOP mirror wrapped byte 3 mismatch") &&
        ok;

    ps2::u32 value = 0;
    ok =
        expect(system.iop_bus().read32(0x003FFFFEu, value) &&
                   value == 0x44332211u,
               "IOP mirrored boundary read failed") &&
        ok;

    ps2::u16 halfword = 0;
    ok = expect(system.iop_bus().write16(0x00200100u, 0xA1B2u) &&
                    system.iop_bus().read16(0x00000100u, halfword) &&
                    halfword == 0xA1B2u,
                "IOP mirrored halfword fast path mismatch") && ok;
    ok = expect(system.iop_bus().write16(0x005FFFFFu, 0xC3D4u) &&
                    system.iop_bus().read16(0x001FFFFFu, halfword) &&
                    halfword == 0xC3D4u,
                "IOP mirrored halfword boundary mismatch") && ok;

    return ok;
}

bool test_iop_optional_extension_rom_windows() {
    ps2::Ps2System system;

    ps2::u32 value = 0xFFFFFFFFu;
    bool ok = expect(
        system.iop_bus().read32(0xBE000000u, value) && value == 0u,
        "IOP ROM1 uncached window did not read as an absent ROM");
    ok = expect(
             system.iop_bus().read32(0x9E400000u, value) && value == 0u,
             "IOP ROM2 cached window did not read as an absent ROM") &&
         ok;
    ok = expect(
             system.iop_bus().write32(0xBE000000u, 0xFFFFFFFFu) &&
                 system.iop_bus().read32(0x1E000000u, value) &&
                 value == 0u,
             "IOP optional ROM window did not remain read-only") &&
         ok;

    return ok;
}

bool test_ee_iop_startup_interleave() {
    const auto path = create_test_bios();

    ps2::Ps2System system;
    std::string error;
    bool ok =
        expect(system.load_bios(path.string(), error),
               "interleave test BIOS failed to load") &&
        expect(system.boot_bios(error),
               "interleave test BIOS failed to start");

    ok = expect(
             system.bus().write64(0x11008000u, 0u),
             "failed to install VU1 pacing test instruction") &&
         ok;
    system.vu1().start(0u);

    for (int i = 0; i < 7 && ok; ++i) {
        ok =
            expect(system.step_ee(error),
                   "EE failed before first IOP interleave slot") &&
            ok;
    }

    ok =
        expect(system.iop().state().instructions_executed == 0,
               "IOP ran too early in 8:1 startup interleave") &&
        ok;

    ok =
        expect(system.step_ee(error),
               "EE failed at first IOP interleave slot") &&
        ok;
    ok =
        expect(system.iop().state().instructions_executed == 1,
               "IOP did not run after eight EE startup steps") &&
        ok;
    ok =
        expect(system.scheduler().now() == 8,
               "scheduler did not advance with EE startup execution") &&
        ok;
    ok =
        expect(system.vu1().stats().instructions == 8u,
               "VU1 did not advance once per EE startup step") &&
        ok;

    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);
    return ok;
}

bool test_ee_sbus_iop_commands() {
    const auto path = create_test_bios();

    ps2::Ps2System system;
    std::string error;
    bool ok =
        expect(system.load_bios(path.string(), error),
               "SBUS test BIOS failed to load") &&
        expect(system.boot_bios(error),
               "SBUS test BIOS failed to start");

    ok = expect(
             system.iop_ram().write32(0x100u, 0xDEADBEEFu),
             "failed to seed IOP RAM before SBUS reset") &&
         ok;
    ok = expect(
             system.bus().write32(0x1000F240u, 1u << 19) &&
                 system.step_ee(error),
             "EE SBUS IOP reset request failed") &&
         ok;

    ps2::u32 value = 0xFFFFFFFFu;
    ok = expect(
             system.iop_ram().read32(0x100u, value) && value == 0u,
             "EE SBUS reset did not clear IOP RAM") &&
         ok;
    ok = expect(
             system.iop().state().pc == ps2::Bios::kResetVector,
             "EE SBUS reset did not reset the IOP CPU") &&
         ok;
    ok = expect(
             system.iop_bus().read32(0x1F801450u, value) && value == 0x8u,
             "EE SBUS reset did not restore IOP ICFG") &&
         ok;
    ok = expect(
             system.iop_intc().control() == 1u,
             "EE SBUS reset did not enable IOP interrupt control") &&
         ok;

    ok = expect(
             system.bus().write32(0x1000F240u, 1u << 18) &&
                 system.step_ee(error) &&
                 (system.iop_intc().status() & (1u << 1)) != 0,
             "EE SBUS interrupt request did not reach IOP INTC") &&
         ok;

    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);
    return ok;
}

bool test_iop_gte_bootstrap_transfers() {
    ps2::Ps2System system;
    std::string error;
    bool ok = true;

    constexpr ps2::u32 mtc2_r1_d3 =
        (0x12u << 26) | (0x04u << 21) | (1u << 16) | (3u << 11);
    constexpr ps2::u32 mfc2_r2_d3 =
        (0x12u << 26) | (0x00u << 21) | (2u << 16) | (3u << 11);
    constexpr ps2::u32 gte_command =
        (0x12u << 26) | (0x10u << 21) | 0x01u;
    constexpr ps2::u32 lwc2_d4 =
        (0x32u << 26) | (4u << 16) | 0x0100u;
    constexpr ps2::u32 swc2_d4 =
        (0x3Au << 26) | (4u << 16) | 0x0104u;

    ok = expect(
             system.iop_ram().write32(0x0000u, mtc2_r1_d3) &&
             system.iop_ram().write32(0x0004u, mfc2_r2_d3) &&
             system.iop_ram().write32(0x0008u, 0u) &&
             system.iop_ram().write32(0x000Cu, gte_command) &&
             system.iop_ram().write32(0x0010u, lwc2_d4) &&
             system.iop_ram().write32(0x0014u, swc2_d4) &&
             system.iop_ram().write32(0x0100u, 0xA1B2C3D4u),
             "failed to build IOP GTE bootstrap program") && ok;

    system.iop().reset(0x00000000u);
    system.iop().state().gpr[1] = 0x12345678u;

    ok = expect(system.iop().step(error), "IOP MTC2 failed") && ok;
    ok = expect(
             system.iop().state().gte_data[3] == 0x12345678u,
             "IOP MTC2 GTE register mismatch") && ok;

    ok = expect(system.iop().step(error), "IOP MFC2 failed") && ok;
    ok = expect(system.iop().step(error), "IOP MFC2 delay-slot NOP failed") && ok;
    ok = expect(
             system.iop().state().gpr[2] == 0x12345678u,
             "IOP MFC2 load-delay result mismatch") && ok;

    ok = expect(
             system.iop().step(error),
             "IOP bootstrap GTE command retirement failed") && ok;
    ok = expect(
             !system.iop().halted() &&
             system.iop().state().gte_ctrl[31] == 0u,
             "IOP bootstrap GTE command unexpectedly halted") && ok;

    ok = expect(system.iop().step(error), "IOP LWC2 failed") && ok;
    ok = expect(
             system.iop().state().gte_data[4] == 0xA1B2C3D4u,
             "IOP LWC2 GTE data mismatch") && ok;
    ok = expect(system.iop().step(error), "IOP SWC2 failed") && ok;

    ps2::u32 stored = 0;
    ok = expect(
             system.iop_ram().read32(0x0104u, stored) &&
             stored == 0xA1B2C3D4u,
             "IOP SWC2 memory result mismatch") && ok;

    return ok;
}

bool test_iop_cache_isolation_blocks_ram_store() {
    ps2::Ps2System system;

    std::string error;
    bool ok =
        expect(system.iop_ram().write32(0x0000u, 0xAC080100u),
               "failed to install IOP cache-isolation test opcode") &&
        expect(system.iop_ram().write32(0x0100u, 0xDEADBEEFu),
               "failed to seed IOP cache-isolation test RAM");

    system.iop().reset(0x00000000u);
    system.iop().state().gpr[8] = 0x12345678u;
    system.iop().state().cop0[12] |= 0x00010000u;

    ok =
        expect(system.iop().step(error),
               "IOP cache-isolated store instruction failed") &&
        ok;

    ps2::u32 value = 0;
    ok =
        expect(system.iop_ram().read32(0x0100u, value) &&
                   value == 0xDEADBEEFu,
               "cache-isolated IOP store incorrectly modified RAM") &&
        ok;

    return ok;
}

bool test_ee_timer0_clock_sources() {
    ps2::Ps2System system;

    ps2::u32 count = 0;
    bool ok =
        expect(system.bus().write32(0x10000010u, 0x83u),
               "failed to configure Timer0 HBlank clock") &&
        expect(system.bus().write32(0x10000000u, 0),
               "failed to clear Timer0 count");

    system.bus().tick(18875);
    ok =
        expect(system.bus().read32(0x10000000u, count) && count == 0,
               "Timer0 HBlank clock advanced before a scanline") &&
        ok;

    system.bus().tick(1);
    ok =
        expect(system.bus().read32(0x10000000u, count) && count == 1,
               "Timer0 HBlank clock did not advance at one scanline") &&
        ok;

    ok =
        expect(system.bus().write32(0x10000010u, 0x81u),
               "failed to configure Timer0 BUSCLK/16") &&
        expect(system.bus().write32(0x10000000u, 0),
               "failed to clear Timer0 BUSCLK/16 count") &&
        ok;

    system.bus().tick(32);
    ok =
        expect(system.bus().read32(0x10000000u, count) && count == 1,
               "Timer0 BUSCLK/16 divider mismatch") &&
        ok;

    return ok;
}

bool test_iop_timer_progress_and_irq() {
    ps2::IopHwWindow hw;
    ps2::IopIntc intc;
    hw.reset();
    intc.reset();

    bool ok = true;

    // Timer0: target=3, reset on target, repeated target IRQ.
    ok = expect(hw.write16(0x1F801108u, 3u),
                "IOP Timer0 target write failed") && ok;
    ok = expect(hw.write16(
                    0x1F801104u,
                    (1u << 3) | (1u << 4) | (1u << 6)),
                "IOP Timer0 mode write failed") && ok;

    hw.tick(2u, intc);
    ps2::u16 count16 = 0;
    ok = expect(hw.read16(0x1F801100u, count16) &&
                    count16 == 2u,
                "IOP Timer0 count did not advance") && ok;
    ok = expect((intc.status() & (1u << 4)) == 0,
                "IOP Timer0 IRQ fired before target") && ok;

    hw.tick(1u, intc);
    ok = expect(hw.read16(0x1F801100u, count16) &&
                    count16 == 0u,
                "IOP Timer0 did not reset at target") && ok;
    ps2::u16 mode16 = 0;
    ok = expect(hw.read16(0x1F801104u, mode16) &&
                    (mode16 & (1u << 11)) != 0,
                "IOP Timer0 target flag missing") && ok;
    ok = expect((intc.status() & (1u << 4)) != 0,
                "IOP Timer0 target IRQ missing") && ok;

    // Timer4: prescale /8.
    ok = expect(hw.write32(0x1F801494u, 1u << 13),
                "IOP Timer4 prescale write failed") && ok;
    hw.tick(7u, intc);
    ps2::u32 count32 = 0;
    ok = expect(hw.read32(0x1F801490u, count32) &&
                    count32 == 0u,
                "IOP Timer4 /8 advanced early") && ok;
    hw.tick(1u, intc);
    ok = expect(hw.read32(0x1F801490u, count32) &&
                    count32 == 1u,
                "IOP Timer4 /8 divider mismatch") && ok;

    // Reprogramming MODE must replace the cached clock divider and reset
    // the counter, including when returning to the default single tick.
    ok = expect(hw.write32(0x1F801494u, 0u),
                "IOP Timer4 clock-source reset failed") && ok;
    hw.tick(1u, intc);
    ok = expect(hw.read32(0x1F801490u, count32) &&
                    count32 == 1u,
                "IOP Timer4 cached divider was not refreshed") && ok;

    return ok;
}

bool test_cdvd_reset_status() {
    ps2::Ps2System system;

    ps2::u8 value = 0;
    bool ok =
        expect(system.iop_bus().read8(0xBF402005u, value) &&
                   value == 0x4Cu,
               "CDVD N-READY reset value mismatch");

    ok =
        expect(system.bus().read8(0x1F402005u, value) &&
                   value == 0x4Cu,
               "EE CDVD N-READY mapping mismatch") &&
        ok;

    ok =
        expect(system.iop_bus().read8(0xBF40200Au, value) &&
                   value == 0x01u,
               "CDVD tray-open reset status mismatch") &&
        ok;

    ok =
        expect(system.iop_bus().read8(0xBF40200Fu, value) &&
                   value == 0x00u,
               "CDVD reset disc type should be no-disc") &&
        ok;

    return ok;
}

bool test_cdvd_iop_segment_mirror() {
    ps2::Ps2System system;
    bool ok = true;

    ps2::u8 canonical = 0;
    ps2::u8 mirrored = 0;
    ok = expect(
             system.iop_bus().read8(0x1F402005u, canonical) &&
             system.iop_bus().read8(0x1F400005u, mirrored) &&
             canonical == mirrored &&
             mirrored == 0x4Cu,
             "CDVD 1F40 segment mirror read mismatch") && ok;

    // S-command writes through an alternate page must hit the same device.
    ok = expect(
             system.iop_bus().write8(0x1F40A016u, 0x08u),
             "CDVD mirrored S-command write failed") && ok;
    ps2::u8 ready = 0;
    ok = expect(
             system.iop_bus().read8(0x1F402017u, ready) &&
             (ready & 0x40u) == 0u,
             "CDVD mirrored S-command did not expose result FIFO") && ok;

    ps2::u8 first = 0xFFu;
    ok = expect(
             system.iop_bus().read8(0x1F40FF18u, first) &&
             first == 0u,
             "CDVD mirrored result FIFO read mismatch") && ok;

    return ok;
}

bool test_cdvd_scommand_result_fifo() {
    ps2::Ps2System system;

    bool ok =
        expect(system.iop_bus().write8(0xBF402016u, 0x08u),
               "CDVD Read RTC S-command write failed");

    ps2::u8 ready = 0;
    ok =
        expect(system.iop_bus().read8(0xBF402017u, ready) &&
                   (ready & 0x40u) == 0,
               "CDVD S-command result FIFO was not exposed") &&
        ok;

    const std::array<ps2::u8, 8> expected{
        0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x01u, 0x01u, 0x00u};

    for (const ps2::u8 expected_byte : expected) {
        ps2::u8 value = 0xFFu;
        ok =
            expect(system.iop_bus().read8(0xBF402018u, value) &&
                       value == expected_byte,
                   "CDVD RTC result byte mismatch") &&
            ok;
    }

    ok =
        expect(system.iop_bus().read8(0xBF402017u, ready) &&
                   (ready & 0x40u) != 0,
               "CDVD S-command FIFO did not return to empty") &&
        ok;

    ok =
        expect(system.iop_bus().write8(0xBF402017u, 0x30u),
               "CDVD mecacon parameter write failed") &&
        ok;
    ok =
        expect(system.iop_bus().write8(0xBF402016u, 0x03u),
               "CDVD mecacon S-command write failed") &&
        ok;

    ps2::u8 status = 0;
    ps2::u8 tray = 0;
    ok =
        expect(system.iop_bus().read8(0xBF402018u, status) &&
                   status == 0x01u,
               "CDVD mecacon tray status mismatch") &&
        ok;
    ok =
        expect(system.iop_bus().read8(0xBF402018u, tray) &&
                   tray == 0x08u,
               "CDVD mecacon tray detail mismatch") &&
        ok;

    return ok;
}

bool test_cdvd_mechacon_config_nvram() {
    ps2::Ps2System system;
    bool ok = true;

    auto write_param = [&](ps2::u8 value) {
        return system.iop_bus().write8(0xBF402017u, value);
    };
    auto command = [&](ps2::u8 value) {
        return system.iop_bus().write8(0xBF402016u, value);
    };
    auto read_result = [&](ps2::u8& value) {
        return system.iop_bus().read8(0xBF402018u, value);
    };

    // Open config section 1 for two read blocks. With no BIOS identity loaded,
    // the bootstrap model uses the legacy layout and seeds block 1 with the
    // standard English OSD defaults.
    ok = expect(
             write_param(0u) &&
             write_param(1u) &&
             write_param(2u) &&
             command(0x40u),
             "CDVD OpenConfig command failed") && ok;
    ps2::u8 value = 0xFFu;
    ok = expect(
             read_result(value) && value == 0u,
             "CDVD OpenConfig did not return success") && ok;

    ok = expect(
             command(0x41u),
             "CDVD first ReadConfig command failed") && ok;
    std::array<ps2::u8, 16> first{};
    for (auto& byte : first) {
        ok = expect(
                 read_result(byte),
                 "CDVD first config block read failed") && ok;
    }
    ok = expect(
             std::all_of(
                 first.begin(),
                 first.end(),
                 [](ps2::u8 byte) { return byte == 0u; }),
             "CDVD first legacy config block should reset to zero") && ok;

    ok = expect(
             command(0x41u),
             "CDVD second ReadConfig command failed") && ok;
    std::array<ps2::u8, 16> second{};
    for (auto& byte : second) {
        ok = expect(
                 read_result(byte),
                 "CDVD second config block read failed") && ok;
    }
    const std::array<ps2::u8, 16> english{
        0x30u, 0x21u, 0x00u, 0x00u,
        0x00u, 0x70u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x41u,
    };
    ok = expect(
             second == english,
             "CDVD seeded English OSD config mismatch") && ok;

    ok = expect(
             command(0x43u) &&
             read_result(value) &&
             value == 0u,
             "CDVD CloseConfig did not return success") && ok;

    // Legacy i.Link NVRAM starts at 0x1C0. SCMD 0x0A takes a word index
    // and returns status followed by the word in the mechacon byte order.
    ok = expect(
             write_param(0x00u) &&
             write_param(0xE0u) &&
             command(0x0Au),
             "CDVD ReadNVM command failed") && ok;
    ps2::u8 status = 0xFFu;
    ps2::u8 hi = 0xFFu;
    ps2::u8 lo = 0xFFu;
    ok = expect(
             read_result(status) &&
             read_result(hi) &&
             read_result(lo) &&
             status == 0u &&
             hi == 0xACu &&
             lo == 0x00u,
             "CDVD ReadNVM i.Link word mismatch") && ok;

    return ok;
}

bool test_iop_intc_registers() {
    ps2::Ps2System system;

    bool ok =
        expect(system.iop_bus().write32(
                   ps2::IopIntc::kIMask, 1u << 2),
               "IOP I_MASK write failed") &&
        expect(system.iop_bus().write32(
                   ps2::IopIntc::kICtrl, 1),
               "IOP I_CTRL write failed");

    system.iop_intc().raise(2);
    ok =
        expect(system.iop_intc().pending(),
               "IOP INTC did not report enabled pending source") &&
        ok;

    ps2::u32 status = 0;
    ok =
        expect(system.iop_bus().read32(
                   ps2::IopIntc::kIStat, status) &&
                   (status & (1u << 2)) != 0,
               "IOP I_STAT did not latch source 2") &&
        ok;

    ok =
        expect(system.iop_bus().write32(
                   ps2::IopIntc::kIStat, 0),
               "IOP I_STAT acknowledge write failed") &&
        ok;
    ok =
        expect(!system.iop_intc().pending(),
               "IOP I_STAT zero write did not clear pending source") &&
        ok;

    system.iop_intc().raise(2);
    ps2::u32 control = 0;
    ok =
        expect(system.iop_bus().read32(
                   ps2::IopIntc::kICtrl, control) &&
                   control == 1,
               "IOP I_CTRL read value mismatch") &&
        ok;
    ok =
        expect(system.iop_intc().control() == 0,
               "IOP I_CTRL was not read-to-clear") &&
        ok;
    ok =
        expect(!system.iop_intc().pending(),
               "cleared I_CTRL still allowed an interrupt") &&
        ok;

    return ok;
}

bool test_iop_external_interrupt_exception() {
    const auto path = create_test_bios();

    ps2::Ps2System system;
    std::string error;
    bool ok =
        expect(system.load_bios(path.string(), error),
               "INTC test BIOS failed to load") &&
        expect(system.boot_bios(error),
               "INTC test BIOS failed to start");

    // BEV + IP2 mask + current interrupt enable.
    system.iop().state().cop0[12] = 0x00400401u;
    ok =
        expect(system.iop_bus().write32(
                   ps2::IopIntc::kIMask, 1u << 2),
               "INTC test I_MASK write failed") &&
        expect(system.iop_bus().write32(
                   ps2::IopIntc::kICtrl, 1),
               "INTC test I_CTRL write failed") &&
        ok;

    system.iop_intc().raise(2);

    ok =
        expect(system.iop().step(error),
               "IOP failed while taking external interrupt") &&
        ok;

    const auto& state = system.iop().state();
    ok =
        expect(state.last_pc == 0xBFC00180u,
               "IOP external interrupt did not vector through BEV") &&
        ok;
    ok =
        expect(state.cop0[14] == ps2::Bios::kResetVector,
               "IOP interrupt EPC mismatch") &&
        ok;
    ok =
        expect((state.cop0[13] & 0x7Cu) == 0,
               "IOP interrupt exception code was not zero") &&
        ok;
    ok =
        expect((state.cop0[13] & 0x400u) != 0,
               "IOP Cause did not expose external interrupt IP2") &&
        ok;

    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);
    return ok;
}

bool test_cdvd_raises_iop_irq2() {
    ps2::Ps2System system;

    bool ok =
        expect(system.iop_bus().write32(
                   ps2::IopIntc::kIMask, 1u << 2),
               "CDVD IRQ I_MASK write failed") &&
        expect(system.iop_bus().write32(
                   ps2::IopIntc::kICtrl, 1),
               "CDVD IRQ I_CTRL write failed");

    ok =
        expect(system.iop_bus().write8(0xBF402004u, 0x00u),
               "CDVD NOP command write failed") &&
        ok;
    ok =
        expect(system.iop_intc().pending(),
               "CDVD command completion did not assert IOP IRQ2") &&
        ok;

    return ok;
}

bool test_iop_root_counters() {
    ps2::Ps2System system;
    bool ok = true;

    // Timer2: system clock / 8, target=3, reset on target, IRQ on target.
    ok = expect(system.iop_bus().write16(0x1F801128u, 3u),
                "IOP Timer2 target write failed") && ok;
    ok = expect(system.iop_bus().write16(
                    0x1F801124u,
                    static_cast<ps2::u16>(
                        (1u << 9) | // /8 prescaler
                        (1u << 4) | // IRQ on target
                        (1u << 3))), // reset on target
                "IOP Timer2 mode write failed") && ok;

    system.iop_bus().tick(23u);
    ps2::u16 count = 0;
    ok = expect(system.iop_bus().read16(0x1F801120u, count) &&
                    count == 2u,
                "IOP Timer2 prescaler advanced too early") && ok;

    system.iop_bus().tick(1u);
    ok = expect(system.iop_bus().read16(0x1F801120u, count) &&
                    count == 0u,
                "IOP Timer2 target reset mismatch") && ok;
    ok = expect((system.iop_intc().status() & (1u << 6)) != 0,
                "IOP Timer2 target IRQ missing") && ok;

    ps2::u16 mode = 0;
    ok = expect(system.iop_bus().read16(0x1F801124u, mode) &&
                    (mode & (1u << 11)) != 0,
                "IOP Timer2 target flag missing") && ok;

    // Timer4: verify the 32-bit counter and /16 prescaler selection.
    ok = expect(system.iop_bus().write32(0x1F801494u, 2u << 13),
                "IOP Timer4 mode write failed") && ok;
    system.iop_bus().tick(31u);
    ps2::u32 count32 = 0;
    ok = expect(system.iop_bus().read32(0x1F801490u, count32) &&
                    count32 == 1u,
                "IOP Timer4 /16 prescaler mismatch") && ok;

    ok = expect(system.iop_bus().write32(0x1F801494u, 0u),
                "IOP Timer4 default-rate write failed") && ok;
    system.iop_bus().tick(1u);
    ok = expect(system.iop_bus().read32(0x1F801490u, count32) &&
                    count32 == 1u,
                "IOP Timer4 cached rate did not update") && ok;

    return ok;
}

bool test_iop_event_free_tick_matches_regular_tick() {
    ps2::Ps2System fast;
    ps2::Ps2System reference;
    constexpr std::array<ps2::u32, 6> bases = {
        0x1F801100u, 0x1F801110u, 0x1F801120u,
        0x1F801480u, 0x1F801490u, 0x1F8014A0u};
    for (ps2::u32 i = 0; i < bases.size(); ++i) {
        const ps2::u32 target = i < 3u ? 0xFF00u : 0xFFFFFF00u;
        if (!expect(fast.iop_bus().write32(bases[i] + 8u, target) &&
                    reference.iop_bus().write32(bases[i] + 8u, target),
                    "IOP event-free timer setup failed")) return false;
    }
    if (!expect(fast.iop_bus().tick_event_free(1000u),
                "event-free IOP tick rejected a safe interval")) return false;
    reference.iop_bus().tick(1000u);
    for (const ps2::u32 base : bases) {
        ps2::u32 fast_count = 0, reference_count = 0;
        ps2::u32 fast_mode = 0, reference_mode = 0;
        if (!expect(fast.iop_bus().read32(base, fast_count) &&
                    reference.iop_bus().read32(base, reference_count) &&
                    fast_count == reference_count &&
                    fast.iop_bus().read32(base + 4u, fast_mode) &&
                    reference.iop_bus().read32(base + 4u, reference_mode) &&
                    fast_mode == reference_mode,
                    "event-free IOP root counter diverged")) return false;
    }
    ps2::u32 before = 0, after = 0;
    if (!expect(fast.iop_bus().write32(bases[0] + 8u, 1001u) &&
                fast.iop_bus().read32(bases[0], before) &&
                !fast.iop_bus().tick_event_free(1u) &&
                fast.iop_bus().read32(bases[0], after) &&
                before == after,
                "event-free IOP tick crossed a timer target")) return false;
    return true;
}

bool test_iop_spu2_register_window() {
    ps2::Ps2System system;

    ps2::u16 value16 = 0xFFFFu;
    bool ok =
        expect(system.iop_bus().read16(0x1F900344u, value16) &&
                   value16 == 0,
               "SPU2 core0 STATX reset value mismatch");

    value16 = 0xFFFFu;
    ok =
        expect(system.iop_bus().read16(0x1F900744u, value16) &&
                   value16 == 0,
               "SPU2 core1 STATX reset value mismatch") &&
        ok;

    ok =
        expect(system.iop_bus().write16(0xBF900188u, 0x55AAu),
               "SPU2 KSEG1 16-bit write failed") &&
        ok;
    value16 = 0;
    ok =
        expect(system.iop_bus().read16(0x1F900188u, value16) &&
                   value16 == 0x55AAu,
               "SPU2 physical 16-bit readback mismatch") &&
        ok;

    ok =
        expect(system.iop_bus().write32(0x1F900300u, 0x44332211u),
               "SPU2 32-bit bootstrap write failed") &&
        ok;
    ps2::u32 value32 = 0;
    ok =
        expect(system.iop_bus().read32(0xBF900300u, value32) &&
                   value32 == 0x44332211u,
               "SPU2 KSEG1 32-bit readback mismatch") &&
        ok;

    ok =
        expect(system.iop_bus().write16(0xBF900B60u, 0xA55Au),
               "SPU2 high register write failed") &&
        ok;
    value16 = 0;
    ok =
        expect(system.iop_bus().read16(0x1F900B60u, value16) &&
                   value16 == 0xA55Au,
               "SPU2 high register alias mismatch") &&
        ok;

    return ok;
}

bool test_iop_bus_repeating_timer_irq() {
    ps2::Ps2System system;

    constexpr ps2::u32 timer5 = 0x1F8014A0u;
    constexpr ps2::u32 timer5_irq = 1u << 16;
    constexpr ps2::u32 reset_at_target = 1u << 3;
    constexpr ps2::u32 irq_at_target = 1u << 4;
    constexpr ps2::u32 repeat_irq = 1u << 6;

    bool ok =
        expect(system.iop_bus().write32(timer5 + 8u, 3u),
               "IOP Timer5 target write failed") &&
        expect(system.iop_bus().write32(
                   timer5 + 4u,
                   reset_at_target | irq_at_target | repeat_irq),
               "IOP Timer5 repeat mode write failed");

    system.iop_bus().tick(3u);
    ok = expect((system.iop_intc().status() & timer5_irq) != 0,
                "IOP Timer5 first repeated IRQ missing") && ok;

    system.iop_intc().reset();
    system.iop_bus().tick(3u);
    ok = expect((system.iop_intc().status() & timer5_irq) != 0,
                "IOP Timer5 sticky target flag suppressed repeat IRQ") && ok;

    return ok;
}

bool test_cdvd_config_scommands() {
    ps2::Ps2System system;
    bool ok = true;

    // Match the SCPH-39001 BIOS request used by sceCdOpenConfig:
    // read two blocks from configuration area 1.
    for (const ps2::u8 parameter : {0x00u, 0x01u, 0x02u}) {
        ok = expect(system.iop_bus().write8(
                        0xBF402017u, parameter),
                    "CDVD config-open parameter write failed") && ok;
    }
    ok = expect(system.iop_bus().write8(0xBF402016u, 0x40u),
                "CDVD config-open command write failed") && ok;

    ps2::u8 value = 0xFFu;
    ok = expect(system.iop_bus().read8(0xBF402018u, value) &&
                    value == 0,
                "CDVD config-open command failed") && ok;

    ok = expect(system.iop_bus().write8(0xBF402016u, 0x41u),
                "CDVD first config-read command write failed") && ok;
    for (int i = 0; i < 16; ++i) {
        value = 0xFFu;
        ok = expect(system.iop_bus().read8(0xBF402018u, value) &&
                        value == 0,
                    "CDVD first config block mismatch") && ok;
    }

    constexpr std::array<ps2::u8, 16> kUsEnglishConfig{
        0x30u, 0x21u, 0x00u, 0x00u,
        0x00u, 0x70u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x41u};
    ok = expect(system.iop_bus().write8(0xBF402016u, 0x41u),
                "CDVD second config-read command write failed") && ok;
    for (const ps2::u8 expected : kUsEnglishConfig) {
        value = 0xFFu;
        ok = expect(system.iop_bus().read8(0xBF402018u, value) &&
                        value == expected,
                    "CDVD US-English config block mismatch") && ok;
    }

    ok = expect(system.iop_bus().write8(0xBF402016u, 0x43u),
                "CDVD config-close command write failed") && ok;
    value = 0xFFu;
    ok = expect(system.iop_bus().read8(0xBF402018u, value) &&
                    value == 0,
                "CDVD config-close command failed") && ok;

    return ok;
}

bool test_iop_spu2_dma_bootstrap_completion() {
    ps2::Ps2System system;
    bool ok = true;

    // DICR: master enable + DMA4 enable.
    ok = expect(
        system.iop_bus().write32(0x1F8010F4u, 0x00900000u),
        "failed to enable SPU2 DMA4 interrupt") && ok;
    ok = expect(
        system.iop_bus().write16(0x1F9001B0u, 1u),
        "failed to set SPU2 core0 DMA busy token") && ok;
    ok = expect(
        system.iop_bus().write32(0x1F8010C8u, 0x01000201u),
        "SPU2 DMA4 CHCR write failed") && ok;

    ps2::u32 value = 0;
    ok = expect(
        system.iop_bus().read32(0x1F8010C8u, value) &&
            (value & 0x01000000u) == 0,
        "SPU2 DMA4 start bit did not clear") && ok;
    ok = expect(
        system.iop_bus().read32(0x1F8010F4u, value) &&
            (value & (1u << 28)) != 0 &&
            (value & 0x80000000u) != 0,
        "SPU2 DMA4 DICR completion missing") && ok;
    ok = expect(
        system.iop_bus().read32(ps2::IopIntc::kIStat, value) &&
            (value & (1u << 3)) != 0,
        "SPU2 DMA4 did not raise IOP DMA interrupt") && ok;
    ok = expect(
        (value & (1u << 9)) == 0,
        "SPU2 DMA4 raised its dedicated interrupt synchronously") && ok;
    system.iop_bus().tick(47u);
    ok = expect(
        system.iop_bus().read32(ps2::IopIntc::kIStat, value) &&
            (value & (1u << 9)) == 0,
        "SPU2 DMA4 interrupt fired before one SPU2 word interval") && ok;
    system.iop_bus().tick(1u);
    ok = expect(
        system.iop_bus().read32(ps2::IopIntc::kIStat, value) &&
            (value & (1u << 9)) != 0,
        "SPU2 DMA4 did not raise the delayed dedicated interrupt") && ok;

    ps2::u16 statx = 0;
    ok = expect(
        system.iop_bus().read16(0x1F900344u, statx) &&
            (statx & 0x0080u) != 0 &&
            (statx & 0x0400u) == 0,
        "SPU2 core0 STATX did not become DMA-ready") && ok;
    ok = expect(
        system.iop_bus().read16(0x1F9001B0u, statx) && statx == 0u,
        "SPU2 core0 DMA busy token did not clear") && ok;

    // DICR2 routes an enabled channel interrupt even when its master bit is
    // clear. The master bit only controls DICR2's aggregate bit 31.
    ok = expect(
        system.iop_bus().write32(ps2::IopIntc::kIStat, 0xFFFFFFF7u),
        "failed to clear the first-bank DMA interrupt") && ok;
    ok = expect(
        system.iop_bus().write32(0x1F801574u, 0x00010000u),
        "failed to enable SPU2 DMA7 interrupt") && ok;
    ok = expect(
        system.iop_bus().write16(0x1F9005B0u, 2u),
        "failed to set SPU2 core1 DMA busy token") && ok;
    ok = expect(
        system.iop_bus().write32(0x1F801508u, 0x01000201u),
        "SPU2 DMA7 CHCR write failed") && ok;

    ok = expect(
        system.iop_bus().read32(0x1F801508u, value) &&
            (value & 0x01000000u) == 0,
        "SPU2 DMA7 start bit did not clear") && ok;
    ok = expect(
        system.iop_bus().read32(0x1F801574u, value) &&
            (value & (1u << 24)) != 0 &&
            (value & 0x80000000u) == 0,
        "SPU2 DMA7 DICR2 completion missing") && ok;
    ok = expect(
        system.iop_bus().read32(ps2::IopIntc::kIStat, value) &&
            (value & (1u << 3)) != 0,
        "SPU2 DMA7 did not route its enabled DICR2 interrupt") && ok;
    ok = expect(
        system.iop_bus().read16(0x1F900744u, statx) &&
            (statx & 0x0080u) != 0 &&
            (statx & 0x0400u) == 0,
        "SPU2 core1 STATX did not become DMA-ready") && ok;
    ok = expect(
        system.iop_bus().read16(0x1F9005B0u, statx) && statx == 0u,
        "SPU2 core1 DMA busy token did not clear") && ok;

    return ok;
}

bool test_iop_sio2_minimal_transfer_status() {
    ps2::Ps2System system;
    ps2::u32 value = 0;

    bool ok = expect(
        system.iop_bus().read32(0x1F808268u, value) &&
            value == 0x000003BCu,
        "SIO2 CTRL reset value mismatch");
    ok = expect(
        system.iop_bus().read32(0x1F80826Cu, value) &&
            value == 0x0001D100u,
        "SIO2 CMD_STAT reset value mismatch") && ok;
    ok = expect(
        system.iop_bus().read32(0x1F808270u, value) &&
            value == 0x0000000Fu,
        "SIO2 PORT_STAT reset value mismatch") && ok;
    ok = expect(
        system.iop_bus().read32(0x1F808274u, value) &&
            value == 0u,
        "SIO2 FIFO_STAT reset value mismatch") && ok;

    ok = expect(
        system.iop_bus().write32(0x1F808268u, 1u),
        "SIO2 CTRL start write failed") && ok;
    ok = expect(
        system.iop_bus().read32(0x1F80826Cu, value) &&
            value == 0x0003D000u,
        "SIO2 no-device CMD_STAT mismatch") && ok;
    ok = expect(
        system.iop_bus().read32(0x1F808280u, value) &&
            (value & 1u) != 0,
        "SIO2 local interrupt status missing") && ok;
    ok = expect(
        system.iop_bus().read32(ps2::IopIntc::kIStat, value) &&
            (value & (1u << 17)) != 0,
        "SIO2 transfer did not raise IOP IRQ17") && ok;

    ok = expect(
        system.iop_bus().write32(0x1F808280u, 1u) &&
        system.iop_bus().read32(0x1F808280u, value) &&
            value == 0u,
        "SIO2 local interrupt acknowledge failed") && ok;

    return ok;
}

bool test_iop_sio2_dma_bootstrap_completion() {
    ps2::Ps2System system;
    bool ok = expect(
        system.iop_bus().write32(
            0x1F801574u,
            0x00B00000u), // master + DMA11/12 enables
        "failed to enable SIO2 DMA interrupts");

    ok = expect(
        system.iop_bus().write32(0x1F801548u, 0x01000201u),
        "SIO2 IN DMA11 CHCR write failed") && ok;

    ps2::u32 value = 0;
    ok = expect(
        system.iop_bus().read32(0x1F801548u, value) &&
            (value & 0x01000000u) == 0,
        "SIO2 DMA11 start bit did not clear") && ok;
    ok = expect(
        system.iop_bus().read32(0x1F801574u, value) &&
            (value & (1u << 28)) != 0 &&
            (value & 0x80000000u) != 0,
        "SIO2 DMA11 DICR2 completion missing") && ok;

    ok = expect(
        system.iop_bus().write32(0x1F801558u, 0x01000201u),
        "SIO2 OUT DMA12 CHCR write failed") && ok;
    ok = expect(
        system.iop_bus().read32(0x1F801558u, value) &&
            (value & 0x01000000u) == 0,
        "SIO2 DMA12 start bit did not clear") && ok;
    ok = expect(
        system.iop_bus().read32(0x1F801574u, value) &&
            (value & (1u << 29)) != 0,
        "SIO2 DMA12 DICR2 completion missing") && ok;
    ok = expect(
        system.iop_bus().read32(ps2::IopIntc::kIStat, value) &&
            (value & (1u << 3)) != 0,
        "SIO2 DMA did not raise IOP DMA interrupt") && ok;

    return ok;
}

bool test_ee_ipu_dma_bootstrap_paths() {
    ps2::Ps2System system;
    ps2::IpuDma dma;
    dma.reset();
    std::string error;
    bool ok = true;

    constexpr ps2::u32 dmac_ctrl = 0x1000E000u;
    constexpr ps2::u32 dmac_stat = 0x1000E010u;
    constexpr ps2::u32 from_base = 0x1000B000u;
    constexpr ps2::u32 to_base = 0x1000B400u;
    constexpr ps2::u32 str = 1u << 8;

    ok = expect(
             system.bus().write32(dmac_ctrl, 1u),
             "failed to enable DMAC for IPU DMA test") && ok;

    // Firmware probes can arm FROM_IPU with QWC=0 while OFC is empty.
    ok = expect(
             system.bus().write32(from_base + 0x20u, 0u) &&
             system.bus().write32(from_base + 0x00u, str),
             "failed to arm zero-QWC IPU0 DMA") && ok;
    ok = expect(
             dma.service(system.bus(), error),
             "zero-QWC IPU0 DMA service failed") && ok;

    ps2::u32 value = 0;
    ok = expect(
             system.bus().read32(from_base + 0x00u, value) &&
             (value & str) == 0,
             "zero-QWC IPU0 DMA left STR set") && ok;
    ok = expect(
             system.bus().read32(dmac_stat, value) &&
             (value & (1u << 3)) != 0,
             "zero-QWC IPU0 completion IRQ missing") && ok;
    ok = expect(
             system.bus().write32(dmac_stat, 1u << 3),
             "failed to acknowledge IPU0 DMA IRQ") && ok;

    // TO_IPU consumes real memory payload into the modeled input FIFO.
    ok = expect(
             system.bus().write64(
                 0x6000u, 0x0123456789ABCDEFull) &&
             system.bus().write64(
                 0x6008u, 0xFEDCBA9876543210ull) &&
             system.bus().write32(to_base + 0x10u, 0x6000u) &&
             system.bus().write32(to_base + 0x20u, 1u) &&
             system.bus().write32(to_base + 0x00u, str),
             "failed to arm IPU1 input DMA") && ok;
    error.clear();
    ok = expect(
             dma.service(system.bus(), error),
             "IPU1 input DMA service failed") && ok;

    ps2::u64 lo = 0;
    ps2::u64 hi = 0;
    ok = expect(
             system.bus().read64(0x10007010u, lo) &&
             system.bus().read64(0x10007018u, hi) &&
             lo == 0x0123456789ABCDEFull &&
             hi == 0xFEDCBA9876543210ull,
             "IPU1 FIFO payload mismatch") && ok;
    ok = expect(
             system.bus().read32(to_base + 0x00u, value) &&
             (value & str) == 0 &&
             system.bus().read32(dmac_stat, value) &&
             (value & (1u << 4)) != 0,
             "IPU1 DMA completion state mismatch") && ok;

    return ok;
}

bool test_ee_scratchpad_dma_round_trip() {
    ps2::Ps2System system;
    ps2::SprDma dma;
    dma.reset();
    std::string error;
    bool ok = true;

    constexpr ps2::u32 dmac_ctrl = 0x1000E000u;
    constexpr ps2::u32 from_base = 0x1000D000u;
    constexpr ps2::u32 to_base = 0x1000D400u;
    constexpr ps2::u32 str = 1u << 8;

    ok = expect(
             system.bus().write32(dmac_ctrl, 1u),
             "failed to enable EE DMAC for SPR test") && ok;

    // Channel 9: main memory -> scratchpad.
    ok = expect(
             system.bus().write64(0x4000u, 0x1122334455667788ull) &&
             system.bus().write64(0x4008u, 0x99AABBCCDDEEFF00ull) &&
             system.bus().write32(to_base + 0x10u, 0x4000u) &&
             system.bus().write32(to_base + 0x20u, 1u) &&
             system.bus().write32(to_base + 0x80u, 0x20u) &&
             system.bus().write32(to_base + 0x00u, str),
             "failed to program SPR-to DMA") && ok;
    ok = expect(
             dma.service(system.bus(), error),
             "SPR-to DMA service failed") && ok;

    ps2::u64 lo = 0;
    ps2::u64 hi = 0;
    ps2::u32 chcr = 0;
    ps2::u32 qwc = 0;
    ps2::u32 stat = 0;
    ok = expect(
             system.bus().read64(0x70000020u, lo) &&
             system.bus().read64(0x70000028u, hi) &&
             lo == 0x1122334455667788ull &&
             hi == 0x99AABBCCDDEEFF00ull,
             "SPR-to DMA payload mismatch") && ok;
    ok = expect(
             system.bus().read32(to_base + 0x00u, chcr) &&
             (chcr & str) == 0 &&
             system.bus().read32(to_base + 0x20u, qwc) &&
             qwc == 0,
             "SPR-to DMA did not complete") && ok;
    ok = expect(
             system.bus().read32(0x1000E010u, stat) &&
             (stat & (1u << 9)) != 0,
             "SPR-to DMA completion IRQ missing") && ok;

    // Acknowledge channel 9 before checking channel 8 independently.
    ok = expect(
             system.bus().write32(0x1000E010u, 1u << 9),
             "failed to acknowledge SPR-to IRQ") && ok;

    // Channel 8: scratchpad -> main memory, including scratchpad wrap.
    ok = expect(
             system.bus().write64(
                 0x70003FF0u, 0x0123456789ABCDEFull) &&
             system.bus().write64(
                 0x70003FF8u, 0xFEDCBA9876543210ull) &&
             system.bus().write32(from_base + 0x10u, 0x5000u) &&
             system.bus().write32(from_base + 0x20u, 1u) &&
             system.bus().write32(from_base + 0x80u, 0x3FF0u) &&
             system.bus().write32(from_base + 0x00u, str),
             "failed to program SPR-from DMA") && ok;
    error.clear();
    ok = expect(
             dma.service(system.bus(), error),
             "SPR-from DMA service failed") && ok;

    lo = hi = 0;
    ok = expect(
             system.bus().read64(0x5000u, lo) &&
             system.bus().read64(0x5008u, hi) &&
             lo == 0x0123456789ABCDEFull &&
             hi == 0xFEDCBA9876543210ull,
             "SPR-from DMA payload mismatch") && ok;
    ok = expect(
             system.bus().read32(from_base + 0x00u, chcr) &&
             (chcr & str) == 0 &&
             system.bus().read32(0x1000E010u, stat) &&
             (stat & (1u << 8)) != 0,
             "SPR-from DMA completion state mismatch") && ok;

    return ok;
}

bool test_iop_ohci_bootstrap_reset() {
    ps2::Ps2System system;
    bool ok = true;
    ps2::u32 value = 0;

    ok = expect(
             system.iop_bus().read32(0x1F801600u, value) &&
             value == 0x10u,
             "OHCI revision reset value mismatch") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801648u, value) &&
             value == 0x202u,
             "OHCI root-hub descriptor mismatch") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801654u, value) &&
             value == 0x100u,
             "OHCI root-port power/reset state mismatch") && ok;

    // HCR must self-clear. Retaining this bit in a generic register window
    // leaves BIOS USB initialization in a permanent reset poll.
    ok = expect(
             system.iop_bus().write32(0x1F801608u, 1u) &&
             system.iop_bus().read32(0x1F801608u, value) &&
             value == 0u,
             "OHCI host-controller reset bit did not self-clear") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801604u, value) &&
             (value & 0xC0u) == 0xC0u,
             "OHCI soft reset did not enter suspend state") && ok;

    // With no USB devices attached, list-filled flags may retire immediately
    // but must never remain stuck as busy.
    ok = expect(
             system.iop_bus().write32(0x1F801608u, 0x6u) &&
             system.iop_bus().read32(0x1F801608u, value) &&
             value == 0u,
             "OHCI list-filled flags did not retire") && ok;

    ok = expect(
             system.iop_bus().read32(0x1F80163Cu, value) &&
             value == 0u,
             "OHCI frame number did not reset to zero") && ok;
    system.iop_bus().tick(36864u);
    ok = expect(
             system.iop_bus().read32(0x1F80163Cu, value) &&
             value == 1u,
             "OHCI frame number did not advance") && ok;

    return ok;
}

bool test_iop_firewire_bootstrap_probes() {
    ps2::Ps2System system;
    bool ok = true;
    ps2::u32 value = 0;

    ok = expect(
             system.iop_bus().read32(0x1F808400u, value) &&
             value == 0xFFC00001u,
             "i.Link node ID probe mismatch") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F808410u, value) &&
             value == 0x8u,
             "i.Link SCLK ready reset state mismatch") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F80847Cu, value) &&
             value == 0x10000001u,
             "i.Link node comparison probe mismatch") && ok;

    ok = expect(
             system.iop_bus().write32(0x1F808408u, 0x00800055u) &&
             system.iop_bus().read32(0x1F808408u, value) &&
             (value & 0x00800000u) == 0u &&
             (value & 0x55u) == 0x55u,
             "i.Link Bus ID reset bit did not self-clear") && ok;

    ok = expect(
             system.iop_bus().write32(0x1F808410u, 0u) &&
             system.iop_bus().read32(0x1F808410u, value) &&
             value == 0x8u,
             "i.Link SCLK ready bit was lost") && ok;

    // PHY read request must complete instead of leaving the read flag set.
    ok = expect(
             system.iop_bus().write32(0x1F808414u, 0x83000000u) &&
             system.iop_bus().read32(0x1F808414u, value) &&
             (value & 0x80000000u) == 0u &&
             (value & 0x00000F00u) == 0x00000300u,
             "i.Link PHY read request did not complete") && ok;

    return ok;
}

bool test_iop_absent_dev9_aperture() {
    ps2::Ps2System system;
    bool ok = true;

    ps2::u8 value8 = 0xFFu;
    ps2::u16 value16 = 0xFFFFu;
    ps2::u32 value32 = 0xFFFFFFFFu;

    ok = expect(
             system.iop_bus().read8(0x10000000u, value8) &&
             value8 == 0u,
             "absent DEV9 byte probe did not return zero") && ok;
    ok = expect(
             system.iop_bus().read16(0xB000146Eu, value16) &&
             value16 == 0u,
             "absent DEV9 KSEG1 halfword probe did not return zero") && ok;
    ok = expect(
             system.iop_bus().read32(0x10000040u, value32) &&
             value32 == 0u,
             "absent DEV9 word probe did not return zero") && ok;

    ok = expect(
             system.iop_bus().write8(0x10000000u, 0xAAu) &&
             system.iop_bus().write16(0x10000002u, 0x55AAu) &&
             system.iop_bus().write32(0x10000004u, 0x12345678u),
             "absent DEV9 probe writes faulted") && ok;
    ok = expect(
             system.iop_bus().read32(0x10000004u, value32) &&
             value32 == 0u,
             "absent DEV9 write unexpectedly created device state") && ok;

    return ok;
}

bool test_iop_uninstalled_peripheral_open_bus() {
    ps2::Ps2System system;
    bool ok = true;

    ps2::u8 value8 = 0xFFu;
    ps2::u16 value16 = 0xFFFFu;
    ps2::u32 value32 = 0xFFFFFFFFu;

    ok = expect(
             system.iop_bus().read8(0x1F000123u, value8) &&
             value8 == 0u,
             "IOP open-bus byte probe did not return zero") && ok;
    ok = expect(
             system.iop_bus().read16(0xBF000200u, value16) &&
             value16 == 0u,
             "IOP open-bus KSEG1 halfword probe did not return zero") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F500000u, value32) &&
             value32 == 0u,
             "IOP open-bus word probe did not return zero") && ok;

    ok = expect(
             system.iop_bus().write8(0x1F000123u, 0xAAu) &&
             system.iop_bus().write16(0x1F500002u, 0x55AAu) &&
             system.iop_bus().write32(0x1FA00000u, 0x12345678u),
             "IOP open-bus probe writes faulted") && ok;

    // The ROM0 window must remain outside permissive open-bus handling.
    value32 = 0;
    ok = expect(
             !system.iop_bus().read32(0x1FC00000u, value32),
             "unloaded ROM0 was incorrectly treated as open bus") && ok;

    return ok;
}

bool test_iop_dma6_ordering_table_clear() {
    ps2::Ps2System system;
    bool ok = true;

    constexpr ps2::u32 madr = 0x0000100Cu;
    constexpr ps2::u32 words = 4u;

    ok = expect(
             system.iop_bus().write32(
                 0x1F8010F4u,
                 (1u << 23) | (1u << (16u + 6u))),
             "failed to enable IOP DMA6 interrupt") && ok;
    ok = expect(
             system.iop_bus().write32(0x1F8010E0u, madr) &&
             system.iop_bus().write32(0x1F8010E4u, words) &&
             system.iop_bus().write32(0x1F8010E8u, 0x11000002u),
             "IOP DMA6 OTC programming failed") && ok;

    ps2::u32 value = 0;
    ok = expect(
             system.iop_ram().read32(0x100Cu, value) &&
             value == 0x00001008u,
             "IOP DMA6 first OTC link mismatch") && ok;
    ok = expect(
             system.iop_ram().read32(0x1008u, value) &&
             value == 0x00001004u,
             "IOP DMA6 second OTC link mismatch") && ok;
    ok = expect(
             system.iop_ram().read32(0x1004u, value) &&
             value == 0x00001000u,
             "IOP DMA6 third OTC link mismatch") && ok;
    ok = expect(
             system.iop_ram().read32(0x1000u, value) &&
             value == 0x00FFFFFFu,
             "IOP DMA6 OTC terminator mismatch") && ok;

    ok = expect(
             system.iop_bus().read32(0x1F8010E8u, value) &&
             (value & 0x01000000u) == 0u,
             "IOP DMA6 start bit did not clear") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F8010E4u, value) &&
             value == 0u,
             "IOP DMA6 BCR did not complete") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F8010F4u, value) &&
             (value & (1u << 30)) != 0u &&
             (value & 0x80000000u) != 0u,
             "IOP DMA6 completion flag missing") && ok;
    ok = expect(
             system.iop_bus().read32(ps2::IopIntc::kIStat, value) &&
             (value & (1u << 3)) != 0u,
             "IOP DMA6 did not raise DMA interrupt") && ok;

    return ok;
}

bool test_ee_lq_sq_silent_alignment() {
    ps2::Ps2System system;

    constexpr ps2::u32 code = 0x00002000u;
    constexpr ps2::u32 load_base = 0x00001000u;
    constexpr ps2::u64 load_lo = 0x8877665544332211ull;
    constexpr ps2::u64 load_hi = 0xFFEEDDCCBBAA0099ull;

    bool ok =
        expect(system.bus().write64(load_base, load_lo),
               "failed to seed LQ low half") &&
        expect(system.bus().write64(load_base + 8u, load_hi),
               "failed to seed LQ high half");

    // LQ r3, 0(r2). r2 deliberately points inside the 16-byte block.
    const ps2::u32 lq =
        (0x1Eu << 26) | (2u << 21) | (3u << 16);
    ok =
        expect(system.bus().write32(code, lq),
               "failed to install LQ instruction") &&
        ok;

    system.ee().reset(code);
    system.ee().state().gpr[2].lo = load_base + 7u;

    std::string error;
    ok =
        expect(system.ee().step(error),
               "LQ instruction failed") &&
        ok;
    ok =
        expect(system.ee().state().gpr[3].lo == load_lo,
               "LQ low 64-bit half mismatch") &&
        ok;
    ok =
        expect(system.ee().state().gpr[3].hi == load_hi,
               "LQ high 64-bit half mismatch") &&
        ok;

    constexpr ps2::u32 store_base = 0x00001100u;
    constexpr ps2::u64 store_lo = 0x0123456789ABCDEFull;
    constexpr ps2::u64 store_hi = 0x0FEDCBA987654321ull;

    const ps2::u32 sq =
        (0x1Fu << 26) | (2u << 21) | (4u << 16);
    ok =
        expect(system.bus().write32(code, sq),
               "failed to install SQ instruction") &&
        ok;

    system.ee().reset(code);
    system.ee().state().gpr[2].lo = store_base + 0x0Fu;
    system.ee().state().gpr[4].lo = store_lo;
    system.ee().state().gpr[4].hi = store_hi;

    ok =
        expect(system.ee().step(error),
               "SQ instruction failed") &&
        ok;

    ps2::u64 read_lo = 0;
    ps2::u64 read_hi = 0;
    ok =
        expect(system.bus().read64(store_base, read_lo) &&
                   read_lo == store_lo,
               "SQ low 64-bit half mismatch") &&
        ok;
    ok =
        expect(system.bus().read64(store_base + 8u, read_hi) &&
                   read_hi == store_hi,
               "SQ high 64-bit half mismatch") &&
        ok;

    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok = test_ram_little_endian() && ok;
    ok = test_scanout_skips_unchanged_vram() && ok;
    ok = test_gs_sprite_blend_reuses_destination() && ok;
    ok = test_gs_untextured_triangle_without_depth() && ok;
    ok = test_ram_aliases() && ok;
    ok = test_ram_bounds() && ok;
    ok = test_bios_idle_iteration_matches_ee_steps() && ok;
    ok = test_bios_loop_fast_paths_match_ee_steps() && ok;
    ok = test_dmac_running_mask() && ok;
    ok = test_ee_jit_matches_interpreter() && ok;
    ok = test_scheduler_ordering() && ok;
    ok = test_scheduler_single_step() && ok;
    ok = test_scheduler_cancel() && ok;
    ok = test_ee_reset_state() && ok;
    ok = test_bios_mapping_and_startup() && ok;
    ok = test_iop_reset_and_shared_ram() && ok;
    ok = test_iop_ram_mirror_boundary() && ok;
    ok = test_iop_optional_extension_rom_windows() && ok;
    ok = test_ee_iop_startup_interleave() && ok;
    ok = test_ee_sbus_iop_commands() && ok;
    ok = test_iop_gte_bootstrap_transfers() && ok;
    ok = test_iop_cache_isolation_blocks_ram_store() && ok;
    ok = test_ee_timer0_clock_sources() && ok;
    ok = test_iop_timer_progress_and_irq() && ok;
    ok = test_iop_bus_repeating_timer_irq() && ok;
    ok = test_cdvd_reset_status() && ok;
    ok = test_cdvd_iop_segment_mirror() && ok;
    ok = test_cdvd_scommand_result_fifo() && ok;
    ok = test_cdvd_config_scommands() && ok;
    ok = test_cdvd_mechacon_config_nvram() && ok;
    ok = test_iop_intc_registers() && ok;
    ok = test_iop_external_interrupt_exception() && ok;
    ok = test_cdvd_raises_iop_irq2() && ok;
    ok = test_iop_root_counters() && ok;
    ok = test_iop_event_free_tick_matches_regular_tick() && ok;
    ok = test_iop_spu2_register_window() && ok;
    ok = test_iop_spu2_dma_bootstrap_completion() && ok;
    ok = test_iop_sio2_minimal_transfer_status() && ok;
    ok = test_iop_sio2_dma_bootstrap_completion() && ok;
    ok = test_iop_ohci_bootstrap_reset() && ok;
    ok = test_iop_firewire_bootstrap_probes() && ok;
    ok = test_iop_absent_dev9_aperture() && ok;
    ok = test_iop_uninstalled_peripheral_open_bus() && ok;
    ok = test_iop_dma6_ordering_table_clear() && ok;
    ok = test_ee_scratchpad_dma_round_trip() && ok;
    ok = test_ee_ipu_dma_bootstrap_paths() && ok;
    ok = test_ee_lq_sq_silent_alignment() && ok;

    if (!ok) {
        return EXIT_FAILURE;
    }

    std::cout << "VibeStation PS2 scaffold tests passed.\n";
    return EXIT_SUCCESS;
}
