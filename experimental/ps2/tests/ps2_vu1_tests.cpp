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

bool write_micro_pair(
    ps2::Ps2System& system,
    ps2::u32 instruction,
    ps2::u32 lower,
    ps2::u32 upper) {
    const ps2::u32 address =
        0x11008000u + (instruction & 0x7FFu) * 8u;
    return system.bus().write32(address, lower) &&
           system.bus().write32(address + 4u, upper);
}

bool write_qword(
    ps2::Ps2System& system,
    ps2::u32 qword,
    ps2::u64 lo,
    ps2::u64 hi) {
    const ps2::u32 address =
        0x1100C000u + (qword & 0x3FFu) * 16u;
    return system.bus().write64(address, lo) &&
           system.bus().write64(address + 8u, hi);
}

bool test_xgkick_to_gs() {
    ps2::Ps2System system;

    // VI1 = 0x10.
    constexpr ps2::u32 iaddu_vi1 =
        (0x08u << 25) | (1u << 16) | 0x10u;

    // LowerOP / T3_00 / XGKICK, using VI1 as the qword address.
    constexpr ps2::u32 xgkick_vi1 =
        (0x40u << 25) |
        (1u << 11) |
        (0x1Bu << 6) |
        0x3Cu;

    // E flag on the XGKICK pair. One delay-slot pair follows it.
    constexpr ps2::u32 end_flag = 0x40000000u;

    bool ok = true;
    ok = expect(
             write_micro_pair(system, 0u, iaddu_vi1, 0u),
             "failed to write VU1 IADDIU pair") && ok;
    ok = expect(
             write_micro_pair(system, 1u, xgkick_vi1, end_flag),
             "failed to write VU1 XGKICK/E pair") && ok;
    ok = expect(
             write_micro_pair(system, 2u, 0u, 0u),
             "failed to write VU1 E delay-slot pair") && ok;

    const ps2::u64 tag =
        1ull |
        (1ull << 15) |
        (1ull << 60);
    constexpr ps2::u64 ad_descriptor = 0xEull;
    constexpr ps2::u64 frame_value =
        (1ull << 16) | 0x20ull;

    ok = expect(
             write_qword(system, 0x10u, tag, ad_descriptor),
             "failed to write VU1 GIF tag") && ok;
    ok = expect(
             write_qword(system, 0x11u, frame_value, 0x4Cull),
             "failed to write VU1 GIF A+D payload") && ok;

    system.vu1().start(0);
    std::string error;
    const ps2::u64 executed = system.vu1().run(16, error);

    ok = expect(error.empty(), "VU1 execution returned an error") && ok;
    ok = expect(executed == 3u, "VU1 E delay-slot length mismatch") && ok;
    ok = expect(!system.vu1().running(), "VU1 did not stop at E") && ok;
    ok = expect(
             system.vu1().vi(1) == 0x10u,
             "VU1 IADDIU did not initialize VI1") && ok;
    ok = expect(
             system.gs_core().register_value(0x4Cu) == frame_value,
             "VU1 XGKICK did not reach the GS A+D path") && ok;
    ok = expect(
             system.vu1().stats().xgkicks == 1u,
             "VU1 XGKICK statistic mismatch") && ok;
    return ok;
}

bool test_vu1_fmac_flags() {
    ps2::Ps2System system;

    const ps2::u32 a[4] = {
        std::bit_cast<ps2::u32>(-1.0f),
        std::bit_cast<ps2::u32>(2.0f),
        0x00000001u,
        0x7F7FFFFFu,
    };
    const ps2::u32 b[4] = {
        std::bit_cast<ps2::u32>(0.0f),
        std::bit_cast<ps2::u32>(-2.0f),
        0u,
        0x7F7FFFFFu,
    };

    bool ok = true;
    for (ps2::u32 lane = 0; lane < 4u; ++lane) {
        ok = expect(
                 system.bus().write32(
                     0x1100C000u + lane * 4u,
                     a[lane]) &&
                 system.bus().write32(
                     0x1100C010u + lane * 4u,
                     b[lane]),
                 "failed to seed VU1 FMAC vectors") && ok;
    }

    constexpr ps2::u32 lq_vf1 =
        (0xFu << 21) | (1u << 16);
    constexpr ps2::u32 lq_vf2 =
        (0xFu << 21) | (2u << 16) | 1u;
    constexpr ps2::u32 add_vf3 =
        (0xFu << 21) |
        (2u << 16) |
        (1u << 11) |
        (3u << 6) |
        0x28u;

    ok = expect(write_micro_pair(system, 0u, lq_vf1, 0u),
                "failed to write FMAC LQ vf1") && ok;
    ok = expect(write_micro_pair(system, 1u, lq_vf2, 0u),
                "failed to write FMAC LQ vf2") && ok;
    ok = expect(write_micro_pair(system, 2u, 0u, add_vf3),
                "failed to write FMAC ADD") && ok;

    system.vu1().start(0u);
    std::string error;
    ok = expect(system.vu1().run(3u, error) == 3u &&
                    error.empty(),
                "VU1 FMAC flag program failed") && ok;

    // X sign, Y zero, Z underflow+zero, W overflow.
    ok = expect(system.vu1().mac() == 0x1286u,
                "VU1 MAC flag layout mismatch") && ok;
    ok = expect((system.vu1().status() & 0x3CFu) == 0x3CFu,
                "VU1 STATUS current/sticky flags mismatch") && ok;

    ok = expect(
             system.vu1().vf(3u, 0u) ==
                 std::bit_cast<ps2::u32>(-1.0f) &&
             system.vu1().vf(3u, 1u) == 0u &&
             system.vu1().vf(3u, 2u) == 0u &&
             system.vu1().vf(3u, 3u) == 0x7F7FFFFFu,
             "VU1 FMAC result normalization mismatch") && ok;

    return ok;
}

bool test_vu1_efu_and_random_ops() {
    ps2::Ps2System system;

    bool ok = true;
    const ps2::u32 source_words[4] = {
        std::bit_cast<ps2::u32>(1.0f),
        std::bit_cast<ps2::u32>(2.0f),
        std::bit_cast<ps2::u32>(3.0f),
        std::bit_cast<ps2::u32>(4.0f),
    };
    for (ps2::u32 lane = 0; lane < 4u; ++lane) {
        ok = expect(
                 system.bus().write32(
                     0x1100C000u + lane * 4u,
                     source_words[lane]),
                 "failed to seed VU1 EFU vector") && ok;
    }

    // LQ.xyzw vf1, 0(vi0)
    constexpr ps2::u32 lq_vf1 =
        (0xFu << 21) | (1u << 16);
    // ESADD vf1 -> P (T3_00 index 0x1c)
    constexpr ps2::u32 esadd_vf1 =
        (0x40u << 25) |
        (1u << 11) |
        (0x1Cu << 6) |
        0x3Cu;
    // RINIT from vf1.x (T3_10 index 0x10)
    constexpr ps2::u32 rinit_vf1x =
        (0x40u << 25) |
        (1u << 11) |
        (0x10u << 6) |
        0x3Eu;
    // RGET.xyzw -> vf2 (T3_01 index 0x10)
    constexpr ps2::u32 rget_vf2 =
        (0x40u << 25) |
        (0xFu << 21) |
        (2u << 16) |
        (0x10u << 6) |
        0x3Du;
    // RNEXT.xyzw -> vf3 (T3_00 index 0x10)
    constexpr ps2::u32 rnext_vf3 =
        (0x40u << 25) |
        (0xFu << 21) |
        (3u << 16) |
        (0x10u << 6) |
        0x3Cu;

    ok = expect(write_micro_pair(system, 0u, lq_vf1, 0u),
                "failed to write VU1 EFU LQ") && ok;
    ok = expect(write_micro_pair(system, 1u, esadd_vf1, 0u),
                "failed to write VU1 ESADD") && ok;
    ok = expect(write_micro_pair(system, 2u, rinit_vf1x, 0u),
                "failed to write VU1 RINIT") && ok;
    ok = expect(write_micro_pair(system, 3u, rget_vf2, 0u),
                "failed to write VU1 RGET") && ok;
    ok = expect(write_micro_pair(system, 4u, rnext_vf3, 0u),
                "failed to write VU1 RNEXT") && ok;

    system.vu1().start(0u);
    std::string error;
    const ps2::u64 executed = system.vu1().run(5u, error);
    ok = expect(error.empty() && executed == 5u,
                "VU1 EFU/random microprogram failed") && ok;

    ok = expect(
             system.vu1().p() == std::bit_cast<ps2::u32>(14.0f),
             "VU1 ESADD P result mismatch") && ok;

    const ps2::u32 initial_random =
        0x3F800000u | (source_words[0] & 0x007FFFFFu);
    ok = expect(
             system.vu1().vf(2u, 0u) == initial_random &&
             system.vu1().vf(2u, 1u) == initial_random &&
             system.vu1().vf(2u, 2u) == initial_random &&
             system.vu1().vf(2u, 3u) == initial_random,
             "VU1 RGET lanes mismatch") && ok;

    ps2::u32 expected_next = initial_random;
    const ps2::u32 x = (expected_next >> 4) & 1u;
    const ps2::u32 y = (expected_next >> 22) & 1u;
    expected_next <<= 1u;
    expected_next ^= x ^ y;
    expected_next =
        (expected_next & 0x007FFFFFu) | 0x3F800000u;

    ok = expect(
             system.vu1().random() == expected_next &&
             system.vu1().vf(3u, 0u) == expected_next,
             "VU1 RNEXT result mismatch") && ok;
    ok = expect(
             system.vu1().stats().unsupported_lower == 0u,
             "VU1 EFU/random ops counted as unsupported") && ok;

    return ok;
}

bool test_vif1_mscal_starts_vu1() {
    ps2::Ps2System system;
    ps2::Vif1Dma dma;
    dma.attach_vu1(system.vu1());
    dma.reset();

    constexpr ps2::u32 stream = 0x7000u;
    constexpr ps2::u32 mscal =
        (0x14u << 24) | 0x404u;

    bool ok = true;
    ok = expect(
             system.bus().write32(stream + 0u, mscal) &&
                 system.bus().write32(stream + 4u, 0u) &&
                 system.bus().write32(stream + 8u, 0u) &&
                 system.bus().write32(stream + 12u, 0u),
             "failed to build VIF1 MSCAL packet") && ok;
    ok = expect(
             system.bus().write32(0x1000E000u, 1u) &&
                 system.bus().write32(0x10009010u, stream) &&
                 system.bus().write32(0x10009020u, 1u) &&
                 system.bus().write32(0x10009000u, 0x101u),
             "failed to arm VIF1 MSCAL DMA") && ok;

    std::string error;
    ok = expect(
             dma.service(
                 system.bus(),
                 system.gs_core(),
                 system.gs_privileged(),
                 error),
             "VIF1 MSCAL DMA service failed") && ok;
    ok = expect(error.empty(), "VIF1 MSCAL returned an error") && ok;
    ok = expect(system.vu1().running(), "MSCAL did not start VU1") && ok;
    ok = expect(system.vu1().pc() == 0x2020u, "MSCAL 11-bit start PC mismatch") && ok;
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok = test_xgkick_to_gs() && ok;
    ok = test_vu1_fmac_flags() && ok;
    ok = test_vu1_efu_and_random_ops() && ok;
    ok = test_vif1_mscal_starts_vu1() && ok;
    if (!ok) return EXIT_FAILURE;
    std::cout << "VibeStation PS2 VU1 tests passed.\n";
    return EXIT_SUCCESS;
}
