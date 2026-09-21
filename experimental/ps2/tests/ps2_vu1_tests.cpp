#include "core/ps2_system.h"

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
    ok = test_vif1_mscal_starts_vu1() && ok;
    if (!ok) return EXIT_FAILURE;
    std::cout << "VibeStation PS2 VU1 tests passed.\n";
    return EXIT_SUCCESS;
}
