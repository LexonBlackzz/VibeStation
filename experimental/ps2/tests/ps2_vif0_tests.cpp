#include "core/dma/vif0_dma.h"
#include "core/ps2_system.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

bool write_words(
    ps2::EeBus& bus,
    ps2::u32 address,
    ps2::u32 w0,
    ps2::u32 w1,
    ps2::u32 w2,
    ps2::u32 w3) {
    return bus.write32(address + 0u, w0) &&
           bus.write32(address + 4u, w1) &&
           bus.write32(address + 8u, w2) &&
           bus.write32(address + 12u, w3);
}

bool test_vif0_mpg_unpack_and_completion() {
    ps2::Ps2System system;
    ps2::Vif0Dma dma;
    dma.reset();

    constexpr ps2::u32 stream = 0x8000u;
    constexpr ps2::u32 mpg =
        (0x4Au << 24) | (1u << 16) | 2u;
    constexpr ps2::u32 stcycl =
        (0x01u << 24) | 0x0101u;
    constexpr ps2::u32 unpack_v4_32 =
        (0x6Cu << 24) | (1u << 16) | 3u;

    bool ok = true;
    ok = expect(
        write_words(
            system.bus(),
            stream,
            mpg,
            0x11223344u,
            0x55667788u,
            stcycl),
        "failed to build VIF0 MPG qword") && ok;
    ok = expect(
        write_words(
            system.bus(),
            stream + 0x10u,
            unpack_v4_32,
            0x11u,
            0x22u,
            0x33u),
        "failed to build VIF0 UNPACK qword") && ok;
    ok = expect(
        write_words(
            system.bus(),
            stream + 0x20u,
            0x44u,
            0u,
            0u,
            0u),
        "failed to build VIF0 UNPACK tail") && ok;

    ok = expect(
        system.bus().write32(0x1000E000u, 1u) &&
        system.bus().write32(0x10008010u, stream) &&
        system.bus().write32(0x10008020u, 3u) &&
        system.bus().write32(0x10008000u, 0x101u),
        "failed to arm VIF0 normal DMA") && ok;

    std::string error;
    for (ps2::u32 i = 0; i < 3u; ++i) {
        ok = expect(
            dma.service(system.bus(), error),
            "VIF0 normal DMA service failed") && ok;
        if (!error.empty()) std::cerr << error << '\n';
    }

    ps2::u32 value = 0;
    ok = expect(
        system.bus().read32(0x11000010u, value) &&
            value == 0x11223344u,
        "VIF0 MPG low micro word mismatch") && ok;
    ok = expect(
        system.bus().read32(0x11000014u, value) &&
            value == 0x55667788u,
        "VIF0 MPG high micro word mismatch") && ok;

    const ps2::u32 expected[4] = {0x11u, 0x22u, 0x33u, 0x44u};
    for (ps2::u32 lane = 0; lane < 4u; ++lane) {
        ok = expect(
            system.bus().read32(
                0x11004030u + lane * 4u,
                value) &&
                value == expected[lane],
            "VIF0 UNPACK lane mismatch") && ok;
    }

    ok = expect(
        system.bus().read32(0x10008000u, value) &&
            (value & 0x100u) == 0,
        "VIF0 DMA STR did not clear") && ok;
    ok = expect(
        system.bus().read32(0x1000E010u, value) &&
            (value & 1u) != 0,
        "VIF0 DMAC completion cause missing") && ok;
    ok = expect(
        system.bus().read32(0x10003800u, value) &&
            (value & 0x3u) == 0u &&
            ((value >> 24) & 0xFu) == 0u,
        "VIF0 STAT did not return idle") && ok;

    return ok;
}

bool test_vif0_mscal_waits_for_vu0() {
    ps2::Ps2System system;
    ps2::Vif0Dma dma;
    dma.reset();
    dma.attach_vu0(system.vu0());

    constexpr ps2::u32 stream = 0xA000u;
    constexpr ps2::u32 mpg =
        (0x4Au << 24) | (2u << 16);
    constexpr ps2::u32 iaddu_vi1_5 =
        (0x08u << 25) | (1u << 16) | 5u;
    constexpr ps2::u32 iaddu_vi2_7 =
        (0x08u << 25) | (2u << 16) | 7u;
    constexpr ps2::u32 end_flag = 0x40000000u;
    constexpr ps2::u32 mscal = 0x14u << 24;
    constexpr ps2::u32 flush = 0x11u << 24;
    constexpr ps2::u32 mark =
        (0x07u << 24) | 0x1234u;

    bool ok = true;
    ok = expect(
        write_words(
            system.bus(),
            stream,
            mpg,
            iaddu_vi1_5,
            end_flag,
            iaddu_vi2_7) &&
        write_words(
            system.bus(),
            stream + 0x10u,
            0u,
            mscal,
            flush,
            mark),
        "failed to build VIF0 MSCAL stream") && ok;

    ok = expect(
        system.bus().write32(0x1000E000u, 1u) &&
        system.bus().write32(0x10008010u, stream) &&
        system.bus().write32(0x10008020u, 2u) &&
        system.bus().write32(0x10008000u, 0x101u),
        "failed to arm VIF0 MSCAL DMA") && ok;

    std::string error;
    ok = expect(
        dma.service(system.bus(), error),
        "VIF0 MPG service failed") && ok;
    ok = expect(
        dma.service(system.bus(), error),
        "VIF0 MSCAL/FLUSH service failed") && ok;

    ps2::u32 stat = 0;
    ok = expect(
        system.vu0().running(),
        "VIF0 MSCAL did not start VU0") && ok;
    ok = expect(
        system.bus().read32(0x10003800u, stat) &&
            (stat & 0x3u) == 1u,
        "VIF0 FLUSH did not enter VU wait state") && ok;

    std::string vu_error;
    system.vu0().run(8u, vu_error);
    ok = expect(
        vu_error.empty() && !system.vu0().running() &&
            system.vu0().vi(1u) == 5u &&
            system.vu0().vi(2u) == 7u,
        "VIF0-started VU0 program did not retire") && ok;

    error.clear();
    ok = expect(
        dma.service(system.bus(), error),
        "VIF0 did not resume after VU0 completion") && ok;
    ps2::u32 value = 0;
    ok = expect(
        system.bus().read32(0x10003830u, value) &&
            value == 0x1234u,
        "VIF0 deferred MARK did not execute after FLUSH") && ok;
    ok = expect(
        system.bus().read32(0x10008000u, value) &&
            (value & 0x100u) == 0,
        "VIF0 DMA did not complete after VU0 wait") && ok;

    return ok;
}

bool test_vif0_chain_tte() {
    ps2::Ps2System system;
    ps2::Vif0Dma dma;
    dma.reset();

    constexpr ps2::u32 tag = 0x9000u;
    constexpr ps2::u32 tag0 =
        1u | (7u << 28); // END, one qword.
    constexpr ps2::u32 stcycl =
        (0x01u << 24) | 0x0202u;

    bool ok = expect(
        system.bus().write32(tag + 0u, tag0) &&
        system.bus().write32(tag + 4u, 0u) &&
        system.bus().write32(tag + 8u, stcycl) &&
        system.bus().write32(tag + 12u, 0u),
        "failed to build VIF0 chain tag");

    ok = expect(
        write_words(
            system.bus(),
            tag + 0x10u,
            (0x07u << 24) | 0xBEEFu,
            0u,
            0u,
            0u),
        "failed to build VIF0 chain payload") && ok;

    ok = expect(
        system.bus().write32(0x1000E000u, 1u) &&
        system.bus().write32(0x10008030u, tag) &&
        system.bus().write32(
            0x10008000u,
            0x101u | (1u << 2) | (1u << 6)),
        "failed to arm VIF0 chain DMA") && ok;

    std::string error;
    ok = expect(
        dma.service(system.bus(), error),
        "VIF0 chain service failed") && ok;
    if (!error.empty()) std::cerr << error << '\n';

    ps2::u32 value = 0;
    ok = expect(
        system.bus().read32(0x10003840u, value) &&
            value == 0x0202u,
        "VIF0 TTE STCYCL did not execute") && ok;
    ok = expect(
        system.bus().read32(0x10003830u, value) &&
            value == 0xBEEFu,
        "VIF0 chain MARK did not execute") && ok;
    ok = expect(
        system.bus().read32(0x10008000u, value) &&
            (value & 0x100u) == 0,
        "VIF0 chain did not complete") && ok;

    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok = test_vif0_mpg_unpack_and_completion() && ok;
    ok = test_vif0_chain_tte() && ok;
    ok = test_vif0_mscal_waits_for_vu0() && ok;
    if (!ok) return EXIT_FAILURE;
    std::cout << "VibeStation PS2 VIF0 tests passed.\n";
    return EXIT_SUCCESS;
}
