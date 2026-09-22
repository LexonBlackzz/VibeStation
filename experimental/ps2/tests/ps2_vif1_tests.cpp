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

bool setup_normal_dma(
    ps2::Ps2System& system,
    ps2::u32 address,
    ps2::u32 qwc) {
    return system.bus().write32(0x1000E000u, 1u) &&
           system.bus().write32(0x10009010u, address) &&
           system.bus().write32(0x10009020u, qwc) &&
           system.bus().write32(0x10009000u, 0x101u);
}

bool service_n(
    ps2::Vif1Dma& dma,
    ps2::Ps2System& system,
    ps2::u32 count,
    std::string& error) {
    for (ps2::u32 i = 0; i < count; ++i) {
        if (!dma.service(
                system.bus(),
                system.gs_core(),
                system.gs_privileged(),
                error)) {
            return false;
        }
    }
    return true;
}

bool test_vif1_mpg_and_unpack() {
    ps2::Ps2System system;
    ps2::Vif1Dma dma;
    dma.reset();

    constexpr ps2::u32 stream = 0x4000u;
    constexpr ps2::u32 mpg =
        (0x4Au << 24) | (1u << 16) | 2u;
    constexpr ps2::u32 stcycl =
        (0x01u << 24) | 0x0101u;
    constexpr ps2::u32 unpack_v4_32 =
        (0x6Cu << 24) | (1u << 16) | 0x4003u;

    bool ok = true;
    ok = expect(
             write_words(
                 system.bus(),
                 stream,
                 mpg,
                 0x11223344u,
                 0x55667788u,
                 stcycl),
             "failed to build VIF1 MPG packet") && ok;
    ok = expect(
             write_words(
                 system.bus(),
                 stream + 0x10u,
                 unpack_v4_32,
                 0x00000011u,
                 0x00000022u,
                 0x00000033u),
             "failed to build VIF1 UNPACK packet") && ok;
    ok = expect(
             write_words(
                 system.bus(),
                 stream + 0x20u,
                 0x00000044u,
                 0u,
                 0u,
                 0u),
             "failed to build VIF1 UNPACK tail") && ok;
    ok = expect(
             setup_normal_dma(system, stream, 3u),
             "failed to arm VIF1 normal DMA") && ok;

    std::string error;
    ok = expect(
             service_n(dma, system, 3u, error),
             "VIF1 MPG/UNPACK DMA service failed") && ok;
    if (!error.empty()) std::cerr << error << '\n';

    ps2::u32 value = 0;
    ok = expect(
             system.bus().read32(0x11008010u, value) &&
                 value == 0x11223344u,
             "VIF1 MPG low micro word mismatch") && ok;
    ok = expect(
             system.bus().read32(0x11008014u, value) &&
                 value == 0x55667788u,
             "VIF1 MPG high micro word mismatch") && ok;

    const ps2::u32 expected[4] = {
        0x11u, 0x22u, 0x33u, 0x44u,
    };
    for (ps2::u32 i = 0; i < 4u; ++i) {
        ok = expect(
                 system.bus().read32(
                     0x1100C030u + i * 4u,
                     value) &&
                     value == expected[i],
                 "VIF1 UNPACK V4-32 lane mismatch") && ok;
    }

    ok = expect(
             system.bus().read32(0x10003CE0u, value) && value == 0u,
             "VIF1 UNPACK incorrectly changed the MSCAL TOP register") && ok;

    ok = expect(
             system.bus().read32(0x10009000u, value) &&
                 (value & 0x100u) == 0,
             "VIF1 normal DMA STR did not clear") && ok;
    return ok;
}

bool test_vif1_direct_path2() {
    ps2::Ps2System system;
    ps2::Vif1Dma dma;
    dma.reset();

    constexpr ps2::u32 stream = 0x5000u;
    constexpr ps2::u32 direct_two_qwords =
        (0x50u << 24) | 2u;

    const ps2::u64 gif_tag =
        1ull | (1ull << 15) | (1ull << 60);
    constexpr ps2::u64 frame_value = 1ull << 16;

    bool ok = true;
    ok = expect(
             write_words(
                 system.bus(),
                 stream,
                 0u,
                 0u,
                 0u,
                 direct_two_qwords),
             "failed to build VIF1 DIRECT command qword") && ok;
    ok = expect(
             write_words(
                 system.bus(),
                 stream + 0x10u,
                 static_cast<ps2::u32>(gif_tag),
                 static_cast<ps2::u32>(gif_tag >> 32),
                 0xEu,
                 0u),
             "failed to build PATH2 GIF tag") && ok;
    ok = expect(
             write_words(
                 system.bus(),
                 stream + 0x20u,
                 static_cast<ps2::u32>(frame_value),
                 static_cast<ps2::u32>(frame_value >> 32),
                 0x4Cu,
                 0u),
             "failed to build PATH2 A+D payload") && ok;
    ok = expect(
             setup_normal_dma(system, stream, 3u),
             "failed to arm PATH2 VIF1 DMA") && ok;

    std::string error;
    ok = expect(
             service_n(dma, system, 3u, error),
             "VIF1 DIRECT DMA service failed") && ok;
    if (!error.empty()) std::cerr << error << '\n';

    ok = expect(
             system.gs_core().register_value(0x4Cu) == frame_value,
             "VIF1 DIRECT did not reach GIF/GS PATH2") && ok;

    ps2::u32 chcr = 0;
    ok = expect(
             system.bus().read32(0x10009000u, chcr) &&
                 (chcr & 0x100u) == 0,
             "PATH2 VIF1 DMA did not complete") && ok;
    return ok;
}

bool test_vif1_unpack_v4_5_expansion() {
    ps2::Ps2System system;
    ps2::Vif1Dma dma;
    dma.reset();

    constexpr ps2::u32 stream = 0x5600u;
    constexpr ps2::u32 stmod_addrow =
        (0x05u << 24) | 1u;
    constexpr ps2::u32 unpack_v4_5 =
        (0x6Fu << 24) | (1u << 16) | 0x0020u;
    constexpr ps2::u32 packed =
        31u | (1u << 5) | (2u << 10) | (1u << 15);

    bool ok = expect(
        write_words(
            system.bus(),
            stream,
            stmod_addrow,
            unpack_v4_5,
            packed,
            0u),
        "failed to build VIF1 V4-5 packet");
    ok = expect(
        setup_normal_dma(system, stream, 1u),
        "failed to arm VIF1 V4-5 DMA") && ok;

    std::string error;
    ok = expect(
        dma.service(
            system.bus(),
            system.gs_core(),
            system.gs_privileged(),
            error),
        "VIF1 V4-5 DMA service failed") && ok;
    if (!error.empty()) std::cerr << error << '\n';

    const ps2::u32 expected[4] = {
        31u << 3,
        1u << 3,
        2u << 3,
        1u << 7,
    };
    for (ps2::u32 lane = 0; lane < 4u; ++lane) {
        ps2::u32 value = 0;
        ok = expect(
            system.bus().read32(
                0x1100C000u + 0x20u * 16u + lane * 4u,
                value) &&
            value == expected[lane],
            "VIF1 V4-5 expansion/mode mismatch") && ok;
    }

    return ok;
}

bool test_vif1_status_tracks_payload_progress() {
    ps2::Ps2System system;
    ps2::Vif1Dma dma;
    dma.reset();

    constexpr ps2::u32 stream = 0x5800u;
    constexpr ps2::u32 strow = 0x30000000u;

    bool ok = expect(
        write_words(system.bus(), stream, 0u, 0u, 0u, strow) &&
        write_words(system.bus(), stream + 0x10u, 1u, 2u, 3u, 4u),
        "failed to build VIF1 status packet");
    ok = expect(setup_normal_dma(system, stream, 2u),
                "failed to arm VIF1 status DMA") && ok;

    std::string error;
    ok = expect(
        dma.service(
            system.bus(),
            system.gs_core(),
            system.gs_privileged(),
            error),
        "VIF1 status first service failed") && ok;

    ps2::u32 stat = 0;
    ok = expect(
        system.bus().read32(0x10003C00u, stat) &&
            (stat & 0x3u) == 3u,
        "VIF1 VPS did not expose payload transfer") && ok;
    ok = expect(
        ((stat >> 24) & 0x1Fu) == 1u,
        "VIF1 FQC did not track remaining DMA qword") && ok;

    ok = expect(
        dma.service(
            system.bus(),
            system.gs_core(),
            system.gs_privileged(),
            error),
        "VIF1 status second service failed") && ok;
    ok = expect(
        system.bus().read32(0x10003C00u, stat) &&
            (stat & 0x3u) == 0u &&
            ((stat >> 24) & 0x1Fu) == 0u,
        "VIF1 status did not return idle on completion") && ok;

    return ok;
}

bool test_vif1_flush_drains_vu1() {
    ps2::Ps2System system;
    ps2::Vif1Dma dma;
    dma.attach_vu1(system.vu1());
    dma.reset();

    // E-bit pair followed by the architectural delay-slot pair.
    bool ok = expect(
        system.bus().write32(0x11008000u, 0u) &&
        system.bus().write32(0x11008004u, 0x40000000u) &&
        system.bus().write32(0x11008008u, 0u) &&
        system.bus().write32(0x1100800Cu, 0u),
        "failed to build VU1 flush microprogram");
    system.vu1().start(0u);
    ok = expect(system.vu1().running(),
                "VU1 flush test did not start VU1") && ok;

    constexpr ps2::u32 stream = 0x5A00u;
    constexpr ps2::u32 flush = 0x11000000u;
    ok = expect(
        write_words(system.bus(), stream, flush, 0u, 0u, 0u),
        "failed to build VIF1 FLUSH packet") && ok;
    ok = expect(setup_normal_dma(system, stream, 1u),
                "failed to arm VIF1 FLUSH DMA") && ok;

    std::string error;
    ok = expect(
        dma.service(
            system.bus(),
            system.gs_core(),
            system.gs_privileged(),
            error),
        "VIF1 FLUSH service failed") && ok;
    if (!error.empty()) std::cerr << error << '\n';

    ps2::u32 stat = 0;
    ok = expect(system.vu1().running(),
                "VIF1 FLUSH should wait while VU1 is active") && ok;
    ok = expect(
        system.bus().read32(0x10003C00u, stat) &&
            (stat & 0x3u) == 1u,
        "VIF1 FLUSH did not expose VPS waiting") && ok;

    ok = expect(
        system.vu1().run(8u, error) == 2u &&
            !system.vu1().running(),
        "VU1 flush microprogram did not terminate") && ok;
    ok = expect(
        dma.service(
            system.bus(),
            system.gs_core(),
            system.gs_privileged(),
            error),
        "VIF1 FLUSH release service failed") && ok;

    ok = expect(
        system.bus().read32(0x10003C00u, stat) &&
            (stat & 0xFu) == 0u &&
            ((stat >> 24) & 0x1Fu) == 0u,
        "VIF1 FLUSH left busy/wait/FQC status set after release") && ok;

    return ok;
}

bool test_vif1_flush_waits_for_vu1() {
    ps2::Ps2System system;
    ps2::Vif1Dma dma;
    dma.attach_vu1(system.vu1());
    dma.reset();

    // A two-pair VU program: E is observed on pair 0 and pair 1 is its
    // mandatory delay slot.
    bool ok = true;
    ok = expect(
             system.bus().write32(0x11008000u, 0u) &&
                 system.bus().write32(0x11008004u, 0x40000000u) &&
                 system.bus().write32(0x11008008u, 0u) &&
                 system.bus().write32(0x1100800Cu, 0u),
             "failed to install VU1 FLUSH test program") && ok;

    constexpr ps2::u32 stream = 0x5800u;
    constexpr ps2::u32 mscal = (0x14u << 24);
    constexpr ps2::u32 flush = (0x11u << 24);
    constexpr ps2::u32 mark = (0x07u << 24) | 0x1234u;

    ok = expect(
             write_words(
                 system.bus(),
                 stream,
                 mscal,
                 flush,
                 mark,
                 0u),
             "failed to build VIF1 MSCAL/FLUSH packet") && ok;
    ok = expect(
             setup_normal_dma(system, stream, 1u),
             "failed to arm VIF1 FLUSH DMA") && ok;

    std::string error;
    ok = expect(
             dma.service(
                 system.bus(),
                 system.gs_core(),
                 system.gs_privileged(),
                 error),
             "VIF1 FLUSH first service failed") && ok;
    if (!error.empty()) std::cerr << error << '\n';

    ps2::u32 value = 0;
    ok = expect(
             system.vu1().running(),
             "MSCAL did not leave VU1 running at FLUSH") && ok;
    ok = expect(
             system.bus().read32(0x10009000u, value) &&
                 (value & 0x100u) != 0,
             "VIF1 DMA completed before FLUSH released") && ok;
    ok = expect(
             system.bus().read32(0x10003C00u, value) &&
                 (value & 0x3u) == 1u,
             "VIF1 STAT did not expose VPS waiting") && ok;
    ok = expect(
             system.bus().read32(0x10003C30u, value) &&
                 value == 0u,
             "deferred MARK executed before VU1 completed") && ok;

    ok = expect(
             system.vu1().run(8u, error) == 2u &&
                 !system.vu1().running(),
             "VU1 FLUSH test program did not terminate") && ok;
    ok = expect(
             dma.service(
                 system.bus(),
                 system.gs_core(),
                 system.gs_privileged(),
                 error),
             "VIF1 FLUSH release service failed") && ok;

    ok = expect(
             system.bus().read32(0x10003C30u, value) &&
                 value == 0x1234u,
             "deferred MARK was not decoded after FLUSH") && ok;
    ok = expect(
             system.bus().read32(0x10003C00u, value) &&
                 (value & 0x3u) == 0u &&
                 (value & (1u << 6)) != 0,
             "VIF1 STAT did not return idle with MRK latched") && ok;
    ok = expect(
             system.bus().read32(0x10009000u, value) &&
                 (value & 0x100u) == 0u,
             "VIF1 FLUSH DMA did not complete after VU1") && ok;
    return ok;
}

bool test_vif1_source_chain_tte() {
    ps2::Ps2System system;
    ps2::Vif1Dma dma;
    dma.reset();

    constexpr ps2::u32 tag_address = 0x6000u;
    constexpr ps2::u32 tag0 =
        1u | (7u << 28); // END, one payload qword.
    constexpr ps2::u32 stcycl =
        (0x01u << 24) | 0x0202u;

    bool ok = true;
    ok = expect(
             system.bus().write32(tag_address + 0u, tag0) &&
                 system.bus().write32(tag_address + 4u, 0u) &&
                 system.bus().write32(tag_address + 8u, stcycl) &&
                 system.bus().write32(tag_address + 12u, 0u),
             "failed to build VIF1 source-chain tag") && ok;
    ok = expect(
             write_words(
                 system.bus(),
                 tag_address + 0x10u,
                 (0x07u << 24) | 0x55AAu,
                 0u,
                 0u,
                 0u),
             "failed to build VIF1 source-chain payload") && ok;

    ok = expect(
             system.bus().write32(0x1000E000u, 1u) &&
                 system.bus().write32(0x10009030u, tag_address) &&
                 system.bus().write32(
                     0x10009000u,
                     0x101u | (1u << 2) | (1u << 6)),
             "failed to arm VIF1 source-chain DMA") && ok;

    std::string error;
    ok = expect(
             dma.service(
                 system.bus(),
                 system.gs_core(),
                 system.gs_privileged(),
                 error),
             "VIF1 source-chain service failed") && ok;
    if (!error.empty()) std::cerr << error << '\n';

    ps2::u32 value = 0;
    ok = expect(
             system.bus().read32(0x10003C40u, value) &&
                 value == 0x0202u,
             "VIF1 TTE did not decode tag VIFcode") && ok;
    ok = expect(
             system.bus().read32(0x10003C30u, value) &&
                 value == 0x55AAu,
             "VIF1 chain payload command did not execute") && ok;
    ok = expect(
             system.bus().read32(0x10009000u, value) &&
                 (value & 0x100u) == 0,
             "VIF1 source chain did not complete") && ok;
    return ok;
}

bool test_vif1_source_chain_scratchpad_tadr() {
    ps2::Ps2System system;
    ps2::Vif1Dma dma;
    dma.reset();

    constexpr ps2::u32 scratchpad_address = 0x70002290u;
    constexpr ps2::u32 dmac_tadr = 0x80002290u;
    constexpr ps2::u32 tag0 = 1u | (7u << 28); // END, one payload qword.
    constexpr ps2::u32 stcycl = (0x01u << 24) | 0x0202u;
    bool ok = true;
    ok = expect(
             system.bus().write32(scratchpad_address + 0u, tag0) &&
                 system.bus().write32(scratchpad_address + 4u, 0u) &&
                 system.bus().write32(scratchpad_address + 8u, stcycl) &&
                 system.bus().write32(scratchpad_address + 12u, 0u) &&
                 write_words(
                     system.bus(),
                     scratchpad_address + 0x10u,
                     (0x07u << 24) | 0x55AAu,
                     0u,
                     0u,
                     0u),
             "failed to build scratchpad VIF1 source chain") && ok;

    ok = expect(
             system.bus().write32(0x1000E000u, 1u) &&
                 system.bus().write32(0x10009030u, dmac_tadr) &&
                 system.bus().write32(
                     0x10009000u,
                     0x101u | (1u << 2) | (1u << 6)),
             "failed to arm scratchpad VIF1 source chain") && ok;

    std::string error;
    ok = expect(
             dma.service(
                 system.bus(),
                 system.gs_core(),
                 system.gs_privileged(),
                 error),
             "scratchpad VIF1 source-chain service failed") && ok;
    if (!error.empty()) std::cerr << error << '\n';

    ps2::u32 value = 0;
    ok = expect(
             system.bus().read32(0x10003C40u, value) &&
                 value == 0x0202u,
             "scratchpad VIF1 tag command was not decoded") && ok;
    ok = expect(
             system.bus().read32(0x10003C30u, value) &&
                 value == 0x55AAu,
             "scratchpad VIF1 payload command was not decoded") && ok;
    ok = expect(
             system.bus().read32(0x10009000u, value) &&
                 (value & 0x100u) == 0u,
             "scratchpad VIF1 source chain did not complete") && ok;
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok = test_vif1_mpg_and_unpack() && ok;
    ok = test_vif1_direct_path2() && ok;
    ok = test_vif1_flush_waits_for_vu1() && ok;
    ok = test_vif1_unpack_v4_5_expansion() && ok;
    ok = test_vif1_status_tracks_payload_progress() && ok;
    ok = test_vif1_flush_drains_vu1() && ok;
    ok = test_vif1_source_chain_tte() && ok;
    ok = test_vif1_source_chain_scratchpad_tadr() && ok;
    if (!ok) return EXIT_FAILURE;
    std::cout << "VibeStation PS2 VIF1 tests passed.\n";
    return EXIT_SUCCESS;
}
