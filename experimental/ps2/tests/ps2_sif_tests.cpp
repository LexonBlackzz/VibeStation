#include "core/ps2_system.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

bool write_qword_words(
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

bool test_sif1_ee_to_iop() {
    ps2::Ps2System system;
    ps2::SifDma dma;
    dma.reset();

    constexpr ps2::u32 ee_tag = 0x8000u;
    constexpr ps2::u32 iop_destination = 0x00004000u;

    // EE source-chain END tag containing two qwords:
    //  - an IOP destination tag
    //  - one qword of payload
    constexpr ps2::u32 ee_tag0 =
        2u | (7u << 28);

    bool ok = true;
    ok = expect(
             write_qword_words(
                 system.bus(),
                 ee_tag,
                 ee_tag0,
                 0u,
                 0u,
                 0u),
             "failed to build SIF1 EE source tag") && ok;

    // IOP destination tag: ID bit 2 marks end, four payload words.
    ok = expect(
             write_qword_words(
                 system.bus(),
                 ee_tag + 0x10u,
                 0x40000000u | iop_destination,
                 4u,
                 0u,
                 0u),
             "failed to build SIF1 IOP destination tag") && ok;

    ok = expect(
             write_qword_words(
                 system.bus(),
                 ee_tag + 0x20u,
                 0x11223344u,
                 0x55667788u,
                 0x99AABBCCu,
                 0xDDEEFF00u),
             "failed to build SIF1 payload") && ok;

    ok = expect(
             system.bus().write32(0x1000E000u, 1u) &&
             system.bus().write32(0x1000C430u, ee_tag) &&
             system.bus().write32(
                 0x1000C400u,
                 0x100u | (1u << 2)) &&
             system.iop_bus().write32(
                 0x1F801538u,
                 0x01000000u),
             "failed to arm SIF1 DMA") && ok;

    std::string error;
    ok = expect(
             dma.service(
                 system.bus(),
                 system.iop_bus(),
                 system.iop_intc(),
                 error),
             "SIF1 DMA service failed") && ok;
    if (!error.empty()) std::cerr << error << '\n';

    const ps2::u32 expected[4] = {
        0x11223344u,
        0x55667788u,
        0x99AABBCCu,
        0xDDEEFF00u,
    };
    for (ps2::u32 i = 0; i < 4u; ++i) {
        ps2::u32 value = 0;
        ok = expect(
                 system.iop_bus().read32(
                     iop_destination + i * 4u,
                     value) &&
                 value == expected[i],
                 "SIF1 IOP payload mismatch") && ok;
    }

    ps2::u32 value = 0;
    ok = expect(
             system.bus().read32(0x1000C400u, value) &&
             (value & 0x100u) == 0,
             "SIF1 EE STR did not clear") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801538u, value) &&
             (value & 0x01000000u) != 0,
             "SIF1 IOP DMA completed without transfer latency") && ok;
    ok = expect(
             (system.iop_intc().status() & (1u << 3)) == 0,
             "SIF1 raised its IOP interrupt without transfer latency") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801574u, value) &&
             (value & (1u << 27)) == 0,
             "SIF1 set its DMA10 flag without transfer latency") && ok;

    dma.tick_iop(system.iop_bus());
    ok = expect(
             system.iop_bus().read32(0x1F801538u, value) &&
             (value & 0x01000000u) == 0,
             "SIF1 IOP DMA start did not clear after transfer latency") && ok;
    ok = expect(
             (system.iop_intc().status() & (1u << 3)) != 0,
             "SIF1 did not raise its delayed IOP DMA interrupt") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801574u, value) &&
             (value & (1u << 27)) != 0,
             "SIF1 delayed DMA10 DICR2 completion flag missing") && ok;
    ok = expect(
             system.iop_bus().write32(0x1F801574u, 1u << 27) &&
             system.iop_bus().read32(0x1F801574u, value) &&
             (value & (1u << 27)) == 0,
             "SIF1 DMA10 DICR2 flag did not acknowledge") && ok;

    return ok;
}

bool test_sif0_iop_to_ee() {
    ps2::Ps2System system;
    ps2::SifDma dma;
    dma.reset();

    constexpr ps2::u32 iop_tag = 0x00005000u;
    constexpr ps2::u32 iop_source = 0x00006000u;
    constexpr ps2::u32 ee_destination = 0x00009000u;

    bool ok = true;

    // IOP SIF0 tag: source address/termination + word count followed by
    // the EE destination-chain tag in the upper 64 bits.
    ok = expect(
             system.iop_bus().write32(
                 iop_tag + 0u,
                 0x40000000u | iop_source) &&
             system.iop_bus().write32(iop_tag + 4u, 4u) &&
             system.iop_bus().write32(
                 iop_tag + 8u,
                 1u | (7u << 28)) &&
             system.iop_bus().write32(
                 iop_tag + 12u,
                 ee_destination),
             "failed to build SIF0 IOP source tag") && ok;

    const ps2::u32 expected[4] = {
        0xCAFEBABEu,
        0x01234567u,
        0x89ABCDEFu,
        0x0BADF00Du,
    };
    for (ps2::u32 i = 0; i < 4u; ++i) {
        ok = expect(
                 system.iop_bus().write32(
                     iop_source + i * 4u,
                     expected[i]),
                 "failed to build SIF0 IOP payload") && ok;
    }

    ok = expect(
             system.bus().write32(0x1000E000u, 1u) &&
             system.bus().write32(
                 0x1000C000u,
                 0x100u | (1u << 2)) &&
             system.iop_bus().write32(0x1F80152Cu, iop_tag) &&
             system.iop_bus().write32(
                 0x1F801528u,
                 0x01000000u),
             "failed to arm SIF0 DMA") && ok;

    std::string error;
    ok = expect(
             dma.service(
                 system.bus(),
                 system.iop_bus(),
                 system.iop_intc(),
                 error),
             "SIF0 DMA service failed") && ok;
    if (!error.empty()) std::cerr << error << '\n';

    for (ps2::u32 i = 0; i < 4u; ++i) {
        ps2::u32 value = 0;
        ok = expect(
                 system.bus().read32(
                     ee_destination + i * 4u,
                     value) &&
                 value == expected[i],
                 "SIF0 EE payload mismatch") && ok;
    }

    ps2::u32 value = 0;
    ok = expect(
             system.bus().read32(0x1000C000u, value) &&
             (value & 0x100u) != 0,
             "SIF0 EE DMA completed without transfer latency") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801528u, value) &&
             (value & 0x01000000u) != 0,
             "SIF0 IOP DMA completed in the transfer instruction") && ok;
    ok = expect(
             (system.iop_intc().status() & (1u << 3)) == 0,
             "SIF0 raised its IOP interrupt in the transfer instruction") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801574u, value) &&
             (value & (1u << 26)) == 0,
             "SIF0 set its DMA9 flag in the transfer instruction") && ok;

    dma.tick_ee(system.bus());
    for (ps2::u32 i = 0; i < 4u; ++i) {
        dma.tick_iop(system.iop_bus());
    }
    ok = expect(
             system.bus().read32(0x1000C000u, value) &&
             (value & 0x100u) == 0,
             "SIF0 EE STR did not clear after transfer latency") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801528u, value) &&
             (value & 0x01000000u) == 0,
             "SIF0 IOP DMA start did not clear on the next IOP cycle") && ok;
    ok = expect(
             (system.iop_intc().status() & (1u << 3)) != 0,
             "SIF0 did not raise its deferred IOP DMA interrupt") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801574u, value) &&
             (value & (1u << 26)) != 0,
             "SIF0 deferred DMA9 DICR2 completion flag missing") && ok;
    ok = expect(
             system.iop_bus().write32(0x1F801574u, 1u << 26) &&
             system.iop_bus().read32(0x1F801574u, value) &&
             (value & (1u << 26)) == 0,
             "SIF0 DMA9 DICR2 flag did not acknowledge") && ok;

    return ok;
}

bool test_sif0_completes_each_side_independently() {
    ps2::Ps2System system;
    ps2::SifDma dma;
    dma.reset();

    constexpr ps2::u32 iop_tag = 0x00005000u;
    constexpr ps2::u32 first_source = 0x00006000u;
    constexpr ps2::u32 second_source = 0x00006100u;
    constexpr ps2::u32 first_destination = 0x00009000u;
    constexpr ps2::u32 second_destination = 0x00009100u;

    bool ok = true;
    auto write_tag = [&](ps2::u32 tag_address,
                         ps2::u32 source,
                         ps2::u32 ee_tag,
                         ps2::u32 destination) {
        return system.iop_bus().write32(
                   tag_address + 0u,
                   0x80000000u | source) &&
               system.iop_bus().write32(tag_address + 4u, 4u) &&
               system.iop_bus().write32(tag_address + 8u, ee_tag) &&
               system.iop_bus().write32(
                   tag_address + 12u,
                   destination);
    };

    // The first IOP tag interrupts and ends that side, but the EE CNT tag
    // deliberately leaves the destination chain running.
    ok = expect(
             write_tag(
                 iop_tag,
                 first_source,
                 1u | (1u << 28),
                 first_destination) &&
             system.iop_bus().write32(first_source, 0x11111111u) &&
             system.iop_bus().write32(first_source + 4u, 0x22222222u) &&
             system.iop_bus().write32(first_source + 8u, 0x33333333u) &&
             system.iop_bus().write32(first_source + 12u, 0x44444444u),
             "failed to build split-completion SIF0 packet") && ok;

    ok = expect(
             system.bus().write32(0x1000E000u, 1u) &&
             system.bus().write32(
                 0x1000C000u,
                 0x100u | (1u << 2)) &&
             system.iop_bus().write32(0x1F80152Cu, iop_tag) &&
             system.iop_bus().write32(
                 0x1F801528u,
                 0x01000000u),
             "failed to arm split-completion SIF0 DMA") && ok;

    std::string error;
    ok = expect(
             dma.service(
                 system.bus(),
                 system.iop_bus(),
                 system.iop_intc(),
                 error),
             "split-completion SIF0 service failed") && ok;

    ps2::u32 value = 0;
    ok = expect(
             system.bus().read32(0x1000C000u, value) &&
             (value & 0x100u) != 0,
             "SIF0 ended EE chain with only the IOP end tag") && ok;
    ok = expect(
             system.iop_bus().read32(0x1F801528u, value) &&
             (value & 0x01000000u) != 0,
             "SIF0 ended the IOP chain in the transfer instruction") && ok;
    ok = expect(
             system.bus().read32(0x1000E010u, value) &&
             (value & (1u << 5)) == 0,
             "SIF0 raised an early EE completion interrupt") && ok;

    for (ps2::u32 i = 0; i < 4u; ++i) {
        dma.tick_iop(system.iop_bus());
    }
    ok = expect(
             system.iop_bus().read32(0x1F801528u, value) &&
             (value & 0x01000000u) == 0,
             "SIF0 did not end the IOP chain on the next IOP cycle") && ok;

    // Rearming only the IOP side must continue into the still-active EE chain.
    ok = expect(
             write_tag(
                 iop_tag + 0x10u,
                 second_source,
                 1u | (7u << 28),
                 second_destination) &&
             system.iop_bus().write32(second_source, 0xAAAAAAAAu) &&
             system.iop_bus().write32(second_source + 4u, 0xBBBBBBBBu) &&
             system.iop_bus().write32(second_source + 8u, 0xCCCCCCCCu) &&
             system.iop_bus().write32(second_source + 12u, 0xDDDDDDDDu) &&
             system.iop_bus().write32(
                 0x1F801528u,
                 0x01000000u),
             "failed to rearm the IOP SIF0 side") && ok;
    ok = expect(
             dma.service(
                 system.bus(),
                 system.iop_bus(),
                 system.iop_intc(),
                 error),
             "continued SIF0 service failed") && ok;
    ok = expect(
             system.bus().read32(second_destination, value) &&
             value == 0xAAAAAAAAu,
             "continued SIF0 payload mismatch") && ok;
    ok = expect(
             system.bus().read32(0x1000C000u, value) &&
             (value & 0x100u) != 0,
             "SIF0 EE END tag completed without transfer latency") && ok;
    ok = expect(
             system.bus().read32(0x1000E010u, value) &&
             (value & (1u << 5)) == 0,
             "SIF0 EE END tag raised completion without latency") && ok;

    dma.tick_ee(system.bus());
    ok = expect(
             system.bus().read32(0x1000C000u, value) &&
             (value & 0x100u) == 0,
             "SIF0 EE END tag did not stop the EE chain") && ok;
    ok = expect(
             system.bus().read32(0x1000E010u, value) &&
             (value & (1u << 5)) != 0,
             "SIF0 EE END tag did not raise delayed completion") && ok;

    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok = test_sif1_ee_to_iop() && ok;
    ok = test_sif0_iop_to_ee() && ok;
    ok = test_sif0_completes_each_side_independently() && ok;
    if (!ok) return EXIT_FAILURE;
    std::cout << "VibeStation PS2 SIF tests passed.\n";
    return EXIT_SUCCESS;
}
