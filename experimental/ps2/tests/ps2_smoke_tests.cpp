#include "core/ps2_system.h"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
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
    constexpr ps2::u32 kseg0 = 0x80123450u;
    constexpr ps2::u32 kseg1 = 0xA0123450u;
    constexpr ps2::u32 value = 0xCAFEBABEu;

    if (!expect(system.bus().write32(kseg0, value), "KSEG0 write failed")) {
        return false;
    }

    ps2::u32 readback = 0;
    return expect(system.bus().read32(physical, readback) && readback == value,
                  "physical alias mismatch") &&
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
    write32(0x41004, 0x70000000u);

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

    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);
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

bool test_cdvd_reset_status() {
    ps2::Ps2System system;

    ps2::u8 value = 0;
    bool ok =
        expect(system.iop_bus().read8(0xBF402005u, value) &&
                   value == 0x4Cu,
               "CDVD N-READY reset value mismatch");

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

} // namespace

int main() {
    bool ok = true;
    ok = test_ram_little_endian() && ok;
    ok = test_ram_aliases() && ok;
    ok = test_ram_bounds() && ok;
    ok = test_scheduler_ordering() && ok;
    ok = test_scheduler_cancel() && ok;
    ok = test_ee_reset_state() && ok;
    ok = test_bios_mapping_and_startup() && ok;
    ok = test_iop_reset_and_shared_ram() && ok;
    ok = test_iop_ram_mirror_boundary() && ok;
    ok = test_ee_iop_startup_interleave() && ok;
    ok = test_iop_cache_isolation_blocks_ram_store() && ok;
    ok = test_ee_timer0_clock_sources() && ok;
    ok = test_cdvd_reset_status() && ok;
    ok = test_cdvd_scommand_result_fifo() && ok;

    if (!ok) {
        return EXIT_FAILURE;
    }

    std::cout << "VibeStation PS2 scaffold tests passed.\n";
    return EXIT_SUCCESS;
}
