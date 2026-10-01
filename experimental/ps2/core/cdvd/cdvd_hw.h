#pragma once

#include "common/types.h"
#include "core/cdvd/disc_image.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace ps2 {

class Bios;
class IopIntc;

// The IOP side of CDVD data transfers (DMA channel 3).
class CdvdDmaSink {
public:
    virtual ~CdvdDmaSink() = default;
    // Copies one block to IOP RAM if DMA3 is armed for at least that many
    // bytes, advancing MADR/BCR and completing the channel when it drains.
    virtual bool cdvd_dma3_deliver(
        const u8* data, u32 bytes, u8 dec_set, u8 key4) = 0;
    // GetToc ignores the block count: it fills MADR and completes DMA3.
    virtual void cdvd_dma3_write_toc(const u8* data, u32 bytes) = 0;
};

class CdvdHw {
public:
    CdvdHw(IopIntc& intc, const Bios& bios)
        : intc_(intc), bios_(bios) {}

    static constexpr u32 kBase = 0x1F402000u;
    static constexpr u32 kSize = 0x40u;
    static constexpr u64 kNoEvent = ~0ull;

    // Resets the drive controller. NVRAM, the clock and the loaded disc
    // survive, as they do across an IOP reboot on real hardware.
    void reset();

    bool load_disc(const std::string& path, std::string& error);
    void eject_disc();
    [[nodiscard]] bool has_disc() const { return disc_.is_open(); }
    [[nodiscard]] const DiscImage& disc() const { return disc_; }

    // Sets the clock (seconds since 2000-01-01 00:00:00 in GMT+9, which is
    // what the mechacon keeps).
    void set_clock(u64 seconds_since_2000);

    // Advances emulated time; due events (sector reads, command
    // completion) run immediately and may move data through `sink`.
    void tick(u64 iop_cycles, CdvdDmaSink& sink);
    // Time skip used when no event can occur inside the interval.
    void advance(u64 iop_cycles) { clock_cycles_ += iop_cycles; consume(iop_cycles); }
    [[nodiscard]] u64 cycles_to_event() const { return event_cycles_; }
    // The IOP armed DMA3; a stalled read can now deliver its sector.
    void on_dma3_start();

    [[nodiscard]] bool read8(u32 physical, u8& value) const;
    [[nodiscard]] bool read16(u32 physical, u16& value) const;
    [[nodiscard]] bool read32(u32 physical, u32& value) const;

    [[nodiscard]] bool write8(u32 physical, u8 value);
    [[nodiscard]] bool write16(u32 physical, u16 value);
    [[nodiscard]] bool write32(u32 physical, u32 value);

private:
    enum class Event : u8 { None, Action, ReadSector };
    enum class Action : u8 { None, Seek, Standby, Stop, Error };

    [[nodiscard]] static bool contains(u32 physical, u32 width);
    void set_s_result(const u8* data, u8 size);
    void execute_s_command(u8 command);
    void execute_n_command(u8 command);
    [[nodiscard]] bool n_command_allowed();
    void start_read(bool dvd_command);
    void schedule(Event event, u64 cycles);
    void consume(u64 cycles) {
        if (event_cycles_ != kNoEvent) {
            event_cycles_ = cycles >= event_cycles_ ? 0 : event_cycles_ - cycles;
        }
    }
    void run_action(CdvdDmaSink& sink);
    void run_read(CdvdDmaSink& sink);
    [[nodiscard]] u64 seek_cycles(u32 lsn);
    [[nodiscard]] u64 sector_cycles() const;
    void set_irq(u8 cause);
    void engage_disc();
    [[nodiscard]] std::size_t config_base() const;
    void seed_nvram_defaults();

    IopIntc& intc_;
    const Bios& bios_;

    DiscImage disc_;
    u8 disc_type_ = 0;
    bool tray_engaged_ = false;
    std::string seeded_romver_ = "\x01";

    u8 n_command_ = 0;
    u8 ready_ = 0;
    mutable u8 error_ = 0;
    u8 intr_stat_ = 0;
    u8 status_ = 0;
    u8 status_sticky_ = 0;
    u8 how_to_ = 0;
    u8 where_select_ = 0;
    u8 dec_set_ = 0;
    std::array<u8, 16> key_{};
    u8 key_xor_ = 0;

    std::array<u8, 16> n_params_{};
    u8 n_param_count_ = 0;

    // Read engine.
    Event event_ = Event::None;
    Action action_ = Action::None;
    u64 event_cycles_ = kNoEvent;
    u32 current_sector_ = 0;
    u32 seek_sector_ = 0;
    s32 sector_count_ = 0;
    u32 block_size_ = 2048;
    u32 speed_ = 4;
    u32 spindle_ctrl_ = 0;
    bool spinning_ = false;
    bool waiting_dma_ = false;
    bool abort_requested_ = false;
    bool dvd_block_ = false;
    bool toc_pending_ = false;

    // Clock (GMT+9 seconds since 2000-01-01).
    u64 clock_base_ = 0;
    u64 clock_cycles_ = 0;

    u8 s_command_ = 0;
    mutable u8 s_ready_ = 0;
    std::array<u8, 16> s_params_{};
    u8 s_param_count_ = 0;
    std::array<u8, 16> s_results_{};
    u8 s_result_count_ = 0;
    mutable u8 s_result_pos_ = 0;

    // Heap-backed to keep Ps2System under the stack-footprint budget.
    std::vector<u8> nvram_ = std::vector<u8>(1024u, 0u);
    u8 config_read_write_ = 0;
    u8 config_offset_ = 0;
    u8 config_blocks_ = 0;
    u8 config_index_ = 0;
};

} // namespace ps2
