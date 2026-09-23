#pragma once

#include "common/types.h"
#include "core/input/sio2_pad.h"
#include "core/spu2/spu2.h"

#include <array>

namespace ps2 {

class Bios;
class CdvdHw;
class EeHw;
class IopHwWindow;
class IopIntc;
class IopRam;

class IopBus {
public:
    struct RootCounterDebug {
        std::array<u64, 6> count_writes{};
        std::array<u64, 6> mode_writes{};
        std::array<u64, 6> target_writes{};
        std::array<u64, 6> target_events{};
        std::array<u64, 6> overflow_events{};
        std::array<u64, 6> irq_events{};
        std::array<u32, 6> last_mode_write{};
        std::array<u32, 6> last_target_write{};
        std::array<u32, 6> first_nonzero_target{};
    };
    IopBus(
        IopRam& ram,
        IopHwWindow& hw,
        EeHw& ee_hw,
        IopIntc& intc,
        CdvdHw& cdvd,
        const Bios& bios);

    void reset();
    void tick(u64 cycles);
    // Advance root counters only when no target, overflow, or DMA IRQ can
    // occur during the interval. Returns false without changing state.
    bool tick_event_free(u64 cycles);
    [[nodiscard]] bool can_tick_event_free(u64 cycles) const;

    [[nodiscard]] bool read8(u32 address, u8& value) const;
    [[nodiscard]] bool read16(u32 address, u16& value) const;
    [[nodiscard]] bool read32(u32 address, u32& value) const;

    [[nodiscard]] bool write8(u32 address, u8 value);
    [[nodiscard]] bool write16(u32 address, u16 value);
    [[nodiscard]] bool write32(u32 address, u32 value);

    [[nodiscard]] static u32 to_physical(u32 address);
    [[nodiscard]] static bool is_ram_address(u32 address) {
        return to_physical(address) < 0x00800000u;
    }

    [[nodiscard]] bool interrupt_pending() const;
    [[nodiscard]] u16 sif_dma_ready_mask() const;
    void raise_dma_irq(u32 channel);

    Sio2Pad& sio2() { return sio2_; }
    const Sio2Pad& sio2() const { return sio2_; }
    Spu2& spu2() { return spu2_; }
    const Spu2& spu2() const { return spu2_; }
    const RootCounterDebug& root_counter_debug() const {
        return root_counter_debug_;
    }

private:
    struct RootCounter {
        u64 count = 0;
        u32 mode = 0;
        u64 target = 0;
        u64 phase = 0;
        bool target_deferred = false;
    };

    [[nodiscard]] static bool decode_root_counter(
        u32 physical,
        u32& index,
        u32& reg);
    [[nodiscard]] u64 root_counter_rate(u32 index) const;
    [[nodiscard]] bool read_root_counter(u32 physical, u32 width, u32& value) const;
    [[nodiscard]] bool write_root_counter(u32 physical, u32 width, u32 value);

    [[nodiscard]] bool write_dma_icr(u32 physical, u32 value);
    [[nodiscard]] bool read_ohci(u32 physical, u32 width, u32& value) const;
    [[nodiscard]] bool write_ohci(u32 physical, u32 width, u32 value);
    void reset_ohci(bool hard);
    [[nodiscard]] bool read_firewire(u32 physical, u32 width, u32& value) const;
    [[nodiscard]] bool write_firewire(u32 physical, u32 width, u32 value);
    [[nodiscard]] bool read_sif32(u32 physical, u32& value) const;
    [[nodiscard]] bool write_sif32(u32 physical, u32 value);

    IopRam& ram_;
    IopHwWindow& hw_;
    EeHw& ee_hw_;
    IopIntc& intc_;
    CdvdHw& cdvd_;
    const Bios& bios_;
    std::array<u8, 0x100> cache_control_{};
    Sio2Pad sio2_{};
    Spu2 spu2_{};
    std::array<u32, 0x40> ohci_regs_{};
    u64 ohci_frame_phase_ = 0;
    std::array<u32, 0x60> firewire_regs_{};
    std::array<RootCounter, 6> root_counters_{};
    std::array<u32, 6> root_counter_rate_cache_{};
    RootCounterDebug root_counter_debug_{};
    u64 spu2_dma4_irq_cycles_ = 0;
};

} // namespace ps2
