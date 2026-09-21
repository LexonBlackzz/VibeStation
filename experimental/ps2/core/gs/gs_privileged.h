#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>

namespace ps2 {

class GsPrivileged {
public:
    static constexpr u32 kBase = 0x12000000u;
    static constexpr std::size_t kSize = 0x2000;

    void reset();
    [[nodiscard]] bool contains(u32 address, std::size_t width) const;

    [[nodiscard]] bool read8(u32 address, u8& value) const;
    [[nodiscard]] bool read16(u32 address, u16& value) const;
    [[nodiscard]] bool read32(u32 address, u32& value) const;
    [[nodiscard]] bool read64(u32 address, u64& value) const;

    [[nodiscard]] bool write8(u32 address, u8 value);
    [[nodiscard]] bool write16(u32 address, u16 value);
    [[nodiscard]] bool write32(u32 address, u32 value);
    [[nodiscard]] bool write64(u32 address, u64 value);

    void signal(u64 value);
    void finish();
    void label(u64 value);
    void raise_vsync();

    [[nodiscard]] bool irq_pending() const;
    [[nodiscard]] u32 csr() const;
    [[nodiscard]] u32 imr() const;
    [[nodiscard]] u32 busdir() const;
    [[nodiscard]] u32 signal_id() const;
    [[nodiscard]] u32 label_id() const;

private:
    static constexpr u32 kCsr = kBase + 0x1000u;
    static constexpr u32 kImr = kBase + 0x1010u;
    static constexpr u32 kBusdir = kBase + 0x1040u;
    static constexpr u32 kSiglblid = kBase + 0x1080u;

    [[nodiscard]] u32 load32(u32 address) const;
    void store32(u32 address, u32 value);
    void write_csr_command(u32 value);
    void write_imr_value(u32 value);
    void promote_queued_signal();

    std::array<u8, kSize> data_{};
    bool queued_signal_ = false;
    u64 queued_signal_value_ = 0;
};

} // namespace ps2
