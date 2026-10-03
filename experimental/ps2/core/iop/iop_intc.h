#pragma once

#include "common/types.h"

namespace ps2 {

class IopIntc {
public:
    static constexpr u32 kIStat = 0x1F801070u;
    static constexpr u32 kIMask = 0x1F801074u;
    static constexpr u32 kICtrl = 0x1F801078u;

    void reset();

    [[nodiscard]] bool read8(u32 physical, u8& value) const;
    [[nodiscard]] bool read16(u32 physical, u16& value) const;
    [[nodiscard]] bool read32(u32 physical, u32& value) const;

    [[nodiscard]] bool write8(u32 physical, u8 value);
    [[nodiscard]] bool write16(u32 physical, u16 value);
    [[nodiscard]] bool write32(u32 physical, u32 value);

    void raise(u32 irq);

    [[nodiscard]] bool pending() const {
        return ictrl_ != 0 && (istat_ & imask_) != 0;
    }

    [[nodiscard]] u32 status() const { return istat_; }
    [[nodiscard]] u32 mask() const { return imask_; }
    [[nodiscard]] u32 control() const { return ictrl_; }

private:
    [[nodiscard]] static bool decode(
        u32 physical,
        u32& base,
        u32& byte_offset);

    [[nodiscard]] u32 read_register(u32 base) const;
    void write_partial(
        u32 base,
        u32 byte_offset,
        u32 width,
        u32 value);

    u32 istat_ = 0;
    u32 imask_ = 0;
    mutable u32 ictrl_ = 0;
};

} // namespace ps2
