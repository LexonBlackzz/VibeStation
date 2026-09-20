#pragma once

#include "common/types.h"

namespace ps2 {

class EeHw {
public:
    void reset();
    void tick(u64 cycles);

    [[nodiscard]] bool read32(u32 physical, u32& value) const;
    [[nodiscard]] bool write32(u32 physical, u32 value);
    [[nodiscard]] u64 cycles() const { return cycles_; }

private:
    u64 cycles_ = 0;
    u64 timer0_epoch_ = 0;
    u32 timer0_count_base_ = 0;
    u32 timer0_mode_ = 0;
    u32 memory_ctrl_f500_ = 0;
};

} // namespace ps2
