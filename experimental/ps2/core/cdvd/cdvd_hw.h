#pragma once

#include "common/types.h"

#include <array>

namespace ps2 {

class Bios;
class IopIntc;

class CdvdHw {
public:
    CdvdHw(IopIntc& intc, const Bios& bios)
        : intc_(intc), bios_(bios) {}

    static constexpr u32 kBase = 0x1F402000u;
    static constexpr u32 kSize = 0x40u;

    void reset();

    [[nodiscard]] bool read8(u32 physical, u8& value) const;
    [[nodiscard]] bool read16(u32 physical, u16& value) const;
    [[nodiscard]] bool read32(u32 physical, u32& value) const;

    [[nodiscard]] bool write8(u32 physical, u8 value);
    [[nodiscard]] bool write16(u32 physical, u16 value);
    [[nodiscard]] bool write32(u32 physical, u32 value);

private:
    [[nodiscard]] static bool contains(u32 physical, u32 width);
    void set_s_result(const u8* data, u8 size);
    void execute_s_command(u8 command);
    void set_irq(u8 cause);
    [[nodiscard]] std::size_t config_base() const;
    void seed_nvram_defaults();

    IopIntc& intc_;
    const Bios& bios_;

    u8 n_command_ = 0;
    u8 ready_ = 0;
    mutable u8 error_ = 0;
    u8 intr_stat_ = 0;
    u8 status_ = 0;
    u8 status_sticky_ = 0;
    u8 how_to_ = 0;
    u8 where_select_ = 0;
    u8 dec_set_ = 0;

    std::array<u8, 16> n_params_{};
    u8 n_param_count_ = 0;

    u8 s_command_ = 0;
    mutable u8 s_ready_ = 0;
    std::array<u8, 16> s_params_{};
    u8 s_param_count_ = 0;
    std::array<u8, 16> s_results_{};
    u8 s_result_count_ = 0;
    mutable u8 s_result_pos_ = 0;

    std::array<u8, 1024> nvram_{};
    u8 config_read_write_ = 0;
    u8 config_offset_ = 0;
    u8 config_blocks_ = 0;
    u8 config_index_ = 0;
};

} // namespace ps2
