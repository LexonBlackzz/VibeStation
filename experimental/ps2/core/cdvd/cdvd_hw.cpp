#include "core/cdvd/cdvd_hw.h"

#include "core/bios/bios.h"
#include "core/iop/iop_intc.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u8 kDriveDev9Connected = 0x04u;
constexpr u8 kDriveMechaInitialized = 0x08u;
constexpr u8 kDriveReady = 0x40u;
constexpr u8 kStatusTrayOpen = 0x01u;
constexpr u8 kSCommandReady = 0x40u;

} // namespace

void CdvdHw::reset() {
    seed_nvram_defaults();
    config_read_write_ = 0;
    config_offset_ = 0;
    config_blocks_ = 0;
    config_index_ = 0;

    n_command_ = 0;
    ready_ =
        kDriveReady |
        kDriveMechaInitialized |
        kDriveDev9Connected;
    error_ = 0;
    intr_stat_ = 0;
    status_ = kStatusTrayOpen;
    status_sticky_ = kStatusTrayOpen;
    how_to_ = 0;
    where_select_ = 0;
    dec_set_ = 0;
    n_params_.fill(0);
    n_param_count_ = 0;

    s_command_ = 0;
    s_ready_ = kSCommandReady;
    s_params_.fill(0);
    s_param_count_ = 0;
    s_results_.fill(0);
    s_result_count_ = 0;
    s_result_pos_ = 0;

}

std::size_t CdvdHw::config_base() const {
    const std::string& romver = bios_.romver();
    bool modern = romver.size() >= 4u;
    for (std::size_t i = 0; i < 4u && modern; ++i) {
        modern =
            romver[i] >= '0' && romver[i] <= '9';
    }
    if (modern) modern = romver.substr(0, 4) >= "0170";

    switch (config_offset_) {
    case 0:
        return modern ? 0x270u : 0x280u;
    case 2:
        return 0x200u;
    default:
        return modern ? 0x2B0u : 0x300u;
    }
}

void CdvdHw::seed_nvram_defaults() {
    nvram_.fill(0);

    const std::string& romver = bios_.romver();
    bool modern = romver.size() >= 4u;
    for (std::size_t i = 0; i < 4u && modern; ++i) {
        modern =
            romver[i] >= '0' && romver[i] <= '9';
    }
    if (modern) modern = romver.substr(0, 4) >= "0170";

    const std::size_t config1 =
        modern ? 0x2B0u : 0x300u;
    static constexpr std::array<u8, 16> kJapanese{
        0x20u, 0x20u, 0x00u, 0x00u,
        0x00u, 0x70u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x30u,
    };
    static constexpr std::array<u8, 16> kEnglish{
        0x30u, 0x21u, 0x00u, 0x00u,
        0x00u, 0x70u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x41u,
    };

    const char region =
        romver.size() > 4u ? romver[4] : 'A';
    const auto& language =
        region == 'J' ? kJapanese : kEnglish;
    std::copy(
        language.begin(),
        language.end(),
        nvram_.begin() + config1 + 0x10u);

    const std::size_t ilink =
        modern ? 0x1E0u : 0x1C0u;
    static constexpr std::array<u8, 8> kILinkId{
        0x00u, 0xACu, 0xFFu, 0xFFu,
        0xFFu, 0xFFu, 0xB9u, 0x86u,
    };
    std::copy(
        kILinkId.begin(),
        kILinkId.end(),
        nvram_.begin() + ilink);

    // SCPH-3xxxx-era v2.xx firmware reads region parameters from NVRAM even
    // with no memory card or disc inserted. Seed the common values needed by
    // the OSD bootstrap rather than returning an all-zero invalid region.
    if (romver.size() >= 4u &&
        romver[0] == '0' && romver[1] == '2' &&
        romver.substr(0, 4) != "0210") {
        std::array<u8, 12> region_data{};
        if (region == 'J') {
            region_data = {
                0x4Au, 0x4Au, 0x6Au, 0x70u,
                0x6Eu, 0x4Au, 0x4Au, 0, 0, 0, 0, 0};
        } else {
            region_data = {
                0x41u, 0x41u, 0x65u, 0x6Eu,
                0x67u, 0x41u, 0x55u, 0, 0, 0, 0, 0};
        }
        std::copy(
            region_data.begin(),
            region_data.end(),
            nvram_.begin() + 0x180u);
    }
}

void CdvdHw::set_s_result(const u8* data, u8 size) {
    s_results_.fill(0);
    s_result_count_ =
        size > s_results_.size()
            ? static_cast<u8>(s_results_.size())
            : size;
    s_result_pos_ = 0;

    for (u8 i = 0; i < s_result_count_; ++i) {
        s_results_[i] = data[i];
    }

    if (s_result_count_ == 0) {
        s_ready_ |= 0x40u;
    } else {
        s_ready_ &= static_cast<u8>(~0x40u);
    }
}

void CdvdHw::execute_s_command(u8 command) {
    s_command_ = command;
    s_ready_ &= static_cast<u8>(~0x80u);

    auto success = [&]() {
        constexpr std::array<u8, 1> result{0};
        set_s_result(result.data(), 1);
    };
    auto modern_layout = [&]() {
        const std::string& romver = bios_.romver();
        if (romver.size() < 4u) return false;
        for (std::size_t i = 0; i < 4u; ++i) {
            if (romver[i] < '0' || romver[i] > '9') return false;
        }
        return romver.substr(0, 4) >= "0170";
    };

    switch (command) {
    case 0x03: { // Mecacon command.
        const u8 sub = s_param_count_ > 0 ? s_params_[0] : 0xFFu;
        if (sub == 0x00u) {
            constexpr std::array<u8, 4> kMechaVersion{
                0x03u, 0x06u, 0x02u, 0x00u};
            set_s_result(
                kMechaVersion.data(),
                static_cast<u8>(kMechaVersion.size()));
        } else if (sub == 0x30u) {
            const std::array<u8, 2> result{
                status_,
                static_cast<u8>((status_ & 0x01u) != 0 ? 8u : 0u)};
            set_s_result(
                result.data(),
                static_cast<u8>(result.size()));
        } else if (sub == 0x45u) {
            std::array<u8, 9> result{};
            const std::size_t base =
                modern_layout() ? 0x1F0u : 0x1C8u;
            std::copy_n(
                nvram_.begin() + base,
                8u,
                result.begin() + 1u);
            set_s_result(
                result.data(),
                static_cast<u8>(result.size()));
        } else if (sub == 0xFDu) {
            constexpr std::array<u8, 6> result{
                0x00u, 0x04u, 0x12u, 0x10u, 0x01u, 0x30u};
            set_s_result(result.data(), static_cast<u8>(result.size()));
        } else if (sub == 0xEFu) {
            constexpr std::array<u8, 3> result{0x00u, 0x0Fu, 0x05u};
            set_s_result(result.data(), static_cast<u8>(result.size()));
        } else {
            constexpr std::array<u8, 1> unsupported{0x81u};
            set_s_result(unsupported.data(), 1);
        }
        break;
    }

    case 0x05: // Tray request state.
        status_sticky_ = status_ & kStatusTrayOpen;
        success();
        break;

    case 0x06: // Tray control: no physical drive, accept the request.
        success();
        break;

    case 0x08: { // Read RTC. Use a deterministic valid reset date.
        constexpr std::array<u8, 8> rtc{
            0x00u, 0x00u, 0x00u, 0x00u,
            0x00u, 0x01u, 0x01u, 0x00u};
        set_s_result(rtc.data(), static_cast<u8>(rtc.size()));
        break;
    }

    case 0x09: // Write RTC.
        success();
        break;

    case 0x0A: { // Read NVM word.
        const u32 address =
            s_param_count_ >= 2u
                ? (static_cast<u32>(s_params_[0]) << 8) | s_params_[1]
                : 0xFFFFFFFFu;
        if (address >= 512u) {
            constexpr std::array<u8, 1> invalid{0xFFu};
            set_s_result(invalid.data(), 1);
            break;
        }
        const std::size_t byte = address * 2u;
        const std::array<u8, 3> result{
            0u, nvram_[byte + 1u], nvram_[byte]};
        set_s_result(result.data(), static_cast<u8>(result.size()));
        break;
    }

    case 0x0B: { // Write NVM word.
        if (s_param_count_ >= 4u) {
            const u32 address =
                (static_cast<u32>(s_params_[0]) << 8) | s_params_[1];
            if (address < 512u) {
                const std::size_t byte = address * 2u;
                nvram_[byte] = s_params_[3];
                nvram_[byte + 1u] = s_params_[2];
            }
        }
        success();
        break;
    }

    case 0x12: { // Read i.Link ID.
        std::array<u8, 9> result{};
        const std::size_t base =
            modern_layout() ? 0x1E0u : 0x1C0u;
        std::copy_n(
            nvram_.begin() + base,
            8u,
            result.begin() + 1u);
        set_s_result(result.data(), static_cast<u8>(result.size()));
        break;
    }

    case 0x13: // Write i.Link ID.
        success();
        break;

    case 0x14: // Digital audio output control.
    case 0x16: // Auto-adjust control.
    case 0x1B: // Cancel power-off ready.
    case 0x1C: // Blue LED control.
    case 0x24: // Remote-control bypass.
    case 0x29: // Notice game start.
    case 0x31: // Set medium removal.
        success();
        break;

    case 0x15: {
        constexpr std::array<u8, 1> result{5u};
        set_s_result(result.data(), 1);
        break;
    }

    case 0x17: { // Read model number.
        std::array<u8, 9> result{};
        const std::size_t base =
            (modern_layout() ? 0x1B0u : 0x1A0u) +
            (s_param_count_ != 0 ? s_params_[0] : 0u);
        if (base + 8u <= nvram_.size()) {
            std::copy_n(
                nvram_.begin() + base,
                8u,
                result.begin() + 1u);
        }
        set_s_result(result.data(), static_cast<u8>(result.size()));
        break;
    }

    case 0x1A: { // Boot certify.
        constexpr std::array<u8, 1> result{1u};
        set_s_result(result.data(), 1);
        break;
    }

    case 0x1E: {
        constexpr std::array<u8, 5> result{0x00u, 0x14u, 0, 0, 0};
        set_s_result(result.data(), static_cast<u8>(result.size()));
        break;
    }

    case 0x20: {
        constexpr std::array<u8, 3> result{0x00u, 0x01u, 0x00u};
        set_s_result(result.data(), static_cast<u8>(result.size()));
        break;
    }

    case 0x22: {
        constexpr std::array<u8, 10> result{};
        set_s_result(result.data(), static_cast<u8>(result.size()));
        break;
    }

    case 0x32: {
        constexpr std::array<u8, 2> result{};
        set_s_result(result.data(), static_cast<u8>(result.size()));
        break;
    }

    case 0x36: { // Read region parameters.
        std::array<u8, 15> result{};
        result[1] = 0x40u; // Default mechacon encryption zone.
        const std::size_t base = 0x180u;
        std::copy_n(
            nvram_.begin() + base,
            8u,
            result.begin() + 3u);
        set_s_result(result.data(), static_cast<u8>(result.size()));
        break;
    }

    case 0x37: { // Read MAC.
        std::array<u8, 9> result{};
        std::copy_n(
            nvram_.begin() + 0x198u,
            8u,
            result.begin() + 1u);
        set_s_result(result.data(), static_cast<u8>(result.size()));
        break;
    }

    case 0x40: // Open config.
        config_read_write_ =
            s_param_count_ > 0u ? s_params_[0] : 0u;
        config_offset_ =
            s_param_count_ > 1u ? s_params_[1] : 0u;
        config_blocks_ =
            s_param_count_ > 2u ? s_params_[2] : 0u;
        config_index_ = 0;
        success();
        break;

    case 0x41: { // Read config block.
        std::array<u8, 16> result{};
        if (config_read_write_ != 0u) {
            result[0] = 0x80u;
        } else if (config_index_ < config_blocks_) {
            const u32 max_blocks =
                config_offset_ == 0u ? 4u :
                config_offset_ == 1u ? 2u : 7u;
            if (config_index_ < max_blocks) {
                const std::size_t base =
                    config_base() +
                    static_cast<std::size_t>(config_index_) * 16u;
                if (base + 16u <= nvram_.size()) {
                    std::copy_n(
                        nvram_.begin() + base,
                        16u,
                        result.begin());
                }
                ++config_index_;
            }
        }
        set_s_result(result.data(), static_cast<u8>(result.size()));
        break;
    }

    case 0x42: { // Write config block.
        if (config_read_write_ == 1u &&
            config_index_ < config_blocks_ &&
            s_param_count_ >= 16u) {
            const u32 max_blocks =
                config_offset_ == 0u ? 4u :
                config_offset_ == 1u ? 2u : 7u;
            if (config_index_ < max_blocks) {
                const std::size_t base =
                    config_base() +
                    static_cast<std::size_t>(config_index_) * 16u;
                if (base + 16u <= nvram_.size()) {
                    std::copy_n(
                        s_params_.begin(),
                        16u,
                        nvram_.begin() + base);
                }
                ++config_index_;
            }
        }
        success();
        break;
    }

    case 0x43: // Close config.
        config_read_write_ = 0;
        config_offset_ = 0;
        config_blocks_ = 0;
        config_index_ = 0;
        success();
        break;

    default: {
        constexpr std::array<u8, 1> unsupported{0x80u};
        set_s_result(unsupported.data(), 1);
        break;
    }
    }

    s_param_count_ = 0;
}

void CdvdHw::set_irq(u8 cause) {
    if ((intr_stat_ & cause) == 0) {
        intr_stat_ |= cause;
        // The PS2 CDVD controller is wired to IOP INTC source 2.
        intc_.raise(2);
    }
}

bool CdvdHw::contains(u32 physical, u32 width) {
    if (physical < kBase) {
        return false;
    }

    const u32 offset = physical - kBase;
    return offset <= kSize && width <= (kSize - offset);
}

bool CdvdHw::read8(u32 physical, u8& value) const {
    if (!contains(physical, 1)) {
        return false;
    }

    switch (physical - kBase) {
    case 0x04: // NCOMMAND
        value = n_command_;
        return true;
    case 0x05: // N-READY
        value = ready_;
        return true;
    case 0x06: // ERROR; reading acknowledges the latched error.
        value = error_;
        error_ = 0;
        return true;
    case 0x07: // BREAK
        value = 0;
        return true;
    case 0x08: // INTR_STAT
        value = intr_stat_;
        return true;
    case 0x0A: // STATUS
        value = status_;
        return true;
    case 0x0B: // STATUS STICKY
        value = status_sticky_;
        return true;
    case 0x0C: // Current minute
        value = 0;
        return true;
    case 0x0D: // Current second; sector zero reports 00:02:00.
        value = 0x02;
        return true;
    case 0x0E: // Current frame
        value = 0;
        return true;
    case 0x0F: // Disc type: no disc at reset.
        value = 0;
        return true;
    case 0x13: // Spindle speed
    case 0x15: // Reserved / PS1 DESR
        value = 0;
        return true;
    case 0x16: // SCOMMAND
        value = s_command_;
        return true;
    case 0x17: // SREADY
        value = s_ready_;
        return true;
    case 0x18: // SDATAOUT
        if ((s_ready_ & 0x40u) == 0 &&
            s_result_pos_ < s_result_count_) {
            value = s_results_[s_result_pos_++];
            if (s_result_pos_ >= s_result_count_) {
                s_ready_ |= 0x40u;
            }
        } else {
            value = 0;
        }
        return true;
    case 0x20:
    case 0x21:
    case 0x22:
    case 0x23:
    case 0x24:
    case 0x28:
    case 0x29:
    case 0x2A:
    case 0x2B:
    case 0x2C:
    case 0x30:
    case 0x31:
    case 0x32:
    case 0x33:
    case 0x34:
    case 0x38:
    case 0x39:
    case 0x3A:
        value = 0;
        return true;
    default:
        return false;
    }
}

bool CdvdHw::read16(u32 physical, u16& value) const {
    u8 lo = 0;
    u8 hi = 0;
    if (!read8(physical, lo) ||
        !read8(physical + 1, hi)) {
        return false;
    }

    value = static_cast<u16>(lo) |
            (static_cast<u16>(hi) << 8);
    return true;
}

bool CdvdHw::read32(u32 physical, u32& value) const {
    if (!contains(physical, 4)) {
        return false;
    }

    value = 0;
    for (u32 i = 0; i < 4; ++i) {
        u8 byte = 0;
        if (!read8(physical + i, byte)) {
            return false;
        }
        value |= static_cast<u32>(byte) << (i * 8);
    }
    return true;
}

bool CdvdHw::write8(u32 physical, u8 value) {
    if (!contains(physical, 1)) {
        return false;
    }

    switch (physical - kBase) {
    case 0x04: // NCOMMAND
        n_command_ = value;
        if (value <= 0x01u) {
            // NOP and RESET complete immediately in this early model.
            ready_ =
                kDriveReady |
                kDriveMechaInitialized |
                kDriveDev9Connected;
            error_ = 0;
            set_irq(0x01u);
            n_param_count_ = 0;
            if (value == 0x01u) {
                status_ = 0;
            }
        } else {
            // Full seek/read commands are not implemented yet.
            error_ = 0x10u;
            ready_ |= 0x01u;
            set_irq(0x01u);
            n_param_count_ = 0;
        }
        return true;
    case 0x05: // NDATAIN
        if (n_param_count_ >= n_params_.size()) {
            n_param_count_ = 0;
        }
        n_params_[n_param_count_++] = value;
        return true;
    case 0x06: // HOWTO
        how_to_ = value;
        return true;
    case 0x07: // BREAK
        return true;
    case 0x08: // INTR_STAT: writing ones acknowledges those causes.
        intr_stat_ &= static_cast<u8>(~value);
        return true;
    case 0x09: // WHERE_SELECT
        where_select_ = value;
        return true;
    case 0x0A: // STATUS (write is accepted, no state change)
    case 0x0F: // TYPE (write is accepted)
    case 0x14: // PS1-mode speed control
        return true;
    case 0x16: // SCOMMAND
        execute_s_command(value);
        return true;
    case 0x17: // SDATAIN
        if (s_param_count_ >= s_params_.size()) {
            s_param_count_ = 0;
        }
        s_params_[s_param_count_++] = value;
        return true;
    case 0x18: // SDATAOUT write
        return true;
    case 0x3A: // DEC_SET
        dec_set_ = value;
        return true;
    default:
        return false;
    }
}

bool CdvdHw::write16(u32 physical, u16 value) {
    return write8(physical, static_cast<u8>(value)) &&
           write8(physical + 1, static_cast<u8>(value >> 8));
}

bool CdvdHw::write32(u32 physical, u32 value) {
    if (!contains(physical, 4)) {
        return false;
    }

    for (u32 i = 0; i < 4; ++i) {
        if (!write8(
                physical + i,
                static_cast<u8>(value >> (i * 8)))) {
            return false;
        }
    }
    return true;
}

} // namespace ps2
