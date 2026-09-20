#include "core/cdvd/cdvd_hw.h"

#include "core/iop/iop_intc.h"

namespace ps2 {
namespace {

constexpr u8 kDriveDev9Connected = 0x04u;
constexpr u8 kDriveMechaInitialized = 0x08u;
constexpr u8 kDriveReady = 0x40u;
constexpr u8 kStatusTrayOpen = 0x01u;
constexpr u8 kSCommandReady = 0x40u;

} // namespace

void CdvdHw::reset() {
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

    switch (command) {
    case 0x03: { // Mecacon command.
        if (s_param_count_ > 0 && s_params_[0] == 0x00u) {
            constexpr std::array<u8, 4> kMechaVersion{
                0x03u, 0x06u, 0x02u, 0x00u};
            set_s_result(
                kMechaVersion.data(),
                static_cast<u8>(kMechaVersion.size()));
        } else if (s_param_count_ > 0 && s_params_[0] == 0x30u) {
            const std::array<u8, 2> result{
                status_,
                static_cast<u8>((status_ & 0x01u) != 0 ? 8u : 0u)};
            set_s_result(
                result.data(),
                static_cast<u8>(result.size()));
        } else {
            constexpr std::array<u8, 1> kUnsupported{0x80u};
            set_s_result(kUnsupported.data(), 1);
        }
        break;
    }
    case 0x05: { // Tray request state.
        status_sticky_ = status_ & kStatusTrayOpen;
        constexpr std::array<u8, 1> kOk{0};
        set_s_result(kOk.data(), 1);
        break;
    }
    case 0x08: { // Read RTC. Use a deterministic valid reset date.
        constexpr std::array<u8, 8> kRtc{
            0x00u, // status
            0x00u, // second
            0x00u, // minute
            0x00u, // hour
            0x00u, // padding
            0x01u, // day
            0x01u, // month
            0x00u  // year 2000
        };
        set_s_result(kRtc.data(), static_cast<u8>(kRtc.size()));
        break;
    }
    case 0x09: // Write RTC.
    case 0x14: // Digital audio output control.
    case 0x16: { // Auto-adjust control.
        constexpr std::array<u8, 1> kOk{0};
        set_s_result(kOk.data(), 1);
        break;
    }
    case 0x12: { // Read i.Link ID; PCSX2-compatible fallback.
        constexpr std::array<u8, 9> kILinkId{
            0x00u, 0x00u, 0xACu, 0xFFu, 0xFFu,
            0xFFu, 0xFFu, 0xB9u, 0x86u};
        set_s_result(
            kILinkId.data(),
            static_cast<u8>(kILinkId.size()));
        break;
    }
    case 0x15: { // Forbid DVD player.
        constexpr std::array<u8, 1> kResult{5};
        set_s_result(kResult.data(), 1);
        break;
    }
    default: {
        // The command port itself is implemented. Unsupported command
        // semantics return a device-level error instead of becoming a bus
        // fault, which keeps the boundary at the CDVD protocol layer.
        constexpr std::array<u8, 1> kUnsupported{0x80u};
        set_s_result(kUnsupported.data(), 1);
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
