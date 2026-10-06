#include "core/cdvd/cdvd_hw.h"

#include "core/bios/bios.h"
#include "core/iop/iop_intc.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u8 kDriveError = 0x01u;
constexpr u8 kDriveDev9Connected = 0x04u;
constexpr u8 kDriveMechaInitialized = 0x08u;
constexpr u8 kDriveReady = 0x40u;
constexpr u8 kDriveBusy = 0x80u;
constexpr u8 kStatusTrayOpen = 0x01u;
constexpr u8 kStatusSpin = 0x02u;
constexpr u8 kStatusRead = 0x06u;
constexpr u8 kStatusPause = 0x0Au;
constexpr u8 kStatusSeek = 0x12u;
constexpr u8 kSCommandReady = 0x40u;
constexpr u64 kIopClock = 36'864'000ull;

// Drive timing in IOP cycles. These follow PCSX2's model scaled down: the
// guest only needs the ordering (seek, then sectors at a plausible rate).
constexpr u64 kSpinUpCycles = kIopClock / 30u;
constexpr u64 kFastSeekCycles = kIopClock * 8u / 1000u;
constexpr u64 kFullSeekCycles = kIopClock * 25u / 1000u;
constexpr u32 kFastSeekDelta = 14764u;
constexpr u32 kContiguousDelta = 16u;
constexpr u8 kNCommandParamLength[16] = {
    0, 0, 0, 0, 0, 4, 11, 11, 11, 1, 255, 255, 7, 2, 11, 1};

} // namespace

void CdvdHw::reset() {
    if (seeded_romver_ != bios_.romver()) {
        seed_nvram_defaults();
        seeded_romver_ = bios_.romver();
    }
    config_read_write_ = 0;
    config_offset_ = 0;
    config_blocks_ = 0;
    config_index_ = 0;

    n_command_ = 0;
    error_ = 0;
    intr_stat_ = 0;
    how_to_ = 0;
    where_select_ = 0;
    dec_set_ = 0;
    key_.fill(0);
    key_xor_ = 0;
    n_params_.fill(0);
    n_param_count_ = 0;

    event_ = Event::None;
    action_ = Action::None;
    event_cycles_ = kNoEvent;
    current_sector_ = 0;
    sector_count_ = 0;
    spindle_ctrl_ = 0;
    waiting_dma_ = false;
    abort_requested_ = false;
    toc_pending_ = false;
    spinning_ = false;
    if (disc_.is_open()) {
        engage_disc();
    } else {
        ready_ =
            kDriveReady |
            kDriveMechaInitialized |
            kDriveDev9Connected;
        status_ = kStatusTrayOpen;
        status_sticky_ = kStatusTrayOpen;
        disc_type_ = 0;
        tray_engaged_ = false;
    }

    s_command_ = 0;
    s_ready_ = kSCommandReady;
    s_params_.fill(0);
    s_param_count_ = 0;
    s_results_.fill(0);
    s_result_count_ = 0;
    s_result_pos_ = 0;
}

bool CdvdHw::load_disc(const std::string& path, std::string& error) {
    DiscImage image;
    if (!image.open(path, error)) return false;
    disc_ = std::move(image);
    return true;
}

void CdvdHw::eject_disc() {
    disc_.close();
}

void CdvdHw::set_clock(u64 seconds_since_2000) {
    clock_base_ = seconds_since_2000;
    clock_cycles_ = 0;
}

void CdvdHw::engage_disc() {
    disc_type_ = static_cast<u8>(disc_.type());
    tray_engaged_ = true;
    ready_ = kDriveReady | kDriveMechaInitialized | kDriveDev9Connected;
    status_ = kStatusPause;
    status_sticky_ = 0;
    spinning_ = true;
    current_sector_ = 0;
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
    std::fill(nvram_.begin(), nvram_.end(), u8{0});

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
        0x30u, 0x21u, 0x81u, 0x0Eu,
        0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0xE0u,
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

    case 0x06: // Tray control: the virtual tray closes on the loaded disc.
        if (s_param_count_ > 0 && s_params_[0] == 0u) {
            tray_engaged_ = false;
            status_ = kStatusTrayOpen;
            status_sticky_ |= status_;
            ready_ = kDriveMechaInitialized | kDriveDev9Connected;
            spinning_ = false;
        } else if (!tray_engaged_ && disc_.is_open()) {
            engage_disc();
        }
        success();
        break;

    case 0x08: { // Read RTC (BCD, GMT+9).
        std::array<u8, 8> rtc{};
        u64 seconds = clock_base_ + clock_cycles_ / kIopClock;
        const u64 days = seconds / 86400u;
        seconds %= 86400u;
        // Civil date from a day count (Hinnant's algorithm); the clock's
        // epoch is 2000-01-01.
        const s64 z = static_cast<s64>(days) + 10957 + 719468;
        const s64 era = z / 146097;
        const s64 doe = z - era * 146097;
        const s64 yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
        const s64 doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
        const s64 mp = (5 * doy + 2) / 153;
        const s64 day = doy - (153 * mp + 2) / 5 + 1;
        const s64 month = mp < 10 ? mp + 3 : mp - 9;
        const s64 year = yoe + era * 400 + (month <= 2 ? 1 : 0);
        auto bcd = [](u64 v) {
            return static_cast<u8>(((v / 10u) << 4) | (v % 10u));
        };
        rtc[1] = bcd(seconds % 60u);
        rtc[2] = bcd((seconds / 60u) % 60u);
        rtc[3] = bcd(seconds / 3600u);
        rtc[5] = bcd(static_cast<u64>(day));
        rtc[6] = bcd(static_cast<u64>(month));
        rtc[7] = bcd(static_cast<u64>(year - 2000));
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
    abort_requested_ = false;
}

bool CdvdHw::contains(u32 physical, u32 width) {
    if (physical < kBase) {
        return false;
    }

    const u32 offset = physical - kBase;
    return offset <= kSize && width <= (kSize - offset);
}

void CdvdHw::schedule(Event event, u64 cycles) {
    event_ = event;
    event_cycles_ = std::max<u64>(cycles, 1u);
}

u64 CdvdHw::sector_cycles() const {
    // PCSX2 reads DVD at <=1.6x (675 sectors/s) and CD at <=10.3x (75/s);
    // halved here ("fast CDVD").
    const u64 per_second = disc_.is_dvd() ? 675ull : 75ull;
    const double speed = disc_.is_dvd()
        ? std::min<double>(speed_, 1.6) : std::min<double>(speed_, 10.3);
    return static_cast<u64>(
        static_cast<double>(kIopClock) / (per_second * speed) / 2.0);
}

u64 CdvdHw::seek_cycles(u32 lsn) {
    const u32 delta = lsn > current_sector_
        ? lsn - current_sector_ : current_sector_ - lsn;
    if (!spinning_) {
        spinning_ = true;
        return kSpinUpCycles;
    }
    if (delta < kContiguousDelta) return 0;
    return delta >= kFastSeekDelta ? kFullSeekCycles : kFastSeekCycles;
}

bool CdvdHw::n_command_allowed() {
    if (n_command_ > 0u &&
        ((status_ & kStatusTrayOpen) != 0u || disc_type_ == 0u)) {
        error_ = disc_type_ == 0u ? 0x12u : 0x11u; // No disc / tray open.
        ready_ |= kDriveError;
        set_irq(0x01u);
        return false;
    }
    if (n_command_ < 16u &&
        kNCommandParamLength[n_command_] != 255u &&
        n_param_count_ != kNCommandParamLength[n_command_]) {
        error_ = 0x22u; // Invalid parameter for command.
        ready_ |= kDriveError;
        set_irq(0x01u);
        return false;
    }
    if (n_command_ > 0x0Fu) {
        error_ = 0x10u; // Unsupported command.
        ready_ |= kDriveError;
        set_irq(0x01u);
        return false;
    }
    return true;
}

void CdvdHw::start_read(bool dvd_command) {
    const auto word = [&](u32 at) {
        return static_cast<u32>(n_params_[at]) |
               (static_cast<u32>(n_params_[at + 1]) << 8) |
               (static_cast<u32>(n_params_[at + 2]) << 16) |
               (static_cast<u32>(n_params_[at + 3]) << 24);
    };
    const bool dvd = disc_.is_dvd();
    auto fail = [&](u8 error) {
        error_ = error;
        action_ = Action::Error;
        status_ = kStatusSeek;
        status_sticky_ |= status_;
        ready_ = kDriveBusy | kDriveMechaInitialized | kDriveDev9Connected;
        schedule(Event::Action, sector_cycles() * 12u);
    };
    if (dvd_command && !dvd) { // DVD read on a CD.
        error_ = 0x14u;
        ready_ = kDriveReady | kDriveError | kDriveMechaInitialized |
                 kDriveDev9Connected;
        set_irq(0x01u);
        return;
    }
    seek_sector_ = word(0);
    sector_count_ = static_cast<s32>(word(4));
    spindle_ctrl_ = (n_params_[9] & 0x3Fu) != 0u
        ? n_params_[9]
        : static_cast<u32>((n_params_[9] & 0x80u) | (dvd ? 3u : 5u));
    switch (spindle_ctrl_ & 7u) {
    case 1: speed_ = 1; break;
    case 2: speed_ = 2; break;
    case 3: speed_ = 4; break;
    case 4: speed_ = dvd ? 0u : 12u; break;
    case 5: speed_ = dvd ? 0u : 24u; break;
    default: speed_ = 0; break;
    }
    // Only the plain 2048-byte data mode exists for ISO images.
    if (speed_ == 0u || n_params_[10] != 0u) { fail(0x22u); return; }
    // CdRead always moves 2048-byte blocks; DvdRead adds the raw sector frame.
    dvd_block_ = dvd_command;
    block_size_ = dvd_command ? 2064u : 2048u;
    if (sector_count_ <= 0) { fail(0x21u); return; }
    if (seek_sector_ >= disc_.sector_count()) { fail(0x30u); return; }

    const u64 delay = seek_cycles(seek_sector_);
    status_ = kStatusSeek;
    status_sticky_ |= status_;
    ready_ = kDriveBusy | kDriveMechaInitialized | kDriveDev9Connected;
    current_sector_ = seek_sector_;
    waiting_dma_ = false;
    schedule(Event::ReadSector, delay + sector_cycles());
}

void CdvdHw::execute_n_command(u8 command) {
    n_command_ = command;
    abort_requested_ = false;

    if ((ready_ & kDriveReady) == 0u) {
        error_ = 0x13u; // Not ready.
        ready_ |= kDriveError;
        set_irq(0x01u);
        n_param_count_ = 0;
        return;
    }
    if (!n_command_allowed()) {
        n_param_count_ = 0;
        return;
    }

    const auto word = [&](u32 at) {
        return static_cast<u32>(n_params_[at]) |
               (static_cast<u32>(n_params_[at + 1]) << 8) |
               (static_cast<u32>(n_params_[at + 2]) << 16) |
               (static_cast<u32>(n_params_[at + 3]) << 24);
    };
    const u8 idle_ready =
        kDriveReady | kDriveMechaInitialized | kDriveDev9Connected;
    const u8 busy_ready =
        kDriveBusy | kDriveMechaInitialized | kDriveDev9Connected;

    switch (command) {
    case 0x00: // CdNop
        ready_ = idle_ready;
        set_irq(0x01u);
        break;
    case 0x01: // CdReset / CdSync
        ready_ = idle_ready;
        s_param_count_ = 0;
        status_ = 0;
        spinning_ = false;
        s_results_.fill(0);
        set_irq(0x01u);
        break;
    case 0x02: // CdStandby
        ready_ = busy_ready;
        seek_sector_ = 0;
        status_ = kStatusSeek;
        action_ = Action::Standby;
        schedule(Event::Action, seek_cycles(0));
        break;
    case 0x03: // CdStop
        ready_ = busy_ready;
        status_ = kStatusSpin;
        action_ = Action::Stop;
        schedule(Event::Action, kIopClock / 30u);
        break;
    case 0x04: // CdPause
        event_ = Event::None;
        event_cycles_ = kNoEvent;
        waiting_dma_ = false;
        ready_ = idle_ready;
        status_ = kStatusPause;
        set_irq(0x01u);
        break;
    case 0x05: // CdSeek
        ready_ = busy_ready;
        seek_sector_ = word(0);
        status_ = kStatusSeek;
        action_ = Action::Seek;
        schedule(Event::Action, seek_cycles(seek_sector_));
        break;
    case 0x06: // CdRead
        start_read(false);
        break;
    case 0x08: // DvdRead
        start_read(true);
        break;
    case 0x09: // CdGetToc: finishes from the event so DMA3 completes there.
        ready_ = busy_ready;
        status_ = kStatusPause;
        toc_pending_ = true;
        schedule(Event::Action, 64u);
        break;
    case 0x0C: { // CdReadKey: derives the disc key from the boot serial.
        key_.fill(0);
        std::string serial = disc_.boot_path();
        const auto slash = serial.find_last_of("\\/");
        if (slash != std::string::npos) serial = serial.substr(slash + 1);
        // "SLUS_123.45;1" -> letters "SLUS", digits "12345".
        s32 numbers = 0;
        s32 letters = 0;
        if (serial.size() >= 11u) {
            numbers = std::atoi(
                (serial.substr(5, 3) + serial.substr(9, 2)).c_str());
            letters = static_cast<s32>((serial[3] & 0x7F) << 0) |
                      static_cast<s32>((serial[2] & 0x7F) << 7) |
                      static_cast<s32>((serial[1] & 0x7F) << 14) |
                      static_cast<s32>((serial[0] & 0x7F) << 21);
        }
        const u32 arg2 = word(3);
        const u32 key_0_3 = ((numbers & 0x1FC00) >> 10) |
                            ((0x01FFFFFF & letters) << 7);
        const u8 key_4 = static_cast<u8>(
            ((numbers & 0x0001F) << 3) | ((0x0E000000 & letters) >> 25));
        const u8 key_14 = static_cast<u8>(((numbers & 0x003E0) >> 2) | 0x04);
        key_[0] = static_cast<u8>(key_0_3);
        key_[1] = static_cast<u8>(key_0_3 >> 8);
        key_[2] = static_cast<u8>(key_0_3 >> 16);
        key_[3] = static_cast<u8>(key_0_3 >> 24);
        key_[4] = key_4;
        if (arg2 == 75u) {
            key_[14] = key_14;
            key_[15] = 0x05u;
        } else if (arg2 == 4246u) {
            key_ = {0x07u, 0xF7u, 0xF2u, 0x01u, 0x00u};
            key_[15] = 0x01u;
        } else {
            key_[15] = 0x01u;
        }
        key_xor_ = 0;
        ready_ = idle_ready;
        status_ = kStatusPause;
        set_irq(0x01u);
        break;
    }
    case 0x0F: // CdChgSpdlCtrl
        set_irq(0x01u);
        break;
    default: // CDDA reads and unknown commands: report unsupported.
        error_ = 0x10u;
        ready_ = idle_ready | kDriveError;
        set_irq(0x01u);
        break;
    }
    n_param_count_ = 0;
}

void CdvdHw::on_dma3_start() {
    if (waiting_dma_ && event_ == Event::None) {
        waiting_dma_ = false;
        schedule(Event::ReadSector, 64u);
    }
}

void CdvdHw::run_action(CdvdDmaSink& sink) {
    const u8 idle_ready =
        kDriveReady | kDriveMechaInitialized | kDriveDev9Connected;
    if (toc_pending_) {
        toc_pending_ = false;
        std::array<u8, 2048> toc{};
        if (disc_.is_dvd()) {
            // Single-layer DVD structure (see PCSX2 ISOgetTOC).
            toc[0] = 0x04; toc[1] = 0x02; toc[2] = 0xF2; toc[3] = 0x00;
            toc[4] = 0x86; toc[5] = 0x72;
            toc[12] = 0x01; toc[13] = 0x02; toc[14] = 0x01; toc[15] = 0x00;
            toc[16] = 0x00; toc[17] = 0x03; toc[18] = 0x00; toc[19] = 0x00;
            const u32 max_lsn = disc_.sector_count() + 0x30000u - 1u;
            toc[20] = static_cast<u8>(max_lsn >> 24);
            toc[21] = static_cast<u8>(max_lsn >> 16);
            toc[22] = static_cast<u8>(max_lsn >> 8);
            toc[23] = static_cast<u8>(max_lsn);
        } else {
            // One data track: first/last track and the lead-out position.
            toc[0] = 0x41; toc[2] = 0xA0; toc[7] = 0x01;
            toc[12] = 0xA1; toc[17] = 0x01;
            const u32 lba = disc_.sector_count() + 150u;
            auto bcd = [](u32 v) {
                return static_cast<u8>(((v / 10u) << 4) | (v % 10u));
            };
            toc[22] = 0xA2;
            toc[27] = bcd(lba / 4500u);
            toc[28] = bcd((lba / 75u) % 60u);
            toc[29] = bcd(lba % 75u);
            toc[32] = 0x01; toc[37] = 0x00; toc[38] = 0x02; toc[39] = 0x00;
        }
        sink.cdvd_dma3_write_toc(toc.data(), static_cast<u32>(toc.size()));
        ready_ = idle_ready;
        status_ = kStatusPause;
        set_irq(0x01u);
        return;
    }

    u8 ready = idle_ready;
    if (abort_requested_) {
        error_ = 0x01u;
        ready |= kDriveError;
        status_ = kStatusPause;
    } else {
        switch (action_) {
        case Action::Seek:
        case Action::Standby:
            spinning_ = true;
            current_sector_ = seek_sector_;
            status_ = kStatusPause;
            break;
        case Action::Stop:
            spinning_ = false;
            current_sector_ = 0;
            status_ = 0;
            break;
        default: // Action::Error
            ready |= kDriveError;
            status_ = kStatusPause;
            break;
        }
    }
    status_sticky_ |= status_;
    ready_ = ready;
    action_ = Action::None;
    set_irq(0x01u);
}

void CdvdHw::run_read(CdvdDmaSink& sink) {
    const u8 idle_ready =
        kDriveReady | kDriveMechaInitialized | kDriveDev9Connected;
    status_ = kStatusRead;
    status_sticky_ |= status_;
    ready_ = kDriveBusy | kDriveMechaInitialized | kDriveDev9Connected;

    auto fail = [&](u8 error) {
        error_ = error;
        ready_ = idle_ready | kDriveError;
        status_ = kStatusPause;
        set_irq(0x01u);
    };
    if (abort_requested_) { fail(0x01u); return; }
    if (current_sector_ >= disc_.sector_count()) {
        fail(0x32u); // Outermost track reached.
        return;
    }

    std::array<u8, 2064> block{};
    u8* user = dvd_block_ ? block.data() + 12 : block.data();
    if (!disc_.read_sector(current_sector_, user)) { fail(0x30u); return; }
    if (dvd_block_) {
        // Raw DVD sector: 12-byte ID header, data, 4-byte EDC.
        const u32 lsn = current_sector_ + 0x30000u;
        block[0] = 0x20u;
        block[1] = static_cast<u8>(lsn >> 16);
        block[2] = static_cast<u8>(lsn >> 8);
        block[3] = static_cast<u8>(lsn);
    }
    if (!sink.cdvd_dma3_deliver(block.data(), block_size_, dec_set_, key_[4])) {
        // DMA3 isn't armed yet; resume when the IOP starts it.
        status_ = kStatusPause;
        waiting_dma_ = true;
        return;
    }

    ++current_sector_;
    ++seek_sector_;
    if (--sector_count_ <= 0) {
        ready_ = idle_ready;
        status_ = kStatusPause;
        set_irq(0x01u);
        return;
    }
    schedule(Event::ReadSector, sector_cycles());
}

void CdvdHw::tick(u64 iop_cycles, CdvdDmaSink& sink) {
    clock_cycles_ += iop_cycles;
    while (event_cycles_ != kNoEvent) {
        if (iop_cycles < event_cycles_) {
            event_cycles_ -= iop_cycles;
            return;
        }
        iop_cycles -= event_cycles_;
        const Event event = event_;
        event_ = Event::None;
        event_cycles_ = kNoEvent;
        if (event == Event::Action) run_action(sink);
        else if (event == Event::ReadSector) run_read(sink);
    }
}

bool CdvdHw::read8(u32 physical, u8& value) const {
    if (!contains(physical, 1)) {
        return false;
    }

    auto bcd = [](u32 v) {
        return static_cast<u8>(((v / 10u) << 4) | (v % 10u));
    };
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
        value = bcd((current_sector_ / (60u * 75u)) % 100u);
        return true;
    case 0x0D: // Current second; sector zero reports 00:02:00.
        value = bcd((current_sector_ / 75u) % 60u + 2u);
        return true;
    case 0x0E: // Current frame
        value = bcd(current_sector_ % 75u);
        return true;
    case 0x0F: // Disc type
        value = tray_engaged_ ? disc_type_ : 0u;
        return true;
    case 0x13: { // Spindle speed
        u8 speed = static_cast<u8>(spindle_ctrl_ & 0x3Fu);
        const bool dvd = disc_.is_open() && disc_.is_dvd();
        if (speed == 0u) speed = dvd ? 3u : 5u;
        speed = dvd ? static_cast<u8>(speed + 0xFu)
                    : static_cast<u8>(speed - 1u);
        if (!tray_engaged_ || !spinning_) speed = 0u;
        value = speed;
        return true;
    }
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
    case 0x20: case 0x21: case 0x22: case 0x23: case 0x24:
        value = key_[(physical - kBase) - 0x20u];
        return true;
    case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2C:
        value = key_[(physical - kBase) - 0x23u];
        return true;
    case 0x30: case 0x31: case 0x32: case 0x33: case 0x34:
        value = key_[(physical - kBase) - 0x26u];
        return true;
    case 0x38: // Valid parts of the key.
        value = key_[15];
        return true;
    case 0x39: // KEY-XOR
        value = key_xor_;
        return true;
    case 0x3A: // DEC_SET
        value = dec_set_;
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
        execute_n_command(value);
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
        if ((ready_ & kDriveBusy) != 0u) abort_requested_ = true;
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
