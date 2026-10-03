#include "core/gs/gs_privileged.h"

namespace ps2 {
namespace {

constexpr u32 kCsrSignal = 1u << 0;
constexpr u32 kCsrFinish = 1u << 1;
constexpr u32 kCsrHsint  = 1u << 2;
constexpr u32 kCsrVsint  = 1u << 3;
constexpr u32 kCsrEdwint = 1u << 4;
constexpr u32 kCsrReset  = 1u << 9;
constexpr u32 kCsrFifoEmpty = 1u << 14;
constexpr u32 kCsrRevision = 0x1Bu << 16;
constexpr u32 kCsrId = 0x55u << 24;
constexpr u32 kCsrFixed = kCsrFifoEmpty | kCsrRevision | kCsrId;
constexpr u32 kImrMasks = 0x1F00u;
constexpr u32 kImrUndefined = 0x6000u;

} // namespace

u32 GsPrivileged::load32(u32 address) const {
    const u32 offset = address - kBase;
    return static_cast<u32>(data_[offset]) |
           (static_cast<u32>(data_[offset + 1]) << 8) |
           (static_cast<u32>(data_[offset + 2]) << 16) |
           (static_cast<u32>(data_[offset + 3]) << 24);
}

void GsPrivileged::store32(u32 address, u32 value) {
    const u32 offset = address - kBase;
    for (u32 i = 0; i < 4; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
}

void GsPrivileged::reset() {
    data_.fill(0);
    queued_signal_ = false;
    queued_signal_value_ = 0;
    store32(kCsr, kCsrFixed);
    store32(kImr, kImrMasks | kImrUndefined);
    store32(kSiglblid, 0);
    store32(kSiglblid + 4u, 0);
}

bool GsPrivileged::contains(u32 address, std::size_t width) const {
    if (address < kBase) return false;
    const std::size_t offset = static_cast<std::size_t>(address - kBase);
    return offset <= kSize && width <= (kSize - offset);
}

u32 GsPrivileged::csr() const {
    return load32(kCsr);
}

u32 GsPrivileged::imr() const {
    return load32(kImr);
}

u32 GsPrivileged::busdir() const {
    return load32(kBusdir) & 1u;
}

u32 GsPrivileged::signal_id() const {
    return load32(kSiglblid);
}

u32 GsPrivileged::label_id() const {
    return load32(kSiglblid + 4u);
}

bool GsPrivileged::irq_pending() const {
    const u32 events = csr() & 0x1Fu;
    const u32 masks = (imr() >> 8) & 0x1Fu;
    return (events & ~masks) != 0;
}

void GsPrivileged::promote_queued_signal() {
    if (!queued_signal_) return;
    const u32 data = static_cast<u32>(queued_signal_value_);
    const u32 mask = static_cast<u32>(queued_signal_value_ >> 32);
    store32(kSiglblid, (signal_id() & ~mask) | (data & mask));
    store32(kCsr, csr() | kCsrSignal);
    queued_signal_ = false;
    queued_signal_value_ = 0;
}

void GsPrivileged::write_csr_command(u32 value) {
    if ((value & kCsrReset) != 0) {
        reset();
        return;
    }

    u32 state = csr();
    const u32 clear = value &
        (kCsrSignal | kCsrFinish | kCsrHsint | kCsrVsint | kCsrEdwint);
    state &= ~clear;
    state = (state & 0x00003FFFu) | kCsrFixed;
    store32(kCsr, state);

    if ((clear & kCsrSignal) != 0) {
        promote_queued_signal();
    }
}

void GsPrivileged::write_imr_value(u32 value) {
    store32(kImr, (value & kImrMasks) | kImrUndefined);
}

void GsPrivileged::signal(u64 value) {
    if ((csr() & kCsrSignal) != 0) {
        if (!queued_signal_) {
            queued_signal_ = true;
            queued_signal_value_ = value;
        }
        return;
    }

    const u32 data = static_cast<u32>(value);
    const u32 mask = static_cast<u32>(value >> 32);
    store32(kSiglblid, (signal_id() & ~mask) | (data & mask));
    store32(kCsr, csr() | kCsrSignal);
}

void GsPrivileged::finish() {
    store32(kCsr, csr() | kCsrFinish);
}

void GsPrivileged::label(u64 value) {
    const u32 data = static_cast<u32>(value);
    const u32 mask = static_cast<u32>(value >> 32);
    store32(kSiglblid + 4u, (label_id() & ~mask) | (data & mask));
}

void GsPrivileged::raise_vsync() {
    // CSR.FIELD is the currently displayed field: 0=even, 1=odd.  Retail
    // synchronization code polls this bit around VSync, so advance it with
    // each interlaced field instead of exposing a permanently-even display.
    u32 state = csr() ^ (1u << 13);
    state |= kCsrVsint;
    store32(kCsr, state);
}

bool GsPrivileged::read8(u32 address, u8& value) const {
    if (!contains(address, 1)) return false;
    value = data_[address - kBase];
    return true;
}

bool GsPrivileged::read16(u32 address, u16& value) const {
    if (!contains(address, 2)) return false;
    const u32 offset = address - kBase;
    value = static_cast<u16>(data_[offset]) |
            (static_cast<u16>(data_[offset + 1]) << 8);
    return true;
}

bool GsPrivileged::read32(u32 address, u32& value) const {
    if (!contains(address, 4)) return false;
    value = load32(address);
    return true;
}

bool GsPrivileged::read64(u32 address, u64& value) const {
    if (!contains(address, 8)) return false;
    const u32 offset = address - kBase;
    value = 0;
    for (u32 i = 0; i < 8; ++i)
        value |= static_cast<u64>(data_[offset + i]) << (i * 8);
    return true;
}

bool GsPrivileged::write8(u32 address, u8 value) {
    if (!contains(address, 1)) return false;
    if (address >= kCsr && address < kCsr + 4u) {
        write_csr_command(static_cast<u32>(value) << ((address - kCsr) * 8u));
        return true;
    }
    if (address >= kImr && address < kImr + 4u) {
        const u32 shift = (address - kImr) * 8u;
        const u32 merged =
            (imr() & ~(0xFFu << shift)) | (static_cast<u32>(value) << shift);
        write_imr_value(merged);
        return true;
    }
    data_[address - kBase] = value;
    return true;
}

bool GsPrivileged::write16(u32 address, u16 value) {
    if (!contains(address, 2)) return false;
    if (address >= kCsr && address + 2u <= kCsr + 4u) {
        write_csr_command(static_cast<u32>(value) << ((address - kCsr) * 8u));
        return true;
    }
    if (address >= kImr && address + 2u <= kImr + 4u) {
        const u32 shift = (address - kImr) * 8u;
        const u32 merged =
            (imr() & ~(0xFFFFu << shift)) | (static_cast<u32>(value) << shift);
        write_imr_value(merged);
        return true;
    }
    const u32 offset = address - kBase;
    data_[offset] = static_cast<u8>(value);
    data_[offset + 1] = static_cast<u8>(value >> 8);
    return true;
}

bool GsPrivileged::write32(u32 address, u32 value) {
    if (!contains(address, 4)) return false;
    if (address == kCsr) {
        write_csr_command(value);
        return true;
    }
    if (address == kImr) {
        write_imr_value(value);
        return true;
    }
    if (address == kBusdir) {
        store32(kBusdir, value & 1u);
        return true;
    }
    store32(address, value);
    return true;
}

bool GsPrivileged::write64(u32 address, u64 value) {
    if (!contains(address, 8)) return false;
    if (address == kCsr) {
        write_csr_command(static_cast<u32>(value));
        return true;
    }
    if (address == kImr) {
        write_imr_value(static_cast<u32>(value));
        return true;
    }
    if (address == kBusdir) {
        store32(kBusdir, static_cast<u32>(value) & 1u);
        return true;
    }

    const u32 offset = address - kBase;
    for (u32 i = 0; i < 8; ++i)
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    return true;
}

} // namespace ps2
