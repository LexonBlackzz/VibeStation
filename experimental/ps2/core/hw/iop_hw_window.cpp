#include "core/hw/iop_hw_window.h"

#include "core/iop/iop_intc.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u32 kTimer16Base = 0x1F801100u;
constexpr u32 kTimer32Base = 0x1F801480u;
constexpr u32 kTimerStride = 0x10u;

constexpr u32 kTimerCount = 0x0u;
constexpr u32 kTimerMode = 0x4u;
constexpr u32 kTimerTarget = 0x8u;

constexpr u32 kModeResetAtTarget = 1u << 3;
constexpr u32 kModeIrqTarget = 1u << 4;
constexpr u32 kModeIrqOverflow = 1u << 5;
constexpr u32 kModeRepeat = 1u << 6;
constexpr u32 kModeToggle = 1u << 7;
constexpr u32 kModeAltSource = 1u << 8;
constexpr u32 kModeT2Prescale = 1u << 9;
constexpr u32 kModeIrqEnabled = 1u << 10;
constexpr u32 kModeTargetFlag = 1u << 11;
constexpr u32 kModeOverflowFlag = 1u << 12;
constexpr u32 kModePrescale45Mask = 3u << 13;

constexpr u32 kTimerIrqs[6] = {4u, 5u, 6u, 14u, 15u, 16u};

} // namespace

void IopHwWindow::reset() {
    data_.fill(0);
    timer_phase_.fill(0);
    timer_count_.fill(0);
    timer_target_.fill(0);
    timer_mode_.fill(kModeIrqEnabled);
}

bool IopHwWindow::decode_timer(
    u32 address,
    u32& index,
    u32& reg,
    u32& byte_offset) {
    u32 bank = 0;
    u32 first_index = 0;

    if (address >= kTimer16Base &&
        address < kTimer16Base + 3u * kTimerStride) {
        bank = kTimer16Base;
        first_index = 0;
    } else if (address >= kTimer32Base &&
               address < kTimer32Base + 3u * kTimerStride) {
        bank = kTimer32Base;
        first_index = 3;
    } else {
        return false;
    }

    const u32 local = address - bank;
    const u32 slot = local / kTimerStride;
    const u32 within = local % kTimerStride;
    const u32 aligned = within & ~3u;
    if (aligned != kTimerCount &&
        aligned != kTimerMode &&
        aligned != kTimerTarget) {
        return false;
    }

    index = first_index + slot;
    reg = aligned;
    byte_offset = within - aligned;
    return byte_offset < 4u;
}

u32 IopHwWindow::timer_value(u32 index, u32 reg) const {
    if (index >= 6u) return 0;
    const u32 mask = index < 3u ? 0xFFFFu : 0xFFFFFFFFu;
    switch (reg) {
    case kTimerCount:
        return timer_count_[index] & mask;
    case kTimerMode:
        return timer_mode_[index];
    case kTimerTarget:
        return timer_target_[index] & mask;
    default:
        return 0;
    }
}

void IopHwWindow::write_timer(u32 index, u32 reg, u32 value) {
    if (index >= 6u) return;
    const u32 mask = index < 3u ? 0xFFFFu : 0xFFFFFFFFu;

    switch (reg) {
    case kTimerCount:
        timer_count_[index] = value & mask;
        timer_phase_[index] = 0;
        break;

    case kTimerMode: {
        // Match the useful firmware-facing behavior: writable mode bits are
        // replaced, event flags remain sticky, IRQ output is re-armed, and a
        // mode write resets the counter.
        const u32 flags =
            timer_mode_[index] & (kModeTargetFlag | kModeOverflowFlag);
        timer_mode_[index] =
            (value & 0x63FFu) | flags | kModeIrqEnabled;
        timer_count_[index] = 0;
        timer_phase_[index] = 0;
        break;
    }

    case kTimerTarget:
        timer_target_[index] = value & mask;
        break;

    default:
        break;
    }
}

void IopHwWindow::write_timer_partial(
    u32 index,
    u32 reg,
    u32 byte_offset,
    u32 width,
    u32 value) {
    const u32 bits = width * 8u;
    const u32 shift = byte_offset * 8u;
    const u32 field_mask =
        bits >= 32u ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    const u32 lane_mask = field_mask << shift;
    const u32 current = timer_value(index, reg);
    const u32 next =
        (current & ~lane_mask) | ((value << shift) & lane_mask);
    write_timer(index, reg, next);
}

u32 IopHwWindow::timer_rate(u32 index) const {
    if (index >= 6u) return 1u;
    const u32 mode = timer_mode_[index];

    if (index == 0u) {
        // Alternate source is the ~13.5 MHz pixel clock.
        return (mode & kModeAltSource) != 0 ? 3u : 1u;
    }
    if (index == 1u || index == 3u) {
        // Alternate source is HBlank.  36.864 MHz / 15.734 kHz ~= 2343.
        return (mode & kModeAltSource) != 0 ? 2343u : 1u;
    }
    if (index == 2u) {
        return (mode & kModeT2Prescale) != 0 ? 8u : 1u;
    }

    switch ((mode & kModePrescale45Mask) >> 13) {
    case 1: return 8u;
    case 2: return 16u;
    case 3: return 256u;
    default: return 1u;
    }
}

void IopHwWindow::fire_timer_irq(
    IopIntc& intc,
    u32 index,
    bool overflow) {
    if (index >= 6u) return;
    u32& mode = timer_mode_[index];

    const u32 source_bit = overflow ? kModeIrqOverflow : kModeIrqTarget;
    if ((mode & source_bit) == 0 ||
        (mode & kModeIrqEnabled) == 0) {
        return;
    }

    intc.raise(kTimerIrqs[index]);

    if ((mode & kModeRepeat) == 0) {
        mode &= ~kModeIrqEnabled;
    } else if ((mode & kModeToggle) != 0) {
        mode ^= kModeIrqEnabled;
    } else {
        // Repeating pulse mode rearms automatically.
        mode |= kModeIrqEnabled;
    }
}

void IopHwWindow::tick(u64 cycles, IopIntc& intc) {
    for (u32 index = 0; index < 6u; ++index) {
        const u32 rate = std::max(timer_rate(index), 1u);
        timer_phase_[index] += cycles;

        while (timer_phase_[index] >= rate) {
            timer_phase_[index] -= rate;

            const u32 max_value =
                index < 3u ? 0xFFFFu : 0xFFFFFFFFu;
            u32 next = timer_count_[index];

            if (next == max_value) {
                next = 0;
                timer_mode_[index] |= kModeOverflowFlag;
                fire_timer_irq(intc, index, true);
            } else {
                ++next;
            }

            const u32 target = timer_target_[index] & max_value;
            if (next == target) {
                timer_mode_[index] |= kModeTargetFlag;
                fire_timer_irq(intc, index, false);
                if ((timer_mode_[index] & kModeResetAtTarget) != 0) {
                    next = 0;
                }
            }

            timer_count_[index] = next;
        }
    }
}

bool IopHwWindow::contains(u32 address, std::size_t width) const {
    if (address < kBase) {
        return false;
    }

    const std::size_t offset = static_cast<std::size_t>(address - kBase);
    return offset <= kSize && width <= (kSize - offset);
}

bool IopHwWindow::read8(u32 address, u8& value) const {
    if (!contains(address, 1)) {
        return false;
    }

    u32 timer_index = 0;
    u32 timer_reg = 0;
    u32 byte_offset = 0;
    if (decode_timer(address, timer_index, timer_reg, byte_offset)) {
        value = static_cast<u8>(
            timer_value(timer_index, timer_reg) >>
            (byte_offset * 8u));
        return true;
    }

    // Current PCSX2 behavior for these Page-3 IOP registers.
    if (address == 0x1F803100u) {
        value = 0;
        return true;
    }
    if (address == 0x1F803204u) {
        value = 0x7Cu;
        return true;
    }

    value = data_[address - kBase];
    return true;
}

bool IopHwWindow::read16(u32 address, u16& value) const {
    if (!contains(address, 2)) {
        return false;
    }

    u32 timer_index = 0;
    u32 timer_reg = 0;
    u32 byte_offset = 0;
    if (decode_timer(address, timer_index, timer_reg, byte_offset) &&
        byte_offset <= 2u) {
        value = static_cast<u16>(
            timer_value(timer_index, timer_reg) >>
            (byte_offset * 8u));
        return true;
    }

    u8 lo = 0;
    u8 hi = 0;
    if (!read8(address, lo) || !read8(address + 1u, hi)) {
        return false;
    }
    value = static_cast<u16>(lo) | (static_cast<u16>(hi) << 8);
    return true;
}

bool IopHwWindow::read32(u32 address, u32& value) const {
    if (!contains(address, 4)) {
        return false;
    }

    u32 timer_index = 0;
    u32 timer_reg = 0;
    u32 byte_offset = 0;
    if (decode_timer(address, timer_index, timer_reg, byte_offset) &&
        byte_offset == 0u) {
        value = timer_value(timer_index, timer_reg);
        return true;
    }

    value = 0;
    for (u32 i = 0; i < 4; ++i) {
        u8 byte = 0;
        if (!read8(address + i, byte)) return false;
        value |= static_cast<u32>(byte) << (i * 8);
    }
    return true;
}

bool IopHwWindow::read64(u32 address, u64& value) const {
    if (!contains(address, 8)) {
        return false;
    }

    value = 0;
    for (u32 i = 0; i < 8; ++i) {
        u8 byte = 0;
        if (!read8(address + i, byte)) return false;
        value |= static_cast<u64>(byte) << (i * 8);
    }
    return true;
}

bool IopHwWindow::write8(u32 address, u8 value) {
    if (!contains(address, 1)) {
        return false;
    }

    u32 timer_index = 0;
    u32 timer_reg = 0;
    u32 byte_offset = 0;
    if (decode_timer(address, timer_index, timer_reg, byte_offset)) {
        write_timer_partial(
            timer_index,
            timer_reg,
            byte_offset,
            1u,
            value);
        return true;
    }

    data_[address - kBase] = value;
    return true;
}

bool IopHwWindow::write16(u32 address, u16 value) {
    if (!contains(address, 2)) {
        return false;
    }

    u32 timer_index = 0;
    u32 timer_reg = 0;
    u32 byte_offset = 0;
    if (decode_timer(address, timer_index, timer_reg, byte_offset) &&
        byte_offset <= 2u) {
        write_timer_partial(
            timer_index,
            timer_reg,
            byte_offset,
            2u,
            value);
        return true;
    }

    const u32 offset = address - kBase;
    for (u32 i = 0; i < 2; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
    return true;
}

bool IopHwWindow::write32(u32 address, u32 value) {
    if (!contains(address, 4)) {
        return false;
    }

    u32 timer_index = 0;
    u32 timer_reg = 0;
    u32 byte_offset = 0;
    if (decode_timer(address, timer_index, timer_reg, byte_offset) &&
        byte_offset == 0u) {
        write_timer(timer_index, timer_reg, value);
        return true;
    }

    const u32 offset = address - kBase;
    for (u32 i = 0; i < 4; ++i) {
        data_[offset + i] = static_cast<u8>(value >> (i * 8));
    }
    return true;
}

bool IopHwWindow::write64(u32 address, u64 value) {
    if (!contains(address, 8)) {
        return false;
    }

    for (u32 i = 0; i < 8; ++i) {
        if (!write8(address + i, static_cast<u8>(value >> (i * 8)))) {
            return false;
        }
    }
    return true;
}

} // namespace ps2
