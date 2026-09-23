#include "core/input/sio2_pad.h"

namespace ps2 {
namespace {

constexpr u32 kCmdPort = 0x1u;
constexpr u32 kCmdLengthMask = 0x3FFu;

} // namespace

void Sio2Pad::reset() {
    state_ = {};
    commands_.fill(0);
    packet_.clear();
    output_.clear();
    queue_position_ = 0;
    command_length_ = 0;
    dma_block_size_ = 0;
    tx_count_ = 0;
    rx_count_ = 0;
    queue_loaded_ = false;
    mode_ = Mode::Digital;
    config_mode_ = false;
    analog_locked_ = false;
    command_stage_ = false;
    current_command_ = 0;
    small_motor_map_ = 0xFFu;
    large_motor_map_ = 0xFFu;
    response_bytes_ = 0;
    cmd_stat_ = 0x0001D100u;
    intr_ = 0;
}

void Sio2Pad::set_button(Button button, bool pressed) {
    const u16 mask =
        static_cast<u16>(1u << static_cast<u8>(button));
    if (pressed) {
        state_.buttons &= static_cast<u16>(~mask);
    } else {
        state_.buttons |= mask;
    }
}

void Sio2Pad::set_analog(
    u8 lx,
    u8 ly,
    u8 rx,
    u8 ry) {
    state_.lx = lx;
    state_.ly = ly;
    state_.rx = rx;
    state_.ry = ry;
}

void Sio2Pad::set_command(u32 index, u32 value) {
    if (index >= commands_.size()) return;
    commands_[index] = value;
    if (index == 0u) {
        packet_.clear();
        output_.clear();
        queue_position_ = 0;
        command_length_ = 0;
        tx_count_ = 0;
        rx_count_ = 0;
        queue_loaded_ = false;
        cmd_stat_ = 0;
    }
}

u32 Sio2Pad::command(u32 index) const {
    return index < commands_.size() ? commands_[index] : 0u;
}

void Sio2Pad::begin_queue_entry() {
    if (queue_position_ >= commands_.size()) {
        command_length_ = 0;
        queue_loaded_ = true;
        return;
    }

    const u32 cmd = commands_[queue_position_];
    command_length_ = (cmd >> 8) & kCmdLengthMask;
    queue_loaded_ = true;
    packet_.clear();
}

void Sio2Pad::write_byte(u8 value) {
    if (!queue_loaded_) begin_queue_entry();
    if (command_length_ == 0u) return;

    packet_.push_back(value);
    ++tx_count_;

    // Command descriptors delimit devices. DMA block size controls transport
    // padding, not the actual peripheral command length.
    if (packet_.size() >= command_length_) {
        finish_packet();
    }
}

u8 Sio2Pad::read_byte() {
    ++rx_count_;
    if (output_.empty()) return 0xFFu;
    const u8 value = output_.front();
    output_.pop_front();
    return value;
}

void Sio2Pad::start_transfer() {
    intr_ |= 1u;
}

void Sio2Pad::finish_packet() {
    const u32 descriptor =
        queue_position_ < commands_.size()
            ? commands_[queue_position_]
            : 0u;
    const u32 port = descriptor & kCmdPort;

    if (packet_.empty()) {
        output_.push_back(0xFFu);
    } else if (packet_[0] == 0x01u && port == 0u) {
        // Port 1 has a connected DualShock 2.
        if ((cmd_stat_ & 0x00000100u) != 0u)
            cmd_stat_ |= 0x00000200u;
        else
            cmd_stat_ |= 0x00000100u;
        cmd_stat_ |= 0x00001000u;

        const auto response = process_pad_packet(packet_);
        for (u8 b : response) output_.push_back(b);
    } else {
        // Memory cards, multitaps and port 2 are intentionally still absent.
        cmd_stat_ |= port == 0u ? 0x0001D000u : 0x0002D000u;
        for (std::size_t i = 0; i < packet_.size(); ++i)
            output_.push_back(0xFFu);
    }

    packet_.clear();
    ++queue_position_;
    queue_loaded_ = false;
}

std::vector<u8> Sio2Pad::process_pad_packet(
    const std::vector<u8>& packet) {
    std::vector<u8> out;
    out.reserve(packet.size());

    // The first byte is the SIO mode (0x01). Hardware returns dead air for
    // that byte; the actual pad state machine starts with the next byte.
    out.push_back(0xFFu);
    current_command_ = 0;

    for (std::size_t i = 1; i < packet.size(); ++i) {
        out.push_back(process_pad_command_byte(packet[i], i));
    }
    return out;
}

u8 Sio2Pad::poll_byte(std::size_t byte_index) const {
    switch (byte_index) {
    case 3: return static_cast<u8>(state_.buttons >> 8);
    case 4: return static_cast<u8>(state_.buttons);
    case 5: return state_.rx;
    case 6: return state_.ry;
    case 7: return state_.lx;
    case 8: return state_.ly;
    default:
        if (byte_index >= 9u && byte_index <= 20u) {
            static constexpr u8 kPressureBits[] = {
                13, 15, 12, 14, 4, 5, 6, 7, 2, 3, 0, 1
            };
            const u8 bit = kPressureBits[byte_index - 9u];
            return (state_.buttons & (1u << bit)) == 0u
                ? 0xFFu
                : 0u;
        }
        return 0u;
    }
}

u8 Sio2Pad::process_pad_command_byte(
    u8 value,
    std::size_t byte_index) {
    if (byte_index == 1u) {
        current_command_ = value;
        command_stage_ = false;
        return static_cast<u8>(
            config_mode_ ? Mode::Config : mode_);
    }
    if (byte_index == 2u) return 0x5Au;

    switch (current_command_) {
    case 0x40: // Mystery.
        if (byte_index == 5u) return 0x02u;
        if (byte_index == 8u) return 0x5Au;
        return 0u;

    case 0x41: // Query buttons.
        if (mode_ == Mode::Digital) return 0u;
        if (byte_index == 3u || byte_index == 4u) return 0xFFu;
        if (byte_index == 5u) return 0x03u;
        if (byte_index == 8u) return 0x5Au;
        return 0u;

    case 0x42: // Poll.
        return poll_byte(byte_index);

    case 0x43: // Enter/leave config mode.
        if (byte_index == 3u) config_mode_ = value != 0u;
        return 0u;

    case 0x44: // Mode switch.
        if (byte_index == 3u && !analog_locked_) {
            mode_ = value != 0u ? Mode::Analog : Mode::Digital;
        } else if (byte_index == 4u) {
            analog_locked_ = value == 0x03u;
        }
        return 0u;

    case 0x45: // Status information.
        switch (byte_index) {
        case 3: return 0x03u; // Standard pad.
        case 4: return 0x02u;
        case 5: return mode_ == Mode::Digital ? 0u : 1u;
        case 6: return 0x02u;
        case 7: return 0x01u;
        default: return 0u;
        }

    case 0x46:
        if (byte_index == 3u) command_stage_ = value != 0u;
        if (byte_index == 5u) return 0x01u;
        if (byte_index == 6u) return command_stage_ ? 0x01u : 0x02u;
        if (byte_index == 7u) return command_stage_ ? 0x01u : 0u;
        if (byte_index == 8u) return command_stage_ ? 0x14u : 0x0Au;
        return 0u;

    case 0x47:
        if (byte_index == 5u) return 0x02u;
        if (byte_index == 7u) return 0x01u;
        return 0u;

    case 0x4C:
        if (byte_index == 3u) command_stage_ = value != 0u;
        if (byte_index == 6u) return command_stage_ ? 0x07u : 0x04u;
        return 0u;

    case 0x4D: // Vibration-byte mapping.
        if (byte_index == 3u) {
            const u8 old = small_motor_map_;
            small_motor_map_ = value;
            return old;
        }
        if (byte_index == 4u) {
            const u8 old = large_motor_map_;
            large_motor_map_ = value;
            return old;
        }
        return 0xFFu;

    case 0x4F: // Response bytes / DS2 pressure mode.
        if (byte_index == 3u) {
            response_bytes_ = value;
        } else if (byte_index == 4u) {
            response_bytes_ |= static_cast<u32>(value) << 8;
        } else if (byte_index == 5u) {
            response_bytes_ |= static_cast<u32>(value) << 16;
            if (response_bytes_ == 0x0003FFFFu)
                mode_ = Mode::DualShock2;
            else if (response_bytes_ == 0x0000003Fu)
                mode_ = Mode::Analog;
            else
                mode_ = Mode::Digital;
        } else if (byte_index == 8u) {
            return 0x5Au;
        }
        return 0u;

    default:
        return 0u;
    }
}

} // namespace ps2
