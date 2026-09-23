#pragma once

#include "common/types.h"

#include <array>
#include <cstddef>
#include <deque>
#include <vector>

namespace ps2 {

class Sio2Pad {
public:
    enum class Button : u8 {
        L2 = 0,
        R2 = 1,
        L1 = 2,
        R1 = 3,
        Triangle = 4,
        Circle = 5,
        Cross = 6,
        Square = 7,
        Select = 8,
        L3 = 9,
        R3 = 10,
        Start = 11,
        Up = 12,
        Right = 13,
        Down = 14,
        Left = 15,
    };

    struct State {
        // DualShock buttons are active-low on the wire.
        u16 buttons = 0xFFFFu;
        u8 rx = 0x7Fu;
        u8 ry = 0x7Fu;
        u8 lx = 0x7Fu;
        u8 ly = 0x7Fu;
    };

    void reset();

    void set_state(const State& state) { state_ = state; }
    [[nodiscard]] const State& state() const { return state_; }
    void set_button(Button button, bool pressed);
    void set_analog(u8 lx, u8 ly, u8 rx, u8 ry);

    void set_command(u32 index, u32 value);
    [[nodiscard]] u32 command(u32 index) const;

    void write_byte(u8 value);
    [[nodiscard]] u8 read_byte() const;
    void start_transfer();

    void set_dma_block_size(std::size_t bytes) { dma_block_size_ = bytes; }
    void clear_dma_block_size() { dma_block_size_ = 0; }

    [[nodiscard]] u32 cmd_stat() const { return cmd_stat_; }
    [[nodiscard]] u32 port_stat() const { return 0x0000000Fu; }
    [[nodiscard]] u32 fifo_stat() const { return 0u; }
    [[nodiscard]] u32 tx_pos() const { return static_cast<u32>(tx_count_); }
    [[nodiscard]] u32 rx_pos() const { return static_cast<u32>(rx_count_); }
    [[nodiscard]] u32 intr() const { return intr_; }
    void acknowledge_intr(u32 bits) { intr_ &= ~bits; }

private:
    enum class Mode : u8 {
        Digital = 0x41,
        Analog = 0x73,
        DualShock2 = 0x79,
        Config = 0xF3,
    };

    void begin_queue_entry();
    void finish_packet();
    [[nodiscard]] std::vector<u8> process_pad_packet(
        const std::vector<u8>& packet);
    [[nodiscard]] u8 process_pad_command_byte(
        u8 value,
        std::size_t byte_index);
    [[nodiscard]] u8 poll_byte(std::size_t byte_index) const;

    State state_{};
    std::array<u32, 16> commands_{};
    std::vector<u8> packet_{};
    mutable std::deque<u8> output_{};

    std::size_t queue_position_ = 0;
    std::size_t command_length_ = 0;
    std::size_t dma_block_size_ = 0;
    std::size_t tx_count_ = 0;
    mutable std::size_t rx_count_ = 0;
    bool queue_loaded_ = false;

    Mode mode_ = Mode::Digital;
    bool config_mode_ = false;
    bool analog_locked_ = false;
    bool command_stage_ = false;
    u8 current_command_ = 0;
    u8 small_motor_map_ = 0xFFu;
    u8 large_motor_map_ = 0xFFu;
    u32 response_bytes_ = 0;

    u32 cmd_stat_ = 0x0001D100u;
    u32 intr_ = 0;
};

} // namespace ps2
