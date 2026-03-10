#pragma once
#include "types.h"

// ── Serial I/O (Controller & Memory Card Port) ────────────────────
// Handles the communication protocol with controllers and memory cards.
// The SIO port is at 0x1F801040 - 0x1F80104F.

class System;

class Sio {
public:
  void init(System *sys) { sys_ = sys; }
  void reset();

  u8 read8(u32 offset) const;
  u16 read16(u32 offset) const;
  u32 read32(u32 offset) const;
  void write8(u32 offset, u8 val);
  void write16(u32 offset, u16 val);
  void write32(u32 offset, u32 val);

  // Set the controller state (called by input system)
  void set_button_state(u16 buttons) { button_state_ = buttons; }
  void set_analog_state(u8 lx, u8 ly, u8 rx, u8 ry);

  void tick(u32 cycles);
  bool saw_pad_cmd42() const { return saw_pad_cmd42_; }
  bool saw_tx_cmd42() const { return saw_tx_cmd42_; }
  bool saw_pad_id() const { return saw_pad_id_; }
  bool saw_non_ff_button_byte() const { return saw_non_ff_button_byte_; }
  bool saw_full_pad_poll() const { return saw_full_pad_poll_; }
  u16 last_pad_buttons() const { return last_pad_buttons_; }
  u8 last_tx_byte() const { return last_tx_byte_; }
  u8 last_rx_byte() const { return last_rx_byte_; }
  u64 pad_packet_count() const { return pad_packet_count_; }
  u64 pad_cmd42_count() const { return pad_cmd42_count_; }
  u64 pad_poll_count() const { return pad_poll_count_; }
  u64 channel0_poll_count() const { return channel0_poll_count_; }
  u64 channel1_poll_count() const { return channel1_poll_count_; }
  u64 invalid_sequence_count() const { return invalid_sequence_count_; }
  u16 joy_stat_snapshot() const { return stat_; }
  u16 joy_ctrl_snapshot() const { return ctrl_; }
  u64 irq_assert_count() const { return irq_assert_count_; }
  u64 irq_ack_count() const { return irq_ack_count_; }

private:
  System *sys_ = nullptr;

  // SIO registers
  u8 tx_data_ = 0;
  u8 rx_fifo_ = 0xFF;
  u16 stat_ = 0x0005;
  u16 mode_ = 0;
  u16 ctrl_ = 0;
  u16 baud_ = 0;
  bool tx_ready_1_ = true;
  bool rx_not_empty_ = false;
  bool tx_ready_2_ = true;
  bool dsr_input_level_ = false;
  bool irq_flag_ = false;
  int dsr_pending_cycles_ = 0;
  int dsr_pulse_cycles_ = 0;
  bool irq_pending_ = false;
  bool joy_select_active_ = false;
  u32 byte_index_ = 0;
  bool transfer_active_ = false;
  bool rx_pending_valid_ = false;
  u8 rx_pending_byte_ = 0xFF;
  bool tx_queued_valid_ = false;
  u8 tx_queued_byte_ = 0xFF;
  bool connected_ = true;
  bool saw_pad_cmd42_ = false;
  bool saw_tx_cmd42_ = false;
  bool saw_pad_id_ = false;
  bool saw_non_ff_button_byte_ = false;
  bool saw_full_pad_poll_ = false;
  u16 last_pad_buttons_ = 0xFFFF;
  u8 last_tx_byte_ = 0x00;
  u8 last_rx_byte_ = 0xFF;
  u64 pad_packet_count_ = 0;
  u64 pad_cmd42_count_ = 0;
  u64 pad_poll_count_ = 0;
  u64 channel0_poll_count_ = 0;
  u64 channel1_poll_count_ = 0;
  u64 invalid_sequence_count_ = 0;
  u64 irq_assert_count_ = 0;
  u64 irq_ack_count_ = 0;

  // Controller communication state machine
  enum class State {
    Idle,
    SelectedPad,
    SendingID_Hi,
    SendingID_Lo,
    SendingButtons_Lo,
    SendingButtons_Hi,
    SendingAnalog_RX,
    SendingAnalog_RY,
    SendingAnalog_LX,
    SendingAnalog_LY,
  };
  State state_ = State::Idle;

  // Controller data
  u16 button_state_ = 0xFFFF; // All buttons released (active low)
  u8 analog_lx_ = 0x80, analog_ly_ = 0x80;
  u8 analog_rx_ = 0x80, analog_ry_ = 0x80;
  bool analog_mode_ = false;

  void rebuild_stat();
  void schedule_dsr_pulse();
  void raise_sio_irq_if_enabled();
  bool tx_enabled() const;
  struct ByteResult {
    u8 response = 0xFF;
    bool pulse_dsr = false;
  };
  ByteResult process_byte(u8 value);
};
