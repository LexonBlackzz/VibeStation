#pragma once
#include "memory_card.h"
#include "pad_controller.h"
#include "types.h"

#include <array>
#include <string>

class System;

class Sio {
public:
  static constexpr u32 kMemoryCardSlots = 2;

  void init(System *sys) { sys_ = sys; }
  void shutdown();
  void reset();

  u8 read8(u32 offset) const;
  u16 read16(u32 offset) const;
  u32 read32(u32 offset) const;
  void write8(u32 offset, u8 val);
  void write16(u32 offset, u16 val);
  void write32(u32 offset, u32 val);

  void set_button_state(u16 buttons);
  void set_analog_state(u8 lx, u8 ly, u8 rx, u8 ry);
  bool set_memory_card_slot(u32 slot, const std::string &path);
  void flush_memory_cards();
  bool memory_card_inserted(u32 slot) const;
  bool memory_card_dirty(u32 slot) const;
  std::string memory_card_path(u32 slot) const;

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
  enum class TransferState : u8 {
    Idle,
    Transmitting,
    WaitingForAck,
  };

  enum class ActiveDevice : u8 {
    None,
    Controller,
    MemoryCard,
  };

  void rebuild_stat();
  bool is_transmitting() const;
  bool can_transfer() const;
  u32 transfer_ticks() const;
  static constexpr u32 ack_ticks(bool memory_card);
  void trigger_irq();
  void soft_reset();
  void begin_transfer();
  void do_transfer();
  void do_ack();
  void end_transfer();
  void reset_device_transfer_state();

  System *sys_ = nullptr;

  // SIO registers and transfer latches.
  u8 receive_buffer_ = 0xFF;
  u8 transmit_buffer_ = 0x00;
  u8 transmit_value_ = 0x00;
  bool receive_buffer_full_ = false;
  bool transmit_buffer_full_ = false;
  u16 stat_ = 0x0005;
  u16 mode_ = 0;
  u16 ctrl_ = 0;
  u16 baud_ = 0;
  TransferState transfer_state_ = TransferState::Idle;
  int transfer_event_cycles_ = 0;
  bool irq_flag_ = false;
  bool irq_pending_ = false;
  bool ack_input_ = false;
  ActiveDevice active_device_ = ActiveDevice::None;
  u8 active_slot_ = 0;
  bool connected_ = true;

  // Controller protocol and state (separate from transport timing).
  PadController controller_;
  std::array<MemoryCard, kMemoryCardSlots> memory_cards_{};

  // Diagnostics used by boot checks and debug UI.
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
};
