#include "sio.h"
#include "system.h"

#include <algorithm>

namespace {
u64 g_sio_trace_counter = 0;
constexpr u8 kOpenBusByte = 0xFF;
constexpr u16 kFallbackBaud = 0x88;
constexpr u32 kControllerAckTicks = 450;
constexpr u32 kMemoryCardAckTicks = 170;
} // namespace

constexpr u32 Sio::ack_ticks(bool memory_card) {
  return memory_card ? kMemoryCardAckTicks : kControllerAckTicks;
}

void Sio::set_button_state(u16 buttons) { controller_.set_button_state(buttons); }

void Sio::set_analog_state(u8 lx, u8 ly, u8 rx, u8 ry) {
  controller_.set_analog_state(lx, ly, rx, ry);
}

void Sio::shutdown() { flush_memory_cards(); }

bool Sio::set_memory_card_slot(u32 slot, const std::string &path) {
  if (slot >= kMemoryCardSlots) {
    return false;
  }

  bool ok = true;
  if (path.empty()) {
    memory_cards_[slot].eject();
  } else {
    ok = memory_cards_[slot].load_or_create(path);
  }
  if (!ok) {
    return false;
  }

  reset_device_transfer_state();
  return true;
}

void Sio::flush_memory_cards() {
  for (MemoryCard &card : memory_cards_) {
    card.save_if_dirty();
  }
}

bool Sio::memory_card_inserted(u32 slot) const {
  if (slot >= kMemoryCardSlots) {
    return false;
  }
  return memory_cards_[slot].inserted();
}

bool Sio::memory_card_dirty(u32 slot) const {
  if (slot >= kMemoryCardSlots) {
    return false;
  }
  return memory_cards_[slot].dirty();
}

std::string Sio::memory_card_path(u32 slot) const {
  if (slot >= kMemoryCardSlots) {
    return {};
  }
  return memory_cards_[slot].path();
}

void Sio::reset_device_transfer_state() {
  active_device_ = ActiveDevice::None;
  active_slot_ = 0;
  controller_.reset_transfer_state();
  for (MemoryCard &card : memory_cards_) {
    card.reset_transfer_state();
  }
}

void Sio::reset() {
  controller_.reset();
  for (MemoryCard &card : memory_cards_) {
    card.reset();
  }
  connected_ = true;

  saw_pad_cmd42_ = false;
  saw_tx_cmd42_ = false;
  saw_pad_id_ = false;
  saw_non_ff_button_byte_ = false;
  saw_full_pad_poll_ = false;
  last_pad_buttons_ = 0xFFFF;
  last_tx_byte_ = 0x00;
  last_rx_byte_ = kOpenBusByte;
  pad_packet_count_ = 0;
  pad_cmd42_count_ = 0;
  pad_poll_count_ = 0;
  channel0_poll_count_ = 0;
  channel1_poll_count_ = 0;
  invalid_sequence_count_ = 0;
  irq_assert_count_ = 0;
  irq_ack_count_ = 0;

  soft_reset();
}

void Sio::rebuild_stat() {
  stat_ = 0;
  if (!transmit_buffer_full_) {
    stat_ |= 0x0001; // TXRDY
  }
  if (receive_buffer_full_) {
    stat_ |= 0x0002; // RXFIFONEMPTY
  }
  if (!transmit_buffer_full_ && transfer_state_ != TransferState::Transmitting) {
    stat_ |= 0x0004; // TXDONE
  }
  if (ack_input_) {
    stat_ |= 0x0080; // ACKINPUT
  }
  if (irq_flag_) {
    stat_ |= 0x0200; // INTR
  }
}

bool Sio::is_transmitting() const { return transfer_state_ != TransferState::Idle; }

bool Sio::can_transfer() const {
  const bool selected = (ctrl_ & 0x0002u) != 0;
  const bool tx_enabled = (ctrl_ & 0x0001u) != 0;
  return transmit_buffer_full_ && selected && tx_enabled;
}

u32 Sio::transfer_ticks() const {
  const u16 baud = (baud_ != 0) ? baud_ : kFallbackBaud;
  return (std::max)(1u, static_cast<u32>(baud) * 8u);
}

void Sio::trigger_irq() {
  irq_flag_ = true;
  if (sys_ != nullptr && !irq_pending_) {
    sys_->irq().request(Interrupt::SIO);
    irq_pending_ = true;
    ++irq_assert_count_;
  }
}

void Sio::end_transfer() {
  transfer_state_ = TransferState::Idle;
  transfer_event_cycles_ = 0;
}

void Sio::soft_reset() {
  if (is_transmitting()) {
    end_transfer();
  }

  mode_ = 0;
  ctrl_ = 0;
  baud_ = 0;
  ack_input_ = false;
  irq_flag_ = false;
  irq_pending_ = false;

  receive_buffer_ = kOpenBusByte;
  receive_buffer_full_ = false;
  transmit_buffer_ = 0x00;
  transmit_value_ = 0x00;
  transmit_buffer_full_ = false;

  reset_device_transfer_state();
  rebuild_stat();
}

void Sio::begin_transfer() {
  transfer_state_ = TransferState::Transmitting;
  transfer_event_cycles_ = static_cast<int>(transfer_ticks());
  transmit_value_ = transmit_buffer_;
  transmit_buffer_full_ = false;
  ctrl_ |= 0x0004u; // RXEN
  rebuild_stat();
}

void Sio::do_transfer() {
  const bool slot1 = (ctrl_ & 0x2000u) != 0;
  const u8 selected_slot = static_cast<u8>(slot1 ? 1u : 0u);
  const u8 host_byte = transmit_value_;
  u8 device_byte = kOpenBusByte;
  bool ack = false;
  bool memory_card_transfer = false;

  auto consume_pad_result = [&](const PadController::TransferResult &pad_result) {
    device_byte = pad_result.data_out;
    ack = pad_result.ack;
    invalid_sequence_count_ += pad_result.invalid_sequences;
    if (pad_result.saw_id) {
      saw_pad_id_ = true;
    }
    if (pad_result.saw_cmd42) {
      saw_pad_cmd42_ = true;
      ++pad_cmd42_count_;
    }
    if (pad_result.non_ff_button) {
      saw_non_ff_button_byte_ = true;
    }
    if (pad_result.poll_complete) {
      saw_full_pad_poll_ = saw_tx_cmd42_;
      last_pad_buttons_ = pad_result.polled_buttons;
      ++pad_packet_count_;
    }
  };

  auto consume_card_result = [&](const MemoryCard::TransferResult &card_result) {
    device_byte = card_result.data_out;
    ack = card_result.ack;
    memory_card_transfer = true;
  };

  switch (active_device_) {
  case ActiveDevice::None:
    if (host_byte == 0x81 && memory_cards_[selected_slot].inserted()) {
      consume_card_result(memory_cards_[selected_slot].transfer(host_byte));
      if (ack) {
        active_device_ = ActiveDevice::MemoryCard;
        active_slot_ = selected_slot;
      }
    } else if (host_byte != 0x81 && !slot1 && connected_) {
      const PadController::TransferResult pad_result = controller_.transfer(host_byte);
      consume_pad_result(pad_result);
      if (host_byte == 0x01 && pad_result.ack) {
        ++pad_poll_count_;
      }
      if (ack) {
        active_device_ = ActiveDevice::Controller;
        active_slot_ = 0;
      }
    }
    break;
  case ActiveDevice::Controller:
    if (!slot1 && connected_ && active_slot_ == 0) {
      consume_pad_result(controller_.transfer(host_byte));
    } else {
      ack = false;
      device_byte = kOpenBusByte;
    }
    break;
  case ActiveDevice::MemoryCard: {
    memory_card_transfer = true;
    if (active_slot_ < kMemoryCardSlots &&
        selected_slot == active_slot_ &&
        memory_cards_[active_slot_].inserted()) {
      consume_card_result(memory_cards_[active_slot_].transfer(host_byte));
    } else {
      ack = false;
      device_byte = kOpenBusByte;
    }
    break;
  }
  }

  if (g_trace_sio &&
      trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                       g_trace_stride_sio)) {
    LOG_DEBUG(
        "SIO: XFER host=0x%02X dev=0x%02X ack=%d slot=%d active=%d state=%d "
        "invalid=%llu",
        host_byte, device_byte, ack ? 1 : 0, slot1 ? 1 : 0,
        static_cast<int>(active_device_), static_cast<int>(transfer_state_),
        static_cast<unsigned long long>(invalid_sequence_count_));
  }

  receive_buffer_ = device_byte;
  receive_buffer_full_ = true;
  last_rx_byte_ = device_byte;

  if ((ctrl_ & 0x0800u) != 0u) {
    trigger_irq();
  }

  if (!ack) {
    active_device_ = ActiveDevice::None;
    active_slot_ = 0;
    end_transfer();
  } else {
    transfer_state_ = TransferState::WaitingForAck;
    transfer_event_cycles_ = static_cast<int>(ack_ticks(memory_card_transfer));
  }

  rebuild_stat();
}

void Sio::do_ack() {
  ack_input_ = true;

  if ((ctrl_ & 0x1000u) != 0u) {
    trigger_irq();
  }

  if (g_trace_sio &&
      trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                       g_trace_stride_sio)) {
    LOG_DEBUG("SIO: ACK pulse stat=0x%04X ctrl=0x%04X", stat_, ctrl_);
  }

  end_transfer();
  rebuild_stat();

  if (can_transfer()) {
    begin_transfer();
  }
}

u8 Sio::read8(u32 offset) const {
  Sio *self = const_cast<Sio *>(this);
  switch (offset) {
  case 0x0: {
    const u8 value = self->receive_buffer_full_ ? self->receive_buffer_ : kOpenBusByte;
    self->receive_buffer_full_ = false;
    self->last_rx_byte_ = value;
    self->rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R8 DATA -> 0x%02X stat=0x%04X ctrl=0x%04X", value,
                self->stat_, self->ctrl_);
    }
    return value;
  }
  default:
    LOG_WARN("SIO: Unhandled read8 at offset 0x%X", offset);
    return kOpenBusByte;
  }
}

u16 Sio::read16(u32 offset) const {
  Sio *self = const_cast<Sio *>(this);
  switch (offset) {
  case 0x0: {
    const u8 value = self->receive_buffer_full_ ? self->receive_buffer_ : kOpenBusByte;
    self->receive_buffer_full_ = false;
    self->last_rx_byte_ = value;
    self->rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R16 DATA -> 0x%04X stat=0x%04X ctrl=0x%04X",
                static_cast<u16>(value) | static_cast<u16>(value << 8),
                self->stat_, self->ctrl_);
    }
    return static_cast<u16>(value) | static_cast<u16>(value << 8);
  }
  case 0x4: {
    const u16 bits = stat_;
    self->ack_input_ = false;
    self->rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R16 STAT -> 0x%04X", bits);
    }
    return bits;
  }
  case 0x8:
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R16 MODE -> 0x%04X", mode_);
    }
    return mode_;
  case 0xA:
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R16 CTRL -> 0x%04X", ctrl_);
    }
    return ctrl_;
  case 0xE:
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R16 BAUD -> 0x%04X", baud_);
    }
    return baud_;
  default:
    LOG_WARN("SIO: Unhandled read16 at offset 0x%X", offset);
    return 0;
  }
}

u32 Sio::read32(u32 offset) const {
  Sio *self = const_cast<Sio *>(this);
  switch (offset) {
  case 0x0: {
    const u8 value = self->receive_buffer_full_ ? self->receive_buffer_ : kOpenBusByte;
    self->receive_buffer_full_ = false;
    self->last_rx_byte_ = value;
    self->rebuild_stat();
    const u32 full = static_cast<u32>(value) | (static_cast<u32>(value) << 8) |
                     (static_cast<u32>(value) << 16) |
                     (static_cast<u32>(value) << 24);
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R32 DATA -> 0x%08X stat=0x%04X ctrl=0x%04X", full,
                self->stat_, self->ctrl_);
    }
    return full;
  }
  case 0x4: {
    const u32 bits = stat_;
    self->ack_input_ = false;
    self->rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R32 STAT -> 0x%08X", bits);
    }
    return bits;
  }
  case 0x8:
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R32 MODE/CTRL -> 0x%08X",
                (static_cast<u32>(ctrl_) << 16) | mode_);
    }
    return (static_cast<u32>(ctrl_) << 16) | mode_;
  default:
    LOG_WARN("SIO: Unhandled read32 at offset 0x%X", offset);
    return 0;
  }
}

void Sio::write8(u32 offset, u8 val) {
  switch (offset) {
  case 0x0: {
    last_tx_byte_ = val;

    const bool selected = ((ctrl_ & 0x0002u) != 0u) && ((ctrl_ & 0x0001u) != 0u);
    const bool slot1 = (ctrl_ & 0x2000u) != 0u;
    if (val == 0x01 && selected) {
      if (slot1) {
        ++channel1_poll_count_;
      } else {
        ++channel0_poll_count_;
      }
    }
    if (val == 0x42 && selected && !slot1) {
      saw_tx_cmd42_ = true;
    }

    if (transmit_buffer_full_) {
      ++invalid_sequence_count_;
    }
    transmit_buffer_ = val;
    transmit_buffer_full_ = true;

    if ((ctrl_ & 0x0400u) != 0u) {
      trigger_irq();
    }

    if (!is_transmitting() && can_transfer()) {
      begin_transfer();
    }

    rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG(
          "SIO: W8 DATA=0x%02X stat=0x%04X ctrl=0x%04X sel=%d txen=%d slot=%d "
          "state=%d",
          val, stat_, ctrl_, (ctrl_ & 0x0002u) ? 1 : 0, (ctrl_ & 0x0001u) ? 1 : 0,
          slot1 ? 1 : 0, static_cast<int>(transfer_state_));
    }
    break;
  }
  default:
    LOG_WARN("SIO: Unhandled write8 at offset 0x%X = 0x%02X", offset, val);
    break;
  }
}

void Sio::write16(u32 offset, u16 val) {
  switch (offset) {
  case 0x0:
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: W16 DATA=0x%04X", val);
    }
    write8(0x0, static_cast<u8>(val & 0xFFu));
    break;

  case 0x8:
    mode_ = val;
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: W16 MODE=0x%04X", mode_);
    }
    break;

  case 0xA: {
    ctrl_ = static_cast<u16>(val & ~0x0050u);

    if ((val & 0x0040u) != 0u) {
      soft_reset();
      if (g_trace_sio &&
          trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                           g_trace_stride_sio)) {
        LOG_DEBUG("SIO: W16 CTRL=0x%04X -> SOFT RESET", val);
      }
      break;
    }

    if ((val & 0x0010u) != 0u) {
      ++irq_ack_count_;
      irq_flag_ = false;
      irq_pending_ = false;
    }

    if ((ctrl_ & 0x0002u) == 0u) {
      reset_device_transfer_state();
    }

    if ((ctrl_ & 0x0002u) == 0u || (ctrl_ & 0x0001u) == 0u) {
      if (is_transmitting()) {
        end_transfer();
      }
    } else if (!is_transmitting() && can_transfer()) {
      begin_transfer();
    }

    rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG(
          "SIO: W16 CTRL=0x%04X latched=0x%04X sel=%d txen=%d ackint=%d slot=%d "
          "state=%d stat=0x%04X",
          val, ctrl_, (ctrl_ & 0x0002u) ? 1 : 0, (ctrl_ & 0x0001u) ? 1 : 0,
          (ctrl_ & 0x1000u) ? 1 : 0, (ctrl_ & 0x2000u) ? 1 : 0,
          static_cast<int>(transfer_state_), stat_);
    }
    break;
  }

  case 0xE:
    baud_ = val;
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: W16 BAUD=0x%04X", baud_);
    }
    break;

  default:
    LOG_WARN("SIO: Unhandled write16 at offset 0x%X = 0x%04X", offset, val);
    break;
  }
}

void Sio::write32(u32 offset, u32 val) {
  switch (offset) {
  case 0x0:
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: W32 DATA=0x%08X", val);
    }
    write8(0x0, static_cast<u8>(val & 0xFFu));
    break;

  case 0x8:
    write16(0x8, static_cast<u16>(val & 0xFFFFu));
    write16(0xA, static_cast<u16>((val >> 16) & 0xFFFFu));
    break;

  default:
    LOG_WARN("SIO: Unhandled write32 at offset 0x%X = 0x%08X", offset, val);
    break;
  }
}

void Sio::tick(u32 cycles) {
  u32 remaining = cycles;
  while (remaining > 0 && is_transmitting()) {
    if (transfer_event_cycles_ <= 0) {
      transfer_event_cycles_ = 1;
    }

    const u32 step = static_cast<u32>(transfer_event_cycles_);
    if (remaining < step) {
      transfer_event_cycles_ -= static_cast<int>(remaining);
      break;
    }

    remaining -= step;
    transfer_event_cycles_ = 0;

    if (transfer_state_ == TransferState::Transmitting) {
      do_transfer();
    } else if (transfer_state_ == TransferState::WaitingForAck) {
      do_ack();
    }
  }
}
