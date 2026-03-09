#include "sio.h"
#include "system.h"

namespace {
u64 g_sio_trace_counter = 0;
constexpr int kDsrPulseCycles = 256;
}

void Sio::set_analog_state(u8 lx, u8 ly, u8 rx, u8 ry) {
  analog_lx_ = lx;
  analog_ly_ = ly;
  analog_rx_ = rx;
  analog_ry_ = ry;
}

void Sio::reset() {
  tx_data_ = 0;
  rx_fifo_ = 0xFF;
  mode_ = 0;
  ctrl_ = 0;
  baud_ = 0;
  tx_ready_1_ = true;
  rx_not_empty_ = false;
  tx_ready_2_ = true;
  dsr_input_level_ = false;
  irq_flag_ = false;
  dsr_pending_cycles_ = 0;
  dsr_pulse_cycles_ = 0;
  irq_pending_ = false;
  joy_select_active_ = false;
  byte_index_ = 0;
  transfer_active_ = false;
  rx_pending_valid_ = false;
  rx_pending_byte_ = 0xFF;
  tx_queued_valid_ = false;
  tx_queued_byte_ = 0xFF;
  state_ = State::Idle;
  saw_pad_cmd42_ = false;
  saw_tx_cmd42_ = false;
  saw_pad_id_ = false;
  saw_non_ff_button_byte_ = false;
  saw_full_pad_poll_ = false;
  last_pad_buttons_ = 0xFFFF;
  last_tx_byte_ = 0x00;
  last_rx_byte_ = 0xFF;
  pad_packet_count_ = 0;
  pad_cmd42_count_ = 0;
  pad_poll_count_ = 0;
  channel0_poll_count_ = 0;
  channel1_poll_count_ = 0;
  invalid_sequence_count_ = 0;
  irq_assert_count_ = 0;
  irq_ack_count_ = 0;
  rebuild_stat();
}

void Sio::rebuild_stat() {
  stat_ = 0;
  if (tx_ready_1_) {
    stat_ |= 0x0001;
  }
  if (rx_not_empty_) {
    stat_ |= 0x0002;
  }
  if (tx_ready_2_) {
    stat_ |= 0x0004;
  }
  if (dsr_input_level_) {
    stat_ |= 0x0080;
  }
  if (irq_flag_) {
    stat_ |= 0x0200;
  }
}

bool Sio::tx_enabled() const {
  return (ctrl_ & 0x0001) != 0;
}

void Sio::raise_sio_irq_if_enabled() {
  if (!irq_flag_) {
    return;
  }
  if ((ctrl_ & 0x1000) == 0) {
    return;
  }
  if (sys_ != nullptr && !irq_pending_) {
    sys_->irq().request(Interrupt::SIO);
    irq_pending_ = true;
    ++irq_assert_count_;
  }
}

void Sio::schedule_dsr_pulse() {
  const int baud_div = (baud_ != 0) ? static_cast<int>(baud_) : 0x88;
  // Approximate one serial byte time from baud divider so BIOS can observe
  // command-busy/ready phases in the expected order.
  dsr_pending_cycles_ = (std::max)(32, baud_div * 8);
  dsr_pulse_cycles_ = kDsrPulseCycles;
  dsr_input_level_ = false;
  rebuild_stat();
}

u8 Sio::read8(u32 offset) const {
  Sio *self = const_cast<Sio *>(this);
  switch (offset) {
  case 0x0: {
    const u8 value = rx_fifo_;
    self->last_rx_byte_ = value;
    self->rx_not_empty_ = false;
    self->rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R8 DATA -> 0x%02X stat=0x%04X ctrl=0x%04X state=%d", value,
                self->stat_, self->ctrl_, static_cast<int>(self->state_));
    }
    return value;
  }
  default:
    LOG_WARN("SIO: Unhandled read8 at offset 0x%X", offset);
    return 0xFF;
  }
}

u16 Sio::read16(u32 offset) const {
  Sio *self = const_cast<Sio *>(this);
  switch (offset) {
  case 0x0: {
    const u8 value = rx_fifo_;
    self->last_rx_byte_ = value;
    self->rx_not_empty_ = false;
    self->rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R16 DATA -> 0x%04X stat=0x%04X ctrl=0x%04X state=%d",
                static_cast<u16>(value), self->stat_, self->ctrl_,
                static_cast<int>(self->state_));
    }
    return value;
  }
  case 0x4:
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R16 STAT -> 0x%04X", stat_);
    }
    return stat_;
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
    const u8 value = rx_fifo_;
    self->last_rx_byte_ = value;
    self->rx_not_empty_ = false;
    self->rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R32 DATA -> 0x%08X stat=0x%04X ctrl=0x%04X state=%d",
                static_cast<u32>(value), self->stat_, self->ctrl_,
                static_cast<int>(self->state_));
    }
    return value;
  }
  case 0x4:
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: R32 STAT -> 0x%08X", static_cast<u32>(stat_));
    }
    return stat_;
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
  case 0x0: { // TX Data
    tx_data_ = val;
    last_tx_byte_ = val;
    const bool joy_bus_selected = joy_select_active_ && connected_ && tx_enabled();
    const bool channel0 = (ctrl_ & 0x2000u) == 0;
    const bool channel1 = !channel0;
    if (val == 0x01 && joy_bus_selected) {
      if (channel0) {
        ++channel0_poll_count_;
      } else {
        ++channel1_poll_count_;
      }
    }
    if (val == 0x42 && joy_bus_selected) {
      saw_tx_cmd42_ = true;
    }

    if (!joy_bus_selected) {
      // Open-bus response when JOY bus is not selected/enabled.
      dsr_pending_cycles_ = 0;
      dsr_pulse_cycles_ = 0;
      dsr_input_level_ = false;
      irq_flag_ = false;
      rx_fifo_ = 0xFF;
      last_rx_byte_ = rx_fifo_;
      rx_not_empty_ = true;
      tx_ready_1_ = !tx_queued_valid_;
      tx_ready_2_ = true;
      transfer_active_ = false;
      rx_pending_valid_ = false;
      rebuild_stat();
      break;
    }

    if (channel1 || val == 0x81) {
      // This core only emulates a pad on channel 0.
      // Treat channel 1 and memory-card selects as no-device open bus without
      // entering controller protocol state.
      state_ = State::Idle;
      byte_index_ = 0;
      dsr_pending_cycles_ = 0;
      dsr_pulse_cycles_ = 0;
      dsr_input_level_ = false;
      irq_flag_ = false;
      irq_pending_ = false;
      rx_fifo_ = 0xFF;
      last_rx_byte_ = rx_fifo_;
      rx_not_empty_ = true;
      tx_ready_1_ = !tx_queued_valid_;
      tx_ready_2_ = true;
      transfer_active_ = false;
      rx_pending_valid_ = false;
      rebuild_stat();
      break;
    }

    // Do not abort an in-flight transfer on early host writes. Dropping the
    // byte can desync BIOS pad polling; queue one byte to execute right after
    // the current transfer completes.
    if (transfer_active_ || !tx_ready_1_) {
      if (!tx_queued_valid_) {
        tx_queued_byte_ = val;
        tx_queued_valid_ = true;
      } else {
        ++invalid_sequence_count_;
        tx_queued_byte_ = val;
      }
      tx_ready_1_ = false;
      tx_ready_2_ = false;
      rebuild_stat();
      break;
    }

    // Keep protocol active on whichever channel BIOS currently selects.
    // Memory-card probes (0x81) are treated as "no card connected" so they
    // can't poison pad transaction state.
    const ByteResult result = process_byte(val);
    tx_ready_1_ = !tx_queued_valid_;
    tx_ready_2_ = false;
    transfer_active_ = true;
    rx_pending_byte_ = result.response;
    rx_pending_valid_ = true;
    rx_not_empty_ = false;
    if (result.pulse_dsr) {
      schedule_dsr_pulse();
    } else {
      dsr_pending_cycles_ = 0;
      dsr_pulse_cycles_ = 0;
      dsr_input_level_ = false;
      irq_flag_ = false;
      rx_fifo_ = rx_pending_byte_;
      last_rx_byte_ = rx_fifo_;
      rx_not_empty_ = true;
      tx_ready_1_ = !tx_queued_valid_;
      tx_ready_2_ = true;
      transfer_active_ = false;
      rx_pending_valid_ = false;
    }
    ++byte_index_;
    rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG(
          "SIO: W8 DATA=0x%02X -> RX=0x%02X state=%d stat=0x%04X ctrl=0x%04X "
          "sel=%d txen=%d ch=%d txrdy=%d invalid=%llu",
          val, rx_fifo_, static_cast<int>(state_), stat_, ctrl_,
          joy_select_active_ ? 1 : 0, tx_enabled() ? 1 : 0,
          ((ctrl_ & 0x2000u) != 0) ? 1 : 0, tx_ready_1_ ? 1 : 0,
          static_cast<unsigned long long>(invalid_sequence_count_));
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
    write8(0x0, static_cast<u8>(val & 0xFF));
    break;
  case 0x8:
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: W16 MODE=0x%04X", val);
    }
    mode_ = val;
    break;
  case 0xA:
    {
      ctrl_ = static_cast<u16>(val & ~0x0050u);
      joy_select_active_ = (ctrl_ & 0x0002) != 0;
      if (!joy_select_active_) { // /JOY_SELECT deasserted
        state_ = State::Idle;
        byte_index_ = 0;
        transfer_active_ = false;
        rx_pending_valid_ = false;
        dsr_pending_cycles_ = 0;
        dsr_pulse_cycles_ = 0;
        dsr_input_level_ = false;
        tx_queued_valid_ = false;
        tx_queued_byte_ = 0xFF;
        tx_ready_1_ = !tx_queued_valid_;
        tx_ready_2_ = true;
      }
    }
    if (val & 0x0010) { // Acknowledge
      ++irq_ack_count_;
      irq_pending_ = false;
      irq_flag_ = false;
      if (dsr_pulse_cycles_ <= 0) {
        dsr_input_level_ = false;
      }
    }
    if (val & 0x0040) { // Reset
      state_ = State::Idle;
      byte_index_ = 0;
      tx_ready_1_ = !tx_queued_valid_;
      tx_ready_2_ = true;
      rx_not_empty_ = false;
      rx_fifo_ = 0xFF;
      dsr_input_level_ = false;
      irq_flag_ = false;
      irq_pending_ = false;
      dsr_pending_cycles_ = 0;
      dsr_pulse_cycles_ = 0;
      tx_queued_valid_ = false;
      tx_queued_byte_ = 0xFF;
      mode_ = 0;
      ctrl_ = 0;
      baud_ = 0;
      joy_select_active_ = false;
    }
    rebuild_stat();
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG(
          "SIO: W16 CTRL=0x%04X latched=0x%04X sel=%d txen=%d irqen=%d ack=%d "
          "rst=%d state=%d stat=0x%04X",
          val, ctrl_, joy_select_active_ ? 1 : 0, tx_enabled() ? 1 : 0,
          (ctrl_ & 0x1000) ? 1 : 0, (val & 0x0010) ? 1 : 0,
          (val & 0x0040) ? 1 : 0, static_cast<int>(state_), stat_);
    }
    break;
  case 0xE:
    baud_ = val;
    if (g_trace_sio &&
        trace_should_log(g_sio_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: BAUD=0x%04X", baud_);
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
    write8(0, static_cast<u8>(val));
    break;
  case 0x8:
    write16(0x8, static_cast<u16>(val));
    write16(0xA, static_cast<u16>(val >> 16));
    break;
  default:
    LOG_WARN("SIO: Unhandled write32 at offset 0x%X = 0x%08X", offset, val);
    break;
  }
}

Sio::ByteResult Sio::process_byte(u8 value) {
  // BIOS can restart polling after aborted/failed exchanges without always
  // giving us a clean state transition; treat a fresh 0x01 as resync.
  if (value == 0x01 && state_ != State::Idle) {
    state_ = State::SelectedPad;
    ++pad_poll_count_;
    return ByteResult{0xFF, true};
  }

  switch (state_) {
  case State::Idle:
    if (value == 0x01) { // Pad select
      state_ = State::SelectedPad;
      ++pad_poll_count_;
      // BIOS expects DSR/IRQ pacing after the initial select byte before it
      // transmits 0x42; without this pulse it retries 0x01 forever.
      return ByteResult{0xFF, true};
    }
    ++invalid_sequence_count_;
    return ByteResult{0xFF, false};

  case State::SelectedPad:
    if (value == 0x42) { // Read pad command
      state_ = State::SendingID_Hi;
      saw_pad_cmd42_ = true;
      saw_pad_id_ = true;
      ++pad_cmd42_count_;
      // Return pad ID: 0x41 = digital, 0x73 = analog/DualShock
      return ByteResult{analog_mode_ ? static_cast<u8>(0x73)
                                     : static_cast<u8>(0x41),
                        true};
    }
    ++invalid_sequence_count_;
    state_ = State::Idle;
    return ByteResult{0xFF, false};

  case State::SendingID_Hi:
    state_ = State::SendingID_Lo;
    saw_pad_id_ = true;
    return ByteResult{0x5A, true}; // ID byte 2 (always 0x5A)

  case State::SendingID_Lo:
    state_ = State::SendingButtons_Lo;
    if ((button_state_ & 0xFFu) != 0xFFu) {
      saw_non_ff_button_byte_ = true;
    }
    return ByteResult{static_cast<u8>(button_state_ & 0xFF), true};

  case State::SendingButtons_Lo:
    {
      const bool has_more = analog_mode_;
      if (analog_mode_) {
        state_ = State::SendingButtons_Hi;
      } else {
        state_ = State::Idle;
      }
      const u8 hi = static_cast<u8>((button_state_ >> 8) & 0xFF);
      if ((hi & 0xFFu) != 0xFFu) {
        saw_non_ff_button_byte_ = true;
      }
      saw_full_pad_poll_ = saw_tx_cmd42_;
      last_pad_buttons_ = button_state_;
      ++pad_packet_count_;
      return ByteResult{hi, has_more};
    }

  case State::SendingButtons_Hi:
    state_ = State::SendingAnalog_RX;
    return ByteResult{analog_rx_, true};

  case State::SendingAnalog_RX:
    state_ = State::SendingAnalog_RY;
    return ByteResult{analog_ry_, true};

  case State::SendingAnalog_RY:
    state_ = State::SendingAnalog_LX;
    return ByteResult{analog_lx_, true};

  case State::SendingAnalog_LX:
    state_ = State::SendingAnalog_LY;
    return ByteResult{analog_ly_, true};

  case State::SendingAnalog_LY:
    state_ = State::Idle;
    return ByteResult{0xFF, false};

  default:
    state_ = State::Idle;
    return ByteResult{0xFF, false};
  }
}

void Sio::tick(u32 cycles) {
  bool stat_dirty = false;
  if (dsr_pending_cycles_ > 0) {
    dsr_pending_cycles_ -= static_cast<int>(cycles);
    if (dsr_pending_cycles_ <= 0) {
      if (rx_pending_valid_) {
        rx_fifo_ = rx_pending_byte_;
        last_rx_byte_ = rx_fifo_;
        rx_not_empty_ = true;
        rx_pending_valid_ = false;
      }
      tx_ready_1_ = !tx_queued_valid_;
      tx_ready_2_ = true;
      transfer_active_ = false;
      dsr_input_level_ = true;
      irq_flag_ = true;
      raise_sio_irq_if_enabled();
      stat_dirty = true;

      if (tx_queued_valid_) {
        if (joy_select_active_ && connected_ && tx_enabled()) {
          const u8 queued = tx_queued_byte_;
          tx_queued_valid_ = false;
          tx_queued_byte_ = 0xFF;
          const ByteResult result = process_byte(queued);
          tx_ready_1_ = !tx_queued_valid_;
          tx_ready_2_ = false;
          transfer_active_ = true;
          rx_pending_byte_ = result.response;
          rx_pending_valid_ = true;
          rx_not_empty_ = false;
          if (result.pulse_dsr) {
            schedule_dsr_pulse();
          } else {
            dsr_pending_cycles_ = 0;
            dsr_pulse_cycles_ = 0;
            dsr_input_level_ = false;
            irq_flag_ = false;
            rx_fifo_ = rx_pending_byte_;
            last_rx_byte_ = rx_fifo_;
            rx_not_empty_ = true;
            tx_ready_1_ = !tx_queued_valid_;
            tx_ready_2_ = true;
            transfer_active_ = false;
            rx_pending_valid_ = false;
          }
          ++byte_index_;
        } else {
          tx_queued_valid_ = false;
          tx_queued_byte_ = 0xFF;
        }
      }
    }
  }
  if (dsr_pulse_cycles_ > 0) {
    dsr_pulse_cycles_ -= static_cast<int>(cycles);
    if (dsr_pulse_cycles_ <= 0) {
      dsr_pulse_cycles_ = 0;
      dsr_input_level_ = false;
      stat_dirty = true;
    }
  }
  if (stat_dirty) {
    rebuild_stat();
  }
}
