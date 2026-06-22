#pragma once

#include "types.h"

#include <array>

class PadController {
public:
  struct TransferResult {
    u8 data_out = 0xFF;
    bool ack = false;
    bool saw_id = false;
    bool saw_cmd42 = false;
    bool poll_complete = false;
    bool non_ff_button = false;
    u8 invalid_sequences = 0;
    u16 polled_buttons = 0xFFFF;
  };

  void reset();
  void reset_transfer_state();

  void set_button_state(u16 buttons) { button_state_ = buttons; }
  u16 button_state() const { return button_state_; }
  void set_analog_state(u8 lx, u8 ly, u8 rx, u8 ry);
  u8 analog_lx() const { return analog_lx_; }
  u8 analog_ly() const { return analog_ly_; }
  u8 analog_rx() const { return analog_rx_; }
  u8 analog_ry() const { return analog_ry_; }

  TransferResult transfer(u8 data_in);

private:
  enum class Command : u8 {
    Idle,
    Ready,
    ReadPad,           // 0x42
    ConfigModeSetMode, // 0x43
    SetAnalogMode,     // 0x44
    GetAnalogMode,     // 0x45
    Command46,         // 0x46
    Command47,         // 0x47
    Command4C,         // 0x4C
    GetSetRumble,      // 0x4D
  };

  void clear_command_buffers();
  void poll_controller();
  u8 response_halfwords() const;
  u8 mode_id() const;
  u8 id_byte() const;
  void set_analog_mode(bool enabled);

  Command command_ = Command::Idle;
  u8 command_step_ = 0;
  u8 response_length_ = 0;
  std::array<u8, 8> rx_buffer_{};
  std::array<u8, 8> tx_buffer_{};
  std::array<u8, 6> rumble_config_{};

  u16 button_state_ = 0xFFFF;
  u8 analog_lx_ = 0x80;
  u8 analog_ly_ = 0x80;
  u8 analog_rx_ = 0x80;
  u8 analog_ry_ = 0x80;

  bool analog_mode_ = false;
  bool analog_locked_ = false;
  bool config_mode_ = false;
  bool dualshock_enabled_ = false;
  u8 status_byte_ = 0x5A;
};
