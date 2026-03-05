#pragma once
#include "../core/types.h"
#include "controller.h"
#include <SDL.h>
#include <string>
#include <unordered_map>

// ── Input Manager ──────────────────────────────────────────────────
// Bridges SDL2 keyboard/gamepad events to the PS1 controller.
// Supports configurable key bindings and automatic gamepad mapping.

struct KeyBinding {
  SDL_Scancode key;
  PsxButton button;
};

class InputManager {
public:
  InputManager();

  // Process SDL events
  void process_event(const SDL_Event &event);

  // Update controller from current gamepad state
  void update();

  // Get the controller state
  const Controller &controller() const { return controller_; }

  // Binding management
  void set_key_binding(SDL_Scancode key, PsxButton button);
  void clear_key_binding(PsxButton button);
  SDL_Scancode key_for_button(PsxButton button) const;
  void set_default_bindings();

  // Gamepad
  void open_gamepad(int device_index);
  void close_gamepad();
  bool has_gamepad() const { return gamepad_ != nullptr; }
  std::string gamepad_name() const;

  // Get current bindings for UI display
  const std::unordered_map<SDL_Scancode, PsxButton> &key_bindings() const {
    return key_bindings_;
  }

private:
  Controller controller_;
  SDL_GameController *gamepad_ = nullptr;
  SDL_JoystickID gamepad_id_ = -1;

  // Keyboard bindings: scancode → PS1 button
  std::unordered_map<SDL_Scancode, PsxButton> key_bindings_;

  // Gamepad button mapping (using SDL_GameController standard)
  void apply_gamepad_state();

  // Analog stick deadzone
  static constexpr s16 DEADZONE = 4096;
  u8 axis_to_u8(s16 value) const;
};
