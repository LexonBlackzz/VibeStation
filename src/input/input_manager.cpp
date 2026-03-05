#include "input_manager.h"

InputManager::InputManager() { set_default_bindings(); }

void InputManager::set_default_bindings() {
  key_bindings_.clear();
  // D-Pad → Arrow keys
  key_bindings_[SDL_SCANCODE_UP] = PsxButton::Up;
  key_bindings_[SDL_SCANCODE_DOWN] = PsxButton::Down;
  key_bindings_[SDL_SCANCODE_LEFT] = PsxButton::Left;
  key_bindings_[SDL_SCANCODE_RIGHT] = PsxButton::Right;
  // Face buttons → Z/X/A/S
  key_bindings_[SDL_SCANCODE_Z] = PsxButton::Cross;
  key_bindings_[SDL_SCANCODE_X] = PsxButton::Circle;
  key_bindings_[SDL_SCANCODE_A] = PsxButton::Square;
  key_bindings_[SDL_SCANCODE_S] = PsxButton::Triangle;
  // Shoulder buttons → Q/W/E/R
  key_bindings_[SDL_SCANCODE_Q] = PsxButton::L1;
  key_bindings_[SDL_SCANCODE_W] = PsxButton::R1;
  key_bindings_[SDL_SCANCODE_E] = PsxButton::L2;
  key_bindings_[SDL_SCANCODE_R] = PsxButton::R2;
  // Start/Select → Enter/Backspace
  key_bindings_[SDL_SCANCODE_RETURN] = PsxButton::Start;
  key_bindings_[SDL_SCANCODE_BACKSPACE] = PsxButton::Select;
}

void InputManager::set_key_binding(SDL_Scancode key, PsxButton button) {
  key_bindings_.erase(key);
  // Remove any existing binding for this button
  for (auto it = key_bindings_.begin(); it != key_bindings_.end();) {
    if (it->second == button) {
      it = key_bindings_.erase(it);
    } else {
      ++it;
    }
  }
  key_bindings_[key] = button;
}

void InputManager::clear_key_binding(PsxButton button) {
  for (auto it = key_bindings_.begin(); it != key_bindings_.end();) {
    if (it->second == button) {
      it = key_bindings_.erase(it);
    } else {
      ++it;
    }
  }
}

SDL_Scancode InputManager::key_for_button(PsxButton button) const {
  for (const auto &entry : key_bindings_) {
    if (entry.second == button) {
      return entry.first;
    }
  }
  return SDL_SCANCODE_UNKNOWN;
}

void InputManager::process_event(const SDL_Event &event) {
  switch (event.type) {
  case SDL_KEYDOWN: {
    auto it = key_bindings_.find(event.key.keysym.scancode);
    if (it != key_bindings_.end()) {
      controller_.press(it->second);
    }
    break;
  }
  case SDL_KEYUP: {
    auto it = key_bindings_.find(event.key.keysym.scancode);
    if (it != key_bindings_.end()) {
      controller_.release(it->second);
    }
    break;
  }
  case SDL_CONTROLLERDEVICEADDED:
    if (!gamepad_) {
      open_gamepad(event.cdevice.which);
    }
    break;
  case SDL_CONTROLLERDEVICEREMOVED:
    if (gamepad_ && event.cdevice.which == gamepad_id_) {
      close_gamepad();
    }
    break;
  default:
    break;
  }
}

void InputManager::update() {
  if (gamepad_) {
    apply_gamepad_state();
  }
}

void InputManager::open_gamepad(int device_index) {
  gamepad_ = SDL_GameControllerOpen(device_index);
  if (gamepad_) {
    gamepad_id_ =
        SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(gamepad_));
    LOG_INFO("Input: Gamepad connected — %s", SDL_GameControllerName(gamepad_));
  }
}

void InputManager::close_gamepad() {
  if (gamepad_) {
    LOG_INFO("Input: Gamepad disconnected");
    SDL_GameControllerClose(gamepad_);
    gamepad_ = nullptr;
    gamepad_id_ = -1;
  }
}

std::string InputManager::gamepad_name() const {
  if (gamepad_)
    return SDL_GameControllerName(gamepad_);
  return "None";
}

void InputManager::apply_gamepad_state() {
  // Face buttons
  auto btn = [&](SDL_GameControllerButton sdl_btn, PsxButton psx_btn) {
    if (SDL_GameControllerGetButton(gamepad_, sdl_btn)) {
      controller_.press(psx_btn);
    } else {
      controller_.release(psx_btn);
    }
  };

  btn(SDL_CONTROLLER_BUTTON_A, PsxButton::Cross);
  btn(SDL_CONTROLLER_BUTTON_B, PsxButton::Circle);
  btn(SDL_CONTROLLER_BUTTON_X, PsxButton::Square);
  btn(SDL_CONTROLLER_BUTTON_Y, PsxButton::Triangle);
  btn(SDL_CONTROLLER_BUTTON_BACK, PsxButton::Select);
  btn(SDL_CONTROLLER_BUTTON_START, PsxButton::Start);
  btn(SDL_CONTROLLER_BUTTON_LEFTSHOULDER, PsxButton::L1);
  btn(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER, PsxButton::R1);
  btn(SDL_CONTROLLER_BUTTON_LEFTSTICK, PsxButton::L3);
  btn(SDL_CONTROLLER_BUTTON_RIGHTSTICK, PsxButton::R3);
  btn(SDL_CONTROLLER_BUTTON_DPAD_UP, PsxButton::Up);
  btn(SDL_CONTROLLER_BUTTON_DPAD_DOWN, PsxButton::Down);
  btn(SDL_CONTROLLER_BUTTON_DPAD_LEFT, PsxButton::Left);
  btn(SDL_CONTROLLER_BUTTON_DPAD_RIGHT, PsxButton::Right);

  // Triggers → L2/R2
  s16 lt = SDL_GameControllerGetAxis(gamepad_, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
  s16 rt =
      SDL_GameControllerGetAxis(gamepad_, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
  if (lt > 8000)
    controller_.press(PsxButton::L2);
  else
    controller_.release(PsxButton::L2);
  if (rt > 8000)
    controller_.press(PsxButton::R2);
  else
    controller_.release(PsxButton::R2);

  // Analog sticks
  s16 lx = SDL_GameControllerGetAxis(gamepad_, SDL_CONTROLLER_AXIS_LEFTX);
  s16 ly = SDL_GameControllerGetAxis(gamepad_, SDL_CONTROLLER_AXIS_LEFTY);
  s16 rx = SDL_GameControllerGetAxis(gamepad_, SDL_CONTROLLER_AXIS_RIGHTX);
  s16 ry = SDL_GameControllerGetAxis(gamepad_, SDL_CONTROLLER_AXIS_RIGHTY);

  controller_.set_left_stick(axis_to_u8(lx), axis_to_u8(ly));
  controller_.set_right_stick(axis_to_u8(rx), axis_to_u8(ry));
}

u8 InputManager::axis_to_u8(s16 value) const {
  if (value > -DEADZONE && value < DEADZONE)
    return 0x80;
  // Map -32768..32767 to 0..255
  return static_cast<u8>((value + 32768) >> 8);
}
