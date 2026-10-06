#pragma once

// Entry point for running VibeStation 2 (the PS2 app) inside VibeStation's
// window, next to VibeStation 1. Deliberately free of PS2 headers: the main
// executable builds as C++17 while the PS2 code needs C++20.

#include <memory>
#include <string>

struct SDL_Window;

namespace ps2::ui {

class Ps2App;

class Vibestation2 {
public:
    Vibestation2();
    ~Vibestation2();
    Vibestation2(const Vibestation2&) = delete;
    Vibestation2& operator=(const Vibestation2&) = delete;

    // Sets up the PS2 app in an existing window and OpenGL context.
    bool init(SDL_Window* window, void* gl_context, int gl_major, int gl_minor,
              const char* glsl, bool opengl2);
    // Switching to and from VibeStation 2.
    void activate();
    void deactivate();
    // One frame; false when the user quit.
    bool frame();
    // True once, after the user chose "VibeStation 1" and the screen faded.
    bool take_switch_request();
    // A PS1 disc to boot in VibeStation 1 after that switch (empty if none).
    std::string take_ps1_disc();
    // Starts the fade back to VibeStation 1 (--switch-test).
    void begin_switch_back();
    void shutdown();

private:
    std::unique_ptr<Ps2App> app_;
};

} // namespace ps2::ui
