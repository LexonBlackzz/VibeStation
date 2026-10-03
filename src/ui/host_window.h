#pragma once

struct SDL_Window;

// The one window shared by VibeStation 1 (the PS1 app) and VibeStation 2
// (the PS2 app). main() creates it; each app runs inside it with its own
// ImGui context, and only one of them draws at a time.
struct HostWindow {
    SDL_Window* window = nullptr;
    void* gl_context = nullptr; // SDL_GLContext
    int gl_major = 0;
    int gl_minor = 0;
    const char* glsl = "#version 330"; // for the ImGui OpenGL3 backend
    bool opengl2 = false;              // use the ImGui OpenGL2 backend instead
};

// Initialises SDL and creates the window and OpenGL context (4.3 core first,
// so VibeStation 2's GPU renderer can share it, then 3.3, 3.2, 2.1). The
// window is shown already painted black.
bool create_host_window(HostWindow& host, const char* title);
void destroy_host_window(HostWindow& host);
