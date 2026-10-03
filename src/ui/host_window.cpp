#include "ui/host_window.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <cstdio>

bool create_host_window(HostWindow& host, const char* title) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER | SDL_INIT_AUDIO) != 0) {
        std::fprintf(stderr, "[HostWindow] SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    struct Attempt {
        int major;
        int minor;
        int profile;
        const char* glsl;
        bool opengl2;
        const char* label;
    };
    const Attempt attempts[] = {
        {4, 3, SDL_GL_CONTEXT_PROFILE_CORE, "#version 330", false, "OpenGL 4.3 Core"},
        {3, 3, SDL_GL_CONTEXT_PROFILE_CORE, "#version 330", false, "OpenGL 3.3 Core"},
        {3, 2, SDL_GL_CONTEXT_PROFILE_CORE, "#version 150", false, "OpenGL 3.2 Core"},
        {2, 1, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY, "#version 120", true, "OpenGL 2.1 Compatibility"},
    };

    for (const Attempt& attempt : attempts) {
        SDL_GL_ResetAttributes();
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, attempt.major);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, attempt.minor);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, attempt.profile);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

        // Hidden until it has been painted black: no blank flash at launch.
        host.window = SDL_CreateWindow(
            title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 800,
            SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI |
                SDL_WINDOW_HIDDEN);
        if (!host.window) {
            std::printf("[HostWindow] SDL_CreateWindow failed for %s: %s\n", attempt.label, SDL_GetError());
            continue;
        }
        host.gl_context = SDL_GL_CreateContext(host.window);
        if (!host.gl_context) {
            std::printf("[HostWindow] GL context failed for %s: %s\n", attempt.label, SDL_GetError());
            SDL_DestroyWindow(host.window);
            host.window = nullptr;
            continue;
        }

        SDL_GL_MakeCurrent(host.window, host.gl_context);
        host.gl_major = attempt.major;
        host.gl_minor = attempt.minor;
        host.glsl = attempt.glsl;
        host.opengl2 = attempt.opengl2;

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        SDL_GL_SwapWindow(host.window);
        SDL_ShowWindow(host.window);

        const GLubyte* version = glGetString(GL_VERSION);
        std::printf("[HostWindow] %s (driver: %s)\n", attempt.label,
                    version ? reinterpret_cast<const char*>(version) : "unknown");
        std::fflush(stdout);
        return true;
    }

    std::fprintf(stderr, "[HostWindow] No compatible OpenGL context found.\n");
    SDL_Quit();
    return false;
}

void destroy_host_window(HostWindow& host) {
    if (host.gl_context) {
        SDL_GL_DeleteContext(host.gl_context);
        host.gl_context = nullptr;
    }
    if (host.window) {
        SDL_DestroyWindow(host.window);
        host.window = nullptr;
    }
    SDL_Quit();
}
