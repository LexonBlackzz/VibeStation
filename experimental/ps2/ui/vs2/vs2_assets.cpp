#include "ui/vs2/vs2_shared.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <array>

namespace ps2::ui::vs2 {

std::filesystem::path find_asset(const char* name) {
    std::array<std::filesystem::path, 6> candidates{};
    std::error_code ec;
    const std::filesystem::path cwd = std::filesystem::current_path(ec);
    if (!ec) {
        candidates[0] = cwd / "resources" / "vs2" / name;
        candidates[1] = cwd / ".." / "resources" / "vs2" / name;
        candidates[2] = cwd / name;
    }
    if (char* base = SDL_GetBasePath()) {
        const std::filesystem::path base_path(base);
        SDL_free(base);
        candidates[3] = base_path / "resources" / "vs2" / name;
        candidates[4] = base_path / ".." / "resources" / "vs2" / name;
        candidates[5] = base_path / name;
    }
    for (const auto& candidate : candidates) {
        if (!candidate.empty() && std::filesystem::exists(candidate, ec) && !ec) {
            return candidate;
        }
        ec.clear();
    }
    return {};
}

unsigned int create_texture_rgba(int width, int height, const void* pixels, bool linear) {
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    const GLint filter = linear ? GL_LINEAR : GL_NEAREST;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, pixels);
    glBindTexture(GL_TEXTURE_2D, 0);
    return texture;
}

void destroy_texture(unsigned int& texture) {
    if (texture != 0) {
        GLuint id = texture;
        glDeleteTextures(1, &id);
        texture = 0;
    }
}

} // namespace ps2::ui::vs2
