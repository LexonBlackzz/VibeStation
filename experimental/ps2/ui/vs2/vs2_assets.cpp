#include "ui/vs2/vs2_shared.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <stb_image.h>

#include <array>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

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

namespace {

#ifdef _WIN32
// "vs2-boot.wav" -> "VS2_BOOT_WAV", the name resources/vibestation.rc gives it.
std::wstring resource_name(const char* name) {
    std::wstring out;
    for (const char* c = name; *c != '\0'; ++c) {
        const char ch = (*c == '-' || *c == '.') ? '_' : *c;
        out.push_back(static_cast<wchar_t>(ch >= 'a' && ch <= 'z' ? ch - 'a' + 'A' : ch));
    }
    return out;
}

// The embedded copy in this executable, if any (none in the standalone lab).
bool embedded_asset(const char* name, const unsigned char*& data, std::size_t& size) {
    HMODULE module = GetModuleHandleW(nullptr);
    HRSRC resource = module != nullptr
        ? FindResourceW(module, resource_name(name).c_str(), MAKEINTRESOURCEW(10)) // RT_RCDATA
        : nullptr;
    if (resource == nullptr) return false;
    HGLOBAL loaded = LoadResource(module, resource);
    const DWORD bytes = SizeofResource(module, resource);
    data = loaded != nullptr ? static_cast<const unsigned char*>(LockResource(loaded)) : nullptr;
    size = bytes;
    return data != nullptr && size != 0;
}
#else
bool embedded_asset(const char*, const unsigned char*&, std::size_t&) { return false; }
#endif

} // namespace

std::vector<unsigned char> load_asset(const char* name) {
    const unsigned char* data = nullptr;
    std::size_t size = 0;
    if (embedded_asset(name, data, size)) return {data, data + size};

    const std::filesystem::path path = find_asset(name);
    if (path.empty()) return {};
    std::ifstream file(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

bool asset_available(const char* name) {
    const unsigned char* data = nullptr;
    std::size_t size = 0;
    return embedded_asset(name, data, size) || !find_asset(name).empty();
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

unsigned int load_image_texture(const char* name, int& width, int& height) {
    width = height = 0;
    const std::vector<unsigned char> file = load_asset(name);
    if (file.empty()) return 0;
    int channels = 0;
    unsigned char* pixels = stbi_load_from_memory(file.data(), static_cast<int>(file.size()),
                                                  &width, &height, &channels, 4);
    if (pixels == nullptr || width <= 0 || height <= 0) {
        if (pixels != nullptr) stbi_image_free(pixels);
        width = height = 0;
        return 0;
    }

    // Dark artwork on an opaque white page: darkness becomes opacity, so the
    // artwork turns white (and can be tinted) over the dark UI.
    const unsigned char* corner = pixels;
    if (corner[3] == 255 && corner[0] > 235 && corner[1] > 235 && corner[2] > 235) {
        const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        for (std::size_t i = 0; i < count; ++i) {
            unsigned char* p = pixels + i * 4;
            const int luma = (p[0] * 54 + p[1] * 183 + p[2] * 19) >> 8;
            p[3] = static_cast<unsigned char>((255 - luma) * p[3] / 255);
            p[0] = p[1] = p[2] = 255;
        }
    }

    const unsigned int texture = create_texture_rgba(width, height, pixels, true);
    stbi_image_free(pixels);
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
