#include "ui/app.h"
#include "ui/definitive/definitive_shared.h"

#include <SDL.h>
#include <imgui.h>

#include <array>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace {

constexpr std::array<float, 18> kDefinitiveFontSizes = {{
    9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f,
    16.0f, 18.0f, 20.0f, 22.0f, 24.0f, 28.0f,
    32.0f, 36.0f, 40.0f, 48.0f, 54.0f, 60.0f
}};

std::array<ImFont*, kDefinitiveFontSizes.size()> g_definitive_fonts = {};

std::filesystem::path find_definitive_font_path() {
    std::vector<std::filesystem::path> candidates;

    const std::filesystem::path cwd =
        std::filesystem::current_path();
    candidates.push_back(
        cwd / "resources" / "fonts" / "VibeStationMono.ttf");
    candidates.push_back(
        cwd / ".." / "resources" / "fonts" / "VibeStationMono.ttf");

    if (char* base = SDL_GetBasePath()) {
        const std::filesystem::path base_path(base);
        candidates.push_back(
            base_path / "resources" / "fonts" / "VibeStationMono.ttf");
        candidates.push_back(
            base_path / ".." / "resources" / "fonts" / "VibeStationMono.ttf");
        SDL_free(base);
    }

#ifdef _WIN32
    if (const char* windir = std::getenv("WINDIR")) {
        const std::filesystem::path fonts =
            std::filesystem::path(windir) / "Fonts";
        candidates.push_back(fonts / "CascadiaMono.ttf");
        candidates.push_back(fonts / "CascadiaCode.ttf");
        candidates.push_back(fonts / "consola.ttf");
        candidates.push_back(fonts / "lucon.ttf");
    }
#elif defined(__APPLE__)
    candidates.push_back("/System/Library/Fonts/SFNSMono.ttf");
    candidates.push_back("/System/Library/Fonts/Menlo.ttc");
#else
    candidates.push_back(
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf");
    candidates.push_back(
        "/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf");
#endif

    std::error_code ec;
    for (const auto& path : candidates) {
        if (!path.empty() &&
            std::filesystem::exists(path, ec) &&
            !ec) {
            return path;
        }
        ec.clear();
    }

    return {};
}

} // namespace

namespace definitive_ui {

ImFont* font_for_size(float pixel_size) {
    ImFont* best = nullptr;
    float best_distance = FLT_MAX;

    for (size_t i = 0;
         i < g_definitive_fonts.size();
         ++i) {
        ImFont* font = g_definitive_fonts[i];
        if (font == nullptr) {
            continue;
        }

        const float distance =
            std::abs(
                kDefinitiveFontSizes[i] -
                pixel_size);
        if (distance < best_distance) {
            best = font;
            best_distance = distance;
        }
    }

    return best != nullptr
        ? best
        : ImGui::GetFont();
}

} // namespace definitive_ui

void App::initialize_definitive_ui_fonts() {
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();

    const std::filesystem::path font_path =
        find_definitive_font_path();
    const std::string font_path_utf8 =
        font_path.string();

    for (size_t i = 0;
         i < kDefinitiveFontSizes.size();
         ++i) {
        ImFontConfig config;
        config.SizePixels =
            kDefinitiveFontSizes[i];
        config.OversampleH = 3;
        config.OversampleV = 2;
        config.PixelSnapH = false;

        ImFont* font = nullptr;
        if (!font_path_utf8.empty()) {
            font = io.Fonts->AddFontFromFileTTF(
                font_path_utf8.c_str(),
                kDefinitiveFontSizes[i],
                &config);
        }

        if (font == nullptr) {
            font =
                io.Fonts->AddFontDefault(
                    &config);
        }

        g_definitive_fonts[i] = font;
    }

    io.FontDefault =
        g_definitive_fonts[5];
}
