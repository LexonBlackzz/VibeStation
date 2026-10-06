#include "ui/vs2/vs2_shared.h"

#include <SDL.h>

#include <array>
#include <cstdlib>
#include <vector>

namespace ps2::ui::vs2 {

namespace {

// Baked sizes; text picks the nearest one at or above the scaled size, the
// same approach as definitive_fonts.cpp.
constexpr std::array<float, 12> kSizes = {
    11.0f, 13.0f, 15.0f, 17.0f, 19.0f, 22.0f, 26.0f, 30.0f, 36.0f, 44.0f, 54.0f, 66.0f};

std::array<std::array<ImFont*, kSizes.size()>, 4> g_fonts{};
bool g_loaded = false;

std::filesystem::path first_existing(
    const std::vector<std::filesystem::path>& candidates) {
    std::error_code ec;
    for (const auto& path : candidates) {
        if (!path.empty() && std::filesystem::exists(path, ec)) return path;
        ec.clear();
    }
    return {};
}

std::filesystem::path system_font(std::initializer_list<const char*> names) {
    std::vector<std::filesystem::path> candidates;
    for (const char* name : names) {
        const std::filesystem::path bundled = find_asset(name);
        if (!bundled.empty()) candidates.push_back(bundled);
    }
#ifdef _WIN32
    if (const char* windir = std::getenv("WINDIR")) {
        const std::filesystem::path fonts = std::filesystem::path(windir) / "Fonts";
        for (const char* name : names) candidates.push_back(fonts / name);
    }
#else
    for (const char* name : names) {
        candidates.push_back(std::filesystem::path("/usr/share/fonts/truetype/dejavu") / name);
    }
#endif
    return first_existing(candidates);
}

} // namespace

void load_fonts() {
    if (g_loaded) return;
    g_loaded = true;

    ImGuiIO& io = ImGui::GetIO();
    // Keep the existing default font first so the developer view is unchanged.
    if (io.Fonts->Fonts.empty()) io.Fonts->AddFontDefault();

    const std::array<std::filesystem::path, 4> paths = {
        // Semilight rather than Light: Light was too thin to read on black.
        system_font({"segoeuisl.ttf", "segoeuil.ttf", "segoeui.ttf", "DejaVuSans.ttf"}),
        system_font({"segoeui.ttf", "DejaVuSans.ttf"}),
        system_font({"CascadiaMono.ttf", "consola.ttf", "DejaVuSansMono.ttf"}),
        system_font({"segoeuib.ttf", "DejaVuSans-Bold.ttf"}),
    };

    ImFontConfig config{};
    config.OversampleH = 2;
    config.OversampleV = 1;
    for (std::size_t role = 0; role < paths.size(); ++role) {
        for (std::size_t i = 0; i < kSizes.size(); ++i) {
            ImFont* loaded = nullptr;
            if (!paths[role].empty()) {
                loaded = io.Fonts->AddFontFromFileTTF(
                    paths[role].string().c_str(), kSizes[i], &config);
            }
            g_fonts[role][i] = loaded != nullptr ? loaded : io.Fonts->Fonts[0];
        }
    }
}

// Text is drawn a little larger than the layout asks for: small print gains
// the most (11 -> 13, 13 -> 15), headings less (30 -> 33).
float readable(float size) {
    return size * (1.18f - 0.08f * std::clamp((size - 14.0f) / 10.0f, 0.0f, 1.0f));
}

ImFont* font(FontRole role, float pixel_size) {
    const auto& set = g_fonts[static_cast<std::size_t>(role)];
    for (std::size_t i = 0; i < kSizes.size(); ++i) {
        if (kSizes[i] >= pixel_size - 0.5f && set[i] != nullptr) return set[i];
    }
    return set.back() != nullptr ? set.back() : ImGui::GetFont();
}

void text(ImDrawList* draw, FontRole role, float size, const ImVec2& pos,
          ImU32 color, const char* str) {
    const float shown = readable(size);
    draw->AddText(font(role, shown), shown, pos, color, str);
}

ImVec2 text_size(FontRole role, float size, const char* str) {
    const float shown = readable(size);
    return font(role, shown)->CalcTextSizeA(shown, FLT_MAX, 0.0f, str);
}

} // namespace ps2::ui::vs2
