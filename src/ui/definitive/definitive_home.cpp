#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#include <stb_image.h>

#include "ui/app.h"
#include "ui/output_resolution_utils.h"
#include "ui/screenshot_utils.h"
#include "ui/theme_settings.h"
#include "vibestation_version.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace {
constexpr float kDesignWidth = 1280.0f;
constexpr float kDesignHeight = 800.0f;

constexpr std::array<float, 18> kDefinitiveFontSizes = {{
    9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f,
    16.0f, 18.0f, 20.0f, 22.0f, 24.0f, 28.0f,
    32.0f, 36.0f, 40.0f, 48.0f, 54.0f, 60.0f
}};
std::array<ImFont*, kDefinitiveFontSizes.size()> g_definitive_fonts = {};

std::filesystem::path find_definitive_font_path() {
    std::vector<std::filesystem::path> candidates;

    // Prefer an app-local font if one is added later, then use common clean
    // monospace system fonts. Windows normally provides Consolas.
    const std::filesystem::path cwd = std::filesystem::current_path();
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
        if (!path.empty() && std::filesystem::exists(path, ec) && !ec) {
            return path;
        }
        ec.clear();
    }
    return {};
}

ImFont* definitive_font_for_size(float pixel_size) {
    ImFont* best = nullptr;
    float best_distance = FLT_MAX;

    for (size_t i = 0; i < g_definitive_fonts.size(); ++i) {
        ImFont* font = g_definitive_fonts[i];
        if (font == nullptr) {
            continue;
        }

        const float distance =
            std::abs(kDefinitiveFontSizes[i] - pixel_size);
        if (distance < best_distance) {
            best = font;
            best_distance = distance;
        }
    }

    return best != nullptr ? best : ImGui::GetFont();
}

GLuint g_background_texture = 0;
GLuint g_background_soft_texture = 0;
GLuint g_background_blur_texture = 0;
int g_background_width = 0;
int g_background_height = 0;
bool g_background_load_attempted = false;

struct LauncherQuote {
    const char* line1;
    const char* line2;
};

constexpr std::array<LauncherQuote, 48> kLauncherQuotes = {{
    {"A SMALLER PAST", "STILL PLAYS"},
    {"OLD DISC", "NEW NIGHT"},
    {"MEMORY CARD", "STILL WARM"},
    {"BOOT AGAIN", "STAY A WHILE"},
    {"PRESS START", "SEE WHAT RETURNS"},
    {"ONE MORE SAVE", "ONE MORE RUN"},
    {"THE CRT HUMS", "THE DISC SPINS"},
    {"NO PATCH NOTES", "JUST MEMORIES"},
    {"LOAD THE PAST", "PLAY IT FORWARD"},
    {"PIXELS FADE", "MEMORIES DON'T"},
    {"THE ROOM IS DARK", "THE SCREEN IS ON"},
    {"KEEP THE STATIC", "LOSE THE DUST"},
    {"THE DISC KNOWS", "WHERE YOU LEFT OFF"},
    {"OLD HARDWARE", "NEW VIBES"},
    {"WAIT FOR THE CHIME", "THEN BEGIN"},
    {"INSERT MEMORY", "REMOVE TIME"},
    {"THE SAVE IS THERE", "GO FIND IT"},
    {"THE NIGHT IS YOUNG", "THE DISC IS OLD"},
    {"LOW POLY", "HIGH MEMORY"},
    {"PAUSE THE WORLD", "LOAD THE GAME"},
    {"ONE CONSOLE", "MANY NIGHTS"},
    {"LET IT BOOT", "LET IT BREATHE"},
    {"THE PAST", "HAS A FRAME RATE"},
    {"STILL LOADING", "STILL WORTH IT"},
    {"FROM DISC", "TO MEMORY"},
    {"NO CLOUD", "JUST CARDS"},
    {"32 BITS", "ENDLESS NIGHTS"},
    {"START BUTTON", "SAME FEELING"},
    {"OLD SAVE", "NEW CHANCE"},
    {"THE LID CLOSES", "THE WORLD OPENS"},
    {"ONE MORE BOOT", "ONE MORE MEMORY"},
    {"THE SCREEN GLOWS", "THE ROOM DISAPPEARS"},
    {"THEN: YESTERDAY", "NOW: TONIGHT"},
    {"MOTION BLUR", "MEMORY SHARP"},
    {"SAME BUTTONS", "DIFFERENT NIGHT"},
    {"DISC IN", "WORLD OUT"},
    {"THE LOGO FADES", "THE GAME REMAINS"},
    {"READY WHEN", "YOU ARE"},
    {"THE PAST WAITS", "AT 60 FPS"},
    {"LOAD. SAVE.", "REPEAT."},
    {"GOOD GAMES", "GOOD TIMES"},
    {"OLD WORLDS", "STILL OPEN"},
    {"SAVE OFTEN", "STAY LONGER"},
    {"ANOTHER BOOT", "ANOTHER STORY"},
    {"TURN IT ON", "LET TIME STOP"},
    {"THE DISC TURNS", "THE NIGHT MOVES"},
    {"SAME START", "NEW MEMORY"},
    {"WELCOME BACK", "PLAYER ONE"},
}};

size_t g_launcher_quote_index = 0;
bool g_launcher_quote_selected = false;

std::array<float, 5> g_menu_highlight_mix = {};

enum class LauncherStartTransition {
    None,
    Bios,
    Disc
};

LauncherStartTransition g_launcher_start_transition =
    LauncherStartTransition::None;
float g_launcher_start_transition_elapsed = 0.0f;
constexpr float kLauncherStartFadeSeconds = 0.42f;

float g_launcher_intro_elapsed = 0.0f;
bool g_launcher_intro_complete = false;

// Stage 1: a PS1-inspired VibeStation boot presentation on black.
// Stage 2: the launcher itself initializes in staggered groups.
constexpr float kBootLogoBegin = 0.28f;
constexpr float kBootLogoSettle = 1.26f;
constexpr float kBootWordmarkBegin = 1.30f;
constexpr float kBootFadeBegin = 2.68f;
constexpr float kUiBackgroundBegin = 3.00f;
constexpr float kUiBackgroundReady = 3.64f;
constexpr float kUiBrandBegin = 3.30f;
constexpr float kUiMenuBegin = 3.54f;
constexpr float kUiPanelsBegin = 3.96f;
constexpr float kLauncherIntroDuration = 4.58f;


ImU32 rgba(int r, int g, int b, int a = 255) {
    return IM_COL32(r, g, b, a);
}

float animate_towards(float current, float target, float response = 13.0f) {
    const float dt = std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);
    if (dt <= 0.0f) {
        return target;
    }
    const float alpha = 1.0f - std::exp(-response * dt);
    return current + (target - current) * alpha;
}

int glow_alpha(float value) {
    return std::clamp(static_cast<int>(std::round(value)), 0, 255);
}

float smoothstep01(float value) {
    const float t = std::clamp(value, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float timeline_progress(float time, float start, float end) {
    if (end <= start) {
        return time >= end ? 1.0f : 0.0f;
    }
    return smoothstep01((time - start) / (end - start));
}

ImVec2 lerp_point(const ImVec2& a, const ImVec2& b, float t) {
    const float clamped = std::clamp(t, 0.0f, 1.0f);
    return ImVec2(
        a.x + (b.x - a.x) * clamped,
        a.y + (b.y - a.y) * clamped);
}


std::vector<unsigned char> make_blurred_rgba(
    const unsigned char* source, int width, int height, int radius) {
    const size_t pixel_count =
        static_cast<size_t>(width) * static_cast<size_t>(height);
    std::vector<unsigned char> horizontal(pixel_count * 4u);
    std::vector<unsigned char> output(pixel_count * 4u);
    if (source == nullptr || width <= 0 || height <= 0 || radius <= 0) {
        return output;
    }

    const int kernel = radius * 2 + 1;

    // Horizontal pass using a sliding window. Edge pixels are clamped so the
    // backdrop does not darken near the image boundary.
    for (int y = 0; y < height; ++y) {
        for (int channel = 0; channel < 4; ++channel) {
            int sum = 0;
            for (int k = -radius; k <= radius; ++k) {
                const int sx = std::clamp(k, 0, width - 1);
                sum += source[(static_cast<size_t>(y) * width + sx) * 4u + channel];
            }

            for (int x = 0; x < width; ++x) {
                horizontal[(static_cast<size_t>(y) * width + x) * 4u + channel] =
                    static_cast<unsigned char>(sum / kernel);

                const int remove_x = std::clamp(x - radius, 0, width - 1);
                const int add_x = std::clamp(x + radius + 1, 0, width - 1);
                sum -= source[
                    (static_cast<size_t>(y) * width + remove_x) * 4u + channel];
                sum += source[
                    (static_cast<size_t>(y) * width + add_x) * 4u + channel];
            }
        }
    }

    // Vertical pass.
    for (int x = 0; x < width; ++x) {
        for (int channel = 0; channel < 4; ++channel) {
            int sum = 0;
            for (int k = -radius; k <= radius; ++k) {
                const int sy = std::clamp(k, 0, height - 1);
                sum += horizontal[
                    (static_cast<size_t>(sy) * width + x) * 4u + channel];
            }

            for (int y = 0; y < height; ++y) {
                output[(static_cast<size_t>(y) * width + x) * 4u + channel] =
                    static_cast<unsigned char>(sum / kernel);

                const int remove_y = std::clamp(y - radius, 0, height - 1);
                const int add_y = std::clamp(y + radius + 1, 0, height - 1);
                sum -= horizontal[
                    (static_cast<size_t>(remove_y) * width + x) * 4u + channel];
                sum += horizontal[
                    (static_cast<size_t>(add_y) * width + x) * 4u + channel];
            }
        }
    }

    return output;
}


std::vector<unsigned char> make_softened_background(
    const unsigned char* source,
    const std::vector<unsigned char>& blurred,
    int width, int height) {
    const size_t pixel_count =
        static_cast<size_t>(width) * static_cast<size_t>(height);
    std::vector<unsigned char> output(pixel_count * 4u);
    if (source == nullptr || blurred.size() != output.size() ||
        width <= 0 || height <= 0) {
        return output;
    }

    // Keep the left side softly blurred, then blend continuously back into
    // the untouched photograph. Doing this once on the CPU avoids visible
    // strip/seam artifacts from drawing many translucent texture slices.
    constexpr float kBlurStrength = 0.76f;
    constexpr float kBlurSolidEnd = 0.34f;
    constexpr float kBlurFadeEnd = 0.60f;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const float nx = width > 1
                ? static_cast<float>(x) / static_cast<float>(width - 1)
                : 0.0f;

            float blur_mix = kBlurStrength;
            if (nx > kBlurSolidEnd) {
                const float fade_t = (nx - kBlurSolidEnd) /
                    (kBlurFadeEnd - kBlurSolidEnd);
                blur_mix = kBlurStrength * (1.0f - smoothstep01(fade_t));
            }
            if (nx >= kBlurFadeEnd) {
                blur_mix = 0.0f;
            }

            const size_t base =
                (static_cast<size_t>(y) * width + x) * 4u;
            for (int channel = 0; channel < 3; ++channel) {
                const float sharp = static_cast<float>(source[base + channel]);
                const float soft = static_cast<float>(blurred[base + channel]);
                const float blended =
                    sharp + (soft - sharp) * blur_mix;
                output[base + channel] = static_cast<unsigned char>(
                    std::clamp(blended, 0.0f, 255.0f) + 0.5f);
            }
            output[base + 3] = source[base + 3];
        }
    }
    return output;
}

bool upload_rgba_texture(
    GLuint& texture, const unsigned char* pixels, int width, int height) {
    if (pixels == nullptr || width <= 0 || height <= 0) {
        return false;
    }

    glGenTextures(1, &texture);
    if (texture == 0) {
        return false;
    }

    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

std::filesystem::path find_background_path() {
    std::array<std::filesystem::path, 4> candidates{};

    std::error_code ec;
    const std::filesystem::path cwd = std::filesystem::current_path(ec);
    if (!ec) {
        candidates[0] = cwd / "resources" / "ui" / "definitive" / "background.jpg";
        candidates[1] = cwd / ".." / "resources" / "ui" / "definitive" / "background.jpg";
    }

    if (char* base = SDL_GetBasePath()) {
        const std::filesystem::path base_path(base);
        SDL_free(base);
        candidates[2] =
            base_path / "resources" / "ui" / "definitive" / "background.jpg";
        candidates[3] =
            base_path / ".." / "resources" / "ui" / "definitive" / "background.jpg";
    }

    for (const auto& candidate : candidates) {
        if (!candidate.empty() && std::filesystem::exists(candidate, ec) && !ec) {
            return candidate;
        }
        ec.clear();
    }
    return {};
}

bool ensure_background_texture_loaded() {
    if (g_background_texture != 0) {
        return true;
    }
    if (g_background_load_attempted) {
        return false;
    }
    g_background_load_attempted = true;

    const std::filesystem::path path = find_background_path();
    if (path.empty()) {
        return false;
    }

    int channels = 0;
    unsigned char* pixels = stbi_load(
        path.string().c_str(), &g_background_width, &g_background_height, &channels, 4);
    if (pixels == nullptr || g_background_width <= 0 || g_background_height <= 0) {
        if (pixels != nullptr) {
            stbi_image_free(pixels);
        }
        g_background_width = 0;
        g_background_height = 0;
        return false;
    }

    const bool sharp_uploaded = upload_rgba_texture(
        g_background_texture, pixels, g_background_width, g_background_height);

    // A modest one-time blur gives the text zones a glassy backdrop without
    // adding a per-frame render pass.
    const std::vector<unsigned char> blurred =
        make_blurred_rgba(pixels, g_background_width, g_background_height, 7);
    if (!blurred.empty()) {
        // Keep a fully blurred copy for modal overlays, and a left-softened
        // copy for the normal definitive launcher presentation.
        upload_rgba_texture(
            g_background_blur_texture, blurred.data(),
            g_background_width, g_background_height);

        const std::vector<unsigned char> softened =
            make_softened_background(
                pixels, blurred, g_background_width, g_background_height);
        if (!softened.empty()) {
            upload_rgba_texture(
                g_background_soft_texture, softened.data(),
                g_background_width, g_background_height);
        }
    }

    stbi_image_free(pixels);
    return sharp_uploaded;
}

struct Layout {
    ImVec2 origin{};
    float scale = 1.0f;

    ImVec2 point(float x, float y) const {
        return ImVec2(origin.x + x * scale, origin.y + y * scale);
    }
    ImVec2 size(float x, float y) const {
        return ImVec2(x * scale, y * scale);
    }
    float px(float value) const {
        return value * scale;
    }
};

Layout make_layout(const ImVec2& window_pos, const ImVec2& window_size) {
    Layout layout{};
    layout.scale = std::max(
        0.55f, std::min(window_size.x / kDesignWidth, window_size.y / kDesignHeight));
    const ImVec2 design_size(kDesignWidth * layout.scale, kDesignHeight * layout.scale);
    layout.origin = ImVec2(
        window_pos.x + (window_size.x - design_size.x) * 0.5f,
        window_pos.y + (window_size.y - design_size.y) * 0.5f);
    return layout;
}

struct CoverUv {
    float u0 = 0.0f;
    float v0 = 0.0f;
    float u1 = 1.0f;
    float v1 = 1.0f;
};

CoverUv cover_uv_for_size(const ImVec2& size) {
    CoverUv uv{};
    if (g_background_width <= 0 || g_background_height <= 0) {
        return uv;
    }

    const float image_aspect =
        static_cast<float>(g_background_width) / static_cast<float>(g_background_height);
    const float canvas_aspect = size.x / std::max(1.0f, size.y);

    if (canvas_aspect > image_aspect) {
        const float visible_v = image_aspect / canvas_aspect;
        uv.v0 = (1.0f - visible_v) * 0.5f;
        uv.v1 = uv.v0 + visible_v;
    }
    else {
        const float visible_u = canvas_aspect / image_aspect;
        uv.u0 = (1.0f - visible_u) * 0.5f;
        uv.u1 = uv.u0 + visible_u;
    }
    return uv;
}

void draw_cover_region(ImDrawList* draw, GLuint texture,
    const ImVec2& canvas_pos, const ImVec2& canvas_size,
    const ImVec2& region_min, const ImVec2& region_max, ImU32 tint) {
    if (texture == 0 || canvas_size.x <= 0.0f || canvas_size.y <= 0.0f) {
        return;
    }

    const CoverUv uv = cover_uv_for_size(canvas_size);
    const float tx0 = std::clamp(
        (region_min.x - canvas_pos.x) / canvas_size.x, 0.0f, 1.0f);
    const float ty0 = std::clamp(
        (region_min.y - canvas_pos.y) / canvas_size.y, 0.0f, 1.0f);
    const float tx1 = std::clamp(
        (region_max.x - canvas_pos.x) / canvas_size.x, 0.0f, 1.0f);
    const float ty1 = std::clamp(
        (region_max.y - canvas_pos.y) / canvas_size.y, 0.0f, 1.0f);

    const ImVec2 region_uv0(
        uv.u0 + (uv.u1 - uv.u0) * tx0,
        uv.v0 + (uv.v1 - uv.v0) * ty0);
    const ImVec2 region_uv1(
        uv.u0 + (uv.u1 - uv.u0) * tx1,
        uv.v0 + (uv.v1 - uv.v0) * ty1);

    draw->AddImage(
        (ImTextureID)(intptr_t)texture,
        region_min, region_max, region_uv0, region_uv1, tint);
}

void draw_background(ImDrawList* draw, const ImVec2& pos, const ImVec2& size) {
    draw->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y),
        rgba(7, 9, 12, 255));

    if (!ensure_background_texture_loaded()) {
        draw->AddRectFilledMultiColor(
            pos, ImVec2(pos.x + size.x, pos.y + size.y),
            rgba(8, 10, 14, 255), rgba(17, 19, 23, 255),
            rgba(10, 12, 15, 255), rgba(5, 7, 10, 255));
        return;
    }

    const CoverUv uv = cover_uv_for_size(size);
    const GLuint display_texture =
        g_background_soft_texture != 0
            ? g_background_soft_texture
            : g_background_texture;
    draw->AddImage(
        (ImTextureID)(intptr_t)display_texture,
        pos, ImVec2(pos.x + size.x, pos.y + size.y),
        ImVec2(uv.u0, uv.v0), ImVec2(uv.u1, uv.v1));
}

void draw_readability_shade(ImDrawList* draw,
    const ImVec2& pos, const ImVec2& size) {
    constexpr int kSegments = 32;
    constexpr float kSolidEnd = 0.31f;
    constexpr float kFadeEnd = 0.66f;
    constexpr float kTopAlpha = 208.0f;
    constexpr float kBottomAlpha = 216.0f;

    const auto strength_at = [=](float nx) {
        if (nx <= kSolidEnd) {
            return 1.0f;
        }
        if (nx >= kFadeEnd) {
            return 0.0f;
        }
        const float t = (nx - kSolidEnd) / (kFadeEnd - kSolidEnd);
        return 1.0f - smoothstep01(t);
    };

    // Adjacent segments share identical edge alpha values, so this behaves as
    // one continuous nonlinear fade rather than several stacked dark panels.
    for (int i = 0; i < kSegments; ++i) {
        const float n0 = kFadeEnd *
            (static_cast<float>(i) / kSegments);
        const float n1 = kFadeEnd *
            (static_cast<float>(i + 1) / kSegments);
        const float s0 = strength_at(n0);
        const float s1 = strength_at(n1);

        const ImVec2 r0(pos.x + size.x * n0, pos.y);
        const ImVec2 r1(pos.x + size.x * n1, pos.y + size.y);
        draw->AddRectFilledMultiColor(
            r0, r1,
            rgba(0, 2, 5, glow_alpha(kTopAlpha * s0)),
            rgba(0, 2, 5, glow_alpha(kTopAlpha * s1)),
            rgba(0, 2, 5, glow_alpha(kBottomAlpha * s1)),
            rgba(0, 2, 5, glow_alpha(kBottomAlpha * s0)));
    }
}


void draw_centered_intro_text(
    ImDrawList* draw, const ImVec2& center, float font_size,
    ImU32 color, const char* text) {
    ImFont* font = definitive_font_for_size(font_size);
    const ImVec2 text_size = font->CalcTextSizeA(
        font_size, FLT_MAX, 0.0f, text);
    draw->AddText(
        font, font_size,
        ImVec2(center.x - text_size.x * 0.5f,
            center.y - text_size.y * 0.5f),
        color, text);
}

void draw_tapered_beam(
    ImDrawList* draw,
    const ImVec2& source,
    const ImVec2& destination,
    float progress,
    float source_half_width,
    float destination_half_width,
    ImU32 color,
    ImU32 glow_color) {
    const float p = std::clamp(progress, 0.0f, 1.0f);
    if (p <= 0.001f) {
        return;
    }

    const ImVec2 head = lerp_point(source, destination, p);
    const ImVec2 delta(head.x - source.x, head.y - source.y);
    const float length = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    if (length <= 0.001f) {
        return;
    }

    const ImVec2 normal(-delta.y / length, delta.x / length);
    const float head_half_width =
        source_half_width +
        (destination_half_width - source_half_width) * p;

    const ImVec2 s0(
        source.x + normal.x * source_half_width,
        source.y + normal.y * source_half_width);
    const ImVec2 s1(
        source.x - normal.x * source_half_width,
        source.y - normal.y * source_half_width);
    const ImVec2 h0(
        head.x + normal.x * head_half_width,
        head.y + normal.y * head_half_width);
    const ImVec2 h1(
        head.x - normal.x * head_half_width,
        head.y - normal.y * head_half_width);

    // Wide, low-alpha bloom underneath the beam.
    const float glow_scale = 2.8f;
    draw->AddQuadFilled(
        ImVec2(source.x + normal.x * source_half_width * glow_scale,
            source.y + normal.y * source_half_width * glow_scale),
        ImVec2(source.x - normal.x * source_half_width * glow_scale,
            source.y - normal.y * source_half_width * glow_scale),
        ImVec2(head.x - normal.x * head_half_width * glow_scale,
            head.y - normal.y * head_half_width * glow_scale),
        ImVec2(head.x + normal.x * head_half_width * glow_scale,
            head.y + normal.y * head_half_width * glow_scale),
        glow_color);

    draw->AddQuadFilled(s0, s1, h1, h0, color);
}

void draw_boot_presentation(
    const ImVec2& pos, const ImVec2& size, float elapsed) {
    ImDrawList* overlay = ImGui::GetForegroundDrawList();
    const ImVec2 end(pos.x + size.x, pos.y + size.y);

    overlay->AddRectFilled(pos, end, rgba(0, 0, 0, 255));

    const float boot_fade =
        1.0f - timeline_progress(elapsed, kBootFadeBegin, kUiBackgroundBegin);
    if (boot_fade <= 0.001f) {
        return;
    }

    const float unit = std::min(size.x, size.y);
    const ImVec2 convergence(
        pos.x + size.x * 0.50f,
        pos.y + size.y * 0.505f);

    constexpr std::array<ImU32, 4> beam_colors = {
        IM_COL32(194, 44, 56, 255),
        IM_COL32(52, 128, 125, 255),
        IM_COL32(177, 145, 72, 255),
        IM_COL32(52, 93, 157, 255),
    };

    // Sources deliberately begin above the viewport so the mark reads as four
    // light beams sweeping in, rather than four static sticks already on-screen.
    const std::array<ImVec2, 4> sources = {
        ImVec2(pos.x + size.x * 0.31f, pos.y - unit * 0.18f),
        ImVec2(pos.x + size.x * 0.43f, pos.y - unit * 0.24f),
        ImVec2(pos.x + size.x * 0.57f, pos.y - unit * 0.21f),
        ImVec2(pos.x + size.x * 0.69f, pos.y - unit * 0.16f),
    };
    const std::array<ImVec2, 4> targets = {
        ImVec2(convergence.x - unit * 0.030f, convergence.y),
        ImVec2(convergence.x - unit * 0.010f, convergence.y + unit * 0.006f),
        ImVec2(convergence.x + unit * 0.010f, convergence.y + unit * 0.006f),
        ImVec2(convergence.x + unit * 0.030f, convergence.y),
    };

    const float source_width = std::max(8.0f, unit * 0.025f);
    const float destination_width = std::max(1.5f, unit * 0.0040f);

    for (size_t i = 0; i < sources.size(); ++i) {
        const float start =
            kBootLogoBegin + static_cast<float>(i) * 0.085f;
        const float finish =
            0.93f + static_cast<float>(i) * 0.085f;
        const float beam_progress =
            timeline_progress(elapsed, start, finish);

        const ImU32 base = beam_colors[i];
        const int r = (base >> IM_COL32_R_SHIFT) & 0xFF;
        const int g = (base >> IM_COL32_G_SHIFT) & 0xFF;
        const int b = (base >> IM_COL32_B_SHIFT) & 0xFF;

        draw_tapered_beam(
            overlay,
            sources[i],
            targets[i],
            beam_progress,
            source_width,
            destination_width,
            rgba(r, g, b, glow_alpha(238.0f * boot_fade)),
            rgba(r, g, b, glow_alpha(42.0f * boot_fade)));
    }

    // The convergence point is the brightest part of the frame. Its glow
    // swells just as the fourth beam arrives, then settles behind the wordmark.
    const float convergence_in =
        timeline_progress(elapsed, 0.82f, 1.16f);
    const float convergence_settle =
        1.0f - 0.55f * timeline_progress(elapsed, 1.18f, 1.70f);
    const float convergence_energy =
        convergence_in * convergence_settle * boot_fade;

    for (int ring = 10; ring >= 1; --ring) {
        const float t = static_cast<float>(ring) / 10.0f;
        const float radius =
            unit * (0.010f + 0.125f * t) *
            (0.70f + 0.30f * convergence_in);
        overlay->AddCircleFilled(
            convergence,
            radius,
            rgba(214, 224, 238,
                glow_alpha(convergence_energy *
                    (7.0f + (1.0f - t) * 20.0f))),
            64);
    }

    const float core_pulse =
        timeline_progress(elapsed, 1.02f, 1.20f) *
        (1.0f - timeline_progress(elapsed, 1.20f, 1.50f)) *
        boot_fade;
    if (core_pulse > 0.001f) {
        overlay->AddCircleFilled(
            convergence,
            unit * (0.012f + 0.020f * core_pulse),
            rgba(255, 255, 255, glow_alpha(255.0f * core_pulse)),
            48);
    }

    // Once converged, retain a compact four-ray mark so the light beams feel
    // like they assembled into an identity rather than simply disappearing.
    const float assembled_in =
        timeline_progress(elapsed, 1.08f, 1.30f) * boot_fade;
    if (assembled_in > 0.001f) {
        const float arm = unit * 0.065f;
        const float stroke = std::max(2.0f, unit * 0.006f);
        for (size_t i = 0; i < targets.size(); ++i) {
            const ImVec2 outer = lerp_point(
                convergence, sources[i], arm /
                    std::max(1.0f, std::sqrt(
                        (sources[i].x - convergence.x) *
                        (sources[i].x - convergence.x) +
                        (sources[i].y - convergence.y) *
                        (sources[i].y - convergence.y))));
            const ImU32 base = beam_colors[i];
            const int r = (base >> IM_COL32_R_SHIFT) & 0xFF;
            const int g = (base >> IM_COL32_G_SHIFT) & 0xFF;
            const int b = (base >> IM_COL32_B_SHIFT) & 0xFF;
            overlay->AddLine(
                convergence, outer,
                rgba(r, g, b, glow_alpha(220.0f * assembled_in)),
                stroke);
        }
    }

    const float wordmark_in =
        timeline_progress(elapsed, kBootWordmarkBegin, 1.62f);
    if (wordmark_in > 0.001f) {
        const float word_alpha = wordmark_in * boot_fade;
        draw_centered_intro_text(
            overlay,
            ImVec2(convergence.x, convergence.y + unit * 0.145f),
            std::max(20.0f, unit * 0.045f),
            rgba(224, 227, 233, glow_alpha(238.0f * word_alpha)),
            "VibeStation");
        draw_centered_intro_text(
            overlay,
            ImVec2(convergence.x, convergence.y + unit * 0.193f),
            std::max(8.0f, unit * 0.012f),
            rgba(137, 143, 151, glow_alpha(205.0f * word_alpha)),
            "PS1 EMULATOR");
    }

    // Short multi-frame corruption burst. At 60 Hz this spans roughly four
    // frames; at lower refresh rates it still shows at least a couple of
    // distinct patterns because the corruption is time-indexed.
    constexpr float kGlitchStart = 1.66f;
    constexpr float kGlitchEnd = 1.728f;
    if (elapsed >= kGlitchStart && elapsed < kGlitchEnd) {
        const float glitch_alpha = boot_fade;
        const float glitch_time =
            (elapsed - kGlitchStart) / (kGlitchEnd - kGlitchStart);
        const int glitch_pattern = std::clamp(
            static_cast<int>(glitch_time * 4.0f), 0, 3);

        constexpr std::array<float, 4> x_offsets = {
            -2.0f, 1.0f, -1.0f, 2.0f
        };
        constexpr std::array<float, 4> tear_offsets = {
            -0.010f, 0.006f, -0.003f, 0.012f
        };
        constexpr std::array<float, 4> tear_widths = {
            0.33f, 0.24f, 0.39f, 0.28f
        };

        const float px = std::max(1.0f, unit / 800.0f);
        const float channel_shift =
            x_offsets[static_cast<size_t>(glitch_pattern)] * px;

        // Chromatic channel separation across the full beam/mark graphic.
        // Red and cyan copies move in opposite directions by only a few pixels.
        for (size_t i = 0; i < sources.size(); ++i) {
            const ImVec2 red_source(
                sources[i].x + channel_shift, sources[i].y);
            const ImVec2 red_target(
                targets[i].x + channel_shift, targets[i].y);
            const ImVec2 blue_source(
                sources[i].x - channel_shift, sources[i].y);
            const ImVec2 blue_target(
                targets[i].x - channel_shift, targets[i].y);

            draw_tapered_beam(
                overlay,
                red_source, red_target, 1.0f,
                source_width, destination_width,
                rgba(232, 35, 49, glow_alpha(72.0f * glitch_alpha)),
                rgba(232, 35, 49, glow_alpha(14.0f * glitch_alpha)));
            draw_tapered_beam(
                overlay,
                blue_source, blue_target, 1.0f,
                source_width, destination_width,
                rgba(38, 133, 191, glow_alpha(72.0f * glitch_alpha)),
                rgba(38, 133, 191, glow_alpha(14.0f * glitch_alpha)));
        }

        // RGB-split the compact assembled mark too.
        const float arm = unit * 0.065f;
        const float stroke = std::max(2.0f, unit * 0.006f);
        for (size_t i = 0; i < sources.size(); ++i) {
            const float distance = std::max(
                1.0f,
                std::sqrt(
                    (sources[i].x - convergence.x) *
                        (sources[i].x - convergence.x) +
                    (sources[i].y - convergence.y) *
                        (sources[i].y - convergence.y)));
            const ImVec2 outer = lerp_point(
                convergence, sources[i], arm / distance);

            overlay->AddLine(
                ImVec2(convergence.x + channel_shift, convergence.y),
                ImVec2(outer.x + channel_shift, outer.y),
                rgba(235, 38, 52, glow_alpha(132.0f * glitch_alpha)),
                stroke);
            overlay->AddLine(
                ImVec2(convergence.x - channel_shift, convergence.y),
                ImVec2(outer.x - channel_shift, outer.y),
                rgba(36, 132, 190, glow_alpha(132.0f * glitch_alpha)),
                stroke);
        }

        // Wordmark split uses the same pattern so the glitch feels like one
        // system-wide corruption event rather than a text-only overlay.
        draw_centered_intro_text(
            overlay,
            ImVec2(
                convergence.x + channel_shift * 1.6f,
                convergence.y + unit * 0.145f),
            std::max(20.0f, unit * 0.045f),
            rgba(232, 35, 49, glow_alpha(138.0f * glitch_alpha)),
            "VibeStation");
        draw_centered_intro_text(
            overlay,
            ImVec2(
                convergence.x - channel_shift * 1.6f,
                convergence.y + unit * 0.145f),
            std::max(20.0f, unit * 0.045f),
            rgba(38, 133, 191, glow_alpha(138.0f * glitch_alpha)),
            "VibeStation");

        // Uneven primary tear: broken sections, per-pattern placement, and
        // red/blue fringing instead of a single flat horizontal bar.
        const float tear_y =
            convergence.y +
            unit * (0.010f +
                tear_offsets[static_cast<size_t>(glitch_pattern)]);
        const float tear_extent =
            size.x * tear_widths[static_cast<size_t>(glitch_pattern)];
        const float tear_center =
            convergence.x + channel_shift * 3.0f;
        const float tear_h =
            std::max(2.0f, unit * (0.0030f +
                0.0008f * static_cast<float>(glitch_pattern)));

        const std::array<ImVec2, 3> tear_segments = {
            ImVec2(-0.50f, -0.19f),
            ImVec2(-0.12f,  0.15f),
            ImVec2( 0.23f,  0.50f),
        };

        for (size_t i = 0; i < tear_segments.size(); ++i) {
            const float jitter =
                ((glitch_pattern + static_cast<int>(i)) & 1) ? px * 4.0f : -px * 3.0f;
            const float x0 =
                tear_center + tear_extent * tear_segments[i].x + jitter;
            const float x1 =
                tear_center + tear_extent * tear_segments[i].y + jitter;

            // Red fringe above, bright broken core, blue fringe below.
            overlay->AddRectFilled(
                ImVec2(x0 - px * 2.0f, tear_y - px * 1.5f),
                ImVec2(x1, tear_y),
                rgba(230, 38, 52, glow_alpha(128.0f * glitch_alpha)));
            overlay->AddRectFilled(
                ImVec2(x0, tear_y),
                ImVec2(x1, tear_y + tear_h),
                rgba(236, 241, 246, glow_alpha(108.0f * glitch_alpha)));
            overlay->AddRectFilled(
                ImVec2(x0 + px * 2.0f, tear_y + tear_h),
                ImVec2(x1 + px * 3.0f, tear_y + tear_h + px * 1.4f),
                rgba(40, 132, 194, glow_alpha(122.0f * glitch_alpha)));
        }

        // Secondary thinner tear at a different height/length.
        const float tear2_y =
            tear_y + unit * (0.032f +
                0.004f * static_cast<float>(glitch_pattern));
        const float tear2_x0 =
            convergence.x - size.x * (0.11f +
                0.015f * static_cast<float>(glitch_pattern));
        const float tear2_x1 =
            convergence.x + size.x * (0.07f +
                0.012f * static_cast<float>((glitch_pattern + 1) & 3));
        overlay->AddRectFilled(
            ImVec2(tear2_x0, tear2_y),
            ImVec2(tear2_x1, tear2_y + std::max(1.0f, px)),
            rgba(218, 224, 231, glow_alpha(78.0f * glitch_alpha)));

        // Simulated content displacement immediately around the primary tear:
        // clipped chromatic copies of the mark are shifted in opposite
        // directions above/below the rupture.
        const float clip_half = std::max(4.0f, unit * 0.012f);
        overlay->PushClipRect(
            ImVec2(pos.x, tear_y - clip_half),
            ImVec2(pos.x + size.x, tear_y),
            true);
        for (size_t i = 0; i < sources.size(); ++i) {
            const float distance = std::max(
                1.0f,
                std::sqrt(
                    (sources[i].x - convergence.x) *
                        (sources[i].x - convergence.x) +
                    (sources[i].y - convergence.y) *
                        (sources[i].y - convergence.y)));
            const ImVec2 outer = lerp_point(
                convergence, sources[i], arm / distance);
            overlay->AddLine(
                ImVec2(convergence.x + px * 4.0f, convergence.y),
                ImVec2(outer.x + px * 4.0f, outer.y),
                rgba(241, 243, 247, glow_alpha(115.0f * glitch_alpha)),
                stroke);
        }
        overlay->PopClipRect();

        overlay->PushClipRect(
            ImVec2(pos.x, tear_y),
            ImVec2(pos.x + size.x, tear_y + clip_half),
            true);
        for (size_t i = 0; i < sources.size(); ++i) {
            const float distance = std::max(
                1.0f,
                std::sqrt(
                    (sources[i].x - convergence.x) *
                        (sources[i].x - convergence.x) +
                    (sources[i].y - convergence.y) *
                        (sources[i].y - convergence.y)));
            const ImVec2 outer = lerp_point(
                convergence, sources[i], arm / distance);
            overlay->AddLine(
                ImVec2(convergence.x - px * 3.0f, convergence.y),
                ImVec2(outer.x - px * 3.0f, outer.y),
                rgba(241, 243, 247, glow_alpha(96.0f * glitch_alpha)),
                stroke);
        }
        overlay->PopClipRect();
    }

    // Small signs of life prevent the black frame from feeling frozen and
    // make the sequence practical during development.
    const bool cursor_on =
        (static_cast<int>(elapsed * 4.0f) & 1) == 0;
    std::string status = "INITIALIZING VIBESTATION";
    if (cursor_on) {
        status += "_";
    }

    const float status_alpha =
        timeline_progress(elapsed, 0.24f, 0.62f) * boot_fade;
    if (status_alpha > 0.001f) {
        const float status_font_size =
            std::max(9.0f, unit * 0.011f);
        ImFont* status_font =
            definitive_font_for_size(status_font_size);
        overlay->AddText(
            status_font,
            status_font_size,
            ImVec2(pos.x + unit * 0.035f,
                pos.y + size.y - unit * 0.055f),
            rgba(128, 136, 146, glow_alpha(185.0f * status_alpha)),
            status.c_str());

        const char* skip_text = "SPACE / ENTER  SKIP";
        const float font_size = std::max(9.0f, unit * 0.011f);
        ImFont* skip_font =
            definitive_font_for_size(font_size);
        const ImVec2 skip_size =
            skip_font->CalcTextSizeA(
                font_size, FLT_MAX, 0.0f, skip_text);
        overlay->AddText(
            skip_font,
            font_size,
            ImVec2(
                pos.x + size.x - unit * 0.035f - skip_size.x,
                pos.y + size.y - unit * 0.055f),
            rgba(105, 112, 122, glow_alpha(165.0f * status_alpha)),
            skip_text);
    }
}

void draw_ui_initialization_overlay(
    const ImVec2& pos, const ImVec2& size, float elapsed) {
    ImDrawList* overlay = ImGui::GetForegroundDrawList();
    const Layout layout = make_layout(pos, size);

    const float background_reveal =
        timeline_progress(elapsed, kUiBackgroundBegin, kUiBackgroundReady);

    // Overall black veil lets the photographic background arrive first.
    const int global_black =
        glow_alpha(255.0f * (1.0f - background_reveal));
    if (global_black > 0) {
        overlay->AddRectFilled(
            pos, ImVec2(pos.x + size.x, pos.y + size.y),
            rgba(0, 0, 0, global_black));
    }

    // Branding: short downward-to-upward wipe with a very light scan edge.
    const float brand_reveal =
        timeline_progress(elapsed, kUiBrandBegin, kUiBrandBegin + 0.56f);
    const ImVec2 brand0 = layout.point(28.0f, 22.0f);
    const ImVec2 brand1 = layout.point(455.0f, 205.0f);
    if (brand_reveal < 1.0f) {
        const float wipe_y =
            brand1.y - (brand1.y - brand0.y) * brand_reveal;
        overlay->AddRectFilled(
            ImVec2(brand0.x, brand0.y),
            ImVec2(brand1.x, wipe_y),
            rgba(0, 0, 0, 255));
        overlay->AddLine(
            ImVec2(brand0.x, wipe_y),
            ImVec2(brand1.x, wipe_y),
            rgba(186, 211, 232, glow_alpha(105.0f * brand_reveal)),
            layout.px(1.0f));
    }

    // Right-side metadata resolves just after the brand.
    const float meta_reveal =
        timeline_progress(
            elapsed, kUiBrandBegin + 0.12f, kUiBrandBegin + 0.64f);
    const ImVec2 meta0 = layout.point(1030.0f, 24.0f);
    const ImVec2 meta1 = layout.point(1248.0f, 116.0f);
    if (meta_reveal < 1.0f) {
        overlay->AddRectFilled(
            meta0, meta1,
            rgba(0, 0, 0, glow_alpha(255.0f * (1.0f - meta_reveal))));
    }

    // Primary actions initialize one after another from top to bottom. The
    // image concept is used here as a placement/timing guide, not as literal
    // startup artwork.
    constexpr float kMenuX = 28.0f;
    constexpr float kMenuY = 210.0f;
    constexpr float kMenuW = 420.0f;
    constexpr float kMenuH = 66.0f;
    constexpr float kMenuStep = 70.0f;

    for (int i = 0; i < 5; ++i) {
        const float item_start =
            kUiMenuBegin + static_cast<float>(i) * 0.095f;
        const float item_reveal =
            timeline_progress(elapsed, item_start, item_start + 0.46f);

        const ImVec2 row0 =
            layout.point(kMenuX, kMenuY + kMenuStep * i);
        const ImVec2 row1 =
            layout.point(kMenuX + kMenuW,
                kMenuY + kMenuStep * i + kMenuH);

        if (item_reveal < 1.0f) {
            const float wipe_x =
                row0.x + (row1.x - row0.x) * item_reveal;

            overlay->AddRectFilled(
                ImVec2(wipe_x, row0.y),
                row1,
                rgba(0, 0, 0, 255));

            const int veil_alpha =
                glow_alpha(180.0f * (1.0f - item_reveal));
            if (veil_alpha > 0) {
                overlay->AddRectFilled(
                    row0, ImVec2(wipe_x, row1.y),
                    rgba(0, 0, 0, veil_alpha));
            }

            overlay->AddLine(
                ImVec2(wipe_x, row0.y + layout.px(5.0f)),
                ImVec2(wipe_x, row1.y - layout.px(5.0f)),
                rgba(181, 211, 236,
                    glow_alpha(125.0f * item_reveal)),
                layout.px(1.0f));
        }
    }

    // Bottom cards lift in last. Separate timing makes the right system card
    // trail the library by a fraction of a second.
    const std::array<ImVec4, 2> panels = {
        ImVec4(25.0f, 575.0f, 845.0f, 780.0f),
        ImVec4(850.0f, 575.0f, 1255.0f, 780.0f),
    };
    for (size_t i = 0; i < panels.size(); ++i) {
        const float start =
            kUiPanelsBegin + static_cast<float>(i) * 0.11f;
        const float reveal =
            timeline_progress(elapsed, start, start + 0.48f);
        const ImVec2 p0 = layout.point(panels[i].x, panels[i].y);
        const ImVec2 p1 = layout.point(panels[i].z, panels[i].w);

        if (reveal < 1.0f) {
            const float wipe_y =
                p1.y - (p1.y - p0.y) * reveal;
            overlay->AddRectFilled(
                p0, ImVec2(p1.x, wipe_y),
                rgba(0, 0, 0, 255));
            overlay->AddRectFilled(
                ImVec2(p0.x, wipe_y), p1,
                rgba(0, 0, 0,
                    glow_alpha(155.0f * (1.0f - reveal))));
        }
    }
}


void add_text(ImDrawList* draw, const Layout& layout, float x, float y,
    float size, ImU32 color, const char* text) {
    const float font_size = layout.px(size);
    ImFont* font = definitive_font_for_size(font_size);
    draw->AddText(
        font, font_size, layout.point(x, y), color, text);
}

void add_text_right(ImDrawList* draw, const Layout& layout, float right_x, float y,
    float size, ImU32 color, const char* text) {
    const float font_size = layout.px(size);
    ImFont* font = definitive_font_for_size(font_size);
    const ImVec2 text_size = font->CalcTextSizeA(
        font_size, FLT_MAX, 0.0f, text);
    const ImVec2 p = layout.point(right_x, y);
    draw->AddText(font, font_size,
        ImVec2(p.x - text_size.x, p.y), color, text);
}

enum class MenuIcon {
    Play,
    Folder,
    Chip,
    Settings,
    Exit
};

void draw_icon(ImDrawList* draw, const Layout& layout, MenuIcon icon,
    float x, float y, ImU32 color) {
    const ImVec2 p = layout.point(x, y);
    const float s = layout.scale;

    switch (icon) {
    case MenuIcon::Play:
        draw->AddTriangleFilled(
            ImVec2(p.x, p.y),
            ImVec2(p.x, p.y + 22.0f * s),
            ImVec2(p.x + 18.0f * s, p.y + 11.0f * s),
            color);
        break;
    case MenuIcon::Folder:
        draw->AddLine(
            ImVec2(p.x, p.y + 5.0f * s),
            ImVec2(p.x + 8.0f * s, p.y + 5.0f * s), color, 2.0f * s);
        draw->AddLine(
            ImVec2(p.x + 8.0f * s, p.y + 5.0f * s),
            ImVec2(p.x + 12.0f * s, p.y + 9.0f * s), color, 2.0f * s);
        draw->AddRect(
            ImVec2(p.x, p.y + 8.0f * s),
            ImVec2(p.x + 24.0f * s, p.y + 24.0f * s),
            color, 1.5f * s, 0, 2.0f * s);
        break;
    case MenuIcon::Chip:
        draw->AddRect(
            ImVec2(p.x + 4.0f * s, p.y + 3.0f * s),
            ImVec2(p.x + 22.0f * s, p.y + 25.0f * s),
            color, 1.0f * s, 0, 2.0f * s);
        for (int i = 0; i < 4; ++i) {
            const float py = p.y + (6.0f + i * 5.0f) * s;
            draw->AddLine(ImVec2(p.x, py), ImVec2(p.x + 4.0f * s, py),
                color, 1.5f * s);
            draw->AddLine(ImVec2(p.x + 22.0f * s, py),
                ImVec2(p.x + 26.0f * s, py), color, 1.5f * s);
        }
        break;
    case MenuIcon::Settings:
        draw->AddCircle(
            ImVec2(p.x + 13.0f * s, p.y + 14.0f * s),
            9.0f * s, color, 12, 2.0f * s);
        draw->AddCircle(
            ImVec2(p.x + 13.0f * s, p.y + 14.0f * s),
            3.0f * s, color, 12, 2.0f * s);
        for (int i = 0; i < 8; ++i) {
            const float a = static_cast<float>(i) * 3.14159265f / 4.0f;
            const ImVec2 a0(
                p.x + 10.0f * s * std::cos(a) + 13.0f * s,
                p.y + 10.0f * s * std::sin(a) + 14.0f * s);
            const ImVec2 a1(
                p.x + 14.0f * s * std::cos(a) + 13.0f * s,
                p.y + 14.0f * s * std::sin(a) + 14.0f * s);
            draw->AddLine(a0, a1, color, 2.0f * s);
        }
        break;
    case MenuIcon::Exit:
        draw->AddRect(
            ImVec2(p.x + 8.0f * s, p.y + 2.0f * s),
            ImVec2(p.x + 25.0f * s, p.y + 26.0f * s),
            color, 0.0f, 0, 1.8f * s);
        draw->AddLine(
            ImVec2(p.x, p.y + 14.0f * s),
            ImVec2(p.x + 16.0f * s, p.y + 14.0f * s),
            color, 2.0f * s);
        draw->AddLine(
            ImVec2(p.x + 11.0f * s, p.y + 9.0f * s),
            ImVec2(p.x + 16.0f * s, p.y + 14.0f * s),
            color, 2.0f * s);
        draw->AddLine(
            ImVec2(p.x + 11.0f * s, p.y + 19.0f * s),
            ImVec2(p.x + 16.0f * s, p.y + 14.0f * s),
            color, 2.0f * s);
        break;
    }
}

bool menu_button(const Layout& layout, ImDrawList* draw, int index,
    MenuIcon icon, const char* title, const char* subtitle) {
    constexpr float kX = 36.0f;
    constexpr float kY = 218.0f;
    constexpr float kWidth = 396.0f;
    constexpr float kHeight = 62.0f;
    constexpr float kGap = 8.0f;

    const float y = kY + index * (kHeight + kGap);
    const ImVec2 p = layout.point(kX, y);
    const ImVec2 size = layout.size(kWidth, kHeight);

    ImGui::SetCursorScreenPos(p);
    ImGui::PushID(index);
    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(0, 0, 0, 0));
    const bool pressed = ImGui::Button("##definitive_menu", size);
    ImGui::PopStyleColor(3);

    const bool hovered = ImGui::IsItemHovered();
    const bool focused = ImGui::IsItemFocused();
    const bool active = ImGui::IsItemActive();
    const bool engaged = hovered || focused || active;

    // Every item gets an explicit zero target whenever it is not engaged.
    // This prevents the previous menu item from remaining lit after focus moves.
    const float target_mix = engaged ? 1.0f : 0.0f;
    float& highlight_mix = g_menu_highlight_mix[static_cast<size_t>(index)];
    highlight_mix = animate_towards(
        highlight_mix, target_mix, engaged ? 16.0f : 9.5f);
    if (!engaged && highlight_mix < 0.004f) {
        highlight_mix = 0.0f;
    }

    const float pulse = engaged
        ? (0.92f + 0.08f *
            std::sin(static_cast<float>(ImGui::GetTime()) * 2.1f))
        : 1.0f;
    const float glow = std::clamp(
        highlight_mix * pulse + (active ? 0.14f : 0.0f), 0.0f, 1.1f);

    if (glow > 0.01f) {
        const float glow_outer = layout.px(6.0f + glow * 2.0f);
        const float glow_mid = layout.px(3.0f + glow);
        const ImVec2 outer0(p.x - glow_outer, p.y - glow_outer);
        const ImVec2 outer1(p.x + size.x + glow_outer, p.y + size.y + glow_outer);
        const ImVec2 mid0(p.x - glow_mid, p.y - glow_mid);
        const ImVec2 mid1(p.x + size.x + glow_mid, p.y + size.y + glow_mid);

        draw->AddRect(outer0, outer1,
            rgba(90, 154, 216, glow_alpha(17.0f * glow)),
            0.0f, 0, layout.px(1.0f));
        draw->AddRect(mid0, mid1,
            rgba(126, 184, 236, glow_alpha(34.0f * glow)),
            0.0f, 0, layout.px(1.2f));
    }

    if (highlight_mix > 0.01f) {
        const int fill_alpha =
            glow_alpha(116.0f * highlight_mix + (hovered ? 10.0f : 0.0f));
        draw->AddRectFilled(
            p, ImVec2(p.x + size.x, p.y + size.y),
            rgba(12, 17, 23, fill_alpha));

        draw->AddRect(
            p, ImVec2(p.x + size.x, p.y + size.y),
            rgba(211, 229, 246, glow_alpha(235.0f * highlight_mix)),
            0.0f, 0, layout.px(1.35f));
        draw->AddRect(
            ImVec2(p.x + layout.px(2.0f), p.y + layout.px(2.0f)),
            ImVec2(p.x + size.x - layout.px(2.0f),
                p.y + size.y - layout.px(2.0f)),
            rgba(103, 154, 205, glow_alpha(128.0f * highlight_mix)),
            0.0f, 0, layout.px(0.8f));

        const float rail_half = layout.px(16.0f + 5.0f * highlight_mix);
        const float center_y = p.y + size.y * 0.5f;
        draw->AddRectFilled(
            ImVec2(p.x - layout.px(2.0f), center_y - rail_half),
            ImVec2(p.x, center_y + rail_half),
            rgba(205, 231, 255, glow_alpha(235.0f * highlight_mix)));
    }

    const float emphasis = std::clamp(highlight_mix, 0.0f, 1.0f);
    const ImU32 main_color = rgba(
        static_cast<int>(206 + 36 * emphasis),
        static_cast<int>(208 + 38 * emphasis),
        static_cast<int>(211 + 39 * emphasis),
        static_cast<int>(228 + 27 * emphasis));
    const ImU32 sub_color = rgba(
        static_cast<int>(150 + 34 * emphasis),
        static_cast<int>(154 + 38 * emphasis),
        static_cast<int>(161 + 40 * emphasis),
        static_cast<int>(210 + 28 * emphasis));

    const float content_shift = 2.0f * highlight_mix;
    draw_icon(draw, layout, icon,
        kX + 24.0f + content_shift, y + 18.0f, main_color);
    add_text(draw, layout, kX + 72.0f + content_shift, y + 10.0f, 20.5f,
        main_color, title);
    add_text(draw, layout, kX + 72.0f + content_shift, y + 38.0f, 12.0f,
        sub_color, subtitle);

    ImGui::PopID();
    return pressed;
}

bool small_button(const Layout& layout, const char* id, const char* label,
    float x, float y, float w, float h, bool enabled = true) {
    ImGui::SetCursorScreenPos(layout.point(x, y));
    if (!enabled) {
        ImGui::BeginDisabled();
    }
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, rgba(12, 15, 19, 205));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, rgba(24, 31, 39, 225));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, rgba(32, 42, 52, 235));
    ImGui::PushStyleColor(ImGuiCol_Border, rgba(128, 145, 163, 190));
    ImGui::PushStyleColor(ImGuiCol_Text, rgba(226, 230, 235, 245));
    ImGui::PushID(id);
    const bool pressed = ImGui::Button(label, layout.size(w, h));
    ImGui::PopID();
    ImGui::PopStyleColor(5);
    ImGui::PopStyleVar(2);
    if (!enabled) {
        ImGui::EndDisabled();
    }
    return pressed;
}

void draw_panel(ImDrawList* draw, const Layout& layout,
    float x, float y, float w, float h) {
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 p1 = layout.point(x + w, y + h);
    draw->AddRectFilled(p0, p1, rgba(5, 8, 11, 178));
    draw->AddRect(p0, p1, rgba(102, 116, 130, 205), 0.0f, 0, layout.px(1.0f));
}

void draw_folder_badge(ImDrawList* draw, const Layout& layout, float x, float y) {
    const ImVec2 p = layout.point(x, y);
    const float s = layout.scale;
    const ImU32 c = rgba(226, 230, 235, 240);
    draw->AddLine(ImVec2(p.x, p.y + 4.0f * s),
        ImVec2(p.x + 8.0f * s, p.y + 4.0f * s), c, 1.6f * s);
    draw->AddLine(ImVec2(p.x + 8.0f * s, p.y + 4.0f * s),
        ImVec2(p.x + 12.0f * s, p.y + 8.0f * s), c, 1.6f * s);
    draw->AddRect(ImVec2(p.x, p.y + 7.0f * s),
        ImVec2(p.x + 22.0f * s, p.y + 20.0f * s), c, 1.0f * s, 0, 1.6f * s);
}

void draw_info_badge(ImDrawList* draw, const Layout& layout, float x, float y) {
    const ImVec2 p = layout.point(x, y);
    const float s = layout.scale;
    const ImU32 c = rgba(226, 230, 235, 240);
    draw->AddCircle(ImVec2(p.x + 9.0f * s, p.y + 10.0f * s),
        8.0f * s, c, 16, 1.5f * s);
    draw->AddCircleFilled(ImVec2(p.x + 9.0f * s, p.y + 6.0f * s),
        1.0f * s, c);
    draw->AddLine(ImVec2(p.x + 9.0f * s, p.y + 9.0f * s),
        ImVec2(p.x + 9.0f * s, p.y + 15.0f * s), c, 1.5f * s);
}
void draw_settings_section(ImDrawList* draw, const Layout& layout,
    float x, float y, float w, float h, const char* title) {
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 p1 = layout.point(x + w, y + h);
    draw->AddRectFilled(p0, p1, rgba(7, 11, 16, 226), layout.px(4.0f));
    draw->AddRect(p0, p1, rgba(91, 109, 126, 175),
        layout.px(4.0f), 0, layout.px(1.0f));
    add_text(draw, layout, x + 18.0f, y + 14.0f, 12.5f,
        rgba(209, 218, 227, 235), title);
    draw->AddLine(
        layout.point(x + 18.0f, y + 39.0f),
        layout.point(x + w - 18.0f, y + 39.0f),
        rgba(78, 92, 106, 145), layout.px(1.0f));
}

bool definitive_settings_switch(ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w, bool& value) {
    constexpr float kHeight = 68.0f;
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 row_size = layout.size(w, kHeight);

    ImGui::SetCursorScreenPos(p0);
    ImGui::PushID(id);
    const bool pressed = ImGui::InvisibleButton("##settings_switch", row_size);
    const bool hovered = ImGui::IsItemHovered();
    if (pressed) {
        value = !value;
    }

    if (hovered) {
        draw->AddRectFilled(
            p0, ImVec2(p0.x + row_size.x, p0.y + row_size.y),
            rgba(31, 43, 55, 92), layout.px(2.0f));
    }

    add_text(draw, layout, x + 18.0f, y + 8.0f, 13.0f,
        hovered ? rgba(240, 244, 248, 255) : rgba(221, 226, 232, 244),
        label);

    const ImVec2 track0 = layout.point(x + w - 63.0f, y + 18.0f);
    const ImVec2 track1 = layout.point(x + w - 19.0f, y + 40.0f);
    draw->AddRectFilled(
        track0, track1,
        value ? rgba(78, 126, 166, 235) : rgba(55, 62, 70, 230),
        layout.px(11.0f));

    const float knob_x = value ? (x + w - 31.0f) : (x + w - 51.0f);
    draw->AddCircleFilled(
        layout.point(knob_x, y + 29.0f),
        layout.px(8.0f),
        value ? rgba(232, 240, 247, 255) : rgba(178, 184, 191, 245),
        20);

    ImGui::PopID();
    return pressed;
}

bool definitive_settings_action(ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w) {
    constexpr float kHeight = 68.0f;
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 row_size = layout.size(w, kHeight);

    ImGui::SetCursorScreenPos(p0);
    ImGui::PushID(id);
    const bool pressed = ImGui::InvisibleButton("##settings_action", row_size);
    const bool hovered = ImGui::IsItemHovered();

    if (hovered) {
        draw->AddRectFilled(
            p0, ImVec2(p0.x + row_size.x, p0.y + row_size.y),
            rgba(31, 43, 55, 92), layout.px(2.0f));
    }

    add_text(draw, layout, x + 18.0f, y + 8.0f, 13.0f,
        hovered ? rgba(240, 244, 248, 255) : rgba(221, 226, 232, 244),
        label);

    const ImU32 arrow_color =
        hovered ? rgba(226, 237, 247, 250) : rgba(145, 158, 170, 220);
    const ImVec2 a = layout.point(x + w - 34.0f, y + 22.0f);
    draw->AddLine(a, layout.point(x + w - 27.0f, y + 29.0f),
        arrow_color, layout.px(1.5f));
    draw->AddLine(layout.point(x + w - 27.0f, y + 29.0f),
        layout.point(x + w - 34.0f, y + 36.0f),
        arrow_color, layout.px(1.5f));

    ImGui::PopID();
    return pressed;
}

bool definitive_settings_combo(ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w,
    int& current, const char* const items[], int item_count) {
    constexpr float kHeight = 68.0f;
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 row_size = layout.size(w, kHeight);

    const bool row_hovered = ImGui::IsMouseHoveringRect(
        p0, ImVec2(p0.x + row_size.x, p0.y + row_size.y), false);

    if (row_hovered) {
        draw->AddRectFilled(
            p0, ImVec2(p0.x + row_size.x, p0.y + row_size.y),
            rgba(31, 43, 55, 60), layout.px(2.0f));
    }

    add_text(draw, layout, x + 18.0f, y + 8.0f, 13.0f,
        rgba(221, 226, 232, 244), label);

    ImGui::SetCursorScreenPos(layout.point(x + w - 196.0f, y + 14.0f));
    ImGui::PushID(id);
    ImGui::SetNextItemWidth(layout.px(176.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, layout.px(2.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, layout.size(8.0f, 6.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, rgba(14, 20, 27, 245));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, rgba(25, 35, 45, 250));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, rgba(30, 42, 54, 255));
    ImGui::PushStyleColor(ImGuiCol_PopupBg, rgba(8, 12, 17, 252));
    ImGui::PushStyleColor(ImGuiCol_Border, rgba(96, 118, 138, 195));
    ImGui::PushStyleColor(ImGuiCol_Text, rgba(231, 236, 241, 250));
    ImGui::PushStyleColor(ImGuiCol_Header, rgba(54, 77, 97, 215));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, rgba(69, 96, 120, 230));
    const bool changed =
        ImGui::Combo("##value", &current, items, item_count);
    ImGui::PopStyleColor(8);
    ImGui::PopStyleVar(2);
    ImGui::PopID();
    return changed;
}


void definitive_settings_note(
    ImDrawList* draw, const Layout& layout,
    float x, float y, const char* text) {
    const float font_size = layout.px(10.6f);
    const ImVec2 pos = layout.point(x, y);

    // Keep descriptions on the text side of the row. They may wrap to a
    // second line, but are clipped before the right-aligned slider/combo area.
    const float wrap_width = layout.px(270.0f);
    const ImVec4 clip_rect(
        pos.x,
        pos.y,
        pos.x + wrap_width,
        pos.y + layout.px(30.0f));

    ImFont* font = definitive_font_for_size(font_size);
    draw->AddText(
        font,
        font_size,
        pos,
        rgba(171, 181, 191, 235),
        text,
        nullptr,
        wrap_width,
        &clip_rect);
}

bool definitive_settings_color(
    ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w, ImVec4& value) {
    add_text(draw, layout, x + 18.0f, y + 8.0f, 13.0f,
        rgba(221, 226, 232, 244), label);

    ImGui::SetCursorScreenPos(layout.point(x + w - 206.0f, y + 12.0f));
    ImGui::PushID(id);
    ImGui::SetNextItemWidth(layout.px(186.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, layout.px(2.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, rgba(14, 20, 27, 245));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, rgba(25, 35, 45, 250));
    ImGui::PushStyleColor(ImGuiCol_Border, rgba(96, 118, 138, 195));
    const bool changed = ImGui::ColorEdit4(
        "##value", &value.x,
        ImGuiColorEditFlags_DisplayRGB |
        ImGuiColorEditFlags_AlphaBar |
        ImGuiColorEditFlags_NoInputs);
    ImGui::PopStyleColor(3);
    ImGui::PopStyleVar();
    ImGui::PopID();
    return changed;
}

bool definitive_settings_tab_button(
    ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w, bool selected) {
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 sz = layout.size(w, 34.0f);
    ImGui::SetCursorScreenPos(p0);
    ImGui::PushID(id);
    const bool pressed = ImGui::InvisibleButton("##settings_tab", sz);
    const bool hovered = ImGui::IsItemHovered();
    ImGui::PopID();

    if (selected || hovered) {
        draw->AddRectFilled(
            p0, ImVec2(p0.x + sz.x, p0.y + sz.y),
            selected ? rgba(28, 42, 55, 220) : rgba(28, 39, 50, 130),
            layout.px(2.0f));
    }
    if (selected) {
        draw->AddRectFilled(
            layout.point(x, y + 32.0f),
            layout.point(x + w, y + 34.0f),
            rgba(175, 210, 238, 235));
    }

    add_text(draw, layout, x + 12.0f, y + 10.0f, 11.5f,
        selected ? rgba(239, 244, 248, 255)
                 : rgba(174, 184, 194, hovered ? 244 : 215),
        label);
    return pressed;
}

bool definitive_settings_slider_int(
    ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w,
    int& value, int min_value, int max_value,
    const char* format) {
    constexpr float kHeight = 68.0f;
    add_text(draw, layout, x + 18.0f, y + 8.0f, 13.0f,
        rgba(221, 226, 232, 244), label);

    ImGui::SetCursorScreenPos(layout.point(x + w - 206.0f, y + 14.0f));
    ImGui::PushID(id);
    ImGui::SetNextItemWidth(layout.px(186.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, layout.px(2.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, layout.size(7.0f, 5.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, rgba(14, 20, 27, 245));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, rgba(25, 35, 45, 250));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, rgba(30, 42, 54, 255));
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, rgba(167, 201, 230, 235));
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, rgba(219, 235, 248, 255));
    ImGui::PushStyleColor(ImGuiCol_Text, rgba(231, 236, 241, 250));
    const bool changed =
        ImGui::SliderInt("##value", &value, min_value, max_value, format);
    ImGui::PopStyleColor(6);
    ImGui::PopStyleVar(2);
    ImGui::PopID();
    return changed;
}

bool definitive_settings_slider_float(
    ImDrawList* draw, const Layout& layout,
    const char* id, const char* label,
    float x, float y, float w,
    float& value, float min_value, float max_value,
    const char* format) {
    add_text(draw, layout, x + 18.0f, y + 8.0f, 13.0f,
        rgba(221, 226, 232, 244), label);

    ImGui::SetCursorScreenPos(layout.point(x + w - 206.0f, y + 14.0f));
    ImGui::PushID(id);
    ImGui::SetNextItemWidth(layout.px(186.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, layout.px(2.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, layout.size(7.0f, 5.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, rgba(14, 20, 27, 245));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, rgba(25, 35, 45, 250));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, rgba(30, 42, 54, 255));
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, rgba(167, 201, 230, 235));
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, rgba(219, 235, 248, 255));
    ImGui::PushStyleColor(ImGuiCol_Text, rgba(231, 236, 241, 250));
    const bool changed =
        ImGui::SliderFloat("##value", &value, min_value, max_value, format);
    ImGui::PopStyleColor(6);
    ImGui::PopStyleVar(2);
    ImGui::PopID();
    return changed;
}

}

void App::initialize_definitive_ui_fonts() {
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();

    const std::filesystem::path font_path =
        find_definitive_font_path();
    const std::string font_path_utf8 = font_path.string();

    for (size_t i = 0; i < kDefinitiveFontSizes.size(); ++i) {
        ImFontConfig config;
        config.SizePixels = kDefinitiveFontSizes[i];
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

        // Keep a no-dependency fallback for systems where no suitable TTF is
        // present. Even the fallback is baked at native sizes rather than
        // magnifying a single 13 px font.
        if (font == nullptr) {
            font = io.Fonts->AddFontDefault(&config);
        }
        g_definitive_fonts[i] = font;
    }

    // 14 px is a comfortable baseline for legacy ImGui widgets. Definitive
    // draw-list text picks its own nearest native-size font above.
    io.FontDefault = g_definitive_fonts[5];
}

void App::release_definitive_ui_assets() {
    if (g_background_texture != 0) {
        glDeleteTextures(1, &g_background_texture);
        g_background_texture = 0;
    }
    if (g_background_soft_texture != 0) {
        glDeleteTextures(1, &g_background_soft_texture);
        g_background_soft_texture = 0;
    }
    if (g_background_blur_texture != 0) {
        glDeleteTextures(1, &g_background_blur_texture);
        g_background_blur_texture = 0;
    }
    g_background_width = 0;
    g_background_height = 0;
    g_background_load_attempted = false;
    g_launcher_start_transition = LauncherStartTransition::None;
    g_launcher_start_transition_elapsed = 0.0f;
    g_menu_highlight_mix.fill(0.0f);
}

void App::panel_definitive_settings() {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoScrollWithMouse |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoBackground;

    ImGui::Begin("##DefinitiveSettingsOverlay", nullptr, flags);
    ImGui::PopStyleVar(3);

    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 window_pos = ImGui::GetWindowPos();
    const ImVec2 window_size = ImGui::GetWindowSize();
    const ImVec2 window_end(
        window_pos.x + window_size.x,
        window_pos.y + window_size.y);
    const Layout layout = make_layout(window_pos, window_size);

    ensure_background_texture_loaded();

    if (g_background_blur_texture != 0) {
        draw_cover_region(
            draw, g_background_blur_texture,
            window_pos, window_size,
            window_pos, window_end,
            rgba(255, 255, 255, 244));
    }
    else {
        draw->AddRectFilled(window_pos, window_end, rgba(7, 9, 12, 255));
    }
    draw->AddRectFilled(window_pos, window_end, rgba(0, 2, 6, 148));

    constexpr float panel_x = 96.0f;
    constexpr float panel_y = 46.0f;
    constexpr float panel_w = 1088.0f;
    constexpr float panel_h = 708.0f;

    const ImVec2 panel0 = layout.point(panel_x, panel_y);
    const ImVec2 panel1 = layout.point(panel_x + panel_w, panel_y + panel_h);

    draw->AddRectFilled(
        layout.point(panel_x - 10.0f, panel_y + 10.0f),
        layout.point(panel_x + panel_w + 10.0f, panel_y + panel_h + 10.0f),
        rgba(0, 0, 0, 76), layout.px(8.0f));
    draw->AddRectFilled(panel0, panel1, rgba(5, 9, 14, 245), layout.px(5.0f));
    draw->AddRect(panel0, panel1, rgba(111, 132, 151, 218),
        layout.px(5.0f), 0, layout.px(1.0f));

    add_text(draw, layout, panel_x + 38.0f, panel_y + 25.0f, 31.0f,
        rgba(237, 241, 245, 255), "SETTINGS");
    add_text(draw, layout, panel_x + 40.0f, panel_y + 63.0f, 10.0f,
        rgba(143, 154, 165, 228), "DEFINITIVE");

    constexpr std::array<ImU32, 4> settings_accents = {
        IM_COL32(194, 44, 56, 235),
        IM_COL32(52, 128, 125, 235),
        IM_COL32(177, 145, 72, 235),
        IM_COL32(52, 93, 157, 235),
    };
    for (int i = 0; i < 4; ++i) {
        draw->AddRectFilled(
            layout.point(panel_x + 39.0f + i * 23.0f, panel_y + 87.0f),
            layout.point(panel_x + 56.0f + i * 23.0f, panel_y + 92.0f),
            settings_accents[static_cast<size_t>(i)]);
    }

    const ImVec2 close0 =
        layout.point(panel_x + panel_w - 57.0f, panel_y + 26.0f);
    ImGui::SetCursorScreenPos(close0);
    ImGui::PushID("definitive_settings_close");
    const bool close_pressed =
        ImGui::InvisibleButton("##close", layout.size(30.0f, 30.0f));
    const bool close_hovered = ImGui::IsItemHovered();
    ImGui::PopID();

    const ImU32 close_color = close_hovered
        ? rgba(242, 246, 249, 255)
        : rgba(165, 176, 187, 230);
    if (close_hovered) {
        draw->AddRectFilled(
            close0,
            ImVec2(close0.x + layout.px(30.0f),
                close0.y + layout.px(30.0f)),
            rgba(36, 49, 61, 160), layout.px(3.0f));
    }
    draw->AddLine(
        ImVec2(close0.x + layout.px(8.0f), close0.y + layout.px(8.0f)),
        ImVec2(close0.x + layout.px(22.0f), close0.y + layout.px(22.0f)),
        close_color, layout.px(1.6f));
    draw->AddLine(
        ImVec2(close0.x + layout.px(22.0f), close0.y + layout.px(8.0f)),
        ImVec2(close0.x + layout.px(8.0f), close0.y + layout.px(22.0f)),
        close_color, layout.px(1.6f));

    if (close_pressed || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        show_settings_ = false;
        definitive_detailed_settings_ = false;
        ImGui::End();
        return;
    }

    draw->AddLine(
        layout.point(panel_x + 28.0f, panel_y + 108.0f),
        layout.point(panel_x + panel_w - 28.0f, panel_y + 108.0f),
        rgba(82, 97, 111, 155), layout.px(1.0f));

    // Mirror the legacy Settings page's category names, excluding Logging.
    constexpr std::array<const char*, 7> tab_labels = {
        "Input", "Video", "Audio", "System",
        "Memory Cards", "Experimental", "Customize"
    };
    constexpr float tab_x = 124.0f;
    constexpr float tab_y = 164.0f;
    constexpr float tab_w = 147.0f;

    for (int i = 0; i < static_cast<int>(tab_labels.size()); ++i) {
        if (definitive_settings_tab_button(
            draw, layout,
            tab_labels[static_cast<size_t>(i)],
            tab_labels[static_cast<size_t>(i)],
            tab_x + tab_w * i, tab_y, tab_w,
            definitive_settings_tab_ == i)) {
            definitive_settings_tab_ = i;
        }
    }

    constexpr float left_x = 134.0f;
    constexpr float right_x = 654.0f;
    constexpr float column_w = 492.0f;
    constexpr float content_y = 216.0f;
    constexpr float row_step = 68.0f;

    const auto note = [&](float x, float y, const char* text) {
        definitive_settings_note(draw, layout, x + 18.0f, y + 29.0f, text);
    };

    const auto apply_audio_settings = [&]() {
        save_persistent_config();
        if (system_ == nullptr) {
            return;
        }
        const bool was_running = emu_runner_.is_running();
        if (was_running) {
            emu_runner_.pause_and_wait_idle();
        }
        system_->spu().reinitialize_audio_device();
        if (was_running) {
            emu_runner_.set_running(true);
        }
    };

    switch (definitive_settings_tab_) {
    case 0: { // Input
        draw_settings_section(
            draw, layout, left_x, content_y, column_w, 260.0f, "INPUT");

        if (definitive_settings_action(
            draw, layout, "input_bindings", "Configure Keyboard Bindings",
            left_x + 1.0f, content_y + 43.0f, column_w - 2.0f)) {
            show_bindings_config_ = true;
            show_settings_ = false;
            definitive_detailed_settings_ = false;
            ImGui::End();
            return;
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Change the keyboard keys used for PlayStation buttons.");

        if (definitive_settings_switch(
            draw, layout, "input_stop_eof", "Stop Playback at EOF",
            left_x + 1.0f, content_y + 43.0f + row_step, column_w - 2.0f,
            input_movie_stop_at_eof_)) {
            if (input_movie_stop_at_eof_) {
                input_movie_loop_ = false;
            }
            input_recorder_.set_end_behavior(input_movie_end_behavior());
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Stops an input movie when the recorded frames end.");

        if (definitive_settings_switch(
            draw, layout, "input_loop", "Loop Playback",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, input_movie_loop_)) {
            if (input_movie_loop_) {
                input_movie_stop_at_eof_ = false;
            }
            input_recorder_.set_end_behavior(input_movie_end_behavior());
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Restarts input-movie playback automatically at the end.");

        draw_settings_section(
            draw, layout, right_x, content_y, column_w, 160.0f, "GAMEPAD");
        const std::string gamepad_title =
            input_ && input_->has_gamepad()
                ? input_->gamepad_name()
                : "No Gamepad Detected";
        add_text(draw, layout, right_x + 18.0f, content_y + 58.0f, 13.0f,
            input_ && input_->has_gamepad()
                ? rgba(178, 221, 190, 245)
                : rgba(188, 195, 202, 235),
            gamepad_title.c_str());
        add_text(draw, layout, right_x + 18.0f, content_y + 84.0f, 10.6f,
            rgba(171, 181, 191, 235),
            input_ && input_->has_gamepad()
                ? "Connected gamepads are mapped automatically."
                : "Connect a controller and VibeStation will auto-map it.");
        break;
    }

    case 1: { // Video
        draw_settings_section(
            draw, layout, left_x, content_y, column_w, 260.0f, "DISPLAY");

        const char* resolution_modes[] = {
            "320x240", "640x480", "1024x768"
        };
        int resolution_index = static_cast<int>(g_output_resolution_mode);
        if (definitive_settings_combo(
            draw, layout, "video_resolution", "Output Resolution",
            left_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            resolution_index, resolution_modes,
            IM_ARRAYSIZE(resolution_modes))) {
            resolution_index = std::clamp(resolution_index, 0, 2);
            g_output_resolution_mode =
                static_cast<OutputResolutionMode>(resolution_index);
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Sets the final framebuffer size shown by VibeStation.");

        const char* deinterlace_modes[] = {
            "Weave", "Bob", "Blend"
        };
        int deinterlace_index = static_cast<int>(g_deinterlace_mode);
        if (definitive_settings_combo(
            draw, layout, "video_deinterlace", "Deinterlace",
            left_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f,
            deinterlace_index, deinterlace_modes,
            IM_ARRAYSIZE(deinterlace_modes))) {
            deinterlace_index = std::clamp(deinterlace_index, 0, 2);
            g_deinterlace_mode =
                static_cast<DeinterlaceMode>(deinterlace_index);
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Chooses how interlaced PS1 video fields are combined.");

        if (definitive_settings_switch(
            draw, layout, "video_filter", "Bilinear Presentation Filter",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, g_bilinear_filtering)) {
            if (renderer_) {
                renderer_->set_bilinear_filtering(g_bilinear_filtering);
            }
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Smooths the final image when scaling instead of keeping hard pixels.");

        draw_settings_section(
            draw, layout, right_x, content_y, column_w, 190.0f, "GPU");

        if (definitive_settings_switch(
            draw, layout, "video_fast_gpu", "Fast Mode",
            right_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            g_gpu_fast_mode)) {
            if (!g_gpu_fast_mode) {
                g_gpu_extreme_fast_mode = false;
            }
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f,
            "Uses optimized GPU paths for lower CPU usage with some artifact risk.");

        if (definitive_settings_switch(
            draw, layout, "video_extreme_gpu", "Extreme Fast Mode",
            right_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, g_gpu_extreme_fast_mode)) {
            if (g_gpu_extreme_fast_mode) {
                g_gpu_fast_mode = true;
            }
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step,
            "Trades more shading and transparency accuracy for additional speed.");
        break;
    }

    case 2: { // Audio
        draw_settings_section(
            draw, layout, left_x, content_y, column_w, 326.0f, "LATENCY");

        int target_latency = static_cast<int>(g_spu_audio_target_latency_ms);
        if (definitive_settings_slider_int(
            draw, layout, "audio_target", "Target Latency",
            left_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            target_latency, 10, 500, "%d ms")) {
            g_spu_audio_target_latency_ms =
                static_cast<u32>(std::clamp(target_latency, 10, 500));
            g_spu_audio_soft_latency_ms = std::max(
                g_spu_audio_soft_latency_ms, g_spu_audio_target_latency_ms);
            g_spu_audio_max_latency_ms = std::max(
                g_spu_audio_max_latency_ms, g_spu_audio_soft_latency_ms);
            apply_audio_settings();
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Preferred amount of queued audio before playback.");

        int soft_latency = static_cast<int>(g_spu_audio_soft_latency_ms);
        if (definitive_settings_slider_int(
            draw, layout, "audio_soft", "Soft Correction Starts",
            left_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, soft_latency,
            static_cast<int>(g_spu_audio_target_latency_ms),
            750, "%d ms")) {
            g_spu_audio_soft_latency_ms =
                static_cast<u32>(std::clamp(
                    soft_latency,
                    static_cast<int>(g_spu_audio_target_latency_ms), 750));
            g_spu_audio_max_latency_ms = std::max(
                g_spu_audio_max_latency_ms, g_spu_audio_soft_latency_ms);
            apply_audio_settings();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Starts gentle queue correction when latency grows beyond this point.");

        int max_latency = static_cast<int>(g_spu_audio_max_latency_ms);
        if (definitive_settings_slider_int(
            draw, layout, "audio_max", "Maximum Latency",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, max_latency,
            static_cast<int>(g_spu_audio_soft_latency_ms),
            1000, "%d ms")) {
            g_spu_audio_max_latency_ms =
                static_cast<u32>(std::clamp(
                    max_latency,
                    static_cast<int>(g_spu_audio_soft_latency_ms), 1000));
            apply_audio_settings();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Hard ceiling before excess queued audio is trimmed.");

        float xa_buffer = g_spu_xa_buffer_seconds;
        if (definitive_settings_slider_float(
            draw, layout, "audio_xa", "XA Buffer",
            left_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            column_w - 2.0f, xa_buffer, 0.0f, 5.0f, "%.2f sec")) {
            g_spu_xa_buffer_seconds = std::clamp(xa_buffer, 0.0f, 5.0f);
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            "Controls buffering for XA and other streamed CD audio.");

        draw_settings_section(
            draw, layout, right_x, content_y, column_w, 326.0f, "PLAYBACK");

        if (definitive_settings_switch(
            draw, layout, "audio_queue", "Enable Audio Queue",
            right_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            g_spu_enable_audio_queue)) {
            apply_audio_settings();
        }
        note(right_x + 1.0f, content_y + 43.0f,
            "Uses the bounded host queue for steadier playback.");

        if (definitive_settings_switch(
            draw, layout, "audio_trim", "Crossfaded Smooth Trim",
            right_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, g_spu_enable_smooth_trim)) {
            apply_audio_settings();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step,
            "Crossfades queue corrections to make trims less audible.");

        if (definitive_settings_switch(
            draw, layout, "audio_lag_stutter", "Lag Stutter Effect",
            right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, g_spu_enable_lag_stutter)) {
            apply_audio_settings();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Repeats a short audio fragment when emulation falls behind.");

        if (definitive_settings_switch(
            draw, layout, "audio_slow_stutter", "Slowdown Stutter Loop",
            right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            column_w - 2.0f, g_spu_enable_slowdown_stutter)) {
            apply_audio_settings();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            "Loops a small audio segment while slowdown mode is active.");
        break;
    }

    case 3: { // System
        draw_settings_section(
            draw, layout, left_x, content_y, column_w, 326.0f, "CPU & PERFORMANCE");

        const char* cpu_backend_labels[] = {
            "Interpreter", "Decoded Block", "x64 JIT"
        };
        int cpu_backend =
            cpu_execution_mode_to_config_value(g_cpu_execution_mode);
        if (definitive_settings_combo(
            draw, layout, "system_cpu", "CPU Backend",
            left_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            cpu_backend, cpu_backend_labels,
            IM_ARRAYSIZE(cpu_backend_labels))) {
            const bool was_running = emu_runner_.is_running();
            if (was_running) {
                emu_runner_.pause_and_wait_idle();
            }
            g_cpu_execution_mode =
                cpu_execution_mode_from_config_value(cpu_backend);
            if (system_) {
                system_->cpu().flush_cpu_backend();
            }
            save_persistent_config();
            if (was_running) {
                emu_runner_.set_running(true);
            }
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Selects interpreter, decoded blocks, or the x64 JIT CPU core.");

        const char* turbo_modes[] = { "200%", "400%", "Unlimited" };
        int turbo_mode = 0;
        if (config_turbo_speed_percent_ <= 0) {
            turbo_mode = 2;
        }
        else if (config_turbo_speed_percent_ >= 400) {
            turbo_mode = 1;
        }
        if (definitive_settings_combo(
            draw, layout, "system_turbo", "Turbo Speed",
            left_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, turbo_mode, turbo_modes,
            IM_ARRAYSIZE(turbo_modes))) {
            config_turbo_speed_percent_ =
                turbo_mode == 2 ? 0 : (turbo_mode == 1 ? 400 : 200);
            apply_speed_override();
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Sets the emulation speed used while holding the turbo hotkey.");

        int slowdown = config_slowdown_speed_percent_;
        if (definitive_settings_slider_int(
            draw, layout, "system_slowdown", "Slowdown Speed",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, slowdown, 10, 100, "%d%%")) {
            config_slowdown_speed_percent_ = std::clamp(slowdown, 10, 100);
            apply_speed_override();
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Sets the emulation speed used while holding the slowdown hotkey.");

        if (definitive_settings_switch(
            draw, layout, "system_low_spec", "Low-spec Mode",
            left_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            column_w - 2.0f, config_low_spec_mode_)) {
            g_low_spec_mode = config_low_spec_mode_;
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            "Reduces internal work for slower PCs with some quality tradeoffs.");

        draw_settings_section(
            draw, layout, right_x, content_y, column_w, 394.0f, "PLAYBACK & SERVICES");

        if (definitive_settings_switch(
            draw, layout, "system_vsync", "VSync Playback",
            right_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
            config_vsync_)) {
            SDL_GL_SetSwapInterval(config_vsync_ ? 1 : 0);
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f,
            "Synchronizes presentation with the display refresh rate.");

        if (definitive_settings_switch(
            draw, layout, "system_direct_boot", "Direct Disc Boot",
            right_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, config_direct_disc_boot_)) {
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step,
            "Skips the BIOS intro and starts the inserted game directly.");

        if (definitive_settings_switch(
            draw, layout, "system_rewind", "Enable Rewind",
            right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f, config_rewind_enabled_)) {
            emu_runner_.configure_rewind(
                config_rewind_enabled_,
                config_rewind_buffer_seconds_,
                static_cast<int>(system_ ? system_->target_fps() : 60.0));
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Keeps frame snapshots so gameplay can be rewound with Right Ctrl.");

        int rewind_seconds = config_rewind_buffer_seconds_;
        if (definitive_settings_slider_int(
            draw, layout, "system_rewind_seconds", "Rewind Buffer",
            right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            column_w - 2.0f, rewind_seconds, 1, 10, "%d sec")) {
            config_rewind_buffer_seconds_ =
                std::clamp(rewind_seconds, 1, 10);
            emu_runner_.set_rewind_buffer_seconds(
                config_rewind_buffer_seconds_,
                static_cast<int>(system_ ? system_->target_fps() : 60.0));
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            "Controls how many seconds of rewind history are retained.");

        if (definitive_settings_switch(
            draw, layout, "system_discord", "Discord Rich Presence",
            right_x + 1.0f, content_y + 43.0f + row_step * 4.0f,
            column_w - 2.0f, config_discord_rich_presence_)) {
            sync_discord_presence_config();
            save_persistent_config();
        }
        note(right_x + 1.0f, content_y + 43.0f + row_step * 4.0f,
            "Shows the current VibeStation session in the Discord desktop app.");
        break;
    }

    case 4: { // Memory Cards
        draw_settings_section(
            draw, layout, left_x, content_y, 1012.0f, 260.0f, "MEMORY CARDS");

        const char* memory_modes[] = {
            "Generic", "Per-Game", "Disabled"
        };
        int slot1 = std::clamp(config_memory_card_mode_[0], 0, 2);
        if (definitive_settings_combo(
            draw, layout, "memory_slot1", "Slot 1 Mode",
            left_x + 1.0f, content_y + 43.0f, 1010.0f,
            slot1, memory_modes, IM_ARRAYSIZE(memory_modes))) {
            config_memory_card_mode_[0] = slot1;
            apply_memory_card_settings(true);
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Generic shares one card; Per-Game creates a card for each disc.");

        int slot2 = std::clamp(config_memory_card_mode_[1], 0, 2);
        if (definitive_settings_combo(
            draw, layout, "memory_slot2", "Slot 2 Mode",
            left_x + 1.0f, content_y + 43.0f + row_step, 1010.0f,
            slot2, memory_modes, IM_ARRAYSIZE(memory_modes))) {
            config_memory_card_mode_[1] = slot2;
            apply_memory_card_settings(true);
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Controls how the second virtual PlayStation memory card is mounted.");

        if (definitive_settings_action(
            draw, layout, "memory_apply", "Apply Memory Card Settings",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f, 1010.0f)) {
            apply_memory_card_settings(true);
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Refreshes the currently mounted card files using the selected modes.");
        break;
    }

    case 5: { // Experimental
        draw_settings_section(
            draw, layout, left_x, content_y, 1012.0f, 326.0f, "EXPERIMENTAL");

        if (definitive_settings_switch(
            draw, layout, "experimental_bios_size", "Experimental BIOS Size Mode",
            left_x + 1.0f, content_y + 43.0f, 1010.0f,
            g_experimental_bios_size_mode)) {
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Accepts KB-aligned BIOS images outside normal PS1 size checks.");

        if (definitive_settings_switch(
            draw, layout, "experimental_ps2_bios", "Unsafe PS2 BIOS Mode",
            left_x + 1.0f, content_y + 43.0f + row_step, 1010.0f,
            g_unsafe_ps2_bios_mode)) {
            if (g_unsafe_ps2_bios_mode) {
                g_experimental_bios_size_mode = true;
            }
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Maps the full BIOS size for PS2 BIOS experiments; instability is expected.");

        if (definitive_settings_switch(
            draw, layout, "experimental_opcode", "Unhandled Opcode Fallback",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f, 1010.0f,
            g_experimental_unhandled_special_returns_zero)) {
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Lets unknown instructions fall back instead of halting immediately.");

        if (definitive_settings_switch(
            draw, layout, "experimental_dma", "DMA Command Sanitizer",
            left_x + 1.0f, content_y + 43.0f + row_step * 3.0f, 1010.0f,
            g_experimental_dma_command_sanitizer)) {
            save_persistent_config();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
            "Coerces malformed DMA channel commands into legal transfer commands.");
        break;
    }

    case 6: { // Customize
        draw_settings_section(
            draw, layout, left_x, content_y, column_w, 260.0f, "THEME");

        const int preset_count = ui_theme::theme_preset_count();
        const int safe_preset_count = std::min(preset_count, 64);
        const char* theme_preset_labels[64] = {};
        for (int i = 0; i < safe_preset_count; ++i) {
            theme_preset_labels[i] =
                ui_theme::theme_preset_by_index(i).label;
        }
        ui_theme::g_selected_theme_preset_index =
            std::clamp(
                ui_theme::g_selected_theme_preset_index,
                0, std::max(0, safe_preset_count - 1));

        int preset_index = ui_theme::g_selected_theme_preset_index;
        if (safe_preset_count > 0 &&
            definitive_settings_combo(
                draw, layout, "customize_preset", "Preset",
                left_x + 1.0f, content_y + 43.0f, column_w - 2.0f,
                preset_index, theme_preset_labels, safe_preset_count)) {
            ui_theme::g_selected_theme_preset_index = preset_index;
            ui_theme::apply_theme_preset_by_index(preset_index);
            ui_theme::apply_theme_style(ImGui::GetStyle());
            ui_theme::mark_theme_settings_dirty();
        }
        note(left_x + 1.0f, content_y + 43.0f,
            "Applies a predefined color scheme to the standard and detailed UI.");

        bool simple_theme = ui_theme::g_theme_settings.simple;
        if (definitive_settings_switch(
            draw, layout, "customize_simple", "Simple Customization",
            left_x + 1.0f, content_y + 43.0f + row_step,
            column_w - 2.0f, simple_theme)) {
            ui_theme::g_theme_settings.simple = simple_theme;
            ui_theme::mark_theme_settings_dirty();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step,
            "Enables the main background, surface, accent, text and list colors.");

        if (definitive_settings_action(
            draw, layout, "customize_reset", "Reset Theme Colors",
            left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            column_w - 2.0f)) {
            ui_theme::reset_theme_settings();
            ui_theme::apply_theme_style(ImGui::GetStyle());
            ui_theme::mark_theme_settings_dirty();
        }
        note(left_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
            "Restores VibeStation's default theme colors.");

        draw_settings_section(
            draw, layout, right_x, content_y, column_w, 394.0f, "SIMPLE COLORS");

        if (!ui_theme::g_theme_settings.simple) {
            add_text(draw, layout, right_x + 18.0f, content_y + 63.0f, 11.5f,
                rgba(184, 194, 204, 238),
                "Enable Simple Customization to edit these colors.");
        }
        else {
            bool theme_changed = false;

            ImVec4 background = ui_theme::g_theme_settings.background;
            if (definitive_settings_color(
                draw, layout, "customize_background", "Background",
                right_x + 1.0f, content_y + 43.0f,
                column_w - 2.0f, background)) {
                ui_theme::g_theme_settings.background = background;
                theme_changed = true;
            }
            note(right_x + 1.0f, content_y + 43.0f,
                "Base window background color.");

            ImVec4 surface = ui_theme::g_theme_settings.surface;
            if (definitive_settings_color(
                draw, layout, "customize_surface", "Surface",
                right_x + 1.0f, content_y + 43.0f + row_step,
                column_w - 2.0f, surface)) {
                ui_theme::g_theme_settings.surface = surface;
                theme_changed = true;
            }
            note(right_x + 1.0f, content_y + 43.0f + row_step,
                "Panels, controls and raised surfaces.");

            ImVec4 accent = ui_theme::g_theme_settings.accent;
            if (definitive_settings_color(
                draw, layout, "customize_accent", "Accent",
                right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
                column_w - 2.0f, accent)) {
                ui_theme::g_theme_settings.accent = accent;
                theme_changed = true;
            }
            note(right_x + 1.0f, content_y + 43.0f + row_step * 2.0f,
                "Highlights, selections and active controls.");

            ImVec4 text_color = ui_theme::g_theme_settings.text;
            if (definitive_settings_color(
                draw, layout, "customize_text", "Text",
                right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
                column_w - 2.0f, text_color)) {
                ui_theme::g_theme_settings.text = text_color;
                theme_changed = true;
            }
            note(right_x + 1.0f, content_y + 43.0f + row_step * 3.0f,
                "Primary text color used by the standard UI.");

            ImVec4 lists = ui_theme::g_theme_settings.lists;
            if (definitive_settings_color(
                draw, layout, "customize_lists", "Lists",
                right_x + 1.0f, content_y + 43.0f + row_step * 4.0f,
                column_w - 2.0f, lists)) {
                ui_theme::g_theme_settings.lists = lists;
                theme_changed = true;
            }
            note(right_x + 1.0f, content_y + 43.0f + row_step * 4.0f,
                "Popup and list background color.");

            if (theme_changed) {
                ui_theme::sync_theme_overall_from_basics(
                    ui_theme::g_theme_settings);
                ui_theme::rebuild_theme_colors_from_basics(
                    ui_theme::g_theme_settings);
                ui_theme::apply_theme_style(ImGui::GetStyle());
                ui_theme::mark_theme_settings_dirty();
            }
        }
        break;
    }

    default:
        definitive_settings_tab_ = 0;
        break;
    }

    draw->AddLine(
        layout.point(panel_x + 28.0f, panel_y + panel_h - 92.0f),
        layout.point(panel_x + panel_w - 28.0f, panel_y + panel_h - 92.0f),
        rgba(82, 97, 111, 155), layout.px(1.0f));

    ImGui::SetCursorScreenPos(
        layout.point(panel_x + 38.0f, panel_y + panel_h - 66.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, layout.px(2.0f));
    ImGui::PushStyleVar(
        ImGuiStyleVar_FramePadding, layout.size(4.0f, 4.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, rgba(20, 28, 36, 245));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, rgba(31, 44, 56, 250));
    ImGui::PushStyleColor(ImGuiCol_CheckMark, rgba(199, 224, 244, 255));
    ImGui::PushStyleColor(ImGuiCol_Text, rgba(220, 226, 232, 245));

    bool detailed = definitive_detailed_settings_;
    const bool detailed_changed =
        ImGui::Checkbox("Detailed Settings", &detailed);

    ImGui::PopStyleColor(4);
    ImGui::PopStyleVar(2);

    add_text(draw, layout,
        panel_x + 190.0f, panel_y + panel_h - 60.0f, 10.0f,
        rgba(166, 177, 187, 230),
        "Shows logging, diagnostics, profiling and other developer-oriented controls.");

    if (detailed_changed) {
        definitive_detailed_settings_ = detailed;
    }

    add_text_right(
        draw, layout,
        panel_x + panel_w - 38.0f,
        panel_y + panel_h - 58.0f,
        9.5f, rgba(128, 140, 151, 210),
        VIBESTATION_VERSION_STRING);

    ImGui::End();
}

void App::panel_definitive_home() {
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 window_pos = ImGui::GetWindowPos();
    const ImVec2 window_size = ImGui::GetWindowSize();

    if (!g_launcher_intro_complete) {
        const bool skip_intro =
            ImGui::IsKeyPressed(ImGuiKey_Space, false) ||
            ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false);

        if (skip_intro) {
            // End the intro on this frame and return once so the same keypress
            // cannot also activate a launcher button underneath it.
            g_launcher_intro_elapsed = kLauncherIntroDuration;
            g_launcher_intro_complete = true;
            ensure_background_texture_loaded();
            return;
        }

        const float dt = std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);
        g_launcher_intro_elapsed += dt;
        if (g_launcher_intro_elapsed >= kLauncherIntroDuration) {
            g_launcher_intro_elapsed = kLauncherIntroDuration;
            g_launcher_intro_complete = true;
        }
    }

    const bool launcher_intro_active = !g_launcher_intro_complete;

    if (!g_launcher_quote_selected) {
        const Uint64 entropy =
            SDL_GetPerformanceCounter() ^
            (static_cast<Uint64>(SDL_GetTicks()) << 32);
        g_launcher_quote_index =
            static_cast<size_t>(entropy % kLauncherQuotes.size());
        g_launcher_quote_selected = true;
    }

    // Load/soften the photograph while the boot presentation is still on
    // black so the transition into the launcher is hitch-free.
    ensure_background_texture_loaded();

    if (launcher_intro_active &&
        g_launcher_intro_elapsed < kUiBackgroundBegin) {
        draw_boot_presentation(
            window_pos, window_size, g_launcher_intro_elapsed);
        return;
    }

    Layout layout = make_layout(window_pos, window_size);

    draw_background(draw, window_pos, window_size);
    draw_readability_shade(draw, window_pos, window_size);

    const ImVec2 bottom0(window_pos.x, window_pos.y + window_size.y * 0.64f);
    const ImVec2 bottom1(window_pos.x + window_size.x, window_pos.y + window_size.y);
    draw->AddRectFilledMultiColor(bottom0, bottom1,
        rgba(1, 3, 6, 10), rgba(1, 3, 6, 10),
        rgba(1, 3, 6, 206), rgba(1, 3, 6, 206));

    add_text(draw, layout, 48.0f, 36.0f, 54.0f,
        rgba(223, 225, 228, 248), "VibeStation");

    constexpr std::array<ImU32, 4> accent_colors = {
        IM_COL32(194, 44, 56, 255),
        IM_COL32(52, 128, 125, 255),
        IM_COL32(177, 145, 72, 255),
        IM_COL32(52, 93, 157, 255),
    };
    constexpr std::array<ImU32, 4> accent_glow_colors = {
        IM_COL32(194, 44, 56, 40),
        IM_COL32(52, 128, 125, 40),
        IM_COL32(177, 145, 72, 40),
        IM_COL32(52, 93, 157, 40),
    };
    const float accent_time = static_cast<float>(ImGui::GetTime());
    for (int i = 0; i < 4; ++i) {
        const float pulse = 0.55f +
            0.45f * std::sin(accent_time * 1.35f + static_cast<float>(i) * 0.78f);
        const ImVec2 p0 = layout.point(50.0f + i * 32.0f, 116.0f);
        const ImVec2 p1 = layout.point(76.0f + i * 32.0f, 126.0f);
        const float spread = layout.px(1.5f + pulse * 1.25f);
        draw->AddRectFilled(
            ImVec2(p0.x - spread, p0.y - spread),
            ImVec2(p1.x + spread, p1.y + spread),
            accent_glow_colors[static_cast<size_t>(i)]);
        draw->AddRectFilled(p0, p1, accent_colors[static_cast<size_t>(i)]);
    }
    add_text_right(draw, layout, 1235.0f, 34.0f, 11.5f,
        rgba(176, 183, 191, 232), VIBESTATION_VERSION_STRING);
    const LauncherQuote& launcher_quote =
        kLauncherQuotes[g_launcher_quote_index];
    add_text_right(draw, layout, 1235.0f, 57.0f, 12.5f,
        rgba(198, 203, 210, 238), launcher_quote.line1);
    add_text_right(draw, layout, 1235.0f, 77.0f, 12.5f,
        rgba(198, 203, 210, 238), launcher_quote.line2);
    const ImVec2 dash0 = layout.point(1208.0f, 103.0f);
    const ImVec2 dash1 = layout.point(1235.0f, 103.0f);
    draw->AddLine(dash0, dash1, rgba(180, 184, 190, 190), layout.px(1.0f));

    const bool start_pressed = menu_button(layout, draw, 0, MenuIcon::Play,
        "Start Emulation", "Load BIOS and start playing");
    const bool load_game_pressed = menu_button(layout, draw, 1, MenuIcon::Folder,
        "Load Game", "Choose a game from your library");
    const bool change_bios_pressed = menu_button(layout, draw, 2, MenuIcon::Chip,
        "Change BIOS", "Manage BIOS files");
    const bool settings_pressed = menu_button(layout, draw, 3, MenuIcon::Settings,
        "Settings", "Configure emulator options");
    const bool exit_pressed = menu_button(layout, draw, 4, MenuIcon::Exit,
        "Exit", "Close VibeStation");

    const auto choose_bios = [this]() -> bool {
        std::string path = open_file_dialog(
            "BIOS Files (*.bin)\0*.bin\0All Files\0*.*\0", "Select PS1 BIOS");
        if (path.empty()) {
            return false;
        }

        emu_runner_.pause_and_wait_idle();
        disable_ram_reaper_mode();
        disable_gpu_reaper_mode();
        disable_sound_reaper_mode();
        if (!system_->load_bios(path)) {
            status_message_ = "Failed to load BIOS!";
            return false;
        }

        bios_path_ = path;
        save_persistent_config();
        has_started_emulation_ = false;
        set_grim_reaper_mode(false);
        status_message_ = "BIOS loaded: " + system_->bios().get_info();
        return true;
    };

    if (start_pressed && !launcher_intro_active &&
        g_launcher_start_transition == LauncherStartTransition::None) {
        if (!system_->bios_loaded() && !choose_bios()) {
            // File picker cancelled or BIOS failed to load.
        }
        else {
            const bool has_selected_game =
                !game_bin_path_.empty() || system_->disc_loaded();
            g_launcher_start_transition = has_selected_game
                ? LauncherStartTransition::Disc
                : LauncherStartTransition::Bios;
            g_launcher_start_transition_elapsed = 0.0f;
            status_message_ = has_selected_game
                ? "Starting selected game..."
                : "Starting emulation...";
        }
    }

    const bool launcher_transitioning =
        g_launcher_start_transition != LauncherStartTransition::None;

    if (load_game_pressed && !launcher_intro_active &&
        !launcher_transitioning) {
        std::string path = open_file_dialog(
            "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
            "Select PS1 Game");
        if (!path.empty()) {
            std::string bin;
            std::string cue;
            std::string error;
            if (!resolve_disc_paths(path, bin, cue, error)) {
                status_message_ = error;
            }
            else {
                load_disc_from_ui(bin, cue);
            }
        }
    }

    if (change_bios_pressed && !launcher_intro_active &&
        !launcher_transitioning) {
        choose_bios();
    }
    if (settings_pressed && !launcher_intro_active &&
        !launcher_transitioning) {
        show_settings_ = true;
    }
    if (exit_pressed && !launcher_intro_active &&
        !launcher_transitioning) {
        SDL_Event quit_event{};
        quit_event.type = SDL_QUIT;
        SDL_PushEvent(&quit_event);
    }

    float launcher_fade_alpha = 0.0f;
    bool launcher_started_this_frame = false;
    if (g_launcher_start_transition != LauncherStartTransition::None) {
        const float dt =
            std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);
        g_launcher_start_transition_elapsed += dt;

        const float fade_progress = std::clamp(
            g_launcher_start_transition_elapsed /
                kLauncherStartFadeSeconds,
            0.0f, 1.0f);
        launcher_fade_alpha = smoothstep01(fade_progress);

        if (fade_progress >= 1.0f) {
            const LauncherStartTransition requested =
                g_launcher_start_transition;
            g_launcher_start_transition = LauncherStartTransition::None;
            g_launcher_start_transition_elapsed = 0.0f;

            launcher_started_this_frame =
                requested == LauncherStartTransition::Disc
                    ? boot_disc_from_ui()
                    : start_bios_from_ui();

            // Keep this final launcher frame fully black. The next frame is
            // owned by the emulator screen if startup succeeded.
            launcher_fade_alpha = 1.0f;
            if (!launcher_started_this_frame) {
                // The boot helper has already supplied the useful error text.
                launcher_fade_alpha = 0.0f;
            }
        }
    }

    if (game_library_dirty_ ||
        (rom_directory_valid_ &&
            (SDL_GetTicks() - game_library_last_scan_ms_ > 15000u))) {
        refresh_game_library();
    }

    constexpr float panel_y = 585.0f;
    draw_panel(draw, layout, 32.0f, panel_y, 808.0f, 183.0f);
    draw_panel(draw, layout, 854.0f, panel_y, 394.0f, 183.0f);

    draw_folder_badge(draw, layout, 53.0f, panel_y + 17.0f);
    add_text(draw, layout, 86.0f, panel_y + 20.0f, 16.5f,
        rgba(240, 243, 247, 255), "Game Library");
    draw->AddLine(layout.point(46.0f, panel_y + 44.0f),
        layout.point(826.0f, panel_y + 44.0f),
        rgba(105, 116, 128, 165), layout.px(1.0f));

    const std::string rom_label = rom_directory_valid_
        ? "ROM Directory: " + rom_directory_
        : "ROM Directory: not set";
    add_text(draw, layout, 53.0f, panel_y + 52.0f, 12.8f,
        rgba(218, 223, 229, 250), rom_label.c_str());

    if (!rom_directory_valid_) {
        add_text(draw, layout, 53.0f, panel_y + 87.0f, 10.2f,
            rgba(224, 74, 74, 245), "No ROM directory configured.");
        add_text(draw, layout, 53.0f, panel_y + 111.0f, 10.0f,
            rgba(205, 210, 217, 238),
            "Set a ROM directory to scan and list games here.");
    }
    else if (game_library_.empty()) {
        add_text(draw, layout, 53.0f, panel_y + 89.0f, 10.0f,
            rgba(207, 180, 108, 235), "No playable disc images found.");
    }
    else {
        const std::string count_label =
            std::to_string(game_library_.size()) + " games";
        add_text_right(draw, layout, 796.0f, panel_y + 52.0f, 12.0f,
            rgba(213, 220, 228, 248), count_label.c_str());

        ImGui::SetCursorScreenPos(layout.point(53.0f, panel_y + 76.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::PushStyleVar(
            ImGuiStyleVar_ItemSpacing, layout.size(5.0f, 2.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ScrollbarSize, layout.px(7.0f));
        ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_ScrollbarBg, rgba(4, 7, 10, 90));
        ImGui::PushStyleColor(ImGuiCol_ScrollbarGrab, rgba(104, 120, 137, 145));
        ImGui::PushStyleColor(
            ImGuiCol_ScrollbarGrabHovered, rgba(150, 172, 194, 190));
        ImGui::PushStyleColor(
            ImGuiCol_ScrollbarGrabActive, rgba(193, 216, 238, 220));

        const ImGuiWindowFlags library_flags =
            ImGuiWindowFlags_NoBackground |
            (game_library_.size() > 3
                ? ImGuiWindowFlags_AlwaysVerticalScrollbar
                : ImGuiWindowFlags_None);
        ImGui::BeginChild(
            "##DefinitiveGameLibraryScroll",
            layout.size(755.0f, 59.0f), false, library_flags);

        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(game_library_.size()),
            layout.px(19.0f) + ImGui::GetStyle().ItemSpacing.y);
        while (clipper.Step()) {
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                const auto& entry = game_library_[static_cast<size_t>(i)];
                const bool is_selected =
                    entry.bin_path == game_bin_path_ &&
                    entry.cue_path == game_cue_path_;

                ImGui::PushID(i);
                ImGui::PushStyleColor(
                    ImGuiCol_Header, is_selected
                        ? rgba(58, 79, 98, 125)
                        : IM_COL32(0, 0, 0, 0));
                ImGui::PushStyleColor(
                    ImGuiCol_HeaderHovered, rgba(53, 68, 83, 150));
                ImGui::PushStyleColor(
                    ImGuiCol_HeaderActive, rgba(69, 91, 111, 175));
                const bool chosen = ImGui::Selectable(
                    entry.title.c_str(), is_selected, 0,
                    ImVec2(0.0f, layout.px(19.0f)));
                ImGui::PopStyleColor(3);
                ImGui::PopID();

                if (chosen && !launcher_intro_active) {
                    load_disc_from_ui(entry.bin_path, entry.cue_path);
                }
            }
        }

        ImGui::EndChild();
        ImGui::PopStyleColor(5);
        ImGui::PopStyleVar(3);
    }

    if (small_button(layout, "set_rom_dir", "Set Directory",
        53.0f, panel_y + 145.0f, 130.0f, 25.0f) &&
        !launcher_intro_active) {
        const std::string selected = open_folder_dialog("Select ROM Directory");
        if (!selected.empty()) {
            rom_directory_ = selected;
            game_library_dirty_ = true;
            save_persistent_config();
            refresh_game_library();
            status_message_ = "ROM directory set: " + rom_directory_;
        }
    }
    if (small_button(layout, "refresh_rom_dir", "Refresh",
        196.0f, panel_y + 145.0f, 90.0f, 25.0f, rom_directory_valid_) &&
        !launcher_intro_active) {
        game_library_dirty_ = true;
        refresh_game_library();
    }

    draw_info_badge(draw, layout, 875.0f, panel_y + 17.0f);
    add_text(draw, layout, 905.0f, panel_y + 19.0f, 16.5f,
        rgba(240, 243, 247, 255), "System Info");
    draw->AddLine(layout.point(868.0f, panel_y + 44.0f),
        layout.point(1232.0f, panel_y + 44.0f),
        rgba(105, 116, 128, 165), layout.px(1.0f));

    const ImU32 label_color = rgba(216, 222, 229, 250);
    const ImU32 value_color = rgba(244, 247, 250, 255);
    add_text(draw, layout, 875.0f, panel_y + 58.0f, 12.2f,
        label_color, "Emulator:");
    add_text(draw, layout, 995.0f, panel_y + 58.0f, 12.4f,
        value_color, "VibeStation");
    add_text(draw, layout, 875.0f, panel_y + 77.0f, 12.2f,
        label_color, "Version:");
    add_text(draw, layout, 995.0f, panel_y + 77.0f, 12.4f,
        value_color, VIBESTATION_VERSION_STRING);
    add_text(draw, layout, 875.0f, panel_y + 96.0f, 12.2f,
        label_color, "BIOS:");
    add_text(draw, layout, 995.0f, panel_y + 96.0f, 12.4f,
        value_color, system_->bios_loaded() ? "Loaded" : "Not loaded");
    add_text(draw, layout, 875.0f, panel_y + 115.0f, 12.2f,
        label_color, "ROM Directory:");
    add_text(draw, layout, 995.0f, panel_y + 115.0f, 12.4f,
        value_color, rom_directory_valid_ ? "Set" : "Not set");
    add_text(draw, layout, 875.0f, panel_y + 134.0f, 12.2f,
        label_color, "Games Found:");
    const std::string games_found = std::to_string(game_library_.size());
    add_text(draw, layout, 995.0f, panel_y + 134.0f, 12.4f,
        value_color, games_found.c_str());

    // Launcher-to-emulator transition. Use the viewport foreground draw list
    // so the fade also covers child windows (notably the scrollable game list).
    if (launcher_fade_alpha > 0.0f || launcher_started_this_frame) {
        const int fade_alpha = glow_alpha(255.0f * launcher_fade_alpha);
        ImDrawList* fade_draw =
            ImGui::GetForegroundDrawList();
        fade_draw->AddRectFilled(
            window_pos,
            ImVec2(window_pos.x + window_size.x, window_pos.y + window_size.y),
            rgba(0, 0, 0, fade_alpha));
    }

    if (launcher_intro_active) {
        draw_ui_initialization_overlay(
            window_pos, window_size, g_launcher_intro_elapsed);
    }
}
