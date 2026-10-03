#include "ui/definitive/definitive_shared.h"
#include "ui/embedded_resource_ids.h"
#include "ui/embedded_resources.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <stb_image.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace {

GLuint g_intro_icon_texture = 0;
GLuint g_intro_icon_blur_texture = 0;
int g_intro_icon_width = 0;
int g_intro_icon_height = 0;
bool g_intro_icon_load_attempted = false;

std::array<ImVec4, 4> g_intro_ambient_colors = {{
    ImVec4(0.76f, 0.18f, 0.24f, 1.0f),
    ImVec4(0.18f, 0.50f, 0.52f, 1.0f),
    ImVec4(0.72f, 0.57f, 0.26f, 1.0f),
    ImVec4(0.20f, 0.38f, 0.72f, 1.0f),
}};

// Small generated textures: a soft round glow for the colour lights and a
// tiling grey noise for the film grain.
GLuint g_intro_glow_texture = 0;
GLuint g_intro_noise_texture = 0;

// The launcher's colour bars, which the four lights turn into.
constexpr std::array<std::array<int, 3>, 4> kBarColors = {{
    {{194, 44, 56}},
    {{52, 128, 125}},
    {{177, 145, 72}},
    {{52, 93, 157}},
}};

constexpr float kIntroWakeBegin = 0.04f;
constexpr float kIntroIconBegin = 0.24f;
constexpr float kIntroIconFadeEnd = 0.94f;
constexpr float kIntroIconBlurEnd = 1.56f;
constexpr float kIntroIconZoomEnd = 2.72f;
constexpr float kIntroSweepBegin = 2.02f;
constexpr float kIntroSweepEnd = 2.72f;
constexpr float kIntroWordmarkBegin = 2.48f;
constexpr float kIntroWordmarkBlurEnd = 2.78f;
// The icon leaves while the title glides (definitive_shared.h).
constexpr float kIntroIconLeaveEnd = definitive_ui::kIntroGlideStart + 0.55f;
// Each light leaves 80 ms after the previous one.
constexpr float kIntroLightStagger = 0.08f;
constexpr float kIntroLightTravel = 1.25f;
constexpr float kIntroGrainAlpha = 0.018f;

void compute_intro_ambient_colors(
    const unsigned char* pixels,
    int width,
    int height) {
    if (pixels == nullptr ||
        width <= 0 ||
        height <= 0) {
        return;
    }

    struct Accumulator {
        double r = 0.0;
        double g = 0.0;
        double b = 0.0;
        double weight = 0.0;
    };

    std::array<Accumulator, 4> sums{};

    const int step_x =
        std::max(1, width / 96);
    const int step_y =
        std::max(1, height / 96);

    for (int y = 0; y < height; y += step_y) {
        for (int x = 0; x < width; x += step_x) {
            const size_t index =
                (static_cast<size_t>(y) *
                    static_cast<size_t>(width) +
                    static_cast<size_t>(x)) *
                4u;

            const float alpha =
                static_cast<float>(pixels[index + 3u]) /
                255.0f;
            if (alpha < 0.08f) {
                continue;
            }

            const float r =
                static_cast<float>(pixels[index + 0u]) /
                255.0f;
            const float g =
                static_cast<float>(pixels[index + 1u]) /
                255.0f;
            const float b =
                static_cast<float>(pixels[index + 2u]) /
                255.0f;
            const float brightness =
                std::max(r, std::max(g, b));

            if (brightness < 0.08f) {
                continue;
            }

            const int quadrant =
                (x >= width / 2 ? 1 : 0) +
                (y >= height / 2 ? 2 : 0);

            const double weight =
                static_cast<double>(
                    alpha *
                    (0.35f + 0.65f * brightness));

            sums[static_cast<size_t>(quadrant)].r +=
                static_cast<double>(r) * weight;
            sums[static_cast<size_t>(quadrant)].g +=
                static_cast<double>(g) * weight;
            sums[static_cast<size_t>(quadrant)].b +=
                static_cast<double>(b) * weight;
            sums[static_cast<size_t>(quadrant)].weight +=
                weight;
        }
    }

    for (size_t i = 0;
         i < sums.size();
         ++i) {
        if (sums[i].weight <= 0.0001) {
            continue;
        }

        float r =
            static_cast<float>(
                sums[i].r /
                sums[i].weight);
        float g =
            static_cast<float>(
                sums[i].g /
                sums[i].weight);
        float b =
            static_cast<float>(
                sums[i].b /
                sums[i].weight);

        const float luminance =
            r * 0.2126f +
            g * 0.7152f +
            b * 0.0722f;
        constexpr float kSaturation = 1.10f;

        r = std::clamp(
            luminance +
                (r - luminance) *
                    kSaturation,
            0.0f,
            1.0f);
        g = std::clamp(
            luminance +
                (g - luminance) *
                    kSaturation,
            0.0f,
            1.0f);
        b = std::clamp(
            luminance +
                (b - luminance) *
                    kSaturation,
            0.0f,
            1.0f);

        g_intro_ambient_colors[i] =
            ImVec4(r, g, b, 1.0f);
    }
}

void set_additive_blend(const ImDrawList*, const ImDrawCmd*) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
}

void begin_additive(ImDrawList* draw) {
    draw->AddCallback(set_additive_blend, nullptr);
}

void end_additive(ImDrawList* draw) {
    draw->AddCallback(ImDrawCallback_ResetRenderState, nullptr);
}

ImU32 color_with_alpha(float r, float g, float b, float alpha) {
    return IM_COL32(
        definitive_ui::glow_alpha(r),
        definitive_ui::glow_alpha(g),
        definitive_ui::glow_alpha(b),
        definitive_ui::glow_alpha(255.0f * alpha));
}

// A light of `radius` at `center`; call between begin/end_additive.
void draw_glow(
    ImDrawList* draw,
    const ImVec2& center,
    float radius,
    ImU32 color) {
    if (g_intro_glow_texture == 0 || radius <= 0.5f) {
        return;
    }

    draw->AddImage(
        (ImTextureID)(intptr_t)g_intro_glow_texture,
        ImVec2(center.x - radius, center.y - radius),
        ImVec2(center.x + radius, center.y + radius),
        ImVec2(0.0f, 0.0f),
        ImVec2(1.0f, 1.0f),
        color);
}

bool upload_rgba_texture(
    GLuint& texture,
    const unsigned char* pixels,
    int width,
    int height) {
    if (pixels == nullptr ||
        width <= 0 ||
        height <= 0) {
        return false;
    }

    glGenTextures(1, &texture);
    if (texture == 0) {
        return false;
    }

    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(
        GL_TEXTURE_2D,
        GL_TEXTURE_MIN_FILTER,
        GL_LINEAR);
    glTexParameteri(
        GL_TEXTURE_2D,
        GL_TEXTURE_MAG_FILTER,
        GL_LINEAR);
    glTexParameteri(
        GL_TEXTURE_2D,
        GL_TEXTURE_WRAP_S,
        GL_CLAMP_TO_EDGE);
    glTexParameteri(
        GL_TEXTURE_2D,
        GL_TEXTURE_WRAP_T,
        GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGBA,
        width,
        height,
        0,
        GL_RGBA,
        GL_UNSIGNED_BYTE,
        pixels);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

std::vector<unsigned char> make_blurred_rgba(
    const unsigned char* source,
    int width,
    int height,
    int radius) {
    const size_t pixel_count =
        static_cast<size_t>(width) *
        static_cast<size_t>(height);

    std::vector<unsigned char> horizontal(
        pixel_count * 4u);
    std::vector<unsigned char> output(
        pixel_count * 4u);

    if (source == nullptr ||
        width <= 0 ||
        height <= 0 ||
        radius <= 0) {
        return output;
    }

    const int kernel = radius * 2 + 1;

    for (int y = 0; y < height; ++y) {
        for (int channel = 0;
             channel < 4;
             ++channel) {
            int sum = 0;

            for (int k = -radius;
                 k <= radius;
                 ++k) {
                const int sx =
                    std::clamp(
                        k,
                        0,
                        width - 1);
                sum += source[
                    (static_cast<size_t>(y) *
                        width + sx) *
                        4u +
                    channel];
            }

            for (int x = 0;
                 x < width;
                 ++x) {
                horizontal[
                    (static_cast<size_t>(y) *
                        width + x) *
                        4u +
                    channel] =
                    static_cast<unsigned char>(
                        sum / kernel);

                const int remove_x =
                    std::clamp(
                        x - radius,
                        0,
                        width - 1);
                const int add_x =
                    std::clamp(
                        x + radius + 1,
                        0,
                        width - 1);

                sum -= source[
                    (static_cast<size_t>(y) *
                        width +
                        remove_x) *
                        4u +
                    channel];
                sum += source[
                    (static_cast<size_t>(y) *
                        width +
                        add_x) *
                        4u +
                    channel];
            }
        }
    }

    for (int x = 0; x < width; ++x) {
        for (int channel = 0;
             channel < 4;
             ++channel) {
            int sum = 0;

            for (int k = -radius;
                 k <= radius;
                 ++k) {
                const int sy =
                    std::clamp(
                        k,
                        0,
                        height - 1);
                sum += horizontal[
                    (static_cast<size_t>(sy) *
                        width + x) *
                        4u +
                    channel];
            }

            for (int y = 0;
                 y < height;
                 ++y) {
                output[
                    (static_cast<size_t>(y) *
                        width + x) *
                        4u +
                    channel] =
                    static_cast<unsigned char>(
                        sum / kernel);

                const int remove_y =
                    std::clamp(
                        y - radius,
                        0,
                        height - 1);
                const int add_y =
                    std::clamp(
                        y + radius + 1,
                        0,
                        height - 1);

                sum -= horizontal[
                    (static_cast<size_t>(
                        remove_y) *
                        width + x) *
                        4u +
                    channel];
                sum += horizontal[
                    (static_cast<size_t>(
                        add_y) *
                        width + x) *
                        4u +
                    channel];
            }
        }
    }

    return output;
}

std::filesystem::path find_intro_icon_path() {
    std::array<std::filesystem::path, 4>
        candidates{};

    std::error_code ec;
    const std::filesystem::path cwd =
        std::filesystem::current_path(ec);

    if (!ec) {
        candidates[0] =
            cwd / "resources" / "icon512x512.png";
        candidates[1] =
            cwd / ".." / "resources" /
            "icon512x512.png";
    }

    if (char* base = SDL_GetBasePath()) {
        const std::filesystem::path base_path(base);
        SDL_free(base);

        candidates[2] =
            base_path /
            "resources" /
            "icon512x512.png";
        candidates[3] =
            base_path /
            ".." /
            "resources" /
            "icon512x512.png";
    }

    for (const auto& candidate : candidates) {
        if (!candidate.empty() &&
            std::filesystem::exists(
                candidate, ec) &&
            !ec) {
            return candidate;
        }

        ec.clear();
    }

    return {};
}

bool ensure_intro_icon_texture_loaded() {
    if (g_intro_icon_texture != 0) {
        return true;
    }

    if (g_intro_icon_load_attempted) {
        return false;
    }

    g_intro_icon_load_attempted = true;

    int channels = 0;
    unsigned char* pixels = nullptr;

    const vibestation::EmbeddedResourceView embedded =
        vibestation::embedded_resource(
            vibestation::resource_ids::StartupIcon);
    if (embedded) {
        pixels = stbi_load_from_memory(
            embedded.data,
            static_cast<int>(embedded.size),
            &g_intro_icon_width,
            &g_intro_icon_height,
            &channels,
            4);
    }

    if (pixels == nullptr) {
        const std::filesystem::path path =
            find_intro_icon_path();
        if (!path.empty()) {
            pixels =
                stbi_load(
                    path.string().c_str(),
                    &g_intro_icon_width,
                    &g_intro_icon_height,
                    &channels,
                    4);
        }
    }

    if (pixels == nullptr ||
        g_intro_icon_width <= 0 ||
        g_intro_icon_height <= 0) {
        if (pixels != nullptr) {
            stbi_image_free(pixels);
        }

        g_intro_icon_width = 0;
        g_intro_icon_height = 0;
        return false;
    }

    compute_intro_ambient_colors(
        pixels,
        g_intro_icon_width,
        g_intro_icon_height);

    const bool sharp_uploaded =
        upload_rgba_texture(
            g_intro_icon_texture,
            pixels,
            g_intro_icon_width,
            g_intro_icon_height);

    const std::vector<unsigned char> blurred =
        make_blurred_rgba(
            pixels,
            g_intro_icon_width,
            g_intro_icon_height,
            16);

    if (!blurred.empty()) {
        upload_rgba_texture(
            g_intro_icon_blur_texture,
            blurred.data(),
            g_intro_icon_width,
            g_intro_icon_height);
    }

    stbi_image_free(pixels);
    return sharp_uploaded;
}

void draw_vista_wordmark(
    ImDrawList* draw,
    const ImVec2& center,
    float font_size,
    float blur_amount,
    ImU32 color) {
    using namespace definitive_ui;

    ImFont* font = font_for_size(font_size);
    const char* text = "VibeStation";
    const ImVec2 text_size =
        font->CalcTextSizeA(
            font_size,
            FLT_MAX,
            0.0f,
            text);
    const ImVec2 base(
        center.x - text_size.x * 0.5f,
        center.y - text_size.y * 0.5f);

    const float blur =
        std::clamp(
            blur_amount,
            0.0f,
            1.0f);

    if (blur <= 0.01f) {
        draw->AddText(
            font,
            font_size,
            base,
            color,
            text);
        return;
    }

    const int r =
        (color >> IM_COL32_R_SHIFT) & 0xFF;
    const int g =
        (color >> IM_COL32_G_SHIFT) & 0xFF;
    const int b =
        (color >> IM_COL32_B_SHIFT) & 0xFF;
    const int a =
        (color >> IM_COL32_A_SHIFT) & 0xFF;

    const float radius =
        std::max(
            1.0f,
            font_size * 0.13f * blur);

    constexpr std::array<ImVec2, 12> offsets = {{
        {-1.0f, 0.0f},
        {1.0f, 0.0f},
        {0.0f, -1.0f},
        {0.0f, 1.0f},
        {-0.72f, -0.72f},
        {0.72f, -0.72f},
        {-0.72f, 0.72f},
        {0.72f, 0.72f},
        {-1.45f, 0.0f},
        {1.45f, 0.0f},
        {0.0f, -1.45f},
        {0.0f, 1.45f},
    }};

    const int halo_alpha =
        glow_alpha(
            static_cast<float>(a) *
            (0.055f + 0.035f * blur));

    for (const ImVec2& offset : offsets) {
        draw->AddText(
            font,
            font_size,
            ImVec2(
                base.x +
                    offset.x * radius,
                base.y +
                    offset.y * radius),
            rgba(
                r,
                g,
                b,
                halo_alpha),
            text);
    }

    const int core_alpha =
        glow_alpha(
            static_cast<float>(a) *
            (0.88f +
                0.12f *
                (1.0f - blur)));

    draw->AddText(
        font,
        font_size,
        base,
        rgba(
            r,
            g,
            b,
            core_alpha),
        text);
}

void ensure_intro_effect_textures() {
    if (g_intro_glow_texture == 0) {
        // Bright core, 35% at 0.45 of the radius, nothing at the edge.
        constexpr int kSize = 64;
        std::vector<unsigned char> pixels(kSize * kSize * 4u);
        for (int y = 0; y < kSize; ++y) {
            for (int x = 0; x < kSize; ++x) {
                const float dx = (static_cast<float>(x) + 0.5f) / kSize * 2.0f - 1.0f;
                const float dy = (static_cast<float>(y) + 0.5f) / kSize * 2.0f - 1.0f;
                const float d = std::sqrt(dx * dx + dy * dy);
                float a = 0.0f;
                if (d < 0.45f) {
                    a = 1.0f - 0.65f * (d / 0.45f);
                }
                else if (d < 1.0f) {
                    a = 0.35f * (1.0f - (d - 0.45f) / 0.55f);
                }
                unsigned char* p = &pixels[(static_cast<size_t>(y) * kSize + x) * 4u];
                p[0] = p[1] = p[2] = 255;
                p[3] = static_cast<unsigned char>(std::lround(255.0f * a));
            }
        }
        upload_rgba_texture(g_intro_glow_texture, pixels.data(), kSize, kSize);
    }

    if (g_intro_noise_texture == 0) {
        constexpr int kSize = 256;
        std::vector<unsigned char> pixels(kSize * kSize * 4u);
        uint32_t state = 0x2545F491u;
        for (size_t i = 0; i < pixels.size(); i += 4) {
            state = state * 1664525u + 1013904223u;
            const unsigned char v = static_cast<unsigned char>(state >> 24u);
            pixels[i + 0] = pixels[i + 1] = pixels[i + 2] = v;
            pixels[i + 3] = 255;
        }
        if (upload_rgba_texture(g_intro_noise_texture, pixels.data(), kSize, kSize)) {
            glBindTexture(GL_TEXTURE_2D, g_intro_noise_texture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }
}

} // namespace

namespace definitive_ui {

void preload_intro_assets() {
    ensure_intro_icon_texture_loaded();
    ensure_intro_effect_textures();
}

void release_intro_assets() {
    for (GLuint* texture : {&g_intro_icon_texture, &g_intro_icon_blur_texture,
                            &g_intro_glow_texture, &g_intro_noise_texture}) {
        if (*texture != 0) {
            glDeleteTextures(1, texture);
            *texture = 0;
        }
    }

    g_intro_icon_width = 0;
    g_intro_icon_height = 0;
    g_intro_icon_load_attempted = false;
}

float intro_color_bar_alpha(int index, float elapsed) {
    const float start =
        kIntroGlideStart + kIntroLightStagger * static_cast<float>(index);
    const float travel =
        std::clamp((elapsed - start) / kIntroLightTravel, 0.0f, 1.0f);
    return timeline_progress(travel, 0.8f, 1.0f);
}

void draw_intro_presentation(
    const ImVec2& pos,
    const ImVec2& size,
    float elapsed,
    const IntroTargets& targets) {
    ImDrawList* overlay =
        ImGui::GetForegroundDrawList();
    const ImVec2 end(
        pos.x + size.x,
        pos.y + size.y);

    // Until the glide the intro owns the screen; afterwards the launcher is
    // drawn underneath and only the parts still moving are drawn here.
    if (elapsed < kIntroGlideStart) {
        overlay->AddRectFilled(
            pos,
            end,
            rgba(0, 0, 0, 255));
    }

    ensure_intro_icon_texture_loaded();
    ensure_intro_effect_textures();

    const float unit =
        std::min(size.x, size.y);
    // Pixel offsets below were tuned for an 800 px tall window.
    const float design = unit / 800.0f;
    const ImVec2 center(
        pos.x + size.x * 0.5f,
        pos.y + size.y * 0.455f);

    const float wake =
        timeline_progress(
            elapsed,
            kIntroWakeBegin,
            0.92f);

    const float icon_alpha =
        timeline_progress(
            elapsed,
            kIntroIconBegin,
            kIntroIconFadeEnd);

    const float blur_mix =
        1.0f -
        timeline_progress(
            elapsed,
            kIntroIconBegin + 0.08f,
            kIntroIconBlurEnd);

    const float zoom_t =
        timeline_progress(
            elapsed,
            kIntroIconBegin,
            kIntroIconZoomEnd);
    const float settle_t =
        timeline_progress(
            elapsed,
            1.72f,
            kIntroIconZoomEnd);
    const float overshoot =
        0.016f *
        std::sin(
            3.14159265358979323846f *
            settle_t);
    const float zoom =
        0.885f +
        0.115f * zoom_t +
        overshoot;

    // The icon rises a little, shrinks and fades while the title glides.
    const float leave =
        timeline_progress(
            elapsed,
            kIntroGlideStart,
            kIntroIconLeaveEnd);

    const float base_icon_size =
        std::clamp(
            unit * 0.285f,
            164.0f,
            260.0f);

    const float icon_size =
        base_icon_size * zoom * (1.0f - 0.07f * leave);
    const ImVec2 icon_center(
        center.x,
        center.y - 22.0f * design * leave);

    const ImVec2 icon0(
        icon_center.x - icon_size * 0.5f,
        icon_center.y - icon_size * 0.5f);
    const ImVec2 icon1(
        icon_center.x + icon_size * 0.5f,
        icon_center.y + icon_size * 0.5f);

    const float visible =
        icon_alpha * (1.0f - leave);

    // During the first defocused beat, use two tiny color-separated blurred
    // copies. They collapse naturally as the real icon comes into focus.
    if (g_intro_icon_blur_texture != 0 &&
        blur_mix > 0.08f &&
        visible > 0.001f) {
        const float fringe =
            2.5f *
            blur_mix;
        const int fringe_alpha =
            glow_alpha(
                54.0f *
                visible *
                blur_mix);

        overlay->AddImage(
            (ImTextureID)(intptr_t)
                g_intro_icon_blur_texture,
            ImVec2(
                icon0.x - fringe,
                icon0.y),
            ImVec2(
                icon1.x - fringe,
                icon1.y),
            ImVec2(0.0f, 0.0f),
            ImVec2(1.0f, 1.0f),
            rgba(
                255, 72, 82,
                fringe_alpha));

        overlay->AddImage(
            (ImTextureID)(intptr_t)
                g_intro_icon_blur_texture,
            ImVec2(
                icon0.x + fringe,
                icon0.y),
            ImVec2(
                icon1.x + fringe,
                icon1.y),
            ImVec2(0.0f, 0.0f),
            ImVec2(1.0f, 1.0f),
            rgba(
                84, 186, 255,
                fringe_alpha));
    }

    if (g_intro_icon_blur_texture != 0 &&
        blur_mix > 0.001f &&
        visible > 0.001f) {
        overlay->AddImage(
            (ImTextureID)(intptr_t)
                g_intro_icon_blur_texture,
            icon0,
            icon1,
            ImVec2(0.0f, 0.0f),
            ImVec2(1.0f, 1.0f),
            rgba(
                255,
                255,
                255,
                glow_alpha(
                    255.0f *
                    visible *
                    blur_mix)));
    }

    if (g_intro_icon_texture != 0 &&
        visible > 0.001f) {
        overlay->AddImage(
            (ImTextureID)(intptr_t)
                g_intro_icon_texture,
            icon0,
            icon1,
            ImVec2(0.0f, 0.0f),
            ImVec2(1.0f, 1.0f),
            rgba(
                255,
                255,
                255,
                glow_alpha(
                    255.0f *
                    visible *
                    (1.0f - blur_mix))));

        // A soft band of light crosses the icon once it is in focus. It is
        // the icon itself drawn additively in thin slices, so it stays inside
        // the icon's shape and takes on its colours.
        if (elapsed >= kIntroSweepBegin &&
            elapsed <= kIntroSweepEnd) {
            const float sweep =
                timeline_progress(
                    elapsed,
                    kIntroSweepBegin,
                    kIntroSweepEnd);
            const float sweep_peak =
                std::sin(
                    3.14159265358979323846f *
                    sweep);
            constexpr int kSlices = 12;
            constexpr float kHalfWidth = 0.18f;
            const float center_u = -0.25f + 1.5f * sweep;

            begin_additive(overlay);
            for (int s = 0; s < kSlices; ++s) {
                const float a =
                    center_u - kHalfWidth +
                    2.0f * kHalfWidth * static_cast<float>(s) / kSlices;
                const float b = a + 2.0f * kHalfWidth / kSlices;
                const float weight =
                    1.0f - std::fabs((a + b) * 0.5f - center_u) / kHalfWidth;
                const float u0 = std::clamp(a, 0.0f, 1.0f);
                const float u1 = std::clamp(b, 0.0f, 1.0f);
                if (u1 <= u0 + 0.0005f || weight <= 0.0f) {
                    continue;
                }

                overlay->AddImage(
                    (ImTextureID)(intptr_t)
                        g_intro_icon_texture,
                    ImVec2(icon0.x + icon_size * u0, icon0.y),
                    ImVec2(icon0.x + icon_size * u1, icon1.y),
                    ImVec2(u0, 0.0f),
                    ImVec2(u1, 1.0f),
                    color_with_alpha(
                        255.0f, 255.0f, 255.0f,
                        0.41f * visible * sweep_peak * weight));
            }
            end_additive(overlay);
        }
    }

    // The title comes into focus under the icon, then glides into the
    // launcher's own title spot and hands over to it at kIntroGlideEnd.
    if (elapsed >= kIntroWordmarkBegin &&
        elapsed < kIntroGlideEnd) {
        const float word_blur =
            1.0f -
            timeline_progress(
                elapsed,
                kIntroWordmarkBegin,
                kIntroWordmarkBlurEnd);
        const float word_settle =
            timeline_progress(
                elapsed,
                kIntroWordmarkBegin,
                kIntroWordmarkBlurEnd + 0.18f);
        const float start_size =
            std::clamp(
                unit * 0.049f,
                30.0f,
                44.0f) *
            (1.025f - 0.025f * word_settle);
        const ImVec2 start_text =
            font_for_size(start_size)->CalcTextSizeA(
                start_size, FLT_MAX, 0.0f, "VibeStation");
        const ImVec2 start_pos(
            center.x - start_text.x * 0.5f,
            center.y + base_icon_size * 0.70f - start_text.y * 0.5f);

        const float glide =
            ease_in_out_cubic(
                (elapsed - kIntroGlideStart) /
                (kIntroGlideEnd - kIntroGlideStart));
        const auto mix = [glide](float a, float b) {
            return a + (b - a) * glide;
        };

        const float font_size =
            mix(start_size, targets.brand_size);
        const ImVec2 text_size =
            font_for_size(font_size)->CalcTextSizeA(
                font_size, FLT_MAX, 0.0f, "VibeStation");
        const ImVec2 top_left(
            mix(start_pos.x, targets.brand_pos.x),
            mix(start_pos.y, targets.brand_pos.y));

        draw_vista_wordmark(
            overlay,
            ImVec2(
                top_left.x + text_size.x * 0.5f,
                top_left.y + text_size.y * 0.5f),
            font_size,
            glide > 0.0f ? 0.0f : word_blur,
            text_color(
                rgba(
                    static_cast<int>(mix(229.0f, 223.0f)),
                    static_cast<int>(mix(232.0f, 225.0f)),
                    static_cast<int>(mix(236.0f, 228.0f)),
                    static_cast<int>(mix(242.0f, 248.0f)))));
    }

    // Four soft lights in colours sampled from the icon. At the glide each
    // shrinks and arcs into its colour bar under the launcher title.
    constexpr float kLightsDone =
        kIntroGlideStart + 3.0f * kIntroLightStagger +
        kIntroLightTravel + 0.5f;
    if (elapsed < kLightsDone) {
        constexpr std::array<ImVec2, 4> kPoolOffsets = {{
            ImVec2(-0.18f, -0.12f),
            ImVec2(0.18f, -0.10f),
            ImVec2(-0.16f, 0.13f),
            ImVec2(0.17f, 0.14f),
        }};

        begin_additive(overlay);
        for (size_t i = 0; i < kPoolOffsets.size(); ++i) {
            const float start =
                kIntroGlideStart +
                kIntroLightStagger * static_cast<float>(i);
            const float travel =
                std::clamp((elapsed - start) / kIntroLightTravel, 0.0f, 1.0f);
            const float move = ease_in_out_cubic(travel);

            const ImVec2 from(
                center.x + unit * kPoolOffsets[i].x,
                center.y + unit * kPoolOffsets[i].y);
            const ImVec2 to = targets.bar_centers[i];
            const float dx = to.x - from.x;
            const float dy = to.y - from.y;
            const float length = std::max(1.0f, std::sqrt(dx * dx + dy * dy));
            const float arc =
                std::sin(3.14159265358979323846f * move) *
                60.0f * design * (i % 2 != 0 ? 1.0f : -1.0f);
            const ImVec2 light(
                from.x + dx * move - dy / length * arc,
                from.y + dy * move + dx / length * arc);

            const float radius =
                unit * 0.215f +
                (12.0f * targets.ui_scale - unit * 0.215f) *
                    ease_out_cubic(travel);
            const ImVec4& ambient = g_intro_ambient_colors[i];
            const std::array<int, 3>& bar = kBarColors[i];
            const float base = 0.16f * wake;
            const float strength =
                travel <= 0.0f
                    ? base
                    : (base + (0.55f - base) * ease_out_cubic(travel)) *
                        (1.0f - timeline_progress(travel, 0.88f, 1.0f));

            draw_glow(
                overlay,
                light,
                radius,
                color_with_alpha(
                    ambient.x * 255.0f + (bar[0] - ambient.x * 255.0f) * move,
                    ambient.y * 255.0f + (bar[1] - ambient.y * 255.0f) * move,
                    ambient.z * 255.0f + (bar[2] - ambient.z * 255.0f) * move,
                    strength));

            // A brief flash as the light lands on its bar.
            if (travel >= 1.0f) {
                const float landed = start + kIntroLightTravel;
                const float flash =
                    1.0f - timeline_progress(elapsed, landed, landed + 0.5f);
                if (flash > 0.0f) {
                    draw_glow(
                        overlay,
                        to,
                        34.0f * targets.ui_scale,
                        color_with_alpha(
                            static_cast<float>(bar[0]),
                            static_cast<float>(bar[1]),
                            static_cast<float>(bar[2]),
                            0.35f * flash));
                }
            }
        }
        end_additive(overlay);
    }

    // Film grain, fading out with the glide.
    const float grain =
        kIntroGrainAlpha *
        (1.0f -
            timeline_progress(
                elapsed,
                kIntroGlideStart,
                kIntroGlideEnd));
    if (grain > 0.0005f &&
        g_intro_noise_texture != 0) {
        // New grain 24 times a second: jump to another spot in the tile.
        uint32_t state =
            0x9E3779B9u ^
            static_cast<uint32_t>(std::max(0.0f, elapsed) * 24.0f);
        state = state * 1664525u + 1013904223u;
        const float u = static_cast<float>(state >> 16u) / 65536.0f;
        state = state * 1664525u + 1013904223u;
        const float v = static_cast<float>(state >> 16u) / 65536.0f;
        // Each noise texel covers 3x3 pixels.
        const float span_u = size.x / (256.0f * 3.0f);
        const float span_v = size.y / (256.0f * 3.0f);

        begin_additive(overlay);
        overlay->AddImage(
            (ImTextureID)(intptr_t)g_intro_noise_texture,
            pos,
            end,
            ImVec2(u, v),
            ImVec2(u + span_u, v + span_v),
            color_with_alpha(255.0f, 255.0f, 255.0f, grain));
        end_additive(overlay);
    }
}

} // namespace definitive_ui
