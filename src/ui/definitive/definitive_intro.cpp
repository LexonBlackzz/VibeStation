#include "ui/definitive/definitive_shared.h"

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

constexpr float kIntroWakeBegin = 0.04f;
constexpr float kIntroIconBegin = 0.24f;
constexpr float kIntroIconFadeEnd = 0.94f;
constexpr float kIntroIconBlurEnd = 1.56f;
constexpr float kIntroIconZoomEnd = 2.72f;
constexpr float kIntroSweepBegin = 2.02f;
constexpr float kIntroSweepEnd = 2.72f;
constexpr float kIntroWordmarkBegin = 2.48f;
constexpr float kIntroWordmarkBlurEnd = 2.78f;
constexpr float kIntroDetailBegin = 2.86f;
constexpr float kIntroDetailEnd = 3.48f;
constexpr float kIntroHandoffBegin = 3.42f;
constexpr float kIntroOutroBegin = 3.56f;

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

void draw_radial_glow(
    ImDrawList* draw,
    const ImVec2& center,
    float radius,
    const ImVec4& color,
    float strength) {
    if (draw == nullptr ||
        radius <= 1.0f ||
        strength <= 0.001f) {
        return;
    }

    constexpr int kLayers = 10;
    for (int i = 0; i < kLayers; ++i) {
        const float t =
            static_cast<float>(i) /
            static_cast<float>(kLayers - 1);
        const float layer_radius =
            radius *
            (1.0f - 0.58f * t);
        const float alpha =
            strength *
            (0.028f + 0.030f * t);

        draw->AddCircleFilled(
            center,
            layer_radius,
            IM_COL32(
                definitive_ui::glow_alpha(
                    color.x * 255.0f),
                definitive_ui::glow_alpha(
                    color.y * 255.0f),
                definitive_ui::glow_alpha(
                    color.z * 255.0f),
                definitive_ui::glow_alpha(
                    alpha * 255.0f)),
            64);
    }
}

void draw_intro_grain(
    ImDrawList* draw,
    const ImVec2& pos,
    const ImVec2& size,
    float elapsed,
    float strength) {
    if (draw == nullptr ||
        strength <= 0.001f) {
        return;
    }

    uint32_t state =
        0x9E3779B9u ^
        static_cast<uint32_t>(
            std::max(
                0.0f,
                elapsed) *
            24.0f);

    constexpr int kSpecks = 72;
    for (int i = 0; i < kSpecks; ++i) {
        state =
            state * 1664525u +
            1013904223u;
        const float nx =
            static_cast<float>(
                state & 0xFFFFu) /
            65535.0f;

        state =
            state * 1664525u +
            1013904223u;
        const float ny =
            static_cast<float>(
                state & 0xFFFFu) /
            65535.0f;

        const float x =
            pos.x + size.x * nx;
        const float y =
            pos.y + size.y * ny;
        const int alpha =
            definitive_ui::glow_alpha(
                strength *
                (3.0f +
                    static_cast<float>(
                        (state >> 24u) & 0x3u)));

        draw->AddRectFilled(
            ImVec2(x, y),
            ImVec2(x + 1.0f, y + 1.0f),
            IM_COL32(
                228, 235, 244,
                alpha));
    }
}

void draw_centered_detail(
    ImDrawList* draw,
    const ImVec2& center,
    float font_size,
    ImU32 color,
    const char* text) {
    ImFont* font =
        definitive_ui::font_for_size(
            font_size);
    const ImVec2 text_size =
        font->CalcTextSizeA(
            font_size,
            FLT_MAX,
            0.0f,
            text);

    draw->AddText(
        font,
        font_size,
        ImVec2(
            center.x - text_size.x * 0.5f,
            center.y - text_size.y * 0.5f),
        color,
        text);
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

    const std::filesystem::path path =
        find_intro_icon_path();
    if (path.empty()) {
        return false;
    }

    int channels = 0;
    unsigned char* pixels =
        stbi_load(
            path.string().c_str(),
            &g_intro_icon_width,
            &g_intro_icon_height,
            &channels,
            4);

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

} // namespace

namespace definitive_ui {

void preload_intro_assets() {
    ensure_intro_icon_texture_loaded();
}

void release_intro_assets() {
    if (g_intro_icon_texture != 0) {
        glDeleteTextures(
            1,
            &g_intro_icon_texture);
        g_intro_icon_texture = 0;
    }

    if (g_intro_icon_blur_texture != 0) {
        glDeleteTextures(
            1,
            &g_intro_icon_blur_texture);
        g_intro_icon_blur_texture = 0;
    }

    g_intro_icon_width = 0;
    g_intro_icon_height = 0;
    g_intro_icon_load_attempted = false;
}

void draw_intro_presentation(
    const ImVec2& pos,
    const ImVec2& size,
    float elapsed) {
    ImDrawList* overlay =
        ImGui::GetForegroundDrawList();
    const ImVec2 end(
        pos.x + size.x,
        pos.y + size.y);

    const float handoff =
        timeline_progress(
            elapsed,
            kIntroHandoffBegin,
            kLauncherIntroDuration);

    overlay->AddRectFilled(
        pos,
        end,
        rgba(0, 0, 0, 255));

    // The blurred launcher photograph begins to exist before the intro ends.
    // The next launcher frame draws the same backdrop, so there is no black
    // flash between the startup presentation and the UI reveal.
    if (handoff > 0.001f) {
        draw_intro_handoff_background(
            overlay,
            pos,
            size,
            0.10f * handoff);
    }

    ensure_intro_icon_texture_loaded();

    const float unit =
        std::min(size.x, size.y);
    const ImVec2 center(
        pos.x + size.x * 0.5f,
        pos.y + size.y * 0.455f);

    const float wake =
        timeline_progress(
            elapsed,
            kIntroWakeBegin,
            0.92f);
    const float atmosphere_out =
        1.0f -
        timeline_progress(
            elapsed,
            kIntroOutroBegin,
            kLauncherIntroDuration);
    const float atmosphere =
        wake * atmosphere_out;

    // Four low-energy color pools are sampled from the actual icon. They wake
    // the black screen up without turning the intro into an RGB light show.
    constexpr std::array<ImVec2, 4> kGlowOffsets = {{
        ImVec2(-0.18f, -0.12f),
        ImVec2(0.18f, -0.10f),
        ImVec2(-0.16f, 0.13f),
        ImVec2(0.17f, 0.14f),
    }};

    for (size_t i = 0;
         i < kGlowOffsets.size();
         ++i) {
        const ImVec2 glow_center(
            center.x +
                unit *
                kGlowOffsets[i].x,
            center.y +
                unit *
                kGlowOffsets[i].y);

        draw_radial_glow(
            overlay,
            glow_center,
            unit * 0.285f,
            g_intro_ambient_colors[i],
            0.26f * atmosphere);
    }

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

    const float base_icon_size =
        std::clamp(
            unit * 0.285f,
            164.0f,
            260.0f);

    const float icon_size =
        base_icon_size * zoom;

    const ImVec2 icon0(
        center.x - icon_size * 0.5f,
        center.y - icon_size * 0.5f);
    const ImVec2 icon1(
        center.x + icon_size * 0.5f,
        center.y + icon_size * 0.5f);

    const float outro =
        1.0f -
        timeline_progress(
            elapsed,
            kIntroOutroBegin,
            kLauncherIntroDuration);
    const float visible =
        icon_alpha * outro;

    // During the first defocused beat, use two tiny color-separated blurred
    // copies. They collapse naturally as the real icon comes into focus.
    if (g_intro_icon_blur_texture != 0 &&
        blur_mix > 0.08f) {
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
        blur_mix > 0.001f) {
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

    if (g_intro_icon_texture != 0) {
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

        // A single restrained highlight travels across the real icon after it
        // resolves. Cropping the icon itself keeps the sweep inside its alpha.
        if (elapsed >= kIntroSweepBegin &&
            elapsed <= kIntroSweepEnd) {
            const float sweep =
                timeline_progress(
                    elapsed,
                    kIntroSweepBegin,
                    kIntroSweepEnd);
            constexpr float kStripWidth = 0.16f;
            const float center_u =
                -kStripWidth +
                (1.0f + kStripWidth * 2.0f) *
                    sweep;
            const float u0 =
                std::clamp(
                    center_u -
                        kStripWidth * 0.5f,
                    0.0f,
                    1.0f);
            const float u1 =
                std::clamp(
                    center_u +
                        kStripWidth * 0.5f,
                    0.0f,
                    1.0f);

            if (u1 > u0 + 0.001f) {
                const float x0 =
                    icon0.x +
                    icon_size * u0;
                const float x1 =
                    icon0.x +
                    icon_size * u1;
                const float sweep_peak =
                    std::sin(
                        3.14159265358979323846f *
                        sweep);

                overlay->AddImage(
                    (ImTextureID)(intptr_t)
                        g_intro_icon_texture,
                    ImVec2(x0, icon0.y),
                    ImVec2(x1, icon1.y),
                    ImVec2(u0, 0.0f),
                    ImVec2(u1, 1.0f),
                    rgba(
                        255,
                        255,
                        255,
                        glow_alpha(
                            74.0f *
                            visible *
                            sweep_peak)));
            }
        }
    }

    if (elapsed >= kIntroWordmarkBegin) {
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
        const float word_scale =
            1.025f -
            0.025f * word_settle;

        const int word_alpha =
            glow_alpha(
                242.0f * outro);

        draw_vista_wordmark(
            overlay,
            ImVec2(
                center.x,
                center.y +
                    base_icon_size * 0.70f),
            std::clamp(
                unit * 0.049f,
                30.0f,
                44.0f) *
                word_scale,
            word_blur,
            rgba(
                229,
                232,
                236,
                word_alpha));
    }

    if (elapsed >= kIntroDetailBegin &&
        elapsed <= kIntroHandoffBegin + 0.16f) {
        const float detail_in =
            timeline_progress(
                elapsed,
                kIntroDetailBegin,
                kIntroDetailBegin + 0.24f);
        const float detail_out =
            1.0f -
            timeline_progress(
                elapsed,
                kIntroDetailEnd,
                kIntroHandoffBegin + 0.16f);
        const float detail_alpha =
            detail_in *
            detail_out *
            outro;

        draw_centered_detail(
            overlay,
            ImVec2(
                center.x,
                center.y +
                    base_icon_size * 0.92f),
            std::clamp(
                unit * 0.0155f,
                10.0f,
                13.0f),
            rgba(
                164,
                176,
                190,
                glow_alpha(
                    128.0f *
                    detail_alpha)),
            "PS1 EMULATION SYSTEM");
    }

    // The final glow expands rather than simply disappearing. This visually
    // hands the startup illumination to the launcher background underneath.
    if (handoff > 0.001f &&
        handoff < 0.995f) {
        draw_radial_glow(
            overlay,
            center,
            unit *
                (0.30f +
                    0.64f * handoff),
            ImVec4(
                0.58f,
                0.72f,
                0.86f,
                1.0f),
            0.34f *
                (1.0f - handoff));
    }

    draw_intro_grain(
        overlay,
        pos,
        size,
        elapsed,
        1.0f -
            0.72f * handoff);
}

} // namespace definitive_ui
