#include "ui/definitive/definitive_shared.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <stb_image.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <filesystem>
#include <vector>

namespace {

GLuint g_intro_icon_texture = 0;
GLuint g_intro_icon_blur_texture = 0;
int g_intro_icon_width = 0;
int g_intro_icon_height = 0;
bool g_intro_icon_load_attempted = false;

constexpr float kIntroIconBegin = 0.18f;
constexpr float kIntroIconFadeEnd = 0.88f;
constexpr float kIntroIconBlurEnd = 1.58f;
constexpr float kIntroIconZoomEnd = 2.92f;
constexpr float kIntroWordmarkBegin = 2.52f;
constexpr float kIntroWordmarkBlurEnd = 2.78f;
constexpr float kIntroOutroBegin = 3.48f;

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
            (0.42f +
                0.58f *
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

    overlay->AddRectFilled(
        pos,
        end,
        rgba(0, 0, 0, 255));

    ensure_intro_icon_texture_loaded();

    const float unit =
        std::min(size.x, size.y);
    const ImVec2 center(
        pos.x + size.x * 0.5f,
        pos.y + size.y * 0.46f);

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

    const float zoom =
        0.90f + 0.105f * zoom_t;

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
    }

    if (elapsed >= kIntroWordmarkBegin) {
        const float word_blur =
            1.0f -
            timeline_progress(
                elapsed,
                kIntroWordmarkBegin,
                kIntroWordmarkBlurEnd);

        const int word_alpha =
            glow_alpha(242.0f * outro);

        draw_vista_wordmark(
            overlay,
            ImVec2(
                center.x,
                center.y +
                    base_icon_size * 0.70f),
            std::clamp(
                unit * 0.049f,
                30.0f,
                44.0f),
            word_blur,
            rgba(
                229,
                232,
                236,
                word_alpha));
    }
}

} // namespace definitive_ui
