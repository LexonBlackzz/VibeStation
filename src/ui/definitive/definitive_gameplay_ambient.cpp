#include "ui/definitive/definitive_shared.h"

#include <SDL_opengl.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

GLuint g_gameplay_ambient_texture = 0;
int g_gameplay_ambient_width = 0;
int g_gameplay_ambient_height = 0;

std::vector<unsigned char> g_gameplay_ambient_pixels;
std::vector<unsigned char> g_gameplay_ambient_scratch;

void box_blur_rgba(
    const std::vector<unsigned char>& source,
    std::vector<unsigned char>& output,
    std::vector<unsigned char>& scratch,
    int width,
    int height,
    int radius) {
    if (width <= 0 ||
        height <= 0 ||
        radius <= 0 ||
        source.size() <
            static_cast<size_t>(width) *
            static_cast<size_t>(height) * 4u) {
        output = source;
        return;
    }

    const size_t byte_count =
        static_cast<size_t>(width) *
        static_cast<size_t>(height) * 4u;
    scratch.resize(byte_count);
    output.resize(byte_count);

    const int kernel = radius * 2 + 1;

    for (int y = 0; y < height; ++y) {
        for (int channel = 0; channel < 4; ++channel) {
            int sum = 0;
            for (int k = -radius; k <= radius; ++k) {
                const int sx =
                    std::clamp(k, 0, width - 1);
                sum += source[
                    (static_cast<size_t>(y) *
                        static_cast<size_t>(width) +
                        static_cast<size_t>(sx)) *
                        4u +
                    static_cast<size_t>(channel)];
            }

            for (int x = 0; x < width; ++x) {
                scratch[
                    (static_cast<size_t>(y) *
                        static_cast<size_t>(width) +
                        static_cast<size_t>(x)) *
                        4u +
                    static_cast<size_t>(channel)] =
                    static_cast<unsigned char>(sum / kernel);

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
                        static_cast<size_t>(width) +
                        static_cast<size_t>(remove_x)) *
                        4u +
                    static_cast<size_t>(channel)];
                sum += source[
                    (static_cast<size_t>(y) *
                        static_cast<size_t>(width) +
                        static_cast<size_t>(add_x)) *
                        4u +
                    static_cast<size_t>(channel)];
            }
        }
    }

    for (int x = 0; x < width; ++x) {
        for (int channel = 0; channel < 4; ++channel) {
            int sum = 0;
            for (int k = -radius; k <= radius; ++k) {
                const int sy =
                    std::clamp(k, 0, height - 1);
                sum += scratch[
                    (static_cast<size_t>(sy) *
                        static_cast<size_t>(width) +
                        static_cast<size_t>(x)) *
                        4u +
                    static_cast<size_t>(channel)];
            }

            for (int y = 0; y < height; ++y) {
                output[
                    (static_cast<size_t>(y) *
                        static_cast<size_t>(width) +
                        static_cast<size_t>(x)) *
                        4u +
                    static_cast<size_t>(channel)] =
                    static_cast<unsigned char>(sum / kernel);

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

                sum -= scratch[
                    (static_cast<size_t>(remove_y) *
                        static_cast<size_t>(width) +
                        static_cast<size_t>(x)) *
                        4u +
                    static_cast<size_t>(channel)];
                sum += scratch[
                    (static_cast<size_t>(add_y) *
                        static_cast<size_t>(width) +
                        static_cast<size_t>(x)) *
                        4u +
                    static_cast<size_t>(channel)];
            }
        }
    }
}

void ensure_ambient_texture(
    int width,
    int height,
    const unsigned char* pixels) {
    if (width <= 0 ||
        height <= 0 ||
        pixels == nullptr) {
        return;
    }

    if (g_gameplay_ambient_texture == 0) {
        glGenTextures(
            1,
            &g_gameplay_ambient_texture);
        glBindTexture(
            GL_TEXTURE_2D,
            g_gameplay_ambient_texture);
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
    }
    else {
        glBindTexture(
            GL_TEXTURE_2D,
            g_gameplay_ambient_texture);
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    if (g_gameplay_ambient_width != width ||
        g_gameplay_ambient_height != height) {
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
        g_gameplay_ambient_width = width;
        g_gameplay_ambient_height = height;
    }
    else {
        glTexSubImage2D(
            GL_TEXTURE_2D,
            0,
            0,
            0,
            width,
            height,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            pixels);
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glBindTexture(GL_TEXTURE_2D, 0);
}

} // namespace

void definitive_ui::update_gameplay_ambient(
    const std::vector<u32>& rgba,
    int width,
    int height) {
    if (rgba.empty() ||
        width <= 0 ||
        height <= 0 ||
        rgba.size() <
            static_cast<size_t>(width) *
            static_cast<size_t>(height)) {
        return;
    }

    constexpr int kAmbientLongSide = 128;

    int target_width = kAmbientLongSide;
    int target_height = std::max(
        32,
        static_cast<int>(
            std::round(
                static_cast<float>(kAmbientLongSide) *
                static_cast<float>(height) /
                static_cast<float>(width))));

    if (height > width) {
        target_height = kAmbientLongSide;
        target_width = std::max(
            32,
            static_cast<int>(
                std::round(
                    static_cast<float>(kAmbientLongSide) *
                    static_cast<float>(width) /
                    static_cast<float>(height))));
    }

    const size_t target_bytes =
        static_cast<size_t>(target_width) *
        static_cast<size_t>(target_height) *
        4u;

    std::vector<unsigned char> downsampled(
        target_bytes);

    const unsigned char* source_bytes =
        reinterpret_cast<const unsigned char*>(
            rgba.data());

    for (int y = 0; y < target_height; ++y) {
        const int source_y =
            std::clamp(
                (y * height) /
                    std::max(1, target_height),
                0,
                height - 1);

        for (int x = 0; x < target_width; ++x) {
            const int source_x =
                std::clamp(
                    (x * width) /
                        std::max(1, target_width),
                    0,
                    width - 1);

            const size_t source_index =
                (static_cast<size_t>(source_y) *
                    static_cast<size_t>(width) +
                    static_cast<size_t>(source_x)) *
                4u;
            const size_t target_index =
                (static_cast<size_t>(y) *
                    static_cast<size_t>(target_width) +
                    static_cast<size_t>(x)) *
                4u;

            downsampled[target_index + 0u] =
                source_bytes[source_index + 0u];
            downsampled[target_index + 1u] =
                source_bytes[source_index + 1u];
            downsampled[target_index + 2u] =
                source_bytes[source_index + 2u];
            downsampled[target_index + 3u] =
                255u;
        }
    }

    // Two inexpensive box passes over a tiny texture produce a broad,
    // YouTube-like ambient glow without blurring the actual gameplay image.
    box_blur_rgba(
        downsampled,
        g_gameplay_ambient_pixels,
        g_gameplay_ambient_scratch,
        target_width,
        target_height,
        7);

    std::vector<unsigned char> second_pass;
    box_blur_rgba(
        g_gameplay_ambient_pixels,
        second_pass,
        g_gameplay_ambient_scratch,
        target_width,
        target_height,
        6);

    g_gameplay_ambient_pixels.swap(
        second_pass);

    ensure_ambient_texture(
        target_width,
        target_height,
        g_gameplay_ambient_pixels.data());
}

void definitive_ui::draw_gameplay_ambient(
    ImDrawList* draw,
    const ImVec2& area_pos,
    const ImVec2& area_size,
    const ImVec2& game_pos,
    const ImVec2& game_size,
    float bottom_overscan_v) {
    if (draw == nullptr ||
        area_size.x <= 0.0f ||
        area_size.y <= 0.0f) {
        return;
    }

    const ImVec2 area_end(
        area_pos.x + area_size.x,
        area_pos.y + area_size.y);

    // Always erase the customizable static gameplay background first.
    draw->AddRectFilled(
        area_pos,
        area_end,
        IM_COL32(4, 5, 7, 255));

    if (g_gameplay_ambient_texture == 0) {
        return;
    }

    const ImTextureID texture =
        (ImTextureID)(intptr_t)
            g_gameplay_ambient_texture;

    const float v1 =
        std::clamp(
            1.0f - bottom_overscan_v,
            0.0f,
            1.0f);

    constexpr int kAmbientAlpha = 220;

    const float left_width =
        std::max(
            0.0f,
            game_pos.x - area_pos.x);
    const float right_width =
        std::max(
            0.0f,
            area_end.x -
                (game_pos.x + game_size.x));
    const float top_height =
        std::max(
            0.0f,
            game_pos.y - area_pos.y);
    const float bottom_height =
        std::max(
            0.0f,
            area_end.y -
                (game_pos.y + game_size.y));

    if (left_width > 0.5f) {
        const ImVec2 p0 = area_pos;
        const ImVec2 p1(
            game_pos.x,
            area_end.y);

        draw->AddImage(
            texture,
            p0,
            p1,
            ImVec2(0.0f, 0.0f),
            ImVec2(0.46f, v1),
            IM_COL32(
                255, 255, 255,
                kAmbientAlpha));

        draw->AddRectFilledMultiColor(
            p0,
            p1,
            IM_COL32(0, 0, 0, 72),
            IM_COL32(0, 0, 0, 14),
            IM_COL32(0, 0, 0, 14),
            IM_COL32(0, 0, 0, 72));
    }

    if (right_width > 0.5f) {
        const ImVec2 p0(
            game_pos.x + game_size.x,
            area_pos.y);
        const ImVec2 p1 = area_end;

        draw->AddImage(
            texture,
            p0,
            p1,
            ImVec2(0.54f, 0.0f),
            ImVec2(1.0f, v1),
            IM_COL32(
                255, 255, 255,
                kAmbientAlpha));

        draw->AddRectFilledMultiColor(
            p0,
            p1,
            IM_COL32(0, 0, 0, 14),
            IM_COL32(0, 0, 0, 72),
            IM_COL32(0, 0, 0, 72),
            IM_COL32(0, 0, 0, 14));
    }

    if (top_height > 0.5f) {
        const ImVec2 p0 = area_pos;
        const ImVec2 p1(
            area_end.x,
            game_pos.y);

        draw->AddImage(
            texture,
            p0,
            p1,
            ImVec2(0.0f, 0.0f),
            ImVec2(1.0f, 0.46f),
            IM_COL32(
                255, 255, 255,
                kAmbientAlpha));

        draw->AddRectFilledMultiColor(
            p0,
            p1,
            IM_COL32(0, 0, 0, 72),
            IM_COL32(0, 0, 0, 72),
            IM_COL32(0, 0, 0, 14),
            IM_COL32(0, 0, 0, 14));
    }

    if (bottom_height > 0.5f) {
        const ImVec2 p0(
            area_pos.x,
            game_pos.y + game_size.y);
        const ImVec2 p1 = area_end;

        draw->AddImage(
            texture,
            p0,
            p1,
            ImVec2(0.0f, 0.54f),
            ImVec2(1.0f, v1),
            IM_COL32(
                255, 255, 255,
                kAmbientAlpha));

        draw->AddRectFilledMultiColor(
            p0,
            p1,
            IM_COL32(0, 0, 0, 14),
            IM_COL32(0, 0, 0, 14),
            IM_COL32(0, 0, 0, 72),
            IM_COL32(0, 0, 0, 72));
    }
}

void definitive_ui::release_gameplay_ambient_assets() {
    if (g_gameplay_ambient_texture != 0) {
        glDeleteTextures(
            1,
            &g_gameplay_ambient_texture);
        g_gameplay_ambient_texture = 0;
    }

    g_gameplay_ambient_width = 0;
    g_gameplay_ambient_height = 0;
    g_gameplay_ambient_pixels.clear();
    g_gameplay_ambient_scratch.clear();
}
