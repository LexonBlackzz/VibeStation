#include "ui/definitive/definitive_shared.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace {

constexpr int kVerticalSegments = 12;
constexpr int kHorizontalSegments = 16;

using VerticalColors =
    std::array<ImVec4, kVerticalSegments>;
using HorizontalColors =
    std::array<ImVec4, kHorizontalSegments>;

VerticalColors g_left_colors{};
VerticalColors g_right_colors{};
HorizontalColors g_top_colors{};
HorizontalColors g_bottom_colors{};
bool g_ambient_initialized = false;

ImVec4 sample_region(
    const std::vector<u32>& rgba,
    int width,
    int height,
    int x0,
    int y0,
    int x1,
    int y1) {
    x0 = std::clamp(x0, 0, width);
    x1 = std::clamp(x1, 0, width);
    y0 = std::clamp(y0, 0, height);
    y1 = std::clamp(y1, 0, height);

    if (x1 <= x0 || y1 <= y0) {
        return ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
    }

    // Keep ambient sampling cheap even at high output resolutions.
    const int step_x =
        std::max(1, (x1 - x0) / 10);
    const int step_y =
        std::max(1, (y1 - y0) / 14);

    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;
    int sample_count = 0;

    for (int y = y0; y < y1; y += step_y) {
        for (int x = x0; x < x1; x += step_x) {
            const u32 pixel =
                rgba[
                    static_cast<size_t>(y) *
                        static_cast<size_t>(width) +
                    static_cast<size_t>(x)];

            red +=
                static_cast<double>(
                    pixel & 0xFFu);
            green +=
                static_cast<double>(
                    (pixel >> 8u) & 0xFFu);
            blue +=
                static_cast<double>(
                    (pixel >> 16u) & 0xFFu);
            ++sample_count;
        }
    }

    if (sample_count <= 0) {
        return ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
    }

    float r =
        static_cast<float>(
            red /
            static_cast<double>(sample_count) /
            255.0);
    float g =
        static_cast<float>(
            green /
            static_cast<double>(sample_count) /
            255.0);
    float b =
        static_cast<float>(
            blue /
            static_cast<double>(sample_count) /
            255.0);

    // Ambilight should read as emitted light rather than a copied image.
    // Slightly increase saturation and lift useful mid-tones while allowing
    // dark scenes to remain appropriately subdued.
    const float luminance =
        r * 0.2126f +
        g * 0.7152f +
        b * 0.0722f;
    constexpr float kSaturation = 1.30f;

    r = luminance + (r - luminance) * kSaturation;
    g = luminance + (g - luminance) * kSaturation;
    b = luminance + (b - luminance) * kSaturation;

    const float brightness =
        0.82f +
        0.30f *
            std::sqrt(
                std::clamp(luminance, 0.0f, 1.0f));

    r = std::clamp(r * brightness, 0.0f, 1.0f);
    g = std::clamp(g * brightness, 0.0f, 1.0f);
    b = std::clamp(b * brightness, 0.0f, 1.0f);

    return ImVec4(r, g, b, 1.0f);
}

template <size_t N>
void spatial_smooth(
    std::array<ImVec4, N>& colors) {
    if constexpr (N < 2) {
        return;
    }

    const std::array<ImVec4, N> source =
        colors;

    for (size_t i = 0; i < N; ++i) {
        const size_t prev =
            i > 0 ? i - 1 : i;
        const size_t next =
            i + 1 < N ? i + 1 : i;

        colors[i].x =
            source[prev].x * 0.22f +
            source[i].x * 0.56f +
            source[next].x * 0.22f;
        colors[i].y =
            source[prev].y * 0.22f +
            source[i].y * 0.56f +
            source[next].y * 0.22f;
        colors[i].z =
            source[prev].z * 0.22f +
            source[i].z * 0.56f +
            source[next].z * 0.22f;
        colors[i].w = 1.0f;
    }
}

template <size_t N>
void temporal_smooth(
    std::array<ImVec4, N>& current,
    const std::array<ImVec4, N>& target,
    float response) {
    for (size_t i = 0; i < N; ++i) {
        current[i].x +=
            (target[i].x - current[i].x) *
            response;
        current[i].y +=
            (target[i].y - current[i].y) *
            response;
        current[i].z +=
            (target[i].z - current[i].z) *
            response;
        current[i].w = 1.0f;
    }
}

ImU32 ambient_color(
    const ImVec4& color,
    int alpha,
    float strength = 1.0f) {
    const int r =
        std::clamp(
            static_cast<int>(
                std::round(
                    color.x *
                    255.0f *
                    strength)),
            0,
            255);
    const int g =
        std::clamp(
            static_cast<int>(
                std::round(
                    color.y *
                    255.0f *
                    strength)),
            0,
            255);
    const int b =
        std::clamp(
            static_cast<int>(
                std::round(
                    color.z *
                    255.0f *
                    strength)),
            0,
            255);

    return IM_COL32(
        r,
        g,
        b,
        std::clamp(alpha, 0, 255));
}

template <size_t N>
ImVec4 boundary_color(
    const std::array<ImVec4, N>& colors,
    size_t boundary) {
    if (boundary == 0) {
        return colors.front();
    }
    if (boundary >= N) {
        return colors.back();
    }

    const ImVec4& a =
        colors[boundary - 1];
    const ImVec4& b =
        colors[boundary];

    return ImVec4(
        (a.x + b.x) * 0.5f,
        (a.y + b.y) * 0.5f,
        (a.z + b.z) * 0.5f,
        1.0f);
}

void draw_vertical_ambilight(
    ImDrawList* draw,
    const VerticalColors& colors,
    float outer_x,
    float inner_x,
    float top_y,
    float height,
    bool inner_on_right) {
    if (std::abs(inner_x - outer_x) < 0.5f ||
        height <= 0.5f) {
        return;
    }

    for (int i = 0;
         i < kVerticalSegments;
         ++i) {
        const float t0 =
            static_cast<float>(i) /
            static_cast<float>(
                kVerticalSegments);
        const float t1 =
            static_cast<float>(i + 1) /
            static_cast<float>(
                kVerticalSegments);

        const float y0 =
            top_y + height * t0;
        const float y1 =
            top_y + height * t1;

        const ImVec4 top_color =
            boundary_color(
                colors,
                static_cast<size_t>(i));
        const ImVec4 bottom_color =
            boundary_color(
                colors,
                static_cast<size_t>(i + 1));

        const ImU32 inner_top =
            ambient_color(
                top_color,
                208,
                1.06f);
        const ImU32 inner_bottom =
            ambient_color(
                bottom_color,
                208,
                1.06f);
        const ImU32 outer_top =
            ambient_color(
                top_color,
                10,
                0.48f);
        const ImU32 outer_bottom =
            ambient_color(
                bottom_color,
                10,
                0.48f);

        const ImVec2 p0(
            std::min(outer_x, inner_x),
            y0 - 1.0f);
        const ImVec2 p1(
            std::max(outer_x, inner_x),
            y1 + 1.0f);

        if (inner_on_right) {
            draw->AddRectFilledMultiColor(
                p0,
                p1,
                outer_top,
                inner_top,
                inner_bottom,
                outer_bottom);
        }
        else {
            draw->AddRectFilledMultiColor(
                p0,
                p1,
                inner_top,
                outer_top,
                outer_bottom,
                inner_bottom);
        }
    }

    // A concentrated bloom directly beside the game edge keeps the effect
    // feeling like light emitted behind a display rather than a backdrop.
    const float width =
        std::abs(inner_x - outer_x);
    const float bloom_width =
        std::min(
            width,
            std::max(
                18.0f,
                width * 0.34f));

    for (int i = 0;
         i < kVerticalSegments;
         ++i) {
        const float t0 =
            static_cast<float>(i) /
            static_cast<float>(
                kVerticalSegments);
        const float t1 =
            static_cast<float>(i + 1) /
            static_cast<float>(
                kVerticalSegments);

        const float y0 =
            top_y + height * t0;
        const float y1 =
            top_y + height * t1;

        const ImVec4 top_color =
            boundary_color(
                colors,
                static_cast<size_t>(i));
        const ImVec4 bottom_color =
            boundary_color(
                colors,
                static_cast<size_t>(i + 1));

        const float bloom_outer_x =
            inner_on_right
                ? inner_x - bloom_width
                : inner_x + bloom_width;

        const ImVec2 p0(
            std::min(
                bloom_outer_x,
                inner_x),
            y0 - 1.0f);
        const ImVec2 p1(
            std::max(
                bloom_outer_x,
                inner_x),
            y1 + 1.0f);

        const ImU32 edge_top =
            ambient_color(
                top_color,
                108,
                1.12f);
        const ImU32 edge_bottom =
            ambient_color(
                bottom_color,
                108,
                1.12f);
        const ImU32 fade_top =
            ambient_color(
                top_color,
                0,
                0.85f);
        const ImU32 fade_bottom =
            ambient_color(
                bottom_color,
                0,
                0.85f);

        if (inner_on_right) {
            draw->AddRectFilledMultiColor(
                p0,
                p1,
                fade_top,
                edge_top,
                edge_bottom,
                fade_bottom);
        }
        else {
            draw->AddRectFilledMultiColor(
                p0,
                p1,
                edge_top,
                fade_top,
                fade_bottom,
                edge_bottom);
        }
    }
}

void draw_horizontal_ambilight(
    ImDrawList* draw,
    const HorizontalColors& colors,
    float left_x,
    float width,
    float outer_y,
    float inner_y,
    bool inner_on_bottom) {
    if (std::abs(inner_y - outer_y) < 0.5f ||
        width <= 0.5f) {
        return;
    }

    for (int i = 0;
         i < kHorizontalSegments;
         ++i) {
        const float t0 =
            static_cast<float>(i) /
            static_cast<float>(
                kHorizontalSegments);
        const float t1 =
            static_cast<float>(i + 1) /
            static_cast<float>(
                kHorizontalSegments);

        const float x0 =
            left_x + width * t0;
        const float x1 =
            left_x + width * t1;

        const ImVec4 left_color =
            boundary_color(
                colors,
                static_cast<size_t>(i));
        const ImVec4 right_color =
            boundary_color(
                colors,
                static_cast<size_t>(i + 1));

        const ImU32 inner_left =
            ambient_color(
                left_color,
                196,
                1.05f);
        const ImU32 inner_right =
            ambient_color(
                right_color,
                196,
                1.05f);
        const ImU32 outer_left =
            ambient_color(
                left_color,
                8,
                0.46f);
        const ImU32 outer_right =
            ambient_color(
                right_color,
                8,
                0.46f);

        const ImVec2 p0(
            x0 - 1.0f,
            std::min(
                outer_y,
                inner_y));
        const ImVec2 p1(
            x1 + 1.0f,
            std::max(
                outer_y,
                inner_y));

        if (inner_on_bottom) {
            draw->AddRectFilledMultiColor(
                p0,
                p1,
                outer_left,
                outer_right,
                inner_right,
                inner_left);
        }
        else {
            draw->AddRectFilledMultiColor(
                p0,
                p1,
                inner_left,
                inner_right,
                outer_right,
                outer_left);
        }
    }
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

    VerticalColors left_target{};
    VerticalColors right_target{};
    HorizontalColors top_target{};
    HorizontalColors bottom_target{};

    const int edge_band_x =
        std::max(
            2,
            width / 12);
    const int edge_band_y =
        std::max(
            2,
            height / 12);

    for (int i = 0;
         i < kVerticalSegments;
         ++i) {
        const int y0 =
            (height * i) /
            kVerticalSegments;
        const int y1 =
            (height * (i + 1)) /
            kVerticalSegments;

        left_target[
            static_cast<size_t>(i)] =
            sample_region(
                rgba,
                width,
                height,
                0,
                y0,
                edge_band_x,
                y1);

        right_target[
            static_cast<size_t>(i)] =
            sample_region(
                rgba,
                width,
                height,
                width - edge_band_x,
                y0,
                width,
                y1);
    }

    for (int i = 0;
         i < kHorizontalSegments;
         ++i) {
        const int x0 =
            (width * i) /
            kHorizontalSegments;
        const int x1 =
            (width * (i + 1)) /
            kHorizontalSegments;

        top_target[
            static_cast<size_t>(i)] =
            sample_region(
                rgba,
                width,
                height,
                x0,
                0,
                x1,
                edge_band_y);

        bottom_target[
            static_cast<size_t>(i)] =
            sample_region(
                rgba,
                width,
                height,
                x0,
                height - edge_band_y,
                x1,
                height);
    }

    // Spatial smoothing removes visible segment boundaries while preserving
    // where the colors originate along each edge.
    spatial_smooth(left_target);
    spatial_smooth(left_target);
    spatial_smooth(right_target);
    spatial_smooth(right_target);
    spatial_smooth(top_target);
    spatial_smooth(top_target);
    spatial_smooth(bottom_target);
    spatial_smooth(bottom_target);

    if (!g_ambient_initialized) {
        g_left_colors = left_target;
        g_right_colors = right_target;
        g_top_colors = top_target;
        g_bottom_colors = bottom_target;
        g_ambient_initialized = true;
        return;
    }

    // Philips-style lighting has some persistence instead of changing
    // instantly with every frame. This also avoids distracting flicker.
    constexpr float kTemporalResponse = 0.16f;
    temporal_smooth(
        g_left_colors,
        left_target,
        kTemporalResponse);
    temporal_smooth(
        g_right_colors,
        right_target,
        kTemporalResponse);
    temporal_smooth(
        g_top_colors,
        top_target,
        kTemporalResponse);
    temporal_smooth(
        g_bottom_colors,
        bottom_target,
        kTemporalResponse);
}

void definitive_ui::draw_gameplay_ambient(
    ImDrawList* draw,
    const ImVec2& area_pos,
    const ImVec2& area_size,
    const ImVec2& game_pos,
    const ImVec2& game_size,
    float bottom_overscan_v) {
    (void)bottom_overscan_v;

    if (draw == nullptr ||
        area_size.x <= 0.0f ||
        area_size.y <= 0.0f) {
        return;
    }

    const ImVec2 area_end(
        area_pos.x + area_size.x,
        area_pos.y + area_size.y);

    // Ambient light sits on a neutral near-black wall. It intentionally
    // ignores Customize's gameplay background color.
    draw->AddRectFilled(
        area_pos,
        area_end,
        IM_COL32(3, 4, 6, 255));

    if (!g_ambient_initialized) {
        return;
    }

    const float game_right =
        game_pos.x + game_size.x;
    const float game_bottom =
        game_pos.y + game_size.y;

    if (game_pos.x - area_pos.x > 0.5f) {
        draw_vertical_ambilight(
            draw,
            g_left_colors,
            area_pos.x,
            game_pos.x,
            game_pos.y,
            game_size.y,
            true);
    }

    if (area_end.x - game_right > 0.5f) {
        draw_vertical_ambilight(
            draw,
            g_right_colors,
            area_end.x,
            game_right,
            game_pos.y,
            game_size.y,
            false);
    }

    if (game_pos.y - area_pos.y > 0.5f) {
        draw_horizontal_ambilight(
            draw,
            g_top_colors,
            game_pos.x,
            game_size.x,
            area_pos.y,
            game_pos.y,
            true);
    }

    if (area_end.y - game_bottom > 0.5f) {
        draw_horizontal_ambilight(
            draw,
            g_bottom_colors,
            game_pos.x,
            game_size.x,
            area_end.y,
            game_bottom,
            false);
    }
}

void definitive_ui::release_gameplay_ambient_assets() {
    g_left_colors = {};
    g_right_colors = {};
    g_top_colors = {};
    g_bottom_colors = {};
    g_ambient_initialized = false;
}
