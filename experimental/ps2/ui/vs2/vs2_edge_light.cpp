#include "ui/vs2/vs2_edge_light.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ps2::ui::vs2 {

namespace {

ImVec4 sample_region(const std::uint32_t* rgba, int width, int height,
                     int x0, int y0, int x1, int y1) {
    x0 = std::clamp(x0, 0, width);
    x1 = std::clamp(x1, 0, width);
    y0 = std::clamp(y0, 0, height);
    y1 = std::clamp(y1, 0, height);
    if (x1 <= x0 || y1 <= y0) return ImVec4(0, 0, 0, 1);

    // A sparse grid keeps this cheap at any output resolution.
    const int step_x = std::max(1, (x1 - x0) / 10);
    const int step_y = std::max(1, (y1 - y0) / 14);
    double red = 0.0, green = 0.0, blue = 0.0;
    int count = 0;
    for (int y = y0; y < y1; y += step_y) {
        for (int x = x0; x < x1; x += step_x) {
            const std::uint32_t pixel =
                rgba[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                     static_cast<std::size_t>(x)];
            red += static_cast<double>(pixel & 0xFFu);
            green += static_cast<double>((pixel >> 8u) & 0xFFu);
            blue += static_cast<double>((pixel >> 16u) & 0xFFu);
            ++count;
        }
    }
    if (count == 0) return ImVec4(0, 0, 0, 1);

    float r = static_cast<float>(red / count / 255.0);
    float g = static_cast<float>(green / count / 255.0);
    float b = static_cast<float>(blue / count / 255.0);

    // Read as emitted light rather than a copied image: a little more
    // saturation, lifted mid-tones, dark scenes left dark.
    const float luminance = r * 0.2126f + g * 0.7152f + b * 0.0722f;
    constexpr float kSaturation = 1.30f;
    r = luminance + (r - luminance) * kSaturation;
    g = luminance + (g - luminance) * kSaturation;
    b = luminance + (b - luminance) * kSaturation;
    const float brightness = 0.82f + 0.30f * std::sqrt(std::clamp(luminance, 0.0f, 1.0f));
    return ImVec4(std::clamp(r * brightness, 0.0f, 1.0f),
                  std::clamp(g * brightness, 0.0f, 1.0f),
                  std::clamp(b * brightness, 0.0f, 1.0f), 1.0f);
}

template <std::size_t N>
void spatial_smooth(std::array<ImVec4, N>& colors) {
    const std::array<ImVec4, N> source = colors;
    for (std::size_t i = 0; i < N; ++i) {
        const ImVec4& prev = source[i > 0 ? i - 1 : i];
        const ImVec4& next = source[i + 1 < N ? i + 1 : i];
        colors[i] = ImVec4(prev.x * 0.22f + source[i].x * 0.56f + next.x * 0.22f,
                           prev.y * 0.22f + source[i].y * 0.56f + next.y * 0.22f,
                           prev.z * 0.22f + source[i].z * 0.56f + next.z * 0.22f, 1.0f);
    }
}

template <std::size_t N>
void temporal_smooth(std::array<ImVec4, N>& current, const std::array<ImVec4, N>& target,
                     float response) {
    for (std::size_t i = 0; i < N; ++i) {
        current[i].x += (target[i].x - current[i].x) * response;
        current[i].y += (target[i].y - current[i].y) * response;
        current[i].z += (target[i].z - current[i].z) * response;
        current[i].w = 1.0f;
    }
}

ImU32 light(const ImVec4& color, float alpha, float strength) {
    const auto channel = [strength](float v) {
        return std::clamp(static_cast<int>(std::round(v * 255.0f * strength)), 0, 255);
    };
    return IM_COL32(channel(color.x), channel(color.y), channel(color.z),
                    std::clamp(static_cast<int>(std::round(alpha)), 0, 255));
}

template <std::size_t N>
ImVec4 boundary_color(const std::array<ImVec4, N>& colors, std::size_t boundary) {
    if (boundary == 0) return colors.front();
    if (boundary >= N) return colors.back();
    const ImVec4& a = colors[boundary - 1];
    const ImVec4& b = colors[boundary];
    return ImVec4((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f, (a.z + b.z) * 0.5f, 1.0f);
}

// One side strip: a wide falloff plus a concentrated bloom by the picture.
template <std::size_t N>
void draw_vertical(ImDrawList* draw, const std::array<ImVec4, N>& colors, float outer_x,
                   float inner_x, float top_y, float height, bool inner_on_right, float alpha) {
    const float width = std::abs(inner_x - outer_x);
    if (width < 0.5f || height <= 0.5f) return;
    const float bloom_width = std::min(width, std::max(18.0f, width * 0.34f));

    for (std::size_t i = 0; i < N; ++i) {
        // Segments meet exactly; overlapping them stacks alpha into seams.
        const float y0 = top_y + height * static_cast<float>(i) / N;
        const float y1 = top_y + height * static_cast<float>(i + 1) / N;
        const ImVec4 c0 = boundary_color(colors, i);
        const ImVec4 c1 = boundary_color(colors, i + 1);

        const ImU32 in0 = light(c0, 208 * alpha, 1.06f), in1 = light(c1, 208 * alpha, 1.06f);
        const ImU32 out0 = light(c0, 10 * alpha, 0.48f), out1 = light(c1, 10 * alpha, 0.48f);
        const ImVec2 p0(std::min(outer_x, inner_x), y0), p1(std::max(outer_x, inner_x), y1);
        if (inner_on_right) draw->AddRectFilledMultiColor(p0, p1, out0, in0, in1, out1);
        else draw->AddRectFilledMultiColor(p0, p1, in0, out0, out1, in1);

        const float bloom_x = inner_on_right ? inner_x - bloom_width : inner_x + bloom_width;
        const ImU32 edge0 = light(c0, 108 * alpha, 1.12f), edge1 = light(c1, 108 * alpha, 1.12f);
        const ImU32 fade0 = light(c0, 0, 0.85f), fade1 = light(c1, 0, 0.85f);
        const ImVec2 b0(std::min(bloom_x, inner_x), y0), b1(std::max(bloom_x, inner_x), y1);
        if (inner_on_right) draw->AddRectFilledMultiColor(b0, b1, fade0, edge0, edge1, fade1);
        else draw->AddRectFilledMultiColor(b0, b1, edge0, fade0, fade1, edge1);
    }
}

template <std::size_t N>
void draw_horizontal(ImDrawList* draw, const std::array<ImVec4, N>& colors, float left_x,
                     float width, float outer_y, float inner_y, bool inner_on_bottom, float alpha) {
    if (std::abs(inner_y - outer_y) < 0.5f || width <= 0.5f) return;
    for (std::size_t i = 0; i < N; ++i) {
        const float x0 = left_x + width * static_cast<float>(i) / N;
        const float x1 = left_x + width * static_cast<float>(i + 1) / N;
        const ImVec4 c0 = boundary_color(colors, i);
        const ImVec4 c1 = boundary_color(colors, i + 1);
        const ImU32 in0 = light(c0, 196 * alpha, 1.05f), in1 = light(c1, 196 * alpha, 1.05f);
        const ImU32 out0 = light(c0, 8 * alpha, 0.46f), out1 = light(c1, 8 * alpha, 0.46f);
        const ImVec2 p0(x0, std::min(outer_y, inner_y)), p1(x1, std::max(outer_y, inner_y));
        if (inner_on_bottom) draw->AddRectFilledMultiColor(p0, p1, out0, out1, in1, in0);
        else draw->AddRectFilledMultiColor(p0, p1, in0, in1, out1, out0);
    }
}

} // namespace

void EdgeLight::update(const std::uint32_t* rgba, int width, int height) {
    if (rgba == nullptr || width <= 0 || height <= 0) return;

    Vertical left{}, right{};
    Horizontal top{}, bottom{};
    const int band_x = std::max(2, width / 12);
    const int band_y = std::max(2, height / 12);
    for (int i = 0; i < kVertical; ++i) {
        const int y0 = height * i / kVertical, y1 = height * (i + 1) / kVertical;
        left[static_cast<std::size_t>(i)] = sample_region(rgba, width, height, 0, y0, band_x, y1);
        right[static_cast<std::size_t>(i)] =
            sample_region(rgba, width, height, width - band_x, y0, width, y1);
    }
    for (int i = 0; i < kHorizontal; ++i) {
        const int x0 = width * i / kHorizontal, x1 = width * (i + 1) / kHorizontal;
        top[static_cast<std::size_t>(i)] = sample_region(rgba, width, height, x0, 0, x1, band_y);
        bottom[static_cast<std::size_t>(i)] =
            sample_region(rgba, width, height, x0, height - band_y, x1, height);
    }
    for (int pass = 0; pass < 2; ++pass) {
        spatial_smooth(left);
        spatial_smooth(right);
        spatial_smooth(top);
        spatial_smooth(bottom);
    }

    if (!initialized_) {
        left_ = left;
        right_ = right;
        top_ = top;
        bottom_ = bottom;
        initialized_ = true;
        return;
    }
    // Some persistence, like a real backlight, and no flicker.
    constexpr float kResponse = 0.16f;
    temporal_smooth(left_, left, kResponse);
    temporal_smooth(right_, right, kResponse);
    temporal_smooth(top_, top, kResponse);
    temporal_smooth(bottom_, bottom, kResponse);
}

void EdgeLight::draw(ImDrawList* draw, const ImVec2& area_pos, const ImVec2& area_size,
                     const ImVec2& game_pos, const ImVec2& game_size, float alpha) const {
    if (draw == nullptr || area_size.x <= 0.0f || area_size.y <= 0.0f || alpha <= 0.001f) return;
    const ImVec2 area_end(area_pos.x + area_size.x, area_pos.y + area_size.y);
    draw->AddRectFilled(area_pos, area_end, IM_COL32(3, 4, 6, static_cast<int>(255 * alpha)));
    if (!initialized_) return;

    const float game_right = game_pos.x + game_size.x;
    const float game_bottom = game_pos.y + game_size.y;
    if (game_pos.x - area_pos.x > 0.5f)
        draw_vertical(draw, left_, area_pos.x, game_pos.x, game_pos.y, game_size.y, true, alpha);
    if (area_end.x - game_right > 0.5f)
        draw_vertical(draw, right_, area_end.x, game_right, game_pos.y, game_size.y, false, alpha);
    if (game_pos.y - area_pos.y > 0.5f)
        draw_horizontal(draw, top_, game_pos.x, game_size.x, area_pos.y, game_pos.y, true, alpha);
    if (area_end.y - game_bottom > 0.5f)
        draw_horizontal(draw, bottom_, game_pos.x, game_size.x, area_end.y, game_bottom, false, alpha);
}

} // namespace ps2::ui::vs2
