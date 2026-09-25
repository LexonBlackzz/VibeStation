#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#include <stb_image.h>

#include "ui/definitive/definitive_shared.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <filesystem>
#include <vector>

namespace {

GLuint g_background_texture = 0;
GLuint g_background_soft_texture = 0;
GLuint g_background_blur_texture = 0;
int g_background_width = 0;
int g_background_height = 0;
bool g_background_load_attempted = false;

struct CoverUv {
    float u0 = 0.0f;
    float v0 = 0.0f;
    float u1 = 1.0f;
    float v1 = 1.0f;
};

std::vector<unsigned char> make_blurred_definitive_ui::rgba(
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
                blur_mix = kBlurStrength * (1.0f - definitive_ui::smoothstep01(fade_t));
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
        make_blurred_definitive_ui::rgba(pixels, g_background_width, g_background_height, 7);
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

} // namespace

void definitive_ui::preload_background_assets() {
    ensure_background_texture_loaded();
}

void definitive_ui::release_background_assets() {
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
}

void definitive_ui::draw_launcher_background(
    ImDrawList* draw, const ImVec2& pos, const ImVec2& size,
    float opacity) {
    const float alpha = std::clamp(opacity, 0.0f, 1.0f);

    // Pure black is the transition canvas. The photo is composited over it
    // only after the launcher controls have finished their own reveal.
    draw->AddRectFilled(
        pos, ImVec2(pos.x + size.x, pos.y + size.y),
        definitive_ui::rgba(0, 0, 0, 255));

    if (alpha <= 0.001f) {
        return;
    }

    if (!ensure_background_texture_loaded()) {
        draw->AddRectFilledMultiColor(
            pos, ImVec2(pos.x + size.x, pos.y + size.y),
            definitive_ui::background_color(
                definitive_ui::rgba(8, 10, 14, definitive_ui::glow_alpha(255.0f * alpha)), 0.94f),
            definitive_ui::background_color(
                definitive_ui::rgba(17, 19, 23, definitive_ui::glow_alpha(255.0f * alpha)), 0.90f),
            definitive_ui::background_color(
                definitive_ui::rgba(10, 12, 15, definitive_ui::glow_alpha(255.0f * alpha)), 0.92f),
            definitive_ui::background_color(
                definitive_ui::rgba(5, 7, 10, definitive_ui::glow_alpha(255.0f * alpha)), 0.96f));
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
        ImVec2(uv.u0, uv.v0), ImVec2(uv.u1, uv.v1),
        definitive_ui::rgba(255, 255, 255, definitive_ui::glow_alpha(255.0f * alpha)));

    if (definitive_ui::theme_active()) {
        ImVec4 tint = ui_theme::g_theme_settings.background;
        tint.w = std::clamp(0.34f * alpha, 0.0f, 0.34f);
        draw->AddRectFilled(
            pos, ImVec2(pos.x + size.x, pos.y + size.y),
            ImGui::ColorConvertFloat4ToU32(tint));
    }
}

void definitive_ui::draw_launcher_readability_shade(ImDrawList* draw,
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
        return 1.0f - definitive_ui::smoothstep01(t);
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
            definitive_ui::background_color(
                definitive_ui::rgba(0, 2, 5, definitive_ui::glow_alpha(kTopAlpha * s0)), 0.72f),
            definitive_ui::background_color(
                definitive_ui::rgba(0, 2, 5, definitive_ui::glow_alpha(kTopAlpha * s1)), 0.72f),
            definitive_ui::background_color(
                definitive_ui::rgba(0, 2, 5, definitive_ui::glow_alpha(kBottomAlpha * s1)), 0.72f),
            definitive_ui::background_color(
                definitive_ui::rgba(0, 2, 5, definitive_ui::glow_alpha(kBottomAlpha * s0)), 0.72f));
    }
}
