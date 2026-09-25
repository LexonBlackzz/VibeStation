#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#include <stb_image.h>

#include "ui/app.h"
#include "ui/definitive/definitive_shared.h"
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
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace {
constexpr float kDesignWidth = 1280.0f;
constexpr float kDesignHeight = 800.0f;

GLuint g_background_texture = 0;
GLuint g_background_soft_texture = 0;
GLuint g_background_blur_texture = 0;
int g_background_width = 0;
int g_background_height = 0;
bool g_background_load_attempted = false;

enum class UiMenuSound {
    Cursor,
    Open,
    Close
};

struct UiSoundClip {
    std::vector<Uint8> pcm;
};

SDL_AudioDeviceID g_ui_sound_device = 0;
SDL_AudioSpec g_ui_sound_spec{};
bool g_ui_sound_load_attempted = false;
Uint32 g_ui_action_sound_until_ms = 0;
UiSoundClip g_ui_cursor_sound;
UiSoundClip g_ui_open_sound;
UiSoundClip g_ui_close_sound;
UiSoundClip g_ui_logo_sound;
bool g_launcher_startup_sound_played = false;
std::array<bool, 5> g_menu_was_engaged = {};

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
float g_launcher_ui_intro_elapsed = 0.0f;
bool g_launcher_ui_intro_complete = false;
float g_launcher_background_fade_elapsed = 0.0f;
bool g_launcher_background_fade_complete = false;

// This animation is deliberately independent from the boot timer. It begins
// on the frame after the boot sequence ends so the launcher never initializes
// invisibly behind the startup logo.
constexpr float kLauncherUiIntroDuration = 1.68f;
constexpr float kLauncherBackgroundFadeDuration = 0.72f;


ImU32 rgba(int r, int g, int b, int a = 255) {
    return IM_COL32(r, g, b, a);
}

bool definitive_theme_color_differs(
    const ImVec4& a, const ImVec4& b) {
    constexpr float kEpsilon = 0.0025f;
    return std::fabs(a.x - b.x) > kEpsilon ||
        std::fabs(a.y - b.y) > kEpsilon ||
        std::fabs(a.z - b.z) > kEpsilon ||
        std::fabs(a.w - b.w) > kEpsilon;
}

bool definitive_theme_active() {
    const ui_theme::ThemeSettings& theme = ui_theme::g_theme_settings;
    return definitive_theme_color_differs(
               theme.background, ui_theme::kDefaultThemeBackground) ||
        definitive_theme_color_differs(
               theme.surface, ui_theme::kDefaultThemeSurface) ||
        definitive_theme_color_differs(
               theme.accent, ui_theme::kDefaultThemeAccent) ||
        definitive_theme_color_differs(
               theme.text, ui_theme::kDefaultThemeText) ||
        definitive_theme_color_differs(
               theme.lists, ui_theme::kDefaultThemeLists);
}

ImU32 definitive_mix_theme_color(
    ImU32 fallback, const ImVec4& theme_color, float mix) {
    if (!definitive_theme_active()) {
        return fallback;
    }

    const ImVec4 base = ImGui::ColorConvertU32ToFloat4(fallback);
    ImVec4 out = ui_theme::theme_lerp(
        base, theme_color, std::clamp(mix, 0.0f, 1.0f));
    // Preserve the role-specific opacity from the original Definitive color.
    out.w = base.w;
    return ImGui::ColorConvertFloat4ToU32(out);
}

ImU32 definitive_text_color(ImU32 fallback) {
    if (!definitive_theme_active()) {
        return fallback;
    }

    const ImVec4 base = ImGui::ColorConvertU32ToFloat4(fallback);
    const float max_rgb = std::max(base.x, std::max(base.y, base.z));
    const float min_rgb = std::min(base.x, std::min(base.y, base.z));

    // Keep deliberate semantic colors such as warnings and the four-color
    // VibeStation brand accents intact. Neutral UI text follows Text.
    if ((max_rgb - min_rgb) > 0.22f) {
        return fallback;
    }

    return definitive_mix_theme_color(
        fallback, ui_theme::g_theme_settings.text, 0.92f);
}

ImU32 definitive_surface_color(ImU32 fallback, float mix = 0.82f) {
    return definitive_mix_theme_color(
        fallback, ui_theme::g_theme_settings.surface, mix);
}

ImU32 definitive_list_color(ImU32 fallback, float mix = 0.84f) {
    return definitive_mix_theme_color(
        fallback, ui_theme::g_theme_settings.lists, mix);
}

ImU32 definitive_accent_color(ImU32 fallback, float mix = 0.92f) {
    return definitive_mix_theme_color(
        fallback, ui_theme::g_theme_settings.accent, mix);
}

ImU32 definitive_background_color(ImU32 fallback, float mix = 0.82f) {
    return definitive_mix_theme_color(
        fallback, ui_theme::g_theme_settings.background, mix);
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

std::filesystem::path find_ui_sound_path(const char* filename) {
    std::array<std::filesystem::path, 4> candidates{};

    std::error_code ec;
    const std::filesystem::path cwd = std::filesystem::current_path(ec);
    if (!ec) {
        candidates[0] =
            cwd / "resources" / "ui" / "definitive" / "sounds" / filename;
        candidates[1] =
            cwd / ".." / "resources" / "ui" / "definitive" / "sounds" / filename;
    }

    if (char* base = SDL_GetBasePath()) {
        const std::filesystem::path base_path(base);
        SDL_free(base);
        candidates[2] =
            base_path / "resources" / "ui" / "definitive" / "sounds" / filename;
        candidates[3] =
            base_path / ".." / "resources" / "ui" / "definitive" / "sounds" / filename;
    }

    for (const auto& candidate : candidates) {
        if (!candidate.empty() && std::filesystem::exists(candidate, ec) && !ec) {
            return candidate;
        }
        ec.clear();
    }
    return {};
}

bool convert_ui_sound(
    const std::filesystem::path& path,
    const SDL_AudioSpec& target_spec,
    UiSoundClip& out_clip) {
    SDL_AudioSpec source_spec{};
    Uint8* source_buffer = nullptr;
    Uint32 source_length = 0;

    if (path.empty() ||
        SDL_LoadWAV(
            path.string().c_str(),
            &source_spec,
            &source_buffer,
            &source_length) == nullptr) {
        return false;
    }

    SDL_AudioCVT cvt{};
    const int cvt_result = SDL_BuildAudioCVT(
        &cvt,
        source_spec.format,
        source_spec.channels,
        source_spec.freq,
        target_spec.format,
        target_spec.channels,
        target_spec.freq);

    if (cvt_result < 0) {
        SDL_FreeWAV(source_buffer);
        return false;
    }

    if (cvt_result == 0) {
        out_clip.pcm.assign(source_buffer, source_buffer + source_length);
        SDL_FreeWAV(source_buffer);
        return true;
    }

    cvt.len = static_cast<int>(source_length);
    cvt.buf = static_cast<Uint8*>(
        SDL_malloc(static_cast<size_t>(source_length) *
            static_cast<size_t>(cvt.len_mult)));
    if (cvt.buf == nullptr) {
        SDL_FreeWAV(source_buffer);
        return false;
    }

    std::memcpy(cvt.buf, source_buffer, source_length);
    SDL_FreeWAV(source_buffer);

    if (SDL_ConvertAudio(&cvt) != 0) {
        SDL_free(cvt.buf);
        return false;
    }

    out_clip.pcm.assign(cvt.buf, cvt.buf + cvt.len_cvt);
    SDL_free(cvt.buf);
    return true;
}

void apply_ui_sound_fade_in(
    UiSoundClip& clip, const SDL_AudioSpec& spec, float fade_ms) {
    if (clip.pcm.empty() || spec.freq <= 0 || spec.channels == 0 ||
        fade_ms <= 0.0f) {
        return;
    }

    const int bits_per_sample = SDL_AUDIO_BITSIZE(spec.format);
    if (bits_per_sample <= 0 || (bits_per_sample % 8) != 0) {
        return;
    }

    const size_t bytes_per_sample =
        static_cast<size_t>(bits_per_sample / 8);
    const size_t bytes_per_frame =
        bytes_per_sample * static_cast<size_t>(spec.channels);
    if (bytes_per_frame == 0) {
        return;
    }

    const size_t frame_count = clip.pcm.size() / bytes_per_frame;
    const size_t requested_fade_frames = static_cast<size_t>(
        std::max(1.0f,
            static_cast<float>(spec.freq) * fade_ms / 1000.0f));
    const size_t fade_frames =
        std::min(frame_count, requested_fade_frames);
    if (fade_frames == 0) {
        return;
    }

    const size_t sample_count =
        fade_frames * static_cast<size_t>(spec.channels);

    switch (spec.format) {
    case AUDIO_F32SYS: {
        float* samples =
            reinterpret_cast<float*>(clip.pcm.data());
        for (size_t i = 0; i < sample_count; ++i) {
            const size_t frame =
                i / static_cast<size_t>(spec.channels);
            const float gain =
                static_cast<float>(frame + 1) /
                static_cast<float>(fade_frames);
            samples[i] *= gain;
        }
        break;
    }
    case AUDIO_S16SYS: {
        Sint16* samples =
            reinterpret_cast<Sint16*>(clip.pcm.data());
        for (size_t i = 0; i < sample_count; ++i) {
            const size_t frame =
                i / static_cast<size_t>(spec.channels);
            const float gain =
                static_cast<float>(frame + 1) /
                static_cast<float>(fade_frames);
            samples[i] = static_cast<Sint16>(
                static_cast<float>(samples[i]) * gain);
        }
        break;
    }
    case AUDIO_S32SYS: {
        Sint32* samples =
            reinterpret_cast<Sint32*>(clip.pcm.data());
        for (size_t i = 0; i < sample_count; ++i) {
            const size_t frame =
                i / static_cast<size_t>(spec.channels);
            const float gain =
                static_cast<float>(frame + 1) /
                static_cast<float>(fade_frames);
            samples[i] = static_cast<Sint32>(
                static_cast<double>(samples[i]) *
                static_cast<double>(gain));
        }
        break;
    }
    case AUDIO_S8: {
        Sint8* samples =
            reinterpret_cast<Sint8*>(clip.pcm.data());
        for (size_t i = 0; i < sample_count; ++i) {
            const size_t frame =
                i / static_cast<size_t>(spec.channels);
            const float gain =
                static_cast<float>(frame + 1) /
                static_cast<float>(fade_frames);
            samples[i] = static_cast<Sint8>(
                static_cast<float>(samples[i]) * gain);
        }
        break;
    }
    case AUDIO_U8: {
        Uint8* samples = clip.pcm.data();
        for (size_t i = 0; i < sample_count; ++i) {
            const size_t frame =
                i / static_cast<size_t>(spec.channels);
            const float gain =
                static_cast<float>(frame + 1) /
                static_cast<float>(fade_frames);
            const float centered =
                static_cast<float>(samples[i]) - 128.0f;
            samples[i] = static_cast<Uint8>(std::clamp(
                128.0f + centered * gain, 0.0f, 255.0f));
        }
        break;
    }
    default:
        // The requested device format is float32, so this is only a fallback
        // for an unusual backend format we do not need to modify.
        break;
    }
}

bool ensure_ui_sounds_loaded() {
    if (g_ui_sound_device != 0) {
        return true;
    }
    if (g_ui_sound_load_attempted) {
        return false;
    }
    g_ui_sound_load_attempted = true;

    const std::filesystem::path cursor_path =
        find_ui_sound_path("cursor.wav");
    const std::filesystem::path open_path =
        find_ui_sound_path("open.wav");
    const std::filesystem::path close_path =
        find_ui_sound_path("close.wav");
    const std::filesystem::path logo_path =
        find_ui_sound_path("logo.wav");

    if (cursor_path.empty() || open_path.empty() || close_path.empty() ||
        logo_path.empty()) {
        return false;
    }

    SDL_AudioSpec desired{};
    desired.freq = 44100;
    desired.format = AUDIO_F32SYS;
    desired.channels = 2;
    desired.samples = 512;
    desired.callback = nullptr;

    g_ui_sound_device = SDL_OpenAudioDevice(
        nullptr,
        0,
        &desired,
        &g_ui_sound_spec,
        SDL_AUDIO_ALLOW_ANY_CHANGE);
    if (g_ui_sound_device == 0) {
        return false;
    }

    const bool loaded =
        convert_ui_sound(cursor_path, g_ui_sound_spec, g_ui_cursor_sound) &&
        convert_ui_sound(open_path, g_ui_sound_spec, g_ui_open_sound) &&
        convert_ui_sound(close_path, g_ui_sound_spec, g_ui_close_sound) &&
        convert_ui_sound(logo_path, g_ui_sound_spec, g_ui_logo_sound);

    if (loaded) {
        // A tiny attack ramp removes the transient click when cursor.wav is
        // rapidly restarted while moving through menu items.
        apply_ui_sound_fade_in(
            g_ui_cursor_sound, g_ui_sound_spec, 15.0f);
    }

    if (!loaded) {
        SDL_CloseAudioDevice(g_ui_sound_device);
        g_ui_sound_device = 0;
        g_ui_cursor_sound.pcm.clear();
        g_ui_open_sound.pcm.clear();
        g_ui_close_sound.pcm.clear();
        g_ui_logo_sound.pcm.clear();
        return false;
    }

    SDL_PauseAudioDevice(g_ui_sound_device, 0);
    return true;
}

Uint32 ui_sound_duration_ms(const UiSoundClip& clip) {
    const int bits_per_sample =
        SDL_AUDIO_BITSIZE(g_ui_sound_spec.format);
    const Uint32 bytes_per_second =
        (g_ui_sound_spec.freq > 0 &&
         g_ui_sound_spec.channels > 0 &&
         bits_per_sample > 0)
            ? static_cast<Uint32>(
                g_ui_sound_spec.freq *
                g_ui_sound_spec.channels *
                (bits_per_sample / 8))
            : 0u;

    if (bytes_per_second == 0 || clip.pcm.empty()) {
        return 0;
    }

    return static_cast<Uint32>(
        (static_cast<Uint64>(clip.pcm.size()) * 1000u) /
        bytes_per_second);
}

void play_startup_logo_sound() {
    if (g_launcher_startup_sound_played ||
        !ensure_ui_sounds_loaded() ||
        g_ui_sound_device == 0 ||
        g_ui_logo_sound.pcm.empty()) {
        return;
    }

    // The startup sound owns the UI audio device until it has naturally
    // finished. Cursor highlights remain suppressed during this interval.
    SDL_ClearQueuedAudio(g_ui_sound_device);
    if (SDL_QueueAudio(
            g_ui_sound_device,
            g_ui_logo_sound.pcm.data(),
            static_cast<Uint32>(g_ui_logo_sound.pcm.size())) == 0) {
        g_launcher_startup_sound_played = true;
        const Uint32 duration_ms =
            std::max<Uint32>(
                ui_sound_duration_ms(g_ui_logo_sound), 40u);
        g_ui_action_sound_until_ms =
            SDL_GetTicks() + duration_ms;
    }
}

void stop_startup_logo_sound() {
    if (!g_launcher_startup_sound_played ||
        g_ui_sound_device == 0) {
        return;
    }

    SDL_ClearQueuedAudio(g_ui_sound_device);
    g_ui_action_sound_until_ms = 0;
}

void play_menu_sound(UiMenuSound sound) {
    if (!ensure_ui_sounds_loaded() || g_ui_sound_device == 0) {
        return;
    }

    const UiSoundClip* clip = nullptr;
    switch (sound) {
    case UiMenuSound::Cursor:
        clip = &g_ui_cursor_sound;
        break;
    case UiMenuSound::Open:
        clip = &g_ui_open_sound;
        break;
    case UiMenuSound::Close:
        clip = &g_ui_close_sound;
        break;
    }

    if (clip == nullptr || clip->pcm.empty()) {
        return;
    }

    const Uint32 now = SDL_GetTicks();
    const bool action_sound_active =
        g_ui_action_sound_until_ms != 0 &&
        !SDL_TICKS_PASSED(now, g_ui_action_sound_until_ms);

    if (sound == UiMenuSound::Cursor) {
        // While an explicit open/close effect is playing, automatic focus
        // changes must not interrupt it. Otherwise cursor ticks are retriggered
        // immediately so fast navigation feels responsive instead of queued.
        if (action_sound_active) {
            return;
        }
        SDL_ClearQueuedAudio(g_ui_sound_device);
    }
    else {
        // Open/close effects take priority over cursor ticks and protect their
        // own playback window from subsequent automatic highlight sounds.
        SDL_ClearQueuedAudio(g_ui_sound_device);

        const Uint32 duration_ms =
            std::max<Uint32>(
                ui_sound_duration_ms(*clip), 40u);
        g_ui_action_sound_until_ms =
            now + std::max<Uint32>(duration_ms, 40u);
    }

    SDL_QueueAudio(
        g_ui_sound_device,
        clip->pcm.data(),
        static_cast<Uint32>(clip->pcm.size()));
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

void draw_background(
    ImDrawList* draw, const ImVec2& pos, const ImVec2& size,
    float opacity = 1.0f) {
    const float alpha = std::clamp(opacity, 0.0f, 1.0f);

    // Pure black is the transition canvas. The photo is composited over it
    // only after the launcher controls have finished their own reveal.
    draw->AddRectFilled(
        pos, ImVec2(pos.x + size.x, pos.y + size.y),
        rgba(0, 0, 0, 255));

    if (alpha <= 0.001f) {
        return;
    }

    if (!ensure_background_texture_loaded()) {
        draw->AddRectFilledMultiColor(
            pos, ImVec2(pos.x + size.x, pos.y + size.y),
            definitive_background_color(
                rgba(8, 10, 14, glow_alpha(255.0f * alpha)), 0.94f),
            definitive_background_color(
                rgba(17, 19, 23, glow_alpha(255.0f * alpha)), 0.90f),
            definitive_background_color(
                rgba(10, 12, 15, glow_alpha(255.0f * alpha)), 0.92f),
            definitive_background_color(
                rgba(5, 7, 10, glow_alpha(255.0f * alpha)), 0.96f));
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
        rgba(255, 255, 255, glow_alpha(255.0f * alpha)));

    if (definitive_theme_active()) {
        ImVec4 tint = ui_theme::g_theme_settings.background;
        tint.w = std::clamp(0.34f * alpha, 0.0f, 0.34f);
        draw->AddRectFilled(
            pos, ImVec2(pos.x + size.x, pos.y + size.y),
            ImGui::ColorConvertFloat4ToU32(tint));
    }
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
            definitive_background_color(
                rgba(0, 2, 5, glow_alpha(kTopAlpha * s0)), 0.72f),
            definitive_background_color(
                rgba(0, 2, 5, glow_alpha(kTopAlpha * s1)), 0.72f),
            definitive_background_color(
                rgba(0, 2, 5, glow_alpha(kBottomAlpha * s1)), 0.72f),
            definitive_background_color(
                rgba(0, 2, 5, glow_alpha(kBottomAlpha * s0)), 0.72f));
    }
}


void draw_launcher_initialization_overlay(
    const ImVec2& pos, const ImVec2& size, float elapsed) {
    ImDrawList* overlay = ImGui::GetForegroundDrawList();
    const Layout layout = make_layout(pos, size);

    // The first launcher frame starts fully black, then the entire UI fades
    // into view. This is separate from the individual brand/menu/panel wipes,
    // so nothing can pop in abruptly on the frame after the boot intro ends.
    const float ui_fade =
        timeline_progress(elapsed, 0.00f, 0.62f);
    const int ui_black_alpha =
        glow_alpha(255.0f * (1.0f - ui_fade));
    if (ui_black_alpha > 0) {
        overlay->AddRectFilled(
            pos,
            ImVec2(pos.x + size.x, pos.y + size.y),
            rgba(0, 0, 0, ui_black_alpha));
    }

    // UI pieces assemble over a fully black background. The photograph fades
    // in only after every launcher element has completed this animation.
    const float brand_reveal =
        timeline_progress(elapsed, 0.10f, 0.68f);
    const ImVec2 brand0 = layout.point(24.0f, 18.0f);
    const ImVec2 brand1 = layout.point(460.0f, 156.0f);
    if (brand_reveal < 1.0f) {
        const float wipe_y =
            brand1.y - (brand1.y - brand0.y) * brand_reveal;
        overlay->AddRectFilled(
            brand0,
            ImVec2(brand1.x, wipe_y),
            rgba(0, 0, 0, 255));
        overlay->AddLine(
            ImVec2(brand0.x, wipe_y),
            ImVec2(brand1.x, wipe_y),
            rgba(185, 210, 232,
                glow_alpha(92.0f * brand_reveal)),
            layout.px(1.0f));
    }

    // Version/quote block trails the brand slightly.
    const float meta_reveal =
        timeline_progress(elapsed, 0.24f, 0.72f);
    if (meta_reveal < 1.0f) {
        overlay->AddRectFilled(
            layout.point(1010.0f, 18.0f),
            layout.point(1252.0f, 118.0f),
            rgba(0, 0, 0,
                glow_alpha(255.0f * (1.0f - meta_reveal))));
    }

    // Main actions appear one-by-one from top to bottom.
    constexpr float kMenuX = 30.0f;
    constexpr float kMenuY = 212.0f;
    constexpr float kMenuW = 410.0f;
    constexpr float kMenuH = 67.0f;
    constexpr float kMenuStep = 70.0f;

    for (int i = 0; i < 5; ++i) {
        const float start =
            0.34f + static_cast<float>(i) * 0.09f;
        const float reveal =
            timeline_progress(elapsed, start, start + 0.42f);

        const ImVec2 row0 =
            layout.point(kMenuX, kMenuY + kMenuStep * i);
        const ImVec2 row1 =
            layout.point(
                kMenuX + kMenuW,
                kMenuY + kMenuStep * i + kMenuH);

        if (reveal < 1.0f) {
            const float wipe_x =
                row0.x + (row1.x - row0.x) * reveal;

            overlay->AddRectFilled(
                ImVec2(wipe_x, row0.y),
                row1,
                rgba(0, 0, 0, 255));

            const int veil =
                glow_alpha(150.0f * (1.0f - reveal));
            if (veil > 0) {
                overlay->AddRectFilled(
                    row0,
                    ImVec2(wipe_x, row1.y),
                    rgba(0, 0, 0, veil));
            }
        }
    }

    // Bottom cards arrive last, with the system card just behind the library.
    const std::array<ImVec4, 2> panels = {{
        ImVec4(32.0f, 585.0f, 840.0f, 768.0f),
        ImVec4(854.0f, 585.0f, 1248.0f, 768.0f),
    }};
    for (size_t i = 0; i < panels.size(); ++i) {
        const float start =
            0.88f + static_cast<float>(i) * 0.10f;
        const float reveal =
            timeline_progress(elapsed, start, start + 0.46f);
        if (reveal >= 1.0f) {
            continue;
        }

        const ImVec4& p = panels[i];
        const ImVec2 p0 = layout.point(p.x, p.y);
        const ImVec2 p1 = layout.point(p.z, p.w);
        const float wipe_y =
            p1.y - (p1.y - p0.y) * reveal;

        overlay->AddRectFilled(
            p0,
            ImVec2(p1.x, wipe_y),
            rgba(0, 0, 0, 255));
        overlay->AddRectFilled(
            ImVec2(p0.x, wipe_y),
            p1,
            rgba(0, 0, 0,
                glow_alpha(145.0f * (1.0f - reveal))));
    }
}


void add_text(ImDrawList* draw, const Layout& layout, float x, float y,
    float size, ImU32 color, const char* text) {
    const float font_size = layout.px(size);
    ImFont* font = definitive_ui::font_for_size(font_size);
    draw->AddText(
        font, font_size, layout.point(x, y),
        definitive_text_color(color), text);
}

void add_text_right(ImDrawList* draw, const Layout& layout, float right_x, float y,
    float size, ImU32 color, const char* text) {
    const float font_size = layout.px(size);
    ImFont* font = definitive_ui::font_for_size(font_size);
    const ImVec2 text_size = font->CalcTextSizeA(
        font_size, FLT_MAX, 0.0f, text);
    const ImVec2 p = layout.point(right_x, y);
    draw->AddText(font, font_size,
        ImVec2(p.x - text_size.x, p.y),
        definitive_text_color(color), text);
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
    MenuIcon icon, const char* title, const char* subtitle,
    bool interaction_enabled = true) {
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

    const bool hovered =
        interaction_enabled && ImGui::IsItemHovered();
    const bool focused =
        interaction_enabled && ImGui::IsItemFocused();
    const bool active =
        interaction_enabled && ImGui::IsItemActive();
    const bool engaged = hovered || focused || active;

    bool& was_engaged =
        g_menu_was_engaged[static_cast<size_t>(index)];
    if (interaction_enabled && engaged && !was_engaged) {
        play_menu_sound(UiMenuSound::Cursor);
    }
    was_engaged = engaged;

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
            definitive_accent_color(
                rgba(90, 154, 216, glow_alpha(17.0f * glow))),
            0.0f, 0, layout.px(1.0f));
        draw->AddRect(mid0, mid1,
            definitive_accent_color(
                rgba(126, 184, 236, glow_alpha(34.0f * glow))),
            0.0f, 0, layout.px(1.2f));
    }

    if (highlight_mix > 0.01f) {
        const int fill_alpha =
            glow_alpha(116.0f * highlight_mix + (hovered ? 10.0f : 0.0f));
        draw->AddRectFilled(
            p, ImVec2(p.x + size.x, p.y + size.y),
            definitive_surface_color(rgba(12, 17, 23, fill_alpha), 0.90f));

        draw->AddRect(
            p, ImVec2(p.x + size.x, p.y + size.y),
            definitive_accent_color(
                rgba(211, 229, 246, glow_alpha(235.0f * highlight_mix))),
            0.0f, 0, layout.px(1.35f));
        draw->AddRect(
            ImVec2(p.x + layout.px(2.0f), p.y + layout.px(2.0f)),
            ImVec2(p.x + size.x - layout.px(2.0f),
                p.y + size.y - layout.px(2.0f)),
            definitive_accent_color(
                rgba(103, 154, 205, glow_alpha(128.0f * highlight_mix))),
            0.0f, 0, layout.px(0.8f));

        const float rail_half = layout.px(16.0f + 5.0f * highlight_mix);
        const float center_y = p.y + size.y * 0.5f;
        draw->AddRectFilled(
            ImVec2(p.x - layout.px(2.0f), center_y - rail_half),
            ImVec2(p.x, center_y + rail_half),
            definitive_accent_color(
                rgba(205, 231, 255, glow_alpha(235.0f * highlight_mix))));
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
        kX + 24.0f + content_shift, y + 18.0f,
        definitive_text_color(main_color));
    add_text(draw, layout, kX + 72.0f + content_shift, y + 10.0f, 20.5f,
        main_color, title);
    add_text(draw, layout, kX + 72.0f + content_shift, y + 38.0f, 12.0f,
        sub_color, subtitle);

    ImGui::PopID();
    return interaction_enabled && pressed;
}

bool small_button(const Layout& layout, const char* id, const char* label,
    float x, float y, float w, float h, bool enabled = true) {
    ImGui::SetCursorScreenPos(layout.point(x, y));
    if (!enabled) {
        ImGui::BeginDisabled();
    }
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button,
        definitive_surface_color(rgba(12, 15, 19, 205), 0.88f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
        definitive_surface_color(rgba(24, 31, 39, 225), 0.72f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
        definitive_accent_color(rgba(32, 42, 52, 235), 0.76f));
    ImGui::PushStyleColor(ImGuiCol_Border,
        definitive_accent_color(rgba(128, 145, 163, 190), 0.68f));
    ImGui::PushStyleColor(ImGuiCol_Text,
        definitive_text_color(rgba(226, 230, 235, 245)));
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
    draw->AddRectFilled(
        p0, p1, definitive_list_color(rgba(5, 8, 11, 178), 0.88f));
    draw->AddRect(
        p0, p1, definitive_accent_color(rgba(102, 116, 130, 205), 0.58f),
        0.0f, 0, layout.px(1.0f));
}

void draw_folder_badge(ImDrawList* draw, const Layout& layout, float x, float y) {
    const ImVec2 p = layout.point(x, y);
    const float s = layout.scale;
    const ImU32 c = definitive_text_color(rgba(226, 230, 235, 240));
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
    const ImU32 c = definitive_text_color(rgba(226, 230, 235, 240));
    draw->AddCircle(ImVec2(p.x + 9.0f * s, p.y + 10.0f * s),
        8.0f * s, c, 16, 1.5f * s);
    draw->AddCircleFilled(ImVec2(p.x + 9.0f * s, p.y + 6.0f * s),
        1.0f * s, c);
    draw->AddLine(ImVec2(p.x + 9.0f * s, p.y + 9.0f * s),
        ImVec2(p.x + 9.0f * s, p.y + 15.0f * s), c, 1.5f * s);
}
}

void App::play_ui_cursor_sound() {
    play_menu_sound(UiMenuSound::Cursor);
}

void App::play_ui_open_sound() {
    play_menu_sound(UiMenuSound::Open);
}

void App::play_ui_close_sound() {
    play_menu_sound(UiMenuSound::Close);
}

void App::release_definitive_ui_assets() {
    if (g_ui_sound_device != 0) {
        SDL_ClearQueuedAudio(g_ui_sound_device);
        SDL_CloseAudioDevice(g_ui_sound_device);
        g_ui_sound_device = 0;
    }
    g_ui_sound_spec = {};
    g_ui_sound_load_attempted = false;
    g_ui_action_sound_until_ms = 0;
    g_ui_cursor_sound.pcm.clear();
    g_ui_open_sound.pcm.clear();
    g_ui_close_sound.pcm.clear();
    g_ui_logo_sound.pcm.clear();
    g_launcher_startup_sound_played = false;
    g_menu_was_engaged.fill(false);

    definitive_ui::release_intro_assets();

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
    g_launcher_intro_elapsed = 0.0f;
    g_launcher_intro_complete = false;
    g_launcher_ui_intro_elapsed = 0.0f;
    g_launcher_ui_intro_complete = false;
    g_launcher_background_fade_elapsed = 0.0f;
    g_launcher_background_fade_complete = false;
    definitive_settings_transition_ =
        DefinitiveSettingsTransition::Closed;
    definitive_settings_transition_elapsed_ = 0.0f;
    g_menu_highlight_mix.fill(0.0f);
}

void App::panel_definitive_home() {
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 window_pos = ImGui::GetWindowPos();
    const ImVec2 window_size = ImGui::GetWindowSize();

    if (!g_launcher_startup_sound_played) {
        play_startup_logo_sound();
    }

    if (!g_launcher_intro_complete) {
        const bool skip_intro =
            ImGui::IsKeyPressed(ImGuiKey_Space, false) ||
            ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false);

        if (skip_intro) {
            stop_startup_logo_sound();

            // End the intro on this frame and return once so the same keypress
            // cannot also activate a launcher button underneath it.
            g_launcher_intro_elapsed = definitive_ui::kLauncherIntroDuration;
            g_launcher_intro_complete = true;
            g_launcher_ui_intro_elapsed = 0.0f;
            g_launcher_ui_intro_complete = false;
            g_launcher_background_fade_elapsed = 0.0f;
            g_launcher_background_fade_complete = false;
            ensure_background_texture_loaded();
            definitive_ui::draw_intro_presentation(
                window_pos, window_size, g_launcher_intro_elapsed);
            return;
        }

        const float dt = std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);
        g_launcher_intro_elapsed += dt;
        if (g_launcher_intro_elapsed >= definitive_ui::kLauncherIntroDuration) {
            g_launcher_intro_elapsed = definitive_ui::kLauncherIntroDuration;
            g_launcher_intro_complete = true;
            definitive_ui::draw_intro_presentation(
                window_pos, window_size, g_launcher_intro_elapsed);
            return;
        }
    }

    const bool launcher_intro_active = !g_launcher_intro_complete;

    if (g_launcher_intro_complete && !g_launcher_ui_intro_complete) {
        const float dt =
            std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);
        g_launcher_ui_intro_elapsed += dt;
        if (g_launcher_ui_intro_elapsed >= kLauncherUiIntroDuration) {
            g_launcher_ui_intro_elapsed = kLauncherUiIntroDuration;
            g_launcher_ui_intro_complete = true;
        }
    }

    if (g_launcher_ui_intro_complete &&
        !g_launcher_background_fade_complete) {
        const float dt =
            std::clamp(ImGui::GetIO().DeltaTime, 0.0f, 0.05f);
        g_launcher_background_fade_elapsed += dt;
        if (g_launcher_background_fade_elapsed >=
            kLauncherBackgroundFadeDuration) {
            g_launcher_background_fade_elapsed =
                kLauncherBackgroundFadeDuration;
            g_launcher_background_fade_complete = true;
        }
    }

    const bool launcher_ui_initializing =
        g_launcher_intro_complete && !g_launcher_ui_intro_complete;
    const bool launcher_background_fading =
        g_launcher_ui_intro_complete &&
        !g_launcher_background_fade_complete;
    const bool launcher_ready =
        g_launcher_intro_complete &&
        g_launcher_ui_intro_complete &&
        g_launcher_background_fade_complete;

    const float launcher_background_alpha =
        !g_launcher_ui_intro_complete
            ? 0.0f
            : (g_launcher_background_fade_complete
                ? 1.0f
                : smoothstep01(std::clamp(
                    g_launcher_background_fade_elapsed /
                        kLauncherBackgroundFadeDuration,
                    0.0f, 1.0f)));

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
    definitive_ui::preload_intro_assets();
    ensure_ui_sounds_loaded();

    if (launcher_intro_active) {
        definitive_ui::draw_intro_presentation(
            window_pos, window_size, g_launcher_intro_elapsed);
        return;
    }

    Layout layout = make_layout(window_pos, window_size);

    draw_background(
        draw, window_pos, window_size, launcher_background_alpha);
    draw_readability_shade(draw, window_pos, window_size);

    const ImVec2 bottom0(window_pos.x, window_pos.y + window_size.y * 0.64f);
    const ImVec2 bottom1(window_pos.x + window_size.x, window_pos.y + window_size.y);
    draw->AddRectFilledMultiColor(bottom0, bottom1,
        definitive_background_color(rgba(1, 3, 6, 10), 0.76f),
        definitive_background_color(rgba(1, 3, 6, 10), 0.76f),
        definitive_background_color(rgba(1, 3, 6, 206), 0.82f),
        definitive_background_color(rgba(1, 3, 6, 206), 0.82f));

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
        "Start Emulation", "Load BIOS and start playing", launcher_ready);
    const bool load_game_pressed = menu_button(layout, draw, 1, MenuIcon::Folder,
        "Load Game", "Choose a game from your library", launcher_ready);
    const bool change_bios_pressed = menu_button(layout, draw, 2, MenuIcon::Chip,
        "Change BIOS", "Manage BIOS files", launcher_ready);
    const bool settings_pressed = menu_button(layout, draw, 3, MenuIcon::Settings,
        "Settings", "Configure emulator options", launcher_ready);
    const bool exit_pressed = menu_button(layout, draw, 4, MenuIcon::Exit,
        "Exit", "Close VibeStation", launcher_ready);

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

    if (start_pressed && launcher_ready &&
        g_launcher_start_transition == LauncherStartTransition::None) {
        play_ui_open_sound();
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

    if (load_game_pressed && launcher_ready &&
        !launcher_transitioning) {
        play_ui_open_sound();
        std::string path = open_file_dialog(
            "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
            "Select PS1 Game");
        play_ui_close_sound();
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

    if (change_bios_pressed && launcher_ready &&
        !launcher_transitioning) {
        play_ui_open_sound();
        choose_bios();
        play_ui_close_sound();
    }
    if (settings_pressed && launcher_ready &&
        !launcher_transitioning) {
        play_ui_open_sound();
        open_definitive_settings();
    }
    if (exit_pressed && launcher_ready &&
        !launcher_transitioning) {
        play_ui_close_sound();
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
        ImGui::PushStyleColor(ImGuiCol_ScrollbarBg, definitive_list_color(rgba(4, 7, 10, 90), 0.88f));
        ImGui::PushStyleColor(ImGuiCol_ScrollbarGrab, definitive_accent_color(rgba(104, 120, 137, 145), 0.58f));
        ImGui::PushStyleColor(
            ImGuiCol_ScrollbarGrabHovered, definitive_accent_color(rgba(150, 172, 194, 190), 0.74f));
        ImGui::PushStyleColor(
            ImGuiCol_ScrollbarGrabActive, definitive_accent_color(rgba(193, 216, 238, 220), 0.88f));

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
                        ? definitive_accent_color(rgba(58, 79, 98, 125), 0.72f)
                        : IM_COL32(0, 0, 0, 0));
                ImGui::PushStyleColor(
                    ImGuiCol_HeaderHovered,
                    definitive_surface_color(rgba(53, 68, 83, 150), 0.78f));
                ImGui::PushStyleColor(
                    ImGuiCol_HeaderActive,
                    definitive_accent_color(rgba(69, 91, 111, 175), 0.76f));
                const bool chosen = ImGui::Selectable(
                    entry.title.c_str(), is_selected, 0,
                    ImVec2(0.0f, layout.px(19.0f)));
                ImGui::PopStyleColor(3);
                ImGui::PopID();

                if (chosen && launcher_ready) {
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
        launcher_ready) {
        play_ui_open_sound();
        const std::string selected = open_folder_dialog("Select ROM Directory");
        play_ui_close_sound();
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
        launcher_ready) {
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
    if (launcher_ui_initializing) {
        draw_launcher_initialization_overlay(
            window_pos, window_size, g_launcher_ui_intro_elapsed);
    }

    (void)launcher_background_fading;

    if (launcher_fade_alpha > 0.0f || launcher_started_this_frame) {
        const int fade_alpha = glow_alpha(255.0f * launcher_fade_alpha);
        ImDrawList* fade_draw =
            ImGui::GetForegroundDrawList();
        fade_draw->AddRectFilled(
            window_pos,
            ImVec2(window_pos.x + window_size.x, window_pos.y + window_size.y),
            rgba(0, 0, 0, fade_alpha));
    }

}
