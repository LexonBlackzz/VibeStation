#include "ui/app.h"
#include "ui/definitive/definitive_shared.h"

#include <SDL.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include <vector>

namespace {

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

std::filesystem::path find_ui_sound_path(
    const char* filename) {
    std::array<std::filesystem::path, 4>
        candidates{};

    std::error_code ec;
    const std::filesystem::path cwd =
        std::filesystem::current_path(ec);

    if (!ec) {
        candidates[0] =
            cwd / "resources" / "ui" /
            "definitive" / "sounds" / filename;
        candidates[1] =
            cwd / ".." / "resources" / "ui" /
            "definitive" / "sounds" / filename;
    }

    if (char* base = SDL_GetBasePath()) {
        const std::filesystem::path base_path(base);
        SDL_free(base);

        candidates[2] =
            base_path / "resources" / "ui" /
            "definitive" / "sounds" / filename;
        candidates[3] =
            base_path / ".." / "resources" / "ui" /
            "definitive" / "sounds" / filename;
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
    const int cvt_result =
        SDL_BuildAudioCVT(
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
        out_clip.pcm.assign(
            source_buffer,
            source_buffer + source_length);
        SDL_FreeWAV(source_buffer);
        return true;
    }

    cvt.len =
        static_cast<int>(source_length);
    cvt.buf =
        static_cast<Uint8*>(
            SDL_malloc(
                static_cast<size_t>(
                    source_length) *
                static_cast<size_t>(
                    cvt.len_mult)));

    if (cvt.buf == nullptr) {
        SDL_FreeWAV(source_buffer);
        return false;
    }

    std::memcpy(
        cvt.buf,
        source_buffer,
        source_length);
    SDL_FreeWAV(source_buffer);

    if (SDL_ConvertAudio(&cvt) != 0) {
        SDL_free(cvt.buf);
        return false;
    }

    out_clip.pcm.assign(
        cvt.buf,
        cvt.buf + cvt.len_cvt);
    SDL_free(cvt.buf);
    return true;
}

void apply_ui_sound_fade_in(
    UiSoundClip& clip,
    const SDL_AudioSpec& spec,
    float fade_ms) {
    if (clip.pcm.empty() ||
        spec.freq <= 0 ||
        spec.channels == 0 ||
        fade_ms <= 0.0f) {
        return;
    }

    const int bits_per_sample =
        SDL_AUDIO_BITSIZE(spec.format);
    if (bits_per_sample <= 0 ||
        (bits_per_sample % 8) != 0) {
        return;
    }

    const size_t bytes_per_sample =
        static_cast<size_t>(
            bits_per_sample / 8);
    const size_t bytes_per_frame =
        bytes_per_sample *
        static_cast<size_t>(
            spec.channels);

    if (bytes_per_frame == 0) {
        return;
    }

    const size_t frame_count =
        clip.pcm.size() /
        bytes_per_frame;
    const size_t requested_fade_frames =
        static_cast<size_t>(
            std::max(
                1.0f,
                static_cast<float>(
                    spec.freq) *
                    fade_ms /
                    1000.0f));
    const size_t fade_frames =
        std::min(
            frame_count,
            requested_fade_frames);

    if (fade_frames == 0) {
        return;
    }

    const size_t sample_count =
        fade_frames *
        static_cast<size_t>(
            spec.channels);

    switch (spec.format) {
    case AUDIO_F32SYS: {
        float* samples =
            reinterpret_cast<float*>(
                clip.pcm.data());

        for (size_t i = 0;
             i < sample_count;
             ++i) {
            const size_t frame =
                i /
                static_cast<size_t>(
                    spec.channels);
            const float gain =
                static_cast<float>(frame + 1) /
                static_cast<float>(
                    fade_frames);
            samples[i] *= gain;
        }
        break;
    }

    case AUDIO_S16SYS: {
        Sint16* samples =
            reinterpret_cast<Sint16*>(
                clip.pcm.data());

        for (size_t i = 0;
             i < sample_count;
             ++i) {
            const size_t frame =
                i /
                static_cast<size_t>(
                    spec.channels);
            const float gain =
                static_cast<float>(frame + 1) /
                static_cast<float>(
                    fade_frames);

            samples[i] =
                static_cast<Sint16>(
                    static_cast<float>(
                        samples[i]) *
                    gain);
        }
        break;
    }

    case AUDIO_S32SYS: {
        Sint32* samples =
            reinterpret_cast<Sint32*>(
                clip.pcm.data());

        for (size_t i = 0;
             i < sample_count;
             ++i) {
            const size_t frame =
                i /
                static_cast<size_t>(
                    spec.channels);
            const float gain =
                static_cast<float>(frame + 1) /
                static_cast<float>(
                    fade_frames);

            samples[i] =
                static_cast<Sint32>(
                    static_cast<double>(
                        samples[i]) *
                    static_cast<double>(
                        gain));
        }
        break;
    }

    case AUDIO_S8: {
        Sint8* samples =
            reinterpret_cast<Sint8*>(
                clip.pcm.data());

        for (size_t i = 0;
             i < sample_count;
             ++i) {
            const size_t frame =
                i /
                static_cast<size_t>(
                    spec.channels);
            const float gain =
                static_cast<float>(frame + 1) /
                static_cast<float>(
                    fade_frames);

            samples[i] =
                static_cast<Sint8>(
                    static_cast<float>(
                        samples[i]) *
                    gain);
        }
        break;
    }

    case AUDIO_U8: {
        Uint8* samples = clip.pcm.data();

        for (size_t i = 0;
             i < sample_count;
             ++i) {
            const size_t frame =
                i /
                static_cast<size_t>(
                    spec.channels);
            const float gain =
                static_cast<float>(frame + 1) /
                static_cast<float>(
                    fade_frames);
            const float centered =
                static_cast<float>(
                    samples[i]) -
                128.0f;

            samples[i] =
                static_cast<Uint8>(
                    std::clamp(
                        128.0f +
                            centered *
                            gain,
                        0.0f,
                        255.0f));
        }
        break;
    }

    default:
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

    if (cursor_path.empty() ||
        open_path.empty() ||
        close_path.empty() ||
        logo_path.empty()) {
        return false;
    }

    SDL_AudioSpec desired{};
    desired.freq = 44100;
    desired.format = AUDIO_F32SYS;
    desired.channels = 2;
    desired.samples = 512;
    desired.callback = nullptr;

    g_ui_sound_device =
        SDL_OpenAudioDevice(
            nullptr,
            0,
            &desired,
            &g_ui_sound_spec,
            SDL_AUDIO_ALLOW_ANY_CHANGE);

    if (g_ui_sound_device == 0) {
        return false;
    }

    const bool loaded =
        convert_ui_sound(
            cursor_path,
            g_ui_sound_spec,
            g_ui_cursor_sound) &&
        convert_ui_sound(
            open_path,
            g_ui_sound_spec,
            g_ui_open_sound) &&
        convert_ui_sound(
            close_path,
            g_ui_sound_spec,
            g_ui_close_sound) &&
        convert_ui_sound(
            logo_path,
            g_ui_sound_spec,
            g_ui_logo_sound);

    if (loaded) {
        apply_ui_sound_fade_in(
            g_ui_cursor_sound,
            g_ui_sound_spec,
            15.0f);
    }

    if (!loaded) {
        SDL_CloseAudioDevice(
            g_ui_sound_device);
        g_ui_sound_device = 0;

        g_ui_cursor_sound.pcm.clear();
        g_ui_open_sound.pcm.clear();
        g_ui_close_sound.pcm.clear();
        g_ui_logo_sound.pcm.clear();
        return false;
    }

    SDL_PauseAudioDevice(
        g_ui_sound_device,
        0);
    return true;
}

Uint32 ui_sound_duration_ms(
    const UiSoundClip& clip) {
    const int bits_per_sample =
        SDL_AUDIO_BITSIZE(
            g_ui_sound_spec.format);

    const Uint32 bytes_per_second =
        (g_ui_sound_spec.freq > 0 &&
         g_ui_sound_spec.channels > 0 &&
         bits_per_sample > 0)
            ? static_cast<Uint32>(
                g_ui_sound_spec.freq *
                g_ui_sound_spec.channels *
                (bits_per_sample / 8))
            : 0u;

    if (bytes_per_second == 0 ||
        clip.pcm.empty()) {
        return 0;
    }

    return static_cast<Uint32>(
        (static_cast<Uint64>(
            clip.pcm.size()) *
            1000u) /
        bytes_per_second);
}

void play_menu_sound(UiMenuSound sound) {
    if (!ensure_ui_sounds_loaded() ||
        g_ui_sound_device == 0) {
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

    if (clip == nullptr ||
        clip->pcm.empty()) {
        return;
    }

    const Uint32 now = SDL_GetTicks();
    const bool action_sound_active =
        g_ui_action_sound_until_ms != 0 &&
        !SDL_TICKS_PASSED(
            now,
            g_ui_action_sound_until_ms);

    if (sound == UiMenuSound::Cursor) {
        if (action_sound_active) {
            return;
        }

        SDL_ClearQueuedAudio(
            g_ui_sound_device);
    }
    else {
        SDL_ClearQueuedAudio(
            g_ui_sound_device);

        const Uint32 duration_ms =
            std::max<Uint32>(
                ui_sound_duration_ms(*clip),
                40u);

        g_ui_action_sound_until_ms =
            now + duration_ms;
    }

    SDL_QueueAudio(
        g_ui_sound_device,
        clip->pcm.data(),
        static_cast<Uint32>(
            clip->pcm.size()));
}

} // namespace

namespace definitive_ui {

void preload_audio_assets() {
    ensure_ui_sounds_loaded();
}

void release_audio_assets() {
    if (g_ui_sound_device != 0) {
        SDL_ClearQueuedAudio(
            g_ui_sound_device);
        SDL_CloseAudioDevice(
            g_ui_sound_device);
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
}

void play_cursor_sound() {
    play_menu_sound(
        UiMenuSound::Cursor);
}

void play_open_sound() {
    play_menu_sound(
        UiMenuSound::Open);
}

void play_close_sound() {
    play_menu_sound(
        UiMenuSound::Close);
}

void play_startup_sound() {
    if (g_launcher_startup_sound_played ||
        !ensure_ui_sounds_loaded() ||
        g_ui_sound_device == 0 ||
        g_ui_logo_sound.pcm.empty()) {
        return;
    }

    SDL_ClearQueuedAudio(
        g_ui_sound_device);

    if (SDL_QueueAudio(
            g_ui_sound_device,
            g_ui_logo_sound.pcm.data(),
            static_cast<Uint32>(
                g_ui_logo_sound.pcm.size())) == 0) {
        g_launcher_startup_sound_played = true;

        const Uint32 duration_ms =
            std::max<Uint32>(
                ui_sound_duration_ms(
                    g_ui_logo_sound),
                40u);

        g_ui_action_sound_until_ms =
            SDL_GetTicks() +
            duration_ms;
    }
}

void stop_startup_sound() {
    if (!g_launcher_startup_sound_played ||
        g_ui_sound_device == 0) {
        return;
    }

    SDL_ClearQueuedAudio(
        g_ui_sound_device);
    g_ui_action_sound_until_ms = 0;
}

} // namespace definitive_ui

void App::play_ui_cursor_sound() {
    definitive_ui::play_cursor_sound();
}

void App::play_ui_open_sound() {
    definitive_ui::play_open_sound();
}

void App::play_ui_close_sound() {
    definitive_ui::play_close_sound();
}
