#include "ui/vs2/vs2_shared.h"

#include <SDL.h>

#include <algorithm>
#include <array>
#include <vector>

namespace ps2::ui::vs2 {

namespace {

// Menu sounds, played through the frontend mixer (vs2_mixer.cpp), which adds
// the reverb. As in definitive_audio.cpp, an action sound (open, close,
// select) owns the channel until it finishes so a cursor tick cannot cut it
// off.
enum class Clip { Highlight, Open, Close, Select, Count };

constexpr std::array<const char*, static_cast<std::size_t>(Clip::Count)> kFiles = {
    "vs2-highlight.wav", "vs2-menuopen.wav", "vs2-menuclose.wav", "vs2-selected.wav"};
// Per-clip level: highlight, close and select sit under the open sound.
constexpr std::array<float, static_cast<std::size_t>(Clip::Count)> kClipGains = {
    0.6f, 1.0f, 0.6f, 0.6f};
// The back-to-menu jingle is boosted (with clipping protection).
constexpr float kBackToMenuGain = 2.5f; // the file peaks at -22 dBFS

bool g_loaded = false;
bool g_enabled = true;
Uint32 g_action_until_ms = 0;
std::array<std::vector<float>, static_cast<std::size_t>(Clip::Count)> g_clips{};

void ensure_loaded() {
    if (g_loaded) return;
    g_loaded = true;
    for (std::size_t i = 0; i < kFiles.size(); ++i) {
        load_wav_stereo(kFiles[i], g_clips[i]);
        for (float& s : g_clips[i]) s *= kClipGains[i];
    }
}

void play(Clip clip) {
    if (!g_enabled) return;
    ensure_loaded();
    const auto& pcm = g_clips[static_cast<std::size_t>(clip)];
    if (pcm.empty()) return;

    const Uint32 now = SDL_GetTicks();
    const bool action_playing =
        g_action_until_ms != 0 && !SDL_TICKS_PASSED(now, g_action_until_ms);
    if (clip == Clip::Highlight) {
        if (action_playing) return;
    } else {
        const Uint32 duration_ms = static_cast<Uint32>(pcm.size() * 1000u / (2u * kMixerRate));
        g_action_until_ms = now + std::max<Uint32>(duration_ms, 40u);
    }
    mixer_play_ui(&pcm);
}

SDL_AudioDeviceID g_boot_device = 0;
// The back-to-menu jingle has its own device so it can play over a boot
// sound that is still fading out after a skip.
SDL_AudioDeviceID g_menu_jingle_device = 0;
std::vector<Uint8> g_jingle_pcm; // the clip last queued on g_boot_device
SDL_AudioSpec g_jingle_spec{};

} // namespace

void preload_sounds() { ensure_loaded(); }

namespace {

// Multiplies the samples by gain, clamping instead of wrapping on overflow.
void scale_pcm(Uint8* pcm, Uint32 length, const SDL_AudioSpec& spec, float gain) {
    switch (spec.format) {
    case AUDIO_S16SYS: {
        auto* s = reinterpret_cast<Sint16*>(pcm);
        for (Uint32 i = 0; i < length / 2; ++i)
            s[i] = static_cast<Sint16>(
                std::clamp(static_cast<float>(s[i]) * gain, -32768.0f, 32767.0f));
        break;
    }
    case AUDIO_F32SYS: {
        auto* s = reinterpret_cast<float*>(pcm);
        for (Uint32 i = 0; i < length / 4; ++i) s[i] = std::clamp(s[i] * gain, -1.0f, 1.0f);
        break;
    }
    case AUDIO_U8:
        for (Uint32 i = 0; i < length; ++i)
            pcm[i] = static_cast<Uint8>(std::clamp(
                128.0f + (static_cast<float>(pcm[i]) - 128.0f) * gain, 0.0f, 255.0f));
        break;
    default:
        break;
    }
}

// Opens `device` for a WAV clip and plays it at `gain`. With `keep`, the
// samples are kept for fade_out_boot_sound().
void play_jingle(const char* file, SDL_AudioDeviceID& device, bool keep, float gain = 1.0f) {
    if (device != 0) {
        SDL_ClearQueuedAudio(device);
        SDL_CloseAudioDevice(device);
        device = 0;
    }
    const std::vector<unsigned char> wav = load_asset(file);
    SDL_AudioSpec spec{};
    Uint8* buffer = nullptr;
    Uint32 length = 0;
    if (wav.empty() ||
        SDL_LoadWAV_RW(SDL_RWFromConstMem(wav.data(), static_cast<int>(wav.size())), 1, &spec,
                       &buffer, &length) == nullptr) {
        return;
    }
    spec.callback = nullptr;
    spec.userdata = nullptr;
    if (gain != 1.0f) scale_pcm(buffer, length, spec, gain);
    // allowed_changes = 0: SDL converts to the hardware format for us.
    device = SDL_OpenAudioDevice(nullptr, 0, &spec, nullptr, 0);
    if (device != 0) {
        SDL_QueueAudio(device, buffer, length);
        SDL_PauseAudioDevice(device, 0);
        if (keep) {
            // fade_out_boot_sound() needs it: the queue is the clip's remainder.
            g_jingle_pcm.assign(buffer, buffer + length);
            g_jingle_spec = spec;
        }
    }
    SDL_FreeWAV(buffer);
}

// Linear ramp to silence over the whole buffer; false for unhandled formats.
bool fade_out_pcm(std::vector<Uint8>& pcm, const SDL_AudioSpec& spec) {
    const std::size_t channels = spec.channels;
    const std::size_t bytes = static_cast<std::size_t>(SDL_AUDIO_BITSIZE(spec.format) / 8);
    if (channels == 0 || bytes == 0) return false;
    const std::size_t frames = pcm.size() / (bytes * channels);
    const auto gain = [frames](std::size_t frame) {
        return frames <= 1 ? 0.0f
                           : 1.0f - static_cast<float>(frame) / static_cast<float>(frames - 1);
    };
    switch (spec.format) {
    case AUDIO_S16SYS: {
        auto* s = reinterpret_cast<Sint16*>(pcm.data());
        for (std::size_t i = 0; i < frames * channels; ++i)
            s[i] = static_cast<Sint16>(static_cast<float>(s[i]) * gain(i / channels));
        return true;
    }
    case AUDIO_F32SYS: {
        auto* s = reinterpret_cast<float*>(pcm.data());
        for (std::size_t i = 0; i < frames * channels; ++i) s[i] *= gain(i / channels);
        return true;
    }
    case AUDIO_U8:
        for (std::size_t i = 0; i < frames * channels; ++i)
            pcm[i] = static_cast<Uint8>(
                128.0f + (static_cast<float>(pcm[i]) - 128.0f) * gain(i / channels));
        return true;
    default:
        return false;
    }
}

} // namespace

void play_boot_sound() {
    stop_boot_sound();
    play_jingle("vs2-boot.wav", g_boot_device, true);
}

void fade_out_boot_sound(float fade_ms) {
    if (g_boot_device == 0 || g_jingle_pcm.empty()) return;
    const std::size_t frame_bytes =
        static_cast<std::size_t>(SDL_AUDIO_BITSIZE(g_jingle_spec.format) / 8) *
        g_jingle_spec.channels;
    const std::size_t total = g_jingle_pcm.size();
    const std::size_t queued =
        std::min<std::size_t>(SDL_GetQueuedAudioSize(g_boot_device), total);
    if (frame_bytes == 0 || queued < frame_bytes) return;
    const std::size_t position = (total - queued) / frame_bytes * frame_bytes;
    const std::size_t fade_frames = std::max<std::size_t>(
        1u, static_cast<std::size_t>(static_cast<float>(g_jingle_spec.freq) * fade_ms / 1000.0f));
    const std::size_t length =
        std::min((total - position) / frame_bytes * frame_bytes, fade_frames * frame_bytes);
    std::vector<Uint8> tail(g_jingle_pcm.begin() + static_cast<std::ptrdiff_t>(position),
                            g_jingle_pcm.begin() + static_cast<std::ptrdiff_t>(position + length));
    if (!fade_out_pcm(tail, g_jingle_spec)) {
        stop_boot_sound();
        return;
    }
    SDL_ClearQueuedAudio(g_boot_device);
    SDL_QueueAudio(g_boot_device, tail.data(), static_cast<Uint32>(tail.size()));
}

void play_back_to_menu_sound() {
    if (g_enabled) play_jingle("vs2-backtomenu.wav", g_menu_jingle_device, false, kBackToMenuGain);
}

void stop_boot_sound() {
    if (g_boot_device != 0) {
        SDL_ClearQueuedAudio(g_boot_device);
        SDL_CloseAudioDevice(g_boot_device);
        g_boot_device = 0;
    }
    if (g_menu_jingle_device != 0) {
        SDL_ClearQueuedAudio(g_menu_jingle_device);
        SDL_CloseAudioDevice(g_menu_jingle_device);
        g_menu_jingle_device = 0;
    }
}

// Call after mixer_release(): the mixer's voices point at these clips.
void release_sounds() {
    stop_boot_sound();
    for (auto& clip : g_clips) clip.clear();
    g_loaded = false;
    g_action_until_ms = 0;
}

void play_highlight_sound() { play(Clip::Highlight); }
void play_open_sound() { play(Clip::Open); }
void play_close_sound() { play(Clip::Close); }
void play_select_sound() { play(Clip::Select); }
void set_sounds_enabled(bool enabled) { g_enabled = enabled; }

} // namespace ps2::ui::vs2
