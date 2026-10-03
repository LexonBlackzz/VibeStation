#include "ui/vs2/vs2_shared.h"

#include <SDL.h>

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

bool g_loaded = false;
bool g_enabled = true;
Uint32 g_action_until_ms = 0;
std::array<std::vector<float>, static_cast<std::size_t>(Clip::Count)> g_clips{};

void ensure_loaded() {
    if (g_loaded) return;
    g_loaded = true;
    for (std::size_t i = 0; i < kFiles.size(); ++i) load_wav_stereo(kFiles[i], g_clips[i]);
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

} // namespace

void preload_sounds() { ensure_loaded(); }

void play_boot_sound() {
    stop_boot_sound();
    const std::filesystem::path path = find_asset("vs2-boot.wav");
    SDL_AudioSpec spec{};
    Uint8* buffer = nullptr;
    Uint32 length = 0;
    if (path.empty() || SDL_LoadWAV(path.string().c_str(), &spec, &buffer, &length) == nullptr) {
        return;
    }
    spec.callback = nullptr;
    spec.userdata = nullptr;
    // allowed_changes = 0: SDL converts to the hardware format for us.
    g_boot_device = SDL_OpenAudioDevice(nullptr, 0, &spec, nullptr, 0);
    if (g_boot_device != 0) {
        SDL_QueueAudio(g_boot_device, buffer, length);
        SDL_PauseAudioDevice(g_boot_device, 0);
    }
    SDL_FreeWAV(buffer);
}

void stop_boot_sound() {
    if (g_boot_device != 0) {
        SDL_ClearQueuedAudio(g_boot_device);
        SDL_CloseAudioDevice(g_boot_device);
        g_boot_device = 0;
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
