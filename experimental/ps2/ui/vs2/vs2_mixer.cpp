#include "ui/vs2/vs2_shared.h"

#include <SDL.h>

#include <array>
#include <atomic>
#include <cstring>
#include <random>
#include <vector>

#if defined(_M_X64) || defined(__x86_64__) || defined(_M_IX86) || defined(__i386__)
#include <xmmintrin.h>
#define VS2_HAVE_SSE 1
#endif

namespace ps2::ui::vs2 {

namespace {

// The frontend's sound mixer, on its own SDL device (the boot sound is the
// only thing that plays elsewhere):
// - menu sounds: one at a time; a new one fades the previous out quickly
// - ambience: vs2-ambientbg.wav loops quietly; the first pass plays as
//   recorded, every later pass tape-style at a new speed (and pitch).
//   vs2-certainstatic.wav washes in at random intervals like waves, each one
//   slowed down, pitch-shifted and low-passed differently.
// The menu sounds go through a short room reverb, the static through a
// long hall.

constexpr float kBackgroundVolume = 0.16f;
constexpr float kLoopFadeSeconds = 0.08f;
constexpr float kAmbienceFadeSeconds = 1.6f;
constexpr float kUiCutSeconds = 0.015f;

// Two reverbs. The menu sounds get a short room so they stay crisp; the
// static gets a long, airy hall so each wave lingers far away.
struct ReverbSettings {
    float feedback;  // room size: higher = longer tail
    float damp;      // higher = darker tail
    float predelay;  // seconds before the reverb starts
    float ret;       // return level
};
constexpr ReverbSettings kRoom{0.84f, 0.42f, 0.025f, 0.55f};
constexpr ReverbSettings kHall{0.95f, 0.28f, 0.045f, 0.42f};
constexpr float kStaticDry = 0.75f;
constexpr float kStaticSend = 0.9f; // into the hall
constexpr float kUiSend = 0.32f;    // into the room

// ------------------------------------------------------------------ reverb

// Freeverb (Jezar at Dreampoint, public domain): eight damped comb filters in
// parallel, then four all-pass filters in series, per channel; here with a
// stereo pre-delay in front.
class Reverb {
public:
    void init(int rate, const ReverbSettings& settings) {
        settings_ = settings;
        static constexpr std::array<int, 8> kCombs = {1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617};
        static constexpr std::array<int, 4> kAllpasses = {556, 441, 341, 225};
        constexpr int kStereoSpread = 23;
        const float scale = rate / 44100.0f;
        for (int ch = 0; ch < 2; ++ch) {
            for (std::size_t i = 0; i < kCombs.size(); ++i) {
                combs_[ch][i].buffer.assign(static_cast<std::size_t>((kCombs[i] + ch * kStereoSpread) * scale), 0.0f);
            }
            for (std::size_t i = 0; i < kAllpasses.size(); ++i) {
                allpasses_[ch][i].buffer.assign(static_cast<std::size_t>((kAllpasses[i] + ch * kStereoSpread) * scale), 0.0f);
            }
        }
        predelay_.assign(std::max<std::size_t>(1, static_cast<std::size_t>(settings.predelay * rate)) * 2, 0.0f);
        predelay_pos_ = 0;
    }

    // Takes the send, adds the (returned) reverb to out_l/out_r.
    void process(float send_l, float send_r, float& out_l, float& out_r) {
        if (predelay_.empty()) return;
        float* slot = &predelay_[predelay_pos_ * 2];
        const float in_l = slot[0], in_r = slot[1];
        slot[0] = send_l;
        slot[1] = send_r;
        if (++predelay_pos_ >= predelay_.size() / 2) predelay_pos_ = 0;

        constexpr float kInputGain = 0.015f;
        const float feedback = settings_.feedback, damp = settings_.damp;
        const float input = (in_l + in_r) * kInputGain;
        float acc[2] = {0.0f, 0.0f};
        for (int ch = 0; ch < 2; ++ch) {
            for (Comb& c : combs_[ch]) {
                float& slot_c = c.buffer[c.pos];
                const float y = slot_c;
                c.store = y * (1.0f - damp) + c.store * damp;
                slot_c = input + c.store * feedback;
                if (++c.pos >= c.buffer.size()) c.pos = 0;
                acc[ch] += y;
            }
            for (Allpass& a : allpasses_[ch]) {
                float& slot = a.buffer[a.pos];
                const float buffered = slot;
                slot = acc[ch] + buffered * 0.5f;
                acc[ch] = buffered - acc[ch];
                if (++a.pos >= a.buffer.size()) a.pos = 0;
            }
        }
        out_l += acc[0] * settings_.ret;
        out_r += acc[1] * settings_.ret;
    }

private:
    ReverbSettings settings_{};
    std::vector<float> predelay_; // interleaved stereo ring buffer
    std::size_t predelay_pos_ = 0;
    struct Comb {
        std::vector<float> buffer;
        std::size_t pos = 0;
        float store = 0.0f;
    };
    struct Allpass {
        std::vector<float> buffer;
        std::size_t pos = 0;
    };
    std::array<std::array<Comb, 8>, 2> combs_{};
    std::array<std::array<Allpass, 4>, 2> allpasses_{};
};

// ------------------------------------------------------------------ state

struct Voice {
    std::vector<float> owned;               // static waves own their samples
    const std::vector<float>* clip = nullptr; // menu sounds point at a loaded clip
    std::size_t pos = 0;
    float gain = 1.0f;
    float fade_step = 0.0f;                 // > 0 while being cut
    [[nodiscard]] const std::vector<float>& samples() const { return clip ? *clip : owned; }
    [[nodiscard]] bool done() const { return pos + 1 >= samples().size() || gain <= 0.0f; }
};

struct Mixer {
    SDL_AudioDeviceID device = 0;
    bool attempted = false;

    // Background loop (interleaved stereo s16 at kMixerRate).
    std::vector<Sint16> background;
    double bg_pos = 0.0;
    double bg_rate = 1.0;
    int bg_semitones = 0;

    // Static source (mono float) and the waves currently sounding.
    std::vector<float> static_source;
    std::vector<Voice> waves;   // guarded by SDL_LockAudioDevice
    std::vector<Voice> ui;      // guarded by SDL_LockAudioDevice

    Reverb room; // menu sounds
    Reverb hall; // static

    std::atomic<float> ambience_target{0.0f};
    float ambience_gain = 0.0f;
    std::uint32_t seed = 0x2545F491u;

    float next_wave_in = 3.0f;
    std::mt19937 rng{std::random_device{}()};
};

Mixer g_mixer;

float frand(std::uint32_t& s) { // audio-thread friendly
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return static_cast<float>(s & 0xFFFFFF) / static_cast<float>(0x1000000);
}

void pick_background_rate(Mixer& m) {
    // A new tape speed for each pass after the first, never the same twice.
    static constexpr int kSteps[] = {-3, -2, -1, 1, 2};
    int next = m.bg_semitones;
    while (next == m.bg_semitones) {
        next = kSteps[static_cast<int>(frand(m.seed) * 5.0f) % 5];
    }
    m.bg_semitones = next;
    m.bg_rate = std::pow(2.0, next / 12.0);
}

// Adds one voice's next frame to the dry mix and the reverb send.
void play_voice(Voice& v, float level, float send_level, float& dl, float& dr, float& sl, float& sr) {
    if (v.done()) return;
    const std::vector<float>& s = v.samples();
    const float l = s[v.pos] * v.gain * level, r = s[v.pos + 1] * v.gain * level;
    v.pos += 2;
    if (v.fade_step > 0.0f) v.gain -= v.fade_step;
    dl += l;
    dr += r;
    sl += l * send_level;
    sr += r * send_level;
}

void SDLCALL mix(void* userdata, Uint8* stream, int len) {
    Mixer& m = *static_cast<Mixer*>(userdata);
#ifdef VS2_HAVE_SSE
    // Flush denormals: decaying reverb feedback otherwise spikes the CPU.
    _mm_setcsr(_mm_getcsr() | 0x8040);
#endif
    float* out = reinterpret_cast<float*>(stream);
    const int frames = len / static_cast<int>(sizeof(float) * 2);
    const float target = m.ambience_target.load(std::memory_order_relaxed);
    const float k = 1.0f - std::exp(-1.0f / (kAmbienceFadeSeconds * kMixerRate));
    const std::size_t bg_frames = m.background.size() / 2;
    const double loop_fade = kLoopFadeSeconds * kMixerRate;

    for (int i = 0; i < frames; ++i) {
        m.ambience_gain += (target - m.ambience_gain) * k;
        const float amb = m.ambience_gain;
        float dl = 0.0f, dr = 0.0f;   // dry
        float hl = 0.0f, hr = 0.0f;   // send to the hall (static)
        float rl = 0.0f, rr = 0.0f;   // send to the room (menu sounds)

        if (bg_frames > 2 && amb > 1e-4f) {
            const std::size_t idx = static_cast<std::size_t>(m.bg_pos);
            const float frac = static_cast<float>(m.bg_pos - static_cast<double>(idx));
            const Sint16* a = &m.background[idx * 2];
            const Sint16* b = &m.background[(idx + 1) * 2];
            // Short fades at the loop point so the speed change never clicks.
            const double edge = std::min(m.bg_pos, static_cast<double>(bg_frames - 1) - m.bg_pos);
            const float env = static_cast<float>(std::min(1.0, edge / loop_fade)) * kBackgroundVolume * amb / 32768.0f;
            dl += (a[0] + (b[0] - a[0]) * frac) * env;
            dr += (a[1] + (b[1] - a[1]) * frac) * env;
            m.bg_pos += m.bg_rate;
            if (m.bg_pos >= static_cast<double>(bg_frames - 1)) {
                m.bg_pos = 0.0;
                pick_background_rate(m);
            }
        }
        for (Voice& v : m.waves) play_voice(v, kStaticDry * amb, kStaticSend / kStaticDry, dl, dr, hl, hr);
        for (Voice& v : m.ui) play_voice(v, 1.0f, kUiSend, dl, dr, rl, rr);

        m.hall.process(hl, hr, dl, dr);
        m.room.process(rl, rr, dl, dr);

        out[i * 2] = std::clamp(dl, -1.0f, 1.0f);
        out[i * 2 + 1] = std::clamp(dr, -1.0f, 1.0f);
    }
    std::erase_if(m.waves, [](const Voice& v) { return v.done(); });
    std::erase_if(m.ui, [](const Voice& v) { return v.done(); });
}

bool load_converted(const char* name, SDL_AudioFormat format, Uint8 channels, std::vector<Uint8>& out) {
    const std::filesystem::path path = find_asset(name);
    SDL_AudioSpec spec{};
    Uint8* buffer = nullptr;
    Uint32 length = 0;
    if (path.empty() || SDL_LoadWAV(path.string().c_str(), &spec, &buffer, &length) == nullptr) return false;
    SDL_AudioCVT cvt{};
    if (SDL_BuildAudioCVT(&cvt, spec.format, spec.channels, spec.freq, format, channels, kMixerRate) < 0) {
        SDL_FreeWAV(buffer);
        return false;
    }
    cvt.len = static_cast<int>(length);
    out.assign(static_cast<std::size_t>(length) * std::max(1, cvt.len_mult), 0);
    std::memcpy(out.data(), buffer, length);
    SDL_FreeWAV(buffer);
    cvt.buf = out.data();
    if (cvt.needed && SDL_ConvertAudio(&cvt) != 0) return false;
    out.resize(static_cast<std::size_t>(cvt.needed ? cvt.len_cvt : cvt.len));
    return true;
}

bool ensure_open() {
    Mixer& m = g_mixer;
    if (m.device != 0) return true;
    if (m.attempted) return false;
    m.attempted = true;

    std::vector<Uint8> bytes;
    if (load_converted("vs2-ambientbg.wav", AUDIO_S16SYS, 2, bytes)) {
        m.background.resize(bytes.size() / sizeof(Sint16));
        std::memcpy(m.background.data(), bytes.data(), m.background.size() * sizeof(Sint16));
    }
    if (load_converted("vs2-certainstatic.wav", AUDIO_F32SYS, 1, bytes)) {
        m.static_source.resize(bytes.size() / sizeof(float));
        std::memcpy(m.static_source.data(), bytes.data(), m.static_source.size() * sizeof(float));
    }
    m.room.init(kMixerRate, kRoom);
    m.hall.init(kMixerRate, kHall);

    SDL_AudioSpec want{};
    want.freq = kMixerRate;
    want.format = AUDIO_F32SYS;
    want.channels = 2;
    want.samples = 512;
    want.callback = mix;
    want.userdata = &m;
    m.device = SDL_OpenAudioDevice(nullptr, 0, &want, nullptr, 0);
    if (m.device == 0) return false;
    SDL_PauseAudioDevice(m.device, 0);
    return true;
}

// One wave of static: slowed down (time-stretched to 1.8-2.6x its length),
// a new pitch, a new low-pass, its own level and place in the stereo field,
// swelling in and washing out.
std::vector<float> shape_wave(const std::vector<float>& src, std::mt19937& rng) {
    std::uniform_real_distribution<float> semis(-5.0f, 3.0f);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    const float pitch = std::pow(2.0f, semis(rng) / 12.0f);
    const float stretch = 1.8f + 0.8f * unit(rng);
    const float cutoff = 600.0f * std::pow(5000.0f / 600.0f, unit(rng)); // log-uniform
    const float level = 0.08f + 0.09f * unit(rng);
    const float pan = (unit(rng) - 0.5f) * 1.2f;

    // Granular overlap-add: Hann grains read at `pitch` from a source
    // position that advances 1/stretch as fast as the output, so pitch and
    // length are set independently (50% overlap sums to unity). A little
    // jitter per grain keeps the stretched static from sounding looped.
    constexpr int kGrain = 2048, kHop = kGrain / 2;
    const int n = static_cast<int>(src.size());
    const int out_n = static_cast<int>(n * stretch);
    std::vector<float> shifted(static_cast<std::size_t>(out_n), 0.0f);
    for (int start = -kHop; start < out_n; start += kHop) {
        const float jitter = (unit(rng) - 0.5f) * 512.0f;
        const float src_start = std::clamp(std::max(0, start) / stretch + jitter, 0.0f, static_cast<float>(n - 1));
        for (int j = 0; j < kGrain; ++j) {
            const int o = start + j;
            if (o < 0 || o >= out_n) continue;
            const float w = 0.5f - 0.5f * std::cos(2.0f * 3.14159265f * j / kGrain);
            const float s = src_start + j * pitch;
            const int si = static_cast<int>(s);
            if (si + 1 >= n) continue;
            const float f = s - si;
            shifted[static_cast<std::size_t>(o)] += (src[si] + (src[si + 1] - src[si]) * f) * w;
        }
    }

    // Biquad low-pass (RBJ cookbook, Q = 0.707).
    const float w0 = 2.0f * 3.14159265f * cutoff / kMixerRate;
    const float alpha = std::sin(w0) / (2.0f * 0.7071f);
    const float cw = std::cos(w0);
    const float a0 = 1.0f + alpha;
    const float b0 = (1.0f - cw) * 0.5f / a0, b1 = (1.0f - cw) / a0, b2 = b0;
    const float a1 = -2.0f * cw / a0, a2 = (1.0f - alpha) / a0;
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;

    const float left = level * std::sqrt(0.5f * (1.0f - pan));
    const float right = level * std::sqrt(0.5f * (1.0f + pan));
    // Swell in, wash out: S-curve fades, the tail longer than the attack.
    const float fade_in = (0.35f + 0.25f * unit(rng)) * kMixerRate;
    const float fade_out = (0.8f + 0.5f * unit(rng)) * kMixerRate;
    std::vector<float> stereo(static_cast<std::size_t>(out_n) * 2);
    for (int i = 0; i < out_n; ++i) {
        const float x = shifted[static_cast<std::size_t>(i)];
        const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x; y2 = y1; y1 = y;
        const float env = smoothstep(0.0f, fade_in, static_cast<float>(i)) *
                          smoothstep(0.0f, fade_out, static_cast<float>(out_n - 1 - i));
        stereo[static_cast<std::size_t>(i) * 2] = y * env * left;
        stereo[static_cast<std::size_t>(i) * 2 + 1] = y * env * right;
    }
    return stereo;
}

} // namespace

bool load_wav_stereo(const char* name, std::vector<float>& out) {
    std::vector<Uint8> bytes;
    if (!load_converted(name, AUDIO_F32SYS, 2, bytes)) return false;
    out.resize(bytes.size() / sizeof(float));
    std::memcpy(out.data(), bytes.data(), out.size() * sizeof(float));
    return true;
}

// Opens the device and loads the ambience up front (at startup, so neither
// the boot animation nor the first menu sound has to wait for it).
void mixer_open() { ensure_open(); }

void mixer_play_ui(const std::vector<float>* clip) {
    Mixer& m = g_mixer;
    if (clip == nullptr || clip->empty() || !ensure_open()) return;
    Voice voice;
    voice.clip = clip;
    SDL_LockAudioDevice(m.device);
    // Fade out whatever menu sound is still playing; its reverb tail rings on.
    for (Voice& v : m.ui) {
        if (v.fade_step <= 0.0f) v.fade_step = v.gain / (kUiCutSeconds * kMixerRate);
    }
    m.ui.push_back(voice);
    SDL_UnlockAudioDevice(m.device);
}

void ambience_set_active(bool active) {
    if (active && !ensure_open()) return;
    g_mixer.ambience_target.store(active ? 1.0f : 0.0f, std::memory_order_relaxed);
}

void ambience_update(float dt) {
    Mixer& m = g_mixer;
    if (m.device == 0 || m.static_source.empty()) return;
    if (m.ambience_target.load(std::memory_order_relaxed) <= 0.0f) return;
    m.next_wave_in -= dt;
    if (m.next_wave_in > 0.0f) return;

    Voice wave;
    wave.owned = shape_wave(m.static_source, m.rng);
    SDL_LockAudioDevice(m.device);
    m.waves.push_back(std::move(wave));
    SDL_UnlockAudioDevice(m.device);

    // Waves: mostly a few seconds apart, sometimes one right after another.
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    m.next_wave_in = unit(m.rng) < 0.2f ? 0.9f + 0.8f * unit(m.rng) : 3.0f + 5.0f * unit(m.rng);
}

void mixer_release() {
    Mixer& m = g_mixer;
    if (m.device != 0) {
        SDL_CloseAudioDevice(m.device);
        m.device = 0;
    }
    m.background.clear();
    m.static_source.clear();
    m.waves.clear();
    m.ui.clear();
    m.attempted = false;
    m.ambience_gain = 0.0f;
    m.ambience_target.store(0.0f);
}

} // namespace ps2::ui::vs2
