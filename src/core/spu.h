#pragma once
#include "types.h"
#include <SDL.h>
#include <array>
#include <vector>

// ── Sound Processing Unit (SPU) ────────────────────────────────────
// 24-channel ADPCM + CD-DA audio
// This is a functional stub that acknowledges register writes
// and produces silence. Full audio will be implemented later.

class System;

class Spu {
public:
  struct AudioDiag {
    u64 generated_frames = 0;
    u64 queued_frames = 0;
    u64 dropped_frames = 0;
    u64 overrun_events = 0;
    u64 capture_frames = 0;
    u64 reverb_mix_frames = 0;
    u64 reverb_ram_writes = 0;
    u64 key_on_events = 0;
    u64 key_off_events = 0;
    u64 end_flag_events = 0;
    u64 loop_end_events = 0;
    u64 nonloop_end_events = 0;
    u64 release_to_off_events = 0;
    u64 kon_retrigger_events = 0;
    u64 koff_high_env_events = 0;
    u64 release_samples_total = 0;
    u32 release_samples_min = 0;
    u32 release_samples_max = 0;
    u64 release_fast_events = 0;
    u64 active_voice_samples = 0;
    u64 active_voice_accum = 0;
    u32 active_voice_peak = 0;
    u64 clip_events_dry = 0;
    u64 clip_events_wet = 0;
    u64 clip_events_out = 0;
    u64 reverb_guard_events = 0;
    bool gaussian_active = false;
    bool reverb_enabled = false;
    bool saw_reverb_config_write = false;
    float peak_dry_l = 0.0f;
    float peak_dry_r = 0.0f;
    float peak_wet_l = 0.0f;
    float peak_wet_r = 0.0f;
    float peak_mix_l = 0.0f;
    float peak_mix_r = 0.0f;
    u32 queue_peak_bytes = 0;
    u32 queue_last_bytes = 0;
  };

  void init(System *sys);
  void shutdown();
  void reset();

  u16 read16(u32 offset) const;
  void write16(u32 offset, u16 value);
  void dma_write(u32 value);
  u32 dma_read();
  bool dma_request() const {
    const u16 transfer_mode = static_cast<u16>((spucnt_ >> 4) & 0x3u);
    return transfer_mode == 2u || transfer_mode == 3u;
  }
  void push_cd_audio_samples(const std::vector<s16> &samples, u32 sample_rate);

  // Produce audio samples (stub: silence)
  void tick(u32 cycles);

  // Status register
  u16 status() const { return spustat_; }
  const AudioDiag &audio_diag() const { return audio_diag_; }

  void set_audio_capture(bool enabled) { capture_enabled_ = enabled; }
  bool audio_capture_enabled() const { return capture_enabled_; }
  void clear_audio_capture();
  const std::vector<s16> &audio_capture_samples() const {
    return capture_samples_;
  }

private:
  static constexpr int SAMPLE_RATE = 44100;
  static constexpr int NUM_VOICES = 24;
  static constexpr int VOICE_REG_STRIDE = 0x10;
  static constexpr u32 SPU_RAM_MASK = 0x7FFFFu;
  static constexpr u32 SPU_RAM_WORD_MASK = 0x7FFFEu;
  static constexpr u32 HOST_TARGET_QUEUE_BYTES = 8192u;
  static constexpr u32 HOST_MAX_QUEUE_BYTES = 24576u;
  static constexpr size_t HOST_STAGING_MAX_SAMPLES =
      static_cast<size_t>(SAMPLE_RATE) * 2 * 4;
  static constexpr size_t CAPTURE_MAX_SAMPLES =
      static_cast<size_t>(SAMPLE_RATE) * 2 * 180;
  static constexpr size_t CD_INPUT_MAX_SAMPLES =
      static_cast<size_t>(SAMPLE_RATE) * 2 * 8;

  struct VoiceState {
    bool key_on = false;
    u32 addr = 0;
    u32 repeat_addr = 0;
    int sample_index = 28;
    bool end_reached = false;
    bool release_tracking = false;
    u64 release_start_sample = 0;
    u32 pitch_counter = 0; // 12.0 fixed-point sample counter
    std::array<s16, 28> decoded{};
    s16 hist1 = 0;
    s16 hist2 = 0;
    std::array<s16, 4> gauss_hist{};
    bool gauss_ready = false;
    s16 current_vol_l = 0;
    s16 current_vol_r = 0;
    s16 sweep_vol_l = 0; // per-voice volume sweep state (left)
    s16 sweep_vol_r = 0; // per-voice volume sweep state (right)

    // ADSR envelope
    enum class AdsrPhase { Off, Attack, Decay, Sustain, Release };
    AdsrPhase phase = AdsrPhase::Off;
    u16 env_level = 0; // 0..0x7FFF
    u16 sustain_level = 0;
    u16 adsr_counter = 0;
    u8 attack_shift = 0;
    u8 attack_step = 0;
    u8 decay_shift = 0;
    u8 sustain_shift = 0;
    u8 sustain_step = 0;
    u8 release_shift = 0;
    bool attack_exp = false;
    bool sustain_exp = false;
    bool sustain_decrease = false;
    bool release_exp = false;
  };

  struct ReverbRegs {
    u16 dAPF1 = 0;
    u16 dAPF2 = 0;
    s16 vIIR = 0;
    s16 vCOMB1 = 0;
    s16 vCOMB2 = 0;
    s16 vCOMB3 = 0;
    s16 vCOMB4 = 0;
    s16 vWALL = 0;
    s16 vAPF1 = 0;
    s16 vAPF2 = 0;
    s16 mLSAME = 0;
    s16 mRSAME = 0;
    s16 mLCOMB1 = 0;
    s16 mRCOMB1 = 0;
    s16 mLCOMB2 = 0;
    s16 mRCOMB2 = 0;
    s16 dLSAME = 0;
    s16 dRSAME = 0;
    s16 mLDIFF = 0;
    s16 mRDIFF = 0;
    s16 mLCOMB3 = 0;
    s16 mRCOMB3 = 0;
    s16 mLCOMB4 = 0;
    s16 mRCOMB4 = 0;
    s16 dLDIFF = 0;
    s16 dRDIFF = 0;
    s16 mLAPF1 = 0;
    s16 mRAPF1 = 0;
    s16 mLAPF2 = 0;
    s16 mRAPF2 = 0;
    s16 vLIN = 0;
    s16 vRIN = 0;
  };

  System *sys_ = nullptr;
  SDL_AudioDeviceID audio_device_ = 0;
  bool audio_enabled_ = false;
  bool capture_enabled_ = false;

  // SPU registers (0x1F801C00 - 0x1F801FFF = 0x400 bytes)
  std::array<u16, 0x200> regs_{};

  // Key SPU registers
  u16 spucnt_ = 0;  // SPU Control (0x1F801DAA)
  u16 spustat_ = 0; // SPU Status  (0x1F801DAE)
  u32 transfer_addr_ = 0;
  u32 transfer_busy_cycles_ = 0;
  u8 capture_half_ = 0;
  double sample_accum_ = 0.0;
  std::array<VoiceState, NUM_VOICES> voices_{};
  u32 reverb_on_mask_ = 0;
  u32 endx_mask_ = 0;
  s16 master_vol_l_ = 0x3FFF;
  s16 master_vol_r_ = 0x3FFF;
  s16 reverb_depth_l_ = 0;
  s16 reverb_depth_r_ = 0;
  s16 cd_vol_l_ = 0;
  s16 cd_vol_r_ = 0;
  s16 ext_vol_l_ = 0;
  s16 ext_vol_r_ = 0;
  u32 reverb_base_addr_ = 0;
  u32 reverb_cursor_ = 0;
  bool reverb_half_rate_phase_ = false;
  float reverb_hold_l_ = 0.0f;
  float reverb_hold_r_ = 0.0f;
  ReverbRegs reverb_regs_{};
  AudioDiag audio_diag_{};
  std::vector<s16> host_staging_samples_{};
  std::vector<s16> capture_samples_{};
  std::vector<s16> cd_input_samples_{};
  size_t cd_input_read_pos_ = 0;
  u64 sample_clock_ = 0;

  // SPU RAM (512KB)
  std::array<u8, 512 * 1024> spu_ram_{};

  bool decode_block(int voice);
  bool fetch_decoded_sample(int voice, s16 &sample);
  void prime_gaussian_history(int voice);
  void advance_gaussian_history(int voice);
  s16 gaussian_interpolate(const VoiceState &vs) const;
  s16 apply_envelope(s16 sample, const VoiceState &vs) const;
  float decode_volume(u16 raw) const;
  s32 decode_sweep_step(u16 raw, s16 current) const;
  void write_reverb_reg(u32 offset, u16 value);
  void tick_volume_sweep();
  void mix_reverb(float in_l, float in_r, float &wet_l, float &wet_r);
  u32 reverb_work_area_span() const;
  u32 reverb_wrap_addr(s32 addr) const;
  u32 reverb_addr_from_reg(s16 reg, s32 extra = 0) const;
  s16 reverb_read_s16(u32 addr) const;
  void reverb_write_s16(u32 addr, s16 value);
  static s16 sat16(s32 value);
  static s32 mul_q15(s32 a, s32 b);
  static float soft_clip(float value);
  void queue_host_audio(const std::vector<s16> &samples);
  void tick_adsr(VoiceState &vs);
  void key_on_voice(int voice);
  void key_off_voice(int voice);
  void apply_pending_key_strobes();

  u32 pending_kon_mask_ = 0;
  u32 pending_koff_mask_ = 0;
};
