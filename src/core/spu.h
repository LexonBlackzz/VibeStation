#pragma once

#include "types.h"
#include <SDL.h>
#include <algorithm>
#include <atomic>
#include <array>
#include <vector>

class System;

class Spu {
public:
  static constexpr u32 RAM_SIZE_BYTES = 512u * 1024u;
  struct AudioDiag {
    u64 generated_frames = 0;
    u64 queued_frames = 0;
    u64 dropped_frames = 0;
    u64 overrun_events = 0;
    u64 underrun_events = 0;
    u64 capture_frames = 0;
    u64 reverb_mix_frames = 0;
    u64 reverb_ram_writes = 0;
    u64 key_on_events = 0;
    u64 key_off_events = 0;
    u64 end_flag_events = 0;
    u64 loop_end_events = 0;
    u64 nonloop_end_events = 0;
    u64 off_due_to_end_flag = 0;
    u64 off_due_to_release_env0 = 0;
    u64 release_to_off_events = 0;
    u64 kon_retrigger_events = 0;
    u64 koff_high_env_events = 0;
    u64 kon_to_retrigger_events = 0;
    u64 kon_to_retrigger_total_samples = 0;
    u32 kon_to_retrigger_min_samples = 0;
    u32 kon_to_retrigger_max_samples = 0;
    u64 kon_to_koff_events = 0;
    u64 kon_to_koff_total_samples = 0;
    u32 kon_to_koff_min_samples = 0;
    u32 kon_to_koff_max_samples = 0;
    u32 kon_to_koff_min_voice = 0xFFFFFFFFu;
    u32 kon_to_koff_min_addr = 0;
    u16 kon_to_koff_min_pitch = 0;
    u16 kon_to_koff_min_adsr2 = 0;
    u64 koff_short_window_events = 0;

    // UI-visible KON/KOFF write aggregation diagnostics.
    u64 kon_write_events_low = 0;
    u64 kon_write_events_high = 0;
    u64 koff_write_events_low = 0;
    u64 koff_write_events_high = 0;
    u64 kon_bits_collected = 0;
    u64 koff_bits_collected = 0;
    u64 kon_multiwrite_same_sample_events = 0;
    u64 koff_multiwrite_same_sample_events = 0;

    u64 v16_kon_write_events = 0;
    u64 v16_koff_write_events = 0;
    u64 v16_kon_write_to_koff_events = 0;
    u64 v16_kon_write_to_koff_total_cpu_cycles = 0;
    u64 v16_kon_write_to_koff_min_cpu_cycles = 0;
    u64 v16_kon_write_to_koff_max_cpu_cycles = 0;
    u64 v16_kon_write_to_koff_total_samples = 0;
    u32 v16_kon_write_to_koff_min_samples = 0;
    u32 v16_kon_write_to_koff_max_samples = 0;
    u64 v16_kon_apply_events = 0;
    u64 v16_koff_apply_events = 0;
    u64 v16_kon_to_koff_apply_events = 0;
    u64 v16_kon_to_koff_apply_total_samples = 0;
    u32 v16_kon_to_koff_apply_min_samples = 0;
    u32 v16_kon_to_koff_apply_max_samples = 0;
    u64 v16_kon_reapply_without_koff = 0;
    u64 v16_koff_without_kon = 0;
    u64 v16_strobe_samples_with_kon = 0;
    u64 v16_strobe_samples_with_koff = 0;
    u64 v16_strobe_samples_with_both = 0;
    u64 key_write_unsynced_events = 0;
    u64 key_write_unsynced_max_cpu_lag = 0;
    u64 key_write_while_spu_disabled = 0;
    u64 keyon_ignored_while_disabled = 0;
    u64 keyoff_ignored_while_disabled = 0;
    u64 spucnt_enable_set_events = 0;
    u64 spucnt_enable_clear_events = 0;
    u64 spu_disable_forced_off_voices = 0;
    u64 spu_disable_span_events = 0;
    u64 spu_disable_span_total_samples = 0;
    u32 spu_disable_span_min_samples = 0;
    u32 spu_disable_span_max_samples = 0;
    u64 release_samples_total = 0;
    u32 release_samples_min = 0;
    u32 release_samples_max = 0;
    u64 release_fast_events = 0;
    u64 logical_voice_samples = 0;
    u64 logical_voice_accum = 0;
    u32 logical_voice_peak = 0;
    u64 logical_voice_peak_sample = 0;
    u64 logical_voice_peak_key_on_events = 0;
    u64 logical_voice_peak_key_off_events = 0;
    u32 logical_voice_peak_endx_mask = 0;
    u64 env_voice_samples = 0;
    u64 env_voice_accum = 0;
    u32 env_voice_peak = 0;
    u64 env_voice_peak_sample = 0;
    u64 env_voice_peak_key_on_events = 0;
    u64 env_voice_peak_key_off_events = 0;
    u32 env_voice_peak_endx_mask = 0;
    u64 audible_voice_samples = 0;
    u64 audible_voice_accum = 0;
    u32 audible_voice_peak = 0;
    u64 audible_voice_peak_sample = 0;
    u64 audible_voice_peak_key_on_events = 0;
    u64 audible_voice_peak_key_off_events = 0;
    u32 audible_voice_peak_endx_mask = 0;
    u64 active_voice_samples = 0;
    u64 active_voice_accum = 0;
    u32 active_voice_peak = 0;
    u64 voice_cap_frames = 0;
    u64 no_voice_frames = 0;
    u64 muted_output_frames = 0;
    u64 cd_frames_mixed = 0;
    u64 spucnt_writes = 0;
    u64 spucnt_mute_toggle_events = 0;
    u64 spucnt_mute_set_events = 0;
    u64 spucnt_mute_clear_events = 0;
    u16 spucnt_last = 0;
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
  void corrupt_ram_byte(u32 offset, u8 value);
  void corrupt_runtime_state(u32 selector, u32 value);
  void dma_write(u32 value);
  u32 dma_read();
  bool dma_request() const;
  void push_cd_audio_samples(const std::vector<s16> &samples, u32 sample_rate);
  void set_output_speed(double speed) {
    const double clamped = std::max(0.25, std::min(speed, 4.0));
    audio_output_speed_.store(clamped, std::memory_order_release);
  }

  void tick(u32 cycles);
  void mark_synced_to_cpu(u64 cpu_cycle) { last_synced_cpu_cycle_ = cpu_cycle; }

  u16 status() const { return spustat_; }
  const AudioDiag &audio_diag() const { return audio_diag_; }
  void reset_audio_diag() { audio_diag_ = AudioDiag{}; }

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
  static constexpr int TRACE_VOICE = 16;
  static constexpr u32 KOFF_DEBOUNCE_SAMPLES = 64u;
  static constexpr u32 SPU_RAM_MASK = 0x7FFFFu;
  static constexpr u32 SPU_RAM_WORD_MASK = 0x7FFFEu;
  static constexpr u32 SPUCNT_MODE_APPLY_DELAY_CYCLES = 0x100u;
  static constexpr u32 HOST_TARGET_QUEUE_BYTES_MIN = 16384u;
  static constexpr u32 HOST_MAX_QUEUE_BYTES_MIN = 65536u;
  static constexpr size_t HOST_STAGING_MAX_SAMPLES =
      static_cast<size_t>(SAMPLE_RATE) * 2 * 4;
  static constexpr size_t CAPTURE_MAX_SAMPLES =
      static_cast<size_t>(SAMPLE_RATE) * 2 * 180;
  static constexpr size_t CD_INPUT_MAX_SAMPLES =
      static_cast<size_t>(SAMPLE_RATE) * 2 * 8;

  struct VoiceState {
    enum class AdsrPhase { Off, Attack, Decay, Sustain, Release };

    bool key_on = false;
    u32 addr = 0;
    u32 repeat_addr = 0;
    u32 last_block_addr = 0;
    u8 last_adpcm_flags = 0;
    int sample_index = 28;
    bool stop_after_block = false;
    u32 pitch_counter = 0; // 12.12 fixed-point

    std::array<s16, 28> decoded{};
    s16 hist1 = 0;
    s16 hist2 = 0;
    std::array<s16, 4> gauss_hist{};
    bool gauss_ready = false;

    s16 current_vol_l = 0;
    s16 current_vol_r = 0;
    s16 sweep_vol_l = 0;
    s16 sweep_vol_r = 0;

    AdsrPhase phase = AdsrPhase::Off;
    u16 env_level = 0;
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

    bool release_tracking = false;
    u64 release_start_sample = 0;
  };

  struct ReverbRegs {
    std::array<u16, 32> raw = {};
    u16 d_apf1 = 0;
    u16 d_apf2 = 0;
    s16 v_iir = 0;
    s16 v_comb1 = 0;
    s16 v_comb2 = 0;
    s16 v_comb3 = 0;
    s16 v_comb4 = 0;
    s16 v_wall = 0;
    s16 v_apf1 = 0;
    s16 v_apf2 = 0;
    u16 m_lsame = 0;
    u16 m_rsame = 0;
    u16 m_lcomb1 = 0;
    u16 m_rcomb1 = 0;
    u16 m_lcomb2 = 0;
    u16 m_rcomb2 = 0;
    u16 d_lsame = 0;
    u16 d_rsame = 0;
    u16 m_ldiff = 0;
    u16 m_rdiff = 0;
    u16 m_lcomb3 = 0;
    u16 m_rcomb3 = 0;
    u16 m_lcomb4 = 0;
    u16 m_rcomb4 = 0;
    u16 d_ldiff = 0;
    u16 d_rdiff = 0;
    u16 m_lapf1 = 0;
    u16 m_rapf1 = 0;
    u16 m_lapf2 = 0;
    u16 m_rapf2 = 0;
    s16 v_lin = 0;
    s16 v_rin = 0;
  };

  struct ReverbState {
    bool process_right = false;
    u32 cursor = 0;
    std::array<s16, 39> in_hist_l = {};
    std::array<s16, 39> in_hist_r = {};
    std::array<s16, 39> out_hist_l = {};
    std::array<s16, 39> out_hist_r = {};
    s16 last_in_l = 0;
    s16 last_in_r = 0;
  };

  System *sys_ = nullptr;
  SDL_AudioDeviceID audio_device_ = 0;
  bool audio_enabled_ = false;
  bool audio_started_ = false;
  bool capture_enabled_ = false;
  u32 host_buffer_bytes_ = 0;
  u32 opened_audio_samples_ = 0;
  u32 host_target_queue_bytes_ = HOST_TARGET_QUEUE_BYTES_MIN;
  u32 host_max_queue_bytes_ = HOST_MAX_QUEUE_BYTES_MIN;

  std::array<u16, 0x200> regs_ = {};
  std::array<u8, 512 * 1024> spu_ram_ = {};
  std::array<VoiceState, NUM_VOICES> voices_ = {};

  u16 spucnt_ = 0;
  u16 spustat_ = 0;
  u16 spucnt_mode_latched_ = 0;
  u16 spucnt_mode_pending_ = 0;
  u32 spucnt_mode_delay_cycles_ = 0;
  u32 irq_addr_ = 0;
  u32 transfer_addr_ = 0;
  u32 transfer_busy_cycles_ = 0;
  u8 capture_half_ = 0;

  u32 pitch_mod_mask_ = 0;
  u32 noise_on_mask_ = 0;
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
  ReverbRegs reverb_regs_ = {};
  ReverbState reverb_state_ = {};

  double sample_accum_ = 0.0;
  u64 sample_clock_ = 0;
  u64 last_synced_cpu_cycle_ = 0;
  std::atomic<double> audio_output_speed_{1.0};

  s16 noise_level_ = 1;
  s32 noise_timer_ = 0;

  AudioDiag audio_diag_ = {};
  std::vector<s16> host_staging_samples_;
  size_t host_staging_read_pos_ = 0;
  std::vector<s16> mix_buffer_;
  std::vector<s16> host_silence_samples_;
  std::vector<s16> capture_samples_;
  std::vector<s16> cd_input_samples_;
  size_t cd_input_read_pos_ = 0;
  s16 cd_last_sample_l_ = 0;
  s16 cd_last_sample_r_ = 0;
  bool cd_stream_started_ = false;
  u32 cd_gap_ramp_samples_ = 0;
  bool cd_gap_active_ = false;
  u32 cd_rejoin_blend_samples_ = 0;
  s16 cd_rejoin_from_l_ = 0;
  s16 cd_rejoin_from_r_ = 0;
  double cd_resample_src_pos_ = 1.0;
  u32 cd_resample_in_rate_ = SAMPLE_RATE;
  bool cd_resample_prev_valid_ = false;
  s16 cd_resample_prev_l_ = 0;
  s16 cd_resample_prev_r_ = 0;
  double turbo_resample_src_pos_ = 0.0;
  u32 turbo_resample_in_rate_ = SAMPLE_RATE;
  bool turbo_resample_prev_valid_ = false;
  s16 turbo_resample_prev_l_ = 0;
  s16 turbo_resample_prev_r_ = 0;

  std::array<u64, NUM_VOICES> last_kon_sample_ = {};
  std::array<bool, NUM_VOICES> has_last_kon_ = {};

  u64 v16_last_kon_write_cpu_cycle_ = 0;
  u64 v16_last_kon_write_sample_ = 0;
  bool v16_has_last_kon_write_ = false;
  u64 v16_last_kon_apply_sample_ = 0;
  bool v16_has_last_kon_apply_ = false;
  bool v16_waiting_for_koff_ = false;

  bool spu_disabled_window_active_ = false;
  u64 spu_disable_start_sample_ = 0;

  u32 pending_kon_mask_ = 0;
  u32 pending_koff_mask_ = 0;

  u64 last_kon_write_sample_ = 0;
  bool has_last_kon_write_sample_ = false;
  u64 last_koff_write_sample_ = 0;
  bool has_last_koff_write_sample_ = false;

  static s16 sat16(s32 value);
  static s32 mul_q15(s32 a, s32 b);
  static float q15_to_float(s16 value);
  static u32 popcount32(u32 value);

  bool decode_adpcm_block(int voice);
  bool fetch_voice_sample(int voice, s16 &sample);
  void seed_gaussian_history(int voice);
  void shift_gaussian_history(int voice);
  s16 interpolate_gaussian(const VoiceState &voice) const;
  s16 apply_envelope(s16 sample, const VoiceState &voice) const;

  s32 decode_sweep_step(u16 raw, s16 current) const;
  void tick_global_sweeps();
  void tick_adsr(int voice, VoiceState &voice_state);

  void key_on_voice(int voice);
  void key_off_voice(int voice);
  void force_off_all_voices_immediate();
  void apply_pending_key_strobes();

  s16 next_noise_sample();

  void write_reverb_reg(u32 offset, u16 value);
  void decode_reverb_regs();
  u32 reverb_work_size_bytes() const;
  u32 reverb_addr_from_reg(u16 reg, s32 delta_bytes);
  s16 read_spu_s16(u32 addr);
  void write_spu_s16(u32 addr, s16 value, bool reverb_write);
  s16 fir39_q15(const std::array<s16, 39> &history) const;
  s16 step_reverb_channel(bool right_channel, s16 lin, s16 rin,
                          bool writes_enabled, bool same_diff_enabled);
  std::array<float, 2> step_reverb(float send_l, float send_r, u16 spucnt_eff);
  void queue_host_audio(const std::vector<s16> &samples);

  u16 spucnt_effective() const;
  void tick_spucnt_mode_delay(u32 cycles);
  void clear_irq9_flag();
  void maybe_raise_irq9_for_ram_access(u32 start_addr, u32 byte_count);
};
