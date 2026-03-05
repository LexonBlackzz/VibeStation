#include "spu.h"
#include "system.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>

namespace {
u64 g_spu_trace_reg_counter = 0;
u64 g_spu_trace_dma_write_counter = 0;
u64 g_spu_trace_dma_read_counter = 0;
constexpr s32 kEnvMax = 0x7FFF;

constexpr std::array<s16, 512> kGaussTable = {
#include "spu_gauss_table.inc"
};

constexpr std::array<s16, 39> kReverbFirTable = {
    -1,    0,     2,    0,     -10,   0,     35,   0,     -103, 0,
    266,   0,     -616, 0,     1332,  0,     -2960, 0,    10246, 16384,
    10246, 0,     -2960, 0,    1332,  0,     -616, 0,     266,  0,
    -103,  0,     35,   0,     -10,   0,     2,    0,     -1,
};

void push_history(std::array<s16, 39> &history, s16 sample) {
  for (size_t i = 0; i + 1 < history.size(); ++i) {
    history[i] = history[i + 1];
  }
  history[history.size() - 1] = sample;
}

inline s16 decode_fixed_volume_q15(u16 raw) {
  s32 value = static_cast<s32>(raw & 0x7FFFu);
  if ((value & 0x4000) != 0) {
    value -= 0x8000;
  }
  value *= 2;
  if (value < -32768) {
    value = -32768;
  } else if (value > 32767) {
    value = 32767;
  }
  return static_cast<s16>(value);
}

inline s32 adsr_step_base(u8 step, bool decreasing) {
  s32 base = 7 - static_cast<s32>(step & 0x3u);
  return decreasing ? ~base : base;
}

std::vector<s16> resample_stereo_linear(const std::vector<s16> &input,
                                        u32 in_rate, u32 out_rate) {
  if (input.empty() || in_rate == 0 || out_rate == 0) {
    return {};
  }
  if ((input.size() & 1u) != 0) {
    return {};
  }
  if (in_rate == out_rate) {
    return input;
  }

  const size_t in_frames = input.size() / 2;
  if (in_frames < 2) {
    return input;
  }

  const double ratio =
      static_cast<double>(in_rate) / static_cast<double>(out_rate);
  const size_t out_frames = std::max<size_t>(
      1, static_cast<size_t>(std::llround(static_cast<double>(in_frames) / ratio)));

  std::vector<s16> output(out_frames * 2);
  for (size_t i = 0; i < out_frames; ++i) {
    const double src_pos = static_cast<double>(i) * ratio;
    size_t idx0 = static_cast<size_t>(src_pos);
    if (idx0 >= in_frames) {
      idx0 = in_frames - 1;
    }
    const size_t idx1 = std::min(idx0 + 1, in_frames - 1);
    const double frac =
        std::clamp(src_pos - static_cast<double>(idx0), 0.0, 1.0);

    const s16 l0 = input[idx0 * 2 + 0];
    const s16 r0 = input[idx0 * 2 + 1];
    const s16 l1 = input[idx1 * 2 + 0];
    const s16 r1 = input[idx1 * 2 + 1];

    const s32 l = static_cast<s32>(std::lround(
        static_cast<double>(l0) + (static_cast<double>(l1 - l0) * frac)));
    const s32 r = static_cast<s32>(std::lround(
        static_cast<double>(r0) + (static_cast<double>(r1 - r0) * frac)));

    output[i * 2 + 0] = static_cast<s16>(std::clamp(l, -32768, 32767));
    output[i * 2 + 1] = static_cast<s16>(std::clamp(r, -32768, 32767));
  }

  return output;
}
} // namespace

void Spu::init(System *sys) {
  sys_ = sys;
  if (audio_device_ != 0) {
    return;
  }
  if ((SDL_WasInit(SDL_INIT_AUDIO) & SDL_INIT_AUDIO) == 0) {
    return;
  }

  SDL_AudioSpec desired{};
  desired.freq = SAMPLE_RATE;
  desired.format = AUDIO_S16SYS;
  desired.channels = 2;
  const u32 requested_samples = std::clamp<u32>(g_spu_desired_samples, 64u, 65535u);
  desired.samples = static_cast<Uint16>(requested_samples);
  desired.callback = nullptr;

  SDL_AudioSpec obtained{};
  audio_device_ = SDL_OpenAudioDevice(nullptr, 0, &desired, &obtained, 0);
  if (audio_device_ == 0) {
    LOG_WARN("SPU: Failed to open audio device: %s", SDL_GetError());
    audio_enabled_ = false;
    return;
  }
  if (obtained.format != AUDIO_S16SYS || obtained.channels != 2) {
    LOG_WARN("SPU: Unsupported host audio format, disabling output.");
    SDL_CloseAudioDevice(audio_device_);
    audio_device_ = 0;
    audio_enabled_ = false;
    return;
  }

  host_buffer_bytes_ = static_cast<u32>(obtained.samples) *
                       static_cast<u32>(obtained.channels) * sizeof(s16);
  host_target_queue_bytes_ =
      std::max(HOST_TARGET_QUEUE_BYTES_MIN, host_buffer_bytes_ * 2u);
  host_max_queue_bytes_ =
      std::max(HOST_MAX_QUEUE_BYTES_MIN, host_buffer_bytes_ * 4u);
  if (host_max_queue_bytes_ <= host_target_queue_bytes_) {
    host_max_queue_bytes_ = host_target_queue_bytes_ + host_buffer_bytes_;
  }

  audio_enabled_ = true;
  audio_started_ = false;
  opened_audio_samples_ = requested_samples;
  SDL_PauseAudioDevice(audio_device_, 1);
  LOG_INFO("SPU: Audio initialized (%d Hz, device samples=%u, queue target/max=%u/%u bytes)",
           obtained.freq, static_cast<unsigned>(obtained.samples),
           static_cast<unsigned>(host_target_queue_bytes_),
           static_cast<unsigned>(host_max_queue_bytes_));
}

void Spu::shutdown() {
  if (audio_device_ != 0) {
    SDL_CloseAudioDevice(audio_device_);
    audio_device_ = 0;
  }
  audio_enabled_ = false;
  audio_started_ = false;
  host_buffer_bytes_ = 0;
  opened_audio_samples_ = 0;
  host_target_queue_bytes_ = HOST_TARGET_QUEUE_BYTES_MIN;
  host_max_queue_bytes_ = HOST_MAX_QUEUE_BYTES_MIN;
}

void Spu::clear_audio_capture() { capture_samples_.clear(); }

void Spu::reset() {
  const u32 requested_samples = std::clamp<u32>(g_spu_desired_samples, 64u, 65535u);
  if (audio_device_ != 0 && requested_samples != opened_audio_samples_) {
    shutdown();
    init(sys_);
  }
  if (audio_device_ != 0) {
    SDL_PauseAudioDevice(audio_device_, 1);
    SDL_ClearQueuedAudio(audio_device_);
    audio_started_ = false;
  }

  regs_.fill(0);
  spu_ram_.fill(0);
  voices_ = {};

  spucnt_ = 0;
  spustat_ = 0;
  spucnt_mode_latched_ = 0;
  spucnt_mode_pending_ = 0;
  spucnt_mode_delay_cycles_ = 0;
  irq_addr_ = 0;
  transfer_addr_ = 0;
  transfer_busy_cycles_ = 0;
  capture_half_ = 0;
  pitch_mod_mask_ = 0;
  noise_on_mask_ = 0;
  reverb_on_mask_ = 0;
  endx_mask_ = 0;
  master_vol_l_ = 0x3FFF;
  master_vol_r_ = 0x3FFF;
  reverb_depth_l_ = 0;
  reverb_depth_r_ = 0;
  cd_vol_l_ = 0;
  cd_vol_r_ = 0;
  ext_vol_l_ = 0;
  ext_vol_r_ = 0;
  reverb_base_addr_ = 0;
  reverb_regs_ = {};
  reverb_state_ = {};
  sample_accum_ = 0.0;
  sample_clock_ = 0;
  last_synced_cpu_cycle_ = 0;
  noise_level_ = 1;
  noise_timer_ = 0;

  audio_diag_ = {};
  host_staging_samples_.clear();
  host_staging_read_pos_ = 0;
  mix_buffer_.clear();
  host_silence_samples_.clear();
  capture_samples_.clear();
  cd_input_samples_.clear();
  cd_input_read_pos_ = 0;

  last_kon_sample_.fill(0);
  has_last_kon_.fill(false);
  v16_last_kon_write_cpu_cycle_ = 0;
  v16_last_kon_write_sample_ = 0;
  v16_has_last_kon_write_ = false;
  v16_last_kon_apply_sample_ = 0;
  v16_has_last_kon_apply_ = false;
  v16_waiting_for_koff_ = false;
  spu_disabled_window_active_ = false;
  spu_disable_start_sample_ = 0;
  pending_kon_mask_ = 0;
  pending_koff_mask_ = 0;
  last_kon_write_sample_ = 0;
  has_last_kon_write_sample_ = false;
  last_koff_write_sample_ = 0;
  has_last_koff_write_sample_ = false;
}

u16 Spu::spucnt_effective() const {
  return static_cast<u16>((spucnt_ & 0xFFC0u) | (spucnt_mode_latched_ & 0x003Fu));
}

bool Spu::dma_request() const {
  const u16 mode = static_cast<u16>((spucnt_effective() >> 4) & 0x3u);
  return mode == 2u || mode == 3u;
}

void Spu::tick_spucnt_mode_delay(u32 cycles) {
  if (spucnt_mode_delay_cycles_ == 0) {
    return;
  }
  if (cycles >= spucnt_mode_delay_cycles_) {
    spucnt_mode_delay_cycles_ = 0;
    spucnt_mode_latched_ = spucnt_mode_pending_;
  } else {
    spucnt_mode_delay_cycles_ -= cycles;
  }
}

void Spu::clear_irq9_flag() { spustat_ &= static_cast<u16>(~0x0040u); }

void Spu::maybe_raise_irq9_for_ram_access(u32 start_addr, u32 byte_count) {
  if ((spucnt_effective() & 0x0040u) == 0) {
    return;
  }
  if ((spustat_ & 0x0040u) != 0) {
    return;
  }

  const u32 irq = irq_addr_ & SPU_RAM_WORD_MASK;
  for (u32 i = 0; i < byte_count; ++i) {
    const u32 a = (start_addr + i) & SPU_RAM_WORD_MASK;
    if (a == irq) {
      spustat_ |= 0x0040u;
      if (sys_ != nullptr) {
        sys_->irq().request(Interrupt::SPU);
      }
      return;
    }
  }
}

u16 Spu::read16(u32 offset) const {
  switch (offset) {
  case 0x188:
  case 0x18A:
  case 0x18C:
  case 0x18E:
    return 0;
  case 0x19C:
    return static_cast<u16>(endx_mask_ & 0xFFFFu);
  case 0x19E:
    return static_cast<u16>((endx_mask_ >> 16) & 0x00FFu);
  case 0x1AA:
    return spucnt_;
  case 0x1AE: {
    const u16 spucnt_eff = spucnt_effective();
    u16 stat = static_cast<u16>((spucnt_eff & 0x3Fu) | (spustat_ & 0x0040u));
    const u16 transfer_mode = static_cast<u16>((spucnt_eff >> 4) & 0x3u);
    stat |= static_cast<u16>(((spucnt_eff >> 5) & 0x1u) << 7);
    stat |= static_cast<u16>(transfer_mode == 2u ? 1u : 0u) << 8;
    stat |= static_cast<u16>(transfer_mode == 3u ? 1u : 0u) << 9;
    stat |= static_cast<u16>(transfer_busy_cycles_ > 0u ? 1u : 0u) << 10;
    stat |= static_cast<u16>(capture_half_ & 0x1u) << 11;
    return stat;
  }
  default:
    break;
  }

  if (offset < 0x180u && (offset % VOICE_REG_STRIDE) == 0xCu) {
    const int voice = static_cast<int>(offset / VOICE_REG_STRIDE);
    if (voice >= 0 && voice < NUM_VOICES) {
      return voices_[voice].env_level;
    }
  }

  if (offset >= 0x200u && offset < 0x260u) {
    const u32 rel = offset - 0x200u;
    const int voice = static_cast<int>(rel >> 2);
    const u32 lane = rel & 0x3u;
    if (voice >= 0 && voice < NUM_VOICES) {
      if (lane == 0u) {
        return static_cast<u16>(voices_[voice].current_vol_l);
      }
      if (lane == 2u) {
        return static_cast<u16>(voices_[voice].current_vol_r);
      }
    }
    return 0;
  }

  if (offset < 0x400u) {
    return regs_[offset / 2u];
  }

  LOG_WARN("SPU: Unhandled read16 offset=0x%X", offset);
  return 0;
}

void Spu::decode_reverb_regs() {
  auto u = [&](size_t index) -> u16 { return reverb_regs_.raw[index]; };
  auto s = [&](size_t index) -> s16 { return static_cast<s16>(reverb_regs_.raw[index]); };

  reverb_regs_.d_apf1 = u(0);
  reverb_regs_.d_apf2 = u(1);
  reverb_regs_.v_iir = s(2);
  reverb_regs_.v_comb1 = s(3);
  reverb_regs_.v_comb2 = s(4);
  reverb_regs_.v_comb3 = s(5);
  reverb_regs_.v_comb4 = s(6);
  reverb_regs_.v_wall = s(7);
  reverb_regs_.v_apf1 = s(8);
  reverb_regs_.v_apf2 = s(9);
  reverb_regs_.m_lsame = u(10);
  reverb_regs_.m_rsame = u(11);
  reverb_regs_.m_lcomb1 = u(12);
  reverb_regs_.m_rcomb1 = u(13);
  reverb_regs_.m_lcomb2 = u(14);
  reverb_regs_.m_rcomb2 = u(15);
  reverb_regs_.d_lsame = u(16);
  reverb_regs_.d_rsame = u(17);
  reverb_regs_.m_ldiff = u(18);
  reverb_regs_.m_rdiff = u(19);
  reverb_regs_.m_lcomb3 = u(20);
  reverb_regs_.m_rcomb3 = u(21);
  reverb_regs_.m_lcomb4 = u(22);
  reverb_regs_.m_rcomb4 = u(23);
  reverb_regs_.d_ldiff = u(24);
  reverb_regs_.d_rdiff = u(25);
  reverb_regs_.m_lapf1 = u(26);
  reverb_regs_.m_rapf1 = u(27);
  reverb_regs_.m_lapf2 = u(28);
  reverb_regs_.m_rapf2 = u(29);
  reverb_regs_.v_lin = s(30);
  reverb_regs_.v_rin = s(31);
}

void Spu::write_reverb_reg(u32 offset, u16 value) {
  if (offset < 0x1C0u || offset > 0x1FEu || (offset & 1u) != 0u) {
    return;
  }
  const size_t index = static_cast<size_t>((offset - 0x1C0u) / 2u);
  if (index < reverb_regs_.raw.size()) {
    reverb_regs_.raw[index] = value;
    decode_reverb_regs();
  }
  audio_diag_.saw_reverb_config_write = true;
}
void Spu::write16(u32 offset, u16 value) {
  if (g_trace_spu) {
    if ((offset >= 0x180u ||
         (offset < 0x180u && (offset % VOICE_REG_STRIDE) == 0u)) &&
        trace_should_log(g_spu_trace_reg_counter, g_trace_burst_spu,
                         g_trace_stride_spu)) {
      LOG_DEBUG("SPU: W16 off=0x%03X val=0x%04X", offset, value);
    }
  }

  if (offset < 0x400u) {
    if (offset == 0x188u || offset == 0x18Au || offset == 0x18Cu ||
        offset == 0x18Eu) {
      regs_[offset / 2u] = 0;
    } else {
      regs_[offset / 2u] = value;
    }
  }

  if (offset < 0x180u && (offset % VOICE_REG_STRIDE) == 0xEu) {
    const int voice = static_cast<int>(offset / VOICE_REG_STRIDE);
    if (voice >= 0 && voice < NUM_VOICES) {
      voices_[voice].repeat_addr =
          (static_cast<u32>(value) * 8u) & static_cast<u32>(SPU_RAM_MASK);
    }
  }

  const u64 cpu_cycle = (sys_ != nullptr) ? sys_->cpu().cycle_count() : 0;
  auto note_key_write_timing = [&]() {
    const u64 lag = (cpu_cycle > last_synced_cpu_cycle_)
                        ? (cpu_cycle - last_synced_cpu_cycle_)
                        : 0u;
    if (lag != 0u) {
      ++audio_diag_.key_write_unsynced_events;
      audio_diag_.key_write_unsynced_max_cpu_lag =
          std::max(audio_diag_.key_write_unsynced_max_cpu_lag, lag);
    }
    if ((spucnt_effective() & 0x8000u) == 0) {
      ++audio_diag_.key_write_while_spu_disabled;
    }
  };

  auto note_kon_write = [&](bool high_lane, u16 raw_value) {
    if (high_lane) {
      ++audio_diag_.kon_write_events_high;
    } else {
      ++audio_diag_.kon_write_events_low;
    }
    if (has_last_kon_write_sample_ && last_kon_write_sample_ == sample_clock_) {
      ++audio_diag_.kon_multiwrite_same_sample_events;
    }
    has_last_kon_write_sample_ = true;
    last_kon_write_sample_ = sample_clock_;
    const u32 lane_bits =
        high_lane ? static_cast<u32>(raw_value & 0x00FFu)
                  : static_cast<u32>(raw_value);
    audio_diag_.kon_bits_collected += popcount32(lane_bits);
  };

  auto note_koff_write = [&](bool high_lane, u16 raw_value) {
    if (high_lane) {
      ++audio_diag_.koff_write_events_high;
    } else {
      ++audio_diag_.koff_write_events_low;
    }
    if (has_last_koff_write_sample_ && last_koff_write_sample_ == sample_clock_) {
      ++audio_diag_.koff_multiwrite_same_sample_events;
    }
    has_last_koff_write_sample_ = true;
    last_koff_write_sample_ = sample_clock_;
    const u32 lane_bits =
        high_lane ? static_cast<u32>(raw_value & 0x00FFu)
                  : static_cast<u32>(raw_value);
    audio_diag_.koff_bits_collected += popcount32(lane_bits);
  };

  switch (offset) {
  case 0x180:
    master_vol_l_ = decode_fixed_volume_q15(value);
    break;
  case 0x182:
    master_vol_r_ = decode_fixed_volume_q15(value);
    break;
  case 0x184:
    reverb_depth_l_ = decode_fixed_volume_q15(value);
    audio_diag_.saw_reverb_config_write = true;
    break;
  case 0x186:
    reverb_depth_r_ = decode_fixed_volume_q15(value);
    audio_diag_.saw_reverb_config_write = true;
    break;
  case 0x188:
    note_kon_write(false, value);
    if (value != 0u) {
      note_key_write_timing();
      if ((spucnt_effective() & 0x8000u) == 0u) {
        ++audio_diag_.keyon_ignored_while_disabled;
        break;
      }
    }
    pending_kon_mask_ =
        (pending_kon_mask_ & 0x00FF0000u) | static_cast<u32>(value);
    break;
  case 0x18A:
    note_kon_write(true, value);
    if ((value & 0x00FFu) != 0u) {
      note_key_write_timing();
      if ((value & 0x0001u) != 0u) {
        ++audio_diag_.v16_kon_write_events;
        v16_last_kon_write_cpu_cycle_ = cpu_cycle;
        v16_last_kon_write_sample_ = sample_clock_;
        v16_has_last_kon_write_ = true;
      }
      if ((spucnt_effective() & 0x8000u) == 0u) {
        ++audio_diag_.keyon_ignored_while_disabled;
        break;
      }
    }
    pending_kon_mask_ = (pending_kon_mask_ & 0x0000FFFFu) |
                        (static_cast<u32>(value & 0x00FFu) << 16);
    break;
  case 0x18C:
    note_koff_write(false, value);
    if (value != 0u) {
      note_key_write_timing();
      if ((spucnt_effective() & 0x8000u) == 0u) {
        ++audio_diag_.keyoff_ignored_while_disabled;
        break;
      }
    }
    pending_koff_mask_ =
        (pending_koff_mask_ & 0x00FF0000u) | static_cast<u32>(value);
    break;
  case 0x18E:
    note_koff_write(true, value);
    if ((value & 0x00FFu) != 0u) {
      note_key_write_timing();
      if ((value & 0x0001u) != 0u) {
        ++audio_diag_.v16_koff_write_events;
        if (v16_has_last_kon_write_) {
          const u64 delta_cycles = cpu_cycle - v16_last_kon_write_cpu_cycle_;
          const u64 delta_samples = sample_clock_ - v16_last_kon_write_sample_;
          const u32 delta_samples32 =
              (delta_samples > static_cast<u64>(std::numeric_limits<u32>::max()))
                  ? std::numeric_limits<u32>::max()
                  : static_cast<u32>(delta_samples);
          ++audio_diag_.v16_kon_write_to_koff_events;
          audio_diag_.v16_kon_write_to_koff_total_cpu_cycles += delta_cycles;
          audio_diag_.v16_kon_write_to_koff_total_samples += delta_samples;
          if (audio_diag_.v16_kon_write_to_koff_events == 1) {
            audio_diag_.v16_kon_write_to_koff_min_cpu_cycles = delta_cycles;
            audio_diag_.v16_kon_write_to_koff_max_cpu_cycles = delta_cycles;
            audio_diag_.v16_kon_write_to_koff_min_samples = delta_samples32;
            audio_diag_.v16_kon_write_to_koff_max_samples = delta_samples32;
          } else {
            audio_diag_.v16_kon_write_to_koff_min_cpu_cycles = std::min(
                audio_diag_.v16_kon_write_to_koff_min_cpu_cycles, delta_cycles);
            audio_diag_.v16_kon_write_to_koff_max_cpu_cycles = std::max(
                audio_diag_.v16_kon_write_to_koff_max_cpu_cycles, delta_cycles);
            audio_diag_.v16_kon_write_to_koff_min_samples = std::min(
                audio_diag_.v16_kon_write_to_koff_min_samples, delta_samples32);
            audio_diag_.v16_kon_write_to_koff_max_samples = std::max(
                audio_diag_.v16_kon_write_to_koff_max_samples, delta_samples32);
          }
        }
      }
      if ((spucnt_effective() & 0x8000u) == 0u) {
        ++audio_diag_.keyoff_ignored_while_disabled;
        break;
      }
    }
    pending_koff_mask_ = (pending_koff_mask_ & 0x0000FFFFu) |
                         (static_cast<u32>(value & 0x00FFu) << 16);
    break;
  case 0x190:
    pitch_mod_mask_ = (pitch_mod_mask_ & 0x00FF0000u) | static_cast<u32>(value);
    break;
  case 0x192:
    pitch_mod_mask_ = (pitch_mod_mask_ & 0x0000FFFFu) |
                      (static_cast<u32>(value & 0x00FFu) << 16);
    break;
  case 0x194:
    noise_on_mask_ = (noise_on_mask_ & 0x00FF0000u) | static_cast<u32>(value);
    break;
  case 0x196:
    noise_on_mask_ = (noise_on_mask_ & 0x0000FFFFu) |
                     (static_cast<u32>(value & 0x00FFu) << 16);
    break;
  case 0x198:
    reverb_on_mask_ = (reverb_on_mask_ & 0x00FF0000u) | static_cast<u32>(value);
    break;
  case 0x19A:
    reverb_on_mask_ = (reverb_on_mask_ & 0x0000FFFFu) |
                      (static_cast<u32>(value & 0x00FFu) << 16);
    break;
  case 0x1A2:
    reverb_base_addr_ =
        (static_cast<u32>(value) * 8u) & static_cast<u32>(SPU_RAM_WORD_MASK);
    if (const u32 work_size = reverb_work_size_bytes(); work_size > 0u) {
      reverb_state_.cursor %= work_size;
    } else {
      reverb_state_.cursor = 0;
    }
    audio_diag_.saw_reverb_config_write = true;
    break;
  case 0x1A4:
    irq_addr_ =
        (static_cast<u32>(value) * 8u) & static_cast<u32>(SPU_RAM_WORD_MASK);
    break;
  case 0x1A6:
    transfer_addr_ =
        (static_cast<u32>(value) * 8u) & static_cast<u32>(SPU_RAM_MASK);
    transfer_busy_cycles_ = 0;
    break;
  case 0x1A8:
    maybe_raise_irq9_for_ram_access(transfer_addr_, 2);
    spu_ram_[transfer_addr_ & SPU_RAM_MASK] = static_cast<u8>(value & 0x00FFu);
    spu_ram_[(transfer_addr_ + 1u) & SPU_RAM_MASK] =
        static_cast<u8>((value >> 8) & 0x00FFu);
    transfer_addr_ = (transfer_addr_ + 2u) & static_cast<u32>(SPU_RAM_MASK);
    transfer_busy_cycles_ = 64;
    break;
  case 0x1AA: {
    const bool was_enabled = (spucnt_ & 0x8000u) != 0u;
    const bool now_enabled = (value & 0x8000u) != 0u;
    if (was_enabled != now_enabled) {
      if (now_enabled) {
        ++audio_diag_.spucnt_enable_set_events;
        if (spu_disabled_window_active_) {
          const u64 dur64 = sample_clock_ - spu_disable_start_sample_;
          const u32 dur =
              (dur64 > static_cast<u64>(std::numeric_limits<u32>::max()))
                  ? std::numeric_limits<u32>::max()
                  : static_cast<u32>(dur64);
          ++audio_diag_.spu_disable_span_events;
          audio_diag_.spu_disable_span_total_samples += dur64;
          if (audio_diag_.spu_disable_span_events == 1) {
            audio_diag_.spu_disable_span_min_samples = dur;
            audio_diag_.spu_disable_span_max_samples = dur;
          } else {
            audio_diag_.spu_disable_span_min_samples =
                std::min(audio_diag_.spu_disable_span_min_samples, dur);
            audio_diag_.spu_disable_span_max_samples =
                std::max(audio_diag_.spu_disable_span_max_samples, dur);
          }
          spu_disabled_window_active_ = false;
        }
      } else {
        ++audio_diag_.spucnt_enable_clear_events;
        spu_disabled_window_active_ = true;
        spu_disable_start_sample_ = sample_clock_;
        force_off_all_voices_immediate();
        pending_kon_mask_ = 0;
        pending_koff_mask_ = 0;
      }
    }

    ++audio_diag_.spucnt_writes;
    if (((spucnt_ ^ value) & 0x4000u) != 0u) {
      ++audio_diag_.spucnt_mute_toggle_events;
      if ((value & 0x4000u) != 0u) {
        ++audio_diag_.spucnt_mute_clear_events;
      } else {
        ++audio_diag_.spucnt_mute_set_events;
      }
    }
    audio_diag_.spucnt_last = value;
    spucnt_ = value;
    if ((value & 0x0040u) == 0u) {
      clear_irq9_flag();
    }
    spucnt_mode_pending_ = static_cast<u16>(value & 0x003Fu);
    if (spucnt_mode_pending_ == spucnt_mode_latched_) {
      spucnt_mode_delay_cycles_ = 0;
    } else {
      spucnt_mode_delay_cycles_ = SPUCNT_MODE_APPLY_DELAY_CYCLES;
    }
    break;
  }
  case 0x1B0:
    cd_vol_l_ = static_cast<s16>(value);
    break;
  case 0x1B2:
    cd_vol_r_ = static_cast<s16>(value);
    break;
  case 0x1B4:
    ext_vol_l_ = static_cast<s16>(value);
    break;
  case 0x1B6:
    ext_vol_r_ = static_cast<s16>(value);
    break;
  default:
    if (offset >= 0x1C0u && offset <= 0x1FEu && (offset & 1u) == 0u) {
      write_reverb_reg(offset, value);
    }
    break;
  }
}
void Spu::dma_write(u32 value) {
  if (g_trace_spu &&
      trace_should_log(g_spu_trace_dma_write_counter, g_trace_burst_spu,
                       g_trace_stride_spu)) {
    LOG_DEBUG("SPU: DMA write addr=0x%05X val=0x%08X", transfer_addr_, value);
  }

  maybe_raise_irq9_for_ram_access(transfer_addr_, 4);
  spu_ram_[transfer_addr_ & SPU_RAM_MASK] =
      static_cast<u8>((value >> 0) & 0xFFu);
  spu_ram_[(transfer_addr_ + 1u) & SPU_RAM_MASK] =
      static_cast<u8>((value >> 8) & 0xFFu);
  spu_ram_[(transfer_addr_ + 2u) & SPU_RAM_MASK] =
      static_cast<u8>((value >> 16) & 0xFFu);
  spu_ram_[(transfer_addr_ + 3u) & SPU_RAM_MASK] =
      static_cast<u8>((value >> 24) & 0xFFu);
  transfer_addr_ = (transfer_addr_ + 4u) & static_cast<u32>(SPU_RAM_MASK);
  transfer_busy_cycles_ = 64;
}

u32 Spu::dma_read() {
  if (g_trace_spu &&
      trace_should_log(g_spu_trace_dma_read_counter, g_trace_burst_spu,
                       g_trace_stride_spu)) {
    LOG_DEBUG("SPU: DMA read addr=0x%05X", transfer_addr_);
  }

  maybe_raise_irq9_for_ram_access(transfer_addr_, 4);
  u32 value = 0;
  value |= static_cast<u32>(spu_ram_[transfer_addr_ & SPU_RAM_MASK]) << 0;
  value |=
      static_cast<u32>(spu_ram_[(transfer_addr_ + 1u) & SPU_RAM_MASK]) << 8;
  value |=
      static_cast<u32>(spu_ram_[(transfer_addr_ + 2u) & SPU_RAM_MASK]) << 16;
  value |=
      static_cast<u32>(spu_ram_[(transfer_addr_ + 3u) & SPU_RAM_MASK]) << 24;
  transfer_addr_ = (transfer_addr_ + 4u) & static_cast<u32>(SPU_RAM_MASK);
  transfer_busy_cycles_ = 64;
  return value;
}

void Spu::key_on_voice(int voice) {
  if (voice < 0 || voice >= NUM_VOICES) {
    return;
  }
  VoiceState &vs = voices_[voice];

  if (voice == TRACE_VOICE) {
    ++audio_diag_.v16_kon_apply_events;
    if (v16_waiting_for_koff_) {
      ++audio_diag_.v16_kon_reapply_without_koff;
    }
    v16_waiting_for_koff_ = true;
    v16_last_kon_apply_sample_ = sample_clock_;
    v16_has_last_kon_apply_ = true;
  }

  if (vs.phase != VoiceState::AdsrPhase::Off && has_last_kon_[voice]) {
    const u64 delta64 = sample_clock_ - last_kon_sample_[voice];
    const u32 delta =
        (delta64 > static_cast<u64>(std::numeric_limits<u32>::max()))
            ? std::numeric_limits<u32>::max()
            : static_cast<u32>(delta64);
    ++audio_diag_.kon_to_retrigger_events;
    audio_diag_.kon_to_retrigger_total_samples += delta64;
    if (audio_diag_.kon_to_retrigger_events == 1) {
      audio_diag_.kon_to_retrigger_min_samples = delta;
      audio_diag_.kon_to_retrigger_max_samples = delta;
    } else {
      audio_diag_.kon_to_retrigger_min_samples =
          std::min(audio_diag_.kon_to_retrigger_min_samples, delta);
      audio_diag_.kon_to_retrigger_max_samples =
          std::max(audio_diag_.kon_to_retrigger_max_samples, delta);
    }
  }
  if (vs.phase != VoiceState::AdsrPhase::Off) {
    ++audio_diag_.kon_retrigger_events;
  }

  vs = VoiceState{};
  vs.key_on = true;
  const u32 base = static_cast<u32>(voice) * VOICE_REG_STRIDE;
  vs.addr =
      (static_cast<u32>(regs_[(base + 0x6u) / 2u]) * 8u) & static_cast<u32>(SPU_RAM_MASK);
  vs.repeat_addr =
      (static_cast<u32>(regs_[(base + 0xEu) / 2u]) * 8u) & static_cast<u32>(SPU_RAM_MASK);
  if (vs.repeat_addr == 0u) {
    vs.repeat_addr = vs.addr;
  }

  const u16 adsr1 = regs_[(base + 0x8u) / 2u];
  const u16 adsr2 = regs_[(base + 0xAu) / 2u];
  vs.attack_exp = (adsr1 & 0x8000u) != 0u;
  vs.attack_shift = static_cast<u8>((adsr1 >> 10) & 0x1Fu);
  vs.attack_step = static_cast<u8>((adsr1 >> 8) & 0x3u);
  vs.decay_shift = static_cast<u8>((adsr1 >> 4) & 0x0Fu);
  const u32 sustain_nibble = static_cast<u32>(adsr1 & 0x000Fu);
  vs.sustain_level = static_cast<u16>(
      std::min<s32>(static_cast<s32>((sustain_nibble + 1u) * 0x800u), kEnvMax));
  vs.sustain_exp = (adsr2 & 0x8000u) != 0u;
  vs.sustain_decrease = (adsr2 & 0x4000u) != 0u;
  vs.sustain_shift = static_cast<u8>((adsr2 >> 8) & 0x1Fu);
  vs.sustain_step = static_cast<u8>((adsr2 >> 6) & 0x3u);
  vs.release_exp = (adsr2 & 0x0020u) != 0u;
  vs.release_shift = static_cast<u8>(adsr2 & 0x1Fu);
  vs.phase = VoiceState::AdsrPhase::Attack;
  vs.env_level = 0;

  endx_mask_ &= ~(1u << voice);
  ++audio_diag_.key_on_events;
  last_kon_sample_[voice] = sample_clock_;
  has_last_kon_[voice] = true;
}

void Spu::key_off_voice(int voice) {
  if (voice < 0 || voice >= NUM_VOICES) {
    return;
  }
  VoiceState &vs = voices_[voice];
  if (vs.phase == VoiceState::AdsrPhase::Off) {
    return;
  }

  if (voice == TRACE_VOICE) {
    ++audio_diag_.v16_koff_apply_events;
    if (!v16_waiting_for_koff_) {
      ++audio_diag_.v16_koff_without_kon;
    }
    if (v16_has_last_kon_apply_) {
      const u64 delta64 = sample_clock_ - v16_last_kon_apply_sample_;
      const u32 delta =
          (delta64 > static_cast<u64>(std::numeric_limits<u32>::max()))
              ? std::numeric_limits<u32>::max()
              : static_cast<u32>(delta64);
      ++audio_diag_.v16_kon_to_koff_apply_events;
      audio_diag_.v16_kon_to_koff_apply_total_samples += delta64;
      if (audio_diag_.v16_kon_to_koff_apply_events == 1) {
        audio_diag_.v16_kon_to_koff_apply_min_samples = delta;
        audio_diag_.v16_kon_to_koff_apply_max_samples = delta;
      } else {
        audio_diag_.v16_kon_to_koff_apply_min_samples = std::min(
            audio_diag_.v16_kon_to_koff_apply_min_samples, delta);
        audio_diag_.v16_kon_to_koff_apply_max_samples = std::max(
            audio_diag_.v16_kon_to_koff_apply_max_samples, delta);
      }
    }
    v16_waiting_for_koff_ = false;
  }

  if (has_last_kon_[voice]) {
    const u64 delta64 = sample_clock_ - last_kon_sample_[voice];
    const u32 delta = (delta64 > static_cast<u64>(std::numeric_limits<u32>::max()))
                          ? std::numeric_limits<u32>::max()
                          : static_cast<u32>(delta64);
    ++audio_diag_.kon_to_koff_events;
    audio_diag_.kon_to_koff_total_samples += delta64;
    const bool first = (audio_diag_.kon_to_koff_events == 1);
    const bool new_min = first || (delta < audio_diag_.kon_to_koff_min_samples);
    if (first) {
      audio_diag_.kon_to_koff_min_samples = delta;
      audio_diag_.kon_to_koff_max_samples = delta;
    } else {
      audio_diag_.kon_to_koff_min_samples =
          std::min(audio_diag_.kon_to_koff_min_samples, delta);
      audio_diag_.kon_to_koff_max_samples =
          std::max(audio_diag_.kon_to_koff_max_samples, delta);
    }

    if (new_min) {
      const u32 base = static_cast<u32>(voice) * VOICE_REG_STRIDE;
      audio_diag_.kon_to_koff_min_voice = static_cast<u32>(voice);
      audio_diag_.kon_to_koff_min_addr = vs.addr & SPU_RAM_MASK;
      audio_diag_.kon_to_koff_min_pitch = regs_[(base + 0x4u) / 2u];
      audio_diag_.kon_to_koff_min_adsr2 = regs_[(base + 0xAu) / 2u];
    }
    if (delta < KOFF_DEBOUNCE_SAMPLES) {
      ++audio_diag_.koff_short_window_events;
    }
  }

  if (vs.env_level >= 0x4000u) {
    ++audio_diag_.koff_high_env_events;
  }
  vs.key_on = false;
  vs.stop_after_block = false;
  vs.phase = VoiceState::AdsrPhase::Release;
  vs.adsr_counter = 0;
  vs.release_tracking = true;
  vs.release_start_sample = sample_clock_;
  ++audio_diag_.key_off_events;
}

void Spu::force_off_all_voices_immediate() {
  u32 forced = 0;
  for (int v = 0; v < NUM_VOICES; ++v) {
    VoiceState &vs = voices_[v];
    if (vs.phase == VoiceState::AdsrPhase::Off) {
      continue;
    }
    ++forced;
    vs.phase = VoiceState::AdsrPhase::Off;
    vs.env_level = 0;
    vs.adsr_counter = 0;
    vs.key_on = false;
    vs.stop_after_block = false;
    vs.release_tracking = false;
    vs.current_vol_l = 0;
    vs.current_vol_r = 0;
  }
  if (forced != 0u) {
    audio_diag_.spu_disable_forced_off_voices += forced;
  }
}

void Spu::apply_pending_key_strobes() {
  const u32 kon = pending_kon_mask_ & 0x00FFFFFFu;
  const u32 koff = pending_koff_mask_ & 0x00FFFFFFu;
  pending_kon_mask_ = 0;
  pending_koff_mask_ = 0;

  const u32 v16_mask = (1u << TRACE_VOICE);
  if ((kon & v16_mask) != 0u) {
    ++audio_diag_.v16_strobe_samples_with_kon;
  }
  if ((koff & v16_mask) != 0u) {
    ++audio_diag_.v16_strobe_samples_with_koff;
  }
  if (((kon & v16_mask) != 0u) && ((koff & v16_mask) != 0u)) {
    ++audio_diag_.v16_strobe_samples_with_both;
  }

  for (int v = 0; v < NUM_VOICES; ++v) {
    if ((koff & (1u << v)) != 0u) {
      key_off_voice(v);
    }
  }
  for (int v = 0; v < NUM_VOICES; ++v) {
    if ((kon & (1u << v)) != 0u) {
      key_on_voice(v);
    }
  }
}

s16 Spu::sat16(s32 value) {
  if (value < -32768) {
    return -32768;
  }
  if (value > 32767) {
    return 32767;
  }
  return static_cast<s16>(value);
}

s32 Spu::mul_q15(s32 a, s32 b) {
  const s64 p = static_cast<s64>(a) * static_cast<s64>(b);
  if (p >= 0) {
    return static_cast<s32>((p + 0x4000) >> 15);
  }
  return static_cast<s32>((p - 0x4000) >> 15);
}

float Spu::q15_to_float(s16 value) {
  return static_cast<float>(value) / 32768.0f;
}

u32 Spu::popcount32(u32 value) {
  u32 c = 0;
  while (value != 0u) {
    c += (value & 1u);
    value >>= 1;
  }
  return c;
}

u32 Spu::reverb_work_size_bytes() const {
  const u32 base = reverb_base_addr_ & SPU_RAM_WORD_MASK;
  if (base >= 0x80000u) {
    return 0;
  }
  return 0x80000u - base;
}

u32 Spu::reverb_addr_from_reg(u16 reg, s32 delta_bytes) {
  const u32 work_size = reverb_work_size_bytes();
  if (work_size < 2u) {
    ++audio_diag_.reverb_guard_events;
    return reverb_base_addr_ & SPU_RAM_WORD_MASK;
  }

  const u32 cursor = reverb_state_.cursor % work_size;
  const s64 rel = static_cast<s64>(cursor) +
                  static_cast<s64>(static_cast<u32>(reg) * 8u) +
                  static_cast<s64>(delta_bytes);
  s64 wrapped = rel % static_cast<s64>(work_size);
  if (wrapped < 0) {
    wrapped += static_cast<s64>(work_size);
  }
  u32 wrapped_u = static_cast<u32>(wrapped);
  if ((wrapped_u & 1u) != 0u) {
    wrapped_u &= ~1u;
    ++audio_diag_.reverb_guard_events;
  }

  return ((reverb_base_addr_ & SPU_RAM_WORD_MASK) + wrapped_u) & SPU_RAM_WORD_MASK;
}

s16 Spu::read_spu_s16(u32 addr) {
  const u32 a = addr & SPU_RAM_WORD_MASK;
  maybe_raise_irq9_for_ram_access(a, 2);
  const u16 lo = static_cast<u16>(spu_ram_[a & SPU_RAM_MASK]);
  const u16 hi = static_cast<u16>(spu_ram_[(a + 1u) & SPU_RAM_MASK]);
  return static_cast<s16>(static_cast<u16>(lo | static_cast<u16>(hi << 8)));
}

void Spu::write_spu_s16(u32 addr, s16 value, bool reverb_write) {
  const u32 a = addr & SPU_RAM_WORD_MASK;
  maybe_raise_irq9_for_ram_access(a, 2);
  const u16 raw = static_cast<u16>(value);
  spu_ram_[a & SPU_RAM_MASK] = static_cast<u8>(raw & 0x00FFu);
  spu_ram_[(a + 1u) & SPU_RAM_MASK] = static_cast<u8>((raw >> 8) & 0x00FFu);
  if (reverb_write) {
    ++audio_diag_.reverb_ram_writes;
  }
}

s16 Spu::fir39_q15(const std::array<s16, 39> &history) const {
  s64 acc = 0;
  for (size_t i = 0; i < history.size(); ++i) {
    acc += static_cast<s64>(history[i]) * static_cast<s64>(kReverbFirTable[i]);
  }
  if (acc >= 0) {
    return sat16(static_cast<s32>((acc + 0x4000) >> 15));
  }
  return sat16(static_cast<s32>((acc - 0x4000) >> 15));
}

s16 Spu::step_reverb_channel(bool right_channel, s16 lin, s16 rin,
                             bool writes_enabled, bool same_diff_enabled) {
  const u16 m_same = right_channel ? reverb_regs_.m_rsame : reverb_regs_.m_lsame;
  const u16 d_same = right_channel ? reverb_regs_.d_rsame : reverb_regs_.d_lsame;
  const u16 m_diff = right_channel ? reverb_regs_.m_rdiff : reverb_regs_.m_ldiff;
  const u16 d_cross =
      right_channel ? reverb_regs_.d_ldiff : reverb_regs_.d_rdiff;
  const u16 m_comb1 =
      right_channel ? reverb_regs_.m_rcomb1 : reverb_regs_.m_lcomb1;
  const u16 m_comb2 =
      right_channel ? reverb_regs_.m_rcomb2 : reverb_regs_.m_lcomb2;
  const u16 m_comb3 =
      right_channel ? reverb_regs_.m_rcomb3 : reverb_regs_.m_lcomb3;
  const u16 m_comb4 =
      right_channel ? reverb_regs_.m_rcomb4 : reverb_regs_.m_lcomb4;
  const u16 m_apf1 =
      right_channel ? reverb_regs_.m_rapf1 : reverb_regs_.m_lapf1;
  const u16 m_apf2 =
      right_channel ? reverb_regs_.m_rapf2 : reverb_regs_.m_lapf2;

  const s16 in_same = right_channel ? rin : lin;
  const s16 in_cross = right_channel ? lin : rin;

  if (same_diff_enabled) {
    const s16 same_tap = read_spu_s16(reverb_addr_from_reg(d_same, 0));
    const s16 diff_tap = read_spu_s16(reverb_addr_from_reg(d_cross, 0));

    const s16 same_ref =
        sat16(static_cast<s32>(in_same) +
              mul_q15(static_cast<s32>(reverb_regs_.v_wall),
                      static_cast<s32>(same_tap)));
    const s16 diff_ref =
        sat16(static_cast<s32>(in_cross) +
              mul_q15(static_cast<s32>(reverb_regs_.v_wall),
                      static_cast<s32>(diff_tap)));

    const s16 same_prev = read_spu_s16(reverb_addr_from_reg(m_same, -2));
    const s16 diff_prev = read_spu_s16(reverb_addr_from_reg(m_diff, -2));
    const s32 iir_alpha = static_cast<s32>(reverb_regs_.v_iir);
    const s32 iir_beta = 0x7FFF - iir_alpha;

    const s16 same_out =
        sat16(mul_q15(iir_alpha, static_cast<s32>(same_ref)) +
              mul_q15(iir_beta, static_cast<s32>(same_prev)));
    const s16 diff_out =
        sat16(mul_q15(iir_alpha, static_cast<s32>(diff_ref)) +
              mul_q15(iir_beta, static_cast<s32>(diff_prev)));

    if (writes_enabled) {
      write_spu_s16(reverb_addr_from_reg(m_same, 0), same_out, true);
      write_spu_s16(reverb_addr_from_reg(m_diff, 0), diff_out, true);
    }
  }

  s32 comb = 0;
  comb += mul_q15(static_cast<s32>(reverb_regs_.v_comb1),
                  static_cast<s32>(read_spu_s16(reverb_addr_from_reg(m_comb1, 0))));
  comb += mul_q15(static_cast<s32>(reverb_regs_.v_comb2),
                  static_cast<s32>(read_spu_s16(reverb_addr_from_reg(m_comb2, 0))));
  comb += mul_q15(static_cast<s32>(reverb_regs_.v_comb3),
                  static_cast<s32>(read_spu_s16(reverb_addr_from_reg(m_comb3, 0))));
  comb += mul_q15(static_cast<s32>(reverb_regs_.v_comb4),
                  static_cast<s32>(read_spu_s16(reverb_addr_from_reg(m_comb4, 0))));
  s16 out = sat16(comb);

  const s16 apf1_tap =
      read_spu_s16(reverb_addr_from_reg(m_apf1, -static_cast<s32>(reverb_regs_.d_apf1) * 8));
  s16 apf1_in =
      sat16(static_cast<s32>(out) -
            mul_q15(static_cast<s32>(reverb_regs_.v_apf1), static_cast<s32>(apf1_tap)));
  if (writes_enabled) {
    write_spu_s16(reverb_addr_from_reg(m_apf1, 0), apf1_in, true);
  } else {
    apf1_in = read_spu_s16(reverb_addr_from_reg(m_apf1, 0));
  }
  out = sat16(mul_q15(static_cast<s32>(reverb_regs_.v_apf1), static_cast<s32>(apf1_in)) +
              static_cast<s32>(apf1_tap));

  const s16 apf2_tap =
      read_spu_s16(reverb_addr_from_reg(m_apf2, -static_cast<s32>(reverb_regs_.d_apf2) * 8));
  s16 apf2_in =
      sat16(static_cast<s32>(out) -
            mul_q15(static_cast<s32>(reverb_regs_.v_apf2), static_cast<s32>(apf2_tap)));
  if (writes_enabled) {
    write_spu_s16(reverb_addr_from_reg(m_apf2, 0), apf2_in, true);
  } else {
    apf2_in = read_spu_s16(reverb_addr_from_reg(m_apf2, 0));
  }
  out = sat16(mul_q15(static_cast<s32>(reverb_regs_.v_apf2), static_cast<s32>(apf2_in)) +
              static_cast<s32>(apf2_tap));

  return out;
}

std::array<float, 2> Spu::step_reverb(float send_l, float send_r, u16 spucnt_eff) {
  if (g_low_spec_mode) {
    return {0.0f, 0.0f};
  }

  const s16 in_l = sat16(static_cast<s32>(
      std::lround(std::clamp(send_l, -1.0f, 1.0f) * 32767.0f)));
  const s16 in_r = sat16(static_cast<s32>(
      std::lround(std::clamp(send_r, -1.0f, 1.0f) * 32767.0f)));
  reverb_state_.last_in_l = in_l;
  reverb_state_.last_in_r = in_r;
  push_history(reverb_state_.in_hist_l, in_l);
  push_history(reverb_state_.in_hist_r, in_r);

  const s16 down_l = fir39_q15(reverb_state_.in_hist_l);
  const s16 down_r = fir39_q15(reverb_state_.in_hist_r);
  const s16 lin = sat16(mul_q15(static_cast<s32>(reverb_regs_.v_lin),
                                static_cast<s32>(down_l)));
  const s16 rin = sat16(mul_q15(static_cast<s32>(reverb_regs_.v_rin),
                                static_cast<s32>(down_r)));

  const bool reverb_master_enable = (spucnt_eff & 0x0080u) != 0u;
  const bool writes_enabled = reverb_master_enable;
  const bool same_diff_enabled = reverb_master_enable;

  s16 wet_22050_l = 0;
  s16 wet_22050_r = 0;
  if (reverb_state_.process_right) {
    wet_22050_r = step_reverb_channel(true, lin, rin, writes_enabled, same_diff_enabled);
    reverb_state_.process_right = false;
    const u32 work_size = reverb_work_size_bytes();
    if (work_size >= 2u) {
      reverb_state_.cursor = (reverb_state_.cursor + 2u) % work_size;
    }
    push_history(reverb_state_.out_hist_l, 0);
    push_history(reverb_state_.out_hist_r, wet_22050_r);
  } else {
    wet_22050_l = step_reverb_channel(false, lin, rin, writes_enabled, same_diff_enabled);
    reverb_state_.process_right = true;
    push_history(reverb_state_.out_hist_l, wet_22050_l);
    push_history(reverb_state_.out_hist_r, 0);
  }

  ++audio_diag_.reverb_mix_frames;

  const s32 up_l = static_cast<s32>(fir39_q15(reverb_state_.out_hist_l)) * 2;
  const s32 up_r = static_cast<s32>(fir39_q15(reverb_state_.out_hist_r)) * 2;
  const s16 wet_l = sat16(mul_q15(static_cast<s32>(reverb_depth_l_), up_l));
  const s16 wet_r = sat16(mul_q15(static_cast<s32>(reverb_depth_r_), up_r));
  return {q15_to_float(wet_l), q15_to_float(wet_r)};
}

bool Spu::decode_adpcm_block(int voice) {
  static constexpr int kFilterA[5] = {0, 60, 115, 98, 122};
  static constexpr int kFilterB[5] = {0, 0, -52, -55, -60};

  VoiceState &vs = voices_[voice];
  const u32 block_addr = vs.addr & SPU_RAM_MASK;
  maybe_raise_irq9_for_ram_access(block_addr, 16);

  const u8 predict_shift = spu_ram_[block_addr & SPU_RAM_MASK];
  const u8 flags = spu_ram_[(block_addr + 1u) & SPU_RAM_MASK];
  vs.last_block_addr = block_addr;
  vs.last_adpcm_flags = flags;

  const int raw_shift = static_cast<int>(predict_shift & 0x0Fu);
  const int shift = (raw_shift > 12) ? 9 : raw_shift;
  const int filter = static_cast<int>((predict_shift >> 4) & 0x07u);
  const int f = (filter <= 4) ? filter : 0;

  for (int i = 0; i < 28; ++i) {
    const u8 packed =
        spu_ram_[(block_addr + 2u + static_cast<u32>(i >> 1)) & SPU_RAM_MASK];
    int nibble = ((i & 1) != 0) ? static_cast<int>(packed >> 4)
                                : static_cast<int>(packed & 0x0Fu);
    if ((nibble & 0x8) != 0) {
      nibble -= 16;
    }

    int sample = (nibble << 12);
    sample >>= shift;
    sample += (kFilterA[f] * static_cast<int>(vs.hist1) +
               kFilterB[f] * static_cast<int>(vs.hist2) + 32) >>
              6;
    sample = std::clamp(sample, -32768, 32767);

    vs.hist2 = vs.hist1;
    vs.hist1 = static_cast<s16>(sample);
    vs.decoded[static_cast<size_t>(i)] = static_cast<s16>(sample);
  }

  vs.sample_index = 0;
  vs.addr = (block_addr + 16u) & SPU_RAM_MASK;

  if ((flags & 0x04u) != 0u) {
    vs.repeat_addr = block_addr;
  }
  if ((flags & 0x01u) != 0u) {
    ++audio_diag_.end_flag_events;
    endx_mask_ |= (1u << voice);

    const bool repeat = (flags & 0x02u) != 0u;
    if (repeat) {
      ++audio_diag_.loop_end_events;
      vs.addr = vs.repeat_addr;
      vs.stop_after_block = false;
    } else {
      ++audio_diag_.nonloop_end_events;
      const bool noise_voice = (noise_on_mask_ & (1u << voice)) != 0u;
      if (noise_voice) {
        vs.addr = vs.repeat_addr;
        vs.stop_after_block = false;
      } else {
        vs.stop_after_block = true;
      }
    }
  } else {
    vs.stop_after_block = false;
  }
  return true;
}

bool Spu::fetch_voice_sample(int voice, s16 &sample) {
  VoiceState &vs = voices_[voice];
  if (vs.phase == VoiceState::AdsrPhase::Off) {
    sample = 0;
    return false;
  }

  if (vs.sample_index >= 28) {
    if (vs.stop_after_block) {
      vs.stop_after_block = false;
      vs.phase = VoiceState::AdsrPhase::Off;
      vs.env_level = 0;
      vs.adsr_counter = 0;
      vs.release_tracking = false;
      vs.key_on = false;
      vs.current_vol_l = 0;
      vs.current_vol_r = 0;
      ++audio_diag_.off_due_to_end_flag;
      sample = 0;
      return false;
    }

    if (!decode_adpcm_block(voice) || vs.sample_index >= 28) {
      sample = 0;
      return false;
    }
  }

  sample = vs.decoded[static_cast<size_t>(vs.sample_index++)];
  return true;
}

void Spu::seed_gaussian_history(int voice) {
  VoiceState &vs = voices_[voice];
  if (vs.gauss_ready) {
    return;
  }
  for (int i = 0; i < 4; ++i) {
    s16 s = 0;
    fetch_voice_sample(voice, s);
    vs.gauss_hist[static_cast<size_t>(i)] = s;
  }
  vs.gauss_ready = true;
}

void Spu::shift_gaussian_history(int voice) {
  VoiceState &vs = voices_[voice];
  s16 next = 0;
  fetch_voice_sample(voice, next);
  vs.gauss_hist[0] = vs.gauss_hist[1];
  vs.gauss_hist[1] = vs.gauss_hist[2];
  vs.gauss_hist[2] = vs.gauss_hist[3];
  vs.gauss_hist[3] = next;
}

s16 Spu::interpolate_gaussian(const VoiceState &voice) const {
  if (!voice.gauss_ready) {
    return 0;
  }
  const u32 n = (voice.pitch_counter >> 4) & 0xFFu;
  const s32 g0 = kGaussTable[0x0FFu - n];
  const s32 g1 = kGaussTable[0x1FFu - n];
  const s32 g2 = kGaussTable[0x100u + n];
  const s32 g3 = kGaussTable[0x000u + n];

  const s32 s0 = static_cast<s32>(voice.gauss_hist[0]);
  const s32 s1 = static_cast<s32>(voice.gauss_hist[1]);
  const s32 s2 = static_cast<s32>(voice.gauss_hist[2]);
  const s32 s3 = static_cast<s32>(voice.gauss_hist[3]);
  const s32 mixed = (s0 * g0 + s1 * g1 + s2 * g2 + s3 * g3 + 0x4000) >> 15;
  return sat16(mixed);
}

s16 Spu::apply_envelope(s16 sample, const VoiceState &voice) const {
  const s32 env = static_cast<s32>(voice.env_level);
  const s32 out = (static_cast<s32>(sample) * env) / kEnvMax;
  return sat16(out);
}

s32 Spu::decode_sweep_step(u16 raw, s16 current) const {
  const bool exponential = (raw & 0x4000u) != 0u;
  const bool decreasing = (raw & 0x2000u) != 0u;
  const u8 shift = static_cast<u8>((raw >> 2) & 0x1Fu);
  const u8 step = static_cast<u8>(raw & 0x3u);

  s32 delta = adsr_step_base(step, decreasing);
  delta <<= std::max(0, 11 - static_cast<int>(shift));
  if (exponential && !decreasing && current > 0x6000) {
    delta >>= 2;
  }
  if (exponential && decreasing) {
    delta = (delta * static_cast<s32>(current)) / 0x8000;
  }
  return static_cast<s32>(current) + delta;
}

void Spu::tick_global_sweeps() {
  if ((regs_[0x180u / 2u] & 0x8000u) != 0u) {
    master_vol_l_ = sat16(decode_sweep_step(regs_[0x180u / 2u], master_vol_l_));
  }
  if ((regs_[0x182u / 2u] & 0x8000u) != 0u) {
    master_vol_r_ = sat16(decode_sweep_step(regs_[0x182u / 2u], master_vol_r_));
  }
  if ((regs_[0x184u / 2u] & 0x8000u) != 0u) {
    reverb_depth_l_ = sat16(decode_sweep_step(regs_[0x184u / 2u], reverb_depth_l_));
  }
  if ((regs_[0x186u / 2u] & 0x8000u) != 0u) {
    reverb_depth_r_ = sat16(decode_sweep_step(regs_[0x186u / 2u], reverb_depth_r_));
  }
}

s16 Spu::next_noise_sample() {
  const u16 spucnt_eff = spucnt_effective();
  const s32 noise_shift = static_cast<s32>((spucnt_eff >> 10) & 0x0Fu);
  const s32 noise_step = 4 + static_cast<s32>((spucnt_eff >> 8) & 0x03u);
  const s32 timer_reload = 0x20000 >> noise_shift;

  noise_timer_ -= noise_step;
  const u16 level_u = static_cast<u16>(noise_level_);
  const u16 parity = static_cast<u16>(
      ((level_u >> 15) ^ (level_u >> 12) ^ (level_u >> 11) ^ (level_u >> 10) ^ 1u) &
      1u);

  if (noise_timer_ < 0) {
    noise_level_ = static_cast<s16>((level_u << 1) | parity);
    noise_timer_ += timer_reload;
    if (noise_timer_ < 0) {
      noise_timer_ += timer_reload;
    }
  }
  return noise_level_;
}

void Spu::queue_host_audio(const std::vector<s16> &samples) {
  if (samples.empty()) {
    return;
  }

  if (capture_enabled_) {
    if (capture_samples_.size() + samples.size() > CAPTURE_MAX_SAMPLES) {
      size_t drop = capture_samples_.size() + samples.size() - CAPTURE_MAX_SAMPLES;
      drop &= ~static_cast<size_t>(1);
      if (drop >= capture_samples_.size()) {
        capture_samples_.clear();
      } else if (drop > 0) {
        capture_samples_.erase(capture_samples_.begin(),
                               capture_samples_.begin() +
                                   static_cast<s64>(drop));
      }
    }
    capture_samples_.insert(capture_samples_.end(), samples.begin(), samples.end());
    audio_diag_.capture_frames += samples.size() / 2;
    return;
  }

  if (!audio_enabled_ || audio_device_ == 0) {
    return;
  }

  if (host_staging_read_pos_ >= host_staging_samples_.size()) {
    host_staging_samples_.clear();
    host_staging_read_pos_ = 0;
  } else if (host_staging_read_pos_ > 0 &&
             (host_staging_read_pos_ >= 16384u ||
              host_staging_samples_.size() >= (HOST_STAGING_MAX_SAMPLES * 2u))) {
    const size_t unread = host_staging_samples_.size() - host_staging_read_pos_;
    std::move(host_staging_samples_.begin() +
                  static_cast<s64>(host_staging_read_pos_),
              host_staging_samples_.end(), host_staging_samples_.begin());
    host_staging_samples_.resize(unread);
    host_staging_read_pos_ = 0;
  }

  const size_t unread = host_staging_samples_.size() - host_staging_read_pos_;
  if (unread + samples.size() > HOST_STAGING_MAX_SAMPLES) {
    size_t drop = unread + samples.size() - HOST_STAGING_MAX_SAMPLES;
    drop = std::min(drop, unread);
    drop &= ~static_cast<size_t>(1);
    if (drop > 0) {
      host_staging_read_pos_ += drop;
      audio_diag_.dropped_frames += drop / 2;
      ++audio_diag_.overrun_events;
    }
    if (host_staging_read_pos_ >= host_staging_samples_.size()) {
      host_staging_samples_.clear();
      host_staging_read_pos_ = 0;
    }
  }

  host_staging_samples_.insert(host_staging_samples_.end(), samples.begin(),
                               samples.end());

  while (host_staging_read_pos_ < host_staging_samples_.size()) {
    const u32 queued = SDL_GetQueuedAudioSize(audio_device_);
    audio_diag_.queue_last_bytes = queued;
    audio_diag_.queue_peak_bytes = std::max(audio_diag_.queue_peak_bytes, queued);

    if (queued >= host_max_queue_bytes_) {
      break;
    }

    u32 room = 0;
    if (queued < host_target_queue_bytes_) {
      room = host_target_queue_bytes_ - queued;
    } else {
      room = host_max_queue_bytes_ - queued;
    }
    if (room < sizeof(s16) * 2u) {
      break;
    }

    const size_t unread_samples =
        host_staging_samples_.size() - host_staging_read_pos_;
    size_t sample_room = room / sizeof(s16);
    sample_room &= ~static_cast<size_t>(1);
    size_t to_queue = std::min(sample_room, unread_samples);
    to_queue &= ~static_cast<size_t>(1);
    if (to_queue == 0) {
      break;
    }

    SDL_QueueAudio(audio_device_,
                   host_staging_samples_.data() + host_staging_read_pos_,
                   static_cast<Uint32>(to_queue * sizeof(s16)));
    audio_diag_.queued_frames += to_queue / 2;
    host_staging_read_pos_ += to_queue;
  }

  if (host_staging_read_pos_ >= host_staging_samples_.size()) {
    host_staging_samples_.clear();
    host_staging_read_pos_ = 0;
  }

  u32 queued = SDL_GetQueuedAudioSize(audio_device_);
  audio_diag_.queue_last_bytes = queued;
  audio_diag_.queue_peak_bytes = std::max(audio_diag_.queue_peak_bytes, queued);

  // Queue starvation hysteresis: if queue starves while running, pause
  // playback and re-enter prebuffer mode. Resume only after refilled.
  if (audio_started_) {
    const u32 starve_threshold = std::max(host_buffer_bytes_ / 8u, 1024u);
    if (queued <= starve_threshold) {
      ++audio_diag_.underrun_events;
      SDL_PauseAudioDevice(audio_device_, 1);
      audio_started_ = false;
    }
  }

  if (!audio_started_) {
    const u32 startup_threshold = std::max(host_target_queue_bytes_, 16384u);
    if (queued >= startup_threshold) {
      SDL_PauseAudioDevice(audio_device_, 0);
      audio_started_ = true;
    }
  }
}

void Spu::push_cd_audio_samples(const std::vector<s16> &samples,
                                u32 sample_rate) {
  if (samples.empty()) {
    return;
  }
  if ((samples.size() & 1u) != 0u) {
    return;
  }

  const std::vector<s16> resampled =
      resample_stereo_linear(samples, sample_rate, SAMPLE_RATE);
  if (resampled.empty()) {
    return;
  }

  if (cd_input_read_pos_ >= cd_input_samples_.size()) {
    cd_input_samples_.clear();
    cd_input_read_pos_ = 0;
  } else if (cd_input_read_pos_ > 0 &&
             (cd_input_read_pos_ >= 8192u ||
              cd_input_samples_.size() >= (CD_INPUT_MAX_SAMPLES * 2u))) {
    const size_t unread = cd_input_samples_.size() - cd_input_read_pos_;
    std::move(cd_input_samples_.begin() + static_cast<s64>(cd_input_read_pos_),
              cd_input_samples_.end(), cd_input_samples_.begin());
    cd_input_samples_.resize(unread);
    cd_input_read_pos_ = 0;
  }

  const size_t unread = cd_input_samples_.size() - cd_input_read_pos_;
  if (unread + resampled.size() > CD_INPUT_MAX_SAMPLES) {
    size_t drop = unread + resampled.size() - CD_INPUT_MAX_SAMPLES;
    drop = std::min(drop, unread);
    drop &= ~static_cast<size_t>(1);
    if (drop > 0) {
      cd_input_read_pos_ += drop;
    }
    if (cd_input_read_pos_ >= cd_input_samples_.size()) {
      cd_input_samples_.clear();
      cd_input_read_pos_ = 0;
    }
  }

  cd_input_samples_.insert(cd_input_samples_.end(), resampled.begin(),
                           resampled.end());
}
void Spu::tick_adsr(int voice, VoiceState &vs) {
  auto apply_rate = [&](u8 rate, u8 rate_mask, bool decreasing, bool exponential) {
    s32 step = adsr_step_base(static_cast<u8>(rate & 0x3u), decreasing);

    u32 counter_inc = 0x8000u;
    if (rate < 44u) {
      step <<= (11 - static_cast<s32>(rate >> 2));
    } else if (rate >= 48u) {
      counter_inc >>= (static_cast<u32>(rate >> 2) - 11u);
      if ((rate & rate_mask) != rate_mask) {
        counter_inc = std::max<u32>(counter_inc, 1u);
      }
    }

    u32 this_inc = counter_inc;
    s32 this_step = step;
    const s32 current = static_cast<s32>(vs.env_level);
    if (exponential) {
      if (decreasing) {
        this_step = (this_step * current) >> 15;
      } else if (vs.env_level >= 0x6000u) {
        if (rate < 40u) {
          this_step >>= 2;
        } else if (rate >= 44u) {
          this_inc >>= 2;
        } else {
          this_step >>= 1;
          this_inc >>= 1;
        }
      }
    }

    const u32 sum =
        static_cast<u32>(vs.adsr_counter) + static_cast<u32>(this_inc);
    if (sum < 0x8000u) {
      vs.adsr_counter = static_cast<u16>(sum);
      return;
    }
    vs.adsr_counter = static_cast<u16>(sum - 0x8000u);
    const s32 next = std::clamp(current + this_step, 0, kEnvMax);
    vs.env_level = static_cast<u16>(next);
  };

  switch (vs.phase) {
  case VoiceState::AdsrPhase::Attack: {
    const u8 rate = static_cast<u8>((vs.attack_shift << 2) | (vs.attack_step & 0x3u));
    apply_rate(rate, 0x7Fu, false, vs.attack_exp);
    if (vs.env_level >= kEnvMax) {
      vs.env_level = kEnvMax;
      vs.phase = VoiceState::AdsrPhase::Decay;
    }
    break;
  }
  case VoiceState::AdsrPhase::Decay: {
    const u8 rate = static_cast<u8>((vs.decay_shift & 0x0Fu) << 2);
    apply_rate(rate, static_cast<u8>(0x1Fu << 2), true, true);
    if (vs.env_level <= vs.sustain_level) {
      vs.env_level = vs.sustain_level;
      vs.phase = VoiceState::AdsrPhase::Sustain;
    }
    break;
  }
  case VoiceState::AdsrPhase::Sustain: {
    const u8 rate =
        static_cast<u8>((vs.sustain_shift << 2) | (vs.sustain_step & 0x3u));
    apply_rate(rate, 0x7Fu, vs.sustain_decrease, vs.sustain_exp);
    break;
  }
  case VoiceState::AdsrPhase::Release: {
    const u8 rate = static_cast<u8>((vs.release_shift & 0x1Fu) << 2);
    apply_rate(rate, static_cast<u8>(0x1Fu << 2), true, vs.release_exp);
    if (vs.env_level == 0u) {
      if (vs.release_tracking) {
        const u64 dur64 = sample_clock_ - vs.release_start_sample;
        const u32 dur =
            (dur64 > static_cast<u64>(std::numeric_limits<u32>::max()))
                ? std::numeric_limits<u32>::max()
                : static_cast<u32>(dur64);
        audio_diag_.release_samples_total += dur64;
        if (audio_diag_.release_to_off_events == 0u) {
          audio_diag_.release_samples_min = dur;
          audio_diag_.release_samples_max = dur;
        } else {
          audio_diag_.release_samples_min =
              std::min(audio_diag_.release_samples_min, dur);
          audio_diag_.release_samples_max =
              std::max(audio_diag_.release_samples_max, dur);
        }
        if (dur < 512u) {
          ++audio_diag_.release_fast_events;
        }
        vs.release_tracking = false;
      }
      vs.phase = VoiceState::AdsrPhase::Off;
      vs.key_on = false;
      ++audio_diag_.off_due_to_release_env0;
      ++audio_diag_.release_to_off_events;
    }
    break;
  }
  case VoiceState::AdsrPhase::Off:
  default:
    break;
  }

  (void)voice;
}

void Spu::tick(u32 cycles) {
  const bool profile_detailed = g_profile_detailed_timing;
  std::chrono::high_resolution_clock::time_point start{};
  if (profile_detailed) {
    start = std::chrono::high_resolution_clock::now();
  }

  tick_spucnt_mode_delay(cycles);
  if (transfer_busy_cycles_ > 0u) {
    transfer_busy_cycles_ =
        (transfer_busy_cycles_ > cycles) ? (transfer_busy_cycles_ - cycles) : 0u;
  }

  const double prev_accum = sample_accum_;
  sample_accum_ +=
      (static_cast<double>(cycles) * static_cast<double>(SAMPLE_RATE)) /
      static_cast<double>(psx::CPU_CLOCK_HZ);
#ifndef NDEBUG
  assert(sample_accum_ >= prev_accum);
#endif

  const int samples_to_generate = static_cast<int>(sample_accum_);
  if (samples_to_generate <= 0) {
    return;
  }
  sample_accum_ -= samples_to_generate;
  audio_diag_.generated_frames += static_cast<u64>(samples_to_generate);

  const u16 spucnt_eff = spucnt_effective();
  const bool enabled = (spucnt_eff & 0x8000u) != 0u;
  const bool muted = (spucnt_eff & 0x4000u) == 0u;

  const size_t out_samples = static_cast<size_t>(samples_to_generate) * 2u;
  mix_buffer_.resize(out_samples);
  std::fill(mix_buffer_.begin(), mix_buffer_.end(), 0);
  auto &out = mix_buffer_;

  if (!enabled) {
    for (int i = 0; i < samples_to_generate; ++i) {
      ++sample_clock_;
      capture_half_ ^= 1u;
      ++audio_diag_.muted_output_frames;
    }
    queue_host_audio(out);
    if (profile_detailed && sys_ != nullptr) {
      const auto end = std::chrono::high_resolution_clock::now();
      sys_->add_spu_time(
          std::chrono::duration<double, std::milli>(end - start).count());
    }
    return;
  }

  const bool track_sample_diag = g_spu_advanced_sound_status;
  audio_diag_.gaussian_active = true;
  audio_diag_.reverb_enabled = (spucnt_eff & 0x0080u) != 0u;
  for (int s = 0; s < samples_to_generate; ++s) {
    ++sample_clock_;
    capture_half_ ^= 1u;
    apply_pending_key_strobes();
    tick_global_sweeps();

    const s16 noise_raw = next_noise_sample();
    std::array<s16, NUM_VOICES> mod_source{};
    float dry_l = 0.0f;
    float dry_r = 0.0f;
    float rev_send_l = 0.0f;
    float rev_send_r = 0.0f;

    u32 logical_voices = 0;
    u32 env_voices = 0;
    u32 audible_voices = 0;

    for (int v = 0; v < NUM_VOICES; ++v) {
      VoiceState &vs = voices_[v];
      if (vs.phase == VoiceState::AdsrPhase::Off) {
        vs.current_vol_l = 0;
        vs.current_vol_r = 0;
        mod_source[static_cast<size_t>(v)] = 0;
        continue;
      }

      ++logical_voices;
      const bool use_noise = (noise_on_mask_ & (1u << v)) != 0u;
      seed_gaussian_history(v);

      const u32 base = static_cast<u32>(v) * VOICE_REG_STRIDE;
      const u16 pitch_raw = regs_[(base + 0x4u) / 2u];
      const u16 vol_l_reg = regs_[(base + 0x0u) / 2u];
      const u16 vol_r_reg = regs_[(base + 0x2u) / 2u];

      float voice_vol_l = 0.0f;
      float voice_vol_r = 0.0f;
      if ((vol_l_reg & 0x8000u) != 0u) {
        vs.sweep_vol_l = sat16(decode_sweep_step(vol_l_reg, vs.sweep_vol_l));
        voice_vol_l = q15_to_float(vs.sweep_vol_l);
      } else {
        vs.sweep_vol_l = decode_fixed_volume_q15(vol_l_reg);
        voice_vol_l = q15_to_float(vs.sweep_vol_l);
      }
      if ((vol_r_reg & 0x8000u) != 0u) {
        vs.sweep_vol_r = sat16(decode_sweep_step(vol_r_reg, vs.sweep_vol_r));
        voice_vol_r = q15_to_float(vs.sweep_vol_r);
      } else {
        vs.sweep_vol_r = decode_fixed_volume_q15(vol_r_reg);
        voice_vol_r = q15_to_float(vs.sweep_vol_r);
      }

      u32 step = static_cast<u32>(pitch_raw);
      if (v > 0 && ((pitch_mod_mask_ & (1u << v)) != 0u)) {
        const s32 signed_pitch = static_cast<s16>(pitch_raw);
        const s32 factor =
            static_cast<s32>(static_cast<u16>(mod_source[static_cast<size_t>(v - 1)] +
                                              0x8000));
        const s64 p = static_cast<s64>(signed_pitch) * static_cast<s64>(factor);
        step = static_cast<u32>(static_cast<s32>(p >> 15)) & 0xFFFFu;
      }
      if (step > 0x3FFFu) {
        step = 0x3FFFu;
      }

      tick_adsr(v, vs);

      const s16 raw = use_noise ? noise_raw : interpolate_gaussian(vs);
      const s16 env_applied = apply_envelope(raw, vs);
      mod_source[static_cast<size_t>(v)] = env_applied;
      const float signal = q15_to_float(env_applied);

      const float out_l = signal * voice_vol_l;
      const float out_r = signal * voice_vol_r;
      const float out_l_clamped = std::clamp(out_l, -1.0f, 1.0f);
      const float out_r_clamped = std::clamp(out_r, -1.0f, 1.0f);
      vs.current_vol_l = static_cast<s16>(out_l_clamped * 32767.0f);
      vs.current_vol_r = static_cast<s16>(out_r_clamped * 32767.0f);

      if (vs.env_level > 0u) {
        ++env_voices;
      }
      if (vs.env_level > 0u &&
          (vs.current_vol_l != 0 || vs.current_vol_r != 0)) {
        ++audible_voices;
      }

      dry_l += out_l;
      dry_r += out_r;
      if ((reverb_on_mask_ & (1u << v)) != 0u) {
        rev_send_l += out_l;
        rev_send_r += out_r;
      }

      vs.pitch_counter += step;
      while (vs.pitch_counter >= 0x1000u) {
        vs.pitch_counter -= 0x1000u;
        shift_gaussian_history(v);
      }
    }

    if (track_sample_diag) {
      audio_diag_.logical_voice_samples += 1;
      audio_diag_.logical_voice_accum += logical_voices;
      if (logical_voices > audio_diag_.logical_voice_peak) {
        audio_diag_.logical_voice_peak = logical_voices;
        audio_diag_.logical_voice_peak_sample = sample_clock_;
        audio_diag_.logical_voice_peak_key_on_events = audio_diag_.key_on_events;
        audio_diag_.logical_voice_peak_key_off_events = audio_diag_.key_off_events;
        audio_diag_.logical_voice_peak_endx_mask = endx_mask_ & 0x00FFFFFFu;
      }

      audio_diag_.env_voice_samples += 1;
      audio_diag_.env_voice_accum += env_voices;
      if (env_voices > audio_diag_.env_voice_peak) {
        audio_diag_.env_voice_peak = env_voices;
        audio_diag_.env_voice_peak_sample = sample_clock_;
        audio_diag_.env_voice_peak_key_on_events = audio_diag_.key_on_events;
        audio_diag_.env_voice_peak_key_off_events = audio_diag_.key_off_events;
        audio_diag_.env_voice_peak_endx_mask = endx_mask_ & 0x00FFFFFFu;
      }

      audio_diag_.audible_voice_samples += 1;
      audio_diag_.audible_voice_accum += audible_voices;
      if (audible_voices > audio_diag_.audible_voice_peak) {
        audio_diag_.audible_voice_peak = audible_voices;
        audio_diag_.audible_voice_peak_sample = sample_clock_;
        audio_diag_.audible_voice_peak_key_on_events = audio_diag_.key_on_events;
        audio_diag_.audible_voice_peak_key_off_events = audio_diag_.key_off_events;
        audio_diag_.audible_voice_peak_endx_mask = endx_mask_ & 0x00FFFFFFu;
      }

      audio_diag_.active_voice_samples = audio_diag_.audible_voice_samples;
      audio_diag_.active_voice_accum = audio_diag_.audible_voice_accum;
      audio_diag_.active_voice_peak = audio_diag_.audible_voice_peak;
      if (audible_voices >= NUM_VOICES) {
        ++audio_diag_.voice_cap_frames;
      }
      if (audible_voices == 0u) {
        ++audio_diag_.no_voice_frames;
      }
    }

    if ((spucnt_eff & 0x0001u) != 0u) {
      float cd_l = 0.0f;
      float cd_r = 0.0f;
      if (cd_input_read_pos_ + 1u < cd_input_samples_.size()) {
        cd_l = q15_to_float(cd_input_samples_[cd_input_read_pos_ + 0u]);
        cd_r = q15_to_float(cd_input_samples_[cd_input_read_pos_ + 1u]);
        cd_input_read_pos_ += 2u;
        if (track_sample_diag) {
          ++audio_diag_.cd_frames_mixed;
        }
      }

      const float cd_mix_l = cd_l * q15_to_float(cd_vol_l_);
      const float cd_mix_r = cd_r * q15_to_float(cd_vol_r_);
      dry_l += cd_mix_l;
      dry_r += cd_mix_r;

      if ((spucnt_eff & 0x0004u) != 0u) {
        rev_send_l += cd_mix_l;
        rev_send_r += cd_mix_r;
      }
    }

    if (track_sample_diag) {
      audio_diag_.peak_dry_l = std::max(audio_diag_.peak_dry_l, std::abs(dry_l));
      audio_diag_.peak_dry_r = std::max(audio_diag_.peak_dry_r, std::abs(dry_r));
      if (std::abs(dry_l) > 1.0f || std::abs(dry_r) > 1.0f) {
        ++audio_diag_.clip_events_dry;
      }
    }

    float wet_l = 0.0f;
    float wet_r = 0.0f;
    if (!g_low_spec_mode) {
      const std::array<float, 2> wet = step_reverb(rev_send_l, rev_send_r, spucnt_eff);
      wet_l = wet[0];
      wet_r = wet[1];
    }
    if (track_sample_diag) {
      audio_diag_.peak_wet_l = std::max(audio_diag_.peak_wet_l, std::abs(wet_l));
      audio_diag_.peak_wet_r = std::max(audio_diag_.peak_wet_r, std::abs(wet_r));
      if (std::abs(wet_l) > 0.98f || std::abs(wet_r) > 0.98f) {
        ++audio_diag_.clip_events_wet;
      }
    }

    float mix_l = dry_l + wet_l;
    float mix_r = dry_r + wet_r;
    mix_l *= q15_to_float(master_vol_l_);
    mix_r *= q15_to_float(master_vol_r_);

    if (track_sample_diag) {
      audio_diag_.peak_mix_l = std::max(audio_diag_.peak_mix_l, std::abs(mix_l));
      audio_diag_.peak_mix_r = std::max(audio_diag_.peak_mix_r, std::abs(mix_r));
      if (std::abs(mix_l) > 1.0f || std::abs(mix_r) > 1.0f) {
        ++audio_diag_.clip_events_out;
      }
    }

    mix_l = std::clamp(mix_l, -1.0f, 1.0f);
    mix_r = std::clamp(mix_r, -1.0f, 1.0f);
    if (muted) {
      ++audio_diag_.muted_output_frames;
      out[static_cast<size_t>(s) * 2u + 0u] = 0;
      out[static_cast<size_t>(s) * 2u + 1u] = 0;
    } else {
      out[static_cast<size_t>(s) * 2u + 0u] =
          static_cast<s16>(mix_l * 30000.0f);
      out[static_cast<size_t>(s) * 2u + 1u] =
          static_cast<s16>(mix_r * 30000.0f);
    }
  }

  if (cd_input_read_pos_ >= cd_input_samples_.size()) {
    cd_input_samples_.clear();
    cd_input_read_pos_ = 0;
  } else if (cd_input_read_pos_ > 0 &&
             (cd_input_read_pos_ >= 8192u ||
              cd_input_samples_.size() >= (CD_INPUT_MAX_SAMPLES * 2u))) {
    const size_t unread = cd_input_samples_.size() - cd_input_read_pos_;
    std::move(cd_input_samples_.begin() + static_cast<s64>(cd_input_read_pos_),
              cd_input_samples_.end(), cd_input_samples_.begin());
    cd_input_samples_.resize(unread);
    cd_input_read_pos_ = 0;
  }

  queue_host_audio(out);

  if (profile_detailed && sys_ != nullptr) {
    const auto end = std::chrono::high_resolution_clock::now();
    sys_->add_spu_time(
        std::chrono::duration<double, std::milli>(end - start).count());
  }
}
