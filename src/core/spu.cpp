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

// PSX Gaussian interpolation table from PSX-SPX.
constexpr std::array<s16, 512> kGaussTable = {
#include "spu_gauss_table.inc"
};

inline s32 adsr_base_step(u8 step, bool decreasing) {
  s32 v = 7 - static_cast<s32>(step & 0x3);
  if (decreasing) {
    v = ~v; // +7..+4 => -8..-5  (PSX-SPX: NOT AdsrStep)
  }
  return v;
}

inline s32 clamp_env_level(s32 v) {
  if (v < 0) {
    return 0;
  }
  if (v > kEnvMax) {
    return kEnvMax;
  }
  return v;
}

inline float q15_to_float(s16 value) {
  return static_cast<float>(value) / 32768.0f;
}

inline s16 decode_fixed_volume_q15(u16 raw) {
  // Fixed volume uses bits14..0 as signed "volume/2" in range -0x4000..+0x3FFF.
  s32 v = static_cast<s32>(raw & 0x7FFFu);
  if (v & 0x4000) {
    v -= 0x8000;
  }
  v *= 2;
  if (v < -32768) {
    return -32768;
  }
  if (v > 32767) {
    return 32767;
  }
  return static_cast<s16>(v);
}

std::vector<s16> resample_stereo_linear(const std::vector<s16> &in, u32 in_rate,
                                        u32 out_rate) {
  if (in.empty() || in_rate == 0 || out_rate == 0) {
    return {};
  }
  if (in_rate == out_rate) {
    return in;
  }

  const size_t in_frames = in.size() / 2;
  if (in_frames < 2) {
    return in;
  }

  const double ratio =
      static_cast<double>(in_rate) / static_cast<double>(out_rate);
  const size_t out_frames =
      std::max<size_t>(1, static_cast<size_t>(std::llround(
                              static_cast<double>(in_frames) / ratio)));

  std::vector<s16> out(out_frames * 2);
  for (size_t i = 0; i < out_frames; ++i) {
    const double src_pos = static_cast<double>(i) * ratio;
    size_t i0 = static_cast<size_t>(src_pos);
    if (i0 >= in_frames) {
      i0 = in_frames - 1;
    }
    const size_t i1 = std::min(i0 + 1, in_frames - 1);
    const double frac = std::clamp(src_pos - static_cast<double>(i0), 0.0, 1.0);

    const s16 l0 = in[i0 * 2];
    const s16 r0 = in[i0 * 2 + 1];
    const s16 l1 = in[i1 * 2];
    const s16 r1 = in[i1 * 2 + 1];

    const s32 lo = static_cast<s32>(std::lround(
        static_cast<double>(l0) + (static_cast<double>(l1 - l0) * frac)));
    const s32 ro = static_cast<s32>(std::lround(
        static_cast<double>(r0) + (static_cast<double>(r1 - r0) * frac)));
    out[i * 2] = static_cast<s16>(std::clamp(lo, -32768, 32767));
    out[i * 2 + 1] = static_cast<s16>(std::clamp(ro, -32768, 32767));
  }

  return out;
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
  desired.samples = 1024;
  desired.callback = nullptr;

  SDL_AudioSpec obtained{};
  audio_device_ = SDL_OpenAudioDevice(nullptr, 0, &desired, &obtained, 0);
  if (audio_device_ == 0) {
    LOG_WARN("SPU: Failed to open audio device: %s", SDL_GetError());
    audio_enabled_ = false;
    return;
  }
  if (obtained.format != AUDIO_S16SYS || obtained.channels != 2) {
    LOG_WARN("SPU: Unsupported audio format; disabling SPU audio");
    SDL_CloseAudioDevice(audio_device_);
    audio_device_ = 0;
    audio_enabled_ = false;
    return;
  }

  audio_enabled_ = true;
  SDL_PauseAudioDevice(audio_device_, 0);
  LOG_INFO("SPU: Audio device initialized at %d Hz", obtained.freq);
}

void Spu::shutdown() {
  if (audio_device_ != 0) {
    SDL_CloseAudioDevice(audio_device_);
    audio_device_ = 0;
  }
  audio_enabled_ = false;
}

void Spu::clear_audio_capture() { capture_samples_.clear(); }

void Spu::reset() {
  if (audio_device_ != 0) {
    SDL_ClearQueuedAudio(audio_device_);
  }

  sample_accum_ = 0.0;
  sample_clock_ = 0;
  for (int v = 0; v < NUM_VOICES; ++v) {
    voices_[v] = VoiceState{};
  }

  regs_.fill(0);
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
  endx_mask_ = 0;
  reverb_on_mask_ = 0;
  master_vol_l_ = 0x3FFF;
  master_vol_r_ = 0x3FFF;
  reverb_depth_l_ = 0;
  reverb_depth_r_ = 0;
  cd_vol_l_ = 0;
  cd_vol_r_ = 0;
  ext_vol_l_ = 0;
  ext_vol_r_ = 0;
  reverb_base_addr_ = 0;
  reverb_current_addr_ = 0;
  reverb_downsample_buffer_ = {};
  reverb_upsample_buffer_ = {};
  reverb_resample_pos_ = 0;
  reverb_regs_ = ReverbRegs{};
  audio_diag_ = AudioDiag{};
  host_staging_samples_.clear();
  capture_samples_.clear();
  cd_input_samples_.clear();
  last_kon_sample_.fill(0);
  has_last_kon_.fill(false);
  v16_last_kon_write_cpu_cycle_ = 0;
  v16_last_kon_write_sample_ = 0;
  v16_has_last_kon_write_ = false;
  v16_last_kon_apply_sample_ = 0;
  v16_has_last_kon_apply_ = false;
  v16_waiting_for_koff_ = false;
  cd_input_read_pos_ = 0;
  noise_level_ = 1;
  noise_timer_ = 0;
  last_synced_cpu_cycle_ = 0;
  spu_disabled_window_active_ = false;
  spu_disable_start_sample_ = 0;
  pending_kon_mask_ = 0;
  pending_koff_mask_ = 0;
  spu_ram_.fill(0);
}

u16 Spu::spucnt_effective() const {
  return static_cast<u16>((spucnt_ & 0xFFC0u) | (spucnt_mode_latched_ & 0x3Fu));
}

void Spu::tick_spucnt_mode_delay(u32 cycles) {
  if (spucnt_mode_delay_cycles_ == 0) {
    return;
  }
  if (cycles >= spucnt_mode_delay_cycles_) {
    spucnt_mode_delay_cycles_ = 0;
    spucnt_mode_latched_ = static_cast<u16>(spucnt_mode_pending_ & 0x3Fu);
    spustat_ = static_cast<u16>((spustat_ & ~0x3Fu) | spucnt_mode_latched_);
    return;
  }
  spucnt_mode_delay_cycles_ -= cycles;
}

void Spu::clear_irq9_flag() { spustat_ &= static_cast<u16>(~0x0040u); }

void Spu::maybe_raise_irq9_for_ram_access(u32 start_addr, u32 byte_count) {
  if (byte_count == 0) {
    return;
  }
  // SPUCNT bit6 gates SPU IRQ9 generation.
  if ((spucnt_effective() & 0x0040u) == 0) {
    return;
  }
  if ((spustat_ & 0x0040u) != 0) {
    return;
  }

  const u32 irq = irq_addr_ & SPU_RAM_WORD_MASK;
  for (u32 i = 0; i < byte_count; ++i) {
    const u32 accessed = (start_addr + i) & SPU_RAM_WORD_MASK;
    if (accessed == irq) {
      spustat_ |= 0x0040u;
      if (sys_) {
        sys_->irq().request(Interrupt::SPU);
      }
      return;
    }
  }
}

u16 Spu::read16(u32 offset) const {
  switch (offset) {
  case 0x188: // Key On low (strobe, write-only)
  case 0x18A: // Key On high (strobe, write-only)
  case 0x18C: // Key Off low (strobe, write-only)
  case 0x18E: // Key Off high (strobe, write-only)
    return 0;
  case 0x19C:
    return static_cast<u16>(endx_mask_ & 0xFFFFu);
  case 0x19E:
    return static_cast<u16>((endx_mask_ >> 16) & 0x00FFu);
  case 0x1AA:
    return spucnt_;
  case 0x1AE: {
    // PSX-SPX SPUSTAT:
    //  5-0 current SPU mode (mirrors SPUCNT bits 5-0)
    //  6   IRQ9 flag
    //  7   transfer DMA R/W request
    //  8   transfer DMA write request
    //  9   transfer DMA read request
    // 10   transfer busy
    // 11   capture half toggle
    const u16 spucnt_eff = spucnt_effective();
    u16 stat = static_cast<u16>((spucnt_eff & 0x3F) | (spustat_ & 0x0040u));
    const u16 transfer_mode = static_cast<u16>((spucnt_eff >> 4) & 0x3);
    stat |= ((spucnt_eff >> 5) & 1) << 7;
    stat |= static_cast<u16>(transfer_mode == 2 ? 1 : 0) << 8;
    stat |= static_cast<u16>(transfer_mode == 3 ? 1 : 0) << 9;
    stat |= static_cast<u16>(transfer_busy_cycles_ > 0 ? 1 : 0) << 10;
    stat |= static_cast<u16>(capture_half_ & 1) << 11;
    return stat;
  }
  default:
    if (offset >= 0x200 && offset < 0x260) {
      // Readback of current per-voice output volume (left/right).
      // Layout: voice N at 0x200 + N*4 (left), +2 (right).
      const u32 rel = offset - 0x200;
      const int voice = static_cast<int>(rel >> 2);
      const u32 lane = rel & 0x3u;
      if (voice >= 0 && voice < NUM_VOICES) {
        if (lane == 0) {
          return static_cast<u16>(voices_[voice].current_vol_l);
        }
        if (lane == 2) {
          return static_cast<u16>(voices_[voice].current_vol_r);
        }
      }
      return 0;
    }
    if (offset < 0x200) {
      return regs_[offset / 2];
    }
    LOG_WARN("SPU: Unhandled read16 at offset 0x%X", offset);
    return 0;
  }
}

void Spu::write16(u32 offset, u16 value) {
  if (g_trace_spu) {
    if ((offset >= 0x180 ||
         (offset < 0x180 && (offset % VOICE_REG_STRIDE) == 0)) &&
        trace_should_log(g_spu_trace_reg_counter, g_trace_burst_spu,
                         g_trace_stride_spu)) {
      LOG_DEBUG("SPU: W16 off=0x%03X val=0x%04X", offset, value);
    }
  }

  if (offset < 0x400) {
    if (offset == 0x188 || offset == 0x18A || offset == 0x18C ||
        offset == 0x18E) {
      // KON/KOFF are write-strobe registers; they do not latch readable state.
      regs_[offset / 2] = 0;
    } else {
      regs_[offset / 2] = value;
    }
  }
  if (offset < 0x180 && (offset % VOICE_REG_STRIDE) == 0xE) {
    const int v = static_cast<int>(offset / VOICE_REG_STRIDE);
    if (v >= 0 && v < NUM_VOICES) {
      voices_[v].repeat_addr =
          (static_cast<u32>(value) * 8u) & static_cast<u32>(SPU_RAM_MASK);
    }
  }

  const u64 cpu_cycle = (sys_ != nullptr) ? sys_->cpu().cycle_count() : 0;
  auto note_v16_kon_write = [&]() {
    ++audio_diag_.v16_kon_write_events;
    v16_last_kon_write_cpu_cycle_ = cpu_cycle;
    v16_last_kon_write_sample_ = sample_clock_;
    v16_has_last_kon_write_ = true;
    if (g_trace_spu) {
      LOG_DEBUG("SPU_V16_WRITE KON cpu=%llu sample=%llu val=0x%04X",
                static_cast<unsigned long long>(cpu_cycle),
                static_cast<unsigned long long>(sample_clock_), value);
    }
  };
  auto note_v16_koff_write = [&]() {
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
        audio_diag_.v16_kon_write_to_koff_min_cpu_cycles =
            std::min(audio_diag_.v16_kon_write_to_koff_min_cpu_cycles,
                     delta_cycles);
        audio_diag_.v16_kon_write_to_koff_max_cpu_cycles =
            std::max(audio_diag_.v16_kon_write_to_koff_max_cpu_cycles,
                     delta_cycles);
        audio_diag_.v16_kon_write_to_koff_min_samples =
            std::min(audio_diag_.v16_kon_write_to_koff_min_samples,
                     delta_samples32);
        audio_diag_.v16_kon_write_to_koff_max_samples =
            std::max(audio_diag_.v16_kon_write_to_koff_max_samples,
                     delta_samples32);
      }
    }
    if (g_trace_spu) {
      LOG_DEBUG("SPU_V16_WRITE KOFF cpu=%llu sample=%llu val=0x%04X",
                static_cast<unsigned long long>(cpu_cycle),
                static_cast<unsigned long long>(sample_clock_), value);
    }
  };
  auto note_key_write_timing = [&]() {
    const u64 sync_lag =
        (cpu_cycle > last_synced_cpu_cycle_) ? (cpu_cycle - last_synced_cpu_cycle_) : 0;
    if (sync_lag != 0) {
      ++audio_diag_.key_write_unsynced_events;
      audio_diag_.key_write_unsynced_max_cpu_lag =
          std::max(audio_diag_.key_write_unsynced_max_cpu_lag, sync_lag);
    }
    if ((spucnt_effective() & 0x8000u) == 0) {
      ++audio_diag_.key_write_while_spu_disabled;
    }
  };

  switch (offset) {
  case 0x180: // Main volume left
    master_vol_l_ = decode_fixed_volume_q15(value);
    break;
  case 0x182: // Main volume right
    master_vol_r_ = decode_fixed_volume_q15(value);
    break;
  case 0x184: // Reverb depth left
    reverb_depth_l_ = decode_fixed_volume_q15(value);
    audio_diag_.saw_reverb_config_write = true;
    break;
  case 0x186: // Reverb depth right
    reverb_depth_r_ = decode_fixed_volume_q15(value);
    audio_diag_.saw_reverb_config_write = true;
    break;
  case 0x188: // Key On low (voices 0-15)
    if (value != 0) {
      note_key_write_timing();
      if ((spucnt_effective() & 0x8000u) == 0) {
        ++audio_diag_.keyon_ignored_while_disabled;
        break;
      }
    } else if ((spucnt_effective() & 0x8000u) == 0) {
      break;
    }
    // KEY strobe semantics: low write replaces low 16 bits (not OR).
    pending_kon_mask_ =
        (pending_kon_mask_ & 0x00FF0000u) | static_cast<u32>(value);
    break;
  case 0x18A: // Key On high (voices 16-23)
    if ((value & 0x00FFu) != 0) {
      note_key_write_timing();
      if ((value & 0x0001u) != 0) {
        note_v16_kon_write();
      }
      if ((spucnt_effective() & 0x8000u) == 0) {
        ++audio_diag_.keyon_ignored_while_disabled;
        break;
      }
    } else if ((spucnt_effective() & 0x8000u) == 0) {
      break;
    }
    // High register uses only low 8 bits and replaces bits 16..23.
    pending_kon_mask_ =
        (pending_kon_mask_ & 0x0000FFFFu) |
        (static_cast<u32>(value & 0x00FFu) << 16);
    break;
  case 0x18C: // Key Off low
    if (value != 0) {
      note_key_write_timing();
      if ((spucnt_effective() & 0x8000u) == 0) {
        ++audio_diag_.keyoff_ignored_while_disabled;
        break;
      }
    } else if ((spucnt_effective() & 0x8000u) == 0) {
      break;
    }
    pending_koff_mask_ =
        (pending_koff_mask_ & 0x00FF0000u) | static_cast<u32>(value);
    break;
  case 0x18E: // Key Off high
    if ((value & 0x00FFu) != 0) {
      note_key_write_timing();
      if ((value & 0x0001u) != 0) {
        note_v16_koff_write();
      }
      if ((spucnt_effective() & 0x8000u) == 0) {
        ++audio_diag_.keyoff_ignored_while_disabled;
        break;
      }
    } else if ((spucnt_effective() & 0x8000u) == 0) {
      break;
    }
    pending_koff_mask_ =
        (pending_koff_mask_ & 0x0000FFFFu) |
        (static_cast<u32>(value & 0x00FFu) << 16);
    break;
  case 0x190: // Pitch modulation on low
    pitch_mod_mask_ = (pitch_mod_mask_ & 0xFF0000u) | value;
    break;
  case 0x192: // Pitch modulation on high
    pitch_mod_mask_ = (pitch_mod_mask_ & 0x00FFFFu) |
                      (static_cast<u32>(value & 0x00FFu) << 16);
    break;
  case 0x194: // Noise mode on low
    noise_on_mask_ = (noise_on_mask_ & 0xFF0000u) | value;
    break;
  case 0x196: // Noise mode on high
    noise_on_mask_ = (noise_on_mask_ & 0x00FFFFu) |
                     (static_cast<u32>(value & 0x00FFu) << 16);
    break;
  case 0x198: // Reverb on low
    reverb_on_mask_ = (reverb_on_mask_ & 0xFF0000u) | value;
    break;
  case 0x19A: // Reverb on high
    reverb_on_mask_ = (reverb_on_mask_ & 0x00FFFFu) |
                      (static_cast<u32>(value & 0x00FF) << 16);
    break;
  case 0x1A2: // Reverb work area start
    reverb_base_addr_ = (static_cast<u32>(value) * 8u) & SPU_RAM_WORD_MASK;
    reverb_current_addr_ = reverb_base_addr_ >> 1;
    reverb_resample_pos_ = 0;
    reverb_downsample_buffer_ = {};
    reverb_upsample_buffer_ = {};
    audio_diag_.saw_reverb_config_write = true;
    break;
  case 0x1A4: // IRQ address
    irq_addr_ = (static_cast<u32>(value) * 8u) & SPU_RAM_WORD_MASK;
    break;
  case 0x1A6: // Transfer address
    transfer_addr_ =
        (static_cast<u32>(value) * 8u) & static_cast<u32>(SPU_RAM_MASK);
    transfer_busy_cycles_ = 0;
    break;
  case 0x1A8: // Data FIFO
    maybe_raise_irq9_for_ram_access(transfer_addr_, 2);
    spu_ram_[transfer_addr_ & SPU_RAM_MASK] = static_cast<u8>(value & 0xFF);
    spu_ram_[(transfer_addr_ + 1u) & SPU_RAM_MASK] =
        static_cast<u8>(value >> 8);
    transfer_addr_ = (transfer_addr_ + 2u) & static_cast<u32>(SPU_RAM_MASK);
    transfer_busy_cycles_ = 64;
    break;
  case 0x1AA: // SPU control
    {
      const bool was_enabled = (spucnt_ & 0x8000u) != 0;
      const bool now_enabled = (value & 0x8000u) != 0;
      if (was_enabled != now_enabled) {
        if (now_enabled) {
          ++audio_diag_.spucnt_enable_set_events;
          if (spu_disabled_window_active_) {
            const u64 span64 = sample_clock_ - spu_disable_start_sample_;
            const u32 span =
                (span64 > static_cast<u64>(std::numeric_limits<u32>::max()))
                    ? std::numeric_limits<u32>::max()
                    : static_cast<u32>(span64);
            ++audio_diag_.spu_disable_span_events;
            audio_diag_.spu_disable_span_total_samples += span64;
            if (audio_diag_.spu_disable_span_events == 1) {
              audio_diag_.spu_disable_span_min_samples = span;
              audio_diag_.spu_disable_span_max_samples = span;
            } else {
              audio_diag_.spu_disable_span_min_samples =
                  std::min(audio_diag_.spu_disable_span_min_samples, span);
              audio_diag_.spu_disable_span_max_samples =
                  std::max(audio_diag_.spu_disable_span_max_samples, span);
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
        if (g_trace_spu) {
          LOG_DEBUG("SPU: ENABLE %d->%d cpu=%llu sample=%llu", was_enabled ? 1 : 0,
                    now_enabled ? 1 : 0,
                    static_cast<unsigned long long>(cpu_cycle),
                    static_cast<unsigned long long>(sample_clock_));
        }
      }
    }
    ++audio_diag_.spucnt_writes;
    if ((spucnt_ ^ value) & 0x4000u) {
      ++audio_diag_.spucnt_mute_toggle_events;
      // SPUCNT bit14 is mute_n (1=unmuted, 0=muted).
      if ((value & 0x4000u) != 0) {
        ++audio_diag_.spucnt_mute_clear_events;
      } else {
        ++audio_diag_.spucnt_mute_set_events;
      }
    }
    audio_diag_.spucnt_last = value;
    spucnt_ = value;
    // IRQ9 flag is cleared when IRQ enable is disabled.
    if ((value & 0x0040u) == 0) {
      clear_irq9_flag();
    }
    spucnt_mode_pending_ = static_cast<u16>(value & 0x3Fu);
    if (spucnt_mode_pending_ == spucnt_mode_latched_) {
      spucnt_mode_delay_cycles_ = 0;
    } else {
      spucnt_mode_delay_cycles_ = SPUCNT_MODE_APPLY_DELAY_CYCLES;
    }
    break;
  case 0x1B0: // CD input volume left
    cd_vol_l_ = static_cast<s16>(value);
    break;
  case 0x1B2: // CD input volume right
    cd_vol_r_ = static_cast<s16>(value);
    break;
  case 0x1B4: // External input volume left
    ext_vol_l_ = static_cast<s16>(value);
    break;
  case 0x1B6: // External input volume right
    ext_vol_r_ = static_cast<s16>(value);
    break;
  default:
    if (offset >= 0x1C0 && offset <= 0x1FE && (offset & 1u) == 0) {
      write_reverb_reg(offset, value);
    }
    break;
  }
}

void Spu::dma_write(u32 value) {
  if (g_trace_spu && trace_should_log(g_spu_trace_dma_write_counter,
                                      g_trace_burst_spu, g_trace_stride_spu)) {
    LOG_DEBUG("SPU: DMA write addr=0x%05X val=0x%08X", transfer_addr_, value);
  }
  maybe_raise_irq9_for_ram_access(transfer_addr_, 4);
  spu_ram_[transfer_addr_ & SPU_RAM_MASK] = static_cast<u8>(value & 0xFF);
  spu_ram_[(transfer_addr_ + 1u) & SPU_RAM_MASK] =
      static_cast<u8>((value >> 8) & 0xFF);
  spu_ram_[(transfer_addr_ + 2u) & SPU_RAM_MASK] =
      static_cast<u8>((value >> 16) & 0xFF);
  spu_ram_[(transfer_addr_ + 3u) & SPU_RAM_MASK] =
      static_cast<u8>((value >> 24) & 0xFF);
  transfer_addr_ = (transfer_addr_ + 4u) & static_cast<u32>(SPU_RAM_MASK);
  transfer_busy_cycles_ = 64;
}

void Spu::key_on_voice(int v) {
  if (v < 0 || v >= NUM_VOICES) {
    return;
  }
  VoiceState &vs = voices_[v];
  if (v == TRACE_VOICE) {
    ++audio_diag_.v16_kon_apply_events;
    if (v16_waiting_for_koff_) {
      ++audio_diag_.v16_kon_reapply_without_koff;
    }
    v16_waiting_for_koff_ = true;
    v16_last_kon_apply_sample_ = sample_clock_;
    v16_has_last_kon_apply_ = true;
    if (g_trace_spu) {
      LOG_DEBUG("SPU_V16_APPLY KON sample=%llu phase=%d",
                static_cast<unsigned long long>(sample_clock_),
                static_cast<int>(vs.phase));
    }
  }
  if (vs.phase != VoiceState::AdsrPhase::Off && has_last_kon_[v]) {
    const u64 delta64 = sample_clock_ - last_kon_sample_[v];
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
  vs.key_on = true;
  vs.hist1 = 0;
  vs.hist2 = 0;
  vs.gauss_hist = {};
  vs.gauss_ready = false;
  vs.sample_index = 28;
  vs.end_reached = false;
  vs.pitch_counter = 0;
  vs.release_tracking = false;
  vs.release_start_sample = 0;
  vs.adsr_counter = 0;
  vs.sweep_vol_l = 0;
  vs.sweep_vol_r = 0;
  vs.addr = (static_cast<u32>(regs_[(v * VOICE_REG_STRIDE + 0x6) / 2]) * 8u) &
            SPU_RAM_MASK;
  vs.repeat_addr =
      (static_cast<u32>(regs_[(v * VOICE_REG_STRIDE + 0xE) / 2]) * 8u) &
      SPU_RAM_MASK;
  if (vs.repeat_addr == 0) {
    // Hardware commonly treats zero loop-register as "use current start".
    // Without this fallback, end/loop handling can jump to SPU RAM base.
    vs.repeat_addr = vs.addr;
  }

  // ADSR registers and Attack start.
  const u16 adsr_lo = regs_[(v * VOICE_REG_STRIDE + 0x8) / 2];
  const u16 adsr_hi = regs_[(v * VOICE_REG_STRIDE + 0xA) / 2];
  vs.attack_exp = (adsr_lo & 0x8000u) != 0;
  vs.attack_shift = static_cast<u8>((adsr_lo >> 10) & 0x1F);
  vs.attack_step = static_cast<u8>((adsr_lo >> 8) & 0x3);
  vs.decay_shift = static_cast<u8>((adsr_lo >> 4) & 0xF);
  const int sustain_raw = adsr_lo & 0xF;
  vs.sustain_level =
      static_cast<u16>(std::min<s32>((sustain_raw + 1) * 0x800, kEnvMax));
  vs.sustain_exp = (adsr_hi & 0x8000u) != 0;
  vs.sustain_decrease = (adsr_hi & 0x4000u) != 0;
  vs.sustain_shift = static_cast<u8>((adsr_hi >> 8) & 0x1F);
  vs.sustain_step = static_cast<u8>((adsr_hi >> 6) & 0x3);
  vs.release_exp = (adsr_hi & 0x20u) != 0;
  vs.release_shift = static_cast<u8>(adsr_hi & 0x1F);

  vs.phase = VoiceState::AdsrPhase::Attack;
  vs.env_level = 0;
  endx_mask_ &= ~(1u << v);
  ++audio_diag_.key_on_events;
  last_kon_sample_[v] = sample_clock_;
  has_last_kon_[v] = true;
  if (g_trace_spu) {
    LOG_DEBUG(
        "SPU: KON v=%d start=0x%05X repeat=0x%05X adsr1=0x%04X adsr2=0x%04X "
        "atk_sh=%u atk_st=%u dec_sh=%u sus_lv=0x%04X sus_sh=%u sus_st=%u "
        "sus_dec=%d rel_sh=%u rel_exp=%d",
        v, vs.addr, vs.repeat_addr, static_cast<unsigned>(adsr_lo),
        static_cast<unsigned>(adsr_hi), static_cast<unsigned>(vs.attack_shift),
        static_cast<unsigned>(vs.attack_step),
        static_cast<unsigned>(vs.decay_shift),
        static_cast<unsigned>(vs.sustain_level),
        static_cast<unsigned>(vs.sustain_shift),
        static_cast<unsigned>(vs.sustain_step), vs.sustain_decrease ? 1 : 0,
        static_cast<unsigned>(vs.release_shift), vs.release_exp ? 1 : 0);
  }
}

void Spu::key_off_voice(int v) {
  if (v < 0 || v >= NUM_VOICES) {
    return;
  }
  VoiceState &vs = voices_[v];
  if (vs.phase != VoiceState::AdsrPhase::Off) {
    if (v == TRACE_VOICE) {
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
          audio_diag_.v16_kon_to_koff_apply_min_samples =
              std::min(audio_diag_.v16_kon_to_koff_apply_min_samples, delta);
          audio_diag_.v16_kon_to_koff_apply_max_samples =
              std::max(audio_diag_.v16_kon_to_koff_apply_max_samples, delta);
        }
      }
      v16_waiting_for_koff_ = false;
      if (g_trace_spu) {
        LOG_DEBUG("SPU_V16_APPLY KOFF sample=%llu env=0x%04X phase=%d",
                  static_cast<unsigned long long>(sample_clock_),
                  static_cast<unsigned>(vs.env_level),
                  static_cast<int>(vs.phase));
      }
    }
    if (has_last_kon_[v]) {
      const u64 delta64 = sample_clock_ - last_kon_sample_[v];
      const u32 delta =
          (delta64 > static_cast<u64>(std::numeric_limits<u32>::max()))
              ? std::numeric_limits<u32>::max()
              : static_cast<u32>(delta64);
      ++audio_diag_.kon_to_koff_events;
      audio_diag_.kon_to_koff_total_samples += delta64;
      const bool is_first = (audio_diag_.kon_to_koff_events == 1);
      const bool is_new_min =
          is_first || (delta < audio_diag_.kon_to_koff_min_samples);
      if (audio_diag_.kon_to_koff_events == 1) {
        audio_diag_.kon_to_koff_min_samples = delta;
        audio_diag_.kon_to_koff_max_samples = delta;
      } else {
        audio_diag_.kon_to_koff_min_samples =
            std::min(audio_diag_.kon_to_koff_min_samples, delta);
        audio_diag_.kon_to_koff_max_samples =
            std::max(audio_diag_.kon_to_koff_max_samples, delta);
      }

      if (is_new_min) {
        const u32 base = static_cast<u32>(v) * VOICE_REG_STRIDE;
        audio_diag_.kon_to_koff_min_voice = static_cast<u32>(v);
        audio_diag_.kon_to_koff_min_addr = vs.addr & SPU_RAM_MASK;
        audio_diag_.kon_to_koff_min_pitch = regs_[(base + 0x4) / 2];
        audio_diag_.kon_to_koff_min_adsr2 = regs_[(base + 0xA) / 2];
      }

      if (delta < KOFF_DEBOUNCE_SAMPLES) {
        ++audio_diag_.koff_short_window_events;
      }
    }
    if (vs.env_level >= 0x4000u) {
      ++audio_diag_.koff_high_env_events;
    }
    vs.key_on = false;
    vs.end_reached = false;
    vs.phase = VoiceState::AdsrPhase::Release;
    vs.adsr_counter = 0;
    vs.release_tracking = true;
    vs.release_start_sample = sample_clock_;
    ++audio_diag_.key_off_events;
    if (g_trace_spu) {
      LOG_DEBUG("SPU: VOICE v=%d ->Release reason=KOFF sample=%llu env=0x%04X "
                "rel_sh=%u rel_exp=%d",
                v, static_cast<unsigned long long>(sample_clock_),
                static_cast<unsigned>(vs.env_level),
                static_cast<unsigned>(vs.release_shift),
                vs.release_exp ? 1 : 0);
    }
  }
}

/*void Spu::force_off_all_voices_immediate(int v) {
    auto &vs = voices_[v];

    vs.phase = VoiceState::AdsrPhase::Off;
    vs.key_on = false;
    vs.env_level = 0;
    vs.adsr_counter = 0;
    vs.release_tracking = false;
}*/



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
    vs.end_reached = false;
    vs.release_tracking = false;
    vs.current_vol_l = 0;
    vs.current_vol_r = 0;
    if (g_trace_spu) {
      LOG_DEBUG("SPU: VOICE v=%d ->Off reason=SPU_DISABLE sample=%llu", v,
                static_cast<unsigned long long>(sample_clock_));
    }
  }
  if (forced != 0) {
    audio_diag_.spu_disable_forced_off_voices += forced;
  }
}

void Spu::apply_pending_key_strobes() {
  const u32 valid_mask = 0x00FFFFFFu;
  const u32 kon = pending_kon_mask_ & valid_mask;
  const u32 koff = pending_koff_mask_ & valid_mask;
  pending_kon_mask_ = 0;
  pending_koff_mask_ = 0;
  const u32 v16_mask = (1u << TRACE_VOICE);
  if ((kon & v16_mask) != 0) {
    ++audio_diag_.v16_strobe_samples_with_kon;
  }
  if ((koff & v16_mask) != 0) {
    ++audio_diag_.v16_strobe_samples_with_koff;
  }
  if ((kon & v16_mask) != 0 && (koff & v16_mask) != 0) {
    ++audio_diag_.v16_strobe_samples_with_both;
  }
  if (g_trace_spu && (((kon | koff) & v16_mask) != 0)) {
    LOG_DEBUG("SPU_V16_STROBE sample=%llu kon=%u koff=%u",
              static_cast<unsigned long long>(sample_clock_),
              (kon & v16_mask) != 0 ? 1u : 0u,
              (koff & v16_mask) != 0 ? 1u : 0u);
  }

  // Apply KOFF first, then KON so KON wins same-sample conflicts.
  for (int v = 0; v < NUM_VOICES; ++v) {
    if (koff & (1u << v)) {
      key_off_voice(v);
    }
  }
  for (int v = 0; v < NUM_VOICES; ++v) {
    if (kon & (1u << v)) {
      key_on_voice(v);
    }
  }
}

u32 Spu::dma_read() {
  if (g_trace_spu && trace_should_log(g_spu_trace_dma_read_counter,
                                      g_trace_burst_spu, g_trace_stride_spu)) {
    LOG_DEBUG("SPU: DMA read addr=0x%05X", transfer_addr_);
  }
  maybe_raise_irq9_for_ram_access(transfer_addr_, 4);
  u32 value = 0;
  value |= static_cast<u32>(spu_ram_[transfer_addr_ & SPU_RAM_MASK]);
  value |= static_cast<u32>(spu_ram_[(transfer_addr_ + 1u) & SPU_RAM_MASK])
           << 8;
  value |= static_cast<u32>(spu_ram_[(transfer_addr_ + 2u) & SPU_RAM_MASK])
           << 16;
  value |= static_cast<u32>(spu_ram_[(transfer_addr_ + 3u) & SPU_RAM_MASK])
           << 24;
  transfer_addr_ = (transfer_addr_ + 4u) & static_cast<u32>(SPU_RAM_MASK);
  transfer_busy_cycles_ = 64;
  return value;
}

float Spu::decode_volume(u16 raw) const {
  if ((raw & 0x8000u) == 0) {
    return static_cast<float>(decode_fixed_volume_q15(raw)) / 32768.0f;
  }

  // Sweep mode decode path (register-bit driven approximation of hardware
  // sweep envelope timing and exponent handling).
  const s16 base = static_cast<s16>((raw & 0x7FFFu) << 1);
  const s32 stepped = decode_sweep_step(raw, base);
  return static_cast<float>(sat16(stepped)) / 32767.0f;
}

s32 Spu::decode_sweep_step(u16 raw, s16 current) const {
  const bool exponential = (raw & 0x4000u) != 0;
  const bool decreasing = (raw & 0x2000u) != 0;
  const u8 shift = static_cast<u8>((raw >> 2) & 0x1F);
  const u8 step = static_cast<u8>(raw & 0x3);
  s32 delta = adsr_base_step(step, decreasing);
  delta <<= std::max(0, 11 - static_cast<int>(shift));
  if (exponential && !decreasing && current > 0x6000) {
    delta >>= 2;
  }
  if (exponential && decreasing) {
    delta = (delta * static_cast<s32>(current)) / 0x8000;
  }
  return static_cast<s32>(current) + delta;
}

bool Spu::decode_block(int voice) {
  static constexpr int kFilterA[5] = {0, 60, 115, 98, 122};
  static constexpr int kFilterB[5] = {0, 0, -52, -55, -60};

  VoiceState &vs = voices_[voice];
  const u32 block_addr = vs.addr & SPU_RAM_MASK;
  maybe_raise_irq9_for_ram_access(block_addr, 16);
  const u8 predict_shift = spu_ram_[block_addr & SPU_RAM_MASK];
  const u8 flags = spu_ram_[(block_addr + 1) & SPU_RAM_MASK];
  vs.last_block_addr = block_addr;
  vs.last_adpcm_flags = flags;
  const int raw_shift = predict_shift & 0x0F;
  // Reserved shift values 13..15 behave like shift=9 on real hardware.
  const int shift = (raw_shift > 12) ? 9 : raw_shift;
  const int filter = (predict_shift >> 4) & 0x07;
  const int f = (filter <= 4) ? filter : 0;

  for (int i = 0; i < 28; ++i) {
    const u8 packed =
        spu_ram_[(block_addr + 2u + static_cast<u32>(i >> 1)) & SPU_RAM_MASK];
    int nibble = (i & 1) ? (packed >> 4) : (packed & 0x0F);
    if (nibble & 0x8) {
      nibble -= 16;
    }
    int sample = (nibble << 12);
    sample >>= shift;
    sample += (kFilterA[f] * vs.hist1 + kFilterB[f] * vs.hist2 + 32) >> 6;
    sample = std::clamp(sample, -32768, 32767);

    vs.hist2 = vs.hist1;
    vs.hist1 = static_cast<s16>(sample);
    vs.decoded[i] = static_cast<s16>(sample);
  }

  vs.sample_index = 0;
  vs.addr = (block_addr + 16u) & SPU_RAM_MASK;

  // ADPCM flags: bit2 loop start, bit1 loop repeat, bit0 loop end.
  if (flags & 0x04) {
    vs.repeat_addr = block_addr;
  }
  if (flags & 0x01) {
    ++audio_diag_.end_flag_events;
    // ENDX latches on end-flag blocks (including loop-repeat end markers).
    endx_mask_ |= (1u << voice);
    // Jump is applied after playing this block; preloading address here is fine.
    vs.addr = vs.repeat_addr;
    if ((flags & 0x02) != 0) {
      // End+Repeat: continue playback from repeat address.
      vs.end_reached = false;
      ++audio_diag_.loop_end_events;
    } else {
      // End+Mute: finish current block, then silence voice.
      // When noise mode is enabled, hardware continues and ignores End+Mute.
      const bool noise_voice = (noise_on_mask_ & (1u << voice)) != 0;
      //*vs.end_reached = !noise_voice;
      if (!noise_voice) {
          vs.phase = VoiceState::AdsrPhase::Off;
          vs.key_on = false;
          vs.env_level = 0;
          vs.adsr_counter = 0;
          vs.release_tracking = false;
          vs.current_vol_l = 0;
          vs.current_vol_r = 0;
          vs.end_reached = true;
      }
      ++audio_diag_.nonloop_end_events;
    }
  } else {
    vs.end_reached = false;
  }

  return true;
}

bool Spu::fetch_decoded_sample(int voice, s16 &sample) {
  VoiceState &vs = voices_[voice];
  if (vs.phase == VoiceState::AdsrPhase::Off) {
    sample = 0;
    return false;
  }
  if (vs.sample_index >= 28) {
    if (vs.end_reached) {
      vs.end_reached = false;
      // End+Mute forces the voice off after completing the flagged block.
      ++audio_diag_.off_due_to_end_flag;
      vs.phase = VoiceState::AdsrPhase::Off;
      vs.env_level = 0;
      vs.adsr_counter = 0;
      vs.release_tracking = false;
      vs.key_on = false;
      if (g_trace_spu) {
        LOG_DEBUG(
            "SPU: VOICE v=%d ->Off reason=END_FLAG sample=%llu block=0x%05X "
            "flags=0x%02X",
            voice, static_cast<unsigned long long>(sample_clock_),
            vs.last_block_addr & SPU_RAM_MASK, static_cast<unsigned>(vs.last_adpcm_flags));
      }
      sample = 0;
      return false;
    }
    if (!decode_block(voice) || vs.sample_index >= 28) {
      sample = 0;
      return false;
    }
  }
  sample = vs.decoded[vs.sample_index++];
  return true;
}

void Spu::prime_gaussian_history(int voice) {
  VoiceState &vs = voices_[voice];
  if (vs.gauss_ready) {
    return;
  }
  for (int i = 0; i < 4; ++i) {
    s16 s = 0;
    fetch_decoded_sample(voice, s);
    vs.gauss_hist[static_cast<size_t>(i)] = s;
  }
  vs.gauss_ready = true;
}

void Spu::advance_gaussian_history(int voice) {
  VoiceState &vs = voices_[voice];
  s16 next = 0;
  fetch_decoded_sample(voice, next);
  vs.gauss_hist[0] = vs.gauss_hist[1];
  vs.gauss_hist[1] = vs.gauss_hist[2];
  vs.gauss_hist[2] = vs.gauss_hist[3];
  vs.gauss_hist[3] = next;
}

s16 Spu::gaussian_interpolate(const VoiceState &vs) const {
  if (!vs.gauss_ready) {
    return 0;
  }
  const u32 n = (vs.pitch_counter >> 4) & 0xFFu;
  const s32 g0 = kGaussTable[0x0FFu - n];
  const s32 g1 = kGaussTable[0x1FFu - n];
  const s32 g2 = kGaussTable[0x100u + n];
  const s32 g3 = kGaussTable[0x000u + n];
  const s32 s0 = static_cast<s32>(vs.gauss_hist[0]);
  const s32 s1 = static_cast<s32>(vs.gauss_hist[1]);
  const s32 s2 = static_cast<s32>(vs.gauss_hist[2]);
  const s32 s3 = static_cast<s32>(vs.gauss_hist[3]);
  const s32 out = (s0 * g0 + s1 * g1 + s2 * g2 + s3 * g3 + 0x4000) >> 15;
  return sat16(out);
}

s16 Spu::apply_envelope(s16 sample, const VoiceState &vs) const {
  const s32 env = static_cast<s32>(vs.env_level);
  const s32 mixed = (static_cast<s32>(sample) * env) / kEnvMax;
  return sat16(mixed);
}

void Spu::write_reverb_reg(u32 offset, u16 value) {
  audio_diag_.saw_reverb_config_write = true;
  switch (offset) {
  case 0x1C0:
    reverb_regs_.dAPF1 = value;
    break;
  case 0x1C2:
    reverb_regs_.dAPF2 = value;
    break;
  case 0x1C4:
    reverb_regs_.vIIR = static_cast<s16>(value);
    break;
  case 0x1C6:
    reverb_regs_.vCOMB1 = static_cast<s16>(value);
    break;
  case 0x1C8:
    reverb_regs_.vCOMB2 = static_cast<s16>(value);
    break;
  case 0x1CA:
    reverb_regs_.vCOMB3 = static_cast<s16>(value);
    break;
  case 0x1CC:
    reverb_regs_.vCOMB4 = static_cast<s16>(value);
    break;
  case 0x1CE:
    reverb_regs_.vWALL = static_cast<s16>(value);
    break;
  case 0x1D0:
    reverb_regs_.vAPF1 = static_cast<s16>(value);
    break;
  case 0x1D2:
    reverb_regs_.vAPF2 = static_cast<s16>(value);
    break;
  case 0x1D4:
    reverb_regs_.mLSAME = static_cast<s16>(value);
    break;
  case 0x1D6:
    reverb_regs_.mRSAME = static_cast<s16>(value);
    break;
  case 0x1D8:
    reverb_regs_.mLCOMB1 = static_cast<s16>(value);
    break;
  case 0x1DA:
    reverb_regs_.mRCOMB1 = static_cast<s16>(value);
    break;
  case 0x1DC:
    reverb_regs_.mLCOMB2 = static_cast<s16>(value);
    break;
  case 0x1DE:
    reverb_regs_.mRCOMB2 = static_cast<s16>(value);
    break;
  case 0x1E0:
    reverb_regs_.dLSAME = static_cast<s16>(value);
    break;
  case 0x1E2:
    reverb_regs_.dRSAME = static_cast<s16>(value);
    break;
  case 0x1E4:
    reverb_regs_.mLDIFF = static_cast<s16>(value);
    break;
  case 0x1E6:
    reverb_regs_.mRDIFF = static_cast<s16>(value);
    break;
  case 0x1E8:
    reverb_regs_.mLCOMB3 = static_cast<s16>(value);
    break;
  case 0x1EA:
    reverb_regs_.mRCOMB3 = static_cast<s16>(value);
    break;
  case 0x1EC:
    reverb_regs_.mLCOMB4 = static_cast<s16>(value);
    break;
  case 0x1EE:
    reverb_regs_.mRCOMB4 = static_cast<s16>(value);
    break;
  case 0x1F0:
    reverb_regs_.dLDIFF = static_cast<s16>(value);
    break;
  case 0x1F2:
    reverb_regs_.dRDIFF = static_cast<s16>(value);
    break;
  case 0x1F4:
    reverb_regs_.mLAPF1 = static_cast<s16>(value);
    break;
  case 0x1F6:
    reverb_regs_.mRAPF1 = static_cast<s16>(value);
    break;
  case 0x1F8:
    reverb_regs_.mLAPF2 = static_cast<s16>(value);
    break;
  case 0x1FA:
    reverb_regs_.mRAPF2 = static_cast<s16>(value);
    break;
  case 0x1FC:
    reverb_regs_.vLIN = static_cast<s16>(value);
    break;
  case 0x1FE:
    reverb_regs_.vRIN = static_cast<s16>(value);
    break;
  default:
    break;
  }
}

u32 Spu::reverb_memory_address(u32 address_words) const {
  // Addressing matches hardware quirk behavior used by Mednafen/DuckStation.
  static constexpr u32 kWordMask = SPU_RAM_MASK >> 1;
  const u32 base_word = (reverb_base_addr_ >> 1) & kWordMask;
  u32 offset = reverb_current_addr_ + (address_words & kWordMask);
  offset += base_word & static_cast<u32>(static_cast<s32>(offset << 13) >> 31);
  return (offset & kWordMask) << 1;
}

s16 Spu::reverb_read_s16(u32 address_words, s32 offset_words) const {
  const s32 word_addr =
      static_cast<s32>(address_words << 2) + offset_words;
  const u32 a = static_cast<u32>(reverb_memory_address(
      static_cast<u32>(word_addr))) &
                SPU_RAM_WORD_MASK;
  return static_cast<s16>(
      static_cast<u16>(spu_ram_[a]) |
      (static_cast<u16>(spu_ram_[(a + 1) & SPU_RAM_MASK]) << 8));
}

void Spu::reverb_write_s16(u32 address_words, s16 value) {
  const u32 a =
      static_cast<u32>(reverb_memory_address(address_words << 2)) &
      SPU_RAM_WORD_MASK;
  maybe_raise_irq9_for_ram_access(a, 2);
  spu_ram_[a] = static_cast<u8>(value & 0xFF);
  spu_ram_[(a + 1) & SPU_RAM_MASK] = static_cast<u8>((value >> 8) & 0xFF);
  ++audio_diag_.reverb_ram_writes;
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

s16 Spu::next_noise_sample() {
  // PSX-SPX noise generator:
  //  - shift: SPUCNT bits13..10
  //  - step : SPUCNT bits9..8 (+4 bias)
  //  - timer runs at 44.1kHz and clocks a 16-bit LFSR-like shift register.
  const u16 spucnt_eff = spucnt_effective();
  const s32 noise_shift = static_cast<s32>((spucnt_eff >> 10) & 0xFu);
  const s32 noise_step = 4 + static_cast<s32>((spucnt_eff >> 8) & 0x3u);
  const s32 timer_reload = 0x20000 >> noise_shift;

  noise_timer_ -= noise_step;
  const u16 level_u = static_cast<u16>(noise_level_);
  const u16 parity = static_cast<u16>(
      ((level_u >> 15) ^ (level_u >> 12) ^ (level_u >> 11) ^ (level_u >> 10) ^
       1u) &
      1u);
  if (noise_timer_ < 0) {
    noise_level_ = static_cast<s16>((level_u << 1) | parity);
  }
  if (noise_timer_ < 0) {
    noise_timer_ += timer_reload;
  }
  if (noise_timer_ < 0) {
    noise_timer_ += timer_reload;
  }
  return noise_level_;
}

void Spu::mix_reverb(float in_l, float in_r, float &wet_l, float &wet_r) {
  wet_l = 0.0f;
  wet_r = 0.0f;
  if ((spucnt_effective() & 0x0080u) == 0 || g_low_spec_mode) {
    reverb_resample_pos_ = 0;
    reverb_downsample_buffer_ = {};
    reverb_upsample_buffer_ = {};
    return;
  }

  // Input/output resampling to 22.05kHz (39-tap FIR, zero taps removed).
  static constexpr std::array<s32, 20> kResampleCoeff = {
      -0x0001, 0x0002,  -0x000A, 0x0023,  -0x0067, 0x010A,  -0x0268,
      0x0534,  -0x0B90, 0x2806,  0x2806,  -0x0B90, 0x0534,  -0x0268,
      0x010A,  -0x0067, 0x0023,  -0x000A, 0x0002,  -0x0001};
  auto clamp16 = [](s32 v) { return std::clamp(v, -32768, 32767); };
  auto apply_volume = [](s32 sample, s16 volume) {
    return (sample * static_cast<s32>(volume)) >> 15;
  };
  auto neg = [](s32 sample) { return (sample == -32768) ? 0x7FFF : -sample; };
  auto iiasm = [&](s16 sample) {
    if (reverb_regs_.vIIR == -32768) {
      return (sample == -32768) ? 0 : (sample * -65536);
    }
    return sample * (32768 - static_cast<s32>(reverb_regs_.vIIR));
  };
  auto ureg = [](s16 v) { return static_cast<u32>(static_cast<u16>(v)); };

  const s16 in_l_s = sat16(static_cast<s32>(
      std::lround(std::clamp(in_l, -1.0f, 1.0f) * 32767.0f)));
  const s16 in_r_s = sat16(static_cast<s32>(
      std::lround(std::clamp(in_r, -1.0f, 1.0f) * 32767.0f)));

  const u32 pos = reverb_resample_pos_;
  reverb_downsample_buffer_[0][pos | 0x00] =
      reverb_downsample_buffer_[0][pos | 0x40] = in_l_s;
  reverb_downsample_buffer_[1][pos | 0x00] =
      reverb_downsample_buffer_[1][pos | 0x40] = in_r_s;

  std::array<s32, 2> out{};
  if ((pos & 1u) != 0) {
    std::array<s32, 2> downsampled{};
    for (size_t ch = 0; ch < 2; ++ch) {
      const s16 *src = &reverb_downsample_buffer_[ch][(pos - 38u) & 0x3Fu];
      s64 acc = 0;
      for (size_t i = 0; i < kResampleCoeff.size(); ++i) {
        acc += static_cast<s64>(kResampleCoeff[i]) *
               static_cast<s64>(src[i * 2]);
      }
      acc += static_cast<s64>(0x4000) * static_cast<s64>(src[19]);
      downsampled[ch] = clamp16(static_cast<s32>(acc >> 15));
    }

    const u32 iir_dest_a[2] = {ureg(reverb_regs_.mLSAME), ureg(reverb_regs_.mRSAME)};
    const u32 acc_src_a[2] = {ureg(reverb_regs_.mLCOMB1), ureg(reverb_regs_.mRCOMB1)};
    const u32 acc_src_b[2] = {ureg(reverb_regs_.mLCOMB2), ureg(reverb_regs_.mRCOMB2)};
    const u32 iir_src_a[2] = {ureg(reverb_regs_.dLSAME), ureg(reverb_regs_.dRSAME)};
    const u32 iir_dest_b[2] = {ureg(reverb_regs_.mLDIFF), ureg(reverb_regs_.mRDIFF)};
    const u32 acc_src_c[2] = {ureg(reverb_regs_.mLCOMB3), ureg(reverb_regs_.mRCOMB3)};
    const u32 acc_src_d[2] = {ureg(reverb_regs_.mLCOMB4), ureg(reverb_regs_.mRCOMB4)};
    const u32 iir_src_b[2] = {ureg(reverb_regs_.dLDIFF), ureg(reverb_regs_.dRDIFF)};
    const u32 mix_dest_a[2] = {ureg(reverb_regs_.mLAPF1), ureg(reverb_regs_.mRAPF1)};
    const u32 mix_dest_b[2] = {ureg(reverb_regs_.mLAPF2), ureg(reverb_regs_.mRAPF2)};
    const s16 in_coef[2] = {reverb_regs_.vLIN, reverb_regs_.vRIN};

    for (size_t ch = 0; ch < 2; ++ch) {
      const s32 iir_input_a =
          clamp16((((static_cast<s32>(reverb_read_s16(iir_src_a[ch])) *
                     static_cast<s32>(reverb_regs_.vWALL)) >>
                    14) +
                   ((downsampled[ch] * static_cast<s32>(in_coef[ch])) >> 14)) >>
                  1);
      const s32 iir_input_b = clamp16(
          (((static_cast<s32>(reverb_read_s16(iir_src_b[ch ^ 1])) *
             static_cast<s32>(reverb_regs_.vWALL)) >>
            14) +
           ((downsampled[ch] * static_cast<s32>(in_coef[ch])) >> 14)) >>
          1);
      const s32 iir_a = clamp16(
          (((iir_input_a * static_cast<s32>(reverb_regs_.vIIR)) >> 14) +
           (iiasm(reverb_read_s16(iir_dest_a[ch], -1)) >> 14)) >>
          1);
      const s32 iir_b = clamp16(
          (((iir_input_b * static_cast<s32>(reverb_regs_.vIIR)) >> 14) +
           (iiasm(reverb_read_s16(iir_dest_b[ch], -1)) >> 14)) >>
          1);
      reverb_write_s16(iir_dest_a[ch], static_cast<s16>(iir_a));
      reverb_write_s16(iir_dest_b[ch], static_cast<s16>(iir_b));

      const s32 acc =
          ((static_cast<s32>(reverb_read_s16(acc_src_a[ch])) *
            static_cast<s32>(reverb_regs_.vCOMB1)) >>
           14) +
          ((static_cast<s32>(reverb_read_s16(acc_src_b[ch])) *
            static_cast<s32>(reverb_regs_.vCOMB2)) >>
           14) +
          ((static_cast<s32>(reverb_read_s16(acc_src_c[ch])) *
            static_cast<s32>(reverb_regs_.vCOMB3)) >>
           14) +
          ((static_cast<s32>(reverb_read_s16(acc_src_d[ch])) *
            static_cast<s32>(reverb_regs_.vCOMB4)) >>
           14);

      const s32 fb_a = static_cast<s32>(
          reverb_read_s16(mix_dest_a[ch] - ureg(static_cast<s16>(reverb_regs_.dAPF1))));
      const s32 fb_b = static_cast<s32>(
          reverb_read_s16(mix_dest_b[ch] - ureg(static_cast<s16>(reverb_regs_.dAPF2))));
      const s32 mda = clamp16(
          (acc + ((fb_a * neg(static_cast<s32>(reverb_regs_.vAPF1))) >> 14)) >>
          1);
      const s32 mdb = clamp16(
          fb_a + ((((mda * static_cast<s32>(reverb_regs_.vAPF1)) >> 14) +
                   ((fb_b * neg(static_cast<s32>(reverb_regs_.vAPF2))) >> 14)) >>
                  1));

      const s16 up = sat16(clamp16(
          fb_b + ((mdb * static_cast<s32>(reverb_regs_.vAPF2)) >> 15)));
      reverb_upsample_buffer_[ch][(pos >> 1) | 0x20] = up;
      reverb_upsample_buffer_[ch][pos >> 1] = up;

      reverb_write_s16(mix_dest_a[ch], static_cast<s16>(mda));
      reverb_write_s16(mix_dest_b[ch], static_cast<s16>(mdb));
    }

    static constexpr u32 kWordMask = SPU_RAM_MASK >> 1;
    reverb_current_addr_ = (reverb_current_addr_ + 1u) & kWordMask;
    if (reverb_current_addr_ == 0) {
      reverb_current_addr_ = (reverb_base_addr_ >> 1) & kWordMask;
    }

    for (size_t ch = 0; ch < 2; ++ch) {
      const s16 *src =
          &reverb_upsample_buffer_[ch][((pos >> 1) - 19u) & 0x1Fu];
      s64 acc = 0;
      for (size_t i = 0; i < kResampleCoeff.size(); ++i) {
        acc += static_cast<s64>(kResampleCoeff[i]) *
               static_cast<s64>(src[i]);
      }
      out[ch] = clamp16(static_cast<s32>(acc >> 14));
    }
    ++audio_diag_.reverb_mix_frames;
  } else {
    const size_t idx = (((pos >> 1) - 19u) & 0x1Fu) + 9u;
    out[0] = reverb_upsample_buffer_[0][idx];
    out[1] = reverb_upsample_buffer_[1][idx];
  }

  reverb_resample_pos_ = (pos + 1u) & 0x3Fu;

  wet_l = static_cast<float>(sat16(apply_volume(out[0], reverb_depth_l_))) /
          32768.0f;
  wet_r = static_cast<float>(sat16(apply_volume(out[1], reverb_depth_r_))) /
          32768.0f;
}

void Spu::tick_volume_sweep() {
  if (regs_[0x180 / 2] & 0x8000u) {
    master_vol_l_ = sat16(decode_sweep_step(regs_[0x180 / 2], master_vol_l_));
  }
  if (regs_[0x182 / 2] & 0x8000u) {
    master_vol_r_ = sat16(decode_sweep_step(regs_[0x182 / 2], master_vol_r_));
  }
  if (regs_[0x184 / 2] & 0x8000u) {
    reverb_depth_l_ =
        sat16(decode_sweep_step(regs_[0x184 / 2], reverb_depth_l_));
  }
  if (regs_[0x186 / 2] & 0x8000u) {
    reverb_depth_r_ =
        sat16(decode_sweep_step(regs_[0x186 / 2], reverb_depth_r_));
  }
}

void Spu::push_cd_audio_samples(const std::vector<s16> &samples,
                                u32 sample_rate) {
  if (samples.empty()) {
    return;
  }
  if ((samples.size() & 1u) != 0) {
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
             cd_input_read_pos_ >= cd_input_samples_.size() / 2) {
    cd_input_samples_.erase(cd_input_samples_.begin(),
                            cd_input_samples_.begin() +
                                static_cast<s64>(cd_input_read_pos_));
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

void Spu::queue_host_audio(const std::vector<s16> &samples) {
  if (samples.empty()) {
    return;
  }

  if (capture_enabled_) {
    if (capture_samples_.size() + samples.size() > CAPTURE_MAX_SAMPLES) {
      size_t drop =
          capture_samples_.size() + samples.size() - CAPTURE_MAX_SAMPLES;
      drop &= ~static_cast<size_t>(1);
      if (drop > 0 && drop < capture_samples_.size()) {
        capture_samples_.erase(capture_samples_.begin(),
                               capture_samples_.begin() +
                                   static_cast<s64>(drop));
      } else if (drop >= capture_samples_.size()) {
        capture_samples_.clear();
      }
    }
    capture_samples_.insert(capture_samples_.end(), samples.begin(),
                            samples.end());
    audio_diag_.capture_frames += samples.size() / 2;
    // Deterministic offline capture mode: avoid SDL queue coupling.
    return;
  }

  if (!audio_enabled_ || audio_device_ == 0) {
    return;
  }

  host_staging_samples_.insert(host_staging_samples_.end(), samples.begin(),
                               samples.end());

  if (host_staging_samples_.size() > HOST_STAGING_MAX_SAMPLES) {
    size_t drop = host_staging_samples_.size() - HOST_STAGING_MAX_SAMPLES;
    drop &= ~static_cast<size_t>(1);
    if (drop > 0) {
      host_staging_samples_.erase(host_staging_samples_.begin(),
                                  host_staging_samples_.begin() +
                                      static_cast<s64>(drop));
      audio_diag_.dropped_frames += drop / 2;
      ++audio_diag_.overrun_events;
    }
  }

  while (!host_staging_samples_.empty()) {
    const u32 queued = SDL_GetQueuedAudioSize(audio_device_);
    audio_diag_.queue_last_bytes = queued;
    audio_diag_.queue_peak_bytes =
        std::max(audio_diag_.queue_peak_bytes, queued);

    if (queued >= HOST_MAX_QUEUE_BYTES) {
      break;
    }

    u32 room = 0;
    if (queued < HOST_TARGET_QUEUE_BYTES) {
      room = HOST_TARGET_QUEUE_BYTES - queued;
    } else {
      room = HOST_MAX_QUEUE_BYTES - queued;
    }
    if (room < sizeof(s16) * 2u) {
      break;
    }

    size_t samples_room = room / sizeof(s16);
    samples_room &= ~static_cast<size_t>(1);
    size_t to_queue = std::min(samples_room, host_staging_samples_.size());
    to_queue &= ~static_cast<size_t>(1);
    if (to_queue == 0) {
      break;
    }

    SDL_QueueAudio(audio_device_, host_staging_samples_.data(),
                   static_cast<Uint32>(to_queue * sizeof(s16)));
    audio_diag_.queued_frames += to_queue / 2;
    host_staging_samples_.erase(host_staging_samples_.begin(),
                                host_staging_samples_.begin() +
                                    static_cast<s64>(to_queue));
  }
}

void Spu::tick(u32 cycles) {
  const bool profile_detailed = g_profile_detailed_timing;
  std::chrono::high_resolution_clock::time_point start{};
  if (profile_detailed) {
    start = std::chrono::high_resolution_clock::now();
  }
  tick_spucnt_mode_delay(cycles);

  if (transfer_busy_cycles_ > 0) {
    transfer_busy_cycles_ =
        (transfer_busy_cycles_ > cycles) ? (transfer_busy_cycles_ - cycles) : 0;
  }

  const double accum_before = sample_accum_;
  sample_accum_ +=
      (static_cast<double>(cycles) * static_cast<double>(SAMPLE_RATE)) /
      static_cast<double>(psx::CPU_CLOCK_HZ);
#ifndef NDEBUG
  assert(sample_accum_ >= accum_before);
#endif

  const int samples_to_gen = static_cast<int>(sample_accum_);
  if (samples_to_gen <= 0) {
    return;
  }
  sample_accum_ -= samples_to_gen;
  audio_diag_.generated_frames += static_cast<u64>(samples_to_gen);

  const u16 spucnt_eff = spucnt_effective();
  const bool enabled = (spucnt_eff & 0x8000u) != 0;
  const bool muted = (spucnt_eff & 0x4000u) == 0;

  if (!enabled) {
    std::vector<s16> out(static_cast<size_t>(samples_to_gen) * 2, 0);
    for (int s = 0; s < samples_to_gen; ++s) {
      ++sample_clock_;
      capture_half_ ^= 1u;
      ++audio_diag_.muted_output_frames;
    }
    queue_host_audio(out);
    if (profile_detailed && sys_) {
      const auto end = std::chrono::high_resolution_clock::now();
      sys_->add_spu_time(
          std::chrono::duration<double, std::milli>(end - start).count());
    }
    return;
  }

  audio_diag_.reverb_enabled = (spucnt_eff & 0x0080u) != 0;
  audio_diag_.gaussian_active = true;

  std::vector<s16> out(static_cast<size_t>(samples_to_gen) * 2);
  for (int s = 0; s < samples_to_gen; ++s) {
    ++sample_clock_;
    capture_half_ ^= 1u;
    apply_pending_key_strobes();
    tick_volume_sweep();
    const s16 noise_raw = next_noise_sample();
    std::array<s16, NUM_VOICES> voice_out_for_pmod{};
    float mix_l = 0.0f;
    float mix_r = 0.0f;
    float wet_l = 0.0f;
    float wet_r = 0.0f;
    float rev_send_l = 0.0f;
    float rev_send_r = 0.0f;
    u32 logical_voices_this_sample = 0;
    u32 env_voices_this_sample = 0;
    u32 audible_voices_this_sample = 0;

    for (int v = 0; v < NUM_VOICES; ++v) {
      VoiceState &vs = voices_[v];
      if (vs.phase == VoiceState::AdsrPhase::Off) {
        vs.current_vol_l = 0;
        vs.current_vol_r = 0;
        voice_out_for_pmod[static_cast<size_t>(v)] = 0;
        continue;
      }
      ++logical_voices_this_sample;
      const bool noise_voice = (noise_on_mask_ & (1u << v)) != 0;
      prime_gaussian_history(v);

      const u32 base = static_cast<u32>(v) * VOICE_REG_STRIDE;
      const u16 pitch = regs_[(base + 0x4) / 2];
      const u16 vol_reg_l = regs_[(base + 0x0) / 2];
      const u16 vol_reg_r = regs_[(base + 0x2) / 2];
      float vol_l, vol_r;
      // Per-voice volume: sweep mode (bit15 set) vs fixed mode.
      if (vol_reg_l & 0x8000u) {
        // Sweep mode left: tick the persistent sweep state.
        vs.sweep_vol_l = sat16(decode_sweep_step(vol_reg_l, vs.sweep_vol_l));
        vol_l = q15_to_float(vs.sweep_vol_l);
      } else {
        // Fixed mode left: use register value directly, also latch into sweep
        // state.
        vs.sweep_vol_l = decode_fixed_volume_q15(vol_reg_l);
        vol_l = q15_to_float(vs.sweep_vol_l);
      }
      if (vol_reg_r & 0x8000u) {
        vs.sweep_vol_r = sat16(decode_sweep_step(vol_reg_r, vs.sweep_vol_r));
        vol_r = q15_to_float(vs.sweep_vol_r);
      } else {
        vs.sweep_vol_r = decode_fixed_volume_q15(vol_reg_r);
        vol_r = q15_to_float(vs.sweep_vol_r);
      }
      u32 step = static_cast<u32>(pitch);
      if (v > 0 && ((pitch_mod_mask_ & (1u << v)) != 0)) {
        const s32 pitch_signed = static_cast<s16>(pitch);
        const s32 factor = static_cast<s32>(
            static_cast<u16>(voice_out_for_pmod[static_cast<size_t>(v - 1)] +
                             0x8000));
        const s64 mod_prod =
            static_cast<s64>(pitch_signed) * static_cast<s64>(factor);
        step = static_cast<u32>(static_cast<s32>(mod_prod >> 15)) & 0xFFFFu;
      }
      if (step > 0x3FFFu) {
        step = 0x3FFFu;
      }

      tick_adsr(v, vs);
      const s16 raw = noise_voice ? noise_raw : gaussian_interpolate(vs);
      const s16 env = apply_envelope(raw, vs);
      voice_out_for_pmod[static_cast<size_t>(v)] = env;
      const float smp = q15_to_float(env);

      const float v_l = smp * vol_l;
      const float v_r = smp * vol_r;
      vs.current_vol_l = sat16(static_cast<s32>(
          std::lround(std::clamp(v_l, -1.0f, 1.0f) * 32767.0f)));
      vs.current_vol_r = sat16(static_cast<s32>(
          std::lround(std::clamp(v_r, -1.0f, 1.0f) * 32767.0f)));
      if (vs.env_level > 0) {
        ++env_voices_this_sample;
      }
      if (vs.env_level > 0 &&
          (vs.current_vol_l != 0 || vs.current_vol_r != 0)) {
        ++audible_voices_this_sample;
      }
      mix_l += v_l;
      mix_r += v_r;
      if (reverb_on_mask_ & (1u << v)) {
        rev_send_l += v_l;
        rev_send_r += v_r;
      }

      vs.pitch_counter += step;
      while (vs.pitch_counter >= 0x1000u) {
        vs.pitch_counter -= 0x1000u;
        advance_gaussian_history(v);
      }
    }
    audio_diag_.logical_voice_samples += 1;
    audio_diag_.logical_voice_accum += logical_voices_this_sample;
    if (logical_voices_this_sample > audio_diag_.logical_voice_peak) {
      audio_diag_.logical_voice_peak = logical_voices_this_sample;
      audio_diag_.logical_voice_peak_sample = sample_clock_;
      audio_diag_.logical_voice_peak_key_on_events = audio_diag_.key_on_events;
      audio_diag_.logical_voice_peak_key_off_events = audio_diag_.key_off_events;
      audio_diag_.logical_voice_peak_endx_mask = endx_mask_ & 0x00FFFFFFu;
    }
    audio_diag_.env_voice_samples += 1;
    audio_diag_.env_voice_accum += env_voices_this_sample;
    if (env_voices_this_sample > audio_diag_.env_voice_peak) {
      audio_diag_.env_voice_peak = env_voices_this_sample;
      audio_diag_.env_voice_peak_sample = sample_clock_;
      audio_diag_.env_voice_peak_key_on_events = audio_diag_.key_on_events;
      audio_diag_.env_voice_peak_key_off_events = audio_diag_.key_off_events;
      audio_diag_.env_voice_peak_endx_mask = endx_mask_ & 0x00FFFFFFu;
    }
    audio_diag_.audible_voice_samples += 1;
    audio_diag_.audible_voice_accum += audible_voices_this_sample;
    if (audible_voices_this_sample > audio_diag_.audible_voice_peak) {
      audio_diag_.audible_voice_peak = audible_voices_this_sample;
      audio_diag_.audible_voice_peak_sample = sample_clock_;
      audio_diag_.audible_voice_peak_key_on_events = audio_diag_.key_on_events;
      audio_diag_.audible_voice_peak_key_off_events = audio_diag_.key_off_events;
      audio_diag_.audible_voice_peak_endx_mask = endx_mask_ & 0x00FFFFFFu;
    }

    // Keep active_voice_* for compatibility with existing UI/tools and treat it
    // as "audible this sample".
    audio_diag_.active_voice_samples = audio_diag_.audible_voice_samples;
    audio_diag_.active_voice_accum = audio_diag_.audible_voice_accum;
    audio_diag_.active_voice_peak = audio_diag_.audible_voice_peak;
    if (audible_voices_this_sample >= NUM_VOICES) {
      ++audio_diag_.voice_cap_frames;
    }
    if (audible_voices_this_sample == 0) {
      ++audio_diag_.no_voice_frames;
    }

    // CD audio (XA/CDDA) input path mixed through CD input volumes.
    if ((spucnt_eff & 0x0001u) != 0) {
      float cd_l = 0.0f;
      float cd_r = 0.0f;
      if (cd_input_read_pos_ + 1 < cd_input_samples_.size()) {
        cd_l = q15_to_float(cd_input_samples_[cd_input_read_pos_]);
        cd_r = q15_to_float(cd_input_samples_[cd_input_read_pos_ + 1]);
        cd_input_read_pos_ += 2;
        ++audio_diag_.cd_frames_mixed;
      }

      const float cd_mix_l = cd_l * q15_to_float(cd_vol_l_);
      const float cd_mix_r = cd_r * q15_to_float(cd_vol_r_);
      mix_l += cd_mix_l;
      mix_r += cd_mix_r;

      if ((spucnt_eff & 0x0004u) != 0) {
        rev_send_l += cd_mix_l;
        rev_send_r += cd_mix_r;
      }
    }

    audio_diag_.peak_dry_l = std::max(audio_diag_.peak_dry_l, std::abs(mix_l));
    audio_diag_.peak_dry_r = std::max(audio_diag_.peak_dry_r, std::abs(mix_r));
    if (std::abs(mix_l) > 1.0f || std::abs(mix_r) > 1.0f) {
      ++audio_diag_.clip_events_dry;
    }

    mix_reverb(rev_send_l, rev_send_r, wet_l, wet_r);
    audio_diag_.peak_wet_l = std::max(audio_diag_.peak_wet_l, std::abs(wet_l));
    audio_diag_.peak_wet_r = std::max(audio_diag_.peak_wet_r, std::abs(wet_r));
    if (std::abs(wet_l) > 0.98f || std::abs(wet_r) > 0.98f) {
      ++audio_diag_.clip_events_wet;
    }

    mix_l += wet_l;
    mix_r += wet_r;

    // Master volume.
    const float mvl = q15_to_float(master_vol_l_);
    const float mvr = q15_to_float(master_vol_r_);
    mix_l *= mvl;
    mix_r *= mvr;

    audio_diag_.peak_mix_l = std::max(audio_diag_.peak_mix_l, std::abs(mix_l));
    audio_diag_.peak_mix_r = std::max(audio_diag_.peak_mix_r, std::abs(mix_r));
    if (std::abs(mix_l) > 1.0f || std::abs(mix_r) > 1.0f) {
      ++audio_diag_.clip_events_out;
    }

    mix_l = std::clamp(mix_l, -1.0f, 1.0f);
    mix_r = std::clamp(mix_r, -1.0f, 1.0f);
    if (muted) {
      ++audio_diag_.muted_output_frames;
      out[static_cast<size_t>(s) * 2] = 0;
      out[static_cast<size_t>(s) * 2 + 1] = 0;
    } else {
      out[static_cast<size_t>(s) * 2] = static_cast<s16>(mix_l * 30000.0f);
      out[static_cast<size_t>(s) * 2 + 1] = static_cast<s16>(mix_r * 30000.0f);
    }

    
  }

  queue_host_audio(out);
  if (profile_detailed && sys_) {
    const auto end = std::chrono::high_resolution_clock::now();
    sys_->add_spu_time(
        std::chrono::duration<double, std::milli>(end - start).count());
  }
}

void Spu::tick_adsr(int voice, VoiceState &vs) {
  auto tick_rate = [&](u8 rate, u8 rate_mask, bool decreasing,
                       bool exponential) {
    s32 step = 7 - static_cast<s32>(rate & 0x3u);
    step = decreasing ? ~step : step;

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

    const u32 counter_sum =
        static_cast<u32>(vs.adsr_counter) + static_cast<u32>(this_inc);
    if (counter_sum < 0x8000u) {
      vs.adsr_counter = static_cast<u16>(counter_sum);
      return;
    }
    // Keep the fractional remainder instead of resetting to zero. This matches
    // hardware-style ADSR timing more closely and avoids rate skew.
    vs.adsr_counter = static_cast<u16>(counter_sum - 0x8000u);

    const s32 next = std::clamp(current + this_step, 0, kEnvMax);
    vs.env_level = static_cast<u16>(next);
  };

  switch (vs.phase) {
  case VoiceState::AdsrPhase::Attack: {
    const u8 rate =
        static_cast<u8>((static_cast<u8>(vs.attack_shift << 2)) |
                        static_cast<u8>(vs.attack_step & 0x3u));
    tick_rate(rate, 0x7Fu, false, vs.attack_exp);
    if (vs.env_level >= kEnvMax) {
      vs.env_level = kEnvMax;
      vs.phase = VoiceState::AdsrPhase::Decay;
    }
    break;
  }
  case VoiceState::AdsrPhase::Decay: {
    const u8 rate = static_cast<u8>((vs.decay_shift & 0x0Fu) << 2);
    tick_rate(rate, static_cast<u8>(0x1Fu << 2), true, true);
    if (vs.env_level <= vs.sustain_level) {
      vs.env_level = vs.sustain_level;
      vs.phase = VoiceState::AdsrPhase::Sustain;
    }
    break;
  }
  case VoiceState::AdsrPhase::Sustain: {
    const u8 rate =
        static_cast<u8>((static_cast<u8>(vs.sustain_shift << 2)) |
                        static_cast<u8>(vs.sustain_step & 0x3u));
    tick_rate(rate, 0x7Fu, vs.sustain_decrease, vs.sustain_exp);
    break;
  }
  case VoiceState::AdsrPhase::Release: {
    // Release uses only shift bits (step sub-bits are zero for KOFF path).
    const u8 rate = static_cast<u8>((vs.release_shift & 0x1Fu) << 2);
    tick_rate(rate, static_cast<u8>(0x1Fu << 2), true, vs.release_exp);
    if (vs.env_level == 0) {
      if (vs.release_tracking) {
        const u64 dur64 = sample_clock_ - vs.release_start_sample;
        const u32 dur =
            (dur64 > static_cast<u64>(std::numeric_limits<u32>::max()))
                ? std::numeric_limits<u32>::max()
                : static_cast<u32>(dur64);
        audio_diag_.release_samples_total += dur64;
        if (audio_diag_.release_to_off_events == 0) {
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
      if (g_trace_spu) {
        LOG_DEBUG("SPU: VOICE v=%d ->Off reason=RELEASE_ENV0 sample=%llu",
                  voice, static_cast<unsigned long long>(sample_clock_));
      }
    }
    break;
  }
  case VoiceState::AdsrPhase::Off:
  default:
    break;
  }
}
