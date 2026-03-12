#pragma once
#include "bios.h"
#include "cdrom.h"
#include "cpu.h"
#include "dma.h"
#include "gpu.h"
#include "interrupt.h"
#include "mdec.h"
#include "ram.h"
#include "sio.h"
#include "spu.h"
#include "timer.h"
#include "types.h"
#include <atomic>
#include <cstring>
#include <random>
#include <string>
#include <vector>

// System container for all PS1 core devices and memory bus dispatch.

class System {
public:
  struct RamReaperConfig {
    bool enabled = false;
    u32 range_start = 0;
    u32 range_end = psx::RAM_SIZE - 1u;
    u32 writes_per_frame = 1;
    float intensity_percent = 100.0f;
    bool affect_main_ram = true;
    bool affect_vram = true;
    bool affect_spu_ram = true;
    bool use_custom_seed = false;
    u64 seed = 1;
  };

  struct GpuReaperConfig {
    bool enabled = false;
    u32 writes_per_frame = 1;
    float intensity_percent = 100.0f;
    bool affect_geometry = true;
    bool affect_texture_state = true;
    bool affect_display_state = false;
    bool use_custom_seed = false;
    u64 seed = 1;
  };

  struct SoundReaperConfig {
    bool enabled = false;
    u32 writes_per_frame = 1;
    float intensity_percent = 100.0f;
    bool affect_pitch = true;
    bool affect_envelope = true;
    bool affect_reverb = true;
    bool affect_mixer = true;
    bool use_custom_seed = false;
    u64 seed = 1;
  };

  struct BootDiagnostics {
    bool saw_cd_io = false;
    bool saw_sio_io = false;
    bool saw_pad_cmd42 = false;
    bool saw_tx_cmd42 = false;
    bool saw_pad_id = false;
    bool saw_pad_button = false;
    bool saw_full_pad_poll = false;
    bool saw_cd_read_cmd = false;
    bool saw_cd_sector_visible = false;
    bool saw_cd_getid = false;
    bool saw_cd_setloc = false;
    bool saw_cd_seekl = false;
    bool saw_cd_readn_or_reads = false;
    bool saw_logo_present = false;
    bool logo_visible_persisted = false;
    bool fell_back_to_bios_after_non_bios = false;
    u64 first_cd_io_cycle = 0;
    u64 first_sio_io_cycle = 0;
    u32 first_cd_io_addr = 0;
    u32 first_sio_io_addr = 0;
    u64 cd_io_count = 0;
    u64 sio_io_count = 0;
    u64 pad_cmd42_count = 0;
    u64 pad_poll_count = 0;
    u64 pad_packet_count = 0;
    u64 ch0_poll_count = 0;
    u64 ch1_poll_count = 0;
    u64 sio_invalid_seq_count = 0;
    u16 last_pad_buttons = 0xFFFF;
    u8 last_sio_tx = 0;
    u8 last_sio_rx = 0xFF;
    u16 last_joy_stat = 0;
    u16 last_joy_ctrl = 0;
    u64 sio_irq_assert_count = 0;
    u64 sio_irq_ack_count = 0;
    u64 cd_read_command_count = 0;
    u64 cd_irq_int1_count = 0;
    u64 cd_irq_int2_count = 0;
    u64 cd_irq_int3_count = 0;
    u64 cd_irq_int4_count = 0;
    u64 cd_irq_int5_count = 0;
    u32 frame_counter = 0;
    u32 logo_visible_run_frames = 0;
    s32 first_black_after_logo_frame = -1;
    u64 display_non_black_pixels = 0;
    u32 display_hash = 0;
    u16 display_width = 0;
    u16 display_height = 0;
    u16 display_x_start = 0;
    u16 display_y_start = 0;
    u8 display_is_24bit = 0;
    u8 display_enabled = 0;
  };

  struct ProfilingStats {
    double cpu_ms = 0.0;
    double gpu_ms = 0.0;
    double spu_ms = 0.0;
    double dma_ms = 0.0;
    double timers_ms = 0.0;
    double cdrom_ms = 0.0;
    double total_ms = 0.0;
  };

  struct MdecUploadProbe {
    static constexpr size_t kSampleWords = 8;
    static constexpr size_t kUploadHistory = 8;
    static constexpr size_t kMacroblockHistory = 32;

    bool dma1_seen = false;
    u32 dma1_base_addr = 0;
    u32 dma1_words = 0;
    u8 dma1_depth = 0;
    u8 dma1_first_block = 0;
    u32 dma1_sample_count = 0;
    std::array<u32, kSampleWords> dma1_addrs{};
    std::array<u32, kSampleWords> dma1_words_sample{};
    u32 dma1_mb_sample_count = 0;
    std::array<u32, kSampleWords> dma1_mb_seq{};
    std::array<u32, kSampleWords> dma1_mb_addrs{};
    std::array<u32, kSampleWords> dma1_mb_words_sample{};
    u32 dma1_mb_hist_count = 0;
    std::array<u32, kMacroblockHistory> dma1_mb_hist_seq{};
    std::array<u32, kMacroblockHistory> dma1_mb_hist_addrs{};
    std::array<u32, kMacroblockHistory> dma1_mb_hist_words_sample{};
    u32 dma1_range_bytes = 0;

    bool gpu_upload_seen = false;
    bool gpu_upload_data_seen = false;
    u32 gpu_upload_count = 0;
    u16 gpu_x = 0;
    u16 gpu_y = 0;
    u16 gpu_w = 0;
    u16 gpu_h = 0;
    u32 gpu_total_words = 0;
    u32 gpu_words_seen = 0;
    u32 gpu_sample_count = 0;
    std::array<u32, kSampleWords> gpu_words_sample{};
    std::array<u32, kSampleWords> gpu_dma_src_addrs{};
    std::array<u8, kSampleWords> gpu_word_from_dma{};
    u32 gpu_dma_src_base = 0;
    u32 gpu_dma_src_range_bytes = 0;
    std::array<u16, kUploadHistory> gpu_hist_x{};
    std::array<u16, kUploadHistory> gpu_hist_y{};
    std::array<u16, kUploadHistory> gpu_hist_w{};
    std::array<u16, kUploadHistory> gpu_hist_h{};
    std::array<u8, kUploadHistory> gpu_hist_from_dma{};
    u32 gpu_hist_count = 0;
    u32 gpu_frame_upload_count = 0;
    std::array<u16, kUploadHistory> gpu_frame_hist_x{};
    std::array<u16, kUploadHistory> gpu_frame_hist_y{};
    std::array<u16, kUploadHistory> gpu_frame_hist_w{};
    std::array<u16, kUploadHistory> gpu_frame_hist_h{};
    std::array<u8, kUploadHistory> gpu_frame_hist_from_dma{};
    bool gpu_last_frame_valid = false;
    u32 gpu_last_frame_upload_count = 0;
    std::array<u16, kUploadHistory> gpu_last_frame_hist_x{};
    std::array<u16, kUploadHistory> gpu_last_frame_hist_y{};
    std::array<u16, kUploadHistory> gpu_last_frame_hist_w{};
    std::array<u16, kUploadHistory> gpu_last_frame_hist_h{};
    std::array<u8, kUploadHistory> gpu_last_frame_hist_from_dma{};

    u32 mdec_read_sample_count = 0;
    std::array<u32, kSampleWords> mdec_read_addrs{};
    std::array<u32, kSampleWords> mdec_read_values{};
    std::array<u32, kSampleWords> mdec_read_pcs{};
    std::array<u8, kSampleWords> mdec_read_origin{};
    std::array<u8, kSampleWords> mdec_read_sizes{};

    u32 gpu_src_write_sample_count = 0;
    std::array<u32, kSampleWords> gpu_src_write_addrs{};
    std::array<u32, kSampleWords> gpu_src_write_values{};
    std::array<u32, kSampleWords> gpu_src_write_pcs{};
    std::array<u8, kSampleWords> gpu_src_write_origin{};
    std::array<u8, kSampleWords> gpu_src_write_sizes{};

    u32 gpu_copy_count = 0;
    u32 gpu_copy_sample_count = 0;
    std::array<u16, kSampleWords> gpu_copy_src_x{};
    std::array<u16, kSampleWords> gpu_copy_src_y{};
    std::array<u16, kSampleWords> gpu_copy_dst_x{};
    std::array<u16, kSampleWords> gpu_copy_dst_y{};
    std::array<u16, kSampleWords> gpu_copy_w{};
    std::array<u16, kSampleWords> gpu_copy_h{};
  };

  System() = default;

  // Initialization (called when user loads BIOS)
  void init_hardware();
  bool load_bios(const std::string &path);
  bool load_game(const std::string &bin_path, const std::string &cue_path);
  bool swap_disc_image(const std::string &bin_path, const std::string &cue_path) {
    return cdrom_.swap_disc_image(bin_path, cue_path);
  }
  bool set_memory_card_slot(u32 slot, const std::string &path) {
    return sio_.set_memory_card_slot(slot, path);
  }
  void flush_memory_cards() { sio_.flush_memory_cards(); }
  bool memory_card_inserted(u32 slot) const {
    return sio_.memory_card_inserted(slot);
  }
  bool memory_card_dirty(u32 slot) const { return sio_.memory_card_dirty(slot); }
  std::string memory_card_path(u32 slot) const {
    return sio_.memory_card_path(slot);
  }
  void notify_disc_inserted() { cdrom_.notify_disc_inserted(); }
  bool boot_disc(bool direct_boot = false);
  void reset();
  void shutdown();

  // Emulation control
  void run_frame(bool sample_display_diag = true,
                 bool skip_spu_for_turbo = false);
  void step();
  double target_fps() const;
  void set_audio_output_speed(double speed) { spu_.set_output_speed(speed); }
  void set_spu_reverb_mix_multiplier(double multiplier) {
    spu_.set_reverb_mix_multiplier(multiplier);
  }
  void set_spu_force_reverb(bool enabled) { spu_.set_force_reverb(enabled); }

  // State
  bool is_running() const { return running_; }
  void set_running(bool r) { running_ = r; }
  bool bios_loaded() const { return bios_.is_loaded(); }
  bool disc_loaded() const { return cdrom_.is_disc_inserted(); }
  bool hardware_ready() const { return hw_init_; }
  const BootDiagnostics &boot_diag() const { return boot_diag_; }
  const ProfilingStats &profiling_stats() const { return profiling_stats_; }
  void reset_profiling_stats() { profiling_stats_ = {}; }
  void add_cpu_time(double ms) { profiling_stats_.cpu_ms += ms; }
  void add_gpu_time(double ms) { profiling_stats_.gpu_ms += ms; }
  void add_cdrom_time(double ms) { profiling_stats_.cdrom_ms += ms; }
  void add_spu_time(double ms) { profiling_stats_.spu_ms += ms; }
  void add_dma_time(double ms) { profiling_stats_.dma_ms += ms; }
  void add_timers_time(double ms) { profiling_stats_.timers_ms += ms; }
  void set_total_time(double ms) { profiling_stats_.total_ms = ms; }
  u64 irq_request_count(Interrupt irq) const { return irq_.request_count(irq); }
  const Spu::AudioDiag &spu_audio_diag() const { return spu_.audio_diag(); }
  void reset_spu_audio_diag() { spu_.reset_audio_diag(); }
  void set_spu_audio_capture(bool enabled) { spu_.set_audio_capture(enabled); }
  bool spu_audio_capture_enabled() const { return spu_.audio_capture_enabled(); }
  void clear_spu_audio_capture() { spu_.clear_audio_capture(); }
  const std::vector<s16> &spu_audio_capture_samples() const {
    return spu_.audio_capture_samples();
  }
  bool save_spu_voice_sample_to_file(int voice, const std::string &path,
                                     std::string *error = nullptr);
  bool save_spu_voice_samples_to_file(const std::vector<int> &voices,
                                      const std::string &path,
                                      std::string *error = nullptr);
  bool load_spu_replacement_sample_from_file(const std::string &path,
                                             std::string *error = nullptr);
  void clear_spu_replacement_sample() { spu_.clear_replacement_sample(); }
  void set_spu_replacement_sample_enabled(bool enabled) {
    spu_.set_replacement_sample_enabled(enabled);
  }
  bool spu_replacement_sample_enabled() const {
    return spu_.replacement_sample_enabled();
  }
  bool spu_replacement_sample_loaded() const {
    return spu_.replacement_sample_loaded();
  }
  size_t spu_replacement_sample_bytes() const {
    return spu_.replacement_sample_bytes();
  }
  void push_cd_audio_samples(const std::vector<s16> &samples, u32 sample_rate) {
    if (spu_skip_sync_for_turbo_) {
      return;
    }
    spu_.push_cd_audio_samples(samples, sample_rate);
  }
  void update_display_diag(const DisplaySampleInfo &display_sample);

  // RAM Reaper (experimental real-time RAM corruption)
  void set_ram_reaper_config(const RamReaperConfig &config);
  RamReaperConfig ram_reaper_config() const;
  u64 ram_reaper_last_seed() const {
    return ram_reaper_last_seed_.load(std::memory_order_acquire);
  }
  u64 ram_reaper_total_mutations() const {
    return ram_reaper_total_mutations_.load(std::memory_order_acquire);
  }
  void disable_ram_reaper();
  void set_gpu_reaper_config(const GpuReaperConfig &config);
  GpuReaperConfig gpu_reaper_config() const;
  u64 gpu_reaper_last_seed() const {
    return gpu_reaper_last_seed_.load(std::memory_order_acquire);
  }
  u64 gpu_reaper_total_mutations() const {
    return gpu_reaper_total_mutations_.load(std::memory_order_acquire);
  }
  void disable_gpu_reaper();
  void set_sound_reaper_config(const SoundReaperConfig &config);
  SoundReaperConfig sound_reaper_config() const;
  u64 sound_reaper_last_seed() const {
    return sound_reaper_last_seed_.load(std::memory_order_acquire);
  }
  u64 sound_reaper_total_mutations() const {
    return sound_reaper_total_mutations_.load(std::memory_order_acquire);
  }
  void disable_sound_reaper();

  // Memory bus interface
  // Fast path used by CPU instruction fetch to avoid full I/O dispatch on the
  // hot RAM/BIOS paths.
  u32 read32_instruction(u32 addr) {
    const u32 phys = psx::mask_address(addr);
    if (phys < 0x00800000u) {
      const u32 off = phys & 0x1FFFFFu;
      if (!g_trace_ram) {
        u32 value = 0;
        std::memcpy(&value, ram_.data() + off, sizeof(value));
        return value;
      }
      return ram_.read32(off);
    }

    if (phys >= psx::BIOS_BASE &&
        static_cast<u64>(phys) <
            (static_cast<u64>(psx::BIOS_BASE) + bios_.mapped_size())) {
      return bios_.read32(phys - psx::BIOS_BASE);
    }

    if (phys >= 0x1F800000u && phys < 0x1F801000u) {
      return ram_.scratch_read32(phys - 0x1F800000u);
    }

    return read32(addr);
  }

  u8 read8_data(u32 addr) {
    const u32 phys = psx::mask_address(addr);
    if (phys < 0x00800000u) {
      const u32 off = phys & 0x1FFFFFu;
      if (!g_trace_ram) {
        return ram_.data()[off];
      }
      return ram_.read8(off);
    }

    if (phys >= psx::BIOS_BASE &&
        static_cast<u64>(phys) <
            (static_cast<u64>(psx::BIOS_BASE) + bios_.mapped_size())) {
      return bios_.read8(phys - psx::BIOS_BASE);
    }

    if (phys >= 0x1F800000u && phys < 0x1F801000u) {
      const u32 off = (phys - 0x1F800000u) & (psx::SCRATCHPAD_SIZE - 1u);
      if (!g_trace_ram) {
        return ram_.scratch_data()[off];
      }
      return ram_.scratch_read8(off);
    }

    return read8(addr);
  }

  u16 read16_data(u32 addr) {
    const u32 phys = psx::mask_address(addr);
    if (phys < 0x00800000u) {
      const u32 off = phys & 0x1FFFFFu;
      if (!g_trace_ram) {
        u16 value = 0;
        std::memcpy(&value, ram_.data() + off, sizeof(value));
        return value;
      }
      return ram_.read16(off);
    }

    if (phys >= psx::BIOS_BASE &&
        static_cast<u64>(phys) <
            (static_cast<u64>(psx::BIOS_BASE) + bios_.mapped_size())) {
      return bios_.read16(phys - psx::BIOS_BASE);
    }

    if (phys >= 0x1F800000u && phys < 0x1F801000u) {
      const u32 off = (phys - 0x1F800000u) & (psx::SCRATCHPAD_SIZE - 1u);
      if (!g_trace_ram) {
        u16 value = 0;
        std::memcpy(&value, ram_.scratch_data() + off, sizeof(value));
        return value;
      }
      return ram_.scratch_read16(off);
    }

    return read16(addr);
  }

  u32 read32_data(u32 addr) {
    const u32 phys = psx::mask_address(addr);
    if (phys < 0x00800000u) {
      const u32 off = phys & 0x1FFFFFu;
      if (!g_trace_ram) {
        u32 value = 0;
        std::memcpy(&value, ram_.data() + off, sizeof(value));
        return value;
      }
      return ram_.read32(off);
    }

    if (phys >= psx::BIOS_BASE &&
        static_cast<u64>(phys) <
            (static_cast<u64>(psx::BIOS_BASE) + bios_.mapped_size())) {
      return bios_.read32(phys - psx::BIOS_BASE);
    }

    if (phys >= 0x1F800000u && phys < 0x1F801000u) {
      const u32 off = (phys - 0x1F800000u) & (psx::SCRATCHPAD_SIZE - 1u);
      if (!g_trace_ram) {
        u32 value = 0;
        std::memcpy(&value, ram_.scratch_data() + off, sizeof(value));
        return value;
      }
      return ram_.scratch_read32(off);
    }

    return read32(addr);
  }

  void write8_data(u32 addr, u8 val) {
    const u32 phys = psx::mask_address(addr);
    if (phys < 0x00800000u) {
      const u32 off = phys & 0x1FFFFFu;
      if (!g_trace_ram && !g_ram_watch_diagnostics) {
        ram_.data()[off] = val;
        if (g_mdec_debug_upload_probe || g_cpu_deep_diagnostics) {
          debug_note_main_ram_write(off, val, 1);
        }
        return;
      }
      write8(addr, val);
      return;
    }

    if (phys >= 0x1F800000u && phys < 0x1F801000u) {
      const u32 off = (phys - 0x1F800000u) & (psx::SCRATCHPAD_SIZE - 1u);
      if (!g_trace_ram) {
        ram_.scratch_data()[off] = val;
        return;
      }
      ram_.scratch_write8(off, val);
      return;
    }

    write8(addr, val);
  }

  void write16_data(u32 addr, u16 val) {
    const u32 phys = psx::mask_address(addr);
    if (phys < 0x00800000u) {
      const u32 off = phys & 0x1FFFFFu;
      if (!g_trace_ram && !g_ram_watch_diagnostics) {
        std::memcpy(ram_.data() + off, &val, sizeof(val));
        if (g_mdec_debug_upload_probe || g_cpu_deep_diagnostics) {
          debug_note_main_ram_write(off, val, 2);
        }
        return;
      }
      write16(addr, val);
      return;
    }

    if (phys >= 0x1F800000u && phys < 0x1F801000u) {
      const u32 off = (phys - 0x1F800000u) & (psx::SCRATCHPAD_SIZE - 1u);
      if (!g_trace_ram) {
        std::memcpy(ram_.scratch_data() + off, &val, sizeof(val));
        return;
      }
      ram_.scratch_write16(off, val);
      return;
    }

    write16(addr, val);
  }

  void write32_data(u32 addr, u32 val) {
    const u32 phys = psx::mask_address(addr);
    if (phys < 0x00800000u) {
      const u32 off = phys & 0x1FFFFFu;
      if (!g_trace_ram && !g_ram_watch_diagnostics) {
        std::memcpy(ram_.data() + off, &val, sizeof(val));
        if (g_mdec_debug_upload_probe || g_cpu_deep_diagnostics) {
          debug_note_main_ram_write(off, val, 4);
        }
        return;
      }
      write32(addr, val);
      return;
    }

    if (phys >= 0x1F800000u && phys < 0x1F801000u) {
      const u32 off = (phys - 0x1F800000u) & (psx::SCRATCHPAD_SIZE - 1u);
      if (!g_trace_ram) {
        std::memcpy(ram_.scratch_data() + off, &val, sizeof(val));
        return;
      }
      ram_.scratch_write32(off, val);
      return;
    }

    write32(addr, val);
  }

  u8 read8(u32 addr);
  u16 read16(u32 addr);
  u32 read32(u32 addr);
  void write8(u32 addr, u8 val);
  void write16(u32 addr, u16 val);
  void write32(u32 addr, u32 val);

  // Component access (for DMA)
  bool irq_pending() { return irq_.pending(); }
  void gpu_gp0(u32 val);
  void gpu_gp0_dma(u32 val, u32 src_addr);
  u32 gpu_read() { return gpu_.read_data(); }
  bool gpu_dma_request() const { return gpu_.dma_request(); }
  u32 cdrom_dma_read() { return cdrom_.dma_read(); }
  bool cdrom_dma_request() const { return cdrom_.dma_request(); }
  u32 cdrom_dma_words_available() const { return cdrom_.dma_words_available(); }
  void mdec_dma_write(u32 val) { mdec_.dma_write(val); }
  u32 mdec_dma_read() { return mdec_.dma_read(); }
  u8 mdec_dma_out_block() const { return mdec_.dma_out_block(); }
  u8 mdec_dma_out_depth() const { return mdec_.dma_out_depth(); }
  u32 mdec_dma_out_macroblock_seq() const { return mdec_.dma_out_macroblock_seq(); }
  bool mdec_dma_in_request() const { return mdec_.dma_in_request(); }
  bool mdec_dma_out_request() const { return mdec_.dma_out_request(); }
  u32 mdec_dma_out_words_available() const {
    return mdec_.dma_out_words_available();
  }
  const Mdec::DebugStats &mdec_debug_stats() const { return mdec_.debug_stats(); }
  void reset_mdec_debug_stats() { mdec_.reset_debug_stats(); }
  const Mdec::DebugCompare &mdec_debug_compare() const {
    return mdec_.debug_compare();
  }
  void reset_mdec_debug_compare() { mdec_.reset_debug_compare(); }
  DisplayDebugInfo gpu_display_debug_info() const {
    return gpu_.debug_display_info();
  }
  GpuCommandDebugInfo gpu_command_debug_info() const {
    return gpu_.debug_command_info();
  }
  const MdecUploadProbe &mdec_upload_probe() const { return mdec_upload_probe_; }
  void reset_mdec_upload_probe() { mdec_upload_probe_ = {}; }
  void debug_begin_dma_bus_access(u8 channel);
  void debug_end_dma_bus_access();
  void debug_note_mdec_dma_out_begin(u32 base_addr, u32 words, u8 depth,
                                     u8 first_block);
  void debug_note_mdec_dma_out_word(u32 write_addr, u32 value,
                                    u32 macroblock_seq);
  void debug_note_gpu_image_load_begin(u16 x, u16 y, u16 w, u16 h);
  void debug_note_gpu_vblank();
  void debug_note_gpu_image_load_word(u32 value);
  void debug_note_gpu_vram_copy(u16 src_x, u16 src_y, u16 dst_x, u16 dst_y,
                                u16 w, u16 h);
  void spu_dma_write(u32 val);
  u32 spu_dma_read();
  bool spu_dma_request() const { return spu_.dma_request(); }
  const DmaController::TransferDebug &dma_last_debug(int channel) const {
    return dma_.last_debug(channel);
  }
  void debug_log_recent_ram_writes(u32 addr, u32 radius_bytes) const;

  // Public component access
  Gpu &gpu() { return gpu_; }
  Cpu &cpu() { return cpu_; }
  Sio &sio() { return sio_; }
  const Spu &spu() const { return spu_; }
  const CdRom &cdrom() const { return cdrom_; }
  const Bios &bios() const { return bios_; }
  InterruptController &irq() { return irq_; }

private:
  struct RamAccessLogEntry {
    u32 addr = 0;
    u32 value = 0;
    u32 pc = 0;
    u8 size = 0;
    u8 origin = 0;
  };
  struct StackTopBurstDebug {
    bool active = false;
    bool logged_context = false;
    u32 pc = 0xFFFFFFFFu;
    u32 start_addr = 0;
    u32 next_addr = 0;
    u32 value = 0;
    u8 size = 0;
    u8 origin = 0;
    u32 count = 0;
  };
  static constexpr size_t kRamWriteHistorySize = 8192u;

  void debug_note_main_ram_read(u32 addr, u32 value, u8 size);
  void debug_note_main_ram_write(u32 addr, u32 value, u8 size);
  void debug_track_stack_top_write(const RamAccessLogEntry &entry);
  void debug_log_stack_top_burst_context(const RamAccessLogEntry &entry);
  void populate_gpu_src_write_samples_from_history();

  // Hardware components
  Bios bios_;
  Ram ram_;
  InterruptController irq_;
  Timers timers_;
  Spu spu_;
  Mdec mdec_;
  Sio sio_;
  CdRom cdrom_;
  DmaController dma_;
  Gpu gpu_;
  Cpu cpu_;

  bool running_ = false;
  bool hw_init_ = false;
  u64 frame_cycles_ = 0;
  double frame_cycle_remainder_ = 0.0;
  std::string last_disc_bin_path_;
  std::string last_disc_cue_path_;

  // Memory control registers (written by BIOS, mostly ignored)
  u32 mem_ctrl_[9] = {};
  u32 ram_size_ = 0;
  u32 cache_ctrl_ = 0;
  u32 mdec_command_shadow_ = 0;
  u32 mdec_command_shadow_mask_ = 0;
  u32 mdec_control_shadow_ = 0;
  u32 mdec_control_shadow_mask_ = 0;
  u32 gpu_gp0_shadow_ = 0;
  u32 gpu_gp0_shadow_mask_ = 0;
  u32 gpu_gp1_shadow_ = 0;
  u32 gpu_gp1_shadow_mask_ = 0;
  u8 post_reg_ = 0;
  MdecUploadProbe mdec_upload_probe_ = {};
  bool gpu_gp0_source_valid_ = false;
  bool gpu_gp0_source_from_dma_ = false;
  u32 gpu_gp0_source_addr_ = 0;
  bool bus_access_from_dma_ = false;
  u8 bus_access_dma_channel_ = 0xFFu;
  std::array<RamAccessLogEntry, kRamWriteHistorySize> ram_write_history_{};
  u32 ram_write_history_pos_ = 0;
  u32 ram_write_history_count_ = 0;
  StackTopBurstDebug stack_top_burst_ = {};
  u32 stack_top_write_log_budget_ = 64;
  bool stack_top_write_log_suppressed_ = false;
  BootDiagnostics boot_diag_ = {};
  ProfilingStats profiling_stats_ = {};
  bool saw_non_bios_exec_ = false;
  u32 bios_menu_streak_after_non_bios_ = 0;
  u64 spu_synced_cpu_cycle_ = 0;
  bool spu_skip_sync_for_turbo_ = false;
  std::atomic<bool> ram_reaper_enabled_{false};
  std::atomic<u32> ram_reaper_range_start_{0};
  std::atomic<u32> ram_reaper_range_end_{psx::RAM_SIZE - 1u};
  std::atomic<u32> ram_reaper_writes_per_frame_{1};
  std::atomic<u32> ram_reaper_intensity_x10_{1000};
  std::atomic<bool> ram_reaper_affect_main_ram_{true};
  std::atomic<bool> ram_reaper_affect_vram_{true};
  std::atomic<bool> ram_reaper_affect_spu_ram_{true};
  std::atomic<bool> ram_reaper_use_custom_seed_{false};
  std::atomic<u64> ram_reaper_seed_{1};
  std::atomic<u64> ram_reaper_last_seed_{0};
  std::atomic<u64> ram_reaper_total_mutations_{0};
  std::mt19937 ram_reaper_rng_{};
  bool ram_reaper_rng_seeded_ = false;
  bool ram_reaper_prev_enabled_ = false;
  bool ram_reaper_prev_use_custom_seed_ = false;
  u64 ram_reaper_prev_seed_ = 0;
  std::atomic<bool> gpu_reaper_enabled_{false};
  std::atomic<u32> gpu_reaper_writes_per_frame_{1};
  std::atomic<u32> gpu_reaper_intensity_x10_{1000};
  std::atomic<bool> gpu_reaper_affect_geometry_{true};
  std::atomic<bool> gpu_reaper_affect_texture_state_{true};
  std::atomic<bool> gpu_reaper_affect_display_state_{false};
  std::atomic<bool> gpu_reaper_use_custom_seed_{false};
  std::atomic<u64> gpu_reaper_seed_{1};
  std::atomic<u64> gpu_reaper_last_seed_{0};
  std::atomic<u64> gpu_reaper_total_mutations_{0};
  std::mt19937 gpu_reaper_rng_{};
  bool gpu_reaper_rng_seeded_ = false;
  bool gpu_reaper_prev_enabled_ = false;
  bool gpu_reaper_prev_use_custom_seed_ = false;
  u64 gpu_reaper_prev_seed_ = 0;
  std::atomic<bool> sound_reaper_enabled_{false};
  std::atomic<u32> sound_reaper_writes_per_frame_{1};
  std::atomic<u32> sound_reaper_intensity_x10_{1000};
  std::atomic<bool> sound_reaper_affect_pitch_{true};
  std::atomic<bool> sound_reaper_affect_envelope_{true};
  std::atomic<bool> sound_reaper_affect_reverb_{true};
  std::atomic<bool> sound_reaper_affect_mixer_{true};
  std::atomic<bool> sound_reaper_use_custom_seed_{false};
  std::atomic<u64> sound_reaper_seed_{1};
  std::atomic<u64> sound_reaper_last_seed_{0};
  std::atomic<u64> sound_reaper_total_mutations_{0};
  std::mt19937 sound_reaper_rng_{};
  bool sound_reaper_rng_seeded_ = false;
  bool sound_reaper_prev_enabled_ = false;
  bool sound_reaper_prev_use_custom_seed_ = false;
  u64 sound_reaper_prev_seed_ = 0;

  void note_cdrom_io(u32 phys_addr);
  void note_sio_io(u32 phys_addr);
  void maybe_log_ram_watch_write(u32 phys_addr, u32 value, u32 size_bytes);
  void sync_spu_to_cpu();
  void apply_ram_reaper_for_frame();
  void apply_gpu_reaper_for_frame();
  void apply_sound_reaper_for_frame();
};

// Timer IRQ helper (called from timer.cpp)
void timer_fire_irq(System *sys, int index);
