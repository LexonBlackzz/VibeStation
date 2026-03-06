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
    u64 cd_irq_int3_count = 0;
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

  System() = default;

  // Initialization (called when user loads BIOS)
  void init_hardware();
  bool load_bios(const std::string &path);
  bool load_game(const std::string &bin_path, const std::string &cue_path);
  bool swap_disc_image(const std::string &bin_path, const std::string &cue_path) {
    return cdrom_.swap_disc_image(bin_path, cue_path);
  }
  void notify_disc_inserted() { cdrom_.notify_disc_inserted(); }
  bool boot_disc();
  void reset();
  void shutdown();

  // Emulation control
  void run_frame(bool sample_display_diag = true);
  void step();
  double target_fps() const;

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
  void push_cd_audio_samples(const std::vector<s16> &samples, u32 sample_rate) {
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
  u8 read8(u32 addr);
  u16 read16(u32 addr);
  u32 read32(u32 addr);
  void write8(u32 addr, u8 val);
  void write16(u32 addr, u16 val);
  void write32(u32 addr, u32 val);

  // Component access (for DMA)
  bool irq_pending() { return irq_.pending(); }
  void gpu_gp0(u32 val) { gpu_.gp0(val); }
  u32 gpu_read() { return gpu_.read_data(); }
  bool gpu_dma_request() const { return gpu_.dma_request(); }
  u32 cdrom_dma_read() { return cdrom_.dma_read(); }
  bool cdrom_dma_request() const { return cdrom_.dma_request(); }
  void mdec_dma_write(u32 val) { mdec_.dma_write(val); }
  u32 mdec_dma_read() { return mdec_.dma_read(); }
  bool mdec_dma_in_request() const { return mdec_.dma_in_request(); }
  bool mdec_dma_out_request() const { return mdec_.dma_out_request(); }
  void spu_dma_write(u32 val);
  u32 spu_dma_read();
  bool spu_dma_request() const { return spu_.dma_request(); }

  // Public component access
  Gpu &gpu() { return gpu_; }
  Cpu &cpu() { return cpu_; }
  Sio &sio() { return sio_; }
  const Spu &spu() const { return spu_; }
  const CdRom &cdrom() const { return cdrom_; }
  const Bios &bios() const { return bios_; }
  InterruptController &irq() { return irq_; }

private:
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
  u32 mdec_control_shadow_ = 0;
  u8 post_reg_ = 0;
  BootDiagnostics boot_diag_ = {};
  ProfilingStats profiling_stats_ = {};
  bool saw_non_bios_exec_ = false;
  u32 bios_menu_streak_after_non_bios_ = 0;
  u64 spu_synced_cpu_cycle_ = 0;
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
  void sync_spu_to_cpu();
  void apply_ram_reaper_for_frame();
  void apply_gpu_reaper_for_frame();
  void apply_sound_reaper_for_frame();
};

// Timer IRQ helper (called from timer.cpp)
void timer_fire_irq(System *sys, int index);
