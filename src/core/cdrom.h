#pragma once
#include "types.h"
#include <array>
#include <cstddef>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

// ── CD-ROM Controller ──────────────────────────────────────────────
// State machine for the PS1 CD-ROM drive.
// Handles .bin/.cue disc images.

class System;

// Parsed .cue file track info
struct CdTrack {
  int number = 0;
  std::string type; // "MODE2/2352", "AUDIO", etc.
  std::string filename;
  int sector_size = 2352;
  int pregap_sectors = 0;
  int index01_file_lba = 0; // INDEX 01 position in current file (sectors)
  int index01_abs_lba = 150; // Absolute disc LBA for INDEX 01
  u64 index01_file_offset = 0; // Byte offset of INDEX 01 in current file
};

class CdRom {
public:
  void init(System *sys) { sys_ = sys; }
  void reset();

  bool load_bin_cue(const std::string &bin_path, const std::string &cue_path);
  bool swap_disc_image(const std::string &bin_path, const std::string &cue_path);
  void notify_disc_inserted();
  bool is_disc_inserted() const { return disc_loaded_; }
  bool track_map_valid() const { return track_map_valid_; }
  const std::string &resolved_disc_path() const { return resolved_disc_path_; }
  u64 command_count() const { return command_counter_; }
  u64 command_count_for(u8 cmd) const {
    return command_hist_[static_cast<size_t>(cmd)];
  }
  u64 sector_count() const { return sector_counter_; }
  u8 last_irq_code() const { return last_irq_code_; }
  int busy_cycles_remaining() const { return command_busy_cycles_; }
  size_t pending_irq_count() const {
    return pending_irqs_.size() + (pending_second_.active ? 1u : 0u);
  }
  size_t response_fifo_size() const {
    if (response_index_ >= static_cast<int>(response_fifo_.size())) {
      return 0;
    }
    return response_fifo_.size() - static_cast<size_t>(response_index_);
  }
  size_t param_fifo_size() const { return param_fifo_.size(); }
  bool saw_read_command() const { return saw_read_command_; }
  bool saw_sector_visible() const { return saw_sector_visible_; }
  u64 read_command_count() const { return read_command_count_; }
  bool saw_getid() const { return saw_getid_; }
  bool saw_setloc() const { return saw_setloc_; }
  bool saw_seekl() const { return saw_seekl_; }
  bool saw_readn_or_reads() const { return saw_readn_or_reads_; }
  u64 irq_int1_count() const { return irq_int1_count_; }
  u64 irq_int2_count() const { return irq_int2_count_; }
  u64 irq_int3_count() const { return irq_int3_count_; }
  u64 irq_int4_count() const { return irq_int4_count_; }
  u64 irq_int5_count() const { return irq_int5_count_; }
  u64 read_buffer_stall_count() const { return read_buffer_stall_count_; }
  u64 response_promotion_count() const { return response_promotion_count_; }
  u64 status_e0_poll_count() const { return status_e0_poll_count_; }
  u64 status_e0_streak_max() const { return status_e0_streak_max_; }

  u8 read8(u32 offset);
  void write8(u32 offset, u8 value);

  void tick(u32 cycles);

  // DMA reads a word from the data buffer
  u32 dma_read();
  bool dma_request() const { return data_ready_ && data_request_; }
  u32 dma_words_available() const {
    if (!data_ready_ || data_index_ >= static_cast<int>(data_buffer_.size())) {
      return 0;
    }
    const size_t remaining_bytes =
        data_buffer_.size() - static_cast<size_t>(data_index_);
    return static_cast<u32>(remaining_bytes / 4u);
  }

private:
  System *sys_ = nullptr;
  bool disc_loaded_ = false;

  // Disc image
  std::ifstream bin_file_;
  std::vector<CdTrack> tracks_;
  std::string resolved_disc_path_;
  bool track_map_valid_ = false;

  // Registers
  u8 index_reg_ = 0; // Index register (bits 0-1 of status)
  u8 interrupt_enable_ = 0;
  u8 interrupt_flag_ = 0;

  // Command/Response FIFO
  std::vector<u8> param_fifo_;
  std::vector<u8> response_fifo_;
  int response_index_ = 0;

  // Data buffer (one sector = 2048 bytes of data)
  std::vector<u8> data_buffer_;
  int data_index_ = 0;
  bool data_ready_ = false;
  bool data_request_ = false;
  bool motor_on_ = false;
  bool shell_open_ = false;
  bool seek_error_ = false;
  bool id_error_ = false;

  // State
  enum class State {
    Idle,
    Seeking,
    Reading,
  };
  State state_ = State::Idle;

  // Deferred second response for multi-response commands
  struct PendingSecondResponse {
    bool active = false;
    int delay = 0;            // Cycles until delivery
    u8 irq = 0;               // INT number for second response
    std::vector<u8> response; // Response FIFO payload
  };
  PendingSecondResponse pending_second_;
  struct PendingIrq {
    u8 irq = 0;
    bool wait_for_command_idle = true;
    std::vector<u8> response;
  };
  std::deque<PendingIrq> pending_irqs_;

  // Seek target
  u8 seek_mm_ = 0, seek_ss_ = 0, seek_ff_ = 0;
  int read_lba_ = 0;
  int pending_cycles_ = 0;
  int read_period_cycles_ = 0;
  bool command_busy_ = false;
  int command_busy_cycles_ = 0;
  u8 last_command_ = 0;
  u8 last_irq_code_ = 0;
  u8 mode_ = 0;
  u8 filter_file_ = 0;
  u8 filter_channel_ = 0;
  std::array<u8, 8> host_audio_regs_ = {};
  u8 host_audio_apply_ = 0;
  u64 command_counter_ = 0;
  std::array<u64, 256> command_hist_ = {};
  u64 sector_counter_ = 0;
  bool saw_read_command_ = false;
  bool saw_getid_ = false;
  bool saw_setloc_ = false;
  bool saw_seekl_ = false;
  bool saw_readn_or_reads_ = false;
  bool saw_sector_visible_ = false;
  u64 read_command_count_ = 0;
  u64 irq_int1_count_ = 0;
  u64 irq_int2_count_ = 0;
  u64 irq_int3_count_ = 0;
  u64 irq_int4_count_ = 0;
  u64 irq_int5_count_ = 0;
  u64 read_buffer_stall_count_ = 0;
  u64 response_promotion_count_ = 0;
  u64 status_e0_poll_count_ = 0;
  u64 status_e0_streak_max_ = 0;
  u64 status_e0_streak_current_ = 0;
  u64 bin_size_ = 0;
  int seek_target_lba_ = 0;
  bool seek_target_valid_ = false;
  bool seek_complete_ = false;
  bool read_whole_sector_ = false;
  bool pending_read_start_ = false;
  bool pending_reads_mode_ = false;
  bool cdda_playing_ = false;
  bool cdda_cmd_muted_ = false;
  bool cdda_adp_muted_ = false;
  int adpcm_busy_cycles_ = 0;
  std::array<u8, 4> atv_pending_ = {0x80u, 0x00u, 0x80u, 0x00u};
  std::array<u8, 4> atv_active_ = {0x80u, 0x00u, 0x80u, 0x00u};
  std::array<s16, 2> xa_hist1_ = {};
  std::array<s16, 2> xa_hist2_ = {};
  bool xa_stream_valid_ = false;
  u8 xa_stream_file_ = 0;
  u8 xa_stream_channel_ = 0;
  bool insert_probe_active_ = false;
  int insert_probe_delay_cycles_ = 0;
  int insert_probe_stage_ = 0;

  // Status byte
  u8 stat_byte() const;

  // Command handlers
  void execute_command(u8 cmd);
  void cmd_getstat();
  void cmd_setloc();
  void cmd_play();
  void cmd_readn();
  void cmd_pause();
  void cmd_init();
  void cmd_setmode();
  void cmd_seekl();
  void cmd_getid();
  void cmd_reads();
  void cmd_test();
  void cmd_gettn();
  void cmd_gettd();
  void cmd_stop();
  void cmd_standby();
  void cmd_mute();
  void cmd_demute();
  void cmd_setfilter();
  void cmd_setsession();
  void cmd_getloc_l();
  void cmd_getloc_p();
  void cmd_readtoc();
  void execute_internal_command(u8 cmd, std::initializer_list<u8> params);

  // Helpers
  int msf_to_lba(u8 mm, u8 ss, u8 ff) const;
  int read_period_for_mode() const;
  int command_busy_for(u8 cmd) const;
  void refresh_read_period();
  void schedule_second_response(int delay_cycles, u8 irq,
                                std::vector<u8> response);
  void start_read_stream(bool reads_mode);
  bool read_raw_sector_for_lba(int psx_lba, std::vector<u8> &raw_sector,
                               const CdTrack **track_out = nullptr);
  int track_end_lba(const CdTrack *track) const;
  bool stream_cdda_sector();
  bool cd_audio_muted() const;
  void apply_host_audio_matrix(std::vector<s16> &samples) const;
  bool read_sector();
  void maybe_decode_xa_audio(const std::vector<u8> &raw_sector,
                             const CdTrack *track);
  void fire_irq(u8 irq_num);
  void enqueue_irq(u8 irq_num, std::vector<u8> response,
                   bool wait_for_command_idle = true);
  void service_pending_irq();
  void refresh_irq_line();
  const CdTrack *track_for_lba(int lba) const;
  bool parse_cue(const std::string &cue_path, const std::string &bin_dir);

  // BCD conversion
  static u8 to_bcd(u8 val) { return ((val / 10) << 4) | (val % 10); }
  static u8 from_bcd(u8 bcd) { return (bcd >> 4) * 10 + (bcd & 0xF); }
};
