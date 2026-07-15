#include "cdrom.h"
#include "cpu.h"
#include "dma.h"
#include "gpu.h"
#include "gte.h"
#include "interrupt.h"
#include "mdec.h"
#include "spu.h"
#include "sio.h"
#include "timer.h"

#include <cstring>
#include <vector>

// ── GTE ─────────────────────────────────────────────────────────────

void Gte::save_state(std::vector<u8>& buf) const {
  auto write = [&](const void* data, size_t size) {
    const u8* src = static_cast<const u8*>(data);
    buf.insert(buf.end(), src, src + size);
  };
  auto write_val = [&](const auto& v) { write(&v, sizeof(v)); };

  write(v0, sizeof(v0));
  write(v1, sizeof(v1));
  write(v2, sizeof(v2));
  write(rgbc, sizeof(rgbc));
  write(ir, sizeof(ir));
  write(sx, sizeof(sx));
  write(sy, sizeof(sy));
  write(sz, sizeof(sz));
  write(rgb_fifo, sizeof(rgb_fifo));
  write(mac, sizeof(mac));
  write_val(otz);
  write_val(lzcs);
  write_val(lzcr);
  write(rotation, sizeof(rotation));
  write(translation, sizeof(translation));
  write(light, sizeof(light));
  write(bg_color, sizeof(bg_color));
  write(color_matrix, sizeof(color_matrix));
  write(far_color, sizeof(far_color));
  write_val(ofx);
  write_val(ofy);
  write_val(h);
  write_val(dqa);
  write_val(dqb);
  write_val(zsf3);
  write_val(zsf4);
  write_val(flags);
  write_val(static_cast<u8>(lm));
  write_val(static_cast<s32>(sf));
  write_val(current_command_);
}

void Gte::restore_state(const u8*& pos, size_t& remaining) {
  auto read = [&](void* dest, size_t size) {
    std::memcpy(dest, pos, size);
    pos += size;
    remaining -= size;
  };
  auto read_val = [&](auto& v) { read(&v, sizeof(v)); };

  read(v0, sizeof(v0));
  read(v1, sizeof(v1));
  read(v2, sizeof(v2));
  read(rgbc, sizeof(rgbc));
  read(ir, sizeof(ir));
  read(sx, sizeof(sx));
  read(sy, sizeof(sy));
  read(sz, sizeof(sz));
  read(rgb_fifo, sizeof(rgb_fifo));
  read(mac, sizeof(mac));
  read_val(otz);
  read_val(lzcs);
  read_val(lzcr);
  read(rotation, sizeof(rotation));
  read(translation, sizeof(translation));
  read(light, sizeof(light));
  read(bg_color, sizeof(bg_color));
  read(color_matrix, sizeof(color_matrix));
  read(far_color, sizeof(far_color));
  read_val(ofx);
  read_val(ofy);
  read_val(h);
  read_val(dqa);
  read_val(dqb);
  read_val(zsf3);
  read_val(zsf4);
  read_val(flags);
  { u8 v; read_val(v); lm = v != 0; }
  { s32 v; read_val(v); sf = v; }
  read_val(current_command_);
}

// ── InterruptController ─────────────────────────────────────────────

void InterruptController::save_state(std::vector<u8>& buf) const {
  auto write_val = [&](const auto& v) { buf.insert(buf.end(), reinterpret_cast<const u8*>(&v), reinterpret_cast<const u8*>(&v) + sizeof(v)); };

  write_val(i_stat_);
  write_val(i_mask_);
  write_val(line_state_);
}

void InterruptController::restore_state(const u8*& pos, size_t& remaining) {
  auto read_val = [&](auto& v) { std::memcpy(&v, pos, sizeof(v)); pos += sizeof(v); remaining -= sizeof(v); };

  read_val(i_stat_);
  read_val(i_mask_);
  read_val(line_state_);
}

// ── Timers ──────────────────────────────────────────────────────────

void Timers::save_state(std::vector<u8>& buf) const {
  auto write_val = [&](const auto& v) { buf.insert(buf.end(), reinterpret_cast<const u8*>(&v), reinterpret_cast<const u8*>(&v) + sizeof(v)); };

  for (int i = 0; i < 3; ++i) {
    const Timer& t = timers_[i];
    write_val(t.counter);
    write_val(t.target);
    write_val(t.mode);
    write_val(static_cast<u8>(t.one_shot_done));
    write_val(static_cast<u8>(t.sync_released));
    write_val(static_cast<u8>(t.irq_pulse_restore_pending));
  }
  write_val(static_cast<u8>(hblank_active_));
  write_val(static_cast<u8>(vblank_active_));
  write_val(timer0_dot_cycle_remainder_);
  write_val(timer2_sysclk8_cycle_remainder_);
}

void Timers::restore_state(const u8*& pos, size_t& remaining) {
  auto read_val = [&](auto& v) { std::memcpy(&v, pos, sizeof(v)); pos += sizeof(v); remaining -= sizeof(v); };

  for (int i = 0; i < 3; ++i) {
    Timer& t = timers_[i];
    read_val(t.counter);
    read_val(t.target);
    read_val(t.mode);
    { u8 v; read_val(v); t.one_shot_done = v != 0; }
    { u8 v; read_val(v); t.sync_released = v != 0; }
    { u8 v; read_val(v); t.irq_pulse_restore_pending = v != 0; }
  }
  { u8 v; read_val(v); hblank_active_ = v != 0; }
  { u8 v; read_val(v); vblank_active_ = v != 0; }
  read_val(timer0_dot_cycle_remainder_);
  read_val(timer2_sysclk8_cycle_remainder_);
}

// ── DmaController ───────────────────────────────────────────────────

void DmaController::save_state(std::vector<u8>& buf) const {
  auto write_val = [&](const auto& v) { buf.insert(buf.end(), reinterpret_cast<const u8*>(&v), reinterpret_cast<const u8*>(&v) + sizeof(v)); };

  for (int i = 0; i < 7; ++i) {
    const DmaChannel& ch = channels_[i];
    write_val(ch.base_addr);
    write_val(ch.block_ctrl);
    write_val(ch.channel_ctrl);
    write_val(ch.block_words_remaining);
  }
  write_val(dpcr_);
  write_val(dicr_);
}

void DmaController::restore_state(const u8*& pos, size_t& remaining) {
  auto read_val = [&](auto& v) { std::memcpy(&v, pos, sizeof(v)); pos += sizeof(v); remaining -= sizeof(v); };

  for (int i = 0; i < 7; ++i) {
    DmaChannel& ch = channels_[i];
    read_val(ch.base_addr);
    read_val(ch.block_ctrl);
    read_val(ch.channel_ctrl);
    read_val(ch.block_words_remaining);
  }
  read_val(dpcr_);
  read_val(dicr_);
}

// ── Sio ─────────────────────────────────────────────────────────────

void Sio::save_state(std::vector<u8>& buf) const {
  auto write_val = [&](const auto& v) { buf.insert(buf.end(), reinterpret_cast<const u8*>(&v), reinterpret_cast<const u8*>(&v) + sizeof(v)); };

  write_val(receive_buffer_);
  write_val(transmit_buffer_);
  write_val(transmit_value_);
  write_val(static_cast<u8>(receive_buffer_full_));
  write_val(static_cast<u8>(transmit_buffer_full_));
  write_val(stat_);
  write_val(mode_);
  write_val(ctrl_);
  write_val(baud_);
  write_val(static_cast<u8>(transfer_state_));
  write_val(static_cast<s32>(transfer_event_cycles_));
  write_val(static_cast<u8>(irq_flag_));
  write_val(static_cast<u8>(irq_pending_));
  write_val(static_cast<u8>(ack_input_));
  write_val(static_cast<u8>(active_device_));
  write_val(active_slot_);
  write_val(static_cast<u8>(connected_));
}

void Sio::restore_state(const u8*& pos, size_t& remaining) {
  auto read_val = [&](auto& v) { std::memcpy(&v, pos, sizeof(v)); pos += sizeof(v); remaining -= sizeof(v); };

  read_val(receive_buffer_);
  read_val(transmit_buffer_);
  read_val(transmit_value_);
  { u8 v; read_val(v); receive_buffer_full_ = v != 0; }
  { u8 v; read_val(v); transmit_buffer_full_ = v != 0; }
  read_val(stat_);
  read_val(mode_);
  read_val(ctrl_);
  read_val(baud_);
  { u8 v; read_val(v); transfer_state_ = static_cast<TransferState>(v); }
  { s32 v; read_val(v); transfer_event_cycles_ = v; }
  { u8 v; read_val(v); irq_flag_ = v != 0; }
  { u8 v; read_val(v); irq_pending_ = v != 0; }
  { u8 v; read_val(v); ack_input_ = v != 0; }
  { u8 v; read_val(v); active_device_ = static_cast<ActiveDevice>(v); }
  read_val(active_slot_);
  { u8 v; read_val(v); connected_ = v != 0; }
}

// ── CdRom ───────────────────────────────────────────────────────────

void CdRom::save_state(std::vector<u8>& buf) const {
  auto write = [&](const void* data, size_t size) {
    const u8* src = static_cast<const u8*>(data);
    buf.insert(buf.end(), src, src + size);
  };
  auto write_val = [&](const auto& v) { buf.insert(buf.end(), reinterpret_cast<const u8*>(&v), reinterpret_cast<const u8*>(&v) + sizeof(v)); };
  auto write_vec = [&](const auto& vec) {
    u32 sz = static_cast<u32>(vec.size());
    write_val(sz);
    if (sz > 0) {
      buf.insert(buf.end(), vec.begin(), vec.end());
    }
  };

  write_val(index_reg_);
  write_val(interrupt_enable_);
  write_val(interrupt_flag_);
  write_vec(param_fifo_);
  write_vec(response_fifo_);
  write_val(static_cast<s32>(response_index_));
  write_vec(data_buffer_);
  write_val(static_cast<s32>(data_index_));
  write_val(static_cast<u8>(data_ready_));
  write_val(static_cast<u8>(data_request_));
  write_val(static_cast<u8>(motor_on_));
  write_val(static_cast<u8>(shell_open_));
  write_val(static_cast<u8>(seek_error_));
  write_val(static_cast<u8>(id_error_));
  write_val(static_cast<u8>(state_));

  write_val(static_cast<u8>(pending_second_.active));
  write_val(static_cast<u32>(pending_second_.delay));
  write_val(pending_second_.irq);
  write_vec(pending_second_.response);

  write_val(static_cast<u8>(pending_async_irq_.active));
  write_val(static_cast<u32>(pending_async_irq_.delay));
  write_val(pending_async_irq_.irq);
  write_vec(pending_async_irq_.response);

  u32 pending_irq_count = static_cast<u32>(pending_irqs_.size());
  write_val(pending_irq_count);
  for (const auto& irq : pending_irqs_) {
    write_val(irq.irq);
    write_val(static_cast<u8>(irq.wait_for_command_idle));
    write_vec(irq.response);
  }

  u32 queued_count = static_cast<u32>(queued_sector_buffers_.size());
  write_val(queued_count);
  for (const auto& buf_entry : queued_sector_buffers_) {
    write_vec(buf_entry.payload);
    write_val(static_cast<s32>(buf_entry.data_lba));
    write_val(static_cast<u8>(buf_entry.realtime_stream));
  }

  write_val(seek_mm_);
  write_val(seek_ss_);
  write_val(seek_ff_);
  write_val(static_cast<s32>(read_lba_));
  write_val(static_cast<s32>(pending_cycles_));
  write_val(static_cast<s32>(read_period_cycles_));
  write_val(static_cast<u8>(command_busy_));
  write_val(static_cast<s32>(command_busy_cycles_));
  write_val(last_command_);
  write_val(last_irq_code_);
  write_val(mode_);
  write_val(filter_file_);
  write_val(filter_channel_);
  write(host_audio_regs_.data(), host_audio_regs_.size());
  write_val(host_audio_apply_);
  write_val(command_counter_);
  write(command_hist_.data(), command_hist_.size() * sizeof(u64));
  write_val(sector_counter_);
  write_val(static_cast<u8>(saw_read_command_));
  write_val(static_cast<u8>(saw_getid_));
  write_val(static_cast<u8>(saw_setloc_));
  write_val(static_cast<u8>(saw_seekl_));
  write_val(static_cast<u8>(saw_readn_or_reads_));
  write_val(static_cast<u8>(saw_sector_visible_));
  write_val(read_command_count_);
  write_val(irq_int1_count_);
  write_val(irq_int2_count_);
  write_val(irq_int3_count_);
  write_val(irq_int4_count_);
  write_val(irq_int5_count_);
  write_val(read_buffer_stall_count_);
  write_val(response_promotion_count_);
  write_val(status_e0_poll_count_);
  write_val(status_e0_streak_max_);
  write_val(status_e0_streak_current_);
  write_val(bin_size_);
  write_val(static_cast<s32>(seek_target_lba_));
  write_val(static_cast<u8>(seek_target_valid_));
  write_val(static_cast<u8>(seek_complete_));
  write_val(static_cast<u8>(read_whole_sector_));
  write_val(static_cast<u8>(pending_read_start_));
  write_val(static_cast<u8>(read_startup_pending_));
  write_val(static_cast<u8>(pending_reads_mode_));
  write_val(static_cast<u8>(cdda_playing_));
  write_val(static_cast<u8>(cdda_cmd_muted_));
  write_val(static_cast<u8>(cdda_adp_muted_));
  write_val(static_cast<s32>(adpcm_busy_cycles_));
  write(atv_pending_.data(), atv_pending_.size());
  write(atv_active_.data(), atv_active_.size());
  write(xa_hist1_.data(), xa_hist1_.size() * sizeof(s16));
  write(xa_hist2_.data(), xa_hist2_.size() * sizeof(s16));
  write_val(static_cast<u8>(xa_stream_valid_));
  write_val(xa_stream_file_);
  write_val(xa_stream_channel_);
  write_val(static_cast<u8>(insert_probe_active_));
  write_val(static_cast<s32>(insert_probe_delay_cycles_));
  write_val(static_cast<s32>(insert_probe_stage_));
  write_val(static_cast<u8>(irq_line_request_pending_));
  write_val(static_cast<s32>(irq_line_delay_cycles_));
  write_val(static_cast<u8>(sector_redelivery_pending_));
  write_val(static_cast<s32>(sector_redelivery_delay_cycles_));
  write_val(last_irq_clear_cycle_);

  write_vec(last_sector_payload_);
  write_val(static_cast<u8>(last_sector_had_data_));
  write_val(static_cast<u8>(last_sector_realtime_stream_));
  write(last_sector_location_.data(), last_sector_location_.size());
  write_val(static_cast<u8>(last_sector_location_valid_));
  write_val(static_cast<s32>(last_sector_lba_));
  write_val(static_cast<s32>(active_data_lba_));
  write_val(static_cast<u8>(active_data_realtime_stream_));
  write_val(static_cast<u8>(active_data_stream_header_consumed_));
}

void CdRom::restore_state(const u8*& pos, size_t& remaining) {
  auto read_val = [&](auto& v) { std::memcpy(&v, pos, sizeof(v)); pos += sizeof(v); remaining -= sizeof(v); };
  auto read_vec = [&](auto& vec) {
    u32 sz = 0;
    read_val(sz);
    vec.resize(sz);
    if (sz > 0) {
      std::memcpy(vec.data(), pos, sz);
      pos += sz;
      remaining -= sz;
    }
  };

  read_val(index_reg_);
  read_val(interrupt_enable_);
  read_val(interrupt_flag_);
  read_vec(param_fifo_);
  read_vec(response_fifo_);
  { s32 v; read_val(v); response_index_ = v; }
  read_vec(data_buffer_);
  { s32 v; read_val(v); data_index_ = v; }
  { u8 v; read_val(v); data_ready_ = v != 0; }
  { u8 v; read_val(v); data_request_ = v != 0; }
  { u8 v; read_val(v); motor_on_ = v != 0; }
  { u8 v; read_val(v); shell_open_ = v != 0; }
  { u8 v; read_val(v); seek_error_ = v != 0; }
  { u8 v; read_val(v); id_error_ = v != 0; }
  { u8 v; read_val(v); state_ = static_cast<State>(v); }

  { u8 v; read_val(v); pending_second_.active = v != 0; }
  { u32 v; read_val(v); pending_second_.delay = static_cast<int>(v); }
  read_val(pending_second_.irq);
  read_vec(pending_second_.response);

  { u8 v; read_val(v); pending_async_irq_.active = v != 0; }
  { u32 v; read_val(v); pending_async_irq_.delay = static_cast<int>(v); }
  read_val(pending_async_irq_.irq);
  read_vec(pending_async_irq_.response);

  pending_irqs_.clear();
  u32 pending_irq_count = 0;
  read_val(pending_irq_count);
  for (u32 i = 0; i < pending_irq_count; ++i) {
    PendingIrq irq_entry{};
    read_val(irq_entry.irq);
    { u8 v; read_val(v); irq_entry.wait_for_command_idle = v != 0; }
    read_vec(irq_entry.response);
    pending_irqs_.push_back(std::move(irq_entry));
  }

  queued_sector_buffers_.clear();
  u32 queued_count = 0;
  read_val(queued_count);
  for (u32 i = 0; i < queued_count; ++i) {
    QueuedSectorBuffer qb{};
    read_vec(qb.payload);
    { s32 v; read_val(v); qb.data_lba = v; }
    { u8 v; read_val(v); qb.realtime_stream = v != 0; }
    queued_sector_buffers_.push_back(std::move(qb));
  }

  read_val(seek_mm_);
  read_val(seek_ss_);
  read_val(seek_ff_);
  { s32 v; read_val(v); read_lba_ = v; }
  { s32 v; read_val(v); pending_cycles_ = v; }
  { s32 v; read_val(v); read_period_cycles_ = v; }
  { u8 v; read_val(v); command_busy_ = v != 0; }
  { s32 v; read_val(v); command_busy_cycles_ = v; }
  read_val(last_command_);
  read_val(last_irq_code_);
  read_val(mode_);
  read_val(filter_file_);
  read_val(filter_channel_);
  std::memcpy(host_audio_regs_.data(), pos, host_audio_regs_.size());
  pos += host_audio_regs_.size();
  remaining -= host_audio_regs_.size();
  read_val(host_audio_apply_);
  read_val(command_counter_);
  std::memcpy(command_hist_.data(), pos, command_hist_.size() * sizeof(u64));
  pos += command_hist_.size() * sizeof(u64);
  remaining -= command_hist_.size() * sizeof(u64);
  read_val(sector_counter_);
  { u8 v; read_val(v); saw_read_command_ = v != 0; }
  { u8 v; read_val(v); saw_getid_ = v != 0; }
  { u8 v; read_val(v); saw_setloc_ = v != 0; }
  { u8 v; read_val(v); saw_seekl_ = v != 0; }
  { u8 v; read_val(v); saw_readn_or_reads_ = v != 0; }
  { u8 v; read_val(v); saw_sector_visible_ = v != 0; }
  read_val(read_command_count_);
  read_val(irq_int1_count_);
  read_val(irq_int2_count_);
  read_val(irq_int3_count_);
  read_val(irq_int4_count_);
  read_val(irq_int5_count_);
  read_val(read_buffer_stall_count_);
  read_val(response_promotion_count_);
  read_val(status_e0_poll_count_);
  read_val(status_e0_streak_max_);
  read_val(status_e0_streak_current_);
  read_val(bin_size_);
  { s32 v; read_val(v); seek_target_lba_ = v; }
  { u8 v; read_val(v); seek_target_valid_ = v != 0; }
  { u8 v; read_val(v); seek_complete_ = v != 0; }
  { u8 v; read_val(v); read_whole_sector_ = v != 0; }
  { u8 v; read_val(v); pending_read_start_ = v != 0; }
  { u8 v; read_val(v); read_startup_pending_ = v != 0; }
  { u8 v; read_val(v); pending_reads_mode_ = v != 0; }
  { u8 v; read_val(v); cdda_playing_ = v != 0; }
  { u8 v; read_val(v); cdda_cmd_muted_ = v != 0; }
  { u8 v; read_val(v); cdda_adp_muted_ = v != 0; }
  { s32 v; read_val(v); adpcm_busy_cycles_ = v; }
  std::memcpy(atv_pending_.data(), pos, atv_pending_.size());
  pos += atv_pending_.size();
  remaining -= atv_pending_.size();
  std::memcpy(atv_active_.data(), pos, atv_active_.size());
  pos += atv_active_.size();
  remaining -= atv_active_.size();
  std::memcpy(xa_hist1_.data(), pos, xa_hist1_.size() * sizeof(s16));
  pos += xa_hist1_.size() * sizeof(s16);
  remaining -= xa_hist1_.size() * sizeof(s16);
  std::memcpy(xa_hist2_.data(), pos, xa_hist2_.size() * sizeof(s16));
  pos += xa_hist2_.size() * sizeof(s16);
  remaining -= xa_hist2_.size() * sizeof(s16);
  { u8 v; read_val(v); xa_stream_valid_ = v != 0; }
  read_val(xa_stream_file_);
  read_val(xa_stream_channel_);
  { u8 v; read_val(v); insert_probe_active_ = v != 0; }
  { s32 v; read_val(v); insert_probe_delay_cycles_ = v; }
  { s32 v; read_val(v); insert_probe_stage_ = v; }
  { u8 v; read_val(v); irq_line_request_pending_ = v != 0; }
  { s32 v; read_val(v); irq_line_delay_cycles_ = v; }
  { u8 v; read_val(v); sector_redelivery_pending_ = v != 0; }
  { s32 v; read_val(v); sector_redelivery_delay_cycles_ = v; }
  read_val(last_irq_clear_cycle_);

  read_vec(last_sector_payload_);
  { u8 v; read_val(v); last_sector_had_data_ = v != 0; }
  { u8 v; read_val(v); last_sector_realtime_stream_ = v != 0; }
  std::memcpy(last_sector_location_.data(), pos, last_sector_location_.size());
  pos += last_sector_location_.size();
  remaining -= last_sector_location_.size();
  { u8 v; read_val(v); last_sector_location_valid_ = v != 0; }
  { s32 v; read_val(v); last_sector_lba_ = v; }
  { s32 v; read_val(v); active_data_lba_ = v; }
  { u8 v; read_val(v); active_data_realtime_stream_ = v != 0; }
  { u8 v; read_val(v); active_data_stream_header_consumed_ = v != 0; }
}

// ── Spu ─────────────────────────────────────────────────────────────

void Spu::save_state(std::vector<u8>& buf) const {
  auto write = [&](const void* data, size_t size) {
    const u8* src = static_cast<const u8*>(data);
    buf.insert(buf.end(), src, src + size);
  };
  auto write_val = [&](const auto& v) { write(&v, sizeof(v)); };

  write(spu_ram_.data(), spu_ram_.size());
  write(regs_.data(), regs_.size() * sizeof(u16));

  for (int i = 0; i < NUM_VOICES; ++i) {
    const VoiceState& v = voices_[i];
    write_val(v.key_on);
    write_val(v.addr);
    write_val(v.repeat_addr);
    write_val(v.last_block_addr);
    write_val(v.last_adpcm_flags);
    write_val(static_cast<u32>(v.sample_index));
    write_val(v.stop_after_block);
    write_val(v.pitch_counter);
    write(v.decoded.data(), v.decoded.size() * sizeof(s16));
    write_val(v.hist1);
    write_val(v.hist2);
    write(v.gauss_hist.data(), v.gauss_hist.size() * sizeof(s16));
    write_val(v.gauss_ready);
    write_val(v.current_vol_l);
    write_val(v.current_vol_r);
    write_val(v.sweep_vol_l);
    write_val(v.sweep_vol_r);
    write_val(static_cast<u32>(v.phase));
    write_val(v.env_level);
    write_val(v.sustain_level);
    write_val(v.adsr_counter);
    write_val(v.attack_shift);
    write_val(v.attack_step);
    write_val(v.decay_shift);
    write_val(v.sustain_shift);
    write_val(v.sustain_step);
    write_val(v.release_shift);
    write_val(v.attack_exp);
    write_val(v.sustain_exp);
    write_val(v.sustain_decrease);
    write_val(v.release_exp);
    write_val(v.release_tracking);
    write_val(v.release_start_sample);
    write_val(v.use_replacement_sample);
  }

  write_val(spucnt_);
  write_val(spustat_);
  write_val(spucnt_mode_latched_);
  write_val(spucnt_mode_pending_);
  write_val(spucnt_mode_delay_cycles_);
  write_val(irq_addr_);
  write_val(transfer_addr_);
  write_val(transfer_busy_cycles_);
  write_val(capture_half_);

  write_val(pitch_mod_mask_);
  write_val(noise_on_mask_);
  write_val(reverb_on_mask_);
  write_val(endx_mask_);

  write_val(master_vol_l_);
  write_val(master_vol_r_);
  write_val(reverb_depth_l_);
  write_val(reverb_depth_r_);
  write_val(cd_vol_l_);
  write_val(cd_vol_r_);
  write_val(ext_vol_l_);
  write_val(ext_vol_r_);

  write_val(reverb_base_addr_);
  write(&reverb_regs_, sizeof(ReverbRegs));
  write(&reverb_state_, sizeof(ReverbState));

  write_val(sample_accum_);
  write_val(sample_clock_);
  write_val(last_synced_cpu_cycle_);

  write_val(noise_level_);
  write_val(noise_timer_);

  write_val(pending_kon_mask_);
  write_val(pending_koff_mask_);

  write_val(last_kon_write_sample_);
  write_val(has_last_kon_write_sample_);
  write_val(last_koff_write_sample_);
  write_val(has_last_koff_write_sample_);
}

void Spu::restore_state(const u8*& pos, size_t& remaining) {
  auto read = [&](void* dest, size_t size) {
    std::memcpy(dest, pos, size);
    pos += size;
    remaining -= size;
  };
  auto read_val = [&](auto& v) { read(&v, sizeof(v)); };

  read(spu_ram_.data(), spu_ram_.size());
  read(regs_.data(), regs_.size() * sizeof(u16));

  for (int i = 0; i < NUM_VOICES; ++i) {
    VoiceState& v = voices_[i];
    read_val(v.key_on);
    read_val(v.addr);
    read_val(v.repeat_addr);
    read_val(v.last_block_addr);
    read_val(v.last_adpcm_flags);
    { u32 sv; read_val(sv); v.sample_index = static_cast<int>(sv); }
    read_val(v.stop_after_block);
    read_val(v.pitch_counter);
    read(v.decoded.data(), v.decoded.size() * sizeof(s16));
    read_val(v.hist1);
    read_val(v.hist2);
    read(v.gauss_hist.data(), v.gauss_hist.size() * sizeof(s16));
    read_val(v.gauss_ready);
    read_val(v.current_vol_l);
    read_val(v.current_vol_r);
    read_val(v.sweep_vol_l);
    read_val(v.sweep_vol_r);
    { u32 sv; read_val(sv); v.phase = static_cast<VoiceState::AdsrPhase>(sv); }
    read_val(v.env_level);
    read_val(v.sustain_level);
    read_val(v.adsr_counter);
    read_val(v.attack_shift);
    read_val(v.attack_step);
    read_val(v.decay_shift);
    read_val(v.sustain_shift);
    read_val(v.sustain_step);
    read_val(v.release_shift);
    read_val(v.attack_exp);
    read_val(v.sustain_exp);
    read_val(v.sustain_decrease);
    read_val(v.release_exp);
    read_val(v.release_tracking);
    read_val(v.release_start_sample);
    read_val(v.use_replacement_sample);
  }

  read_val(spucnt_);
  read_val(spustat_);
  read_val(spucnt_mode_latched_);
  read_val(spucnt_mode_pending_);
  read_val(spucnt_mode_delay_cycles_);
  read_val(irq_addr_);
  read_val(transfer_addr_);
  read_val(transfer_busy_cycles_);
  read_val(capture_half_);

  read_val(pitch_mod_mask_);
  read_val(noise_on_mask_);
  read_val(reverb_on_mask_);
  read_val(endx_mask_);

  read_val(master_vol_l_);
  read_val(master_vol_r_);
  read_val(reverb_depth_l_);
  read_val(reverb_depth_r_);
  read_val(cd_vol_l_);
  read_val(cd_vol_r_);
  read_val(ext_vol_l_);
  read_val(ext_vol_r_);

  read_val(reverb_base_addr_);
  read(&reverb_regs_, sizeof(ReverbRegs));
  read(&reverb_state_, sizeof(ReverbState));

  read_val(sample_accum_);
  read_val(sample_clock_);
  read_val(last_synced_cpu_cycle_);

  read_val(noise_level_);
  read_val(noise_timer_);

  read_val(pending_kon_mask_);
  read_val(pending_koff_mask_);

  read_val(last_kon_write_sample_);
  read_val(has_last_kon_write_sample_);
  read_val(last_koff_write_sample_);
  read_val(has_last_koff_write_sample_);
}

// ── Mdec ────────────────────────────────────────────────────────────

void Mdec::save_state(std::vector<u8>& buf) const {
  auto write = [&](const void* data, size_t size) {
    const u8* src = static_cast<const u8*>(data);
    buf.insert(buf.end(), src, src + size);
  };
  auto write_val = [&](const auto& v) { write(&v, sizeof(v)); };
  auto write_deque_u32 = [&](const std::deque<u32>& d) {
    u32 sz = static_cast<u32>(d.size());
    write_val(sz);
    for (u32 i = 0; i < sz; ++i) { write_val(d[i]); }
  };
  auto write_deque_u16 = [&](const std::deque<u16>& d) {
    u32 sz = static_cast<u32>(d.size());
    write_val(sz);
    for (u32 i = 0; i < sz; ++i) { write_val(d[i]); }
  };
  auto write_deque_u8 = [&](const std::deque<u8>& d) {
    u32 sz = static_cast<u32>(d.size());
    write_val(sz);
    for (u32 i = 0; i < sz; ++i) { write_val(d[i]); }
  };

  write_val(control_);
  write_val(static_cast<u8>(command_busy_));
  write_val(static_cast<u8>(expect_command_word_));
  write_val(command_id_);
  write_val(command_word_);
  write_val(in_words_remaining_);
  write_val(decode_halfwords_remaining_);
  write_val(static_cast<u8>(in_unlimited_));
  write_deque_u16(in_halfword_fifo_);

  write(quant_luma_.data(), quant_luma_.size());
  write(quant_chroma_.data(), quant_chroma_.size());
  write(scale_table_.data(), scale_table_.size() * sizeof(s16));

  write_val(status_command_bits_);
  write_val(current_block_);
  write_val(decode_block_index_);
  write_val(current_coefficient_);
  write_val(current_q_scale_);
  write(decode_blocks_.data(), sizeof(MacroblockBlocks));

  write_val(output_depth_);
  write_val(static_cast<u8>(output_signed_));
  write_val(static_cast<u8>(output_set_bit15_));
  write_val(output_pack_word_);
  write_val(output_pack_bytes_);
  write_val(output_word_block_id_);
  write_val(output_macroblock_seq_);
  write_val(current_output_macroblock_seq_);
  write_val(output_ready_delay_cycles_);
  write_val(out_depth_latched_);

  write_deque_u32(out_fifo_);
  write_deque_u8(out_block_fifo_);
  write_deque_u32(out_macroblock_fifo_);
  write_deque_u32(pending_out_fifo_);
  write_deque_u8(pending_out_block_fifo_);
  write_deque_u32(pending_out_macroblock_fifo_);
}

void Mdec::restore_state(const u8*& pos, size_t& remaining) {
  auto read = [&](void* dest, size_t size) {
    std::memcpy(dest, pos, size);
    pos += size;
    remaining -= size;
  };
  auto read_val = [&](auto& v) { read(&v, sizeof(v)); };
  auto read_deque_u32 = [&](std::deque<u32>& d) {
    d.clear();
    u32 sz = 0;
    read_val(sz);
    for (u32 i = 0; i < sz; ++i) { u32 v; read_val(v); d.push_back(v); }
  };
  auto read_deque_u16 = [&](std::deque<u16>& d) {
    d.clear();
    u32 sz = 0;
    read_val(sz);
    for (u32 i = 0; i < sz; ++i) { u16 v; read_val(v); d.push_back(v); }
  };
  auto read_deque_u8 = [&](std::deque<u8>& d) {
    d.clear();
    u32 sz = 0;
    read_val(sz);
    for (u32 i = 0; i < sz; ++i) { u8 v; read_val(v); d.push_back(v); }
  };

  read_val(control_);
  { u8 v; read_val(v); command_busy_ = v != 0; }
  { u8 v; read_val(v); expect_command_word_ = v != 0; }
  read_val(command_id_);
  read_val(command_word_);
  read_val(in_words_remaining_);
  read_val(decode_halfwords_remaining_);
  { u8 v; read_val(v); in_unlimited_ = v != 0; }
  read_deque_u16(in_halfword_fifo_);

  read(quant_luma_.data(), quant_luma_.size());
  read(quant_chroma_.data(), quant_chroma_.size());
  read(scale_table_.data(), scale_table_.size() * sizeof(s16));

  read_val(status_command_bits_);
  read_val(current_block_);
  read_val(decode_block_index_);
  read_val(current_coefficient_);
  read_val(current_q_scale_);
  read(decode_blocks_.data(), sizeof(MacroblockBlocks));

  read_val(output_depth_);
  { u8 v; read_val(v); output_signed_ = v != 0; }
  { u8 v; read_val(v); output_set_bit15_ = v != 0; }
  read_val(output_pack_word_);
  read_val(output_pack_bytes_);
  read_val(output_word_block_id_);
  read_val(output_macroblock_seq_);
  read_val(current_output_macroblock_seq_);
  read_val(output_ready_delay_cycles_);
  read_val(out_depth_latched_);

  read_deque_u32(out_fifo_);
  read_deque_u8(out_block_fifo_);
  read_deque_u32(out_macroblock_fifo_);
  read_deque_u32(pending_out_fifo_);
  read_deque_u8(pending_out_block_fifo_);
  read_deque_u32(pending_out_macroblock_fifo_);
}

// ── Gpu ─────────────────────────────────────────────────────────────

void Gpu::save_state(std::vector<u8>& buf) const {
  auto write = [&](const void* data, size_t size) {
    const u8* src = static_cast<const u8*>(data);
    buf.insert(buf.end(), src, src + size);
  };
  auto write_val = [&](const auto& v) { write(&v, sizeof(v)); };

  write(vram_.data(), vram_.size() * sizeof(u16));

  write_val(draw_x_min_);
  write_val(draw_y_min_);
  write_val(draw_x_max_);
  write_val(draw_y_max_);
  write_val(draw_x_offset_);
  write_val(draw_y_offset_);
  write_val(texpage_);
  write_val(clut_);
  write_val(tex_window_mask_x_);
  write_val(tex_window_mask_y_);
  write_val(tex_window_off_x_);
  write_val(tex_window_off_y_);

  write(&display_, sizeof(DisplayMode));

  write_val(dma_direction_);
  write_val(static_cast<u8>(dither_enabled_));
  write_val(static_cast<u8>(draw_to_display_));
  write_val(static_cast<u8>(texture_disable_));
  write_val(static_cast<u8>(semi_transparency_mode_));
  write_val(semi_transparency_);
  write_val(static_cast<u8>(tex_rect_x_flip_));
  write_val(static_cast<u8>(tex_rect_y_flip_));
  write_val(static_cast<u8>(irq1_pending_));
  write_val(static_cast<u8>(force_set_mask_bit_));
  write_val(static_cast<u8>(check_mask_before_draw_));
  write_val(static_cast<u8>(interlace_field_));
  write_val(gpuread_latch_);

  write_val(static_cast<u32>(gp0_mode_));
  write_val(gp0_words_remaining_);
  write_val(gp0_command_);

  write_val(vram_tx_x_);
  write_val(vram_tx_y_);
  write_val(vram_tx_w_);
  write_val(vram_tx_h_);
  write_val(vram_tx_pos_);
  write_val(vram_tx_total_);

  write_val(static_cast<u8>(frame_complete_));
}

void Gpu::restore_state(const u8*& pos, size_t& remaining) {
  auto read = [&](void* dest, size_t size) {
    std::memcpy(dest, pos, size);
    pos += size;
    remaining -= size;
  };
  auto read_val = [&](auto& v) { read(&v, sizeof(v)); };

  read(vram_.data(), vram_.size() * sizeof(u16));

  read_val(draw_x_min_);
  read_val(draw_y_min_);
  read_val(draw_x_max_);
  read_val(draw_y_max_);
  read_val(draw_x_offset_);
  read_val(draw_y_offset_);
  read_val(texpage_);
  read_val(clut_);
  read_val(tex_window_mask_x_);
  read_val(tex_window_mask_y_);
  read_val(tex_window_off_x_);
  read_val(tex_window_off_y_);

  read(&display_, sizeof(DisplayMode));

  read_val(dma_direction_);
  { u8 v; read_val(v); dither_enabled_ = v != 0; }
  { u8 v; read_val(v); draw_to_display_ = v != 0; }
  { u8 v; read_val(v); texture_disable_ = v != 0; }
  { u8 v; read_val(v); semi_transparency_mode_ = v != 0; }
  read_val(semi_transparency_);
  { u8 v; read_val(v); tex_rect_x_flip_ = v != 0; }
  { u8 v; read_val(v); tex_rect_y_flip_ = v != 0; }
  { u8 v; read_val(v); irq1_pending_ = v != 0; }
  { u8 v; read_val(v); force_set_mask_bit_ = v != 0; }
  { u8 v; read_val(v); check_mask_before_draw_ = v != 0; }
  { u8 v; read_val(v); interlace_field_ = v != 0; }
  read_val(gpuread_latch_);

  { u32 v; read_val(v); gp0_mode_ = static_cast<Gp0Mode>(v); }
  read_val(gp0_words_remaining_);
  read_val(gp0_command_);

  read_val(vram_tx_x_);
  read_val(vram_tx_y_);
  read_val(vram_tx_w_);
  read_val(vram_tx_h_);
  read_val(vram_tx_pos_);
  read_val(vram_tx_total_);

  { u8 v; read_val(v); frame_complete_ = v != 0; }
}

// ── Cpu ─────────────────────────────────────────────────────────────

void Cpu::save_state(std::vector<u8>& buf) const {
  auto write = [&](const void* data, size_t size) {
    const u8* src = static_cast<const u8*>(data);
    buf.insert(buf.end(), src, src + size);
  };
  auto write_val = [&](const auto& v) { write(&v, sizeof(v)); };

  CpuDebugState ds = debug_state();
  write(&ds, sizeof(CpuDebugState));

  gte.save_state(buf);

  write(cop0_regs_, sizeof(cop0_regs_));
  write_val(gte_input_ready_cycle_);
  write_val(gte_result_ready_cycle_);
  write_val(muldiv_result_ready_cycle_);
}

void Cpu::restore_state(const u8*& pos, size_t& remaining) {
  auto read = [&](void* dest, size_t size) {
    std::memcpy(dest, pos, size);
    pos += size;
    remaining -= size;
  };
  auto read_val = [&](auto& v) { read(&v, sizeof(v)); };

  CpuDebugState ds;
  read(&ds, sizeof(CpuDebugState));
  debug_set_state(ds);

  gte.restore_state(pos, remaining);

  read(cop0_regs_, sizeof(cop0_regs_));
  read_val(gte_input_ready_cycle_);
  read_val(gte_result_ready_cycle_);
  read_val(muldiv_result_ready_cycle_);
}
