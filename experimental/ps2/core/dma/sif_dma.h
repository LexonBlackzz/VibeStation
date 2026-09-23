#pragma once

#include "common/types.h"

#include <array>
#include <string>
#include <vector>

namespace ps2 {

class EeBus;
class IopBus;
class IopIntc;

class SifDma {
public:
    struct Sif0PacketRecord {
        u32 source_tag = 0;
        u32 source_words = 0;
        u32 destination_tag = 0;
        u32 destination = 0;
        std::array<u32, 4> payload{};
    };

    struct Sif1PacketRecord {
        u32 destination_tag = 0;
        u32 words = 0;
        u32 destination = 0;
        std::array<u32, 4> payload{};
    };

    struct RpcCallRecord {
        u32 sid = 0;
        u32 rpc_number = 0;
        u32 send_size = 0;
        u32 server = 0;
        u32 server_buffer = 0;
        std::array<u32, 16> payload{};
        u32 payload_words = 0;
    };

    struct Stats {
        u64 sif0_packets = 0;
        u64 sif0_padded_packets = 0;
        u64 sif0_padding_words = 0;
        u64 sif0_stream_mismatches = 0;
        std::array<Sif0PacketRecord, 16> recent_sif0_packets{};
        u32 recent_sif0_count = 0;
        u32 recent_sif0_next = 0;
        u64 sif1_packets = 0;
        std::array<Sif1PacketRecord, 16> recent_sif1_packets{};
        u32 recent_sif1_count = 0;
        u32 recent_sif1_next = 0;

        u64 rpc_calls = 0;
        u64 sound_rpc_calls = 0;
        u64 sound_st_init_calls = 0;
        u64 sound_bgm_play_calls = 0;
        u64 sound_bgm_stop_calls = 0;
        u64 sound_bgm_open_calls = 0;
        u64 sound_tick_mode_calls = 0;
        u64 sound_master_volume_calls = 0;
        u64 sound_timer_start_calls = 0;
        u64 sound_se_play_calls = 0;
        u64 sound_set_param_calls = 0;
        u64 sound_set_switch_calls = 0;
        u64 sound_set_addr_calls = 0;
        std::array<RpcCallRecord, 256> recent_rpc_calls{};
        u32 recent_rpc_count = 0;
        u32 recent_rpc_next = 0;
    };

    void reset();

    [[nodiscard]] const Stats& stats() const { return stats_; }

    [[nodiscard]] bool service(
        EeBus& ee_bus,
        IopBus& iop_bus,
        IopIntc& iop_intc,
        std::string& error);

    void tick_ee(EeBus& ee_bus);
    [[nodiscard]] bool ee_completion_pending() const {
        return sif0_ee_completion_cycles_ != 0;
    }
    [[nodiscard]] bool iop_completion_pending() const {
        return sif0_iop_completion_cycles_ != 0 ||
               sif1_iop_completion_cycles_ != 0;
    }
    void tick_iop(IopBus& iop_bus);

private:
    [[nodiscard]] bool service_sif0(
        EeBus& ee_bus,
        IopBus& iop_bus,
        IopIntc& iop_intc,
        std::string& error);
    [[nodiscard]] bool service_sif1(
        EeBus& ee_bus,
        IopBus& iop_bus,
        IopIntc& iop_intc,
        std::string& error);

    [[nodiscard]] bool collect_ee_source_chain(
        EeBus& bus,
        u32 channel_base,
        std::vector<u32>& words,
        u32& final_chcr,
        std::string& error);

    static void append_qword(
        std::vector<u32>& words,
        u64 lo,
        u64 hi);

    Stats stats_{};
    u32 sif0_ee_completion_cycles_ = 0;
    u32 sif0_iop_completion_cycles_ = 0;
    u32 sif1_iop_completion_cycles_ = 0;
};

} // namespace ps2
