#pragma once

#include "common/types.h"

#include <array>
#include <string>

namespace ps2 {

class EeBus;
class GsCore;

struct Vu1Stats {
    u64 instructions = 0;
    u64 unsupported_upper = 0;
    u64 unsupported_lower = 0;
    u64 xgkicks = 0;
    u64 xgkick_qwords = 0;
};

class Vu1 {
public:
    Vu1(EeBus& bus, GsCore& gs);

    void reset();
    void start(u32 address);
    void continue_run();

    [[nodiscard]] bool step(std::string& error);
    u64 run(u64 instruction_budget, std::string& error);

    [[nodiscard]] bool running() const { return running_; }
    [[nodiscard]] u32 pc() const { return pc_; }
    [[nodiscard]] u16 vi(u32 index) const { return vi_[index & 0xFu]; }
    [[nodiscard]] u32 vf(u32 index, u32 lane) const {
        return vf_[index & 31u][lane & 3u];
    }
    [[nodiscard]] u32 p() const { return p_; }
    [[nodiscard]] u32 random() const { return r_; }
    [[nodiscard]] u32 status() const { return status_; }
    [[nodiscard]] u32 mac() const { return mac_; }
    [[nodiscard]] const Vu1Stats& stats() const { return stats_; }

private:
    [[nodiscard]] bool execute_upper(u32 code, std::string& error);
    [[nodiscard]] bool execute_upper_special(
        u32 code, u32 group, u32 index, std::string& error);
    [[nodiscard]] bool execute_lower(u32 code, std::string& error);
    [[nodiscard]] bool execute_lower_special(
        u32 code, u32 group, u32 index, std::string& error);

    void schedule_branch(u32 target);
    [[nodiscard]] bool xgkick(u16 qword_address, std::string& error);

    [[nodiscard]] bool read_data_word(u32 byte_offset, u32& value) const;
    [[nodiscard]] bool write_data_word(u32 byte_offset, u32 value);
    [[nodiscard]] bool read_data_qword(
        u32 byte_offset, u64& lo, u64& hi) const;

    [[nodiscard]] static bool lane_enabled(u32 code, u32 lane);
    [[nodiscard]] static s16 sign_extend_11(u32 value);
    [[nodiscard]] static float as_float(u32 value);
    [[nodiscard]] static u32 as_bits(float value);

    void begin_fmac();
    [[nodiscard]] u32 fmac_result(u32 lane, float value);
    void finish_fmac();
    void write_vf_lane(u32 reg, u32 lane, u32 value);
    void write_vi(u32 reg, u16 value);

    EeBus& bus_;
    GsCore& gs_;

    std::array<std::array<u32, 4>, 32> vf_{};
    std::array<u16, 16> vi_{};
    std::array<u32, 4> acc_{};

    u32 i_ = 0;
    u32 q_ = 0x3F800000u;
    u32 p_ = 0;
    u32 r_ = 0x3F800000u;
    u32 status_ = 0;
    u32 mac_ = 0;
    u32 clip_ = 0;

    u32 pc_ = 0;
    u32 branch_target_ = 0;
    u32 branch_countdown_ = 0;
    u32 end_countdown_ = 0;
    bool running_ = false;

    Vu1Stats stats_{};
};

} // namespace ps2
