#include "core/vu/vu1.h"

#include "core/gs/gs_core.h"
#include "core/memory/ee_bus.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>

namespace ps2 {
namespace {

constexpr u32 kVu1MicroBase = 0x11008000u;
constexpr u32 kVu1DataBase = 0x1100C000u;
constexpr u32 kVif1Itop = 0x10003CD0u;
constexpr u32 kVif1Top = 0x10003CE0u;

u32 ft(u32 code) { return (code >> 16) & 0x1Fu; }
u32 fs(u32 code) { return (code >> 11) & 0x1Fu; }
u32 fd(u32 code) { return (code >> 6) & 0x1Fu; }
u32 it(u32 code) { return ft(code) & 0xFu; }
u32 is(u32 code) { return fs(code) & 0xFu; }
u32 id(u32 code) { return fd(code) & 0xFu; }
u32 fsf(u32 code) { return (code >> 21) & 0x3u; }
u32 ftf(u32 code) { return (code >> 23) & 0x3u; }

s16 as_s16(u16 value) {
    return static_cast<s16>(value);
}

u16 wrap_vi(s32 value) {
    return static_cast<u16>(value);
}

s32 saturating_float_to_int(float value) {
    if (std::isnan(value)) return 0;
    if (value >= static_cast<float>(std::numeric_limits<s32>::max())) {
        return std::numeric_limits<s32>::max();
    }
    if (value <= static_cast<float>(std::numeric_limits<s32>::min())) {
        return std::numeric_limits<s32>::min();
    }
    return static_cast<s32>(value);
}

float eatan_approx(float value) {
    static constexpr float c[9] = {
        0.999999344348907f,
        -0.333298563957214f,
        0.199465364217758f,
        -0.139085337519646f,
        0.096420042216778f,
        -0.055909886956215f,
        0.021861229091883f,
        -0.004054057877511f,
        0.785398185253143f,
    };

    const float v2 = value * value;
    float power = value;
    float result = 0.0f;
    for (u32 i = 0; i < 8u; ++i) {
        result += c[i] * power;
        power *= v2;
    }
    return result + c[8];
}

float esin_approx(float value) {
    static constexpr float c[5] = {
        1.0f,
        -0.166666567325592f,
        0.008333025500178f,
        -0.000198074136279f,
        0.000002601886990f,
    };

    const float v2 = value * value;
    float power = value;
    float result = 0.0f;
    for (u32 i = 0; i < 5u; ++i) {
        result += c[i] * power;
        power *= v2;
    }
    return result;
}

float eexp_approx(float value) {
    static constexpr float c[6] = {
        0.249998688697815f,
        0.031257584691048f,
        0.002591371303424f,
        0.000171562001924f,
        0.000005430199963f,
        0.000000690600018f,
    };

    float poly = 1.0f;
    float power = value;
    for (u32 i = 0; i < 6u; ++i) {
        poly += c[i] * power;
        power *= value;
    }
    const float fourth = poly * poly * poly * poly;
    return fourth == 0.0f ? 0.0f : 1.0f / fourth;
}

} // namespace

Vu1::Vu1(EeBus& bus, GsCore& gs)
    : bus_(bus), gs_(gs) {
    reset();
}

void Vu1::reset() {
    vf_.fill({});
    vi_.fill(0);
    acc_.fill(0);
    vf_[0][3] = as_bits(1.0f);
    i_ = 0;
    q_ = as_bits(1.0f);
    p_ = 0;
    r_ = as_bits(1.0f);
    status_ = 0;
    mac_ = 0;
    clip_ = 0;
    pc_ = 0;
    branch_target_ = 0;
    branch_countdown_ = 0;
    end_countdown_ = 0;
    running_ = false;
    stats_ = {};
}

void Vu1::start(u32 address) {
    pc_ = ((address & 0x7FFu) * 8u) & 0x3FFFu;
    branch_countdown_ = 0;
    end_countdown_ = 0;
    running_ = true;
}

void Vu1::continue_run() {
    running_ = true;
}

bool Vu1::lane_enabled(u32 code, u32 lane) {
    return ((code >> (24u - (lane & 3u))) & 1u) != 0;
}

s16 Vu1::sign_extend_11(u32 value) {
    value &= 0x7FFu;
    if ((value & 0x400u) != 0) value |= 0xF800u;
    return static_cast<s16>(value);
}

float Vu1::as_float(u32 value) {
    return std::bit_cast<float>(value);
}

u32 Vu1::as_bits(float value) {
    return std::bit_cast<u32>(value);
}

void Vu1::begin_fmac() {
    mac_ = 0;
}

u32 Vu1::fmac_result(u32 lane, float value) {
    const u32 shift = 3u - (lane & 3u);
    const u32 bits = as_bits(value);
    const u32 sign = bits & 0x80000000u;
    const u32 exponent = (bits >> 23) & 0xFFu;

    if (sign != 0) {
        mac_ |= 0x0010u << shift;
    }

    if (value == 0.0f) {
        mac_ |= 0x0001u << shift;
        return bits;
    }

    if (exponent == 0u) {
        // VU FMAC flushes denormal results to signed zero while recording
        // both zero and underflow for the component.
        mac_ |= 0x0101u << shift;
        return sign;
    }

    if (exponent == 0xFFu) {
        // Clamp infinities/NaNs to the largest finite VU value and report
        // overflow. This matches the bootstrap-visible VU1 overflow mode.
        mac_ |= 0x1000u << shift;
        return sign | 0x7F7FFFFFu;
    }

    return bits;
}

void Vu1::finish_fmac() {
    u32 current = 0;
    if ((mac_ & 0x000Fu) != 0) current |= 0x1u;
    if ((mac_ & 0x00F0u) != 0) current |= 0x2u;
    if ((mac_ & 0x0F00u) != 0) current |= 0x4u;
    if ((mac_ & 0xF000u) != 0) current |= 0x8u;

    // Bits 0..3 are current Z/S/U/O flags. Bits 6..9 are sticky copies.
    // Preserve the remaining status control bits while accumulating sticky
    // conditions across FMAC operations.
    status_ = (status_ & 0xFF0u) | current | (current << 6);
}

void Vu1::write_vf_lane(u32 reg, u32 lane, u32 value) {
    if ((reg & 31u) == 0) return;
    vf_[reg & 31u][lane & 3u] = value;
}

void Vu1::write_vi(u32 reg, u16 value) {
    reg &= 0xFu;
    if (reg == 0) return;
    vi_[reg] = value;
}

bool Vu1::read_data_word(u32 byte_offset, u32& value) const {
    return bus_.read32(kVu1DataBase + (byte_offset & 0x3FFFu), value);
}

bool Vu1::write_data_word(u32 byte_offset, u32 value) {
    return bus_.write32(kVu1DataBase + (byte_offset & 0x3FFFu), value);
}

bool Vu1::read_data_qword(
    u32 byte_offset,
    u64& lo,
    u64& hi) const {
    u32 words[4]{};
    for (u32 lane = 0; lane < 4u; ++lane) {
        if (!read_data_word(byte_offset + lane * 4u, words[lane])) {
            return false;
        }
    }
    lo = static_cast<u64>(words[0]) |
         (static_cast<u64>(words[1]) << 32);
    hi = static_cast<u64>(words[2]) |
         (static_cast<u64>(words[3]) << 32);
    return true;
}

void Vu1::schedule_branch(u32 target) {
    branch_target_ = target & 0x3FFFu;
    branch_countdown_ = 2;
}

bool Vu1::xgkick(u16 qword_address, std::string& error) {
    error.clear();
    u32 byte_offset =
        (static_cast<u32>(qword_address) & 0x3FFu) * 16u;
    bool current_tag_eop = false;

    // XGKICK is asynchronous on hardware.  For the bootstrap interpreter we
    // serialize one packet immediately into PATH1.  This keeps ordering
    // deterministic without needing a separate GIF FIFO scheduler yet.
    for (u32 guard = 0; guard < 4096u; ++guard) {
        u64 lo = 0;
        u64 hi = 0;
        if (!read_data_qword(byte_offset, lo, hi)) {
            error = "VU1 XGKICK data read fault";
            return false;
        }

        if (!gs_.packet_active()) {
            current_tag_eop = ((lo >> 15) & 1u) != 0;
        }

        gs_.write_gif_qword(lo, hi);
        ++stats_.xgkick_qwords;
        byte_offset = (byte_offset + 16u) & 0x3FFFu;

        if (!gs_.packet_active() && current_tag_eop) {
            ++stats_.xgkicks;
            return true;
        }
    }

    // A malformed/no-EOP packet should not deadlock the whole BIOS bootstrap.
    // Keep this tolerant until PATH1 timing and GIF arbitration are modeled.
    ++stats_.xgkicks;
    return true;
}

bool Vu1::execute_upper(u32 code, std::string& error) {
    error.clear();
    const u32 op = code & 0x3Fu;
    const u32 s = fs(code);
    const u32 t = ft(code);
    const u32 d = fd(code);

    auto binary = [&](u32 dst, auto fn) {
        begin_fmac();
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float result = fn(
                as_float(vf_[s][lane]),
                as_float(vf_[t][lane]));
            write_vf_lane(dst, lane, fmac_result(lane, result));
        }
        finish_fmac();
    };
    auto scalar = [&](u32 dst, float value, auto fn) {
        begin_fmac();
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float result =
                fn(as_float(vf_[s][lane]), value);
            write_vf_lane(dst, lane, fmac_result(lane, result));
        }
        finish_fmac();
    };
    auto acc_binary = [&](auto fn) {
        begin_fmac();
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float result = fn(
                as_float(vf_[s][lane]),
                as_float(vf_[t][lane]));
            acc_[lane] = fmac_result(lane, result);
        }
        finish_fmac();
    };
    auto acc_scalar = [&](float value, auto fn) {
        begin_fmac();
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float result =
                fn(as_float(vf_[s][lane]), value);
            acc_[lane] = fmac_result(lane, result);
        }
        finish_fmac();
    };
    auto madd_binary = [&](u32 dst, bool subtract) {
        begin_fmac();
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float product =
                as_float(vf_[s][lane]) * as_float(vf_[t][lane]);
            const float result =
                as_float(acc_[lane]) + (subtract ? -product : product);
            write_vf_lane(dst, lane, fmac_result(lane, result));
        }
        finish_fmac();
    };
    auto madd_scalar = [&](u32 dst, float value, bool subtract) {
        begin_fmac();
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float product = as_float(vf_[s][lane]) * value;
            const float result =
                as_float(acc_[lane]) + (subtract ? -product : product);
            write_vf_lane(dst, lane, fmac_result(lane, result));
        }
        finish_fmac();
    };
    auto madda_binary = [&](bool subtract) {
        begin_fmac();
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float product =
                as_float(vf_[s][lane]) * as_float(vf_[t][lane]);
            const float result =
                as_float(acc_[lane]) + (subtract ? -product : product);
            acc_[lane] = fmac_result(lane, result);
        }
        finish_fmac();
    };
    auto madda_scalar = [&](float value, bool subtract) {
        begin_fmac();
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float product = as_float(vf_[s][lane]) * value;
            const float result =
                as_float(acc_[lane]) + (subtract ? -product : product);
            acc_[lane] = fmac_result(lane, result);
        }
        finish_fmac();
    };

    const auto add = [](float a, float b) { return a + b; };
    const auto sub = [](float a, float b) { return a - b; };
    const auto mul = [](float a, float b) { return a * b; };
    const auto vmax = [](float a, float b) { return std::fmax(a, b); };
    const auto vmin = [](float a, float b) { return std::fmin(a, b); };

    if (op <= 0x03u) {
        scalar(d, as_float(vf_[t][op & 3u]), add);
        return true;
    }
    if (op >= 0x04u && op <= 0x07u) {
        scalar(d, as_float(vf_[t][op & 3u]), sub);
        return true;
    }
    if (op >= 0x08u && op <= 0x0Bu) {
        madd_scalar(d, as_float(vf_[t][op & 3u]), false);
        return true;
    }
    if (op >= 0x0Cu && op <= 0x0Fu) {
        madd_scalar(d, as_float(vf_[t][op & 3u]), true);
        return true;
    }
    if (op >= 0x10u && op <= 0x13u) {
        scalar(d, as_float(vf_[t][op & 3u]), vmax);
        return true;
    }
    if (op >= 0x14u && op <= 0x17u) {
        scalar(d, as_float(vf_[t][op & 3u]), vmin);
        return true;
    }
    if (op >= 0x18u && op <= 0x1Bu) {
        scalar(d, as_float(vf_[t][op & 3u]), mul);
        return true;
    }

    switch (op) {
    case 0x1C: scalar(d, as_float(q_), mul); return true;
    case 0x1D: scalar(d, as_float(i_), vmax); return true;
    case 0x1E: scalar(d, as_float(i_), mul); return true;
    case 0x1F: scalar(d, as_float(i_), vmin); return true;
    case 0x20: scalar(d, as_float(q_), add); return true;
    case 0x21: madd_scalar(d, as_float(q_), false); return true;
    case 0x22: scalar(d, as_float(i_), add); return true;
    case 0x23: madd_scalar(d, as_float(i_), false); return true;
    case 0x24: scalar(d, as_float(q_), sub); return true;
    case 0x25: madd_scalar(d, as_float(q_), true); return true;
    case 0x26: scalar(d, as_float(i_), sub); return true;
    case 0x27: madd_scalar(d, as_float(i_), true); return true;
    case 0x28: binary(d, add); return true;
    case 0x29: madd_binary(d, false); return true;
    case 0x2A: binary(d, mul); return true;
    case 0x2B: binary(d, vmax); return true;
    case 0x2C: binary(d, sub); return true;
    case 0x2D: madd_binary(d, true); return true;
    case 0x2E: { // OPMSUB
        const float results[3] = {
            as_float(acc_[0]) -
                as_float(vf_[s][1]) * as_float(vf_[t][2]),
            as_float(acc_[1]) -
                as_float(vf_[s][2]) * as_float(vf_[t][0]),
            as_float(acc_[2]) -
                as_float(vf_[s][0]) * as_float(vf_[t][1]),
        };
        begin_fmac();
        for (u32 lane = 0; lane < 3u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            write_vf_lane(d, lane, fmac_result(lane, results[lane]));
        }
        finish_fmac();
        return true;
    }
    case 0x2F: binary(d, vmin); return true;
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F:
        return execute_upper_special(
            code, op, (code >> 6) & 0x1Fu, error);
    default:
        ++stats_.unsupported_upper;
        return true;
    }
}

bool Vu1::execute_upper_special(
    u32 code,
    u32 group,
    u32 index,
    std::string& error) {
    error.clear();
    const u32 s = fs(code);
    const u32 t = ft(code);

    auto acc_scalar_math = [&](float scalar, int kind) {
        begin_fmac();
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float lhs = as_float(vf_[s][lane]);
            float result = 0.0f;
            if (kind == 0) result = lhs + scalar;
            else if (kind == 1) result = lhs - scalar;
            else if (kind == 2) {
                result = as_float(acc_[lane]) + lhs * scalar;
            } else if (kind == 3) {
                result = as_float(acc_[lane]) - lhs * scalar;
            } else {
                result = lhs * scalar;
            }
            acc_[lane] = fmac_result(lane, result);
        }
        finish_fmac();
    };
    auto acc_vector_math = [&](int kind) {
        begin_fmac();
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float lhs = as_float(vf_[s][lane]);
            const float rhs = as_float(vf_[t][lane]);
            float result = 0.0f;
            if (kind == 0) result = lhs + rhs;
            else if (kind == 1) result = lhs - rhs;
            else if (kind == 2) {
                result = as_float(acc_[lane]) + lhs * rhs;
            } else if (kind == 3) {
                result = as_float(acc_[lane]) - lhs * rhs;
            } else {
                result = lhs * rhs;
            }
            acc_[lane] = fmac_result(lane, result);
        }
        finish_fmac();
    };
    auto convert_itof = [&](u32 shift) {
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float value =
                static_cast<float>(
                    static_cast<s32>(vf_[s][lane])) /
                static_cast<float>(1u << shift);
            write_vf_lane(t, lane, as_bits(value));
        }
    };
    auto convert_ftoi = [&](u32 shift) {
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            const float scaled =
                as_float(vf_[s][lane]) *
                static_cast<float>(1u << shift);
            write_vf_lane(
                t,
                lane,
                static_cast<u32>(saturating_float_to_int(scaled)));
        }
    };

    const u32 broadcast_lane = group - 0x3Cu;
    const float bc = as_float(vf_[t][broadcast_lane]);

    switch (index) {
    case 0x00: acc_scalar_math(bc, 0); return true;
    case 0x01: acc_scalar_math(bc, 1); return true;
    case 0x02: acc_scalar_math(bc, 2); return true;
    case 0x03: acc_scalar_math(bc, 3); return true;
    case 0x04:
        convert_itof(group == 0x3Cu ? 0u :
                     group == 0x3Du ? 4u :
                     group == 0x3Eu ? 12u : 15u);
        return true;
    case 0x05:
        convert_ftoi(group == 0x3Cu ? 0u :
                     group == 0x3Du ? 4u :
                     group == 0x3Eu ? 12u : 15u);
        return true;
    case 0x06: acc_scalar_math(bc, 4); return true;
    case 0x07:
        if (group == 0x3Cu) {
            acc_scalar_math(as_float(q_), 4);
            return true;
        }
        if (group == 0x3Du) {
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (!lane_enabled(code, lane)) continue;
                write_vf_lane(
                    t,
                    lane,
                    vf_[s][lane] & 0x7FFFFFFFu);
            }
            return true;
        }
        if (group == 0x3Eu) {
            acc_scalar_math(as_float(i_), 4);
            return true;
        }
        if (group == 0x3Fu) {
            const float w = std::fabs(as_float(vf_[t][3]));
            clip_ = (clip_ << 6) & 0xFFFFFFu;
            const float values[3] = {
                as_float(vf_[s][0]),
                as_float(vf_[s][1]),
                as_float(vf_[s][2]),
            };
            for (u32 lane = 0; lane < 3u; ++lane) {
                if (values[lane] > w) clip_ |= 1u << (lane * 2u);
                if (values[lane] < -w) {
                    clip_ |= 1u << (lane * 2u + 1u);
                }
            }
            return true;
        }
        break;
    case 0x08:
        if (group == 0x3Cu) acc_scalar_math(as_float(q_), 0);
        else if (group == 0x3Du) acc_scalar_math(as_float(q_), 2);
        else if (group == 0x3Eu) acc_scalar_math(as_float(i_), 0);
        else acc_scalar_math(as_float(i_), 2);
        return true;
    case 0x09:
        if (group == 0x3Cu) acc_scalar_math(as_float(q_), 1);
        else if (group == 0x3Du) acc_scalar_math(as_float(q_), 3);
        else if (group == 0x3Eu) acc_scalar_math(as_float(i_), 1);
        else acc_scalar_math(as_float(i_), 3);
        return true;
    case 0x0A:
        if (group == 0x3Cu) acc_vector_math(0);
        else if (group == 0x3Du) acc_vector_math(2);
        else if (group == 0x3Eu) acc_vector_math(4);
        else {
            ++stats_.unsupported_upper;
        }
        return true;
    case 0x0B:
        if (group == 0x3Cu) acc_vector_math(1);
        else if (group == 0x3Du) acc_vector_math(3);
        else if (group == 0x3Eu) {
            const float results[3] = {
                as_float(vf_[s][1]) * as_float(vf_[t][2]),
                as_float(vf_[s][2]) * as_float(vf_[t][0]),
                as_float(vf_[s][0]) * as_float(vf_[t][1]),
            };
            begin_fmac();
            for (u32 lane = 0; lane < 3u; ++lane) {
                if (!lane_enabled(code, lane)) continue;
                acc_[lane] = fmac_result(lane, results[lane]);
            }
            finish_fmac();
        } else {
            // VNOP
        }
        return true;
    default:
        ++stats_.unsupported_upper;
        return true;
    }

    ++stats_.unsupported_upper;
    return true;
}

bool Vu1::execute_lower(u32 code, std::string& error) {
    error.clear();
    const u32 op = code >> 25;

    auto read_vector = [&](u32 reg, u32 qword) -> bool {
        if (reg == 0) return true;
        const u32 base = (qword & 0x3FFu) * 16u;
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            u32 value = 0;
            if (!read_data_word(base + lane * 4u, value)) return false;
            write_vf_lane(reg, lane, value);
        }
        return true;
    };
    auto write_vector = [&](u32 reg, u32 qword) -> bool {
        const u32 base = (qword & 0x3FFu) * 16u;
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            if (!write_data_word(base + lane * 4u, vf_[reg][lane])) {
                return false;
            }
        }
        return true;
    };

    switch (op) {
    case 0x00: { // LQ
        const s16 imm = sign_extend_11(code);
        const u16 address =
            wrap_vi(as_s16(vi_[is(code)]) + imm);
        if (!read_vector(ft(code), address)) {
            error = "VU1 LQ data read fault";
            return false;
        }
        return true;
    }
    case 0x01: { // SQ
        const s16 imm = sign_extend_11(code);
        const u16 address =
            wrap_vi(as_s16(vi_[it(code)]) + imm);
        if (!write_vector(fs(code), address)) {
            error = "VU1 SQ data write fault";
            return false;
        }
        return true;
    }
    case 0x04: { // ILW
        const s16 imm = sign_extend_11(code);
        const u16 address =
            wrap_vi(as_s16(vi_[is(code)]) + imm);
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            u32 value = 0;
            if (!read_data_word(
                    static_cast<u32>(address) * 16u + lane * 4u,
                    value)) {
                error = "VU1 ILW data read fault";
                return false;
            }
            write_vi(it(code), static_cast<u16>(value));
        }
        return true;
    }
    case 0x05: { // ISW
        const s16 imm = sign_extend_11(code);
        const u16 address =
            wrap_vi(as_s16(vi_[is(code)]) + imm);
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            if (!write_data_word(
                    static_cast<u32>(address) * 16u + lane * 4u,
                    static_cast<u32>(vi_[it(code)]))) {
                error = "VU1 ISW data write fault";
                return false;
            }
        }
        return true;
    }
    case 0x08: { // IADDIU
        const u16 imm = static_cast<u16>(
            ((code >> 10) & 0x7800u) | (code & 0x7FFu));
        write_vi(
            it(code),
            wrap_vi(as_s16(vi_[is(code)]) + imm));
        return true;
    }
    case 0x09: { // ISUBIU
        const u16 imm = static_cast<u16>(
            ((code >> 10) & 0x7800u) | (code & 0x7FFu));
        write_vi(
            it(code),
            wrap_vi(as_s16(vi_[is(code)]) - imm));
        return true;
    }
    case 0x10: // FCEQ
        write_vi(1, (clip_ & 0xFFFFFFu) == (code & 0xFFFFFFu));
        return true;
    case 0x11: // FCSET
        clip_ = code & 0xFFFFFFu;
        return true;
    case 0x12: // FCAND
        write_vi(
            1,
            ((clip_ & 0xFFFFFFu) & (code & 0xFFFFFFu)) != 0);
        return true;
    case 0x13: // FCOR
        write_vi(
            1,
            (((clip_ & 0xFFFFFFu) | (code & 0xFFFFFFu)) ==
             0xFFFFFFu));
        return true;
    case 0x14: { // FSEQ
        const u16 imm = static_cast<u16>(
            (((code >> 21) & 1u) << 11) | (code & 0x7FFu));
        write_vi(it(code), (status_ & 0xFFFu) == imm);
        return true;
    }
    case 0x15: { // FSSET
        const u16 imm = static_cast<u16>(
            (((code >> 21) & 1u) << 11) | (code & 0x7FFu));
        status_ = (status_ & 0x3Fu) | (imm & 0xFC0u);
        return true;
    }
    case 0x16: { // FSAND
        const u16 imm = static_cast<u16>(
            (((code >> 21) & 1u) << 11) | (code & 0x7FFu));
        write_vi(it(code), static_cast<u16>((status_ & 0xFFFu) & imm));
        return true;
    }
    case 0x17: { // FSOR
        const u16 imm = static_cast<u16>(
            (((code >> 21) & 1u) << 11) | (code & 0x7FFu));
        write_vi(it(code), static_cast<u16>((status_ & 0xFFFu) | imm));
        return true;
    }
    case 0x18: // FMEQ
        write_vi(
            it(code),
            (mac_ & 0xFFFFu) == vi_[is(code)]);
        return true;
    case 0x1A: // FMAND
        write_vi(
            it(code),
            static_cast<u16>((mac_ & 0xFFFFu) & vi_[is(code)]));
        return true;
    case 0x1B: // FMOR
        write_vi(
            it(code),
            static_cast<u16>((mac_ & 0xFFFFu) | vi_[is(code)]));
        return true;
    case 0x1C: // FCGET
        write_vi(it(code), static_cast<u16>(clip_ & 0x0FFFu));
        return true;
    case 0x20: // B
        schedule_branch(
            pc_ + static_cast<s32>(sign_extend_11(code)) * 8);
        return true;
    case 0x21: // BAL
        write_vi(it(code), static_cast<u16>((pc_ + 8u) / 8u));
        schedule_branch(
            pc_ + static_cast<s32>(sign_extend_11(code)) * 8);
        return true;
    case 0x24: // JR
        schedule_branch(static_cast<u32>(vi_[is(code)]) * 8u);
        return true;
    case 0x25: { // JALR
        const u32 target = static_cast<u32>(vi_[is(code)]) * 8u;
        write_vi(it(code), static_cast<u16>((pc_ + 8u) / 8u));
        schedule_branch(target);
        return true;
    }
    case 0x28: // IBEQ
        if (vi_[it(code)] == vi_[is(code)]) {
            schedule_branch(
                pc_ + static_cast<s32>(sign_extend_11(code)) * 8);
        }
        return true;
    case 0x29: // IBNE
        if (vi_[it(code)] != vi_[is(code)]) {
            schedule_branch(
                pc_ + static_cast<s32>(sign_extend_11(code)) * 8);
        }
        return true;
    case 0x2C: // IBLTZ
        if (as_s16(vi_[is(code)]) < 0) {
            schedule_branch(
                pc_ + static_cast<s32>(sign_extend_11(code)) * 8);
        }
        return true;
    case 0x2D: // IBGTZ
        if (as_s16(vi_[is(code)]) > 0) {
            schedule_branch(
                pc_ + static_cast<s32>(sign_extend_11(code)) * 8);
        }
        return true;
    case 0x2E: // IBLEZ
        if (as_s16(vi_[is(code)]) <= 0) {
            schedule_branch(
                pc_ + static_cast<s32>(sign_extend_11(code)) * 8);
        }
        return true;
    case 0x2F: // IBGEZ
        if (as_s16(vi_[is(code)]) >= 0) {
            schedule_branch(
                pc_ + static_cast<s32>(sign_extend_11(code)) * 8);
        }
        return true;
    case 0x40: {
        const u32 sub = code & 0x3Fu;
        if (sub == 0x30u) {
            write_vi(
                id(code),
                wrap_vi(
                    as_s16(vi_[is(code)]) +
                    as_s16(vi_[it(code)])));
            return true;
        }
        if (sub == 0x31u) {
            write_vi(
                id(code),
                wrap_vi(
                    as_s16(vi_[is(code)]) -
                    as_s16(vi_[it(code)])));
            return true;
        }
        if (sub == 0x32u) {
            s16 imm = static_cast<s16>((code >> 6) & 0x1Fu);
            if ((imm & 0x10) != 0) imm |= static_cast<s16>(0xFFF0);
            write_vi(
                it(code),
                wrap_vi(as_s16(vi_[is(code)]) + imm));
            return true;
        }
        if (sub == 0x34u) {
            write_vi(id(code), vi_[is(code)] & vi_[it(code)]);
            return true;
        }
        if (sub == 0x35u) {
            write_vi(id(code), vi_[is(code)] | vi_[it(code)]);
            return true;
        }
        if (sub >= 0x3Cu) {
            return execute_lower_special(
                code,
                sub,
                (code >> 6) & 0x1Fu,
                error);
        }
        ++stats_.unsupported_lower;
        return true;
    }
    default:
        ++stats_.unsupported_lower;
        return true;
    }
}

bool Vu1::execute_lower_special(
    u32 code,
    u32 group,
    u32 index,
    std::string& error) {
    error.clear();

    auto read_vector = [&](u32 reg, u32 qword) -> bool {
        if (reg == 0) return true;
        const u32 base = (qword & 0x3FFu) * 16u;
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            u32 value = 0;
            if (!read_data_word(base + lane * 4u, value)) return false;
            write_vf_lane(reg, lane, value);
        }
        return true;
    };
    auto write_vector = [&](u32 reg, u32 qword) -> bool {
        const u32 base = (qword & 0x3FFu) * 16u;
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (!lane_enabled(code, lane)) continue;
            if (!write_data_word(base + lane * 4u, vf_[reg][lane])) {
                return false;
            }
        }
        return true;
    };
    auto set_p = [&](float value) {
        p_ = as_bits(value);
    };
    auto vector_sum_squares = [&]() {
        const u32 s = fs(code);
        const float x = as_float(vf_[s][0]);
        const float y = as_float(vf_[s][1]);
        const float z = as_float(vf_[s][2]);
        return x * x + y * y + z * z;
    };
    auto random_to_ft = [&]() {
        if (ft(code) == 0u) return;
        for (u32 lane = 0; lane < 4u; ++lane) {
            if (lane_enabled(code, lane)) {
                write_vf_lane(ft(code), lane, r_);
            }
        }
    };

    if (group == 0x3Cu) {
        switch (index) {
        case 0x0C: // MOVE
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (lane_enabled(code, lane)) {
                    write_vf_lane(ft(code), lane, vf_[fs(code)][lane]);
                }
            }
            return true;
        case 0x0D: { // LQI
            const u16 address = vi_[is(code)];
            if (!read_vector(ft(code), address)) {
                error = "VU1 LQI data read fault";
                return false;
            }
            write_vi(is(code), static_cast<u16>(address + 1u));
            return true;
        }
        case 0x0E: { // DIV
            const float numerator =
                as_float(vf_[fs(code)][fsf(code)]);
            const float denominator =
                as_float(vf_[ft(code)][ftf(code)]);
            if (denominator == 0.0f) {
                q_ = as_bits(
                    std::copysign(
                        std::numeric_limits<float>::max(),
                        numerator * denominator));
            } else {
                q_ = as_bits(numerator / denominator);
            }
            return true;
        }
        case 0x0F: // MTIR
            write_vi(
                it(code),
                static_cast<u16>(vf_[fs(code)][fsf(code)]));
            return true;
        case 0x10: // RNEXT
            if (ft(code) != 0u) {
                const u32 x = (r_ >> 4) & 1u;
                const u32 y = (r_ >> 22) & 1u;
                r_ <<= 1u;
                r_ ^= x ^ y;
                r_ = (r_ & 0x007FFFFFu) | 0x3F800000u;
                random_to_ft();
            }
            return true;
        case 0x1C: // ESADD
            set_p(vector_sum_squares());
            return true;
        case 0x1D: { // EATANxy
            const u32 s = fs(code);
            const float x = as_float(vf_[s][0]);
            const float y = as_float(vf_[s][1]);
            set_p(x == 0.0f ? 0.0f : eatan_approx(y / x));
            return true;
        }
        case 0x1E: { // ESQRT
            float value = as_float(vf_[fs(code)][fsf(code)]);
            if (value >= 0.0f) value = std::sqrt(value);
            set_p(value);
            return true;
        }
        case 0x1F: // ESIN
            set_p(esin_approx(
                as_float(vf_[fs(code)][fsf(code)])));
            return true;
        case 0x19: // MFP
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (lane_enabled(code, lane)) {
                    write_vf_lane(ft(code), lane, p_);
                }
            }
            return true;
        case 0x1A: { // XTOP
            u32 value = 0;
            if (!bus_.read32(kVif1Top, value)) {
                error = "VU1 XTOP VIF1 TOP read fault";
                return false;
            }
            write_vi(it(code), static_cast<u16>(value));
            return true;
        }
        case 0x1B: // XGKICK
            return xgkick(vi_[is(code)], error);
        default:
            ++stats_.unsupported_lower;
            return true;
        }
    }

    if (group == 0x3Du) {
        switch (index) {
        case 0x0C: { // MR32
            if (ft(code) == 0) return true;
            const auto source = vf_[fs(code)];
            if (lane_enabled(code, 0)) write_vf_lane(ft(code), 0, source[1]);
            if (lane_enabled(code, 1)) write_vf_lane(ft(code), 1, source[2]);
            if (lane_enabled(code, 2)) write_vf_lane(ft(code), 2, source[3]);
            if (lane_enabled(code, 3)) write_vf_lane(ft(code), 3, source[0]);
            return true;
        }
        case 0x0D: { // SQI
            const u16 address = vi_[it(code)];
            if (!write_vector(fs(code), address)) {
                error = "VU1 SQI data write fault";
                return false;
            }
            write_vi(it(code), static_cast<u16>(address + 1u));
            return true;
        }
        case 0x0E: { // SQRT
            const float value =
                as_float(vf_[ft(code)][ftf(code)]);
            q_ = as_bits(std::sqrt(std::fabs(value)));
            return true;
        }
        case 0x0F: { // MFIR
            const s32 value = static_cast<s32>(as_s16(vi_[is(code)]));
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (lane_enabled(code, lane)) {
                    write_vf_lane(
                        ft(code), lane, static_cast<u32>(value));
                }
            }
            return true;
        }
        case 0x10: // RGET
            random_to_ft();
            return true;
        case 0x1C: { // ERSADD
            float value = vector_sum_squares();
            if (value != 0.0f) value = 1.0f / value;
            set_p(value);
            return true;
        }
        case 0x1D: { // EATANxz
            const u32 s = fs(code);
            const float x = as_float(vf_[s][0]);
            const float z = as_float(vf_[s][2]);
            set_p(x == 0.0f ? 0.0f : eatan_approx(z / x));
            return true;
        }
        case 0x1E: { // ERSQRT
            float value = as_float(vf_[fs(code)][fsf(code)]);
            if (value >= 0.0f) {
                value = std::sqrt(value);
                if (value != 0.0f) value = 1.0f / value;
            }
            set_p(value);
            return true;
        }
        case 0x1F: // EATAN
            set_p(eatan_approx(
                as_float(vf_[fs(code)][fsf(code)])));
            return true;
        case 0x1A: { // XITOP
            u32 value = 0;
            if (!bus_.read32(kVif1Itop, value)) {
                error = "VU1 XITOP VIF1 ITOP read fault";
                return false;
            }
            write_vi(it(code), static_cast<u16>(value));
            return true;
        }
        default:
            ++stats_.unsupported_lower;
            return true;
        }
    }

    if (group == 0x3Eu) {
        switch (index) {
        case 0x0D: { // LQD
            const u16 address = static_cast<u16>(vi_[is(code)] - 1u);
            write_vi(is(code), address);
            if (!read_vector(ft(code), address)) {
                error = "VU1 LQD data read fault";
                return false;
            }
            return true;
        }
        case 0x0E: { // RSQRT
            const float numerator =
                as_float(vf_[fs(code)][fsf(code)]);
            const float denominator =
                std::sqrt(std::fabs(
                    as_float(vf_[ft(code)][ftf(code)])));
            if (denominator == 0.0f) {
                q_ = as_bits(
                    std::copysign(
                        std::numeric_limits<float>::max(),
                        numerator));
            } else {
                q_ = as_bits(numerator / denominator);
            }
            return true;
        }
        case 0x0F: { // ILWR
            const u32 base =
                static_cast<u32>(vi_[is(code)] & 0x3FFu) * 16u;
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (!lane_enabled(code, lane)) continue;
                u32 value = 0;
                if (!read_data_word(base + lane * 4u, value)) {
                    error = "VU1 ILWR data read fault";
                    return false;
                }
                write_vi(it(code), static_cast<u16>(value));
            }
            return true;
        }
        case 0x10: // RINIT
            r_ =
                0x3F800000u |
                (vf_[fs(code)][fsf(code)] & 0x007FFFFFu);
            return true;
        case 0x1C: { // ELENG
            float value = vector_sum_squares();
            if (value >= 0.0f) value = std::sqrt(value);
            set_p(value);
            return true;
        }
        case 0x1D: { // ESUM
            const u32 s = fs(code);
            set_p(
                as_float(vf_[s][0]) +
                as_float(vf_[s][1]) +
                as_float(vf_[s][2]) +
                as_float(vf_[s][3]));
            return true;
        }
        case 0x1E: { // ERCPR
            float value = as_float(vf_[fs(code)][fsf(code)]);
            if (value != 0.0f) value = 1.0f / value;
            set_p(value);
            return true;
        }
        case 0x1F: // EEXP
            set_p(eexp_approx(
                as_float(vf_[fs(code)][fsf(code)])));
            return true;
        default:
            ++stats_.unsupported_lower;
            return true;
        }
    }

    if (group == 0x3Fu) {
        switch (index) {
        case 0x0D: { // SQD
            const u16 address = static_cast<u16>(vi_[it(code)] - 1u);
            write_vi(it(code), address);
            if (!write_vector(fs(code), address)) {
                error = "VU1 SQD data write fault";
                return false;
            }
            return true;
        }
        case 0x10: // RXOR
            r_ =
                0x3F800000u |
                ((r_ ^ vf_[fs(code)][fsf(code)]) & 0x007FFFFFu);
            return true;
        case 0x1C: { // ERLENG
            float value = vector_sum_squares();
            if (value >= 0.0f) {
                value = std::sqrt(value);
                if (value != 0.0f) value = 1.0f / value;
            }
            set_p(value);
            return true;
        }
        case 0x0E: // WAITQ
        case 0x1A: // WAITP
            return true;
        case 0x0F: { // ISWR
            const u32 base =
                static_cast<u32>(vi_[is(code)] & 0x3FFu) * 16u;
            for (u32 lane = 0; lane < 4u; ++lane) {
                if (!lane_enabled(code, lane)) continue;
                if (!write_data_word(
                        base + lane * 4u,
                        static_cast<u32>(vi_[it(code)]))) {
                    error = "VU1 ISWR data write fault";
                    return false;
                }
            }
            return true;
        }
        default:
            ++stats_.unsupported_lower;
            return true;
        }
    }

    ++stats_.unsupported_lower;
    return true;
}

bool Vu1::step(std::string& error) {
    error.clear();
    if (!running_) return true;

    u32 lower = 0;
    u32 upper = 0;
    if (!bus_.read32(kVu1MicroBase + (pc_ & 0x3FFFu), lower) ||
        !bus_.read32(
            kVu1MicroBase + ((pc_ + 4u) & 0x3FFFu),
            upper)) {
        error = "VU1 microcode fetch fault";
        running_ = false;
        return false;
    }

    pc_ = (pc_ + 8u) & 0x3FFFu;

    if ((upper & 0x40000000u) != 0 && end_countdown_ == 0) {
        // E executes one delay-slot instruction before stopping.
        end_countdown_ = 2;
    }

    if (!execute_upper(upper, error)) {
        running_ = false;
        return false;
    }

    if ((upper & 0x80000000u) != 0) {
        // I flag turns the lower 32 bits into the immediate float register.
        i_ = lower;
    } else if (!execute_lower(lower, error)) {
        running_ = false;
        return false;
    }

    ++stats_.instructions;
    vi_[0] = 0;
    vf_[0][0] = 0;
    vf_[0][1] = 0;
    vf_[0][2] = 0;
    vf_[0][3] = as_bits(1.0f);

    if (branch_countdown_ != 0) {
        --branch_countdown_;
        if (branch_countdown_ == 0) {
            pc_ = branch_target_ & 0x3FFFu;
        }
    }

    if (end_countdown_ != 0) {
        --end_countdown_;
        if (end_countdown_ == 0) {
            running_ = false;
        }
    }

    return true;
}

u64 Vu1::run(u64 instruction_budget, std::string& error) {
    error.clear();
    u64 executed = 0;
    while (running_ && executed < instruction_budget) {
        if (!step(error)) break;
        ++executed;
    }
    return executed;
}

} // namespace ps2
