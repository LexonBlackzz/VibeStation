#pragma once

#include "common/types.h"

#include <array>
#include <string>

namespace ps2 {

class EeBus;

class Vif0Dma {
public:
    void reset();

    [[nodiscard]] bool service(
        EeBus& bus,
        std::string& error);

private:
    enum class Payload {
        None,
        Mask,
        Row,
        Col,
        Mpg,
        Unpack,
    };

    [[nodiscard]] bool complete(EeBus& bus, u32 chcr);
    [[nodiscard]] bool consume_qword(
        EeBus& bus,
        u64 lo,
        u64 hi,
        std::string& error);
    [[nodiscard]] bool consume_word(
        EeBus& bus,
        u32 word,
        std::string& error);
    [[nodiscard]] bool begin_command(
        EeBus& bus,
        u32 word,
        std::string& error);
    [[nodiscard]] bool consume_payload_word(
        EeBus& bus,
        u32 word,
        std::string& error);
    [[nodiscard]] bool emit_unpack_vector(
        EeBus& bus,
        const std::array<u32, 4>& vector,
        bool filling,
        std::string& error);
    [[nodiscard]] bool finish_command(EeBus& bus);
    void reset_unpack();
    void set_vps(EeBus& bus, u32 vps);
    void set_fqc(EeBus& bus, u32 qwc);

    bool end_after_qwc_ = false;
    Payload payload_ = Payload::None;
    bool command_irq_pending_ = false;
    u32 payload_index_ = 0;

    u32 cycle_ = 0;
    u32 mode_ = 0;
    u32 mask_ = 0;
    std::array<u32, 4> row_{};
    std::array<u32, 4> col_{};
    u32 itop_ = 0;

    u32 mpg_address_ = 0;
    u32 mpg_words_remaining_ = 0;

    u32 unpack_dest_ = 0;
    u32 unpack_target_remaining_ = 0;
    u32 unpack_source_remaining_ = 0;
    u32 unpack_cycle_pos_ = 0;
    u32 unpack_vn_ = 0;
    u32 unpack_vl_ = 0;
    bool unpack_unsigned_ = false;
    bool unpack_masked_ = false;
    u64 unpack_bits_ = 0;
    u32 unpack_bit_count_ = 0;
    std::array<u32, 4> unpack_vector_{};
    u32 unpack_component_ = 0;
};

} // namespace ps2
