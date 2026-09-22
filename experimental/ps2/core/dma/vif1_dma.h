#pragma once

#include "common/types.h"

#include <array>
#include <string>

namespace ps2 {

class EeBus;
class GsCore;
class GsPrivileged;
class Vu1;

class Vif1Dma {
public:
    struct RecentTag {
        u32 address = 0;
        u32 tag0 = 0;
        u32 tag1 = 0;
        u32 next_tadr = 0;
        u32 madr = 0;
    };

    void reset();
    void attach_vu1(Vu1& vu1) { vu1_ = &vu1; }
    [[nodiscard]] const std::array<RecentTag, 32>& recent_tags() const {
        return recent_tags_;
    }
    [[nodiscard]] u32 recent_tag_next() const { return recent_tag_next_; }
    [[nodiscard]] u32 recent_tag_count() const { return recent_tag_count_; }

    [[nodiscard]] bool service(
        EeBus& bus,
        GsCore& gs,
        const GsPrivileged& privileged,
        std::string& error);

private:
    enum class Payload {
        None,
        Mask,
        Row,
        Col,
        Mpg,
        Direct,
        Unpack,
        WaitVu,
    };

    [[nodiscard]] bool complete(EeBus& bus, u32 chcr);
    [[nodiscard]] bool service_forward(
        EeBus& bus,
        GsCore& gs,
        u32 chcr,
        std::string& error);
    [[nodiscard]] bool service_reverse(
        EeBus& bus,
        GsCore& gs,
        const GsPrivileged& privileged,
        u32 chcr,
        std::string& error);

    [[nodiscard]] bool consume_qword(
        EeBus& bus,
        GsCore& gs,
        u64 lo,
        u64 hi,
        std::string& error);
    [[nodiscard]] bool consume_word(
        EeBus& bus,
        GsCore& gs,
        u32 word,
        std::string& error);
    [[nodiscard]] bool begin_command(
        EeBus& bus,
        u32 word,
        std::string& error);
    [[nodiscard]] bool consume_payload_word(
        EeBus& bus,
        GsCore& gs,
        u32 word,
        std::string& error);
    [[nodiscard]] bool emit_unpack_vector(
        EeBus& bus,
        const std::array<u32, 4>& vector,
        bool filling,
        std::string& error);
    [[nodiscard]] bool finish_command(EeBus& bus);
    void reset_unpack();

    bool end_after_qwc_ = false;

    Payload payload_ = Payload::None;
    bool command_irq_pending_ = false;
    u32 active_code_ = 0;
    u32 payload_index_ = 0;
    std::array<u32, 4> deferred_words_{};
    u32 deferred_word_count_ = 0;
    u32 deferred_word_index_ = 0;

    u32 cycle_ = 0;
    u32 mode_ = 0;
    u32 mask_ = 0;
    std::array<u32, 4> row_{};
    std::array<u32, 4> col_{};
    u32 base_ = 0;
    u32 offset_ = 0;
    u32 tops_ = 0;
    u32 top_ = 0;
    u32 itop_ = 0;
    bool double_buffer_ = false;

    std::array<RecentTag, 32> recent_tags_{};
    u32 recent_tag_next_ = 0;
    u32 recent_tag_count_ = 0;

    u32 mpg_address_ = 0;
    u32 mpg_words_remaining_ = 0;

    std::array<u32, 4> direct_words_{};
    u32 direct_word_count_ = 0;
    u32 direct_words_remaining_ = 0;

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

    Vu1* vu1_ = nullptr;
};

} // namespace ps2
