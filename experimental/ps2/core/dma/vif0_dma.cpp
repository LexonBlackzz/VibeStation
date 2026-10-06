#include "core/dma/vif0_dma.h"
#include "core/ee/ee_cpu.h"

#include "core/memory/ee_bus.h"
#include "core/vu/vu1.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u32 kDmacCtrl = 0x1000E000u;
constexpr u32 kVif0Chcr = 0x10008000u;
constexpr u32 kVif0Madr = 0x10008010u;
constexpr u32 kVif0Qwc = 0x10008020u;
constexpr u32 kVif0Tadr = 0x10008030u;
constexpr u32 kVif0Asr0 = 0x10008040u;
constexpr u32 kVif0Asr1 = 0x10008050u;

constexpr u32 kVif0Stat = 0x10003800u;
constexpr u32 kVif0Mark = 0x10003830u;
constexpr u32 kVif0Cycle = 0x10003840u;
constexpr u32 kVif0Mode = 0x10003850u;
constexpr u32 kVif0Num = 0x10003860u;
constexpr u32 kVif0Mask = 0x10003870u;
constexpr u32 kVif0Code = 0x10003880u;
constexpr u32 kVif0Itops = 0x10003890u;
constexpr u32 kVif0Itop = 0x100038D0u;
constexpr u32 kVif0Row = 0x10003900u;
constexpr u32 kVif0Col = 0x10003940u;

constexpr u32 kVu0Micro = 0x11000000u;
constexpr u32 kVu0Data = 0x11004000u;

constexpr u32 kChcrDir = 1u << 0;
constexpr u32 kChcrTte = 1u << 6;
constexpr u32 kChcrTie = 1u << 7;
constexpr u32 kChcrStr = 1u << 8;

constexpr u32 kModeNormal = 0;
constexpr u32 kModeChain = 1;

constexpr u32 kVpsMask = 0x3u;
constexpr u32 kMrk = 1u << 6;
constexpr u32 kInt = 1u << 11;
constexpr u32 kEr1 = 1u << 13;
constexpr u32 kFqcMask = 0xFu << 24;

u32 apply_spr(u32 address, bool spr) {
    if (!spr) return address & 0x7FFFFFF0u;
    return 0x70000000u | (address & 0x3FF0u);
}

u32 cycle_length(u32 value) {
    return value == 0 ? 256u : value;
}

u32 extend_element(u32 value, u32 bits, bool is_unsigned) {
    if (bits >= 32u || is_unsigned) return value;
    const u32 sign = 1u << (bits - 1u);
    const u32 mask = (1u << bits) - 1u;
    value &= mask;
    if ((value & sign) != 0) value |= ~mask;
    return value;
}

bool supported_unpack_format(u32 format) {
    switch (format) {
    case 0x0:
    case 0x1:
    case 0x2:
    case 0x4:
    case 0x5:
    case 0x6:
    case 0x8:
    case 0x9:
    case 0xA:
    case 0xC:
    case 0xD:
    case 0xE:
    case 0xF:
        return true;
    default:
        return false;
    }
}

} // namespace

void Vif0Dma::reset() {
    end_after_qwc_ = false;
    payload_ = Payload::None;
    command_irq_pending_ = false;
    payload_index_ = 0;
    deferred_words_.fill(0);
    deferred_word_count_ = 0;
    deferred_word_index_ = 0;
    cycle_ = 0;
    mode_ = 0;
    mask_ = 0;
    row_.fill(0);
    col_.fill(0);
    itop_ = 0;
    mpg_address_ = 0;
    mpg_words_remaining_ = 0;
    reset_unpack();
}

void Vif0Dma::reset_unpack() {
    unpack_dest_ = 0;
    unpack_target_remaining_ = 0;
    unpack_source_remaining_ = 0;
    unpack_cycle_pos_ = 0;
    unpack_vn_ = 0;
    unpack_vl_ = 0;
    unpack_unsigned_ = false;
    unpack_masked_ = false;
    unpack_bits_ = 0;
    unpack_bit_count_ = 0;
    unpack_vector_.fill(0);
    unpack_component_ = 0;
}

void Vif0Dma::set_vps(EeBus& bus, u32 vps) {
    u32 stat = 0;
    if (!bus.read32(kVif0Stat, stat)) return;
    stat = (stat & ~kVpsMask) | (vps & kVpsMask);
    bus.write32(kVif0Stat, stat);
}

void Vif0Dma::set_fqc(EeBus& bus, u32 qwc) {
    u32 stat = 0;
    if (!bus.read32(kVif0Stat, stat)) return;
    stat =
        (stat & ~kFqcMask) |
        ((std::min(qwc, 8u) & 0xFu) << 24);
    bus.write32(kVif0Stat, stat);
}

bool Vif0Dma::complete(EeBus& bus, u32 chcr) {
    end_after_qwc_ = false;
    set_vps(bus, 0u);
    set_fqc(bus, 0u);
    if (!bus.write32(kVif0Qwc, 0u)) return false;
    if (!bus.write32(kVif0Chcr, chcr & ~kChcrStr)) return false;
    bus.raise_dmac(0);
    return true;
}

bool Vif0Dma::finish_command(EeBus& bus) {
    payload_ = Payload::None;
    payload_index_ = 0;
    set_vps(bus, 0u);

    if (command_irq_pending_) {
        u32 stat = 0;
        if (bus.read32(kVif0Stat, stat)) {
            bus.write32(kVif0Stat, stat | kInt);
        }
        bus.raise_intc(4);
    }
    command_irq_pending_ = false;
    return true;
}

bool Vif0Dma::begin_command(
    EeBus& bus,
    u32 word,
    std::string& error) {
    error.clear();
    command_irq_pending_ = (word & 0x80000000u) != 0;
    const u32 command = (word >> 24) & 0x7Fu;
    const u32 num_field = (word >> 16) & 0xFFu;
    const u32 immediate = word & 0xFFFFu;

    if (!bus.write32(kVif0Code, word)) {
        error = "failed to mirror VIF0 CODE";
        return false;
    }
    set_vps(bus, 2u);

    switch (command) {
    case 0x00: // NOP
        return finish_command(bus);

    case 0x01: // STCYCL
        cycle_ = immediate;
        if (!bus.write32(kVif0Cycle, cycle_)) {
            error = "failed to write VIF0 CYCLE";
            return false;
        }
        return finish_command(bus);

    case 0x04: // ITOP
        itop_ = immediate & 0xFFu;
        if (!bus.write32(kVif0Itop, itop_) ||
            !bus.write32(kVif0Itops, itop_)) {
            error = "failed to write VIF0 ITOP";
            return false;
        }
        return finish_command(bus);

    case 0x05: // STMOD
        mode_ = immediate & 0x3u;
        if (!bus.write32(kVif0Mode, mode_)) {
            error = "failed to write VIF0 MODE";
            return false;
        }
        return finish_command(bus);

    case 0x07: { // MARK
        if (!bus.write32(kVif0Mark, immediate)) {
            error = "failed to write VIF0 MARK";
            return false;
        }
        u32 stat = 0;
        if (bus.read32(kVif0Stat, stat)) {
            bus.write32(kVif0Stat, stat | kMrk);
        }
        return finish_command(bus);
    }

    case 0x10: // FLUSHE
    case 0x11: // FLUSH
    case 0x13: // FLUSHA
        if (vu0_ != nullptr && vu0_->running()) {
            payload_ = Payload::WaitVu;
            set_vps(bus, 1u);
            return true;
        }
        return finish_command(bus);

    case 0x14: // MSCAL
    case 0x15: // MSCALF
        if (vu0_ != nullptr) {
            // Macro registers need copying only when micro execution starts,
            // not on every EE step while VIF0 DMA remains armed.
            if (ee_ != nullptr && !vu0_->running()) {
                ee_->sync_vu0_to_micro();
            }
            vu0_->start(immediate & 0x1FFu);
        }
        return finish_command(bus);

    case 0x17: // MSCNT
        if (vu0_ != nullptr) {
            if (ee_ != nullptr && !vu0_->running()) {
                ee_->sync_vu0_to_micro();
            }
            vu0_->continue_run();
        }
        return finish_command(bus);

    case 0x20: // STMASK
        payload_ = Payload::Mask;
        payload_index_ = 0;
        set_vps(bus, 3u);
        return true;

    case 0x30: // STROW
        payload_ = Payload::Row;
        payload_index_ = 0;
        set_vps(bus, 3u);
        return true;

    case 0x31: // STCOL
        payload_ = Payload::Col;
        payload_index_ = 0;
        set_vps(bus, 3u);
        return true;

    case 0x4A: { // MPG
        const u32 instructions = num_field == 0 ? 256u : num_field;
        mpg_address_ = (immediate * 8u) & 0xFFFu;
        mpg_words_remaining_ = instructions * 2u;
        payload_ = Payload::Mpg;
        payload_index_ = 0;
        set_vps(bus, 3u);
        if (!bus.write32(kVif0Num, num_field)) {
            error = "failed to write VIF0 MPG NUM";
            return false;
        }
        return true;
    }

    default:
        if (command >= 0x60u && command <= 0x7Fu) {
            const u32 format = command & 0x0Fu;
            if (!supported_unpack_format(format)) {
                u32 stat = 0;
                if (bus.read32(kVif0Stat, stat)) {
                    bus.write32(kVif0Stat, stat | kEr1);
                }
                return finish_command(bus);
            }

            reset_unpack();
            unpack_vn_ = (format >> 2) & 0x3u;
            unpack_vl_ = format & 0x3u;
            if (unpack_vl_ == 3u && unpack_vn_ != 3u) {
                u32 stat = 0;
                if (bus.read32(kVif0Stat, stat)) {
                    bus.write32(kVif0Stat, stat | kEr1);
                }
                return finish_command(bus);
            }

            unpack_unsigned_ = (immediate & 0x4000u) != 0;
            unpack_masked_ = (command & 0x10u) != 0;
            unpack_target_remaining_ =
                num_field == 0 ? 256u : num_field;

            const u32 cl = cycle_length(cycle_ & 0xFFu);
            const u32 wl = cycle_length((cycle_ >> 8) & 0xFFu);
            if (cl < wl) {
                unpack_source_remaining_ =
                    (unpack_target_remaining_ / wl) * cl +
                    std::min(unpack_target_remaining_ % wl, cl);
            } else {
                unpack_source_remaining_ = unpack_target_remaining_;
            }

            unpack_dest_ = immediate & 0xFFu;
            payload_ = Payload::Unpack;
            set_vps(bus, 3u);
            if (!bus.write32(kVif0Num, num_field)) {
                error = "failed to write VIF0 UNPACK NUM";
                return false;
            }

            if (unpack_source_remaining_ == 0) {
                while (unpack_target_remaining_ != 0) {
                    if (!emit_unpack_vector(bus, row_, true, error)) {
                        return false;
                    }
                }
                return finish_command(bus);
            }
            return true;
        }

        // VIF0 has fewer legal commands than VIF1. Treat reserved commands as
        // VIFcode errors rather than host-fatal errors so firmware probing can
        // observe ER1 and continue through its normal recovery path.
        {
            u32 stat = 0;
            if (bus.read32(kVif0Stat, stat)) {
                bus.write32(kVif0Stat, stat | kEr1);
            }
        }
        return finish_command(bus);
    }
}

bool Vif0Dma::emit_unpack_vector(
    EeBus& bus,
    const std::array<u32, 4>& vector,
    bool filling,
    std::string& error) {
    if (unpack_target_remaining_ == 0) return true;

    const u32 address =
        kVu0Data + ((unpack_dest_ & 0xFFu) * 16u);
    const u32 mask_row = std::min(unpack_cycle_pos_, 3u);

    for (u32 component = 0; component < 4u; ++component) {
        u32 value = filling ? row_[component] : vector[component];
        bool protect = false;
        const bool mode_enabled = unpack_vl_ != 3u;

        if (!filling && unpack_masked_) {
            const u32 action =
                (mask_ >> (mask_row * 8u + component * 2u)) & 0x3u;
            if (action == 1u) {
                value = row_[component];
            } else if (action == 2u) {
                value = col_[mask_row];
            } else if (action == 3u) {
                protect = true;
            } else if (mode_enabled) {
                switch (mode_ & 0x3u) {
                case 1: value += row_[component]; break;
                case 2:
                    row_[component] += value;
                    value = row_[component];
                    break;
                case 3:
                    row_[component] = value;
                    break;
                default:
                    break;
                }
            }
        } else if (!filling && mode_enabled) {
            switch (mode_ & 0x3u) {
            case 1: value += row_[component]; break;
            case 2:
                row_[component] += value;
                value = row_[component];
                break;
            case 3:
                row_[component] = value;
                break;
            default:
                break;
            }
        }

        if (!protect &&
            !bus.write32(address + component * 4u, value)) {
            error = "VIF0 UNPACK VU0 data write fault";
            return false;
        }
    }

    --unpack_target_remaining_;
    unpack_dest_ = (unpack_dest_ + 1u) & 0xFFu;
    ++unpack_cycle_pos_;

    const u32 cl = cycle_length(cycle_ & 0xFFu);
    const u32 wl = cycle_length((cycle_ >> 8) & 0xFFu);
    if (cl < wl) {
        if (!filling &&
            unpack_cycle_pos_ == cl &&
            unpack_target_remaining_ != 0) {
            for (u32 i = 0;
                 i < wl - cl && unpack_target_remaining_ != 0;
                 ++i) {
                if (!emit_unpack_vector(bus, row_, true, error)) {
                    return false;
                }
            }
        }
        if (unpack_cycle_pos_ >= wl) unpack_cycle_pos_ = 0;
    } else if (unpack_cycle_pos_ >= wl) {
        unpack_dest_ =
            (unpack_dest_ + (cl - wl)) & 0xFFu;
        unpack_cycle_pos_ = 0;
    }

    const u32 encoded_remaining =
        unpack_target_remaining_ == 256u
            ? 0u
            : (unpack_target_remaining_ & 0xFFu);
    if (!bus.write32(kVif0Num, encoded_remaining)) {
        error = "failed to update VIF0 UNPACK NUM";
        return false;
    }
    return true;
}

bool Vif0Dma::consume_payload_word(
    EeBus& bus,
    u32 word,
    std::string& error) {
    switch (payload_) {
    case Payload::Mask:
        mask_ = word;
        if (!bus.write32(kVif0Mask, mask_)) {
            error = "failed to write VIF0 MASK";
            return false;
        }
        return finish_command(bus);

    case Payload::Row:
        row_[payload_index_] = word;
        if (!bus.write32(
                kVif0Row + payload_index_ * 0x10u,
                word)) {
            error = "failed to write VIF0 ROW";
            return false;
        }
        if (++payload_index_ == 4u) return finish_command(bus);
        return true;

    case Payload::Col:
        col_[payload_index_] = word;
        if (!bus.write32(
                kVif0Col + payload_index_ * 0x10u,
                word)) {
            error = "failed to write VIF0 COL";
            return false;
        }
        if (++payload_index_ == 4u) return finish_command(bus);
        return true;

    case Payload::Mpg: {
        if (!bus.write32(
                kVu0Micro + (mpg_address_ & 0xFFCu),
                word)) {
            error = "VIF0 MPG VU0 micro write fault";
            return false;
        }
        mpg_address_ = (mpg_address_ + 4u) & 0xFFFu;
        if (mpg_words_remaining_ != 0) --mpg_words_remaining_;
        const u32 remaining_instructions =
            (mpg_words_remaining_ + 1u) / 2u;
        if (!bus.write32(
                kVif0Num,
                remaining_instructions == 256u
                    ? 0u
                    : (remaining_instructions & 0xFFu))) {
            error = "failed to update VIF0 MPG NUM";
            return false;
        }
        if (mpg_words_remaining_ == 0) return finish_command(bus);
        return true;
    }

    case Payload::Unpack: {
        unpack_bits_ |= static_cast<u64>(word) << unpack_bit_count_;
        unpack_bit_count_ += 32u;

        while (unpack_source_remaining_ != 0) {
            u32 bits = 0;
            if (unpack_vl_ == 3u) {
                static constexpr u32 v45_bits[4] =
                    {5u, 5u, 5u, 1u};
                bits = v45_bits[unpack_component_ & 3u];
            } else {
                bits = 32u >> unpack_vl_;
            }
            if (unpack_bit_count_ < bits) break;

            u32 raw = 0;
            if (bits == 32u) {
                raw = static_cast<u32>(unpack_bits_);
                unpack_bits_ >>= 32u;
                unpack_bit_count_ -= 32u;
            } else {
                const u32 element_mask = (1u << bits) - 1u;
                raw =
                    static_cast<u32>(unpack_bits_) &
                    element_mask;
                unpack_bits_ >>= bits;
                unpack_bit_count_ -= bits;
            }

            if (unpack_vl_ == 3u) {
                unpack_vector_[unpack_component_] =
                    unpack_component_ == 3u
                        ? (raw << 7u)
                        : (raw << 3u);
            } else {
                unpack_vector_[unpack_component_] =
                    extend_element(
                        raw,
                        bits,
                        unpack_unsigned_);
            }
            ++unpack_component_;

            const u32 components = unpack_vn_ + 1u;
            if (unpack_component_ < components) continue;

            if (unpack_vn_ == 0u) {
                unpack_vector_[1] = unpack_vector_[0];
                unpack_vector_[2] = unpack_vector_[0];
                unpack_vector_[3] = unpack_vector_[0];
            } else if (unpack_vn_ == 1u) {
                unpack_vector_[2] = unpack_vector_[0];
                unpack_vector_[3] = unpack_vector_[1];
            } else if (unpack_vn_ == 2u) {
                unpack_vector_[3] = 0;
            }

            if (!emit_unpack_vector(
                    bus,
                    unpack_vector_,
                    false,
                    error)) {
                return false;
            }

            unpack_vector_.fill(0);
            unpack_component_ = 0;
            --unpack_source_remaining_;

            if (unpack_source_remaining_ == 0) {
                unpack_bits_ = 0;
                unpack_bit_count_ = 0;
                return finish_command(bus);
            }
        }
        return true;
    }

    case Payload::WaitVu:
        error = "VIF0 attempted to consume data while waiting for VU0";
        return false;

    case Payload::None:
        break;
    }

    error = "invalid VIF0 payload state";
    return false;
}

bool Vif0Dma::consume_word(
    EeBus& bus,
    u32 word,
    std::string& error) {
    if (payload_ == Payload::None) {
        return begin_command(bus, word, error);
    }
    return consume_payload_word(bus, word, error);
}

bool Vif0Dma::consume_qword(
    EeBus& bus,
    u64 lo,
    u64 hi,
    std::string& error) {
    const u32 words[4] = {
        static_cast<u32>(lo),
        static_cast<u32>(lo >> 32),
        static_cast<u32>(hi),
        static_cast<u32>(hi >> 32),
    };
    for (u32 i = 0; i < 4u; ++i) {
        if (payload_ == Payload::WaitVu) {
            deferred_word_count_ = 0;
            deferred_word_index_ = 0;
            for (u32 j = i; j < 4u; ++j) {
                deferred_words_[deferred_word_count_++] = words[j];
            }
            return true;
        }
        if (!consume_word(bus, words[i], error)) return false;
    }
    return true;
}

bool Vif0Dma::service(
    EeBus& bus,
    std::string& error) {
    error.clear();

    u32 chcr = 0;
    if (!bus.read32(kVif0Chcr, chcr)) {
        error = "failed to read VIF0 CHCR";
        return false;
    }
    if ((chcr & kChcrStr) == 0) return true;

    u32 ctrl = 0;
    if (!bus.read32(kDmacCtrl, ctrl)) {
        error = "failed to read DMAC CTRL";
        return false;
    }
    if ((ctrl & 1u) == 0) return true;

    // Channel 0 is memory -> VIF0. There is no VIF0 reverse GS readback path.
    if ((chcr & kChcrDir) == 0) return true;

    const u32 mode = (chcr >> 2) & 0x3u;
    if (mode != kModeNormal && mode != kModeChain) {
        return true;
    }

    u32 qwc = 0;
    u32 madr = 0;
    u32 tadr = 0;
    if (!bus.read32(kVif0Qwc, qwc) ||
        !bus.read32(kVif0Madr, madr) ||
        !bus.read32(kVif0Tadr, tadr)) {
        error = "VIF0 DMA state read failed";
        return false;
    }
    qwc &= 0xFFFFu;
    set_fqc(bus, qwc);

    if (payload_ == Payload::WaitVu) {
        set_vps(bus, 1u);
        if (vu0_ != nullptr && vu0_->running()) {
            return true;
        }

        if (!finish_command(bus)) {
            error = "failed to finish VIF0 VU0 wait";
            return false;
        }

        while (deferred_word_index_ < deferred_word_count_) {
            const u32 word = deferred_words_[deferred_word_index_++];
            if (!consume_word(bus, word, error)) {
                if (error.empty()) {
                    error = "VIF0 deferred command decode failed";
                }
                return false;
            }
            if (payload_ == Payload::WaitVu) {
                set_vps(bus, 1u);
                return true;
            }
        }
        deferred_word_count_ = 0;
        deferred_word_index_ = 0;
    }

    if (qwc != 0u && payload_ == Payload::None) {
        set_vps(bus, 1u);
    }

    if (mode == kModeNormal && qwc == 0) {
        if (!complete(bus, chcr)) {
            error = "VIF0 normal DMA completion failed";
            return false;
        }
        return true;
    }

    for (u32 guard = 0;
         mode == kModeChain && qwc == 0 && guard < 8u;
         ++guard) {
        if (end_after_qwc_) {
            if (!complete(bus, chcr)) {
                error = "VIF0 chain completion failed";
                return false;
            }
            return true;
        }

        u64 tag_lo = 0;
        u64 tag_hi = 0;
        const u32 tag_address = tadr & 0x7FFFFFF0u;
        if (!bus.read64(tag_address, tag_lo) ||
            !bus.read64(tag_address + 8u, tag_hi)) {
            error = "VIF0 DMA tag fetch fault";
            return false;
        }

        const u32 tag0 = static_cast<u32>(tag_lo);
        const u32 tag1 = static_cast<u32>(tag_lo >> 32);
        qwc = tag0 & 0xFFFFu;
        const u32 id = (tag0 >> 28) & 0x7u;
        const bool irq = (tag0 & 0x80000000u) != 0;
        const bool spr = (tag1 & 0x80000000u) != 0;
        const u32 address = apply_spr(tag1, spr);

        chcr =
            (chcr & 0x0000FFFFu) |
            (tag0 & 0xFFFF0000u);
        end_after_qwc_ =
            irq && ((chcr & kChcrTie) != 0);

        switch (id) {
        case 0: // REFE
            madr = address;
            tadr = tag_address + 16u;
            end_after_qwc_ = true;
            break;
        case 1: // CNT
            madr = tag_address + 16u;
            tadr = madr + qwc * 16u;
            break;
        case 2: // NEXT
            madr = tag_address + 16u;
            tadr = address;
            break;
        case 3: // REF
        case 4: // REFS
            madr = address;
            tadr = tag_address + 16u;
            break;
        case 5: { // CALL
            madr = tag_address + 16u;
            const u32 return_address =
                madr + qwc * 16u;
            u32 asp = (chcr >> 4) & 0x3u;
            if (asp == 0) {
                if (!bus.write32(kVif0Asr0, return_address)) {
                    error = "VIF0 DMA ASR0 write failed";
                    return false;
                }
                asp = 1;
            } else if (asp == 1) {
                if (!bus.write32(kVif0Asr1, return_address)) {
                    error = "VIF0 DMA ASR1 write failed";
                    return false;
                }
                asp = 2;
            } else {
                end_after_qwc_ = true;
            }
            chcr =
                (chcr & ~(0x3u << 4)) |
                (asp << 4);
            tadr = address;
            break;
        }
        case 6: { // RET
            madr = tag_address + 16u;
            u32 asp = (chcr >> 4) & 0x3u;
            if (asp == 2) {
                if (!bus.read32(kVif0Asr1, tadr) ||
                    !bus.write32(kVif0Asr1, 0u)) {
                    error = "VIF0 DMA ASR1 return failed";
                    return false;
                }
                asp = 1;
            } else if (asp == 1) {
                if (!bus.read32(kVif0Asr0, tadr) ||
                    !bus.write32(kVif0Asr0, 0u)) {
                    error = "VIF0 DMA ASR0 return failed";
                    return false;
                }
                asp = 0;
            } else {
                end_after_qwc_ = true;
            }
            chcr =
                (chcr & ~(0x3u << 4)) |
                (asp << 4);
            break;
        }
        case 7: // END
            madr = tag_address + 16u;
            end_after_qwc_ = true;
            break;
        }

        if (!bus.write32(kVif0Chcr, chcr) ||
            !bus.write32(kVif0Madr, madr) ||
            !bus.write32(kVif0Qwc, qwc) ||
            !bus.write32(kVif0Tadr, tadr)) {
            error = "VIF0 DMA tag state write failed";
            return false;
        }
        set_fqc(bus, qwc);

        if ((chcr & kChcrTte) != 0) {
            const u32 tte_words[2] = {
                static_cast<u32>(tag_hi),
                static_cast<u32>(tag_hi >> 32),
            };
            for (u32 i = 0; i < 2u; ++i) {
                if (payload_ == Payload::WaitVu) {
                    deferred_word_count_ = 0;
                    deferred_word_index_ = 0;
                    for (u32 j = i; j < 2u; ++j) {
                        deferred_words_[deferred_word_count_++] =
                            tte_words[j];
                    }
                    break;
                }
                if (!consume_word(bus, tte_words[i], error)) {
                    if (error.empty()) {
                        error = "VIF0 TTE command decode failed";
                    }
                    return false;
                }
            }
        }

        if (qwc == 0 && end_after_qwc_ &&
            payload_ != Payload::WaitVu) {
            if (!complete(bus, chcr)) {
                error = "VIF0 empty chain completion failed";
                return false;
            }
            return true;
        }
    }

    if (qwc == 0) return true;

    u64 lo = 0;
    u64 hi = 0;
    if (!bus.read64(madr, lo) ||
        !bus.read64(madr + 8u, hi)) {
        error = "VIF0 DMA payload fetch fault";
        return false;
    }
    if (!consume_qword(bus, lo, hi, error)) {
        if (error.empty()) {
            error = "VIF0 command stream decode failed";
        }
        return false;
    }

    madr = (madr + 16u) & 0x7FFFFFF0u;
    --qwc;
    if (!bus.write32(kVif0Madr, madr) ||
        !bus.write32(kVif0Qwc, qwc)) {
        error = "failed to update VIF0 DMA progress";
        return false;
    }
    set_fqc(bus, qwc);

    if (qwc == 0 &&
        payload_ != Payload::WaitVu &&
        (mode == kModeNormal || end_after_qwc_)) {
        if (!complete(bus, chcr)) {
            error = "VIF0 DMA completion failed";
            return false;
        }
    }

    return true;
}

} // namespace ps2
