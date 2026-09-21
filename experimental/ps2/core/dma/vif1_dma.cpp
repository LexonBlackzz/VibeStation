#include "core/dma/vif1_dma.h"

#include "core/gs/gs_core.h"
#include "core/gs/gs_privileged.h"
#include "core/memory/ee_bus.h"
#include "core/vu/vu1.h"

#include <algorithm>

namespace ps2 {
namespace {

constexpr u32 kDmacCtrl = 0x1000E000u;
constexpr u32 kVif1Chcr = 0x10009000u;
constexpr u32 kVif1Madr = 0x10009010u;
constexpr u32 kVif1Qwc = 0x10009020u;
constexpr u32 kVif1Tadr = 0x10009030u;
constexpr u32 kVif1Asr0 = 0x10009040u;
constexpr u32 kVif1Asr1 = 0x10009050u;

constexpr u32 kVif1Stat = 0x10003C00u;
constexpr u32 kVif1Mark = 0x10003C30u;
constexpr u32 kVif1Cycle = 0x10003C40u;
constexpr u32 kVif1Mode = 0x10003C50u;
constexpr u32 kVif1Num = 0x10003C60u;
constexpr u32 kVif1Mask = 0x10003C70u;
constexpr u32 kVif1Code = 0x10003C80u;
constexpr u32 kVif1Itops = 0x10003C90u;
constexpr u32 kVif1Base = 0x10003CA0u;
constexpr u32 kVif1Ofst = 0x10003CB0u;
constexpr u32 kVif1Tops = 0x10003CC0u;
constexpr u32 kVif1Itop = 0x10003CD0u;
constexpr u32 kVif1Top = 0x10003CE0u;
constexpr u32 kVif1Row = 0x10003D00u;
constexpr u32 kVif1Col = 0x10003D40u;

constexpr u32 kVu1Micro = 0x11008000u;
constexpr u32 kVu1Data = 0x1100C000u;

constexpr u32 kChcrDir = 1u << 0;
constexpr u32 kChcrModeMask = 3u << 2;
constexpr u32 kChcrTte = 1u << 6;
constexpr u32 kChcrTie = 1u << 7;
constexpr u32 kChcrStr = 1u << 8;
constexpr u32 kVif1Fdr = 1u << 23;

constexpr u32 kModeNormal = 0;
constexpr u32 kModeChain = 1;

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

void Vif1Dma::reset() {
    end_after_qwc_ = false;
    payload_ = Payload::None;
    command_irq_pending_ = false;
    active_code_ = 0;
    payload_index_ = 0;

    cycle_ = 0;
    mode_ = 0;
    mask_ = 0;
    row_.fill(0);
    col_.fill(0);
    base_ = 0;
    offset_ = 0;
    tops_ = 0;
    top_ = 0;
    itop_ = 0;
    double_buffer_ = false;

    mpg_address_ = 0;
    mpg_words_remaining_ = 0;
    direct_words_.fill(0);
    direct_word_count_ = 0;
    direct_words_remaining_ = 0;
    reset_unpack();
}

void Vif1Dma::reset_unpack() {
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

bool Vif1Dma::complete(EeBus& bus, u32 chcr) {
    end_after_qwc_ = false;
    if (!bus.write32(kVif1Qwc, 0u)) return false;
    if (!bus.write32(kVif1Chcr, chcr & ~kChcrStr)) return false;
    bus.raise_dmac(1);
    return true;
}

bool Vif1Dma::finish_command(EeBus& bus) {
    payload_ = Payload::None;
    payload_index_ = 0;
    if (command_irq_pending_) {
        // VIF1 interrupts are routed through EE INTC source 5.  A full VIS
        // stall model can be layered on later; raising the interrupt here is
        // enough for BIOS handlers while keeping the bootstrap stream moving.
        bus.raise_intc(5);
    }
    command_irq_pending_ = false;
    return true;
}

bool Vif1Dma::begin_command(
    EeBus& bus,
    u32 word,
    std::string& error) {
    error.clear();
    active_code_ = word;
    command_irq_pending_ = (word & 0x80000000u) != 0;
    const u32 command = (word >> 24) & 0x7Fu;
    const u32 num_field = (word >> 16) & 0xFFu;
    const u32 immediate = word & 0xFFFFu;

    if (!bus.write32(kVif1Code, word)) {
        error = "failed to mirror VIF1 CODE";
        return false;
    }

    switch (command) {
    case 0x00: // NOP
        return finish_command(bus);

    case 0x01: // STCYCL
        cycle_ = immediate;
        if (!bus.write32(kVif1Cycle, cycle_)) {
            error = "failed to write VIF1 CYCLE";
            return false;
        }
        return finish_command(bus);

    case 0x02: // OFFSET
        offset_ = immediate & 0x3FFu;
        double_buffer_ = false;
        tops_ = base_;
        if (!bus.write32(kVif1Ofst, offset_) ||
            !bus.write32(kVif1Tops, tops_)) {
            error = "failed to write VIF1 OFFSET state";
            return false;
        }
        return finish_command(bus);

    case 0x03: // BASE
        base_ = immediate & 0x3FFu;
        if (!bus.write32(kVif1Base, base_)) {
            error = "failed to write VIF1 BASE";
            return false;
        }
        return finish_command(bus);

    case 0x04: // ITOP
        itop_ = immediate & 0x3FFu;
        if (!bus.write32(kVif1Itop, itop_) ||
            !bus.write32(kVif1Itops, itop_)) {
            error = "failed to write VIF1 ITOP";
            return false;
        }
        return finish_command(bus);

    case 0x05: // STMOD
        mode_ = immediate & 0x3u;
        if (!bus.write32(kVif1Mode, mode_)) {
            error = "failed to write VIF1 MODE";
            return false;
        }
        return finish_command(bus);

    case 0x06: // MSKPATH3
        // PATH arbitration is currently serialized by the bootstrap core, so
        // retaining the command in CODE is sufficient until FIFO contention is
        // modeled.  Do not stall PATH3: that would deadlock a FIFO-less model.
        return finish_command(bus);

    case 0x07: // MARK
        if (!bus.write32(kVif1Mark, immediate)) {
            error = "failed to write VIF1 MARK";
            return false;
        }
        return finish_command(bus);

    case 0x10: // FLUSHE
    case 0x11: // FLUSH
    case 0x13: // FLUSHA
        // VU1 micro execution is not scheduled yet, therefore there is no
        // asynchronous VU/PATH1 work for these commands to wait on.
        return finish_command(bus);

    case 0x14: // MSCAL
    case 0x15: // MSCALF
        top_ = tops_;
        double_buffer_ = !double_buffer_;
        tops_ = double_buffer_ ? ((base_ + offset_) & 0x3FFu) : base_;
        if (!bus.write32(kVif1Top, top_) ||
            !bus.write32(kVif1Tops, tops_)) {
            error = "failed to update VIF1 double-buffer state";
            return false;
        }
        if (vu1_ != nullptr) {
            vu1_->start(immediate & 0x3FFu);
        }
        return finish_command(bus);

    case 0x17: // MSCNT
        if (vu1_ != nullptr) {
            vu1_->continue_run();
        }
        return finish_command(bus);

    case 0x20: // STMASK
        payload_ = Payload::Mask;
        payload_index_ = 0;
        return true;

    case 0x30: // STROW
        payload_ = Payload::Row;
        payload_index_ = 0;
        return true;

    case 0x31: // STCOL
        payload_ = Payload::Col;
        payload_index_ = 0;
        return true;

    case 0x4A: { // MPG
        const u32 instructions = num_field == 0 ? 256u : num_field;
        mpg_address_ = (immediate * 8u) & 0x3FFFu;
        mpg_words_remaining_ = instructions * 2u;
        payload_ = Payload::Mpg;
        payload_index_ = 0;
        if (!bus.write32(kVif1Num, num_field)) {
            error = "failed to write VIF1 MPG NUM";
            return false;
        }
        return true;
    }

    case 0x50: // DIRECT
    case 0x51: { // DIRECTHL
        const u32 qwords = immediate == 0 ? 65536u : immediate;
        direct_words_remaining_ = qwords * 4u;
        direct_word_count_ = 0;
        direct_words_.fill(0);
        payload_ = Payload::Direct;
        payload_index_ = 0;
        return true;
    }

    default:
        if (command >= 0x60u && command <= 0x7Fu) {
            const u32 format = command & 0x0Fu;
            if (!supported_unpack_format(format)) {
                error = "unsupported VIF1 UNPACK format";
                return false;
            }

            reset_unpack();
            unpack_vn_ = (format >> 2) & 0x3u;
            unpack_vl_ = format & 0x3u;
            if (unpack_vl_ == 3u && unpack_vn_ != 3u) {
                error = "reserved VIF1 UNPACK format";
                return false;
            }

            unpack_unsigned_ = (immediate & 0x4000u) != 0;
            unpack_masked_ = (command & 0x10u) != 0;
            unpack_target_remaining_ = num_field == 0 ? 256u : num_field;

            const u32 cl = cycle_length(cycle_ & 0xFFu);
            const u32 wl = cycle_length((cycle_ >> 8) & 0xFFu);
            if (cl < wl) {
                unpack_source_remaining_ =
                    (unpack_target_remaining_ / wl) * cl +
                    std::min(unpack_target_remaining_ % wl, cl);
            } else {
                unpack_source_remaining_ = unpack_target_remaining_;
            }

            unpack_dest_ = immediate & 0x3FFu;
            if ((immediate & 0x8000u) != 0) {
                unpack_dest_ = (unpack_dest_ + tops_) & 0x3FFu;
            }

            payload_ = Payload::Unpack;
            if (!bus.write32(kVif1Num, num_field)) {
                error = "failed to write VIF1 UNPACK NUM";
                return false;
            }

            // Degenerate fill-only cycles are legal in the encoding but not
            // useful during bootstrap.  Complete cleanly rather than hanging.
            if (unpack_source_remaining_ == 0) {
                while (unpack_target_remaining_ != 0) {
                    if (!emit_unpack_vector(bus, row_, true, error)) return false;
                }
                top_ = unpack_dest_ & 0x3FFu;
                bus.write32(kVif1Top, top_);
                return finish_command(bus);
            }
            return true;
        }

        error = "unsupported VIF1 command";
        return false;
    }
}

bool Vif1Dma::emit_unpack_vector(
    EeBus& bus,
    const std::array<u32, 4>& vector,
    bool filling,
    std::string& error) {
    if (unpack_target_remaining_ == 0) return true;

    const u32 address = kVu1Data + ((unpack_dest_ & 0x3FFu) * 16u);
    const u32 mask_row = std::min(unpack_cycle_pos_, 3u);

    for (u32 component = 0; component < 4u; ++component) {
        u32 value = filling ? row_[component] : vector[component];
        bool protect = false;

        if (!filling && unpack_masked_) {
            const u32 action =
                (mask_ >> (mask_row * 8u + component * 2u)) & 0x3u;
            if (action == 1u) {
                value = row_[component];
            } else if (action == 2u) {
                value = col_[mask_row];
            } else if (action == 3u) {
                protect = true;
            } else {
                switch (mode_ & 0x3u) {
                case 1:
                    value += row_[component];
                    break;
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
        } else if (!filling) {
            switch (mode_ & 0x3u) {
            case 1:
                value += row_[component];
                break;
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

        if (!protect && !bus.write32(address + component * 4u, value)) {
            error = "VIF1 UNPACK VU1 data write fault";
            return false;
        }
    }

    --unpack_target_remaining_;
    unpack_dest_ = (unpack_dest_ + 1u) & 0x3FFu;
    ++unpack_cycle_pos_;

    const u32 cl = cycle_length(cycle_ & 0xFFu);
    const u32 wl = cycle_length((cycle_ >> 8) & 0xFFu);

    if (cl < wl) {
        if (!filling && unpack_cycle_pos_ == cl &&
            unpack_target_remaining_ != 0) {
            const u32 fill_count = wl - cl;
            for (u32 i = 0; i < fill_count &&
                            unpack_target_remaining_ != 0; ++i) {
                if (!emit_unpack_vector(bus, row_, true, error)) return false;
            }
        }
        if (unpack_cycle_pos_ >= wl) unpack_cycle_pos_ = 0;
    } else if (unpack_cycle_pos_ >= wl) {
        unpack_dest_ = (unpack_dest_ + (cl - wl)) & 0x3FFu;
        unpack_cycle_pos_ = 0;
    }

    const u32 encoded_remaining =
        unpack_target_remaining_ == 256u ? 0u :
        (unpack_target_remaining_ & 0xFFu);
    if (!bus.write32(kVif1Num, encoded_remaining)) {
        error = "failed to update VIF1 UNPACK NUM";
        return false;
    }
    return true;
}

bool Vif1Dma::consume_payload_word(
    EeBus& bus,
    GsCore& gs,
    u32 word,
    std::string& error) {
    switch (payload_) {
    case Payload::Mask:
        mask_ = word;
        if (!bus.write32(kVif1Mask, mask_)) {
            error = "failed to write VIF1 MASK";
            return false;
        }
        return finish_command(bus);

    case Payload::Row:
        row_[payload_index_] = word;
        if (!bus.write32(kVif1Row + payload_index_ * 0x10u, word)) {
            error = "failed to write VIF1 ROW";
            return false;
        }
        if (++payload_index_ == 4u) return finish_command(bus);
        return true;

    case Payload::Col:
        col_[payload_index_] = word;
        if (!bus.write32(kVif1Col + payload_index_ * 0x10u, word)) {
            error = "failed to write VIF1 COL";
            return false;
        }
        if (++payload_index_ == 4u) return finish_command(bus);
        return true;

    case Payload::Mpg: {
        if (!bus.write32(kVu1Micro + (mpg_address_ & 0x3FFCu), word)) {
            error = "VIF1 MPG VU1 micro write fault";
            return false;
        }
        mpg_address_ = (mpg_address_ + 4u) & 0x3FFFu;
        if (mpg_words_remaining_ != 0) --mpg_words_remaining_;
        const u32 remaining_instructions = (mpg_words_remaining_ + 1u) / 2u;
        if (!bus.write32(
                kVif1Num,
                remaining_instructions == 256u
                    ? 0u
                    : (remaining_instructions & 0xFFu))) {
            error = "failed to update VIF1 MPG NUM";
            return false;
        }
        if (mpg_words_remaining_ == 0) return finish_command(bus);
        return true;
    }

    case Payload::Direct:
        direct_words_[direct_word_count_++] = word;
        if (direct_words_remaining_ != 0) --direct_words_remaining_;
        if (direct_word_count_ == 4u) {
            const u64 lo =
                static_cast<u64>(direct_words_[0]) |
                (static_cast<u64>(direct_words_[1]) << 32);
            const u64 hi =
                static_cast<u64>(direct_words_[2]) |
                (static_cast<u64>(direct_words_[3]) << 32);
            gs.write_gif_qword(lo, hi);
            direct_word_count_ = 0;
        }
        if (direct_words_remaining_ == 0) return finish_command(bus);
        return true;

    case Payload::Unpack: {
        unpack_bits_ |= static_cast<u64>(word) << unpack_bit_count_;
        unpack_bit_count_ += 32u;

        while (unpack_source_remaining_ != 0) {
            u32 bits = 0;
            if (unpack_vl_ == 3u) {
                static constexpr u32 v45_bits[4] = {5u, 5u, 5u, 1u};
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
                raw = static_cast<u32>(unpack_bits_) & element_mask;
                unpack_bits_ >>= bits;
                unpack_bit_count_ -= bits;
            }

            unpack_vector_[unpack_component_] =
                extend_element(raw, bits, unpack_unsigned_);
            ++unpack_component_;

            const u32 components = unpack_vn_ + 1u;
            if (unpack_component_ < components) continue;

            if (unpack_vn_ == 0u) {
                unpack_vector_[1] = unpack_vector_[0];
                unpack_vector_[2] = unpack_vector_[0];
                unpack_vector_[3] = unpack_vector_[0];
            } else if (unpack_vn_ == 1u) {
                // Real VIF hardware repeats V2 into Z/W rather than leaving
                // those lanes as zero.
                unpack_vector_[2] = unpack_vector_[0];
                unpack_vector_[3] = unpack_vector_[1];
            } else if (unpack_vn_ == 2u) {
                // V3.W is architecturally indeterminate.  Keeping it stable
                // at zero is preferable to leaking host state during bootstrap.
                unpack_vector_[3] = 0;
            }

            if (!emit_unpack_vector(bus, unpack_vector_, false, error)) {
                return false;
            }

            unpack_vector_.fill(0);
            unpack_component_ = 0;
            --unpack_source_remaining_;

            if (unpack_source_remaining_ == 0) {
                // Any residual bits in the last 32-bit payload word are VIF
                // padding, not the beginning of the next command.
                unpack_bits_ = 0;
                unpack_bit_count_ = 0;
                top_ = unpack_dest_ & 0x3FFu;
                if (!bus.write32(kVif1Top, top_)) {
                    error = "failed to update VIF1 TOP";
                    return false;
                }
                return finish_command(bus);
            }
        }
        return true;
    }

    case Payload::None:
        break;
    }

    error = "invalid VIF1 payload state";
    return false;
}

bool Vif1Dma::consume_word(
    EeBus& bus,
    GsCore& gs,
    u32 word,
    std::string& error) {
    if (payload_ == Payload::None) {
        return begin_command(bus, word, error);
    }
    return consume_payload_word(bus, gs, word, error);
}

bool Vif1Dma::consume_qword(
    EeBus& bus,
    GsCore& gs,
    u64 lo,
    u64 hi,
    std::string& error) {
    const u32 words[4] = {
        static_cast<u32>(lo),
        static_cast<u32>(lo >> 32),
        static_cast<u32>(hi),
        static_cast<u32>(hi >> 32),
    };
    for (u32 word : words) {
        if (!consume_word(bus, gs, word, error)) return false;
    }
    return true;
}

bool Vif1Dma::service_forward(
    EeBus& bus,
    GsCore& gs,
    u32 chcr,
    std::string& error) {
    const u32 mode = (chcr >> 2) & 0x3u;
    if (mode != kModeNormal && mode != kModeChain) {
        // Interleave/MFIFO need their own address-generation rules.  Leave the
        // channel armed instead of inventing transfer progress.
        return true;
    }

    u32 qwc = 0;
    u32 madr = 0;
    u32 tadr = 0;
    if (!bus.read32(kVif1Qwc, qwc) ||
        !bus.read32(kVif1Madr, madr) ||
        !bus.read32(kVif1Tadr, tadr)) {
        error = "VIF1 forward DMA state read failed";
        return false;
    }
    qwc &= 0xFFFFu;

    if (mode == kModeNormal && qwc == 0) {
        if (!complete(bus, chcr)) {
            error = "VIF1 normal DMA completion failed";
            return false;
        }
        return true;
    }

    // Walk empty source-chain tags immediately.  This is important for BIOS
    // setup chains that use CNT/NEXT tags solely to carry VIFcodes via TTE.
    for (u32 guard = 0; mode == kModeChain && qwc == 0 && guard < 8u; ++guard) {
        if (end_after_qwc_) {
            if (!complete(bus, chcr)) {
                error = "VIF1 chain completion failed";
                return false;
            }
            return true;
        }

        u64 tag_lo = 0;
        u64 tag_hi = 0;
        const u32 tag_address = tadr & 0x7FFFFFF0u;
        if (!bus.read64(tag_address, tag_lo) ||
            !bus.read64(tag_address + 8u, tag_hi)) {
            error = "VIF1 DMA tag fetch fault";
            return false;
        }

        const u32 tag0 = static_cast<u32>(tag_lo);
        const u32 tag1 = static_cast<u32>(tag_lo >> 32);
        qwc = tag0 & 0xFFFFu;
        const u32 id = (tag0 >> 28) & 0x7u;
        const bool irq = (tag0 & 0x80000000u) != 0;
        const bool spr = (tag1 & 0x80000000u) != 0;
        const u32 address = apply_spr(tag1, spr);

        chcr = (chcr & 0x0000FFFFu) | (tag0 & 0xFFFF0000u);
        end_after_qwc_ = irq && ((chcr & kChcrTie) != 0);

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
            const u32 return_address = madr + qwc * 16u;
            u32 asp = (chcr >> 4) & 0x3u;
            if (asp == 0) {
                if (!bus.write32(kVif1Asr0, return_address)) {
                    error = "VIF1 DMA ASR0 write failed";
                    return false;
                }
                asp = 1;
            } else if (asp == 1) {
                if (!bus.write32(kVif1Asr1, return_address)) {
                    error = "VIF1 DMA ASR1 write failed";
                    return false;
                }
                asp = 2;
            } else {
                end_after_qwc_ = true;
            }
            chcr = (chcr & ~(0x3u << 4)) | (asp << 4);
            tadr = address;
            break;
        }
        case 6: { // RET
            madr = tag_address + 16u;
            u32 asp = (chcr >> 4) & 0x3u;
            if (asp == 2) {
                if (!bus.read32(kVif1Asr1, tadr) ||
                    !bus.write32(kVif1Asr1, 0u)) {
                    error = "VIF1 DMA ASR1 return failed";
                    return false;
                }
                asp = 1;
            } else if (asp == 1) {
                if (!bus.read32(kVif1Asr0, tadr) ||
                    !bus.write32(kVif1Asr0, 0u)) {
                    error = "VIF1 DMA ASR0 return failed";
                    return false;
                }
                asp = 0;
            } else {
                end_after_qwc_ = true;
            }
            chcr = (chcr & ~(0x3u << 4)) | (asp << 4);
            break;
        }
        case 7: // END
            madr = tag_address + 16u;
            end_after_qwc_ = true;
            break;
        }

        if (!bus.write32(kVif1Chcr, chcr) ||
            !bus.write32(kVif1Madr, madr) ||
            !bus.write32(kVif1Qwc, qwc) ||
            !bus.write32(kVif1Tadr, tadr)) {
            error = "VIF1 DMA tag state write failed";
            return false;
        }

        if ((chcr & kChcrTte) != 0) {
            if (!consume_word(bus, gs, static_cast<u32>(tag_hi), error) ||
                !consume_word(
                    bus, gs, static_cast<u32>(tag_hi >> 32), error)) {
                if (error.empty()) error = "VIF1 TTE command decode failed";
                return false;
            }
        }

        if (qwc == 0 && end_after_qwc_) {
            if (!complete(bus, chcr)) {
                error = "VIF1 empty chain completion failed";
                return false;
            }
            return true;
        }
    }

    if (qwc == 0) return true;

    u64 lo = 0;
    u64 hi = 0;
    if (!bus.read64(madr, lo) || !bus.read64(madr + 8u, hi)) {
        error = "VIF1 DMA payload fetch fault";
        return false;
    }
    if (!consume_qword(bus, gs, lo, hi, error)) {
        if (error.empty()) error = "VIF1 command stream decode failed";
        return false;
    }

    madr = (madr + 16u) & 0x7FFFFFF0u;
    --qwc;
    if (!bus.write32(kVif1Madr, madr) ||
        !bus.write32(kVif1Qwc, qwc)) {
        error = "failed to update VIF1 forward DMA progress";
        return false;
    }

    if (qwc == 0 && (mode == kModeNormal || end_after_qwc_)) {
        if (!complete(bus, chcr)) {
            error = "VIF1 forward DMA completion failed";
            return false;
        }
    }

    return true;
}

bool Vif1Dma::service_reverse(
    EeBus& bus,
    GsCore& gs,
    const GsPrivileged& privileged,
    u32 chcr,
    std::string& error) {
    if ((chcr & kChcrModeMask) != 0) return true;
    if (privileged.busdir() == 0) return true;

    u32 qwc = 0;
    if (!bus.read32(kVif1Qwc, qwc)) {
        error = "failed to read VIF1 QWC";
        return false;
    }
    if (qwc == 0) {
        if (!complete(bus, chcr)) {
            error = "failed to complete zero-length VIF1 DMA";
            return false;
        }
        return true;
    }

    u64 lo = 0;
    u64 hi = 0;
    if (!gs.read_local_to_host_qword(lo, hi)) {
        // Real hardware stalls the DMA while GS readback data is unavailable.
        return true;
    }

    u32 madr = 0;
    if (!bus.read32(kVif1Madr, madr)) {
        error = "failed to read VIF1 MADR";
        return false;
    }
    madr &= 0x7FFFFFF0u;

    if (!bus.write64(madr, lo) || !bus.write64(madr + 8u, hi)) {
        error = "VIF1 reverse DMA RAM write fault";
        return false;
    }

    madr = (madr + 16u) & 0x7FFFFFF0u;
    --qwc;
    if (!bus.write32(kVif1Madr, madr) ||
        !bus.write32(kVif1Qwc, qwc)) {
        error = "failed to update VIF1 reverse DMA registers";
        return false;
    }

    if (qwc == 0 && !complete(bus, chcr)) {
        error = "failed to complete VIF1 reverse DMA";
        return false;
    }

    return true;
}

bool Vif1Dma::service(
    EeBus& bus,
    GsCore& gs,
    const GsPrivileged& privileged,
    std::string& error) {
    error.clear();

    u32 chcr = 0;
    if (!bus.read32(kVif1Chcr, chcr)) {
        error = "failed to read VIF1 CHCR";
        return false;
    }
    if ((chcr & kChcrStr) == 0) return true;

    u32 ctrl = 0;
    if (!bus.read32(kDmacCtrl, ctrl)) {
        error = "failed to read DMAC CTRL";
        return false;
    }
    if ((ctrl & 1u) == 0) return true;

    u32 vif1_stat = 0;
    if (!bus.read32(kVif1Stat, vif1_stat)) {
        error = "failed to read VIF1 STAT";
        return false;
    }

    if ((chcr & kChcrDir) != 0) {
        // Memory -> VIF1.  FDR must agree with the channel direction.
        if ((vif1_stat & kVif1Fdr) != 0) return true;
        return service_forward(bus, gs, chcr, error);
    }

    // VIF1 -> memory is the GS local-to-host readback path.
    if ((vif1_stat & kVif1Fdr) == 0) return true;
    return service_reverse(bus, gs, privileged, chcr, error);
}

} // namespace ps2
