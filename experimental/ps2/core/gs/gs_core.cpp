#include "core/gs/gs_core.h"

#include <bit>

namespace ps2 {
namespace {

constexpr u32 kGifPacked = 0;
constexpr u32 kGifReglist = 1;
constexpr u32 kGifImage = 2;
constexpr u32 kGifImage2 = 3;

constexpr u32 kRegPrim = 0x00;
constexpr u32 kRegRgbaq = 0x01;
constexpr u32 kRegSt = 0x02;
constexpr u32 kRegUv = 0x03;
constexpr u32 kRegXyzf2 = 0x04;
constexpr u32 kRegXyz2 = 0x05;
constexpr u32 kRegXyzf3 = 0x0C;
constexpr u32 kRegXyz3 = 0x0D;

u32 descriptor_at(u64 regs, u32 cursor) {
    return static_cast<u32>((regs >> ((cursor & 0xFu) * 4u)) & 0xFu);
}

} // namespace

void GsCore::reset() {
    registers_.fill(0);
    fifo_words_.fill(0);
    fifo_word_mask_ = 0;
    gif_ = {};
    stats_ = {};
    primitive_vertex_count_ = 0;
}

bool GsCore::write_gif_fifo32(u32 physical, u32 value) {
    if (physical < kGifFifoBase || physical >= kGifFifoBase + 0x10u ||
        (physical & 3u) != 0) {
        return false;
    }

    const u32 lane = (physical - kGifFifoBase) >> 2;
    fifo_words_[lane] = value;
    fifo_word_mask_ |= static_cast<u8>(1u << lane);

    if (fifo_word_mask_ == 0x0Fu) {
        const u64 lo =
            static_cast<u64>(fifo_words_[0]) |
            (static_cast<u64>(fifo_words_[1]) << 32);
        const u64 hi =
            static_cast<u64>(fifo_words_[2]) |
            (static_cast<u64>(fifo_words_[3]) << 32);
        fifo_word_mask_ = 0;
        write_gif_qword(lo, hi);
    }
    return true;
}

bool GsCore::write_gif_fifo64(u32 physical, u64 value) {
    if (physical != kGifFifoBase && physical != kGifFifoBase + 8u) {
        return false;
    }

    return write_gif_fifo32(physical, static_cast<u32>(value)) &&
           write_gif_fifo32(physical + 4u, static_cast<u32>(value >> 32));
}

void GsCore::begin_tag(u64 lo, u64 hi) {
    const u32 nloop = static_cast<u32>(lo & 0x7FFFu);
    gif_.eop = ((lo >> 15) & 1u) != 0;
    const bool pre = ((lo >> 46) & 1u) != 0;
    const u32 prim = static_cast<u32>((lo >> 47) & 0x7FFu);
    gif_.mode = static_cast<u32>((lo >> 58) & 0x3u);
    gif_.nreg = static_cast<u32>((lo >> 60) & 0xFu);
    if (gif_.nreg == 0) gif_.nreg = 16;
    gif_.reg_cursor = 0;
    gif_.regs = hi;
    gif_.values_remaining =
        (gif_.mode == kGifImage || gif_.mode == kGifImage2)
            ? nloop
            : nloop * gif_.nreg;
    gif_.active = gif_.values_remaining != 0;

    ++stats_.gif_tags;

    if (pre) {
        write_register(kRegPrim, prim);
    }

    if (!gif_.active) {
        finish_packet();
    }
}

void GsCore::finish_packet() {
    if (gif_.eop) {
        ++stats_.eop_packets;
    }
    gif_ = {};
}

void GsCore::process_packed(u32 descriptor, u64 lo, u64 hi) {
    ++stats_.packed_writes;

    switch (descriptor) {
    case 0x00: // PRIM
        write_register(kRegPrim, lo & 0x7FFu);
        return;
    case 0x01: { // RGBA
        const u64 old_q = registers_[kRegRgbaq] & 0xFFFFFFFF00000000ull;
        const u64 rgba =
            ((lo >> 0) & 0xFFu) |
            (((lo >> 32) & 0xFFu) << 8) |
            (((hi >> 0) & 0xFFu) << 16) |
            (((hi >> 32) & 0xFFu) << 24);
        write_register(kRegRgbaq, old_q | rgba);
        return;
    }
    case 0x02: { // STQ
        const u64 st =
            static_cast<u64>(static_cast<u32>(lo)) |
            (static_cast<u64>(static_cast<u32>(lo >> 32)) << 32);
        write_register(kRegSt, st);
        registers_[kRegRgbaq] =
            (registers_[kRegRgbaq] & 0xFFFFFFFFull) |
            (static_cast<u64>(static_cast<u32>(hi)) << 32);
        return;
    }
    case 0x03: { // UV
        const u64 uv =
            static_cast<u64>(static_cast<u32>(lo) & 0x3FFFu) |
            (static_cast<u64>(static_cast<u32>(lo >> 32) & 0x3FFFu) << 16);
        write_register(kRegUv, uv);
        return;
    }
    case 0x04: // XYZF2
    case 0x0C: { // XYZF3
        const u64 xyzf =
            static_cast<u64>(static_cast<u32>(lo) & 0xFFFFu) |
            (static_cast<u64>(static_cast<u32>(lo >> 32) & 0xFFFFu) << 16) |
            (static_cast<u64>(static_cast<u32>(hi) & 0x00FFFFFFu) << 32) |
            (static_cast<u64>((hi >> 36) & 0xFFu) << 56);
        write_register(descriptor == 0x04 ? kRegXyzf2 : kRegXyzf3, xyzf);
        return;
    }
    case 0x05: // XYZ2
    case 0x0D: { // XYZ3
        const u64 xyz =
            static_cast<u64>(static_cast<u32>(lo) & 0xFFFFu) |
            (static_cast<u64>(static_cast<u32>(lo >> 32) & 0xFFFFu) << 16) |
            (static_cast<u64>(static_cast<u32>(hi)) << 32);
        write_register(descriptor == 0x05 ? kRegXyz2 : kRegXyz3, xyz);
        return;
    }
    case 0x06: // TEX0_1
    case 0x07: // TEX0_2
    case 0x08: // CLAMP_1
    case 0x09: // CLAMP_2
    case 0x0A: // FOG
        write_register(descriptor, lo);
        return;
    case 0x0E: // A+D
        write_register(static_cast<u32>(hi & 0xFFu), lo);
        return;
    case 0x0F: // NOP
        return;
    default:
        ++stats_.unsupported_packed;
        return;
    }
}

void GsCore::process_reglist_value(u32 descriptor, u64 value) {
    ++stats_.reglist_writes;

    if (descriptor == 0x0Fu) {
        return;
    }

    // In REGLIST mode the 64-bit payload is already in the native GS register
    // layout. The descriptor numbers line up with the low GS register IDs.
    write_register(descriptor, value);
}

void GsCore::write_register(u32 address, u64 value) {
    address &= 0x7Fu;
    registers_[address] = value;
    ++stats_.register_writes;

    if (address == kRegPrim) {
        primitive_vertex_count_ = 0;
    } else if (address == kRegXyz2 || address == kRegXyz3 ||
               address == kRegXyzf2 || address == kRegXyzf3) {
        note_vertex_kick();
    }
}

void GsCore::note_vertex_kick() {
    ++stats_.vertices;
    ++primitive_vertex_count_;

    switch (current_prim()) {
    case 0: // point
        ++stats_.primitives;
        break;
    case 1: // line list
        if ((primitive_vertex_count_ % 2u) == 0) ++stats_.primitives;
        break;
    case 2: // line strip
        if (primitive_vertex_count_ >= 2u) ++stats_.primitives;
        break;
    case 3: // triangle list
        if ((primitive_vertex_count_ % 3u) == 0) ++stats_.primitives;
        break;
    case 4: // triangle strip
    case 5: // triangle fan
        if (primitive_vertex_count_ >= 3u) ++stats_.primitives;
        break;
    case 6: // sprite
        if ((primitive_vertex_count_ % 2u) == 0) ++stats_.primitives;
        break;
    default:
        break;
    }
}

void GsCore::write_gif_qword(u64 lo, u64 hi) {
    ++stats_.gif_qwords;

    if (!gif_.active) {
        begin_tag(lo, hi);
        return;
    }

    if (gif_.mode == kGifPacked) {
        const u32 descriptor = descriptor_at(gif_.regs, gif_.reg_cursor);
        process_packed(descriptor, lo, hi);
        ++gif_.reg_cursor;
        --gif_.values_remaining;
    } else if (gif_.mode == kGifReglist) {
        for (u32 half = 0; half < 2 && gif_.values_remaining != 0; ++half) {
            const u32 descriptor = descriptor_at(gif_.regs, gif_.reg_cursor);
            process_reglist_value(descriptor, half == 0 ? lo : hi);
            ++gif_.reg_cursor;
            --gif_.values_remaining;
        }
    } else {
        ++stats_.image_qwords;
        --gif_.values_remaining;
    }

    if (gif_.reg_cursor >= gif_.nreg) {
        gif_.reg_cursor %= gif_.nreg;
    }

    if (gif_.values_remaining == 0) {
        finish_packet();
    }
}

} // namespace ps2
