#include "core/gs/gs_core.h"

#include "core/gs/gs_privileged.h"

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
constexpr u32 kRegTex0_1 = 0x06;
constexpr u32 kRegClamp1 = 0x08;
constexpr u32 kRegXyoffset1 = 0x18;
constexpr u32 kRegPrmodecont = 0x1A;
constexpr u32 kRegPrmode = 0x1B;
constexpr u32 kRegTexclut = 0x1C;
constexpr u32 kRegScanmsk = 0x22;
constexpr u32 kRegTexa = 0x3B;
constexpr u32 kRegFogcol = 0x3D;
constexpr u32 kRegScissor1 = 0x40;
constexpr u32 kRegAlpha1 = 0x42;
constexpr u32 kRegDimx = 0x44;
constexpr u32 kRegDthe = 0x45;
constexpr u32 kRegColclamp = 0x46;
constexpr u32 kRegTest1 = 0x47;
constexpr u32 kRegPabe = 0x49;
constexpr u32 kRegFba1 = 0x4A;
constexpr u32 kRegFrame1 = 0x4C;
constexpr u32 kRegZbuf1 = 0x4E;
constexpr u32 kRegBitbltbuf = 0x50;
constexpr u32 kRegTrxpos = 0x51;
constexpr u32 kRegTrxreg = 0x52;
constexpr u32 kRegTrxdir = 0x53;
constexpr u32 kRegSignal = 0x60;
constexpr u32 kRegFinish = 0x61;
constexpr u32 kRegLabel = 0x62;

u32 descriptor_at(u64 regs, u32 cursor) {
    return static_cast<u32>((regs >> ((cursor & 0xFu) * 4u)) & 0xFu);
}

} // namespace

void GsCore::reset() {
    registers_.fill(0);
    fifo_words_.fill(0);
    fifo_word_mask_ = 0;
    gif_ = {};
    transfer_ = {};
    stats_ = {};
    vram_.reset();
    draw_vertices_.fill({});
    draw_vertex_count_ = 0;
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
        // Packed FOG stores F in bits 100..107 of the 128-bit payload.
        write_register(0x0A, static_cast<u64>((hi >> 36) & 0xFFu) << 56);
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

    if (address == kRegTrxdir) {
        const u32 xdir = static_cast<u32>(value & 0x3u);
        if (xdir == 0u) {
            begin_host_to_local();
        } else if (xdir == 1u) {
            begin_local_to_host();
        } else if (xdir == 2u) {
            transfer_ = {};
            execute_local_to_local();
        } else {
            transfer_ = {};
        }
    } else if (address == kRegSignal) {
        ++stats_.signal_events;
        if (privileged_ != nullptr) privileged_->signal(value);
    } else if (address == kRegFinish) {
        ++stats_.finish_events;
        if (privileged_ != nullptr) privileged_->finish();
    } else if (address == kRegLabel) {
        ++stats_.label_events;
        if (privileged_ != nullptr) privileged_->label(value);
    }

    if (address == kRegPrim) {
        draw_vertex_count_ = 0;
    } else if (address == kRegXyz2) {
        submit_vertex(value, false);
    } else if (address == kRegXyzf2) {
        submit_vertex(value, true);
    }
}


void GsCore::begin_host_to_local() {
    transfer_ = {};

    const u64 blit = registers_[kRegBitbltbuf];
    const u64 pos = registers_[kRegTrxpos];
    const u64 reg = registers_[kRegTrxreg];

    transfer_.bp = static_cast<u32>((blit >> 32) & 0x3FFFu);
    transfer_.bw = static_cast<u32>((blit >> 48) & 0x3Fu);
    transfer_.psm = static_cast<u32>((blit >> 56) & 0x3Fu);
    transfer_.dsax = static_cast<u32>((pos >> 32) & 0x7FFu);
    transfer_.dsay = static_cast<u32>((pos >> 48) & 0x7FFu);
    transfer_.diry = ((pos >> 59) & 1u) != 0;
    transfer_.dirx = ((pos >> 60) & 1u) != 0;
    transfer_.width = static_cast<u32>(reg & 0xFFFu);
    transfer_.height = static_cast<u32>((reg >> 32) & 0xFFFu);
    transfer_.total_pixels = transfer_.width * transfer_.height;

    if (transfer_.bw == 0 || transfer_.total_pixels == 0 ||
        !GsVram::supported_transfer_psm(transfer_.psm)) {
        ++stats_.unsupported_transfers;
        transfer_.active = false;
        return;
    }

    transfer_.active = true;
    ++stats_.host_to_local_transfers;
}


void GsCore::begin_local_to_host() {
    transfer_ = {};

    const u64 blit = registers_[kRegBitbltbuf];
    const u64 pos = registers_[kRegTrxpos];
    const u64 reg = registers_[kRegTrxreg];

    transfer_.local_to_host = true;
    transfer_.bp = static_cast<u32>(blit & 0x3FFFu);
    transfer_.bw = static_cast<u32>((blit >> 16) & 0x3Fu);
    transfer_.psm = static_cast<u32>((blit >> 24) & 0x3Fu);
    transfer_.dsax = static_cast<u32>(pos & 0x7FFu);
    transfer_.dsay = static_cast<u32>((pos >> 16) & 0x7FFu);
    transfer_.diry = ((pos >> 59) & 1u) != 0;
    transfer_.dirx = ((pos >> 60) & 1u) != 0;
    transfer_.width = static_cast<u32>(reg & 0xFFFu);
    transfer_.height = static_cast<u32>((reg >> 32) & 0xFFFu);
    transfer_.total_pixels = transfer_.width * transfer_.height;

    if (transfer_.bw == 0 || transfer_.total_pixels == 0 ||
        !GsVram::supported_transfer_psm(transfer_.psm)) {
        ++stats_.unsupported_transfers;
        transfer_.active = false;
        registers_[kRegTrxdir] =
            (registers_[kRegTrxdir] & ~0x3ull) | 0x3ull;
        return;
    }

    transfer_.active = true;
    ++stats_.local_to_host_transfers;
}

bool GsCore::read_local_to_host_qword(u64& lo, u64& hi) {
    lo = 0;
    hi = 0;

    if (!transfer_.active || !transfer_.local_to_host) {
        return false;
    }

    auto next_pixel = [&]() -> u32 {
        const u32 linear_x = transfer_.pixel_index % transfer_.width;
        const u32 linear_y = transfer_.pixel_index / transfer_.width;
        const u32 x = transfer_.dsax +
            (transfer_.dirx ? (transfer_.width - 1u - linear_x) : linear_x);
        const u32 y = transfer_.dsay +
            (transfer_.diry ? (transfer_.height - 1u - linear_y) : linear_y);
        const u32 value = vram_.read_transfer_pixel(
            transfer_.psm, x, y, transfer_.bp, transfer_.bw);
        ++transfer_.pixel_index;
        ++stats_.local_to_host_pixels;
        return value;
    };

    const bool four_bit =
        transfer_.psm == 20u || transfer_.psm == 36u ||
        transfer_.psm == 44u;

    while (transfer_.pending_size < 16u &&
           transfer_.pixel_index < transfer_.total_pixels) {
        if (four_bit) {
            const u8 low = static_cast<u8>(next_pixel() & 0x0Fu);
            u8 high = 0;
            if (transfer_.pixel_index < transfer_.total_pixels) {
                high = static_cast<u8>(next_pixel() & 0x0Fu);
            }
            transfer_.pending[transfer_.pending_size++] =
                static_cast<u8>(low | (high << 4));
            ++stats_.local_to_host_bytes;
            continue;
        }

        u32 bytes_per_pixel = 0;
        switch (transfer_.psm) {
        case 0:
        case 48:
            bytes_per_pixel = 4;
            break;
        case 1:
        case 49:
            bytes_per_pixel = 3;
            break;
        case 2:
        case 10:
        case 50:
        case 58:
            bytes_per_pixel = 2;
            break;
        case 19:
        case 27:
            bytes_per_pixel = 1;
            break;
        default:
            transfer_.active = false;
            ++stats_.unsupported_transfers;
            return false;
        }

        const u32 value = next_pixel();
        for (u32 i = 0; i < bytes_per_pixel; ++i) {
            transfer_.pending[transfer_.pending_size++] =
                static_cast<u8>(value >> (i * 8));
        }
        stats_.local_to_host_bytes += bytes_per_pixel;
    }

    if (transfer_.pending_size == 0) {
        transfer_.active = false;
        registers_[kRegTrxdir] =
            (registers_[kRegTrxdir] & ~0x3ull) | 0x3ull;
        return false;
    }

    const u32 emitted = transfer_.pending_size < 16u
        ? transfer_.pending_size
        : 16u;
    for (u32 i = 0; i < emitted; ++i) {
        if (i < 8u) {
            lo |= static_cast<u64>(transfer_.pending[i]) << (i * 8);
        } else {
            hi |= static_cast<u64>(transfer_.pending[i]) << ((i - 8u) * 8);
        }
    }

    const u32 remaining = transfer_.pending_size - emitted;
    for (u32 i = 0; i < remaining; ++i) {
        transfer_.pending[i] = transfer_.pending[emitted + i];
    }
    transfer_.pending_size = remaining;
    ++stats_.local_to_host_qwords;

    if (transfer_.pixel_index >= transfer_.total_pixels &&
        transfer_.pending_size == 0) {
        transfer_.active = false;
        registers_[kRegTrxdir] =
            (registers_[kRegTrxdir] & ~0x3ull) | 0x3ull;
    }

    return true;
}


void GsCore::execute_local_to_local() {
    const u64 blit = registers_[kRegBitbltbuf];
    const u64 pos = registers_[kRegTrxpos];
    const u64 reg = registers_[kRegTrxreg];

    const u32 sbp = static_cast<u32>(blit & 0x3FFFu);
    const u32 sbw = static_cast<u32>((blit >> 16) & 0x3Fu);
    const u32 spsm = static_cast<u32>((blit >> 24) & 0x3Fu);
    const u32 dbp = static_cast<u32>((blit >> 32) & 0x3FFFu);
    const u32 dbw = static_cast<u32>((blit >> 48) & 0x3Fu);
    const u32 dpsm = static_cast<u32>((blit >> 56) & 0x3Fu);

    const u32 ssax = static_cast<u32>(pos & 0x7FFu);
    const u32 ssay = static_cast<u32>((pos >> 16) & 0x7FFu);
    const u32 dsax = static_cast<u32>((pos >> 32) & 0x7FFu);
    const u32 dsay = static_cast<u32>((pos >> 48) & 0x7FFu);
    const bool diry = ((pos >> 59) & 1u) != 0;
    const bool dirx = ((pos >> 60) & 1u) != 0;

    const u32 width = static_cast<u32>(reg & 0xFFFu);
    const u32 height = static_cast<u32>((reg >> 32) & 0xFFFu);
    const u64 total = static_cast<u64>(width) * height;

    const u32 src_bpp = GsVram::transfer_bpp(spsm);
    const u32 dst_bpp = GsVram::transfer_bpp(dpsm);
    if (sbw == 0 || dbw == 0 || total == 0 ||
        !GsVram::supported_transfer_psm(spsm) ||
        !GsVram::supported_transfer_psm(dpsm) ||
        src_bpp == 0 || src_bpp != dst_bpp) {
        ++stats_.unsupported_transfers;
        registers_[kRegTrxdir] =
            (registers_[kRegTrxdir] & ~0x3ull) | 0x3ull;
        return;
    }

    ++stats_.local_to_local_transfers;

    for (u32 linear_y = 0; linear_y < height; ++linear_y) {
        const u32 row = diry ? (height - 1u - linear_y) : linear_y;
        for (u32 linear_x = 0; linear_x < width; ++linear_x) {
            const u32 column = dirx ? (width - 1u - linear_x) : linear_x;
            const u32 sx = ssax + column;
            const u32 sy = ssay + row;
            const u32 dx = dsax + column;
            const u32 dy = dsay + row;

            const u32 value = vram_.read_transfer_pixel(
                spsm, sx, sy, sbp, sbw);
            if (!vram_.write_transfer_pixel(
                    dpsm, dx, dy, dbp, dbw, value)) {
                ++stats_.unsupported_transfers;
                registers_[kRegTrxdir] =
                    (registers_[kRegTrxdir] & ~0x3ull) | 0x3ull;
                return;
            }
            ++stats_.local_to_local_pixels;
        }
    }

    registers_[kRegTrxdir] =
        (registers_[kRegTrxdir] & ~0x3ull) | 0x3ull;
}

void GsCore::consume_image_qword(u64 lo, u64 hi) {
    ++stats_.image_qwords;
    stats_.image_bytes += 16;

    if (!transfer_.active) return;

    const u8 bytes[16] = {
        static_cast<u8>(lo), static_cast<u8>(lo >> 8),
        static_cast<u8>(lo >> 16), static_cast<u8>(lo >> 24),
        static_cast<u8>(lo >> 32), static_cast<u8>(lo >> 40),
        static_cast<u8>(lo >> 48), static_cast<u8>(lo >> 56),
        static_cast<u8>(hi), static_cast<u8>(hi >> 8),
        static_cast<u8>(hi >> 16), static_cast<u8>(hi >> 24),
        static_cast<u8>(hi >> 32), static_cast<u8>(hi >> 40),
        static_cast<u8>(hi >> 48), static_cast<u8>(hi >> 56),
    };

    for (u32 i = 0; i < 16; ++i) {
        if (transfer_.pending_size < transfer_.pending.size()) {
            transfer_.pending[transfer_.pending_size++] = bytes[i];
        }
    }

    consume_pending_pixels();
}

void GsCore::consume_pending_pixels() {
    auto store_pixel = [&](u32 value) {
        const u32 linear_x = transfer_.pixel_index % transfer_.width;
        const u32 linear_y = transfer_.pixel_index / transfer_.width;
        const u32 x = transfer_.dsax +
            (transfer_.dirx ? (transfer_.width - 1u - linear_x) : linear_x);
        const u32 y = transfer_.dsay +
            (transfer_.diry ? (transfer_.height - 1u - linear_y) : linear_y);

        const bool stored = vram_.write_transfer_pixel(
            transfer_.psm, x, y, transfer_.bp, transfer_.bw, value);
        if (!stored) {
            transfer_.active = false;
            ++stats_.unsupported_transfers;
            return;
        }

        ++transfer_.pixel_index;
        ++stats_.host_to_local_pixels;
        if (transfer_.pixel_index >= transfer_.total_pixels) {
            transfer_.active = false;
            registers_[kRegTrxdir] =
                (registers_[kRegTrxdir] & ~0x3ull) | 0x3ull;
        }
    };

    u32 consumed = 0;

    if (transfer_.psm == 20u || transfer_.psm == 36u ||
        transfer_.psm == 44u) {
        while (transfer_.active && consumed < transfer_.pending_size) {
            const u8 packed = transfer_.pending[consumed++];
            store_pixel(packed & 0x0Fu);
            if (transfer_.active) store_pixel(packed >> 4);
        }
    } else {
        u32 bytes_per_pixel = 0;
        switch (transfer_.psm) {
        case 0: // PSMCT32
        case 48: bytes_per_pixel = 4; break; // PSMZ32
        case 1: // PSMCT24
        case 49: bytes_per_pixel = 3; break; // PSMZ24
        case 2: // PSMCT16
        case 10: // PSMCT16S
        case 50: // PSMZ16
        case 58: bytes_per_pixel = 2; break; // PSMZ16S
        case 19: // PSMT8
        case 27: bytes_per_pixel = 1; break; // PSMT8H
        default: return;
        }

        while (transfer_.active &&
               transfer_.pending_size - consumed >= bytes_per_pixel) {
            u32 value = 0;
            for (u32 i = 0; i < bytes_per_pixel; ++i) {
                value |= static_cast<u32>(
                    transfer_.pending[consumed + i]) << (i * 8);
            }
            consumed += bytes_per_pixel;
            store_pixel(value);
        }
    }

    if (!transfer_.active) {
        transfer_.pending_size = 0;
        return;
    }

    if (consumed != 0) {
        const u32 remaining = transfer_.pending_size - consumed;
        for (u32 i = 0; i < remaining; ++i) {
            transfer_.pending[i] = transfer_.pending[consumed + i];
        }
        transfer_.pending_size = remaining;
    }
}

u64 GsCore::effective_prim() const {
    const u64 prim = registers_[kRegPrim] & 0x7FFu;
    if ((registers_[kRegPrmodecont] & 1u) != 0) return prim;
    return (prim & 0x7u) | (registers_[kRegPrmode] & 0x7F8u);
}

GsRasterContext GsCore::raster_context() const {
    const u64 prim = effective_prim();
    const u32 ctxt = static_cast<u32>((prim >> 9) & 1u);
    const u64 scissor = registers_[kRegScissor1 + ctxt];
    const u64 alpha = registers_[kRegAlpha1 + ctxt];
    const u64 test = registers_[kRegTest1 + ctxt];
    const u64 frame = registers_[kRegFrame1 + ctxt];
    const u64 zbuf = registers_[kRegZbuf1 + ctxt];

    GsRasterContext ctx{};
    ctx.fbp = static_cast<u32>(frame & 0x1FFu) << 5;
    ctx.fbw = static_cast<u32>((frame >> 16) & 0x3Fu);
    ctx.psm = static_cast<u32>((frame >> 24) & 0x3Fu);
    ctx.fbmask = static_cast<u32>(frame >> 32);
    ctx.scax0 = static_cast<s32>(scissor & 0x7FFu);
    ctx.scax1 = static_cast<s32>((scissor >> 16) & 0x7FFu);
    ctx.scay0 = static_cast<s32>((scissor >> 32) & 0x7FFu);
    ctx.scay1 = static_cast<s32>((scissor >> 48) & 0x7FFu);

    ctx.gouraud = (prim & (1ull << 3)) != 0;
    ctx.alpha_blend = (prim & (1ull << 6)) != 0;

    ctx.ate = (test & 1u) != 0;
    ctx.atst = static_cast<u32>((test >> 1) & 0x7u);
    ctx.aref = static_cast<u32>((test >> 4) & 0xFFu);
    ctx.afail = static_cast<u32>((test >> 12) & 0x3u);
    ctx.date = ((test >> 14) & 1u) != 0;
    ctx.datm = ((test >> 15) & 1u) != 0;
    ctx.zte = ((test >> 16) & 1u) != 0;
    ctx.ztst = static_cast<u32>((test >> 17) & 0x3u);

    ctx.zbp = static_cast<u32>(zbuf & 0x1FFu) << 5;
    ctx.zpsm = static_cast<u32>((zbuf >> 24) & 0x3Fu);
    ctx.zmask = ((zbuf >> 32) & 1u) != 0;

    ctx.alpha_a = static_cast<u32>(alpha & 0x3u);
    ctx.alpha_b = static_cast<u32>((alpha >> 2) & 0x3u);
    ctx.alpha_c = static_cast<u32>((alpha >> 4) & 0x3u);
    ctx.alpha_d = static_cast<u32>((alpha >> 6) & 0x3u);
    ctx.alpha_fix = static_cast<u32>((alpha >> 32) & 0xFFu);
    ctx.pabe = (registers_[kRegPabe] & 1u) != 0;
    ctx.fba = (registers_[kRegFba1 + ctxt] & 1u) != 0;
    ctx.color_clamp = (registers_[kRegColclamp] & 1u) != 0;
    ctx.fog_enabled = (prim & (1ull << 5)) != 0;
    ctx.fog_color = static_cast<u32>(registers_[kRegFogcol]) & 0x00FFFFFFu;
    ctx.scanmask = static_cast<u32>(registers_[kRegScanmsk]) & 0x3u;
    ctx.dither = (registers_[kRegDthe] & 1u) != 0;
    ctx.dimx = registers_[kRegDimx];

    if ((prim & (1ull << 4)) != 0) {
        const u64 tex0 = registers_[kRegTex0_1 + ctxt];
        const u64 clamp = registers_[kRegClamp1 + ctxt];
        auto& texture = ctx.texture;
        texture.enabled = true;
        texture.bp = static_cast<u32>(tex0 & 0x3FFFu);
        texture.bw = static_cast<u32>((tex0 >> 14) & 0x3Fu);
        texture.psm = static_cast<u32>((tex0 >> 20) & 0x3Fu);
        const u32 tw = static_cast<u32>((tex0 >> 26) & 0xFu);
        const u32 th = static_cast<u32>((tex0 >> 30) & 0xFu);
        texture.width = tw < 31u ? (1u << tw) : 0u;
        texture.height = th < 31u ? (1u << th) : 0u;
        texture.tcc = ((tex0 >> 34) & 1u) != 0;
        texture.tfx = static_cast<u32>((tex0 >> 35) & 0x3u);
        texture.fst = (prim & (1ull << 8)) != 0;

        texture.cbp = static_cast<u32>((tex0 >> 37) & 0x3FFFu);
        texture.cpsm = static_cast<u32>((tex0 >> 51) & 0xFu);
        texture.csm2 = ((tex0 >> 55) & 1u) != 0;
        texture.csa = static_cast<u32>((tex0 >> 56) & 0x1Fu);

        const u64 texclut = registers_[kRegTexclut];
        texture.clut_bw = static_cast<u32>(texclut & 0x3Fu);
        texture.clut_u = static_cast<u32>((texclut >> 6) & 0x3Fu);
        texture.clut_v = static_cast<u32>((texclut >> 12) & 0x3FFu);

        const u64 texa = registers_[kRegTexa];
        texture.ta0 = static_cast<u32>(texa & 0xFFu);
        texture.aem = ((texa >> 15) & 1u) != 0;
        texture.ta1 = static_cast<u32>((texa >> 32) & 0xFFu);

        texture.wms = static_cast<u32>(clamp & 0x3u);
        texture.wmt = static_cast<u32>((clamp >> 2) & 0x3u);
        texture.minu = static_cast<u32>((clamp >> 4) & 0x3FFu);
        texture.maxu = static_cast<u32>((clamp >> 14) & 0x3FFu);
        texture.minv = static_cast<u32>((clamp >> 24) & 0x3FFu);
        texture.maxv = static_cast<u32>((clamp >> 34) & 0x3FFu);
    }

    return ctx;
}

bool GsCore::raster_state_supported() const {
    const u64 prim = effective_prim();

    // AA1 coverage is still an explicit skip. Fog, Gouraud, alpha blending,
    // alpha/destination tests and Z buffering are handled by the software
    // pixel pipeline.
    constexpr u64 kUnsupportedPrim =
        (1ull << 7);  // AA1
    if ((prim & kUnsupportedPrim) != 0) return false;

    const GsRasterContext ctx = raster_context();
    if (!GsRasterizer::supported_target(ctx)) return false;

    if (ctx.texture.enabled) {
        if (!GsRasterizer::supported_texture(ctx.texture)) return false;
    }

    return true;
}

void GsCore::emit_primitive(
    const GsRasterVertex& a,
    const GsRasterVertex& b,
    const GsRasterVertex& c,
    u32 vertex_count) {
    ++stats_.primitives;

    const u32 prim = static_cast<u32>(effective_prim() & 0x7u);
    if (!raster_state_supported()) {
        ++stats_.skipped_raster_draws;
        return;
    }

    const GsRasterContext ctx = raster_context();
    u64 pixels = 0;
    if (prim == 0u && vertex_count >= 1u) {
        pixels = GsRasterizer::draw_point(vram_, ctx, a);
    } else if ((prim == 1u || prim == 2u) && vertex_count >= 2u) {
        pixels = GsRasterizer::draw_line(vram_, ctx, a, b);
    } else if (prim == 6u && vertex_count >= 2u) {
        pixels = GsRasterizer::draw_sprite(vram_, ctx, a, b);
    } else if ((prim == 3u || prim == 4u || prim == 5u) && vertex_count >= 3u) {
        pixels = GsRasterizer::draw_triangle(vram_, ctx, a, b, c);
    } else {
        ++stats_.skipped_raster_draws;
        return;
    }

    ++stats_.raster_draws;
    stats_.raster_pixels += pixels;
    if (ctx.texture.enabled) {
        ++stats_.textured_raster_draws;
        stats_.texture_samples += pixels;
    }
}

void GsCore::submit_vertex(u64 xyz, bool xyzf) {
    ++stats_.vertices;

    const u64 prim_reg = effective_prim();
    const u32 prim = static_cast<u32>(prim_reg & 0x7u);
    const u32 ctxt = static_cast<u32>((prim_reg >> 9) & 1u);
    const u64 xyoffset = registers_[kRegXyoffset1 + ctxt];

    GsRasterVertex v{};
    v.x = static_cast<s32>(static_cast<u32>(xyz) & 0xFFFFu) -
          static_cast<s32>(static_cast<u32>(xyoffset) & 0xFFFFu);
    v.y = static_cast<s32>((static_cast<u32>(xyz) >> 16) & 0xFFFFu) -
          static_cast<s32>(static_cast<u32>(xyoffset >> 32) & 0xFFFFu);
    v.z = xyzf
        ? static_cast<u32>((xyz >> 32) & 0x00FFFFFFu)
        : static_cast<u32>(xyz >> 32);
    v.fog = xyzf
        ? static_cast<u32>((xyz >> 56) & 0xFFu)
        : static_cast<u32>((registers_[0x0A] >> 56) & 0xFFu);
    v.rgba = static_cast<u32>(registers_[kRegRgbaq]);
    const u64 uv = registers_[kRegUv];
    v.u = static_cast<s32>(uv & 0x3FFFu);
    v.v = static_cast<s32>((uv >> 16) & 0x3FFFu);
    const u64 st = registers_[kRegSt];
    v.s = std::bit_cast<float>(static_cast<u32>(st));
    v.t = std::bit_cast<float>(static_cast<u32>(st >> 32));
    v.q = std::bit_cast<float>(static_cast<u32>(registers_[kRegRgbaq] >> 32));

    switch (prim) {
    case 0: // point
        draw_vertices_[0] = v;
        emit_primitive(v, {}, {}, 1);
        draw_vertex_count_ = 0;
        break;
    case 1: // line list
        draw_vertices_[draw_vertex_count_++] = v;
        if (draw_vertex_count_ == 2) {
            emit_primitive(draw_vertices_[0], draw_vertices_[1], {}, 2);
            draw_vertex_count_ = 0;
        }
        break;
    case 2: // line strip
        if (draw_vertex_count_ == 0) {
            draw_vertices_[0] = v;
            draw_vertex_count_ = 1;
        } else {
            draw_vertices_[1] = v;
            emit_primitive(draw_vertices_[0], draw_vertices_[1], {}, 2);
            draw_vertices_[0] = draw_vertices_[1];
            draw_vertex_count_ = 1;
        }
        break;
    case 3: // triangle list
        draw_vertices_[draw_vertex_count_++] = v;
        if (draw_vertex_count_ == 3) {
            emit_primitive(draw_vertices_[0], draw_vertices_[1], draw_vertices_[2], 3);
            draw_vertex_count_ = 0;
        }
        break;
    case 4: // triangle strip
        draw_vertices_[draw_vertex_count_++] = v;
        if (draw_vertex_count_ == 3) {
            emit_primitive(draw_vertices_[0], draw_vertices_[1], draw_vertices_[2], 3);
            draw_vertices_[0] = draw_vertices_[1];
            draw_vertices_[1] = draw_vertices_[2];
            draw_vertex_count_ = 2;
        }
        break;
    case 5: // triangle fan
        draw_vertices_[draw_vertex_count_++] = v;
        if (draw_vertex_count_ == 3) {
            emit_primitive(draw_vertices_[0], draw_vertices_[1], draw_vertices_[2], 3);
            draw_vertices_[1] = draw_vertices_[2];
            draw_vertex_count_ = 2;
        }
        break;
    case 6: // sprite
        draw_vertices_[draw_vertex_count_++] = v;
        if (draw_vertex_count_ == 2) {
            emit_primitive(draw_vertices_[0], draw_vertices_[1], {}, 2);
            draw_vertex_count_ = 0;
        }
        break;
    default:
        ++stats_.skipped_raster_draws;
        draw_vertex_count_ = 0;
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
        consume_image_qword(lo, hi);
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
