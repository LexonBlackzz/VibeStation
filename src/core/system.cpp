#include "system.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
    // PSX-SPX vertical refresh rates (native region clock):
    // NTSC interlaced     ~59.940 Hz
    // NTSC non-interlaced ~59.826 Hz
    // PAL  interlaced     ~50.000 Hz
    // PAL  non-interlaced ~49.761 Hz
    constexpr double kNtscInterlacedFps = 60000.0 / 1001.0;
    constexpr double kNtscProgressiveFps = 59.826;
    constexpr double kPalInterlacedFps = 50.0;
    constexpr double kPalProgressiveFps = 49.761;

    bool is_menu_idle_pc(u32 pc) {
        return ((pc >= 0x80059D00 && pc <= 0x80059FFF) ||
            (pc >= 0xA0059D00 && pc <= 0xA0059FFF) ||
            (pc >= 0x8003D700 && pc <= 0x8003D8FF) ||
            (pc >= 0xA003D700 && pc <= 0xA003D8FF));
    }

    bool is_bios_rom_pc(u32 pc) {
        return (pc >= 0xBFC00000 && pc < 0xBFC80000);
    }

    bool is_bios_ram_pc(u32 pc) {
        return ((pc >= 0x80000000 && pc < 0x80080000) ||
            (pc >= 0xA0000000 && pc < 0xA0080000));
    }

    bool is_non_bios_pc(u32 pc) { return !is_bios_rom_pc(pc) && !is_bios_ram_pc(pc); }
} // namespace

// Timer IRQ helper
void timer_fire_irq(System* sys, int index) {
    Interrupt irqs[] = { Interrupt::Timer0, Interrupt::Timer1, Interrupt::Timer2 };
    if (index >= 0 && index < 3) {
        sys->irq().request(irqs[index]);
    }
}

void System::note_cdrom_io(u32 phys_addr) {
    ++boot_diag_.cd_io_count;
    if (!boot_diag_.saw_cd_io) {
        boot_diag_.saw_cd_io = true;
        boot_diag_.first_cd_io_cycle = cpu_.cycle_count();
        boot_diag_.first_cd_io_addr = phys_addr;
    }
}

void System::note_sio_io(u32 phys_addr) {
    ++boot_diag_.sio_io_count;
    if (!boot_diag_.saw_sio_io) {
        boot_diag_.saw_sio_io = true;
        boot_diag_.first_sio_io_cycle = cpu_.cycle_count();
        boot_diag_.first_sio_io_addr = phys_addr;
    }
}

void System::sync_spu_to_cpu() {
    const u64 target_cycle = cpu_.cycle_count();
    if (target_cycle <= spu_synced_cpu_cycle_) {
        spu_.mark_synced_to_cpu(spu_synced_cpu_cycle_);
        return;
    }

    u64 delta = target_cycle - spu_synced_cpu_cycle_;
    while (delta > 0) {
        const u32 step =
            (delta > static_cast<u64>(std::numeric_limits<u32>::max()))
            ? std::numeric_limits<u32>::max()
            : static_cast<u32>(delta);
        spu_.tick(step);
        delta -= step;
    }
    spu_synced_cpu_cycle_ = target_cycle;
    spu_.mark_synced_to_cpu(spu_synced_cpu_cycle_);
}

void System::init_hardware() {
    if (hw_init_)
        return; // Already initialized

    irq_.init(this);
    timers_.init(this);
    spu_.init(this);
    sio_.init(this);
    cdrom_.init(this);
    dma_.init(this);
    gpu_.init(this);
    cpu_.init(this);

    hw_init_ = true;
    printf("[System] Hardware initialized\n");
    fflush(stdout);
}

bool System::load_bios(const std::string& path) {
    // Initialize hardware on first BIOS load
    if (!hw_init_) {
        init_hardware();
    }
    return bios_.load(path);
}

bool System::load_game(const std::string& bin_path,
    const std::string& cue_path) {
    const bool ok = cdrom_.load_bin_cue(bin_path, cue_path);
    if (ok) {
        last_disc_bin_path_ = bin_path;
        last_disc_cue_path_ = cue_path;
    }
    return ok;
}

bool System::boot_disc() {
    if (!bios_loaded()) {
        return false;
    }

    const std::string bin = last_disc_bin_path_;
    const std::string cue = last_disc_cue_path_;

    set_running(false);
    reset();

    if (!cue.empty()) {
        if (!load_game(bin, cue)) {
            return false;
        }
    }

    if (!disc_loaded()) {
        return false;
    }

    set_running(true);
    return true;
}

void System::reset() {
    if (!hw_init_) {
        init_hardware();
    }
    irq_.reset();
    timers_.reset();
    dma_.reset();
    cdrom_.reset();
    mdec_.reset();
    sio_.reset();
    gpu_.reset();
    spu_.reset();
    cpu_.reset();
    spu_synced_cpu_cycle_ = cpu_.cycle_count();
    spu_.mark_synced_to_cpu(spu_synced_cpu_cycle_);
    ram_.reset();
    frame_cycles_ = 0;
    frame_cycle_remainder_ = 0.0;
    boot_diag_ = {};
    saw_non_bios_exec_ = false;
    bios_menu_streak_after_non_bios_ = 0;
    std::memset(mem_ctrl_, 0, sizeof(mem_ctrl_));
    ram_size_ = 0;
    cache_ctrl_ = 0;
    post_reg_ = 0;
}

void System::shutdown() { spu_.shutdown(); }

double System::target_fps() const {
    const DisplayMode& mode = gpu_.display_mode();
    const bool effective_interlaced = mode.interlaced && (mode.vres != 0);
    if (mode.is_pal) {
        return effective_interlaced ? kPalInterlacedFps : kPalProgressiveFps;
    }
    return effective_interlaced ? kNtscInterlacedFps : kNtscProgressiveFps;
}

void System::run_frame(bool sample_display_diag) {
    auto start_total = std::chrono::high_resolution_clock::now();
    reset_profiling_stats();
    const bool profile_detailed = g_profile_detailed_timing;

    const bool pal = gpu_.display_mode().is_pal;
    const u32 scanlines_per_frame = pal ? 314 : 263;
    const u32 vblank_scanline = pal ? 288 : 240;
    const double fps = target_fps();
    const double cycles_exact = static_cast<double>(psx::CPU_CLOCK_HZ) / fps;
    frame_cycle_remainder_ += cycles_exact;
    const u32 cycles_per_frame = std::max<u32>(
        1u, static_cast<u32>(std::floor(frame_cycle_remainder_)));
    frame_cycle_remainder_ -= static_cast<double>(cycles_per_frame);
    const u32 base_cycles_per_scanline = cycles_per_frame / scanlines_per_frame;
    const u32 extra_cycles_per_frame = cycles_per_frame % scanlines_per_frame;
    static constexpr u32 kCpuSliceCycles = 32;
    static constexpr u32 kDmaTickStride = 16;
    u32 extra_cycle_error = 0;
    u32 dma_tick_budget = 0;

    for (u32 scanline = 0; scanline < scanlines_per_frame; scanline++) {
        u32 cycles_this_scanline = base_cycles_per_scanline;
        extra_cycle_error += extra_cycles_per_frame;
        if (extra_cycle_error >= scanlines_per_frame) {
            ++cycles_this_scanline;
            extra_cycle_error -= scanlines_per_frame;
        }

        const bool in_vblank = scanline >= vblank_scanline;
        timers_.set_vblank(in_vblank);

        std::chrono::high_resolution_clock::time_point start_loop{};
        if (profile_detailed) {
            start_loop = std::chrono::high_resolution_clock::now();
        }
        u32 cycles_remaining = cycles_this_scanline;
        while (cycles_remaining > 0) {
            const u32 slice = std::min(cycles_remaining, kCpuSliceCycles);
            for (u32 c = 0; c < slice; ++c) {
                cpu_.step();
                ++frame_cycles_;
            }
            cycles_remaining -= slice;

            dma_tick_budget += slice;
            while (dma_tick_budget >= kDmaTickStride) {
                dma_.tick();
                dma_tick_budget -= kDmaTickStride;
            }
        }
        if (dma_tick_budget > 0) {
            dma_.tick();
            dma_tick_budget = 0;
        }
        if (profile_detailed) {
            const auto end_loop = std::chrono::high_resolution_clock::now();
            add_cpu_time(
                std::chrono::duration<double, std::milli>(end_loop - start_loop)
                .count());
        }

        sio_.tick(cycles_this_scanline);

        // Batch devices that already accept cycle counts to reduce per-cycle call
        // overhead.
        std::chrono::high_resolution_clock::time_point start_timers{};
        if (profile_detailed) {
            start_timers = std::chrono::high_resolution_clock::now();
        }
        timers_.tick(cycles_this_scanline);
        timers_.hblank_pulse();
        if (profile_detailed) {
            const auto end_timers = std::chrono::high_resolution_clock::now();
            add_timers_time(
                std::chrono::duration<double, std::milli>(end_timers - start_timers)
                .count());
        }

        cdrom_.tick(cycles_this_scanline);
        sync_spu_to_cpu();
        if (!boot_diag_.saw_pad_cmd42 && sio_.saw_pad_cmd42()) {
            boot_diag_.saw_pad_cmd42 = true;
        }
        if (!boot_diag_.saw_tx_cmd42 && sio_.saw_tx_cmd42()) {
            boot_diag_.saw_tx_cmd42 = true;
        }
        if (!boot_diag_.saw_pad_id && sio_.saw_pad_id()) {
            boot_diag_.saw_pad_id = true;
        }
        if (!boot_diag_.saw_pad_button && sio_.saw_non_ff_button_byte()) {
            boot_diag_.saw_pad_button = true;
        }
        if (!boot_diag_.saw_full_pad_poll && sio_.saw_full_pad_poll()) {
            boot_diag_.saw_full_pad_poll = true;
        }
        boot_diag_.pad_cmd42_count = sio_.pad_cmd42_count();
        boot_diag_.pad_poll_count = sio_.pad_poll_count();
        boot_diag_.pad_packet_count = sio_.pad_packet_count();
        boot_diag_.ch0_poll_count = sio_.channel0_poll_count();
        boot_diag_.ch1_poll_count = sio_.channel1_poll_count();
        boot_diag_.sio_invalid_seq_count = sio_.invalid_sequence_count();
        boot_diag_.last_pad_buttons = sio_.last_pad_buttons();
        boot_diag_.last_sio_tx = sio_.last_tx_byte();
        boot_diag_.last_sio_rx = sio_.last_rx_byte();
        boot_diag_.last_joy_stat = sio_.joy_stat_snapshot();
        boot_diag_.last_joy_ctrl = sio_.joy_ctrl_snapshot();
        boot_diag_.sio_irq_assert_count = sio_.irq_assert_count();
        boot_diag_.sio_irq_ack_count = sio_.irq_ack_count();

        if (!boot_diag_.saw_cd_read_cmd && cdrom_.saw_read_command()) {
            boot_diag_.saw_cd_read_cmd = true;
        }
        if (!boot_diag_.saw_cd_sector_visible && cdrom_.saw_sector_visible()) {
            boot_diag_.saw_cd_sector_visible = true;
        }
        if (!boot_diag_.saw_cd_getid && cdrom_.saw_getid()) {
            boot_diag_.saw_cd_getid = true;
        }
        if (!boot_diag_.saw_cd_setloc && cdrom_.saw_setloc()) {
            boot_diag_.saw_cd_setloc = true;
        }
        if (!boot_diag_.saw_cd_seekl && cdrom_.saw_seekl()) {
            boot_diag_.saw_cd_seekl = true;
        }
        if (!boot_diag_.saw_cd_readn_or_reads && cdrom_.saw_readn_or_reads()) {
            boot_diag_.saw_cd_readn_or_reads = true;
        }
        boot_diag_.cd_read_command_count = cdrom_.read_command_count();
        boot_diag_.cd_irq_int1_count = cdrom_.irq_int1_count();
        boot_diag_.cd_irq_int3_count = cdrom_.irq_int3_count();

        if (scanline == vblank_scanline) {
            gpu_.vblank();
            irq_.request(Interrupt::VBlank);
        }
    }
    timers_.set_vblank(false);
    ++boot_diag_.frame_counter;

    const u32 pc = cpu_.pc();
    if (is_non_bios_pc(pc)) {
        saw_non_bios_exec_ = true;
        bios_menu_streak_after_non_bios_ = 0;
    }
    if (saw_non_bios_exec_ && is_menu_idle_pc(pc)) {
        ++bios_menu_streak_after_non_bios_;
        // Treat fallback as a persistent return to BIOS/menu idle, not a brief hop.
        if (bios_menu_streak_after_non_bios_ >= 120) {
            boot_diag_.fell_back_to_bios_after_non_bios = true;
        }
    }
    else if (!is_non_bios_pc(pc)) {
        bios_menu_streak_after_non_bios_ = 0;
    }

    if (sample_display_diag) {
        const DisplaySampleInfo display_sample = gpu_.build_display_rgba(nullptr);
        update_display_diag(display_sample);
    }

    auto end_total = std::chrono::high_resolution_clock::now();
    set_total_time(
        std::chrono::duration<double, std::milli>(end_total - start_total)
        .count());
}

void System::update_display_diag(const DisplaySampleInfo& display_sample) {
    boot_diag_.display_hash = display_sample.hash;
    boot_diag_.display_non_black_pixels = display_sample.non_black_pixels;
    boot_diag_.display_width = static_cast<u16>(std::max(0, display_sample.width));
    boot_diag_.display_height =
        static_cast<u16>(std::max(0, display_sample.height));
    boot_diag_.display_x_start =
        static_cast<u16>(std::max(0, display_sample.x_start));
    boot_diag_.display_y_start =
        static_cast<u16>(std::max(0, display_sample.y_start));
    boot_diag_.display_is_24bit = display_sample.is_24bit ? 1u : 0u;
    boot_diag_.display_enabled = display_sample.display_enabled ? 1u : 0u;
    const u64 pixel_count = static_cast<u64>(std::max(1, display_sample.width)) *
        static_cast<u64>(std::max(1, display_sample.height));
    const bool logo_visible_now =
        display_sample.display_enabled &&
        (display_sample.non_black_pixels > (pixel_count / 16u));
    if (logo_visible_now && saw_non_bios_exec_) {
        boot_diag_.saw_logo_present = true;
        ++boot_diag_.logo_visible_run_frames;
        if (boot_diag_.logo_visible_run_frames >= 120) {
            boot_diag_.logo_visible_persisted = true;
        }
    }
    else {
        if (boot_diag_.saw_logo_present &&
            boot_diag_.first_black_after_logo_frame < 0) {
            boot_diag_.first_black_after_logo_frame =
                static_cast<s32>(boot_diag_.frame_counter);
        }
        boot_diag_.logo_visible_run_frames = 0;
    }
}

void System::step() {
    cpu_.step();
    sync_spu_to_cpu();
}

void System::spu_dma_write(u32 val) {
    sync_spu_to_cpu();
    spu_.dma_write(val);
}

u32 System::spu_dma_read() {
    sync_spu_to_cpu();
    return spu_.dma_read();
}

// ── Memory Bus ─────────────────────────────────────────────────────
// The PS1 memory map translated to hardware component dispatches.

u8 System::read8(u32 addr) {
    u32 phys = psx::mask_address(addr);
    if (g_trace_bus && phys >= 0x1F801000 && phys < 0x1F803000) {
        static u64 bus_r8_count = 0;
        if (trace_should_log(bus_r8_count, g_trace_burst_bus, g_trace_stride_bus)) {
            LOG_DEBUG("BUS: R8  addr=0x%08X phys=0x%08X", addr, phys);
        }
    }

    // Main RAM (2MB) mirrored across the low physical region.
    if (phys < 0x1F000000)
        return ram_.read8(phys & 0x1FFFFF);

    // BIOS
    if (phys >= psx::BIOS_BASE &&
        static_cast<u64>(phys) <
        (static_cast<u64>(psx::BIOS_BASE) + bios_.mapped_size()))
        return bios_.read8(phys - psx::BIOS_BASE);

    // Scratchpad
    if (phys >= 0x1F800000 && phys < 0x1F800400)
        return ram_.scratch_read8(phys - 0x1F800000);

    // I/O Ports
    if (phys >= 0x1F801000 && phys < 0x1F803000) {
        u32 io = phys - 0x1F801000;
        // Unused timer slot / peripheral gap (e.g. 1F801130h..13Fh): open bus.
        if (io >= 0x130 && io < 0x140) {
            return 0xFF;
        }
        // SIO (controller)
        if (io >= 0x040 && io < 0x050) {
            note_sio_io(phys);
            return sio_.read8(io - 0x040);
        }
        // CDROM
        if (io >= 0x800 && io < 0x804) {
            note_cdrom_io(phys);
            return cdrom_.read8(io - 0x800);
        }
        // MDEC
        if (io >= 0x820 && io < 0x828) {
            const u32 reg = (io & 0x4u) ? mdec_.read_status() : mdec_.read_data();
            const u32 shift = (io & 0x3u) * 8u;
            return static_cast<u8>((reg >> shift) & 0xFFu);
        }
        // Expansion 2
        if (phys == 0x1F802041)
            return post_reg_;
        if (phys >= 0x1F802000)
            return 0xFF;

        LOG_WARN("BUS: Unhandled read8 at I/O 0x%08X", phys);
        return 0xFF;
    }

    // Expansion 1
    if (phys >= 0x1F000000 && phys < 0x1F800000)
        return 0xFF;

    LOG_WARN("BUS: Unhandled read8 at 0x%08X", phys);
    return 0xFF;
}

u16 System::read16(u32 addr) {
    u32 phys = psx::mask_address(addr);
    if (g_trace_bus && phys >= 0x1F801000 && phys < 0x1F803000) {
        static u64 bus_r16_count = 0;
        if (trace_should_log(bus_r16_count, g_trace_burst_bus,
            g_trace_stride_bus)) {
            LOG_DEBUG("BUS: R16 addr=0x%08X phys=0x%08X", addr, phys);
        }
    }

    if (phys < 0x1F000000)
        return ram_.read16(phys & 0x1FFFFF);
    if (phys >= psx::BIOS_BASE &&
        static_cast<u64>(phys) <
        (static_cast<u64>(psx::BIOS_BASE) + bios_.mapped_size()))
        return bios_.read16(phys - psx::BIOS_BASE);
    if (phys >= 0x1F800000 && phys < 0x1F800400)
        return ram_.scratch_read16(phys - 0x1F800000);

    if (phys >= 0x1F801000 && phys < 0x1F803000) {
        u32 io = phys - 0x1F801000;
        // Unused timer slot / peripheral gap (e.g. 1F801130h..13Fh): open bus.
        if (io >= 0x130 && io < 0x140) {
            return 0xFFFF;
        }
        // Interrupt controller
        if (io >= 0x070 && io < 0x078)
            return static_cast<u16>(irq_.read(io - 0x070));
        // Timers
        if (io >= 0x100 && io < 0x130)
            return static_cast<u16>(timers_.read(io - 0x100));
        // SIO
        if (io >= 0x040 && io < 0x050) {
            note_sio_io(phys);
            return sio_.read16(io - 0x040);
        }
        // CDROM
        if (io >= 0x800 && io < 0x804) {
            note_cdrom_io(phys);
            u16 lo = cdrom_.read8(io - 0x800);
            u16 hi = cdrom_.read8(std::min<u32>(io - 0x800 + 1, 3));
            return lo | static_cast<u16>(hi << 8);
        }
        // MDEC
        if (io >= 0x820 && io < 0x828) {
            const u32 reg = (io & 0x4u) ? mdec_.read_status() : mdec_.read_data();
            const u32 shift = (io & 0x2u) ? 16u : 0u;
            return static_cast<u16>((reg >> shift) & 0xFFFFu);
        }
        // SPU
        if (io >= 0xC00 && io < 0x1000) {
            sync_spu_to_cpu();
            return spu_.read16(io - 0xC00);
        }

        LOG_WARN("BUS: Unhandled read16 at I/O 0x%08X", phys);
        return 0xFFFF;
    }

    LOG_WARN("BUS: Unhandled read16 at 0x%08X", phys);
    return 0xFFFF;
}

u32 System::read32(u32 addr) {
    u32 phys = psx::mask_address(addr);
    if (g_trace_bus && phys >= 0x1F801000 && phys < 0x1F803000) {
        static u64 bus_r32_count = 0;
        if (trace_should_log(bus_r32_count, g_trace_burst_bus,
            g_trace_stride_bus)) {
            LOG_DEBUG("BUS: R32 addr=0x%08X phys=0x%08X", addr, phys);
        }
    }

    if (phys < 0x1F000000)
        return ram_.read32(phys & 0x1FFFFF);
    if (phys >= psx::BIOS_BASE &&
        static_cast<u64>(phys) <
        (static_cast<u64>(psx::BIOS_BASE) + bios_.mapped_size()))
        return bios_.read32(phys - psx::BIOS_BASE);
    if (phys >= 0x1F800000 && phys < 0x1F800400)
        return ram_.scratch_read32(phys - 0x1F800000);

    if (phys >= 0x1F801000 && phys < 0x1F803000) {
        u32 io = phys - 0x1F801000;
        // Unused timer slot / peripheral gap (e.g. 1F801130h..13Fh): open bus.
        if (io >= 0x130 && io < 0x140) {
            return 0xFFFFFFFFu;
        }
        // Memory control
        if (io < 0x024)
            return mem_ctrl_[io / 4];
        // RAM size
        if (io == 0x060)
            return ram_size_;
        // Interrupt controller
        if (io >= 0x070 && io < 0x078)
            return irq_.read(io - 0x070);
        // DMA
        if (io >= 0x080 && io < 0x100)
            return dma_.read(io - 0x080);
        // Timers
        if (io >= 0x100 && io < 0x130)
            return timers_.read(io - 0x100);
        // SIO
        if (io >= 0x040 && io < 0x050) {
            note_sio_io(phys);
            return sio_.read32(io - 0x040);
        }
        // GPU
        if (io == 0x810)
            return gpu_.read_data();
        if (io == 0x814)
            return gpu_.read_stat();
        // CDROM
        if (io >= 0x800 && io < 0x804) {
            note_cdrom_io(phys);
            u32 b0 = cdrom_.read8(io - 0x800);
            u32 b1 = cdrom_.read8(std::min<u32>(io - 0x800 + 1, 3));
            u32 b2 = cdrom_.read8(std::min<u32>(io - 0x800 + 2, 3));
            u32 b3 = cdrom_.read8(std::min<u32>(io - 0x800 + 3, 3));
            return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
        }
        // MDEC
        if (io == 0x820) {
            return mdec_.read_data();
        }
        if (io == 0x824) {
            return mdec_.read_status();
        }
        // SPU
        if (io >= 0xC00 && io < 0x1000) {
            sync_spu_to_cpu();
            u16 lo = spu_.read16(io - 0xC00);
            u16 hi = spu_.read16(io - 0xC00 + 2);
            return lo | (static_cast<u32>(hi) << 16);
        }

        LOG_WARN("BUS: Unhandled read32 at I/O 0x%08X", phys);
        return 0xFFFFFFFFu;
    }

    // Expansion 1
    if (phys >= 0x1F000000 && phys < 0x1F800000)
        return 0xFFFFFFFF;

    // KSEG2 (cache control)
    if (addr == 0xFFFE0130)
        return cache_ctrl_;

    LOG_WARN("BUS: Unhandled read32 at 0x%08X", phys);
    return 0xFFFFFFFFu;
}

void System::write8(u32 addr, u8 val) {
    u32 phys = psx::mask_address(addr);
    if (g_trace_bus && phys >= 0x1F801000 && phys < 0x1F803000) {
        static u64 bus_w8_count = 0;
        if (trace_should_log(bus_w8_count, g_trace_burst_bus, g_trace_stride_bus)) {
            LOG_DEBUG("BUS: W8  addr=0x%08X phys=0x%08X val=0x%02X", addr, phys, val);
        }
    }

    if (phys < 0x1F000000) {
        ram_.write8(phys & 0x1FFFFF, val);
        return;
    }
    if (phys >= 0x1F800000 && phys < 0x1F800400) {
        ram_.scratch_write8(phys - 0x1F800000, val);
        return;
    }

    if (phys >= 0x1F801000 && phys < 0x1F803000) {
        u32 io = phys - 0x1F801000;
        // Unused timer slot / peripheral gap (e.g. 1F801130h..13Fh): ignore writes.
        if (io >= 0x130 && io < 0x140) {
            return;
        }
        if (io >= 0x040 && io < 0x050) {
            note_sio_io(phys);
            sio_.write8(io - 0x040, val);
            return;
        }
        if (io >= 0x800 && io < 0x804) {
            note_cdrom_io(phys);
            cdrom_.write8(io - 0x800, val);
            return;
        }
        if (io >= 0x820 && io < 0x828) {
            // MDEC is normally programmed via 32-bit accesses; ignore byte writes.
            return;
        }
        // Expansion 2 (POST register, etc.)
        if (phys == 0x1F802041) {
            post_reg_ = val;
            return;
        }
        if (phys >= 0x1F802000)
            return;
        LOG_WARN("BUS: Unhandled write8 at I/O 0x%08X = 0x%02X", phys, val);
        return;
    }

    if (phys >= 0x1F000000 && phys < 0x1F800000)
        return; // Expansion 1

    LOG_WARN("BUS: Unhandled write8 at 0x%08X = 0x%02X", phys, val);
}

void System::write16(u32 addr, u16 val) {
    u32 phys = psx::mask_address(addr);
    if (g_trace_bus && phys >= 0x1F801000 && phys < 0x1F803000) {
        static u64 bus_w16_count = 0;
        if (trace_should_log(bus_w16_count, g_trace_burst_bus,
            g_trace_stride_bus)) {
            LOG_DEBUG("BUS: W16 addr=0x%08X phys=0x%08X val=0x%04X", addr, phys, val);
        }
    }

    if (phys < 0x1F000000) {
        ram_.write16(phys & 0x1FFFFF, val);
        return;
    }
    if (phys >= 0x1F800000 && phys < 0x1F800400) {
        ram_.scratch_write16(phys - 0x1F800000, val);
        return;
    }

    if (phys >= 0x1F801000 && phys < 0x1F803000) {
        u32 io = phys - 0x1F801000;
        // Unused timer slot / peripheral gap (e.g. 1F801130h..13Fh): ignore writes.
        if (io >= 0x130 && io < 0x140) {
            return;
        }
        if (io >= 0x070 && io < 0x078) {
            irq_.write(io - 0x070, val);
            return;
        }
        if (io >= 0x100 && io < 0x130) {
            timers_.write(io - 0x100, val);
            return;
        }
        if (io >= 0x040 && io < 0x050) {
            note_sio_io(phys);
            sio_.write16(io - 0x040, val);
            return;
        }
        if (io >= 0x800 && io < 0x804) {
            note_cdrom_io(phys);
            cdrom_.write8(io - 0x800, static_cast<u8>(val & 0xFF));
            cdrom_.write8(std::min<u32>(io - 0x800 + 1, 3),
                static_cast<u8>((val >> 8) & 0xFF));
            return;
        }
        if (io >= 0x820 && io < 0x828) {
            // MDEC is normally programmed via 32-bit accesses; ignore halfword writes.
            return;
        }
        if (io >= 0xC00 && io < 0x1000) {
            sync_spu_to_cpu();
            spu_.write16(io - 0xC00, val);
            return;
        }

        LOG_WARN("BUS: Unhandled write16 at I/O 0x%08X = 0x%04X", phys, val);
        return;
    }

    LOG_WARN("BUS: Unhandled write16 at 0x%08X = 0x%04X", phys, val);
}

void System::write32(u32 addr, u32 val) {
    u32 phys = psx::mask_address(addr);
    if (g_trace_bus && phys >= 0x1F801000 && phys < 0x1F803000) {
        static u64 bus_w32_count = 0;
        if (trace_should_log(bus_w32_count, g_trace_burst_bus,
            g_trace_stride_bus)) {
            LOG_DEBUG("BUS: W32 addr=0x%08X phys=0x%08X val=0x%08X", addr, phys, val);
        }
    }

    if (phys < 0x1F000000) {
        ram_.write32(phys & 0x1FFFFF, val);
        return;
    }
    if (phys >= 0x1F800000 && phys < 0x1F800400) {
        ram_.scratch_write32(phys - 0x1F800000, val);
        return;
    }

    if (phys >= 0x1F801000 && phys < 0x1F803000) {
        u32 io = phys - 0x1F801000;
        // Unused timer slot / peripheral gap (e.g. 1F801130h..13Fh): ignore writes.
        if (io >= 0x130 && io < 0x140) {
            return;
        }
        // Memory control
        if (io < 0x024) {
            mem_ctrl_[io / 4] = val;
            return;
        }
        // RAM size
        if (io == 0x060) {
            ram_size_ = val;
            return;
        }
        // Interrupt controller
        if (io >= 0x070 && io < 0x078) {
            irq_.write(io - 0x070, val);
            return;
        }
        // DMA
        if (io >= 0x080 && io < 0x100) {
            dma_.write(io - 0x080, val);
            return;
        }
        // Timers
        if (io >= 0x100 && io < 0x130) {
            timers_.write(io - 0x100, val);
            return;
        }
        // SIO
        if (io >= 0x040 && io < 0x050) {
            note_sio_io(phys);
            sio_.write32(io - 0x040, val);
            return;
        }
        // GPU
        if (io == 0x810) {
            gpu_.gp0(val);
            return;
        }
        if (io == 0x814) {
            gpu_.gp1(val);
            return;
        }
        // CDROM
        if (io >= 0x800 && io < 0x804) {
            note_cdrom_io(phys);
            cdrom_.write8(io - 0x800, static_cast<u8>(val & 0xFF));
            cdrom_.write8(std::min<u32>(io - 0x800 + 1, 3),
                static_cast<u8>((val >> 8) & 0xFF));
            cdrom_.write8(std::min<u32>(io - 0x800 + 2, 3),
                static_cast<u8>((val >> 16) & 0xFF));
            cdrom_.write8(std::min<u32>(io - 0x800 + 3, 3),
                static_cast<u8>((val >> 24) & 0xFF));
            return;
        }
        // MDEC
        if (io == 0x820) {
            mdec_.write_command(val);
            return;
        }
        if (io == 0x824) {
            mdec_.write_control(val);
            return;
        }
        // SPU
        if (io >= 0xC00 && io < 0x1000) {
            sync_spu_to_cpu();
            spu_.write16(io - 0xC00, static_cast<u16>(val));
            spu_.write16(io - 0xC00 + 2, static_cast<u16>(val >> 16));
            return;
        }

        LOG_WARN("BUS: Unhandled write32 at I/O 0x%08X = 0x%08X", phys, val);
        return;
    }

    // Expansion 1
    if (phys >= 0x1F000000 && phys < 0x1F800000)
        return;

    // KSEG2 cache control
    if (addr == 0xFFFE0130) {
        cache_ctrl_ = val;
        return;
    }

    LOG_WARN("BUS: Unhandled write32 at 0x%08X = 0x%08X", phys, val);
}