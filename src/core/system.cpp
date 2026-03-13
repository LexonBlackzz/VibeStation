#include "system.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
    constexpr u32 kMainRamMirrorWindow = 0x00800000u;
    constexpr u32 kRamWatchStart = 0x000479D0u;
    constexpr u32 kRamWatchEnd = 0x00047A10u;
    constexpr u32 kRamWatchWord0 = 0x000479D0u;
    constexpr u32 kRamWatchLogLimit = 64u;
    constexpr u32 kRr4SourceWatchStart = 0x001A8E40u;
    constexpr u32 kRr4SourceWatchEnd = 0x001A8EC0u;

    struct BusWarnLimiter {
        u32 last_addr = 0xFFFFFFFFu;
        u64 suppressed = 0;
        u64 total = 0;
    };

void log_unhandled_bus_read(BusWarnLimiter& limiter, const char* width,
        u32 phys, bool io_space) {
        ++limiter.total;
        if (phys == limiter.last_addr) {
            ++limiter.suppressed;
            if (limiter.suppressed <= 3 ||
                (limiter.suppressed % 256u) == 0u) {
                if (io_space) {
                    LOG_WARN("BUS: Unhandled %s at I/O 0x%08X (repeat=%llu total=%llu)",
                        width, phys,
                        static_cast<unsigned long long>(limiter.suppressed),
                        static_cast<unsigned long long>(limiter.total));
                }
                else {
                    LOG_WARN("BUS: Unhandled %s at 0x%08X (repeat=%llu total=%llu)",
                        width, phys,
                        static_cast<unsigned long long>(limiter.suppressed),
                        static_cast<unsigned long long>(limiter.total));
                }
            }
            return;
        }

        if (limiter.suppressed > 3) {
            LOG_WARN("BUS: Previous unhandled read at 0x%08X repeated %llu times",
                limiter.last_addr,
                static_cast<unsigned long long>(limiter.suppressed));
        }

        limiter.last_addr = phys;
        limiter.suppressed = 0;
        if (io_space) {
            LOG_WARN("BUS: Unhandled %s at I/O 0x%08X (total=%llu)",
                width, phys, static_cast<unsigned long long>(limiter.total));
        }
        else {
            LOG_WARN("BUS: Unhandled %s at 0x%08X (total=%llu)",
                width, phys, static_cast<unsigned long long>(limiter.total));
        }
    }

    void log_ram_size_access(const char* op, u32 value, u32 pc, u32 sp, u32 ra) {
        static u32 count = 0;
        if (count >= 16u) {
            return;
        }
        ++count;
        LOG_WARN(
            "BUS: RAM_SIZE %s val=0x%08X pc=0x%08X sp=0x%08X ra=0x%08X",
            op, value, pc, sp, ra);
    }

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

    bool probe_contains_addr(u32 base, u32 size_bytes, u32 addr) {
        if (size_bytes == 0) {
            return false;
        }
        const u32 start = base & 0x001FFFFFu;
        const u32 end = start + size_bytes;
        return (addr >= start) && (addr < end);
    }

    System::RamReaperConfig sanitize_ram_reaper_config(
        const System::RamReaperConfig& input) {
        System::RamReaperConfig cfg = input;
        const u32 max_ram_off = psx::RAM_SIZE - 1u;
        cfg.range_start = std::min(cfg.range_start, max_ram_off);
        cfg.range_end = std::min(cfg.range_end, max_ram_off);
        cfg.writes_per_frame = std::min(cfg.writes_per_frame, psx::RAM_SIZE);
        cfg.intensity_percent = std::max(0.0f, std::min(100.0f, cfg.intensity_percent));
        if (cfg.range_start > cfg.range_end) {
            std::swap(cfg.range_start, cfg.range_end);
        }
        return cfg;
    }

    System::GpuReaperConfig sanitize_gpu_reaper_config(
        const System::GpuReaperConfig& input) {
        System::GpuReaperConfig cfg = input;
        cfg.writes_per_frame = std::min(cfg.writes_per_frame, 5000u);
        cfg.intensity_percent = std::max(0.0f, std::min(100.0f, cfg.intensity_percent));
        return cfg;
    }

    System::SoundReaperConfig sanitize_sound_reaper_config(
        const System::SoundReaperConfig& input) {
        System::SoundReaperConfig cfg = input;
        cfg.writes_per_frame = std::min(cfg.writes_per_frame, 5000u);
        cfg.intensity_percent = std::max(0.0f, std::min(100.0f, cfg.intensity_percent));
        return cfg;
    }

    void seed_mt19937(std::mt19937& rng, u64 seed) {
        const u32 lo = static_cast<u32>(seed & 0xFFFFFFFFull);
        const u32 hi = static_cast<u32>((seed >> 32) & 0xFFFFFFFFull);
        std::seed_seq seq{ lo, hi, 0x9E3779B9u, 0x243F6A88u };
        rng.seed(seq);
    }
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

void System::maybe_log_ram_watch_write(u32 phys_addr, u32 value, u32 size_bytes) {
    if (!g_ram_watch_diagnostics) {
        return;
    }
    static u32 watch_log_count = 0;
    static u32 last_code_dump_pc = 0xFFFFFFFFu;
    const u32 ram_off = phys_addr & 0x001FFFFFu;
    if (ram_off < kRamWatchStart || ram_off >= kRamWatchEnd) {
        return;
    }
    const u32 word0_end = kRamWatchWord0 + 4u;
    if ((ram_off + size_bytes) <= kRamWatchWord0 || ram_off >= word0_end) {
        return;
    }
    if (is_bios_rom_pc(cpu_.pc())) {
        return;
    }
    if (watch_log_count >= kRamWatchLogLimit) {
        return;
    }
    u32 old_word = ram_.read32(kRamWatchWord0);
    u32 new_word = old_word;
    for (u32 i = 0; i < size_bytes; ++i) {
        const u32 byte_addr = ram_off + i;
        if (byte_addr < kRamWatchWord0 || byte_addr >= word0_end) {
            continue;
        }
        const u32 shift = (byte_addr - kRamWatchWord0) * 8u;
        const u32 byte_val = (value >> (i * 8u)) & 0xFFu;
        new_word = (new_word & ~(0xFFu << shift)) | (byte_val << shift);
    }
    ++watch_log_count;
    LOG_WARN(
        "BUS: RAM watch write%u off=0x%08X phys=0x%08X val=0x%08X word0=0x%08X->0x%08X pc=0x%08X sp=0x%08X ra=0x%08X",
        size_bytes * 8u, ram_off, phys_addr, value, old_word, new_word, cpu_.pc(),
        cpu_.reg(29), cpu_.reg(31));

    if (ram_off == kRamWatchWord0) {
        LOG_WARN(
            "BUS: stream-word0 write%u val=0x%08X slot=0x%08X sp=0x%08X ra=0x%08X",
            size_bytes * 8u, value, ram_off, cpu_.reg(29), cpu_.reg(31));
    }

    if (ram_off == kRamWatchWord0 || (ram_off + size_bytes) > kRamWatchWord0) {
        const u32 pc = cpu_.pc() & 0x1FFFFFFFu;
        if (pc != last_code_dump_pc) {
            last_code_dump_pc = pc;
            LOG_WARN(
                "BUS: watch code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
                pc - 0x08u, read32(pc - 0x08u), pc - 0x04u, read32(pc - 0x04u),
                pc + 0x00u, read32(pc + 0x00u), pc + 0x04u, read32(pc + 0x04u),
                pc + 0x08u, read32(pc + 0x08u));
        }
    }
}

void System::sync_spu_to_cpu() {
    const u64 target_cycle = cpu_.cycle_count();
    if (target_cycle <= spu_synced_cpu_cycle_) {
        spu_.mark_synced_to_cpu(spu_synced_cpu_cycle_);
        return;
    }

    if (spu_skip_sync_for_turbo_) {
        spu_synced_cpu_cycle_ = target_cycle;
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

bool System::save_spu_voice_sample_to_file(int voice, const std::string& path,
                                           std::string* error) {
    sync_spu_to_cpu();
    return spu_.export_voice_sample_to_file(voice, path, error);
}

bool System::save_spu_voice_samples_to_file(const std::vector<int>& voices,
                                            const std::string& path,
                                            std::string* error) {
    sync_spu_to_cpu();
    return spu_.export_voice_samples_to_file(voices, path, error);
}

bool System::load_spu_replacement_sample_from_file(const std::string& path,
                                                   std::string* error) {
    sync_spu_to_cpu();
    return spu_.load_replacement_sample_from_file(path, error);
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
    spu_.clear_replacement_sample();
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

bool System::boot_disc(bool direct_boot) {
    if (!bios_loaded()) {
        return false;
    }

    const std::string bin = last_disc_bin_path_;
    const std::string cue = last_disc_cue_path_;

    set_running(false);
    reset();

    if (!bin.empty()) {
        if (!load_game(bin, cue)) {
            return false;
        }
    }

    if (!disc_loaded()) {
        return false;
    }

    if (direct_boot) {
        if (!bios_.apply_fast_boot_patch()) {
            LOG_WARN("System: direct disc boot patch failed; falling back to normal BIOS boot.");
        }
    }

    set_running(true);
    return true;
}

void System::reset() {
    if (!hw_init_) {
        init_hardware();
    }
    bios_.restore_original_image();
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
    mdec_command_shadow_ = 0;
    mdec_command_shadow_mask_ = 0;
    mdec_control_shadow_ = 0;
    mdec_control_shadow_mask_ = 0;
    gpu_gp0_shadow_ = 0;
    gpu_gp0_shadow_mask_ = 0;
    gpu_gp1_shadow_ = 0;
    gpu_gp1_shadow_mask_ = 0;
    post_reg_ = 0;
    mdec_upload_probe_ = {};
    stack_top_burst_ = {};
    stack_top_write_log_budget_ = 64;
    stack_top_write_log_suppressed_ = false;
    gpu_gp0_source_valid_ = false;
    gpu_gp0_source_from_dma_ = false;
    gpu_gp0_source_addr_ = 0;
}

void System::shutdown() {
    sio_.shutdown();
    spu_.shutdown();
}

void System::gpu_gp0(u32 val) {
    gpu_gp0_source_valid_ = true;
    gpu_gp0_source_from_dma_ = false;
    gpu_gp0_source_addr_ = 0;
    gpu_.gp0(val);
    gpu_gp0_source_valid_ = false;
}

void System::gpu_gp0_dma(u32 val, u32 src_addr) {
    gpu_gp0_source_valid_ = true;
    gpu_gp0_source_from_dma_ = true;
    gpu_gp0_source_addr_ = src_addr & 0x001FFFFCu;
    gpu_.gp0(val);
    gpu_gp0_source_valid_ = false;
}

void System::debug_begin_dma_bus_access(u8 channel) {
    bus_access_from_dma_ = true;
    bus_access_dma_channel_ = channel;
}

void System::debug_end_dma_bus_access() {
    bus_access_from_dma_ = false;
    bus_access_dma_channel_ = 0xFFu;
}

void System::debug_note_main_ram_read(u32 addr, u32 value, u8 size) {
    if (!g_mdec_debug_upload_probe) {
        return;
    }
    if (!probe_contains_addr(mdec_upload_probe_.dma1_base_addr,
                             mdec_upload_probe_.dma1_range_bytes, addr)) {
        return;
    }

    const u32 index = mdec_upload_probe_.mdec_read_sample_count;
    if (index >= MdecUploadProbe::kSampleWords) {
        return;
    }
    mdec_upload_probe_.mdec_read_addrs[index] = addr;
    mdec_upload_probe_.mdec_read_values[index] = value;
    mdec_upload_probe_.mdec_read_pcs[index] = bus_access_from_dma_ ? 0u : cpu_.pc();
    mdec_upload_probe_.mdec_read_origin[index] =
        bus_access_from_dma_ ? static_cast<u8>(0x80u | bus_access_dma_channel_) : 0u;
    mdec_upload_probe_.mdec_read_sizes[index] = size;
    ++mdec_upload_probe_.mdec_read_sample_count;
}

void System::debug_note_main_ram_write(u32 addr, u32 value, u8 size) {
    if (!g_mdec_debug_upload_probe && !g_cpu_deep_diagnostics) {
        return;
    }
    RamAccessLogEntry &entry =
        ram_write_history_[ram_write_history_pos_ % static_cast<u32>(kRamWriteHistorySize)];
    entry.addr = addr;
    entry.value = value;
    entry.pc = bus_access_from_dma_ ? 0u : cpu_.pc();
    entry.size = size;
    entry.origin =
        bus_access_from_dma_ ? static_cast<u8>(0x80u | bus_access_dma_channel_) : 0u;
    ++ram_write_history_pos_;
    ram_write_history_count_ =
        std::min<u32>(ram_write_history_count_ + 1u, static_cast<u32>(kRamWriteHistorySize));

    if (g_cpu_deep_diagnostics) {
        debug_track_stack_top_write(entry);
    }

    if (g_cpu_deep_diagnostics && addr >= 0x001FFF00u && addr < 0x00200000u &&
        !(entry.origin == 0u && entry.size == 2u && entry.value == 0u)) {
        if (stack_top_write_log_budget_ != 0u) {
            --stack_top_write_log_budget_;
            if ((entry.origin & 0x80u) != 0u) {
                LOG_WARN("BUS: stack-top W%u 0x%08X = 0x%08X <- DMA%u",
                         static_cast<unsigned>(size), addr, value,
                         static_cast<unsigned>(entry.origin & 0x7Fu));
            } else {
                LOG_WARN("BUS: stack-top W%u 0x%08X = 0x%08X pc=0x%08X",
                         static_cast<unsigned>(size), addr, value, entry.pc);
            }
        } else if (!stack_top_write_log_suppressed_) {
            stack_top_write_log_suppressed_ = true;
            LOG_WARN("BUS: stack-top per-write logging suppressed after budget exhaustion; retaining burst and history diagnostics");
        }
    }

    if (g_cpu_deep_diagnostics && addr >= 0x00047680u && addr < 0x000476C0u) {
        static u32 low_stub_write_log_count = 0;
        static bool low_stub_context_logged = false;
        if (!low_stub_context_logged) {
            low_stub_context_logged = true;
            LOG_WARN(
                "BUS: low-stub context sp=0x%08X ra=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X t4=0x%08X t5=0x%08X t6=0x%08X t7=0x%08X",
                cpu_.reg(29), cpu_.reg(31), cpu_.reg(4), cpu_.reg(5),
                cpu_.reg(6), cpu_.reg(7), cpu_.reg(8), cpu_.reg(9),
                cpu_.reg(10), cpu_.reg(11), cpu_.reg(12), cpu_.reg(13),
                cpu_.reg(14), cpu_.reg(15));
            LOG_WARN(
                "BUS: low-stub bios code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
                0x1FC02B58u, read32(0x1FC02B58u), 0x1FC02B5Cu,
                read32(0x1FC02B5Cu), 0x1FC02B60u, read32(0x1FC02B60u),
                0x1FC02B64u, read32(0x1FC02B64u), 0x1FC02B68u,
                read32(0x1FC02B68u));
            LOG_WARN(
                "BUS: low-stub bios code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
                0x1FC02B6Cu, read32(0x1FC02B6Cu), 0x1FC02B70u,
                read32(0x1FC02B70u), 0x1FC02B74u, read32(0x1FC02B74u),
                0x1FC02B78u, read32(0x1FC02B78u), 0x1FC02B7Cu,
                read32(0x1FC02B7Cu));
        }
        if (low_stub_write_log_count < 64u) {
            ++low_stub_write_log_count;
            if ((entry.origin & 0x80u) != 0u) {
                LOG_WARN("BUS: low-stub W%u 0x%08X = 0x%08X <- DMA%u",
                         static_cast<unsigned>(size), addr, value,
                         static_cast<unsigned>(entry.origin & 0x7Fu));
            } else {
                LOG_WARN("BUS: low-stub W%u 0x%08X = 0x%08X pc=0x%08X",
                         static_cast<unsigned>(size), addr, value, entry.pc);
            }
        }
    }

    if (g_cpu_deep_diagnostics && addr >= kRr4SourceWatchStart &&
        addr < kRr4SourceWatchEnd) {
        static u32 rr4_source_write_log_count = 0;
        static bool rr4_source_context_logged = false;
        if (!rr4_source_context_logged) {
            rr4_source_context_logged = true;
            LOG_WARN(
                "BUS: rr4-source context sp=0x%08X ra=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X s0=0x%08X s1=0x%08X s2=0x%08X s3=0x%08X t0=0x%08X t1=0x%08X t2=0x%08X t3=0x%08X",
                cpu_.reg(29), cpu_.reg(31), cpu_.reg(4), cpu_.reg(5),
                cpu_.reg(6), cpu_.reg(7), cpu_.reg(16), cpu_.reg(17),
                cpu_.reg(18), cpu_.reg(19), cpu_.reg(8), cpu_.reg(9),
                cpu_.reg(10), cpu_.reg(11));
            const u32 pc = cpu_.pc() & 0x1FFFFFFFu;
            LOG_WARN(
                "BUS: rr4-source code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
                (pc - 8u) & 0x1FFFFFFFu, read32(pc - 8u),
                (pc - 4u) & 0x1FFFFFFFu, read32(pc - 4u),
                pc & 0x1FFFFFFFu, read32(pc),
                (pc + 4u) & 0x1FFFFFFFu, read32(pc + 4u),
                (pc + 8u) & 0x1FFFFFFFu, read32(pc + 8u));
        }
        if (rr4_source_write_log_count < 64u) {
            ++rr4_source_write_log_count;
            if ((entry.origin & 0x80u) != 0u) {
                LOG_WARN("BUS: rr4-source W%u 0x%08X = 0x%08X <- DMA%u",
                         static_cast<unsigned>(size), addr, value,
                         static_cast<unsigned>(entry.origin & 0x7Fu));
            } else {
                LOG_WARN("BUS: rr4-source W%u 0x%08X = 0x%08X pc=0x%08X",
                         static_cast<unsigned>(size), addr, value, entry.pc);
            }
        }
    }

    if (!probe_contains_addr(mdec_upload_probe_.gpu_dma_src_base,
                             mdec_upload_probe_.gpu_dma_src_range_bytes, addr)) {
        return;
    }

    const u32 index = mdec_upload_probe_.gpu_src_write_sample_count;
    if (index >= MdecUploadProbe::kSampleWords) {
        return;
    }
    mdec_upload_probe_.gpu_src_write_addrs[index] = addr;
    mdec_upload_probe_.gpu_src_write_values[index] = value;
    mdec_upload_probe_.gpu_src_write_pcs[index] = entry.pc;
    mdec_upload_probe_.gpu_src_write_origin[index] = entry.origin;
    mdec_upload_probe_.gpu_src_write_sizes[index] = size;
    ++mdec_upload_probe_.gpu_src_write_sample_count;
}

void System::debug_track_stack_top_write(const RamAccessLogEntry& entry) {
    if (entry.addr < 0x001FFF00u || entry.addr >= 0x00200000u) {
        stack_top_burst_ = {};
        return;
    }

    const bool is_trackable_zero_halfword =
        entry.origin == 0u && entry.size == 2u && entry.value == 0u;
    if (!is_trackable_zero_halfword) {
        stack_top_burst_ = {};
        return;
    }

    const bool continues_burst =
        stack_top_burst_.active &&
        stack_top_burst_.pc == entry.pc &&
        stack_top_burst_.origin == entry.origin &&
        stack_top_burst_.size == entry.size &&
        stack_top_burst_.value == entry.value &&
        stack_top_burst_.next_addr == entry.addr;

    if (!continues_burst) {
        stack_top_burst_ = {};
        stack_top_burst_.active = true;
        stack_top_burst_.pc = entry.pc;
        stack_top_burst_.origin = entry.origin;
        stack_top_burst_.size = entry.size;
        stack_top_burst_.value = entry.value;
        stack_top_burst_.start_addr = entry.addr;
        stack_top_burst_.next_addr = entry.addr + entry.size;
        stack_top_burst_.count = 1;
        return;
    }

    ++stack_top_burst_.count;
    stack_top_burst_.next_addr += entry.size;

    if (!stack_top_burst_.logged_context && stack_top_burst_.count >= 8u) {
        stack_top_burst_.logged_context = true;
        debug_log_stack_top_burst_context(entry);
    }
}

void System::debug_log_stack_top_burst_context(const RamAccessLogEntry& entry) {
    const u32 end_addr =
        stack_top_burst_.next_addr >= entry.size ? (stack_top_burst_.next_addr - entry.size)
                                                 : stack_top_burst_.next_addr;
    LOG_WARN(
        "BUS: stack-top zero sweep pc=0x%08X range=0x%08X..0x%08X count=%u sp=0x%08X ra=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X v0=0x%08X v1=0x%08X",
        entry.pc, stack_top_burst_.start_addr, end_addr, stack_top_burst_.count,
        cpu_.reg(29), cpu_.reg(31), cpu_.reg(4), cpu_.reg(5), cpu_.reg(6),
        cpu_.reg(7), cpu_.reg(2), cpu_.reg(3));

    const u32 pc = entry.pc;
    LOG_WARN(
        "BUS: stack-top zero code %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X %08X=%08X",
        (pc - 8u) & 0x1FFFFFFFu, read32_instruction(pc - 8u),
        (pc - 4u) & 0x1FFFFFFFu, read32_instruction(pc - 4u),
        pc & 0x1FFFFFFFu, read32_instruction(pc),
        (pc + 4u) & 0x1FFFFFFFu, read32_instruction(pc + 4u),
        (pc + 8u) & 0x1FFFFFFFu, read32_instruction(pc + 8u),
        (pc + 12u) & 0x1FFFFFFFu, read32_instruction(pc + 12u));
}

void System::debug_log_recent_ram_writes(u32 addr, u32 radius_bytes) const {
    if (ram_write_history_count_ == 0) {
        LOG_WARN("BUS: no RAM write history available");
        return;
    }

    const u32 center = psx::mask_address(addr) & 0x001FFFFFu;
    const u32 start = (center > radius_bytes) ? (center - radius_bytes) : 0u;
    const u32 end = std::min<u32>(center + radius_bytes, psx::RAM_SIZE - 1u);
    LOG_WARN("BUS: recent RAM writes near 0x%08X (window=0x%08X..0x%08X)",
             center, start, end);

    bool found = false;
    u32 printed = 0;
    u32 suppressed = 0;
    constexpr u32 kMaxPrintedEntries = 64u;
    for (u32 i = 0; i < ram_write_history_count_; ++i) {
        const u32 hist_index =
            (ram_write_history_pos_ + static_cast<u32>(kRamWriteHistorySize) -
             ram_write_history_count_ + i) %
            static_cast<u32>(kRamWriteHistorySize);
        const RamAccessLogEntry &entry = ram_write_history_[hist_index];
        if (entry.addr < start || entry.addr > end) {
            continue;
        }
        found = true;
        if (printed >= kMaxPrintedEntries) {
            ++suppressed;
            continue;
        }
        ++printed;
        if ((entry.origin & 0x80u) != 0u) {
            LOG_WARN("BUS: W%u 0x%08X = 0x%08X <- DMA%u",
                     static_cast<unsigned>(entry.size), entry.addr, entry.value,
                     static_cast<unsigned>(entry.origin & 0x7Fu));
        } else {
            LOG_WARN("BUS: W%u 0x%08X = 0x%08X pc=0x%08X",
                     static_cast<unsigned>(entry.size), entry.addr, entry.value,
                     entry.pc);
        }
    }

    if (!found) {
        LOG_WARN("BUS: no RAM writes captured in requested window");
    } else if (suppressed != 0u) {
        LOG_WARN("BUS: suppressed %u additional RAM write entries in requested window",
                 suppressed);
    }
}

void System::populate_gpu_src_write_samples_from_history() {
    if (!g_mdec_debug_upload_probe) {
        return;
    }
    if (mdec_upload_probe_.gpu_dma_src_base == 0 ||
        mdec_upload_probe_.gpu_src_write_sample_count != 0 ||
        ram_write_history_count_ == 0) {
        return;
    }

    for (u32 i = 0; i < ram_write_history_count_; ++i) {
        const u32 hist_index =
            (ram_write_history_pos_ + static_cast<u32>(kRamWriteHistorySize) - ram_write_history_count_ + i) %
            static_cast<u32>(kRamWriteHistorySize);
        const RamAccessLogEntry &entry = ram_write_history_[hist_index];
        if (!probe_contains_addr(mdec_upload_probe_.gpu_dma_src_base,
                                 mdec_upload_probe_.gpu_dma_src_range_bytes, entry.addr)) {
            continue;
        }

        const u32 sample_index = mdec_upload_probe_.gpu_src_write_sample_count;
        if (sample_index >= MdecUploadProbe::kSampleWords) {
            break;
        }
        mdec_upload_probe_.gpu_src_write_addrs[sample_index] = entry.addr;
        mdec_upload_probe_.gpu_src_write_values[sample_index] = entry.value;
        mdec_upload_probe_.gpu_src_write_pcs[sample_index] = entry.pc;
        mdec_upload_probe_.gpu_src_write_origin[sample_index] = entry.origin;
        mdec_upload_probe_.gpu_src_write_sizes[sample_index] = entry.size;
        ++mdec_upload_probe_.gpu_src_write_sample_count;
    }
}

void System::debug_note_mdec_dma_out_begin(u32 base_addr, u32 words, u8 depth,
                                           u8 first_block) {
    if (!g_mdec_debug_upload_probe) {
        return;
    }
    mdec_upload_probe_.dma1_seen = true;
    mdec_upload_probe_.dma1_base_addr = base_addr & 0x001FFFFCu;
    mdec_upload_probe_.dma1_words = words;
    mdec_upload_probe_.dma1_range_bytes = words * 4u;
    mdec_upload_probe_.dma1_depth = depth;
    mdec_upload_probe_.dma1_first_block = first_block;
    mdec_upload_probe_.dma1_sample_count = 0;
    mdec_upload_probe_.dma1_addrs.fill(0);
    mdec_upload_probe_.dma1_words_sample.fill(0);
    mdec_upload_probe_.dma1_mb_sample_count = 0;
    mdec_upload_probe_.dma1_mb_seq.fill(0);
    mdec_upload_probe_.dma1_mb_addrs.fill(0);
    mdec_upload_probe_.dma1_mb_words_sample.fill(0);
    mdec_upload_probe_.mdec_read_sample_count = 0;
    mdec_upload_probe_.mdec_read_addrs.fill(0);
    mdec_upload_probe_.mdec_read_values.fill(0);
    mdec_upload_probe_.mdec_read_pcs.fill(0);
    mdec_upload_probe_.mdec_read_origin.fill(0);
    mdec_upload_probe_.mdec_read_sizes.fill(0);
}

void System::debug_note_mdec_dma_out_word(u32 write_addr, u32 value,
                                          u32 macroblock_seq) {
    if (!g_mdec_debug_upload_probe) {
        return;
    }
    if (!mdec_upload_probe_.dma1_seen) {
        return;
    }
    const u32 masked_addr = write_addr & 0x001FFFFCu;
    const u32 word_index = mdec_upload_probe_.dma1_sample_count;
    if (word_index < MdecUploadProbe::kSampleWords) {
        mdec_upload_probe_.dma1_addrs[word_index] = masked_addr;
        mdec_upload_probe_.dma1_words_sample[word_index] = value;
        ++mdec_upload_probe_.dma1_sample_count;
    }

    const bool new_transfer_seq =
        (mdec_upload_probe_.dma1_mb_sample_count == 0u) ||
        (mdec_upload_probe_.dma1_mb_seq[mdec_upload_probe_.dma1_mb_sample_count - 1u] !=
         macroblock_seq);
    if (new_transfer_seq &&
        mdec_upload_probe_.dma1_mb_sample_count < MdecUploadProbe::kSampleWords) {
        const u32 mb_index = mdec_upload_probe_.dma1_mb_sample_count++;
        mdec_upload_probe_.dma1_mb_seq[mb_index] = macroblock_seq;
        mdec_upload_probe_.dma1_mb_addrs[mb_index] = masked_addr;
        mdec_upload_probe_.dma1_mb_words_sample[mb_index] = value;
    }

    const bool new_history_seq =
        (mdec_upload_probe_.dma1_mb_hist_count == 0u) ||
        (mdec_upload_probe_
             .dma1_mb_hist_seq[(mdec_upload_probe_.dma1_mb_hist_count - 1u) %
                               static_cast<u32>(MdecUploadProbe::kMacroblockHistory)] !=
         macroblock_seq);
    if (new_history_seq) {
        const u32 hist_index =
            mdec_upload_probe_.dma1_mb_hist_count %
            static_cast<u32>(MdecUploadProbe::kMacroblockHistory);
        mdec_upload_probe_.dma1_mb_hist_seq[hist_index] = macroblock_seq;
        mdec_upload_probe_.dma1_mb_hist_addrs[hist_index] = masked_addr;
        mdec_upload_probe_.dma1_mb_hist_words_sample[hist_index] = value;
        ++mdec_upload_probe_.dma1_mb_hist_count;
    }
}

void System::debug_note_gpu_image_load_begin(u16 x, u16 y, u16 w, u16 h) {
    if (!g_mdec_debug_upload_probe) {
        return;
    }
    mdec_upload_probe_.gpu_upload_seen = true;
    mdec_upload_probe_.gpu_upload_data_seen = false;
    ++mdec_upload_probe_.gpu_upload_count;
    mdec_upload_probe_.gpu_x = x;
    mdec_upload_probe_.gpu_y = y;
    mdec_upload_probe_.gpu_w = w;
    mdec_upload_probe_.gpu_h = h;
    mdec_upload_probe_.gpu_total_words =
        (static_cast<u32>(w) * static_cast<u32>(h) + 1u) / 2u;
    mdec_upload_probe_.gpu_words_seen = 0;
    mdec_upload_probe_.gpu_sample_count = 0;
    mdec_upload_probe_.gpu_words_sample.fill(0);
    mdec_upload_probe_.gpu_dma_src_addrs.fill(0);
    mdec_upload_probe_.gpu_word_from_dma.fill(0);
    mdec_upload_probe_.gpu_dma_src_base = 0;
    mdec_upload_probe_.gpu_dma_src_range_bytes = mdec_upload_probe_.gpu_total_words * 4u;
    mdec_upload_probe_.gpu_src_write_sample_count = 0;
    mdec_upload_probe_.gpu_src_write_addrs.fill(0);
    mdec_upload_probe_.gpu_src_write_values.fill(0);
    mdec_upload_probe_.gpu_src_write_pcs.fill(0);
    mdec_upload_probe_.gpu_src_write_origin.fill(0);
    mdec_upload_probe_.gpu_src_write_sizes.fill(0);
    mdec_upload_probe_.gpu_copy_count = 0;
    mdec_upload_probe_.gpu_copy_sample_count = 0;
    mdec_upload_probe_.gpu_copy_src_x.fill(0);
    mdec_upload_probe_.gpu_copy_src_y.fill(0);
    mdec_upload_probe_.gpu_copy_dst_x.fill(0);
    mdec_upload_probe_.gpu_copy_dst_y.fill(0);
    mdec_upload_probe_.gpu_copy_w.fill(0);
    mdec_upload_probe_.gpu_copy_h.fill(0);

    const u32 hist_index =
        mdec_upload_probe_.gpu_hist_count % static_cast<u32>(MdecUploadProbe::kUploadHistory);
    mdec_upload_probe_.gpu_hist_x[hist_index] = x;
    mdec_upload_probe_.gpu_hist_y[hist_index] = y;
    mdec_upload_probe_.gpu_hist_w[hist_index] = w;
    mdec_upload_probe_.gpu_hist_h[hist_index] = h;
    mdec_upload_probe_.gpu_hist_from_dma[hist_index] = 0u;
    ++mdec_upload_probe_.gpu_hist_count;

    const u32 frame_index =
        mdec_upload_probe_.gpu_frame_upload_count %
        static_cast<u32>(MdecUploadProbe::kUploadHistory);
    mdec_upload_probe_.gpu_frame_hist_x[frame_index] = x;
    mdec_upload_probe_.gpu_frame_hist_y[frame_index] = y;
    mdec_upload_probe_.gpu_frame_hist_w[frame_index] = w;
    mdec_upload_probe_.gpu_frame_hist_h[frame_index] = h;
    mdec_upload_probe_.gpu_frame_hist_from_dma[frame_index] = 0u;
    ++mdec_upload_probe_.gpu_frame_upload_count;
}

void System::debug_note_gpu_image_load_word(u32 value) {
    if (!g_mdec_debug_upload_probe) {
        return;
    }
    if (!mdec_upload_probe_.gpu_upload_seen) {
        return;
    }
    mdec_upload_probe_.gpu_upload_data_seen = true;
    const u32 index = mdec_upload_probe_.gpu_sample_count;
    if (index < MdecUploadProbe::kSampleWords) {
        mdec_upload_probe_.gpu_words_sample[index] = value;
        if (gpu_gp0_source_valid_ && gpu_gp0_source_from_dma_) {
            if (mdec_upload_probe_.gpu_dma_src_base == 0) {
                mdec_upload_probe_.gpu_dma_src_base =
                    gpu_gp0_source_addr_ & 0x001FFFFCu;
                populate_gpu_src_write_samples_from_history();
            }
            mdec_upload_probe_.gpu_dma_src_addrs[index] =
                gpu_gp0_source_addr_ & 0x001FFFFCu;
            mdec_upload_probe_.gpu_word_from_dma[index] = 1u;
            if (mdec_upload_probe_.gpu_hist_count != 0) {
                const u32 hist_index =
                    (mdec_upload_probe_.gpu_hist_count - 1u) %
                    static_cast<u32>(MdecUploadProbe::kUploadHistory);
                mdec_upload_probe_.gpu_hist_from_dma[hist_index] = 1u;
            }
            if (mdec_upload_probe_.gpu_frame_upload_count != 0) {
                const u32 frame_index =
                    (mdec_upload_probe_.gpu_frame_upload_count - 1u) %
                    static_cast<u32>(MdecUploadProbe::kUploadHistory);
                mdec_upload_probe_.gpu_frame_hist_from_dma[frame_index] = 1u;
            }
        }
        ++mdec_upload_probe_.gpu_sample_count;
    }
    ++mdec_upload_probe_.gpu_words_seen;
}

void System::debug_note_gpu_vblank() {
    if (!g_mdec_debug_upload_probe) {
        return;
    }
    const u32 frame_count = std::min<u32>(
        mdec_upload_probe_.gpu_frame_upload_count,
        static_cast<u32>(MdecUploadProbe::kUploadHistory));
    mdec_upload_probe_.gpu_last_frame_valid = frame_count != 0u;
    mdec_upload_probe_.gpu_last_frame_upload_count =
        mdec_upload_probe_.gpu_frame_upload_count;
    mdec_upload_probe_.gpu_last_frame_hist_x.fill(0);
    mdec_upload_probe_.gpu_last_frame_hist_y.fill(0);
    mdec_upload_probe_.gpu_last_frame_hist_w.fill(0);
    mdec_upload_probe_.gpu_last_frame_hist_h.fill(0);
    mdec_upload_probe_.gpu_last_frame_hist_from_dma.fill(0);
    for (u32 i = 0; i < frame_count; ++i) {
        const u32 src_index =
            (mdec_upload_probe_.gpu_frame_upload_count - frame_count + i) %
            static_cast<u32>(MdecUploadProbe::kUploadHistory);
        mdec_upload_probe_.gpu_last_frame_hist_x[i] =
            mdec_upload_probe_.gpu_frame_hist_x[src_index];
        mdec_upload_probe_.gpu_last_frame_hist_y[i] =
            mdec_upload_probe_.gpu_frame_hist_y[src_index];
        mdec_upload_probe_.gpu_last_frame_hist_w[i] =
            mdec_upload_probe_.gpu_frame_hist_w[src_index];
        mdec_upload_probe_.gpu_last_frame_hist_h[i] =
            mdec_upload_probe_.gpu_frame_hist_h[src_index];
        mdec_upload_probe_.gpu_last_frame_hist_from_dma[i] =
            mdec_upload_probe_.gpu_frame_hist_from_dma[src_index];
    }
    mdec_upload_probe_.gpu_frame_upload_count = 0;
    mdec_upload_probe_.gpu_frame_hist_x.fill(0);
    mdec_upload_probe_.gpu_frame_hist_y.fill(0);
    mdec_upload_probe_.gpu_frame_hist_w.fill(0);
    mdec_upload_probe_.gpu_frame_hist_h.fill(0);
    mdec_upload_probe_.gpu_frame_hist_from_dma.fill(0);
}

void System::debug_note_gpu_vram_copy(u16 src_x, u16 src_y, u16 dst_x, u16 dst_y,
                                      u16 w, u16 h) {
    if (!g_mdec_debug_upload_probe) {
        return;
    }
    ++mdec_upload_probe_.gpu_copy_count;
    const u32 index = mdec_upload_probe_.gpu_copy_sample_count;
    if (index >= MdecUploadProbe::kSampleWords) {
        return;
    }
    mdec_upload_probe_.gpu_copy_src_x[index] = src_x;
    mdec_upload_probe_.gpu_copy_src_y[index] = src_y;
    mdec_upload_probe_.gpu_copy_dst_x[index] = dst_x;
    mdec_upload_probe_.gpu_copy_dst_y[index] = dst_y;
    mdec_upload_probe_.gpu_copy_w[index] = w;
    mdec_upload_probe_.gpu_copy_h[index] = h;
    ++mdec_upload_probe_.gpu_copy_sample_count;
}

double System::target_fps() const {
    const DisplayMode& mode = gpu_.display_mode();
    const bool effective_interlaced = mode.interlaced && (mode.vres != 0);
    if (mode.is_pal) {
        return effective_interlaced ? kPalInterlacedFps : kPalProgressiveFps;
    }
    return effective_interlaced ? kNtscInterlacedFps : kNtscProgressiveFps;
}

void System::run_frame(bool sample_display_diag, bool skip_spu_for_turbo) {
    auto start_total = std::chrono::high_resolution_clock::now();
    reset_profiling_stats();
    spu_skip_sync_for_turbo_ = skip_spu_for_turbo;
    const bool profile_detailed = g_profile_detailed_timing;
    apply_ram_reaper_for_frame();
    apply_gpu_reaper_for_frame();
    apply_sound_reaper_for_frame();

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
    // Aggressive fast mode intentionally trades timing stability for throughput.
    const bool fast_mode = g_gpu_fast_mode;
    const bool aggressive_fast_mode = fast_mode && g_gpu_extreme_fast_mode;
    const u32 cpu_instruction_slice =
        aggressive_fast_mode ? 256u : (fast_mode ? 128u : 32u);
    // FMV/CD streaming is sensitive to DMA and CDROM service jitter.
    // Keep those devices at near-baseline cadence even in fast mode.
    const u32 dma_tick_stride = 16u;
    const u32 spu_sync_scanline_stride =
        aggressive_fast_mode ? 32u : (fast_mode ? 16u : 4u);
    const u32 cdrom_tick_scanline_stride = 1u;
    u32 extra_cycle_error = 0;
    u32 dma_tick_budget = 0;
    u32 cdrom_tick_budget = 0;

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
        const double gpu_ms_before_loop = profiling_stats_.gpu_ms;
        if (profile_detailed) {
            start_loop = std::chrono::high_resolution_clock::now();
        }
        u32 cycles_remaining = cycles_this_scanline;
        while (cycles_remaining > 0) {
            const u32 target_slice_cycles =
                std::min(cycles_remaining, cpu_instruction_slice * 4u);
            u32 spent_in_slice = 0;
            u32 instructions_executed = 0;
            u32 sio_slice_cycles = 0;
            while (cycles_remaining > 0 && spent_in_slice < target_slice_cycles &&
                   instructions_executed < cpu_instruction_slice) {
                const u32 consumed = cpu_.step();
                spent_in_slice += consumed;
                frame_cycles_ += consumed;
                if (fast_mode) {
                    sio_slice_cycles += consumed;
                }
                else {
                    // Advance SIO at instruction granularity so JOYPAD serial
                    // handshakes don't stall for an entire scanline worth of CPU
                    // polling loops.
                    sio_.tick(consumed);
                }
                ++instructions_executed;
                cycles_remaining =
                    (consumed >= cycles_remaining) ? 0 : (cycles_remaining - consumed);
            }
            if (fast_mode && sio_slice_cycles > 0) {
                sio_.tick(sio_slice_cycles);
            }

            dma_tick_budget += spent_in_slice;
            while (dma_tick_budget >= dma_tick_stride) {
                dma_.tick();
                dma_tick_budget -= dma_tick_stride;
            }
        }
        if (dma_tick_budget > 0) {
            dma_.tick();
            dma_tick_budget = 0;
        }
        if (profile_detailed) {
            const auto end_loop = std::chrono::high_resolution_clock::now();
            const double loop_ms =
                std::chrono::duration<double, std::milli>(end_loop - start_loop)
                    .count();
            const double gpu_ms_inside_loop =
                profiling_stats_.gpu_ms - gpu_ms_before_loop;
            add_cpu_time(std::max(0.0, loop_ms - gpu_ms_inside_loop));
        }

        // HBlank-sourced timer events still occur at scanline boundaries.
        std::chrono::high_resolution_clock::time_point start_timers{};
        if (profile_detailed) {
            start_timers = std::chrono::high_resolution_clock::now();
        }
        timers_.hblank_pulse();
        if (profile_detailed) {
            const auto end_timers = std::chrono::high_resolution_clock::now();
            add_timers_time(
                std::chrono::duration<double, std::milli>(end_timers - start_timers)
                .count());
        }

        cdrom_tick_budget += cycles_this_scanline;
        if (((scanline + 1u) % cdrom_tick_scanline_stride) == 0u ||
            ((scanline + 1u) == scanlines_per_frame)) {
            cdrom_.tick(cdrom_tick_budget);
            cdrom_tick_budget = 0;
        }
        const bool sync_spu_now =
            (((scanline + 1u) % spu_sync_scanline_stride) == 0u) ||
            ((scanline + 1u) == scanlines_per_frame);
        if (sync_spu_now) {
            sync_spu_to_cpu();
        }

        if (scanline == vblank_scanline) {
            gpu_.vblank();
            irq_.request(Interrupt::VBlank);
        }
    }
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
    boot_diag_.cd_irq_int2_count = cdrom_.irq_int2_count();
    boot_diag_.cd_irq_int3_count = cdrom_.irq_int3_count();
    boot_diag_.cd_irq_int4_count = cdrom_.irq_int4_count();
    boot_diag_.cd_irq_int5_count = cdrom_.irq_int5_count();

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
    spu_skip_sync_for_turbo_ = false;
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

void System::set_ram_reaper_config(const RamReaperConfig& config) {
    const RamReaperConfig sanitized = sanitize_ram_reaper_config(config);
    ram_reaper_enabled_.store(sanitized.enabled, std::memory_order_release);
    ram_reaper_range_start_.store(sanitized.range_start, std::memory_order_release);
    ram_reaper_range_end_.store(sanitized.range_end, std::memory_order_release);
    ram_reaper_writes_per_frame_.store(sanitized.writes_per_frame,
        std::memory_order_release);
    const u32 intensity_x10 = static_cast<u32>(std::lround(
        static_cast<double>(sanitized.intensity_percent) * 10.0));
    ram_reaper_intensity_x10_.store(intensity_x10, std::memory_order_release);
    ram_reaper_affect_main_ram_.store(sanitized.affect_main_ram,
        std::memory_order_release);
    ram_reaper_affect_vram_.store(sanitized.affect_vram, std::memory_order_release);
    ram_reaper_affect_spu_ram_.store(sanitized.affect_spu_ram,
        std::memory_order_release);
    ram_reaper_use_custom_seed_.store(sanitized.use_custom_seed,
        std::memory_order_release);
    ram_reaper_seed_.store(sanitized.seed, std::memory_order_release);
}

System::RamReaperConfig System::ram_reaper_config() const {
    RamReaperConfig cfg{};
    cfg.enabled = ram_reaper_enabled_.load(std::memory_order_acquire);
    cfg.range_start = ram_reaper_range_start_.load(std::memory_order_acquire);
    cfg.range_end = ram_reaper_range_end_.load(std::memory_order_acquire);
    cfg.writes_per_frame =
        ram_reaper_writes_per_frame_.load(std::memory_order_acquire);
    cfg.intensity_percent =
        static_cast<float>(ram_reaper_intensity_x10_.load(std::memory_order_acquire)) /
        10.0f;
    cfg.affect_main_ram =
        ram_reaper_affect_main_ram_.load(std::memory_order_acquire);
    cfg.affect_vram = ram_reaper_affect_vram_.load(std::memory_order_acquire);
    cfg.affect_spu_ram =
        ram_reaper_affect_spu_ram_.load(std::memory_order_acquire);
    cfg.use_custom_seed =
        ram_reaper_use_custom_seed_.load(std::memory_order_acquire);
    cfg.seed = ram_reaper_seed_.load(std::memory_order_acquire);
    return sanitize_ram_reaper_config(cfg);
}

void System::disable_ram_reaper() {
    RamReaperConfig cfg = ram_reaper_config();
    cfg.enabled = false;
    set_ram_reaper_config(cfg);
}

void System::set_gpu_reaper_config(const GpuReaperConfig& config) {
    const GpuReaperConfig sanitized = sanitize_gpu_reaper_config(config);
    gpu_reaper_enabled_.store(sanitized.enabled, std::memory_order_release);
    gpu_reaper_writes_per_frame_.store(sanitized.writes_per_frame,
        std::memory_order_release);
    const u32 intensity_x10 = static_cast<u32>(std::lround(
        static_cast<double>(sanitized.intensity_percent) * 10.0));
    gpu_reaper_intensity_x10_.store(intensity_x10, std::memory_order_release);
    gpu_reaper_affect_geometry_.store(sanitized.affect_geometry,
        std::memory_order_release);
    gpu_reaper_affect_texture_state_.store(sanitized.affect_texture_state,
        std::memory_order_release);
    gpu_reaper_affect_display_state_.store(sanitized.affect_display_state,
        std::memory_order_release);
    gpu_reaper_use_custom_seed_.store(sanitized.use_custom_seed,
        std::memory_order_release);
    gpu_reaper_seed_.store(sanitized.seed, std::memory_order_release);
}

System::GpuReaperConfig System::gpu_reaper_config() const {
    GpuReaperConfig cfg{};
    cfg.enabled = gpu_reaper_enabled_.load(std::memory_order_acquire);
    cfg.writes_per_frame =
        gpu_reaper_writes_per_frame_.load(std::memory_order_acquire);
    cfg.intensity_percent =
        static_cast<float>(gpu_reaper_intensity_x10_.load(std::memory_order_acquire)) /
        10.0f;
    cfg.affect_geometry =
        gpu_reaper_affect_geometry_.load(std::memory_order_acquire);
    cfg.affect_texture_state =
        gpu_reaper_affect_texture_state_.load(std::memory_order_acquire);
    cfg.affect_display_state =
        gpu_reaper_affect_display_state_.load(std::memory_order_acquire);
    cfg.use_custom_seed =
        gpu_reaper_use_custom_seed_.load(std::memory_order_acquire);
    cfg.seed = gpu_reaper_seed_.load(std::memory_order_acquire);
    return sanitize_gpu_reaper_config(cfg);
}

void System::disable_gpu_reaper() {
    GpuReaperConfig cfg = gpu_reaper_config();
    cfg.enabled = false;
    set_gpu_reaper_config(cfg);
}

void System::set_sound_reaper_config(const SoundReaperConfig& config) {
    const SoundReaperConfig sanitized = sanitize_sound_reaper_config(config);
    sound_reaper_enabled_.store(sanitized.enabled, std::memory_order_release);
    sound_reaper_writes_per_frame_.store(sanitized.writes_per_frame,
        std::memory_order_release);
    const u32 intensity_x10 = static_cast<u32>(std::lround(
        static_cast<double>(sanitized.intensity_percent) * 10.0));
    sound_reaper_intensity_x10_.store(intensity_x10, std::memory_order_release);
    sound_reaper_affect_pitch_.store(sanitized.affect_pitch, std::memory_order_release);
    sound_reaper_affect_envelope_.store(sanitized.affect_envelope,
        std::memory_order_release);
    sound_reaper_affect_reverb_.store(sanitized.affect_reverb, std::memory_order_release);
    sound_reaper_affect_mixer_.store(sanitized.affect_mixer, std::memory_order_release);
    sound_reaper_use_custom_seed_.store(sanitized.use_custom_seed,
        std::memory_order_release);
    sound_reaper_seed_.store(sanitized.seed, std::memory_order_release);
}

System::SoundReaperConfig System::sound_reaper_config() const {
    SoundReaperConfig cfg{};
    cfg.enabled = sound_reaper_enabled_.load(std::memory_order_acquire);
    cfg.writes_per_frame =
        sound_reaper_writes_per_frame_.load(std::memory_order_acquire);
    cfg.intensity_percent =
        static_cast<float>(sound_reaper_intensity_x10_.load(std::memory_order_acquire)) /
        10.0f;
    cfg.affect_pitch = sound_reaper_affect_pitch_.load(std::memory_order_acquire);
    cfg.affect_envelope = sound_reaper_affect_envelope_.load(std::memory_order_acquire);
    cfg.affect_reverb = sound_reaper_affect_reverb_.load(std::memory_order_acquire);
    cfg.affect_mixer = sound_reaper_affect_mixer_.load(std::memory_order_acquire);
    cfg.use_custom_seed =
        sound_reaper_use_custom_seed_.load(std::memory_order_acquire);
    cfg.seed = sound_reaper_seed_.load(std::memory_order_acquire);
    return sanitize_sound_reaper_config(cfg);
}

void System::disable_sound_reaper() {
    SoundReaperConfig cfg = sound_reaper_config();
    cfg.enabled = false;
    set_sound_reaper_config(cfg);
}

void System::apply_ram_reaper_for_frame() {
    const RamReaperConfig cfg = ram_reaper_config();
    if (!cfg.enabled) {
        ram_reaper_prev_enabled_ = false;
        ram_reaper_rng_seeded_ = false;
        return;
    }

    bool reseed = !ram_reaper_rng_seeded_ || !ram_reaper_prev_enabled_;
    if (cfg.use_custom_seed != ram_reaper_prev_use_custom_seed_) {
        reseed = true;
    }
    if (cfg.use_custom_seed && (cfg.seed != ram_reaper_prev_seed_)) {
        reseed = true;
    }

    if (reseed) {
        const u64 seed =
            cfg.use_custom_seed
                ? cfg.seed
                : ((static_cast<u64>(std::random_device{}()) << 32) ^
                   static_cast<u64>(std::random_device{}()));
        seed_mt19937(ram_reaper_rng_, seed);
        ram_reaper_last_seed_.store(seed, std::memory_order_release);
        ram_reaper_rng_seeded_ = true;
    }

    ram_reaper_prev_enabled_ = true;
    ram_reaper_prev_use_custom_seed_ = cfg.use_custom_seed;
    ram_reaper_prev_seed_ = cfg.seed;

    if (cfg.writes_per_frame == 0 || cfg.intensity_percent <= 0.0f) {
        return;
    }

    const bool target_main = cfg.affect_main_ram;
    const bool target_vram = cfg.affect_vram;
    const bool target_spu = cfg.affect_spu_ram;
    const u32 target_count = static_cast<u32>(target_main ? 1u : 0u) +
        static_cast<u32>(target_vram ? 1u : 0u) +
        static_cast<u32>(target_spu ? 1u : 0u);
    if (target_count == 0) {
        return;
    }

    const double desired_writes =
        static_cast<double>(cfg.writes_per_frame) *
        (static_cast<double>(cfg.intensity_percent) / 100.0);
    u32 writes_this_frame = static_cast<u32>(std::floor(desired_writes));
    const double frac = desired_writes - static_cast<double>(writes_this_frame);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    if (unit_dist(ram_reaper_rng_) < frac) {
        ++writes_this_frame;
    }

    if (writes_this_frame == 0) {
        return;
    }

    // Add occasional bursty spikes for a cartridge-tilt-like feel.
    std::uniform_int_distribution<u32> pct_dist(0u, 99u);
    const u32 burst_chance = static_cast<u32>(cfg.intensity_percent * 0.35f);
    if (pct_dist(ram_reaper_rng_) < burst_chance) {
        const u32 burst_cap = std::max<u32>(1u, writes_this_frame / 2u + 1u);
        std::uniform_int_distribution<u32> burst_dist(1u, burst_cap);
        writes_this_frame += burst_dist(ram_reaper_rng_);
    }

    std::uniform_int_distribution<u32> addr_dist(cfg.range_start, cfg.range_end);
    std::uniform_int_distribution<u32> byte_dist(0u, 0xFFu);
    std::uniform_int_distribution<u32> vram_dist(
        0u, static_cast<u32>(psx::VRAM_WIDTH * psx::VRAM_HEIGHT - 1u));
    std::uniform_int_distribution<u32> spu_dist(0u, Spu::RAM_SIZE_BYTES - 1u);
    std::uniform_int_distribution<u32> target_dist(0u, target_count - 1u);
    u64 mutations = 0;
    for (u32 i = 0; i < writes_this_frame; ++i) {
        const u32 target_index = target_dist(ram_reaper_rng_);
        u32 cursor = 0;
        if (target_main) {
            if (cursor == target_index) {
                const u32 offset = addr_dist(ram_reaper_rng_);
                ram_.write8(offset, static_cast<u8>(byte_dist(ram_reaper_rng_)));
                ++mutations;
                continue;
            }
            ++cursor;
        }
        if (target_vram) {
            if (cursor == target_index) {
                gpu_.corrupt_vram_word(vram_dist(ram_reaper_rng_),
                    static_cast<u16>(ram_reaper_rng_() & 0xFFFFu));
                ++mutations;
                continue;
            }
            ++cursor;
        }
        if (target_spu) {
            if (cursor == target_index) {
                spu_.corrupt_ram_byte(spu_dist(ram_reaper_rng_),
                    static_cast<u8>(byte_dist(ram_reaper_rng_)));
                ++mutations;
                continue;
            }
        }
    }
    ram_reaper_total_mutations_.fetch_add(mutations, std::memory_order_acq_rel);
}

void System::apply_gpu_reaper_for_frame() {
    const GpuReaperConfig cfg = gpu_reaper_config();
    if (!cfg.enabled) {
        gpu_reaper_prev_enabled_ = false;
        gpu_reaper_rng_seeded_ = false;
        gpu_.set_reaper_pulse(0, 0, 0);
        return;
    }

    bool reseed = !gpu_reaper_rng_seeded_ || !gpu_reaper_prev_enabled_;
    if (cfg.use_custom_seed != gpu_reaper_prev_use_custom_seed_) {
        reseed = true;
    }
    if (cfg.use_custom_seed && (cfg.seed != gpu_reaper_prev_seed_)) {
        reseed = true;
    }

    if (reseed) {
        const u64 seed =
            cfg.use_custom_seed
                ? cfg.seed
                : ((static_cast<u64>(std::random_device{}()) << 32) ^
                   static_cast<u64>(std::random_device{}()));
        seed_mt19937(gpu_reaper_rng_, seed);
        gpu_reaper_last_seed_.store(seed, std::memory_order_release);
        gpu_reaper_rng_seeded_ = true;
    }

    gpu_reaper_prev_enabled_ = true;
    gpu_reaper_prev_use_custom_seed_ = cfg.use_custom_seed;
    gpu_reaper_prev_seed_ = cfg.seed;

    if (cfg.writes_per_frame == 0 || cfg.intensity_percent <= 0.0f) {
        return;
    }

    const u32 target_count = static_cast<u32>(cfg.affect_geometry ? 1u : 0u) +
        static_cast<u32>(cfg.affect_texture_state ? 1u : 0u) +
        static_cast<u32>(cfg.affect_display_state ? 1u : 0u);
    if (target_count == 0) {
        return;
    }

    const double desired_writes =
        static_cast<double>(cfg.writes_per_frame) *
        (static_cast<double>(cfg.intensity_percent) / 100.0);
    u32 writes_this_frame = static_cast<u32>(std::floor(desired_writes));
    const double frac = desired_writes - static_cast<double>(writes_this_frame);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    if (unit_dist(gpu_reaper_rng_) < frac) {
        ++writes_this_frame;
    }
    if (writes_this_frame == 0) {
        return;
    }

    std::uniform_int_distribution<u32> pct_dist(0u, 99u);
    const u32 burst_chance = static_cast<u32>(cfg.intensity_percent * 0.40f);
    if (pct_dist(gpu_reaper_rng_) < burst_chance) {
        const u32 burst_cap = std::max<u32>(1u, writes_this_frame / 2u + 1u);
        std::uniform_int_distribution<u32> burst_dist(1u, burst_cap);
        writes_this_frame += burst_dist(gpu_reaper_rng_);
    }

    std::uniform_int_distribution<u32> target_dist(0u, target_count - 1u);
    u32 geometry_mutations = 0;
    u32 texture_mutations = 0;
    u64 mutations = 0;
    for (u32 i = 0; i < writes_this_frame; ++i) {
        const u32 target_index = target_dist(gpu_reaper_rng_);
        u32 cursor = 0;
        if (cfg.affect_geometry) {
            if (cursor == target_index) {
                ++geometry_mutations;
                ++mutations;
                continue;
            }
            ++cursor;
        }
        if (cfg.affect_texture_state) {
            if (cursor == target_index) {
                ++texture_mutations;
                ++mutations;
                continue;
            }
            ++cursor;
        }
        if (cfg.affect_display_state && cursor == target_index) {
            gpu_.corrupt_render_state(6u + (gpu_reaper_rng_() % 3u),
                gpu_reaper_rng_());
            ++mutations;
        }
    }
    gpu_.set_reaper_pulse(geometry_mutations, texture_mutations, gpu_reaper_rng_());
    gpu_reaper_total_mutations_.fetch_add(mutations, std::memory_order_acq_rel);
}

void System::apply_sound_reaper_for_frame() {
    const SoundReaperConfig cfg = sound_reaper_config();
    if (!cfg.enabled) {
        sound_reaper_prev_enabled_ = false;
        sound_reaper_rng_seeded_ = false;
        return;
    }

    bool reseed = !sound_reaper_rng_seeded_ || !sound_reaper_prev_enabled_;
    if (cfg.use_custom_seed != sound_reaper_prev_use_custom_seed_) {
        reseed = true;
    }
    if (cfg.use_custom_seed && (cfg.seed != sound_reaper_prev_seed_)) {
        reseed = true;
    }

    if (reseed) {
        const u64 seed =
            cfg.use_custom_seed
                ? cfg.seed
                : ((static_cast<u64>(std::random_device{}()) << 32) ^
                   static_cast<u64>(std::random_device{}()));
        seed_mt19937(sound_reaper_rng_, seed);
        sound_reaper_last_seed_.store(seed, std::memory_order_release);
        sound_reaper_rng_seeded_ = true;
    }

    sound_reaper_prev_enabled_ = true;
    sound_reaper_prev_use_custom_seed_ = cfg.use_custom_seed;
    sound_reaper_prev_seed_ = cfg.seed;

    if (cfg.writes_per_frame == 0 || cfg.intensity_percent <= 0.0f) {
        return;
    }

    const u32 target_count = static_cast<u32>(cfg.affect_pitch ? 1u : 0u) +
        static_cast<u32>(cfg.affect_envelope ? 1u : 0u) +
        static_cast<u32>(cfg.affect_reverb ? 1u : 0u) +
        static_cast<u32>(cfg.affect_mixer ? 1u : 0u);
    if (target_count == 0) {
        return;
    }

    const double desired_writes =
        static_cast<double>(cfg.writes_per_frame) *
        (static_cast<double>(cfg.intensity_percent) / 100.0);
    u32 writes_this_frame = static_cast<u32>(std::floor(desired_writes));
    const double frac = desired_writes - static_cast<double>(writes_this_frame);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    if (unit_dist(sound_reaper_rng_) < frac) {
        ++writes_this_frame;
    }
    if (writes_this_frame == 0) {
        return;
    }

    std::uniform_int_distribution<u32> pct_dist(0u, 99u);
    const u32 burst_chance = static_cast<u32>(cfg.intensity_percent * 0.35f);
    if (pct_dist(sound_reaper_rng_) < burst_chance) {
        const u32 burst_cap = std::max<u32>(1u, writes_this_frame / 2u + 1u);
        std::uniform_int_distribution<u32> burst_dist(1u, burst_cap);
        writes_this_frame += burst_dist(sound_reaper_rng_);
    }

    std::uniform_int_distribution<u32> target_dist(0u, target_count - 1u);
    u64 mutations = 0;
    for (u32 i = 0; i < writes_this_frame; ++i) {
        const u32 target_index = target_dist(sound_reaper_rng_);
        u32 cursor = 0;
        if (cfg.affect_pitch) {
          if (cursor == target_index) {
            spu_.corrupt_runtime_state(sound_reaper_rng_() % 1u, sound_reaper_rng_());
            ++mutations;
            continue;
          }
          ++cursor;
        }
        if (cfg.affect_envelope) {
          if (cursor == target_index) {
            spu_.corrupt_runtime_state(1u + (sound_reaper_rng_() % 2u), sound_reaper_rng_());
            ++mutations;
            continue;
          }
          ++cursor;
        }
        if (cfg.affect_reverb) {
          if (cursor == target_index) {
            spu_.corrupt_runtime_state(3u + (sound_reaper_rng_() % 2u), sound_reaper_rng_());
            ++mutations;
            continue;
          }
          ++cursor;
        }
        if (cfg.affect_mixer && cursor == target_index) {
          spu_.corrupt_runtime_state(5u + (sound_reaper_rng_() % 3u), sound_reaper_rng_());
          ++mutations;
        }
    }
    sound_reaper_total_mutations_.fetch_add(mutations, std::memory_order_acq_rel);
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
    static BusWarnLimiter unhandled_r8_io;
    static BusWarnLimiter unhandled_r8;
    u32 phys = psx::mask_address(addr);
    if (g_trace_bus && phys >= 0x1F801000 && phys < 0x1F803000) {
        static u64 bus_r8_count = 0;
        if (trace_should_log(bus_r8_count, g_trace_burst_bus, g_trace_stride_bus)) {
            LOG_DEBUG("BUS: R8  addr=0x%08X phys=0x%08X", addr, phys);
        }
    }

    // Main RAM (2MB) mirrored across the low 8MB physical window.
    if (phys < kMainRamMirrorWindow) {
        const u32 ram_addr = phys & 0x1FFFFF;
        const u8 value = ram_.read8(ram_addr);
        debug_note_main_ram_read(ram_addr, value, 1);
        return value;
    }

    // BIOS
    if (phys >= psx::BIOS_BASE &&
        static_cast<u64>(phys) <
        (static_cast<u64>(psx::BIOS_BASE) + bios_.mapped_size()))
        return bios_.read8(phys - psx::BIOS_BASE);

    // Scratchpad (1 KiB mirrored within 0x1F800000-0x1F800FFF).
    if (phys >= 0x1F800000 && phys < 0x1F801000)
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
        // DMA registers (byte access)
        if (io >= 0x080 && io < 0x100) {
            const u32 dma_off = (io - 0x080) & ~0x3u;
            const u32 word = dma_.read(dma_off);
            const u32 shift = (io & 0x3u) * 8u;
            return static_cast<u8>((word >> shift) & 0xFFu);
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

        log_unhandled_bus_read(unhandled_r8_io, "read8", phys, true);
        return 0xFF;
    }

    // Expansion 1
    if (phys >= 0x1F000000 && phys < 0x1F800000)
        return 0xFF;
    // KSEG2/unused high virtual space: model as open bus.
    if (phys >= 0xC0000000u)
        return 0xFF;

    log_unhandled_bus_read(unhandled_r8, "read8", phys, false);
    return 0xFF;
}

u16 System::read16(u32 addr) {
    static BusWarnLimiter unhandled_r16_io;
    static BusWarnLimiter unhandled_r16;
    u32 phys = psx::mask_address(addr);
    if (g_trace_bus && phys >= 0x1F801000 && phys < 0x1F803000) {
        static u64 bus_r16_count = 0;
        if (trace_should_log(bus_r16_count, g_trace_burst_bus,
            g_trace_stride_bus)) {
            LOG_DEBUG("BUS: R16 addr=0x%08X phys=0x%08X", addr, phys);
        }
    }

    if (phys < kMainRamMirrorWindow) {
        const u32 ram_addr = phys & 0x1FFFFF;
        const u16 value = ram_.read16(ram_addr);
        debug_note_main_ram_read(ram_addr, value, 2);
        return value;
    }
    if (phys >= psx::BIOS_BASE &&
        static_cast<u64>(phys) <
        (static_cast<u64>(psx::BIOS_BASE) + bios_.mapped_size()))
        return bios_.read16(phys - psx::BIOS_BASE);
    if (phys >= 0x1F800000 && phys < 0x1F801000)
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
        // DMA registers (halfword access)
        if (io >= 0x080 && io < 0x100) {
            const u32 dma_off = (io - 0x080) & ~0x3u;
            const u32 word = dma_.read(dma_off);
            const u32 shift = (io & 0x2u) * 8u;
            return static_cast<u16>((word >> shift) & 0xFFFFu);
        }
        // Timers
        if (io >= 0x100 && io < 0x130)
            return static_cast<u16>(timers_.read(io - 0x100));
        // SIO
        if (io >= 0x040 && io < 0x050) {
            note_sio_io(phys);
            return sio_.read16(io - 0x040);
        }
        if (g_cpu_deep_diagnostics && io >= 0x058 && io < 0x060) {
            static u32 io105x_read16_count = 0;
            if (io105x_read16_count < 8u) {
                ++io105x_read16_count;
                LOG_WARN(
                    "BUS: io105x read16 phys=0x%08X pc=0x%08X sp=0x%08X ra=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X",
                    phys, cpu_.pc(), cpu_.reg(29), cpu_.reg(31), cpu_.reg(4),
                    cpu_.reg(5), cpu_.reg(6), cpu_.reg(7));
            }
        }
        // CDROM
        if (io >= 0x800 && io < 0x804) {
            note_cdrom_io(phys);
            const u32 port = io - 0x800;
            const u16 lo = cdrom_.read8(port);
            // RDDATA is a byte FIFO at 1F801802h. Wider reads starting there
            // must keep consuming FIFO bytes rather than spilling into 803h.
            const u16 hi = cdrom_.read8(port == 2 ? 2u : std::min<u32>(port + 1, 3));
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

        log_unhandled_bus_read(unhandled_r16_io, "read16", phys, true);
        return 0xFFFF;
    }

    if (phys >= 0xC0000000u)
        return 0xFFFF;
    log_unhandled_bus_read(unhandled_r16, "read16", phys, false);
    return 0xFFFF;
}

u32 System::read32(u32 addr) {
    static BusWarnLimiter unhandled_r32_io;
    static BusWarnLimiter unhandled_r32;
    u32 phys = psx::mask_address(addr);
    if (g_trace_bus && phys >= 0x1F801000 && phys < 0x1F803000) {
        static u64 bus_r32_count = 0;
        if (trace_should_log(bus_r32_count, g_trace_burst_bus,
            g_trace_stride_bus)) {
            LOG_DEBUG("BUS: R32 addr=0x%08X phys=0x%08X", addr, phys);
        }
    }

    if (phys < kMainRamMirrorWindow) {
        const u32 ram_addr = phys & 0x1FFFFF;
        const u32 value = ram_.read32(ram_addr);

        debug_note_main_ram_read(ram_addr, value, 4);
        return value;
    }
    if (phys >= psx::BIOS_BASE &&
        static_cast<u64>(phys) <
        (static_cast<u64>(psx::BIOS_BASE) + bios_.mapped_size()))
        return bios_.read32(phys - psx::BIOS_BASE);
    if (phys >= 0x1F800000 && phys < 0x1F801000)
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
        if (io == 0x060) {
            log_ram_size_access("read", ram_size_, cpu_.pc(), cpu_.reg(29),
                cpu_.reg(31));
            return ram_size_;
        }
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
            const u32 port = io - 0x800;
            const u32 b0 = cdrom_.read8(port);
            // RDDATA is a byte FIFO at 1F801802h. Wider reads starting there
            // must keep consuming FIFO bytes rather than spilling into 803h.
            const u32 b1 = cdrom_.read8(port == 2 ? 2u : std::min<u32>(port + 1, 3));
            const u32 b2 = cdrom_.read8(port == 2 ? 2u : std::min<u32>(port + 2, 3));
            const u32 b3 = cdrom_.read8(port == 2 ? 2u : std::min<u32>(port + 3, 3));
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

        log_unhandled_bus_read(unhandled_r32_io, "read32", phys, true);
        return 0xFFFFFFFFu;
    }

    // Expansion 1
    if (phys >= 0x1F000000 && phys < 0x1F800000)
        return 0xFFFFFFFF;

    // KSEG2 (cache control)
    if (addr == 0xFFFE0130)
        return cache_ctrl_;
    if (phys >= 0xC0000000u)
        return 0xFFFFFFFFu;

    log_unhandled_bus_read(unhandled_r32, "read32", phys, false);
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

    if (phys < kMainRamMirrorWindow) {
        if (g_ram_watch_diagnostics) {
            maybe_log_ram_watch_write(phys, val, 1);
        }
        const u32 ram_addr = phys & 0x1FFFFF;
        ram_.write8(ram_addr, val);
        debug_note_main_ram_write(ram_addr, val, 1);
        return;
    }
    if (phys >= 0x1F800000 && phys < 0x1F801000) {
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
        if (io >= 0x080 && io < 0x100) {
            const u32 dma_off = (io - 0x080) & ~0x3u;
            const u32 shift = (io & 0x3u) * 8u;
            const u32 mask = 0xFFu << shift;
            const u32 merged =
                (dma_.read(dma_off) & ~mask) | (static_cast<u32>(val) << shift);
            dma_.write(dma_off, merged);
            return;
        }
        if (io >= 0x800 && io < 0x804) {
            note_cdrom_io(phys);
            cdrom_.write8(io - 0x800, val);
            return;
        }
        if (io >= 0x810 && io < 0x818) {
            const u32 reg = io & ~0x3u;
            const u32 shift = (io & 0x3u) * 8u;
            const u32 mask = 0xFFu << shift;
            if (reg == 0x810) {
                gpu_gp0_shadow_ =
                    (gpu_gp0_shadow_ & ~mask) | (static_cast<u32>(val) << shift);
                gpu_gp0_shadow_mask_ |= mask;
                if (gpu_gp0_shadow_mask_ == 0xFFFFFFFFu) {
                    gpu_gp0(gpu_gp0_shadow_);
                    gpu_gp0_shadow_ = 0;
                    gpu_gp0_shadow_mask_ = 0;
                }
            }
            if (reg == 0x814) {
                gpu_gp1_shadow_ =
                    (gpu_gp1_shadow_ & ~mask) | (static_cast<u32>(val) << shift);
                gpu_gp1_shadow_mask_ |= mask;
                if (gpu_gp1_shadow_mask_ == 0xFFFFFFFFu) {
                    gpu_.gp1(gpu_gp1_shadow_);
                    gpu_gp1_shadow_ = 0;
                    gpu_gp1_shadow_mask_ = 0;
                }
            }
            return;
        }
        if (io >= 0x820 && io < 0x828) {
            const u32 reg = io & ~0x3u;
            const u32 shift = (io & 0x3u) * 8u;
            const u32 mask = 0xFFu << shift;
            if (reg == 0x820) {
                mdec_command_shadow_ =
                    (mdec_command_shadow_ & ~mask) | (static_cast<u32>(val) << shift);
                mdec_command_shadow_mask_ |= mask;
                if (mdec_command_shadow_mask_ == 0xFFFFFFFFu) {
                    mdec_.write_command(mdec_command_shadow_);
                    mdec_command_shadow_ = 0;
                    mdec_command_shadow_mask_ = 0;
                }
            }
            if (reg == 0x824) {
                mdec_control_shadow_ =
                    (mdec_control_shadow_ & ~mask) | (static_cast<u32>(val) << shift);
                mdec_control_shadow_mask_ |= mask;
                if (mdec_control_shadow_mask_ == 0xFFFFFFFFu) {
                    mdec_.write_control(mdec_control_shadow_);
                    mdec_control_shadow_ = 0;
                    mdec_control_shadow_mask_ = 0;
                }
            }
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
    if (phys >= 0xC0000000u)
        return;

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

    if (phys < kMainRamMirrorWindow) {
        if (g_ram_watch_diagnostics) {
            maybe_log_ram_watch_write(phys, val, 2);
        }
        const u32 ram_addr = phys & 0x1FFFFF;
        ram_.write16(ram_addr, val);
        debug_note_main_ram_write(ram_addr, val, 2);
        return;
    }
    if (phys >= 0x1F800000 && phys < 0x1F801000) {
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
        if (io >= 0x080 && io < 0x100) {
            const u32 dma_off = (io - 0x080) & ~0x3u;
            const u32 shift = (io & 0x2u) * 8u;
            const u32 mask = 0xFFFFu << shift;
            const u32 merged =
                (dma_.read(dma_off) & ~mask) | (static_cast<u32>(val) << shift);
            dma_.write(dma_off, merged);
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
        if (g_cpu_deep_diagnostics && io >= 0x058 && io < 0x060) {
            static u32 io105x_write16_count = 0;
            if (io105x_write16_count < 8u) {
                ++io105x_write16_count;
                LOG_WARN(
                    "BUS: io105x write16 phys=0x%08X val=0x%04X pc=0x%08X sp=0x%08X ra=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X",
                    phys, val, cpu_.pc(), cpu_.reg(29), cpu_.reg(31),
                    cpu_.reg(4), cpu_.reg(5), cpu_.reg(6), cpu_.reg(7));
            }
        }
        if (io >= 0x800 && io < 0x804) {
            note_cdrom_io(phys);
            cdrom_.write8(io - 0x800, static_cast<u8>(val & 0xFF));
            cdrom_.write8(std::min<u32>(io - 0x800 + 1, 3),
                static_cast<u8>((val >> 8) & 0xFF));
            return;
        }
        if (io >= 0x810 && io < 0x818) {
            const u32 reg = io & ~0x3u;
            const u32 shift = (io & 0x2u) * 8u;
            const u32 mask = 0xFFFFu << shift;
            if (reg == 0x810) {
                gpu_gp0_shadow_ =
                    (gpu_gp0_shadow_ & ~mask) | (static_cast<u32>(val) << shift);
                gpu_gp0_shadow_mask_ |= mask;
                if (gpu_gp0_shadow_mask_ == 0xFFFFFFFFu) {
                    gpu_gp0(gpu_gp0_shadow_);
                    gpu_gp0_shadow_ = 0;
                    gpu_gp0_shadow_mask_ = 0;
                }
            }
            if (reg == 0x814) {
                gpu_gp1_shadow_ =
                    (gpu_gp1_shadow_ & ~mask) | (static_cast<u32>(val) << shift);
                gpu_gp1_shadow_mask_ |= mask;
                if (gpu_gp1_shadow_mask_ == 0xFFFFFFFFu) {
                    gpu_.gp1(gpu_gp1_shadow_);
                    gpu_gp1_shadow_ = 0;
                    gpu_gp1_shadow_mask_ = 0;
                }
            }
            return;
        }
        if (io >= 0x820 && io < 0x828) {
            const u32 reg = io & ~0x3u;
            const u32 shift = (io & 0x2u) * 8u;
            const u32 mask = 0xFFFFu << shift;
            if (reg == 0x820) {
                mdec_command_shadow_ =
                    (mdec_command_shadow_ & ~mask) | (static_cast<u32>(val) << shift);
                mdec_command_shadow_mask_ |= mask;
                if (mdec_command_shadow_mask_ == 0xFFFFFFFFu) {
                    mdec_.write_command(mdec_command_shadow_);
                    mdec_command_shadow_ = 0;
                    mdec_command_shadow_mask_ = 0;
                }
            }
            if (reg == 0x824) {
                mdec_control_shadow_ =
                    (mdec_control_shadow_ & ~mask) | (static_cast<u32>(val) << shift);
                mdec_control_shadow_mask_ |= mask;
                if (mdec_control_shadow_mask_ == 0xFFFFFFFFu) {
                    mdec_.write_control(mdec_control_shadow_);
                    mdec_control_shadow_ = 0;
                    mdec_control_shadow_mask_ = 0;
                }
            }
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

    if (phys >= 0xC0000000u)
        return;
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

    if (phys < kMainRamMirrorWindow) {
        const u32 ram_addr = phys & 0x1FFFFF;
        if (g_ram_watch_diagnostics) {
            maybe_log_ram_watch_write(phys, val, 4);
        }
        ram_.write32(ram_addr, val);
        debug_note_main_ram_write(ram_addr, val, 4);
        return;
    }
    if (phys >= 0x1F800000 && phys < 0x1F801000) {
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
            log_ram_size_access("write", val, cpu_.pc(), cpu_.reg(29),
                cpu_.reg(31));
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
            gpu_gp0_shadow_ = 0;
            gpu_gp0_shadow_mask_ = 0;
            gpu_gp0(val);
            return;
        }
        if (io == 0x814) {
            gpu_gp1_shadow_ = 0;
            gpu_gp1_shadow_mask_ = 0;
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
            mdec_command_shadow_ = 0;
            mdec_command_shadow_mask_ = 0;
            mdec_.write_command(val);
            return;
        }
        if (io == 0x824) {
            mdec_control_shadow_ = 0;
            mdec_control_shadow_mask_ = 0;
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
    if (phys >= 0xC0000000u)
        return;

    LOG_WARN("BUS: Unhandled write32 at 0x%08X = 0x%08X", phys, val);
}
