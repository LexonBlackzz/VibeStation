#include "core/ps2_system.h"

#include <algorithm>
#include <chrono>

namespace ps2 {
namespace {
// Keep independent VU1 execution synchronized to EE instruction steps.
constexpr u64 kVu1InstructionsPerEeStep = 1;
constexpr u64 kQuietEeBatchLimit = 4096u;
constexpr u64 kQuietEeSuperbatchLimit = 65536u;
// Let the EE run ahead of an active IOP by a small bounded interval, then
// repay the accumulated 8:1 cycle debt in one catch-up burst. This removes
// the pathological "return to C++ every IOP instruction" behavior while
// keeping the first experiment deliberately far below PCSX2-scale windows.
constexpr u64 kActiveIopEeLeadLimit = 128u;
constexpr u32 kEeMainRamSize = 32u * 1024u * 1024u;

u64 elapsed_profile_ns(
    std::chrono::steady_clock::time_point begin) {
    return static_cast<u64>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - begin).count());
}

bool quiet_ram_span(u32 virtual_address, u32 width, u32 alignment_mask = 0u) {
    if (virtual_address >= 0xC0000000u) return false;
    const u32 aligned = virtual_address & ~alignment_mask;
    const u32 physical = EeBus::to_physical(aligned);
    return physical < kEeMainRamSize &&
           width <= kEeMainRamSize - physical;
}

bool quiet_ee_static_instruction(u32 instruction) {
    if (instruction == 0u) return true;
    const u32 opcode = instruction >> 26;
    switch (opcode) {
    case 0x00u: // SPECIAL
    case 0x01u: // REGIMM
    case 0x02u: // J
    case 0x03u: // JAL
    case 0x04u: // BEQ
    case 0x05u: // BNE
    case 0x06u: // BLEZ
    case 0x07u: // BGTZ
    case 0x08u: // ADDI
    case 0x09u: // ADDIU
    case 0x0Au: // SLTI
    case 0x0Bu: // SLTIU
    case 0x0Cu: // ANDI
    case 0x0Du: // ORI
    case 0x0Eu: // XORI
    case 0x0Fu: // LUI
    case 0x11u: // COP1
    case 0x14u: // BEQL
    case 0x15u: // BNEL
    case 0x16u: // BLEZL
    case 0x17u: // BGTZL
    case 0x18u: // DADDI
    case 0x19u: // DADDIU
    case 0x1Au: // LDL
    case 0x1Bu: // LDR
    case 0x1Cu: // MMI
    case 0x1Eu: // LQ
    case 0x1Fu: // SQ
    case 0x20u: // LB
    case 0x21u: // LH
    case 0x22u: // LWL
    case 0x23u: // LW
    case 0x24u: // LBU
    case 0x25u: // LHU
    case 0x26u: // LWR
    case 0x27u: // LWU
    case 0x28u: // SB
    case 0x29u: // SH
    case 0x2Au: // SWL
    case 0x2Bu: // SW
    case 0x2Cu: // SDL
    case 0x2Du: // SDR
    case 0x2Eu: // SWR
    case 0x2Fu: // CACHE
    case 0x30u: // LL
    case 0x31u: // LWC1
    case 0x33u: // PREF
    case 0x34u: // LLD
    case 0x36u: // LQC2
    case 0x37u: // LD
    case 0x38u: // SC
    case 0x39u: // SWC1
    case 0x3Cu: // SCD
    case 0x3Eu: // SQC2
    case 0x3Fu: // SD
        return true;
    default:
        return false;
    }
}

bool quiet_ee_control_flow(u32 instruction) {
    const u32 opcode = instruction >> 26;
    if (opcode == 0x01u) {
        const u32 rt = (instruction >> 16) & 31u;
        if (rt <= 0x03u || (rt >= 0x10u && rt <= 0x13u)) {
            return true;
        }
    }
    if ((opcode >= 0x02u && opcode <= 0x07u) ||
        (opcode >= 0x14u && opcode <= 0x17u)) {
        return true;
    }
    if (opcode == 0x00u) {
        const u32 funct = instruction & 63u;
        return funct == 0x08u || funct == 0x09u ||
               funct == 0x0Cu || funct == 0x0Du;
    }
    if (opcode == 0x11u) {
        return ((instruction >> 21) & 31u) == 0x08u;
    }
    return false;
}

bool quiet_ee_store(u32 instruction) {
    switch (instruction >> 26) {
    case 0x1Fu: // SQ
    case 0x28u: // SB
    case 0x29u: // SH
    case 0x2Au: // SWL
    case 0x2Bu: // SW
    case 0x2Cu: // SDL
    case 0x2Du: // SDR
    case 0x2Eu: // SWR
    case 0x38u: // SC
    case 0x39u: // SWC1
    case 0x3Cu: // SCD
    case 0x3Eu: // SQC2
    case 0x3Fu: // SD
        return true;
    default:
        return false;
    }
}

bool quiet_ee_instruction_value(
    const EeCpuState& state,
    u32 instruction) {
    if (!quiet_ee_static_instruction(instruction)) return false;
    if (instruction == 0u) return true;

    const u32 opcode = instruction >> 26;
    const u32 rs = (instruction >> 21) & 31u;

    // Register/control-flow/FPU/MMI operations have no EE MMIO side effects.
    switch (opcode) {
    case 0x00u:
    case 0x01u:
    case 0x02u:
    case 0x03u:
    case 0x04u:
    case 0x05u:
    case 0x06u:
    case 0x07u:
    case 0x08u:
    case 0x09u:
    case 0x0Au:
    case 0x0Bu:
    case 0x0Cu:
    case 0x0Du:
    case 0x0Eu:
    case 0x0Fu:
    case 0x11u:
    case 0x14u:
    case 0x15u:
    case 0x16u:
    case 0x17u:
    case 0x18u:
    case 0x19u:
    case 0x1Cu:
    case 0x2Fu:
    case 0x33u:
        return true;
    default:
        break;
    }

    const s16 immediate = static_cast<s16>(instruction & 0xFFFFu);
    const u32 address = static_cast<u32>(
        state.gpr[rs].lo + static_cast<u64>(static_cast<s64>(immediate)));

    switch (opcode) {
    case 0x20u:
    case 0x24u:
    case 0x28u:
        return quiet_ram_span(address, 1u);
    case 0x21u:
    case 0x25u:
    case 0x29u:
        return quiet_ram_span(address, 2u);
    case 0x22u:
    case 0x26u:
    case 0x2Au:
    case 0x2Eu:
        return quiet_ram_span(address, 4u, 3u);
    case 0x23u:
    case 0x27u:
    case 0x2Bu:
    case 0x30u:
    case 0x31u:
    case 0x38u:
    case 0x39u:
        return quiet_ram_span(address, 4u);
    case 0x1Au:
    case 0x1Bu:
    case 0x2Cu:
    case 0x2Du:
        return quiet_ram_span(address, 8u, 7u);
    case 0x34u:
    case 0x37u:
    case 0x3Cu:
    case 0x3Fu:
        return quiet_ram_span(address, 8u);
    case 0x1Eu:
    case 0x1Fu:
    case 0x36u:
    case 0x3Eu:
        return quiet_ram_span(address, 16u, 15u);
    default:
        return false;
    }
}

bool quiet_ee_instruction(
    const EeCpuState& state,
    const EeBus& bus,
    u32& instruction) {
    if (state.pc >= 0xC0000000u) return false;
    instruction = 0;
    return bus.fetch32(state.pc, instruction) &&
           quiet_ee_instruction_value(state, instruction);
}
}
Ps2System::Ps2System():cdvd_(iop_intc_,bios_),iop_bus_(iop_ram_,iop_hw_,hw_,iop_intc_,cdvd_,bios_),bus_(ram_,scratchpad_,hw_,iop_hw_,iop_ram_,cdvd_,gs_,gs_core_,bios_),vu0_(bus_,gs_core_,0x11000000u,0x11004000u,0x0FFFu,0x100038D0u,0x100038E0u,false),vu1_(bus_,gs_core_),ee_(bus_,&vu0_),iop_(iop_bus_){gs_core_.attach_privileged(gs_);vif0_dma_.attach_vu0(vu0_);vif0_dma_.attach_ee(ee_);vif1_dma_.attach_vu1(vu1_);reset();}
void Ps2System::reset(u32 entry_point) {
    ram_.reset(); scratchpad_.reset(); bus_.reset(); hw_.reset();
    iop_hw_.reset(); iop_intc_.reset(); cdvd_.reset(); iop_ram_.reset();
    iop_bus_.reset(); gs_.reset(); gs_core_.reset(); gs_display_.reset();
    scheduler_.reset(); video_timing_.reset(); gif_dma_.reset();
    vif0_dma_.reset(); vif1_dma_.reset(); sif_dma_.reset();
    spr_dma_.reset(); ipu_dma_.reset(); vu0_.reset(); vu1_.reset();
    ee_.clear_jit_cache();
    ee_.reset(entry_point); iop_.reset(Bios::kResetVector);
    bios_started_ = false;
    reset_instruction_ = 0;
    iop_reset_instruction_ = 0;
    ee_iop_phase_ = 0;
    skipped_bios_idle_iterations_ = 0;
    skipped_bios_idle_offphase_batches_ = 0;
    skipped_bios_zero_iterations_ = 0;
    skipped_bios_nibble_iterations_ = 0;
    skipped_bios_count_wait_iterations_ = 0;
    skipped_bios_countdown_iterations_ = 0;
    skipped_bios_copy_iterations_ = 0;
    skipped_bios_mmio_poll_iterations_ = 0;
    sif_poll_fast_samples_ = 0;
    sif_poll_stable_returns_ = 0;
    fast_sif_getreg_calls_ = 0;
    fast_sif_getreg_active_iop_calls_ = 0;
    fast_sif_getreg_args_.fill(0u);
    fast_sif_getreg_rejects_.fill(0u);
    fast_sif_getreg_active_iop_zero_dma_ = 0;
    fast_sif_getreg_active_iop_sif_only_ = 0;
    fast_sif_getreg_active_iop_other_dma_ = 0;
    fast_sif_getreg_active_iop_first_pc_ = 0;
    skipped_iop_idle_pairs_ = 0;
    skipped_bios_literal_iterations_ = 0;
    quiet_ee_batch_instructions_ = 0;
    quiet_ee_active_iop_instructions_ = 0;
    quiet_ee_batches_ = 0;
    quiet_block_instructions_ = 0;
    quiet_block_hits_ = 0;
    quiet_block_compiles_ = 0;
    fast_interpreter_instructions_ = 0;
    fast_interpreter_calls_ = 0;
    quiet_superbatch_calls_ = 0;
    quiet_superbatch_instructions_ = 0;
    profile_run_ns_ = 0;
    profile_ee_ns_ = 0;
    profile_iop_ns_ = 0;
    profile_slow_path_ns_ = 0;
    profile_slow_path_samples_ = 0;
    std::fill(
        quiet_ee_blocks_.begin(),
        quiet_ee_blocks_.end(),
        QuietEeBlock{});
    native_fallback_opcodes_.fill(0);
    idle_skip_reasons_.fill(0);
}
Ps2System::QuietEeBlock* Ps2System::quiet_ee_block(u32 pc) {
    if (pc >= 0xC0000000u) return nullptr;

    const u32 physical = EeBus::to_physical(pc);
    if (physical >= EeRam::kSize) return nullptr;

    const u32 page_offset = physical & (EeRam::kPageSize - 1u);
    ram_.track_code_page(physical);
    const u32 generation = ram_.page_generation(physical);
    const u64 block_hash =
        static_cast<u64>(pc >> 2) * 11400714819323198485ull;
    const std::size_t index =
        static_cast<std::size_t>(block_hash ^ (block_hash >> 32)) &
        (quiet_ee_blocks_.size() - 1u);
    QuietEeBlock& block = quiet_ee_blocks_[index];

    if (block.count != 0u &&
        block.pc == pc &&
        block.page_generation == generation) {
        ++quiet_block_hits_;
        ++block.hits;
        return &block;
    }

    block = {};
    block.pc = pc;
    block.page_generation = generation;

    const u32 instructions_left_in_page =
        (EeRam::kPageSize - page_offset) / 4u;
    const u32 limit = std::min<u32>(
        static_cast<u32>(block.words.size()),
        instructions_left_in_page);

    bool fetch_delay_slot = false;
    for (u32 i = 0; i < limit; ++i) {
        u32 instruction = 0;
        const u32 address = pc + i * 4u;
        if (!bus_.fetch32(address, instruction) ||
            !quiet_ee_static_instruction(instruction)) {
            break;
        }

        const u32 slot = block.count;
        block.words[block.count++] = instruction;
        const u32 opcode = instruction >> 26;
        switch (opcode) {
        case 0x1Au: case 0x1Bu: case 0x1Eu: case 0x1Fu:
        case 0x20u: case 0x21u: case 0x22u: case 0x23u:
        case 0x24u: case 0x25u: case 0x26u: case 0x27u:
        case 0x28u: case 0x29u: case 0x2Au: case 0x2Bu:
        case 0x2Cu: case 0x2Du: case 0x2Eu:
        case 0x30u: case 0x31u: case 0x34u:
        case 0x36u: case 0x37u: case 0x38u: case 0x39u:
        case 0x3Cu: case 0x3Eu: case 0x3Fu:
            block.memory_mask |= 1u << slot;
            break;
        default:
            break;
        }
        if (quiet_ee_store(instruction)) {
            block.store_mask |= 1u << slot;
        }

        if (fetch_delay_slot) {
            break;
        }

        // Native stores carry a page-generation write barrier. If a store
        // targets this code page the generated block exits immediately;
        // interpreter fallback below also stops after a store. This lets the
        // cache span ordinary data stores without risking stale self-modified
        // code. Control flow still caches one architectural delay slot.
        if (quiet_ee_control_flow(instruction)) {
            fetch_delay_slot = true;
        }
    }

    if (block.count == 0u) return nullptr;
    ++quiet_block_compiles_;
    return &block;
}

std::vector<std::pair<u32, u64>>
Ps2System::quiet_block_hotspots(std::size_t limit) const {
    std::vector<std::pair<u32, u64>> out;
    out.reserve(quiet_ee_blocks_.size());
    for (const auto& block : quiet_ee_blocks_) {
        if (block.count != 0u && block.hits != 0u) {
            out.emplace_back(block.pc, block.hits);
        }
    }
    std::sort(
        out.begin(), out.end(),
        [](const auto& a, const auto& b) {
            return a.second > b.second;
        });
    if (out.size() > limit) out.resize(limit);
    return out;
}

bool Ps2System::load_bios(const std::string& path,std::string& error){if(!bios_.load_file(path,error))return false;reset();return true;}
bool Ps2System::boot_bios(std::string& error){error.clear();if(!bios_.loaded()){error="No PS2 BIOS is loaded.";return false;}reset(Bios::kResetVector);if(!bus_.read32(ee_.state().pc,reset_instruction_)){error="BIOS loaded, but the EE reset vector could not be fetched.";reset();return false;}if(!iop_bus_.read32(iop_.state().pc,iop_reset_instruction_)){error="BIOS loaded, but the IOP reset vector could not be fetched.";reset();return false;}bios_started_=true;return true;}
bool Ps2System::advance_iop_for_ee_step(std::string& error){
    ++ee_iop_phase_;
    if(ee_iop_phase_<8)return true;
    ee_iop_phase_=0;

    if(iop_.halted()){
        // Bootstrap mode: retain the exact IOP halt for the debugger, but
        // allow the EE/GS side to continue far enough to expose BIOS video.
        error.clear();
        return true;
    }

    if(!iop_.step_hot(iop_step_error_scratch_)){
        if(iop_.halted()){
            error.clear();
            return true;
        }
        error="IOP step failed: "+iop_step_error_scratch_;
        return false;
    }
    // IopCpu::step already advances the IOP bus root counters. The
    // IopHwWindow's duplicate timer bank is not on the CPU-visible path.
    sif_dma_.tick_iop(iop_bus_);
    return true;
}
void Ps2System::reset_iop_subsystem(){iop_hw_.reset();iop_intc_.reset();cdvd_.reset();iop_ram_.reset();iop_bus_.reset();iop_.reset(Bios::kResetVector);ee_iop_phase_=0;(void)iop_bus_.write32(0x1F801450u,0x8u);(void)iop_intc_.write32(IopIntc::kICtrl,1u);}
bool Ps2System::step_ee(std::string& error) {
    error.clear();
    if (!bios_started_) {
        error = "BIOS has not been started.";
        return false;
    }
    if (halted()) {
        error = halt_reason();
        return false;
    }
    return step_ee_core(error);
}

bool Ps2System::step_ee_core(std::string& error) {
    if (!ee_.step(error)) {
        error = "EE halted: " + error;
        return false;
    }
    if (hw_.take_iop_interrupt_request()) iop_intc_.raise(1);
    if (hw_.take_iop_reset_request()) reset_iop_subsystem();
    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0;
    if ((active_dma & (1u << 2)) != 0 &&
        !gif_dma_.service(bus_, gs_core_, error)) {
        error = "GIF DMA: " + error;
        return false;
    }
    if ((active_dma & (1u << 0)) != 0 &&
        !vif0_dma_.service(bus_, error)) {
        error = "VIF0 DMA: " + error;
        return false;
    }
    if ((active_dma & (1u << 1)) != 0 &&
        !vif1_dma_.service(bus_, gs_core_, gs_, error)) {
        error = "VIF1 DMA: " + error;
        return false;
    }
    // The EE and IOP halves must both be armed before SIF can transfer.
    // Firmware leaves unmatched channels active for long periods; avoid
    // repeatedly probing their MMIO registers on every EE instruction.
    const u16 active_sif =
        active_dma & ((1u << 5) | (1u << 6));
    if (active_sif != 0 &&
        (active_sif & iop_bus_.sif_dma_ready_mask()) != 0 &&
        !sif_dma_.service(bus_, iop_bus_, iop_intc_, error)) {
        error = "SIF DMA: " + error;
        return false;
    }
    sif_dma_.tick_ee(bus_);
    if ((active_dma & ((1u << 8) | (1u << 9))) != 0 &&
        !spr_dma_.service(bus_, error)) {
        error = "SPR DMA: " + error;
        return false;
    }
    if ((active_dma & ((1u << 3) | (1u << 4))) != 0 &&
        !ipu_dma_.service(bus_, error)) {
        error = "IPU DMA: " + error;
        return false;
    }
    if (vu0_.running()) {
        ee_.set_vu0_micro_running(true);
        std::string vu_error;
        vu0_.run(256, vu_error);
        if (!vu_error.empty()) {
            error = "VU0: " + vu_error;
            return false;
        }
        if (!vu0_.running()) {
            ee_.set_vu0_micro_running(false);
            ee_.sync_vu0_from_micro();
        }
    }
    if (vu1_.running()) {
        std::string vu_error;
        vu1_.run(kVu1InstructionsPerEeStep, vu_error);
        if (!vu_error.empty()) {
            error = "VU1: " + vu_error;
            return false;
        }
    }
    scheduler_.advance_one();
    const u64 fields_before = video_timing_.fields_started();
    video_timing_.tick(1, hw_, iop_intc_);
    if (video_timing_.fields_started() != fields_before) {
        gs_.raise_vsync();
        // PCRTC is field-timed, but the UI presents a full bobbed frame.
        // After first visibility, scan out once per two interlaced fields so
        // the raster worker is not synchronously drained twice per frame.
        const u64 field = video_timing_.fields_started();
        if (!gs_display_.has_visible_pixels() || (field & 1u) == 0u) {
            gs_display_.update(gs_, gs_core_.vram());
        }
    }
    if (gs_.irq_pending()) hw_.raise_intc(0);
    return advance_iop_for_ee_step(error);
}
void Ps2System::advance_iop_for_ee_cycles(u64 cycles, std::string& error) {
    const u64 total_phase = ee_iop_phase_ + cycles;
    ee_iop_phase_ = static_cast<u32>(total_phase & 7u);
    const u64 steps = total_phase / 8u;
    const auto profile_begin = std::chrono::steady_clock::now();
    for (u64 i = 0; i < steps;) {
        if (iop_.halted()) break;
        if (i + 1u < steps && !sif_dma_.iop_completion_pending()) {
            const u64 pairs = iop_.skip_osdsys_idle_pairs(
                (steps - i) / 2u);
            if (pairs != 0u) {
                i += pairs * 2u;
                skipped_iop_idle_pairs_ += pairs;
                continue;
            }
        }
        if (i + 1u < steps && !sif_dma_.iop_completion_pending() &&
            iop_.skip_osdsys_idle_pair()) {
            i += 2u;
            continue;
        }

        // The IOP recompiler only executes ALU/control/direct-RAM blocks and
        // does not advance device time itself. Run it only across an interval
        // which the IOP bus proves event-free, then repay the exact retired
        // cycle count in one hardware tick. This mirrors the PS1 V4
        // run-slice model without allowing native code to skip observable IOP
        // timer/SIF/IRQ boundaries.
        if (i + 1u < steps &&
            !sif_dma_.iop_completion_pending() &&
            !iop_bus_.interrupt_pending()) {
            u64 native_budget =
                std::min<u64>(steps - i, 256u);
            while (native_budget >= 2u &&
                   !iop_bus_.can_tick_event_free(native_budget)) {
                native_budget >>= 1u;
            }
            if (native_budget >= 2u) {
                const u32 native_retired =
                    iop_.run_native_quiet(
                        static_cast<u32>(native_budget));
                if (native_retired != 0u) {
                    iop_bus_.tick(native_retired);
                    i += native_retired;
                    continue;
                }
            }
        }

        if (!iop_.step_hot(iop_step_error_scratch_)) {
            if (!iop_.halted()) {
                error = "IOP step failed: " + iop_step_error_scratch_;
            }
            break;
        }
        sif_dma_.tick_iop(iop_bus_);
        ++i;
    }
    if (steps != 0u) {
        profile_iop_ns_ += elapsed_profile_ns(profile_begin);
    }
}
bool Ps2System::step_iop(std::string& error){error.clear();if(!bios_started_){error="BIOS has not been started.";return false;}if(iop_.halted()){error=iop_.halt_reason();return false;}if(!iop_.step(error))return false;sif_dma_.tick_iop(iop_bus_);return true;}
u64 Ps2System::try_skip_bios_idle_iterations(
    u64 budget, std::string& error) {
    constexpr u64 kIdleInstructions = 8u;
    const u32 idle_pc = ee_.state().pc;
    const bool idle_offphase = idle_pc != 0x00081FC0u;
    if (idle_pc < 0x00081FC0u ||
        idle_pc > 0x00081FDCu ||
        ((idle_pc - 0x00081FC0u) & 3u) != 0u) {
        return 0;
    }
    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
    const u16 sif_channels = (1u << 5) | (1u << 6);
    const u16 ready_sif =
        (active_dma & sif_channels) &
        iop_bus_.sif_dma_ready_mask();
    const bool sif0_needs_service =
        (ready_sif & (1u << 5)) != 0u &&
        !sif_dma_.sif0_completion_pending();
    const bool sif1_needs_service =
        (ready_sif & (1u << 6)) != 0u &&
        !sif_dma_.sif1_completion_pending();
    if ((active_dma & ~sif_channels) != 0u ||
        sif0_needs_service || sif1_needs_service) {
        ++idle_skip_reasons_[2]; return 0;
    }
    if (hw_.timer_irq_possible()) {
        ++idle_skip_reasons_[7]; return 0;
    }

    // When the IOP is doing real work, the old idle skip gave up because an
    // IOP store could become EE-visible part-way through an eight-instruction
    // idle-loop iteration. Instead, advance the verified EE idle loop only up
    // to the next exact 8:1 IOP boundary, retire that one IOP instruction, then
    // re-check every shared-device condition before skipping further. This
    // preserves ordering while removing the EE NOP/branch interpreter tax
    // during active IOP bootstrap work.
    const u32 initial_iop_pc = iop_.state().pc;
    if (!iop_.halted() &&
        initial_iop_pc != 0x0000AE94u &&
        initial_iop_pc != 0x0000AE98u) {
        const u64 co_limit = std::min<u64>(budget, 65536u);
        u64 co_retired = 0u;

        while (co_retired < co_limit && !ee_.halted()) {
            const u32 pc = ee_.state().pc;
            if (pc < 0x00081FC0u || pc > 0x00081FDCu ||
                ((pc - 0x00081FC0u) & 3u) != 0u) {
                break;
            }

            const u16 co_active_dma =
                hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
            const u16 co_ready_sif =
                (co_active_dma & sif_channels) &
                iop_bus_.sif_dma_ready_mask();
            const bool co_sif0_needs_service =
                (co_ready_sif & (1u << 5)) != 0u &&
                !sif_dma_.sif0_completion_pending();
            const bool co_sif1_needs_service =
                (co_ready_sif & (1u << 6)) != 0u &&
                !sif_dma_.sif1_completion_pending();
            if ((co_active_dma & ~sif_channels) != 0u ||
                co_sif0_needs_service || co_sif1_needs_service ||
                !scheduler_.empty() ||
                sif_dma_.ee_completion_pending() ||
                vu0_.running() || vu1_.running() ||
                gs_.irq_pending() ||
                bus_.intc_pending() || bus_.dmac_pending()) {
                break;
            }

            const auto& co_cpu = ee_.state();
            const u32 co_status = co_cpu.cop0[12];
            const u32 co_cause = co_cpu.cop0[13] & ~0x00000C00u;
            if ((co_cause & co_status & 0x0000FF00u) != 0u &&
                (co_status & 0x00010001u) == 0x00010001u &&
                (co_status & 0x6u) == 0u) {
                break;
            }

            u64 room = co_limit - co_retired;
            const u64 until_iop =
                ee_iop_phase_ == 0u ? 8u : 8u - ee_iop_phase_;
            room = std::min(room, until_iop);

            const u64 video_room =
                video_timing_.cycles_to_transition();
            if (video_room <= 1u) break;
            room = std::min<u64>(room, video_room - 1u);

            const u32 ee_completion =
                sif_dma_.ee_completion_cycles();
            if (ee_completion != 0u) {
                room = std::min<u64>(room, ee_completion);
            }

            const u32 compare_distance =
                co_cpu.cop0[11] - co_cpu.cop0[9];
            if (compare_distance != 0u) {
                if (compare_distance <= 1u) break;
                room = std::min<u64>(
                    room,
                    static_cast<u64>(compare_distance - 1u));
            }

            if (room == 0u ||
                !ee_.skip_bios_idle_instructions(
                    static_cast<u32>(room))) {
                break;
            }

            sif_dma_.tick_ee_cycles(bus_, room);
            scheduler_.run_until(
                scheduler_.now() + room, {});
            video_timing_.tick(room, hw_, iop_intc_);
            advance_iop_for_ee_cycles(room, error);
            if (gs_.irq_pending()) hw_.raise_intc(0);

            co_retired += room;
            if (!error.empty()) break;
        }

        if (co_retired != 0u) {
            skipped_bios_idle_iterations_ +=
                co_retired / kIdleInstructions;
            if (idle_offphase) {
                ++skipped_bios_idle_offphase_batches_;
            }
            return co_retired;
        }
    }

    // The legacy one-iteration path below is retained for the canonical idle
    // entry. It only runs when the co-paced path above could not make progress.
    const u32 iop_pc = iop_.state().pc;
    if (!iop_.halted()) {
        if (iop_bus_.interrupt_pending()) {
            ++idle_skip_reasons_[0]; return 0;
        }
        u32 instruction = 0;
        if (!iop_bus_.read32(iop_pc, instruction)) {
            ++idle_skip_reasons_[0]; return 0;
        }
        const u32 opcode = instruction >> 26;
        if (opcode >= 0x28u) {
            ++idle_skip_reasons_[0]; return 0;
        }
    }
    if (!scheduler_.empty() ||
        video_timing_.cycles_to_transition() <= kIdleInstructions) {
        ++idle_skip_reasons_[1]; return 0;
    }
    if (sif_dma_.ee_completion_pending() || vu0_.running() ||
        vu1_.running() || gs_.irq_pending()) {
        ++idle_skip_reasons_[3]; return 0;
    }
    if (bus_.intc_pending() || bus_.dmac_pending()) {
        ++idle_skip_reasons_[4]; return 0;
    }
    const auto& cpu = ee_.state();
    const u32 status = cpu.cop0[12];
    const u32 cause = cpu.cop0[13] & ~0x00000C00u;
    if ((cause & status & 0x0000FF00u) != 0 &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0) {
        ++idle_skip_reasons_[5]; return 0;
    }
    if (!iop_.halted() &&
        (iop_pc == 0x0000AE94u || iop_pc == 0x0000AE98u) &&
        !sif_dma_.iop_completion_pending()) {
        u64 safe_cycles = budget;
        safe_cycles = std::min<u64>(
            safe_cycles,
            video_timing_.cycles_to_transition() - 1u);

        const u32 ee_completion =
            sif_dma_.ee_completion_cycles();
        if (ee_completion != 0u) {
            safe_cycles = std::min<u64>(
                safe_cycles, ee_completion);
        }

        const u32 iop_completion =
            sif_dma_.iop_completion_steps();
        if (iop_completion != 0u) {
            const u64 iop_room =
                static_cast<u64>(iop_completion) * 8u -
                ee_iop_phase_;
            safe_cycles = std::min<u64>(
                safe_cycles, iop_room);
        }

        u64 iterations = std::min<u64>(
            65536u, safe_cycles / kIdleInstructions);
        const u32 distance = cpu.cop0[11] - cpu.cop0[9];
        if (distance != 0u) {
            iterations = std::min<u64>(iterations,
                (static_cast<u64>(distance) - 1u) /
                    kIdleInstructions);
        }
        while (iterations > 1u &&
               !iop_bus_.can_tick_event_free(iterations)) {
            iterations >>= 1u;
        }
        if (iterations > 1u &&
            ee_.skip_bios_idle_iterations(
                static_cast<u32>(iterations))) {
            const u64 cycles = iterations * kIdleInstructions;
            sif_dma_.tick_ee_cycles(bus_, cycles);
            scheduler_.run_until(scheduler_.now() + cycles, {});
            video_timing_.tick(cycles, hw_, iop_intc_);
            advance_iop_for_ee_cycles(cycles, error);
            skipped_bios_idle_iterations_ += iterations;
            if (idle_offphase) ++skipped_bios_idle_offphase_batches_;
            return cycles;
        }
    }
    // Starting from an off-phase PC is valuable only when we can retire a
    // real bulk interval. A single eight-cycle specialized skip pays more
    // scheduler/SIF/IOP bookkeeping than the cached interpreter it replaces.
    // Keep the old one-iteration fallback only for the canonical 0x81FC0
    // entry, where it was already a net win before phase-aware skipping.
    if (idle_offphase) {
        return 0;
    }

    const u32 ee_completion =
        sif_dma_.ee_completion_cycles();
    if (ee_completion != 0u &&
        ee_completion < kIdleInstructions) {
        ++idle_skip_reasons_[2]; return 0;
    }
    const u32 iop_completion =
        sif_dma_.iop_completion_steps();
    if (iop_completion != 0u) {
        const u64 iop_room =
            static_cast<u64>(iop_completion) * 8u -
            ee_iop_phase_;
        if (iop_room < kIdleInstructions) {
            ++idle_skip_reasons_[2]; return 0;
        }
    }
    if (!ee_.skip_bios_idle_iteration()) {
        ++idle_skip_reasons_[6]; return 0;
    }
    ++skipped_bios_idle_iterations_;
    if (idle_offphase) ++skipped_bios_idle_offphase_batches_;

    sif_dma_.tick_ee_cycles(bus_, kIdleInstructions);
    scheduler_.run_until(scheduler_.now() + kIdleInstructions, {});
    video_timing_.tick(kIdleInstructions, hw_, iop_intc_);
    for (u32 i = 0; i < kIdleInstructions; ++i) {
        if (!advance_iop_for_ee_step(error)) break;
    }
    return kIdleInstructions;
}

u64 Ps2System::try_skip_bios_zero_loop(u64 budget, std::string& error) {
    constexpr u64 kInstructionsPerIteration = 7u;
    if (budget < kInstructionsPerIteration || !scheduler_.empty() ||
        video_timing_.cycles_to_transition() <= kInstructionsPerIteration ||
        hw_.timer_irq_possible() || hw_.dmac_running_mask() != 0u ||
        sif_dma_.ee_completion_pending() || vu0_.running() ||
        vu1_.running() || gs_.irq_pending() ||
        bus_.intc_pending() || bus_.dmac_pending()) return 0;

    const auto& cpu = ee_.state();
    const u32 status = cpu.cop0[12];
    const u32 cause = cpu.cop0[13] & ~0x00000C00u;
    if ((cause & status & 0x0000FF00u) != 0 &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0) return 0;
    const u64 start = cpu.gpr[16].lo;
    const u64 end = cpu.gpr[4].lo;
    if (start >= end || end > EeRam::kSize) return 0;

    u64 iterations = std::min<u64>(4096u, budget / kInstructionsPerIteration);
    iterations = std::min<u64>(iterations, (end - start - 1u) / 16u);
    iterations = std::min<u64>(iterations,
        (video_timing_.cycles_to_transition() - 1u) /
            kInstructionsPerIteration);
    const u32 distance = cpu.cop0[11] - cpu.cop0[9];
    if (distance != 0u) {
        iterations = std::min<u64>(iterations,
            (static_cast<u64>(distance) - 1u) / kInstructionsPerIteration);
    }
    if (iterations == 0u ||
        !ee_.skip_bios_zero_loop(static_cast<u32>(iterations))) return 0;

    const u64 cycles = iterations * kInstructionsPerIteration;
    scheduler_.run_until(scheduler_.now() + cycles, {});
    video_timing_.tick(cycles, hw_, iop_intc_);
    advance_iop_for_ee_cycles(cycles, error);
    skipped_bios_zero_iterations_ += iterations;
    return cycles;
}

u64 Ps2System::try_skip_bios_nibble_loop(u64 budget, std::string& error) {
    constexpr u64 kInstructionsPerIteration = 9u;
    if (budget < kInstructionsPerIteration || !scheduler_.empty() ||
        video_timing_.cycles_to_transition() <= kInstructionsPerIteration ||
        hw_.timer_irq_possible() || sif_dma_.ee_completion_pending() ||
        vu0_.running() || vu1_.running() || gs_.irq_pending() ||
        bus_.intc_pending() || bus_.dmac_pending()) return 0;
    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
    const u16 sif_channels = (1u << 5) | (1u << 6);
    if ((active_dma & ~sif_channels) != 0u ||
        ((active_dma & sif_channels) &
            iop_bus_.sif_dma_ready_mask()) != 0u) return 0;

    const auto& cpu = ee_.state();
    const u32 status = cpu.cop0[12];
    const u32 cause = cpu.cop0[13] & ~0x00000C00u;
    if ((cause & status & 0x0000FF00u) != 0 &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0) return 0;
    const u64 start = cpu.gpr[5].lo;
    if (start >= EeRam::kSize || cpu.gpr[6].lo == 0u) return 0;

    u64 iterations = std::min<u64>(4096u, budget / kInstructionsPerIteration);
    iterations = std::min<u64>(iterations, EeRam::kSize - start);
    iterations = std::min<u64>(iterations, cpu.gpr[6].lo);
    iterations = std::min<u64>(iterations,
        (video_timing_.cycles_to_transition() - 1u) /
            kInstructionsPerIteration);
    const u32 distance = cpu.cop0[11] - cpu.cop0[9];
    if (distance != 0u) {
        iterations = std::min<u64>(iterations,
            (static_cast<u64>(distance) - 1u) / kInstructionsPerIteration);
    }
    if (iterations == 0u ||
        !ee_.skip_bios_nibble_loop(static_cast<u32>(iterations))) return 0;

    const u64 cycles = iterations * kInstructionsPerIteration;
    scheduler_.run_until(scheduler_.now() + cycles, {});
    video_timing_.tick(cycles, hw_, iop_intc_);
    advance_iop_for_ee_cycles(cycles, error);
    skipped_bios_nibble_iterations_ += iterations;
    return cycles;
}

u64 Ps2System::try_skip_bios_count_wait(u64 budget, std::string& error) {
    constexpr u64 kInstructionsPerIteration = 7u;
    if (budget < kInstructionsPerIteration || !scheduler_.empty() ||
        video_timing_.cycles_to_transition() <= kInstructionsPerIteration ||
        hw_.timer_irq_possible() || hw_.dmac_running_mask() != 0u ||
        sif_dma_.ee_completion_pending() || vu0_.running() ||
        vu1_.running() || gs_.irq_pending() ||
        bus_.intc_pending() || bus_.dmac_pending()) return 0;

    const auto& cpu = ee_.state();
    const u32 status = cpu.cop0[12];
    const u32 cause = cpu.cop0[13] & ~0x00000C00u;
    if ((cause & status & 0x0000FF00u) != 0 &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0) return 0;

    u64 max_iterations = std::min<u64>(4096u,
        budget / kInstructionsPerIteration);
    max_iterations = std::min<u64>(max_iterations,
        (video_timing_.cycles_to_transition() - 1u) /
            kInstructionsPerIteration);
    const u32 distance = cpu.cop0[11] - cpu.cop0[9];
    if (distance != 0u) {
        max_iterations = std::min<u64>(max_iterations,
            (static_cast<u64>(distance) - 1u) / kInstructionsPerIteration);
    }
    const u32 iterations = ee_.skip_bios_count_wait(
        static_cast<u32>(max_iterations));
    if (iterations == 0u) return 0;

    const u64 cycles = iterations * kInstructionsPerIteration;
    scheduler_.run_until(scheduler_.now() + cycles, {});
    video_timing_.tick(cycles, hw_, iop_intc_);
    advance_iop_for_ee_cycles(cycles, error);
    skipped_bios_count_wait_iterations_ += iterations;
    return cycles;
}

u64 Ps2System::try_skip_bios_countdown_wait(
    u64 budget, std::string& error) {
    constexpr u64 kInstructionsPerIteration = 7u;
    if (budget < kInstructionsPerIteration || !scheduler_.empty() ||
        video_timing_.cycles_to_transition() <= kInstructionsPerIteration ||
        hw_.timer_irq_possible() || sif_dma_.ee_completion_pending() ||
        vu0_.running() || vu1_.running() || gs_.irq_pending() ||
        bus_.intc_pending() || bus_.dmac_pending()) return 0;
    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
    const u16 sif_channels = (1u << 5) | (1u << 6);
    if ((active_dma & ~sif_channels) != 0u ||
        ((active_dma & sif_channels) &
            iop_bus_.sif_dma_ready_mask()) != 0u) return 0;

    const auto& cpu = ee_.state();
    const u32 status = cpu.cop0[12];
    const u32 cause = cpu.cop0[13] & ~0x00000C00u;
    if ((cause & status & 0x0000FF00u) != 0 &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0) return 0;

    u64 max_iterations = std::min<u64>(4096u,
        budget / kInstructionsPerIteration);
    max_iterations = std::min<u64>(max_iterations,
        (video_timing_.cycles_to_transition() - 1u) /
            kInstructionsPerIteration);
    const u32 distance = cpu.cop0[11] - cpu.cop0[9];
    if (distance != 0u) {
        max_iterations = std::min<u64>(max_iterations,
            (static_cast<u64>(distance) - 1u) / kInstructionsPerIteration);
    }
    const u32 iterations = ee_.skip_bios_countdown_wait(
        static_cast<u32>(max_iterations));
    if (iterations == 0u) return 0;

    const u64 cycles = iterations * kInstructionsPerIteration;
    scheduler_.run_until(scheduler_.now() + cycles, {});
    video_timing_.tick(cycles, hw_, iop_intc_);
    advance_iop_for_ee_cycles(cycles, error);
    skipped_bios_countdown_iterations_ += iterations;
    return cycles;
}

u64 Ps2System::try_skip_bios_copy_iterations(
    u64 budget, std::string& error) {
    constexpr u64 kInstructionsPerIteration = 7u;
    if (budget < kInstructionsPerIteration) return 0;
    if (!scheduler_.empty() ||
        video_timing_.cycles_to_transition() <= kInstructionsPerIteration ||
        hw_.timer_irq_possible() || sif_dma_.ee_completion_pending() ||
        vu0_.running() || vu1_.running() || gs_.irq_pending() ||
        bus_.intc_pending() || bus_.dmac_pending() ||
        iop_bus_.interrupt_pending()) return 0;
    const u32 iop_pc = iop_.state().pc;
    if (iop_pc != 0x0000AE94u && iop_pc != 0x0000AE98u) return 0;
    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
    const u16 sif_channels = (1u << 5) | (1u << 6);
    if ((active_dma & ~sif_channels) != 0u ||
        ((active_dma & sif_channels) &
            iop_bus_.sif_dma_ready_mask()) != 0u) return 0;

    const auto& cpu = ee_.state();
    const u32 status = cpu.cop0[12];
    const u32 cause = cpu.cop0[13] & ~0x00000C00u;
    if ((cause & status & 0x0000FF00u) != 0 &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0) return 0;
    u64 iterations = std::min<u64>(4096u,
        budget / kInstructionsPerIteration);
    iterations = std::min<u64>(iterations, cpu.gpr[4].lo);
    iterations = std::min<u64>(iterations,
        (video_timing_.cycles_to_transition() - 1u) /
            kInstructionsPerIteration);
    const u32 distance = cpu.cop0[11] - cpu.cop0[9];
    if (distance != 0u) {
        iterations = std::min<u64>(iterations,
            (static_cast<u64>(distance) - 1u) /
                kInstructionsPerIteration);
    }
    if (iterations == 0u) return 0;
    if (sif_dma_.iop_completion_pending()) iterations = 1u;
    while (iterations > 1u) {
        const u64 iop_steps =
            (ee_iop_phase_ + iterations * kInstructionsPerIteration) / 8u;
        if (iop_bus_.can_tick_event_free(iop_steps)) break;
        iterations >>= 1u;
    }
    if (!ee_.skip_bios_copy_iterations(static_cast<u32>(iterations)))
        return 0;

    const u64 cycles = iterations * kInstructionsPerIteration;
    scheduler_.run_until(scheduler_.now() + cycles, {});
    video_timing_.tick(cycles, hw_, iop_intc_);
    advance_iop_for_ee_cycles(cycles, error);
    skipped_bios_copy_iterations_ += iterations;
    return cycles;
}

u64 Ps2System::try_skip_bios_mmio_poll_iterations(
    u64 budget, std::string& error) {
    constexpr u64 kCyclesPerIteration = 7u;
    if (!scheduler_.empty() ||
        video_timing_.cycles_to_transition() <= kCyclesPerIteration ||
        hw_.timer_irq_possible() ||
        sif_dma_.ee_completion_pending() || vu0_.running() ||
        vu1_.running() || gs_.irq_pending() ||
        bus_.intc_pending() || bus_.dmac_pending()) return 0;
    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
    const u16 sif_channels = (1u << 5) | (1u << 6);
    if ((active_dma & ~sif_channels) != 0u ||
        ((active_dma & sif_channels) &
            iop_bus_.sif_dma_ready_mask()) != 0u) return 0;
    const auto& cpu = ee_.state();
    const u32 status = cpu.cop0[12];
    const u32 cause = cpu.cop0[13] & ~0x00000C00u;
    if ((cause & status & 0x0000FF00u) != 0 &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0) return 0;
    u64 iterations = std::min<u64>(4096u, budget / kCyclesPerIteration);
    iterations = std::min<u64>(iterations,
        (video_timing_.cycles_to_transition() - 1u) /
            kCyclesPerIteration);
    const u32 distance = cpu.cop0[11] - cpu.cop0[9];
    if (distance != 0u) {
        iterations = std::min<u64>(iterations,
            (static_cast<u64>(distance) - 1u) / kCyclesPerIteration);
    }
    if (iterations == 0u) return 0;
    if (sif_dma_.iop_completion_pending() ||
        iop_bus_.interrupt_pending() ||
        (iop_.state().pc != 0x0000AE94u &&
         iop_.state().pc != 0x0000AE98u)) iterations = 1u;
    while (iterations > 1u) {
        const u64 iop_steps =
            (ee_iop_phase_ + iterations * kCyclesPerIteration) / 8u;
        if (iop_bus_.can_tick_event_free(iop_steps)) break;
        iterations >>= 1u;
    }
    iterations = ee_.skip_bios_mmio_poll_iterations(
        static_cast<u32>(iterations));
    if (iterations == 0u) return 0;
    const u64 kCycles = iterations * kCyclesPerIteration;
    scheduler_.run_until(scheduler_.now() + kCycles, {});
    video_timing_.tick(kCycles, hw_, iop_intc_);
    advance_iop_for_ee_cycles(kCycles, error);
    skipped_bios_mmio_poll_iterations_ += iterations;
    return kCycles;
}

u64 Ps2System::try_skip_bios_literal_iterations(
    u64 budget, std::string& error) {
    constexpr u64 kCyclesPerIteration = 22u;
    if (!scheduler_.empty() ||
        video_timing_.cycles_to_transition() <= kCyclesPerIteration ||
        hw_.timer_irq_possible() || sif_dma_.ee_completion_pending() ||
        sif_dma_.iop_completion_pending() ||
        vu0_.running() || vu1_.running() || gs_.irq_pending() ||
        bus_.intc_pending() || bus_.dmac_pending() ||
        iop_bus_.interrupt_pending()) return 0;
    const u32 iop_pc = iop_.state().pc;
    if (iop_pc != 0x0000AE94u && iop_pc != 0x0000AE98u) return 0;
    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
    const u16 sif_channels = (1u << 5) | (1u << 6);
    if ((active_dma & ~sif_channels) != 0u ||
        ((active_dma & sif_channels) &
            iop_bus_.sif_dma_ready_mask()) != 0u) return 0;
    const auto& cpu = ee_.state();
    const u32 status = cpu.cop0[12];
    const u32 cause = cpu.cop0[13] & ~0x00000C00u;
    if ((cause & status & 0x0000FF00u) != 0 &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0) return 0;
    u64 iterations = std::min<u64>(4096u, budget / kCyclesPerIteration);
    iterations = std::min<u64>(iterations,
        (video_timing_.cycles_to_transition() - 1u) /
            kCyclesPerIteration);
    const u32 distance = cpu.cop0[11] - cpu.cop0[9];
    if (distance != 0u) {
        iterations = std::min<u64>(iterations,
            (static_cast<u64>(distance) - 1u) / kCyclesPerIteration);
    }
    while (iterations != 0u) {
        const u64 iop_steps =
            (ee_iop_phase_ + iterations * kCyclesPerIteration) / 8u;
        if (iop_bus_.can_tick_event_free(iop_steps)) break;
        iterations >>= 1u;
    }
    if (iterations == 0u) return 0;
    iterations = ee_.skip_bios_literal_iterations(
        static_cast<u32>(iterations));
    if (iterations == 0u) return 0;
    const u64 kCycles = iterations * kCyclesPerIteration;
    scheduler_.run_until(scheduler_.now() + kCycles, {});
    video_timing_.tick(kCycles, hw_, iop_intc_);
    advance_iop_for_ee_cycles(kCycles, error);
    skipped_bios_literal_iterations_ += iterations;
    return kCycles;
}

u64 Ps2System::try_skip_hot_sif_getreg(
    u64 budget,
    std::string& error) {
    constexpr u64 kCycles = 106u;
    if (ee_.state().pc == 0x0024DE74u) {
        const u32 arg =
            static_cast<u32>(ee_.state().gpr[4].lo);
        if (arg < fast_sif_getreg_args_.size()) {
            ++fast_sif_getreg_args_[arg];
        }
    }
    if (budget < kCycles ||
        ee_.state().pc != 0x0024DE74u) {
        ++fast_sif_getreg_rejects_[0];
        return 0u;
    }
    if (!scheduler_.empty()) {
        ++fast_sif_getreg_rejects_[1];
        return 0u;
    }
    if (video_timing_.cycles_to_transition() <= kCycles) {
        ++fast_sif_getreg_rejects_[2];
        return 0u;
    }
    if (sif_dma_.ee_completion_pending() ||
        sif_dma_.iop_completion_pending()) {
        ++fast_sif_getreg_rejects_[3];
        return 0u;
    }
    if (vu0_.running() || vu1_.running() ||
        gs_.irq_pending() ||
        bus_.intc_pending() || bus_.dmac_pending() ||
        iop_bus_.interrupt_pending()) {
        ++fast_sif_getreg_rejects_[4];
        return 0u;
    }

    const auto& cpu = ee_.state();
    const u32 status = cpu.cop0[12];
    const u32 cause = cpu.cop0[13] & ~0x00000C00u;
    if ((cause & status & 0x0000FF00u) != 0u &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0u) {
        ++fast_sif_getreg_rejects_[5];
        return 0u;
    }

    const u64 timer_room = hw_.cycles_to_timer_irq();
    const u32 compare_distance =
        cpu.cop0[11] - cpu.cop0[9];
    if ((timer_room != ~u64{0} && timer_room <= kCycles) ||
        (compare_distance != 0u &&
         compare_distance <= kCycles)) {
        ++fast_sif_getreg_rejects_[6];
        return 0u;
    }

    const bool iop_halted = iop_.halted();
    const bool iop_idle =
        !iop_halted && iop_.in_osdsys_idle_loop();
    const bool iop_active =
        !iop_halted && !iop_idle;

    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
    const u16 sif_channels = (1u << 5) | (1u << 6);

    // The active-IOP fast path is deliberately narrower than the idle path:
    // no EE DMA may be armed while we let the IOP run to the exact SMFLAG
    // sample point. Profiling shows this covers virtually every hot call.
    if (iop_active && active_dma != 0u) {
        ++fast_sif_getreg_rejects_[7];
        return 0u;
    }

    if (!iop_active &&
        ((active_dma & ~sif_channels) != 0u ||
         ((active_dma & sif_channels) &
          iop_bus_.sif_dma_ready_mask()) != 0u)) {
        ++fast_sif_getreg_rejects_[8];
        return 0u;
    }

    if (iop_idle) {
        const u64 iop_steps =
            (static_cast<u64>(ee_iop_phase_) + kCycles) / 8u;
        if (iop_steps != 0u &&
            !iop_bus_.can_tick_event_free(iop_steps)) {
            ++fast_sif_getreg_rejects_[9];
            return 0u;
        }
    }

    // Validate all EE architectural/code guards before advancing active IOP
    // state. Once the IOP has moved to the sample point we must be able to
    // commit the collapsed EE round-trip without falling back.
    if (!ee_.can_skip_hot_sif_getreg()) {
        ++fast_sif_getreg_rejects_[11];
        return 0u;
    }

    constexpr u64 kSampleOffset = 61u;
    constexpr u64 kTailCycles = kCycles - kSampleOffset;

    if (iop_active) {
        // The real kernel reaches the SMFLAG load after 61 retired EE
        // instructions. Run only the IOP half of time to that exact instant;
        // video/timers are proven event-free for the entire 106-cycle window
        // by the guards above, so their bulk update can remain at commit.
        advance_iop_for_ee_cycles(kSampleOffset, error);
        if (!error.empty()) return kSampleOffset;
    }

    u32 smflag = 0u;
    if (!bus_.read32(0x1000F230u, smflag)) {
        ++fast_sif_getreg_rejects_[10];
        return 0u;
    }

    // For the active path the value was sampled at the exact guest load
    // instant, so even the BOOTEND transition value is safe to return. The
    // idle path retains the old conservative transition fallback.
    if (!iop_active &&
        (smflag & 0x00040000u) != 0u) {
        ++fast_sif_getreg_rejects_[10];
        return 0u;
    }

    if (!ee_.skip_hot_sif_getreg(smflag)) {
        ++fast_sif_getreg_rejects_[11];
        return 0u;
    }

    scheduler_.run_until(
        scheduler_.now() + kCycles, {});
    video_timing_.tick(kCycles, hw_, iop_intc_);

    if (iop_active) {
        advance_iop_for_ee_cycles(kTailCycles, error);
        ++fast_sif_getreg_active_iop_calls_;
    } else {
        advance_iop_for_ee_cycles(kCycles, error);
    }
    if (!error.empty()) return kCycles;

    if (gs_.irq_pending()) hw_.raise_intc(0);
    ++fast_sif_getreg_calls_;
    return kCycles;
}

u64 Ps2System::try_run_quiet_ee_batch(
    u64 budget, std::string& error) {
    if (budget < 2u ||
        sif_dma_.ee_completion_pending() ||
        vu0_.running() || vu1_.running() || gs_.irq_pending()) {
        return 0;
    }

    // An armed but unmatched SIF channel is intentionally left running by
    // the BIOS. A ready channel only requires immediate service when no
    // transfer is already in its completion countdown. During that latency
    // the service routine is a no-op, so quiet EE execution may continue up
    // to (but not past) the exact completion edge.
    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
    const u16 sif_channels = (1u << 5) | (1u << 6);
    const u16 ready_sif =
        (active_dma & sif_channels) &
        iop_bus_.sif_dma_ready_mask();
    const bool sif0_needs_service =
        (ready_sif & (1u << 5)) != 0u &&
        !sif_dma_.sif0_completion_pending();
    const bool sif1_needs_service =
        (ready_sif & (1u << 6)) != 0u &&
        !sif_dma_.sif1_completion_pending();
    if ((active_dma & ~sif_channels) != 0u ||
        sif0_needs_service || sif1_needs_service) {
        return 0;
    }

    auto& cpu = ee_.state();
    if (bus_.intc_pending()) cpu.cop0[13] |= 0x00000400u;
    else cpu.cop0[13] &= ~0x00000400u;
    if (bus_.dmac_pending()) cpu.cop0[13] |= 0x00000800u;
    else cpu.cop0[13] &= ~0x00000800u;

    const u32 status = cpu.cop0[12];
    const u32 cause = cpu.cop0[13];
    if ((cause & status & 0x0000FF00u) != 0u &&
        (status & 0x00010001u) == 0x00010001u &&
        (status & 0x6u) == 0u) {
        return 0;
    }

    const bool iop_halted = iop_.halted();
    const bool iop_idle =
        !iop_halted && iop_.in_osdsys_idle_loop();

    // SIF completion countdowns are explicit event boundaries. EE-side
    // completion is bulk-ticked after retirement; IOP-side completion is
    // still ticked on the exact due IOP instruction by
    // advance_iop_for_ee_cycles().
    const u64 video_room = video_timing_.cycles_to_transition();
    if (video_room <= 1u) return 0;

    u64 maximum = std::min<u64>(budget, kQuietEeBatchLimit);
    maximum = std::min<u64>(maximum, video_room - 1u);

    // A queued scheduler event is a deadline, not a reason to abandon the
    // quiet/native path completely. Retire up to the exact event edge, then
    // return to the system layer before the next EE instruction can observe
    // post-event state.
    if (const auto next_event = scheduler_.next_event_time()) {
        if (*next_event <= scheduler_.now()) return 0;
        maximum = std::min<u64>(
            maximum, *next_event - scheduler_.now());
    }

    const u32 sif_ee_completion =
        sif_dma_.ee_completion_cycles();
    if (sif_ee_completion != 0u) {
        maximum = std::min<u64>(
            maximum, sif_ee_completion);
    }

    const u32 sif_iop_completion =
        sif_dma_.iop_completion_steps();
    if (sif_iop_completion != 0u) {
        const u64 iop_room =
            static_cast<u64>(sif_iop_completion) * 8u -
            ee_iop_phase_;
        maximum = std::min<u64>(maximum, iop_room);
    }

    // Active IOP execution no longer forces an EE dispatcher round-trip
    // every eight cycles. Allow a bounded EE lead, then repay the accumulated
    // 8:1 IOP cycle debt in advance_iop_for_ee_cycles() after this batch.
    //
    // Pending IOP interrupts remain tightly interleaved so an already-visible
    // interrupt is not delayed by the lead window. Likewise, if an IOP-side
    // timer/SPU/DMA event can occur inside the proposed catch-up interval,
    // shrink the window until the IOP bus reports the whole interval event-
    // free. SIF completion has already installed its own exact deadline above.
    if (!iop_halted && !iop_idle) {
        maximum = std::min<u64>(
            maximum, kActiveIopEeLeadLimit);

        if (iop_bus_.interrupt_pending()) {
            const u64 until_iop_step =
                ee_iop_phase_ == 0u ? 8u : 8u - ee_iop_phase_;
            maximum = std::min<u64>(maximum, until_iop_step);
        } else {
            while (maximum > 1u) {
                const u64 iop_steps =
                    (static_cast<u64>(ee_iop_phase_) + maximum) / 8u;
                if (iop_steps == 0u ||
                    iop_bus_.can_tick_event_free(iop_steps)) {
                    break;
                }
                maximum >>= 1u;
            }
        }
    }

    // Device time can be coalesced as long as the block ends no later than
    // the first enabled EE timer IRQ edge. If an edge lands on the final
    // retired instruction, bus_.tick() raises it before the next EE
    // instruction is allowed to execute.
    const u64 timer_irq_room = hw_.cycles_to_timer_irq();
    if (timer_irq_room != ~u64{0}) {
        maximum = std::min<u64>(maximum, timer_irq_room);
    }
    const bool defer_ee_tick = true;

    // Stop on the instruction that reaches COP0 Compare. It can set IP7 at
    // retirement; the next outer iteration then takes the exact IRQ.
    const u32 compare_distance = cpu.cop0[11] - cpu.cop0[9];
    if (compare_distance != 0u) {
        maximum = std::min<u64>(maximum, compare_distance);
    }

    // The IOP idle pair has no architectural side effects, but its timers,
    // SPU2 cadence and DMA IRQ countdown still matter. Only defer the pair
    // when the complete interval is event-free.
    if (iop_idle) {
        while (maximum > 1u) {
            const u64 iop_steps = (ee_iop_phase_ + maximum) / 8u;
            if (iop_steps == 0u ||
                iop_bus_.can_tick_event_free(iop_steps)) {
                break;
            }
            maximum >>= 1u;
        }
    }
    if (maximum < 2u) return 0;

    const auto ee_profile_begin = std::chrono::steady_clock::now();
    u64 retired = 0;
    while (retired < maximum && !ee_.halted()) {
        // SifGetReg(4) has a system-level exact-timing accelerator that must
        // advance the active IOP to the real SMFLAG sample point. If a quiet
        // trace reaches the syscall wrapper mid-batch, yield here so run_ee()
        // can apply that accelerator instead of interpreting the 106-cycle
        // kernel round-trip inside this otherwise side-effect-free batch.
        if (retired != 0u &&
            ee_.state().pc == 0x0024DE74u) {
            break;
        }

        bool progressed = false;

        // Same helper, first sample. Retire only the MMIO load here; the
        // following NOP/debounce interval remains subject to the normal quiet
        // batch event/IOP boundary so device time cannot move across the next
        // observation.
        if (ee_.state().pc == 0x80005F08u) {
            static constexpr std::array<u32, 3> kSifStableEntry = {
                0x3C02B000u, 0x3442F230u, 0x8C440000u};
            const bool code_matches =
                bus_.matches_code(0x80005F00u, kSifStableEntry);
            const u32 physical =
                EeBus::to_physical(
                    static_cast<u32>(ee_.state().gpr[2].lo));
            if (code_matches &&
                physical == 0x1000F230u &&
                ee_.step_quiet_unchecked_predecoded(
                    kSifStableEntry[2], error)) {
                ++retired;
                ++quiet_block_instructions_;
                ++sif_poll_fast_samples_;
                progressed = true;
                if (!error.empty()) break;
                continue;
            }
            if (!error.empty()) break;
        }

        // Retail OSDSYS SIF synchronization helper. At 0x80005F5C the
        // routine takes its second sample of SMFLG (0x1000F230) after the
        // preceding debounce/NOP interval. Handle that one MMIO read inside
        // the already event-bounded quiet batch. When it matches the prior
        // sample in a0, retire the taken BEQ + delay slot + JR + delay slot
        // as well, avoiding a full system-step round trip for the common
        // stable-return case without moving either MMIO sample in time.
        if (ee_.state().pc == 0x80005F5Cu) {
            static constexpr std::array<u32, 3> kSifStableHead = {
                0x8C420000u, 0x1082001Cu, 0x00000000u};
            static constexpr std::array<u32, 2> kSifStableReturn = {
                0x03E00008u, 0x0080102Du};
            const bool code_matches =
                bus_.matches_code(0x80005F5Cu, kSifStableHead) &&
                bus_.matches_code(0x80005FD4u, kSifStableReturn);
            const u32 physical =
                EeBus::to_physical(
                    static_cast<u32>(ee_.state().gpr[2].lo));
            if (code_matches &&
                physical == 0x1000F230u) {
                const u64 previous =
                    ee_.state().gpr[4].lo;
                if (ee_.step_quiet_unchecked_predecoded(
                        kSifStableHead[0], error)) {
                    ++retired;
                    ++quiet_block_instructions_;
                    ++sif_poll_fast_samples_;
                    progressed = true;
                    if (!error.empty()) break;

                    const bool stable =
                        ee_.state().gpr[2].lo == previous;
                    if (stable && maximum - retired >= 4u) {
                        const std::array<u32, 4> kStableExit = {
                            kSifStableHead[1],
                            kSifStableHead[2],
                            kSifStableReturn[0],
                            kSifStableReturn[1]};
                        bool exit_ok = true;
                        for (const u32 instruction : kStableExit) {
                            if (!ee_.step_quiet_unchecked_predecoded(
                                    instruction, error)) {
                                exit_ok = false;
                                break;
                            }
                            ++retired;
                            ++quiet_block_instructions_;
                            if (!error.empty()) {
                                exit_ok = false;
                                break;
                            }
                        }
                        if (!exit_ok || !error.empty()) break;
                        ++sif_poll_stable_returns_;
                    }
                    continue;
                }
                if (!error.empty()) break;
            }
        }

        // Interpreter-only RAM trace: bypass cached basic-block lookup and
        // generation validation for the common case. The EE decoder fetches
        // each instruction fresh from RAM and follows supported branches and
        // delay slots directly until an unsupported/MMIO opcode or the exact
        // device/event boundary is reached.
        if (defer_ee_tick && !ee_.jit_enabled()) {
            const u32 pc = ee_.state().pc;
            const u32 physical = EeBus::to_physical(pc);
            if ((pc & 3u) == 0u &&
                physical <= ram_.size() - sizeof(u32)) {
                const u32 trace_retired = ee_.run_quiet_fast_prefix(
                    0u,
                    nullptr,
                    0u,
                    static_cast<u32>(maximum - retired),
                    nullptr,
                    ram_.data(),
                    ram_.data(),
                    ram_.page_generation_data());
                if (trace_retired != 0u) {
                    retired += trace_retired;
                    quiet_block_instructions_ += trace_retired;
                    fast_interpreter_instructions_ += trace_retired;
                    ++fast_interpreter_calls_;
                    progressed = true;
                    continue;
                }
            }
        }

        if (QuietEeBlock* block = quiet_ee_block(ee_.state().pc)) {
            const u32 block_pc = block->pc;

            if (defer_ee_tick && ee_.jit_enabled()) {
                const u32 native_retired = ee_.run_native_block(
                    block_pc,
                    block->page_generation,
                    block->words.data(),
                    block->count,
                    static_cast<u32>(maximum - retired),
                    ram_.data(),
                    ram_.page_generation_data(),
                    0x0024DE74u);
                if (native_retired != 0u) {
                    retired += native_retired;
                    quiet_block_instructions_ += native_retired;
                    progressed = true;
                    continue;
                }

                const u32 remaining =
                    static_cast<u32>(maximum - retired);
                if (block->count != 0u &&
                    remaining >= block->count &&
                    quiet_ee_instruction_value(
                        ee_.state(), block->words[0])) {
                    ++native_fallback_opcodes_[
                        block->words[0] >> 26];
                }
            }

            const u32 fast_prefix = 0u;

            for (u32 i = fast_prefix;
                 i < block->count && retired < maximum;
                 ++i) {
                if (ee_.state().pc != block_pc + i * 4u) break;

                const u32 instruction = block->words[i];
                if ((block->memory_mask & (1u << i)) != 0u &&
                    !quiet_ee_instruction_value(
                        ee_.state(), instruction)) {
                    break;
                }

                const bool ok = defer_ee_tick
                    ? ee_.step_quiet_unchecked_predecoded(
                        instruction, error)
                    : ee_.step_predecoded(instruction, error);
                if (!ok) break;

                ++retired;
                ++quiet_block_instructions_;
                progressed = true;
                if (!error.empty()) break;

                if ((block->store_mask & (1u << i)) != 0u) {
                    const u32 code_physical =
                        EeBus::to_physical(block_pc);
                    if (ram_.page_generation(code_physical) !=
                        block->page_generation) {
                        break;
                    }
                }
            }
            if (!error.empty()) break;
            if (progressed) continue;
        }

        // BIOS ROM and uncommon dynamically unsafe instructions retain the
        // original classified single-step fallback.
        u32 instruction = 0;
        if (!quiet_ee_instruction(ee_.state(), bus_, instruction)) {
            break;
        }
        const bool ok = defer_ee_tick
            ? ee_.step_quiet_unchecked_predecoded(instruction, error)
            : ee_.step_predecoded(instruction, error);
        if (!ok) break;
        ++retired;
        if (!error.empty()) break;
    }
    profile_ee_ns_ += elapsed_profile_ns(ee_profile_begin);
    if (retired == 0u) return 0;

    // step_quiet deliberately leaves EE hardware time untouched. Apply the
    // exact number of retired cycles once, before exposing the next MMIO or
    // event boundary to the guest. The timer-IRQ path used step() and has
    // already advanced this clock instruction by instruction.
    if (defer_ee_tick) bus_.tick(retired);
    sif_dma_.tick_ee_cycles(bus_, retired);
    scheduler_.run_until(scheduler_.now() + retired, {});

    const u64 fields_before = video_timing_.fields_started();
    video_timing_.tick(retired, hw_, iop_intc_);
    if (video_timing_.fields_started() != fields_before) {
        // maximum is capped before the next transition, so this is defensive.
        gs_.raise_vsync();
        const u64 field = video_timing_.fields_started();
        if (!gs_display_.has_visible_pixels() || (field & 1u) == 0u) {
            gs_display_.update(gs_, gs_core_.vram());
        }
    }

    advance_iop_for_ee_cycles(retired, error);
    if (gs_.irq_pending()) hw_.raise_intc(0);
    quiet_ee_batch_instructions_ += retired;
    if (!iop_halted && !iop_idle) {
        quiet_ee_active_iop_instructions_ += retired;
    }
    ++quiet_ee_batches_;
    return retired;
}

u64 Ps2System::try_run_quiet_ee_superbatch(
    u64 budget, std::string& error) {
    const u64 limit =
        std::min<u64>(budget, kQuietEeSuperbatchLimit);
    u64 retired = 0u;

    auto specialized_skip_entry = [](u32 pc) {
        switch (pc) {
        case 0x00081FC0u:
        case 0x8000E3C8u:
        case 0x0020A0E8u:
        case 0x9FC42930u:
        case 0x000826B0u:
        case 0x00252758u:
        case 0x00252DE8u:
        case 0x00200DE8u:
        case 0x00100BE0u:
        case 0x8000DAD0u:
        case 0x00082180u:
        case 0x00266118u:
        case 0x00200D70u:
            return true;
        default:
            return false;
        }
    };

    while (retired < limit && !ee_.halted()) {
        if (retired != 0u &&
            specialized_skip_entry(ee_.state().pc)) {
            break;
        }

        const u64 chunk = try_run_quiet_ee_batch(
            limit - retired, error);
        if (chunk == 0u) break;

        retired += chunk;
        if (!error.empty()) break;
    }

    if (retired != 0u) {
        ++quiet_superbatch_calls_;
        quiet_superbatch_instructions_ += retired;
    }
    return retired;
}

u64 Ps2System::run_ee(u64 instruction_budget,std::string& error){
    error.clear();
    if(!bios_started_){error="BIOS has not been started.";return 0;}
    const auto profile_begin = std::chrono::steady_clock::now();
    u64 executed=0;
    while(executed<instruction_budget){
        if (instruction_budget - executed >= 8u &&
            ee_.state().pc >= 0x00081FC0u &&
            ee_.state().pc <= 0x00081FDCu &&
            ((ee_.state().pc - 0x00081FC0u) & 3u) == 0u) {
            const u64 skipped = try_skip_bios_idle_iterations(
                instruction_budget - executed, error);
            if (skipped != 0u) {
                executed += skipped;
                if (!error.empty()) break;
                continue;
            }
        }
        if (ee_.state().pc == 0x8000E3C8u) {
            const u64 skipped = try_skip_bios_zero_loop(
                instruction_budget - executed, error);
            if (skipped != 0u) {
                executed += skipped;
                if (!error.empty()) break;
                continue;
            }
        }
        if (ee_.state().pc == 0x0020A0E8u) {
            const u64 skipped = try_skip_bios_nibble_loop(
                instruction_budget - executed, error);
            if (skipped != 0u) {
                executed += skipped;
                if (!error.empty()) break;
                continue;
            }
        }
        if (ee_.state().pc == 0x9FC42930u) {
            const u64 skipped = try_skip_bios_count_wait(
                instruction_budget - executed, error);
            if (skipped != 0u) {
                executed += skipped;
                if (!error.empty()) break;
                continue;
            }
        }
        if (ee_.state().pc == 0x000826B0u ||
            ee_.state().pc == 0x00252758u ||
            ee_.state().pc == 0x00252DE8u) {
            const u64 skipped = try_skip_bios_countdown_wait(
                instruction_budget - executed, error);
            if (skipped != 0u) {
                executed += skipped;
                if (!error.empty()) break;
                continue;
            }
        }
        if ((ee_.state().pc == 0x00200DE8u ||
             ee_.state().pc == 0x00100BE0u) &&
            instruction_budget - executed >= 7u) {
            const u64 skipped = try_skip_bios_copy_iterations(
                instruction_budget - executed, error);
            if (skipped != 0u) {
                executed += skipped;
                if (!error.empty()) break;
                continue;
            }
        }
        if ((ee_.state().pc == 0x8000DAD0u ||
             ee_.state().pc == 0x00082180u ||
             ee_.state().pc == 0x00266118u) &&
            instruction_budget - executed >= 7u) {
            const u64 skipped = try_skip_bios_mmio_poll_iterations(
                instruction_budget - executed, error);
            if (skipped != 0u) {
                executed += skipped;
                if (!error.empty()) break;
                continue;
            }
        }
        if (ee_.state().pc == 0x00200D70u &&
            instruction_budget - executed >= 22u) {
            const u64 skipped = try_skip_bios_literal_iterations(
                instruction_budget - executed, error);
            if (skipped != 0u) {
                executed += skipped;
                if (!error.empty()) break;
                continue;
            }
        }
        if (ee_.state().pc == 0x0024DE74u &&
            instruction_budget - executed >= 106u) {
            const u64 skipped = try_skip_hot_sif_getreg(
                instruction_budget - executed, error);
            if (skipped != 0u) {
                executed += skipped;
                if (!error.empty()) break;
                continue;
            }
        }

        const u64 quiet_batch = try_run_quiet_ee_batch(
            instruction_budget - executed, error);
        if (quiet_batch != 0u) {
            executed += quiet_batch;
            if (!error.empty()) break;
            continue;
        }

        const u64 before=ee_.state().instructions_executed;
        const bool profile_this_slow_step =
            ((profile_slow_path_samples_++ & 255u) == 0u);
        std::chrono::steady_clock::time_point slow_profile_begin{};
        if (profile_this_slow_step) {
            slow_profile_begin = std::chrono::steady_clock::now();
        }
        const bool step_ok = step_ee_core(error);
        if (profile_this_slow_step) {
            profile_slow_path_ns_ +=
                elapsed_profile_ns(slow_profile_begin) * 256u;
        }
        if(!step_ok){
            if(ee_.state().instructions_executed!=before)++executed;
            break;
        }
        ++executed;
    }
    profile_run_ns_ += elapsed_profile_ns(profile_begin);
    return executed;
}
void Ps2System::refresh_display(){gs_display_.update(gs_,gs_core_.vram());}
std::string Ps2System::halt_reason()const{if(ee_.halted())return "EE: "+ee_.halt_reason();return {};}
} // namespace ps2
