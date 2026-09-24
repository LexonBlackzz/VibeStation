#include "core/ps2_system.h"

#include <algorithm>

namespace ps2 {
namespace {
// Keep independent VU1 execution synchronized to EE instruction steps.
constexpr u64 kVu1InstructionsPerEeStep = 1;
constexpr u64 kQuietEeBatchLimit = 4096u;
constexpr u32 kEeMainRamSize = 32u * 1024u * 1024u;

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
    if (opcode == 0x01u ||
        (opcode >= 0x02u && opcode <= 0x07u) ||
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
    ee_.reset(entry_point); iop_.reset(Bios::kResetVector);
    bios_started_ = false;
    reset_instruction_ = 0;
    iop_reset_instruction_ = 0;
    ee_iop_phase_ = 0;
    skipped_bios_idle_iterations_ = 0;
    skipped_bios_zero_iterations_ = 0;
    skipped_bios_nibble_iterations_ = 0;
    skipped_bios_count_wait_iterations_ = 0;
    skipped_bios_countdown_iterations_ = 0;
    skipped_bios_copy_iterations_ = 0;
    skipped_bios_mmio_poll_iterations_ = 0;
    skipped_iop_idle_pairs_ = 0;
    skipped_bios_literal_iterations_ = 0;
    quiet_ee_batch_instructions_ = 0;
    quiet_ee_active_iop_instructions_ = 0;
    quiet_ee_batches_ = 0;
    quiet_block_instructions_ = 0;
    quiet_block_hits_ = 0;
    quiet_block_compiles_ = 0;
    std::fill(
        quiet_ee_blocks_.begin(),
        quiet_ee_blocks_.end(),
        QuietEeBlock{});
    idle_skip_reasons_.fill(0);
}
Ps2System::QuietEeBlock* Ps2System::quiet_ee_block(u32 pc) {
    if (pc >= 0xC0000000u) return nullptr;

    const u32 physical = EeBus::to_physical(pc);
    if (physical >= EeRam::kSize) return nullptr;

    const u32 page_offset = physical & (EeRam::kPageSize - 1u);
    ram_.track_code_page(physical);
    const u32 generation = ram_.page_generation(physical);
    const std::size_t index =
        ((static_cast<u64>(pc >> 2) * 2654435761ull) >> 19) &
        (quiet_ee_blocks_.size() - 1u);
    QuietEeBlock& block = quiet_ee_blocks_[index];

    if (block.count != 0u &&
        block.pc == pc &&
        block.page_generation == generation) {
        ++quiet_block_hits_;
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

    for (u32 i = 0; i < limit; ++i) {
        u32 instruction = 0;
        const u32 address = pc + i * 4u;
        if (!bus_.fetch32(address, instruction) ||
            !quiet_ee_static_instruction(instruction)) {
            break;
        }

        block.words[block.count++] = instruction;

        // A store can invalidate this block's code page. End immediately so
        // the next lookup observes the new generation before executing more
        // cached words. Control flow also forms a natural block boundary.
        if (quiet_ee_store(instruction) ||
            quiet_ee_control_flow(instruction)) {
            break;
        }
    }

    if (block.count == 0u) return nullptr;
    ++quiet_block_compiles_;
    return &block;
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

    std::string iop_error;
    if(!iop_.step(iop_error)){
        if(iop_.halted()){
            error.clear();
            return true;
        }
        error="IOP step failed: "+iop_error;
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
        std::string iop_error;
        if (!iop_.step(iop_error)) {
            if (!iop_.halted()) error = "IOP step failed: " + iop_error;
            break;
        }
        sif_dma_.tick_iop(iop_bus_);
        ++i;
    }
}
bool Ps2System::step_iop(std::string& error){error.clear();if(!bios_started_){error="BIOS has not been started.";return false;}if(iop_.halted()){error=iop_.halt_reason();return false;}if(!iop_.step(error))return false;sif_dma_.tick_iop(iop_bus_);return true;}
u64 Ps2System::try_skip_bios_idle_iterations(
    u64 budget, std::string& error) {
    constexpr u64 kIdleInstructions = 8u;
    if (ee_.state().pc != 0x00081FC0u) return 0;
    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
    const u16 sif_channels = (1u << 5) | (1u << 6);
    if ((active_dma & ~sif_channels) != 0u ||
        ((active_dma & sif_channels) &
            iop_bus_.sif_dma_ready_mask()) != 0u) {
        ++idle_skip_reasons_[2]; return 0;
    }
    if (hw_.timer_irq_possible()) {
        ++idle_skip_reasons_[7]; return 0;
    }
    // With arbitrary EE/IOP phase, exactly one IOP instruction can retire
    // during these eight EE cycles. A store could change EE-visible SIF/INTC
    // state midway through the interval, so keep those steps cycle-exact.
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
        u64 iterations = std::min<u64>(4096u, budget / kIdleInstructions);
        iterations = std::min<u64>(iterations,
            (video_timing_.cycles_to_transition() - 1u) /
                kIdleInstructions);
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
            scheduler_.run_until(scheduler_.now() + cycles, {});
            video_timing_.tick(cycles, hw_, iop_intc_);
            advance_iop_for_ee_cycles(cycles, error);
            skipped_bios_idle_iterations_ += iterations;
            return cycles;
        }
    }
    if (!ee_.skip_bios_idle_iteration()) {
        ++idle_skip_reasons_[6]; return 0;
    }
    ++skipped_bios_idle_iterations_;

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

u64 Ps2System::try_run_quiet_ee_batch(
    u64 budget, std::string& error) {
    if (budget < 2u || !scheduler_.empty() ||
        sif_dma_.ee_completion_pending() ||
        vu0_.running() || vu1_.running() || gs_.irq_pending()) {
        return 0;
    }

    // An armed but unmatched SIF channel is intentionally left running by
    // the BIOS. It does not need to be reprobed after every EE instruction;
    // only a ready SIF pair or another active DMA channel is observable here.
    const u16 active_dma =
        hw_.dmac_enabled() ? hw_.dmac_running_mask() : 0u;
    const u16 sif_channels = (1u << 5) | (1u << 6);
    if ((active_dma & ~sif_channels) != 0u ||
        ((active_dma & sif_channels) &
         iop_bus_.sif_dma_ready_mask()) != 0u) {
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

    // IOP-side SIF completion is safe while doing exact 8:1 interleave:
    // advance_iop_for_ee_cycles() ticks it immediately after the due IOP
    // instruction. Long idle-loop skipping still requires an event-free
    // interval and is guarded below.
    if (iop_idle && sif_dma_.iop_completion_pending()) return 0;

    const u64 video_room = video_timing_.cycles_to_transition();
    if (video_room <= 1u) return 0;

    u64 maximum = std::min<u64>(budget, kQuietEeBatchLimit);
    maximum = std::min<u64>(maximum, video_room - 1u);

    // Preserve the current 8:1 EE/IOP interleave exactly while the IOP is
    // doing real work. We may batch several EE instructions, but never cross
    // the point where the next IOP instruction is due.
    if (!iop_halted && !iop_idle) {
        const u64 until_iop_step =
            ee_iop_phase_ == 0u ? 8u : 8u - ee_iop_phase_;
        maximum = std::min<u64>(maximum, until_iop_step);
    }

    // If EE timer IRQs are possible, retain EeCpu::step's instruction-exact
    // hardware tick/IRQ polling while still batching the much larger system
    // layer. Otherwise the hardware tick itself can also be coalesced.
    const bool defer_ee_tick = !hw_.timer_irq_possible();
    if (defer_ee_tick) {
        // Stop on the instruction that reaches COP0 Compare. It can set IP7
        // at retirement; the next outer iteration then takes the exact IRQ.
        const u32 compare_distance = cpu.cop0[11] - cpu.cop0[9];
        if (compare_distance != 0u) {
            maximum = std::min<u64>(maximum, compare_distance);
        }
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

    u64 retired = 0;
    while (retired < maximum && !ee_.halted()) {
        bool progressed = false;

        if (QuietEeBlock* block = quiet_ee_block(ee_.state().pc)) {
            const u32 block_pc = block->pc;
            for (u32 i = 0;
                 i < block->count && retired < maximum;
                 ++i) {
                if (ee_.state().pc != block_pc + i * 4u) break;

                const u32 instruction = block->words[i];
                if (!quiet_ee_instruction_value(
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
    if (retired == 0u) return 0;

    // step_quiet deliberately leaves EE hardware time untouched. Apply the
    // exact number of retired cycles once, before exposing the next MMIO or
    // event boundary to the guest. The timer-IRQ path used step() and has
    // already advanced this clock instruction by instruction.
    if (defer_ee_tick) bus_.tick(retired);
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

u64 Ps2System::run_ee(u64 instruction_budget,std::string& error){
    error.clear();
    if(!bios_started_){error="BIOS has not been started.";return 0;}
    u64 executed=0;
    while(executed<instruction_budget){
        if (instruction_budget - executed >= 8u &&
            ee_.state().pc == 0x00081FC0u) {
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
        const u64 quiet_batch = try_run_quiet_ee_batch(
            instruction_budget - executed, error);
        if (quiet_batch != 0u) {
            executed += quiet_batch;
            if (!error.empty()) break;
            continue;
        }

        const u64 before=ee_.state().instructions_executed;
        if(!step_ee_core(error)){
            if(ee_.state().instructions_executed!=before)++executed;
            break;
        }
        ++executed;
    }
    return executed;
}
void Ps2System::refresh_display(){gs_display_.update(gs_,gs_core_.vram());}
std::string Ps2System::halt_reason()const{if(ee_.halted())return "EE: "+ee_.halt_reason();return {};}
} // namespace ps2
