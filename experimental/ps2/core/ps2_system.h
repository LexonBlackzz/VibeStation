#pragma once
#include "core/bios/bios.h"
#include "core/cdvd/cdvd_hw.h"
#include "core/dma/gif_dma.h"
#include "core/dma/ipu_dma.h"
#include "core/dma/sif_dma.h"
#include "core/dma/spr_dma.h"
#include "core/dma/vif0_dma.h"
#include "core/dma/vif1_dma.h"
#include "core/ee/ee_cpu.h"
#include "core/gs/gs_core.h"
#include "core/gs/gs_display.h"
#include "core/gs/gs_privileged.h"
#include "core/hw/ee_hw.h"
#include "core/hw/iop_hw_window.h"
#include "core/iop/iop_bus.h"
#include "core/iop/iop_cpu.h"
#include "core/iop/iop_intc.h"
#include "core/iop/iop_ram.h"
#include "core/memory/ee_bus.h"
#include "core/memory/ee_ram.h"
#include "core/memory/ee_scratchpad.h"
#include "core/scheduler/scheduler.h"
#include "core/video/video_timing.h"
#include "core/vu/vu1.h"
#include <string>
#include <array>
#include <vector>
#include <utility>
namespace ps2 {
class Ps2System {
public:
    Ps2System();
    void reset(u32 entry_point=0);
    bool load_bios(const std::string& path,std::string& error);
    bool boot_bios(std::string& error);
    bool step_ee(std::string& error); bool step_iop(std::string& error);
    u64 run_ee(u64 instruction_budget,std::string& error);
    void refresh_display();
    Bios& bios(){return bios_;} const Bios& bios()const{return bios_;}
    EeRam& ram(){return ram_;} const EeRam& ram()const{return ram_;}
    IopRam& iop_ram(){return iop_ram_;} const IopRam& iop_ram()const{return iop_ram_;}
    EeBus& bus(){return bus_;} const EeBus& bus()const{return bus_;}
    IopBus& iop_bus(){return iop_bus_;} const IopBus& iop_bus()const{return iop_bus_;}
    Sio2Pad& pad(){return iop_bus_.sio2();} const Sio2Pad& pad()const{return iop_bus_.sio2();}
    Spu2& spu2(){return iop_bus_.spu2();} const Spu2& spu2()const{return iop_bus_.spu2();}
    Scheduler& scheduler(){return scheduler_;} const Scheduler& scheduler()const{return scheduler_;}
    const SifDma& sif_dma() const { return sif_dma_; }
    const Vif1Dma& vif1_dma() const { return vif1_dma_; }
    EeCpu& ee(){return ee_;} const EeCpu& ee()const{return ee_;}
    IopCpu& iop(){return iop_;} const IopCpu& iop()const{return iop_;}
    IopIntc& iop_intc(){return iop_intc_;} const IopIntc& iop_intc()const{return iop_intc_;}
    GsCore& gs_core(){return gs_core_;} const GsCore& gs_core()const{return gs_core_;}
    GsPrivileged& gs_privileged(){return gs_;} const GsPrivileged& gs_privileged()const{return gs_;}
    GsDisplay& gs_display(){return gs_display_;} const GsDisplay& gs_display()const{return gs_display_;}
    Vu1& vu0(){return vu0_;} const Vu1& vu0()const{return vu0_;}
    Vu1& vu1(){return vu1_;} const Vu1& vu1()const{return vu1_;}
    bool bios_started()const{return bios_started_;}
    // The EE owns the user-visible bootstrap run state. An IOP halt is
    // retained for diagnostics but does not discard EE/GS progress.
    bool halted()const{return ee_.halted();}
    bool iop_halted()const{return iop_.halted();}
    std::string halt_reason()const;
    u32 reset_instruction()const{return reset_instruction_;}
    u32 iop_reset_instruction()const{return iop_reset_instruction_;}
    u64 video_fields_started() const { return video_timing_.fields_started(); }
    u64 skipped_bios_idle_iterations() const { return skipped_bios_idle_iterations_; }
    u64 skipped_bios_idle_offphase_batches() const {
        return skipped_bios_idle_offphase_batches_;
    }
    u64 skipped_bios_zero_iterations() const { return skipped_bios_zero_iterations_; }
    u64 skipped_bios_nibble_iterations() const { return skipped_bios_nibble_iterations_; }
    u64 skipped_bios_count_wait_iterations() const { return skipped_bios_count_wait_iterations_; }
    u64 skipped_bios_countdown_iterations() const { return skipped_bios_countdown_iterations_; }
    u64 skipped_bios_copy_iterations() const { return skipped_bios_copy_iterations_; }
    u64 skipped_bios_mmio_poll_iterations() const { return skipped_bios_mmio_poll_iterations_; }
    u64 sif_poll_fast_samples() const { return sif_poll_fast_samples_; }
    u64 sif_poll_stable_returns() const { return sif_poll_stable_returns_; }
    u64 skipped_iop_idle_pairs() const { return skipped_iop_idle_pairs_; }
    u64 skipped_bios_literal_iterations() const { return skipped_bios_literal_iterations_; }
    u64 quiet_ee_batch_instructions() const { return quiet_ee_batch_instructions_; }
    u64 quiet_ee_active_iop_instructions() const {
        return quiet_ee_active_iop_instructions_;
    }
    u64 quiet_ee_batches() const { return quiet_ee_batches_; }
    u64 quiet_block_instructions() const { return quiet_block_instructions_; }
    u64 quiet_block_hits() const { return quiet_block_hits_; }
    u64 quiet_block_compiles() const { return quiet_block_compiles_; }
    u64 fast_interpreter_instructions() const {
        return fast_interpreter_instructions_;
    }
    u64 fast_interpreter_calls() const {
        return fast_interpreter_calls_;
    }
    u64 quiet_superbatch_calls() const {
        return quiet_superbatch_calls_;
    }
    u64 quiet_superbatch_instructions() const {
        return quiet_superbatch_instructions_;
    }
    const std::array<u64, 64>& native_fallback_opcodes() const {
        return native_fallback_opcodes_;
    }
    const std::array<u64, 8>& idle_skip_reasons() const { return idle_skip_reasons_; }
    std::vector<std::pair<u32, u64>> quiet_block_hotspots(
        std::size_t limit = 32u) const;
private:
    bool advance_iop_for_ee_step(std::string& error);
    bool step_ee_core(std::string& error);
    void advance_iop_for_ee_cycles(u64 cycles, std::string& error);
    u64 try_skip_bios_idle_iterations(u64 budget, std::string& error);
    u64 try_skip_bios_zero_loop(u64 budget, std::string& error);
    u64 try_skip_bios_nibble_loop(u64 budget, std::string& error);
    u64 try_skip_bios_count_wait(u64 budget, std::string& error);
    u64 try_skip_bios_countdown_wait(u64 budget, std::string& error);
    u64 try_skip_bios_copy_iterations(u64 budget, std::string& error);
    u64 try_skip_bios_mmio_poll_iterations(u64 budget, std::string& error);
    u64 try_skip_bios_literal_iterations(u64 budget, std::string& error);
    u64 try_run_quiet_ee_batch(u64 budget, std::string& error);
    u64 try_run_quiet_ee_superbatch(u64 budget, std::string& error);
    struct QuietEeBlock {
        u32 pc = 0;
        u32 page_generation = 0;
        u8 count = 0;
        u32 memory_mask = 0;
        u32 store_mask = 0;
        u64 hits = 0;
        std::array<u32, 32> words{};
    };
    QuietEeBlock* quiet_ee_block(u32 pc);
    void reset_iop_subsystem();
    Bios bios_{}; IopIntc iop_intc_{}; CdvdHw cdvd_; EeRam ram_{}; EeScratchpad scratchpad_{};
    EeHw hw_{}; IopHwWindow iop_hw_{}; IopRam iop_ram_{}; GsPrivileged gs_{}; GsCore gs_core_{}; GsDisplay gs_display_{};
    IopBus iop_bus_; EeBus bus_; Vu1 vu0_; Vu1 vu1_; Scheduler scheduler_{}; VideoTiming video_timing_{}; GifDma gif_dma_{}; IpuDma ipu_dma_{}; Vif0Dma vif0_dma_{}; Vif1Dma vif1_dma_{}; SifDma sif_dma_{}; SprDma spr_dma_{}; EeCpu ee_; IopCpu iop_;
    std::string iop_step_error_scratch_{};
    bool bios_started_=false; u32 reset_instruction_=0; u32 iop_reset_instruction_=0; u32 ee_iop_phase_=0;
    u64 skipped_bios_idle_iterations_=0;
    u64 skipped_bios_idle_offphase_batches_=0;
    u64 skipped_bios_zero_iterations_=0;
    u64 skipped_bios_nibble_iterations_=0;
    u64 skipped_bios_count_wait_iterations_=0;
    u64 skipped_bios_countdown_iterations_=0;
    u64 skipped_bios_copy_iterations_=0;
    u64 skipped_bios_mmio_poll_iterations_=0;
    u64 sif_poll_fast_samples_=0;
    u64 sif_poll_stable_returns_=0;
    u64 skipped_iop_idle_pairs_=0;
    u64 skipped_bios_literal_iterations_=0;
    u64 quiet_ee_batch_instructions_=0;
    u64 quiet_ee_active_iop_instructions_=0;
    u64 quiet_ee_batches_=0;
    u64 quiet_block_instructions_=0;
    u64 quiet_block_hits_=0;
    u64 quiet_block_compiles_=0;
    u64 fast_interpreter_instructions_=0;
    u64 fast_interpreter_calls_=0;
    u64 quiet_superbatch_calls_=0;
    u64 quiet_superbatch_instructions_=0;
    std::vector<QuietEeBlock> quiet_ee_blocks_{32768};
    std::array<u64, 64> native_fallback_opcodes_{};
    std::array<u64, 8> idle_skip_reasons_{};
};
} // namespace ps2
