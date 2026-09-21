#pragma once
#include "core/bios/bios.h"
#include "core/cdvd/cdvd_hw.h"
#include "core/dma/gif_dma.h"
#include "core/dma/sif_dma.h"
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
    Scheduler& scheduler(){return scheduler_;} const Scheduler& scheduler()const{return scheduler_;}
    EeCpu& ee(){return ee_;} const EeCpu& ee()const{return ee_;}
    IopCpu& iop(){return iop_;} const IopCpu& iop()const{return iop_;}
    IopIntc& iop_intc(){return iop_intc_;} const IopIntc& iop_intc()const{return iop_intc_;}
    GsCore& gs_core(){return gs_core_;} const GsCore& gs_core()const{return gs_core_;}
    GsPrivileged& gs_privileged(){return gs_;} const GsPrivileged& gs_privileged()const{return gs_;}
    GsDisplay& gs_display(){return gs_display_;} const GsDisplay& gs_display()const{return gs_display_;}
    Vu1& vu1(){return vu1_;} const Vu1& vu1()const{return vu1_;}
    bool bios_started()const{return bios_started_;}
    // The EE owns the user-visible bootstrap run state. An IOP halt is
    // retained for diagnostics but does not discard EE/GS progress.
    bool halted()const{return ee_.halted();}
    bool iop_halted()const{return iop_.halted();}
    std::string halt_reason()const;
    u32 reset_instruction()const{return reset_instruction_;}
    u32 iop_reset_instruction()const{return iop_reset_instruction_;}
private:
    bool advance_iop_for_ee_step(std::string& error);
    Bios bios_{}; IopIntc iop_intc_{}; CdvdHw cdvd_; EeRam ram_{}; EeScratchpad scratchpad_{};
    EeHw hw_{}; IopHwWindow iop_hw_{}; IopRam iop_ram_{}; GsPrivileged gs_{}; GsCore gs_core_{}; GsDisplay gs_display_{};
    IopBus iop_bus_; EeBus bus_; Vu1 vu1_; Scheduler scheduler_{}; VideoTiming video_timing_{}; GifDma gif_dma_{}; Vif0Dma vif0_dma_{}; Vif1Dma vif1_dma_{}; SifDma sif_dma_{}; EeCpu ee_; IopCpu iop_;
    bool bios_started_=false; u32 reset_instruction_=0; u32 iop_reset_instruction_=0; u32 ee_iop_phase_=0;
};
} // namespace ps2
