#include "core/ps2_system.h"
namespace ps2 {
Ps2System::Ps2System():cdvd_(iop_intc_),iop_bus_(iop_ram_,iop_hw_,hw_,iop_intc_,cdvd_,bios_),bus_(ram_,scratchpad_,hw_,iop_hw_,iop_ram_,gs_,gs_core_,bios_),vu1_(bus_,gs_core_),ee_(bus_),iop_(iop_bus_){gs_core_.attach_privileged(gs_);vif1_dma_.attach_vu1(vu1_);reset();}
void Ps2System::reset(u32 entry_point){ram_.reset();scratchpad_.reset();bus_.reset();hw_.reset();iop_hw_.reset();iop_intc_.reset();cdvd_.reset();iop_ram_.reset();iop_bus_.reset();gs_.reset();gs_core_.reset();gs_display_.reset();scheduler_.reset();video_timing_.reset();gif_dma_.reset();vif0_dma_.reset();vif1_dma_.reset();sif_dma_.reset();vu1_.reset();ee_.reset(entry_point);iop_.reset(Bios::kResetVector);bios_started_=false;reset_instruction_=0;iop_reset_instruction_=0;ee_iop_phase_=0;}
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
    iop_hw_.tick(1, iop_intc_);
    return true;
}
bool Ps2System::step_ee(std::string& error){error.clear();if(!bios_started_){error="BIOS has not been started.";return false;}if(halted()){error=halt_reason();return false;}if(!ee_.step(error)){error="EE halted: "+error;return false;}if(!gif_dma_.service(bus_,gs_core_,error)){error="GIF DMA: "+error;return false;}if(!vif0_dma_.service(bus_,error)){error="VIF0 DMA: "+error;return false;}if(!vif1_dma_.service(bus_,gs_core_,gs_,error)){error="VIF1 DMA: "+error;return false;}if(!sif_dma_.service(bus_,iop_bus_,iop_intc_,error)){error="SIF DMA: "+error;return false;}if(vu1_.running()){std::string vu_error;vu1_.run(256,vu_error);if(!vu_error.empty()){error="VU1: "+vu_error;return false;}}scheduler_.run_until(scheduler_.now()+1,{});const u64 fields_before=video_timing_.fields_started();video_timing_.tick(1,hw_,iop_intc_);if(video_timing_.fields_started()!=fields_before){gs_.raise_vsync();gs_display_.update(gs_,gs_core_.vram());}if(gs_.irq_pending())hw_.raise_intc(0);return advance_iop_for_ee_step(error);}
bool Ps2System::step_iop(std::string& error){error.clear();if(!bios_started_){error="BIOS has not been started.";return false;}if(iop_.halted()){error=iop_.halt_reason();return false;}if(!iop_.step(error))return false;iop_hw_.tick(1,iop_intc_);return true;}
u64 Ps2System::run_ee(u64 instruction_budget,std::string& error){error.clear();if(!bios_started_){error="BIOS has not been started.";return 0;}u64 executed=0;while(executed<instruction_budget&&!halted()){const u64 before=ee_.state().instructions_executed;if(!step_ee(error)){if(ee_.state().instructions_executed!=before)++executed;break;}++executed;}return executed;}
void Ps2System::refresh_display(){gs_display_.update(gs_,gs_core_.vram());}
std::string Ps2System::halt_reason()const{if(ee_.halted())return "EE: "+ee_.halt_reason();return {};}
} // namespace ps2
