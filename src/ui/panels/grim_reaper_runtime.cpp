#include "ui/app.h"

void App::sync_ram_reaper_config() {
    if (!system_) {
        return;
    }
    System::RamReaperConfig cfg{};
    cfg.enabled = ram_reaper_enabled_;
    cfg.range_start = ram_reaper_range_start_;
    cfg.range_end = ram_reaper_range_end_;
    cfg.writes_per_frame = ram_reaper_writes_per_frame_;
    cfg.intensity_percent = ram_reaper_intensity_percent_;
    cfg.affect_main_ram = ram_reaper_affect_main_ram_;
    cfg.affect_vram = ram_reaper_affect_vram_;
    cfg.affect_spu_ram = ram_reaper_affect_spu_ram_;
    cfg.use_custom_seed = ram_reaper_use_custom_seed_;
    cfg.seed = ram_reaper_seed_;
    system_->set_ram_reaper_config(cfg);
    ram_reaper_active_seed_ = system_->ram_reaper_last_seed();
    ram_reaper_total_mutations_ = system_->ram_reaper_total_mutations();
}

void App::disable_ram_reaper_mode() {
    ram_reaper_enabled_ = false;
    if (system_) {
        system_->disable_ram_reaper();
    }
}

void App::sync_gpu_reaper_config() {
    if (!system_) {
        return;
    }
    System::GpuReaperConfig cfg{};
    cfg.enabled = gpu_reaper_enabled_;
    cfg.writes_per_frame = gpu_reaper_writes_per_frame_;
    cfg.intensity_percent = gpu_reaper_intensity_percent_;
    cfg.affect_geometry = gpu_reaper_affect_geometry_;
    cfg.affect_texture_state = gpu_reaper_affect_texture_state_;
    cfg.affect_display_state = gpu_reaper_affect_display_state_;
    cfg.use_custom_seed = gpu_reaper_use_custom_seed_;
    cfg.seed = gpu_reaper_seed_;
    system_->set_gpu_reaper_config(cfg);
    gpu_reaper_active_seed_ = system_->gpu_reaper_last_seed();
    gpu_reaper_total_mutations_ = system_->gpu_reaper_total_mutations();
}

void App::disable_gpu_reaper_mode() {
    gpu_reaper_enabled_ = false;
    if (system_) {
        system_->disable_gpu_reaper();
    }
}

void App::sync_sound_reaper_config() {
    if (!system_) {
        return;
    }
    System::SoundReaperConfig cfg{};
    cfg.enabled = sound_reaper_enabled_;
    cfg.writes_per_frame = sound_reaper_writes_per_frame_;
    cfg.intensity_percent = sound_reaper_intensity_percent_;
    cfg.affect_pitch = sound_reaper_affect_pitch_;
    cfg.affect_envelope = sound_reaper_affect_envelope_;
    cfg.affect_reverb = sound_reaper_affect_reverb_;
    cfg.affect_mixer = sound_reaper_affect_mixer_;
    cfg.use_custom_seed = sound_reaper_use_custom_seed_;
    cfg.seed = sound_reaper_seed_;
    system_->set_sound_reaper_config(cfg);
    sound_reaper_active_seed_ = system_->sound_reaper_last_seed();
    sound_reaper_total_mutations_ = system_->sound_reaper_total_mutations();
}

void App::disable_sound_reaper_mode() {
    sound_reaper_enabled_ = false;
    if (system_) {
        system_->disable_sound_reaper();
    }
}
