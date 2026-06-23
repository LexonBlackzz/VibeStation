#pragma once
#include "core/cpu.h"
#include "core/types.h"

void draw_cpu_backend_instruction_mix(const CpuBackendStats& stats);
void draw_cpu_backend_mode_summary(
    const CpuBackendStats& stats, CpuExecutionMode requested_mode);
bool draw_cpu_backend_logging_section(
    const CpuBackendStats& stats, CpuExecutionMode mode);
