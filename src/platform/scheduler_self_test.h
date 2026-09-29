#pragma once

#include <string>

// Headless device-level tests for the event-driven scheduler: tick additivity
// and exact event deadlines for the timers, CD-ROM, MDEC, DMA and SIO, plus
// frame-edge accounting and (when a BIOS path is given) save-state
// determinism. Returns 0 when every test passes.
int run_scheduler_self_tests(const std::string &bios_path);
