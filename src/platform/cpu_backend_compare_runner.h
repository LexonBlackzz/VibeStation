#pragma once

// Run the CPU backend compare-test suite. Returns 0 on success, non-zero on
// failure. When memory_only is true, only cases that exercise memory helpers
// are executed (used by the legacy --jit-memory-compare-test flag).
int run_cpu_backend_compare_test(bool memory_only = false);
