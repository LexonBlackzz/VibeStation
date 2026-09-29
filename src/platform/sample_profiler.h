#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// Minimal statistical profiler for development builds. A sampler thread
// periodically suspends the thread that called start() and records its
// instruction pointer. Samples inside caller-supplied ranges (for example
// generated JIT code) are attributed to the range name; the rest are resolved
// to function symbols (Windows DbgHelp; build with PDBs for useful names).
// Windows-only; start() returns false elsewhere.
namespace sample_profiler {

struct CodeRange {
  uintptr_t begin = 0;
  uintptr_t end = 0;
  std::string name;
};

bool start(unsigned interval_us);
// Stops sampling and prints the top_n buckets. Ranges are evaluated at report
// time, so pass the final extents of any code arena that grew while sampling.
void stop_and_report(std::FILE *out, size_t top_n,
                     const std::vector<CodeRange> &ranges);

} // namespace sample_profiler
