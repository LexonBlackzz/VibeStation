#include "sample_profiler.h"

#if defined(_WIN32)

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <dbghelp.h>
#include <timeapi.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>
#include <unordered_map>

namespace sample_profiler {
namespace {

constexpr size_t kMaxSamples = 1u << 20;

HANDLE g_target = nullptr;
std::thread g_sampler;
std::atomic<bool> g_running{false};
// Preallocated: nothing may allocate while the target is suspended, since it
// could be holding the process heap lock.
std::vector<uintptr_t> g_samples;
size_t g_sample_count = 0;

void sampler_loop(unsigned interval_us) {
  const auto interval = std::chrono::microseconds(interval_us);
  auto next = std::chrono::steady_clock::now();
  while (g_running.load(std::memory_order_relaxed) &&
         g_sample_count < kMaxSamples) {
    next += interval;
    std::this_thread::sleep_until(next);
    if (SuspendThread(g_target) == static_cast<DWORD>(-1)) {
      continue;
    }
    CONTEXT context{};
    context.ContextFlags = CONTEXT_CONTROL;
    const bool captured = GetThreadContext(g_target, &context) != FALSE;
    ResumeThread(g_target);
    if (captured) {
      g_samples[g_sample_count++] = static_cast<uintptr_t>(context.Rip);
    }
  }
}

} // namespace

bool start(unsigned interval_us) {
  if (g_running.load()) {
    return false;
  }
  if (!DuplicateHandle(GetCurrentProcess(), GetCurrentThread(),
                       GetCurrentProcess(), &g_target,
                       THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT |
                           THREAD_QUERY_INFORMATION,
                       FALSE, 0)) {
    g_target = nullptr;
    return false;
  }
  g_samples.assign(kMaxSamples, 0u);
  g_sample_count = 0;
  timeBeginPeriod(1);
  g_running.store(true);
  g_sampler = std::thread(sampler_loop, std::max(100u, interval_us));
  return true;
}

void stop_and_report(std::FILE *out, size_t top_n,
                     const std::vector<CodeRange> &ranges) {
  if (!g_running.exchange(false)) {
    return;
  }
  g_sampler.join();
  timeEndPeriod(1);
  CloseHandle(g_target);
  g_target = nullptr;
  g_samples.resize(g_sample_count);

  const HANDLE process = GetCurrentProcess();
  SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
  const bool symbols = SymInitialize(process, nullptr, TRUE) != FALSE;

  std::unordered_map<uintptr_t, std::string> name_cache;
  std::unordered_map<std::string, size_t> buckets;
  alignas(SYMBOL_INFO) char symbol_storage[sizeof(SYMBOL_INFO) + 512];
  for (const uintptr_t ip : g_samples) {
    std::string name;
    for (const CodeRange &range : ranges) {
      if (ip >= range.begin && ip < range.end) {
        name = range.name;
        break;
      }
    }
    if (name.empty()) {
      auto cached = name_cache.find(ip);
      if (cached != name_cache.end()) {
        name = cached->second;
      } else {
        auto *symbol = reinterpret_cast<SYMBOL_INFO *>(symbol_storage);
        symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
        symbol->MaxNameLen = 511;
        DWORD64 displacement = 0;
        if (symbols && SymFromAddr(process, ip, &displacement, symbol)) {
          name.assign(symbol->Name, symbol->NameLen);
        } else {
          char unknown[64];
          std::snprintf(unknown, sizeof(unknown), "<unknown %p>",
                        reinterpret_cast<void *>(ip & ~uintptr_t{0xFFF}));
          name = unknown;
        }
        name_cache.emplace(ip, name);
      }
    }
    ++buckets[name];
  }
  if (symbols) {
    SymCleanup(process);
  }

  std::vector<std::pair<size_t, std::string>> sorted;
  sorted.reserve(buckets.size());
  for (auto &entry : buckets) {
    sorted.emplace_back(entry.second, entry.first);
  }
  std::sort(sorted.begin(), sorted.end(),
            [](const auto &a, const auto &b) { return a.first > b.first; });
  const double total = static_cast<double>(g_samples.size());
  std::fprintf(out, "SAMPLE_PROFILE samples=%zu buckets=%zu\n",
               g_samples.size(), sorted.size());
  for (size_t i = 0; i < sorted.size() && i < top_n; ++i) {
    std::fprintf(out, "SAMPLE_PROFILE %6.2f%% %7zu  %s\n",
                 total > 0.0 ? 100.0 * static_cast<double>(sorted[i].first) /
                                   total
                             : 0.0,
                 sorted[i].first, sorted[i].second.c_str());
  }
}

} // namespace sample_profiler

#else

namespace sample_profiler {
bool start(unsigned) { return false; }
void stop_and_report(std::FILE *, size_t, const std::vector<CodeRange> &) {}
} // namespace sample_profiler

#endif
