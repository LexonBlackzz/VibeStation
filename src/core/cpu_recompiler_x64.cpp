#include "cpu_recompiler.h"

bool CpuOptimizedBackend::x64_jit_available() const {
#if defined(VIBESTATION_ENABLE_X64_JIT)
  // Xbyak is wired into the build for the future native backend, but this file
  // intentionally does not emit code until guest instructions are implemented
  // directly in x64. A helper-loop trampoline is not a JIT.
  return false;
#else
  return false;
#endif
}
