#pragma once
#include <string>
#include <vector>

// Phase 5 (live pulls).
//
// --grim-pull-test [bios] [--map file.json] [--frames N=420] [--backend interpreter|recompiler]
//     Generation (determinism, risk, family toggles, rot, novelty), Mercy, death
//     descriptions, the pull library, and the live death watch on a real machine
//     (stock alive, hang, exception loop, recovery, wrong-BIOS rejection). Prints
//     GRIM_PULL_TEST PASS/FAIL lines; exit 0 when everything passes. Run it once
//     per backend: the PULL_LIVE lines must match.
// --grim-pull-yield [bios] [--map file.json] [--pulls N=40] [--frames N=900]
//     [--intensity a,b,c] [--threads N=6] [--families audio,code,iface]
//     Generates real pulls with the live generator and runs each headless under the
//     live liveness config. Reports live/dead and survived-and-audible separately.
int run_grim_pull_test(const std::vector<std::string> &args);
int run_grim_pull_yield(const std::vector<std::string> &args);
