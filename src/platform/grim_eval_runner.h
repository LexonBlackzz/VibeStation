#pragma once

#include <string>
#include <vector>

// Grim Reaper 2.0 Phase 1 command-line modes. `args` are the tokens after the
// mode flag. All force the interpreter and default the log level to Error.
// [bios] may be omitted when VIBESTATION_BIOS is set.
//
//   --grim-eval [bios] <frames> <out.jsonl> [--scenario nodisc]
//       [--max-cycles N] [--max-instructions N] [--watchdog-seconds S]
//       [--no-stop-on-death]
//     Exit code: 0 alive, 2 dead, 1 error. A silent run that drew an image
//     is alive (silent=1 in the result line).
//   --grim-determinism-test [bios] [frames=600] [threads=2] [processes=2]
//     Exit code 0 when every run (sequential, parallel threads, parallel
//     child processes) produced identical per-frame telemetry.
//   --grim-self-test [bios] [frames=1500]
//     Liveness-gate unit tests, plus stock/broken BIOS classification when a
//     BIOS path is given. Exit code 0 when everything passes.
int run_grim_eval_cli(const std::vector<std::string> &args);
int run_grim_determinism_test(const std::vector<std::string> &args,
                              const std::string &self_exe);
int run_grim_self_test(const std::vector<std::string> &args);
