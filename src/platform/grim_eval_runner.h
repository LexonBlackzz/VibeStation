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

// Phase 2.
//   --grim-eval ... also takes: --genome file.json, --dump-wav out.wav,
//     --dump-frames dir every_n (BMP). The result line gains genome=0x<hash>.
//   --grim-random-genome <seed> <out.json> [gene_count]
//   --grim-explore <seed_start> <count> <out_dir> [frames=900] [--bios path]
//       [--timeout S=300] [--families spu|gpu|both] [--genes N]
//     Generates a genome per seed and evaluates it in a child process with a
//     hard timeout. Survivors (alive) keep genome, telemetry, WAV and a few
//     frames under <out_dir>/survivors/seed_N; hangs and host crashes save the
//     genome under <out_dir>/crashes. Ends with a summary table
//     (also <out_dir>/summary.tsv).
//     --mix samples/all compares captured PCM against a clean boot over the
//     same frames. Each survivor includes audibility.json; the summary reports
//     provisional audibility, peak residual dB and time above threshold.
//     sample_survival.csv separates survived+audible from survived+inaudible.
//   --grim-gene-test [bios] [frames=900]
//     Genome, filter, trigger and end-to-end tests. Needs the BIOS for the
//     end-to-end, determinism and explore tests.
int run_grim_random_genome_cli(const std::vector<std::string> &args);
int run_grim_explore_cli(const std::vector<std::string> &args,
                         const std::string &self_exe);
int run_grim_gene_test(const std::vector<std::string> &args,
                       const std::string &self_exe);

// Shared with grim_gene_test.cpp.
void grim_prepare_eval_process();
std::string grim_take_bios_arg(std::vector<std::string> &args);
