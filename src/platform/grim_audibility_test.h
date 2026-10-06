#pragma once
#include <string>
#include <vector>

// --grim-audibility-test [bios] [--labels file] [--unit-only]
// Labels with audible:null are pending listening verdicts and are skipped.
int run_grim_audibility_test(const std::vector<std::string> &args);
