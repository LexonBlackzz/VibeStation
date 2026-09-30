#pragma once
#include <string>
#include <vector>

// ROM-only ADPCM discovery, optional clean-boot map confirmation/voice annotation.
// --grim-samples [bios] <out.json> [--map file.json]
int run_grim_samples_cli(const std::vector<std::string> &args);
// --grim-sample-test [bios] [--out-dir dir] [--frames N=600] [--seeds N=3]
int run_grim_sample_test(const std::vector<std::string> &args);
