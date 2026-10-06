#pragma once
#include <string>
#include <vector>

// --grim-audibility <bios> <frames> <genome|clean> <clean.wav> <out.json>
//   [--native-cpu] [--dump-wav path] [--telemetry path]
// --grim-audio-compare <clean.wav> <mutated.wav> [out.json]
int run_grim_audibility_cli(const std::vector<std::string> &args);
int run_grim_audio_compare_cli(const std::vector<std::string> &args);
