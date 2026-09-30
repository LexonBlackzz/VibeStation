#pragma once

#include <string>
#include <vector>

// Child-process runner for Grim Reaper 2.0 exploration. A hung or crashing
// emulator must never take the search down, so every candidate runs as a
// child with a hard timeout.
//
// Windows: CreateProcess (suspended), assign to a Job Object with
// kill-on-close, resume; on timeout the whole job is terminated.
// POSIX: fork/exec in its own process group; on timeout the group is SIGKILLed.

struct GrimChildResult {
  bool started = false;   // the process was created
  bool timed_out = false; // killed because the timeout expired
  bool crashed = false;   // died from an exception / fatal signal (not our kill)
  int exit_code = -1;     // process exit code (128+signal on POSIX crashes)
};

// stdout and stderr go to `output_path`. timeout_seconds <= 0 means no limit.
GrimChildResult grim_run_child(const std::string &exe,
                               const std::vector<std::string> &args,
                               const std::string &output_path,
                               double timeout_seconds);

// Absolute path of the running executable (falls back to argv0).
std::string grim_self_exe_path(const std::string &argv0);
