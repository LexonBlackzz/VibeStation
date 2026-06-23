#pragma once

#include <filesystem>
#include <string>

std::string cue_trim_copy(std::string text);
std::string parse_cue_file_target(const std::string& line);
std::string resolve_first_bin_from_cue(const std::filesystem::path& cue_path);
