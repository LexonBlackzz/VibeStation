#include "platform/disc_path_utils.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

std::string cue_trim_copy(std::string text) {
    const size_t begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const size_t end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1u);
}

std::string parse_cue_file_target(const std::string& line) {
    const size_t q0 = line.find('"');
    if (q0 != std::string::npos) {
        const size_t q1 = line.find('"', q0 + 1u);
        if (q1 != std::string::npos && q1 > q0 + 1u) {
            return line.substr(q0 + 1u, q1 - q0 - 1u);
        }
    }

    std::istringstream iss(line);
    std::string token;
    std::string file_name;
    iss >> token >> file_name;
    return file_name;
}

std::string resolve_first_bin_from_cue(const std::filesystem::path& cue_path) {
    std::ifstream cue(cue_path);
    if (!cue.is_open()) {
        return {};
    }

    std::string line;
    while (std::getline(cue, line)) {
        line = cue_trim_copy(line);
        if (line.empty() || line[0] == ';') {
            continue;
        }
        if (line.size() < 4u) {
            continue;
        }

        std::string head = line.substr(0u, 4u);
        std::transform(head.begin(), head.end(), head.begin(), [](unsigned char c) {
            return static_cast<char>(std::toupper(c));
            });
        if (head != "FILE") {
            continue;
        }

        std::string file_name = parse_cue_file_target(line);
        if (file_name.empty()) {
            continue;
        }

        std::filesystem::path resolved(file_name);
        if (!resolved.is_absolute()) {
            resolved = cue_path.parent_path() / resolved;
        }
        if (std::filesystem::exists(resolved)) {
            return resolved.string();
        }
        return {};
    }
    return {};
}
