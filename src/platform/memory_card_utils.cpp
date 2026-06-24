#include "platform/memory_card_utils.h"

#include <cctype>

std::string sanitize_memory_card_stem(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (char ch : text) {
        const unsigned char uch = static_cast<unsigned char>(ch);
        if (std::isalnum(uch)) {
            out.push_back(static_cast<char>(std::tolower(uch)));
        }
        else if (ch == '-' || ch == '_') {
            out.push_back(ch);
        }
        else if (std::isspace(uch)) {
            out.push_back('_');
        }
    }
    if (out.empty()) {
        out = "game";
    }
    return out;
}
