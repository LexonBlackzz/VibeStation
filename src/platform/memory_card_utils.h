#pragma once

#include <string>

constexpr const char* kMemoryCardDirName = "memcards";

std::string sanitize_memory_card_stem(const std::string& text);
