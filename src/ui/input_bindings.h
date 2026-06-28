#pragma once

#include "input/controller.h"

struct KeyboardBindEntry {
    const char* label;
    const char* config_key;
    PsxButton button;
};

constexpr int kKeyboardBindEntryCount = 14;
extern const KeyboardBindEntry kKeyboardBindEntries[kKeyboardBindEntryCount];
