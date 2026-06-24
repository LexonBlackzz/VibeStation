#pragma once
#include "core/types.h"

struct GrimReaperRange {
    const char* label;
    const char* slug;
    u32 start;
    u32 end;
};

constexpr GrimReaperRange kGrimReaperRanges[] = {
    {"Intro/Bootmenu (0x18000-0x63FFF)", "intro", 0x18000u, 0x63FFFu},
    {"Character Sets (0x64000-0x7FF31)", "charset", 0x64000u, 0x7FF31u},
    {"End (0x7FF32-0x7FFFF)", "end", 0x7FF32u, 0x7FFFFu},
    {"Custom (Hex Range)", "custom", 0x00000u, 0x00000u},
};

constexpr int kGrimReaperRangeCount =
    static_cast<int>(sizeof(kGrimReaperRanges) / sizeof(kGrimReaperRanges[0]));
