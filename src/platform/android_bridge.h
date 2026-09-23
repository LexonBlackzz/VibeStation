#pragma once

#include <string>

enum class AndroidPickerKind : int {
    None = 0,
    Bios = 1,
    Game = 2,
    RomDirectory = 3,
};

struct AndroidPickerResult {
    AndroidPickerKind kind = AndroidPickerKind::None;
    bool cancelled = false;
    std::string path;
    std::string display_name;
};

bool android_request_picker(AndroidPickerKind kind);
bool android_poll_picker_result(AndroidPickerResult& out);
