#include "ui/ps2_app.h"

#include <cerrno>
#include <cstdlib>
#include <cstdio>
#include <string>

int main(int argc, char** argv) {
    std::string bios_path;
    std::string capture_path;
    unsigned long long capture_after_ee = 0;

    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--bios" && i + 1 < argc) {
            bios_path = argv[++i];
        } else if (argument == "--capture-visible" && i + 1 < argc) {
            capture_path = argv[++i];
        } else if (argument == "--capture-after-ee" && i + 1 < argc) {
            const char* value = argv[++i];
            char* end = nullptr;
            errno = 0;
            capture_after_ee = std::strtoull(value, &end, 10);
            if (errno != 0 || end == value || *end != '\0' || value[0] == '-') {
                std::fprintf(stderr, "Invalid --capture-after-ee value.\n");
                return 2;
            }
        } else {
            std::fprintf(
                stderr,
                "Usage: VibeStationPS2Lab [--bios <path>] "
                "[--capture-visible <window.ppm>] "
                "[--capture-after-ee <instructions>]\n");
            return 2;
        }
    }

    if ((!capture_path.empty() && bios_path.empty()) ||
        (capture_after_ee != 0 && capture_path.empty())) {
        std::fprintf(
            stderr,
            "Capture requires --bios and --capture-visible.\n");
        return 2;
    }

    ps2::ui::Ps2App app;
    if (!app.init()) {
        return 1;
    }

    if (!bios_path.empty() && !app.launch_bios(bios_path)) {
        app.shutdown();
        return 1;
    }
    if (!capture_path.empty()) {
        app.capture_visible_window(capture_path, capture_after_ee);
    }

    const int result = app.run();
    app.shutdown();
    return result;
}
