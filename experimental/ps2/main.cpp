#include "ui/ps2_app.h"

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <string>

int main(int argc, char** argv) {
    const auto startup_clock = std::chrono::steady_clock::now();
    std::string bios_path;
    std::string disc_path;
    std::string ps1_bios_path;
    std::string capture_path;
    unsigned long long capture_after_ee = 0;
    bool ee_jit = false;
    bool ee_dynarec = false;
    unsigned long long benchmark_fields = 0;
    bool gpu_gs = false;
    bool developer_view = false;

    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--bios" && i + 1 < argc) {
            bios_path = argv[++i];
        } else if (argument == "--disc" && i + 1 < argc) {
            disc_path = argv[++i];
        } else if (argument == "--ps1-bios" && i + 1 < argc) {
            ps1_bios_path = argv[++i];
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
        } else if (argument == "--benchmark-fields" && i + 1 < argc) {
            benchmark_fields = std::strtoull(argv[++i], nullptr, 10);
        } else if (argument == "--dev") {
            developer_view = true;
        } else if (argument == "--gpu-gs") {
            gpu_gs = true;
        } else if (argument == "--ee-jit") {
            ee_jit = true;
        } else if (argument == "--ee-dynarec") {
            ee_dynarec = true;
        } else {
            std::fprintf(
                stderr,
                "Usage: VibeStationPS2Lab [--bios <path>] [--disc <image>] "
                "[--ps1-bios <path>] "
                "[--ee-jit|--ee-dynarec] "
                "[--capture-visible <window.ppm>] "
                "[--capture-after-ee <instructions>] "
                "[--benchmark-fields <fields>] [--gpu-gs]\n");
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

    if (ee_jit && ee_dynarec) {
        std::fprintf(
            stderr,
            "--ee-jit and --ee-dynarec are mutually exclusive.\n");
        return 2;
    }

    ps2::ui::Ps2App app;
    if (!app.init()) {
        return 1;
    }
    app.set_ps1_bios(ps1_bios_path);
    // Only override the saved EE core when asked to on the command line.
    if (ee_jit) app.set_ee_jit_enabled(true);
    if (ee_dynarec) app.set_ee_dynarec_enabled(true);
    if (developer_view) app.set_developer_view(true);
    if (gpu_gs) app.set_gpu_gs_enabled(true);

    if (!disc_path.empty() && !app.load_disc_from_path(disc_path)) {
        app.shutdown();
        return 1;
    }
    if (!bios_path.empty() && !app.launch_bios(bios_path)) {
        app.shutdown();
        return 1;
    }
    if (!capture_path.empty()) {
        app.capture_visible_window(capture_path, capture_after_ee);
    }
    if (benchmark_fields != 0) {
        app.benchmark_visible_fields(benchmark_fields);
    }

    const int result = app.run();
    if (!capture_path.empty()) {
        std::fprintf(stdout, "UI_CAPTURE_WALL_MS=%lld\n",
            static_cast<long long>(std::chrono::duration_cast<
                std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - startup_clock).count()));
    }
    app.shutdown();
    return result;
}
