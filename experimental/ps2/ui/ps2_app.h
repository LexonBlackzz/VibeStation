#pragma once

#include "core/ps2_system.h"

#include <array>
#include <string>

struct SDL_Window;
typedef void* SDL_GLContext;

namespace ps2::ui {

class Ps2App {
public:
    bool init();
    void run();
    void shutdown();

private:
    void process_events(bool& quit);
    void render_ui();
    void menu_bar();
    void panel_main();
    void panel_system();
    void panel_ee_debug();
    void panel_iop_debug();
    void panel_gs_debug();
    void panel_scheduler();
    void panel_settings();
    void panel_about();

    std::string open_bios_dialog();
    bool load_bios_from_path(const std::string& path);
    bool start_bios();
    bool step_ee_once();
    bool step_iop_once();
    void update_emulation();
    void reset_core();

    SDL_Window* window_ = nullptr;
    SDL_GLContext gl_context_ = nullptr;
    const char* imgui_glsl_version_ = "#version 330";
    bool use_imgui_opengl2_backend_ = false;

    Ps2System system_{};

    bool show_system_ = false;
    bool show_ee_debug_ = false;
    bool show_iop_debug_ = false;
    bool show_gs_debug_ = false;
    bool show_scheduler_ = false;
    bool show_settings_ = false;
    bool show_about_ = false;
    bool emulation_running_ = false;

    std::array<char, 1024> bios_path_input_{};
    std::string status_message_ = "PS2 experimental core ready";
};

} // namespace ps2::ui
