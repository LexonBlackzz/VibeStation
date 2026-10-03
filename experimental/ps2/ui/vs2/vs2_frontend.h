#pragma once

#include "ui/vs2/vs2_host.h"
#include "ui/vs2/vs2_boot.h"
#include "ui/vs2/vs2_orbit.h"
#include "ui/vs2/vs2_shared.h"

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ps2::ui::vs2 {

// The VibeStation 2 frontend: PS2-styled menu over a black void with orbiting
// lights, laid out in the same 1280x800 design space as the PS1 definitive UI.
class Frontend {
public:
    explicit Frontend(Host& host);
    ~Frontend();
    Frontend(const Frontend&) = delete;
    Frontend& operator=(const Frontend&) = delete;

    // After the ImGui context exists, before the first frame.
    void init();
    // While the GL context is still current.
    void shutdown();

    // Draws the whole window. Call between ImGui::NewFrame() and Render().
    void frame();

    // Coming back from the developer view.
    void on_shown();
    // Going to the developer view: fades the menu ambience out.
    void on_hidden();
    // Replays the boot animation into the menu (each switch to VibeStation 2).
    void restart_boot();
    // Fades to black and hands the window back to VibeStation 1.
    void leave_to_vs1() { if (leave_t0_ < 0.0) leave_t0_ = now_; }
    // False while a menu is up, so pad keys do not reach a running game.
    [[nodiscard]] bool wants_game_input() const { return screen_ == Screen::InGame; }

private:
    enum class Screen { Intro, Home, Browser, Config, Version, Reaper, Exit, Starting, InGame };

    struct Game {
        std::string path;
        std::string title;
        std::string serial;
        std::string kind; // "PS2 DVD", "PS2 CD", "PS1 CD", ...
    };

    struct Settings {
        std::string rom_dir;
        std::string bios_path;
        bool startup_video = true;
        bool sounds = true;
        bool ambience = true;
    };

    struct Input {
        bool up = false, down = false, left = false, right = false;
        bool accept = false, back = false, triangle = false, square = false;
        float wheel = 0.0f;
        bool clicked = false;
        ImVec2 mouse{};
    };

    // vs2_frontend.cpp
    void go(Screen next, bool quiet = false);
    void begin_reveal();
    void update_intro(const Input& in);
    void draw_intro(ImDrawList* draw, const Layout& layout);
    void draw_in_game(ImDrawList* draw, const ImVec2& pos, const ImVec2& size);
    void update_in_game();
    void start_session();
    void boot_game(const Game& game);
    void toast(std::string message);
    void draw_toast(ImDrawList* draw, const Layout& layout);
    void draw_hints(ImDrawList* draw, const Layout& layout, float alpha,
                    std::initializer_list<std::pair<char, const char*>> hints);
    Input read_input() const;
    void load_settings();
    void save_settings() const;
    void apply_saved_settings();

    // vs2_home.cpp
    void update_home(const Input& in, const Layout& layout);
    void draw_home(ImDrawList* draw, const Layout& layout);
    void activate_home_item();

    // vs2_screens.cpp
    void update_browser(const Input& in, const Layout& layout);
    void draw_browser(ImDrawList* draw, const Layout& layout, float alpha);
    void update_config(const Input& in, const Layout& layout);
    void draw_config(ImDrawList* draw, const Layout& layout, float alpha);
    void change_config(int row, int direction);
    void draw_version(ImDrawList* draw, const Layout& layout, float alpha);
    void draw_reaper(ImDrawList* draw, const Layout& layout, float alpha);
    void update_exit(const Input& in, const Layout& layout);
    void draw_exit(ImDrawList* draw, const Layout& layout, float alpha);
    void draw_title(ImDrawList* draw, const Layout& layout, float alpha,
                    const char* title, const std::string& crumb, ImU32 color);
    void rescan_games();
    void stop_scan();

    Host& host_;
    Orbit orbit_{};
    BootAnimation boot_{};
    double boot_t0_ = 0.0;
    Settings settings_{};

    Screen screen_ = Screen::Home;
    Screen fading_screen_ = Screen::Home; // sub-screen still fading out
    double now_ = 0.0;
    float dt_ = 0.0f;
    double reveal_t0_ = -1.0;
    double screen_t0_ = 0.0;
    bool intro_pending_ = false;
    bool intro_revealed_ = false;
    bool first_frame_ = true;

    // Starts hidden: the menu only fades in with the reveal, never at launch.
    float home_alpha_ = 0.0f;
    float sub_alpha_ = 0.0f;
    float game_alpha_ = 0.0f;

    int home_sel_ = 1;
    float home_sel_anim_ = 1.0f;
    int config_sel_ = 0;
    int browser_sel_ = 0;
    float browser_sel_anim_ = 0.0f;
    int exit_sel_ = 0;

    std::string toast_{};
    double toast_t0_ = -10.0;
    double leave_t0_ = -1.0; // fading out towards VibeStation 1
    double game_entered_t0_ = 0.0;

    std::mutex games_mutex_{};
    std::vector<Game> games_{};
    std::thread scan_thread_{};
    std::atomic<bool> scanning_{false};
    std::atomic<bool> stop_scan_{false};
};

} // namespace ps2::ui::vs2
