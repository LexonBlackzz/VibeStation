#include "ui/vs2/vs2_frontend.h"

#include <SDL.h>

#include <cstdio>
#include <fstream>
#include <sstream>

namespace ps2::ui::vs2 {

namespace {

constexpr const char* kSettingsFile = "vibestation2.ini";
constexpr float kRevealAt = 4.5f;      // vs2start.mp4 turns black here
constexpr float kStartingSeconds = 1.1f;
constexpr float kHandoffSeconds = 1.2f;
constexpr float kDisclaimerSeconds = 2.0f;

bool is_sub_screen(int screen) {
    // Browser, Config, Version, Reaper, Exit
    return screen >= 2 && screen <= 6;
}

const char* ee_core_key(EeCore core) {
    switch (core) {
    case EeCore::Jit: return "jit";
    case EeCore::Dynarec: return "dynarec";
    default: return "interpreter";
    }
}

} // namespace

Frontend::Frontend(Host& host) : host_(host) {}

Frontend::~Frontend() { stop_scan(); }

void Frontend::init() {
    load_fonts();
    load_settings();
    apply_saved_settings();
    set_sounds_enabled(settings_.sounds);
    orbit_.create_textures();
    preload_sounds();
    mixer_open();

    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    boot_.create_textures();
    ps1_logo_.texture = load_image_texture("ps.png", ps1_logo_.width, ps1_logo_.height);
    ps2_logo_.texture = load_image_texture("ps2.png", ps2_logo_.width, ps2_logo_.height);
    if (settings_.startup_video && !host_.session_active()) {
        intro_pending_ = true;
        orbit_.hide();
        screen_ = Screen::Intro;
    }
    if (!settings_.rom_dir.empty()) rescan_games();
}

void Frontend::shutdown() {
    stop_scan();
    boot_.destroy_textures();
    orbit_.destroy_textures();
    destroy_texture(ps1_logo_.texture);
    destroy_texture(ps2_logo_.texture);
    mixer_release();
    release_sounds();
}

void Frontend::on_hidden() {
    ambience_set_active(false);
    stop_boot_sound();
}

void Frontend::restart_boot() {
    leave_t0_ = -1.0;
    stop_boot_sound();
    home_alpha_ = sub_alpha_ = game_alpha_ = 0.0f;
    home_sel_ = 1;
    home_sel_anim_ = 1.0f;
    reveal_t0_ = -1.0;
    first_frame_ = true;
    // The boot animation plays once per run; coming back from VibeStation 1
    // goes straight to the menu (with vs2-backtomenu.wav, see frame()).
    intro_pending_ = settings_.startup_video && !shown_before_;
    if (intro_pending_) {
        orbit_.hide();
        screen_ = Screen::Intro;
    } else {
        screen_ = Screen::Home;
    }
}

void Frontend::on_shown() {
    if (host_.session_running()) {
        go(Screen::InGame, true);
        game_alpha_ = 1.0f;
    } else if (screen_ == Screen::InGame || screen_ == Screen::Starting) {
        go(Screen::Home, true);
    }
}

// ------------------------------------------------------------------ frame

void Frontend::frame() {
    const double now = static_cast<double>(SDL_GetTicks64()) / 1000.0;
    dt_ = first_frame_ ? 1.0f / 60.0f : std::clamp(static_cast<float>(now - now_), 0.0f, 0.1f);
    now_ = now;

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(viewport->Size);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin("##vibestation2", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBackground |
                     ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollWithMouse);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 pos = viewport->Pos;
    const ImVec2 size = viewport->Size;
    draw->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(0, 0, 0, 255));
    const Layout layout = make_layout(pos, size);

    if (first_frame_) {
        first_frame_ = false;
        // The first time VibeStation 2 appears in a run, a short
        // non-affiliation notice comes before anything else (not when a game
        // was launched straight from the command line).
        if (!shown_before_ && !host_.session_running()) disclaimer_t0_ = now_;
        else begin_first_screen();
        shown_before_ = true;
    }
    if (disclaimer_t0_ >= 0.0) {
        draw_disclaimer(draw, layout, static_cast<float>(now_ - disclaimer_t0_));
        if (now_ - disclaimer_t0_ >= kDisclaimerSeconds) {
            disclaimer_t0_ = -1.0;
            begin_first_screen();
        }
        ImGui::End();
        ImGui::PopStyleVar(3);
        return;
    }

    Input in = read_input();
    // A hint at the bottom was clicked last frame: act as if its button was pressed.
    switch (hint_clicked_) {
    case 'x': in.accept = true; break;
    case 'o': in.back = true; break;
    case 't': in.triangle = true; break;
    case 's': in.square = true; break;
    default: break;
    }
    hint_clicked_ = 0;
    switch (screen_) {
    case Screen::Intro: update_intro(in); break;
    case Screen::Home: update_home(in, layout); break;
    case Screen::Browser: update_browser(in, layout); break;
    case Screen::Config: update_config(in, layout); break;
    case Screen::Version:
    case Screen::Reaper:
        if (in.back || in.accept || in.clicked) go(Screen::Home);
        break;
    case Screen::Exit: update_exit(in, layout); break;
    case Screen::Starting:
        if (now_ - screen_t0_ >= kStartingSeconds) {
            if (host_.session_active()) {
                go(Screen::InGame, true);
                game_entered_t0_ = now_;
            } else {
                toast(host_.status_message());
                go(Screen::Home, true);
            }
        }
        break;
    case Screen::InGame: update_in_game(); break;
    case Screen::Handoff:
        // Show the message for a moment, then fade out to VibeStation 1.
        if (now_ - screen_t0_ >= kHandoffSeconds && leave_t0_ < 0.0) leave_t0_ = now_;
        break;
    }

    const int screen = static_cast<int>(screen_);
    if (is_sub_screen(screen)) fading_screen_ = screen_;
    home_alpha_ = approach(home_alpha_, screen_ == Screen::Home ? 1.0f : 0.0f, 10.0f, dt_);
    sub_alpha_ = approach(sub_alpha_, is_sub_screen(screen) ? 1.0f : 0.0f, 10.0f, dt_);
    game_alpha_ = approach(game_alpha_, screen_ == Screen::InGame ? 1.0f : 0.0f, 9.0f, dt_);

    float presence = 1.0f;
    if (is_sub_screen(screen)) presence = .15f;
    if (screen_ == Screen::Exit) presence = .15f;
    if (screen_ == Screen::Starting || screen_ == Screen::Handoff) presence = .6f;
    if (screen_ == Screen::InGame) presence = 0.0f;
    const bool reaper =
        (screen_ == Screen::Home && home_sel_ == 3) || screen_ == Screen::Reaper;
    orbit_.set_targets(presence, reaper ? 1.0f : 0.0f, screen_ == Screen::Starting ? 1.0f : 0.0f);
    orbit_.update(now_, dt_);

    // Ambience: only in the menus, a few seconds after the reveal so it does
    // not fight the boot sound.
    const bool in_menus = screen_ != Screen::Intro && screen_ != Screen::Starting &&
                          screen_ != Screen::InGame && screen_ != Screen::Handoff;
    ambience_set_active(settings_.ambience && in_menus && reveal_t0_ >= 0.0 && now_ - reveal_t0_ > 3.0);
    ambience_update(dt_);

    if (game_alpha_ < 0.999f) {
        orbit_.draw(draw, layout);
        if (screen_ == Screen::Intro) draw_intro(draw, layout);
        if (home_alpha_ > 0.01f) draw_home(draw, layout);
        if (sub_alpha_ > 0.01f) {
            switch (fading_screen_) {
            case Screen::Browser: draw_browser(draw, layout, sub_alpha_); break;
            case Screen::Config: draw_config(draw, layout, sub_alpha_); break;
            case Screen::Version: draw_version(draw, layout, sub_alpha_); break;
            case Screen::Reaper: draw_reaper(draw, layout, sub_alpha_); break;
            case Screen::Exit: draw_exit(draw, layout, sub_alpha_); break;
            default: break;
            }
        }
        if (screen_ == Screen::Starting) {
            const float a = smoothstep(0.15f, 0.5f, static_cast<float>(now_ - screen_t0_));
            const char* label = "Starting BIOS...";
            const ImVec2 ts = text_size(FontRole::Light, layout.px(22), label);
            text(draw, FontRole::Light, layout.px(22),
                 ImVec2(layout.point(640, 0).x - ts.x * 0.5f, layout.point(0, 600).y),
                 with_alpha(color::kText, a), label);
        }
        if (screen_ == Screen::Handoff) {
            const float a = smoothstep(0.1f, 0.45f, static_cast<float>(now_ - screen_t0_));
            const char* label = "Switching to VibeStation 1";
            const ImVec2 ts = text_size(FontRole::Light, layout.px(26), label);
            text(draw, FontRole::Light, layout.px(26),
                 ImVec2(layout.point(640, 0).x - ts.x * 0.5f, layout.point(0, 560).y),
                 with_alpha(color::kText, a), label);
            const std::string sub = "PS1 games run in VibeStation 1 · " + handoff_title_;
            const ImVec2 ss = text_size(FontRole::Regular, layout.px(13), sub.c_str());
            text(draw, FontRole::Regular, layout.px(13),
                 ImVec2(layout.point(640, 0).x - ss.x * 0.5f, layout.point(0, 606).y),
                 with_alpha(color::kSub, a), sub.c_str());
        }
    }
    if (game_alpha_ > 0.001f) draw_in_game(draw, pos, size);
    draw_toast(draw, layout);

    // "VibeStation 1" chosen: fade to black, then hand the window back.
    if (leave_t0_ >= 0.0) {
        const float k = smoothstep(0.0f, 0.5f, static_cast<float>(now_ - leave_t0_));
        draw->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y),
                            IM_COL32(0, 0, 0, static_cast<int>(255.0f * k)));
        if (now_ - leave_t0_ >= 0.6) {
            leave_t0_ = -1.0;
            host_.switch_to_vs1();
        }
    }

    ImGui::End();
    ImGui::PopStyleVar(3);
}

void Frontend::begin_first_screen() {
    if (host_.session_running()) {
        // Launched with --bios or a disc: go straight to the game. (A game
        // paused before switching away waits behind the menu instead.)
        intro_pending_ = false;
        begin_reveal();
        go(Screen::InGame, true);
        game_alpha_ = 1.0f;
    } else if (intro_pending_) {
        boot_t0_ = now_;
        play_boot_sound();
    } else {
        // No boot animation (switched back, or turned off): the menu
        // animates in to its own jingle.
        begin_reveal();
        play_back_to_menu_sound();
    }
}

void Frontend::draw_disclaimer(ImDrawList* draw, const Layout& layout, float t) {
    const float a = smoothstep(0.0f, 0.3f, t) *
                    (1.0f - smoothstep(kDisclaimerSeconds - 0.35f, kDisclaimerSeconds, t));
    if (a <= 0.001f) return;
    constexpr std::array<const char*, 3> kLines = {{
        "VibeStation is an independent, non-commercial fan project.",
        "It is not affiliated with, endorsed by or sponsored by Sony Interactive Entertainment.",
        "PlayStation names, logos and sounds belong to Sony and are used under fair use.",
    }};
    for (std::size_t i = 0; i < kLines.size(); ++i) {
        const float size = layout.px(i == 0 ? 19.0f : 14.0f);
        const FontRole role = i == 0 ? FontRole::Light : FontRole::Regular;
        const ImVec2 ts = text_size(role, size, kLines[i]);
        text(draw, role, size,
             ImVec2(layout.point(640, 0).x - ts.x * 0.5f,
                    layout.point(0, i == 0 ? 356.0f : 372.0f + 26.0f * static_cast<float>(i)).y),
             with_alpha(i == 0 ? color::kText : color::kSub, a), kLines[i]);
    }
}

void Frontend::go(Screen next, bool quiet) {
    if (next == screen_) return;
    if (!quiet) {
        if (next == Screen::Starting) play_select_sound();
        else if (next == Screen::Home) play_close_sound();
        else if (next != Screen::InGame && next != Screen::Intro) play_open_sound();
    }
    screen_ = next;
    screen_t0_ = now_;
}

void Frontend::begin_reveal() {
    reveal_t0_ = now_;
    intro_revealed_ = true;
    orbit_.begin_reveal(now_);
}

// ------------------------------------------------------------------ intro

void Frontend::update_intro(const Input& in) {
    const double t = now_ - boot_t0_;
    const bool skip = in.accept || in.back || in.clicked;
    if (t >= kRevealAt || skip) {
        // The boot sound plays on under the menu; skipping swaps it for the
        // back-to-menu jingle.
        intro_pending_ = false;
        begin_reveal();
        if (t < kRevealAt) {
            stop_boot_sound(); // also when menu sounds (and so the jingle) are off
            play_back_to_menu_sound();
        }
        go(Screen::Home, true);
    }
}

void Frontend::draw_intro(ImDrawList* draw, const Layout& layout) {
    boot_.draw(draw, layout, static_cast<float>(now_ - boot_t0_));
}

// ------------------------------------------------------------------ game

void Frontend::start_session() {
    if (!host_.bios_loaded()) {
        const std::string path = host_.pick_bios_file();
        if (path.empty()) {
            toast("Choose a PS2 BIOS to start emulation.");
            return;
        }
        if (!host_.load_bios(path)) {
            toast(host_.status_message());
            return;
        }
        settings_.bios_path = path;
        save_settings();
    }
    if (host_.start_bios_session()) {
        last_boot_disc_.clear();
        edge_light_.reset();
        go(Screen::Starting);
    } else {
        toast(host_.status_message());
    }
}

void Frontend::boot_game(const Game& game) {
    // Inside VibeStation, PS1 discs go to VibeStation 1 with its own toolbar
    // and overlay; VibeStation 2's are built around the PS2 core. (The
    // standalone lab keeps running them on its built-in PS1 core.)
    if (game.kind.rfind("PS1", 0) == 0 && host_.can_switch_to_vs1()) {
        play_select_sound();
        host_.pause_session();
        host_.hand_ps1_disc_to_vs1(game.path);
        handoff_title_ = game.title;
        go(Screen::Handoff, true);
        return;
    }
    if (!host_.bios_loaded() && game.kind.rfind("PS1", 0) != 0) {
        const std::string path = host_.pick_bios_file();
        if (path.empty() || !host_.load_bios(path)) {
            toast(path.empty() ? "PS2 games need a PS2 BIOS. Choose one with Change BIOS."
                               : host_.status_message());
            return;
        }
        settings_.bios_path = path;
        save_settings();
    }
    if (host_.boot_disc(game.path) && host_.session_active()) {
        last_boot_disc_ = game.path;
        edge_light_.reset();
        go(Screen::Starting);
    } else {
        toast(host_.status_message());
    }
}

void Frontend::update_in_game() {
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false) ||
        ImGui::IsKeyPressed(ImGuiKey_GamepadBack, false)) {
        host_.pause_session();
        home_sel_ = 0;
        home_sel_anim_ = 0.0f;
        play_open_sound();
        go(Screen::Home, true);
        return;
    }
    if (!host_.session_active()) {
        toast(host_.status_message());
        go(Screen::Home, true);
    }
}

void Frontend::draw_in_game(ImDrawList* draw, const ImVec2& pos, const ImVec2& size) {
    const float a = game_alpha_;
    draw->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(0, 0, 0, static_cast<int>(255 * a)));

    // The console always outputs 4:3, whatever raster size is scanned out.
    constexpr float aspect = 4.0f / 3.0f;
    ImVec2 image = size;
    if (image.x / image.y > aspect) image.x = image.y * aspect;
    else image.y = image.x / aspect;
    image.x = std::floor(image.x + 0.5f);
    image.y = std::floor(image.y + 0.5f);
    const ImVec2 p0(std::floor(pos.x + (size.x - image.x) * 0.5f),
                    std::floor(pos.y + (size.y - image.y) * 0.5f));

    // Light from the picture's edges spills onto the wall around it, as in
    // VibeStation 1.
    edge_light_.draw(draw, pos, size, p0, image, a);

    const GameView view = host_.game_view();
    if (view.texture != 0 && view.width > 0 && view.height > 0) {
        draw->AddImage(static_cast<ImTextureID>(view.texture), p0,
                       ImVec2(p0.x + image.x, p0.y + image.y), ImVec2(0, 0), ImVec2(1, 1),
                       IM_COL32(255, 255, 255, static_cast<int>(255 * a)));
    }

    if (screen_ == Screen::InGame && a > 0.9f) {
        if (show_perf_) draw_perf_overlay(draw, p0, image);
        draw_toolbar(p0, image, view.ps1);
    } else {
        set_turbo_held(false);
    }

    // Brief readout after entering the game.
    const float osd = a * (1.0f - smoothstep(3.0f, 3.8f, static_cast<float>(now_ - game_entered_t0_)));
    if (osd > 0.01f && screen_ == Screen::InGame && !show_perf_) {
        char line[96];
        if (host_.frames_per_second() > 0.0) {
            std::snprintf(line, sizeof(line), "%.1f FPS  ·  %.0f%%  ·  Esc  Menu",
                          host_.frames_per_second(), host_.speed_percent());
        } else {
            std::snprintf(line, sizeof(line), "Esc  Menu");
        }
        text(draw, FontRole::Mono, 13.0f, ImVec2(pos.x + 14, pos.y + 12),
             with_alpha(color::kDim, osd), line);
    }
}

// ------------------------------------------------------------------ chrome

void Frontend::toast(std::string message) {
    if (message.empty()) return;
    toast_ = std::move(message);
    toast_t0_ = now_;
}

void Frontend::draw_toast(ImDrawList* draw, const Layout& layout) {
    const float age = static_cast<float>(now_ - toast_t0_);
    const float a = smoothstep(0.0f, 0.2f, age) * (1.0f - smoothstep(4.0f, 4.6f, age));
    if (a <= 0.01f || toast_.empty()) return;
    const float size = layout.px(13);
    const ImVec2 ts = text_size(FontRole::Mono, size, toast_.c_str());
    const ImVec2 c = layout.point(640, screen_ == Screen::InGame ? 770 : 708);
    const ImVec2 p0(c.x - ts.x * 0.5f - layout.px(14), c.y - layout.px(8));
    const ImVec2 p1(c.x + ts.x * 0.5f + layout.px(14), c.y + ts.y + layout.px(8));
    draw->AddRectFilled(p0, p1, IM_COL32(12, 14, 18, static_cast<int>(220 * a)), layout.px(2));
    draw->AddRect(p0, p1, IM_COL32(70, 80, 100, static_cast<int>(160 * a)), layout.px(2));
    text(draw, FontRole::Mono, size, ImVec2(c.x - ts.x * 0.5f, c.y),
         with_alpha(color::kText, a), toast_.c_str());
}

void Frontend::draw_hints(ImDrawList* draw, const Layout& layout, float alpha,
                          std::initializer_list<std::pair<char, const char*>> hints) {
    if (alpha <= 0.01f || hints.size() == 0) return;
    const float label_size = layout.px(18);
    const float r = layout.px(11);
    const float gap = layout.px(9);
    const float y = layout.point(0, 742).y;
    const float left = layout.point(300, 0).x;
    const float right = layout.point(980, 0).x;
    const std::size_t n = hints.size();

    std::size_t i = 0;
    for (const auto& [glyph, label] : hints) {
        const ImVec2 ts = text_size(FontRole::Light, label_size, label);
        const float width = r * 2 + gap + ts.x;
        float x = left;
        if (n == 1) x = (left + right - width) * 0.5f;
        else if (i == n - 1) x = right - width;
        else if (i > 0) x = left + (right - left) * (static_cast<float>(i) / (n - 1)) - width * 0.5f;

        const ImVec2 c(x + r, y);
        // Hints double as buttons for mouse users. Only the screen that is
        // fully shown takes the click, not one still fading out.
        const ImVec2 h0(x - layout.px(8), y - r - layout.px(6));
        const ImVec2 h1(x + width + layout.px(8), y + r + layout.px(6));
        if (alpha > 0.95f && ImGui::IsMouseHoveringRect(h0, h1, false)) {
            draw->AddRectFilled(h0, h1, IM_COL32(255, 255, 255, static_cast<int>(16 * alpha)), layout.px(4));
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) hint_clicked_ = glyph;
        }
        draw->AddCircleFilled(c, r, IM_COL32(38, 44, 56, static_cast<int>(255 * alpha)));
        draw->AddCircle(c, r, IM_COL32(93, 104, 128, static_cast<int>(255 * alpha)), 0, std::max(1.0f, layout.px(1)));
        const float s = r * 0.42f;
        const float th = std::max(1.0f, layout.px(1.6f));
        switch (glyph) {
        case 'x': {
            const ImU32 col = IM_COL32(158, 197, 255, static_cast<int>(255 * alpha));
            draw->AddLine(ImVec2(c.x - s, c.y - s), ImVec2(c.x + s, c.y + s), col, th);
            draw->AddLine(ImVec2(c.x - s, c.y + s), ImVec2(c.x + s, c.y - s), col, th);
            break;
        }
        case 'o':
            draw->AddCircle(c, s * 1.1f, IM_COL32(255, 156, 156, static_cast<int>(255 * alpha)), 0, th);
            break;
        case 't':
            draw->AddTriangle(ImVec2(c.x, c.y - s * 1.15f), ImVec2(c.x + s * 1.1f, c.y + s * 0.8f),
                              ImVec2(c.x - s * 1.1f, c.y + s * 0.8f),
                              IM_COL32(143, 240, 200, static_cast<int>(255 * alpha)), th);
            break;
        case 's':
            draw->AddRect(ImVec2(c.x - s, c.y - s), ImVec2(c.x + s, c.y + s),
                          IM_COL32(243, 166, 228, static_cast<int>(255 * alpha)), 0, 0, th);
            break;
        default: break;
        }
        text(draw, FontRole::Light, label_size, ImVec2(c.x + r + gap, y - ts.y * 0.5f),
             with_alpha(IM_COL32(184, 194, 208, 255), alpha), label);
        ++i;
    }
}

Frontend::Input Frontend::read_input() const {
    Input in{};
    const auto repeat = [](ImGuiKey key) { return ImGui::IsKeyPressed(key, true); };
    const auto once = [](ImGuiKey key) { return ImGui::IsKeyPressed(key, false); };
    in.up = repeat(ImGuiKey_UpArrow) || repeat(ImGuiKey_GamepadDpadUp) || repeat(ImGuiKey_GamepadLStickUp);
    in.down = repeat(ImGuiKey_DownArrow) || repeat(ImGuiKey_GamepadDpadDown) || repeat(ImGuiKey_GamepadLStickDown);
    in.left = repeat(ImGuiKey_LeftArrow) || repeat(ImGuiKey_GamepadDpadLeft) || repeat(ImGuiKey_GamepadLStickLeft);
    in.right = repeat(ImGuiKey_RightArrow) || repeat(ImGuiKey_GamepadDpadRight) || repeat(ImGuiKey_GamepadLStickRight);
    in.accept = once(ImGuiKey_Enter) || once(ImGuiKey_KeypadEnter) || once(ImGuiKey_Space) ||
                once(ImGuiKey_X) || once(ImGuiKey_GamepadFaceDown);
    in.back = once(ImGuiKey_Escape) || once(ImGuiKey_Backspace) || once(ImGuiKey_C) ||
              once(ImGuiKey_GamepadFaceRight);
    in.triangle = once(ImGuiKey_T) || once(ImGuiKey_GamepadFaceUp);
    in.square = once(ImGuiKey_D) || once(ImGuiKey_GamepadFaceLeft);
    const ImGuiIO& io = ImGui::GetIO();
    in.wheel = io.MouseWheel;
    in.clicked = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
    in.mouse = io.MousePos;
    return in;
}

// ------------------------------------------------------------------ settings

void Frontend::load_settings() {
    std::ifstream file(kSettingsFile);
    std::string line;
    while (std::getline(file, line)) {
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = line.substr(0, eq);
        const std::string value = line.substr(eq + 1);
        if (key == "rom_dir") settings_.rom_dir = value;
        else if (key == "bios_path") settings_.bios_path = value;
        else if (key == "startup_video") settings_.startup_video = value != "0";
        else if (key == "sounds") settings_.sounds = value != "0";
        else if (key == "ambience") settings_.ambience = value != "0";
    }
}

void Frontend::apply_saved_settings() {
    std::ifstream file(kSettingsFile);
    std::string line;
    while (std::getline(file, line)) {
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = line.substr(0, eq);
        const std::string value = line.substr(eq + 1);
        if (key == "ee_core") {
            host_.set_ee_core(value == "dynarec" ? EeCore::Dynarec
                              : value == "jit"   ? EeCore::Jit
                                                 : EeCore::Interpreter);
        } else if (key == "gpu_gs") {
            if (host_.gpu_gs_available()) host_.set_gpu_gs_enabled(value == "1");
        } else if (key == "limit_speed") {
            host_.set_limit_speed(value != "0");
        } else if (key == "display_filter") {
            host_.set_display_filter(value == "0" ? 0 : 1);
        } else if (key == "lag_stutter") {
            host_.set_lag_stutter(value != "0");
        }
    }
    std::error_code ec;
    if (!host_.bios_loaded() && !settings_.bios_path.empty() &&
        std::filesystem::exists(settings_.bios_path, ec)) {
        host_.load_bios(settings_.bios_path);
    }
}

void Frontend::save_settings() const {
    std::ofstream file(kSettingsFile, std::ios::trunc);
    if (!file) return;
    file << "rom_dir=" << settings_.rom_dir << '\n'
         << "bios_path=" << settings_.bios_path << '\n'
         << "startup_video=" << (settings_.startup_video ? 1 : 0) << '\n'
         << "sounds=" << (settings_.sounds ? 1 : 0) << '\n'
         << "ambience=" << (settings_.ambience ? 1 : 0) << '\n'
         << "ee_core=" << ee_core_key(host_.ee_core()) << '\n'
         << "gpu_gs=" << (host_.gpu_gs_enabled() ? 1 : 0) << '\n'
         << "limit_speed=" << (host_.limit_speed() ? 1 : 0) << '\n'
         << "display_filter=" << host_.display_filter() << '\n'
         << "lag_stutter=" << (host_.lag_stutter() ? 1 : 0) << '\n';
}

} // namespace ps2::ui::vs2
