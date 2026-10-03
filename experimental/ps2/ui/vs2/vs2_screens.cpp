#include "ui/vs2/vs2_frontend.h"

#include "core/cdvd/disc_image.h"

#include <algorithm>
#include <array>
#include <cctype>

namespace ps2::ui::vs2 {

namespace {

// ------------------------------------------------------------------ helpers

std::string to_upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// "cdrom0:\SLUS_206.66;1" -> "SLUS-20666"
std::string serial_from_boot_path(const std::string& boot) {
    std::size_t start = boot.find_last_of("\\/:");
    std::string name = boot.substr(start == std::string::npos ? 0 : start + 1);
    if (const auto semi = name.find(';'); semi != std::string::npos) name.resize(semi);
    std::string out;
    for (char c : name) {
        if (c == '.') continue;
        out.push_back(c == '_' ? '-' : static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }
    return out;
}

const char* disc_kind(DiscType type) {
    switch (type) {
    case DiscType::Ps1Cd: return "PS1 CD";
    case DiscType::Ps2Cd: return "PS2 CD";
    case DiscType::Ps2Dvd: return "PS2 DVD";
    case DiscType::DvdVideo: return "DVD Video";
    default: return "Disc";
    }
}

// ROMVER "0160AC20011004" -> "v1.60 · USA · 2001-10-04"
std::string describe_romver(const std::string& romver) {
    if (romver.size() < 14 ||
        !std::all_of(romver.begin(), romver.begin() + 4,
                     [](char c) { return std::isdigit(static_cast<unsigned char>(c)) != 0; })) {
        return romver.empty() ? "Loaded" : romver;
    }
    std::string out = "v" + std::to_string(std::stoi(romver.substr(0, 2))) + "." + romver.substr(2, 2);
    switch (romver[4]) {
    case 'J': out += " · Japan"; break;
    case 'A': out += " · USA"; break;
    case 'E': out += " · Europe"; break;
    case 'H': out += " · Asia"; break;
    case 'C': out += " · China"; break;
    default: break;
    }
    out += " · " + romver.substr(6, 4) + "-" + romver.substr(10, 2) + "-" + romver.substr(12, 2);
    return out;
}

const char* ee_core_name(EeCore core) {
    switch (core) {
    case EeCore::Jit: return "JIT (x64)";
    case EeCore::Dynarec: return "Dynarec";
    default: return "Interpreter";
    }
}

enum ConfigRow {
    kEeCore, kGsRenderer, kSpeedLimit, kDisplayFilter, kLagStutter,
    kStartupVideo, kMenuSounds, kAmbience, kDeveloperView, kConfigRows
};

} // namespace

void Frontend::draw_title(ImDrawList* draw, const Layout& layout, float alpha,
                          const char* title, const std::string& crumb, ImU32 col) {
    const ImVec2 p = layout.point(90, 58);
    for (int i = 0; i < 8; ++i) {
        const float a = static_cast<float>(i) * 0.785398f;
        text(draw, FontRole::Light, layout.px(30),
             ImVec2(p.x + std::cos(a) * layout.px(2), p.y + std::sin(a) * layout.px(2)),
             with_alpha(col, alpha * 0.07f), title);
    }
    text(draw, FontRole::Light, layout.px(30), p, with_alpha(col, alpha), title);
    if (!crumb.empty()) {
        text(draw, FontRole::Mono, layout.px(11), layout.point(92, 102),
             with_alpha(color::kDim, alpha), crumb.c_str());
    }
}

// ------------------------------------------------------------------ browser

void Frontend::stop_scan() {
    stop_scan_.store(true);
    if (scan_thread_.joinable()) scan_thread_.join();
    stop_scan_.store(false);
}

void Frontend::rescan_games() {
    stop_scan();
    {
        std::lock_guard lock(games_mutex_);
        games_.clear();
    }
    browser_sel_ = 0;
    browser_sel_anim_ = 0.0f;
    if (settings_.rom_dir.empty()) return;

    scanning_.store(true);
    scan_thread_ = std::thread([this, root = settings_.rom_dir] {
        namespace fs = std::filesystem;
        std::vector<fs::path> candidates;
        std::error_code ec;
        fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec);
        for (; !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
            if (stop_scan_.load()) break;
            if (it.depth() > 2) { it.disable_recursion_pending(); continue; }
            if (!it->is_regular_file(ec)) continue;
            const std::string ext = to_upper(it->path().extension().string());
            if (ext == ".ISO" || ext == ".CUE" || ext == ".BIN" || ext == ".IMG") {
                candidates.push_back(it->path());
            }
        }
        // A .bin next to a .cue is the cue's track: list the cue only.
        std::vector<fs::path> files;
        for (const auto& path : candidates) {
            if (to_upper(path.extension().string()) == ".BIN") {
                const bool has_cue = std::any_of(candidates.begin(), candidates.end(), [&](const fs::path& p) {
                    return p.parent_path() == path.parent_path() &&
                           to_upper(p.extension().string()) == ".CUE";
                });
                if (has_cue) continue;
            }
            files.push_back(path);
        }

        for (const auto& path : files) {
            if (stop_scan_.load()) break;
            Game game;
            game.path = path.string();
            game.title = path.stem().string();
            DiscImage disc;
            std::string error;
            if (disc.open(game.path, error)) {
                game.kind = disc_kind(disc.type());
                game.serial = serial_from_boot_path(disc.boot_path());
            } else {
                game.kind = "Unreadable";
            }
            std::lock_guard lock(games_mutex_);
            games_.push_back(std::move(game));
            std::sort(games_.begin(), games_.end(),
                      [](const Game& a, const Game& b) { return to_upper(a.title) < to_upper(b.title); });
        }
        scanning_.store(false);
    });
}

void Frontend::update_browser(const Input& in, const Layout& layout) {
    std::vector<Game> games;
    {
        std::lock_guard lock(games_mutex_);
        games = games_;
    }
    const int count = static_cast<int>(games.size());
    const int before = browser_sel_;
    if (count > 0) {
        if (in.left || in.wheel > 0.0f) browser_sel_ = std::max(0, browser_sel_ - 1);
        if (in.right || in.wheel < 0.0f) browser_sel_ = std::min(count - 1, browser_sel_ + 1);
        browser_sel_ = std::clamp(browser_sel_, 0, count - 1);
        if (in.clicked) {
            for (int i = 0; i < count; ++i) {
                const float k = static_cast<float>(i) - browser_sel_anim_;
                if (std::fabs(k) > 3.5f) continue;
                const float x = 565.0f + k * 190.0f;
                if (ImGui::IsMouseHoveringRect(layout.point(x, 270), layout.point(x + 150, 460), false)) {
                    if (i == browser_sel_) {
                        boot_game(games[static_cast<std::size_t>(i)]);
                        return;
                    }
                    browser_sel_ = i;
                }
            }
        }
    }
    if (browser_sel_ != before) play_highlight_sound();

    if (in.accept && count > 0) {
        boot_game(games[static_cast<std::size_t>(browser_sel_)]);
    } else if (in.square) {
        const std::string dir = host_.pick_folder("Choose your game folder");
        if (!dir.empty()) {
            settings_.rom_dir = dir;
            save_settings();
            rescan_games();
            play_select_sound();
        }
    } else if (in.back) {
        go(Screen::Home);
    }
}

void Frontend::draw_browser(ImDrawList* draw, const Layout& layout, float alpha) {
    browser_sel_anim_ = approach(browser_sel_anim_, static_cast<float>(browser_sel_), 11.0f, dt_);
    std::vector<Game> games;
    {
        std::lock_guard lock(games_mutex_);
        games = games_;
    }
    const std::string crumb = settings_.rom_dir.empty()
        ? std::string("NO GAME FOLDER YET")
        : "GAME FOLDER · " + to_upper(settings_.rom_dir);
    draw_title(draw, layout, alpha, "Browser", crumb, color::kSelect);

    if (games.empty()) {
        const char* line = settings_.rom_dir.empty() ? "Choose the folder that holds your disc images."
                         : scanning_.load()          ? "Looking for games..."
                                                     : "No disc images found in this folder.";
        const char* sub = "ISO, BIN/CUE and IMG images are supported.";
        const ImVec2 ls = text_size(FontRole::Light, layout.px(22), line);
        const ImVec2 ss = text_size(FontRole::Regular, layout.px(13), sub);
        text(draw, FontRole::Light, layout.px(22),
             ImVec2(layout.point(640, 0).x - ls.x * 0.5f, layout.point(0, 360).y),
             with_alpha(color::kText, alpha), line);
        text(draw, FontRole::Regular, layout.px(13),
             ImVec2(layout.point(640, 0).x - ss.x * 0.5f, layout.point(0, 398).y),
             with_alpha(color::kSub, alpha), sub);
        draw_hints(draw, layout, alpha, {{'s', "Set Directory"}, {'o', "Back"}});
        return;
    }

    // Disc tiles on a shelf; the selected one sits forward.
    for (int pass = 0; pass < 2; ++pass) { // far tiles first, selected last
        for (int i = 0; i < static_cast<int>(games.size()); ++i) {
            const float k = static_cast<float>(i) - browser_sel_anim_;
            const bool selected = std::fabs(k) < 0.5f;
            if ((pass == 1) != selected || std::fabs(k) > 3.6f) continue;
            const float a = alpha * std::max(0.0f, 1.0f - std::fabs(k) * 0.28f);
            const float scale = selected ? 1.06f : 1.0f - std::min(1.0f, std::fabs(k)) * 0.06f;
            const float w = 150.0f * scale, h = 190.0f * scale;
            const float cx = 565.0f + 75.0f + k * 190.0f;
            const float top = 270.0f + (190.0f - h) * 0.5f;
            const ImVec2 p0 = layout.point(cx - w * 0.5f, top);
            const ImVec2 p1 = layout.point(cx + w * 0.5f, top + h);

            draw->AddRectFilledMultiColor(p0, p1,
                IM_COL32(150, 190, 255, static_cast<int>(52 * a)), IM_COL32(60, 95, 170, static_cast<int>(30 * a)),
                IM_COL32(40, 70, 140, static_cast<int>(18 * a)), IM_COL32(150, 190, 255, static_cast<int>(26 * a)));
            draw->AddRect(p0, p1, selected ? with_alpha(color::kSelect, a) : IM_COL32(170, 205, 255, static_cast<int>(90 * a)),
                          layout.px(4), 0, std::max(1.0f, layout.px(selected ? 1.4f : 1.0f)));
            if (selected) {
                for (int g = 1; g <= 3; ++g) {
                    const float o = layout.px(static_cast<float>(g) * 3.0f);
                    draw->AddRect(ImVec2(p0.x - o, p0.y - o), ImVec2(p1.x + o, p1.y + o),
                                  with_alpha(color::kSelect, a * 0.12f / g), layout.px(5), 0, layout.px(2));
                }
            }
            // Disc face
            const ImVec2 c = layout.point(cx, top + h * 0.5f);
            const ImU32 line = IM_COL32(200, 225, 255, static_cast<int>(180 * a));
            draw->AddCircle(c, layout.px(33 * scale), line, 48, std::max(1.0f, layout.px(1.2f)));
            draw->AddCircle(c, layout.px(22 * scale), IM_COL32(200, 225, 255, static_cast<int>(45 * a)), 48, layout.px(10 * scale));
            draw->AddCircle(c, layout.px(9 * scale), IM_COL32(200, 225, 255, static_cast<int>(150 * a)), 32, std::max(1.0f, layout.px(1)));
            // Reflection on the floor
            const ImVec2 r0 = layout.point(cx - w * 0.5f, top + h + 8);
            const ImVec2 r1 = layout.point(cx + w * 0.5f, top + h + 70);
            draw->AddRectFilledMultiColor(r0, r1,
                IM_COL32(150, 190, 255, static_cast<int>(20 * a)), IM_COL32(150, 190, 255, static_cast<int>(20 * a)),
                IM_COL32(150, 190, 255, 0), IM_COL32(150, 190, 255, 0));
        }
    }

    const Game& game = games[static_cast<std::size_t>(std::clamp(browser_sel_, 0, static_cast<int>(games.size()) - 1))];
    const ImVec2 ts = text_size(FontRole::Light, layout.px(26), game.title.c_str());
    text(draw, FontRole::Light, layout.px(26),
         ImVec2(layout.point(640, 0).x - ts.x * 0.5f, layout.point(0, 560).y),
         with_alpha(IM_COL32(230, 237, 246, 255), alpha), game.title.c_str());
    const std::string meta = game.serial.empty() ? game.kind : game.serial + " · " + game.kind;
    const ImVec2 ms = text_size(FontRole::Mono, layout.px(12), meta.c_str());
    text(draw, FontRole::Mono, layout.px(12),
         ImVec2(layout.point(640, 0).x - ms.x * 0.5f, layout.point(0, 600).y),
         with_alpha(color::kDim, alpha), meta.c_str());

    draw_hints(draw, layout, alpha, {{'x', "Boot"}, {'s', "Set Directory"}, {'o', "Back"}});
}

// ------------------------------------------------------------------ config

void Frontend::change_config(int row, int direction) {
    switch (row) {
    case kEeCore: {
        const int next = (static_cast<int>(host_.ee_core()) + direction + 3) % 3;
        host_.set_ee_core(static_cast<EeCore>(next));
        break;
    }
    case kGsRenderer:
        if (!host_.gpu_gs_available()) return;
        host_.set_gpu_gs_enabled(!host_.gpu_gs_enabled());
        break;
    case kSpeedLimit: host_.set_limit_speed(!host_.limit_speed()); break;
    case kDisplayFilter: host_.set_display_filter(host_.display_filter() == 0 ? 1 : 0); break;
    case kLagStutter: host_.set_lag_stutter(!host_.lag_stutter()); break;
    case kStartupVideo: settings_.startup_video = !settings_.startup_video; break;
    case kAmbience:
        settings_.ambience = !settings_.ambience;
        break;
    case kMenuSounds:
        settings_.sounds = !settings_.sounds;
        set_sounds_enabled(settings_.sounds);
        break;
    case kDeveloperView:
        play_select_sound();
        host_.open_developer_view();
        return;
    default: return;
    }
    play_select_sound();
    save_settings();
}

void Frontend::update_config(const Input& in, const Layout& layout) {
    const int before = config_sel_;
    if (in.up) config_sel_ = (config_sel_ + kConfigRows - 1) % kConfigRows;
    if (in.down) config_sel_ = (config_sel_ + 1) % kConfigRows;
    if (in.clicked) {
        for (int i = 0; i < kConfigRows; ++i) {
            const float y = 172.0f + i * 46.0f;
            if (ImGui::IsMouseHoveringRect(layout.point(230, y), layout.point(1050, y + 44), false)) {
                if (i == config_sel_) change_config(i, 1);
                else config_sel_ = i;
            }
        }
    }
    if (config_sel_ != before) play_highlight_sound();
    if (in.left && config_sel_ != kDeveloperView) change_config(config_sel_, -1);
    else if (in.right && config_sel_ != kDeveloperView) change_config(config_sel_, 1);
    else if (in.accept) change_config(config_sel_, 1);
    else if (in.back) go(Screen::Home);
}

void Frontend::draw_config(ImDrawList* draw, const Layout& layout, float alpha) {
    draw_title(draw, layout, alpha, "System Configuration", "LEFT / RIGHT CHANGES A VALUE", color::kSelect);

    const bool boot_sound_found = !find_asset("vs2-boot.wav").empty();
    struct Row {
        const char* label;
        std::string value;
        const char* note;
    };
    const std::array<Row, kConfigRows> rows = {{
        {"EE core", ee_core_name(host_.ee_core()),
         "Dynarec is the fastest. The interpreter is the most accurate and the easiest to debug."},
        {"GS renderer", host_.gpu_gs_enabled() ? "GPU compute" : "CPU (parallel)",
         host_.gpu_gs_available() ? "The parallel CPU rasterizer is usually faster than the GPU path today."
                                  : "GPU compute needs OpenGL 4.3, which this system does not offer."},
        {"Speed limit", host_.limit_speed() ? "On (100%)" : "Off", "Off runs as fast as your computer allows."},
        {"Display filter", host_.display_filter() == 0 ? "Nearest" : "Bilinear",
         "How the picture is scaled to the window."},
        {"Lag stutter effect", host_.lag_stutter() ? "On" : "Off",
         "Loops the last 400 ms of sound while emulation falls behind, instead of crackling."},
        {"Startup animation", settings_.startup_video ? "On" : "Skip",
         boot_sound_found ? "Plays the VibeStation 2 boot animation before the menu."
                          : "Plays the boot animation silently: vs2-boot.wav is missing from resources/vs2."},
        {"Menu sounds", settings_.sounds ? "On" : "Off", "Sounds for moving, opening and confirming."},
        {"Ambience", settings_.ambience ? "On" : "Off", "A soft background hum and distant static while you are in the menus."},
        {"Developer view", "Open", "The PS2 lab interface with the EE, IOP and GS debuggers. F12 switches back."},
    }};

    for (int i = 0; i < kConfigRows; ++i) {
        const float y = 172.0f + i * 46.0f;
        const bool sel = i == config_sel_;
        const ImVec2 p0 = layout.point(230, y);
        const ImVec2 p1 = layout.point(1050, y + 44);
        if (sel) {
            draw->AddRectFilledMultiColor(p0, p1,
                with_alpha(color::kSelect, 0.20f * alpha), with_alpha(color::kSelect, 0.04f * alpha),
                with_alpha(color::kSelect, 0.04f * alpha), with_alpha(color::kSelect, 0.20f * alpha));
            draw->AddRectFilled(p0, ImVec2(p0.x + std::max(1.0f, layout.px(2)), p1.y), with_alpha(color::kSelect, alpha));
        }
        const float label_size = layout.px(19);
        const ImVec2 ls = text_size(FontRole::Light, label_size, rows[i].label);
        const float mid = (p0.y + p1.y) * 0.5f;
        text(draw, FontRole::Light, label_size, ImVec2(layout.point(252, 0).x, mid - ls.y * 0.5f),
             with_alpha(sel ? IM_COL32(255, 255, 255, 255) : IM_COL32(170, 180, 195, 255), alpha), rows[i].label);

        std::string value = rows[i].value;
        if (sel && i != kDeveloperView) value = "<  " + value + "  >";
        const float value_size = layout.px(15);
        const ImVec2 vs = text_size(FontRole::Mono, value_size, value.c_str());
        text(draw, FontRole::Mono, value_size, ImVec2(layout.point(1028, 0).x - vs.x, mid - vs.y * 0.5f),
             with_alpha(sel ? color::kSelect : IM_COL32(215, 223, 234, 255), alpha), value.c_str());
    }
    text(draw, FontRole::Regular, layout.px(13), layout.point(252, 600),
         with_alpha(IM_COL32(117, 131, 154, 255), alpha), rows[static_cast<std::size_t>(config_sel_)].note);

    draw_hints(draw, layout, alpha,
               {{'x', config_sel_ == kDeveloperView ? "Open" : "Change"}, {'o', "Back"}});
}

// ------------------------------------------------------------------ version

void Frontend::draw_version(ImDrawList* draw, const Layout& layout, float alpha) {
    draw_title(draw, layout, alpha, "Version Information", "", color::kSelect);

    std::size_t game_count = 0;
    {
        std::lock_guard lock(games_mutex_);
        game_count = games_.size();
    }
    const std::string bios = host_.bios_loaded() ? describe_romver(host_.bios_romver()) : "Not loaded";
    const std::string ee = std::string("R5900 · 294.912 MHz · ") + ee_core_name(host_.ee_core());
    const std::array<std::pair<const char*, std::string>, 8> rows = {{
        {"Emulator", "VibeStation 2 (PS2 lab)"},
        {"Version", kVersionLabel},
        {"BIOS", bios},
        {"EE core", ee},
        {"IOP core", "R3000A · 36.864 MHz"},
        {"GS raster", host_.gpu_gs_enabled() ? "GPU compute" : "CPU · parallel row bands"},
        {"Game folder", settings_.rom_dir.empty() ? "Not set" : settings_.rom_dir},
        {"Games found", std::to_string(game_count)},
    }};

    const ImVec2 p0 = layout.point(330, 190);
    const ImVec2 p1 = layout.point(950, 190 + 60 + rows.size() * 30.0f);
    draw->AddRectFilled(p0, p1, IM_COL32(14, 14, 16, static_cast<int>(180 * alpha)));
    draw->AddRect(p0, p1, IM_COL32(200, 205, 215, static_cast<int>(46 * alpha)));
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const float y = 220.0f + i * 30.0f;
        text(draw, FontRole::Mono, layout.px(14), layout.point(368, y),
             with_alpha(IM_COL32(117, 131, 154, 255), alpha), rows[i].first);
        text(draw, FontRole::Mono, layout.px(14), layout.point(578, y),
             with_alpha(IM_COL32(199, 208, 221, 255), alpha), rows[i].second.c_str());
    }
    draw_hints(draw, layout, alpha, {{'o', "Back"}});
}

// ------------------------------------------------------------------ reaper

void Frontend::draw_reaper(ImDrawList* draw, const Layout& layout, float alpha) {
    draw_title(draw, layout, alpha, "Grim Reaper", "CORRUPT EE RAM · VRAM · SPU2 · BIOS", color::kReaper);
    const char* line = "The PS2 Grim Reaper is coming.";
    const char* sub = "Corruption tools for EE RAM, GS VRAM, SPU2 and BIOS are not built yet.";
    const ImVec2 ls = text_size(FontRole::Light, layout.px(22), line);
    const ImVec2 ss = text_size(FontRole::Regular, layout.px(13), sub);
    text(draw, FontRole::Light, layout.px(22), ImVec2(layout.point(640, 0).x - ls.x * 0.5f, layout.point(0, 380).y),
         with_alpha(color::kText, alpha), line);
    text(draw, FontRole::Regular, layout.px(13), ImVec2(layout.point(640, 0).x - ss.x * 0.5f, layout.point(0, 418).y),
         with_alpha(color::kSub, alpha), sub);
    draw_hints(draw, layout, alpha, {{'o', "Back"}});
}

// ------------------------------------------------------------------ exit

void Frontend::update_exit(const Input& in, const Layout& layout) {
    const int before = exit_sel_;
    if (in.left || in.right) exit_sel_ ^= 1;
    if (in.clicked) {
        for (int i = 0; i < 2; ++i) {
            const float x = i == 0 ? 560.0f : 680.0f;
            if (ImGui::IsMouseHoveringRect(layout.point(x - 10, 420), layout.point(x + 70, 460), false)) {
                exit_sel_ = i;
                if (i == 1) {
                    play_select_sound();
                    host_.request_quit();
                } else {
                    go(Screen::Home);
                }
                return;
            }
        }
    }
    if (exit_sel_ != before) play_highlight_sound();
    if (in.accept) {
        if (exit_sel_ == 1) {
            play_select_sound();
            host_.request_quit();
        } else {
            go(Screen::Home);
        }
    } else if (in.back) {
        go(Screen::Home);
    }
}

void Frontend::draw_exit(ImDrawList* draw, const Layout& layout, float alpha) {
    const char* line = "Exit VibeStation 2?";
    const ImVec2 ls = text_size(FontRole::Light, layout.px(26), line);
    text(draw, FontRole::Light, layout.px(26), ImVec2(layout.point(640, 0).x - ls.x * 0.5f, layout.point(0, 350).y),
         with_alpha(color::kText, alpha), line);
    const std::array<const char*, 2> options = {"No", "Yes"};
    for (int i = 0; i < 2; ++i) {
        const bool sel = i == exit_sel_;
        const float x = i == 0 ? 560.0f : 680.0f;
        text(draw, FontRole::Light, layout.px(24), layout.point(x, 422),
             with_alpha(sel ? color::kSelect : color::kIdle, alpha), options[static_cast<std::size_t>(i)]);
    }
    draw_hints(draw, layout, alpha, {{'x', "Select"}, {'o', "Back"}});
}

} // namespace ps2::ui::vs2
