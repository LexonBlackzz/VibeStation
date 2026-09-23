#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#include <stb_image.h>

#include "ui/app.h"
#include "vibestation_version.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <string>

namespace {
constexpr float kDesignWidth = 1280.0f;
constexpr float kDesignHeight = 800.0f;

GLuint g_background_texture = 0;
int g_background_width = 0;
int g_background_height = 0;
bool g_background_load_attempted = false;

ImU32 rgba(int r, int g, int b, int a = 255) {
    return IM_COL32(r, g, b, a);
}

std::filesystem::path find_background_path() {
    std::array<std::filesystem::path, 4> candidates{};

    std::error_code ec;
    const std::filesystem::path cwd = std::filesystem::current_path(ec);
    if (!ec) {
        candidates[0] = cwd / "resources" / "ui" / "definitive" / "background.jpg";
        candidates[1] = cwd / ".." / "resources" / "ui" / "definitive" / "background.jpg";
    }

    if (char* base = SDL_GetBasePath()) {
        const std::filesystem::path base_path(base);
        SDL_free(base);
        candidates[2] =
            base_path / "resources" / "ui" / "definitive" / "background.jpg";
        candidates[3] =
            base_path / ".." / "resources" / "ui" / "definitive" / "background.jpg";
    }

    for (const auto& candidate : candidates) {
        if (!candidate.empty() && std::filesystem::exists(candidate, ec) && !ec) {
            return candidate;
        }
        ec.clear();
    }
    return {};
}

bool ensure_background_texture_loaded() {
    if (g_background_texture != 0) {
        return true;
    }
    if (g_background_load_attempted) {
        return false;
    }
    g_background_load_attempted = true;

    const std::filesystem::path path = find_background_path();
    if (path.empty()) {
        return false;
    }

    int channels = 0;
    unsigned char* pixels = stbi_load(
        path.string().c_str(), &g_background_width, &g_background_height, &channels, 4);
    if (pixels == nullptr || g_background_width <= 0 || g_background_height <= 0) {
        if (pixels != nullptr) {
            stbi_image_free(pixels);
        }
        g_background_width = 0;
        g_background_height = 0;
        return false;
    }

    glGenTextures(1, &g_background_texture);
    glBindTexture(GL_TEXTURE_2D, g_background_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA, g_background_width, g_background_height, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(pixels);
    return g_background_texture != 0;
}

struct Layout {
    ImVec2 origin{};
    float scale = 1.0f;

    ImVec2 point(float x, float y) const {
        return ImVec2(origin.x + x * scale, origin.y + y * scale);
    }
    ImVec2 size(float x, float y) const {
        return ImVec2(x * scale, y * scale);
    }
    float px(float value) const {
        return value * scale;
    }
};

Layout make_layout(const ImVec2& window_pos, const ImVec2& window_size) {
    Layout layout{};
    layout.scale = std::max(
        0.55f, std::min(window_size.x / kDesignWidth, window_size.y / kDesignHeight));
    const ImVec2 design_size(kDesignWidth * layout.scale, kDesignHeight * layout.scale);
    layout.origin = ImVec2(
        window_pos.x + (window_size.x - design_size.x) * 0.5f,
        window_pos.y + (window_size.y - design_size.y) * 0.5f);
    return layout;
}

void draw_background(ImDrawList* draw, const ImVec2& pos, const ImVec2& size) {
    draw->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y),
        rgba(7, 9, 12, 255));

    if (!ensure_background_texture_loaded()) {
        draw->AddRectFilledMultiColor(
            pos, ImVec2(pos.x + size.x, pos.y + size.y),
            rgba(8, 10, 14, 255), rgba(17, 19, 23, 255),
            rgba(10, 12, 15, 255), rgba(5, 7, 10, 255));
        return;
    }

    float u0 = 0.0f;
    float v0 = 0.0f;
    float u1 = 1.0f;
    float v1 = 1.0f;
    const float image_aspect =
        static_cast<float>(g_background_width) / static_cast<float>(g_background_height);
    const float canvas_aspect = size.x / std::max(1.0f, size.y);

    if (canvas_aspect > image_aspect) {
        const float visible_v = image_aspect / canvas_aspect;
        v0 = (1.0f - visible_v) * 0.5f;
        v1 = v0 + visible_v;
    }
    else {
        const float visible_u = canvas_aspect / image_aspect;
        u0 = (1.0f - visible_u) * 0.5f;
        u1 = u0 + visible_u;
    }

    draw->AddImage(
        reinterpret_cast<ImTextureID>(static_cast<intptr_t>(g_background_texture)),
        pos, ImVec2(pos.x + size.x, pos.y + size.y),
        ImVec2(u0, v0), ImVec2(u1, v1));
}

void add_text(ImDrawList* draw, const Layout& layout, float x, float y,
    float size, ImU32 color, const char* text) {
    draw->AddText(
        ImGui::GetFont(), layout.px(size), layout.point(x, y), color, text);
}

void add_text_right(ImDrawList* draw, const Layout& layout, float right_x, float y,
    float size, ImU32 color, const char* text) {
    const float font_size = layout.px(size);
    const ImVec2 text_size = ImGui::GetFont()->CalcTextSizeA(
        font_size, FLT_MAX, 0.0f, text);
    const ImVec2 p = layout.point(right_x, y);
    draw->AddText(ImGui::GetFont(), font_size,
        ImVec2(p.x - text_size.x, p.y), color, text);
}

enum class MenuIcon {
    Play,
    Folder,
    Chip,
    Settings,
    Exit
};

void draw_icon(ImDrawList* draw, const Layout& layout, MenuIcon icon,
    float x, float y, ImU32 color) {
    const ImVec2 p = layout.point(x, y);
    const float s = layout.scale;

    switch (icon) {
    case MenuIcon::Play:
        draw->AddTriangleFilled(
            ImVec2(p.x, p.y),
            ImVec2(p.x, p.y + 22.0f * s),
            ImVec2(p.x + 18.0f * s, p.y + 11.0f * s),
            color);
        break;
    case MenuIcon::Folder:
        draw->AddLine(
            ImVec2(p.x, p.y + 5.0f * s),
            ImVec2(p.x + 8.0f * s, p.y + 5.0f * s), color, 2.0f * s);
        draw->AddLine(
            ImVec2(p.x + 8.0f * s, p.y + 5.0f * s),
            ImVec2(p.x + 12.0f * s, p.y + 9.0f * s), color, 2.0f * s);
        draw->AddRect(
            ImVec2(p.x, p.y + 8.0f * s),
            ImVec2(p.x + 24.0f * s, p.y + 24.0f * s),
            color, 1.5f * s, 0, 2.0f * s);
        break;
    case MenuIcon::Chip:
        draw->AddRect(
            ImVec2(p.x + 4.0f * s, p.y + 3.0f * s),
            ImVec2(p.x + 22.0f * s, p.y + 25.0f * s),
            color, 1.0f * s, 0, 2.0f * s);
        for (int i = 0; i < 4; ++i) {
            const float py = p.y + (6.0f + i * 5.0f) * s;
            draw->AddLine(ImVec2(p.x, py), ImVec2(p.x + 4.0f * s, py),
                color, 1.5f * s);
            draw->AddLine(ImVec2(p.x + 22.0f * s, py),
                ImVec2(p.x + 26.0f * s, py), color, 1.5f * s);
        }
        break;
    case MenuIcon::Settings:
        draw->AddCircle(
            ImVec2(p.x + 13.0f * s, p.y + 14.0f * s),
            9.0f * s, color, 12, 2.0f * s);
        draw->AddCircle(
            ImVec2(p.x + 13.0f * s, p.y + 14.0f * s),
            3.0f * s, color, 12, 2.0f * s);
        for (int i = 0; i < 8; ++i) {
            const float a = static_cast<float>(i) * 3.14159265f / 4.0f;
            const ImVec2 a0(
                p.x + 10.0f * s * std::cos(a) + 13.0f * s,
                p.y + 10.0f * s * std::sin(a) + 14.0f * s);
            const ImVec2 a1(
                p.x + 14.0f * s * std::cos(a) + 13.0f * s,
                p.y + 14.0f * s * std::sin(a) + 14.0f * s);
            draw->AddLine(a0, a1, color, 2.0f * s);
        }
        break;
    case MenuIcon::Exit:
        draw->AddRect(
            ImVec2(p.x + 8.0f * s, p.y + 2.0f * s),
            ImVec2(p.x + 25.0f * s, p.y + 26.0f * s),
            color, 0.0f, 0, 1.8f * s);
        draw->AddLine(
            ImVec2(p.x, p.y + 14.0f * s),
            ImVec2(p.x + 16.0f * s, p.y + 14.0f * s),
            color, 2.0f * s);
        draw->AddLine(
            ImVec2(p.x + 11.0f * s, p.y + 9.0f * s),
            ImVec2(p.x + 16.0f * s, p.y + 14.0f * s),
            color, 2.0f * s);
        draw->AddLine(
            ImVec2(p.x + 11.0f * s, p.y + 19.0f * s),
            ImVec2(p.x + 16.0f * s, p.y + 14.0f * s),
            color, 2.0f * s);
        break;
    }
}

bool menu_button(const Layout& layout, ImDrawList* draw, int index,
    MenuIcon icon, const char* title, const char* subtitle, int& selected_index) {
    constexpr float kX = 36.0f;
    constexpr float kY = 218.0f;
    constexpr float kWidth = 396.0f;
    constexpr float kHeight = 62.0f;
    constexpr float kGap = 8.0f;

    const float y = kY + index * (kHeight + kGap);
    const ImVec2 p = layout.point(kX, y);
    const ImVec2 size = layout.size(kWidth, kHeight);

    ImGui::SetCursorScreenPos(p);
    ImGui::PushID(index);
    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(0, 0, 0, 0));
    const bool pressed = ImGui::Button("##definitive_menu", size);
    ImGui::PopStyleColor(3);

    const bool hovered = ImGui::IsItemHovered();
    const bool focused = ImGui::IsItemFocused();
    if (hovered || focused) {
        selected_index = index;
    }

    const bool selected = selected_index == index;
    if (selected) {
        draw->AddRectFilled(
            p, ImVec2(p.x + size.x, p.y + size.y),
            rgba(12, 17, 23, hovered ? 128 : 92));
        draw->AddRect(
            p, ImVec2(p.x + size.x, p.y + size.y),
            rgba(208, 226, 244, 245), 0.0f, 0, layout.px(1.4f));
        draw->AddRect(
            ImVec2(p.x + layout.px(2.0f), p.y + layout.px(2.0f)),
            ImVec2(p.x + size.x - layout.px(2.0f),
                p.y + size.y - layout.px(2.0f)),
            rgba(111, 153, 197, 120), 0.0f, 0, layout.px(0.8f));
    }

    const ImU32 main_color = selected
        ? rgba(240, 244, 248, 255)
        : rgba(206, 208, 211, 228);
    const ImU32 sub_color = selected
        ? rgba(181, 188, 197, 235)
        : rgba(150, 154, 161, 210);

    draw_icon(draw, layout, icon, kX + 24.0f, y + 18.0f, main_color);
    add_text(draw, layout, kX + 72.0f, y + 13.0f, 18.0f, main_color, title);
    add_text(draw, layout, kX + 72.0f, y + 38.0f, 10.5f, sub_color, subtitle);

    ImGui::PopID();
    return pressed;
}

bool small_button(const Layout& layout, const char* id, const char* label,
    float x, float y, float w, float h, bool enabled = true) {
    ImGui::SetCursorScreenPos(layout.point(x, y));
    if (!enabled) {
        ImGui::BeginDisabled();
    }
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, rgba(12, 15, 19, 205));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, rgba(24, 31, 39, 225));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, rgba(32, 42, 52, 235));
    ImGui::PushStyleColor(ImGuiCol_Border, rgba(128, 145, 163, 190));
    ImGui::PushStyleColor(ImGuiCol_Text, rgba(226, 230, 235, 245));
    ImGui::PushID(id);
    const bool pressed = ImGui::Button(label, layout.size(w, h));
    ImGui::PopID();
    ImGui::PopStyleColor(5);
    ImGui::PopStyleVar(2);
    if (!enabled) {
        ImGui::EndDisabled();
    }
    return pressed;
}

void draw_panel(ImDrawList* draw, const Layout& layout,
    float x, float y, float w, float h) {
    const ImVec2 p0 = layout.point(x, y);
    const ImVec2 p1 = layout.point(x + w, y + h);
    draw->AddRectFilled(p0, p1, rgba(5, 8, 11, 198));
    draw->AddRect(p0, p1, rgba(102, 116, 130, 205), 0.0f, 0, layout.px(1.0f));
}

void draw_folder_badge(ImDrawList* draw, const Layout& layout, float x, float y) {
    const ImVec2 p = layout.point(x, y);
    const float s = layout.scale;
    const ImU32 c = rgba(226, 230, 235, 240);
    draw->AddLine(ImVec2(p.x, p.y + 4.0f * s),
        ImVec2(p.x + 8.0f * s, p.y + 4.0f * s), c, 1.6f * s);
    draw->AddLine(ImVec2(p.x + 8.0f * s, p.y + 4.0f * s),
        ImVec2(p.x + 12.0f * s, p.y + 8.0f * s), c, 1.6f * s);
    draw->AddRect(ImVec2(p.x, p.y + 7.0f * s),
        ImVec2(p.x + 22.0f * s, p.y + 20.0f * s), c, 1.0f * s, 0, 1.6f * s);
}

void draw_info_badge(ImDrawList* draw, const Layout& layout, float x, float y) {
    const ImVec2 p = layout.point(x, y);
    const float s = layout.scale;
    const ImU32 c = rgba(226, 230, 235, 240);
    draw->AddCircle(ImVec2(p.x + 9.0f * s, p.y + 10.0f * s),
        8.0f * s, c, 16, 1.5f * s);
    draw->AddCircleFilled(ImVec2(p.x + 9.0f * s, p.y + 6.0f * s),
        1.0f * s, c);
    draw->AddLine(ImVec2(p.x + 9.0f * s, p.y + 9.0f * s),
        ImVec2(p.x + 9.0f * s, p.y + 15.0f * s), c, 1.5f * s);
}
}

void App::release_definitive_ui_assets() {
    if (g_background_texture != 0) {
        glDeleteTextures(1, &g_background_texture);
        g_background_texture = 0;
    }
    g_background_width = 0;
    g_background_height = 0;
    g_background_load_attempted = false;
}

void App::panel_definitive_home() {
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 window_pos = ImGui::GetWindowPos();
    const ImVec2 window_size = ImGui::GetWindowSize();
    const Layout layout = make_layout(window_pos, window_size);

    draw_background(draw, window_pos, window_size);

    const ImVec2 left0 = window_pos;
    const ImVec2 left1(window_pos.x + window_size.x * 0.58f, window_pos.y + window_size.y);
    draw->AddRectFilledMultiColor(left0, left1,
        rgba(0, 2, 5, 210), rgba(0, 2, 5, 38),
        rgba(0, 2, 5, 218), rgba(0, 2, 5, 68));

    const ImVec2 bottom0(window_pos.x, window_pos.y + window_size.y * 0.64f);
    const ImVec2 bottom1(window_pos.x + window_size.x, window_pos.y + window_size.y);
    draw->AddRectFilledMultiColor(bottom0, bottom1,
        rgba(1, 3, 6, 10), rgba(1, 3, 6, 10),
        rgba(1, 3, 6, 206), rgba(1, 3, 6, 206));

    add_text(draw, layout, 48.0f, 36.0f, 54.0f,
        rgba(223, 225, 228, 248), "VibeStation");

    constexpr std::array<ImU32, 4> accent_colors = {
        IM_COL32(194, 44, 56, 255),
        IM_COL32(52, 128, 125, 255),
        IM_COL32(177, 145, 72, 255),
        IM_COL32(52, 93, 157, 255),
    };
    for (int i = 0; i < 4; ++i) {
        const ImVec2 p0 = layout.point(50.0f + i * 32.0f, 116.0f);
        const ImVec2 p1 = layout.point(76.0f + i * 32.0f, 126.0f);
        draw->AddRectFilled(p0, p1, accent_colors[static_cast<size_t>(i)]);
    }
    add_text(draw, layout, 185.0f, 112.0f, 14.0f,
        rgba(190, 193, 199, 238), "PS1 EMULATOR");

    add_text(draw, layout, 49.0f, 149.0f, 11.5f,
        rgba(184, 187, 193, 220), "GAMES");
    add_text(draw, layout, 49.0f, 166.0f, 11.5f,
        rgba(184, 187, 193, 220), "MEMORIES");
    add_text(draw, layout, 49.0f, 183.0f, 11.5f,
        rgba(184, 187, 193, 220), "STILL RUN");

    add_text_right(draw, layout, 1235.0f, 36.0f, 9.5f,
        rgba(152, 158, 166, 215), VIBESTATION_VERSION_STRING);
    add_text_right(draw, layout, 1235.0f, 55.0f, 10.0f,
        rgba(173, 177, 184, 220), "A SMALLER PAST");
    add_text_right(draw, layout, 1235.0f, 70.0f, 10.0f,
        rgba(173, 177, 184, 220), "STILL PLAYS");
    const ImVec2 dash0 = layout.point(1217.0f, 93.0f);
    const ImVec2 dash1 = layout.point(1235.0f, 93.0f);
    draw->AddLine(dash0, dash1, rgba(180, 184, 190, 190), layout.px(1.0f));

    static int selected_menu = 0;
    const bool start_pressed = menu_button(layout, draw, 0, MenuIcon::Play,
        "Start Emulation", "Load BIOS and start playing", selected_menu);
    const bool load_game_pressed = menu_button(layout, draw, 1, MenuIcon::Folder,
        "Load Game", "Choose a game from your library", selected_menu);
    const bool change_bios_pressed = menu_button(layout, draw, 2, MenuIcon::Chip,
        "Change BIOS", "Manage BIOS files", selected_menu);
    const bool settings_pressed = menu_button(layout, draw, 3, MenuIcon::Settings,
        "Settings", "Configure emulator options", selected_menu);
    const bool exit_pressed = menu_button(layout, draw, 4, MenuIcon::Exit,
        "Exit", "Close VibeStation", selected_menu);

    const auto choose_bios = [this]() -> bool {
        std::string path = open_file_dialog(
            "BIOS Files (*.bin)\0*.bin\0All Files\0*.*\0", "Select PS1 BIOS");
        if (path.empty()) {
            return false;
        }

        emu_runner_.pause_and_wait_idle();
        disable_ram_reaper_mode();
        disable_gpu_reaper_mode();
        disable_sound_reaper_mode();
        if (!system_->load_bios(path)) {
            status_message_ = "Failed to load BIOS!";
            return false;
        }

        bios_path_ = path;
        save_persistent_config();
        has_started_emulation_ = false;
        set_grim_reaper_mode(false);
        status_message_ = "BIOS loaded: " + system_->bios().get_info();
        return true;
    };

    if (start_pressed) {
        if (!system_->bios_loaded() && !choose_bios()) {
            // File picker cancelled or BIOS failed to load.
        }
        else if (!game_bin_path_.empty() || system_->disc_loaded()) {
            boot_disc_from_ui();
        }
        else {
            start_bios_from_ui();
        }
    }

    if (load_game_pressed) {
        std::string path = open_file_dialog(
            "PS1 Games (*.bin;*.cue)\0*.bin;*.cue\0All Files\0*.*\0",
            "Select PS1 Game");
        if (!path.empty()) {
            std::string bin;
            std::string cue;
            std::string error;
            if (!resolve_disc_paths(path, bin, cue, error)) {
                status_message_ = error;
            }
            else {
                load_disc_from_ui(bin, cue);
            }
        }
    }

    if (change_bios_pressed) {
        choose_bios();
    }
    if (settings_pressed) {
        show_settings_ = true;
    }
    if (exit_pressed) {
        SDL_Event quit_event{};
        quit_event.type = SDL_QUIT;
        SDL_PushEvent(&quit_event);
    }

    if (game_library_dirty_ ||
        (rom_directory_valid_ &&
            (SDL_GetTicks() - game_library_last_scan_ms_ > 15000u))) {
        refresh_game_library();
    }

    constexpr float panel_y = 585.0f;
    draw_panel(draw, layout, 32.0f, panel_y, 808.0f, 183.0f);
    draw_panel(draw, layout, 854.0f, panel_y, 394.0f, 183.0f);

    draw_folder_badge(draw, layout, 53.0f, panel_y + 17.0f);
    add_text(draw, layout, 86.0f, panel_y + 15.0f, 13.5f,
        rgba(226, 230, 235, 245), "Game Library");
    draw->AddLine(layout.point(46.0f, panel_y + 44.0f),
        layout.point(826.0f, panel_y + 44.0f),
        rgba(105, 116, 128, 165), layout.px(1.0f));

    const std::string rom_label = rom_directory_valid_
        ? "ROM Directory: " + rom_directory_
        : "ROM Directory: not set";
    add_text(draw, layout, 53.0f, panel_y + 55.0f, 9.8f,
        rgba(163, 168, 176, 225), rom_label.c_str());

    if (!rom_directory_valid_) {
        add_text(draw, layout, 53.0f, panel_y + 87.0f, 10.2f,
            rgba(224, 74, 74, 245), "No ROM directory configured.");
        add_text(draw, layout, 53.0f, panel_y + 111.0f, 9.7f,
            rgba(187, 191, 198, 225),
            "Set a ROM directory to scan and list games here.");
    }
    else if (game_library_.empty()) {
        add_text(draw, layout, 53.0f, panel_y + 89.0f, 10.0f,
            rgba(207, 180, 108, 235), "No playable disc images found.");
    }
    else {
        const size_t visible_count = std::min<size_t>(3, game_library_.size());
        for (size_t i = 0; i < visible_count; ++i) {
            const float row_y = panel_y + 79.0f + static_cast<float>(i) * 22.0f;
            ImGui::SetCursorScreenPos(layout.point(53.0f, row_y));
            ImGui::PushID(static_cast<int>(i));
            ImGui::PushStyleColor(ImGuiCol_Selectable, IM_COL32(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_SelectableHovered, rgba(45, 55, 67, 155));
            ImGui::PushStyleColor(ImGuiCol_SelectableActive, rgba(55, 68, 82, 175));
            const bool chosen = ImGui::Selectable(
                game_library_[i].title.c_str(), false, 0, layout.size(510.0f, 19.0f));
            ImGui::PopStyleColor(3);
            ImGui::PopID();
            if (chosen) {
                load_disc_from_ui(game_library_[i].bin_path, game_library_[i].cue_path);
            }
        }
        if (game_library_.size() > visible_count) {
            const std::string more = "+" +
                std::to_string(game_library_.size() - visible_count) + " more";
            add_text(draw, layout, 584.0f, panel_y + 124.0f, 9.0f,
                rgba(145, 151, 160, 215), more.c_str());
        }
    }

    if (small_button(layout, "set_rom_dir", "Set Directory",
        53.0f, panel_y + 145.0f, 130.0f, 25.0f)) {
        const std::string selected = open_folder_dialog("Select ROM Directory");
        if (!selected.empty()) {
            rom_directory_ = selected;
            game_library_dirty_ = true;
            save_persistent_config();
            refresh_game_library();
            status_message_ = "ROM directory set: " + rom_directory_;
        }
    }
    if (small_button(layout, "refresh_rom_dir", "Refresh",
        196.0f, panel_y + 145.0f, 90.0f, 25.0f, rom_directory_valid_)) {
        game_library_dirty_ = true;
        refresh_game_library();
    }

    draw_info_badge(draw, layout, 875.0f, panel_y + 17.0f);
    add_text(draw, layout, 905.0f, panel_y + 15.0f, 13.5f,
        rgba(226, 230, 235, 245), "System Info");
    draw->AddLine(layout.point(868.0f, panel_y + 44.0f),
        layout.point(1232.0f, panel_y + 44.0f),
        rgba(105, 116, 128, 165), layout.px(1.0f));

    const ImU32 label_color = rgba(165, 170, 177, 225);
    const ImU32 value_color = rgba(211, 214, 219, 235);
    add_text(draw, layout, 875.0f, panel_y + 58.0f, 9.5f,
        label_color, "Emulator:");
    add_text(draw, layout, 995.0f, panel_y + 58.0f, 9.5f,
        value_color, "VibeStation");
    add_text(draw, layout, 875.0f, panel_y + 77.0f, 9.5f,
        label_color, "Version:");
    add_text(draw, layout, 995.0f, panel_y + 77.0f, 9.5f,
        value_color, VIBESTATION_VERSION_STRING);
    add_text(draw, layout, 875.0f, panel_y + 96.0f, 9.5f,
        label_color, "BIOS:");
    add_text(draw, layout, 995.0f, panel_y + 96.0f, 9.5f,
        value_color, system_->bios_loaded() ? "Loaded" : "Not loaded");
    add_text(draw, layout, 875.0f, panel_y + 115.0f, 9.5f,
        label_color, "ROM Directory:");
    add_text(draw, layout, 995.0f, panel_y + 115.0f, 9.5f,
        value_color, rom_directory_valid_ ? "Set" : "Not set");
    add_text(draw, layout, 875.0f, panel_y + 134.0f, 9.5f,
        label_color, "Games Found:");
    const std::string games_found = std::to_string(game_library_.size());
    add_text(draw, layout, 995.0f, panel_y + 134.0f, 9.5f,
        value_color, games_found.c_str());

    draw->AddLine(layout.point(875.0f, panel_y + 155.0f),
        layout.point(1232.0f, panel_y + 155.0f),
        rgba(91, 101, 111, 145), layout.px(1.0f));
    add_text(draw, layout, 875.0f, panel_y + 164.0f, 8.8f,
        rgba(145, 150, 158, 205), "Same console. Different vibes.");
}
