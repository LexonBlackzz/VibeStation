#include "ui/app.h"
#include <SDL_opengl.h>
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

void App::update_vram_debug_texture() {
    if (!system_) {
        return;
    }

    if (vram_debug_texture_ == 0) {
        glGenTextures(1, &vram_debug_texture_);
        glBindTexture(GL_TEXTURE_2D, vram_debug_texture_);
        // VRAM debug is usually displayed downscaled; linear filtering avoids
        // harsh temporal aliasing that can look like z-fighting/flicker.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    std::vector<u16> vram_snapshot;
    const u16* vram = nullptr;
    const size_t pixel_count =
        static_cast<size_t>(psx::VRAM_WIDTH) * psx::VRAM_HEIGHT;
    if (emu_runner_.consume_latest_vram_snapshot(vram_snapshot) &&
        vram_snapshot.size() == pixel_count) {
        vram = vram_snapshot.data();
    }
    else if (!emu_runner_.is_running()) {
        vram = system_->gpu().vram();
    }
    else {
        return;
    }

    std::vector<u32> rgba(psx::VRAM_WIDTH * psx::VRAM_HEIGHT);
    for (size_t i = 0; i < rgba.size(); ++i) {
        const u16 p = vram[i];
        const u8 r = static_cast<u8>((p & 0x1F) << 3);
        const u8 g = static_cast<u8>(((p >> 5) & 0x1F) << 3);
        const u8 b = static_cast<u8>(((p >> 10) & 0x1F) << 3);
        rgba[i] = r | (static_cast<u32>(g) << 8) | (static_cast<u32>(b) << 16) |
            0xFF000000;
    }

    glBindTexture(GL_TEXTURE_2D, vram_debug_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, psx::VRAM_WIDTH, psx::VRAM_HEIGHT, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
}

void App::panel_vram() {
    ImGui::SetNextWindowSize(ImVec2(980, 620), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("VRAM Debug", &show_vram_)) {
        ImGui::Text("Raw VRAM 1024x512 (15-bit)");
        ImGui::Separator();



        ImVec2 avail = ImGui::GetContentRegionAvail();
        const float tex_w = static_cast<float>(psx::VRAM_WIDTH);
        const float tex_h = static_cast<float>(psx::VRAM_HEIGHT);
        const float scale = std::min(avail.x / tex_w, avail.y / tex_h);
        ImVec2 draw_size(std::floor(tex_w * scale), std::floor(tex_h * scale));
        draw_size.x = std::max(1.0f, draw_size.x);
        draw_size.y = std::max(1.0f, draw_size.y);

        if (vram_debug_texture_ != 0) {
            ImGui::Image((ImTextureID)(intptr_t)vram_debug_texture_, draw_size,
                ImVec2(0, 0), ImVec2(1, 1));
        }
    }
    ImGui::End();
}
