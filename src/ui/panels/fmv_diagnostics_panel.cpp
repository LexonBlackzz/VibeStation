#include "ui/app.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cstddef>

namespace {
    struct RectSourceRegion {
        u16 x = 0;
        u16 y = 0;
        u16 w = 0;
        u16 h = 0;
        bool valid = false;
    };

    RectSourceRegion compute_rect_source_region(
        u16 texpage, u8 depth, u8 u, u8 v, u16 w, u16 h) {
        RectSourceRegion region{};
        if (w == 0 || h == 0) {
            return region;
        }
        const u16 tex_base_x = static_cast<u16>((texpage & 0xFu) * 64u);
        const u16 tex_base_y = static_cast<u16>(((texpage >> 4) & 0x1u) * 256u);
        u16 src_x = tex_base_x;
        u16 src_w = 0;
        switch (depth & 0x3u) {
        case 0: {
            src_x = static_cast<u16>(tex_base_x + (u >> 2));
            const u16 last_u = static_cast<u16>(u + w - 1u);
            src_w = static_cast<u16>((last_u >> 2) - (u >> 2) + 1u);
            break;
        }
        case 1: {
            src_x = static_cast<u16>(tex_base_x + (u >> 1));
            const u16 last_u = static_cast<u16>(u + w - 1u);
            src_w = static_cast<u16>((last_u >> 1) - (u >> 1) + 1u);
            break;
        }
        case 2:
            src_x = static_cast<u16>(tex_base_x + u);
            src_w = w;
            break;
        default:
            return region;
        }
        region.x = static_cast<u16>(src_x & (psx::VRAM_WIDTH - 1u));
        region.y = static_cast<u16>((tex_base_y + v) & (psx::VRAM_HEIGHT - 1u));
        region.w = src_w;
        region.h = h;
        region.valid = true;
        return region;
    }

    bool rects_overlap(u16 ax, u16 ay, u16 aw, u16 ah,
        u16 bx, u16 by, u16 bw, u16 bh) {
        if (aw == 0 || ah == 0 || bw == 0 || bh == 0) {
            return false;
        }
        const u32 ax2 = static_cast<u32>(ax) + aw;
        const u32 ay2 = static_cast<u32>(ay) + ah;
        const u32 bx2 = static_cast<u32>(bx) + bw;
        const u32 by2 = static_cast<u32>(by) + bh;
        return ax < bx2 && bx < ax2 && ay < by2 && by < ay2;
    }

    int find_recent_upload_overlap(const System::MdecUploadProbe& probe,
        const RectSourceRegion& region) {
        if (!region.valid) {
            return -1;
        }
        const u32 count = std::min<u32>(probe.gpu_hist_count,
            static_cast<u32>(System::MdecUploadProbe::kUploadHistory));
        for (u32 i = 0; i < count; ++i) {
            const u32 hist_index =
                (probe.gpu_hist_count - 1u - i) %
                static_cast<u32>(System::MdecUploadProbe::kUploadHistory);
            if (rects_overlap(region.x, region.y, region.w, region.h,
                    probe.gpu_hist_x[hist_index], probe.gpu_hist_y[hist_index],
                    probe.gpu_hist_w[hist_index], probe.gpu_hist_h[hist_index])) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    int find_recent_copy_overlap(const System::MdecUploadProbe& probe,
        const RectSourceRegion& region) {
        if (!region.valid) {
            return -1;
        }
        const u32 count = std::min<u32>(probe.gpu_copy_sample_count,
            static_cast<u32>(System::MdecUploadProbe::kSampleWords));
        for (u32 i = 0; i < count; ++i) {
            const u32 hist_index = count - 1u - i;
            if (rects_overlap(region.x, region.y, region.w, region.h,
                    probe.gpu_copy_dst_x[hist_index], probe.gpu_copy_dst_y[hist_index],
                    probe.gpu_copy_w[hist_index], probe.gpu_copy_h[hist_index])) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    RectSourceRegion compute_poly_source_region(const GpuCommandDebugInfo& gcmd,
        u32 poly_index) {
        RectSourceRegion region{};
        const u8 vertex_count = gcmd.poly_vertex_count[poly_index];
        if (vertex_count == 0 || !gcmd.poly_textured[poly_index]) {
            return region;
        }
        u8 min_u = 255;
        u8 min_v = 255;
        u8 max_u = 0;
        u8 max_v = 0;
        for (u8 i = 0; i < vertex_count && i < 4; ++i) {
            min_u = std::min(min_u, gcmd.poly_u[poly_index][i]);
            min_v = std::min(min_v, gcmd.poly_v[poly_index][i]);
            max_u = std::max(max_u, gcmd.poly_u[poly_index][i]);
            max_v = std::max(max_v, gcmd.poly_v[poly_index][i]);
        }
        return compute_rect_source_region(
            gcmd.poly_texpage[poly_index],
            gcmd.poly_depth[poly_index],
            min_u,
            min_v,
            static_cast<u16>(max_u - min_u + 1u),
            static_cast<u16>(max_v - min_v + 1u));
    }

    u32 compute_poly_bbox_area(const GpuCommandDebugInfo& gcmd, u32 poly_index) {
        const u8 vertex_count = gcmd.poly_vertex_count[poly_index];
        if (vertex_count < 3) {
            return 0;
        }
        s16 min_x = gcmd.poly_x[poly_index][0];
        s16 max_x = gcmd.poly_x[poly_index][0];
        s16 min_y = gcmd.poly_y[poly_index][0];
        s16 max_y = gcmd.poly_y[poly_index][0];
        for (u8 i = 1; i < vertex_count && i < 4; ++i) {
            min_x = std::min(min_x, gcmd.poly_x[poly_index][i]);
            max_x = std::max(max_x, gcmd.poly_x[poly_index][i]);
            min_y = std::min(min_y, gcmd.poly_y[poly_index][i]);
            max_y = std::max(max_y, gcmd.poly_y[poly_index][i]);
        }
        const int width = std::max(0, static_cast<int>(max_x) - static_cast<int>(min_x));
        const int height = std::max(0, static_cast<int>(max_y) - static_cast<int>(min_y));
        return static_cast<u32>(width * height);
    }

    u32 compute_poly_visible_bbox_area(const GpuCommandDebugInfo& gcmd, u32 poly_index,
        int view_x, int view_y, int view_w, int view_h) {
        const u8 vertex_count = gcmd.poly_vertex_count[poly_index];
        if (vertex_count < 3 || view_w <= 0 || view_h <= 0) {
            return 0;
        }
        s16 min_x = gcmd.poly_x[poly_index][0];
        s16 max_x = gcmd.poly_x[poly_index][0];
        s16 min_y = gcmd.poly_y[poly_index][0];
        s16 max_y = gcmd.poly_y[poly_index][0];
        for (u8 i = 1; i < vertex_count && i < 4; ++i) {
            min_x = std::min(min_x, gcmd.poly_x[poly_index][i]);
            max_x = std::max(max_x, gcmd.poly_x[poly_index][i]);
            min_y = std::min(min_y, gcmd.poly_y[poly_index][i]);
            max_y = std::max(max_y, gcmd.poly_y[poly_index][i]);
        }
        const int clipped_min_x = std::max(static_cast<int>(min_x), view_x);
        const int clipped_max_x = std::min(static_cast<int>(max_x), view_x + view_w);
        const int clipped_min_y = std::max(static_cast<int>(min_y), view_y);
        const int clipped_max_y = std::min(static_cast<int>(max_y), view_y + view_h);
        const int width = std::max(0, clipped_max_x - clipped_min_x);
        const int height = std::max(0, clipped_max_y - clipped_min_y);
        return static_cast<u32>(width * height);
    }
}

void App::draw_fmv_diagnostics_content() {
    ImGui::Text("MDEC Debug Isolation");
    ImGui::Checkbox("Disable DMA1 Reorder (MDEC out)",
        &g_mdec_debug_disable_dma1_reorder);
    ImGui::Checkbox("Disable Chroma (Cb/Cr=0)", &g_mdec_debug_disable_chroma);
    ImGui::Checkbox("Disable Luma (Y=0)", &g_mdec_debug_disable_luma);
    ImGui::Checkbox("Force Solid MDEC Output", &g_mdec_debug_force_solid_output);
    ImGui::Checkbox("Swap MDEC Input Halfwords", &g_mdec_debug_swap_input_halfwords);
    if (ImGui::Checkbox("Enable Macroblock Compare (slow)",
            &g_mdec_debug_compare_macroblocks) &&
        !g_mdec_debug_compare_macroblocks && system_) {
        system_->reset_mdec_debug_compare();
    }
    if (ImGui::Checkbox("Enable Upload Probe (slow)",
            &g_mdec_debug_upload_probe) &&
        !g_mdec_debug_upload_probe && system_) {
        system_->reset_mdec_upload_probe();
    }

    bool y1 = (g_mdec_debug_color_block_mask & 0x1u) != 0u;
    bool y2 = (g_mdec_debug_color_block_mask & 0x2u) != 0u;
    bool y3 = (g_mdec_debug_color_block_mask & 0x4u) != 0u;
    bool y4 = (g_mdec_debug_color_block_mask & 0x8u) != 0u;
    if (ImGui::Checkbox("Y1 (top-left 8x8)", &y1)) {
        g_mdec_debug_color_block_mask =
            static_cast<u8>((g_mdec_debug_color_block_mask & ~0x1u) | (y1 ? 0x1u : 0x0u));
    }
    if (ImGui::Checkbox("Y2 (top-right 8x8)", &y2)) {
        g_mdec_debug_color_block_mask =
            static_cast<u8>((g_mdec_debug_color_block_mask & ~0x2u) | (y2 ? 0x2u : 0x0u));
    }
    if (ImGui::Checkbox("Y3 (bottom-left 8x8)", &y3)) {
        g_mdec_debug_color_block_mask =
            static_cast<u8>((g_mdec_debug_color_block_mask & ~0x4u) | (y3 ? 0x4u : 0x0u));
    }
    if (ImGui::Checkbox("Y4 (bottom-right 8x8)", &y4)) {
        g_mdec_debug_color_block_mask =
            static_cast<u8>((g_mdec_debug_color_block_mask & ~0x8u) | (y4 ? 0x8u : 0x0u));
    }
    if (ImGui::Button("Reset MDEC Isolation")) {
        g_mdec_debug_disable_dma1_reorder = false;
        g_mdec_debug_disable_chroma = false;
        g_mdec_debug_disable_luma = false;
        g_mdec_debug_force_solid_output = false;
        g_mdec_debug_swap_input_halfwords = false;
        g_mdec_debug_compare_macroblocks = false;
        g_mdec_debug_upload_probe = false;
        g_mdec_debug_color_block_mask = 0x0Fu;
        if (system_) {
            system_->reset_mdec_debug_compare();
            system_->reset_mdec_upload_probe();
        }
    }
    ImGui::TextDisabled("Use these toggles to localize FMV artifacts by stage.");

    if (!system_) {
        return;
    }

    ImGui::Separator();
    const auto& mstats = system_->mdec_debug_stats();
    const double blocks = static_cast<double>(mstats.blocks_decoded);
    const double dc_only_pct =
        (blocks > 0.0) ? (100.0 * static_cast<double>(mstats.dc_only_blocks) / blocks) : 0.0;
    const double q0_pct =
        (blocks > 0.0) ? (100.0 * static_cast<double>(mstats.qscale_zero_blocks) / blocks) : 0.0;
    const double avg_q =
        (blocks > 0.0) ? (static_cast<double>(mstats.qscale_sum) / blocks) : 0.0;
    const double avg_nonzero_coeff =
        (blocks > 0.0) ? (static_cast<double>(mstats.nonzero_coeff_count) / blocks) : 0.0;

    ImGui::Text("MDEC Decode Stats");
    ImGui::Text("Cmd: decode=%llu quant=%llu scale=%llu",
        static_cast<unsigned long long>(mstats.decode_commands),
        static_cast<unsigned long long>(mstats.set_quant_commands),
        static_cast<unsigned long long>(mstats.set_scale_commands));
    ImGui::Text("Quant: L0=%u C0=%u Lavg=%u Cavg=%u",
        mstats.quant_luma0, mstats.quant_chroma0,
        mstats.quant_luma_avg, mstats.quant_chroma_avg);
    ImGui::Text("Blocks=%llu  DC-only=%llu (%.1f%%)",
        static_cast<unsigned long long>(mstats.blocks_decoded),
        static_cast<unsigned long long>(mstats.dc_only_blocks), dc_only_pct);
    ImGui::Text("q=0=%llu (%.1f%%)  q_avg=%.2f q_max=%u",
        static_cast<unsigned long long>(mstats.qscale_zero_blocks), q0_pct,
        avg_q, mstats.qscale_max);
    ImGui::Text("Nonzero coeff/block=%.2f  EOB=%llu  Overflow=%llu",
        avg_nonzero_coeff,
        static_cast<unsigned long long>(mstats.eob_markers),
        static_cast<unsigned long long>(mstats.overflow_breaks));
    if (ImGui::CollapsingHeader("MDEC Command History")) {
        ImGui::Text("Recent Cmds");
        const u32 cmd_total = mstats.command_history_count;
        const u32 cmd_shown = std::min<u32>(
            cmd_total, static_cast<u32>(Mdec::DebugStats::kCommandHistory));
        for (u32 i = 0; i < cmd_shown; ++i) {
            const u32 hist_index =
                (cmd_total - cmd_shown + i) %
                static_cast<u32>(Mdec::DebugStats::kCommandHistory);
            ImGui::Text("  C[%u] id=%u word=0x%08X",
                i,
                static_cast<unsigned>(mstats.command_history_ids[hist_index]),
                mstats.command_history_words[hist_index]);
        }
        ImGui::Text("Recent Writes");
        const u32 write_total = mstats.write_history_count;
        const u32 write_shown = std::min<u32>(
            write_total, static_cast<u32>(Mdec::DebugStats::kWriteHistory));
        for (u32 i = 0; i < write_shown; ++i) {
            const u32 hist_index =
                (write_total - write_shown + i) %
                static_cast<u32>(Mdec::DebugStats::kWriteHistory);
            ImGui::Text("  W[%u] cmd=%u active=%u word=0x%08X",
                i,
                static_cast<unsigned>(mstats.write_history_expect_command[hist_index]),
                static_cast<unsigned>(mstats.write_history_active_command[hist_index]),
                mstats.write_history_words[hist_index]);
        }
        ImGui::Text("Recent Ctrl");
        const u32 ctrl_total = mstats.control_history_count;
        const u32 ctrl_shown = std::min<u32>(
            ctrl_total, static_cast<u32>(Mdec::DebugStats::kCommandHistory));
        for (u32 i = 0; i < ctrl_shown; ++i) {
            const u32 hist_index =
                (ctrl_total - ctrl_shown + i) %
                static_cast<u32>(Mdec::DebugStats::kCommandHistory);
            ImGui::Text("  CT[%u] 0x%08X", i, mstats.control_history_words[hist_index]);
        }
    }
    if (ImGui::Button("Reset MDEC Decode Stats")) {
        system_->reset_mdec_debug_stats();
    }

    ImGui::Separator();
    const auto& mcompare = system_->mdec_debug_compare();
    static const char* kMdecCompareStages[] = {
        "none", "coeff", "idct", "packed-output", "match" };
    static const char* kMdecCompareBlocks[] = {
        "Cr", "Cb", "Y1", "Y2", "Y3", "Y4" };
    ImGui::Text("MDEC Macroblock Compare");
    ImGui::Text("Seen=%s  Macroblocks=%llu  Stage=%s",
        mcompare.captured ? "yes" : "no",
        static_cast<unsigned long long>(mcompare.macroblocks_compared),
        kMdecCompareStages[static_cast<unsigned>(mcompare.stage)]);
    if (ImGui::CollapsingHeader("MDEC Macroblock Compare Details")) {
        if (mcompare.captured && mcompare.stage != Mdec::DebugCompare::Stage::Match &&
            mcompare.stage != Mdec::DebugCompare::Stage::None) {
            const char* block_name =
                (mcompare.mismatch_block < 6u) ? kMdecCompareBlocks[mcompare.mismatch_block] : "n/a";
            ImGui::Text("Mismatch: block=%s idx=%u cur=0x%08X ref=0x%08X",
                block_name, static_cast<unsigned>(mcompare.mismatch_index),
                static_cast<unsigned>(mcompare.current_value),
                static_cast<unsigned>(mcompare.reference_value));
        }
        if (mcompare.captured) {
            ImGui::Text("QScale: Cr=%u Cb=%u Y1=%u Y2=%u Y3=%u Y4=%u",
                static_cast<unsigned>(mcompare.qscales[0]),
                static_cast<unsigned>(mcompare.qscales[1]),
                static_cast<unsigned>(mcompare.qscales[2]),
                static_cast<unsigned>(mcompare.qscales[3]),
                static_cast<unsigned>(mcompare.qscales[4]),
                static_cast<unsigned>(mcompare.qscales[5]));
            for (u32 i = 0; i < std::min<u32>(
                     mcompare.input_halfword_count,
                     static_cast<u32>(Mdec::DebugCompare::kInputSampleHalfwords)); ++i) {
                ImGui::Text("  In[%u] = 0x%04X",
                    i, static_cast<unsigned>(mcompare.input_halfwords[i]));
            }
        }
    }
    if (ImGui::Button("Reset MDEC Compare")) {
        system_->reset_mdec_debug_compare();
    }

    ImGui::Separator();
    const auto& uprobes = system_->mdec_upload_probe();
    ImGui::Text("MDEC Upload Probe");
    ImGui::Text("DMA1: seen=%s base=0x%08X words=%u depth=%u block=%u",
        uprobes.dma1_seen ? "yes" : "no",
        uprobes.dma1_base_addr, uprobes.dma1_words,
        static_cast<unsigned>(uprobes.dma1_depth),
        static_cast<unsigned>(uprobes.dma1_first_block));
    ImGui::Text("GPU Upload: seen=%s data=%s xy=(%u,%u) wh=(%u,%u) words=%u/%u",
        uprobes.gpu_upload_seen ? "yes" : "no",
        uprobes.gpu_upload_data_seen ? "yes" : "no",
        static_cast<unsigned>(uprobes.gpu_x),
        static_cast<unsigned>(uprobes.gpu_y),
        static_cast<unsigned>(uprobes.gpu_w),
        static_cast<unsigned>(uprobes.gpu_h),
        uprobes.gpu_words_seen, uprobes.gpu_total_words);
    if (ImGui::CollapsingHeader("MDEC Upload Probe Details")) {
        ImGui::Text("GPU Upload History: count=%u", uprobes.gpu_upload_count);
        const u32 hist_count =
            std::min<u32>(uprobes.gpu_hist_count,
                          static_cast<u32>(System::MdecUploadProbe::kUploadHistory));
        for (u32 i = 0; i < hist_count; ++i) {
            const u32 hist_index =
                (uprobes.gpu_hist_count - hist_count + i) %
                static_cast<u32>(System::MdecUploadProbe::kUploadHistory);
            ImGui::Text("  UP[%u] (%u,%u) wh=(%u,%u) <- %s",
                i,
                static_cast<unsigned>(uprobes.gpu_hist_x[hist_index]),
                static_cast<unsigned>(uprobes.gpu_hist_y[hist_index]),
                static_cast<unsigned>(uprobes.gpu_hist_w[hist_index]),
                static_cast<unsigned>(uprobes.gpu_hist_h[hist_index]),
                uprobes.gpu_hist_from_dma[hist_index] ? "DMA2" : "CPU");
        }
        ImGui::Text("GPU Last Frame Uploads: count=%u valid=%s",
            uprobes.gpu_last_frame_upload_count,
            uprobes.gpu_last_frame_valid ? "yes" : "no");
    }
    if (ImGui::Button("Reset MDEC Upload Probe")) {
        system_->reset_mdec_upload_probe();
    }

    ImGui::Separator();
    const auto gdisp = system_->gpu_display_debug_info();
    const auto gcmd = system_->gpu_command_debug_info();
    ImGui::Text("GPU: %dx%d 24bit=%s interlace=%s  GP0 rect=%u",
        gdisp.width, gdisp.height,
        gdisp.is_24bit ? "yes" : "no",
        gdisp.interlaced ? "yes" : "no",
        gcmd.gp0_textured_rect_count);
    if (ImGui::CollapsingHeader("GPU Display Debug")) {
        ImGui::Text("Mode=%dx%d 24bit=%s interlace=%s div=%d",
            gdisp.mode_width, gdisp.mode_height,
            gdisp.is_24bit ? "yes" : "no",
            gdisp.interlaced ? "yes" : "no",
            gdisp.divisor);
        ImGui::Text("Regs: start=(%d,%d) x=(%d,%d) y=(%d,%d)",
            gdisp.x_start, gdisp.y_start,
            gdisp.x1, gdisp.x2, gdisp.y1, gdisp.y2);
        ImGui::Text("Derived: wh=(%d,%d) src=(%d,%d)",
            gdisp.width, gdisp.height,
            gdisp.src_width, gdisp.src_height);
        ImGui::Text("VRAM: xy=(%d,%d) wh=(%d,%d) skip_x=%d",
            gdisp.display_vram_left, gdisp.display_vram_top,
            gdisp.display_vram_width, gdisp.display_vram_height,
            gdisp.display_skip_x);
    }
    if (ImGui::CollapsingHeader("GPU Command Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("GP1: area=%u hrange=%u vrange=%u mode=%u",
            gcmd.gp1_display_area_count,
            gcmd.gp1_horizontal_range_count,
            gcmd.gp1_vertical_range_count,
            gcmd.gp1_display_mode_count);
        ImGui::Text("Last Area: raw=0x%06X xy=(%d,%d)",
            gcmd.gp1_display_area_raw & 0xFFFFFFu,
            gcmd.gp1_display_area_x, gcmd.gp1_display_area_y);
        ImGui::Text("Last Range: x=(%d,%d) y=(%d,%d) mode=0x%02X",
            gcmd.gp1_horizontal_range_x1,
            gcmd.gp1_horizontal_range_x2,
            gcmd.gp1_vertical_range_y1,
            gcmd.gp1_vertical_range_y2,
            gcmd.gp1_display_mode_raw & 0x7Fu);
        ImGui::Text("GP0 Textured: tri=%u quad=%u rect=%u",
            gcmd.gp0_textured_tri_count,
            gcmd.gp0_textured_quad_count,
            gcmd.gp0_textured_rect_count);
        if (!g_mdec_debug_upload_probe) {
            ImGui::TextDisabled(
                "Enable MDEC Upload Probe to capture recent GPU uploads/copies.");
        }
        else {
            const u32 upload_hist_count =
                std::min<u32>(uprobes.gpu_hist_count,
                    static_cast<u32>(System::MdecUploadProbe::kUploadHistory));
            ImGui::Text("Recent GPU Uploads: total=%u frame=%u",
                uprobes.gpu_upload_count,
                uprobes.gpu_last_frame_upload_count);
            for (u32 i = 0; i < std::min<u32>(upload_hist_count, 4u); ++i) {
                const u32 hist_index =
                    (uprobes.gpu_hist_count - 1u - i) %
                    static_cast<u32>(System::MdecUploadProbe::kUploadHistory);
                ImGui::Text(
                    "UP[%u]: xy=(%u,%u) wh=(%u,%u) <- %s",
                    i,
                    static_cast<unsigned>(uprobes.gpu_hist_x[hist_index]),
                    static_cast<unsigned>(uprobes.gpu_hist_y[hist_index]),
                    static_cast<unsigned>(uprobes.gpu_hist_w[hist_index]),
                    static_cast<unsigned>(uprobes.gpu_hist_h[hist_index]),
                    uprobes.gpu_hist_from_dma[hist_index] ? "DMA2" : "CPU");
            }
            ImGui::Text("Recent VRAM Copies: total=%u",
                uprobes.gpu_copy_count);
            for (u32 i = 0; i < std::min<u32>(uprobes.gpu_copy_sample_count, 4u); ++i) {
                const u32 hist_index = uprobes.gpu_copy_sample_count - 1u - i;
                ImGui::Text(
                    "CP[%u]: src=(%u,%u) dst=(%u,%u) wh=(%u,%u)",
                    i,
                    static_cast<unsigned>(uprobes.gpu_copy_src_x[hist_index]),
                    static_cast<unsigned>(uprobes.gpu_copy_src_y[hist_index]),
                    static_cast<unsigned>(uprobes.gpu_copy_dst_x[hist_index]),
                    static_cast<unsigned>(uprobes.gpu_copy_dst_y[hist_index]),
                    static_cast<unsigned>(uprobes.gpu_copy_w[hist_index]),
                    static_cast<unsigned>(uprobes.gpu_copy_h[hist_index]));
            }
        }
        if (gcmd.gp0_textured_rect_count > 0) {
            const u32 last_rect_index =
                (gcmd.gp0_textured_rect_count - 1u) %
                static_cast<u32>(GpuCommandDebugInfo::kRecentRects);
            const RectSourceRegion last_rect_src = compute_rect_source_region(
                gcmd.rect_texpage[last_rect_index],
                gcmd.rect_depth[last_rect_index],
                gcmd.rect_u[last_rect_index],
                gcmd.rect_v[last_rect_index],
                gcmd.rect_w[last_rect_index],
                gcmd.rect_h[last_rect_index]);
            const int last_rect_up_hit =
                find_recent_upload_overlap(uprobes, last_rect_src);
            const int last_rect_cp_hit =
                find_recent_copy_overlap(uprobes, last_rect_src);
            ImGui::Text(
                "Last RECT: xy=(%d,%d) wh=(%u,%u) uv=(%u,%u) rgb=(%u,%u,%u)",
                static_cast<int>(gcmd.rect_x[last_rect_index]),
                static_cast<int>(gcmd.rect_y[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_w[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_h[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_u[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_v[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_r[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_g[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_b[last_rect_index]));
            ImGui::Text(
                "Last RECT State: op=0x%02X semi=%u blend=%u clut=0x%04X page=0x%03X depth=%u raw=%u",
                static_cast<unsigned>(gcmd.rect_opcode[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_semi[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_blend[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_clut[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_texpage[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_depth[last_rect_index]),
                static_cast<unsigned>(gcmd.rect_raw[last_rect_index]));
            if (last_rect_src.valid) {
                ImGui::Text(
                    "Last RECT Src: vram=(%u,%u) wh=(%u,%u) up=%s cp=%s",
                    static_cast<unsigned>(last_rect_src.x),
                    static_cast<unsigned>(last_rect_src.y),
                    static_cast<unsigned>(last_rect_src.w),
                    static_cast<unsigned>(last_rect_src.h),
                    last_rect_up_hit >= 0 ? "hit" : "-",
                    last_rect_cp_hit >= 0 ? "hit" : "-");
            }
            for (u32 i = 0; i < 6 && i < gcmd.gp0_textured_rect_count; ++i) {
                const u32 rect_index =
                    (gcmd.gp0_textured_rect_count - 1u - i) %
                    static_cast<u32>(GpuCommandDebugInfo::kRecentRects);
                const RectSourceRegion rect_src = compute_rect_source_region(
                    gcmd.rect_texpage[rect_index],
                    gcmd.rect_depth[rect_index],
                    gcmd.rect_u[rect_index],
                    gcmd.rect_v[rect_index],
                    gcmd.rect_w[rect_index],
                    gcmd.rect_h[rect_index]);
                const int rect_up_hit =
                    find_recent_upload_overlap(uprobes, rect_src);
                const int rect_cp_hit =
                    find_recent_copy_overlap(uprobes, rect_src);
                ImGui::Text(
                    "RECT[%u]: op=0x%02X xy=(%d,%d) wh=(%u,%u) uv=(%u,%u) semi=%u blend=%u clut=0x%04X page=0x%03X depth=%u raw=%u src=(%u,%u)+(%u,%u) up=%s cp=%s",
                    i,
                    static_cast<unsigned>(gcmd.rect_opcode[rect_index]),
                    static_cast<int>(gcmd.rect_x[rect_index]),
                    static_cast<int>(gcmd.rect_y[rect_index]),
                    static_cast<unsigned>(gcmd.rect_w[rect_index]),
                    static_cast<unsigned>(gcmd.rect_h[rect_index]),
                    static_cast<unsigned>(gcmd.rect_u[rect_index]),
                    static_cast<unsigned>(gcmd.rect_v[rect_index]),
                    static_cast<unsigned>(gcmd.rect_semi[rect_index]),
                    static_cast<unsigned>(gcmd.rect_blend[rect_index]),
                    static_cast<unsigned>(gcmd.rect_clut[rect_index]),
                    static_cast<unsigned>(gcmd.rect_texpage[rect_index]),
                    static_cast<unsigned>(gcmd.rect_depth[rect_index]),
                    static_cast<unsigned>(gcmd.rect_raw[rect_index]),
                    static_cast<unsigned>(rect_src.x),
                    static_cast<unsigned>(rect_src.y),
                    static_cast<unsigned>(rect_src.w),
                    static_cast<unsigned>(rect_src.h),
                    rect_up_hit >= 0 ? "hit" : "-",
                    rect_cp_hit >= 0 ? "hit" : "-");
            }
        }
        const u32 poly_count =
            std::min<u32>(gcmd.gp0_poly_count,
                static_cast<u32>(GpuCommandDebugInfo::kRecentPolys));
        u32 shown_polys = 0;
        for (u32 i = 0; i < poly_count && shown_polys < 8; ++i) {
            const u32 poly_index =
                (gcmd.gp0_poly_count - 1u - i) %
                static_cast<u32>(GpuCommandDebugInfo::kRecentPolys);
            const RectSourceRegion poly_src =
                gcmd.poly_textured[poly_index]
                ? compute_poly_source_region(gcmd, poly_index)
                : RectSourceRegion{};
            const int poly_up_hit =
                find_recent_upload_overlap(uprobes, poly_src);
            const int poly_cp_hit =
                find_recent_copy_overlap(uprobes, poly_src);
            ImGui::Text(
                "POLY[%u]: op=0x%02X tex=%u vc=%u sh=%u semi=%u blend=%u clut=0x%04X page=0x%03X depth=%u raw=%u src=(%u,%u)+(%u,%u) up=%s cp=%s xy0=(%d,%d) xy1=(%d,%d) xy2=(%d,%d) xy3=(%d,%d) rgb0=(%u,%u,%u) rgb1=(%u,%u,%u) rgb2=(%u,%u,%u) rgb3=(%u,%u,%u) uv0=(%u,%u) uv1=(%u,%u) uv2=(%u,%u) uv3=(%u,%u)",
                shown_polys,
                static_cast<unsigned>(gcmd.poly_opcode[poly_index]),
                static_cast<unsigned>(gcmd.poly_textured[poly_index]),
                static_cast<unsigned>(gcmd.poly_vertex_count[poly_index]),
                static_cast<unsigned>(gcmd.poly_shaded[poly_index]),
                static_cast<unsigned>(gcmd.poly_semi[poly_index]),
                static_cast<unsigned>(gcmd.poly_blend[poly_index]),
                static_cast<unsigned>(gcmd.poly_clut[poly_index]),
                static_cast<unsigned>(gcmd.poly_texpage[poly_index]),
                static_cast<unsigned>(gcmd.poly_depth[poly_index]),
                static_cast<unsigned>(gcmd.poly_raw[poly_index]),
                static_cast<unsigned>(poly_src.x),
                static_cast<unsigned>(poly_src.y),
                static_cast<unsigned>(poly_src.w),
                static_cast<unsigned>(poly_src.h),
                poly_up_hit >= 0 ? "hit" : "-",
                poly_cp_hit >= 0 ? "hit" : "-",
                static_cast<int>(gcmd.poly_x[poly_index][0]),
                static_cast<int>(gcmd.poly_y[poly_index][0]),
                static_cast<int>(gcmd.poly_x[poly_index][1]),
                static_cast<int>(gcmd.poly_y[poly_index][1]),
                static_cast<int>(gcmd.poly_x[poly_index][2]),
                static_cast<int>(gcmd.poly_y[poly_index][2]),
                static_cast<int>(gcmd.poly_x[poly_index][3]),
                static_cast<int>(gcmd.poly_y[poly_index][3]),
                static_cast<unsigned>(gcmd.poly_r[poly_index][0]),
                static_cast<unsigned>(gcmd.poly_g[poly_index][0]),
                static_cast<unsigned>(gcmd.poly_b[poly_index][0]),
                static_cast<unsigned>(gcmd.poly_r[poly_index][1]),
                static_cast<unsigned>(gcmd.poly_g[poly_index][1]),
                static_cast<unsigned>(gcmd.poly_b[poly_index][1]),
                static_cast<unsigned>(gcmd.poly_r[poly_index][2]),
                static_cast<unsigned>(gcmd.poly_g[poly_index][2]),
                static_cast<unsigned>(gcmd.poly_b[poly_index][2]),
                static_cast<unsigned>(gcmd.poly_r[poly_index][3]),
                static_cast<unsigned>(gcmd.poly_g[poly_index][3]),
                static_cast<unsigned>(gcmd.poly_b[poly_index][3]),
                static_cast<unsigned>(gcmd.poly_u[poly_index][0]),
                static_cast<unsigned>(gcmd.poly_v[poly_index][0]),
                static_cast<unsigned>(gcmd.poly_u[poly_index][1]),
                static_cast<unsigned>(gcmd.poly_v[poly_index][1]),
                static_cast<unsigned>(gcmd.poly_u[poly_index][2]),
                static_cast<unsigned>(gcmd.poly_v[poly_index][2]),
                static_cast<unsigned>(gcmd.poly_u[poly_index][3]),
                static_cast<unsigned>(gcmd.poly_v[poly_index][3]));
            ++shown_polys;
        }
        if (poly_count > 0) {
            std::array<u32, GpuCommandDebugInfo::kRecentPolys> largest_indices{};
            for (u32 i = 0; i < poly_count; ++i) {
                largest_indices[static_cast<size_t>(i)] =
                    (gcmd.gp0_poly_count - 1u - i) %
                    static_cast<u32>(GpuCommandDebugInfo::kRecentPolys);
            }
            std::sort(largest_indices.begin(),
                largest_indices.begin() + static_cast<std::ptrdiff_t>(poly_count),
                [&](u32 a, u32 b) {
                    return compute_poly_bbox_area(gcmd, a) >
                        compute_poly_bbox_area(gcmd, b);
                });
            ImGui::Separator();
            ImGui::Text("Largest Recent Polys");
            for (u32 i = 0; i < std::min<u32>(poly_count, 6u); ++i) {
                const u32 poly_index = largest_indices[static_cast<size_t>(i)];
                ImGui::Text(
                    "BIG[%u]: area=%u op=0x%02X tex=%u sh=%u xy0=(%d,%d) xy1=(%d,%d) xy2=(%d,%d) xy3=(%d,%d) rgb0=(%u,%u,%u)",
                    i,
                    compute_poly_bbox_area(gcmd, poly_index),
                    static_cast<unsigned>(gcmd.poly_opcode[poly_index]),
                    static_cast<unsigned>(gcmd.poly_textured[poly_index]),
                    static_cast<unsigned>(gcmd.poly_shaded[poly_index]),
                    static_cast<int>(gcmd.poly_x[poly_index][0]),
                    static_cast<int>(gcmd.poly_y[poly_index][0]),
                    static_cast<int>(gcmd.poly_x[poly_index][1]),
                    static_cast<int>(gcmd.poly_y[poly_index][1]),
                    static_cast<int>(gcmd.poly_x[poly_index][2]),
                    static_cast<int>(gcmd.poly_y[poly_index][2]),
                    static_cast<int>(gcmd.poly_x[poly_index][3]),
                    static_cast<int>(gcmd.poly_y[poly_index][3]),
                    static_cast<unsigned>(gcmd.poly_r[poly_index][0]),
                    static_cast<unsigned>(gcmd.poly_g[poly_index][0]),
                    static_cast<unsigned>(gcmd.poly_b[poly_index][0]));
            }
            std::sort(largest_indices.begin(),
                largest_indices.begin() + static_cast<std::ptrdiff_t>(poly_count),
                [&](u32 a, u32 b) {
                    return compute_poly_visible_bbox_area(
                               gcmd, a, gdisp.display_vram_left, gdisp.display_vram_top,
                               gdisp.display_vram_width, gdisp.display_vram_height) >
                        compute_poly_visible_bbox_area(
                               gcmd, b, gdisp.display_vram_left, gdisp.display_vram_top,
                               gdisp.display_vram_width, gdisp.display_vram_height);
                });
            ImGui::Text("Visible Recent Polys");
            for (u32 i = 0; i < std::min<u32>(poly_count, 6u); ++i) {
                const u32 poly_index = largest_indices[static_cast<size_t>(i)];
                const u32 visible_area = compute_poly_visible_bbox_area(
                    gcmd, poly_index, gdisp.display_vram_left, gdisp.display_vram_top,
                    gdisp.display_vram_width, gdisp.display_vram_height);
                if (visible_area == 0) {
                    continue;
                }
                ImGui::Text(
                    "VIS[%u]: vis=%u op=0x%02X tex=%u sh=%u xy0=(%d,%d) xy1=(%d,%d) xy2=(%d,%d) xy3=(%d,%d) rgb0=(%u,%u,%u)",
                    i,
                    visible_area,
                    static_cast<unsigned>(gcmd.poly_opcode[poly_index]),
                    static_cast<unsigned>(gcmd.poly_textured[poly_index]),
                    static_cast<unsigned>(gcmd.poly_shaded[poly_index]),
                    static_cast<int>(gcmd.poly_x[poly_index][0]),
                    static_cast<int>(gcmd.poly_y[poly_index][0]),
                    static_cast<int>(gcmd.poly_x[poly_index][1]),
                    static_cast<int>(gcmd.poly_y[poly_index][1]),
                    static_cast<int>(gcmd.poly_x[poly_index][2]),
                    static_cast<int>(gcmd.poly_y[poly_index][2]),
                    static_cast<int>(gcmd.poly_x[poly_index][3]),
                    static_cast<int>(gcmd.poly_y[poly_index][3]),
                    static_cast<unsigned>(gcmd.poly_r[poly_index][0]),
                    static_cast<unsigned>(gcmd.poly_g[poly_index][0]),
                    static_cast<unsigned>(gcmd.poly_b[poly_index][0]));
            }
        }
    }
}

void App::panel_fmv_diagnostics() {
    if (!g_log_fmv_diagnostics) {
        show_fmv_diagnostics_ = false;
        return;
    }
    ImGui::SetNextWindowSize(ImVec2(900, 760), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("FMV Diagnostics", &show_fmv_diagnostics_)) {
        ImGui::BeginChild("FMVDiagnosticsWindowChild", ImVec2(0.0f, 0.0f),
            false, ImGuiWindowFlags_HorizontalScrollbar);
        draw_fmv_diagnostics_content();
        ImGui::EndChild();
    }
    ImGui::End();
}
