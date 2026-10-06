// Grim Reaper 2.0, Phase 5: the New Corruption panel (DESIGN.md, "Screen 2").
// Drawn inside the definitive Grim Reaper window by App::panel_definitive_grim_reaper().
#include "definitive_shared.h"
#include "ui/app.h"
#include "core/grim_share.h"

#include <algorithm>
#include <cstdio>

namespace {
// One place for every colour (DESIGN.md "Colours and theming"). Family colours reuse
// the four title-screen squares and always mean the same family.
struct GrimTheme {
    ImU32 raised = IM_COL32(26, 26, 29, 255);
    ImU32 line = IM_COL32(42, 42, 46, 255);
    ImU32 text = IM_COL32(236, 233, 226, 255);
    ImU32 muted = IM_COL32(154, 151, 143, 255);
    ImU32 faint = IM_COL32(134, 131, 124, 255);
    ImU32 audio = IM_COL32(91, 134, 224, 255);
    ImU32 visual = IM_COL32(211, 169, 63, 255);
    ImU32 code = IM_COL32(224, 83, 94, 255);
    ImU32 hardware = IM_COL32(160, 123, 224, 255);
    ImU32 alive = IM_COL32(63, 179, 166, 255);
    ImU32 working = IM_COL32(211, 169, 63, 255);
    ImU32 dead = IM_COL32(224, 83, 94, 255);
    ImU32 primary = IM_COL32(179, 35, 47, 255);
    ImU32 primary_hot = IM_COL32(204, 52, 66, 255);
};
const GrimTheme kTheme;

ImU32 family_color(u32 family) {
    switch (family) {
    case kGrimFamilyAudio: return kTheme.audio;
    case kGrimFamilyVisual: return kTheme.visual;
    case kGrimFamilyCode: return kTheme.code;
    default: return kTheme.hardware;
    }
}

ImU32 with_alpha(ImU32 c, int a) {
    return (c & 0x00FFFFFFu) | (static_cast<ImU32>(a) << 24);
}

void caption(const char* left, const char* right = nullptr, ImU32 right_color = 0) {
    ImGui::PushStyleColor(ImGuiCol_Text, kTheme.muted);
    ImGui::TextUnformatted(left);
    ImGui::PopStyleColor();
    if (right != nullptr) {
        ImGui::SameLine();
        const float w = ImGui::CalcTextSize(right).x;
        ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - w);
        ImGui::PushStyleColor(ImGuiCol_Text, right_color != 0 ? right_color : kTheme.muted);
        ImGui::TextUnformatted(right);
        ImGui::PopStyleColor();
    }
}

void dot(ImDrawList* draw, ImVec2 center, ImU32 color, float half = 4.0f) {
    draw->AddRectFilled(ImVec2(center.x - half, center.y - half),
                        ImVec2(center.x + half, center.y + half), color, 2.0f);
}

// A toggle button with a family dot. Returns true when clicked.
bool family_button(const char* id, const char* label, ImU32 color, bool on, bool enabled,
                   float width) {
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImVec2 size(width, 40.0f);
    ImGui::PushID(id);
    const bool clicked = ImGui::InvisibleButton("##family", size) && enabled;
    const bool hovered = ImGui::IsItemHovered();
    ImGui::PopID();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImU32 fill = on && enabled ? with_alpha(color, 40) : kTheme.raised;
    const ImU32 border = on && enabled ? color : (hovered ? kTheme.muted : kTheme.line);
    draw->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), fill, 6.0f);
    draw->AddRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), border, 6.0f);
    const ImVec2 ts = ImGui::CalcTextSize(label);
    const float total = 8.0f + 6.0f + ts.x;
    const float x0 = pos.x + (size.x - total) * 0.5f;
    dot(draw, ImVec2(x0 + 4.0f, pos.y + size.y * 0.5f), enabled ? color : kTheme.line);
    draw->AddText(ImVec2(x0 + 14.0f, pos.y + (size.y - ts.y) * 0.5f),
                  on && enabled ? kTheme.text : kTheme.muted, label);
    return clicked;
}

const char* risk_text(const char* label) {
    const std::string l = label;
    if (l == "safe") return "Survival biases on. Mostly alive, mild.";
    if (l == "mild") return "Survival biases a little weaker. Some strange, few dead.";
    if (l == "risky") return "Survival biases weakened. Expect some deaths.";
    return "Biases off. Frequent deaths, occasionally something insane.";
}

ImU32 risk_color(const char* label) {
    const std::string l = label;
    return l == "safe" ? kTheme.alive : l == "lethal" ? kTheme.dead : kTheme.working;
}

std::string seconds_text(double s) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.1f s", s);
    return b;
}
} // namespace

void App::draw_grim_pull_tab() {
    GrimPullState& s = grim_pull_state();
    GrimPullContext ctx;
    ctx.rom = s.rom.get();
    ctx.sample = s.sample.get();
    ctx.bios_hash = s.bios_hash;
    const u32 available = grim_pull_available_families(ctx);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float full = ImGui::GetContentRegionAvail().x;
    const bool bios_ready = system_ && system_->bios_loaded();

    // ---- this machine ----
    if (s.has_machine) {
        if (s.status.dead) {
            char right[48];
            std::snprintf(right, sizeof(right), "x dead at %.1f s", s.status.death.seconds);
            caption("THIS MACHINE", right, kTheme.dead);
        } else {
            char right[48];
            std::snprintf(right, sizeof(right), "\xE2\x97\x8F alive %.1f s", s.status.seconds);
            caption("THIS MACHINE", right, kTheme.alive);
        }
        char line[96];
        std::snprintf(line, sizeof(line), "%s  \xC2\xB7  pull #%llu", s.machine_id.c_str(),
                      static_cast<unsigned long long>(s.pull));
        ImGui::TextUnformatted(line);
        if (s.status.dead) {
            const ImVec2 p = ImGui::GetCursorScreenPos();
            const float h = 96.0f;
            draw->AddRectFilled(p, ImVec2(p.x + full, p.y + h), kTheme.raised, 6.0f);
            draw->AddRect(p, ImVec2(p.x + full, p.y + h), with_alpha(kTheme.dead, 110), 6.0f);
            ImGui::SetCursorScreenPos(ImVec2(p.x + 12.0f, p.y + 8.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, kTheme.muted);
            ImGui::TextUnformatted("CAUSE OF DEATH");
            ImGui::PopStyleColor();
            ImGui::SetCursorScreenPos(ImVec2(p.x + 12.0f, p.y + 26.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, kTheme.text);
            ImGui::TextUnformatted(s.status.death.headline.c_str());
            ImGui::PopStyleColor();
            ImGui::SetCursorScreenPos(ImVec2(p.x + 12.0f, p.y + 44.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, kTheme.muted);
            ImGui::PushTextWrapPos(p.x + full - 12.0f);
            ImGui::TextUnformatted(s.status.death.detail.c_str());
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
            const s32 culprit = s.status.death.culprit_gene;
            if (culprit >= 0 && static_cast<size_t>(culprit) < s.lines.size()) {
                ImGui::SetCursorScreenPos(ImVec2(p.x + 12.0f, p.y + h - 22.0f));
                const std::string text = "Likely culprit: " + s.lines[static_cast<size_t>(culprit)].title;
                ImGui::PushStyleColor(ImGuiCol_Text, kTheme.text);
                ImGui::TextUnformatted(text.c_str());
                ImGui::PopStyleColor();
            }
            ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + h + 4.0f));
            ImGui::Dummy(ImVec2(1.0f, 1.0f));
        } else if (s.status.silent) {
            ImGui::PushStyleColor(ImGuiCol_Text, kTheme.muted);
            ImGui::TextUnformatted("Alive, but quiet so far.");
            ImGui::PopStyleColor();
        }
    } else {
        caption("THIS MACHINE");
        ImGui::PushStyleColor(ImGuiCol_Text, kTheme.muted);
        ImGui::TextWrapped(bios_ready ? "No corrupted machine yet. Press NEW CORRUPTION."
                                      : "Load a BIOS to start.");
        ImGui::PopStyleColor();
    }
    if (!s.message.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, kTheme.working);
        ImGui::TextWrapped("%s", s.message.c_str());
        ImGui::PopStyleColor();
    }

    // ---- actions ----
    ImGui::Spacing();
    {
        ImGui::PushStyleColor(ImGuiCol_Button, kTheme.primary);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kTheme.primary_hot);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, kTheme.primary);
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 255, 255, 255));
        if (!bios_ready) {
            ImGui::BeginDisabled();
        }
        const ImVec2 p = ImGui::GetCursorScreenPos();
        if (ImGui::Button("##new_corruption", ImVec2(-1.0f, 54.0f))) {
            s.rerolls = 0; // a pull you asked for: Mercy starts counting again
            grim_pull_new();
        }
        if (!bios_ready) {
            ImGui::EndDisabled();
        }
        const char* title = "NEW CORRUPTION";
        const char* sub = s.has_machine && s.status.dead ? "pull again" : "random \xC2\xB7 nobody knows until it boots";
        const ImVec2 t1 = ImGui::CalcTextSize(title);
        const ImVec2 t2 = ImGui::CalcTextSize(sub);
        draw->AddText(ImVec2(p.x + (full - t1.x) * 0.5f, p.y + 10.0f), IM_COL32(255, 255, 255, 255), title);
        draw->AddText(ImVec2(p.x + (full - t2.x) * 0.5f, p.y + 31.0f), IM_COL32(255, 255, 255, 200), sub);
        ImGui::PopStyleColor(4);
    }
    {
        const float gap = 8.0f;
        const float w = (full - 2.0f * gap) / 3.0f;
        const bool can = s.has_machine;
        const GrimPullEntry* entry = can ? s.library.find(s.pull) : nullptr;
        const bool kept = entry != nullptr && entry->kept;
        if (!can) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Revive", ImVec2(w, 36.0f))) {
            grim_pull_boot(s.genome, s.pull);
        }
        ImGui::SameLine(0.0f, gap);
        if (ImGui::Button(kept ? "Kept" : (s.status.dead ? "Keep anyway" : "Keep"), ImVec2(w, 36.0f))) {
            grim_pull_keep(s.pull, !kept);
        }
        ImGui::SameLine(0.0f, gap);
        if (ImGui::Button("Clean machine", ImVec2(w, 36.0f))) {
            grim_pull_clean_machine();
        }
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("Reboot an ordinary PlayStation (and the loaded game, if any).\n"
                              "Booting anything else from the menus does the same.");
        }
        const float half = (full - gap) * 0.5f;
        if (ImGui::Button("Copy code", ImVec2(half, 30.0f))) {
            ImGui::SetClipboardText(grim_share_code(s.genome).c_str());
            s.message = "Corruption code copied. Anyone with the same BIOS can paste it.";
        }
        if (!can) {
            ImGui::EndDisabled();
        }
        ImGui::SameLine(0.0f, gap);
        if (!bios_ready) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Paste code", ImVec2(half, 30.0f))) {
            const char* clip = ImGui::GetClipboardText();
            grim_pull_paste_code(clip != nullptr ? clip : "");
        }
        if (!bios_ready) {
            ImGui::EndDisabled();
        }
    }

    // ---- intensity ----
    ImGui::Spacing();
    ImGui::Spacing();
    const GrimPullPlan plan = grim_pull_plan(s.settings.intensity);
    {
        caption("INTENSITY");
        const std::string readout = grim_pull_readout(s.settings.intensity);
        ImGui::SameLine();
        const float rw = ImGui::CalcTextSize(readout.c_str()).x;
        ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - rw);
        ImGui::PushStyleColor(ImGuiCol_Text, risk_color(plan.risk_label));
        ImGui::TextUnformatted(readout.c_str());
        ImGui::PopStyleColor();
        int intensity = static_cast<int>(s.settings.intensity);
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::SliderInt("##grim_intensity", &intensity, 0, 100, "")) {
            s.settings.intensity = static_cast<u32>(std::clamp(intensity, 0, 100));
        }
        ImGui::PushStyleColor(ImGuiCol_Text, kTheme.faint);
        ImGui::TextUnformatted("scratched \xC2\xB7 safe");
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x -
                             ImGui::CalcTextSize("possessed \xC2\xB7 lethal").x);
        ImGui::TextUnformatted("possessed \xC2\xB7 lethal");
        ImGui::PopStyleColor();
        ImGui::PushStyleColor(ImGuiCol_Text, kTheme.muted);
        ImGui::TextWrapped("%s", risk_text(plan.risk_label));
        ImGui::PopStyleColor();
    }

    // ---- families ----
    ImGui::Spacing();
    ImGui::Spacing();
    {
        u32 on = 0;
        for (u32 f : {kGrimFamilyAudio, kGrimFamilyVisual, kGrimFamilyCode, kGrimFamilyHardware}) {
            on += (s.settings.families & f & available) != 0 ? 1u : 0u;
        }
        char right[32];
        std::snprintf(right, sizeof(right), "%u of 4", on);
        caption("GENE FAMILIES", right);
        const float gap = 6.0f;
        const float w = (full - 3.0f * gap) / 4.0f;
        const struct {
            const char* label;
            u32 family;
            const char* tip;
        } rows[] = {
            {"Audio", kGrimFamilyAudio, "The BIOS sound bank and what the CPU tells the sound chip."},
            {"Visual", kGrimFamilyVisual, "What the CPU tells the GPU: vertices, colours, textures, draw state."},
            {"Code", kGrimFamilyCode, (available & kGrimFamilyCode) != 0
                ? "The BIOS program itself. The most lethal family."
                : "Needs the BIOS map; it is made once per BIOS (see the status line)."},
            {"Hardware", kGrimFamilyHardware,
             "Faulty hardware simulator: failing RAM, VRAM and sound RAM. Kernel memory is almost never hit."},
        };
        for (size_t i = 0; i < 4; ++i) {
            if (i > 0) {
                ImGui::SameLine(0.0f, gap);
            }
            const bool enabled = (available & rows[i].family) != 0;
            const bool on_now = (s.settings.families & rows[i].family) != 0;
            if (family_button(rows[i].label, rows[i].label, family_color(rows[i].family), on_now,
                              enabled, w)) {
                s.settings.families ^= rows[i].family;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s", rows[i].tip);
            }
        }
        ImGui::Checkbox("Rot mode: starts healthy, decays (all but BIOS patches)", &s.settings.rot);
        ImGui::Checkbox("Mercy: reroll deaths in the first 4 s", &s.mercy);
    }

    // ---- genome ----
    if (s.has_machine) {
        ImGui::Spacing();
        ImGui::Spacing();
        char head[48];
        std::snprintf(head, sizeof(head), "GENOME \xC2\xB7 %zu GENE%s", s.lines.size(),
                      s.lines.size() == 1 ? "" : "S");
        caption(head);
        for (size_t i = 0; i < s.lines.size(); ++i) {
            const GrimGeneLine& l = s.lines[i];
            const ImVec2 p = ImGui::GetCursorScreenPos();
            const bool culprit = s.status.dead && s.status.death.culprit_gene == static_cast<s32>(i);
            dot(draw, ImVec2(p.x + 4.0f, p.y + 9.0f), family_color(l.domain));
            ImGui::SetCursorScreenPos(ImVec2(p.x + 18.0f, p.y));
            ImGui::PushStyleColor(ImGuiCol_Text, culprit ? kTheme.dead : kTheme.text);
            ImGui::TextUnformatted(l.title.c_str());
            ImGui::PopStyleColor();
            // Long details (hardware genes) go on their own line under the title.
            const float room = full - 18.0f - 48.0f - ImGui::CalcTextSize(l.title.c_str()).x;
            if (ImGui::CalcTextSize(l.detail.c_str()).x > room) {
                ImGui::SetCursorScreenPos(ImVec2(p.x + 18.0f, p.y + ImGui::GetTextLineHeightWithSpacing()));
            } else {
                ImGui::SameLine();
            }
            ImGui::PushStyleColor(ImGuiCol_Text, kTheme.muted);
            ImGui::PushTextWrapPos(p.x + full - 48.0f);
            ImGui::TextUnformatted(l.detail.c_str());
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
            const char* tag = l.tag.c_str();
            draw->AddText(ImVec2(p.x + full - ImGui::CalcTextSize(tag).x, p.y), kTheme.faint, tag);
        }
    }

    // ---- history ----
    ImGui::Spacing();
    ImGui::Spacing();
    {
        const GrimLibraryStats stats = s.library.stats();
        caption("LAST PULLS");
        std::vector<const GrimPullEntry*> shown;
        const auto& all = s.library.entries();
        for (auto it = all.rbegin(); it != all.rend() && shown.size() < 8; ++it) {
            if (!it->mercy) {
                shown.push_back(&*it);
            }
        }
        if (shown.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, kTheme.faint);
            ImGui::TextUnformatted("Nothing pulled yet.");
            ImGui::PopStyleColor();
        }
        const float gap = 6.0f;
        const float w = (full - 7.0f * gap) / 8.0f;
        for (size_t i = 0; i < shown.size(); ++i) {
            const GrimPullEntry& e = *shown[i];
            if (i > 0) {
                ImGui::SameLine(0.0f, gap);
            }
            const ImVec2 p = ImGui::GetCursorScreenPos();
            ImGui::PushID(static_cast<int>(e.pull));
            const bool clicked = ImGui::InvisibleButton("##hist", ImVec2(w, 28.0f));
            ImGui::PopID();
            const ImU32 fill = e.dead ? with_alpha(kTheme.dead, 45) : with_alpha(kTheme.alive, 70);
            draw->AddRectFilled(p, ImVec2(p.x + w, p.y + 28.0f), fill, 4.0f);
            draw->AddRect(p, ImVec2(p.x + w, p.y + 28.0f),
                          e.kept ? kTheme.text : (e.dead ? with_alpha(kTheme.dead, 120) : kTheme.line), 4.0f);
            if (e.dead) {
                const ImVec2 t = ImGui::CalcTextSize("x");
                draw->AddText(ImVec2(p.x + (w - t.x) * 0.5f, p.y + 7.0f), kTheme.dead, "x");
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("pull #%llu%s\n%s\nclick to boot it again",
                                  static_cast<unsigned long long>(e.pull), e.kept ? " (kept)" : "",
                                  e.dead ? e.headline.c_str() : "alive");
            }
            if (clicked) {
                grim_pull_boot_entry(e.pull);
            }
        }
        char summary[128];
        std::snprintf(summary, sizeof(summary), "%llu pulls \xC2\xB7 %llu deaths \xC2\xB7 %llu kept",
                      static_cast<unsigned long long>(stats.pulls),
                      static_cast<unsigned long long>(stats.deaths),
                      static_cast<unsigned long long>(stats.kept));
        ImGui::PushStyleColor(ImGuiCol_Text, kTheme.faint);
        ImGui::TextUnformatted(summary);
        if (stats.mercy_rerolls > 0) {
            ImGui::Text("Mercy replaced %llu early deaths.", static_cast<unsigned long long>(stats.mercy_rerolls));
        }
        if (stats.longest_alive_seconds > 0.0) {
            ImGui::Text("Longest-lived: pull #%llu, %s", static_cast<unsigned long long>(stats.longest_alive_pull),
                        seconds_text(stats.longest_alive_seconds).c_str());
        }
        ImGui::PopStyleColor();
    }

    // ---- kept ----
    {
        size_t kept_count = 0;
        for (const GrimPullEntry& e : s.library.entries()) {
            kept_count += e.kept ? 1u : 0u;
        }
        char title[48];
        std::snprintf(title, sizeof(title), "Kept (%zu)", kept_count);
        if (ImGui::CollapsingHeader(title)) {
            u64 unkeep = 0;
            u64 boot = 0;
            for (const GrimPullEntry& e : s.library.entries()) {
                if (!e.kept) {
                    continue;
                }
                char row[96];
                std::snprintf(row, sizeof(row), "#%04X  pull %llu  %s",
                              static_cast<unsigned>(e.genome_hash & 0xFFFFu),
                              static_cast<unsigned long long>(e.pull),
                              e.dead ? e.headline.c_str() : seconds_text(e.seconds).c_str());
                ImGui::PushID(static_cast<int>(e.pull));
                if (ImGui::Button("Boot")) {
                    boot = e.pull;
                }
                ImGui::SameLine();
                if (ImGui::Button("Drop")) {
                    unkeep = e.pull;
                }
                ImGui::SameLine();
                ImGui::TextUnformatted(row);
                ImGui::PopID();
            }
            if (kept_count == 0) {
                ImGui::PushStyleColor(ImGuiCol_Text, kTheme.faint);
                ImGui::TextUnformatted("Keep a machine to save it here.");
                ImGui::PopStyleColor();
            }
            if (unkeep != 0) {
                grim_pull_keep(unkeep, false);
            }
            if (boot != 0) {
                grim_pull_boot_entry(boot);
            }
        }
    }

    // ---- status ----
    ImGui::Spacing();
    {
        const bool watching = s.watch && !s.status.dead;
        const ImVec2 p = ImGui::GetCursorScreenPos();
        dot(draw, ImVec2(p.x + 4.0f, p.y + 9.0f),
            s.status.dead ? kTheme.dead : (watching ? kTheme.alive : kTheme.faint), 3.5f);
        ImGui::SetCursorScreenPos(ImVec2(p.x + 16.0f, p.y));
        char line[160];
        std::snprintf(line, sizeof(line), "%s \xC2\xB7 Mercy %s \xC2\xB7 pull #%llu",
                      s.status.dead ? "Dead" : (watching ? "Death watch on" : "Idle"), s.mercy ? "on" : "off",
                      static_cast<unsigned long long>(s.library.next_pull() - 1u));
        ImGui::PushStyleColor(ImGuiCol_Text, kTheme.muted);
        ImGui::TextUnformatted(line);
        const GrimPullState::MapState ms = s.map_state;
        if (ms == GrimPullState::MapState::Running) {
            ImGui::TextUnformatted("Mapping this BIOS once for Code genes...");
        } else if (ms == GrimPullState::MapState::Ready) {
            ImGui::TextUnformatted("BIOS map ready: all families unlocked.");
        } else if (ms == GrimPullState::MapState::Failed) {
            ImGui::PushStyleColor(ImGuiCol_Text, kTheme.working);
            ImGui::TextWrapped("Mapping failed (%s). Code genes stay off.", s.map_error.c_str());
            ImGui::PopStyleColor();
        }
        ImGui::PopStyleColor();
        if (ms == GrimPullState::MapState::Failed && ImGui::Button("Retry mapping")) {
            grim_pull_start_mapping();
        }
    }
}
