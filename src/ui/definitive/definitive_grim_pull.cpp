// Grim Reaper 2.0: the New Corruption panel (DESIGN.md, "Screen 2").
// Drawn inside the definitive Grim Reaper window by App::panel_definitive_grim_reaper(),
// which adds the Back button underneath.
#include "definitive_shared.h"
#include "ui/app.h"
#include "core/grim_share.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iterator>

namespace {
using namespace definitive_ui;

// Every colour comes from here. Surfaces and text follow the user's theme (the ImGui
// style it sets); the primary button follows the theme accent once the theme is changed.
// Only meaning colours are fixed: the four families and alive / working / dead.
struct Palette {
    ImU32 raised, raised_hot, line, text, muted, faint, primary, primary_hot;
    ImU32 audio = IM_COL32(91, 134, 224, 255);
    ImU32 visual = IM_COL32(211, 169, 63, 255);
    ImU32 code = IM_COL32(224, 83, 94, 255);
    ImU32 hardware = IM_COL32(160, 123, 224, 255);
    ImU32 fmv = IM_COL32(96, 196, 112, 255);
    ImU32 alive = IM_COL32(63, 179, 166, 255);
    ImU32 working = IM_COL32(211, 169, 63, 255);
    ImU32 dead = IM_COL32(224, 83, 94, 255);
};

ImU32 with_alpha(ImU32 c, int a) {
    return (c & 0x00FFFFFFu) | (static_cast<ImU32>(std::clamp(a, 0, 255)) << 24);
}

Palette palette() {
    Palette p;
    p.raised = ImGui::GetColorU32(ImGuiCol_FrameBg);
    p.raised_hot = ImGui::GetColorU32(ImGuiCol_FrameBgHovered);
    p.line = ImGui::GetColorU32(ImGuiCol_Separator);
    p.text = ImGui::GetColorU32(ImGuiCol_Text);
    p.muted = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    p.faint = with_alpha(p.muted, 170);
    p.primary = accent_color(IM_COL32(179, 35, 47, 255));
    p.primary_hot = accent_color(IM_COL32(204, 52, 66, 255));
    return p;
}

ImU32 family_color(const Palette& p, u32 family) {
    switch (family) {
    case kGrimFamilyAudio: return p.audio;
    case kGrimFamilyVisual: return p.visual;
    case kGrimFamilyCode: return p.code;
    case kGrimFamilyFmv: return p.fmv;
    default: return p.hardware;
    }
}

const char* risk_text(const char* label) {
    const std::string l = label;
    if (l == "safe") return "Survival biases on. Mostly alive, mild.";
    if (l == "mild") return "Survival biases a little weaker. Some strange, few dead.";
    if (l == "risky") return "Survival biases weakened. Expect some deaths.";
    return "Biases off. Frequent deaths, occasionally something insane.";
}

ImU32 risk_color(const Palette& p, const char* label) {
    const std::string l = label;
    return l == "safe" ? p.alive : l == "lethal" ? p.dead : p.working;
}

double steady_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

void dot(ImDrawList* draw, ImVec2 c, ImU32 color, float half = 4.0f) {
    draw->AddRectFilled(ImVec2(c.x - half, c.y - half), ImVec2(c.x + half, c.y + half), color, 2.0f);
}

// Section caption: small muted capitals on the left, optional text on the right.
void caption(const Palette& p, const char* left, const char* right = nullptr, ImU32 right_color = 0) {
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Text, p.muted);
    ImGui::TextUnformatted(left);
    ImGui::PopStyleColor();
    if (right != nullptr) {
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize(right).x);
        ImGui::PushStyleColor(ImGuiCol_Text, right_color != 0 ? right_color : p.muted);
        ImGui::TextUnformatted(right);
        ImGui::PopStyleColor();
    }
}

// A card whose background is drawn after its content, so it fits whatever was inside.
struct Card {
    ImVec2 p0;
    float width = 0.0f;
    float pad = 12.0f;
};

Card begin_card(float width, float pad = 12.0f) {
    Card c;
    c.p0 = ImGui::GetCursorScreenPos();
    c.width = width;
    c.pad = pad;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->ChannelsSplit(2);
    draw->ChannelsSetCurrent(1);
    ImGui::SetCursorScreenPos(ImVec2(c.p0.x + pad, c.p0.y + pad));
    ImGui::BeginGroup();
    ImGui::PushTextWrapPos(c.p0.x - ImGui::GetWindowPos().x + width - pad);
    return c;
}

void end_card(const Card& c, ImU32 fill, ImU32 border) {
    ImGui::PopTextWrapPos();
    ImGui::EndGroup();
    const float h = ImGui::GetItemRectMax().y - c.p0.y + c.pad;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->ChannelsSetCurrent(0);
    const ImVec2 p1(c.p0.x + c.width, c.p0.y + h);
    draw->AddRectFilled(c.p0, p1, fill, 8.0f);
    draw->AddRect(c.p0, p1, border, 8.0f);
    draw->ChannelsMerge();
    ImGui::SetCursorScreenPos(ImVec2(c.p0.x, p1.y));
    ImGui::Dummy(ImVec2(c.width, 0.0f));
}

void colored_text(ImU32 color, const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextUnformatted(text);
    ImGui::PopStyleColor();
}

void wrapped_text(ImU32 color, const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextWrapped("%s", text);
    ImGui::PopStyleColor();
}

// Bordered button for the secondary actions. Disabled ones are visibly dimmed.
bool action_button(const Palette& p, const char* label, ImVec2 size, bool enabled, const char* tip = nullptr) {
    ImGui::PushStyleColor(ImGuiCol_Button, p.raised);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, p.raised_hot);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, p.raised_hot);
    ImGui::PushStyleColor(ImGuiCol_Border, p.line);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);
    if (!enabled) {
        ImGui::BeginDisabled();
    }
    const bool clicked = ImGui::Button(label, size);
    if (!enabled) {
        ImGui::EndDisabled();
    }
    if (tip != nullptr && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("%s", tip);
    }
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(4);
    return clicked && enabled;
}

// A family toggle chip with its colour dot. Returns true when clicked.
bool family_chip(const Palette& p, const char* label, ImU32 color, bool on, bool enabled, float width) {
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImVec2 size(width, 36.0f);
    ImGui::PushID(label);
    const bool clicked = ImGui::InvisibleButton("##family", size) && enabled;
    const bool hovered = ImGui::IsItemHovered();
    ImGui::PopID();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const bool lit = on && enabled;
    draw->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), lit ? with_alpha(color, 40) : p.raised, 6.0f);
    draw->AddRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), lit ? color : (hovered ? p.muted : p.line), 6.0f);
    const ImVec2 ts = ImGui::CalcTextSize(label);
    const float x0 = pos.x + (size.x - (14.0f + ts.x)) * 0.5f;
    dot(draw, ImVec2(x0 + 4.0f, pos.y + size.y * 0.5f), enabled ? color : p.line);
    draw->AddText(ImVec2(x0 + 14.0f, pos.y + (size.y - ts.y) * 0.5f), lit ? p.text : p.muted, label);
    return clicked;
}

// A switch row: toggle, title, one-line description. Returns true when flipped.
bool switch_row(const Palette& p, const char* id, const char* title, const char* desc, bool& value, float width) {
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const float line = ImGui::GetTextLineHeight();
    const ImVec2 size(width, line * 2.0f + 6.0f);
    ImGui::PushID(id);
    const bool clicked = ImGui::InvisibleButton("##switch", size);
    ImGui::PopID();
    if (clicked) {
        value = !value;
    }
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float sy = pos.y + (size.y - 16.0f) * 0.5f;
    draw->AddRectFilled(ImVec2(pos.x, sy), ImVec2(pos.x + 30.0f, sy + 16.0f), value ? p.primary : p.line, 8.0f);
    const float knob = value ? pos.x + 22.0f : pos.x + 8.0f;
    draw->AddCircleFilled(ImVec2(knob, sy + 8.0f), 6.0f, value ? IM_COL32(255, 255, 255, 255) : p.muted);
    draw->AddText(ImVec2(pos.x + 42.0f, pos.y + 1.0f), p.text, title);
    draw->AddText(ImVec2(pos.x + 42.0f, pos.y + line + 4.0f), p.muted, desc);
    return clicked;
}

std::string seconds_text(double s) {
    char b[32];
    std::snprintf(b, sizeof(b), s < 10.0 ? "%.1f s" : "%.0f s", s);
    return b;
}
} // namespace

void App::draw_grim_pull_tab() {
    GrimPullState& s = grim_pull_state();
    const Palette p = palette();
    GrimPullContext ctx;
    ctx.rom = s.rom.get();
    ctx.sample = s.sample.get();
    ctx.bios_hash = s.bios_hash;
    const u32 available = grim_pull_available_families(ctx);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float full = ImGui::GetContentRegionAvail().x;
    const float gap = 6.0f;
    const bool bios_ready = system_ && system_->bios_loaded();
    const GrimPullPlan plan = grim_pull_plan(s.settings.intensity);

    // Culprit, as an index into the whole genome (the watch judges the running genes).
    s32 culprit = -1;
    if (s.has_machine && s.status.dead && s.status.death.culprit_gene >= 0 &&
        static_cast<size_t>(s.status.death.culprit_gene) < s.running_index.size()) {
        culprit = static_cast<s32>(s.running_index[static_cast<size_t>(s.status.death.culprit_gene)]);
    }

    // ---- this machine ----
    ImGui::Spacing();
    if (!s.has_machine) {
        const Card c = begin_card(full, 14.0f);
        colored_text(p.text, bios_ready ? "No machine yet" : "Load a BIOS to start");
        wrapped_text(p.muted, bios_ready
            ? "Every pull is a surprise. Load a game first to corrupt it instead of the BIOS."
            : "Pick a BIOS from the launcher, then come back here.");
        end_card(c, with_alpha(p.raised, 90), p.line);
    } else if (!s.status.dead) {
        const Card c = begin_card(full);
        ImGui::PushFont(font_for_size(24.0f));
        colored_text(p.text, s.machine_id.c_str());
        ImGui::PopFont();
        ImGui::SameLine(0.0f, 14.0f);
        ImGui::BeginGroup();
        char line[96];
        std::snprintf(line, sizeof(line), "Alive \xC2\xB7 %s%s", seconds_text(s.status.seconds).c_str(),
                      s.status.silent ? " \xC2\xB7 quiet so far" : "");
        colored_text(p.alive, line);
        std::snprintf(line, sizeof(line), "Pull %llu \xC2\xB7 %zu gene%s", static_cast<unsigned long long>(s.pull),
                      s.running.genes.size(), s.running.genes.size() == 1 ? "" : "s");
        colored_text(p.muted, line);
        ImGui::SameLine(0.0f, 8.0f);
        const ImVec2 dp = ImGui::GetCursorScreenPos();
        u32 fams = 0;
        for (const GrimGene& g : s.running.genes) {
            fams |= grim_gene_family(g.type);
        }
        float x = dp.x + 4.0f;
        for (u32 f : {kGrimFamilyAudio, kGrimFamilyVisual, kGrimFamilyCode, kGrimFamilyHardware, kGrimFamilyFmv}) {
            if (fams & f) {
                dot(draw, ImVec2(x, dp.y + ImGui::GetTextLineHeight() * 0.5f), family_color(p, f));
                x += 12.0f;
            }
        }
        ImGui::NewLine();
        ImGui::EndGroup();
        end_card(c, p.raised, p.alive);
    } else {
        const Card c = begin_card(full);
        char line[96];
        std::snprintf(line, sizeof(line), "x  Dead at %s", seconds_text(s.status.death.seconds).c_str());
        colored_text(p.dead, line);
        std::snprintf(line, sizeof(line), "%s \xC2\xB7 pull %llu", s.machine_id.c_str(),
                      static_cast<unsigned long long>(s.pull));
        ImGui::SameLine();
        ImGui::SetCursorScreenPos(ImVec2(c.p0.x + full - c.pad - ImGui::CalcTextSize(line).x,
                                         ImGui::GetCursorScreenPos().y));
        colored_text(p.muted, line);
        ImGui::PushFont(font_for_size(17.0f));
        colored_text(p.text, s.status.death.headline.c_str());
        ImGui::PopFont();
        wrapped_text(p.muted, s.status.death.detail.c_str());
        if (culprit >= 0 && static_cast<size_t>(culprit) < s.lines.size()) {
            ImGui::Spacing();
            colored_text(p.muted, "Likely culprit:");
            ImGui::SameLine();
            colored_text(p.dead, s.lines[static_cast<size_t>(culprit)].title.c_str());
            if (action_button(p, "Revive without it", ImVec2(full - 2.0f * c.pad, 28.0f), true,
                              "Switch that gene off and boot the rest again.")) {
                s.gene_on[static_cast<size_t>(culprit)] = 0;
                grim_pull_revive();
            }
        }
        end_card(c, with_alpha(p.dead, 38), p.dead);
    }

    // ---- actions ----
    ImGui::Spacing();
    {
        const float revive_w = 76.0f;
        const float main_w = full - revive_w - gap;
        const ImVec2 bp = ImGui::GetCursorScreenPos();
        ImGui::PushStyleColor(ImGuiCol_Button, p.primary);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, p.primary_hot);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, p.primary);
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8.0f);
        if (!bios_ready) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("##new_corruption", ImVec2(main_w, 54.0f))) {
            s.rerolls = 0; // a pull you asked for: Mercy starts counting again
            grim_pull_new();
        }
        if (!bios_ready) {
            ImGui::EndDisabled();
        }
        ImGui::PopStyleVar();
        ImGui::PopStyleColor(3);
        const char* title = "NEW CORRUPTION";
        std::string sub = s.has_machine && s.status.dead
            ? std::string("pull again")
            : grim_pull_readout(s.settings.intensity) + " \xC2\xB7 nobody knows until it boots";
        if (ImGui::CalcTextSize(sub.c_str()).x > main_w - 16.0f) {
            sub = s.has_machine && s.status.dead ? "pull again" : grim_pull_readout(s.settings.intensity);
        }
        const ImVec2 t1 = ImGui::CalcTextSize(title);
        const ImVec2 t2 = ImGui::CalcTextSize(sub.c_str());
        draw->AddText(ImVec2(bp.x + (main_w - t1.x) * 0.5f, bp.y + 9.0f), IM_COL32(255, 255, 255, 255), title);
        draw->AddText(ImVec2(bp.x + (main_w - t2.x) * 0.5f, bp.y + 30.0f), IM_COL32(255, 255, 255, 200),
                      sub.c_str());
        ImGui::SameLine(0.0f, gap);
        const bool changed_mask = s.has_machine && s.gene_on != s.running_on;
        if (action_button(p, changed_mask ? "Revive*" : "Revive", ImVec2(revive_w, 54.0f), s.has_machine,
                          changed_mask ? "Boot this machine again with the genes you switched on."
                                       : "Boot this machine again from the start.")) {
            grim_pull_revive();
        }

        const float w = (full - 3.0f * gap) / 4.0f;
        const GrimPullEntry* entry = s.has_machine ? s.library.find(s.pull) : nullptr;
        const bool kept = entry != nullptr && entry->kept;
        if (action_button(p, kept ? "Kept" : (s.status.dead ? "Keep anyway" : "Keep"), ImVec2(w, 34.0f),
                          s.has_machine, "Save this machine in your Kept list.")) {
            grim_pull_keep(s.pull, !kept);
        }
        ImGui::SameLine(0.0f, gap);
        if (action_button(p, "Clean", ImVec2(w, 34.0f), s.has_machine,
                          "Reboot an ordinary PlayStation (and the loaded game, if any).\n"
                          "Booting anything else from the menus does the same.")) {
            grim_pull_clean_machine();
        }
        ImGui::SameLine(0.0f, gap);
        if (action_button(p, "Copy code", ImVec2(w, 34.0f), s.has_machine,
                          "Copy this machine as a corruption code anyone with the same BIOS can paste.")) {
            ImGui::SetClipboardText(grim_share_code(s.running).c_str());
            s.message = "Corruption code copied.";
        }
        ImGui::SameLine(0.0f, gap);
        if (action_button(p, "Paste code", ImVec2(w, 34.0f), bios_ready, "Boot a corruption code from the clipboard.")) {
            const char* clip = ImGui::GetClipboardText();
            grim_pull_paste_code(clip != nullptr ? clip : "");
        }
    }
    if (!s.message.empty()) {
        wrapped_text(p.working, s.message.c_str());
    }

    // ---- genome ----
    if (s.has_machine) {
        size_t on = 0;
        for (char c : s.gene_on) {
            on += c != 0 ? 1u : 0u;
        }
        char head[48], right[48];
        std::snprintf(head, sizeof(head), "GENOME \xC2\xB7 %zu GENE%s", s.lines.size(), s.lines.size() == 1 ? "" : "S");
        std::snprintf(right, sizeof(right), on == s.lines.size() ? "all on" : "%zu of %zu on", on, s.lines.size());
        caption(p, head, right, on == s.lines.size() ? 0 : p.working);
        for (size_t i = 0; i < s.lines.size() && i < s.gene_on.size(); ++i) {
            const GrimGeneLine& l = s.lines[i];
            const bool enabled = s.gene_on[i] != 0;
            const bool is_culprit = static_cast<s32>(i) == culprit;
            const ImVec2 rp = ImGui::GetCursorScreenPos();
            draw->AddLine(ImVec2(rp.x, rp.y), ImVec2(rp.x + full, rp.y), p.line);
            ImGui::SetCursorScreenPos(ImVec2(rp.x, rp.y + 5.0f));
            // Switch: a small checkbox square.
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::InvisibleButton("##gene_on", ImVec2(16.0f, 16.0f))) {
                s.gene_on[i] = enabled ? 0 : 1;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(enabled ? "Switch this gene off (applies on Revive)"
                                          : "Switch this gene on (applies on Revive)");
            }
            ImGui::PopID();
            const ImVec2 bmin = ImGui::GetItemRectMin(), bmax = ImGui::GetItemRectMax();
            draw->AddRect(bmin, bmax, enabled ? p.text : p.line, 3.0f);
            if (enabled) {
                draw->AddRectFilled(ImVec2(bmin.x + 4.0f, bmin.y + 4.0f), ImVec2(bmax.x - 4.0f, bmax.y - 4.0f),
                                    p.text, 1.5f);
            }
            dot(draw, ImVec2(rp.x + 30.0f, rp.y + 13.0f), enabled ? family_color(p, l.domain) : p.line);
            const ImU32 title_color = is_culprit ? p.dead : (enabled ? p.text : p.faint);
            ImGui::SetCursorScreenPos(ImVec2(rp.x + 42.0f, rp.y + 5.0f));
            colored_text(title_color, l.title.c_str());
            const float tag_w = ImGui::CalcTextSize(l.tag.c_str()).x;
            draw->AddText(ImVec2(rp.x + full - tag_w, rp.y + 5.0f), p.faint, l.tag.c_str());
            const float room = full - 42.0f - tag_w - 16.0f - ImGui::CalcTextSize(l.title.c_str()).x;
            if (ImGui::CalcTextSize(l.detail.c_str()).x > room) {
                ImGui::SetCursorScreenPos(ImVec2(rp.x + 42.0f, ImGui::GetCursorScreenPos().y));
            } else {
                ImGui::SameLine(0.0f, 8.0f);
            }
            ImGui::PushTextWrapPos(rp.x - ImGui::GetWindowPos().x + full - tag_w - 10.0f);
            wrapped_text(enabled ? p.muted : p.faint, l.detail.c_str());
            ImGui::PopTextWrapPos();
            ImGui::Dummy(ImVec2(full, 2.0f));
        }
        if (s.gene_on != s.running_on) {
            wrapped_text(p.working, "Press Revive to boot the machine with these genes.");
        }
    }

    // ---- recipe ----
    ImGui::Spacing();
    {
        u32 fam_on = 0;
        for (u32 f : {kGrimFamilyAudio, kGrimFamilyVisual, kGrimFamilyCode, kGrimFamilyHardware, kGrimFamilyFmv}) {
            fam_on += (s.settings.families & f & available) != 0 ? 1u : 0u;
        }
        char summary[96];
        std::snprintf(summary, sizeof(summary), "%s \xC2\xB7 %u of 5 \xC2\xB7 Mercy %s",
                      grim_pull_readout(s.settings.intensity).c_str(), fam_on, s.mercy ? "on" : "off");
        const ImVec2 hp = ImGui::GetCursorScreenPos();
        const float hh = ImGui::GetTextLineHeight() + 10.0f;
        if (ImGui::InvisibleButton("##recipe_header", ImVec2(full, hh))) {
            s.recipe_open = !s.recipe_open;
        }
        const bool hot = ImGui::IsItemHovered();
        draw->AddLine(ImVec2(hp.x, hp.y), ImVec2(hp.x + full, hp.y), p.line);
        draw->AddText(ImVec2(hp.x, hp.y + 5.0f), hot ? p.text : p.muted, s.recipe_open ? "v  RECIPE" : ">  RECIPE");
        draw->AddText(ImVec2(hp.x + full - ImGui::CalcTextSize(summary).x, hp.y + 5.0f), p.muted, summary);
    }
    if (s.recipe_open) {
        // Intensity: the track shows the four risk bands under a standard (navigable) slider.
        ImGui::TextUnformatted("Intensity");
        char value[48];
        std::snprintf(value, sizeof(value), "%u \xC2\xB7 %s", s.settings.intensity, plan.risk_label);
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize(value).x);
        colored_text(risk_color(p, plan.risk_label), value);
        {
            const ImVec2 sp = ImGui::GetCursorScreenPos();
            const float sh = ImGui::GetFrameHeight();
            const float inset = 10.0f;
            const float x0 = sp.x + inset, x1 = sp.x + full - inset, y = sp.y + sh * 0.5f;
            const struct {
                float from, to;
                ImU32 color;
            } bands[] = {{0, 20, p.alive}, {20, 45, with_alpha(p.working, 150)}, {45, 70, p.working}, {70, 100, p.dead}};
            for (const auto& b : bands) {
                draw->AddRectFilled(ImVec2(x0 + (x1 - x0) * b.from / 100.0f, y - 2.0f),
                                    ImVec2(x0 + (x1 - x0) * b.to / 100.0f, y + 2.0f), b.color);
            }
            int intensity = static_cast<int>(s.settings.intensity);
            ImGui::PushStyleColor(ImGuiCol_FrameBg, IM_COL32(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, IM_COL32(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_FrameBgActive, IM_COL32(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_SliderGrab, risk_color(p, plan.risk_label));
            ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, IM_COL32(255, 255, 255, 255));
            ImGui::PushStyleVar(ImGuiStyleVar_GrabRounding, 8.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_GrabMinSize, 14.0f);
            ImGui::SetNextItemWidth(full);
            if (ImGui::SliderInt("##grim_intensity", &intensity, 0, 100, "")) {
                s.settings.intensity = static_cast<u32>(std::clamp(intensity, 0, 100));
            }
            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor(5);
        }
        colored_text(p.faint, "scratched \xC2\xB7 safe");
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("possessed \xC2\xB7 lethal").x);
        colored_text(p.faint, "possessed \xC2\xB7 lethal");
        wrapped_text(p.muted, risk_text(plan.risk_label));

        ImGui::Spacing();
        const float w = (full - 4.0f * gap) / 5.0f;
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
            {"FMV", kGrimFamilyFmv,
             "Full-motion video, corrupted as the MDEC decodes it. Movies break, they never stall."},
        };
        for (size_t i = 0; i < std::size(rows); ++i) {
            if (i > 0) {
                ImGui::SameLine(0.0f, gap);
            }
            const bool enabled = (available & rows[i].family) != 0;
            if (family_chip(p, rows[i].label, family_color(p, rows[i].family),
                            (s.settings.families & rows[i].family) != 0, enabled, w)) {
                s.settings.families ^= rows[i].family;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s", rows[i].tip);
            }
        }
        ImGui::Spacing();
        switch_row(p, "rot", "Rot", "Starts healthy, then decays (not BIOS patches)", s.settings.rot, full);
        switch_row(p, "mercy", "Mercy", "Quietly rerolls deaths in the first 4 s", s.mercy, full);
    }

    // ---- last pulls ----
    {
        size_t kept_count = 0;
        for (const GrimPullEntry& e : s.library.entries()) {
            kept_count += e.kept ? 1u : 0u;
        }
        char kept_label[32];
        std::snprintf(kept_label, sizeof(kept_label), s.kept_open ? "Kept (%zu)  v" : "Kept (%zu)  >", kept_count);
        caption(p, "LAST PULLS");
        ImGui::SameLine();
        const float kw = ImGui::CalcTextSize(kept_label).x;
        ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - kw);
        const ImVec2 kp = ImGui::GetCursorScreenPos();
        if (ImGui::InvisibleButton("##kept_toggle", ImVec2(kw, ImGui::GetTextLineHeight()))) {
            s.kept_open = !s.kept_open;
        }
        draw->AddText(kp, ImGui::IsItemHovered() ? p.text : p.muted, kept_label);

        std::vector<const GrimPullEntry*> shown;
        const auto& all = s.library.entries();
        for (auto it = all.rbegin(); it != all.rend() && shown.size() < 12; ++it) {
            if (!it->mercy) {
                shown.push_back(&*it);
            }
        }
        if (shown.empty()) {
            colored_text(p.faint, "Nothing pulled yet.");
        }
        const float right = ImGui::GetCursorScreenPos().x + full;
        for (size_t i = 0; i < shown.size(); ++i) {
            const GrimPullEntry& e = *shown[i];
            const bool live = s.has_machine && e.pull == s.pull && !s.status.dead;
            char label[48];
            std::snprintf(label, sizeof(label), "%s#%04X %s%s", e.dead ? "x " : "",
                          static_cast<unsigned>(e.genome_hash & 0xFFFFu),
                          live ? "live" : seconds_text(e.seconds).c_str(), e.kept ? " *" : "");
            const ImVec2 size(ImGui::CalcTextSize(label).x + 14.0f, ImGui::GetTextLineHeight() + 8.0f);
            if (i > 0) {
                ImGui::SameLine(0.0f, 5.0f);
                if (ImGui::GetCursorScreenPos().x + size.x > right) {
                    ImGui::NewLine();
                }
            }
            const ImVec2 cp = ImGui::GetCursorScreenPos();
            ImGui::PushID(static_cast<int>(e.pull));
            const bool clicked = ImGui::InvisibleButton("##hist", size);
            ImGui::PopID();
            const bool current = s.has_machine && e.pull == s.pull;
            const ImU32 border = current ? p.text : e.dead ? with_alpha(p.dead, 170) : p.line;
            draw->AddRectFilled(cp, ImVec2(cp.x + size.x, cp.y + size.y), p.raised, 5.0f);
            draw->AddRect(cp, ImVec2(cp.x + size.x, cp.y + size.y), border, 5.0f);
            draw->AddText(ImVec2(cp.x + 7.0f, cp.y + 4.0f), e.dead ? p.dead : p.text, label);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("pull #%llu%s\n%s%s%s\nclick to boot it again",
                                  static_cast<unsigned long long>(e.pull), e.kept ? " (kept)" : "",
                                  e.dead ? e.headline.c_str() : "alive", e.note.empty() ? "" : "\n",
                                  e.note.c_str());
            }
            if (clicked) {
                grim_pull_boot_entry(e.pull);
            }
        }
        const GrimLibraryStats stats = s.library.stats();
        char line[160];
        std::snprintf(line, sizeof(line), "%llu pulls \xC2\xB7 %llu deaths", static_cast<unsigned long long>(stats.pulls),
                      static_cast<unsigned long long>(stats.deaths));
        std::string text = line;
        if (stats.longest_alive_seconds > 0.0) {
            std::snprintf(line, sizeof(line), " \xC2\xB7 longest pull #%llu, %s",
                          static_cast<unsigned long long>(stats.longest_alive_pull),
                          seconds_text(stats.longest_alive_seconds).c_str());
            text += line;
        }
        if (stats.mercy_rerolls > 0) {
            std::snprintf(line, sizeof(line), " \xC2\xB7 Mercy rerolled %llu",
                          static_cast<unsigned long long>(stats.mercy_rerolls));
            text += line;
        }
        wrapped_text(p.faint, text.c_str());

        if (s.kept_open) {
            u64 unkeep = 0, boot = 0;
            for (const GrimPullEntry& e : s.library.entries()) {
                if (!e.kept) {
                    continue;
                }
                ImGui::PushID(static_cast<int>(e.pull));
                if (action_button(p, "Boot", ImVec2(52.0f, 24.0f), true)) {
                    boot = e.pull;
                }
                ImGui::SameLine(0.0f, 4.0f);
                if (action_button(p, "Drop", ImVec2(52.0f, 24.0f), true)) {
                    unkeep = e.pull;
                }
                ImGui::PopID();
                ImGui::SameLine(0.0f, 8.0f);
                char row[160];
                std::snprintf(row, sizeof(row), "#%04X  pull %llu  %s%s%s",
                              static_cast<unsigned>(e.genome_hash & 0xFFFFu), static_cast<unsigned long long>(e.pull),
                              e.dead ? e.headline.c_str() : seconds_text(e.seconds).c_str(),
                              e.note.empty() ? "" : "  \xC2\xB7 ", e.note.c_str());
                colored_text(e.dead ? p.dead : p.text, row);
            }
            if (kept_count == 0) {
                colored_text(p.faint, "Keep a machine to save it here.");
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
        const ImVec2 sp = ImGui::GetCursorScreenPos();
        draw->AddLine(ImVec2(sp.x, sp.y), ImVec2(sp.x + full, sp.y), p.line);
        ImGui::Dummy(ImVec2(full, 4.0f));
        const bool watching = s.watch && !s.status.dead;
        const ImVec2 lp = ImGui::GetCursorScreenPos();
        dot(draw, ImVec2(lp.x + 4.0f, lp.y + ImGui::GetTextLineHeight() * 0.5f),
            s.status.dead ? p.dead : (watching ? p.alive : p.faint), 3.5f);
        ImGui::SetCursorScreenPos(ImVec2(lp.x + 14.0f, lp.y));
        colored_text(p.muted, s.status.dead ? "Dead \xC2\xB7 verdict fixed"
                              : watching    ? "Death watch on"
                                            : "Idle");
        const GrimPullState::MapState ms = s.map_state;
        if (ms == GrimPullState::MapState::Running) {
            const char* m = "Mapping this BIOS for Code genes";
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize(m).x);
            colored_text(p.muted, m);
            // An estimate: discovery takes about 20-30 s on a desktop CPU.
            const float frac = static_cast<float>(std::min(0.95, (steady_seconds() - s.map_started) / 25.0));
            const ImVec2 bp = ImGui::GetCursorScreenPos();
            draw->AddRectFilled(ImVec2(bp.x, bp.y + 2.0f), ImVec2(bp.x + full, bp.y + 5.0f), p.line, 2.0f);
            draw->AddRectFilled(ImVec2(bp.x, bp.y + 2.0f), ImVec2(bp.x + full * frac, bp.y + 5.0f), p.code, 2.0f);
            ImGui::Dummy(ImVec2(full, 7.0f));
        } else if (ms == GrimPullState::MapState::Failed) {
            wrapped_text(p.working, ("Mapping failed (" + s.map_error + "). Code genes stay off.").c_str());
            if (action_button(p, "Retry mapping", ImVec2(full, 28.0f), true)) {
                grim_pull_start_mapping();
            }
        }
    }
}
