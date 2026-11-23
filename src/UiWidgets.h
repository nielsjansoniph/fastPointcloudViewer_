#pragma once
#include <algorithm>
#include <imgui.h>

// Slider with -/+ buttons left and right; each click divides/multiplies the value by `factor`
inline bool SliderFloatWithButtons(const char* label, float* v, float vMin, float vMax, float factor = 1.25f,
                                   const char* format = "%.3f", ImGuiSliderFlags flags = 0)
{
    ImGui::PushID(label);
    bool changed = false;
    float buttonWidth = ImGui::GetFrameHeight();
    float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
    float clickMin = vMin > 0.0f ? vMin : 0.001f;   // keep clicking from getting stuck at 0
    if (ImGui::Button("-", ImVec2(buttonWidth, 0))){
        *v = std::max(clickMin, *v / factor);
        changed = true;
    }
    ImGui::SameLine(0, spacing);
    ImGui::SetNextItemWidth(ImGui::CalcItemWidth() - 2 * (buttonWidth + spacing));
    changed |= ImGui::SliderFloat("##slider", v, vMin, vMax, format, flags);
    ImGui::SameLine(0, spacing);
    if (ImGui::Button("+", ImVec2(buttonWidth, 0))){
        *v = std::min(vMax, std::max(clickMin, *v) * factor);
        changed = true;
    }
    ImGui::SameLine(0, spacing);
    ImGui::TextUnformatted(label);
    ImGui::PopID();
    return changed;
}
