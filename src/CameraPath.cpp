#include "CameraPath.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <string>

#include <imgui.h>
#include <opencv2/core.hpp>

#include "Paths.h"

namespace {
// Interpolation: at most this distance (m) and angle (degrees) per frame
constexpr float kFrameDistance = 0.1f;
constexpr float kFrameAngleDeg = 2.0f;
constexpr double kVideoFps = 25.0;
}

void CameraPath::addWaypointIfChanged(const Camera& camera)
{
    if (!positions.empty() && positions.back() == camera.Position && orientations.back() == camera.Orientation)
        return;
    positions.push_back(camera.Position);
    orientations.push_back(camera.Orientation);
}

void CameraPath::drawUi(Camera& camera)
{
    if (ImGui::Button("Replay")) request = ReplayRequest::All;
    ImGui::SameLine();
    ImGui::Text("%d Positions", (int)positions.size());
    ImGui::SameLine();
    ImGui::Checkbox("with loop", &loop);
    ImGui::SameLine();
    ImGui::Checkbox("and record", &record);
    if (ImGui::Button("Replay from selection")) request = ReplayRequest::FromSelection;
    ImGui::SameLine();
    if (ImGui::Button("Replay one step")) request = ReplayRequest::OneStep;

    int toDelete = -1, toMoveUp = -1, toMoveDown = -1;
    if (ImGui::BeginTable("Waypoints", 8, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)){
        ImGui::TableSetupColumn("#");
        ImGui::TableSetupColumn("Pos x");
        ImGui::TableSetupColumn("Pos y");
        ImGui::TableSetupColumn("Pos z");
        ImGui::TableSetupColumn("Ori x");
        ImGui::TableSetupColumn("Ori y");
        ImGui::TableSetupColumn("Ori z");
        ImGui::TableHeadersRow();

        for (int i = 0; i < (int)positions.size(); i++){
            ImGui::PushID(i);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            // clicking a row moves the camera to that waypoint
            if (ImGui::Selectable(std::to_string(i).c_str(), selection == i, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap)){
                if (!ImGui::GetIO().KeyCtrl){
                    selection = i;
                    camera.Position = positions[i];
                    camera.Orientation = orientations[i];
                }
            }
            const glm::vec3& p = positions[i];
            const glm::vec3& o = orientations[i];
            float values[6] = {p.x, p.y, p.z, o.x, o.y, o.z};
            for (int col = 0; col < 6; col++){
                ImGui::TableSetColumnIndex(col + 1);
                ImGui::Text("%.3f", values[col]);
            }
            ImGui::TableSetColumnIndex(7);
            if (ImGui::SmallButton("x")) toDelete = i;
            ImGui::SameLine();
            if (ImGui::SmallButton("up")) toMoveUp = i;
            ImGui::SameLine();
            if (ImGui::SmallButton("down")) toMoveDown = i;
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    // Apply edits after the table has been drawn
    if (toDelete >= 0){
        positions.erase(positions.begin() + toDelete);
        orientations.erase(orientations.begin() + toDelete);
        if (toDelete <= selection)
            selection--;
    }
    if (toMoveUp >= 1){
        std::swap(positions[toMoveUp], positions[toMoveUp - 1]);
        std::swap(orientations[toMoveUp], orientations[toMoveUp - 1]);
    }
    if (toMoveDown >= 0 && toMoveDown + 1 < (int)positions.size()){
        std::swap(positions[toMoveDown], positions[toMoveDown + 1]);
        std::swap(orientations[toMoveDown], orientations[toMoveDown + 1]);
    }
}

void CameraPath::update(Camera& camera, int framebufferWidth, int framebufferHeight)
{
    if (request != ReplayRequest::None && !isReplaying() && positions.size() > 1){
        int last = (int)positions.size() - 1;
        int first = 0;
        if (request == ReplayRequest::FromSelection || request == ReplayRequest::OneStep)
            first = (selection >= 0 && selection <= last) ? selection : 0;   // no selection: from the start
        if (request == ReplayRequest::OneStep && first < last)
            last = first + 1;
        startReplay(first, last, framebufferWidth, framebufferHeight);
    }
    request = ReplayRequest::None;

    if (isReplaying()){
        // blend between the two neighboring steps (live replay is between frames)
        int last = (int)interpPositions.size() - 1;
        int i = std::min((int)replayPos, last);
        int j = std::min(i + 1, last);
        float f = (float)(replayPos - i);
        camera.Position = glm::mix(interpPositions[i], interpPositions[j], f);
        glm::vec3 o = glm::mix(interpOrientations[i], interpOrientations[j], f);
        camera.Orientation = glm::length(o) > 1e-4f ? glm::normalize(o) : interpOrientations[i];
    }
}

void CameraPath::afterRender(float deltaTime)
{
    if (!isReplaying())
        return;

    if (writer.isOpened()){
        // read the back buffer before it is swapped (its content is undefined afterwards)
        cv::Mat frame(recordSize.height, recordSize.width, CV_8UC3);
        glPixelStorei(GL_PACK_ALIGNMENT, (frame.step & 3) ? 1 : 4);
        glPixelStorei(GL_PACK_ROW_LENGTH, (GLint)(frame.step / frame.elemSize()));
        glReadPixels(0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, frame.data);
        glPixelStorei(GL_PACK_ROW_LENGTH, 0);
        cv::Mat flipped;
        cv::flip(frame, flipped, 0);
        writer.write(flipped);
    }

    // the last pose has been shown: done
    double last = (double)(interpPositions.size() - 1);
    if (replayPos >= last){
        replayPos = -1.0;
        if (writer.isOpened()){
            writer.release();
            std::cout << "Recording finished" << std::endl;
        }
        return;
    }
    if (writer.isOpened())
        replayPos += 1.0;                                   // every step becomes one video frame
    else
        replayPos += (double)deltaTime * kVideoFps;         // live: same speed as in the video
    replayPos = std::min(replayPos, last);                  // always show the final pose
}

void CameraPath::startReplay(int first, int last, int framebufferWidth, int framebufferHeight)
{
    interpPositions.clear();
    interpOrientations.clear();
    for (int i = first; i < last; i++)
        appendSegment(positions[i], orientations[i], positions[i + 1], orientations[i + 1]);
    if (loop)
        appendSegment(positions.back(), orientations.back(), positions.front(), orientations.front());
    else{
        // also show the last waypoint itself
        interpPositions.push_back(positions[last]);
        interpOrientations.push_back(glm::normalize(orientations[last]));
    }
    replayPos = 0.0;

    if (record)
        openVideo(framebufferWidth, framebufferHeight);
}

// Interpolates from (p0, o0) to (p1, o1), excluding the end pose. The number of
// steps depends on both distance and rotation, so pure rotations are replayed too.
void CameraPath::appendSegment(const glm::vec3& p0, const glm::vec3& o0, const glm::vec3& p1, const glm::vec3& o1)
{
    glm::vec3 n0 = glm::normalize(o0);
    glm::vec3 n1 = glm::normalize(o1);
    float angleDeg = glm::degrees(std::acos(glm::clamp(glm::dot(n0, n1), -1.0f, 1.0f)));
    int steps = std::max({1,
                          (int)std::ceil(glm::length(p1 - p0) / kFrameDistance),
                          (int)std::ceil(angleDeg / kFrameAngleDeg)});
    for (int i = 0; i < steps; i++){
        float t = (float)i / steps;
        interpPositions.push_back(glm::mix(p0, p1, t));
        glm::vec3 o = glm::mix(n0, n1, t);
        // opposite directions would give a zero vector halfway: keep the previous one
        interpOrientations.push_back(glm::length(o) > 1e-4f ? glm::normalize(o) : n0);
    }
}

void CameraPath::openVideo(int width, int height)
{
    char timestamp[50];
    std::time_t now = std::time(nullptr);
    std::strftime(timestamp, sizeof(timestamp), "%Y_%m_%d_%H_%M_%S", std::localtime(&now));
    std::string dir = recordsDir();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    std::string filename = dir + "/record_" + timestamp + ".avi";

    recordSize = cv::Size(width, height);
    writer.open(filename, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), kVideoFps, recordSize, true);
    if (!writer.isOpened()){
        // replay anyway, just without recording
        std::cerr << "Could not open the output video file for write: " << filename << std::endl;
        record = false;
    }
    else
        std::cout << "Recording to " << filename << std::endl;
}
