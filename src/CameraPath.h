#pragma once
// Camera tracking shots: waypoints, interpolated replay and video recording
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/videoio.hpp>

#include "Camera.h"

class CameraPath
{
public:
    std::vector<glm::vec3> positions;      // waypoints
    std::vector<glm::vec3> orientations;
    bool loop = true;                      // return to the first waypoint at the end
    bool record = false;                   // write the replay to a video in records/

    bool isReplaying() const { return replayPos >= 0.0; }
    bool isRecording() const { return writer.isOpened(); }

    // Appends the current camera pose unless it equals the last waypoint
    void addWaypointIfChanged(const Camera& camera);

    // Replay buttons and waypoint table (inside an ImGui window)
    void drawUi(Camera& camera);

    // Starts a requested replay and moves the camera to the current replay pose.
    // framebufferWidth/Height = size of the recorded video.
    void update(Camera& camera, int framebufferWidth, int framebufferHeight);

    // After the frame is rendered (before swapping buffers): records it and advances the replay
    // by one video frame (recording) or by deltaTime (live, same speed as in the video)
    void afterRender(float deltaTime);

private:
    enum class ReplayRequest { None, All, FromSelection, OneStep };
    ReplayRequest request = ReplayRequest::None;
    int selection = -1;

    std::vector<glm::vec3> interpPositions;
    std::vector<glm::vec3> interpOrientations;
    // Position in interpPositions (fractional when replaying live): one step per video frame
    // while recording, otherwise advanced by the frame time at the video frame rate.
    double replayPos = -1.0;               // < 0: not replaying

    cv::VideoWriter writer;
    cv::Size recordSize;

    void startReplay(int first, int last, int framebufferWidth, int framebufferHeight);
    void appendSegment(const glm::vec3& p0, const glm::vec3& o0, const glm::vec3& p1, const glm::vec3& o1);
    void openVideo(int width, int height);
};
