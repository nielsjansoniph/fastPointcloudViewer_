#pragma once
// Reading point cloud files on a background thread and sorting the points into
// chunks for frustum culling and level of detail. No OpenGL in here.
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "VBO.h"

struct Rgba8 { std::uint8_t r, g, b, a; };

// A block of points that are close together; the points of a chunk are stored
// contiguously and in random order, so any prefix is an even subsample (LOD).
struct Chunk
{
    std::uint32_t first = 0;      // index of the first point in the vertex buffer
    std::uint32_t count = 0;
    glm::vec3 min = glm::vec3(0.0f);   // tight bounding box
    glm::vec3 max = glm::vec3(0.0f);
};

// CPU side result of loading a file, uploaded to the GPU by Cloud
struct CloudData
{
    std::vector<VertexPos> positions;
    std::vector<Rgba8> colors;        // empty if the file has no colors
    glm::vec3 min = glm::vec3(0.0f);
    glm::vec3 max = glm::vec3(0.0f);
    std::vector<Chunk> chunks;
};

// Shared between the loading thread and the UI
struct LoadProgress
{
    std::atomic<float> fraction{-1.0f};          // 0..1, < 0: unknown (PCL reader)
    std::atomic<const char*> stage{"Reading"};   // static strings only
    std::atomic<bool> cancel{false};             // set by the UI; the loader throws LoadCancelled
};

struct LoadCancelled {};

// Reads .ply, .pcd or .obj and builds the chunks. Throws std::runtime_error on failure
// and LoadCancelled if progress.cancel is set. Safe to run on any thread.
CloudData loadCloudData(const std::string& filename, LoadProgress& progress);
