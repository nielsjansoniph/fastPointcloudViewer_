#pragma once
#include <glm/glm.hpp>
#include <string>
#include <vector>

#include "CloudLoader.h"
#include "VAO.h"

// What the cloud needs to know about the view to cull and thin out its chunks
struct CloudView
{
    glm::mat4 clipFromObject;      // camera matrix * model matrix
    glm::vec3 cameraPosition;      // in object (file) coordinates
    float pixelsPerUnit;           // projected size of 1 m at 1 m distance, in pixels
    bool lod = true;               // false: all points of the visible chunks
};

// A point cloud loaded from .ply, .pcd or .obj (vertices only). The points live only on the GPU,
// sorted into chunks (see CloudLoader.h); the CPU side keeps the chunk boxes and display settings.
class Cloud
{
public:
	// Uploads the loaded data to the GPU (needs the OpenGL context); data is consumed
	Cloud(const std::string& filename, CloudData&& data);

	// Frustum culling: selects the visible chunks and their projected size for this frame
	void prepare(const CloudView& view);

	// Level of detail: a visible chunk draws min(all its points, detail * (projected size in
	// pixels)^2) points, i.e. close chunks are complete and distant ones are thinned out.
	// Returns the number of points the prepared chunks would draw with this detail.
	double pointsForDetail(float detail) const;

	// Draws the prepared chunks with the given detail (see pointsForDetail). Chunks that get
	// fewer points than with fullDetail (thinned out by the point budget) are drawn with
	// thinnedPointSize to close the gaps, the others with pointSize. A point size of 0 leaves
	// it to the shader (cube/sphere). The shader must already be active and set up.
	void Draw(float detail, float fullDetail, float pointSize, float thinnedPointSize);

	std::string filename;
	GLsizei numPoints = 0;
	std::size_t numChunks() const { return chunks.size(); }
	std::size_t drawnPoints = 0;       // points drawn in the last frame
	bool hasColor = false;             // file contains rgb(a) colors (uploaded as vertex attribute 1)
	glm::vec3 min = glm::vec3(0.0f);   // bounding box in file coordinates
	glm::vec3 max = glm::vec3(0.0f);

	// Display settings
	int shaderType = 1;                // index into the point shader list (see PointShaders.h)
	float pointssize = 5;              // pixels; cube/sphere shader: size in cm
	glm::vec3 point_color = glm::vec3(1.0f);

private:
	::VAO VAO;
	std::vector<Chunk> chunks;
	std::vector<std::uint32_t> visible;   // indices of the visible chunks (prepare)
	std::vector<float> pixels2;           // squared projected size per visible chunk (prepare)
	std::vector<GLint> drawFirst[2];      // glMultiDrawArrays arguments: complete / thinned chunks
	std::vector<GLsizei> drawCount[2];
};
