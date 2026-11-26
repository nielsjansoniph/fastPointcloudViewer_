#include "Cloud.h"

#include <algorithm>
#include <cmath>

#include "VBO.h"

namespace {

// Frustum planes (a, b, c, d with inside: a*x + b*y + c*z + d >= 0) from a clip matrix
void frustumPlanes(const glm::mat4& m, glm::vec4 planes[6])
{
    glm::vec4 row[4];
    for (int i = 0; i < 4; i++)
        row[i] = glm::vec4(m[0][i], m[1][i], m[2][i], m[3][i]);
    planes[0] = row[3] + row[0];   // left
    planes[1] = row[3] - row[0];   // right
    planes[2] = row[3] + row[1];   // bottom
    planes[3] = row[3] - row[1];   // top
    planes[4] = row[3] + row[2];   // near
    planes[5] = row[3] - row[2];   // far
}

// false if the box is completely outside one of the planes
bool boxInFrustum(const glm::vec4 planes[6], const glm::vec3& lo, const glm::vec3& hi)
{
    for (int i = 0; i < 6; i++){
        const glm::vec4& p = planes[i];
        // corner of the box farthest along the plane normal
        glm::vec3 v(p.x >= 0.0f ? hi.x : lo.x, p.y >= 0.0f ? hi.y : lo.y, p.z >= 0.0f ? hi.z : lo.z);
        if (p.x * v.x + p.y * v.y + p.z * v.z + p.w < 0.0f)
            return false;
    }
    return true;
}

} // namespace

Cloud::Cloud(const std::string& filename, CloudData&& data)
	: filename(filename)
{
	CloudData d = std::move(data);   // freed at the end of the constructor
	numPoints = (GLsizei)d.positions.size();
	hasColor = !d.colors.empty();
	min = d.min;
	max = d.max;
	chunks = std::move(d.chunks);

	// Positions (12 bytes/point) and, if present, colors (4 bytes/point) in separate buffers
	VAO.Bind();
	VBO positionBuffer(d.positions);
	VAO.LinkAttrib(positionBuffer, 0, 3, GL_FLOAT, sizeof(VertexPos), (void*)0);
	if (hasColor){
		VBO colorBuffer(d.colors.data(), d.colors.size() * sizeof(Rgba8));
		VAO.LinkAttrib(colorBuffer, 1, 4, GL_UNSIGNED_BYTE, sizeof(Rgba8), (void*)0, GL_TRUE);
	}
	VAO.Unbind();
	positionBuffer.Unbind();
}

void Cloud::prepare(const CloudView& view)
{
	glm::vec4 planes[6];
	frustumPlanes(view.clipFromObject, planes);

	visible.clear();
	pixels2.clear();
	for (std::uint32_t i = 0; i < (std::uint32_t)chunks.size(); i++){
		const Chunk& c = chunks[i];
		if (!boxInFrustum(planes, c.min, c.max))
			continue;
		float p2 = INFINITY;   // camera inside the box or no LOD: all points
		if (view.lod){
			// projected size of the chunk in pixels: the closer, the more points
			float size = glm::length(c.max - c.min);
			float dist = glm::length(view.cameraPosition - glm::clamp(view.cameraPosition, c.min, c.max));
			if (dist > 0.0f){
				float pixels = size / dist * view.pixelsPerUnit;
				p2 = pixels * pixels;
			}
		}
		visible.push_back(i);
		pixels2.push_back(p2);
	}
}

double Cloud::pointsForDetail(float detail) const
{
	double total = 0.0;
	for (std::size_t v = 0; v < visible.size(); v++)
		total += std::clamp(std::ceil(detail * pixels2[v]), 1.0f, (float)chunks[visible[v]].count);
	return total;
}

void Cloud::Draw(float detail, float fullDetail, float pointSize, float thinnedPointSize)
{
	for (int k = 0; k < 2; k++){
		drawFirst[k].clear();
		drawCount[k].clear();
	}
	drawnPoints = 0;
	for (std::size_t v = 0; v < visible.size(); v++){
		const Chunk& c = chunks[visible[v]];
		float n = std::clamp(std::ceil(detail * pixels2[v]), 1.0f, (float)c.count);
		float full = std::clamp(std::ceil(fullDetail * pixels2[v]), 1.0f, (float)c.count);
		int k = n < full ? 1 : 0;
		drawFirst[k].push_back((GLint)c.first);
		drawCount[k].push_back((GLsizei)n);
		drawnPoints += (std::size_t)n;
	}

	if (!hasColor)
		glVertexAttrib4f(1, 1.0f, 1.0f, 1.0f, 1.0f);   // shaders reading the color get white
	VAO.Bind();
	for (int k = 0; k < 2; k++){
		if (drawFirst[k].empty())
			continue;
		if (pointSize > 0.0f)
			glPointSize(k == 0 ? pointSize : thinnedPointSize);
		glMultiDrawArrays(GL_POINTS, drawFirst[k].data(), drawCount[k].data(), (GLsizei)drawFirst[k].size());
	}
	VAO.Unbind();
}
