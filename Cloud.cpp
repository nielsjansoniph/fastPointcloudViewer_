#include "Cloud.h"


Cloud::Cloud(std::vector <VertexPosCol>& vertices)
{
	numPoints = (GLsizei)vertices.size();

	VAO.Bind();
	// Generates Vertex Buffer Object and links it to vertices
	VBO VBO(vertices);
	// Links VBO attributes such as coordinates and colors to VAO
    VAO.LinkAttrib(VBO, 0, 3, GL_FLOAT, sizeof(VertexPosCol), (void*)0);
    VAO.LinkAttrib(VBO, 1, 3, GL_FLOAT, sizeof(VertexPosCol), (void*)(3 * sizeof(float)));
    // Unbind all to prevent accidentally modifying them
	VAO.Unbind();
	VBO.Unbind();
}




Cloud::Cloud(const std::string filename){
	Cloud::filename = filename;

	std::vector<VertexPos> positions;
	{
		// PCL cloud only lives in this scope, so its memory is freed before the upload finishes
		pcl::PointCloud<pcl::PointXYZ> tmp;
		std::string ext = std::filesystem::path(filename).extension().string();
		for (auto& ch : ext) ch = (char)std::tolower((unsigned char)ch);

		int result = -1;
		if (ext == ".ply"){
			pcl::PLYReader reader;
			result = reader.read(filename, tmp);
		}
		else if (ext == ".pcd"){
			pcl::PCDReader reader;
			result = reader.read(filename, tmp);
		}
		else
			throw std::runtime_error("unsupported file type: " + filename);

		if (result < 0)
			throw std::runtime_error("could not read " + filename);
		if (tmp.points.empty())
			throw std::runtime_error("no points in " + filename);

		pcl::PointXYZ minPt, maxPt;
		pcl::getMinMax3D(tmp, minPt, maxPt);
		min.position = glm::vec3(minPt.x, minPt.y, minPt.z);
		max.position = glm::vec3(maxPt.x, maxPt.y, maxPt.z);

		// Positions only (12 bytes/point); the shaders do not use a per-point color
		positions.resize(tmp.points.size());
		for (size_t i = 0; i < tmp.points.size(); i++)
			positions[i].position = glm::vec3(tmp.points[i].x, tmp.points[i].y, tmp.points[i].z);
	}
	numPoints = (GLsizei)positions.size();

	VAO.Bind();
	VBO VBO(positions);
	VAO.LinkAttrib(VBO, 0, 3, GL_FLOAT, sizeof(VertexPos), (void*)0);
	VAO.Unbind();
	VBO.Unbind();
	// positions goes out of scope here: the data is kept on the GPU only
}


void Cloud::Draw(Camera& camera)
{
	// Bind shader to be able to access uniforms
	currentShader->Activate();
	//shader.Activate();
	VAO.Bind();

	// Keep track of how many of each type of textures we have
	
	// Take care of the camera Matrix
	glUniform3f(glGetUniformLocation(currentShader->ID, "camPos"), camera.Position.x, camera.Position.y, camera.Position.z);
	//camera.Matrix(shader, "camMatrix");
    camera.Matrix(*currentShader, "camMatrix");

	// Draw the actual mesh
	glDrawArrays(GL_POINTS, 0, numPoints);
	//glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
}

void Cloud::DrawElements(Camera& camera)
{
	// Bind shader to be able to access uniforms
	currentShader->Activate();
	//shader.Activate();
	VAO.Bind();

	// Keep track of how many of each type of textures we have
	
	// Take care of the camera Matrix
	glUniform3f(glGetUniformLocation(currentShader->ID, "camPos"), camera.Position.x, camera.Position.y, camera.Position.z);
	//camera.Matrix(shader, "camMatrix");
    camera.Matrix(*currentShader, "camMatrix");

	// Draw the actual mesh
	glDrawArrays(GL_POINTS, 0, numPoints);
	//glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
}