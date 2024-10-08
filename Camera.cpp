#include"Camera.h"


Camera::Camera(int width, int height, glm::vec3 position)
{
	Camera::width = width;
	Camera::height = height;
	Position = position;
}

void Camera::updateMatrix(float FOVdeg, float nearPlane, float farPlane)
{
	// Initializes matrices since otherwise they will be the null matrix
	glm::mat4 view = glm::mat4(1.0f);
	glm::mat4 projection = glm::mat4(1.0f);

	// Makes camera look in the right direction from the right position
	view = glm::lookAt(Position, Position + Orientation, Up);
	// Adds perspective to the scene
	projection = glm::perspective(glm::radians(FOVdeg), (float)width / height, nearPlane, farPlane);

	// Sets new camera matrix
	cameraMatrix = projection * view;
}

void Camera::Matrix(Shader& shader, const char* uniform)
{
	// Exports the camera matrix to the Vertex Shader
	glUniformMatrix4fv(glGetUniformLocation(shader.ID, uniform), 1, GL_FALSE, glm::value_ptr(cameraMatrix));

	GLuint nearUniID = glGetUniformLocation(shader.ID, "near"); 
	glUniform1f(nearUniID, nearDist);

    GLuint farUniID = glGetUniformLocation(shader.ID, "far"); 
	glUniform1f(farUniID, farDist);

	GLuint colorID = glGetUniformLocation(shader.ID, "color");
	//glUniform3fv(colorID, 1, glm::value_ptr(Orientation));

	GLuint camPosID = glGetUniformLocation(shader.ID, "cameraPosition");
	glUniform3fv(camPosID, 1, glm::value_ptr(Position));

	if (useDepthOnPointsize)
		glUniform1f(glGetUniformLocation(shader.ID, "useDepthOnPointsize"), 1.0f);
	else
		glUniform1f(glGetUniformLocation(shader.ID, "useDepthOnPointsize"), 0.0f);
	
	if (useDepthOnPointBrightness)
		glUniform1f(glGetUniformLocation(shader.ID, "useDepthOnPointBrightness"), 1.0f);
	else
		glUniform1f(glGetUniformLocation(shader.ID, "useDepthOnPointBrightness"), 0.0f);

	if (useShadow)
		glUniform1f(glGetUniformLocation(shader.ID, "useShadow"), 1.0f);
	else
		glUniform1f(glGetUniformLocation(shader.ID, "useShadow"), 0.0f);

}

void Camera::forward(){
	Position += speed * Orientation;
}
void Camera::backward(){
	Position += speed * -Orientation;;
}
void Camera::left(){
	Position += speed * -glm::normalize(glm::cross(Orientation, Up));
}
void Camera::right(){
	Position += speed * glm::normalize(glm::cross(Orientation, Up));
}
void Camera::up(){
	Position += speed * Up;
}
void Camera::down(){
	Position += speed * -Up;
}

void Camera::Inputs(GLFWwindow* window)
{

	// Handles mouse inputs
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	{
		// Hides mouse cursor
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

		// Prevents camera from jumping on the first click
		if (firstClick)
		{
			glfwSetCursorPos(window, (width / 2), (height / 2));
			firstClick = false;
		}

		// Stores the coordinates of the cursor
		double mouseX;
		double mouseY;
		// Fetches the coordinates of the cursor
		glfwGetCursorPos(window, &mouseX, &mouseY);

		// Normalizes and shifts the coordinates of the cursor such that they begin in the middle of the screen
		// and then "transforms" them into degrees 
		float rotX = sensitivity * (float)(mouseY - (height / 2)) / height;
		float rotY = sensitivity * (float)(mouseX - (width / 2)) / width;

		// Calculates upcoming vertical change in the Orientation
		glm::vec3 newOrientation = glm::rotate(Orientation, glm::radians(-rotX), glm::normalize(glm::cross(Orientation, Up)));

		// Decides whether or not the next vertical Orientation is legal or not
		if (abs(glm::angle(newOrientation, Up) - glm::radians(90.0f)) <= glm::radians(85.0f))
		{
			Orientation = newOrientation;
		}

		// Rotates the Orientation left and right
		Orientation = glm::rotate(Orientation, glm::radians(-rotY), Up);

		// Sets mouse cursor to the middle of the screen so that it doesn't end up roaming around
		glfwSetCursorPos(window, (width / 2), (height / 2));
	}
	else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
	{
		// Unhides cursor since camera is not looking around anymore
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		// Makes sure the next time the camera looks around it doesn't jump
		firstClick = true;
	}
}