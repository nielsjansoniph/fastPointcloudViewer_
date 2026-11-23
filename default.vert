#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 pointPosition;

uniform mat4 camMatrix;
// Model rotation (identity unless the model is rotated in the viewer)
uniform mat4 model = mat4(1.0);

void main()
{
   vec4 worldPos = model * vec4(aPos, 1.0);
   gl_Position = camMatrix * worldPos;
   pointPosition = worldPos.xyz;
   //color = aColor;
}