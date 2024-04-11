#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 pointPosition;

uniform mat4 camMatrix;

void main()
{
   gl_Position = camMatrix * vec4(aPos, 1.0);
   pointPosition = aPos;
   //color = aColor;
}