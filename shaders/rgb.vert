#version 330 core
// Like points.vert, additionally passes the per-point color (attribute 1)
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColor;

out vec3 pointPosition;
out vec3 pointColor;

uniform mat4 camMatrix;
uniform mat4 model = mat4(1.0);

void main()
{
   vec4 worldPos = model * vec4(aPos, 1.0);
   gl_Position = camMatrix * worldPos;
   pointPosition = worldPos.xyz;
   pointColor = aColor.rgb;
}
