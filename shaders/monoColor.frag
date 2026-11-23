#version 330 core
// One color for all points, round points
#include "common.glsl"

out vec4 FragColor;
in vec3 pointPosition;

uniform vec3 color = vec3(1.0);

void main()
{
	float depth = logisticDepth(gl_FragCoord.z);
	float dBrightness = 1.0 - depth * useDepthOnPointBrightness;
	float dSize = 1.0 - depth * useDepthOnPointsize;

	if (dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) > dSize / 4.0)
		discard;

	float z = spriteHeight(dSize);
	FragColor = vec4(vec3(1.0 - ((1.0 - z) * useShadow)) * dBrightness * color, 1.0);
}
