#version 330 core
// Colors stored in the point cloud file, round points; with "Use shadow" they are
// shaded like small spheres, "Use depth for point brightness" darkens distant points
#include "common.glsl"

out vec4 FragColor;
in vec3 pointPosition;
in vec3 pointColor;

void main()
{
	float depth = logisticDepth(gl_FragCoord.z);
	float dBrightness = 1.0 - depth * useDepthOnPointBrightness;
	float dSize = 1.0 - depth * useDepthOnPointsize;

	if (dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) > dSize / 4.0)
		discard;

	float z = spriteHeight(dSize);
	float shade = 1.0 - (1.0 - z) * useShadow;
	FragColor = vec4(pointColor * shade * dBrightness, 1.0);
}
