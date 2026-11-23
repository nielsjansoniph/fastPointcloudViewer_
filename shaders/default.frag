#version 330 core
// Color by distance (hue), round points
#include "common.glsl"

out vec4 FragColor;
in vec3 pointPosition;

uniform float cFactor = 1.0;

void main()
{
	float depth = logisticDepth(gl_FragCoord.z);
	float dBrightness = 1.0 - depth * useDepthOnPointBrightness;
	float dSize = 1.0 - depth * useDepthOnPointsize;

	if (dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) > dSize / 4.0)
		discard;

	float z = spriteHeight(dSize);
	float h = depth * cFactor;
	float v = (1.0 - ((1.0 - z) * useShadow)) * dBrightness;
	FragColor = vec4(hsv2rgb(vec3(h, 1.0, v)), 1.0);
}
