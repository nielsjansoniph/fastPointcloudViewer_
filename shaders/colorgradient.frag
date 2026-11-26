#version 330 core
// Hue gradient by distance, limited by startDist/endDist, round points.
// Brightness fades from startBrightness (near) to endBrightness (far).
#include "common.glsl"

out vec4 FragColor;
in vec3 pointPosition;

uniform float cFactor = 1.0;
uniform float startDist = 0.0;
uniform float endDist = 0.3;
uniform float startBrightness = 1.0;
uniform float endBrightness = 1.0;

void main()
{
	float depth = logisticDepth(gl_FragCoord.z);
	float dBrightness = mix(1.0, mix(startBrightness, endBrightness, depth), useDepthOnPointBrightness);
	float dSize = 1.0 - depth * useDepthOnPointsize;

	if (dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) > dSize / 4.0)
		discard;

	float z = spriteHeight(dSize);
	float h = depth * cFactor;
	h = clamp(h, 0.0, endDist);
	h = h + startDist;
	float v = (1.0 - ((1.0 - z) * useShadow)) * dBrightness;
	FragColor = vec4(hsv2rgb(vec3(h, 1.0, v)), 1.0);
}
