#version 330 core
// Split view: right of `split` hue by height z (startDist..endDist),
// left of it gray levels (below / inside / above the height range)
#include "common.glsl"

out vec4 FragColor;
in vec3 pointPosition;

uniform float startDist = 0.0;
uniform float endDist = 0.3;
uniform float split;

void main()
{
	float depth = logisticDepth(gl_FragCoord.z);
	float dBrightness = 1.0 - depth * useDepthOnPointBrightness;
	float dSize = 1.0 - depth * useDepthOnPointsize;

	if (dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) > dSize / 4.0)
		discard;

	if (gl_FragCoord.x > split){
		float h = map(pointPosition.z, startDist, endDist, 0.1, 0.9);
		float z = spriteHeight(dSize);
		float v = (1.0 - ((1.0 - z) * useShadow)) * dBrightness;
		FragColor = vec4(hsv2rgb(vec3(h, 1.0, v)), 1.0);
		if (pointPosition.z < startDist)
			FragColor = vec4(hsv2rgb(vec3(0.1, 1.0, v)), 1.0);
		if (pointPosition.z > endDist)
			FragColor = vec4(hsv2rgb(vec3(0.9, 1.0, v)), 1.0);
	}
	else{
		FragColor = vec4(vec3(0.5), 1.0);
		if (pointPosition.z < startDist)
			FragColor = vec4(vec3(0.25), 1.0);
		if (pointPosition.z > endDist)
			FragColor = vec4(vec3(0.75), 1.0);
	}
}
