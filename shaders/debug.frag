#version 330 core
// Split view: right of `split` color by distance (startColor..endColor),
// left of it gray levels (before / inside / beyond the distance range)
#include "common.glsl"

out vec4 FragColor;
in vec3 pointPosition;

uniform float startDist = 0.0;
uniform float endDist = 0.3;
uniform float split;
uniform vec3 startColor;
uniform vec3 endColor;

void main()
{
	float depth = linearizeDepth(gl_FragCoord.z);
	float dSize = 1.0 - depth * useDepthOnPointsize;

	if (dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) > dSize / 4.0)
		discard;

	if (gl_FragCoord.x > split){
		float r = map(depth, startDist, endDist, startColor.r, endColor.r);
		float g = map(depth, startDist, endDist, startColor.g, endColor.g);
		float b = map(depth, startDist, endDist, startColor.b, endColor.b);
		FragColor = vec4(r, g, b, 1.0);
		if (depth < startDist)
			FragColor = vec4(startColor, 1.0);
		if (depth > endDist)
			FragColor = vec4(endColor, 1.0);
	}
	else{
		FragColor = vec4(vec3(0.5), 1.0);
		if (depth < startDist)
			FragColor = vec4(vec3(0.25), 1.0);
		if (depth > endDist)
			FragColor = vec4(vec3(0.75), 1.0);
	}
}
