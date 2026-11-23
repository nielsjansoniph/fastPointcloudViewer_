// Shared uniforms and helper functions for the point fragment shaders.
// Included with  #include "common.glsl"  (resolved by Shader.cpp, must follow #version).

// Set by Camera::Matrix() for every point shader
uniform float near = 0.01;
uniform float far = 10.0;
uniform float useDepthOnPointsize = 1.0;
uniform float useDepthOnPointBrightness = 1.0;
uniform float useShadow = 1.0;
uniform vec3 cameraPosition;

// Window depth (gl_FragCoord.z) -> distance, using the Near/Far sliders
float linearizeDepth(float depth)
{
	return (2.0 * near * far) / (far + near - (depth * 2.0 - 1.0) * (far - near));
}

// 0 (near) .. 1 (far), used for the depth dependent brightness
float logisticDepth(float depth)
{
	const float steepness = 0.5;
	const float offset = 5.0;
	float zVal = linearizeDepth(depth);
	return 1.0 / (1.0 + exp(-steepness * (zVal - offset)));
}

vec3 hsv2rgb(vec3 c)
{
	vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
	vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
	return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

// Linear mapping of value from [min1, max1] to [min2, max2]
float map(float value, float min1, float max1, float min2, float max2)
{
	return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

// Pseudo height of a round point sprite at the current fragment (1 in the center,
// 0 at the border) for the "Use shadow" shading; dSize = relative sprite size.
float spriteHeight(float dSize)
{
	vec2 d = gl_PointCoord - 0.5;
	return sqrt(dSize - d.x * d.x * 4.0 / dSize / dSize - d.y * d.y * 4.0 / dSize / dSize);
}
