#version 330 core
// Sphere per point: analytic ray-sphere intersection. The color changes all
// around the sphere with the surface normal, blending the same six axis colors
// as the cube shader (+X red, -X cyan, +Y green, -Y magenta, +Z blue, -Z yellow).
out vec4 FragColor;

flat in vec3 sphereCenter;

#include "common.glsl"
#include "sphere.glsl"

void main()
{
	vec3 nor, rd;
	float depth;
	if (!raycastSphere(sphereCenter, nor, rd, depth))
		discard;

	// Blend the six axis colors by the squared normal components (sum to 1)
	vec3 n2 = nor * nor;
	vec3 col = n2.x * (nor.x > 0.0 ? vec3(0.90, 0.20, 0.20) : vec3(0.20, 0.80, 0.80))
	         + n2.y * (nor.y > 0.0 ? vec3(0.25, 0.85, 0.25) : vec3(0.80, 0.25, 0.80))
	         + n2.z * (nor.z > 0.0 ? vec3(0.30, 0.50, 1.00) : vec3(0.95, 0.85, 0.20));

	float brightness = 1.0 - logisticDepth(depth) * useDepthOnPointBrightness;
	FragColor = vec4(col * brightness, 1.0);
}
