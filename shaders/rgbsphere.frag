#version 330 core
// Sphere per point (ray-cast like sphere.frag) in the color stored in the file.
// "Use shadow" shades it with a light at the camera, "Use depth for point
// brightness" darkens distant spheres.
out vec4 FragColor;

flat in vec3 sphereCenter;
flat in vec3 sphereColor;

#include "common.glsl"
#include "sphere.glsl"

void main()
{
	vec3 nor, rd;
	float depth;
	if (!raycastSphere(sphereCenter, nor, rd, depth))
		discard;

	float diffuse = max(dot(nor, -rd), 0.0);
	float shade = mix(1.0, 0.25 + 0.75 * diffuse, useShadow);
	float brightness = 1.0 - logisticDepth(depth) * useDepthOnPointBrightness;
	FragColor = vec4(sphereColor * shade * brightness, 1.0);
}
