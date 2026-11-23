#version 330 core
// Sphere per point: analytic ray-sphere intersection. The color changes all
// around the sphere with the surface normal, blending the same six axis colors
// as the cube shader (+X red, -X cyan, +Y green, -Y magenta, +Z blue, -Z yellow).
out vec4 FragColor;

flat in vec3 sphereCenter;

uniform mat4 camMatrix;
uniform mat4 invCamMatrix;
uniform vec2 viewportSize;
uniform vec3 cameraPosition;
uniform float sphereSize = 0.05;
uniform float useDepthOnPointBrightness = 1.0;
uniform float near = 0.01f;
uniform float far = 10.0f;

// Same depth-to-brightness mapping as the other point shaders (reacts to the Near/Far sliders)
float linearizeDepth(float depth)
{
	return (2.0 * near * far) / (far + near - (depth * 2.0 - 1.0) * (far - near));
}

float logisticDepth(float depth)
{
	const float steepness = 0.5f;
	const float offset = 5.0f;
	float zVal = linearizeDepth(depth);
	return (1 / (1 + exp(-steepness * (zVal - offset))));
}

void main()
{
	// Ray from the camera through this pixel
	vec2 ndc = gl_FragCoord.xy / viewportSize * 2.0 - 1.0;
	vec4 farPoint = invCamMatrix * vec4(ndc, 1.0, 1.0);
	vec3 ro = cameraPosition;
	vec3 rd = normalize(farPoint.xyz / farPoint.w - ro);

	// Ray-sphere intersection
	float r = 0.5 * sphereSize;
	vec3 oc = ro - sphereCenter;
	float b = dot(oc, rd);
	float c = dot(oc, oc) - r * r;
	float h = b * b - c;
	if (h < 0.0)
		discard;               // missed
	float t = -b - sqrt(h);
	if (t <= 0.0)
		discard;               // camera inside the sphere or sphere behind it

	vec3 hit = ro + t * rd;
	vec3 nor = (hit - sphereCenter) / r;

	// Blend the six axis colors by the squared normal components (sum to 1)
	vec3 n2 = nor * nor;
	vec3 col = n2.x * (nor.x > 0.0 ? vec3(0.90, 0.20, 0.20) : vec3(0.20, 0.80, 0.80))
	         + n2.y * (nor.y > 0.0 ? vec3(0.25, 0.85, 0.25) : vec3(0.80, 0.25, 0.80))
	         + n2.z * (nor.z > 0.0 ? vec3(0.30, 0.50, 1.00) : vec3(0.95, 0.85, 0.20));

	// Depth of the hit point instead of the sprite center
	vec4 clip = camMatrix * vec4(hit, 1.0);
	float depth = clip.z / clip.w * 0.5 + 0.5;
	gl_FragDepth = depth;

	float brightness = 1.0 - logisticDepth(depth) * useDepthOnPointBrightness;
	FragColor = vec4(col * brightness, 1.0);
}
