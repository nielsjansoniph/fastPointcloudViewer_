#version 330 core
// Axis-aligned cube per point: analytic ray-box intersection (slab test).
// Each face gets its own color; correct depth is written so cubes occlude each other.
out vec4 FragColor;

flat in vec3 cubeCenter;

uniform mat4 camMatrix;
uniform mat4 invCamMatrix;
uniform vec2 viewportSize;
uniform vec3 cameraPosition;
uniform float cubeSize = 0.05;
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

	// Slab test against the box [center - h, center + h]
	vec3 m = 1.0 / rd;
	vec3 n = m * (ro - cubeCenter);
	vec3 k = abs(m) * (0.5 * cubeSize);
	vec3 t1 = -n - k;
	vec3 t2 = -n + k;
	float tN = max(max(t1.x, t1.y), t1.z);
	float tF = min(min(t2.x, t2.y), t2.z);
	if (tN > tF || tN <= 0.0)
		discard;   // missed, or camera inside the cube

	// Face normal: the axis whose entry plane was hit last
	vec3 nor = -sign(rd) * step(vec3(tN), t1);

	vec3 col;
	if      (nor.x >  0.5) col = vec3(0.90, 0.20, 0.20);   // +X red
	else if (nor.x < -0.5) col = vec3(0.20, 0.80, 0.80);   // -X cyan
	else if (nor.y >  0.5) col = vec3(0.25, 0.85, 0.25);   // +Y green
	else if (nor.y < -0.5) col = vec3(0.80, 0.25, 0.80);   // -Y magenta
	else if (nor.z >  0.5) col = vec3(0.30, 0.50, 1.00);   // +Z blue
	else                   col = vec3(0.95, 0.85, 0.20);   // -Z yellow

	// Depth of the hit point instead of the sprite center
	vec3 hit = ro + tN * rd;
	vec4 clip = camMatrix * vec4(hit, 1.0);
	float depth = clip.z / clip.w * 0.5 + 0.5;
	gl_FragDepth = depth;

	float brightness = 1.0 - logisticDepth(depth) * useDepthOnPointBrightness;
	FragColor = vec4(col * brightness, 1.0);
}
