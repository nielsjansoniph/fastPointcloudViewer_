// Ray casting of the sphere around a point, shared by sphere.frag and rgbsphere.frag.
// Needs common.glsl (cameraPosition) included before this file.

uniform mat4 camMatrix;
uniform mat4 invCamMatrix;
uniform vec2 viewportSize;
uniform float sphereSize = 0.05;    // diameter in world units

// Intersects the view ray through this pixel with the sphere at center. Returns false
// if it misses (the caller discards); otherwise sets the surface normal, the view ray
// direction and gl_FragDepth of the hit point, and returns its window depth.
bool raycastSphere(vec3 center, out vec3 nor, out vec3 rd, out float depth)
{
	// Ray from the camera through this pixel
	vec2 ndc = gl_FragCoord.xy / viewportSize * 2.0 - 1.0;
	vec4 farPoint = invCamMatrix * vec4(ndc, 1.0, 1.0);
	vec3 ro = cameraPosition;
	rd = normalize(farPoint.xyz / farPoint.w - ro);

	// Ray-sphere intersection
	float r = 0.5 * sphereSize;
	vec3 oc = ro - center;
	float b = dot(oc, rd);
	float c = dot(oc, oc) - r * r;
	float h = b * b - c;
	nor = vec3(0.0);
	depth = 1.0;
	if (h < 0.0)
		return false;          // missed
	float t = -b - sqrt(h);
	if (t <= 0.0)
		return false;          // camera inside the sphere or sphere behind it

	vec3 hit = ro + t * rd;
	nor = (hit - center) / r;

	// Depth of the hit point instead of the sprite center
	vec4 clip = camMatrix * vec4(hit, 1.0);
	depth = clip.z / clip.w * 0.5 + 0.5;
	gl_FragDepth = depth;
	return true;
}
