#version 330 core
// Renders each point as a screen-aligned sprite that is large enough to
// contain the projected sphere; sphere.frag ray-casts the actual sphere in it.
layout (location = 0) in vec3 aPos;

flat out vec3 sphereCenter;

uniform mat4 camMatrix;
uniform mat4 model = mat4(1.0);
uniform float sphereSize = 0.05;    // diameter in world units
uniform float projScale = 1000.0;   // viewport height in pixels * projection[1][1]

void main()
{
   vec4 worldPos = model * vec4(aPos, 1.0);
   sphereCenter = worldPos.xyz;
   gl_Position = camMatrix * worldPos;

   // Projected diameter of the sphere (+ margin for perspective at the screen border)
   float r = 0.5 * sphereSize;
   float w = gl_Position.w;
   gl_PointSize = 1.3 * r * projScale / sqrt(max(w * w - r * r, 1e-6));
}
