#version 330 core
out vec4 FragColor;

uniform vec3 color = vec3(1.0f);
uniform float near = 0.01f;
uniform float far = 10.0f;
uniform float useDepthOnPointsize = 1.0;
uniform float useDepthOnPointBrightness = 1.0;

float linearizeDepth(float depth)
{
	return (2.0 * near * far) / (far + near - (depth * 2.0 - 1.0) * (far - near));
}

float logisticDepth(float depth, float steepness = 0.5f, float offset = 5.0f)
{
	float zVal = linearizeDepth(depth);
	return (1 / (1 + exp(-steepness * (zVal - offset))));
}

void main()
{
   	float depth = logisticDepth(gl_FragCoord.z);
	float dBrightness = 1.0-depth*useDepthOnPointBrightness;
	float dSize = 1.0-depth*useDepthOnPointsize;
	//if(dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) > 0.25)
	if(dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) > dSize/4.0)
		discard;
	else{
		float z = sqrt(1 - gl_PointCoord.x*gl_PointCoord.x - gl_PointCoord.y*gl_PointCoord.y);
   		//FragColor = vec4(vec3(-depth+1.0)*(color+1.0)/2.0, 1.0);
		FragColor = vec4(vec3(z)*dBrightness, 1.0);
	}	

}