#version 330 core
out vec4 FragColor;

in vec3 pointPosition;

uniform vec3 color = vec3(1.0f);
uniform float near = 0.01f;
uniform float far = 10.0f;
uniform float useDepthOnPointsize = 1.0;
uniform float useDepthOnPointBrightness = 1.0;
uniform float useShadow = 1.0;
uniform vec3 cameraPosition;
uniform float cFactor = 1.0;


float linearizeDepth(float depth)
{
	return (2.0 * near * far) / (far + near - (depth * 2.0 - 1.0) * (far - near));
}

float logisticDepth(float depth, float steepness = 0.5f, float offset = 5.0f)
{
	float zVal = linearizeDepth(depth);
	return (1 / (1 + exp(-steepness * (zVal - offset))));
}

vec3 rotY(vec3 v, float a){
    float ca = cos(a);
    float sa = sin(a);
    
    return vec3(ca * v.x - sa * v.z, v.y, sa * v.x + ca * v.z);
}

vec3 rotX(vec3 v, float a){
    float ca = cos(a);
    float sa = sin(a);
    
    return vec3(v.x, ca * v.y + sa * v.z, -sa * v.y + ca * v.z);
}

float sphIntersect( vec3 ro, vec3 rd, vec4 sph )
{
    vec3 oc = ro - sph.xyz;
    float b = dot( oc, rd );
    float c = dot( oc, oc ) - sph.w*sph.w;
    float h = b*b - c;
    if( h<0.0 ) return -1.0;
    h = sqrt( h );
    return -b - h;
}

vec3 sphNormal( vec3 pos, vec4 sph )
{
    return (pos - sph.xyz)/sph.w;    
}

vec3 hsv2rgb(vec3 c)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

void main()
{
   	float depth = logisticDepth(gl_FragCoord.z);
	float dBrightness = 1.0-depth*useDepthOnPointBrightness;
	float dSize = 1.0-depth*useDepthOnPointsize;
	
	if(dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) > dSize/4.0)
		discard;
	else{
		float z = sqrt(dSize - (gl_PointCoord.x-0.5)*(gl_PointCoord.x-0.5)*4.0/dSize/dSize - (gl_PointCoord.y-0.5)*(gl_PointCoord.y-0.5)*4.0/dSize/dSize);
		float h = depth * cFactor;
		float v = (1-((1.0-z)*useShadow))*dBrightness;
		vec3 c = hsv2rgb(vec3(h, 1.0, v));
		FragColor = vec4(c, 1.0);
		//FragColor = vec4(vec3(1-((1.0-z)*useShadow))*dBrightness, 1.0);

		/*vec3 ro = cameraPosition;
		vec3 ta = pointPosition;

		vec3 ww = normalize( ta - ro );
		vec3 uu = normalize( cross(ww,vec3(0.0,1.0,0.0) ) );
		vec3 vv = normalize( cross(uu,ww));
		vec3 rd = normalize( gl_PointCoord.x*uu + gl_PointCoord.y*vv + 2.0*ww );

		vec4 sph = vec4(pointPosition, 1);
		float t2 = sphIntersect(cameraPosition, rd, sph );
		vec3 pos = cameraPosition + t2*rd;
        vec3 nor = sphNormal( pos, sph );*/
		//FragColor = vec4( (nor+1.0)/2.0*dBrightness, 1.0 );
		//FragColor = vec4(pos*dBrightness, 1.0);
	}	


}