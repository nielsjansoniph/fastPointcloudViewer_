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

vec4 iBox( in vec3 ro, in vec3 rd, in mat4 txx, in mat4 txi, in vec3 rad ) 
{
    // convert from ray to box space
	vec3 rdd = (txx*vec4(rd,0.0)).xyz;
	vec3 roo = (txx*vec4(ro,1.0)).xyz;

	// ray-box intersection in box space
    vec3 m = 1.0/rdd;
    #if 1
    vec3 n = m*roo;
    vec3 k = abs(m)*rad;
    vec3 t1 = -n - k;
    vec3 t2 = -n + k;
	#else
    // more robust
    vec3 k = vec3(rdd.x>=0.0?rad.x:-rad.x, rdd.y>=0.0?rad.y:-rad.y, rdd.z>=0.0?rad.z:-rad.z);
    vec3 t1 = (-roo - k)*m;
    vec3 t2 = (-roo + k)*m;
    #endif
    float tN = max(max(t1.x,t1.y),t1.z);
    float tF = min(min(t2.x,t2.y),t2.z);
    
    // no intersection
	if( tN>tF || tF<0.0 ) return vec4(-1.0);

    #if 1
    // this works as long as the ray origin is not inside the box
    vec4 res = vec4(tN, step(tN,t1) );
    #else
    // use this instead if your rays origin can be inside the box
    vec4 res = (tN>0.0) ? vec4( tN, step(vec3(tN),t1)) :
                          vec4( tF, step(t2,vec3(tF)));
    #endif
    
    // add sign to normal and convert to ray space
	res.yzw = (txi * vec4(-sign(rdd)*res.yzw,0.0)).xyz;

	return res;
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
	
	//if(dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) > dSize/4.0)
	//	discard;
	//else{
		float z = sqrt(dSize - (gl_PointCoord.x-0.5)*(gl_PointCoord.x-0.5)*4.0/dSize/dSize - (gl_PointCoord.y-0.5)*(gl_PointCoord.y-0.5)*4.0/dSize/dSize);
		

		vec3 ro = cameraPosition;
		vec3 ta = pointPosition;

		vec3 ww = normalize( ta - ro );
		vec3 uu = normalize( cross(ww,vec3(0.0,1.0,0.0) ) );
		vec3 vv = normalize( cross(uu,ww));
		vec3 rd = normalize( (gl_PointCoord.x-0.5)*uu + (gl_PointCoord.y-0.5)*vv + 2.0*ww );

        
        mat4 t = mat4(1.0, 0.0, 0.0, 0.0,
                      0.0, 1.0, 0.0, 0.0,
                      0.0, 0.0, 1.0, 0.0, 
                      0.0, 0.0, 0.0, 1.0);
        

		vec3 box = vec3(0.5,0.5,0.5) ;
        vec4 res = iBox( ro, rd, t, inverse(t), box);
        
        vec3  nor = vec3(0.0);
        
        float tmin = 10000.0;

        
        if( res.x>0.0 && res.x<10000.0 )
        {
            tmin = res.x; 
            nor = res.yzw;
            FragColor = vec4( (nor+1.0)/2.0*dBrightness, 1.0 );
        }
        else   
            discard;
        
		
		//FragColor = vec4(pos*dBrightness, 1.0);

}