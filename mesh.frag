#version 330 core

// Outputs colors in RGBA
out vec4 FragColor;


// Inputs the color from the Vertex Shader
in vec3 color;
// Inputs the texture coordinates from the Vertex Shader
in vec2 texCoord;

// Gets the Texture Unit from the main function
uniform sampler2D tex0;

uniform float near = 0.01f;
uniform float far = 10.0f;

float linearizeDepth(float depth)
{
	return (2.0 * near * far) / (far + near - (depth * 2.0 - 1.0) * (far - near));
}


void main()
{
    //FragColor = vec4(color, 1.0f);
    FragColor = vec4(color * 1-(linearizeDepth(gl_FragCoord.z) / far), 1.0f);
	//FragColor = texture(tex0, texCoord);
}