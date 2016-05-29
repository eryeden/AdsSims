#version 410 core

// Interpolated values from the vertex shaders
in vec3 center;

// Ouput data
out vec3 color;

const float r = 0.1;


void main()
{

	// Output color = red

	color = vec3(1,0,0);

}