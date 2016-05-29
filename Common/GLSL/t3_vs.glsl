#version 410 core

// a Shader for geometry shading

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec2 vertexPosition_modelspace;

// Output data ; will be interpolated for each fragment.
//out vec3 Fragment_color;

//a Interface for geometry shader
out VS_OUT {
    vec3 Fragment_color;
} vs_out;

// Values that stay constant for the whole mesh.
uniform mat4 MVP;
uniform vec3 Color;

void main() {

	// Output position of the vertex, in clip space : MVP * position
	gl_Position =  MVP * vec4(vertexPosition_modelspace, 0.0, 1.0);

	// Color of the vertex
	vs_out.Fragment_color = Color;
}
