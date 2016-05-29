#version 410 core

// Input data ; will be interpolated for each fragment.
in vec3 Fragment_color;

out vec4 color;

void main() {

   color = vec4(Fragment_color, 1.0f);
   //color = vec4(0.8, 0.1, 0.1, 1.0f);

}
