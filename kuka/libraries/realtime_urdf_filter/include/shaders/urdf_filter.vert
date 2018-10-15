#version 120
in vec3 vertex;
varying out vec3 normal;

void main() {
	vec3 temp = gl_NormalMatrix * gl_Normal;
	normal = vec3 (-temp.x, temp.y, -temp.z);
	
	gl_Position = (gl_ModelViewProjectionMatrix * (vec4(vertex, 1.0))); //normal * 0.01
}

