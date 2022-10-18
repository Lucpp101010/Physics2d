#version 440 core

layout (location=0) in vec2 pos;

out vec4 gl_Position;

uniform vec2 trans;
uniform vec2 scale;
uniform float angle;

void main()
{
	vec2 perp = vec2(-pos.y, pos.x);
	vec2 p = cos(angle) * pos + sin(angle) * perp;
	gl_Position = vec4((scale*p) + trans, 0, 1);
}