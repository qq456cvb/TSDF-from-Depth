#version 400
in vec2 vTexCoord;
out vec4 frag_color;

uniform sampler2D tex;

void main() {
    frag_color = texture(tex, vTexCoord);
}
