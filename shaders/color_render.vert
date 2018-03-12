//#version 150
//
//in vec4 aPosCoord;
//in vec4 aColor;
//uniform mat4 uMVPMatrix;
//
//out vec4 vColor;
//
//void main() {
//    vColor = aColor;
//    gl_Position = aPosCoord;
//}
#version 400
in vec3 vp;
uniform mat4 uMVPMatrix;

void main() {
    gl_Position = uMVPMatrix * vec4(vp, 1.0);
    gl_Position = vec4(gl_Position.xy /  gl_Position.z, 1.0, 1.0);
    gl_Position.x = (gl_Position.x - 318.27) / 468 * 480 / 640;
    gl_Position.y = (gl_Position.y - 243.99) / 468;
    gl_Position.w = 1.0;
}

