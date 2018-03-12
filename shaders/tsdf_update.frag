#version 400
in vec2 vTexCoord;

uniform sampler2D rgb;
uniform sampler2D depth;

out vec4 frag_color;

//uniform mat4 c2w;
uniform mat4 w2s;
//uniform mat3 Rcw;
//uniform vec3 Tcw;
//uniform float fx, fy, cx, cy, baseline;
//
const float eps = 0.0001f;
const float vol_dim = 64.;
const float tex_dim = sqrt(vol_dim * vol_dim * vol_dim);
const vec2 texel = vec2(1.0/tex_dim);
const vec3 volStart = vec3(-0.7, -0.4, -1.0);
const vec3 volEnd = vec3(0.8, 1.1, 0.5);
const vec3 vol = volStart - volEnd;
const float mu = 1.5 / vol_dim * 2.;
const float voxel = 1.5 / (vol_dim - 1);
//const float volSize = 2.0, maxWeight = 100.0, mu = 0.1;
//const vec2 imageSize = vec2(640.0, 480.0);
//
vec3 texToPos(vec2 texCoord)
{
    vec3 pos;
    float idx = floor(texCoord.y/texel.y) * tex_dim + floor(texCoord.x/texel.x);
    float idxz = floor(idx/(vol_dim*vol_dim));
    float idxxy = idx - idxz*(vol_dim*vol_dim);
    float idxy = floor(idxxy/vol_dim);
    float idxx = idxxy - idxy*vol_dim;
    pos = vec3(idxx, idxy, idxz) * voxel + volStart;
    return pos;
}

void main(void)
{
//    frag_color = vec4(1, 0, 0, 1);
//    vec3 pos = texToPos(vTexCoord);
    
    vec4 pos = vec4(texToPos(vTexCoord), 1);
    vec4 screen_pos = w2s * pos;
    screen_pos /= screen_pos.z;
    vec2 tex_coord = screen_pos.xy;
    tex_coord.x /= 640.;
    tex_coord.y /= 480.;
    if (tex_coord.x < 0 || tex_coord.x > 1 || tex_coord.y < 0 || tex_coord.y > 1) discard;
    
    float d = texture(depth, tex_coord).r * 65535. / 5000.;
    float diff = d - 1. / screen_pos.w;
//    if (diff > mu || diff < -mu) discard;
//    if (d < 1) discard;
//    if (diff < 0) {
//        frag_color = vec4(0, 0, 1, 1);
//    } else {
        diff = max(min(diff, mu), -mu);
        
//    frag_color = texture(depth, vTexCoord);
//    frag_color = vec4(1, 0, 0, 1);
        vec4 color = texture(rgb, tex_coord);
        frag_color = vec4(color.rgb, diff);
//    }
//    frag_color = texture(rgb, vTexCoord);
    
    
//    frag_color = vec4(texture(depth, vTexCoord).r, 0, 0, 1);
//    gl_FragColor = texture2D(tsdf, vTexCoord) + vec4(0.01, 0.0, 0.0, 0.0);
//    gl_FragColor = vec4(float(int(vTexCoord.x/texel.x)/256)*256.0/4096.0, 0.0, 0.0, 1.0);
//    vec3 worldPos = texToPos(vTexCoord);
//    vec3 cameraPos = Rcw * worldPos + Tcw;
//    float invZ = 1.0/cameraPos.z;
//
//    // map to 0.0~1.0
//    vec2 pixelPos = vec2(fx*cameraPos.x*invZ+cx, fy*cameraPos.y*invZ+cy)/imageSize;
//
//    if (pixelPos.x < 0.0 || pixelPos.x > 1.0 || pixelPos.y < 0.0 || pixelPos.y > 1.0) {
//        gl_FragColor = texture2D(tsdf, vTexCoord);
//        return;
//    }
//
//    float disp = float(texture2D(depth, pixelPos).r)*256.0;
//    if (disp < 0.001) {
//        gl_FragColor = texture2D(tsdf, vTexCoord);
//        return;
//    }
//
//    float actualDepth = fx * baseline / disp;
//    float diff = (actualDepth - cameraPos.z) * sqrt(1.0 + cameraPos.x*cameraPos.x/(cameraPos.z*cameraPos.z) + cameraPos.y*cameraPos.y/(cameraPos.z*cameraPos.z));
//
//    if (diff > -1.0) {
//        float sdf = min(diff, 1.0);
//        vec2 data = texture2D(tsdf, vTexCoord).xy;
//        gl_FragColor = vec4(clamp((data.y*data.x + sdf)/(data.y + 1.0), -mu, mu),
//                            min(data.y + 1.0, maxWeight),
//                            0.0, 1.0);
//    } else {
//        gl_FragColor = texture2D(tsdf, vTexCoord);
//    }
}
