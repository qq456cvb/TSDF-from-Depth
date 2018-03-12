#version 400
in vec2 vTexCoord;

uniform sampler2D tsdf;

out vec4 frag_color;

//uniform mat4 c2w;
uniform mat4 s2w;
//uniform mat3 Rcw;
uniform vec3 c;
//uniform float fx, fy, cx, cy, baseline;

const float eps = 0.00001f;
const float vol_dim = 64;
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
    float idxx = floor(idxxy - idxy*vol_dim);
    pos = vec3(idxx, idxy, idxz) * voxel + volStart;
    return pos;
}

//bool test(vec3 pos) {
//    float idx = floor(texCoord.y/texel.y) * 4096 + floor(texCoord.x/texel.x);
//}

vec2 indToTex(vec3 ind) {
//    ind += eps;
//    vec3 ind = (pos - volStart) / voxel;
    float tex_ind_1d = ind.z * vol_dim * vol_dim + ind.y * vol_dim + ind.x;
    float tex_ind_y = floor(tex_ind_1d / tex_dim);
    float tex_ind_x = tex_ind_1d - tex_dim * tex_ind_y;
    vec2 tex_ind = vec2(tex_ind_x, tex_ind_y) * texel;
//    tex_ind.y = 1. - tex_ind.y;
    return tex_ind;
}

// TODO: add interpolation
vec4 interpTsdf(vec3 pos) {
    pos += eps;
    vec3 ind = (pos - volStart) / voxel;
    vec3 interp = fract(ind);
    ind = floor(ind);
    vec2 tex_lll = indToTex(ind);
    vec2 tex_hll = indToTex(ind + vec3(1, 0, 0));
    vec2 tex_lhl = indToTex(ind + vec3(0, 1, 0));
    vec2 tex_hhl = indToTex(ind + vec3(1, 1, 0));
    vec4 low = mix( mix(texture(tsdf, tex_lll), texture(tsdf, tex_hll), interp.x),  mix(texture(tsdf, tex_lhl), texture(tsdf, tex_hhl), interp.x), interp.y);
    
    vec2 tex_llh = indToTex(ind + vec3(0, 0, 1));
    vec2 tex_hlh = indToTex(ind + vec3(1, 0, 1));
    vec2 tex_lhh = indToTex(ind + vec3(0, 1, 1));
    vec2 tex_hhh = indToTex(ind + vec3(1, 1, 1));
    vec4 high = mix( mix(texture(tsdf, tex_llh), texture(tsdf, tex_hlh), interp.x),  mix(texture(tsdf, tex_lhh), texture(tsdf, tex_hhh), interp.x), interp.y);
    return mix(low, high, interp.z);
}

void main(void)
{
    //    frag_color = vec4(1, 0, 0, 1);
    //    vec3 pos = texToPos(vTexCoord);
    
//    vec4 pos = vec4(texToPos(vTexCoord), 1);
//    vec4 screen_pos = w2s * pos;
//    screen_pos /= screen_pos.z;
//    vec2 tex_coord = screen_pos.yx;
//    tex_coord.x /= 640.;
//    tex_coord.y /= 480.;
//    if (tex_coord.x < 0 || tex_coord.x > 1 || tex_coord.y < 0 || tex_coord.y > 1) discard;
//    
//    float d = texture(depth, tex_coord).r * 65535. / 5000.;
//    float diff = d - 1. / screen_pos.w;
//    diff = max(min(diff, mu), -mu);
//    
//    //    frag_color = vec4(vec3(diff / (3. / 256.)), 1);
//    vec4 color = texture(rgb, tex_coord);
    vec4 screen_pos = vec4(vTexCoord * vec2(640, 480), 1, 1);
    vec4 target = s2w * screen_pos;
    vec3 d = target.xyz - c;
    d = normalize(d);
    vec3 inv_d = 1. / d;
    vec3 tbot = inv_d * (volStart - c);
    vec3 ttop = inv_d * (volEnd - c);
    
    vec3 tmin = vec3(min(ttop.x, tbot.x), min(ttop.y, tbot.y), min(ttop.z, tbot.z));
    float tnear = max(max(tmin.x, tmin.y), tmin.z);
    tnear = max(tnear, 0.1);
    
    vec3 tmax = vec3(max(ttop.x, tbot.x), max(ttop.y, tbot.y), max(ttop.z, tbot.z));
    float tfar = min(min(tmax.x, tmax.y), tmax.z);
    tfar = min(tfar, 100);
    if (tnear > tfar) discard;
    
    float eps = 0.00001f;
    float t = tnear;
    float stepsize = voxel;
//    vec3 p = vec3(0, 0, 0);
//    vec2 tex_c = posToTex(c + t * d);
//    if (distance(texToPos(tex_c), c + t * d) > 0.01) discard;
//    if (tex_c.x < 0 || tex_c.y < 0 || tex_c.x > 1 || tex_c.y > 1) discard;
    float f_t = interpTsdf(c + t * d).a;
    float f_tt = 0;
    vec4 color = vec4(1, 0, 0, 1);
//    color.rgb = interpTsdf(tex_c).rgb;
//    if (f_t < 0) {
//        discard;
//    }
    if (f_t > 0) {
        for(; t < tfar; t += stepsize){
            f_tt = interpTsdf(c + t * d).a;
            if(f_tt < 0.0)                               // got it, jump out of inner loop
                break;
            if(f_tt < voxel / 2)                            // coming closer, reduce stepsize
                stepsize = voxel / 4;
            f_t = f_tt;
        }
        if(f_tt < 0.0){                               // got it, calculate accurate intersection
            t = t + stepsize * f_tt / (f_t - f_tt);
            vec3 pt = c + t * d;
            color.rgb = interpTsdf(pt).rgb;
//            color = vec4(0, 0, 1, 1);
        }
    }
//    frag_color = texture(tsdf, vTexCoord);;
//    color = texture(tsdf, vTexCoord);
//    frag_color = vec4(color.a / mu, 0, 0, 1);
//    if (color.a < 0) discard;
    frag_color = color;
}
