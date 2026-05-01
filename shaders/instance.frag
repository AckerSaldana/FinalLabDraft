#version 450

layout(location = 0) in vec3 vWorldNormal;
layout(location = 1) in vec4 vColor;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 N = normalize(vWorldNormal);
    if (!gl_FrontFacing) N = -N;

    vec3 L = normalize(vec3(0.55, 1.0, 0.35));
    float ndotl = max(dot(N, L), 0.0);
    vec3 base = vColor.rgb;
    vec3 ambient = 0.2 * base;
    vec3 diffuse = 0.8 * ndotl * base;
    outColor = vec4(ambient + diffuse, vColor.a);
}
