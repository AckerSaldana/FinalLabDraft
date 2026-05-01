#version 450

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;

layout(push_constant) uniform Push {
    mat4 viewProj;
} pc;

struct InstanceData {
    mat4 model;
    vec4 color;
};

layout(set = 0, binding = 0, std430) readonly buffer Instances {
    InstanceData instances[];
};

layout(location = 0) out vec3 vWorldNormal;
layout(location = 1) out vec4 vColor;

void main() {
    InstanceData inst = instances[gl_InstanceIndex];
    vec4 world = inst.model * vec4(inPos, 1.0);
    vWorldNormal = mat3(inst.model) * inNormal;
    vColor = inst.color;
    gl_Position = pc.viewProj * world;
}
