#version 450
uniform mat4 V;
uniform mat4 P;
uniform mat4 pose_rot;
uniform mat4 pose_trans;
uniform vec3 class_id;
uniform vec3 instance_id;
uniform vec3 diffuse_color;

layout (location=0) in vec3 position;
layout (location=1) in vec3 input_normal;
layout (location=2) in vec2 texCoords;

out vec2 theCoords;
out vec3 Normal_world;
out vec3 FragPos;
out vec3 Normal_cam;
out vec3 classId;
out vec3 instanceId;
out vec3 Pos_cam;
out vec3 Diffuse_color;
void main() {
    gl_Position = P * V * pose_trans * pose_rot * vec4(position, 1);
    vec4 world_position4 = pose_trans * pose_rot * vec4(position, 1);
    FragPos = vec3(world_position4.xyz / world_position4.w); // in world coordinate
    Normal_world = normalize(mat3(pose_rot) * input_normal); // in world coordinate
    Normal_cam = normalize(mat3(V) * mat3(pose_rot) * input_normal); // in camera coordinate
    vec4 pos_cam4 = V * pose_trans * pose_rot * vec4(position, 1);
    Pos_cam = pos_cam4.xyz / pos_cam4.w;
    theCoords = texCoords;
    classId = class_id;
    instanceId = instance_id;
    Diffuse_color = diffuse_color;
}