#version 450
layout(location = 0) in vec3 inVertex;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUV;

layout(location = 0) out vec4 outPos;
layout(location = 1) out vec3 outNrm;
layout(location = 2) out vec2 outUV;


uniform mat4 u_ViewProjectionMatrix;
uniform mat4 u_ModelMatrix;

layout(std430, binding = 0) buffer matrixBuffer
{
  mat4 objectMatrix;
}
object;


void main()
{
  outUV       = inUV;
  outPos = object.objectMatrix * vec4(inVertex, 1.0f);
  outNrm = vec3(transpose(inverse(object.objectMatrix)) * vec4(inNormal,0.0f)); 
  gl_Position = u_ViewProjectionMatrix * object.objectMatrix * vec4(inVertex, 1.0f);
  outPos.w = gl_Position.z;
}
