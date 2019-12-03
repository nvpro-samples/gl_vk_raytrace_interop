#version 450

layout(location = 0) in vec4 inPos;
layout(location = 1) in vec3 inNrm;
layout(location = 2) in vec2 inUV;

//layout(location = 0) out vec4 fragColor;

layout (location = 0) out vec4 gPosition;
layout (location = 1) out vec4 gNormal;
layout (location = 2) out vec4 gAlbedoSpec;


// Values that stay constant for the whole mesh.
uniform sampler2D myTextureSampler;
uniform vec3 u_color;
uniform bool u_hasTexture;

void main()
{
  vec2 uv    = inUV;  //- 0.5;
  vec3 color = u_color;
  if(u_hasTexture)
    color *= texture(myTextureSampler, uv).rgb;
  // Output to screen
  //fragColor = vec4(color, 1);


  gNormal = vec4(normalize(inNrm),0);
  gPosition = inPos;
  gAlbedoSpec.rgb = color;
  gAlbedoSpec.a = 1.f;

}
