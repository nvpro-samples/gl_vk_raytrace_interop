#version 450
layout(location = 0) in vec2 inUV;

layout(location = 0) out vec4 outColor;

layout(binding = 0) uniform sampler2D gPosition;
layout(binding = 1) uniform sampler2D gNormal;
layout(binding = 2) uniform sampler2D gAlbedoSpec;
layout(binding = 3) uniform sampler2D gOcclusion;

uniform bool ao_use;
uniform bool ao_do_blur;
uniform int  ao_blur_radius;
uniform int  buffer_view;


void main()
{
  //  vec3 nrm = texture(gNormal, inUV).rgb;
  float occlusion = texture(gOcclusion, inUV).r;
  vec4  color     = texture(gAlbedoSpec, inUV);
  vec3  nrm       = texture(gNormal, inUV).rgb;
  vec4  position  = texture(gPosition, inUV);

  if(color.a == 0.0)
    discard;

  if(ao_use == false)
    occlusion = 1.0f;

  if(ao_do_blur && ao_use)
  {
    const int blurRange = ao_blur_radius;
    int       n         = 1;
    vec2      texelSize = 1.0 / vec2(textureSize(gOcclusion, 0));
    float     depth     = position.w;
    float     result    = occlusion;  //0.0;
    for(int x = -blurRange; x < blurRange; x++)
    {
      for(int y = -blurRange; y < blurRange; y++)
      {
        vec2  offset      = vec2(float(x), float(y)) * texelSize;
        float sampleDepth = texture(gPosition, inUV + offset).w;
        if(abs(sampleDepth - depth) < 0.1)
        {
          result += texture(gOcclusion, inUV + offset).r;
          n++;
        }
      }
    }
    occlusion = result / float(n);
  }

  outColor = vec4(color.rgb * occlusion, color.a);

  switch(buffer_view)
  {
    case 1:
      outColor.xyz = abs(position.xyz / 10.0);
      break;
    case 2:
      outColor.xyz = abs(nrm);
      break;
    case 3:
      outColor.xyz = vec3(pow(max(1.0f - position.w / 40.0, 0.0), 3.0));
      break;
    case 4:
      outColor.xyz = color.rgb;
      break;
    case 5:
      outColor.xyz = vec3(occlusion);
      break;
  }
}
