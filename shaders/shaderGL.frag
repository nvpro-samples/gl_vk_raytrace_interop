/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */
 
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
