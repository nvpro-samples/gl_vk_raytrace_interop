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

#include "imgui/backends/imgui_impl_gl.h"
#include "nvgl/extensions_gl.hpp"
#include "nvvkhl/appbase_vk.hpp"
#include "raytrace_interop.hpp"

//--------------------------------------------------------------------------------------------------
// Simple example showing some objects, simple material and lighting, camera movement
//
class VkGlExample : public nvvkhl::AppBaseVk
{
public:
  VkGlExample() = default;

  // AppBaseVk
  void setup(const VkInstance& instance, const VkDevice& device, const VkPhysicalDevice& physicalDevice, uint32_t graphicsQueueIndex) override;
  void destroy() override;
  void onResize(int w, int h) override;
  void onKeyboardChar(unsigned char) override;
  void onKeyboard(int key, int scancode, int action, int mods) override;
  void onFramebufferSize(int w, int h) override;

  void createExample();
  void drawUI();
  void onWindowRefresh();
  void initUI(int width, int height);

private:
  // Structure of the Vertices used by the shaders
  struct Vertex
  {
    glm::vec3 pos;
    glm::vec3 nrm;
    glm::vec2 uv;
  };

  // Simple triangle mesh structure
  struct Meshes
  {
    interop::BufferVkGL vertices;
    interop::BufferVkGL indices;
    uint32_t            indexCount  = 0;   // Nb of indices to draw
    uint32_t            vertexCount = 0;   // Nb vertices
    GLuint              oglID       = 0;   // OpenGL
    VkGeometryNV        rayGeometry = {};  // Raytrace

    void destroy(nvvk::ResourceAllocator& alloc)
    {
      vertices.destroy(alloc);
      indices.destroy(alloc);
    }
  };

  struct Instance
  {
    uint32_t      objectIndex{0};               // Reference of the mesh
    uint32_t      materialIndex{0};             // Reference of the material
    glm::mat4 transform{glm::mat4(1)};  // identity
  };


private:
  // User interface settings
  bool m_aoUse        = true;
  bool m_aoBlur       = true;
  int  m_aoBlurRadius = 2;
  int  m_rtNbRays     = 64;
  int  m_bufferView   = 0;
  int  m_frameNumber  = 0;

  std::vector<Meshes>   m_meshes;
  std::vector<Instance> m_instances;

  // The buffers used by the example
  struct
  {
    interop::BufferVkGL matrices;  // matrices of all instances
  } m_uniformBuffers;

  bool         needToResetFrame();
  void         createScene();
  void         prepareUniformBuffers();
  void         vulkanMeshToOpenGL(Meshes& mesh);
  void         createGBuffers();
  VkGeometryNV meshToGeometry(const Meshes& mesh);
  Meshes       createPlane();
  Meshes       createFacetedTetrahedron();
  void         loadImage(const std::string& filename, interop::Texture2DVkGL& img);
  void         createShaders();

  GLuint    m_gFramebuffer    = 0;
  OglShader m_shaderRaster    = {};
  OglShader m_shaderComposite = {};

  interop::ResourceAllocatorGLInterop   m_allocInterop;  // The Vulkan buffer and image allocator with Export
  interop::RtInterop                    m_ray;
  std::array<interop::Texture2DVkGL, 2> m_imageVkGL;
  std::vector<interop::Texture2DVkGL>   m_gBufferColor{{}, {}, {}};  // Position, Normal, Albedo
};
