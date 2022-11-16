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

#include <vulkan/vulkan.hpp>

#include <array>

#include "imgui/backends/imgui_impl_gl.h"
#include "nvgl/extensions_gl.hpp"
#include "nvmath/nvmath.h"
#include "nvvkhl/appbase_vkpp.hpp"
#include "nvvk/memallocator_dma_vk.hpp"
#include "nvvk/resourceallocator_vk.hpp"

#include "gl_vkpp.hpp"
#include "raytrace_interop.hpp"

//--------------------------------------------------------------------------------------------------
// Simple example showing some objects, simple material and lighting, camera movement
//
class VkGlExample : public nvvkhl::AppBase
{
public:
  // Structure of the Vertices used by the shaders
  struct Vertex
  {
    nvmath::vec3f pos;
    nvmath::vec3f nrm;
    nvmath::vec2f uv;
  };

  // Simple triangle mesh structure
  struct Meshes
  {
    interop::BufferVkGL vertices;
    interop::BufferVkGL indices;
    uint32_t            indexCount{0};   // Nb of indices to draw
    uint32_t            vertexCount{0};  // Nb vertices
    GLuint              oglID{0};        // OpenGL
    vk::GeometryNV      rayGeometry;     // Raytrace

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
    nvmath::mat4f transform{nvmath::mat4f(1)};  // identity
  };


  VkGlExample() = default;

  void setup(const vk::Instance& instance, const vk::Device& device, const vk::PhysicalDevice& physicalDevice, uint32_t graphicsQueueIndex) override
  {
    AppBase::setup(instance, device, physicalDevice, graphicsQueueIndex);

    m_allocInterop.init(device, physicalDevice);

    m_ray.setup(device, physicalDevice, graphicsQueueIndex);
  }

  void initExample();
  bool needToResetFrame();
  void drawUI();

  void onWindowRefresh();
  void destroy() override;
  void onResize(int w, int h) override;
  void onKeyboardChar(unsigned char) override;
  void onKeyboard(int key, int scancode, int action, int mods) override;
  void loadImage(const std::string& filename);
  void createShaders();

  //--------------------------------------------------------------------------------------------------
  // Initialization of the GUI
  // - Need to be call after the device creation
  //
  void initUI(int width, int height)
  {
    onFramebufferSize(width, height);

    // UI
    ImGui::CreateContext();
    auto& io       = ImGui::GetIO();
    io.IniFilename = nullptr;                          // Avoiding the INI file
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;  // Enable Docking
    ImGuiH::setStyle();
    ImGuiH::setFonts();

    ImGui::InitGL();
  }

  //- Override the default resize, no swapchain
  void onFramebufferSize(int w, int h) override
  {
    m_size.width  = w;
    m_size.height = h;
    CameraManip.setWindowSize(w, h);
  }

  std::vector<Meshes>   m_meshes;
  std::vector<Instance> m_instances;


private:
  // Ui stuff
  bool m_aoUse        = true;
  bool m_aoBlur       = true;
  int  m_aoBlurRadius = 2;
  int  m_rtNbRays     = 64;
  int  m_bufferView   = 0;

  int m_frameNumber = 0;

  // The buffers used by the example
  struct
  {
    interop::BufferVkGL matrices;  // matrices of all instances
  } m_uniformBuffers;

  void createScene();
  void prepareUniformBuffers();
  void vulkanMeshToOpenGL(Meshes& mesh);
  void createGBuffers();

  VkGeometryNV meshToGeometry(const Meshes& mesh);

  Meshes createPlane();
  Meshes createFacetedTetrahedron();

  interop::Texture2DVkGL              m_imageVkGL;
  std::vector<interop::Texture2DVkGL> m_gBufferColor{{}, {}, {}};  // Position, Normal, Albedo

  GLuint m_gFramebuffer = 0;
  Shader m_shaderRaster;
  Shader m_shaderComposite;
  
  interop::ResourceAllocatorGLInterop m_allocInterop;  // The Vulkan buffer and image allocator with Export

  interop::RtInterop m_ray;
};
