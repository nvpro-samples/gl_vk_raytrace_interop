/* Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <array>
#include <vulkan/vulkan.hpp>

#include "gl_vkpp.hpp"
#include "imgui_impl_gl.h"
#include "nvgl/extensions_gl.hpp"
#include "nvmath/nvmath.h"
#include "nvvk/allocator_dma_vkgl.hpp"
#include "nvvk/appbase_vkpp.hpp"
#include "raytrace_interop.hpp"
#include "utility_ogl.hpp"

//--------------------------------------------------------------------------------------------------
// Simple example showing some objects, simple material and lighting, camera movement
//
class VkGlExample : public nvvk::AppBase
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

    void destroy(nvvk::AllocatorDma& alloc)
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
    m_dmaAllocGL.init(device, physicalDevice);
    m_alloc.init(device, physicalDevice, &m_dmaAllocGL);

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
    ImGui::GetIO().IniFilename = nullptr;  // Avoiding the INI file
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

  vk::GeometryNV meshToGeometry(const Meshes& mesh);

  Meshes createPlane();
  Meshes createFacetedTetrahedron();

  interop::Texture2DVkGL              m_imageVkGL;
  std::vector<interop::Texture2DVkGL> m_gBufferColor{{}, {}, {}};  // Position, Normal, Albedo

  GLuint m_gFramebuffer = 0;
  Shader m_shaderRaster;
  Shader m_shaderComposite;

  nvvk::AllocatorDma            m_alloc;  // The Vulkan buffer and image allocator with Export
  nvvk::DeviceMemoryAllocatorGL m_dmaAllocGL;

  interop::RtInterop m_ray;
};
