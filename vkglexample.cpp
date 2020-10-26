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


// In this example,
// - Many tetrahedrons are created (createScene, createFacetedTetrahedron)
//   with alternative color, a plane and it lit with a directional light.
// - Each tetrahedron has it own matrix and all matrices will be placed in a single buffer.
// - It is possible to switch between a recorded command buffer, or simply drawing each
//   element individually.
// - By default, the rendering is done in multi-sampling and resolve in the back buffer


#ifdef WIN32
#include <concrt.h>
#endif
#include <iostream>
#include <random>  // Used to randomly place objects
#include <sstream>

#include <fileformats/stb_image.h>

#include "vkglexample.h"

#include "imgui_helper.h"
#include "imgui_impl_glfw.h"
#include <nvvk/extensions_vk.hpp>


#define NB_MAX_TETRAHEDRONS 10000
#define UBO_MATRIX 0

extern std::vector<std::string> defaultSearchPaths;

//--------------------------------------------------------------------------------------------------
// Overridden function that is called after the base class create()
//
void VkGlExample::initExample()
{
  // Enable debug output
  glEnable(GL_DEBUG_OUTPUT);
  glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
  glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_FALSE);
  glDebugMessageControl(GL_DEBUG_SOURCE_SHADER_COMPILER, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
  glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DEBUG_TYPE_ERROR, GL_DONT_CARE, 0, nullptr, GL_TRUE);
  glDebugMessageCallback(GLDebugCallback, nullptr);

  // Creation of the scene
  createShaders();
  loadImage(nvh::findFile(R"(data/1200px-Nvidia_image_logo.svg.png)", defaultSearchPaths));
  createGBuffers();
  createScene();
  prepareUniformBuffers();

  // Raytracing
  {
    std::vector<std::vector<VkGeometryNV>> blass;
    blass.reserve(m_meshes.size());
    for(auto& m : m_meshes)  // Two meshes: plane and tetrahedron
    {
      blass.push_back({meshToGeometry(m)});
    }

    // One instance has the plane objectID, all others the tetrahedron with a different transform
    std::vector<nvvk::RaytracingBuilderNV::Instance> rayInst(m_instances.size());
    for(size_t i = 0; i < m_instances.size(); i++)
    {
      rayInst[i].instanceId = uint32_t(i);
      rayInst[i].blasId     = m_instances[i].objectIndex;
      rayInst[i].transform  = m_instances[i].transform;
    }

    m_ray.builder().buildBlas(blass);
    m_ray.builder().buildTlas(rayInst);
    m_ray.createOutputImage(m_size);
    m_ray.createDescriptorSet(m_gBufferColor);
    m_ray.createPipeline();
    m_ray.createShadingBindingTable();
    m_ray.createSemaphores();
  }
}

//--------------------------------------------------------------------------------------------------
// Overridden function called on shutdown
//
void VkGlExample::destroy()
{
  m_device.waitIdle();
  for(auto& mesh : m_meshes)
  {
    mesh.destroy(m_alloc);
  }


  m_imageVkGL.destroy(m_alloc);
  m_gBufferColor[0].destroy(m_alloc);
  m_gBufferColor[1].destroy(m_alloc);
  m_gBufferColor[2].destroy(m_alloc);

  m_uniformBuffers.matrices.destroy(m_alloc);
  m_ray.destroy();
  m_alloc.deinit();
  m_dmaAllocGL.deinit();
  AppBase::destroy();
}

//--------------------------------------------------------------------------------------------------
// Create a scene with many tetrahedron and a plane
// There will be one tetrahedron mesh, and multiple instances, each having a transform and a
// material.
//
void VkGlExample::createScene()
{
  std::random_device              rd;         // Will be used to obtain a seed for the random number engine
  std::mt19937                    gen(rd());  // Standard mersenne_twister_engine seeded with rd()
  std::normal_distribution<float> dis(0.0f, 2.0f);
  std::normal_distribution<float> disn(0.1f, 0.15f);

  Instance inst{};

  // Adding many tetrahedron
  m_meshes.push_back(createFacetedTetrahedron());
  for(int n = 0; n < NB_MAX_TETRAHEDRONS; ++n)
  {
    inst.objectIndex = static_cast<uint32_t>(m_meshes.size() - 1);
    float scale      = disn(gen);
    inst.transform.as_translation(nvmath::vec3f(dis(gen), dis(gen), dis(gen)));
    inst.transform.rotate(dis(gen), nvmath::vec3f(1.0f, 0.0f, 0.0f));
    inst.transform.scale(nvmath::vec3f(scale, scale, scale));
    inst.materialIndex = n % 2;  // Alternating the material color
    m_instances.push_back(inst);
  }

  // Adding the plane
  m_meshes.push_back(createPlane());
  inst             = {};
  inst.objectIndex = static_cast<uint32_t>(m_meshes.size() - 1);
  inst.transform.as_translation(nvmath::vec3f(0.f, -5.f, 0.f));
  inst.materialIndex = 2;
  m_instances.push_back(inst);
}

//--------------------------------------------------------------------------------------------------
//
//
VkGlExample::Meshes VkGlExample::createFacetedTetrahedron()
{
  std::vector<Vertex>   vert;
  std::vector<uint32_t> indices;

  // Position and indices of the tetrahedron
  std::vector<nvmath::vec3f> tpos(4);
  tpos[0]                    = nvmath::vec3f(sqrt(8.f / 9.f), 0, -1.f / 3.f);
  tpos[1]                    = nvmath::vec3f(-sqrt(2.f / 9.f), sqrt(2.f / 3.f), -1.f / 3.f);
  tpos[2]                    = nvmath::vec3f(-sqrt(2.f / 9.f), -sqrt(2.f / 3.f), -1.f / 3.f);
  tpos[3]                    = nvmath::vec3f(0, 0, 1);
  std::vector<uint32_t> tidx = {0, 2, 1, /*0*/ 0, 1, 3, /*1*/ 0, 3, 2, /*2*/ 1, 2, 3 /*3*/};

  std::vector<nvmath::vec2f> tuv = {{0, 0}, {1, 0}, {0.5f, 1}};

  vert.resize(4 * 3);     // 4 triangles
  indices.resize(4 * 3);  // 12 indices

  // Computing the normal for each vertex/triangle
  uint32_t vidx = 0;
  for(size_t t = 0; t < 4; ++t)
  {
    nvmath::vec3f& v0  = tpos[tidx[t * 3 + 0]];
    nvmath::vec3f& v1  = tpos[tidx[t * 3 + 1]];
    nvmath::vec3f& v2  = tpos[tidx[t * 3 + 2]];
    nvmath::vec3f  nrm = nvmath::normalize(nvmath::cross(v1 - v0, v2 - v0));

    for(size_t v = 0; v < 3; ++v)
    {
      vert[vidx + v].pos = tpos[tidx[t * 3 + v]];
      vert[vidx + v].nrm = nrm;
      vert[vidx + v].uv  = tuv[v];
      indices[vidx + v]  = vidx + static_cast<uint32_t>(v);
    }
    vidx += 3;
  }

  // Storing the mesh in a buffer of vertices and a buffer of indices
  // #VKGL: The buffer creation need this export extension
  vk::ExportMemoryAllocateInfo exportAllocInfo(vk::ExternalMemoryHandleTypeFlagBits::eOpaqueWin32);
  Meshes                       mesh;
  mesh.indexCount  = (uint32_t)indices.size();
  mesh.vertexCount = (uint32_t)vert.size();

  {
    nvvk::ScopeCommandBuffer cmdBuf(m_device, m_graphicsQueueIndex);
    mesh.vertices.bufVk = m_alloc.createBuffer<Vertex>(cmdBuf, vert, vk::BufferUsageFlagBits::eVertexBuffer);
    mesh.indices.bufVk  = m_alloc.createBuffer<uint32_t>(cmdBuf, indices, vk::BufferUsageFlagBits::eIndexBuffer);
    vulkanMeshToOpenGL(mesh);
  }
  m_alloc.finalizeAndReleaseStaging();

  return mesh;
}


//--------------------------------------------------------------------------------------------------
//
//
VkGlExample::Meshes VkGlExample::createPlane()
{
  std::vector<Vertex>   vert;
  std::vector<uint32_t> indices;

#define PLANE_SIZE 10
  // Position and indices of the tetrahedron
  vert.resize(4);
  vert[0].pos = nvmath::vec3f(-PLANE_SIZE, 0, -PLANE_SIZE);
  vert[1].pos = nvmath::vec3f(-PLANE_SIZE, 0, PLANE_SIZE);
  vert[2].pos = nvmath::vec3f(PLANE_SIZE, 0, PLANE_SIZE);
  vert[3].pos = nvmath::vec3f(PLANE_SIZE, 0, -PLANE_SIZE);

  indices.resize(6);
  indices = {0, 1, 2, /*0*/ 0, 2, 3 /*1*/};

  // Same normal for all vertices
  nvmath::vec3f nrm = {0, 1, 0};
  for(auto& v : vert)
  {
    v.nrm = nrm;
  }

  // Storing the mesh in a buffer of vertices and a buffer of indices
  vk::ExportMemoryAllocateInfo exportAllocInfo(vk::ExternalMemoryHandleTypeFlagBits::eOpaqueWin32);
  Meshes                       mesh;
  mesh.indexCount  = (uint32_t)indices.size();
  mesh.vertexCount = (uint32_t)vert.size();
  {
    nvvk::ScopeCommandBuffer cmdBuf(m_device, m_graphicsQueueIndex);
    mesh.vertices.bufVk = m_alloc.createBuffer<Vertex>(cmdBuf, vert, vk::BufferUsageFlagBits::eVertexBuffer);
    mesh.indices.bufVk  = m_alloc.createBuffer<uint32_t>(cmdBuf, indices, vk::BufferUsageFlagBits::eIndexBuffer);
  }
  m_alloc.finalizeAndReleaseStaging();

  vulkanMeshToOpenGL(mesh);
  return mesh;
}

//--------------------------------------------------------------------------------------------------
// Converting the Vulkan Buffer for the Mesh to VAO
//
void VkGlExample::vulkanMeshToOpenGL(Meshes& mesh)
{
  createBufferGL(mesh.vertices, m_dmaAllocGL);
  createBufferGL(mesh.indices, m_dmaAllocGL);


  int pos_loc = 0;  // See shaderGL.vert location
  int nrm_loc = 1;
  int uv_loc  = 2;
  glCreateVertexArrays(1, &mesh.oglID);
  glEnableVertexArrayAttrib(mesh.oglID, pos_loc);
  glEnableVertexArrayAttrib(mesh.oglID, nrm_loc);
  glEnableVertexArrayAttrib(mesh.oglID, uv_loc);

  glVertexArrayAttribFormat(mesh.oglID, pos_loc, 3, GL_FLOAT, GL_FALSE, offsetof(Vertex, pos));
  glVertexArrayAttribBinding(mesh.oglID, pos_loc, 0);
  glVertexArrayAttribFormat(mesh.oglID, nrm_loc, 3, GL_FLOAT, GL_FALSE, offsetof(Vertex, nrm));
  glVertexArrayAttribBinding(mesh.oglID, nrm_loc, 0);
  glVertexArrayAttribFormat(mesh.oglID, uv_loc, 2, GL_FLOAT, GL_FALSE, offsetof(Vertex, uv));
  glVertexArrayAttribBinding(mesh.oglID, uv_loc, 0);

  glVertexArrayVertexBuffer(mesh.oglID, 0, mesh.vertices.oglId, 0, sizeof(Vertex));
  glVertexArrayElementBuffer(mesh.oglID, mesh.indices.oglId);
}

//--------------------------------------------------------------------------------------------------
// Converting the Mesh to Vulkan raytracing geometry
//
vk::GeometryNV VkGlExample::meshToGeometry(const Meshes& mesh)
{
  vk::GeometryTrianglesNV triangles;
  triangles.setVertexData(mesh.vertices.bufVk.buffer);
  triangles.setVertexOffset(0);
  triangles.setVertexCount(mesh.vertexCount);
  triangles.setVertexStride(sizeof(Vertex));
  triangles.setVertexFormat(vk::Format::eR32G32B32Sfloat);  // Limitation to 3xfloat32 for vertices
  triangles.setIndexData(mesh.indices.bufVk.buffer);
  triangles.setIndexOffset(0);
  triangles.setIndexCount(mesh.indexCount);
  triangles.setIndexType(vk::IndexType::eUint32);  // Limitation to 32-bit indices
  vk::GeometryDataNV geoData;
  geoData.setTriangles(triangles);
  vk::GeometryNV geometry;
  geometry.setGeometry(geoData);
  geometry.setFlags(vk::GeometryFlagBitsNV::eOpaque);

  return geometry;
}

//--------------------------------------------------------------------------------------------------
// Creating the buffers to store all matrices of all objects
//
void VkGlExample::prepareUniformBuffers()
{
  // Adding all matrices in a single buffer.
  std::vector<nvmath::mat4f> allMatrices;
  allMatrices.reserve(m_instances.size());
  for(auto& inst : m_instances)
  {
    allMatrices.push_back(inst.transform);
  }

  {
    nvvk::ScopeCommandBuffer cmdBuf(m_device, m_graphicsQueueIndex);
    m_uniformBuffers.matrices.bufVk =
        m_alloc.createBuffer<nvmath::mat4f>(cmdBuf, allMatrices, vk::BufferUsageFlagBits::eStorageBuffer);
    createBufferGL(m_uniformBuffers.matrices, m_dmaAllocGL);
  }
  m_alloc.finalizeAndReleaseStaging();
}

//--------------------------------------------------------------------------------
// The main display for this window...
//
void VkGlExample::onWindowRefresh()
{
  // Draw in G-Buffers the scene
  glBindFramebuffer(GL_FRAMEBUFFER, m_gFramebuffer);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, m_size.width, m_size.height);
  CheckGLError();

  // Set the camera matrix for the raster
  m_shaderRaster.use();
  const float   aspectRatio = m_size.width / static_cast<float>(m_size.height);
  nvmath::mat4f modelView   = CameraManip.getMatrix();  // Retrieving the camera matrix
  nvmath::mat4f proj        = nvmath::perspective(CameraManip.getFov(), aspectRatio, 0.001f, 1000.0f);
  nvmath::mat4f matrix      = proj * modelView;
  m_shaderRaster.setMat4("u_ViewProjectionMatrix", matrix);

  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, UBO_MATRIX, 0);  // Bind the buffer containing all instance matrices
  glBindTextureUnit(0, m_imageVkGL.oglId);                    // Bind the NVIDIA Logo

  // Draw all tetrahedrons
  glBindVertexArray(m_meshes[0].oglID);
  m_shaderRaster.setBool("u_hasTexture", true);
  m_shaderRaster.setVec3("u_color", 1, 1, 1);
  for(int i = 0; i < NB_MAX_TETRAHEDRONS; i++)
  {
    // Offset for the instance matrix
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, UBO_MATRIX, m_uniformBuffers.matrices.oglId,  //
                      sizeof(nvmath::mat4f) * i, sizeof(nvmath::mat4f));
    glDrawElements(GL_TRIANGLES, m_meshes[0].indexCount, GL_UNSIGNED_INT, nullptr);
  }

  // Draw grid plane
  size_t planeIndex         = m_meshes.size() - 1;     // Last mesh
  size_t planeInstanceIndex = m_instances.size() - 1;  // Last instance
  glBindVertexArray(m_meshes[planeIndex].oglID);
  m_shaderRaster.setBool("u_hasTexture", false);
  m_shaderRaster.setVec3("u_color", .7f, .7f, .7f);
  glBindBufferRange(GL_SHADER_STORAGE_BUFFER, UBO_MATRIX, m_uniformBuffers.matrices.oglId,  //
                    sizeof(nvmath::mat4f) * planeInstanceIndex, sizeof(nvmath::mat4f));
  glDrawElements(GL_TRIANGLES, m_meshes[planeIndex].indexCount, GL_UNSIGNED_INT, nullptr);

  // Done rendering in FBO
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // Raytracing the scene
  GLuint outRayID = m_ray.outputImage().oglId;
  {
    if(needToResetFrame())
      m_frameNumber = 0;
    // Once render is complete, signal the Vulkan semaphore indicating it can start render
    GLenum dstLayout = GL_LAYOUT_SHADER_READ_ONLY_EXT;
    GLenum srcLayout = GL_LAYOUT_COLOR_ATTACHMENT_EXT;
    glSignalSemaphoreEXT(m_ray.semaphores().glComplete, 0, nullptr, 1, &outRayID, &dstLayout);
    m_ray.run(m_frameNumber);
    // And wait (on the GPU) for the raytraced image
    glWaitSemaphoreEXT(m_ray.semaphores().glReady, 0, nullptr, 1, &outRayID, &srcLayout);
    m_frameNumber++;
  }

  // Recompose the image
  glClearColor(0.5f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  m_shaderComposite.use();
  // Controls
  m_shaderComposite.setBool("ao_use", m_aoUse);
  m_shaderComposite.setBool("ao_do_blur", m_aoBlur);
  m_shaderComposite.setInt("ao_blur_radius", m_aoBlurRadius);
  m_shaderComposite.setInt("buffer_view", m_bufferView);
  // Textures
  glBindTextureUnit(0, m_gBufferColor[0].oglId);  // position
  glBindTextureUnit(1, m_gBufferColor[1].oglId);  // normal
  glBindTextureUnit(2, m_gBufferColor[2].oglId);  // albedo
  glBindTextureUnit(3, outRayID);                 // occlusion
  // Draw full quad
  glDrawArrays(GL_TRIANGLES, 0, 3);
}

//--------------------------------------------------------------------------------------------------
// Return the current frame number
// Check if the camera matrix has changed, if yes, then reset the frame to 0
// otherwise, increment
//
bool VkGlExample::needToResetFrame()
{
  static nvmath::mat4f refCamMatrix;

  for(int i = 0; i < 16; i++)
  {
    if(CameraManip.getMatrix().mat_array[i] != refCamMatrix.mat_array[i])
    {
      refCamMatrix = CameraManip.getMatrix();
      return true;
    }
  }

  return false;
}

//--------------------------------------------------------------------------------------------------
// When the frames are redone, we also need to re-record the command buffer
//
void VkGlExample::onResize(int w, int h)
{
  if(w == 0 || h == 0)
    return;
  m_frameNumber = 0;
  createGBuffers();
  m_ray.createOutputImage(m_size);
}

//--------------------------------------------------------------------------------------------------
// Load NVIDIA logo for Vulkan and OpenGL
//
void VkGlExample::loadImage(const std::string& filename)
{
  int            w, h, comp;
  unsigned char* dataImage = stbi_load(filename.c_str(), &w, &h, &comp, STBI_rgb_alpha);
  size_t         dataSize  = w * h * 4;

  auto imgSize = vk::Extent2D(w, h);
  auto imgInfo = nvvk::makeImage2DCreateInfo(imgSize, vk::Format::eR8G8B8A8Unorm, vk::ImageUsageFlagBits::eSampled, true);


  {
    nvvk::ScopeCommandBuffer cmdBuf(m_device, m_graphicsQueueIndex);
    nvvk::ImageDma image = m_alloc.createImage(cmdBuf, dataSize, dataImage, imgInfo, vk::ImageLayout::eShaderReadOnlyOptimal);
    nvvk::cmdGenerateMipmaps(cmdBuf, image.image, vk::Format::eR8G8B8A8Unorm, imgSize, imgInfo.mipLevels);
    vk::ImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, imgInfo);
    vk::SamplerCreateInfo   defaultSampler;
    m_imageVkGL.texVk = m_alloc.createTexture(image, ivInfo, defaultSampler);
  }

  stbi_image_free(dataImage);

  m_imageVkGL.imgSize   = imgSize;
  m_imageVkGL.mipLevels = imgInfo.mipLevels;

  createTextureGL(m_imageVkGL, GL_RGBA8, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_REPEAT, m_dmaAllocGL);
}


//--------------------------------------------------------------------------------------------------
// Creating the OpenGL shaders
//
void VkGlExample::createShaders()
{
  m_shaderRaster = Shader(nvh::findFile("shaders/shaderGL.vert", defaultSearchPaths),
                          nvh::findFile("shaders/shaderGL.frag", defaultSearchPaths));

  m_shaderComposite = Shader(nvh::findFile("shaders/passthrough.vert", defaultSearchPaths),
                             nvh::findFile("shaders/composite.frag", defaultSearchPaths));
}

//--------------------------------------------------------------------------------------------------
//
//
void VkGlExample::drawUI()
{
  using Gui = ImGuiH::Control;
  ImGui_ImplGlfw_NewFrame();

  bool modified = false;
  ImGui::NewFrame();

  ImGuiH::Panel::Begin(ImGuiH::Panel::Side::Right);
  {
    if(ImGui::CollapsingHeader("Ambient Occlusion", ImGuiTreeNodeFlags_DefaultOpen))
    {
      int defInt{2};
      modified |= Gui::Checkbox("Use", "", &m_aoUse);
      modified |= Gui::Checkbox("Blur", "", &m_aoBlur);
      modified |= Gui::Slider("Radius", "", &m_aoBlurRadius, &defInt, ImGuiH::Control::Flags::Normal, 1, 5);
    }
    if(ImGui::CollapsingHeader("Raytrace AO", ImGuiTreeNodeFlags_DefaultOpen))
    {
      static interop::RtInterop::PushConstant d{};
      modified |= Gui::Slider("samples", "", &m_ray.m_pushC.rtao_samples, &d.rtao_samples, ImGuiH::Control::Flags::Normal, 1, 256);
      modified |= Gui::Slider("radius", "", &m_ray.m_pushC.rtao_radius, &d.rtao_radius, ImGuiH::Control::Flags::Normal,
                              0.5f, 10.f, "%.1f");
      modified |= Gui::Slider("power", "", &m_ray.m_pushC.rtao_power, &d.rtao_power, ImGuiH::Control::Flags::Normal, 1.f,
                              3.f, "%.1f");
    }
    if(ImGui::CollapsingHeader("Buffers", ImGuiTreeNodeFlags_DefaultOpen))
    {
      ImGui::Columns(2, "cl");
      ImGui::RadioButton("Final", &m_bufferView, 0);
      ImGui::RadioButton("Position", &m_bufferView, 1);
      ImGui::RadioButton("Normal", &m_bufferView, 2);
      ImGui::NextColumn();
      ImGui::RadioButton("Depth", &m_bufferView, 3);
      ImGui::RadioButton("Albedo", &m_bufferView, 4);
      ImGui::RadioButton("AO", &m_bufferView, 5);
      ImGui::Columns(1);
    }
    ImGui::Separator();
    Gui::Info("", "", "Press F10 to hide", ImGuiH::Control::Flags::Disabled);
  }
  ImGui::End();
  ImGui::Render();
  ImGui::RenderDrawDataGL(ImGui::GetDrawData());

  if(modified)
    m_frameNumber = 0;
}

//--------------------------------------------------------------------------------------------------
//
//
void VkGlExample::createGBuffers()
{
  m_queue.waitIdle();
  m_device.waitIdle();

  if(m_gFramebuffer != 0)
  {
    glDeleteFramebuffers(1, &m_gFramebuffer);
    m_gBufferColor[0].destroy(m_alloc);
    m_gBufferColor[1].destroy(m_alloc);
    m_gBufferColor[2].destroy(m_alloc);
  }

  glCreateFramebuffers(1, &m_gFramebuffer);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_gFramebuffer);


  vk::DeviceSize imgSize = m_size.width * m_size.height * 4 * sizeof(float);

  auto samplerInfo = vk::SamplerCreateInfo();

  m_gBufferColor[0].imgSize = m_size;
  m_gBufferColor[1].imgSize = m_size;
  m_gBufferColor[2].imgSize = m_size;

  vk::ImageCreateInfo floatInfo = nvvk::makeImage2DCreateInfo(m_size, vk::Format::eR32G32B32A32Sfloat);
  vk::ImageCreateInfo unormInfo = nvvk::makeImage2DCreateInfo(m_size, vk::Format::eR8G8B8A8Unorm);

  std::array<nvvk::ImageDma, 3> image;
  image[0] = m_alloc.createImage(floatInfo);
  image[1] = m_alloc.createImage(floatInfo);
  image[2] = m_alloc.createImage(unormInfo);
  std::array<vk::ImageViewCreateInfo, 3> ivInfo;
  ivInfo[0] = nvvk::makeImageViewCreateInfo(image[0].image, floatInfo);
  ivInfo[1] = nvvk::makeImageViewCreateInfo(image[1].image, floatInfo);
  ivInfo[2] = nvvk::makeImageViewCreateInfo(image[2].image, unormInfo);

  m_gBufferColor[0].texVk = m_alloc.createTexture(image[0], ivInfo[0], samplerInfo);
  m_gBufferColor[1].texVk = m_alloc.createTexture(image[1], ivInfo[1], samplerInfo);
  m_gBufferColor[2].texVk = m_alloc.createTexture(image[2], ivInfo[2], samplerInfo);


  createTextureGL(m_gBufferColor[0], GL_RGBA32F, GL_NEAREST, GL_NEAREST, GL_REPEAT, m_dmaAllocGL);
  createTextureGL(m_gBufferColor[1], GL_RGBA32F, GL_NEAREST, GL_NEAREST, GL_REPEAT, m_dmaAllocGL);
  createTextureGL(m_gBufferColor[2], GL_RGBA8, GL_NEAREST, GL_NEAREST, GL_REPEAT, m_dmaAllocGL);

  glNamedFramebufferTexture(m_gFramebuffer, GL_COLOR_ATTACHMENT0, m_gBufferColor[0].oglId, 0);
  glNamedFramebufferTexture(m_gFramebuffer, GL_COLOR_ATTACHMENT1, m_gBufferColor[1].oglId, 0);
  glNamedFramebufferTexture(m_gFramebuffer, GL_COLOR_ATTACHMENT2, m_gBufferColor[2].oglId, 0);

  {
    nvvk::ScopeCommandBuffer cmdBuf(m_device, m_graphicsQueueIndex);
    nvvk::cmdBarrierImageLayout(cmdBuf, m_gBufferColor[0].texVk.image, vk::ImageLayout::eUndefined,
                                vk::ImageLayout::eShaderReadOnlyOptimal);
    nvvk::cmdBarrierImageLayout(cmdBuf, m_gBufferColor[1].texVk.image, vk::ImageLayout::eUndefined,
                                vk::ImageLayout::eShaderReadOnlyOptimal);
    nvvk::cmdBarrierImageLayout(cmdBuf, m_gBufferColor[2].texVk.image, vk::ImageLayout::eUndefined,
                                vk::ImageLayout::eShaderReadOnlyOptimal);
  }

  // - tell OpenGL which color attachments we'll use (of this framebuffer) for rendering
  unsigned int attachments[3] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
  glDrawBuffers(3, attachments);


  // create and attach depth buffer (render buffer)
  unsigned int rboDepth;
  glGenRenderbuffers(1, &rboDepth);
  glBindRenderbuffer(GL_RENDERBUFFER, rboDepth);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, m_size.width, m_size.height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rboDepth);
  // finally check if framebuffer is complete
  if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
  {
    std::cout << "Framebuffer not complete!" << std::endl;
  }
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}


//--------------------------------------------------------------------------------------------------
// Catch for SPACE and F
//
void VkGlExample::onKeyboardChar(unsigned char key)
{
  nvvk::AppBase::onKeyboardChar(key);
  if(ImGui::GetIO().WantCaptureKeyboard)
  {
    return;
  }

  if(key == ' ' || key == 'i' || key == 'I')
  {
    double x, y;
    glfwGetCursorPos(m_window, &x, &y);
    nvmath::vec4f pixelf;
    glGetTextureSubImage(m_gBufferColor[0].oglId, 0, x, m_size.height - y, 0, 1, 1, 1, GL_RGBA, GL_FLOAT,
                         sizeof(nvmath::vec4f), &pixelf);
    nvmath::vec3f eye, ctr, up;
    CameraManip.getLookat(eye, ctr, up);
    CameraManip.setLookat(eye, nvmath::vec3f(pixelf), up, false);
  }

  // Frame the BBOX
  if(key == 'f')
  {
    fitCamera({-5., 0, -5}, {5, 2, 5}, false);
  }
}

//--------------------------------------------------------------------------------------------------
// Catch for home 'reset camera'
//
void VkGlExample::onKeyboard(int key, int scancode, int action, int mods)
{
  nvvk::AppBase::onKeyboard(key, scancode, action, mods);

  if(key == GLFW_KEY_HOME)
  {
    CameraManip.setLookat({10, 10, 10}, {0, 0, 0}, {0, 1, 0}, false);
  }
}
