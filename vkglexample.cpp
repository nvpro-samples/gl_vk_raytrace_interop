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


// In this example,
// - Many tetrahedrons are created (createScene, createFacetedTetrahedron)
//   with alternative color, a plane and it lit with a directional light.
// - Each tetrahedron has it own matrix and all matrices will be placed in a single buffer.
// - It is possible to switch between a recorded command buffer, or simply drawing each
//   element individually.
// - By default, the rendering is done in multi-sampling and resolve in the back buffer


#include "vkglexample.h"
#include "backends/imgui_impl_glfw.h"
#include "nvh/fileoperations.hpp"
#include <random>  // Used to randomly place objects
#include <stb_image.h>

constexpr auto NB_MAX_TETRAHEDRONS = 10000;
constexpr auto UBO_MATRIX          = 0;

extern std::vector<std::string> defaultSearchPaths;

//--------------------------------------------------------------------------------------------------------------
// Setting up the AppBase, which in this application just holds Vulkan context, not using the framework.
// Create the memory allocator for doing the interop between OpenGL and Vulkan
// Create the ray tracer for ambient-occlusion
//
void VkGlExample::setup(const VkInstance& instance, const VkDevice& device, const VkPhysicalDevice& physicalDevice, uint32_t graphicsQueueIndex)
{
  nvvkhl::AppBaseVk::setup(instance, device, physicalDevice, graphicsQueueIndex);

  m_allocInterop.init(device, physicalDevice);

  m_ray.setup(device, physicalDevice, graphicsQueueIndex);
}

//--------------------------------------------------------------------------------------------------
// Creating the shaders, the scene, loading images, ...
//
void VkGlExample::createExample()
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
  loadImage(nvh::findFile(R"(data/nvidia_logo.png)", defaultSearchPaths), m_imageVkGL[0]);
  loadImage(nvh::findFile(R"(data/sun.png)", defaultSearchPaths), m_imageVkGL[1]);
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
      rayInst[i].instanceId = static_cast<uint32_t>(i);
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
  vkDeviceWaitIdle(m_device);

  for(auto& mesh : m_meshes)
    mesh.destroy(m_allocInterop);

  for(auto& img : m_imageVkGL)
    img.destroy(m_allocInterop);

  for(auto& buffer : m_gBufferColor)
    buffer.destroy(m_allocInterop);

  m_uniformBuffers.matrices.destroy(m_allocInterop);
  m_ray.destroy();
  m_allocInterop.deinit();
  m_allocInterop.deinit();
  AppBaseVk::destroy();
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
  std::normal_distribution<float> dis(0.0F, 2.0F);
  std::normal_distribution<float> disn(0.1F, 0.15F);

  Instance inst{};

  // Adding many tetrahedron
  m_meshes.push_back(createFacetedTetrahedron());
  for(int n = 0; n < NB_MAX_TETRAHEDRONS; ++n)
  {
    inst.objectIndex  = static_cast<uint32_t>(m_meshes.size() - 1);
    const float scale = disn(gen);
    inst.transform.as_translation(nvmath::vec3f(dis(gen), dis(gen), dis(gen)));
    inst.transform.rotate(dis(gen), nvmath::vec3f(1.0F, 0.0F, 0.0F));
    inst.transform.scale(nvmath::vec3f(scale, scale, scale));
    inst.materialIndex = n % 2;  // Alternating the material color
    m_instances.push_back(inst);
  }

  // Adding the plane
  m_meshes.push_back(createPlane());
  inst             = {};
  inst.objectIndex = static_cast<uint32_t>(m_meshes.size() - 1);
  inst.transform.as_translation(nvmath::vec3f(0.F, -5.F, 0.F));
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
  tpos[0]                    = nvmath::vec3f(sqrt(8.F / 9.F), 0, -1.F / 3.F);
  tpos[1]                    = nvmath::vec3f(-sqrt(2.F / 9.F), sqrt(2.F / 3.F), -1.F / 3.F);
  tpos[2]                    = nvmath::vec3f(-sqrt(2.F / 9.F), -sqrt(2.F / 3.F), -1.F / 3.F);
  tpos[3]                    = nvmath::vec3f(0, 0, 1);
  std::vector<uint32_t> tidx = {0, 2, 1, /*0*/ 0, 1, 3, /*1*/ 0, 3, 2, /*2*/ 1, 2, 3 /*3*/};

  std::vector<nvmath::vec2f> tuv = {{0, 0}, {1, 0}, {0.5F, 1}};

  vert.resize(4ULL * 3);     // 4 triangles
  indices.resize(4ULL * 3);  // 12 indices

  // Computing the normal for each vertex/triangle
  uint32_t vidx = 0;
  for(size_t t = 0; t < 4; ++t)
  {
    const nvmath::vec3f& v0  = tpos[tidx[t * 3 + 0]];
    const nvmath::vec3f& v1  = tpos[tidx[t * 3 + 1]];
    const nvmath::vec3f& v2  = tpos[tidx[t * 3 + 2]];
    const nvmath::vec3f  nrm = nvmath::normalize(nvmath::cross(v1 - v0, v2 - v0));

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
  Meshes mesh;
  mesh.indexCount  = static_cast<uint32_t>(indices.size());
  mesh.vertexCount = static_cast<uint32_t>(vert.size());

  // The buffers are created on Vulkan and shared with OpenGL
  {
    const nvvk::ScopeCommandBuffer cmd(m_device, m_graphicsQueueIndex);
    mesh.vertices.bufVk = m_allocInterop.createBuffer<Vertex>(cmd, vert, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    mesh.indices.bufVk  = m_allocInterop.createBuffer<uint32_t>(cmd, indices, VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
  }
  m_allocInterop.finalizeAndReleaseStaging();

  // Creating the OpenGL buffers from Vulkan
  vulkanMeshToOpenGL(mesh);

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
  const nvmath::vec3f nrm = {0.0F, 1.0F, 0.0F};
  for(auto& v : vert)
  {
    v.nrm = nrm;
  }

  // Storing the mesh in a buffer of vertices and a buffer of indices
  Meshes mesh;
  mesh.indexCount  = static_cast<uint32_t>(indices.size());
  mesh.vertexCount = static_cast<uint32_t>(vert.size());
  {
    const nvvk::ScopeCommandBuffer cmd(m_device, m_graphicsQueueIndex);
    mesh.vertices.bufVk = m_allocInterop.createBuffer<Vertex>(cmd, vert, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    mesh.indices.bufVk  = m_allocInterop.createBuffer<uint32_t>(cmd, indices, VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
  }
  m_allocInterop.finalizeAndReleaseStaging();

  // Creating the OpenGL buffers from Vulkan
  vulkanMeshToOpenGL(mesh);
  return mesh;
}

//--------------------------------------------------------------------------------------------------
// Converting the Vulkan Buffer for the Mesh to VAO
//
void VkGlExample::vulkanMeshToOpenGL(Meshes& mesh)
{
  createBufferGL(mesh.vertices, m_allocInterop);
  createBufferGL(mesh.indices, m_allocInterop);


  const int pos_loc = 0;  // See shaderGL.vert location
  const int nrm_loc = 1;
  const int uv_loc  = 2;
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
VkGeometryNV VkGlExample::meshToGeometry(const Meshes& mesh)
{
  VkGeometryTrianglesNV triangles{VK_STRUCTURE_TYPE_GEOMETRY_TRIANGLES_NV};
  triangles.vertexData   = mesh.vertices.bufVk.buffer;
  triangles.vertexOffset = 0;
  triangles.vertexCount  = mesh.vertexCount;
  triangles.vertexStride = sizeof(Vertex);
  triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;  // Limitation to 3xfloat32 for vertices
  triangles.indexData    = mesh.indices.bufVk.buffer;
  triangles.indexOffset  = 0;
  triangles.indexCount   = mesh.indexCount;
  triangles.indexType    = VK_INDEX_TYPE_UINT32;  // Limitation to 32-bit indices
  VkGeometryDataNV geoData{};
  geoData.triangles = triangles;
  geoData.aabbs     = {VK_STRUCTURE_TYPE_GEOMETRY_AABB_NV};
  VkGeometryNV geometry{VK_STRUCTURE_TYPE_GEOMETRY_NV};
  geometry.geometry = geoData;
  geometry.flags    = VK_GEOMETRY_OPAQUE_BIT_NV;

  return static_cast<VkGeometryNV>(geometry);
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
    const nvvk::ScopeCommandBuffer cmd(m_device, m_graphicsQueueIndex);
    m_uniformBuffers.matrices.bufVk =
        m_allocInterop.createBuffer<nvmath::mat4f>(cmd, allMatrices, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    createBufferGL(m_uniformBuffers.matrices, m_allocInterop);
  }
  m_allocInterop.finalizeAndReleaseStaging();
}

//--------------------------------------------------------------------------------
// The main display for this window...
//
void VkGlExample::onWindowRefresh()
{
  AppBaseVk::updateCamera();

  // Draw in G-Buffers the scene
  glBindFramebuffer(GL_FRAMEBUFFER, m_gFramebuffer);
  glClearColor(0.0F, 0.0F, 0.0F, 0.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, static_cast<GLsizei>(m_size.width), static_cast<GLsizei>(m_size.height));
  CheckGLError();

  // Set the camera matrix for the raster
  m_shaderRaster.use();
  const float         aspectRatio = static_cast<float>(m_size.width) / static_cast<float>(m_size.height);
  const nvmath::mat4f modelView   = CameraManip.getMatrix();  // Retrieving the camera matrix
  const nvmath::mat4f proj        = nvmath::perspective(CameraManip.getFov(), aspectRatio, 0.001F, 1000.0F);
  const nvmath::mat4f matrix      = proj * modelView;
  m_shaderRaster.setMat4("u_ViewProjectionMatrix", matrix);

  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, UBO_MATRIX, 0);  // Bind the buffer containing all instance matrices

  // Draw all tetrahedrons
  glBindVertexArray(m_meshes[0].oglID);
  m_shaderRaster.setBool("u_hasTexture", true);
  m_shaderRaster.setVec3("u_color", 1, 1, 1);
  for(int32_t i = 0; i < NB_MAX_TETRAHEDRONS; i++)
  {
    glBindTextureUnit(0, m_imageVkGL.at(i % 2).oglId);  // Bind the image, alternate between the two images

    // Offset for the instance matrix
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, UBO_MATRIX, m_uniformBuffers.matrices.oglId,  //
                      static_cast<GLintptr>(sizeof(nvmath::mat4f) * i), sizeof(nvmath::mat4f));
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_meshes[0].indexCount), GL_UNSIGNED_INT, nullptr);
  }

  // Draw grid plane
  const size_t planeIndex         = m_meshes.size() - 1;     // Last mesh
  const size_t planeInstanceIndex = m_instances.size() - 1;  // Last instance
  glBindVertexArray(m_meshes[planeIndex].oglID);
  m_shaderRaster.setBool("u_hasTexture", false);
  m_shaderRaster.setVec3("u_color", .7f, .7f, .7f);
  glBindBufferRange(GL_SHADER_STORAGE_BUFFER, UBO_MATRIX, m_uniformBuffers.matrices.oglId,  //
                    static_cast<GLintptr>(sizeof(nvmath::mat4f) * planeInstanceIndex), sizeof(nvmath::mat4f));
  glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_meshes[planeIndex].indexCount), GL_UNSIGNED_INT, nullptr);

  // Done rendering in FBO
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // Raytracing the scene
  const GLuint outRayID = m_ray.outputImage().oglId;
  {
    if(needToResetFrame())
      m_frameNumber = 0;
    // Once render is complete, signal the Vulkan semaphore indicating it can start render
    const GLenum dstLayout = GL_LAYOUT_SHADER_READ_ONLY_EXT;
    const GLenum srcLayout = GL_LAYOUT_COLOR_ATTACHMENT_EXT;
    glSignalSemaphoreEXT(m_ray.semaphores().glComplete, 0, nullptr, 1, &outRayID, &dstLayout);
    m_ray.run(m_frameNumber);
    // And wait (on the GPU) for the raytraced image
    glWaitSemaphoreEXT(m_ray.semaphores().glReady, 0, nullptr, 1, &outRayID, &srcLayout);
    m_frameNumber++;
  }

  // Recompose the image
  glClearColor(0.5F, 0.0F, 0.0F, 0.0F);
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

  for(int32_t i = 0; i < 16; i++)
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
void VkGlExample::loadImage(const std::string& filename, interop::Texture2DVkGL& img)
{
  int          w{0}, h{0}, comp{0};
  uint8_t*     dataImage = stbi_load(filename.c_str(), &w, &h, &comp, STBI_rgb_alpha);
  const size_t dataSize  = static_cast<size_t>(w) * h * 4;

  auto                    imgSize = VkExtent2D{static_cast<uint32_t>(w), static_cast<uint32_t>(h)};
  const VkImageCreateInfo imgInfo =
      nvvk::makeImage2DCreateInfo(imgSize, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_SAMPLED_BIT, true);


  {
    const nvvk::ScopeCommandBuffer cmd(m_device, m_graphicsQueueIndex);
    const nvvk::Image              image =
        m_allocInterop.createImage(cmd, dataSize, dataImage, imgInfo, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    nvvk::cmdGenerateMipmaps(cmd, image.image, VK_FORMAT_R8G8B8A8_UNORM, imgSize, imgInfo.mipLevels);
    const VkImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, imgInfo);
    const VkSamplerCreateInfo   defaultSampler{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    img.texVk = m_allocInterop.createTexture(image, ivInfo, defaultSampler);
  }

  stbi_image_free(dataImage);

  img.imgSize   = imgSize;
  img.mipLevels = imgInfo.mipLevels;

  createTextureGL(img, GL_RGBA8, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_REPEAT, m_allocInterop);
}


//--------------------------------------------------------------------------------------------------
// Creating the OpenGL shaders
//
void VkGlExample::createShaders()
{
  m_shaderRaster = OglShader(nvh::findFile("shaders/shaderGL.vert", defaultSearchPaths),
                             nvh::findFile("shaders/shaderGL.frag", defaultSearchPaths));

  m_shaderComposite = OglShader(nvh::findFile("shaders/passthrough.vert", defaultSearchPaths),
                                nvh::findFile("shaders/composite.frag", defaultSearchPaths));
}

//--------------------------------------------------------------------------------------------------
// Initialization of the GUI
// - Need to be call after the device creation
//
void VkGlExample::initUI(int width, int height)
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
void VkGlExample::onFramebufferSize(int w, int h)
{
  m_size.width  = w;
  m_size.height = h;
  CameraManip.setWindowSize(w, h);
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
                              0.5F, 10.F, "%.1f");
      modified |= Gui::Slider("power", "", &m_ray.m_pushC.rtao_power, &d.rtao_power, ImGuiH::Control::Flags::Normal, 1.F, 3.F, "%.1f");
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
  vkQueueWaitIdle(m_queue);
  vkDeviceWaitIdle(m_device);

  if(m_gFramebuffer != 0)
  {
    glDeleteFramebuffers(1, &m_gFramebuffer);
    m_gBufferColor[0].destroy(m_allocInterop);
    m_gBufferColor[1].destroy(m_allocInterop);
    m_gBufferColor[2].destroy(m_allocInterop);
  }

  glCreateFramebuffers(1, &m_gFramebuffer);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_gFramebuffer);


  const auto imgSize = static_cast<VkDeviceSize>(m_size.width) * m_size.height * 4 * sizeof(float);

  const VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};

  m_gBufferColor[0].imgSize = m_size;
  m_gBufferColor[1].imgSize = m_size;
  m_gBufferColor[2].imgSize = m_size;

  const VkImageCreateInfo floatInfo = nvvk::makeImage2DCreateInfo(m_size, VK_FORMAT_R32G32B32A32_SFLOAT);
  const VkImageCreateInfo unormInfo = nvvk::makeImage2DCreateInfo(m_size, VK_FORMAT_R8G8B8A8_UNORM);

  std::array<nvvk::Image, 3> image;
  image[0] = m_allocInterop.createImage(floatInfo);
  image[1] = m_allocInterop.createImage(floatInfo);
  image[2] = m_allocInterop.createImage(unormInfo);
  std::array<VkImageViewCreateInfo, 3> ivInfo{};
  ivInfo[0] = nvvk::makeImageViewCreateInfo(image[0].image, floatInfo);
  ivInfo[1] = nvvk::makeImageViewCreateInfo(image[1].image, floatInfo);
  ivInfo[2] = nvvk::makeImageViewCreateInfo(image[2].image, unormInfo);

  m_gBufferColor[0].texVk = m_allocInterop.createTexture(image[0], ivInfo[0], samplerInfo);
  m_gBufferColor[1].texVk = m_allocInterop.createTexture(image[1], ivInfo[1], samplerInfo);
  m_gBufferColor[2].texVk = m_allocInterop.createTexture(image[2], ivInfo[2], samplerInfo);


  createTextureGL(m_gBufferColor[0], GL_RGBA32F, GL_NEAREST, GL_NEAREST, GL_REPEAT, m_allocInterop);
  createTextureGL(m_gBufferColor[1], GL_RGBA32F, GL_NEAREST, GL_NEAREST, GL_REPEAT, m_allocInterop);
  createTextureGL(m_gBufferColor[2], GL_RGBA8, GL_NEAREST, GL_NEAREST, GL_REPEAT, m_allocInterop);

  glNamedFramebufferTexture(m_gFramebuffer, GL_COLOR_ATTACHMENT0, m_gBufferColor[0].oglId, 0);
  glNamedFramebufferTexture(m_gFramebuffer, GL_COLOR_ATTACHMENT1, m_gBufferColor[1].oglId, 0);
  glNamedFramebufferTexture(m_gFramebuffer, GL_COLOR_ATTACHMENT2, m_gBufferColor[2].oglId, 0);

  {
    const nvvk::ScopeCommandBuffer cmd(m_device, m_graphicsQueueIndex);
    nvvk::cmdBarrierImageLayout(cmd, m_gBufferColor[0].texVk.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    nvvk::cmdBarrierImageLayout(cmd, m_gBufferColor[1].texVk.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    nvvk::cmdBarrierImageLayout(cmd, m_gBufferColor[2].texVk.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  }

  // - tell OpenGL which color attachments we'll use (of this framebuffer) for rendering
  std::array<uint32_t, 3> attachments = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
  glDrawBuffers(3, attachments.data());


  // create and attach depth buffer (render buffer)
  GLuint rboDepth{0};
  glGenRenderbuffers(1, &rboDepth);
  glBindRenderbuffer(GL_RENDERBUFFER, rboDepth);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, static_cast<GLsizei>(m_size.width),
                        static_cast<GLsizei>(m_size.height));
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
  nvvkhl::AppBaseVk::onKeyboardChar(key);
  if(ImGui::GetIO().WantCaptureKeyboard)
  {
    return;
  }

  if(key == ' ' || key == 'i' || key == 'I')
  {
    double x{0.0}, y{0.0};
    glfwGetCursorPos(m_window, &x, &y);
    nvmath::vec4f pixelf;
    glGetTextureSubImage(m_gBufferColor[0].oglId, 0, static_cast<GLint>(x), static_cast<GLint>(m_size.height - y), 0, 1,
                         1, 1, GL_RGBA, GL_FLOAT, sizeof(nvmath::vec4f), &pixelf);
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
  nvvkhl::AppBaseVk::onKeyboard(key, scancode, action, mods);

  if(key == GLFW_KEY_HOME)
  {
    CameraManip.setLookat({10, 10, 10}, {0, 0, 0}, {0, 1, 0}, false);
  }
}
