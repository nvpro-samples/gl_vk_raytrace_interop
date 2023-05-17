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


#pragma once

//////////////////////////////////////////////////////////////////////////
// Raytracing implementation for the Vulkan Interop (G-Buffers)
//////////////////////////////////////////////////////////////////////////

#include "gl_vkpp.hpp"
#include "nvvk/raytraceNV_vk.hpp"


namespace interop {

struct RtInterop
{
  using vkDT   = VkDescriptorType;
  using vkSS   = VkShaderStageFlagBits;
  using vkCB   = VkCommandBufferUsageFlagBits;
  using vkDSLB = VkDescriptorSetLayoutBinding;

  // Information push at each call
  struct PushConstant
  {
    int   rtao_samples = 64;
    float rtao_radius  = 5.F;
    float rtao_power   = 2.F;
    int   frame_number = 0;
  } m_pushC;

  // For synchronizing with OpenGL
  struct Semaphore
  {
    VkSemaphore vkReady;
    VkSemaphore vkComplete;
    GLuint      glReady    = 0;
    GLuint      glComplete = 0;
  } m_semaphores;

  // Default constructor
  RtInterop() = default;

  // Accessors
  const Semaphore&              semaphores() const { return m_semaphores; }
  const interop::Texture2DVkGL& outputImage() const { return m_rtOutputGL; }
  nvvk::RaytracingBuilderNV&    builder() { return m_rtBuilder; }

  //
  void setup(const VkDevice& device, const VkPhysicalDevice& physicalDevice, uint32_t queueIndex);
  void destroy();
  void createOutputImage(VkExtent2D size);
  void createDescriptorSet(const std::vector<interop::Texture2DVkGL>& gBuffers);
  void updateDescriptorSet(const std::vector<interop::Texture2DVkGL>& gBuffers);
  void createPipeline();
  void createShadingBindingTable();
  void createSemaphores();
  void run(int frame_number);


private:
  // Allocator and memory manager
  ResourceAllocatorGLInterop m_allocGL;
  nvvk::ResourceAllocatorDma m_alloc;
  //
  VkDevice m_device     = VK_NULL_HANDLE;
  uint32_t m_queueIndex = 0;

  // Ray tracing specific
  interop::Texture2DVkGL    m_rtOutputGL;
  nvvk::Buffer              m_rtSBTBuffer;
  nvvk::RaytracingBuilderNV m_rtBuilder;
  VkCommandPool             m_rtCmdPool        = VK_NULL_HANDLE;
  VkCommandBuffer           m_rtCmdBuffer      = VK_NULL_HANDLE;  // CmdBuf use for ray tracing
  VkQueue                   m_rtQueue          = VK_NULL_HANDLE;
  VkDescriptorPool          m_rtDescPool       = VK_NULL_HANDLE;
  VkDescriptorSetLayout     m_rtDescSetLayout  = VK_NULL_HANDLE;
  VkDescriptorSet           m_rtDescSet        = VK_NULL_HANDLE;
  VkPipelineLayout          m_rtPipelineLayout = VK_NULL_HANDLE;
  VkPipeline                m_rtPipeline       = VK_NULL_HANDLE;

  // Properties
  VkPhysicalDeviceRayTracingPropertiesNV m_rtProperties{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PROPERTIES_NV};
};
}  // namespace interop
