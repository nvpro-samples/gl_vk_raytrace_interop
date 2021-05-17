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

#include "nvmath/nvmath.h"

#include "gl_vkpp.hpp"
#include "nvh/fileoperations.hpp"
#include "nvvk/raytraceNV_vk.hpp"


namespace interop {

struct RtInterop
{
  using vkDT   = vk::DescriptorType;
  using vkSS   = vk::ShaderStageFlagBits;
  using vkCB   = vk::CommandBufferUsageFlagBits;
  using vkDSLB = vk::DescriptorSetLayoutBinding;

  // Information push at each call
  struct PushConstant
  {
    int   rtao_samples = 64;
    float rtao_radius  = 5.f;
    float rtao_power   = 2.f;
    int   frame_number = 0;
  } m_pushC;

  // For synchronizing with OpenGL
  struct Semaphore
  {
    vk::Semaphore vkReady;
    vk::Semaphore vkComplete;
    GLuint        glReady    = 0;
    GLuint        glComplete = 0;
  } m_semaphores;

  // Default constructor
  RtInterop() = default;

  // Accessors
  const Semaphore&              semaphores() const { return m_semaphores; }
  const interop::Texture2DVkGL& outputImage() const { return m_rtOutputGL; }
  nvvk::RaytracingBuilderNV&    builder() { return m_rtBuilder; }

  //
  void setup(const vk::Device& device, const vk::PhysicalDevice& physicalDevice, uint32_t queueIndex);
  void destroy();
  void createOutputImage(vk::Extent2D size);
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
  vk::Device m_device;
  uint32_t   m_queueIndex;

  // Ray tracing specific
  interop::Texture2DVkGL    m_rtOutputGL;
  vk::CommandPool           m_rtCmdPool;
  vk::CommandBuffer         m_rtCmdBuffer;  // CmdBuf use for ray tracing
  vk::Queue                 m_rtQueue;
  nvvk::Buffer           m_rtSBTBuffer;
  nvvk::RaytracingBuilderNV m_rtBuilder;
  vk::DescriptorPool        m_rtDescPool;
  vk::DescriptorSetLayout   m_rtDescSetLayout;
  vk::DescriptorSet         m_rtDescSet;
  vk::PipelineLayout        m_rtPipelineLayout;
  vk::Pipeline              m_rtPipeline;

  // Properties
  vk::PhysicalDeviceRayTracingPropertiesNV m_rtProperties;
};
}  // namespace interop
