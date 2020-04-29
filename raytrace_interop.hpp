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

#pragma once

//////////////////////////////////////////////////////////////////////////
// Raytracing implementation for the Vulkan Interop (G-Buffers)
//////////////////////////////////////////////////////////////////////////

#define NVVK_ALLOC_DMA
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
  nvvk::DeviceMemoryAllocatorGL m_dmaAllocGL;  // Allocator for the result image
  nvvk::DeviceMemoryAllocator   m_dmaAlloc;    // Allocator for the ray tracer
  nvvk::AllocatorDma            m_allocGL;
  nvvk::AllocatorDma            m_alloc;
  //
  vk::Device m_device;
  uint32_t   m_queueIndex;

  // Ray tracing specific
  interop::Texture2DVkGL    m_rtOutputGL;
  vk::CommandPool           m_rtCmdPool;
  vk::CommandBuffer         m_rtCmdBuffer;  // CmdBuf use for ray tracing
  vk::Queue                 m_rtQueue;
  nvvk::BufferDma           m_rtSBTBuffer;
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
