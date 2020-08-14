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


#include "raytrace_interop.hpp"
#include "nvvk/shaders_vk.hpp"

extern std::vector<std::string> defaultSearchPaths;

//--------------------------------------------------------------------------------------------------
// Initializing the allocator GL which will be use to exchange with OpenGL (output image)
// and DMA allocator for the ray tracer and other buffers
//
void interop::RtInterop::setup(const vk::Device& device, const vk::PhysicalDevice& physicalDevice, uint32_t queueIndex)
{
  m_device     = device;
  m_queueIndex = queueIndex;

  // Requesting raytracing properties
  auto properties = physicalDevice.getProperties2<vk::PhysicalDeviceProperties2, vk::PhysicalDeviceRayTracingPropertiesNV>();
  m_rtProperties = properties.get<vk::PhysicalDeviceRayTracingPropertiesNV>();

  // Using an allocator to exchange the result image
  m_dmaAllocGL.init(device, physicalDevice);
  m_allocGL.init(device, physicalDevice, &m_dmaAllocGL);

  // Allocator for all the rest
  m_dmaAlloc.init(device, physicalDevice);
  m_alloc.init(device, physicalDevice, &m_dmaAlloc);

  // BLAS and TLAS builder
  m_rtBuilder.setup(device, &m_alloc, queueIndex);

  // Command pool as queue family for graphics, even if raytracing
  m_rtCmdPool   = m_device.createCommandPool({vk::CommandPoolCreateFlagBits::eResetCommandBuffer, queueIndex});
  m_rtCmdBuffer = m_device.allocateCommandBuffers({m_rtCmdPool, vk::CommandBufferLevel::ePrimary, 1})[0];
  m_rtQueue     = m_device.getQueue(queueIndex, 0);
}

//--------------------------------------------------------------------------------------------------
//
//
void interop::RtInterop::destroy()
{
  m_device.destroySemaphore(m_semaphores.vkComplete);
  m_device.destroySemaphore(m_semaphores.vkReady);
  m_rtOutputGL.destroy(m_allocGL);
  m_device.freeCommandBuffers(m_rtCmdPool, m_rtCmdBuffer);
  m_device.destroyCommandPool(m_rtCmdPool);
  m_rtBuilder.destroy();

  m_device.destroy(m_rtDescPool);
  m_device.destroy(m_rtDescSetLayout);
  m_device.destroy(m_rtPipeline);
  m_device.destroy(m_rtPipelineLayout);
  m_alloc.destroy(m_rtSBTBuffer);

  m_alloc.deinit();
  m_allocGL.deinit();

  m_dmaAllocGL.deinit();
  m_dmaAlloc.deinit();
}

//--------------------------------------------------------------------------------------------------
// Create the image containing the ambient occlusion information.
// The values are stored in a eR32Sfloat / GL_R32F image
//
void interop::RtInterop::createOutputImage(vk::Extent2D size)
{
  auto usage = vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eStorage | vk::ImageUsageFlagBits::eTransferSrc;
  vk::DeviceSize imgSize = size.width * size.height * 4 * sizeof(float);
  auto           format  = vk::Format::eR32Sfloat;
  auto           layout  = vk::ImageLayout::eGeneral;

  vk::SamplerCreateInfo samplerCreateInfo;  // default values
  vk::ImageCreateInfo   imageCreateInfo = nvvk::makeImage2DCreateInfo(size, format, usage);

  // Creating the image and the descriptor
  nvvk::ImageDma          image  = m_allocGL.createImage(imageCreateInfo);
  vk::ImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, imageCreateInfo);
  m_rtOutputGL.texVk             = m_allocGL.createTexture(image, ivInfo, samplerCreateInfo);
  m_rtOutputGL.imgSize           = size;
  {
    // Setting the layout to eGeneral
    nvvk::ScopeCommandBuffer cmdBuf(m_device, m_queueIndex);
    nvvk::cmdBarrierImageLayout(cmdBuf, m_rtOutputGL.texVk.image, vk::ImageLayout::eUndefined, layout);
  }

  // Making the OpenGL version of texture
  createTextureGL(m_rtOutputGL, GL_R32F, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_REPEAT, m_dmaAllocGL);
}

//--------------------------------------------------------------------------------------------------
//
//
void interop::RtInterop::createDescriptorSet(const std::vector<interop::Texture2DVkGL>& gBuffers)
{
  // Descriptor Pool
  {
    std::vector<vk::DescriptorPoolSize> poolSizes = {{vkDT::eAccelerationStructureNV, 1},  // AS
                                                     {vkDT::eStorageImage, 1},             // Output Image
                                                     {vkDT::eCombinedImageSampler, 3}};    // G-Buffers
    m_rtDescPool = m_device.createDescriptorPool({{}, 3, (uint32_t)poolSizes.size(), poolSizes.data()});
  }

  // Ray Layout
  {
    std::vector<vk::DescriptorSetLayoutBinding> setLayoutBindings = {
        {0, vkDT::eAccelerationStructureNV, 1, vkSS::eRaygenNV},  // AS
        {1, vkDT::eStorageImage, 1, vkSS::eRaygenNV},             // Output Image
        {2, vkDT::eCombinedImageSampler, 3, vkSS::eRaygenNV},     // G-Buffers
    };
    m_rtDescSetLayout = m_device.createDescriptorSetLayout({{}, (uint32_t)setLayoutBindings.size(), setLayoutBindings.data()});
  }

  // Descriptor Set
  m_rtDescSet = m_device.allocateDescriptorSets({m_rtDescPool, 1, &m_rtDescSetLayout})[0];

  // (0) Bind the actual resources into the descriptor set Top-level acceleration structure
  {
    vk::AccelerationStructureNV                   tlas = m_rtBuilder.getAccelerationStructure();
    vk::WriteDescriptorSetAccelerationStructureNV descAsInfo{1, &tlas};
    vk::WriteDescriptorSet                        wds{m_rtDescSet, 0, 0, 1, vkDT::eAccelerationStructureNV};
    wds.setPNext(&descAsInfo);
    m_device.updateDescriptorSets(wds, nullptr);
  }

  updateDescriptorSet(gBuffers);
}

//--------------------------------------------------------------------------------------------------
// Will be called when resizing the window
//
void interop::RtInterop::updateDescriptorSet(const std::vector<interop::Texture2DVkGL>& gBuffers)
{
  // (1) Output buffer
  {
    vk::DescriptorImageInfo imageInfo{{}, m_rtOutputGL.texVk.descriptor.imageView, vk::ImageLayout::eGeneral};
    vk::WriteDescriptorSet  wds{m_rtDescSet, 1, 0, 1, vkDT::eStorageImage, &imageInfo};
    m_device.updateDescriptorSets(wds, nullptr);
  }

  // (2) G-Buffers
  {
    std::vector<vk::DescriptorImageInfo> imageInfo{
        {gBuffers[0].texVk.descriptor.sampler, gBuffers[0].texVk.descriptor.imageView, vk::ImageLayout::eShaderReadOnlyOptimal},
        {gBuffers[1].texVk.descriptor.sampler, gBuffers[1].texVk.descriptor.imageView, vk::ImageLayout::eShaderReadOnlyOptimal},
        {gBuffers[2].texVk.descriptor.sampler, gBuffers[2].texVk.descriptor.imageView, vk::ImageLayout::eShaderReadOnlyOptimal},
    };
    vk::WriteDescriptorSet wds{m_rtDescSet, 2, 0, 3, vkDT::eCombinedImageSampler, imageInfo.data()};
    m_device.updateDescriptorSets(wds, nullptr);
  }
}

//--------------------------------------------------------------------------------------------------
// Loading the ray tracer shaders aand creating the shader groups
//
void interop::RtInterop::createPipeline()
{
  std::vector<std::string> paths = defaultSearchPaths;
  vk::ShaderModule raygenSM = nvvk::createShaderModule(m_device, nvh::loadFile("shaders/raygen.rgen.spv", true, paths));
  vk::ShaderModule missSM   = nvvk::createShaderModule(m_device, nvh::loadFile("shaders/miss.rmiss.spv", true, paths));

  std::vector<vk::PipelineShaderStageCreateInfo>     stages;
  std::vector<vk::RayTracingShaderGroupCreateInfoNV> groups;

  // Raygen
  stages.push_back({{}, vk::ShaderStageFlagBits::eRaygenNV, raygenSM, "main"});
  groups.emplace_back(vk::RayTracingShaderGroupTypeNV::eGeneral, 0, VK_SHADER_UNUSED_NV, VK_SHADER_UNUSED_NV, VK_SHADER_UNUSED_NV);
  // Miss
  stages.push_back({{}, vk::ShaderStageFlagBits::eMissNV, missSM, "main"});
  groups.emplace_back(vk::RayTracingShaderGroupTypeNV::eGeneral, 1, VK_SHADER_UNUSED_NV, VK_SHADER_UNUSED_NV, VK_SHADER_UNUSED_NV);
  // Hit (empty - not needed)

  vk::PushConstantRange        pushConstant{vk::ShaderStageFlagBits::eRaygenNV, 0, sizeof(PushConstant)};
  vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo;
  pipelineLayoutCreateInfo.setSetLayoutCount(1);
  pipelineLayoutCreateInfo.setPSetLayouts(&m_rtDescSetLayout);
  pipelineLayoutCreateInfo.setPushConstantRangeCount(1);
  pipelineLayoutCreateInfo.setPPushConstantRanges(&pushConstant);
  m_rtPipelineLayout = m_device.createPipelineLayout(pipelineLayoutCreateInfo);

  // Assemble the shader stages and recursion depth info into the raytracing pipeline
  vk::RayTracingPipelineCreateInfoNV rayPipelineInfo;
  rayPipelineInfo.setStageCount(static_cast<uint32_t>(stages.size()));
  rayPipelineInfo.setPStages(stages.data());
  rayPipelineInfo.setGroupCount(static_cast<uint32_t>(groups.size()));
  rayPipelineInfo.setPGroups(groups.data());
  rayPipelineInfo.setMaxRecursionDepth(2);
  rayPipelineInfo.setLayout(m_rtPipelineLayout);
  m_rtPipeline = static_cast<const vk::Pipeline&>(m_device.createRayTracingPipelineNV({}, rayPipelineInfo));

  m_device.destroyShaderModule(raygenSM);
  m_device.destroyShaderModule(missSM);
}

//--------------------------------------------------------------------------------------------------
// Creating a buffer with the handles of the shader groups
//
void interop::RtInterop::createShadingBindingTable()
{
  uint32_t groupCount      = 2;                                        // Two shaders: raygen, miss
  uint32_t groupHandleSize = m_rtProperties.shaderGroupHandleSize;     // Size of a program handle
  uint32_t groupSize       = m_rtProperties.shaderGroupBaseAlignment;  // Size of a program identifier

  // Fetch all the shader handles used in the pipeline, so that they can be written in the SBT
  uint32_t             sbtSize = groupCount * groupSize;
  std::vector<uint8_t> shaderHandleStorage(sbtSize);
  m_device.getRayTracingShaderGroupHandlesNV(m_rtPipeline, 0, groupCount, sbtSize, shaderHandleStorage.data());

  m_rtSBTBuffer = m_alloc.createBuffer(sbtSize, vk::BufferUsageFlagBits::eTransferSrc, vk::MemoryPropertyFlagBits::eHostVisible);

  // Write the handles in the SBT
  auto* pData = reinterpret_cast<uint8_t*>(m_alloc.map(m_rtSBTBuffer));
  memcpy(pData, shaderHandleStorage.data() + 0 * groupHandleSize, groupHandleSize);  // raygen
  pData += groupSize;
  memcpy(pData, shaderHandleStorage.data() + 1 * groupHandleSize, groupHandleSize);  // miss
  m_alloc.unmap(m_rtSBTBuffer);
}

//--------------------------------------------------------------------------------------------------
// Creating the semaphores of syncing with OpenGL
//
void interop::RtInterop::createSemaphores()
{
#ifdef WIN32
  HANDLE glReadyHandle{INVALID_HANDLE_VALUE};
  HANDLE glCompleteHandle{INVALID_HANDLE_VALUE};
#else
  int glReadyHandle{-1};
  int glCompleteHandle{-1};
#endif

  // Vulkan
  {
#ifdef WIN32
    auto handleType = vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueWin32;
#else
    auto handleType  = vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd;
#endif
    {
      vk::SemaphoreCreateInfo       sci;
      vk::ExportSemaphoreCreateInfo esci;
      sci.pNext               = &esci;
      esci.handleTypes        = handleType;
      m_semaphores.vkReady    = m_device.createSemaphore(sci);
      m_semaphores.vkComplete = m_device.createSemaphore(sci);
    }
#ifdef WIN32
    glReadyHandle    = m_device.getSemaphoreWin32HandleKHR({m_semaphores.vkReady, handleType});
    glCompleteHandle = m_device.getSemaphoreWin32HandleKHR({m_semaphores.vkComplete, handleType});
#else
    glReadyHandle    = m_device.getSemaphoreFdKHR({m_semaphores.vkReady, handleType});
    glCompleteHandle = m_device.getSemaphoreFdKHR({m_semaphores.vkComplete, handleType});
#endif
  }

  // OpenGL
  {
    // Import semaphores
    glGenSemaphoresEXT(1, &m_semaphores.glReady);
    glGenSemaphoresEXT(1, &m_semaphores.glComplete);
#ifdef WIN32
    glImportSemaphoreWin32HandleEXT(m_semaphores.glReady, GL_HANDLE_TYPE_OPAQUE_WIN32_EXT, glReadyHandle);
    glImportSemaphoreWin32HandleEXT(m_semaphores.glComplete, GL_HANDLE_TYPE_OPAQUE_WIN32_EXT, glCompleteHandle);
#else
    glImportSemaphoreFdEXT(m_semaphores.glReady, GL_HANDLE_TYPE_OPAQUE_FD_EXT, glReadyHandle);
    glImportSemaphoreFdEXT(m_semaphores.glComplete, GL_HANDLE_TYPE_OPAQUE_FD_EXT, glCompleteHandle);
#endif
  }
}

//--------------------------------------------------------------------------------------------------
// Executing ray tracer
//
void interop::RtInterop::run(int frame_number)
{
  uint32_t progSize = m_rtProperties.shaderGroupBaseAlignment;  // Size of a program identifier

  m_pushC.frame_number = frame_number;

  m_rtCmdBuffer.begin({vk::CommandBufferUsageFlagBits::eSimultaneousUse});

  m_rtCmdBuffer.bindPipeline(vk::PipelineBindPoint::eRayTracingNV, m_rtPipeline);
  m_rtCmdBuffer.bindDescriptorSets(vk::PipelineBindPoint::eRayTracingNV, m_rtPipelineLayout, 0, {m_rtDescSet}, {});
  m_rtCmdBuffer.pushConstants<PushConstant>(m_rtPipelineLayout, vk::ShaderStageFlagBits::eRaygenNV, 0, m_pushC);

  vk::DeviceSize rayGenOffset   = 0;
  vk::DeviceSize missOffset     = progSize;
  vk::DeviceSize missStride     = progSize;
  vk::DeviceSize hitGroupOffset = 0;  // unused
  vk::DeviceSize hitGroupStride = 0;

  m_rtCmdBuffer.traceRaysNV(m_rtSBTBuffer.buffer, rayGenOffset, m_rtSBTBuffer.buffer, missOffset, missStride,
                            m_rtSBTBuffer.buffer, hitGroupOffset, hitGroupStride, vk::Buffer{}, 0, 0,
                            m_rtOutputGL.imgSize.width, m_rtOutputGL.imgSize.height, 1);

  m_rtCmdBuffer.end();

  vk::SubmitInfo         submitInfo;
  vk::PipelineStageFlags stageFlags = vk::PipelineStageFlagBits::eRayTracingShaderNV;
  submitInfo.pWaitDstStageMask      = &stageFlags;
  submitInfo.waitSemaphoreCount     = 1;
  submitInfo.pWaitSemaphores        = &m_semaphores.vkComplete;
  submitInfo.signalSemaphoreCount   = 1;
  submitInfo.pSignalSemaphores      = &m_semaphores.vkReady;
  submitInfo.commandBufferCount     = 1;
  submitInfo.pCommandBuffers        = &m_rtCmdBuffer;
  m_rtQueue.submit(submitInfo, vk::Fence());
}
