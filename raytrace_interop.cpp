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


#include "raytrace_interop.hpp"
#include "nvvk/shaders_vk.hpp"
#include "nvvk/error_vk.hpp"
#include "nvvk/buffers_vk.hpp"
#include "nvh/fileoperations.hpp"

extern std::vector<std::string> defaultSearchPaths;

//--------------------------------------------------------------------------------------------------
// Initializing the allocator GL which will be use to exchange with OpenGL (output image)
// and DMA allocator for the ray tracer and other buffers
//
void interop::RtInterop::setup(const VkDevice& device, const VkPhysicalDevice& physicalDevice, uint32_t queueIndex)
{
  m_device     = device;
  m_queueIndex = queueIndex;

  // Requesting raytracing properties
  VkPhysicalDeviceProperties2            prop2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};
  prop2.pNext = &m_rtProperties;
  vkGetPhysicalDeviceProperties2(physicalDevice, &prop2);

  // Using an allocator to exchange the result image
  m_allocGL.init(device, physicalDevice);

  // Allocator for all the rest. Share the memory allocator with m_allocGL
  m_alloc.init(device, physicalDevice);

  // BLAS and TLAS builder
  m_rtBuilder.setup(device, &m_alloc, queueIndex);

  // Command pool as queue family for graphics, even if raytracing
  VkCommandPoolCreateInfo createInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
  createInfo.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  createInfo.queueFamilyIndex = queueIndex;
  NVVK_CHECK(vkCreateCommandPool(m_device, &createInfo, nullptr, &m_rtCmdPool));

  VkCommandBufferAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
  allocateInfo.commandBufferCount = 1;
  allocateInfo.commandPool        = m_rtCmdPool;
  allocateInfo.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  NVVK_CHECK(vkAllocateCommandBuffers(m_device, &allocateInfo, &m_rtCmdBuffer));

  vkGetDeviceQueue(m_device, queueIndex, 0, &m_rtQueue);
}

//--------------------------------------------------------------------------------------------------
//
//
void interop::RtInterop::destroy()
{
  vkDestroySemaphore(m_device, m_semaphores.vkComplete, nullptr);
  vkDestroySemaphore(m_device, m_semaphores.vkReady, nullptr);
  vkFreeCommandBuffers(m_device, m_rtCmdPool, 1, &m_rtCmdBuffer);
  vkDestroyCommandPool(m_device, m_rtCmdPool, nullptr);
  vkDestroyDescriptorPool(m_device, m_rtDescPool, nullptr);
  vkDestroyDescriptorSetLayout(m_device, m_rtDescSetLayout, nullptr);
  vkDestroyPipeline(m_device, m_rtPipeline, nullptr);
  vkDestroyPipelineLayout(m_device, m_rtPipelineLayout, nullptr);

  m_rtOutputGL.destroy(m_allocGL);
  m_rtBuilder.destroy();
  m_alloc.destroy(m_rtSBTBuffer);
  m_alloc.deinit();
  m_allocGL.deinit();
}

//--------------------------------------------------------------------------------------------------
// Create the image containing the ambient occlusion information.
// The values are stored in a VK_FORMAT_R32_SFLOAT / GL_R32F image
//
void interop::RtInterop::createOutputImage(VkExtent2D size)
{
  auto          usage   = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
  VkDeviceSize  imgSize = size.width * size.height * 4 * sizeof(float);
  VkFormat      format  = VK_FORMAT_R32_SFLOAT;
  VkImageLayout layout  = VK_IMAGE_LAYOUT_GENERAL;

  VkSamplerCreateInfo samplerCreateInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};  // default values
  VkImageCreateInfo   imageCreateInfo = nvvk::makeImage2DCreateInfo(size, format, usage);

  // Creating the image and the descriptor
  nvvk::Image           image  = m_allocGL.createImage(imageCreateInfo);
  VkImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(VkImage(image.image), imageCreateInfo);
  m_rtOutputGL.texVk           = m_allocGL.createTexture(image, ivInfo, samplerCreateInfo);
  m_rtOutputGL.imgSize         = size;
  {
    // Setting the layout to eGeneral
    nvvk::ScopeCommandBuffer cmdBuf(m_device, m_queueIndex);
    nvvk::cmdBarrierImageLayout(cmdBuf, m_rtOutputGL.texVk.image, VK_IMAGE_LAYOUT_UNDEFINED, layout);
  }

  // Making the OpenGL version of texture
  createTextureGL(m_rtOutputGL, GL_R32F, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_REPEAT, m_allocGL);
}

//--------------------------------------------------------------------------------------------------
//
//
void interop::RtInterop::createDescriptorSet(const std::vector<interop::Texture2DVkGL>& gBuffers)
{
  // Descriptor Pool
  {
    std::vector<VkDescriptorPoolSize> poolSizes = {{VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_NV, 1},  // AS
                                                   {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1},              // Output Image
                                                   {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 3}};    // G-Buffers

    VkDescriptorPoolCreateInfo createInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    createInfo.maxSets       = 3;
    createInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
    createInfo.pPoolSizes    = poolSizes.data();
    NVVK_CHECK(vkCreateDescriptorPool(m_device, &createInfo, nullptr, &m_rtDescPool));
  }

  // Ray Layout
  {
    std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
        {0, VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_NV, 1, VK_SHADER_STAGE_RAYGEN_BIT_NV},  // AS
        {1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_RAYGEN_BIT_NV},              // Output Image
        {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 3, VK_SHADER_STAGE_RAYGEN_BIT_NV},     // G-Buffers
    };
    VkDescriptorSetLayoutCreateInfo createInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    createInfo.bindingCount = static_cast<uint32_t>(setLayoutBindings.size());
    createInfo.pBindings    = setLayoutBindings.data();
    NVVK_CHECK(vkCreateDescriptorSetLayout(m_device, &createInfo, nullptr, &m_rtDescSetLayout));
  }

  // Descriptor Set
  VkDescriptorSetAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
  allocateInfo.descriptorPool     = m_rtDescPool;
  allocateInfo.descriptorSetCount = 1;
  allocateInfo.pSetLayouts        = &m_rtDescSetLayout;
  NVVK_CHECK(vkAllocateDescriptorSets(m_device, &allocateInfo, &m_rtDescSet));

  // (0) Bind the actual resources into the descriptor set Top-level acceleration structure
  {
    VkAccelerationStructureNV tlas = m_rtBuilder.getAccelerationStructure();
    VkWriteDescriptorSetAccelerationStructureNV descAsInfo{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_NV};
    descAsInfo.accelerationStructureCount = 1;
    descAsInfo.pAccelerationStructures    = &tlas;
    VkWriteDescriptorSet wds{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    wds.dstBinding      = 0;
    wds.descriptorCount = 1;
    wds.descriptorType  = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_NV;
    wds.dstSet          = m_rtDescSet;
    wds.pNext           = &descAsInfo;
    vkUpdateDescriptorSets(m_device, 1, &wds, 0, nullptr);
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
    VkDescriptorImageInfo imageInfo{};
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    imageInfo.imageView   = m_rtOutputGL.texVk.descriptor.imageView;

    VkWriteDescriptorSet wds{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    wds.dstBinding      = 1;
    wds.descriptorCount = 1;
    wds.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    wds.dstSet          = m_rtDescSet;
    wds.pImageInfo      = &imageInfo;

    vkUpdateDescriptorSets(m_device, 1, &wds, 0, nullptr);
  }

  // (2) G-Buffers
  {
    std::vector<VkDescriptorImageInfo> imageInfo{
        {gBuffers[0].texVk.descriptor.sampler, gBuffers[0].texVk.descriptor.imageView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},
        {gBuffers[1].texVk.descriptor.sampler, gBuffers[1].texVk.descriptor.imageView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},
        {gBuffers[2].texVk.descriptor.sampler, gBuffers[2].texVk.descriptor.imageView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},
    };
    VkWriteDescriptorSet wds{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    wds.dstBinding      = 2;
    wds.descriptorCount = 3;
    wds.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    wds.dstSet          = m_rtDescSet;
    wds.pImageInfo      = imageInfo.data();

    vkUpdateDescriptorSets(m_device, 1, &wds, 0, nullptr);
  }
}

//--------------------------------------------------------------------------------------------------
// Loading the ray tracer shaders aand creating the shader groups
//
void interop::RtInterop::createPipeline()
{
  VkShaderModule raygenSM =
      nvvk::createShaderModule(m_device, nvh::loadFile("shaders/raygen.rgen.spv", true, defaultSearchPaths));
  VkShaderModule missSM = nvvk::createShaderModule(m_device, nvh::loadFile("shaders/miss.rmiss.spv", true, defaultSearchPaths));

  std::vector<VkPipelineShaderStageCreateInfo>      stages;
  std::vector<VkRayTracingShaderGroupCreateInfoNV> groups;

  // Raygen
  {
    VkPipelineShaderStageCreateInfo stage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    stage.module = raygenSM;
    stage.pName  = "main";
    stage.stage  = VK_SHADER_STAGE_RAYGEN_BIT_NV;
    stages.push_back(stage);
    VkRayTracingShaderGroupCreateInfoNV group{VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_NV};
    group.type               = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_NV;
    group.generalShader      = 0;
    group.anyHitShader       = VK_SHADER_UNUSED_NV;
    group.closestHitShader   = VK_SHADER_UNUSED_NV;
    group.intersectionShader = VK_SHADER_UNUSED_NV;
    groups.emplace_back(group);
  }
  // Miss
  {
    VkPipelineShaderStageCreateInfo stage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    stage.module = missSM;
    stage.pName  = "main";
    stage.stage  = VK_SHADER_STAGE_MISS_BIT_NV;
    stages.push_back(stage);
    VkRayTracingShaderGroupCreateInfoNV group{VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_NV};
    group.type               = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_NV;
    group.generalShader      = 1;
    group.anyHitShader       = VK_SHADER_UNUSED_NV;
    group.closestHitShader   = VK_SHADER_UNUSED_NV;
    group.intersectionShader = VK_SHADER_UNUSED_NV;
    groups.emplace_back(group);
  }

  // Hit (empty - not needed)
  {
  }

  VkPushConstantRange pushConstant{};
  pushConstant.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_NV;
  pushConstant.size       = sizeof(PushConstant);

  VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
  pipelineLayoutCreateInfo.setLayoutCount         = 1;
  pipelineLayoutCreateInfo.pSetLayouts            = &m_rtDescSetLayout;
  pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
  pipelineLayoutCreateInfo.pPushConstantRanges    = &pushConstant;
  NVVK_CHECK(vkCreatePipelineLayout(m_device, &pipelineLayoutCreateInfo, nullptr, &m_rtPipelineLayout));

  // Assemble the shader stages and recursion depth info into the raytracing pipeline
  VkRayTracingPipelineCreateInfoNV rayPipelineInfo{VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_NV};
  rayPipelineInfo.stageCount        = static_cast<uint32_t>(stages.size());
  rayPipelineInfo.pStages           = stages.data();
  rayPipelineInfo.groupCount        = static_cast<uint32_t>(groups.size());
  rayPipelineInfo.pGroups           = groups.data();
  rayPipelineInfo.maxRecursionDepth = 2;
  rayPipelineInfo.layout            = m_rtPipelineLayout;
  NVVK_CHECK(vkCreateRayTracingPipelinesNV(m_device, {}, 1, &rayPipelineInfo, nullptr, &m_rtPipeline));

  vkDestroyShaderModule(m_device, raygenSM, nullptr);
  vkDestroyShaderModule(m_device, missSM, nullptr);
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
  NVVK_CHECK(vkGetRayTracingShaderGroupHandlesNV(m_device, m_rtPipeline, 0, groupCount, sbtSize, shaderHandleStorage.data()));


  m_rtSBTBuffer = m_alloc.createBuffer(sbtSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);

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
    auto handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT;
#else
    auto handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT;
#endif
    {
      VkSemaphoreCreateInfo       sci{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
      VkExportSemaphoreCreateInfo esci{VK_STRUCTURE_TYPE_EXPORT_SEMAPHORE_CREATE_INFO};
      sci.pNext        = &esci;
      esci.handleTypes = handleType;
      vkCreateSemaphore(m_device, &sci, nullptr, &m_semaphores.vkReady);
      vkCreateSemaphore(m_device, &sci, nullptr, &m_semaphores.vkComplete);
    }
#ifdef WIN32
    {
      VkSemaphoreGetWin32HandleInfoKHR getWin32HandleInfo{VK_STRUCTURE_TYPE_SEMAPHORE_GET_WIN32_HANDLE_INFO_KHR};
      getWin32HandleInfo.handleType = handleType;
      getWin32HandleInfo.semaphore  = m_semaphores.vkReady;
      vkGetSemaphoreWin32HandleKHR(m_device, &getWin32HandleInfo, &glReadyHandle);
    }
    {
      VkSemaphoreGetWin32HandleInfoKHR getWin32HandleInfo{VK_STRUCTURE_TYPE_SEMAPHORE_GET_WIN32_HANDLE_INFO_KHR};
      getWin32HandleInfo.handleType = handleType;
      getWin32HandleInfo.semaphore  = m_semaphores.vkComplete;
      vkGetSemaphoreWin32HandleKHR(m_device, &getWin32HandleInfo, &glCompleteHandle);
    }
#else
    {
      VkSemaphoreGetFdInfoKHR getFdInfo{VK_STRUCTURE_TYPE_SEMAPHORE_GET_FD_INFO_KHR};
      getFdInfo.handleType = handleType;
      getFdInfo.semaphore  = m_semaphores.vkReady;
      vkGetSemaphoreFdKHR(m_device, &getFdInfo, &glReadyHandle);
    }
    {
      VkSemaphoreGetFdInfoKHR getFdInfo{VK_STRUCTURE_TYPE_SEMAPHORE_GET_FD_INFO_KHR};
      getFdInfo.handleType = handleType;
      getFdInfo.semaphore  = m_semaphores.vkComplete;
      vkGetSemaphoreFdKHR(m_device, &getFdInfo, &glCompleteHandle);
    }
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

  VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
  beginInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
  NVVK_CHECK(vkBeginCommandBuffer(m_rtCmdBuffer, &beginInfo));

  vkCmdBindPipeline(m_rtCmdBuffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_NV, m_rtPipeline);
  vkCmdBindDescriptorSets(m_rtCmdBuffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_NV, m_rtPipelineLayout, 0, 1, &m_rtDescSet, 0, nullptr);
  vkCmdPushConstants(m_rtCmdBuffer, m_rtPipelineLayout, VK_SHADER_STAGE_RAYGEN_BIT_NV, 0, sizeof(PushConstant), &m_pushC);


  VkDeviceSize rayGenOffset   = 0;
  VkDeviceSize missOffset     = progSize;
  VkDeviceSize missStride     = progSize;
  VkDeviceSize hitGroupOffset = 0;  // unused
  VkDeviceSize hitGroupStride = 0;

  //m_rtCmdBuffer.traceRaysNV(m_rtSBTBuffer.buffer, rayGenOffset, m_rtSBTBuffer.buffer, missOffset, missStride,
  //                          m_rtSBTBuffer.buffer, hitGroupOffset, hitGroupStride, VkBuffer{}, 0, 0,
  //                          m_rtOutputGL.imgSize.width, m_rtOutputGL.imgSize.height, 1);

  vkCmdTraceRaysNV(m_rtCmdBuffer, m_rtSBTBuffer.buffer, rayGenOffset, m_rtSBTBuffer.buffer, missOffset, missStride,
                   m_rtSBTBuffer.buffer, hitGroupOffset, hitGroupStride, VkBuffer{}, 0, 0, m_rtOutputGL.imgSize.width,
                   m_rtOutputGL.imgSize.height, 1);


  // VkDeviceAddress deviceAddress = nvvk::getBufferDeviceAddressKHR(m_device, m_rtSBTBuffer.buffer);
  //  VkStridedDeviceAddressRegionKHR rayTable{deviceAddress, 0, progSize};
  //  VkStridedDeviceAddressRegionKHR missTable{deviceAddress + progSize, 0, progSize};
  //  VkStridedDeviceAddressRegionKHR hitTable{};
  //  VkStridedDeviceAddressRegionKHR callTable{};
  //
  //  vkCmdTraceRaysNV(m_rtCmdBuffer, &rayTable, &missTable, &hitTable, &callTable, m_rtOutputGL.imgSize.width,
  //                    m_rtOutputGL.imgSize.height, 1);


  NVVK_CHECK(vkEndCommandBuffer(m_rtCmdBuffer));

  VkSubmitInfo         submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
  VkPipelineStageFlags stageFlags = VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_NV;
  submitInfo.pWaitDstStageMask    = &stageFlags;
  submitInfo.waitSemaphoreCount   = 1;
  submitInfo.pWaitSemaphores      = &m_semaphores.vkComplete;
  submitInfo.signalSemaphoreCount = 1;
  submitInfo.pSignalSemaphores    = &m_semaphores.vkReady;
  submitInfo.commandBufferCount   = 1;
  submitInfo.pCommandBuffers      = &m_rtCmdBuffer;
  NVVK_CHECK(vkQueueSubmit(m_rtQueue, 1, &submitInfo, {}));
  NVVK_CHECK(vkQueueWaitIdle(m_rtQueue));
}
