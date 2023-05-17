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

#include <vulkan/vulkan.h>

#ifdef WIN32
#include <handleapi.h>
#endif
#include <nvgl/extensions_gl.hpp>

#include <nvvk/memallocator_dma_vk.hpp>
#include <nvvk/memorymanagement_vkgl.hpp>
#include <nvvk/resourceallocator_vk.hpp>

#include <memory>

#include "utility_ogl.hpp"
#include "nvvk/stagingmemorymanager_vk.hpp"


//--------------------------------------------------------------------------------------------------
// This holds the buffer and texture Vulkan-OpenGL variation
// and the utilities to create the OpenGL version of buffers and textures from the Vulkan objects
//


namespace interop {

//--------------------------------------------------------------------------------------------------
// ResourceAllocatorGLInterop is a ResourceAllocator using DeviceMemoryAllocatorGL as underlying allocator.
// Vulkan device memory allocations will be immediately imported into GL. The corresponding GL memory object
// can be queried via getAllocationGL()
class ResourceAllocatorGLInterop : public nvvk::ExportResourceAllocator
{
public:
  ResourceAllocatorGLInterop() = default;
  ResourceAllocatorGLInterop(VkDevice device, VkPhysicalDevice physicalDevice, VkDeviceSize stagingBlockSize = NVVK_DEFAULT_STAGING_BLOCKSIZE)
  {
    init(device, physicalDevice, stagingBlockSize);
  }
  ~ResourceAllocatorGLInterop() override { deinit(); }

  void init(VkDevice device, VkPhysicalDevice physicalDevice, VkDeviceSize stagingBlockSize = NVVK_DEFAULT_STAGING_BLOCKSIZE)
  {
    m_dmaGL = std::make_unique<nvvk::DeviceMemoryAllocatorGL>(device, physicalDevice);
    nvvk::ExportResourceAllocator::init(device, physicalDevice, m_dmaGL.get(), stagingBlockSize);

    // The staging will only use DMA, without export functionality.
    m_dma = std::make_unique<nvvk::DeviceMemoryAllocator>(device, physicalDevice);
    m_staging = std::make_unique<nvvk::StagingMemoryManager>(dynamic_cast<nvvk::MemAllocator*>(m_dma.get()), stagingBlockSize);
  }

  void deinit()
  {
    nvvk::ExportResourceAllocator::deinit();
    m_dmaGL.reset();
    m_dma.reset();
  }

  nvvk::DeviceMemoryAllocatorGL& getDmaGL() const { return *m_dmaGL; }
  nvvk::AllocationGL             getAllocationGL(nvvk::MemHandle memHandle) const
  {
    return m_dmaGL->getAllocationGL(m_dmaGL->getAllocationID(memHandle));
  }

protected:
  std::unique_ptr<nvvk::DeviceMemoryAllocatorGL> m_dmaGL;
  std::unique_ptr<nvvk::DeviceMemoryAllocator>   m_dma;
};


// #VKGL Extra for Interop
struct BufferVkGL
{
  nvvk::Buffer bufVk;  // The allocated buffer

  GLuint oglId = 0;  // Extra: OpenGL object ID

  void destroy(nvvk::ResourceAllocator& alloc)
  {
    alloc.destroy(bufVk);
    glDeleteBuffers(1, &oglId);
  }
};

// #VKGL Extra for Interop
struct Texture2DVkGL
{
  nvvk::Texture texVk;

  GLuint     oglId{0};  // Extra: OpenGL object ID
  uint32_t   mipLevels{1};
  VkExtent2D imgSize{0, 0};

  void destroy(nvvk::ResourceAllocator& alloc)
  {
    alloc.destroy(texVk);
    glDeleteBuffers(1, &oglId);
  }
};

// Get the Vulkan buffer and create the OpenGL equivalent using the memory allocated in Vulkan
inline void createBufferGL(BufferVkGL& bufGl, ResourceAllocatorGLInterop& memAllocGL)
{
  const nvvk::AllocationGL allocGL = memAllocGL.getAllocationGL(bufGl.bufVk.memHandle);

  glCreateBuffers(1, &bufGl.oglId);
  glNamedBufferStorageMemEXT(bufGl.oglId, static_cast<GLsizeiptr>(allocGL.size), allocGL.memoryObject, allocGL.offset);
}

// Get the Vulkan texture and create the OpenGL equivalent using the memory allocated in Vulkan
inline void createTextureGL(Texture2DVkGL& texGl, int format, int minFilter, int magFilter, int wrap, ResourceAllocatorGLInterop& memAllocGL)
{
  const nvvk::AllocationGL allocGL = memAllocGL.getAllocationGL(texGl.texVk.memHandle);

  // Create a 'memory object' in OpenGL, and associate it with the memory allocated in Vulkan
  glCreateTextures(GL_TEXTURE_2D, 1, &texGl.oglId);
  glTextureStorageMem2DEXT(texGl.oglId, static_cast<GLsizei>(texGl.mipLevels), format,
                           static_cast<GLsizei>(texGl.imgSize.width), static_cast<GLsizei>(texGl.imgSize.height),
                           allocGL.memoryObject, allocGL.offset);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_MIN_FILTER, minFilter);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_MAG_FILTER, magFilter);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_WRAP_S, wrap);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_WRAP_T, wrap);
}

}  // namespace interop
