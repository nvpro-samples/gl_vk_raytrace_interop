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

#include <vulkan/vulkan.hpp>

#ifdef WIN32
#include <handleapi.h>
#endif
#include <nvgl/extensions_gl.hpp>

#include <nvvk/memallocator_dma_vk.hpp>
#include <nvvk/memorymanagement_vkgl.hpp>
#include <nvvk/resourceallocator_vk.hpp>

#include <memory>

#include "utility_ogl.hpp"
//--------------------------------------------------------------------------------------------------
// This holds the buffer and texture Vulkan-OpenGL variation
// and the utilities to create the OpenGL version of buffers and textures from the Vulkan objects
//


namespace interop {

// ResourceAllocatorDmaGL is a convenience class owning a DMAMemoryAllocator and DeviceMemoryAllocatorGL object.
// By deriving from nvvk::ExportResourceAllocator we make sure all created objects' underlying memory
// will be exportable.
class ResourceAllocatorGLInterop : public nvvk::ExportResourceAllocator
{
public:
  ResourceAllocatorGLInterop() = default;
  ResourceAllocatorGLInterop(VkDevice device, VkPhysicalDevice physicalDevice, VkDeviceSize stagingBlockSize = NVVK_DEFAULT_STAGING_BLOCKSIZE)
  {
    init(device, physicalDevice, stagingBlockSize);
  }
  ~ResourceAllocatorGLInterop() { deinit(); }

  void init(VkDevice device, VkPhysicalDevice physicalDevice, VkDeviceSize stagingBlockSize = NVVK_DEFAULT_STAGING_BLOCKSIZE)
  {
    m_dmaGL    = std::make_unique<nvvk::DeviceMemoryAllocatorGL>(device, physicalDevice);
    m_memAlloc = std::make_unique<nvvk::DMAMemoryAllocator>(m_dmaGL.get());
    nvvk::ResourceAllocator::init(device, physicalDevice, m_memAlloc.get(), stagingBlockSize);
  }

  void deinit()
  {
    m_memAlloc.reset();
    m_dmaGL.reset();
    nvvk::ExportResourceAllocator::deinit();
  }

  nvvk::DeviceMemoryAllocatorGL& getDmaGL() const { return *m_dmaGL; }
  nvvk::DMAMemoryAllocator*      getMemoryAllocator() const { return m_memAlloc.get(); }
  nvvk::AllocationGL             getAllocationGL(nvvk::MemHandle memHandle) const
  {
    return m_dmaGL->getAllocationGL(m_memAlloc->getAllocationID(memHandle));
  }

protected:
  std::unique_ptr<nvvk::DeviceMemoryAllocatorGL> m_dmaGL;
  std::unique_ptr<nvvk::DMAMemoryAllocator>      m_memAlloc;
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

  GLuint       oglId{0};  // Extra: OpenGL object ID
  uint32_t     mipLevels{1};
  vk::Extent2D imgSize{0, 0};

  void destroy(nvvk::ResourceAllocator& alloc)
  {
    alloc.destroy(texVk);
    glDeleteBuffers(1, &oglId);
  }
};

// Get the Vulkan buffer and create the OpenGL equivalent using the memory allocated in Vulkan
inline void createBufferGL(BufferVkGL& bufGl, ResourceAllocatorGLInterop& memAllocGL)
{
  nvvk::AllocationGL allocGL = memAllocGL.getAllocationGL(bufGl.bufVk.memHandle);

  glCreateBuffers(1, &bufGl.oglId);
  glNamedBufferStorageMemEXT(bufGl.oglId, allocGL.size, allocGL.memoryObject, allocGL.offset);
}

// Get the Vulkan texture and create the OpenGL equivalent using the memory allocated in Vulkan
inline void createTextureGL(Texture2DVkGL& texGl, int format, int minFilter, int magFilter, int wrap, ResourceAllocatorGLInterop& memAllocGL)
{
  auto allocGL = memAllocGL.getAllocationGL(texGl.texVk.memHandle);

  // Create a 'memory object' in OpenGL, and associate it with the memory allocated in Vulkan
  glCreateTextures(GL_TEXTURE_2D, 1, &texGl.oglId);
  glTextureStorageMem2DEXT(texGl.oglId, texGl.mipLevels, format, texGl.imgSize.width, texGl.imgSize.height,
                           allocGL.memoryObject, allocGL.offset);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_MIN_FILTER, minFilter);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_MAG_FILTER, magFilter);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_WRAP_S, wrap);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_WRAP_T, wrap);
}

}  // namespace interop
