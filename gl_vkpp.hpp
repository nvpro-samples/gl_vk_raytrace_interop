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

#include <vulkan/vulkan.hpp>

#include "nvvk/allocator_dma_vk.hpp"
#include "nvvk/memorymanagement_vkgl.hpp"
#ifdef WIN32
#include <handleapi.h>
#endif
#include <nvgl/extensions_gl.hpp>


//--------------------------------------------------------------------------------------------------
// This holds the buffer and texture Vulkan-OpenGL variation
// and the utilities to create the OpenGL version of buffers and textures from the Vulkan objects
//


namespace interop {

// #VKGL Extra for Interop
struct BufferVkGL
{
  nvvk::BufferDma bufVk;  // The allocated buffer

  GLuint oglId = 0;  // Extra: OpenGL object ID

  void destroy(nvvk::AllocatorDma& alloc)
  {
    alloc.destroy(bufVk);
    glDeleteBuffers(1, &oglId);
  }
};

// #VKGL Extra for Interop
struct Texture2DVkGL
{
  nvvk::TextureDma texVk;

  GLuint       oglId{0};  // Extra: OpenGL object ID
  uint32_t     mipLevels{1};
  vk::Extent2D imgSize{0, 0};

  void destroy(nvvk::AllocatorDma& alloc)
  {
    alloc.destroy(texVk);
    glDeleteBuffers(1, &oglId);
  }
};

// Get the Vulkan buffer and create the OpenGL equivalent using the memory allocated in Vulkan
inline void createBufferGL(BufferVkGL& bufGl, nvvk::DeviceMemoryAllocatorGL& memAllocGL)
{
  nvvk::AllocationGL allocGL = memAllocGL.getAllocationGL(bufGl.bufVk.allocation);

  glCreateBuffers(1, &bufGl.oglId);
  glNamedBufferStorageMemEXT(bufGl.oglId, allocGL.size, allocGL.memoryObject, allocGL.offset);
}

// Get the Vulkan texture and create the OpenGL equivalent using the memory allocated in Vulkan
inline void createTextureGL(Texture2DVkGL& texGl, int format, int minFilter, int magFilter, int wrap, nvvk::DeviceMemoryAllocatorGL& memAllocGL)
{
  auto allocGL = memAllocGL.getAllocationGL(texGl.texVk.allocation);

  // Create a 'memory object' in OpenGL, and associate it with the memory allocated in Vulkan
  glCreateTextures(GL_TEXTURE_2D, 1, &texGl.oglId);
  glTextureStorageMem2DEXT(texGl.oglId, texGl.mipLevels, format, texGl.imgSize.width, texGl.imgSize.height,
                           allocGL.memoryObject, allocGL.offset);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_MIN_FILTER, minFilter);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_MAG_FILTER, magFilter);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_WRAP_S, wrap);
  glTextureParameteri(texGl.oglId, GL_TEXTURE_WRAP_T, wrap);
}


}  // namespace nvvkpp
