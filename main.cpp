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


//--------------------------------------------------------------------------------------------------
// Very simple Vulkan-OpenGL example:
// - The vertex buffer is allocated with Vulkan, but used by OpenGL to render
// - The animation is updating the buffer allocated by Vulkan, and the changes are
//   reflected in the OGL render.
//

#include <accctrl.h>
#include <aclapi.h>
#include <array>
#include <chrono>

#include "vkglexample.h"
#include <nvgl/contextwindow_gl.hpp>
#include <nvgl/extensions_gl.hpp>
#include <nvvkpp/context_vkpp.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "nvvk/extensions_vk.hpp"
#include <fileformats/stb_image.h>

int const SAMPLE_SIZE_WIDTH  = 1200;
int const SAMPLE_SIZE_HEIGHT = 900;

// Default search path for shaders
std::vector<std::string> defaultSearchPaths{
    "./",
    "../",
    std::string(PROJECT_NAME),
    std::string("SPV_" PROJECT_NAME),
    PROJECT_ABSDIRECTORY,
    NVPSystem::exePath() + std::string(PROJECT_RELDIRECTORY),
};

//--------------------------------------------------------------------------------------------------
//
//
int main(int argc, char** argv)
{

  // -------------------------------
  // Basic OpenGL settings
  //
  nvgl::ContextWindowCreateInfo context(4,       //major;
                                        5,       //minor;
                                        false,   //core;
                                        1,       //MSAA;
                                        24,      //depth bits
                                        8,       //stencil bits
                                        false,   //debug;
                                        false,   //robust;
                                        false,   //forward;
                                        false,   //stereo;
                                        nullptr  //share;
  );

  // setup some basic things for the sample, logging file for example
  NVPSystem system(argv[0], PROJECT_NAME);

  nvvkpp::ContextCreateInfo deviceInfo;
  deviceInfo.addInstanceExtension(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
  deviceInfo.addInstanceExtension(VK_KHR_EXTERNAL_MEMORY_CAPABILITIES_EXTENSION_NAME);
  deviceInfo.addInstanceExtension(VK_KHR_EXTERNAL_SEMAPHORE_CAPABILITIES_EXTENSION_NAME);
  deviceInfo.addInstanceExtension(VK_KHR_EXTERNAL_FENCE_CAPABILITIES_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_MAINTENANCE1_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_MEMORY_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_MEMORY_WIN32_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_SEMAPHORE_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_SEMAPHORE_WIN32_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_FENCE_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_FENCE_WIN32_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_NV_RAY_TRACING_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);

  // Creating the Vulkan instance and device
  nvvkpp::Context vkctx;
  vkctx.init(deviceInfo);


  // Loading the C version of the extensions
  load_VK_EXTENSION_SUBSET(vkctx.m_instance, vkGetInstanceProcAddr, vkctx.m_device, vkGetDeviceProcAddr);


  VkGlExample         example;
  nvgl::ContextWindow contextWindowGL;

  // Creating the window
  example.open(0, 0, SAMPLE_SIZE_WIDTH, SAMPLE_SIZE_HEIGHT, PROJECT_NAME);

  // OpenGL context within the Window
  contextWindowGL.init(&context, example.m_internal, PROJECT_NAME);
  contextWindowGL.makeContextCurrent();
  contextWindowGL.swapInterval(0);

  // Loading all OpenGL symbols
  load_GL(nvgl::ContextWindow::sysGetProcAddress);

  // Printing which GPU we are using for Vulkan
  std::cout << "Using " << vkctx.m_physicalDevice.getProperties().deviceName << std::endl;

  // Setup the Vulkan base elements
  example.setup(vkctx.m_device, vkctx.m_physicalDevice, vkctx.m_queueGCT.familyIndex);

  // Initialize the window, UI ..
  example.initUI(SAMPLE_SIZE_WIDTH, SAMPLE_SIZE_HEIGHT);

  // Various ways of creating vertex buffers
  example.initExample();

  // Window system loop
  while(example.pollEvents() && !example.isClosing())
  {
    if(example.isOpen())
    {
      CameraManip.updateAnim();
      glClearColor(0.5f, 0.0f, 0.0f, 0.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glEnable(GL_DEPTH_TEST);

      auto& imgui_io       = ImGui::GetIO();
      imgui_io.DisplaySize = ImVec2(SAMPLE_SIZE_WIDTH, SAMPLE_SIZE_HEIGHT);

      example.onWindowRefresh();
      example.drawUI();

      contextWindowGL.swapBuffers();
    }
  }

  example.destroy();
  vkctx.deinit();
}
