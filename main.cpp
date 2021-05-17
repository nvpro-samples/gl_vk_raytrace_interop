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



//--------------------------------------------------------------------------------------------------
// Very simple Vulkan-OpenGL example:
// - The vertex buffer is allocated with Vulkan, but used by OpenGL to render
// - The animation is updating the buffer allocated by Vulkan, and the changes are
//   reflected in the OGL render.
//

#ifdef WIN32
#include <accctrl.h>
#include <aclapi.h>
#endif
#include <array>
#include <chrono>
#include <vulkan/vulkan.hpp>


#define STB_IMAGE_IMPLEMENTATION

#include "backends/imgui_impl_glfw.h"
#include "imgui.h"
#include "nvgl/contextwindow_gl.hpp"
#include "nvgl/extensions_gl.hpp"
#include "nvpsystem.hpp"
#include "nvvk/context_vk.hpp"
#include "nvvk/extensions_vk.hpp"
#include "stb_image.h"
#include "vkglexample.h"

VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE

int const SAMPLE_SIZE_WIDTH  = 1200;
int const SAMPLE_SIZE_HEIGHT = 900;

// Default search path for shaders
std::vector<std::string> defaultSearchPaths{
    "./",
    "../",
    std::string(PROJECT_NAME),
    std::string("SPV_" PROJECT_NAME),
    NVPSystem::exePath() + PROJECT_RELDIRECTORY,
    NVPSystem::exePath() + std::string(PROJECT_RELDIRECTORY),
};

//--------------------------------------------------------------------------------------------------
//
//
int main(int argc, char** argv)
{
  // setup some basic things for the sample, logging file for example
  NVPSystem system(PROJECT_NAME);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  // Create window with graphics context
  GLFWwindow* window = glfwCreateWindow(SAMPLE_SIZE_WIDTH, SAMPLE_SIZE_HEIGHT, PROJECT_NAME, NULL, NULL);
  if(window == nullptr)
    return 1;
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // Enable vsync

  nvvk::ContextCreateInfo deviceInfo;
  deviceInfo.addInstanceExtension(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
  deviceInfo.addInstanceExtension(VK_KHR_EXTERNAL_MEMORY_CAPABILITIES_EXTENSION_NAME);
  deviceInfo.addInstanceExtension(VK_KHR_EXTERNAL_SEMAPHORE_CAPABILITIES_EXTENSION_NAME);
  deviceInfo.addInstanceExtension(VK_KHR_EXTERNAL_FENCE_CAPABILITIES_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_MAINTENANCE1_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_MEMORY_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_SEMAPHORE_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_FENCE_EXTENSION_NAME);
#ifdef WIN32
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_MEMORY_WIN32_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_SEMAPHORE_WIN32_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_FENCE_WIN32_EXTENSION_NAME);
#else
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_SEMAPHORE_FD_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_EXTERNAL_FENCE_FD_EXTENSION_NAME);
#endif
  deviceInfo.addDeviceExtension(VK_NV_RAY_TRACING_EXTENSION_NAME);
  deviceInfo.addDeviceExtension(VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);

  // Creating the Vulkan instance and device
  nvvk::Context vkctx;
  vkctx.init(deviceInfo);


  // Loading the C version of the extensions
  load_VK_EXTENSION_SUBSET(vkctx.m_instance, vkGetInstanceProcAddr, vkctx.m_device, vkGetDeviceProcAddr);


  VkGlExample         example;
  nvgl::ContextWindow contextWindowGL;

  //// Creating the window
  //example.open(0, 0, SAMPLE_SIZE_WIDTH, SAMPLE_SIZE_HEIGHT, PROJECT_NAME);

  //// OpenGL context within the Window
  //contextWindowGL.init(&context, example.m_internal, PROJECT_NAME);
  //contextWindowGL.makeContextCurrent();
  //contextWindowGL.swapInterval(0);

  // Loading all OpenGL symbols
  load_GL(nvgl::ContextWindow::sysGetProcAddress);

  // Setup the Vulkan base elements
  example.setup(vkctx.m_instance, vkctx.m_device, vkctx.m_physicalDevice, vkctx.m_queueGCT.familyIndex);

  // Printing which GPU we are using for Vulkan
  std::cout << "Using " << example.getPhysicalDevice().getProperties().deviceName << std::endl;


  // Initialize the window, UI ..
  example.initUI(SAMPLE_SIZE_WIDTH, SAMPLE_SIZE_HEIGHT);

  // Various ways of creating vertex buffers
  example.initExample();

  // GLFW Callback
  example.setupGlfwCallbacks(window);
  ImGui_ImplGlfw_InitForOpenGL(window, true);

  // Main loop
  while(!glfwWindowShouldClose(window))
  {
    glfwPollEvents();
    if(example.isMinimized())
      continue;

    CameraManip.updateAnim();
    glClearColor(0.5f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    auto& imgui_io       = ImGui::GetIO();
    imgui_io.DisplaySize = ImVec2(SAMPLE_SIZE_WIDTH, SAMPLE_SIZE_HEIGHT);

    example.onWindowRefresh();
    if(example.showGui())
      example.drawUI();

    glfwSwapBuffers(window);
  }

  example.destroy();
  vkctx.deinit();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
