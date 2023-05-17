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
#ifndef SHADER_H
#define SHADER_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <array>

#include "nvmath/nvmath.h"

class OglShader
{
public:
  OglShader()     = default;
  unsigned int ID = 0;
  // constructor generates the shader on the fly
  // ------------------------------------------------------------------------
  OglShader(const std::string& vertexPath, const std::string& fragmentPath, const std::string& geometryPath = "")
  {
    // 1. retrieve the vertex/fragment source code from filePath
    std::string   vertexCode;
    std::string   fragmentCode;
    std::string   geometryCode;
    std::ifstream vShaderFile;
    std::ifstream fShaderFile;
    std::ifstream gShaderFile;

    try
    {
      // open files
      vShaderFile.open(vertexPath);
      fShaderFile.open(fragmentPath);
      std::stringstream vShaderStream, fShaderStream;
      // read file's buffer contents into streams
      vShaderStream << vShaderFile.rdbuf();
      fShaderStream << fShaderFile.rdbuf();
      // close file handlers
      vShaderFile.close();
      fShaderFile.close();
      // convert stream into string
      vertexCode   = vShaderStream.str();
      fragmentCode = fShaderStream.str();
      // if geometry shader path is present, also load a geometry shader
      if(!geometryPath.empty())
      {
        gShaderFile.open(geometryPath);
        std::stringstream gShaderStream;
        gShaderStream << gShaderFile.rdbuf();
        gShaderFile.close();
        geometryCode = gShaderStream.str();
      }
    }
    catch(std::ifstream::failure& e)
    {
      std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ " << e.what() << std::endl;
    }
    const char* vShaderCode = vertexCode.c_str();
    const char* fShaderCode = fragmentCode.c_str();
    // 2. compile shaders
    GLuint vertex{0}, fragment{0};
    // vertex shader
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderCode, nullptr);
    glCompileShader(vertex);
    checkCompileErrors(vertex, "VERTEX");
    // fragment Shader
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderCode, nullptr);
    glCompileShader(fragment);
    checkCompileErrors(fragment, "FRAGMENT");
    // if geometry shader is given, compile geometry shader
    GLuint geometry{0};
    if(!geometryPath.empty())
    {
      const char* gShaderCode = geometryCode.c_str();
      geometry                = glCreateShader(GL_GEOMETRY_SHADER);
      glShaderSource(geometry, 1, &gShaderCode, nullptr);
      glCompileShader(geometry);
      checkCompileErrors(geometry, "GEOMETRY");
    }
    // shader Program
    ID = glCreateProgram();
    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);
    if(!geometryPath.empty())
    {
      glAttachShader(ID, geometry);
    }
    glLinkProgram(ID);
    checkCompileErrors(ID, "PROGRAM");
    // delete the shaders as they're linked into our program now and no longer necessary
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    if(!geometryPath.empty())
    {
      glDeleteShader(geometry);
    }
  }
  // activate the shader
  // ------------------------------------------------------------------------
  void use() const { glUseProgram(ID); }
  // utility uniform functions
  // ------------------------------------------------------------------------
  void setBool(const std::string& name, bool value) const
  {
    glUniform1i(glGetUniformLocation(ID, name.c_str()), static_cast<int>(value));
  }
  // ------------------------------------------------------------------------
  void setInt(const std::string& name, int value) const { glUniform1i(glGetUniformLocation(ID, name.c_str()), value); }
  // ------------------------------------------------------------------------
  void setFloat(const std::string& name, float value) const
  {
    glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
  }
  // ------------------------------------------------------------------------
  void setVec2(const std::string& name, const nvmath::vec2f& value) const
  {
    glUniform2fv(glGetUniformLocation(ID, name.c_str()), 1, &value.x);
  }
  void setVec2(const std::string& name, float x, float y) const
  {
    glUniform2f(glGetUniformLocation(ID, name.c_str()), x, y);
  }
  // ------------------------------------------------------------------------
  void setVec3(const std::string& name, const nvmath::vec3f& value) const
  {
    glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, &value.x);
  }
  void setVec3(const std::string& name, float x, float y, float z) const
  {
    glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
  }
  // ------------------------------------------------------------------------
  void setVec4(const std::string& name, const nvmath::vec4f& value) const
  {
    glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, &value.x);
  }
  void setVec4(const std::string& name, float x, float y, float z, float w)
  {
    glUniform4f(glGetUniformLocation(ID, name.c_str()), x, y, z, w);
  }
  // ------------------------------------------------------------------------
  //void setMat2(const std::string& name, const nvmath::mat2f& mat) const
  //{
  //  glUniformMatrix2fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
  //}
  // ------------------------------------------------------------------------
  void setMat3(const std::string& name, const nvmath::mat3f& mat) const
  {
    glUniformMatrix3fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat.a00);
  }
  // ------------------------------------------------------------------------
  void setMat4(const std::string& name, const nvmath::mat4f& mat) const
  {
    glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat.a00);
  }

private:
  // utility function for checking shader compilation/linking errors.
  // ------------------------------------------------------------------------
  static void checkCompileErrors(GLuint shader, const std::string& type)
  {
    GLint success{GL_FALSE};

    std::array<GLchar, 1024> infoLog{};
    if(type != "PROGRAM")
    {
      glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
      if(success == GL_FALSE)
      {
        glGetShaderInfoLog(shader, static_cast<GLsizei>(infoLog.size()), nullptr, infoLog.data());
        std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
                  << infoLog.data() << "\n -- --------------------------------------------------- -- " << std::endl;
      }
    }
    else
    {
      glGetProgramiv(shader, GL_LINK_STATUS, &success);
      if(success == GL_FALSE)
      {
        glGetProgramInfoLog(shader, static_cast<GLsizei>(infoLog.size()), nullptr, infoLog.data());
        std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                  << infoLog.data() << "\n -- --------------------------------------------------- -- " << std::endl;
      }
    }
  }
};


inline void /*GLAPIENTRY */ GLDebugCallback(GLenum source,
                                            GLenum type,
                                            GLuint id,
                                            GLenum severity,
                                            GLsizei /*length*/,
                                            const GLchar* message,
                                            const void* /*userParam*/)
{
  std::cout << "[OpenGL]: ";
  std::cout << "Source: ";
  switch(source)
  {
    case GL_DEBUG_SOURCE_API:
      std::cout << "API";
      break;
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
      std::cout << "Window System";
      break;
    case GL_DEBUG_SOURCE_SHADER_COMPILER:
      std::cout << "Shader Compiler";
      break;
    case GL_DEBUG_SOURCE_THIRD_PARTY:
      std::cout << "Third Party";
      break;
    case GL_DEBUG_SOURCE_APPLICATION:
      std::cout << "Application";
      break;
    case GL_DEBUG_SOURCE_OTHER:
      std::cout << "Other";
      break;
  }

  std::cout << ", type: ";
  switch(type)
  {
    case GL_DEBUG_TYPE_ERROR:
      std::cout << "ERROR";
      break;
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
      std::cout << "DEPRECATED_BEHAVIOR";
      break;
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
      std::cout << "UNDEFINED_BEHAVIOR";
      break;
    case GL_DEBUG_TYPE_PORTABILITY:
      std::cout << "PORTABILITY";
      break;
    case GL_DEBUG_TYPE_PERFORMANCE:
      std::cout << "PERFORMANCE";
      break;
    case GL_DEBUG_TYPE_OTHER:
      std::cout << "OTHER";
      break;
    default:
      std::cout << "OTHER";
  }

  std::cout << ", id: " << id;
  std::cout << ", severity: ";
  switch(severity)
  {
    case GL_DEBUG_SEVERITY_LOW:
      std::cout << "LOW";
      break;
    case GL_DEBUG_SEVERITY_MEDIUM:
      std::cout << "MEDIUM";
      break;
    case GL_DEBUG_SEVERITY_HIGH:
      std::cout << "HIGH";
      break;
    default:
      std::cout << "OTHER";
  }

  std::cout << std::endl;
  std::cout << "\"" << message << "\"" << std::endl << std::endl;
}

inline void CheckGLError()
{
  // check OpenGL error
  GLenum err{GL_NO_ERROR};
  while((err = glGetError()) != GL_NO_ERROR)
  {
    std::cerr << "OpenGL error: " << err << std::endl << std::endl;
  }
}

#endif
