#include "renderer.h"
#include <algorithm>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#endif
#include <SDL.h>
#include <SDL_opengl.h>
#include <SDL_opengl_glext.h>

// ── GL 3.3 function pointers (loaded via SDL) ──────────────────────

static PFNGLGENVERTEXARRAYSPROC vs_GenVertexArrays;
static PFNGLBINDVERTEXARRAYPROC vs_BindVertexArray;
static PFNGLCREATESHADERPROC vs_CreateShader;
static PFNGLSHADERSOURCEPROC vs_ShaderSource;
static PFNGLCOMPILESHADERPROC vs_CompileShader;
static PFNGLCREATEPROGRAMPROC vs_CreateProgram;
static PFNGLATTACHSHADERPROC vs_AttachShader;
static PFNGLLINKPROGRAMPROC vs_LinkProgram;
static PFNGLUSEPROGRAMPROC vs_UseProgram;
static PFNGLDELETESHADERPROC vs_DeleteShader;
static PFNGLGENBUFFERSPROC vs_GenBuffers;
static PFNGLBINDBUFFERPROC vs_BindBuffer;
static PFNGLBUFFERDATAPROC vs_BufferData;
static PFNGLVERTEXATTRIBPOINTERPROC vs_VertexAttribPointer;
static PFNGLENABLEVERTEXATTRIBARRAYPROC vs_EnableVertexAttribArray;
static PFNGLGETUNIFORMLOCATIONPROC vs_GetUniformLocation;
static PFNGLUNIFORM1IPROC vs_Uniform1i;
static PFNGLACTIVETEXTUREPROC vs_ActiveTexture;
static PFNGLDELETEVERTEXARRAYSPROC vs_DeleteVertexArrays;
static PFNGLDELETEBUFFERSPROC vs_DeleteBuffers;
static PFNGLDELETEPROGRAMPROC vs_DeleteProgram;

static bool load_gl_functions() {
#define LOAD(var, type, name)                                                  \
  var = (type)SDL_GL_GetProcAddress(name);                                     \
  if (!var)                                                                    \
    return false;
  LOAD(vs_GenVertexArrays, PFNGLGENVERTEXARRAYSPROC, "glGenVertexArrays")
  LOAD(vs_BindVertexArray, PFNGLBINDVERTEXARRAYPROC, "glBindVertexArray")
  LOAD(vs_CreateShader, PFNGLCREATESHADERPROC, "glCreateShader")
  LOAD(vs_ShaderSource, PFNGLSHADERSOURCEPROC, "glShaderSource")
  LOAD(vs_CompileShader, PFNGLCOMPILESHADERPROC, "glCompileShader")
  LOAD(vs_CreateProgram, PFNGLCREATEPROGRAMPROC, "glCreateProgram")
  LOAD(vs_AttachShader, PFNGLATTACHSHADERPROC, "glAttachShader")
  LOAD(vs_LinkProgram, PFNGLLINKPROGRAMPROC, "glLinkProgram")
  LOAD(vs_UseProgram, PFNGLUSEPROGRAMPROC, "glUseProgram")
  LOAD(vs_DeleteShader, PFNGLDELETESHADERPROC, "glDeleteShader")
  LOAD(vs_GenBuffers, PFNGLGENBUFFERSPROC, "glGenBuffers")
  LOAD(vs_BindBuffer, PFNGLBINDBUFFERPROC, "glBindBuffer")
  LOAD(vs_BufferData, PFNGLBUFFERDATAPROC, "glBufferData")
  LOAD(vs_VertexAttribPointer, PFNGLVERTEXATTRIBPOINTERPROC,
       "glVertexAttribPointer")
  LOAD(vs_EnableVertexAttribArray, PFNGLENABLEVERTEXATTRIBARRAYPROC,
       "glEnableVertexAttribArray")
  LOAD(vs_GetUniformLocation, PFNGLGETUNIFORMLOCATIONPROC,
       "glGetUniformLocation")
  LOAD(vs_Uniform1i, PFNGLUNIFORM1IPROC, "glUniform1i")
  LOAD(vs_ActiveTexture, PFNGLACTIVETEXTUREPROC, "glActiveTexture")
  LOAD(vs_DeleteVertexArrays, PFNGLDELETEVERTEXARRAYSPROC,
       "glDeleteVertexArrays")
  LOAD(vs_DeleteBuffers, PFNGLDELETEBUFFERSPROC, "glDeleteBuffers")
  LOAD(vs_DeleteProgram, PFNGLDELETEPROGRAMPROC, "glDeleteProgram")
#undef LOAD
  return true;
}

bool Renderer::init(SDL_Window *window) {
  window_ = window;

  if (!load_gl_functions()) {
    LOG_ERROR("Renderer: Failed to load OpenGL functions");
    return false;
  }

  if (!create_texture())
    return false;
  if (!create_shader())
    return false;

  // Fullscreen quad
  float vertices[] = {
      // pos      // uv
      -1.0f, -1.0f, 0.0f, 1.0f, 1.0f, -1.0f, 1.0f, 1.0f,
      -1.0f, 1.0f,  0.0f, 0.0f, 1.0f, 1.0f,  1.0f, 0.0f,
  };

  vs_GenVertexArrays(1, &vao_);
  vs_BindVertexArray(vao_);

  vs_GenBuffers(1, &vbo_);
  vs_BindBuffer(GL_ARRAY_BUFFER, vbo_);
  vs_BufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  vs_VertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                         (void *)0);
  vs_EnableVertexAttribArray(0);
  vs_VertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                         (void *)(2 * sizeof(float)));
  vs_EnableVertexAttribArray(1);

  LOG_INFO("Renderer: Initialized OpenGL %s", glGetString(GL_VERSION));
  return true;
}

void Renderer::shutdown() {
  if (texture_id_)
    glDeleteTextures(1, &texture_id_);
  if (vbo_)
    vs_DeleteBuffers(1, &vbo_);
  if (vao_)
    vs_DeleteVertexArrays(1, &vao_);
  if (shader_)
    vs_DeleteProgram(shader_);
}

bool Renderer::create_texture() {
  glGenTextures(1, &texture_id_);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, psx::VRAM_WIDTH, psx::VRAM_HEIGHT, 0,
               GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  return true;
}

bool Renderer::create_shader() {
  const char *vert_src = R"(
        #version 330 core
        layout(location = 0) in vec2 aPos;
        layout(location = 1) in vec2 aUV;
        out vec2 vUV;
        void main() {
            gl_Position = vec4(aPos, 0.0, 1.0);
            vUV = aUV;
        }
    )";

  const char *frag_src = R"(
        #version 330 core
        in vec2 vUV;
        out vec4 FragColor;
        uniform sampler2D uTexture;
        void main() {
            FragColor = texture(uTexture, vUV);
        }
    )";

  GLuint vert = vs_CreateShader(GL_VERTEX_SHADER);
  vs_ShaderSource(vert, 1, &vert_src, nullptr);
  vs_CompileShader(vert);

  GLuint frag = vs_CreateShader(GL_FRAGMENT_SHADER);
  vs_ShaderSource(frag, 1, &frag_src, nullptr);
  vs_CompileShader(frag);

  shader_ = vs_CreateProgram();
  vs_AttachShader(shader_, vert);
  vs_AttachShader(shader_, frag);
  vs_LinkProgram(shader_);

  vs_DeleteShader(vert);
  vs_DeleteShader(frag);

  vs_UseProgram(shader_);
  GLint loc = vs_GetUniformLocation(shader_, "uTexture");
  vs_Uniform1i(loc, 0);

  return true;
}

void Renderer::upload_frame(const std::vector<u32> &rgba, int width,
                            int height) {
  const int w = (std::max)(1, width);
  const int h = (std::max)(1, height);
  const size_t expected =
      static_cast<size_t>(w) * static_cast<size_t>(h);
  if (rgba.size() < expected) {
    return;
  }

  last_frame_width_ = w;
  last_frame_height_ = h;

  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE,
               rgba.data());
}
