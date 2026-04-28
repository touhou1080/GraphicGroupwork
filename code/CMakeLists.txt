cmake_minimum_required(VERSION 3.20)
project(AngryBird2D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 【新增】全局屏蔽 macOS 下的 OpenGL 废弃警告，还你一个干净的编译输出
add_compile_definitions(GL_SILENCE_DEPRECATION)

include(FetchContent)

FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
)
FetchContent_MakeAvailable(glfw)

# 【新增】让 CMake 在系统中寻找 OpenGL 库
find_package(OpenGL REQUIRED)

add_executable(
  angry_bird_sim
  main.cpp
  physics.cpp
  collision.cpp
  scene.cpp
  input.cpp
  render.cpp
)


# 因为使用了现代 CMake 的 OpenGL::GL，它会自动处理各平台的链接，
if(WIN32)
  target_link_libraries(angry_bird_sim PRIVATE opengl32)
else()
  target_link_libraries(angry_bird_sim PRIVATE glfw OpenGL::GL)
endif()