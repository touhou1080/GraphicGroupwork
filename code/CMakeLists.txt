cmake_minimum_required(VERSION 3.20)
project(AngryBird2D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(GL_SILENCE_DEPRECATION)

include(FetchContent)

FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
)
FetchContent_MakeAvailable(glfw)

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

if(WIN32)
  target_link_libraries(angry_bird_sim PRIVATE opengl32)
else()
  target_link_libraries(angry_bird_sim PRIVATE glfw OpenGL::GL)
endif()