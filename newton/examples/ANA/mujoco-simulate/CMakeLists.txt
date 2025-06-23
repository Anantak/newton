cmake_minimum_required(VERSION 3.16)
project(mujoco_simulate_project)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)

# Specify MuJoCo paths
set(MUJOCO_ROOT_DIR "/home/ubuntu/Libraries/NEWTON/newton/.venv/lib/python3.11/site-packages/mujoco")  # Update this path

# Direct paths to MuJoCo (update these)
set(MUJOCO_LIBRARY_PATH "${MUJOCO_ROOT_DIR}/libmujoco.so.3.3.3")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")

find_package(Threads REQUIRED)

# Fetch lodepng dependency.
if(NOT TARGET lodepng)
  FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )

  FetchContent_GetProperties(lodepng)
  if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
  endif()
endif()

# Simulate library
add_library(platform_ui_adapter OBJECT)
target_sources(
  platform_ui_adapter
  PUBLIC glfw_adapter.h glfw_dispatch.h platform_ui_adapter.h
  PRIVATE glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc
)
# target_compile_options(platform_ui_adapter PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
# target_include_directories(
#   platform_ui_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
#                              $<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>
# )
target_link_libraries(platform_ui_adapter PUBLIC ${MUJOCO_LIBRARY_PATH})
if(SIMULATE_GLFW_DYNAMIC_SYMBOLS)
  target_compile_definitions(platform_ui_adapter PUBLIC mjGLFW_DYNAMIC_SYMBOLS)
endif()
add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

add_library(libsimulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
set_target_properties(libsimulate PROPERTIES OUTPUT_NAME simulate)
add_library(mujoco::libsimulate ALIAS libsimulate)

target_sources(
  libsimulate
  PUBLIC simulate.h
  PRIVATE simulate.cc array_safety.h
)
target_include_directories(libsimulate PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_link_libraries(libsimulate PUBLIC lodepng mujoco::platform_ui_adapter ${MUJOCO_LIBRARY_PATH})
target_link_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})


include_directories(
  ${MUJOCO_INCLUDE_DIR}
)

set(SIMULATE_RESOURCE_FILES "")

# mujoco : simulate
add_executable(simulate main.cc array_safety.h ${SIMULATE_RESOURCE_FILES})
target_link_libraries(simulate
  libsimulate
  ${MUJOCO_LIBRARY_PATH} #mujoco::mujoco
  glfw
  Threads::Threads
  lodepng
)
