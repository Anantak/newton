cmake_minimum_required(VERSION 3.16)
project(my_mujoco_project)

set(CMAKE_CXX_STANDARD 17)

# Specify MuJoCo paths
set(MUJOCO_ROOT_DIR "/home/ubuntu/Libraries/NEWTON/newton/.venv/lib/python3.11/site-packages/mujoco")  # Update this path

# Direct paths to MuJoCo (update these)
set(MUJOCO_LIBRARY_PATH "${MUJOCO_ROOT_DIR}/libmujoco.so.3.3.3")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")

find_package(Threads REQUIRED)

include_directories(
  ${MUJOCO_INCLUDE_DIR}
)

# mujoco sample : basic
add_executable(basic basic.cc)
target_link_libraries(basic
  ${MUJOCO_LIBRARY_PATH} #mujoco::mujoco
  glfw
  Threads::Threads
)

# mujoco sample : compile
add_executable(compile compile.cc)
target_link_libraries(compile 
  ${MUJOCO_LIBRARY_PATH} #mujoco::mujoco
  Threads::Threads
)

# mujoco sample : testspeed
add_executable(testspeed testspeed.cc)
target_link_libraries(testspeed 
  ${MUJOCO_LIBRARY_PATH} #mujoco::mujoco
  Threads::Threads
)

# mujoco sample : testspeed
add_executable(record record.cc)
target_link_libraries(record 
  ${MUJOCO_LIBRARY_PATH} #mujoco::mujoco
  glfw
  Threads::Threads
)
