cmake_minimum_required(VERSION 3.5)
project(engineer_arm_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(MUJOCO_ROOT_DIR "$ENV{HOME}/.mujoco/mujoco210")
option(USE_GL "Use Mujoco with OpenGL" ON)
set(MUJOCO_BIN_DIR "${MUJOCO_ROOT_DIR}/bin")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")
if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco.h")
  set(MUJOCO_ROOT_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}")
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}/mujoco")
endif()
set(MUJOCO_LIB_DIR "${MUJOCO_ROOT_DIR}/lib")
set(MUJOCO_SAMPLE_DIR "${MUJOCO_ROOT_DIR}/sample")
set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT_DIR}/simulate")
message("${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo root dir: " ${MUJOCO_ROOT_DIR})

set(OpenGL_GL_PREFERENCE "GLVND")

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(engineer_msg REQUIRED)

file(GLOB MUJOCO_LIBS "${MUJOCO_BIN_DIR}/*.so")
file(GLOB MUJOCO_LIBS_GL "${MUJOCO_BIN_DIR}/*.so.3")

include_directories(include)
include_directories(${MUJOCO_INCLUDE_DIR})
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(sim 
                  src/drv_math.cpp
                  src/alg_pid.cpp
                  src/ROS2_controller_thread.cpp 
                  src/sim.cpp)
ament_target_dependencies(sim rclcpp ament_index_cpp engineer_msg)
# add_dependencies(sim ROS2_controller_thread alg_pid drv_math)
target_link_libraries(sim 
                        ${MUJOCO_LIBS} 
                        ${MUJOCO_LIBS_GL} 
                        ${OPENGL_LIBRARIES} 
                        ${GLEW_LIBRARIES} 
                        # ROS2_controller_thread
                        # alg_pid
                        # drv_math
                        )

install(DIRECTORY
  mjcf
  DESTINATION share/${PROJECT_NAME}/
)

# install(TARGETS
#   ROS2_controller_thread
#   alg_pid
#   drv_math
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS 
  sim
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()