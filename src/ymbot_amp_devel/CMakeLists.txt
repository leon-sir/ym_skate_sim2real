cmake_minimum_required(VERSION 3.0.2)
project(ymbot_amp_devel)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
add_compile_options(-Wno-deprecated-declarations)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  geometry_msgs
  tf
)

find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(glfw3 REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 检查系统架构类型（x86_64, ARM）,  根据不同的系统架构类型选择不同的库文件
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(EU_PLANET_LIB_PATH "${PROJECT_SOURCE_DIR}/lib_x86_64")
  message(STATUS "Detected x86_64 architecture. Using library path: ${EU_PLANET_LIB_PATH}")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(EU_PLANET_LIB_PATH "${PROJECT_SOURCE_DIR}/lib_ARM")
  message(STATUS "Detected ARM architecture (aarch64). Using library path: ${EU_PLANET_LIB_PATH}")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
# set(EU_PLANET_LIB_PATH "${PROJECT_SOURCE_DIR}/lib_x86_64")
link_directories(${EU_PLANET_LIB_PATH})
find_library(EU_PLANET_LIB eu_planet PATHS ${EU_PLANET_LIB_PATH})
if(EU_PLANET_LIB)
  message(STATUS "Found eu_planet library: ${EU_PLANET_LIB}")
else()
  message(FATAL_ERROR "Could not find eu_planet library")
endif()

# install(TARGETS eu_canable controlcan
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ymbot_amp_devel joint_eu_api ${EU_PLANET_LIB}
 CATKIN_DEPENDS roscpp rospy roslib std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})
link_directories(${GLFW_LIBRARY_DIRS})


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} 
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/onnxruntime-linux-x64-1.21.0/include
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/mujoco-3.2.7/include
  app
  app/thirdparty/soem/soem
  app/thirdparty/soem/osal
  app/thirdparty/soem/osal/linux
  app/thirdparty/soem/oshw/linux
  app/thirdparty/cpp-readline/src
  /usr/include/eigen3
)



# 添加 ONNX Runtime 的库路径
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/onnxruntime-linux-x64-1.21.0/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/amp_controller/lib
)
# Create the library
add_library(amp_controller
  src/amp_controller.cpp
)

target_link_libraries(amp_controller
  ${catkin_LIBRARIES}
  onnxruntime
)
# --------------------------------------------------------amp
# executable
add_executable(amp_sim2sim src/amp_sim2sim.cpp )
target_link_libraries(amp_sim2sim
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/mujoco-3.2.7/lib/libmujoco.so
  glfw  
  amp_controller
)

add_executable(amp_sim2real src/amp_sim2real.cpp)
add_subdirectory(app/thirdparty)
set(SOURCES app/motor_control.c
            app/motor_control.h
            app/math_ops.c
            app/math_ops.h
            app/transmit.cpp
            app/transmit.h
            app/config.h
            app/queue.h 
            app/command.cpp 
            app/command.h
            )
add_library(motor ${SOURCES})
target_link_libraries(motor soem)
target_link_libraries(amp_sim2real motor soem cpp-readline
  ${catkin_LIBRARIES} -pthread
  glfw  
  amp_controller
)

# 添加新的11DOF版本可执行文件
add_executable(amp_sim2real_11dof src/amp_sim2real_11dof.cpp)
target_link_libraries(amp_sim2real_11dof motor soem cpp-readline
  ${catkin_LIBRARIES} -pthread
  glfw  
  amp_controller
)

##--------------------------------------------------------eu_motor
# 添加动态库
add_executable(eumotor_interface_amp 
  src/ymboy_eumotor_interface.cpp
  src/ymbot_joint_eu_api.cpp
)

# 链接库依赖
target_link_libraries(eumotor_interface_amp
  ${catkin_LIBRARIES}
  yaml-cpp
  ${EU_PLANET_LIB}
  rt
  -pthread 
)
##---------------------------------------------------------encos_motor
# 添加 MotorInterface 可执行文件
add_executable(encos_motor_interface src/MotorInterface.cpp 
                            src/parallejoint.cpp)
# 链接所需的库
target_link_libraries(encos_motor_interface
    ${catkin_LIBRARIES}  # ROS 相关库
    -pthread             # 线程库
    soem                # EtherCAT 库
    cpp-readline        # 命令行库
    yaml-cpp            # YAML 解析库
    motor
)

