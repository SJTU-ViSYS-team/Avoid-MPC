cmake_minimum_required(VERSION 2.8.3)
project(betaflight_ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  uav_utils
  mavros
  quadrotor_msgs
  tf
)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(bfctrl_node 
  src/bfctrl_node.cpp
  src/BfCtrlFSM.cpp
  src/BfCtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

add_dependencies(bfctrl_node quadrotor_msgs)

target_link_libraries(bfctrl_node
  ${catkin_LIBRARIES}
)