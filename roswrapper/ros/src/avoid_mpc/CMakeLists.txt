cmake_minimum_required(VERSION 2.8)
project(avoid_mpc)

set(CMAKE_BUILD_TYPE "Release")
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
add_definitions(-DCOMPILEDWITHC11)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
include(cmake/FindCasadi.cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  quadrotor_msgs
  visualization_msgs
)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
include_directories(
include
${PROJECT_SOURCE_DIR}
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Casadi_INCLUDE_DIR}
)
catkin_package()

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Casadi_LIBRARY}
)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(mpc_obstacle_avoidance_node 
                src/mpc_obstacle_avoidance_node.cpp 
                src/HighLvlMpc.cpp 
                src/AvoidanceStateMachine.cpp 
                src/FrameKDMap.cpp
                src/ParameterManager.cpp
                src/COGFilter.cpp)
target_link_libraries(mpc_obstacle_avoidance_node
  ${catkin_LIBRARIES}
  ${LIBS}
  ${PCL_LIBRARIES}
)

