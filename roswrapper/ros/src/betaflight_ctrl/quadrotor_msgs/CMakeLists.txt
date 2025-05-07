cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  message_generation
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )


add_message_files(
  FILES
  TakeoffLand.msg
  Command.msg
  BfctrlStatue.msg
  SlowDown.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  #CATKIN_DEPENDS geometry_msgs nav_msgs
  #DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


