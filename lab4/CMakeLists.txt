cmake_minimum_required(VERSION 3.0.2)
project(lab4)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  std_srvs
  tf
)


find_package(Boost REQUIRED COMPONENTS system thread)

find_package(stage REQUIRED)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PlotPoint.msg
  PlotPose.msg
  MapPoints.msg
  SimPose.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)


###################################
## catkin specific configuration ##
###################################


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES lab4
CATKIN_DEPENDS message_runtime
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${STAGE_INCLUDE_DIRS}
)


# Declare a cpp executable
add_executable(stageros_lab4 src/stageros_lab4.cpp)
set(${PROJECT_NAME}_extra_libs "")
if(UNIX AND NOT APPLE)
  set(${PROJECT_NAME}_extra_libs dl)
endif()
target_link_libraries(stageros_lab4
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${STAGE_LIBRARIES}
  ${${PROJECT_NAME}_extra_libs}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(stageros_lab4 ${catkin_EXPORTED_TARGETS})
endif()


