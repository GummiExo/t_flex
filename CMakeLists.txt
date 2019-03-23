cmake_minimum_required(VERSION 2.8.3)
project(t_flex)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamixel_workbench_controllers
  dynamixel_workbench_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   GaitEvent.msg
   GaitPhase.msg
   MotorsInfo.msg
   Insole.msg
   IMUData.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# A collection of utilities for the gait phase classifier
add_library(feature_extractor
  src/util/feature_extractor.cpp
)
target_link_libraries(feature_extractor
  ${catkin_LIBRARIES}
)

add_library(gait_cycle_classifier
  src/util/gait_cycle_classifier.cpp
)
target_link_libraries(gait_cycle_classifier
  ${catkin_LIBRARIES}
  feature_extractor
)

# Build main ROS node for threshold-based gait phase detection
add_executable(gait_phases_threshold_node src/nodes/gait_phases_threshold_node.cpp)
target_link_libraries(gait_phases_threshold_node
  ${catkin_LIBRARIES}
  feature_extractor
  gait_cycle_classifier
)

add_executable(threshold_gait_detection_node src/nodes/threshold_gait_detection_node.cpp)
target_link_libraries(threshold_gait_detection_node
  ${catkin_LIBRARIES}
  feature_extractor
  gait_cycle_classifier
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation

install(TARGETS feature_extractor gait_cycle_classifier gait_phases_threshold_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

