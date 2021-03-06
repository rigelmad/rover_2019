cmake_minimum_required(VERSION 2.8.3)
project(watchdog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  rover_diagnostics
  power_sensor
  drive_control
  arm_joy_control
)

## Generate messages in the 'msg' folder
## add_message_files(
##   FILES
## )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  drive_control
  arm_joy_control
  power_sensor
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(watchdog src/watchdog_main.cpp)
target_link_libraries(watchdog ${catkin_LIBRARIES})
add_dependencies(watchdog ${catkin_EXPORTED_TARGETS})
