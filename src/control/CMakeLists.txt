cmake_minimum_required(VERSION 2.8.3)
project(control)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
message_generation
gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_message_files(
  FILES
  FourWheeler.msg
)

# For differential drive plugin
add_library(four_wheel_differential_plugin plugins/src/four_wheel_differential.cc)
target_link_libraries(four_wheel_differential_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

 generate_messages(
   DEPENDENCIES
   std_msgs
   control
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
