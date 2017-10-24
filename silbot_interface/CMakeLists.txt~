cmake_minimum_required(VERSION 2.8.3)
project(silbot_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES deeptask_project
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(silbot_interface src/silbot_interface.cpp)
target_link_libraries(silbot_interface ${catkin_LIBRARIES})

