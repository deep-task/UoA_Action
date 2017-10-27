cmake_minimum_required(VERSION 2.8.3)
project(action_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES deeptask_project
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(action_generator src/action_generator.cpp)
target_link_libraries(action_generator ${catkin_LIBRARIES})
