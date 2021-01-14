cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_tasks)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot3_tasks
  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wall_follower_node src/tb3_follow_wall.cpp)
target_link_libraries(wall_follower_node ${catkin_LIBRARIES})
