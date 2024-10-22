cmake_minimum_required(VERSION 3.0.2)
project(snake_trajectory_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_joint_1 src/move_joint_1.cpp)

target_link_libraries(move_joint_1
  ${catkin_LIBRARIES}
)
