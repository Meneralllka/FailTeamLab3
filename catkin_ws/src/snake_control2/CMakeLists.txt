cmake_minimum_required(VERSION 3.0.2)
project(snake_control2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(snake_control2 src/snake_control.cpp)
add_executable(snake_test src/test.cpp)

target_link_libraries(snake_control2
  ${catkin_LIBRARIES}
)

target_link_libraries(snake_test
  ${catkin_LIBRARIES}
)