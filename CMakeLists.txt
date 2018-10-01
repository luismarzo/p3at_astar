cmake_minimum_required(VERSION 2.8.3)
project(pioneer3at)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)



catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(astar_gazebo "src/astar_gazebo.cpp")
target_link_libraries(astar_gazebo ${catkin_LIBRARIES})
include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)
add_dependencies(astar_gazebo pioneer3at_generate_messages_cpp)

add_executable(keyboard_control "src/keyboard_control.cpp")
target_link_libraries(keyboard_control ${catkin_LIBRARIES})
include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)


