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


add_executable(path_planning "src/path_planning.cpp")
target_link_libraries(path_planning ${catkin_LIBRARIES})
include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)
add_dependencies(path_planning pioneer3at_generate_messages_cpp)

add_executable(keyboard_control "src/keyboard_control.cpp")
target_link_libraries(keyboard_control ${catkin_LIBRARIES})
include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)


