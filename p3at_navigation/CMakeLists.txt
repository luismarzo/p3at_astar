cmake_minimum_required(VERSION 2.8.3)
project(p3at_navigation)



find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  rviz
  xacro
  tf
  
)

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS generate_msgs roscpp rospy
   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(sub_and_pub src/sub_and_pub.cpp)
target_link_libraries(sub_and_pub ${catkin_LIBRARIES})

