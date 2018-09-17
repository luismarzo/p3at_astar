cmake_minimum_required(VERSION 2.8.3)
project(p3at_astar)

find_package(catkin REQUIRED COMPONENTS
  amcl
  base_local_planner
  move_base
  navfn
  roslaunch
  tf
  urdf
  xacro
)
catkin_package(
 INCLUDE_DIRS include
 # LIBRARIES radom_walk
 CATKIN_DEPENDS amcl base_local_planner base_local_planner move_base navfn roslaunch tf urdf xacro
 DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC_FILES "src/*.cpp" "include/*.h" "include/*.inl" )


## Declare a C++ library
# add_library(random_walk src/random_walk.cpp)
# target_link_libraries(random_walk ${catkin_LIBRARIES})

# Declare a C++ executable
add_executable(pose_tf src/pose_tf.cpp)
target_link_libraries(pose_tf ${catkin_LIBRARIES})

add_executable(astar_gazebo "src/astar_gazebo.cpp")
target_link_libraries(astar_gazebo ${catkin_LIBRARIES})
include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)
add_dependencies(astar_gazebo p3at_tutorial_generate_messages_cpp)
