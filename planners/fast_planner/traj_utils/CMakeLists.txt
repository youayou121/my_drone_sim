cmake_minimum_required(VERSION 2.8.3)
project(traj_utils_fast)

find_package(catkin REQUIRED COMPONENTS
  bspline
  bspline_opt_fast
  path_searching_fast
  poly_traj
  roscpp
  std_msgs
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_utils_fast
 CATKIN_DEPENDS path_searching_fast bspline bspline_opt_fast poly_traj
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( traj_utils_fast
    src/planning_visualization.cpp 
    )
target_link_libraries( traj_utils_fast
    ${catkin_LIBRARIES} 
    )  