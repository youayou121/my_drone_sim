cmake_minimum_required(VERSION 2.8.3)
project(path_searching_fast)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env_fast
  cv_bridge
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching_fast
 CATKIN_DEPENDS plan_env_fast
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( path_searching_fast 
    src/kinodynamic_astar.cpp 
    src/astar.cpp
    src/topo_prm.cpp
    )
target_link_libraries( path_searching_fast
    ${catkin_LIBRARIES} 
    )  
