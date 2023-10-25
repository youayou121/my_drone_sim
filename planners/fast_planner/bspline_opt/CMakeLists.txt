cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt_fast)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  nlopt
  plan_env_fast
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline_opt_fast
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

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( bspline_opt_fast 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( bspline_opt_fast
    ${catkin_LIBRARIES} 
    )  
