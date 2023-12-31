cmake_minimum_required(VERSION 2.8.3)
project(rrt_star_planner)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# add your code here: find_package(xxx REQUIRED)
find_package(ompl REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

link_libraries(${OMPL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable( rrt_demo_node 
    src/demo_node.cpp
    src/graph_searcher.cpp)

target_link_libraries(rrt_demo_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    ${OMPL_LIBRARIES}
)

