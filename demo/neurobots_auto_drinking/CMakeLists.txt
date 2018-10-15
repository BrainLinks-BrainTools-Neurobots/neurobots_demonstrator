cmake_minimum_required(VERSION 2.8.3)
project(auto_drinking_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    tf
    tf_conversions
    kuka_moveit_controller
    robot_interface_definition)

catkin_package(
  LIBRARIES auto_drinking_planner
  CATKIN_DEPENDS roscpp kuka_moveit_controller tf tf_conversions robot_interface_definition
)

ADD_DEFINITIONS(
    -std=c++11
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(auto_drinking_planner src/auto_drinking_planner.cpp)
target_link_libraries(auto_drinking_planner ${catkin_LIBRARIES})