cmake_minimum_required(VERSION 2.8.3)
project(kuka_moveit_controller)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  kuka_manager
  wsg_gripper
  sdh2_hand
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MoveItControl
  CATKIN_DEPENDS moveit_ros_planning_interface
	  roscpp
	  rospy
	  std_msgs
	  geometry_msgs
	  sensor_msgs
	  kuka_manager
	  wsg_gripper
	  sdh2_hand
)

###########
## Build ##
###########

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(MoveItControl src/MoveItController.cpp)
target_link_libraries(MoveItControl ${catkin_LIBRARIES})
add_dependencies(MoveItControl sdh2_hand_generate_messages_cpp)
add_dependencies(MoveItControl wsg_gripper_generate_messages_cpp)

add_executable(kuka_moveit_controller src/moveit_controller_main.cpp src/MoveItController.cpp)
target_link_libraries(kuka_moveit_controller ${catkin_LIBRARIES})
add_dependencies(kuka_moveit_controller sdh2_hand_generate_messages_cpp)
add_dependencies(kuka_moveit_controller wsg_gripper_generate_messages_cpp)
