cmake_minimum_required(VERSION 2.8.3)
project(kuka_manager)

find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs 
    roscpp 
    sensor_msgs 
    tf 
    trajectory_msgs 
    message_generation 
    timesync
    omnirob_lbr
    roslib)

find_package(timesync REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################
## Generate messages in the 'msg' folder
 add_message_files(
   FILES 
   Force.msg
   Speed.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   JointState.srv
   SpeedAcceleration.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   kuka_manager
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf trajectory_msgs message_runtime
) 

###########
## Build ##
###########

#definitions
ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

file(GLOB_RECURSE MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(kuka_manager ${MY_SOURCES})

# Forces the generation of msgs and srvs
#add_dependencies(omnirob_manager ${catkin_EXPORTED_TARGETS})
add_dependencies(kuka_manager kuka_manager_gencpp)

# Link libraries
 target_link_libraries(kuka_manager
   ${catkin_LIBRARIES}
   ${timesync_LIBRARIES}
 )
