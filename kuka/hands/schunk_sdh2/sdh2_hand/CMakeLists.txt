cmake_minimum_required(VERSION 2.8.3)
project(sdh2_hand)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

# Find packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs message_generation)

# seems to be important (is used in code)
ADD_DEFINITIONS(-DOSNAME_LINUX)

# Generate msgs
add_message_files(
    FILES 
    TactileMatrix.msg
    TactileSensor.msg 
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SDHAction.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   ${PROJECT_NAME}
)

# Add header
catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES ${PROJECT_NAME}
#    CATKIN_DEPENDS roscpp sensor_msgs message_runtime
#  DEPENDS system_lib
) 

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

link_directories(
    lib     
)

# The executable
add_executable(${PROJECT_NAME} src/main.cpp src/ais_sdh.cpp)
add_dependencies(${PROJECT_NAME} sdh2_hand_gencpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    SDHLibrary-CPP
)
