cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



#add_library(timelib src/TimeClient.cpp src/timelib.cpp src/udpListener.cpp src/udpSender.cpp src/linux_time.cpp)
#rosbuild_link_boost(timelib thread date_time)


#rosbuild_add_executable(time_client src/bin_time_client.cpp)
#target_link_libraries(time_client timelib)

#rosbuild_add_executable(time_server src/bin_time_server.cpp)
#target_link_libraries(time_server timelib)

#rosbuild_add_executable(time_client_alt src/bin_time_client_alternate.cpp)
#target_link_libraries(time_client_alt timelib)




cmake_minimum_required(VERSION 2.8.3)
project(timesync)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES timelib
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf trajectory_msgs message_runtime
#  DEPENDS system_lib
) 

###########
## Build ##
###########
set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE( Boost COMPONENTS thread date_time REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} ) 

#
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    include/TICSyncCore/
    ${catkin_INCLUDE_DIRS}
)
add_library(timelib src/TimeClient.cpp src/timelib.cpp src/udpListener.cpp src/udpSender.cpp src/linux_time.cpp)
target_link_libraries(timelib ${Boost_LIBRARIES})

## Declare a cpp library
# add_library(omnirob_manager
#   src/${PROJECT_NAME}/omnirob_manager.cpp
# )

## Declare a cpp executable
add_executable(time_client src/bin_time_client.cpp)
target_link_libraries(time_client timelib)

add_executable(time_server src/bin_time_server.cpp)
target_link_libraries(time_server timelib)

add_executable(time_client_alt src/bin_time_client_alternate.cpp)
target_link_libraries(time_client_alt timelib)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(omnirob_manager omnirob_manager_generate_messages_cpp)

