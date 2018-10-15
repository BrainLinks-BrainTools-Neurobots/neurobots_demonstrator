cmake_minimum_required(VERSION 2.8.3)
project(omnirob_joy)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# Get packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp sensor_msgs joy)

catkin_package(
    CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs joy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#set the default path for built libraries to the "lib" directory

add_executable(omnirob_joy src/omnirob_joy.cpp)

target_link_libraries(omnirob_joy
   ${catkin_LIBRARIES}
 )
