cmake_minimum_required(VERSION 2.8.3)
project(ais_util)

find_package(catkin REQUIRED ais_definitions ais_log std_msgs roscpp)

##############################################################################
# Eigen
##############################################################################
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(${Eigen3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS chrono system thread filesystem)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ais_util
  	CATKIN_DEPENDS ais_log std_msgs roscpp
	DEPENDS Eigen3 Boost
)

###########
## Build ##
###########
ADD_DEFINITIONS(
    -std=c++11
	-D__cplusplus=201103L #for eclipse
)

set(UTIL_SOURCES 
    src/color.cpp
    src/date.cpp
    src/time.cpp
    src/stop_watch.cpp
    src/progress_bar.cpp
    src/random.cpp)
    

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
    ${UTIL_SOURCES}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lrt)

## Specify additional locations of header files
## Your package locations should be listed before other locations
