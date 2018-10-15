cmake_minimum_required(VERSION 2.8.3)
project(aruco)

SET(CMAKE_BUILD_TYPE Release) #RelWithDebInfo

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  cmake_modules
  cv_bridge
)

find_package(Eigen3)
if ( ${EIGEN3_FOUND} )
	include_directories(${EIGEN3_INCLUDE_DIRS})
	set(CATKIN_EIGEN_DEPENDS Eigen3)
else()
	find_package(Eigen REQUIRED)
	include_directories(${EIGEN_INCLUDE_DIRS})
	set(CATKIN_EIGEN_DEPENDS Eigen)
endif()

find_package(OpenCV REQUIRED)

# Check if we have OpenCV 3
string(REPLACE "." ";" OPENCV_VERSION_LIST ${OpenCV_VERSION})
list(GET OPENCV_VERSION_LIST 0 OPENCV_VERSION_MAJOR)
IF( ${OPENCV_VERSION_MAJOR} STREQUAL "3" )
    ADD_DEFINITIONS(-DOPENCV_VERSION_3)
ENDIF()

#definitions
ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

catkin_package(
	INCLUDE_DIRS aruco/src include
	LIBRARIES aruco
	CATKIN_DEPENDS roscpp tf tf_conversions
	DEPENDS ${CATKIN_EIGEN_DEPENDS} opencv aruco
)

###########
## Build ##
###########
add_subdirectory(aruco)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  aruco/src
)

add_executable(${PROJECT_NAME}_sender
    src/marker_sender.cpp
)

add_dependencies(${PROJECT_NAME}_sender aruco)

## Specify libraries to link a library or executable target against
target_link_libraries(
	${PROJECT_NAME}_sender
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libaruco.so
)

