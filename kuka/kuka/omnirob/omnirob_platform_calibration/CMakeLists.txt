cmake_minimum_required(VERSION 2.8.3)
project(omnirob_platform_calibration)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  sensor_msgs
  tf
  tf_conversions
  cv_bridge
  cmake_modules
  kuka_manager
  kuka_moveit_controller
  aruco
)

find_package(OpenCV REQUIRED)

find_package(Eigen3)
if ( ${EIGEN3_FOUND} )
	include_directories(${EIGEN3_INCLUDE_DIRS})
	set(CATKIN_EIGEN_DEPENDS Eigen3)
else()
	find_package(Eigen REQUIRED)
	include_directories(${EIGEN_INCLUDE_DIRS})
	set(CATKIN_EIGEN_DEPENDS Eigen)
endif()

##############################################################################
# Qt
##############################################################################
find_package(Qt5Core REQUIRED)
add_definitions(${Qt5Core_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${Qt5Core_EXECUTABLE_COMPILE_FLAGS}")

#definitions
ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

catkin_package()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME} 
    src/omnirob_platform_calibration.cpp
    src/camera_calibration.cpp
)
add_dependencies(${PROJECT_NAME} aruco)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Qt5Core_LIBRARIES}
)
