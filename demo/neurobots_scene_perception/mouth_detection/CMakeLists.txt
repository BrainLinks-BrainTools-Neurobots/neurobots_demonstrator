cmake_minimum_required(VERSION 2.8.3)
project(mouth_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    geometry_msgs 
    std_msgs 
    message_generation 
    image_transport 
    sensor_msgs 
    cv_bridge
    message_filters
    tf)

find_package(OpenCV 3)
if (!${OpenCV_FOUND}) 
	find_package(OpenCV 2 REQUIRED)
endif()

#find_package(Eigen3 REQUIRED)
find_package(Eigen3)
if ( ${EIGEN3_FOUND} )
	include_directories(${EIGEN3_INCLUDE_DIRS})
	set(CATKIN_EIGEN_DEPENDS Eigen3)
else()
	find_package(Eigen REQUIRED)
	include_directories(${EIGEN_INCLUDE_DIRS})
	set(CATKIN_EIGEN_DEPENDS Eigen)
endif()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )


## Generate added messages and services with any dependencies listed here
# generate_messages(  
#    DEPENDENCIES
#    std_msgs
#    geometry_msgs
#    sensor_msgs
#  )


catkin_package(
##  INCLUDE_DIRS include
  LIBRARIES mouth_detection
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime image_transport sensor_msgs cv_bridge
 # DEPENDS system_lib
)

ADD_DEFINITIONS(
    -std=c++11
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
#link_directories(
#  ${catkin_LIBRARY_DIRS}
#)



## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mouth_detect src/mouth_detector.cpp src/detect_mouth.cpp)
target_link_libraries(mouth_detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

##install(TARGETS ${PROJECT_NAME}
##    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
##    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
##    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
