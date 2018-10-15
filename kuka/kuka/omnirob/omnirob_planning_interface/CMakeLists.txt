cmake_minimum_required(VERSION 2.8.3)
project(omnirob_planning_interface)

SET(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    geometric_shapes #moveit
    roscpp
    tf
    tf_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3)
if ( ${EIGEN3_FOUND} )
	include_directories(${EIGEN3_INCLUDE_DIRS})
	set(CATKIN_EIGEN_DEPENDS Eigen3)
else()
	find_package(Eigen REQUIRED)
	include_directories(${EIGEN_INCLUDE_DIRS})
	set(CATKIN_EIGEN_DEPENDS Eigen)
endif()

find_package(Boost REQUIRED)

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
    LIBRARIES omnirob_planning_interface
    CATKIN_DEPENDS cmake_modules moveit_core moveit_ros_planning moveit_ros_planning_interface roscpp tf tf_conversions geometric_shapes
    DEPENDS ${CATKIN_EIGEN_DEPENDS} Boost
)

ADD_DEFINITIONS(
    -std=c++11
)

if (${CMAKE_BUILD_TYPE} EQUAL "Release")
    if(CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
        set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary
    endif()
endif()

###########
## Build ##
###########

# TODO: Check names of system library include directories (Eigen, Boost)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(omnirob_planning_interface
    src/omnirob_planning_interface.cpp
    src/plan.cpp
    src/constraint_factory.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(omnirob_planning_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(omnirob_planning_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(omnirob_planning_interface
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

