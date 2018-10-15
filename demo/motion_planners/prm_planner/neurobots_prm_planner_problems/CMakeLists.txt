cmake_minimum_required(VERSION 2.8.3)
project(neurobots_prm_planner_problems)

set(CMAKE_BUILD_TYPE RelWithDebInfo) #RelWithDebInfo

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#  INCLUDE_DIRS include
#  LIBRARIES neurobots_prm_planner_problems
    CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

find_package(prm_planner QUIET)

###########
## Build ##
###########
if (${prm_planner_FOUND})
    add_definitions(
        -std=c++11
        -D__cplusplus=201103L #for eclipse
    )
    
    find_package(OpenMP)
    if (OPENMP_FOUND)
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
    
    add_definitions(-DFOUND_PRM_PLANNER)

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    # include_directories(include)
    include_directories(
      ${catkin_INCLUDE_DIRS}
      ${prm_planner_INCLUDE_DIRS}
      include
    )
    
    # Declare a C++ library
    add_library(neurobots_prm_planner_problems
       src/drinking_problem_definition.cpp
       src/drinking_interactive_marker.cpp
       src/pouring_problem_definition.cpp
       src/pouring_interactive_marker.cpp
    )
    
    ## Specify libraries to link a library or executable target against
    target_link_libraries(neurobots_prm_planner_problems
       ${catkin_LIBRARIES}
       ${prm_planner_LIBRARIES}
    )

endif()

