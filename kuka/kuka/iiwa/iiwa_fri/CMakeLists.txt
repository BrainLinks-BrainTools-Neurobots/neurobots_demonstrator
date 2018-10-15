cmake_minimum_required(VERSION 2.8.3)
project(iiwa_fri)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

set(INCLUDES
    include
    include/iiwa_fri
    src/base
    src/nanopb-0.2.8
    src/protobuf
    src/protobuf_gen
)

catkin_package(
    INCLUDE_DIRS ${INCLUDES}
    LIBRARIES iiwa_fri
    CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${INCLUDES}
)

add_definitions(-DHAVE_SOCKLEN_T 
    -DPB_SYSTEM_HEADER="pb_syshdr.h" 
    -DHAVE_STDINT_H 
    -DHAVE_STDDEF_H 
    -DHAVE_STDBOOL_H 
    -DHAVE_STDLIB_H 
    -DHAVE_STRING_H
)

## Declare a C++ library
add_library(iiwa_fri
    src/base/friClientApplication.cpp
    src/protobuf_gen/FRIMessages.pb.c
    src/nanopb-0.2.8/pb_encode.c
    src/nanopb-0.2.8/pb_decode.c
    src/protobuf/pb_frimessages_callbacks.c
    src/protobuf/friMonitoringMessageDecoder.cpp
    src/protobuf/friCommandMessageEncoder.cpp
    src/connection/friUdpConnection.cpp
    src/client_lbr/friLBRClient.cpp
    src/client_lbr/friLBRCommand.cpp
    src/client_lbr/friLBRState.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(iiwa_fri ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(iiwa_fri_node src/iiwa_fri_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(iiwa_fri_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(iiwa_fri_node
#   ${catkin_LIBRARIES}
# )
