cmake_minimum_required(VERSION 2.8.3)
project(neurobots_commander)

find_package(catkin REQUIRED)
catkin_package()

configure_file(gui/build.in ${CMAKE_CURRENT_SOURCE_DIR}/build @ONLY)
#configure_file(gui/build.in build @ONLY)

#add a target which gets called during the catkin build process (i.e., build is called)
add_custom_target(${PROJECT_NAME} ALL)
add_custom_command(
    TARGET ${PROJECT_NAME}
    PRE_BUILD
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/build
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
