cmake_minimum_required(VERSION 2.8.3)
project(goal_planner_gui_cpp)

set(CMAKE_BUILD_TYPE RelWithDebInfo) #RelWithDebInfo

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  actionlib
  robot_interface_msgs
  database_msgs
  database_conversions
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED) 
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Quick REQUIRED)

set(FAST_DOWNWARD_FIND_PATHS
        $ENV{HOME}/fast-downward
        $ENV{HOME}/planners/fast-downward
        $ENV{HOME}/development/fast-downward
)

find_path(FAST_DOWNWARD_PATH fast-downward.py PATHS ${FAST_DOWNWARD_FIND_PATHS})

if (FAST_DOWNWARD_PATH-NOTFOUND)
        message(WARNING "\n\tCannot find fast-downward planner. It is needed to use\n
                the goal_planner_gui package. If the planner is installed please\n
                add your path to the FAST_DOWNWARD_FIND_PATHS variable in the\n
                goal_planner_guis' CMakeLists.txt file. CMake will then try to\n
                find the planner and automatically regenerate the config file.\n")
else()
        message("\n\t-- Found fast-downward planner in ${FAST_DOWNWARD_PATH}\n")
endif()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES goal_planner_gui_cpp
  CATKIN_DEPENDS roscpp roslib std_msgs actionlib database_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
)

add_definitions(${Qt5Widgets_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} ${Qt5Quick_EXECUTABLE_COMPILE_FLAGS} -Werror=return-type")

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/goal_planner_gui_cpp.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
set(UI_HEADER
        include/goal_planner_gui/gui/selector.h
        include/goal_planner_gui/gui/goal_menu_item.h
        include/goal_planner_gui/gui/misc_menu_item.h
        include/goal_planner_gui/gui/image_text_item.h
        include/goal_planner_gui/gui/ref_item.h
        include/goal_planner_gui/gui/current_goal_item.h
        include/goal_planner_gui/gui/plan_item.h
)

qt5_wrap_cpp(goal_planner_gui_cpp_moc ${UI_HEADER})

add_library(pddl
        src/pddl/exception.cpp
        src/pddl/parser.cpp
        src/pddl/domain.cpp
        src/pddl/types.cpp
        src/pddl/utils.cpp
        src/pddl/modules.cpp
        src/pddl/functions.cpp
        src/pddl/scope.cpp
        src/pddl/visitors.cpp
        src/pddl/base_visitor.cpp
        src/pddl/builtin.cpp
        src/pddl/function_table.cpp
        src/pddl/predicates.cpp
        src/pddl/requirements.cpp
        src/pddl/base_element.cpp
        src/pddl/translators.cpp
        src/pddl/actions.cpp
        src/pddl/axioms.cpp
        src/pddl/problem.cpp
        src/pddl/literal.cpp
        src/pddl/terms.cpp
        src/pddl/parse_context.cpp 
        src/pddl/tagable_object.cpp 

        src/pddl/state/state.cpp
        src/pddl/state/facts.cpp
        src/pddl/state/state_variable.cpp
          
        src/pddl/effects/effect.cpp
        src/pddl/effects/conditional_effect.cpp
        src/pddl/effects/conjunctive_effect.cpp
        src/pddl/effects/simple_effect.cpp
        src/pddl/effects/universal_effect.cpp

        src/pddl/conditions/condition.cpp
        src/pddl/conditions/conjunction.cpp
        src/pddl/conditions/disjunction.cpp
        src/pddl/conditions/intermediate_condition.cpp
        src/pddl/conditions/junction_condition.cpp
        src/pddl/conditions/preference_condition.cpp
        src/pddl/conditions/quantified_condition.cpp
        src/pddl/conditions/universal_condition.cpp
        src/pddl/conditions/existential_condition.cpp
        src/pddl/conditions/literal_condition.cpp
)

add_executable(goal_planner_gui_cpp 
		src/main.cpp 
		src/constants.cpp 
		src/experiment_logger.cpp 
		src/neurobots_world.cpp
        src/partitions.cpp 
        src/planning.cpp 
        src/reference_list.cpp 
        src/relaxed_exploration.cpp
        src/ros_database.cpp 
        src/translator.cpp 
        src/command_line.cpp 
        src/config_parser.cpp
        src/partition_entry.cpp
        src/alternative_partition_entry.cpp
        src/existential_partition_entry.cpp
        src/universal_partition_entry.cpp
        
        src/goals/action_goal.cpp
        src/goals/alternative_action_goal.cpp
        src/goals/alternative_function_goal.cpp
        src/goals/function_goal.cpp
        src/goals/goal_spec.cpp
		src/goals/goal_context.cpp 
        
        src/references/reference.cpp
        src/references/feature_reference.cpp
        src/references/function_reference.cpp
        src/references/identity_reference.cpp
        src/references/other_reference.cpp
        src/references/relational_reference.cpp
        src/references/typename_reference.cpp
        
        src/gui/selector.cpp
        src/gui/goal_menu_item.cpp
        src/gui/back_menu_item.cpp
        src/gui/misc_menu_item.cpp
        src/gui/image_text_item.cpp
        src/gui/ref_item.cpp
        src/gui/current_goal_item.cpp
        src/gui/plan_item.cpp
        
        src/relaxed_exploration/plan_graph.cpp
        src/relaxed_exploration/proposition.cpp
        src/relaxed_exploration/unary_op.cpp
        
        src/planning/planner.cpp
        src/planning/writer.cpp
        src/planning/plan.cpp
        src/planning/plan_node.cpp
        
        ${goal_planner_gui_cpp_moc}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(goal_planner_gui_cpp
    pddl
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    Qt5::Widgets Qt5::Quick
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/config.ini.in ${CMAKE_CURRENT_SOURCE_DIR}/src/config.ini @ONLY)
