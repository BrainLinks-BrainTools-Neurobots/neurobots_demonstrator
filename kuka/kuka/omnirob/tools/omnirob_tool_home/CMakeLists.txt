cmake_minimum_required(VERSION 2.8.3)
project(omnirob_tool_home)

#set some variables which are needed in the configured files (see below)
set(JAVA_INSTALL_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})
set(JAR_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../omnirob_java_libs)
set(JAVA_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/application)
set(JAVA_SRCS "RobotApplication.java")
set(JAR_MAIN_CLASS "application.RobotApplication")

#configure build file (is generated in build folder directly)
configure_file(../../cmake/build_jar.in build_jar @ONLY)

configure_file(../cmake/run.in run @ONLY)
file(
	COPY ${CMAKE_CURRENT_BINARY_DIR}/run
	DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
	FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ GROUP_WRITE
)

#add a target which gets called during the catkin build process (i.e., build_jar is called)
add_custom_target(${PROJECT_NAME} ALL)
add_custom_command(
	TARGET ${PROJECT_NAME}
	PRE_BUILD
	COMMAND bash build_jar
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

