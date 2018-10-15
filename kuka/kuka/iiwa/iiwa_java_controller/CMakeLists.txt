cmake_minimum_required(VERSION 2.8.3)
project(iiwa_java_controller)

find_package(catkin REQUIRED)
catkin_package()

#set some variables which are needed in the configured files (see below)
set(JAVA_INSTALL_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})
set(JAR_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../iiwa_java_libs)
set(JAVA_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/application)
set(JAVA_SRCS "UDPController.java UDPIIWAController.java UDPIIWAJointStatePublisher.java")
set(JAR_MAIN_CLASS "application.UDPController")

#configure build file (is generated in build folder directly)
configure_file(../cmake/build_jar.in build_jar @ONLY)

#configure start_controller script (is placed in src folder)
configure_file(start_iiwa_controller.in start_iiwa_controller @ONLY)
file(
	COPY ${CMAKE_CURRENT_BINARY_DIR}/start_iiwa_controller 
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

