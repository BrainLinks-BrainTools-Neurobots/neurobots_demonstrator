cmake_minimum_required(VERSION 2.8.3)
project(realtime_urdf_filter)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  urdf
  tf
  resource_retriever
  cv_bridge
  image_transport)

catkin_package(
#  DEPEND
  CATKIN_DEPENDS resource_retriever roscpp nodelet urdf tf cv_bridge image_transport
  INCLUDE_DIRS include 
  # LIBRARIES urdf_filter shadowwrapper
  )

include_directories(${catkin_INCLUDE_DIRS} include)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)
find_package(OpenNI)
find_package(OpenCV REQUIRED)
find_package(OpenGL)
find_package(urdfdom_headers REQUIRED)
find_library(freeglut_LIBRARY glut /usr/lib)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${OPENGL_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS} /usr/include)

find_path( freeglut_INCLUDE_DIR GL/freeglut.h
  /usr/include/GL
  /usr/openwin/share/include
  /usr/openwin/include
  /opt/graphics/OpenGL/include
  /opt/graphics/OpenGL/contrib/libglut
  )
include_directories(${freeglut_INCLUDE_DIR})

add_library (FBO src/FrameBufferObject.cpp)
target_link_libraries (FBO GLEW)

add_library (shaderwrapper src/shader_wrapper.cpp)

add_library (urdf_filter
  src/urdf_filter.cpp
  src/urdf_renderer.cpp
  src/renderable.cpp)
target_link_libraries (urdf_filter
  ${OPENGL_LIBRARIES}
  ${freeglut_LIBRARY}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  FBO
  shaderwrapper
  assimp)

#add_executable (urdf_filtered_tracker src/urdf_filtered_tracker.cpp)
#target_link_libraries (urdf_filtered_tracker urdf_filter OpenNI ${catkin_LIBRARIES})

add_executable (realtime_urdf_filter src/realtime_urdf_filter.cpp)
target_link_libraries (realtime_urdf_filter urdf_filter ${catkin_LIBRARIES})

add_library (realtime_urdf_filter_nodelet src/realtime_urdf_filter_nodelet.cpp)
target_link_libraries (realtime_urdf_filter_nodelet urdf_filter ${catkin_LIBRARIES})
