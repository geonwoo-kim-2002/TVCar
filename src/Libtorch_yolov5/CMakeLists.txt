cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(Libtorch_yolov5)

SET(CMAKE_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS 
  cv_bridge
  image_geometry
  image_transport
  #pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  darknet_ros_msgs
)

set(Torch_DIR ~/libtorch/share/cmake/Torch)
#set(Torch_DIR /home/a/yolo_ws/src/libtorch-yolov5/libtorch/share/cmake/Torch)
# set(Torch_DIR /home/a/yolo_ws/src/libtorch-yolov5/libtorch/share/cmake/Torch)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.3)

find_package(Torch REQUIRED)

find_package(OpenCV REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES cpp_yolo
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
  #DEPENDS system_lib
)

include_directories(
${PROJECT_SOURCE_DIR}/include
${OpenCV_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)


add_executable(
  Libtorch_yolov5 
${project_HEADERS}
    src/main.cpp
    src/detector.cpp
    src/libtorch_yolo.cpp)

add_dependencies(Libtorch_yolov5 ${catkin_EXPORTED_TARGETS})
target_link_libraries(Libtorch_yolov5 ${OpenCV_LIBS} ${TORCH_LIBRARIES} ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
#set(CMAKE_PREFIX_PATH /home/nebula/libtorch)

set_property(TARGET Libtorch_yolov5 PROPERTY CXX_STANDARD 14)

if (MSVC)
  file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
  add_custom_command(TARGET ${PROJECT_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     ${TORCH_DLLS}
                     $<TARGET_FILE_DIR:${PROJECT_NAME}>)
endif (MSVC)


