cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_pkg)
add_compile_options(-std=c++11)
 
set(EXAMPLE_FILES 
    keypoints3d.cpp)
  
include(${CMAKE_SOURCE_DIR}/telepose/openpose/cmake/Utils.cmake)


find_package(catkin REQUIRED COMPONENTS
                    std_msgs
                    sensor_msgs
                    roscpp
                    cv_bridge
                    roslib
                    image_transport    
                    pcl_ros
                    pcl_conversions
                    geometry_msgs              
                    openpose_ros_msgs
)

find_package(OpenCV REQUIRED)

MESSAGE("Version: ${OpenCV_VERSION}")

set(OPENPOSE_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/telepose/openpose/include
                          ${CMAKE_SOURCE_DIR}/telepose/openpose/src
                          ${CMAKE_SOURCE_DIR}/telepose/openpose/3rdparty/caffe/distribute/include
                          /usr/local/cuda/include)
MESSAGE("Openpose Include Directories: " ${OPENPOSE_INCLUDE_DIRS})
FIND_LIBRARY(OPENPOSE_LIBRARY openpose openpose/build/lib)
FIND_LIBRARY(CAFFE_LIBRARY caffe openpose/3rdparty/caffe/distribute/lib)
FIND_LIBRARY(CUDA_LIBRARY cuda openpose/build/lib)

message("CAFFE DIR: " ${CAFFE_DIR})
set(CAFFE_LINK_LIBRARY ${CAFFE_DIR}/distribute/lib/libcaffe.so)
message("CAFFE Link Library: " ${CAFFE_LINK_LIBRARY})

FIND_LIBRARY(GFLAGS_LIBRARY gflags /usr/lib/x86_64-linux-gnu)
catkin_package(CATKIN_DEPENDS
               std_msgs
               sensor_msgs
               roscpp
               cv_bridge
               roslib
               image_transport
               pcl_ros
               pcl_conversions
               geometry_msgs
               openpose_ros_msgs
               )



include_directories(
${catkin_INCLUDE_DIRS}
${OPENPOSE_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${CUDA_INCLUDE_DIRS}
)

foreach(EXAMPLE_FILE ${EXAMPLE_FILES})

  get_filename_component(SOURCE_NAME ${EXAMPLE_FILE} NAME_WE)
  
  if (UNIX AND NOT APPLE)
    set(EXE_NAME "${SOURCE_NAME}.bin")
  elseif (WIN32)
    set(EXE_NAME "${SOURCE_NAME}")
  endif ()

  message(STATUS "Adding Example ${EXE_NAME}")
  message(STATUS "Adding Example FILE ${EXAMPLE_FILE}")
  add_executable(${SOURCE_NAME} ${EXAMPLE_FILE})
  
  target_link_libraries(${SOURCE_NAME} openpose ${OPENPOSE_LIBRARY} ${GLOG_LIBRARY} ${GFLAGS_LIBRARY} ${Caffe_LIBS} ${MKL_LIBS} ${GLUT_LIBRARY} ${SPINNAKER_LIB} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} ${CAFFE_LIBRARY})
  message(STATUS "Successfully targeted link libraries") 
  if (WIN32)
    set_property(TARGET ${EXE_NAME} PROPERTY FOLDER "Examples/Tutorial/Wrapper")
    configure_file(${CMAKE_SOURCE_DIR}/cmake/OpenPose.vcxproj.user
        ${CMAKE_CURRENT_BINARY_DIR}/${EXE_NAME}.vcxproj.user @ONLY)
    # Properties->General->Output Directory
    set_property(TARGET ${EXE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/$(Platform)/$(Configuration))
    set_property(TARGET ${EXE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/$(Platform)/$(Configuration))
  endif (WIN32)


add_dependencies(${SOURCE_NAME} ${catkin_EXPORTED_TARGETS})


endforeach()
