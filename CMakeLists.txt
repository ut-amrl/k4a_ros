PROJECT(k4a_to_scan LANGUAGES C CXX)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)

SET(CMAKE_CXX_FLAGS "-march=native -Wall -Werror -g")
set(CMAKE_CXX_STANDARD 14)

FIND_PACKAGE(CUDA)

IF("${CUDA_VERSION_STRING}" STREQUAL "")
  SET(CUDA_FOUND False)
ELSE()
  MESSAGE(STATUS "CUDA detected, enabling GPU code")
  ENABLE_LANGUAGE(CUDA)
  SET(CUDA_FOUND True)
  ADD_DEFINITIONS(-DCUDA_ENABLED)
ENDIF()

MESSAGE(STATUS "CUDA: ${CUDA_FOUND} Version: ${CUDA_VERSION_STRING} Compiler: ${CMAKE_CUDA_COMPILER}")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -fno-builtin-malloc \
      -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -O3 -DNDEBUG")
  SET(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

MESSAGE(STATUS "SOURCE DIR: ${PROJECT_SOURCE_DIR}")

FIND_PACKAGE(Lua51 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/config_reader/include)

ADD_SUBDIRECTORY(src/shared)

ROSBUILD_ADD_EXECUTABLE(stream_pcl
    src/stream_pcl.cc
    src/k4a_wrapper.cc)
ADD_DEPENDENCIES(stream_pcl rosbuild_precompile)
TARGET_LINK_LIBRARIES(stream_pcl roslib roscpp pthread boost_system
    ${LUA_LIBRARIES} rt glog rosbag amrl_shared_lib k4a gflags X11
    ${BUILD_SPECIFIC_LIBRARIES})

ADD_EXECUTABLE(save_rgbd_images
    src/save_rgbd_images.cc
    src/k4a_wrapper.cc)
TARGET_LINK_LIBRARIES(save_rgbd_images pthread
    ${LUA_LIBRARIES} rt glog amrl_shared_lib k4a gflags X11
    ${BUILD_SPECIFIC_LIBRARIES})

ADD_EXECUTABLE(get_calibration
    src/get_calibration.cc
    src/k4a_wrapper.cc)
TARGET_LINK_LIBRARIES(get_calibration pthread
    ${LUA_LIBRARIES} rt glog amrl_shared_lib k4a gflags X11
    ${BUILD_SPECIFIC_LIBRARIES})

ROSBUILD_ADD_EXECUTABLE(depth_to_lidar
    src/depth_to_lidar.cc
    src/k4a_wrapper.cc)
ADD_DEPENDENCIES(depth_to_lidar rosbuild_precompile)
TARGET_LINK_LIBRARIES(depth_to_lidar roslib roscpp pthread boost_system
    ${LUA_LIBRARIES} rt glog rosbag amrl_shared_lib k4a gflags X11
    ${BUILD_SPECIFIC_LIBRARIES})

IF(CUDA_FOUND)
  ADD_LIBRARY(cuda_lib src/processing_kernels_cuda.cu)
  ROSBUILD_ADD_EXECUTABLE(depth_to_costmap
      src/depth_to_costmap.cc
      src/k4a_wrapper.cc)
  ADD_DEPENDENCIES(depth_to_costmap rosbuild_precompile)
  TARGET_LINK_LIBRARIES(depth_to_costmap roslib roscpp pthread boost_system
      ${LUA_LIBRARIES} rt glog rosbag amrl_shared_lib k4a gflags X11 cuda_lib
      ${BUILD_SPECIFIC_LIBRARIES})
ENDIF()

