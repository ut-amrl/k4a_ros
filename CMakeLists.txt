cmake_minimum_required(VERSION 3.10)
project(k4a_ros LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-march=native -Wall -Werror -g")

if(CMAKE_BUILD_TYPE MATCHES "Release")
  message(STATUS "Additional Flags for Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -O3 -DNDEBUG")
  set(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
  message(STATUS "Additional Flags for Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
  set(BUILD_SPECIFIC_LIBRARIES "")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(amrl_shared_lib REQUIRED)
find_package(config_reader REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Lua 5.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/shared)
include_directories(${PROJECT_SOURCE_DIR}/src/config_reader/include)
include_directories(${LUA_INCLUDE_DIR})

if(EXISTS "${PROJECT_SOURCE_DIR}/src/shared/CMakeLists.txt")
  add_subdirectory(src/shared)
endif()

add_executable(depth_to_lidar
  src/depth_to_lidar.cc
  src/k4a_wrapper.cc
)

ament_target_dependencies(depth_to_lidar
  ament_index_cpp
  amrl_shared_lib
  config_reader
  rclcpp
  sensor_msgs
)

target_link_libraries(depth_to_lidar
  pthread
  boost_system
  ${LUA_LIBRARIES}
  rt
  glog
  k4a
  gflags
  X11
  ${OpenCV_LIBS}
  ${BUILD_SPECIFIC_LIBRARIES}
)

install(TARGETS depth_to_lidar
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

