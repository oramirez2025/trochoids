cmake_minimum_required(VERSION 3.8)
project(trochoids_ros2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(trochoids_ros2 SHARED
  ../../src/trochoids.cpp
  ../../src/trochoid_utils.cpp
  ../../src/DubinsStateSpace.cpp
  ../../src/ChebTools.cpp
)

target_compile_features(trochoids_ros2 PUBLIC cxx_std_17)
set_target_properties(trochoids_ros2 PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(trochoids_ros2 Eigen3::Eigen)
target_include_directories(trochoids_ros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS trochoids_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ../../include/trochoids DESTINATION include)

ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_export_include_directories(include)
ament_export_libraries(trochoids_ros2)

ament_package()
