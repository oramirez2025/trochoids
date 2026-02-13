cmake_minimum_required(VERSION 3.0.2)
project(trochoids_ros1)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS ../../include
  LIBRARIES trochoids_ros1
)

add_library(trochoids_ros1
  ../../src/trochoids.cpp
  ../../src/trochoid_utils.cpp
  ../../src/DubinsStateSpace.cpp
  ../../src/ChebTools.cpp
)

target_compile_features(trochoids_ros1 PUBLIC cxx_std_17)
set_target_properties(trochoids_ros1 PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(trochoids_ros1 PUBLIC
  ../../include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(trochoids_ros1 ${catkin_LIBRARIES})

install(TARGETS trochoids_ros1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY ../../include/trochoids
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
