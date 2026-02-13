cmake_minimum_required(VERSION 3.16)
project(trochoids VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Type of build" FORCE)
endif()

option(TROCHOIDS_BUILD_TESTS "Build unit tests" ON)
option(TROCHOIDS_BUILD_BENCHMARKS "Build benchmarks" OFF)
option(TROCHOIDS_BUILD_PYTHON "Build Python module" OFF)

find_package(Eigen3 REQUIRED)

add_library(trochoids_core
  src/trochoids.cpp
  src/trochoid_utils.cpp
  src/DubinsStateSpace.cpp
  src/ChebTools.cpp
)
add_library(trochoids::core ALIAS trochoids_core)

target_compile_features(trochoids_core PUBLIC cxx_std_17)
set_target_properties(trochoids_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(trochoids_core PRIVATE
  $<$<CONFIG:Release>:-O3>
)
target_link_libraries(trochoids_core PUBLIC Eigen3::Eigen)
target_include_directories(trochoids_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

include(GNUInstallDirs)
install(TARGETS trochoids_core
  EXPORT trochoidsTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/trochoids DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/trochoidsConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/trochoidsConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/trochoidsConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/trochoids"
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/trochoidsConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/trochoidsConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/trochoids"
)
install(EXPORT trochoidsTargets
  FILE trochoidsTargets.cmake
  NAMESPACE trochoids::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/trochoids"
)

if(TROCHOIDS_BUILD_TESTS)
  include(CTest)
  enable_testing()
  if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(GTest_FOUND)
      add_executable(trochoids-test test/unit_test_trochoids.cpp)
      target_link_libraries(trochoids-test PRIVATE trochoids_core GTest::gtest_main)
      add_test(NAME trochoids-test COMMAND $<TARGET_FILE:trochoids-test>)

      add_executable(trochoids-3d-test test/unit_test_trochoids_3d.cpp)
      target_link_libraries(trochoids-3d-test PRIVATE trochoids_core GTest::gtest_main)
      add_test(NAME trochoids-3d-test COMMAND $<TARGET_FILE:trochoids-3d-test>)
    endif()
  endif()
endif()

if(TROCHOIDS_BUILD_BENCHMARKS)
  find_package(benchmark REQUIRED)
  add_executable(trochoids-benchmark test/all_benchmarks.cpp)
  target_link_libraries(trochoids-benchmark PRIVATE trochoids_core benchmark::benchmark)
endif()

if(TROCHOIDS_BUILD_PYTHON)
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(_trochoids MODULE python/src/bindings.cpp)
  target_link_libraries(_trochoids PRIVATE trochoids_core)
  target_compile_features(_trochoids PRIVATE cxx_std_17)
  install(TARGETS _trochoids LIBRARY DESTINATION trochoids)
  install(FILES python/src/trochoids/__init__.py DESTINATION trochoids)
endif()
