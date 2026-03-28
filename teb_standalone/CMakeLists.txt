cmake_minimum_required(VERSION 3.14)
project(teb_standalone VERSION 1.0.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Build configuration
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# ---------------------------------------------------------------------------
# Custom CMake modules (FindG2O.cmake, FindSUITESPARSE.cmake)
# ---------------------------------------------------------------------------
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)

# ---------------------------------------------------------------------------
# teb_local_planner_standalone library
# ---------------------------------------------------------------------------
set(TEB_STANDALONE_SOURCES
  src/graph_search.cpp
  src/homotopy_class_planner.cpp
  src/obstacles.cpp
  src/optimal_planner.cpp
  src/recovery_behaviors.cpp
  src/teb_config.cpp
  src/timed_elastic_band.cpp
)

add_library(teb_local_planner_standalone ${TEB_STANDALONE_SOURCES})

target_include_directories(teb_local_planner_standalone PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${SUITESPARSE_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
)

# Eigen3 may export a target or just variables depending on the version.
if(TARGET Eigen3::Eigen)
  target_link_libraries(teb_local_planner_standalone PUBLIC Eigen3::Eigen)
else()
  target_include_directories(teb_local_planner_standalone PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

target_link_libraries(teb_local_planner_standalone PUBLIC
  Boost::system
  Boost::thread
  Boost::graph
  ${SUITESPARSE_LIBRARIES}
  ${G2O_LIBRARIES}
  g2o_core
  g2o_stuff
  g2o_csparse_extension
  g2o_solver_csparse
  g2o_solver_cholmod
  g2o_types_slam2d
  g2o_types_slam3d
)

# Force-include the Boost compatibility header so that every translation unit
# has access to boost::prior, boost::optional, etc. before any project header.
target_compile_options(teb_local_planner_standalone PUBLIC
  -include ${CMAKE_CURRENT_SOURCE_DIR}/include/teb_local_planner/boost_compat.h
)

# ---------------------------------------------------------------------------
# pybind11 Python module (pyteb)
# ---------------------------------------------------------------------------
find_package(pybind11 QUIET)
if(pybind11_FOUND)
  pybind11_add_module(pyteb python/pyteb_bindings.cpp)
  target_link_libraries(pyteb PRIVATE teb_local_planner_standalone)
  message(STATUS "pybind11 found – building pyteb Python module")
else()
  message(STATUS "pybind11 not found – skipping pyteb Python module")
endif()

# ---------------------------------------------------------------------------
# Tests (GTest)
# ---------------------------------------------------------------------------
option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)

  file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

  if(TEST_SOURCES)
    add_executable(teb_standalone_tests ${TEST_SOURCES})
    target_link_libraries(teb_standalone_tests PRIVATE
      teb_local_planner_standalone
      GTest::GTest
      GTest::Main
    )
    add_test(NAME teb_standalone_tests COMMAND teb_standalone_tests)
  else()
    message(STATUS "No test sources found in test/ – skipping test target")
  endif()
endif()
