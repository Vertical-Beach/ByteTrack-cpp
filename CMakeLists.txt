cmake_minimum_required(VERSION 3.14)
project(bytetrack)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -O2 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -MMD -Wall -Wextra -Winit-self")

#--- Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC14)
  message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

find_package(Eigen3 3.3.0 QUIET)
if(NOT EIGEN3_FOUND)
  message(FATAL_ERROR "Eigen > 3.3.0 not found.")
endif()

find_package(OpenCV 3.4.0 QUIET)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 3.4.0 not found.")
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
  )

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE_DIR}/src/BYTETracker.cpp
  ${PROJECT_SOURCE_DIR}/src/kalmanFilter.cpp
  ${PROJECT_SOURCE_DIR}/src/lapjv.cpp
  ${PROJECT_SOURCE_DIR}/src/STrack.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${EIGEN3_LIBS}
  ${OpenCV_LIBS}
  )

# Build the tests if the 'BUILD_BYTETRACK_TEST' variable is set to 'ON'
set(BUILD_BYTETRACK_TEST OFF CACHE BOOL "The flag whether to build the tests or not")
if(BUILD_BYTETRACK_TEST)
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    message(FATAL_ERROR "GTest not found.")
  endif()

  add_executable(${PROJECT_NAME}_test
    ${PROJECT_SOURCE_DIR}/test/test_BYTETracker.cpp
  )

  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
    GTest::GTest
    GTest::Main
    )

  gtest_discover_tests(${PROJECT_NAME}_test)
endif()
