cmake_minimum_required(VERSION 2.8.11)

PROJECT(amrl-shared-lib)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)


SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror -g")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O0")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

SET(libs glog pthread popt ${BUILD_SPECIFIC_LIBRARIES})

INCLUDE_DIRECTORIES(src/)


ADD_LIBRARY(shared_library
            util/helpers.cc
            util/pthread_utils.cc
            util/timer.cc
            util/colorize.cc
            util/random.cc
            util/serialization.cc)
TARGET_LINK_LIBRARIES(shared_library ${libs})
