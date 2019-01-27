cmake_minimum_required(VERSION 2.8.11)

PROJECT(amrl-shared-lib)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)


INCLUDE(FindProtobuf)

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wsign-compare -Werror -g")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
MESSAGE(STATUS "Additional Flags for Release mode")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -openmp -fno-builtin-malloc")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-calloc")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-realloc")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-free -O3 -DNDEBUG")
#SET(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
MESSAGE(STATUS "Additional Flags for Debug mode")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O1")
SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

SET(libs glog pthread popt X11 ${BUILD_SPECIFIC_LIBRARIES})

INCLUDE_DIRECTORIES(src/)


ADD_LIBRARY(shared_library
            src/util/helpers.cc
            src/util/pthread_utils.cc
            src/util/timer.cc
            src/util/colorize.cc
            src/util/random.cc
            src/util/serialization.cc)
TARGET_LINK_LIBRARIES(shared_library ${libs})
