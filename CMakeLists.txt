cmake_minimum_required(VERSION 2.8.11)

PROJECT(amrl-shared-lib)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(libs glog pthread)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")

ADD_LIBRARY(amrl-shared-lib
            util/helpers.cc
            util/pthread_utils.cc
            util/timer.cc
            util/colorize.cc
            util/random.cc
            util/serialization.cc)
TARGET_LINK_LIBRARIES(amrl-shared-lib ${libs})
