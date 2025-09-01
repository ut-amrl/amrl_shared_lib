cmake_minimum_required(VERSION 3.14.4)

PROJECT(amrl-shared-lib)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(LIBS glog pthread)

SET(AMRL_LIBRARY_NAME "amrl_shared_lib"
    CACHE STRING "Name of compiled library")
SET(AMRL_LIB_UNITTEST_NAME "amrl_shared_lib_tests"
    CACHE STRING "Name of compiled unit test executable")
OPTION(GENERATE_SHARED_LIB_UNITTESTS,
       "Option to enable generation of shared library unit tests"
       OFF)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")

ADD_LIBRARY(${AMRL_LIBRARY_NAME} SHARED
        util/helpers.cc
        util/pthread_utils.cc
        util/timer.cc
        util/random.cc
        util/serialization.cc
        util/terminal_colors.cc)
TARGET_LINK_LIBRARIES(${AMRL_LIBRARY_NAME} ${LIBS})

# Make headers available to consumers when building and after install
target_include_directories(${AMRL_LIBRARY_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)

# Install the shared library and headers so colcon/ament installs can find them
install(TARGETS ${AMRL_LIBRARY_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

# Install headers (copy header files and subdirectories such as util/ and math/)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hh")

IF(GENERATE_SHARED_LIB_UNITTESTS)
  ADD_EXECUTABLE(${AMRL_LIB_UNITTEST_NAME}
                tests/math/line2d_tests.cc
                tests/math/math_tests.cc
                tests/util/kdtree_tests.cc)
  TARGET_LINK_LIBRARIES(${AMRL_LIB_UNITTEST_NAME}
                        ${AMRL_LIBRARY_NAME}
                        gtest
                        gtest_main
                        ${LIBS})
ENDIF()
