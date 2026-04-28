cmake_minimum_required(VERSION 3.16)

project(amrl_shared_lib)

find_package(ament_cmake REQUIRED)

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

ADD_LIBRARY(${AMRL_LIBRARY_NAME}
            util/helpers.cc
            util/pthread_utils.cc
            util/timer.cc
            util/random.cc
            util/serialization.cc
            util/terminal_colors.cc)

target_include_directories(${AMRL_LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

TARGET_LINK_LIBRARIES(${AMRL_LIBRARY_NAME} ${LIBS})

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

install(TARGETS ${AMRL_LIBRARY_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY math util
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${AMRL_LIBRARY_NAME})
ament_export_include_directories(include)

ament_package()
