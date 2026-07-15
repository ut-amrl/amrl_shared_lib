cmake_minimum_required(VERSION 3.16)
project(amrl_shared_lib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# glog (which pulls in gflags) from the system package (libgoogle-glog-dev).
find_library(GLOG_LIBRARY NAMES glog REQUIRED)

# Build position-independent so this static lib can be linked into shared
# objects (e.g. the motion_planner Python extension module).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}
  util/helpers.cc
  util/pthread_utils.cc
  util/timer.cc
  util/random.cc
  util/serialization.cc
  util/terminal_colors.cc)

# During the build the sources include their own headers as "util/..." / "math/..."
# (repo root on the include path). For consumers we install the headers under a
# "shared/" prefix so they resolve as "shared/math/..." (AMRL convention), while
# the headers' own cross-includes ("math/..", "util/..") resolve via include/shared.
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/shared>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ${GLOG_LIBRARY}
  pthread)

install(DIRECTORY math DESTINATION include/shared FILES_MATCHING PATTERN "*.h")
install(DIRECTORY util DESTINATION include/shared FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include include/shared)
ament_export_dependencies(Eigen3)

ament_package()
