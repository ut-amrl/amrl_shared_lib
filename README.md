[![Test Status](https://github.com/ut-amrl/amrl_shared_lib/actions/workflows/buildTest.yml/badge.svg)](https://github.com/ut-amrl/amrl_shared_lib/actions)

# amrl_shared_lib
Shared library for all AMRL C++ projects

## ROS1/ROS2 Compatibility ✅

**This library is fully cross-compatible between ROS1 and ROS2.** The compatibility is achieved through:

1. **Conditional Compilation**: Uses `#ifdef ROS2` preprocessor directives to include appropriate headers and use correct APIs  
2. **Type Aliases**: Provides unified type names that map to the correct ROS1 or ROS2 message types
3. **Abstraction Macros**: Logging and timing functions are abstracted through macros that call the appropriate ROS1/ROS2 functions
4. **Optional ROS Dependency**: The core library functionality is completely independent of ROS - ROS features are only in the `ros/` directory

## Build
1. Install dependencies:
   ```
   sudo apt-get install libgtest-dev libgoogle-glog-dev cmake build-essential
   ```
1. Compile:
   ```
   make -j`nproc`
   ```

## Usage in Parent Projects

This library is designed to be used as a git submodule in larger projects. Here's how to integrate it:

### 1. Add as Submodule
In your parent project, add this as a git submodule:
```bash
git submodule add git@github.com:umass-amrl/amrl_shared_lib.git src/shared
```
Or using HTTPS:
```bash
git submodule add https://github.com/ut-amrl/amrl_shared_lib.git src/shared
```

### 2. Basic CMakeLists.txt Setup
In your parent project's main `CMakeLists.txt`:
```cmake
# Include and build amrl_shared_lib (order matters)
INCLUDE_DIRECTORIES(src/shared)
ADD_SUBDIRECTORY(src/shared)

# Your executable
ADD_EXECUTABLE(my-program src/main.cc)
TARGET_LINK_LIBRARIES(my-program amrl_shared_lib)
```

### 3. ROS Integration
To use ROS features, add automatic ROS detection to your parent project's `CMakeLists.txt`:

```cmake
# Auto-detect ROS version from environment
if("$ENV{ROS_VERSION}" STREQUAL "2")
    # ROS2 setup
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    ament_target_dependencies(my-program rclcpp std_msgs geometry_msgs)
    add_compile_definitions(ROS2)
else()
    # ROS1 setup
    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)
    target_link_libraries(my-program ${catkin_LIBRARIES})
endif()

# Standard amrl_shared_lib setup
INCLUDE_DIRECTORIES(src/shared)
ADD_SUBDIRECTORY(src/shared)
ADD_EXECUTABLE(my-program src/main.cc)
TARGET_LINK_LIBRARIES(my-program amrl_shared_lib)
```

### Advanced Configuration

To change the name of the shared library, set `AMRL_LIBRARY_NAME` prior to the  `INCLUDE_DIRECTORIES` call in Step 2, e.g.:

```
SET(AMRL_LIBRARY_NAME "alternate_link_name"
    CACHE STRING "Name of compiled library")
```

To generate the unit tests for the shared library, set `GENERATE_SHARED_LIB_UNITTESTS` to `ON` prior to the  `INCLUDE_DIRECTORIES` call in Step 2, e.g.:

```
SET(GENERATE_SHARED_LIB_UNITTESTS ON)
```
