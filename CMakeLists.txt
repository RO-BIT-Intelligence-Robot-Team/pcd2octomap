cmake_minimum_required(VERSION 2.8.3)
project(cvt_pcd2octomap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
SET(CMAKE_BUILD_TYPE Release)

# set exe output dir
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cvt_pcd2octomap
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/cvt_pcd2octomap.cpp
# )

add_subdirectory(${PROJECT_SOURCE_DIR}/app)

add_executable(filter_z src/filter_z.cpp)
target_link_libraries(filter_z ${catkin_LIBRARIES})
