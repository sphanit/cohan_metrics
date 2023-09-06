cmake_minimum_required(VERSION 2.8.3)
project(cohan_metrics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  cohan_msgs
  roscpp
  rospy
  tf
  roslib
)

find_package(Eigen3 REQUIRED)
# include_directories(${Eigen3_INCLUDE_DIRS})
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include_directories(${EIGEN3_INCLUDE_DIRS})


###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
 LIBRARIES cohan_metrics ${EXTERNAL_LIBS}
 CATKIN_DEPENDS geometry_msgs nav_msgs cohan_msgs roscpp rospy tf roslib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/cohan_metrics.cpp
)

## Specify libraries to link a library or executable target against (always link the target libraries to the C++ Lib or executable)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTERNAL_LIBS})

## Declare a C++ executable
add_executable(metrics_node src/cohan_metrics_node.cpp)

## Add cmake target dependencies of the library
add_dependencies(metrics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(metrics_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${EXTERNAL_LIBS})


#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
