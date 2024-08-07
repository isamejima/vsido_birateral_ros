cmake_minimum_required(VERSION 2.8.3)
project(vsido_birateral_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  actionlib
  control_msgs
  interbotix_xs_msgs
  interbotix_xs_sdk
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS interbotix_xs_msgs actionlib dynamixel_workbench_toolbox message_runtime roscpp sensor_msgs urdf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(${PROJECT_NAME}
#  src/xs_sdk_obj.cpp
#)
#target_link_libraries(${PROJECT_NAME} PUBLIC ${YAML_CPP_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable

add_executable(vsido_birateral_node src/vsido_birateral_node.cpp src/vsido_birateral.cpp)
target_link_libraries(vsido_birateral_node ${catkin_LIBRARIES} yaml-cpp)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#catkin_install_python(PROGRAMS
#  scripts/xs_sdk_sim
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(
#  DIRECTORY
#    launch
#  DESTINATION
#    ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
