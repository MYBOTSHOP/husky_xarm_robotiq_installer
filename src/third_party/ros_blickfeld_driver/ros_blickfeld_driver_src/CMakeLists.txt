cmake_minimum_required(VERSION 3.13)
project(blickfeld_driver)

## Compile as C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  diagnostic_updater
  geometry_msgs
  message_generation
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(blickfeld-scanner 2.18.2 REQUIRED full)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  ImuStaticTransformation.srv
  SetScanPattern.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include modules/ros_${PROJECT_NAME}_core/include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_core
  CATKIN_DEPENDS cv_bridge geometry_msgs message_runtime nodelet roscpp sensor_msgs std_msgs tf2_geometry_msgs tf2_ros
  DEPENDS blickfeld-scanner 
)

###########
## Build ##
###########

include_directories(
  include
  modules/ros_${PROJECT_NAME}_core/include
  ${catkin_INCLUDE_DIRS}
)

################################################
## Submodules ##
################################################
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/modules/ros_${PROJECT_NAME}_core")

## The driver_library itself
add_library(${PROJECT_NAME}
  src/blickfeld_driver.cpp
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_NAME}_core
  PRIVATE
    ${catkin_LIBRARIES}
    blickfeld-scanner
)
add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

## The driver_node
add_executable(${PROJECT_NAME}_node
  src/blickfeld_driver_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)

## The driver nodelet
add_library(${PROJECT_NAME}_nodelet
  src/${PROJECT_NAME}_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  PRIVATE
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)

#############
## Install ##
#############

## Nodelets
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core ${PROJECT_NAME}_nodelet
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Nodes
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Header Files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Launch Files
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY modules/ros_${PROJECT_NAME}_core/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Config Files
install(DIRECTORY config/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# to copy nodelet plugin xml
install(FILES nodelet_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
