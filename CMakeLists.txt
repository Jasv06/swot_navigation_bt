cmake_minimum_required(VERSION 2.8.3)
project(swot_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	move_base_msgs
	roscpp
	nav_msgs
	std_msgs
	swot_msgs
	roslib
	actionlib_msgs
	actionlib
	behaviortree_cpp
)

## The catkin_package macro generates cmake config files for the package
## Declare things to be passed to dependent projects
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES swot_navigation swot_robocup_navigation navigation_tree
 CATKIN_DEPENDS roscpp swot_msgs behaviortree_cpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CMAKE_SOURCE_DIR}/src/swot_navigation/include
)

install(DIRECTORY include/swot_navigation/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

link_directories($ENV{HOME}/catkin_ws/devel/lib)

set(REQUIRED_FILES
  src/swot_robocup_navigation.cpp
  src/navigation_tree.cpp
)

add_library(${PROJECT_NAME}_lib
  ${REQUIRED_FILES}
)

add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} behaviortree_cpp)

add_executable(swot_navigation src/main.cpp)
add_dependencies(swot_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(swot_navigation ${catkin_LIBRARIES} behaviortree_cpp ${PROJECT_NAME}_lib)

#add executable for point controller
add_executable(swot_navigation_monitoring_node src/navigation_monitoring.cpp)
target_link_libraries(swot_navigation_monitoring_node ${catkin_LIBRARIES})

add_executable(swot_move_to_pose src/swot_move_to_pose.cpp)
target_link_libraries(swot_move_to_pose ${catkin_LIBRARIES})
add_executable(swot_euler_pose_publisher src/quaternion_to_euler.cpp)
target_link_libraries(swot_euler_pose_publisher ${catkin_LIBRARIES})



