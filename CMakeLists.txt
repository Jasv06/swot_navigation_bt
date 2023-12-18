cmake_minimum_required(VERSION 2.8.3)
project(swot_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES swot_navigation navigation CHANGE ACCORDINGLY THE DIVISION manipulation pick_classes place_classes shared_classes
 CATKIN_DEPENDS roscpp swot_msgs behaviortree_cpp
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/src/swot_navigation/include
)

install(DIRECTORY include/swot_navigation/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

set(REQUIRED_FILES
  src/manipulation.cpp
  src/pick_classes.cpp
  src/place_classes.cpp
  src/shared_classes.cpp
)

add_library(${PROJECT_NAME}_lib
  ${REQUIRED_FILES}
)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} behaviortree_cpp)

add_executable(swot_navigation src/main.cpp)
add_dependencies(swot_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(swot_navigation ${catkin_LIBRARIES} behaviortree_cpp ${PROJECT_NAME}_lib)
