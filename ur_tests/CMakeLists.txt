cmake_minimum_required(VERSION 2.8.3)
project(ur_tests)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_simple_grasps
  roscpp
  rospy
)
find_package(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_tests
#  CATKIN_DEPENDS cmake_modules moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_simple_grasps roscpp rospy
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(test1 src/test1.cpp)
target_link_libraries(test1 ${catkin_LIBRARIES}) 
