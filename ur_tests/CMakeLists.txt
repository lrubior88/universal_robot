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
  pcl_conversions
  pcl_ros
  laser_geometry
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

add_executable(test2 src/test2.cpp)
target_link_libraries(test2 ${catkin_LIBRARIES})

add_executable(laser2pcl src/laser2pcl.cpp)
target_link_libraries(laser2pcl ${catkin_LIBRARIES})

add_executable(pcl_write src/pcl_write.cpp)
target_link_libraries(pcl_write ${catkin_LIBRARIES})

add_executable(pcl_read src/pcl_read.cpp)
target_link_libraries(pcl_read ${catkin_LIBRARIES})
