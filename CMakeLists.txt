cmake_minimum_required(VERSION 3.0.2)
project(frontier_exploration)

# Find PCL
find_package(PCL 1.10 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  angles
  geometry_msgs
  move_base_msgs
  mrs_msgs
  mrs_modules_msgs
  nav_msgs
  octomap_msgs
  octomap_ros
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  mean_shift_clustering
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib angles geometry_msgs move_base_msgs mrs_msgs nav_msgs octomap_msgs octomap_ros pcl_ros roscpp rospy sensor_msgs std_msgs tf visualization_msgs
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${mrs_msgs_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

link_directories(
    ${PCL_LIBRARY_DIRS})
add_definitions(
    ${PCL_DEFINITIONS})  

add_executable(frontier_exploration src/frontier_exploration.cpp)
target_link_libraries(frontier_exploration
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${mrs_msgs_INCLUDE_DIRS}
)

add_executable(frontier_exploration_livox src/frontier_exploration_livox.cpp)
target_link_libraries(frontier_exploration_livox
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${mrs_msgs_INCLUDE_DIRS}
)