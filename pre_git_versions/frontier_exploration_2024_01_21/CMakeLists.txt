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

add_executable(frontier_extraction src/frontier_extraction.cpp)
target_link_libraries(frontier_extraction
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${mrs_msgs_INCLUDE_DIRS}
)

add_executable(frontier_selection src/frontier_selection.cpp)
target_link_libraries(frontier_selection
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${mrs_msgs_INCLUDE_DIRS}
)

add_executable(path_planning src/path_planning.cpp)
target_link_libraries(path_planning
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${mrs_msgs_INCLUDE_DIRS}
)

add_executable(path_following src/path_following.cpp)
target_link_libraries(path_following
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${mrs_msgs_INCLUDE_DIRS}
)

add_executable(mapping_coverage src/mapping_coverage.cpp)
target_link_libraries(mapping_coverage
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${mrs_msgs_INCLUDE_DIRS}
)