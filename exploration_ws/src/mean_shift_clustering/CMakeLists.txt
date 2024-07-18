cmake_minimum_required(VERSION 2.8.3)
project(mean_shift_clustering)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/MeanShiftAlgorithm.cpp
)
add_executable(${PROJECT_NAME}_node src/cpp_test.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} -fopenmp  ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} -lstdc++fs)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

  install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
