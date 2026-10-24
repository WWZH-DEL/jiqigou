cmake_minimum_required(VERSION 2.8.3)
project(astar_search)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES astar_search
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(astar_search
  src/astar_search.cpp
  src/astar_util.cpp
  src/astar_avoid.cpp
)

target_link_libraries(astar_search
  ${catkin_LIBRARIES}
)

add_dependencies(astar_search
  ${catkin_EXPORTED_TARGETS}
)

add_executable(astar_avoid_node
  src/astar_avoid_node.cpp
)

target_link_libraries(astar_avoid_node
  astar_search
  ${catkin_LIBRARIES}
)

install(TARGETS astar_avoid_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

add_dependencies(astar_avoid_node
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS astar_search astar_avoid_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(
#     astar_search-test
#     test/test_astar_search.test
#     test/src/test_main.cpp
#     test/src/test_astar_util.cpp
#     test/src/test_astar_search.cpp
#     test/src/test_class.cpp
#   )
#   target_link_libraries(astar_search-test ${catkin_LIBRARIES} astar_search)
# endif()
