cmake_minimum_required(VERSION 2.8.3)
project(vrpn_odom_ros)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp rospy tf2_ros)
find_package(VRPN REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vrpn_odom_ros
  CATKIN_DEPENDS geometry_msgs nav_msgs tf2_ros
  DEPENDS VRPN
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${VRPN_INCLUDE_DIR}
)

add_library(vrpn_odom_ros
  src/vrpn_odom_ros.cpp
)
target_link_libraries(vrpn_odom_ros
  ${catkin_LIBRARIES}
  ${VRPN_LIBRARIES}
)

add_executable(vrpn_tracker_node
  src/vrpn_tracker_node.cpp)
add_dependencies(vrpn_tracker_node vrpn_odom_ros)
target_link_libraries(vrpn_tracker_node
  vrpn_odom_ros
  ${catkin_LIBRARIES}
)

add_executable(vrpn_odom_node
  src/vrpn_odom_node.cpp)
add_dependencies(vrpn_odom_node vrpn_odom_ros)
target_link_libraries(vrpn_odom_node
  vrpn_odom_ros
  ${catkin_LIBRARIES}
)

install(TARGETS vrpn_odom_ros vrpn_tracker_node vrpn_odom_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
  roslaunch_add_file_check(launch)
endif()
