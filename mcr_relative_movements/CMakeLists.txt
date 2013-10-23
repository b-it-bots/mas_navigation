cmake_minimum_required(VERSION 2.8.3)
project(mcr_relative_movements)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    actionlib
    geometry_msgs
    mcr_navigation_msgs
    nav_msgs
    roscpp
    rospy
    std_msgs
    std_srvs
    tf  
)
find_package(Eigen REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mcr_navigation_msgs
    nav_msgs
    rospy
    std_msgs
    std_srvs
)

include_directories(
  common/include
  ros/include
  ${Eigen_INCLUDE_DIRS}
)

add_executable(relative_movements_node
  ros/src/relative_movements_node.cpp 
  common/src/homogenous_transform.cpp
)
add_dependencies(relative_movements_node 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(relative_movements_node
  ${catkin_LIBRARIES}
)

install(PROGRAMS
  ros/scripts/test_relative_movements
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS relative_movements_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
