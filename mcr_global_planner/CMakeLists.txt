cmake_minimum_required(VERSION 2.8.3)
project(mcr_global_planner)

find_package(catkin REQUIRED
  COMPONENTS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    global_planner
)

catkin_package(
  INCLUDE_DIRS 
     ros/include/mcr_global_planner_with_orientations
  LIBRARIES 
     mcr_global_planner_with_orientations
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    global_planner
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_library(
    mcr_global_planner_with_orientations ros/src/mcr_global_planner_with_orientations/global_planner_with_orientations.cpp
)
target_link_libraries(
  mcr_global_planner_with_orientations ${catkin_LIBRARIES}
)

add_dependencies(
  mcr_global_planner_with_orientations mcr_global_planner_with_orientations_gencfg
)


install(TARGETS mcr_global_planner_with_orientations
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/mcr_global_planner_with_orientations/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
