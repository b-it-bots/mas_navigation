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
     ros/include
  LIBRARIES 
     ${PROJECT_NAME}
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
  ${PROJECT_NAME} ros/src/${PROJECT_NAME}/global_planner_wo.cpp
)
target_link_libraries(
  ${PROJECT_NAME} ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME} ${PROJECT_NAME}_gencfg
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
