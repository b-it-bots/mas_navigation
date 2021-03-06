cmake_minimum_required(VERSION 2.8.3)
project(mcr_navigation_tools)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    laser_geometry
    message_filters
    nav_msgs
    pcl_ros
    roscpp
    roslint
    std_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    std_msgs
)

include_directories(
    common/include/
    ros/include/
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

### library
add_library(path_length_calculator common/src/path_length_calculator.cpp)

### EXECUTABLES
add_executable(path_length_calculator_node ros/src/path_length_calculator_node.cpp)
target_link_libraries(path_length_calculator_node ${catkin_LIBRARIES} path_length_calculator)

add_executable(pose_array_to_path_node ros/src/pose_array_to_path_node.cpp)
target_link_libraries(pose_array_to_path_node ${catkin_LIBRARIES})

add_executable(laser_distances_node
    ros/src/laser_distances_node.cpp
)
target_link_libraries(laser_distances_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()

roslint_cpp()

### INSTALLS
install(PROGRAMS
  ros/scripts/pose_visualiser
  ros/scripts/save_base_map_poses_to_file
  ros/scripts/map_saver
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS path_length_calculator
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS path_length_calculator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS pose_array_to_path_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
