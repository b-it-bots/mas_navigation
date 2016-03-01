cmake_minimum_required(VERSION 2.8.3)
project(mcr_navigation_tools)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslint
    std_msgs
    nav_msgs
    geometry_msgs
    pcl_ros
)

catkin_package()

include_directories(
    common/include/
    ros/include/
    ${catkin_INCLUDE_DIRS}
)

### library
add_library(path_length_calculator_lib common/src/path_length_calculator.cpp)

### EXECUTABLES
add_executable(path_length_calculator_node ros/src/path_length_calculator_node.cpp)
target_link_libraries(path_length_calculator_node ${catkin_LIBRARIES} path_length_calculator_lib)

add_executable(pose_array_to_path_node ros/src/pose_array_to_path.cpp)
target_link_libraries(pose_array_to_path_node ${catkin_LIBRARIES})

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()

roslint_cpp()

### INSTALLS
install(PROGRAMS
  ros/scripts/navigation_goals_as_marker
  ros/scripts/save_base_map_poses_to_file
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
