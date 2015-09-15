cmake_minimum_required(VERSION 2.8.3)
project(mcr_relative_base_controller)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    roscpp
    roslint
    rospy
    std_msgs
    tf  
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  ros/config/RelativeBaseController.cfg
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    rospy
    std_msgs
    mcr_geometric_relation_monitors
    mcr_common_converters
    mcr_twist_controller
    mcr_twist_limiter
    mcr_twist_synchronizer
    mcr_manipulation_measurers
)



include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(relative_base_controller_node
  ros/src/relative_base_controller_node.cpp 
)
add_dependencies(relative_base_controller_node 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(relative_base_controller_node
  ${catkin_LIBRARIES}
)


add_executable(relative_base_controller_mockup
  ros/mockup/relative_base_controller_mockup.cpp
)
add_dependencies(relative_base_controller_mockup
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(relative_base_controller_mockup
  ${catkin_LIBRARIES}
)

### INSTALLS
install(PROGRAMS
  ros/scripts/direct_base_controller_coordinator_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS relative_base_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS relative_base_controller_mockup
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)


### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)

  add_rostest_gtest(relative_base_controller_test
    ros/test/relative_base_controller.test
    ros/test/relative_base_controller_test.cpp
  )

  target_link_libraries(relative_base_controller_test
    ${catkin_LIBRARIES}
  )
endif()
