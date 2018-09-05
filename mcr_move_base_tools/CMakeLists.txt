cmake_minimum_required(VERSION 2.8.3)
project(mcr_move_base_tools)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
  message_generation
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
   rospy
   std_msgs
   dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#install(PROGRAMS
  #ros/scripts
  #DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
#)
