cmake_minimum_required(VERSION 2.8.3)
project(mcr_recovery_behaviors)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            costmap_2d
            nav_core
            pluginlib
            roscpp
            tf
        )

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES force_field_recovery
    CATKIN_DEPENDS
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf
)

add_library(force_field_recovery src/force_field_recovery.cpp)
target_link_libraries(force_field_recovery ${catkin_LIBRARIES})

install(TARGETS force_field_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES ffr_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
