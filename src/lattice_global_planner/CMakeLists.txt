cmake_minimum_required(VERSION 2.8.3)
project(lattice_global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
	nav_core
	roscpp
	rospy
	std_msgs
	angles
	costmap_2d
	dynamic_reconfigure
	geometry_msgs
	navfn
	nav_msgs
	pluginlib
	roscpp
	tf2_geometry_msgs
	tf2_ros
)


catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES lattice_planner_lib
  CATKIN_DEPENDS 
  	roscpp std_msgs costmap_2d dynamic_reconfigure 
    geometry_msgs nav_core navfn nav_msgs
    pluginlib tf2_ros
  #DEPENDS system_lib
)


include_directories(
  include/lattice_global_planner
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lattice_global_planner.cpp
# )

set_source_files_properties(src/dubins.c PROPERTIES LANGUAGE CXX )

add_library(lattice_planner_lib 
	src/lattice_planner_ros.cpp
	src/utilfunction.cpp
	src/params.cpp
	src/dubins.c
)

#add_dependencies(lattice_planner_lib ${lattice_planner_lib_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(lattice_planner_lib ${catkin_LIBRARIES})


#add_executable(planner src/lattice_planner_ros.cpp)

#add_dependencies(planner ${lattice_planner_lib_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(planner
#  lattice_planner_lib
# ${catkin_LIBRARIES}
#)

#install(TARGETS lattice_planner_lib
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE)

#install(FILES lattice_global_planner_plugin.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

