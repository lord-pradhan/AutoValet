cmake_minimum_required(VERSION 2.8)
project(mygmapping)

find_package(catkin REQUIRED nav_msgs nodelet openslam_gmapping roscpp tf rosbag_storage)
#find_package(catkin REQUIRED nav_msgs nodelet openslam_gmapping roscpp tf)
find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(src)

catkin_package()

add_executable(myslam_gmapping src/myslam_gmapping.cpp src/main.cpp)
target_link_libraries(myslam_gmapping ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS myslam_gmapping 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

