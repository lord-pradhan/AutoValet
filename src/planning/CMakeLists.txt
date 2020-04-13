cmake_minimum_required(VERSION 3.10)

#enable_testing()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#export CXXFLAGS = -I/usr/include/python2.7 -lpython2.7

set_source_files_properties(dubins.c PROPERTIES LANGUAGE CXX )

add_executable(run_planner planner.cpp utilfunction.h utilfunction.cpp params.h params.cpp dubins.h dubins.c)

add_custom_command(TARGET run_planner PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/motion_prims $<TARGET_FILE_DIR:run_planner>)
