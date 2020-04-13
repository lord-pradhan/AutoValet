 g++ -g planner.cpp utilfunction.cpp params.cpp dubins.c -I/usr/include/python2.7 -lpython2.7 -I/home/lord-pradhan/.local/lib/python2.7/site-packages/numpy/core/include

 if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

add_executable(run_planner planner.cpp utilfunction.h utilfunction.cpp params.h params.cpp dubins.h dubins.c )
#target_link_libraries(ps7 fslazywindow geblkernel meshlattice glutil)