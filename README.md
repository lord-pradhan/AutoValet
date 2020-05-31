# AutoValet
This repository houses the code-base for AutoValet - developed as a graduate course project at Carnegie Mellon University. An end-to-end (perception to actuation) software stack was developed to enable a golf-cart mounted with a single LiDAR sensor to autonomously navigate a parking-lot environment in Gazebo. The parking lot can be unseen and contain dynamic obstacles, in which case the robot does real-time mapping along with navigation. 

The entire stack is built in a ROS framework mainly written in C++ using roscpp library. Custom algorithms for planning and mapping are integrated as plugins with popular ROS stacks like 'move_base' and 'gmapping'.

Gmapping is used for mapping and point-cloud data processing, and custom C++ code is developed to adapt the libraries to our application.

A lattice-graph planner using kinodynamic Ackermann primitives, based on [1], is developed in C++ and integrated as a global planner plugin for move_base, with the local planner being adapted from the TEB Local Planner package [2] in ROS.

To compile the code, clone the repo, build the workspace using catkin build, and download the neccesary dependencies.

Four main demos are built in this project -
1. Golf-cart navigating a pre-known map in a static environment. To run this demo - roslaunch src/lattice_global_planner/launch/parking_test.launch
2. Golf-cart navigating a pre-known map with dynamic obstacles. To run this demo - roslaunch src/lattice_global_planner/launch/parking_test_dynamic.launch
3. Golf-cart navigating an unseen map. To run this demo - roslaunch src/lattice_global_planner/launch/parking_test_mapping.launch
4. Golf-cart being operated through teleop while it does mapping. To run this demo - roslaunch src/lattice_global_planner/launch/parking_test_only_mapping.launch

References
[1] Likhachev, Maxim, and Dave Ferguson. "Planning long dynamically feasible maneuvers for autonomous vehicles." The International Journal of Robotics Research 28.8 (2009): 933-945.
[2] https://github.com/rst-tu-dortmund/teb_local_planner
