/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/console.h>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
// #include <vector>
// #include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <global_planner/potential_calculator.h>
#include <global_planner/expander.h>
#include <global_planner/traceback.h>
#include <global_planner/orientation_filter.h>
#include <global_planner/GlobalPlannerConfig.h>

#include <tf/tf.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>
#include <math.h>
#include <array>
#include <queue>
#include <vector>
#include <list>
#include <iterator>
#include <limits>
#include <bits/stdc++.h> 
#include <algorithm> 
#include <chrono> 
#include "utilfunction.h"
#include "params.h"
#include<string>
using namespace std::chrono;
using namespace std;
// #include "./matplotlib-cpp/matplotlibcpp.h"
#include <cmath>
#include "dubins.h"

#define numOfDirs 8

using std::string;

#ifndef LATTICE_PLANNER_ROS_CPP
#define LATTICE_PLANNER_ROS_CPP

namespace lattice_planner {

class LatticePlannerROS : public nav_core::BaseGlobalPlanner {
public:

	LatticePlannerROS();
	LatticePlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	// useful functions
	bool world2Cont(float x_world, float y_world, float& x_cont, float& y_cont);

	void cont2World(float x_cont, float y_cont, float& x_world, float& y_world);

	/** overridden classes from interface nav_core::BaseGlobalPlanner **/
	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

	bool makePlan(const geometry_msgs::PoseStamped& start,
	            const geometry_msgs::PoseStamped& goal,
	            std::vector<geometry_msgs::PoseStamped>& plan);


protected:
    /**
     * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
     */
    costmap_2d::Costmap2D* costmap_;
    std::string frame_id_;
    // ros::Publisher plan_pub_;
    bool initialized_;//, allow_unknown_;
	float originX;
	float originY;
	float resolution;
	int width, height;
	MPrimFile readFile;
	long long int functionCall;
	CoordDisc coordsGoalDiscPrev;

	std::vector<State> fullGraph;
	std::vector< std::vector<State_pre> > gridmap_pre;

};

};
#endif