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

using std::string;

#ifndef LATTICE_PLANNER_ROS_CPP
#define LATTICE_PLANNER_ROS_CPP

namespace lattice_planner {

class LatticePlannerROS : public nav_core::BaseGlobalPlanner {
public:

	LatticePlannerROS();
	LatticePlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	/** overridden classes from interface nav_core::BaseGlobalPlanner **/
	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	bool makePlan(const geometry_msgs::PoseStamped& start,
	            const geometry_msgs::PoseStamped& goal,
	            std::vector<geometry_msgs::PoseStamped>& plan);


protected:
    /**
     * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
     */
    costmap_2d::Costmap2D* costmap_;
    std::string frame_id_;
    ros::Publisher plan_pub_;
    bool initialized_, allow_unknown_;

};

};
#endif