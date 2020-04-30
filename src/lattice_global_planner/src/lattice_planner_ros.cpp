#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include "lattice_planner_ros.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(lattice_planner::LatticePlannerROS, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace lattice_planner 
{

LatticePlannerROS::LatticePlannerROS (): costmap_(NULL), initialized_(false) {
}

LatticePlannerROS::LatticePlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros): costmap_(NULL), 
    initialized_(false){
    
    initialize(name, costmap_ros);
}


void LatticePlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void LatticePlannerROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id){

    if (!initialized_) {

        costmap_ = costmap;
        frame_id_ = frame_id;
        initialized_ = true;

    } 
    else
        ROS_INFO_STREAM("This planner has already been initialized, you can't call it twice, doing nothing");
}

bool LatticePlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  
    std::vector<geometry_msgs::PoseStamped>& plan ){

    // plan.push_back(start);
    // for (int i=0; i<20; i++){

    //     geometry_msgs::PoseStamped new_goal = goal;
    //     tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

    //     new_goal.pose.position.x = -2.5+(0.05*i);
    //     new_goal.pose.position.y = -3.5+(0.05*i);

    //     new_goal.pose.orientation.x = goal_quat.x();
    //     new_goal.pose.orientation.y = goal_quat.y();
    //     new_goal.pose.orientation.z = goal_quat.z();
    //     new_goal.pose.orientation.w = goal_quat.w();

    //     plan.push_back(new_goal);
    // }

    // plan.push_back(goal);
    // return true;

    if (!initialized_) {
        ROS_INFO_STREAM("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
        ROS_INFO("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str() );
        return false;
    }

    if (start.header.frame_id != global_frame) {
        ROS_INFO("start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    // fill out start and goal coords
    Coord coordsinit, goalCoord;

    // get orientation
    tf::Quaternion q1(
    start.pose.orientation.x,
    start.pose.orientation.y,
    start.pose.orientation.z,
    start.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);

    coordsinit.x = start.pose.position.x; coordsinit.y = start.pose.position.y; coordsinit.theta = yaw;

    // get orientation
    tf::Quaternion q2(
    goal.pose.orientation.x,
    goal.pose.orientation.y,
    goal.pose.orientation.z,
    goal.pose.orientation.w);
    tf::Matrix3x3 m2(q2);
    m2.getRPY(roll, pitch, yaw);    

    goalCoord.x = goal.pose.position.x; goalCoord.y = goal.pose.position.y; goalCoord.theta = yaw;

    //shift to discrete
    const CoordDisc coordsInitDisc( ContXY2Disc(coordsinit.x, graph_dx), ContXY2Disc(coordsinit.y, graph_dy),
        ContTheta2Disc(coordsinit.theta, numAngles) );
    const CoordDisc coordsGoalDisc( ContXY2Disc(goalCoord.x, graph_dx), ContXY2Disc(goalCoord.y, graph_dy),
        ContTheta2Disc(goalCoord.theta, numAngles) );

    // read in motion primitives file
    const char* sMotPrimFile = "/home/lord-pradhan/auto_valet/src/lattice_global_planner/include/lattice_global_planner/prim_test.mprim";
    MPrimFile readFile;

    if (sMotPrimFile != NULL) {
        FILE* fMotPrim = fopen(sMotPrimFile, "r");
        if (fMotPrim == NULL) {
            std::stringstream ss;
            ss << "ERROR: unable to open " << sMotPrimFile;
            ROS_INFO("unable to open \n"); 
        }

        if (ReadMotionPrimitives(fMotPrim, readFile) == false) {
            ROS_INFO("ERROR: failed to read in motion primitive file\n");
            // return;
        }
        fclose(fMotPrim);
    }

    auto start1 = high_resolution_clock::now();

    //initialise vars
    int elemCt = 0;
    int x_size = costmap_->getSizeInCellsX();//(int) x_ul/graph_dx +1;
    int y_size = costmap_->getSizeInCellsY();//(int) y_ul/graph_dy +1;

    int dX[numOfDirs] = { -1, -1, -1,  0,  0,  1, 1, 1};
    int dY[numOfDirs] = { -1,  0,  1, -1,  1, -1, 0, 1}; 
    double cost[numOfDirs] = {1.4, 1, 1.4, 1, 1, 1.4, 1, 1.4};

    std::vector<State> fullGraph;

    State_pre state_init_pre;
    std::vector< std::vector<State_pre> > gridmap_pre(y_size, std::vector<State_pre>(x_size, 
                                                state_init_pre) );

    // initialize map
    for (int i =0; i<y_size; i++){
        for (int j=0; j<x_size; j++){

            gridmap_pre[i][j].coords.x = j;
            gridmap_pre[i][j].coords.y = i;
        }
    }

    // Dijkstra backwards expansion for heuristics
    gridmap_pre[coordsGoalDisc.y][coordsGoalDisc.x].G_val = 0.0;

    std::priority_queue< State_pre, std::vector<State_pre>, CompareF_pre > open_set_pre;
    open_set_pre.push(gridmap_pre[coordsGoalDisc.y][coordsGoalDisc.x]);

    while( !open_set_pre.empty() ){

        State_pre temp_pre = open_set_pre.top();
        int x_temp = temp_pre.coords.x, y_temp = temp_pre.coords.y;
        double G_temp = temp_pre.G_val;
        gridmap_pre[y_temp][x_temp].expanded = true;

        open_set_pre.pop();
        // lookUpHEnv[GetIndex(coordTemp)] = temp_pre.G_val;

        for(int dir =0; dir<numOfDirs; dir++){

            int newx = x_temp + dX[dir], newy = y_temp + dY[dir];
            CoordDisc coordsNewTemp(newx, newy, 0);
            double newx_cont = DiscXY2Cont(newx, graph_dx);
            double newy_cont = DiscXY2Cont(newy, graph_dy);

            unsigned int mx_new, my_new;
            if( costmap_->worldToMap(newx_cont, newy_cont, mx_new, my_new) && (gridmap_pre[newy][newx].expanded==false) ){ 

                if( costmap_->worldToMap(newx_cont, newy_cont, mx_new, my_new) < collision_thresh &&
                    gridmap_pre[newy][newx].G_val > G_temp + cost[dir] + (double)(costmap_->getCost(mx_new, my_new)) ){

                    gridmap_pre[newy][newx].G_val = G_temp + cost[dir] + (double)(costmap_->getCost(mx_new, my_new));
                    open_set_pre.push(gridmap_pre[newy][newx]);
                }
            }
        }
    }

    ROS_INFO("Dijkstra expansion done");

    auto stop1 = high_resolution_clock::now();
    auto duration1 = duration_cast<milliseconds>(stop1 - start1);
    ROS_INFO_STREAM("Time taken for Dijkstra is "<< duration1.count());

    // initial conditions
    State state_init(coordsInitDisc);
    state_init.setH(coordsGoalDisc, gridmap_pre[coordsInitDisc.y][coordsInitDisc.x].G_val);
    state_init.setID(elemCt);
    fullGraph.push_back(state_init);
    elemCt++;

    // initiate graph search using A* //
    fullGraph[0].setG(0.0);
    int finID;

    std::priority_queue< State, vector<State>, CompareF> open_set;
    open_set.push(fullGraph[0]);

    std::unordered_set<unsigned long long int> closed_set;
    // lookUpG[GetIndex(fullGraph[0].getCoords())] = 0.0;

    while( !open_set.empty() && !( open_set.top().getCoords() == coordsGoalDisc ) ){

        // generate primitives
        State temp = open_set.top();
        int tempID = open_set.top().getID();
        closed_set.insert(GetIndex(temp.getCoords()));
        open_set.pop();

        // extend implicit graph
        for(auto i : readFile.inputPrims){

            if( temp.getCoords().theta== i.startAngleDisc ){

                CoordDisc tempPose;
                tempPose.x = temp.getCoords().x + i.endPose.x;
                tempPose.y = temp.getCoords().y + i.endPose.y;
                tempPose.theta = i.endPose.theta;
                ROS_INFO("tempPose is %d %d %d", tempPose.x, tempPose.y, tempPose.theta);

                unsigned int new_mx, new_my;
                double newx_cont = DiscXY2Cont(tempPose.x, graph_dx);
                double newy_cont = DiscXY2Cont(tempPose.y, graph_dy);

                bool free = costmap_->worldToMap(newx_cont, newy_cont, new_mx, new_my);

                if(free){

                    if( (double) costmap_->getCost(new_mx, new_my) < collision_thresh &&
                    (closed_set.find(GetIndex(tempPose))==closed_set.end()) ){

                        State newState(tempPose);
                        newState.setH(coordsGoalDisc, gridmap_pre[tempPose.y][tempPose.x].G_val);
                        ROS_INFO_STREAM("H-value for inserted state is "<< newState.getH() );
                        newState.setID(elemCt);

                        GraphEdge exToNew, newToEx;

                        newToEx.ID = tempID;
                        newToEx.cost = i.cost;
                        // printf("newToEx.cost is %lf \n", i.cost);
                        newState.addAdjElem(newToEx);

                        exToNew.ID = elemCt;
                        exToNew.cost= i.cost;
                        fullGraph[tempID].addAdjElem(exToNew);

                        fullGraph.push_back(newState);
                        elemCt++;

                        fullGraph[elemCt-1].setG( fullGraph[tempID].getG() + i.cost + (double) costmap_->getCost(new_mx, new_my) );
                        ROS_INFO_STREAM("G-value for inserted state is "<< fullGraph[elemCt-1].getG() );
                        open_set.push(fullGraph[elemCt-1]);                 
                    }
                }
            }           
        }

        if(open_set.top().getCoords() == coordsGoalDisc){
            ROS_INFO("\nTarget expanded\n");
        }
    }

    std::stack <State> optPath;

    if(!open_set.empty()){

        finID = open_set.top().getID();
        optPath.push(fullGraph[finID]);
        int optID;

        while( !(optPath.top().getCoords() == coordsInitDisc) ){

            double min_G = numeric_limits<double>::infinity();

            for(auto i_edge : optPath.top().getAdjElems()){

                if(min_G > fullGraph[i_edge.ID].getG() + i_edge.cost){

                    min_G = fullGraph[i_edge.ID].getG() + i_edge.cost;
                    optID = i_edge.ID;
                }
            }
            optPath.push( fullGraph[optID] );
        }
        ROS_INFO("Path found! \n");
    }
    else{

        ROS_INFO("Open set is empty, path can't be found. Exiting\n");
    }

    optPath.push(fullGraph[0]);

    auto headerStampInsert = ros::Time::now();
    while(!optPath.empty()){

        auto tempState = optPath.top();

        geometry_msgs::PoseStamped tempInsert = start;
        tempInsert.pose.position.x = DiscXY2Cont(tempState.getCoords().x, graph_dx);
        tempInsert.pose.position.y = DiscXY2Cont(tempState.getCoords().y, graph_dy);

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY( 0, 0, DiscTheta2Cont(tempState.getCoords().theta, numAngles) );
        myQuaternion.normalize();        

        tempInsert.pose.orientation = tf2::toMsg(myQuaternion);
        // tempInsert.pose.orientation.w = myQuaternion.w;
        tempInsert.header.stamp = headerStampInsert;

        plan.push_back(tempInsert);

        optPath.pop();
    }

    // std::reverse(plan.begin(), plan.end());

    if(!plan.empty()){
        return true;
    }
    else{
        return false;
    }
}

}