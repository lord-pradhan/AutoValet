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


// useful functions related to costmap
bool LatticePlannerROS::world2Cont(float x_world, float y_world, float& x_cont, float& y_cont)
{
    if(x_world >= originX || x_world <= resolution*width + originX ||
        y_world >= originY || y_world <= resolution*height + originY ){

        x_cont = x_world - originX;
        y_cont = y_world - originY;
        return true;    
    }
    else{
        ROS_ERROR("out of map position chosen");
        return false;
    }
    
}

void LatticePlannerROS::cont2World(float x_cont, float y_cont, float& x_world, float& y_world)
{
    x_world = x_cont + originX;
    y_world = y_cont + originY;
}


void LatticePlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void LatticePlannerROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id){

    if (!initialized_) {

        costmap_ = costmap;
        frame_id_ = frame_id;
        initialized_ = true;

        // costmap_ros_ = costmap_ros;
        // costmap_ = costmap_ros_->getCostmap();

        // ros::NodeHandle private_nh("~/" + name);

        originX = costmap_->getOriginX();
        originY = costmap_->getOriginY();

        width = costmap_->getSizeInCellsX();
        height = costmap_->getSizeInCellsY();
        resolution = costmap_->getResolution();
        // mapSize = width*height;
        // tBreak = 1+1/(mapSize); 
        // value =0;

        // read in motion primitives file
        ROS_INFO("Reading in motion primitives\n");
        const char* sMotPrimFile = "/home/lord-pradhan/auto_valet/src/lattice_global_planner/include/lattice_global_planner/prim_test.mprim";    

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

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
        goal.pose.position.x, goal.pose.position.y);

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str() );
        return false;
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR("start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    // fill out start and goal coords in continuous frame
    Coord coordsinit, goalCoord;
    float x_start_cont, y_start_cont, x_goal_cont, y_goal_cont;

    // get orientation
    tf::Quaternion q1(
    start.pose.orientation.x,
    start.pose.orientation.y,
    start.pose.orientation.z,
    start.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);

    float x_start_world = start.pose.position.x, y_start_world = start.pose.position.y;
    bool pick1 = world2Cont(x_start_world, y_start_world, x_start_cont, y_start_cont);
    coordsinit.x = x_start_cont; coordsinit.y = y_start_cont; coordsinit.theta = yaw;    

    // get orientation
    tf::Quaternion q2(
    goal.pose.orientation.x,
    goal.pose.orientation.y,
    goal.pose.orientation.z,
    goal.pose.orientation.w);
    tf::Matrix3x3 m2(q2);
    m2.getRPY(roll, pitch, yaw);    

    float x_goal_world = goal.pose.position.x, y_goal_world = goal.pose.position.y;
    bool pick2 = world2Cont(x_goal_world, y_goal_world, x_goal_cont, y_goal_cont);
    goalCoord.x = x_goal_world; goalCoord.y = y_goal_world; goalCoord.theta = yaw;

    if(!pick1 || !pick2){
        return false;
    }

    //shift to discrete
    const CoordDisc coordsInitDisc( ContXY2Disc(coordsinit.x, graph_dx), ContXY2Disc(coordsinit.y, graph_dy),
        ContTheta2Disc(coordsinit.theta, numAngles) );
    const CoordDisc coordsGoalDisc( ContXY2Disc(goalCoord.x, graph_dx), ContXY2Disc(goalCoord.y, graph_dy),
        ContTheta2Disc(goalCoord.theta, numAngles) );
    

    auto start1 = high_resolution_clock::now();

    //initialise vars
    int elemCt = 0;
    int x_size = width;//(int) x_ul/graph_dx +1;
    int y_size = height;//(int) y_ul/graph_dy +1;

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

            // these coords are in discrete frame
            gridmap_pre[i][j].coords.x = j; 
            gridmap_pre[i][j].coords.y = i;
        }
    }

    // Dijkstra backwards expansion for heuristics
    gridmap_pre[coordsGoalDisc.y][coordsGoalDisc.x].G_val = 0.0;

    std::priority_queue< State_pre, std::vector<State_pre>, CompareF_pre > open_set_pre;
    open_set_pre.push(gridmap_pre[coordsGoalDisc.y][coordsGoalDisc.x]);

    ROS_INFO("Before entering Dijkstra expansion while loop");

    while( !open_set_pre.empty() ){

        State_pre temp_pre = open_set_pre.top();

        // these are in discrete
        int x_temp_disc = temp_pre.coords.x, y_temp_disc = temp_pre.coords.y;
        double G_temp = temp_pre.G_val;
        gridmap_pre[y_temp_disc][x_temp_disc].expanded = true;

        open_set_pre.pop();
        // lookUpHEnv[GetIndex(coordTemp)] = temp_pre.G_val;

        for(int dir =0; dir<numOfDirs; dir++){

            int newx_disc = x_temp_disc + dX[dir], newy_disc = y_temp_disc + dY[dir];
            // CoordDisc coordsNewTemp(newx_disc, newy_disc, 0);
            double newx_cont = DiscXY2Cont(newx_disc, graph_dx);
            double newy_cont = DiscXY2Cont(newy_disc, graph_dy);

            // ROS_INFO("newx_cont is %lf", newx_cont);
            // ROS_INFO("newy_cont is %lf", newy_cont);

            float newx_world, newy_world;
            cont2World(newx_cont, newy_cont, newx_world, newy_world);

            unsigned int x_new_cmap, y_new_cmap; // costmap frame

            if( costmap_->worldToMap(newx_world, newy_world, x_new_cmap, y_new_cmap) && 
                (gridmap_pre[newy_disc][newx_disc].expanded==false) ){ 

                if( (double) (costmap_->getCost(x_new_cmap, y_new_cmap)) < collision_thresh &&
                    gridmap_pre[newy_disc][newx_disc].G_val > G_temp + cost[dir] + (double)(costmap_->getCost(x_new_cmap, y_new_cmap)) ){
                    
                    gridmap_pre[newy_disc][newx_disc].G_val = G_temp + cost[dir] + (double)(costmap_->getCost(x_new_cmap, y_new_cmap));
                    open_set_pre.push(gridmap_pre[newy_disc][newx_disc]);
                    // ROS_INFO("pushed H-val during Dijkstra is %lf", gridmap_pre[newy_disc][newx_disc].G_val);
                }
                else if((double) (costmap_->getCost(x_new_cmap, y_new_cmap)) > collision_thresh) {
                    // ROS_INFO("Encountered obstacle");
                    // printf("cost of new cell is %lf\n", (double) costmap_->getCost(x_new_cmap, y_new_cmap) );
                }
            }
            else{
                // ROS_INFO("Out-of-map position or previously expanded state reached");
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

        // all in discrete frame
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
                // ROS_INFO("tempPose in disc frame is %d %d %d", tempPose.x, tempPose.y, tempPose.theta);
                
                double newx_cont = DiscXY2Cont(tempPose.x, graph_dx);
                double newy_cont = DiscXY2Cont(tempPose.y, graph_dy);

                // ROS_INFO("newx_cont is %lf", newx_cont);
                // ROS_INFO("newy_cont is %lf", newy_cont);

                float newx_world, newy_world;
                cont2World(newx_cont, newy_cont, newx_world, newy_world);

                unsigned int new_x_cmap, new_y_cmap;

                bool free = costmap_->worldToMap(newx_world, newy_world, new_x_cmap, new_y_cmap);

                if(free){

                    if( (double) costmap_->getCost(new_x_cmap, new_y_cmap) < collision_thresh &&
                        (closed_set.find(GetIndex(tempPose))==closed_set.end()) ){

                        State newState(tempPose);
                        newState.setH(coordsGoalDisc, gridmap_pre[tempPose.y][tempPose.x].G_val);
                        // ROS_INFO_STREAM("H-value for inserted state is "<< newState.getH() );
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

                        fullGraph[elemCt-1].setG( fullGraph[tempID].getG() + i.cost + 
                                                    (double) costmap_->getCost(new_x_cmap, new_y_cmap) );
                        // ROS_INFO_STREAM("G-value for inserted state is "<< fullGraph[elemCt-1].getG() );
                        open_set.push(fullGraph[elemCt-1]);                 
                    }
                    else{
                        // ROS_INFO("Obstacle or previously expanded state reached");
                    }
                }
                else{
                    // ROS_INFO("Out of map position reached");
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

        geometry_msgs::PoseStamped tempInsert = start; // world frame

        float x_insert_cont, y_insert_cont, x_insert_world, y_insert_world;

        x_insert_cont = DiscXY2Cont(tempState.getCoords().x, graph_dx);
        y_insert_cont = DiscXY2Cont(tempState.getCoords().y, graph_dy);

        cont2World(x_insert_cont, y_insert_cont, x_insert_world, y_insert_world);

        tempInsert.pose.position.x = x_insert_world;
        tempInsert.pose.position.y = y_insert_world;

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