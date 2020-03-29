/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <iostream>
#include <math.h>
#include <mex.h>
#include <array>
#include <queue>
#include <vector>
#include <list>
#include <iterator>
#include <limits>
#include <bits/stdc++.h> 
#include <algorithm> 
#include <chrono> 
using namespace std::chrono;

using namespace std;

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


// define required classes

// class TrajPoint{

// private:
// 	int traj_id;
// 	double g_value;

// public:
// 	int getTrajID() const{return traj_id;}
// 	double getGVal() const{return g_value;}

// 	void setTrajID(int traj_id_){traj_id = traj_id_;}
// 	void setGVal(double g_value_){g_value = g_value_;}
// };	


// define global variables
int function_call =0;
stack <State> finalOptPath;
int final_del_t;
int final_traj_index;


static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
	//initialise vars
	int elemCt = 0;
	std::vector<State> fullGraph;

	// initial conditions
	Coord coordsinit(0.0, 0.0, 0.0);
	State state_init(coordsinit);
	state_init.setID(elemCt);
	fullGraph.push_back(state_init);
	elemCt++;

	// initiate graph search using Dijkstraa's
	fullGraph[0].setG(0.0);

	std::priority_queue< State, vector<State>, CompareF_pre> open_set;
	open_set.push(fullGraph[0]);

	while(!open_set.empty()  ){

		// generate primitives
		State temp = open_set.top();
		int tempID = open_set.top().getID();
		fullGraph[tempID].expand();

		open_set.pop();

		// extend implicit graph
		std::vector<MotionPrimitive> nextStates = getNextStates( temp.coords );

		for (auto i_primitive : nextStates){

			if( freeState(i_primitive.coords) ){

				State pushState(i_primitive.coords);
				pushState.setID( elemCt );
				pushState.addAdjID(tempID);
				fullGraph[tempID].addAdjID(elemCt);
				fullGraph.push_back(pushState);
				elemCt++;
			}
		}

		for(auto id : temp.getAdjIDs()){

			if( fullGraph[id].getG() > temp.getG() + cost )
		}
	}

	// auto start = high_resolution_clock::now();
	// function_call++;

	// // ********** New Planner *********

	// if (function_call==1){

	//     // 9-connected grid
	//     int dX[NUMOFDIRS] = { -1, -1, -1,  0,  0,  1, 1, 1};
	//     int dY[NUMOFDIRS] = { -1,  0,  1, -1,  1, -1, 0, 1}; 


	// 	// Run Dijsktraa's first
	//     State state_init;
	// 	vector<vector<State> > grid_map(y_size, vector<State>(x_size, state_init));

	// 	// initialize start state
	// 	grid_map[robotposeY-1][ robotposeX - 1 ].setG(0.0);

	// 	// initialize map
	// 	for (int i =0; i<y_size; i++){
	// 		for (int j=0; j<x_size; j++){

	// 			grid_map[i][j].setX(j+1);
	// 			grid_map[i][j].setY(i+1);
	// 		}
	// 	}

	// 	// initialize open list
	// 	priority_queue <State, vector<State>, CompareF_pre> open_set_pre;
	// 	open_set_pre.push( grid_map[robotposeY-1][robotposeX-1] );


	// 	// start while loop for Dijkstraa expansion, update all G values
	// 	while( !open_set_pre.empty() ){

	// 		State temp = open_set_pre.top();
	// 		int x_temp = temp.getX();
	// 		int y_temp = temp.getY();
	// 		double g_temp = temp.getG();
	// 		grid_map[y_temp-1][x_temp-1].expand();

	// 		// remove smallest S from open
	// 		open_set_pre.pop();

	// 		// // span through successors at next time step

	// 		for(int dir = 0; dir < NUMOFDIRS; dir++){

	// 	        int newx = x_temp + dX[dir];
	// 	        int newy = y_temp + dY[dir];

	// 	        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
	// 	        {
	// 	            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && 
	// 	            ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) && 
	// 	            (!grid_map[newy-1][newx-1].getExpanded()) )  //if free
	// 	            {

	// 	            	if( grid_map[newy-1][newx-1].getG() > g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)] ){

	// 						grid_map[newy-1][newx-1].setG(g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]);
	// 						open_set_pre.push(grid_map[newy-1][newx-1]);
	// 	            	}
	// 	            }
	// 	        }
	// 	    }
	// 	}

	// 	mexPrintf("Dijkstraa expansion done \n");

	// 	// look for minimum cost point on target_traj
	// 	double g_path = numeric_limits<double>::infinity();
	// 	cout<<"target steps "<<target_steps<<endl;

	// 	//clear optPath
	// 	while(!finalOptPath.empty()){finalOptPath.pop();}

	// 	for (int i=1; i<target_steps; i++){

	// 		stack <State> optPath;

	// 		// push point on traj
	// 		optPath.push(grid_map[  target_traj[i+target_steps] - 1 ][ target_traj[i] - 1]);

	// 		while( optPath.top().getX() != grid_map[robotposeY-1][robotposeX-1].getX() || optPath.top().getY() 
	// 			!= grid_map[robotposeY-1][robotposeX-1].getY() ){

	// 			double min_G = numeric_limits<double>::infinity(); 
	// 			int finX, finY;

	// 			for(int dir1 = 0; dir1 < NUMOFDIRS; dir1++){

	// 		        int newx = optPath.top().getX() + dX[dir1];
	// 		        int newy = optPath.top().getY() + dY[dir1];

	// 		        if (newx >= 1 && newx <= x_size && newy >= 1 && 
	// 		        	newy <= y_size && min_G > grid_map[newy-1][newx-1].getG() ){

	// 					min_G = grid_map[newy-1][newx-1].getG();
	// 					// cout<<"min_g is "<<min_G<<endl;
	// 					finX = newx; finY = newy;
	// 		        }
	// 		    }
			   
	// 		    optPath.push(grid_map[finY-1][finX-1]);
	// 		}

	// 		// time robot has to wait at final spot
	// 		int del_t = -( curr_time + optPath.size() - i );
	// 		int x_end = target_traj[i]; int y_end = target_traj[i+target_steps];

	// 		if ( optPath.size() >0 && del_t >= 0 && (g_path > grid_map[ y_end - 1 ][ x_end - 1 ].getG()
	// 							+ del_t * (int)map[GETMAPINDEX(x_end,y_end,x_size,y_size)] ) ){

	// 			g_path = grid_map[ y_end - 1 ][ x_end - 1 ].getG()
	// 							+ del_t * (int)map[GETMAPINDEX(x_end,y_end,x_size,y_size)];

	// 			finalOptPath = optPath;
	// 			final_del_t = del_t;
	// 			final_traj_index = i;
	// 		}
	// 	}

	// cout<<"size of path "<< finalOptPath.size() <<endl;
	// cout<<"cost of path "<< g_path <<endl;
	// auto stop = high_resolution_clock::now();
	// auto duration = duration_cast<microseconds>(stop - start);
	// mexPrintf("del_t is %d \n", final_del_t);
	// mexPrintf("Final traj index is %d \n", final_traj_index);
	// mexPrintf("Duration is %d \n", duration.count());
	// }

	// int newposeX, newposeY;

	// // if reached final point
	// if ( (robotposeX == target_traj[final_traj_index] && robotposeY == target_traj[final_traj_index+target_steps]) ||
	// 	finalOptPath.size()<=1 ){

	// 	newposeX = robotposeX;
	// 	newposeY = robotposeY;
	// }
	// else{ // else keep moving

	// 	finalOptPath.pop();
	// 	newposeX = finalOptPath.top().getX();
	// 	newposeY = finalOptPath.top().getY();
	// }


	// mexPrintf("Commanded pose is %d %d \n", newposeX, newposeY);
 //    action_ptr[0] = newposeX;
 //    action_ptr[1] = newposeY;

    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}