/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <iostream>
#include <math.h>
// #include <mex.h>
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
using namespace std::chrono;
using namespace std;
#include "./matplotlib-cpp/matplotlibcpp.h"
#include <cmath>

namespace plt = matplotlibcpp;


#define  NORMALIZEDISCTHETA(THETA, THETADIRS) (((THETA>=0)?((THETA)%(THETADIRS)):(((THETA)%(THETADIRS)+THETADIRS)%THETADIRS)))
#define 	DISCXY2CONT(X, CELLSIZE)   ((X)*(CELLSIZE) + (CELLSIZE)/2.0)
#define CONTXY2DISC(X, CELLSIZE) (((X)>=0)?((int)((X)/(CELLSIZE))):((int)((X)/(CELLSIZE))-1))


// #define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

// #if !defined(MAX)
// #define	MAX(A, B)	((A) > (B) ? (A) : (B))
// #endif

// #if !defined(MIN)
// #define	MIN(A, B)	((A) < (B) ? (A) : (B))
// #endif

// // // define global variables
// int function_call =0;
// // stack <State> finalOptPath;
// // int final_del_t;
// // int final_traj_index;


static std::stack<State> planner(const Coord& coordsinit, const Coord& goalCoord, const char* sMotPrimFile)
{
	//shift to discrete
	const CoordDisc coordsInitDisc( CONTXY2DISC(coordsinit.x, graph_dx), CONTXY2DISC(coordsinit.y, graph_dy),
	ContTheta2Disc(coordsinit.theta, numAngles) );
	const CoordDisc coordsGoalDisc( CONTXY2DISC(goalCoord.x, graph_dx), CONTXY2DISC(goalCoord.y, graph_dy),
	ContTheta2Disc(goalCoord.theta, numAngles) );

	// read in motion primitives file
	MPrimFile readFile;

	if (sMotPrimFile != NULL) {
        FILE* fMotPrim = fopen(sMotPrimFile, "r");
        if (fMotPrim == NULL) {
            // std::stringstream ss;
            // ss << "ERROR: unable to open " << sMotPrimFile;
            printf("unable to open \n"); 
        }

        if (ReadMotionPrimitives(fMotPrim, readFile) == false) {
            printf("ERROR: failed to read in motion primitive file");
        }
        fclose(fMotPrim);
    }

	//initialise vars
	int elemCt = 0;
	std::vector<State> fullGraph;

	// initial conditions
	State state_init(coordsInitDisc);
	state_init.setID(elemCt);
	fullGraph.push_back(state_init);
	elemCt++;


	// initiate graph search using Dijkstraa's
	fullGraph[0].setG(0.0);
	int finID;

	std::priority_queue< State, vector<State>, CompareF_pre> open_set;
	open_set.push(fullGraph[0]);

	while( !open_set.empty() && !( open_set.top().getCoords() == coordsGoalDisc ) ){

		// generate primitives
		State temp = open_set.top();
		int tempID = open_set.top().getID();
		fullGraph[tempID].expand();

		open_set.pop();

		// extend implicit graph
		for(auto i : readFile.inputPrims){

			if( temp.getCoords().theta== i.startAngleDisc ){

				CoordDisc tempPose;
				tempPose.x = temp.getCoords().x + i.endPose.x;
				tempPose.y = temp.getCoords().y + i.endPose.y;
				// angle wrap
				// tempPose.theta = ContTheta2Disc( wrap2pi( DiscTheta2Cont( tempPose.theta, numAngles ) ),
				// 					 numAngles );
				tempPose.theta = i.endPose.theta;

				if(freeState(tempPose)){

					State newState(tempPose);
					newState.setID(elemCt);

					GraphEdge exToNew, newToEx;

					newToEx.ID = tempID;
					newToEx.cost = i.cost;
					newState.addAdjElem(newToEx);

					exToNew.ID = elemCt;
					exToNew.cost= i.cost;
					fullGraph[tempID].addAdjElem(exToNew);

					fullGraph.push_back(newState);
					elemCt++;
				}
			}			
		}

		// std::vector<Primitive> nextStates = getNextStates( temp.getCoords() );

		// for (auto i_primitive : nextStates){

		// 	if( freeState(i_primitive.coord_final) ){

		// 		State pushState(i_primitive.coord_final);
		// 		pushState.setID( elemCt );

		// 		GraphEdge existing, push; 

		// 		existing.ID = tempID; 
		// 		existing.cost = i_primitive.cost;
		// 		pushState.addAdjElem(existing);

		// 		push.ID = elemCt;
		// 		push.cost = i_primitive.cost;
		// 		fullGraph[tempID].addAdjElem(push);

		// 		fullGraph.push_back(pushState);
		// 		elemCt++;
		// 	}
		// }

		for(auto edge : fullGraph[tempID].getAdjElems()){

			if( fullGraph[edge.ID].getG() > fullGraph[tempID].getG() + edge.cost ){

				fullGraph[edge.ID].setG( fullGraph[tempID].getG() + edge.cost );
				open_set.push(fullGraph[edge.ID]);
			}
		}

		if(open_set.top().getCoords() == coordsGoalDisc){
			printf("target expanded\n");
		}
	}

	if(!open_set.empty()){

		finID = open_set.top().getID();
	}

	std::stack <State> optPath;
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
	printf("optPath.size() is %d \n", optPath.size());
    return optPath;
}


int main(int argc, char* argv[]){

	const Coord coordsinit(0.0, 0.0, 0.0);
	const Coord goalCoord(20.0, 0, 0);

	std::stack <State> optPath = planner( coordsinit, goalCoord, "prim_test.mprim" );
	printf("optPath.size() is %d \n", optPath.size());

	int pathSize = optPath.size();

	std::vector<double> x(optPath.size()), y(optPath.size());//, z(n), w(n,2);
	for(int i=0; i<pathSize;i++){

		x.at(i) = optPath.top().getX();
		y.at(i) = optPath.top().getY();
		printf("x.at(i) is %lf\n y.at(i) is %lf \n", x.at(i), y.at(i));

		optPath.pop();
	}

	// Set the size of output image to 1200x780 pixels
    plt::figure_size(1200, 780);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(x, y);

    plt::xlim(0, 30);

    plt::show();
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// // 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
// void mexFunction( int nlhs, mxArray *plhs[],
//         int nrhs, const mxArray*prhs[] )
        
// {
    
//     /* Check for proper number of arguments */
//     if (nrhs != 6) {
//         mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
//                 "Six input arguments required.");
//     } else if (nlhs != 1) {
//         mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
//                 "One output argument required.");
//     }
    
//     /* get the dimensions of the map and the map matrix itself*/
//     int x_size = mxGetM(MAP_IN);
//     int y_size = mxGetN(MAP_IN);
//     double* map = mxGetPr(MAP_IN);
    
//     /* get the dimensions of the robotpose and the robotpose itself*/
//     int robotpose_M = mxGetM(ROBOT_IN);
//     int robotpose_N = mxGetN(ROBOT_IN);
//     if(robotpose_M != 1 || robotpose_N != 2){
//         mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
//                 "robotpose vector should be 1 by 2.");
//     }
//     double* robotposeV = mxGetPr(ROBOT_IN);
//     int robotposeX = (int)robotposeV[0];
//     int robotposeY = (int)robotposeV[1];
    
//     /* get the dimensions of the goalpose and the goalpose itself*/
//     int targettraj_M = mxGetM(TARGET_TRAJ);
//     int targettraj_N = mxGetN(TARGET_TRAJ);
    
//     if(targettraj_M < 1 || targettraj_N != 2)
//     {
//         mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
//                 "targettraj vector should be M by 2.");
//     }
//     double* targettrajV = mxGetPr(TARGET_TRAJ);
//     int target_steps = targettraj_M;
    
//     /* get the current position of the target*/
//     int targetpose_M = mxGetM(TARGET_POS);
//     int targetpose_N = mxGetN(TARGET_POS);
//     if(targetpose_M != 1 || targetpose_N != 2){
//         mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
//                 "targetpose vector should be 1 by 2.");
//     }
//     double* targetposeV = mxGetPr(TARGET_POS);
//     int targetposeX = (int)targetposeV[0];
//     int targetposeY = (int)targetposeV[1];
    
//     /* get the current timestep the target is at*/
//     int curr_time = mxGetScalar(CURR_TIME);
    
//     /* Create a matrix for the return action */ 
//     ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
//     double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
//     /* Get collision threshold for problem */
//     int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
//     /* Do the actual planning in a subroutine */
//     planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
//     // printf("DONE PLANNING!\n");
//     return;   
// }