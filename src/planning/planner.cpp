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
#include "dubins.h"

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
	const CoordDisc coordsInitDisc( ContXY2Disc(coordsinit.x, graph_dx), ContXY2Disc(coordsinit.y, graph_dy),
	ContTheta2Disc(coordsinit.theta, numAngles) );
	const CoordDisc coordsGoalDisc( ContXY2Disc(goalCoord.x, graph_dx), ContXY2Disc(goalCoord.y, graph_dy),
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
            // return;
        }
        fclose(fMotPrim);
    }

	//initialise vars
	int elemCt = 0;
	std::vector<State> fullGraph;

	// initial conditions
	State state_init(coordsInitDisc);
	state_init.setH(coordsGoalDisc);
	state_init.setID(elemCt);
	fullGraph.push_back(state_init);
	elemCt++;

	// initiate graph search using A* //
	fullGraph[0].setG(0.0);
	int finID;

	std::priority_queue< State, vector<State>, CompareF_pre> open_set;
	open_set.push(fullGraph[0]);

	std::unordered_map<int, double> lookUpG;
	lookUpG[GetIndex(fullGraph[0].getCoords())] = 0.0;

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
				// printf("tempPose is %d %d %d\n", tempPose.x, tempPose.y, tempPose.theta);

				if(freeState(tempPose) ){// && (lookUpState.find(GetIndex(tempPose)) == lookUpState.end()) ){

					if( (lookUpG.find(GetIndex(tempPose)) == lookUpG.end())  || 
						( lookUpG[GetIndex(tempPose)] > fullGraph[tempID].getG() + i.cost ) ){

						State newState(tempPose);
						newState.setH(coordsGoalDisc);
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

						fullGraph[elemCt-1].setG( fullGraph[tempID].getG() + i.cost );
						open_set.push(fullGraph[elemCt-1]);
						fullGraph[elemCt-1].expand();
						lookUpG[GetIndex(fullGraph[elemCt-1].getCoords())] = fullGraph[tempID].getG() + i.cost;
						// lookUpState.insert(GetIndex(newState.getCoords()));

						// if(fullGraph[elemCt-1].getCoords().theta!=0 && fullGraph[elemCt-1].getCoords().x>0){
						// 	printf("expanded state is %d %d %d\n", fullGraph[elemCt-1].getCoords().x, 
						// 		fullGraph[elemCt-1].getCoords().y, fullGraph[elemCt-1].getCoords().theta );
						// 	// printf("G-value during graph search is %lf\n", fullGraph[tempID].getG() + edge.cost);
						// }
					}
				}
			}			
		}

		// printf("size of fullGraph[tempID].getAdjElems() is %d\n", fullGraph[tempID].getAdjElems().size());
		// for(auto edge : fullGraph[tempID].getAdjElems()){

		// 	if( (fullGraph[edge.ID].getG() > fullGraph[tempID].getG() + edge.cost) ){ //&& 
		// 		// !(fullGraph[edge.ID].getExpanded() ) ){

		// 		fullGraph[edge.ID].setG( fullGraph[tempID].getG() + edge.cost );
		// 		open_set.push(fullGraph[edge.ID]);
		// 		fullGraph[edge.ID].expand();
		// 		lookUpG[GetIndex(fullGraph[edge.ID].getCoords())] = fullGraph[tempID].getG() + edge.cost;

		// 		if(fullGraph[edge.ID].getCoords().theta!=0 && fullGraph[edge.ID].getCoords().x>0){
		// 			printf("expanded state is %d %d %d\n", fullGraph[edge.ID].getCoords().x, 
		// 				fullGraph[edge.ID].getCoords().y, fullGraph[edge.ID].getCoords().theta );
		// 			// printf("G-value during graph search is %lf\n", fullGraph[tempID].getG() + edge.cost);
		// 		}
		// 	}
		// }

		if(open_set.top().getCoords() == coordsGoalDisc){
			printf("target expanded\n");
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
		printf("optPath.size() is %d \n", optPath.size());
	}
	else{

		printf("open set is empty, exiting\n");
	}

	return optPath;

}

int printConfiguration(double q[3], double x, std::vector<double>& x0, 
	std::vector<double>& y0) {

    x0.push_back(q[0]); y0.push_back(q[1]);
    printf("%f, %f, %f, %f\n", q[0], q[1], q[2], x);
    return 0;
}

int main(int argc, char* argv[]){

	const Coord coordsinit(2.0, 82.0, 0.0);
	const Coord goalCoord(38.0, 15, -PI/2); 

	std::vector<double> x0, y0;
	int printConfiguration(double q[3], double x, std::vector<double>& x0, 
		std::vector<double>& y0 );

	double q0[] = { coordsinit.x, coordsinit.y, coordsinit.theta };
    double q1[] = { goalCoord.x, goalCoord.y, goalCoord.theta };
    double turning_radius = 6.0;
    DubinsPath path;
    dubins_shortest_path( &path, q0, q1, turning_radius);

    double init_h = dubins_path_length( &path );
    dubins_path_sample_many( &path, 0.1, printConfiguration, x0, y0);

    // plt::plot(x0, y0);
    // plt::xlim(0, 400); plt::show();
	auto start = high_resolution_clock::now();
    std::stack <State> optPath = planner( coordsinit, goalCoord, "prim_test.mprim" );
    auto stop = high_resolution_clock::now();
	auto duration = duration_cast<milliseconds>(stop - start);
	printf("Duration is %d \n", duration.count());

	printf("optPath.size() is %d \n", optPath.size());

	printf("initial h_val is %lf\n", init_h);
	int pathSize = optPath.size();

	///// Plotting ///////
	std::vector<double> x, y, u, v;//, z(n), w(n,2);
	
	for(int i=0; i<pathSize;i++){

		x.push_back(DiscXY2Cont( optPath.top().getX() , graph_dx));
		y.push_back(DiscXY2Cont( optPath.top().getY(), graph_dy));
		u.push_back(cos( DiscTheta2Cont(optPath.top().getTheta(), numAngles ) ));
		v.push_back(sin( DiscTheta2Cont(optPath.top().getTheta(), numAngles ) ));
		// printf("x.at(i) is %lf\n y.at(i) is %lf \n", x.at(i), y.at(i));

		optPath.pop();
	}

	std::vector<double> x_obstacle, y_obstacle;
	x_obstacle.push_back(0); y_obstacle.push_back(55);
	x_obstacle.push_back(50); y_obstacle.push_back(55);
	x_obstacle.push_back(50); y_obstacle.push_back(60);
	x_obstacle.push_back(0); y_obstacle.push_back(60);

	std::vector<double> x_obstacle1, y_obstacle1;
	x_obstacle1.push_back(100); y_obstacle1.push_back(35);
	x_obstacle1.push_back(50); y_obstacle1.push_back(35);
	x_obstacle1.push_back(50); y_obstacle1.push_back(40);
	x_obstacle1.push_back(100); y_obstacle1.push_back(40);

	// Set the size of output image to 1200x780 pixels
    plt::figure_size(1200, 780);

    plt::plot(x_obstacle, y_obstacle,"r");
    plt::plot(x_obstacle1, y_obstacle1,"r");

    std::map<std::string, std::string> keywords;
    // keywords["scale"]=1;
    plt::plot(x, y);//, u, v);//, 'width', 0.05, 'length', 0.1);
    plt::title("Car navigating a dummy parking-lot (top-view)");
    // plt::plot(x,y);

    plt::xlim(0, 100);

    plt::show();
}
