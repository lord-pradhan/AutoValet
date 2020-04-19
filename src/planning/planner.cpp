/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
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
using namespace std::chrono;
using namespace std;
#include "./matplotlib-cpp/matplotlibcpp.h"
#include <cmath>
#include "dubins.h"

namespace plt = matplotlibcpp;


#define  NORMALIZEDISCTHETA(THETA, THETADIRS) (((THETA>=0)?((THETA)%(THETADIRS)):(((THETA)%(THETADIRS)+THETADIRS)%THETADIRS)))
#define 	DISCXY2CONT(X, CELLSIZE)   ((X)*(CELLSIZE) + (CELLSIZE)/2.0)
#define CONTXY2DISC(X, CELLSIZE) (((X)>=0)?((int)((X)/(CELLSIZE))):((int)((X)/(CELLSIZE))-1))

#define numOfDirs 8

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
            std::stringstream ss;
            ss << "ERROR: unable to open " << sMotPrimFile;
            printf("unable to open \n"); 
        }

        if (ReadMotionPrimitives(fMotPrim, readFile) == false) {
            printf("ERROR: failed to read in motion primitive file\n");
            // return;
        }
        fclose(fMotPrim);
    }

    auto start1 = high_resolution_clock::now();

	//initialise vars
	int elemCt = 0;
	int x_size = (int) x_ul/graph_dx +1;
	int y_size = (int) y_ul/graph_dy +1;

    int dX[numOfDirs] = { -1, -1, -1,  0,  0,  1, 1, 1};
    int dY[numOfDirs] = { -1,  0,  1, -1,  1, -1, 0, 1}; 
    double cost[numOfDirs] = {1.4, 1, 1.4, 1, 1, 1.4, 1, 1};

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

			if(freeState(coordsNewTemp) && (gridmap_pre[newy][newx].expanded==false) &&
				(gridmap_pre[newy][newx].G_val > G_temp + cost[dir]) ){

				gridmap_pre[newy][newx].G_val = G_temp + cost[dir];
				open_set_pre.push(gridmap_pre[newy][newx]);
			}
		}
	}

	std::cout<<"Dijkstra expansion done"<<std::endl;

	auto stop1 = high_resolution_clock::now();
	auto duration1 = duration_cast<milliseconds>(stop1 - start1);
	std::cout<<"Time taken for Dijkstra is "<< duration1.count()<<std::endl;

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
				// printf("tempPose is %d %d %d\n", tempPose.x, tempPose.y, tempPose.theta);

				if( freeState(tempPose) && (closed_set.find(GetIndex(tempPose))==closed_set.end()) ){

					State newState(tempPose);
					newState.setH(coordsGoalDisc, gridmap_pre[tempPose.y][tempPose.x].G_val);
					// std::cout<<"H-value for inserted state is "<< newState.getH() << std::endl;
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
					// std::cout<<"G-value for inserted state is "<< fullGraph[elemCt-1].getG() << std::endl;
					open_set.push(fullGraph[elemCt-1]);					
				}
			}			
		}

		if(open_set.top().getCoords() == coordsGoalDisc){
			printf("\nTarget expanded\n");
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
		printf("Path found! \n");
	}
	else{

		printf("Open set is empty, path can't be found. Exiting\n");
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
	const Coord goalCoord(38.0, 10, -PI/2); 

	std::vector<double> x0, y0;
	int printConfiguration(double q[3], double x, std::vector<double>& x0, 
		std::vector<double>& y0 );


	auto start = high_resolution_clock::now();
    std::stack <State> optPath = planner( coordsinit, goalCoord, "prim_test.mprim" );
    auto stop = high_resolution_clock::now();
	auto duration = duration_cast<milliseconds>(stop - start);
	printf("Time taken for computation is %d milliseconds\n", duration.count());

	printf("Size of path is %d \n", optPath.size());

	// printf("initial h_val is %lf\n", init_h);
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
	x_obstacle.push_back(80); y_obstacle.push_back(55);
	x_obstacle.push_back(80); y_obstacle.push_back(62);
	x_obstacle.push_back(0); y_obstacle.push_back(62);

	std::vector<double> x_obstacle1, y_obstacle1;
	x_obstacle1.push_back(100); y_obstacle1.push_back(35);
	x_obstacle1.push_back(20); y_obstacle1.push_back(35);
	x_obstacle1.push_back(20); y_obstacle1.push_back(42);
	x_obstacle1.push_back(100); y_obstacle1.push_back(42);

	// Set the size of output image to 1200x780 pixels
    plt::figure_size(1200, 780);

    plt::plot(x_obstacle, y_obstacle,"r");
    plt::plot(x_obstacle1, y_obstacle1,"r");

    std::map<std::string, std::string> keywords;
    // keywords["scale"]=1;
    plt::quiver(x, y, u, v);//, 'width', 0.05, 'length', 0.1);
    plt::title("Car navigating a dummy parking-lot (top-view)");
    // plt::plot(x,y);

    plt::xlim(0, 100);

    plt::show();
}
