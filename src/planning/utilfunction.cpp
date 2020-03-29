#include "utilfunction.h"
#include "params.h"
#include <math.h>
#include <vector>

// #define vel_plan 2
// #define euler_dt 0.01

bool freeState( Coord coordsIn ){return true;}

// everything for graph construction
std::vector<Primitive> getNextStates( Coord coordsIn  ){

	std::vector<Primitive> primitives;

	// straight - short (1)
	Control in(vel_plan, 0.0);
	double t_ahead = 1.0;
	Primitive temp; 
	temp.coord_final = motionRollout( coordsIn, in, t_ahead );
	temp.cost = 1.0;
	primitives.push_back( temp );

	// straight - long (2)
	// Control in(vel_plan, 0.0);
	in.vel = vel_plan; in.curv = 0.0;
	t_ahead = 2.0;
	temp.coord_final = motionRollout( coordsIn, in, t_ahead );
	temp.cost = 2.0;
	primitives.push_back( temp );

	// reverse - short (3)
	// Control in(-vel_plan, 0.0);
	in.vel = -vel_plan; in.curv = 0.0;
	t_ahead = 1.0;
	temp.coord_final = motionRollout( coordsIn, in, t_ahead );
	temp.cost = 5.0;
	primitives.push_back( temp );


	// left - sharp (4)
	// Control in( vel_plan, curv_lim );
	in.vel = vel_plan; in.curv = curv_lim;
	t_ahead = 0.5;
	// Primitive temp; 
	temp.coord_final = motionRollout( coordsIn, in, t_ahead );
	temp.cost = 1.0;
	primitives.push_back( temp );

	// right - sharp (5)
	// Control in( vel_plan, -curv_lim );
	in.vel = vel_plan; in.curv = -curv_lim;
	t_ahead = 0.5;
	temp.coord_final = motionRollout( coordsIn, in, t_ahead );
	temp.cost = 1.0;
	primitives.push_back( temp );

	// left - mild (6)
	// Control in( vel_plan, curv_lim/2 );
	in.vel = vel_plan; in.curv = curv_lim/2;
	t_ahead = 0.7;
	temp.coord_final = motionRollout( coordsIn, in, t_ahead );
	temp.cost = 1.0;
	primitives.push_back( temp );


	// right - mild (7)
	// Control in( vel_plan, -curv_lim/2 );
	in.vel = vel_plan; in.curv = -curv_lim/2;
	t_ahead = 0.7;
	temp.coord_final = motionRollout( coordsIn, in, t_ahead );
	temp.cost = 1.0;
	primitives.push_back( temp );
}


Coord motionRollout( Coord coordsIn, Control controlIn, double time_ahead ){

	double x_end = coordsIn.x, y_end = coordsIn.y, theta_end=coordsIn.theta;
	for(int i = 0; i < (int) time_ahead/euler_dt; i++ ){

		theta_end += euler_dt * controlIn.vel * controlIn.curv;
		x_end += euler_dt * controlIn.vel * cos(theta_end);
		y_end += euler_dt * controlIn.vel * sin(theta_end); 

		// insert collision checking here
	}

	Coord coordOut(x_end, y_end, theta_end); 
	// coordOut.x = x_end; coordOut.y = y_end; coordOut.theta = theta_end;
	coordOut.snapToGrid();
	return coordOut;
}

bool Coord::operator=(const Coord &obj){

	if(x == obj.x && y==obj.y && theta == obj.theta)
		return true;
	else
		return false;
}

void Coord::snapToGrid(){

	//snap to grid
	double offset_x = x % graph_dx;
	double offset_y = y % graph_dy;
	double offset_theta = theta % graph_dtheta;

	if(offset_x > graph_dx/2)
		x += graph_dx - offset_x;
	else
		x -= offset_x;

	if(offset_y > graph_dy/2)
		y += graph_dy - offset_x;
	else
		y -= offset_y;

	if(offset_theta > graph_dtheta/2)
		theta += graph_dtheta - offset_theta;
	else
		theta -= offset_theta;
}

// everything for graph search
State::State(): expanded(false){

	g_val = numeric_limits<double>::infinity();
}

State::State(int x_, int y_, double theta_): expanded(false), 
	coords.x(x_), coords.y(y_), coords.theta(theta_){

	g_val = numeric_limits<double>::infinity();
}

State::State(Coord coordsIn): coords(coordsIn), expanded(false){
	
	g_val = numeric_limits<double>::infinity();	
}

int State::getX() const {return coords.x;} 
int State::getY() const {return coords.y;} 
double State::getTheta() const {return coords.theta;}
Coord State::getCoords() const { return coords; }

double State::getG() const {return g_val;} 
bool State::getExpanded() const {return expanded;} 
int State::getID() const {return listID;}
std::vector<Graphedge> State::getAdjElems() const {return adjElems;}

void State::setX(int x_grid_) { x_grid = x_grid_; return;}
void State::setY(int y_grid_) { y_grid = y_grid_; return;}
void State::setTheta(int theta_) {coords.theta = theta_;}
void State::setCoords(Coord coordsIn){ coords = coordsIn; }

void State::setG(double g_val_) { 
	g_val = g_val_;
}
void State::expand() { expanded = true; return; }
void State::setID(int listID_) { listID = listID_; }
void State::addAdjElem(Graphedge adjElem_) {adjElems.push_back(adjElem_);}
