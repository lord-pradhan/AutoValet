#include <vector>
#include <limits>
#include <bits/stdc++.h> 


#ifndef UTILFUNCTION_H

#define UTILFUNCTION_H

class Coord{

public:
	double x, y;
	double theta;

	Coord(){}
	Coord(double x_, double y_, double theta_): x(x_), y(y_), theta(theta_) {}

	bool operator==(const Coord &obj);
	// void snapToGrid();
};



class CoordDisc{

public:
	int x, y;
	int theta;

	CoordDisc(){}
	CoordDisc(int x_, int y_, int theta_): x(x_), y(y_), theta(theta_) {}

	bool operator==(const CoordDisc &obj);

	// void snapToGrid();
};


// class Control{

// public:
// 	Control(double vel_, double curv_): vel(vel_), curv(curv_){}

// 	double vel, curv;
// };


//// graph edge ////
class GraphEdge{

public:
	int ID;
	double cost;
};

// // backwards Dijkstra
class State_pre{

public:
	CoordDisc coords;
	double G_val;
	bool expanded;

	State_pre( CoordDisc coordsIn, double G_valIn ): coords(coordsIn), G_val(G_valIn){}

	State_pre(): G_val(std::numeric_limits<double>::infinity()), expanded(false) {}
};



//// graph state /////
class State{
private:	
	CoordDisc coords;
	double g_val, h_val;
	bool expanded;
	int listID;
	std::vector<GraphEdge> adjElems;

public:
	State();
	State(int x_, int y_, int theta_);
	State(CoordDisc coordsIn);

	int getX() const;
	int getY() const;
	int getTheta() const;
	CoordDisc getCoords() const;
	double getG() const;
	double getH() const;
	bool getExpanded() const;
	int getID() const;
	std::vector<GraphEdge> getAdjElems() const;

	void setX(int x_);
	void setY(int y_);
	void setTheta(int theta_);
	void setCoords(CoordDisc coordsIn);
	void setG(double g_val_);
	void setH(CoordDisc goalDisc, double h_env);
	void expand();
	void setID(int listID_);
	void addAdjElem(GraphEdge elemIn);
};


///// compare struct for the priority queue /////
struct CompareF{
    bool operator()(State const & s1, State const & s2) {
        // return "true" if "p1" is ordered before "p2", for example:
        long int eps=2;
        return s1.getG()+eps*s1.getH() >  s2.getG()+eps*s2.getH();
    }
};

struct CompareF_pre{
    bool operator()(State_pre const & s1, State_pre const & s2) {
        // return "true" if "p1" is ordered before "p2", for example:
        long int eps=1;
        return s1.G_val >  s2.G_val;
    }
};


//// primitive class //////
class Primitive{

public:
	std::vector<Coord> intermPoses;
	CoordDisc endPose;
	double cost;
	int primID, startAngleDisc;
	// Coord applyPrimEnd( Coord poseIn );
};


////// read mprim class //////
class MPrimFile{

public:
	std::vector<Primitive> inputPrims;
	int primTot, numAngles;
};



//// useful functions //////
// std::vector<Primitive> getNextStates( Coord coordsIn  );

// Coord motionRollout( Coord coordsIn, Control controlIn, double time_ahead );

bool freeState( CoordDisc coordsIn );

bool goalRegion(const Coord& ego, const Coord &goal);

double wrap2pi(const double& theta_in);

bool ReadMotionPrimitives(FILE* fMotPrims, MPrimFile& readFile);

bool ReadinMotionPrimitive( Primitive& pMotPrim, FILE* fIn);

bool ReadinCell( CoordDisc& pose, FILE* fIn);

bool ReadinPose( Coord& pose, FILE* fIn);

int ContTheta2Disc(double fTheta, int NUMOFANGLEVALS);

double DiscTheta2Cont(int nTheta, int NUMOFANGLEVALS);

double normalizeAngle(double angle);

unsigned long long int GetIndex(CoordDisc coordsIn);

int ContXY2Disc(double val, double resolution);

double DiscXY2Cont( int val, double resolution );

#endif