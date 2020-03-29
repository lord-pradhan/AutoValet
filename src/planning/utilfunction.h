#ifndef UTILFUNCTION_H

#define UTILFUNCTION_H

class Coord{

public:
	double x, y;
	double theta;

	Coord(){}
	Coord(double x_, double y_, double theta_): x(x_), y(y_), theta(theta_) {}
	void snapToGrid();
};

class Control{

private:
	double vel, curv;

public:
	Control(double vel_, double curv_): vel(vel_), curv(curv_){}
};

// compare struct for the priority queue
struct CompareF_pre{
    bool operator()(State const & s1, State const & s2) {
        // return "true" if "p1" is ordered before "p2", for example:
        return s1.getG() >  s2.getG();
    }
};

//graph state
class State{

private:	
	Coord coords;
	double g_val;
	bool expanded;
	int listID;
	std::vector<int> adjIDs;

public:
	State();
	State(int x_, int y_, double theta_);
	State(Coord coordsIn);

	double getX() const;
	double getY() const;
	double getTheta() const;
	Coord getCoords() const;
	double getG() const;
	bool getExpanded() const;
	int getID() const;
	std::vector<int> getAdjIDs() const;

	void setX(double x_);
	void setY(double y_);
	void setTheta(double theta_);
	void setCoords(Coord coordsIn);
	void setG(double g_val_);
	void expand();
	void setID(int listID_);
	void addAdjID(int adjID_);
};


class MotionPrimitive{

public:
	Coord coord_final;
	double cost;
};






#endif