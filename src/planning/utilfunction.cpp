#include "utilfunction.h"
#include "params.h"
#include <math.h>
#include <vector>
#include <limits>
#include <bits/stdc++.h> 
#include "dubins.h"


#define  NORMALIZEDISCTHETA(THETA, THETADIRS) (((THETA>=0)?((THETA)%(THETADIRS)):(((THETA)%(THETADIRS)+THETADIRS)%THETADIRS)))
#define 	DISCXY2CONT(X, CELLSIZE)   ((X)*(CELLSIZE) + (CELLSIZE)/2.0)
#define CONTXY2DISC(X, CELLSIZE) (((X)>=0)?((int)((X)/(CELLSIZE))):((int)((X)/(CELLSIZE))-1))
//////////////// useful functions ////////


bool freeState( CoordDisc coordsIn ){

    double x_ul = 1000, x_ll = 0, y_ul = 1000, y_ll = 0;
    double x= DISCXY2CONT(coordsIn.x, graph_dx), y = DISCXY2CONT(coordsIn.y, graph_dy);
    // double theta = DiscTheta2Cont(coordsIn.theta, numAngles);

    if( x > x_ul || x< x_ll || y > y_ul || y< y_ll ){

        return false;
    }
    // else if(x<50 && y<62 && y>55 ){
    //     return false;
    // }
    // else if( x>50 && y<42 && y>35 ){
    //     return false;
    // }

    return true;
}


bool goalRegion(const Coord& ego, const Coord &goal){

	if( (goal.x - ego.x) < graph_dx && (goal.x - ego.x) > -graph_dx  && (goal.y - ego.y) < graph_dy 
		&& (goal.y - ego.y) > -graph_dy && (goal.theta - ego.theta) < graph_dtheta && 
		(goal.theta - ego.theta) > -graph_dtheta){			
		
		return true;
	}
	
	else
		return false;
}


double wrap2pi(const double& theta_in){

	double theta_out;
	theta_out = fmod( theta_in, 2*PI );
	if(theta_out<0){
		theta_out += 2*PI;
	}
	return theta_out;
}


bool Coord::operator==(const Coord &obj){

	if(x == obj.x && y==obj.y && theta == obj.theta)
		return true;
	else
		return false;
}


bool CoordDisc::operator==(const CoordDisc &obj){

	if(x == obj.x && y==obj.y && theta == obj.theta)
		return true;
	else
		return false;
}


//// State class ///////
State::State(): expanded(false){

	g_val = std::numeric_limits<double>::infinity();
}

State::State(int x_, int y_, int theta_): expanded(false){

	coords.x=x_;
	coords.y=y_;
	coords.theta=theta_;
	g_val = std::numeric_limits<double>::infinity();
}

State::State(CoordDisc coordsIn): coords(coordsIn), expanded(false){
	
	g_val = std::numeric_limits<double>::infinity();	
}

int State::getX() const {return coords.x;} 
int State::getY() const {return coords.y;} 
int State::getTheta() const {return coords.theta;}
CoordDisc State::getCoords() const { return coords; }

double State::getG() const {return g_val;} 
double State::getH() const {return h_val;} 
bool State::getExpanded() const {return expanded;} 
int State::getID() const {return listID;}
std::vector<GraphEdge> State::getAdjElems() const {return adjElems;}

void State::setX(int x_grid_) { coords.x = x_grid_; return;}
void State::setY(int y_grid_) { coords.y = y_grid_; return;}
void State::setTheta(int theta_) {coords.theta = theta_;}
void State::setCoords(CoordDisc coordsIn){ coords = coordsIn; }

void State::setG(double g_val_) { 
	g_val = g_val_;
}
void State::setH(CoordDisc goalDisc){

    // h_val = (double) sqrt( (coords.x - goalDisc.x)*(coords.x - goalDisc.x) + 
    //     (coords.y - goalDisc.y)*(coords.y - goalDisc.y) + std::min( (coords.theta - goalDisc.theta)*
    //     (coords.theta - goalDisc.theta), (16 - abs(coords.theta - goalDisc.theta))*
    //     (16 - abs(coords.theta - goalDisc.theta)) ) );
    // std::cout<< "h_val is "<<h_val<<"\n";

    double q0[] = { DiscXY2Cont(coords.x,graph_dx), DiscXY2Cont(coords.y, graph_dy), 
        DiscTheta2Cont(coords.theta, numAngles) };

    double q1[] = { DiscXY2Cont(goalDisc.x, graph_dx), DiscXY2Cont(goalDisc.y, graph_dy), 
        DiscTheta2Cont(goalDisc.theta, numAngles) };

    DubinsPath path;
    dubins_shortest_path(&path, q0, q1, 5.0);

    h_val = dubins_path_length( &path )/graph_dx;
    // std::cout<< "dubins_path_length is "<< h_val<<"\n";
}
void State::expand() { expanded = true; return; }
void State::setID(int listID_) { listID = listID_; }
void State::addAdjElem(GraphEdge adjElem_){adjElems.push_back(adjElem_);}



//////// parse motion primitive file //////
bool ReadMotionPrimitives(FILE* fMotPrims, MPrimFile& readFile){

	char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    printf("Reading in motion primitives...");
    fflush(stdout);

    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
        fflush(stdout);
        return false;
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0) {
        return false;
    }

    // if (fabs(fTemp - EnvNAVXYTHETALATCfg.cellsize_m) > ERR_EPS) {
    //     printf("ERROR: invalid resolution %f (instead of %f) in the dynamics file\n", fTemp, EnvNAVXYTHETALATCfg.cellsize_m);
    //     fflush(stdout);
    //     return false;
    // }

    printf("resolution_m: %f\n", fTemp);

    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    printf("sTemp: %s\n", sTemp);

    // read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if (strcmp(sTemp, sExpected) != 0) {
       printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
       return false;
    }
    if (fscanf(fMotPrims, "%d", &dTemp) == 0) {
        return false;
    }

    printf("numberofangles: %d\n", dTemp);
    readFile.numAngles = dTemp;


    // read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0) {
        return false;
    }
    printf("totalnumberofprimitives: %d\n", totalNumofActions);

    // Read in motion primitive for each action
    for (int i = 0; i < totalNumofActions; i++) {
        Primitive motprim;

        if (!ReadinMotionPrimitive(motprim, fMotPrims)) {
            return false;
        }

        readFile.inputPrims.push_back(motprim);
    }
    printf("done");
    fflush(stdout);
    return true;
}


bool ReadinMotionPrimitive( Primitive& pMotPrim, FILE* fIn){

	char sTemp[1024];
    int dTemp;
    char sExpected[1024];
    int numofIntermPoses;
    float fTemp;

    // read in actionID
    strcpy(sExpected, "primID:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
        fflush(stdout);
        return false;
    }
    if (fscanf(fIn, "%d", &pMotPrim.primID) != 1) {
        return false;
    }

    // read in start angle
    strcpy(sExpected, "startangle_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) == 0) {
        printf("ERROR reading startangle\n");
        return false;
    }
    pMotPrim.startAngleDisc = dTemp;

    // read in end pose
    strcpy(sExpected, "endpose_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }

    if (ReadinCell(pMotPrim.endPose, fIn) == false) {
        printf("ERROR: failed to read in endsearchpose\n");
        return false;
    }

    // read in action cost
    strcpy(sExpected, "additionalactioncostmult:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) != 1) {
        return false;
    }
    pMotPrim.cost = dTemp;

    // read in intermediate poses
    strcpy(sExpected, "intermediateposes:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &numofIntermPoses) != 1) {
        return false;
    }
    // all intermposes should be with respect to 0,0 as starting pose since it
    // will be added later and should be done after the action is rotated by
    // initial orientation
    for (int i = 0; i < numofIntermPoses; i++) {
        Coord intermpose;
        if (ReadinPose(intermpose, fIn) == false) {
            printf("ERROR: failed to read in intermediate poses\n");
            return false;
        }
        pMotPrim.intermPoses.push_back(intermpose);
    }

    // Check that the last pose of the motion matches (within lattice
    // resolution) the designated end pose of the primitive
    Coord sourcepose;
    sourcepose.x = DISCXY2CONT(0, graph_dx);
    sourcepose.y = DISCXY2CONT(0, graph_dy);
    sourcepose.theta = DiscTheta2Cont(pMotPrim.startAngleDisc, numAngles);
    double mp_endx_m = sourcepose.x + pMotPrim.intermPoses.back().x;
    double mp_endy_m = sourcepose.y + pMotPrim.intermPoses.back().y;
    double mp_endtheta_rad = pMotPrim.intermPoses.back().theta;

    int endtheta_c;
    int endx_c = CONTXY2DISC(mp_endx_m, graph_dx);
    int endy_c = CONTXY2DISC(mp_endy_m, graph_dy);
    endtheta_c = ContTheta2Disc(mp_endtheta_rad, numAngles);
    if (endx_c != pMotPrim.endPose.x ||
        endy_c != pMotPrim.endPose.y ||
        endtheta_c != pMotPrim.endPose.theta)
    {
        printf( "ERROR: incorrect primitive %d with startangle=%d "
                   "last interm point %f %f %f does not match end pose %d %d %d\n",
                   pMotPrim.primID, pMotPrim.startAngleDisc,
                   pMotPrim.intermPoses[pMotPrim.intermPoses.size() - 1].x,
                   pMotPrim.intermPoses[pMotPrim.intermPoses.size() - 1].y,
                   pMotPrim.intermPoses[pMotPrim.intermPoses.size() - 1].theta,
                   pMotPrim.endPose.x, pMotPrim.endPose.y,
                   pMotPrim.endPose.theta);
        fflush(stdout);
        return false;
    }

    return true;
}

bool ReadinCell( CoordDisc& pose, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose.x = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose.y = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose.theta = atoi(sTemp);

    // normalize the angle
    pose.theta = NORMALIZEDISCTHETA(pose.theta, numAngles);

    return true;
}

bool ReadinPose( Coord& pose, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose.x = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose.y = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose.theta = atof(sTemp);

    pose.theta = normalizeAngle(pose.theta);

    return true;
}

 //converts continuous (radians) version of angle into discrete
 //maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2,...
int ContTheta2Disc(double fTheta, int NUMOFANGLEVALS)
{
	double thetaBinSize = 2.0*PI/NUMOFANGLEVALS;
	return (int)(normalizeAngle(fTheta+thetaBinSize/2.0)/(2.0*PI)*(NUMOFANGLEVALS));
}


double DiscTheta2Cont(int nTheta, int NUMOFANGLEVALS)
{
	double thetaBinSize = 2.0*PI/NUMOFANGLEVALS;
	return nTheta*thetaBinSize;
}

double normalizeAngle(double angle)
{
	double retangle = angle;

	//get to the range from -2PI, 2PI
	if(fabs(retangle) > 2*PI)
	 retangle = retangle - ((int)(retangle/(2*PI)))*2*PI; 

	//get to the range 0, 2PI
	if(retangle < 0)
	 retangle += 2*PI;

	if(retangle < 0 || retangle > 2*PI)
	 {
	 printf("ERROR: after normalization of angle=%f we get angle=%f\n", angle, retangle);
	 }

	return retangle;
}

double DiscXY2Cont( int val, double resolution ){

    // DISCXY2CONT(X, CELLSIZE)   ((X)*(CELLSIZE) + (CELLSIZE)/2.0)
    return (double) val*resolution + resolution/2.0 ;
}

int ContXY2Disc(double val, double resolution){

    // (val>=0)?( (int)  )
    int out;
    int offset = fmod(val,resolution);
    if(offset>resolution/2){
        out = std::floor(val/resolution) + 1;
    }
    else
        out = std::floor(val/resolution);
    
    return out;
}
// CONTXY2DISC(X, CELLSIZE) (((X)>=0)?((int)((X)/(CELLSIZE))):((int)((X)/(CELLSIZE))-1))

int GetIndex(CoordDisc coordsIn){

    // x = x_abs - x_r; y = y_abs - y_r; t = t_as - t_r;
    int x = coordsIn.x, y = coordsIn.y, theta = coordsIn.theta;
    return theta*(2*theta-1)*(2*theta+1)/3 + (y+theta)*(2*theta+1) + x + theta;
}