#ifndef HELPER_FUNCTIONS
#define HELPER_FUNCTIONS

#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <iterator>
// helper functions

using namespace std;

double distance(double x1, double y1, double x2, double y2);

// given a colection of points where the x and y coordinates are stored in separate vectors (maps_x and maps_y)
// it returns (the index of) the closest point to the given point (x,y)
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

// gets the closest waypoint that is smaller than 45 degrees with respect to the current point
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

#endif

