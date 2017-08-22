#ifndef HELPER_FUNCTIONS
#define HELPER_FUNCTIONS

#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <iterator>
// helper functions

using Vehicle_ID = int;
using Unique_ID = const int;
using Frenet_s = double;
using Lane = int;
using Position = std::pair<Frenet_s, Lane>;
using Trajectory = std::vector <Position>;
using Prediction = std::pair<Vehicle_ID,Trajectory>;
using Predictions = std::map<Unique_ID,Prediction>; // using map as each vehicle has its own id

using State = std::string;
using Cost = double;

using namespace std;

struct TrajectoryData {
       int proposed_lane;
       double avg_speed;
       double max_acceleration;
       double rms_acceleration;
       double closest_approach;
       double end_distance_to_goal;
       int end_lanes_from_goal;
       std::pair<bool,int> collides;
       };

double distance(double x1, double y1, double x2, double y2)
{
        return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// given a colection of points where the x and y coordinates are stored in separate vectors (maps_x and maps_y)
// it returns (the index of) the closest point to the given point (x,y)
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

        double closestLen = 100000; //large number
        int closestWaypoint = 0;

        for(int i = 0; i < maps_x.size(); i++)
        {
                double map_x = maps_x[i];
                double map_y = maps_y[i];
                double dist = distance(x,y,map_x,map_y);
                if(dist < closestLen)
                {
                        closestLen = dist;
                        closestWaypoint = i;
                }
        }
        return closestWaypoint;
}

// gets the closest waypoint that is smaller than 45 degrees with respect to the current point
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

        int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

        double map_x = maps_x[closestWaypoint];
        double map_y = maps_y[closestWaypoint];

        double heading = atan2( (map_y-y),(map_x-x) );

        double angle = abs(theta-heading);

        if(angle > M_PI/4)
        {
                closestWaypoint++;
        }

        return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
        int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

        int prev_wp;
        prev_wp = next_wp-1;
        if(next_wp == 0)
        {
                prev_wp  = maps_x.size()-1;
        }

        double n_x = maps_x[next_wp]-maps_x[prev_wp];
        double n_y = maps_y[next_wp]-maps_y[prev_wp];
        double x_x = x - maps_x[prev_wp];
        double x_y = y - maps_y[prev_wp];

        // find the projection of x onto n
        double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
        double proj_x = proj_norm*n_x;
        double proj_y = proj_norm*n_y;

        double frenet_d = distance(x_x,x_y,proj_x,proj_y);

        //see if d value is positive or negative by comparing it to a center point

        double center_x = 1000-maps_x[prev_wp];
        double center_y = 2000-maps_y[prev_wp];
        double centerToPos = distance(center_x,center_y,x_x,x_y);
        double centerToRef = distance(center_x,center_y,proj_x,proj_y);

        if(centerToPos <= centerToRef)
        {
                frenet_d *= -1;
        }

        // calculate s value
        double frenet_s = 0;
        for(int i = 0; i < prev_wp; i++)
        {
                frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
        }

        frenet_s += distance(0,0,proj_x,proj_y);

        return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
        int prev_wp = -1;

        while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
        {
                prev_wp++;
        }

        int wp2 = (prev_wp+1)%maps_x.size();

        double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s-maps_s[prev_wp]);

        double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
        double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

        double perp_heading = heading-M_PI/2;

        double x = seg_x + d*cos(perp_heading);
        double y = seg_y + d*sin(perp_heading);

        return {x,y};

}

//// reviewed
//Position get_position(double t, double x, double y, double vx, double vy,
//                     vector<double>maps_x, vector<double> maps_y)
//{
//    /*
//    Predicts state of vehicle in t seconds (assuming zero acceleration)
//    */
//    double x_new = x+vx*t;
//    double y_new = y+vy*t;
//    if (vx != 0)
//    {
//    double theta = std::atan2(vy,vx);

//    std::vector<double> coordinates = getFrenet(x, y, theta, maps_x, maps_y);

//    // TODO:
//    // convert to Frenet and then
//    int lane_width = 4; //hard coded in here for the moment!
//    int lane = (int)(coordinates[1]) % lane_width;

//    Position position = {coordinates[0], lane};
//    return position;
//}

//// reviewed
//Trajectory get_trajectory(double x, double y, double vx, double vy,
//                               int horizon /*= 10*/, double time_interval,
//                               vector<double>maps_x, vector<double> maps_y)
//{
//    Trajectory predicted_trajectory;
//    for( int it = 0; it < horizon; ++it)
//    {
//        Position predicted_position = get_position(it*time_interval,x,y,vx,vy,maps_x,maps_y);
//        predicted_trajectory.push_back(predicted_position);
//    }
//    return predicted_trajectory;
//}

#endif
