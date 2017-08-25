#ifndef VEHICLE_H
#define VEHICLE_H

#define DEBUG_COST 0
#define MY_ID 9999

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <tuple>

#define TO_METERS_PER_SECOND 0.44704
#define TO_MILES_PER_HOUR 2.23693629

using namespace std;

using State = std::string;

struct Snapshot {
   int lane;
   double s;
   double v;
   double a;
   State state;
   };

using Vehicle_ID = int;
using Frenet_s = double;
using Lane = int;
using Position = std::pair<Frenet_s, Lane>;
using Trajectory = std::vector<Position>;
using Prediction = std::tuple<Vehicle_ID,Snapshot,Trajectory>;
using Predictions = std::vector<Prediction>; // using map as each vehicle has its own id


using Cost = double;

struct TrajectoryData {
        State state;
       int proposed_lane;
       double avg_speed;
       double max_acceleration;
       double rms_acceleration;
       double closest_approach;
       double end_distance_to_goal;
       int end_lanes_from_goal;
       std::pair<bool,int> collides;
       };

struct NewTrajectoryData {
        State state;
        double v_front;
        double v_behind;
        double gap_front;
        double gap_behind;
        int change_lane;
       };

using Full_Trajectory = std::vector <Snapshot>;

//- predictions
//A dictionary. The keys are ids of other vehicles and the values are arrays
//where each entry corresponds to the vehicle's predicted location at the
//corresponding timestep. The FIRST element in the array gives the vehicle's
//current position. Example (showing a car with id 3 moving at 2 m/s):

//{
//  3 : [
//    {"s" : 4, "lane": 0},
//    {"s" : 6, "lane": 0},
//    {"s" : 8, "lane": 0},
//    {"s" : 10, "lane": 0},
//  ]
//  4 : [
//    {"s" : 3, "lane": 1},
//    {"s" : 6, "lane": 1},
//    {"s" : 9, "lane": 1},
//    {"s" : 12, "lane": 1},
//  ]
//}

struct Road{
  vector<double> maps_x; // useful info about road center
  vector<double> maps_y; // useful info about road center
  double lane_width;
  int lanes_available;
};

class Vehicle {
public:

  struct collider{
    bool collision ; // is there a collision?
    double time; // time collision happens
  };

  int m_L = 1;// used to compare if vehicles in the same lane

  double m_preferred_buffer = 30; // was 6: impacts "keep lane" behavior.

  Road m_road;

  int m_id;

  int m_lane;

  double m_x;

  double m_y;

  double m_yaw; // angle in rads

  double m_v; // velocity

  double m_vx; // x component of velocity

  double m_vy; // y component of velocity

  double m_a;  // acceleration

  double m_s; // Frenet s coordinate

  double m_d; // Frenet d coordinate

  double m_target_speed;

  double m_max_acceleration;

  int m_goal_lane;

  double m_goal_s;

  string m_state;

  /**
  * Constructor
  */
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void InitFromFusion(Road road, int v_id, double x, double y, double vx, double vy, double s, double d);

  void UpdateMyCar(Road& road, int& v_id, double& x, double& y, double& s, double& d, double& yaw, double& v);

  void update_state(Predictions predictions);

  void configure(vector<double> road_data);

  std::string display() const;

  void increment(int iter = 1);

  vector<int> state_at(int t);

  bool collides_with(Vehicle other, double at_time);

  collider will_collide_with(Vehicle other, int num_timesteps);

  void realize_state(Predictions predictions);

  void realize_constant_speed();

  double max_vel_for_lane(Predictions predictions, int lane, double s);

  double max_accel_for_lane(Predictions predictions, int lane, double s);

  void realize_keep_lane(Predictions predictions);

  void realize_lane_change(Predictions predictions, std::string direction);

  void realize_prep_lane_change(Predictions predictions, std::string direction);

  Snapshot TakeSnapshot() const;

  Full_Trajectory trajectory_for_state(State& state, Predictions predictions, int horizon=10);

  //bool compare(std::pair<std::string,double> i, std::pair<std::string,double> j) ;

  void restore_state_from_snapshot(Snapshot snapshot);

  std::string get_next_state(Predictions predictions);

  Position position_at(double t);

  Trajectory generate_trajectory(double time_interval, int horizon = 10);

};

#endif
