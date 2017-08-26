#ifndef VEHICLE_H
#define VEHICLE_H

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

struct Info {
        State state;
        double v_front;
        double v_behind;
        double gap_front;
        double gap_behind;
        int change_lane;
        double my_speed;
        double my_lane;
        double my_s;
       };

using Full_Trajectory = std::vector <Snapshot>;

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

  void update_state(Predictions predictions, bool DEBUG);

  void configure(vector<double> road_data);

  std::string display() const;

  void increment(int iter = 1);

  vector<int> state_at(int t);

  void realize_state(Predictions predictions, bool DEBUG);

  void realize_constant_speed(Info info);

  void realize_keep_lane(Predictions predictions,Info info,bool DEBUG);

  void realize_lane_change(Predictions predictions, std::string direction);

  Snapshot TakeSnapshot() const;

  //bool compare(std::pair<std::string,double> i, std::pair<std::string,double> j) ;

  void restore_state_from_snapshot(Snapshot snapshot);

  std::string get_next_state(Predictions predictions, bool DEBUG);

  Position position_at(double t);

  Trajectory generate_trajectory(double time_interval, int horizon = 10);

};

#endif
