#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "vehicle.h"
#include "helper_functions.h"
//#include "helper_functions.hpp"
//#include "cost_functions.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  //start at lane 1
  int lane = 1;

  // reference velocity
  double ref_vel = 0; //mph

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  double s = 0.0; // initial s coordinate
  double v = 0.0;  // initial velocity
  double a = 0.0;  // initial acceleration

  Road road;
  road.lanes_available = 3;
  road.lane_width = 4.0;
  road.maps_x = map_waypoints_x;
  road.maps_y = map_waypoints_y;

  Vehicle my_car;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
              &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &my_car, &road]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
                int id = 0;
                my_car.UpdateMyCar(road, id, car_x, car_y, car_s, car_d, car_yaw, car_speed);

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

                // Note the previous_path contains all the waypoints generated in the previous iteration reduced by
                // the waypoints that the car actually reached in the meantime:
                // e.g. the path calculated in the previous iteration contained 50 points
                // the car reached 10 out of those 50
                // the previous_path now contains only 40 points!
                int prev_size = previous_path_x.size();


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

                /////////////////////////////////////////////////////////////////////////////
                // Build Predictions map from sensor_fusion -  see Road advance
                Predictions predictions;
                // sensor_fusion = {id, x, y, vx, vy, s, d}
                int unique_key = 0;
                for (int i = 0; i < sensor_fusion.size(); i++)
                {
                        int v_id = sensor_fusion[i][0];
                        double x = sensor_fusion[i][1];
                        double y = sensor_fusion[i][2];
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double s = sensor_fusion[i][5];
                        double d = sensor_fusion[i][6];
                        double horizon = 10;
                        double time_interval = 1;
                        // Create a vehicle
                        Vehicle new_car;
                        new_car.InitFromFusion(road,v_id,x,y,vx,vy,s,d);
                        Trajectory trajectory = new_car.generate_trajectory(time_interval,horizon);
                        Prediction prediction = std::make_pair(v_id,trajectory);
                        predictions.insert(std::make_pair(unique_key++,prediction)); // use and then increment unique_key
                }

                /////////////////////////////////////////////////////////////////////////////
                // Update state of my vehicle - see get_next_state vehicle.cpp

                State state = my_car.get_next_state(predictions);
                my_car.m_state = state;
                my_car.realize_state(predictions);


                /////////////////////////////////////////////////////////////////////////////
                // Get data from sensor_fusion
                if (prev_size > 0)
                {
                        car_s = end_path_s;
                }
                bool too_close = false;

                // find ref_v to use
                for (int i = 0; i < sensor_fusion.size(); i++)
                {
                        float d = sensor_fusion[i][6];
                        // if car is in my lane
                        if ((d < (2+4*lane+2)) && (d >(2+4*lane-2)))
                        {
                                double vx = sensor_fusion[i][3];
                                double vy = sensor_fusion[i][4];
                                double check_speed = sqrt(vx*vx+vy*vy);
                                double check_car_s = sensor_fusion[i][5];
                                // predict where this car will be in the future
                                check_car_s += (double)prev_size*.02*check_speed;

                                // check s values greater than mine and s gap
                                const double min_distance = 20;
                                // if car is too close
                                if ((check_car_s>car_s) && ((check_car_s-car_s) < min_distance))
                                {
                                        too_close = true;
                                        //std::cout << "Too close!" << std::endl;
                                        lane = my_car.m_lane;
                                        if (lane>0)
                                        {
//                                           lane =0;
//                                           ref_vel = my_car.m_v;

                                        }
                                }

                        }

                }


                std::cout << "my_car.m_lane = " << my_car.m_lane <<" Lane " << lane << std::endl;
                if (too_close)
                {
                        ref_vel -= .224; // corresponds to 5 mph
                }
                else if (ref_vel < 49.5)
                {
                        ref_vel += .224;
                }
//                lane =2;

                // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
                // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
                vector<double> ptsx;
                vector<double> ptsy;

                double ref_x = car_x;
                double ref_y = car_y;
                double ref_yaw = deg2rad(car_yaw);

                // if previous size is almost empty, use the car as starting reference
                if(prev_size<5)
                {
                    //Use two points that make the path tangent to the car
                    double prev_car_x = car_x - cos(car_yaw);
                    double prev_car_y = car_y - sin(car_yaw);

                    ptsx.push_back(prev_car_x);
                    ptsx.push_back(car_x);

                    ptsy.push_back(prev_car_y);
                    ptsy.push_back(car_y);

                }
                //use the previous path's end point as starting reference
                else
                {

                    //Redefine reference state as previous path end point
                    ref_x = previous_path_x[prev_size-1];
                    ref_y = previous_path_y[prev_size-1];

                    double ref_x_prev = previous_path_x[prev_size-2];
                    double ref_y_prev = previous_path_y[prev_size-2];
                    ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

                    //Use two points that make the path tangent to the previous path's end point
                    ptsx.push_back(ref_x_prev);
                    ptsx.push_back(ref_x);

                    ptsy.push_back(ref_y_prev);
                    ptsy.push_back(ref_y);

                }

                // In Frenet add evenly 30m spaced points ahead of the starting reference
                vector<double> next_wp0 = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp1 = getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp2 = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                ptsx.push_back(next_wp0[0]);
                ptsx.push_back(next_wp1[0]);
                ptsx.push_back(next_wp2[0]);

                ptsy.push_back(next_wp0[1]);
                ptsy.push_back(next_wp1[1]);
                ptsy.push_back(next_wp2[1]);

                for (int i = 0; i< ptsx.size(); i++)
                {

                    //shift car reference angle to 0 degrees
                    double shift_x = ptsx[i]-ref_x;
                    double shift_y = ptsy[i]-ref_y;

                    ptsx[i] = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
                    ptsy[i] = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);

                }

                // create a spline
                tk::spline s;

                // set (x,y) points to the spline
                s.set_points(ptsx, ptsy);

                // define the actual (x,y) points we will use for the planner
                vector<double> next_x_vals;
                vector<double> next_y_vals;

                // Start with all of the previous path points from last time
                for (int i = 0; i < previous_path_x.size(); i++)
                {
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                }

                // Calculate how to break up spline points so that we travel at our desired reference velocity
                double target_x = 30.0;
                double target_y = s(target_x);
                double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

                double x_add_on = 0;

                //Fill out the rest of our path planner after filling it with previous points, here we will always output 50 points
                for (int i = 1; i<= 50-previous_path_x.size(); i++)
                {
                    double N = target_dist/(.02*ref_vel/2.24); //2.24 to convert from mph to meters/s
                    double x_point = x_add_on+target_x/N;
                    double y_point = s(x_point);

                    x_add_on = x_point;

                    double x_ref = x_point;
                    double y_ref = y_point;

                    //rotate back to normal after rotating it earlier
                    x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
                    y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

                    x_point += ref_x;
                    y_point += ref_y;

                    next_x_vals.push_back(x_point);
                    next_y_vals.push_back(y_point);
                }

                json msgJson;

                // END
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































