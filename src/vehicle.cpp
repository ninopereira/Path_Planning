#include <iostream>
#include <string>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <iostream>     // std::cout
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include "vehicle.h"
#include "cost_functions.hpp"
#include "helper_functions.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {
    m_id = -1;
    m_x = -1;
    m_y = -1;
    m_yaw = 0;
    m_vx = -1;
    m_vy = -1;
    m_v = -1;
    m_a = -1;
    m_s = -1;
    m_d = -1;
    m_lane = -1;

    m_max_acceleration = 5;
    m_target_speed = 49.5*TO_METERS_PER_SECOND; //mph
    m_goal_s = 100000;
    m_state = "CS";
    m_goal_lane = -1;
}

void Vehicle::InitFromFusion(Road road, int v_id, double x, double y, double vx, double vy, double s, double d){

    m_road = road;

    m_id = v_id;
    m_x = x;
    m_y = y;
    m_vx = vx;
    m_vy = vy;
    m_s = s;
    m_d = d;

    // calculate extra vars
    if (vx!=0) {m_yaw = atan2(vy,vx);}
    else {m_yaw = 0;}
    m_v = std::sqrt(vx*vx + vy*vy);
    m_a = 0; // assuming zero acceleration;

    if ((int)road.lane_width!=0){
    m_lane = (int)m_d / (int)road.lane_width;}

    m_state = "CS";
    m_max_acceleration = 5;
    m_target_speed = 49.5*TO_METERS_PER_SECOND;
    m_goal_lane = 1;
    m_goal_s = 100000;
}

void Vehicle::UpdateMyCar(Road &road, int &v_id, double &x, double &y, double &s, double &d, double &yaw, double &v)
{
        m_road = road;

        m_id = v_id;
        m_x = x;
        m_y = y;
        m_s = s;
        m_d = d;
        m_yaw = yaw;
        m_v = v*TO_METERS_PER_SECOND;

        // calculate extra vars
        m_vx = m_v * cos(yaw); // assuming yaw in rads
        m_vy = m_v * sin(yaw); // assuming yaw in rads
        m_a = 0; // assuming zero acceleration;

        if ((int)road.lane_width!=0){
            m_lane = (int)m_d / (int)road.lane_width;}

        m_target_speed = 49.5*TO_METERS_PER_SECOND;

        m_goal_s = m_s+100000;
}

Vehicle::~Vehicle() {}

double get_angle_diff(double angle1, double angle2)
{
//    int multiplier = 1000;
//    angle1 = (double)((int)(angle1*multiplier) % (360*multiplier)); // unwrap if necessary
//    angle2 = (double)((int)(angle2*multiplier) % (360*multiplier)); // unwrap if necessary
    double smaller;
    double bigger;
    if (angle1 < angle2)
    {
       smaller = angle1;
       bigger = angle2;
    }
    else
    {
        smaller = angle2;
        bigger = angle1;
    }
    double hipothesis1 = (bigger-smaller);
    double hipothesis2 = ((smaller+360)-bigger);
    return std::min(hipothesis1, hipothesis2);
}

void Vehicle::update_state(Predictions predictions, bool DEBUG) {
    static double prev_yaw = 0;
    const int size =5;
    static double yaw_accel[size];
    static int it=0;
    Snapshot car_state = TakeSnapshot();

    State desired_state = get_next_state(predictions,DEBUG); // compute the best state
    restore_state_from_snapshot(car_state);

    string state = car_state.state;
    yaw_accel[it] = get_angle_diff(prev_yaw,car_state.yaw);
    ++it;
    if (it>=size){it=0;}
    double avg_yaw_accel=0;
    for (int  i = 0; i<size ; i++)
    {
        avg_yaw_accel+=yaw_accel[i];
    }
    avg_yaw_accel = avg_yaw_accel/(double)size;

    if(DEBUG){std::cout << "===================avg_accel=" << avg_yaw_accel << "===================" << state << std::endl;}
    // simple FSM
    switch (hashit(state)){
    case CS:
        if (hashit(desired_state) != CS)
        {
            m_state = "KL";
        }
        realize_state(predictions,DEBUG);
        break;
    case KL:
        realize_state(predictions,DEBUG);

        if ((car_state.v > 25*TO_METERS_PER_SECOND) && avg_yaw_accel<0.1 && (car_state.v < 42*TO_METERS_PER_SECOND))
        {
            switch (hashit(desired_state)){
            case LCL:
                m_state = "LCL";
                realize_state(predictions,DEBUG);
                m_goal_lane = m_lane;
                m_v = car_state.v;//maintain speed
                break;
            case LCR:
                m_state = "LCR";
                realize_state(predictions,DEBUG);
                m_goal_lane = m_lane;
                m_v = car_state.v; //maintain speed
                break;
            default:
                break;
            }
        }

        break;
    case LCL:
        if (car_state.lane == m_goal_lane){
            m_state = "KL";
            realize_state(predictions,DEBUG);
        }
        m_lane = m_goal_lane;
        break;
    case LCR:

        if (car_state.lane == m_goal_lane){
            m_state = "KL";
            realize_state(predictions,DEBUG);
        }
        m_lane = m_goal_lane;
        break;
    case PLCL:
        m_state = "KL";
        realize_state(predictions,DEBUG);
        break;
    case PLCR:
        m_state = "KL";
        realize_state(predictions,DEBUG);
        break;
    default:
        m_state = "KL";
        realize_state(predictions,DEBUG);
        break;
    }
    prev_yaw = car_state.yaw;
}

State Vehicle::get_next_state(Predictions predictions, bool DEBUG)
{
    vector<State> states = {"KL", "LCL", "LCR"};
    if  (m_lane == 0)
        {
        State search_str= "LCL";
        std::vector<State>::iterator result = std::find(states.begin(), states.end(), search_str);
        if (result!= states.end())
                {states.erase(result);}
    }
    if (m_lane == (m_road.lanes_available-1))
    {
        State search_str= "LCR";
        std::vector<State>::iterator result = std::find(states.begin(), states.end(), search_str);
                if (result!= states.end())
                {states.erase(result);}
    }

    vector<std::pair<State,Cost>> costs;
    for (auto state:states)
    {
      if (DEBUG){ std::cout << "| ST=" << state ;}
      Info info = get_info(*this, predictions, state, DEBUG);
      double cost = calculate_cost(*this, predictions, state, info, DEBUG);
          costs.push_back({state, cost});
    }
												 
    // gets the strategy (plan) with minimum cost
    auto it = std::min_element(costs.begin(), costs.end(),
                          [](decltype(costs)::value_type& l, decltype(costs)::value_type& r) -> bool { return l.second < r.second; });

    return (*it).first;

}

Snapshot Vehicle::TakeSnapshot() const
{
    Snapshot snapshot;
    snapshot.lane = m_lane;
    snapshot.s = m_s;
    snapshot.v = m_v;
    snapshot.yaw = m_yaw;
    snapshot.a = m_a;
    snapshot.state = m_state;
    return snapshot;
}

void Vehicle::restore_state_from_snapshot(Snapshot snapshot)
{
    m_lane = snapshot.lane;
    m_s = snapshot.s;
    m_v = snapshot.v;
    m_yaw = snapshot.yaw;
    m_a = snapshot.a;
    m_state = snapshot.state;
}

std::string Vehicle::display() const
{
	ostringstream oss;

    oss << "s:    " << m_s << "\n";
    oss << "lane: " << m_lane << "\n";
    oss << "v:    " << m_v << "\n";
    oss << "a:    " << m_a << "\n";
    return oss.str();
}

void Vehicle::realize_state(Predictions predictions, bool DEBUG)
{
    //Given a state, realize it by adjusting acceleration and lane.

    string state = m_state;
    Info info = get_info(*this, predictions, state, DEBUG);

    if(state.compare("CS") == 0)
    {
        realize_constant_speed(info);
    }
    else if(state.compare("KL") == 0)
    {
        realize_keep_lane(predictions,info,DEBUG);
    }
    else if(state.compare("LCL") == 0)
    {
        realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
        realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
//        realize_prep_lane_change(predictions, "L",info);
    }
    else if(state.compare("PLCR") == 0)
    {
//        realize_prep_lane_change(predictions, "R",info);
    }
}


void Vehicle::realize_constant_speed(Info info)
{
    m_a = 0;
}

void Vehicle::realize_keep_lane(Predictions predictions, Info info, bool DEBUG) {
    if (info.gap_front < 30)
    {
        m_v = info.v_front;
        if(DEBUG){std::cout << " Target speed = " << info.v_front;}
    }
    else{

        m_v = m_target_speed;
    }
    if(DEBUG){std::cout << " Desired speed = " << m_v*TO_MILES_PER_HOUR << std::endl;}
}

void Vehicle::realize_lane_change(Predictions predictions, std::string direction) {
    int delta = 1;
    if (direction.compare("L") == 0)
    {
        delta = -1;
    }
    m_lane += delta;
    int lane = m_lane;
    double s = m_s;
}

Position Vehicle::position_at(double t)
{
    /*
    Predicts state of vehicle in t seconds (assuming zero acceleration)
    */
    double x_new = m_x+m_vx*t;
    double y_new = m_y+m_vy*t;
    double theta = m_yaw;

    std::vector<double> coordinates = getFrenet(x_new, y_new, theta, m_road.maps_x, m_road.maps_y);
    int lane = (int)(coordinates[1]) / (int)m_road.lane_width;

    Position position = {coordinates[0], lane};
    return position;
}

Trajectory Vehicle::generate_trajectory(double time_interval, int horizon /*= 10*/)
{
    Trajectory predicted_trajectory;
    for( int it = 0; it < horizon; ++it)
    {
        Position predicted_position = position_at(it*time_interval);
        predicted_trajectory.push_back(predicted_position);
    }
    return predicted_trajectory;
}
