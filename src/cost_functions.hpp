#ifndef COST_FUNCTIONS
#define COST_FUNCTIONS

#include <cmath>
#include <numeric>
#include <cassert>

#include "helper_functions.h"

// priority levels for costs
#define COLLISION   10000//std::pow(10, 6)
#define DANGER      100//std::pow(10, 5)
#define REACH_GOAL  0//std::pow(10, 5)
#define COMFORT     200//std::pow(10, 5)
#define EFF_VEL     200
#define EFF_ACCEL  0//std::pow(10, 2)

#define DESIRED_BUFFER 1.5 //1.5s # timesteps
#define PLANNING_HORIZON 2

using Lane = int;

//#define DEBUG = false;
// DEBUG = True

double change_lane_cost(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, TrajectoryData data)
{
    // Penalizes any change of lane specially at high speed
    double cost =0;
    if (hashit(data.state)!=KL)
    {
        cost = COMFORT*(abs(vehicle.m_v)/vehicle.m_target_speed);
    }
    if (DEBUG_COST){std::cout << " conf=" <<cost;}
    return cost;
}

double distance_from_goal_lane(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, TrajectoryData data)
{
        double distance = abs(data.end_distance_to_goal);
        distance = max(distance,1.0);
        double time_to_goal = float(distance) / data.avg_speed;
        int lanes = data.end_lanes_from_goal;
        double multiplier = float(5 * lanes / time_to_goal);
        double cost = multiplier * REACH_GOAL;
        return cost;
}

double inefficiency_cost(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, TrajectoryData data)
{
        double speed = data.avg_speed;
        double target_speed = vehicle.m_target_speed;
        double diff = target_speed - speed;
        double pct = float(diff) / target_speed;
        if (DEBUG_COST){std::cout << " v=" << pct * EFF_VEL ;}
        return pct * EFF_VEL;
}

double accel_cost(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, TrajectoryData data)
{
        double accel = data.max_acceleration;
        if (DEBUG_COST){std::cout << " a=" << (-accel) * EFF_ACCEL ;}
        return (-accel) * EFF_ACCEL;
}

double collision_cost(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, TrajectoryData data)
{
    double closest = data.closest_approach; //closest approach is the distance to the closest vehicle
    double cost = 0;
    double min_gap = 10;
    if (abs(closest)<abs(min_gap) && hashit(data.state)!=KL)
    {
        cost = COLLISION;
        std::cout << " state=" << data.state << " closest =" << closest;
    }

//    if (data.collides.first)
//    {
//        double time_til_collision = data.collides.second;
//        double exponent = std::pow((float(time_til_collision)),2);
//        double mult = exp(-exponent);
//        return mult * COLLISION;
//    }
    return cost;
}

double buffer_cost(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, TrajectoryData data)
{
    double closest = data.closest_approach; //closest approach is the distance to the closest vehicle
    double cost = 0;
    //    if (closest == 0)
//    {
//        return 10 * DANGER;
//    }

//    double timesteps_away = closest / data.avg_speed;
//    if (timesteps_away > DESIRED_BUFFER)
//    {
//        return 0.0;
//    }

//    double multiplier = 1.0 - std::pow((timesteps_away/DESIRED_BUFFER),2.0);
    if (closest != 0){
        cost = DANGER/closest;
    }
    else{
        cost = DANGER;
    }

    if (!DEBUG_COST){std::cout << " b=" << cost;}
    return cost;
}

TrajectoryData get_helper_data(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, State state); // forward declaration


NewTrajectoryData get_relevant_data(Vehicle vehicle, Predictions predictions, State state){

    NewTrajectoryData relevant_data;

    double my_lane = 0;
    double my_s = 0;
    double my_v = 0;
    double my_a = 0;
    State my_state = "KL";

    double v_front = 9999;
    double v_behind = 9999;
    double gap_front = 9999;
    double gap_behind = 9999;

    double ev_lane = 0; // lane being evaluated
    for (Prediction& prediction : predictions)
    {
        int v_id = std::get<0>(prediction);
        Snapshot snapshot = std::get<1>(prediction);
        if (v_id == MY_ID){         // hopefully this is the first vehicle in the list
            my_lane = snapshot.lane;
            my_s = snapshot.s;
            my_v = snapshot.v;
            my_a = snapshot.a;
            my_state = snapshot.state;

            switch (hashit(state)){
            case KL:
                ev_lane = my_lane;
                break;
            case LCL:
                ev_lane = my_lane - 1;
                break;
            case LCR:
                ev_lane = my_lane + 1;
                break;
            default:
                ev_lane = my_lane;
                break;
            }
            std::cout << "ev_lane=" << ev_lane <<
                         " my_s=" << my_s <<
                         " my_v=" << my_v <<
                         " my_a=" << my_a <<
                         " my_state=" << my_state <<
                         std::endl;
            continue;
        }
        int car_lane = snapshot.lane;
        double car_s = snapshot.s;
        double car_v = snapshot.v;
        double car_a = snapshot.a;
        State car_state = snapshot.state;

        if (car_lane != ev_lane) // we only care for the evaluated lane
        {
            continue;
        }

        if (car_s > my_s) // vehicle in front of me in the given lane
        {
            if ((car_s-my_s)<gap_front){
                gap_front = (car_s-my_s);
                v_front = car_v;
            }

        }
        else
        {
            if ((my_s-car_s)<gap_behind){
                gap_behind = (my_s-car_s);
                v_behind = car_v;
            }
        }
    }
    if (ev_lane!=my_lane){
        relevant_data.change_lane = 1;
    }
    else
    {
        relevant_data.change_lane = 0;
    }
    relevant_data.v_front = v_front;
    relevant_data.v_behind = v_behind;
    relevant_data.gap_front = gap_front;
    relevant_data.gap_behind = gap_behind;
    return relevant_data;
}

double calculate_cost(Vehicle vehicle, Predictions predictions, State state, bool verbose=false)
{
    NewTrajectoryData data = get_relevant_data(vehicle, predictions, state);

    std::cout << state << " - v_front=" << data.v_front <<
                 " gap_front=" << data.gap_front <<
                 " v_behind=" << data.v_behind <<
                 " gap_behind=" << data.gap_behind /*<< std::endl*/;
    double cost = 0.0;

    double min_change_gap = 30;
    double v_front_cost = vehicle.m_target_speed*TO_MILES_PER_HOUR - data.v_front;
    double v_behind_cost = 0.0;
    double gap_front_cost = -data.gap_front;
    double gap_behind_cost = 0.0;
    double colliding_cost = 0.0;

    if ((data.gap_front+data.gap_behind)<min_change_gap)
    {
        colliding_cost = 9999.9;
    }
    double changing_lane_cost = data.change_lane * 100.0;

    cost += v_front_cost;
    cost += v_behind_cost;
    cost += gap_front_cost;
    cost += gap_behind_cost;
    cost += colliding_cost;
    cost += changing_lane_cost;
////    std::cout << " avg speed = " << trajectory_data.avg_speed << " ";
//    trajectory_data.max_acceleration = vehicle.max_accel_for_lane(predictions,trajectory[1].lane, trajectory[1].s);
////    std::cout << " max_accel = " << trajectory_data.max_acceleration << " ";

//    double max_vel = vehicle.max_vel_for_lane(predictions,trajectory[1].lane, trajectory[1].s);
////    std::cout << "max_vel = " << max_vel ;
//    trajectory_data.avg_speed = max_vel;
//    cost += accel_cost(vehicle, trajectory, predictions, trajectory_data);
////    cost += distance_from_goal_lane(vehicle, trajectory, predictions, trajectory_data);
//    cost += inefficiency_cost(vehicle, trajectory, predictions, trajectory_data);
//    cost += collision_cost(vehicle, trajectory, predictions, trajectory_data);
//    cost += buffer_cost(vehicle, trajectory, predictions, trajectory_data);
//    cost += change_lane_cost(vehicle, trajectory, predictions, trajectory_data);

    std::cout << " t=" << cost << std::endl;
    return cost;
}

bool check_collision(Snapshot snapshot, double s_previous, double s_now); // forward declaration
Predictions filter_predictions_by_lane(Predictions predictions, Lane lane); // forward declaration

TrajectoryData get_helper_data(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, State state)
{

    Snapshot current_snapshot = trajectory[0];
    Snapshot first = trajectory[1];
    Snapshot last = trajectory.back();

    double end_distance_to_goal = vehicle.m_goal_s - last.s;
    int end_lanes_from_goal = abs(vehicle.m_goal_lane - last.lane);
    double dt = float(trajectory.size());
    int proposed_lane = first.lane;
    //double avg_speed = (last.s - current_snapshot.s) / dt;

    // initialize a bunch of variables
    std::vector<double> accels;
    int closest_approach = 999999;
    std::pair<bool,int> collides = {false,-1};
    Snapshot last_snap = trajectory[0];
    Predictions filtered_pred = filter_predictions_by_lane(predictions, proposed_lane);


    double sum_speeds_lane = 0;
    int count = 0;
    for (auto prediction : predictions)
    {
        Trajectory list_positions = std::get<2>(prediction);
        sum_speeds_lane += list_positions[1].first-list_positions[0].first;
        ++count;
    }
    double avg_speed = sum_speeds_lane/count;
    // for i, snapshot in enumerate(trajectory[1:PLANNING_HORIZON+1], 1): // enumerate starts counting from 1

    // the for cycle starts at 1, as 0 is the current position
    int i=1;
//    for (Snapshot snapshot:(trajectory.begin()+1))
    for (std::vector<Snapshot>::iterator itr = trajectory.begin()+1; itr != trajectory.end(); ++itr)
    {
        Snapshot snapshot = *itr;
        int lane = snapshot.lane;
        double s = snapshot.s;
        double v = snapshot.v;
        double a = snapshot.a;

        accels.push_back(a);

        for (Prediction prediction : predictions)
        {
            int vehicle_id = std::get<0>(prediction);
            Trajectory list_positions = std::get<2>(prediction);

            //using Prediction = std::map<int,std::vector < std::pair<double s, int lane> > >;

            Position position = list_positions[i]; // position is the pair: (s,lane)
            Position last_position = list_positions[i-1];
            bool vehicle_collides = check_collision(snapshot, last_position.first, position.first);
            if (vehicle_collides){
                    collides.first = true;
                    collides.second = i;}
            double dist = fabs(position.first - s);
            if (dist < closest_approach){
                    closest_approach = dist;}
        }
        last_snap = snapshot;
        ++i;
    }

    double max_accel = std::fabs(*std::max_element(accels.begin(), accels.end(),[](double lhs, double rhs) {
            return std::fabs(lhs) < std::fabs(rhs);}));

    std::vector<double> rms_accels;
    for (double i:accels)
    {
        rms_accels.push_back(i*i);
    }

    double num_accels = rms_accels.size();
    double sum_of_elems = std::accumulate(rms_accels.begin(), rms_accels.end(), 0.0f);
    assert(num_accels!=0.0);
    double rms_acceleration = sum_of_elems / num_accels;

    TrajectoryData trajectory_data;
    trajectory_data.state = state;
    trajectory_data.proposed_lane = proposed_lane;
    trajectory_data.avg_speed = avg_speed;
    trajectory_data.max_acceleration = max_accel;
    trajectory_data.rms_acceleration = rms_acceleration;
    trajectory_data.closest_approach = closest_approach;
    trajectory_data.end_distance_to_goal = end_distance_to_goal;
    trajectory_data.end_lanes_from_goal = end_lanes_from_goal;
    trajectory_data.collides = collides;
    return trajectory_data;
}


bool check_collision(Snapshot snapshot, double s_previous, double s_now)
{
    double s = snapshot.s;
    double v = snapshot.v;
    double v_target = s_now - s_previous;
    if (s_previous < s){
        if (s_now >= s){
                return true;}
        else{
                return false;}
    }

    if (s_previous > s){
        if (s_now <= s){
                return true;}
        else{
                return false;}
    }

    if (s_previous == s){
        if (v_target > v){
                return false;}
        else{
                return true;}
    }
    //raise ValueError
}

// get predictions by lane
// @return vector of predictions
Predictions filter_predictions_by_lane(Predictions predictions, Lane lane)
{
    Predictions filtered;
    for (Prediction prediction:predictions) {
        Vehicle_ID vehicle_id = std::get<0>(prediction);
        Trajectory trajectory = std::get<2>(prediction);// points to the trajectory
        //if the lane from the first location in the vector of locations == lane
        if (((trajectory[0]).second == lane) && (vehicle_id != -1))
        {
            filtered.push_back(prediction);
        }
    }
    return filtered;
}

#endif
