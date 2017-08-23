#ifndef COST_FUNCTIONS
#define COST_FUNCTIONS

#include <cmath>
#include <numeric>
#include <cassert>

// priority levels for costs
#define COLLISION   std::pow(10, 6)
#define DANGER      std::pow(10, 5)
#define REACH_GOAL  std::pow(10, 5)
#define COMFORT     std::pow(10, 4)
#define EFFICIENCY  std::pow(10, 2)

#define DESIRED_BUFFER 1.5 //s # timesteps
#define PLANNING_HORIZON 2

using Lane = int;

//#define DEBUG = false;
// DEBUG = True

double change_lane_cost(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, TrajectoryData data)
{
        /*
        Penalizes lane changes AWAY from the goal lane and rewards
        lane changes TOWARDS the goal lane.
        */
        int proposed_lanes = data.end_lanes_from_goal;
        int cur_lanes = trajectory[0].lane; // get lane
        double cost = 0;
        if (proposed_lanes > cur_lanes){
                cost = COMFORT;}
        if (proposed_lanes < cur_lanes){
                cost = -COMFORT;}
        if (cost != 0){
                std::cout << "!! \n \ncost for lane change is " << cost << std::endl;}
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
        double multiplier = std::pow(pct,2.0);
        return multiplier * EFFICIENCY;
}

double collision_cost(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, TrajectoryData data)
{
    if (data.collides.first)
    {
        double time_til_collision = data.collides.second;
        double exponent = std::pow((float(time_til_collision)),2);
        double mult = exp(-exponent);
        return mult * COLLISION;
    }
    return 0.0;
}

double buffer_cost(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, TrajectoryData data)
{
    double closest = data.closest_approach;
    if (closest == 0)
    {
        return 10 * DANGER;
    }

    double timesteps_away = closest / data.avg_speed;
    if (timesteps_away > DESIRED_BUFFER)
    {
        return 0.0;
    }

    double multiplier = 1.0 - std::pow((timesteps_away/DESIRED_BUFFER),2.0);
    return multiplier * DANGER;
}

TrajectoryData get_helper_data(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions); // forward declaration

double calculate_cost(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions, bool verbose=false)
{
    TrajectoryData trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    double cost = 0.0;

    cost += distance_from_goal_lane(vehicle, trajectory, predictions, trajectory_data);
    cost += inefficiency_cost(vehicle, trajectory, predictions, trajectory_data);
    cost += collision_cost(vehicle, trajectory, predictions, trajectory_data);
    cost += buffer_cost(vehicle, trajectory, predictions, trajectory_data);
    cost += change_lane_cost(vehicle, trajectory, predictions, trajectory_data);
    return cost;
}

bool check_collision(Snapshot snapshot, double s_previous, double s_now); // forward declaration
Predictions filter_predictions_by_lane(Predictions predictions, Lane lane); // forward declaration

TrajectoryData get_helper_data(Vehicle vehicle, Full_Trajectory trajectory, Predictions predictions)
{

    Snapshot current_snapshot = trajectory[0];
    Snapshot first = trajectory[1];
    Snapshot last = trajectory.back();

    double end_distance_to_goal = vehicle.m_goal_s - last.s;
    int end_lanes_from_goal = abs(vehicle.m_goal_lane - last.lane);
    double dt = float(trajectory.size());
    int proposed_lane = first.lane;
    double avg_speed = (last.s - current_snapshot.s) / dt;

    // initialize a bunch of variables
    std::vector<double> accels;
    int closest_approach = 999999;
    std::pair<bool,int> collides = {false,-1};
    Snapshot last_snap = trajectory[0];
    Predictions filtered_pred = filter_predictions_by_lane(predictions, proposed_lane);

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

        for (Predictions::iterator pred_itr = filtered_pred.begin(); pred_itr != filtered_pred.end(); ++pred_itr)
        {
            Prediction prediction = pred_itr->second;

            int vehicle_id = prediction.first;
            Trajectory list_positions = prediction.second;

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
    for (std::map<Unique_ID,Prediction>::iterator itr = predictions.begin(); itr != predictions.end(); ++itr) {
        Vehicle_ID vehicle_id = itr->first;
        Prediction ptr_prediction = itr->second; // ptr_prediction points to the prediction
        Trajectory ptr_trajectory = ptr_prediction.second; // points to the trajectory
        //if the lane from the first location in the vector of locations == lane
        if (((ptr_trajectory[0]).second == lane) && (vehicle_id != -1))
        {
            filtered.insert(std::make_pair(vehicle_id,ptr_prediction));
        }
    }
    return filtered;
}

#endif
