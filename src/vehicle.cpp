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
//#include "cost_functions.hpp"


// helper functions
enum string_code {
    CS,             // constant speed
    KL,             // keep lane
    LCL,            // lane change left
    LCR,            // lane change right
    PLCL,           // Prepare lane change left
    PLCR            // Prepare lane change right
};

string_code hashit (State const& inString) {

    if (inString == "CS") return CS;
    if (inString == "KL") return KL;
    if (inString == "LCL") return LCL;
    if (inString == "LCR") return LCR;
    if (inString == "PLCL") return PLCL;
    if (inString == "PLCR") return PLCR;
    return CS;
}

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

    m_state = "CS";
    m_max_acceleration = -1;
    m_target_speed = -1;
    m_goal_lane = -1;
    m_goal_s = -1;
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

    m_lane = (int)m_d % (int)road.lane_width;

    m_state = "CS";
    m_max_acceleration = -1;
    m_target_speed = -1;
    m_goal_lane = -1;
    m_goal_s = -1;
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
        m_v = v;

        // calculate extra vars
        m_vx = v * cos(yaw); // assuming yaw in rads
        m_vy = v * sin(yaw); // assuming yaw in rads
        m_a = 0; // assuming zero acceleration;

        m_lane = (int)m_d % (int)road.lane_width;

        m_state = "CS";
        m_max_acceleration = -1;
        m_target_speed = -1;
        m_goal_lane = -1;
        m_goal_s = -1;
}

Vehicle::~Vehicle() {}

// calculate cost of being in a line

// TODO - Implement this method.
void Vehicle::update_state(Predictions predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
	
	
	
	
    // state machine
//    switch (hashit(m_state))
//    {
//        case CS:
//            state = "KL"; // change state
//            break;
//        case KL:
//            state = "KL"; // change state
//            break;
//        case PLCL:
//            state = "KL"; // change state
//            break;
//        case LCL:
//            state = "KL"; // change state
//            break;
//        case PLCR:
//            state = "KL"; // change state
//            break;
//        case LCR:
//            state = "KL"; // change state
//            break;
//        default:
//            state = "CS"; // change state
//            break;
//    }
	
    m_state = get_next_state(predictions);
}

// reviewed
State Vehicle::get_next_state(Predictions predictions)
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
        std::cout << "state = " << state << ", m_lane = " << m_lane << std::endl;
      Full_Trajectory full_trajectory = trajectory_for_state(state, predictions);
      double cost = calculate_cost(*this, full_trajectory, predictions);
	  costs.push_back({state, cost});
	}
												 
	 // gets the strategy (plan) with minimum cost
    auto it = std::min_element(costs.begin(), costs.end(),
			  [](decltype(costs)::value_type& l, decltype(costs)::value_type& r) -> bool { return l.second < r.second; });
	return (*it).first;
}

// reviewed
Full_Trajectory Vehicle::trajectory_for_state(State& state, Predictions predictions, int horizon /*=5*/)
{
    // remember current state
    Snapshot snapshot = TakeSnapshot();

    // pretend to be in new proposed state
    m_state = state;
    Full_Trajectory full_trajectory;
    full_trajectory.push_back(snapshot);

    Predictions predictions_cpy = predictions; //deep copy of Predictions as we are modifying it
    for (int i=0; i<horizon; ++i)
    {
        restore_state_from_snapshot(snapshot);
        m_state = state;
        realize_state(predictions_cpy);

        assert (0 <= m_lane);
        assert (m_lane < m_road.lanes_available);
        increment((double)i);
        full_trajectory.push_back(TakeSnapshot());

        // need to remove first prediction for each vehicle.
        for (Predictions::iterator itr = predictions_cpy.begin(); itr != predictions_cpy.end(); itr++)
        {
            Prediction prediction = itr->second;
            Trajectory traj_itr = prediction.second;

            // DANGER: this is why we require a complete copy of the Predictions
            // to be passed as the argument and not the actual Predictions otherwise
            // we would be deleting elements
            traj_itr.erase (traj_itr.begin()); // remove first element of vector positions
        }
    }
//   restore state from snapshot
    restore_state_from_snapshot(snapshot);
    return full_trajectory;
}

// reviewed
Snapshot Vehicle::TakeSnapshot() const
{
    Snapshot snapshot;
    snapshot.lane = m_lane;
    snapshot.s = m_s;
    snapshot.v = m_v;
    snapshot.a = m_a;
    snapshot.state = m_state;
    return snapshot;
}

// reviewed
void Vehicle::restore_state_from_snapshot(Snapshot snapshot)
{
    m_lane = snapshot.lane;
    m_s = snapshot.s;
    m_v = snapshot.v;
    m_a = snapshot.a;
    m_state = snapshot.state;
}

// reviewed
void Vehicle::configure(vector<double> road_data)
{
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    m_target_speed = road_data[0];
    m_road.lanes_available = road_data[1];
    m_goal_s = road_data[2];
    m_goal_lane = road_data[3];
    m_max_acceleration = road_data[4]; // this should be vehicle data!
}

// reviewed
std::string Vehicle::display() const
{
	ostringstream oss;

    oss << "s:    " << m_s << "\n";
    oss << "lane: " << m_lane << "\n";
    oss << "v:    " << m_v << "\n";
    oss << "a:    " << m_a << "\n";
    return oss.str();
}

// reviewed
void Vehicle::increment(double dt = 1)
{
    m_s += m_v * dt;
    m_v += m_a * dt;
}

//// reviewed
//Position Vehicle::position_at(double t)
//{
//	/*
//    Predicts state of vehicle in t seconds (assuming constant acceleration)
//    */
//    double s = m_s + m_v * t + m_a * t * t / 2;
//    double v = m_v + m_a * t;
//    Position position = {s, m_lane};
//    return position;
//}

// reviewed
bool Vehicle::collides_with(Vehicle other, double time_t)
{
	/*
    Simple collision detection.
    */
    Position vehicle_1 = position_at(time_t);
    Position vehicle_2 = other.position_at(time_t);
    return (vehicle_1.first == vehicle_2.first) && (abs(vehicle_1.second-vehicle_2.second) <= m_L);
}

// reviewed
Vehicle::collider Vehicle::will_collide_with(Vehicle other, int num_timesteps)
{
    Vehicle::collider collider_temp;
    collider_temp.collision = false;
    collider_temp.time = -1;

    for (int t = 0; t < num_timesteps+1; t++)
    {
        if( collides_with(other, (double)t) )
        {
            collider_temp.collision = true;
            collider_temp.time = (double)t;
            return collider_temp;
        }
    }
    return collider_temp;
}

// reviewed
void Vehicle::realize_state(Predictions predictions)
{
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = m_state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
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
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }
}

// reviewed
void Vehicle::realize_constant_speed()
{
    m_a = 0;
}

// reviewed
double Vehicle::max_accel_for_lane(Predictions predictions, int lane, double s)
{
    double delta_v_til_target = m_target_speed - m_v;
    double max_acc = std::min(m_max_acceleration, delta_v_til_target);

    std::vector<Trajectory> in_front;
    // gets trajectories which are in the given lane and only for vehicles ahead of us
    for (Predictions::iterator itr = predictions.begin(); itr != predictions.end(); ++itr)
    {
        Prediction prediction = itr->second;
//        int& vehicle_id = prediction->first;
        Trajectory trajectory = prediction.second;

        // trajectory: first is s, second is lane
        if((trajectory[0].second == lane) && (trajectory[0].first > s))
        {
            in_front.push_back(trajectory);
        }
    }
    
    // in_front contains trajectories of vehicles "ahead of us" for the given lane
    if(in_front.size() > 0)
    {
        // gets the trajectory closest to our vehicle
        double min_s = 1000.0;// very large number
        Trajectory leading;
        for(int i = 0; i < in_front.size(); i++)
        for (Trajectory& trajectory:in_front)
    	{
            Position position = trajectory[0];
            if((position.first-m_s) < min_s)
    		{
                min_s = (position.first-m_s);
                leading = trajectory;
    		}
    	}
    	
        // leading contains the trajectory of the vehicle right "ahead of us" for the given lane
        double next_pos = leading[1].first; // next distance s of that vehicle
        double my_next_pos = m_s + m_v; // this assumes we have one iteration/second;
        double separation_next = next_pos - my_next_pos;
        double available_room = separation_next - m_preferred_buffer;
        max_acc = std::min(max_acc, available_room); // this assumes we have one iteration/second;
    }
    return max_acc;
}

// reviewed
void Vehicle::realize_keep_lane(Predictions predictions) {
    m_a = max_accel_for_lane(predictions, m_lane, m_s);
}

// reviewed
void Vehicle::realize_lane_change(Predictions predictions, std::string direction) {
    int delta = -1;
    if (direction.compare("R") == 0)
    {
    	delta = 1;
    }
    m_lane += delta;
    int lane = m_lane;
    double s = m_s;
    m_a = max_accel_for_lane(predictions, lane, s);
}

// reviewed
void Vehicle::realize_prep_lane_change(Predictions predictions, std::string direction)
{
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    int target_lane = m_lane + delta;

    vector<Trajectory> at_behind;
    //get vehicles behind us in the target lane
    for(Predictions::iterator itr = predictions.begin(); itr != predictions.end(); ++itr)
    {
        Prediction prediction = itr->second;
        // int vehicle_id = prediction->first;
        Trajectory trajectory = prediction.second;

        if((trajectory[0].second == target_lane) && (trajectory[0].first <= m_s))
        {
            at_behind.push_back(trajectory);
        }
    }
    // at_behind contains a list of all trajectories behind us in the target lane

    if(at_behind.size() > 0)
    {
        double max_s = -1000; // very small number
        Trajectory nearest_behind;
        for(Trajectory trajectory:at_behind)
    	{
            Position position = trajectory[0];
            if(position.first > max_s) // Distance
    		{
                max_s = position.first;
                nearest_behind = trajectory;
    		}
    	}
        //nearest_behind contains the closest trajectory to us in the target lane

        double target_vel = nearest_behind[1].first - nearest_behind[0].first; // this assumes we have one iteration/second;
        double delta_v = m_v - target_vel;
        double delta_s = m_s - nearest_behind[0].first;

        // if we are traveling at a different speed than the car behind us in the target lane
        if(delta_v != 0)
    	{
            double time = -2 * delta_s/delta_v; // gets time
            double a;
            if (time == 0) // this means we are at the same distance s, so we need to keep our acceleration
    		{
                a = m_a;
    		}
    		else
    		{
                a = delta_v/time; // we change our accelaration
    		}
            if(a > m_max_acceleration)
    		{
                a = m_max_acceleration; // limits acceleration to vehicle limits
    		}
            if(a < -m_max_acceleration)
    		{
                a = -m_max_acceleration; // limits acceleration to vehicle limits
    		}
            m_a = a; //sets the acceleration
    	}
        // we are traveling at the same speed (which is a good thing, unless we are in same s (lane distance)!)
        else
    	{
            // we apply a negative accelaration to the car : BUG? this doesnt seem to be the best logic here
            int my_min_acc = max(-m_max_acceleration,-delta_s); // this assumes we have one iteration/second;
            m_a = my_min_acc;
    	}
    }
}

// reviewed
Position Vehicle::position_at(double t)
{
    /*
    Predicts state of vehicle in t seconds (assuming zero acceleration)
    */
    double x_new = m_x+m_vx*t;
    double y_new = m_y+m_vy*t;
    double theta = m_yaw;

    std::vector<double> coordinates = getFrenet(x_new, y_new, theta, m_road.maps_x, m_road.maps_y);
    int lane = (int)(coordinates[1]) % (int)m_road.lane_width;

    Position position = {coordinates[0], lane};
    return position;
}

// reviewed
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
