#ifndef COST_FUNCTIONS
#define COST_FUNCTIONS

#include <cmath>
#include <numeric>
#include <cassert>

#include "helper_functions.h"

// priority levels for costs
#define COLLISION   9999.9
#define DANGER      10
#define COMFORT     10

#define DESIRED_BUFFER 1.5 //1.5s # timesteps
#define PLANNING_HORIZON 2

using Lane = int;

Info get_info(Vehicle vehicle, Predictions predictions, State state, bool DEBUG){

    Info info;

    double my_lane = 0;
    double my_s = 0;
    double my_v = 0;
    double my_a = 0;
    State my_state = "KL";

    double v_front = 100;
    double v_behind = 100;
    double gap_front = 100;
    double gap_behind = 100;

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
            if(DEBUG){
            std::cout << "ev_lane=" << ev_lane <<
                         " my_s=" << my_s <<
                         " my_v=" << my_v <<
                         " my_a=" << my_a <<
                         " my_state=" << my_state <<
                         std::endl;}
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
        info.change_lane = 1;
    }
    else
    {
        info.change_lane = 0;
    }
    info.v_front = v_front;
    info.v_behind = v_behind;
    info.gap_front = gap_front;
    info.gap_behind = gap_behind;
    info.my_speed = my_v;
    info.my_lane = my_lane;
    info.my_s = my_s;
    return info;
}

double calculate_cost(Vehicle vehicle, Predictions predictions, State state, Info info, bool DEBUG=false)
{
    if(DEBUG){std::cout << state << " - v_front=" << info.v_front <<
                 " gap_front=" << info.gap_front <<
                 " v_behind=" << info.v_behind <<
                 " gap_behind=" << info.gap_behind /*<< std::endl*/;}
    double cost = 0.0;

    double min_clearance = 12;
    double v_front_cost = vehicle.m_target_speed*TO_MILES_PER_HOUR - info.v_front;
    double v_behind_cost = 0.0;
    double gap_front_cost = -info.gap_front;
    double gap_behind_cost = 0.0;
    double colliding_cost = 0.0;

    double vel_diff_front = info.my_speed - info.v_front;
    double vel_diff_behind = info.my_speed - info.v_behind;
    if ((info.gap_front - vel_diff_front < min_clearance || (info.gap_behind+vel_diff_behind)< min_clearance)
            && info.change_lane)
    {
        colliding_cost = COLLISION;
    }
    double changing_lane_cost = info.change_lane * COMFORT;

    if (info.gap_front < min_clearance){
        gap_front_cost+= (min_clearance-gap_front_cost)*DANGER;
    }

    cost += v_front_cost;
    cost += v_behind_cost;
    cost += gap_front_cost;
    cost += gap_behind_cost;
    cost += colliding_cost;
    cost += changing_lane_cost;

    if(DEBUG){std::cout << " t=" << cost << std::endl;}
    return cost;
}

#endif
