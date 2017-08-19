//
// Created by Marcus Ulmefors on 2017-07-24.
//

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

#endif //PATH_PLANNING_HELPER_H

#include <vector>
#include <cmath>
#include "json.hpp"
#include "spline.h"
#include <chrono>

using json = nlohmann::json;
using namespace std;

struct Plan
{
    Plan(int l, double s, bool c) : lane{l}, speed{s}, change{c} {}
    int lane;
    double speed;
    bool change;
};

class Helper
{
public:
    // Constructors
    Helper() = default;
    Helper(const Helper& other) = default;
    Helper& operator=(const Helper& other) = default;
    Helper(Helper&& other) = default;
    Helper& operator=(Helper&& other) = default;
    ~Helper() = default;

    // Reference speed
    double ref_speed {0};

    // Splines
    tk::spline x;
    tk::spline y;
    tk::spline dx;
    tk::spline dy;

    vector<double> GetLaneSpeeds(double ego_s, int ego_lane, double ego_speed, double target_speed, json vehicles);

    Plan GetPlan(vector<double> lane_speeds, int ego_lane, double max_speed, double ego_speed);

private:
    long lane_change_timestamp {0};
    int lane {-1};
};
