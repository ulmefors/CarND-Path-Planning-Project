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

using json = nlohmann::json;
using namespace std;

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
};

struct Plan
{
    int lane;
    double speed;
};

vector<double> GetLaneSpeeds(double ego_s_pos, int ego_lane, double target_speed, json vehicles);

Plan GetPlan(vector<double> lane_speeds, int ego_lane, double max_speed, double ego_speed);
