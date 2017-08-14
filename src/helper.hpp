//
// Created by Marcus Ulmefors on 2017-07-24.
//

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

#endif //PATH_PLANNING_HELPER_H

#include <vector>
#include <cmath>
#include "json.hpp"
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

using namespace std;

class Helper {

public:
    // Constructors
    Helper() = default;
    Helper(const Helper& other) = default;
    Helper& operator=(const Helper& other) = default;
    Helper(Helper&& other) = default;
    Helper& operator=(Helper&& other) = default;
    ~Helper() = default;

    // Points sent to simulator
    vector<double> x_vals;
    vector<double> y_vals;
    vector<double> s_vals;
    vector<double> d_vals;

    // Splines
    tk::spline x;
    tk::spline y;
    tk::spline dx;
    tk::spline dy;
};

using namespace std;
using json = nlohmann::json;

Eigen::VectorXd JerkMinimizeTrajectory(vector<double> start, vector<double> goal, double time);

double EvaluatePolynomialAtValue(Eigen::VectorXd coeffs, double value);

vector <double> GetLaneSpeeds(double ego_s_pos, double ego_d_pos, double target_speed, json vehicles);
