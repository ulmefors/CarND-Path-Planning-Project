//
// Created by Marcus Ulmefors on 2017-07-24.
//

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

#endif //PATH_PLANNING_HELPER_H

#include <vector>
#include <math.h>
#include "json.hpp"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using json = nlohmann::json;

Eigen::VectorXd JerkMinimizeTrajectory(vector<double> start, vector<double> goal, double time);

double EvaluatePolynomialAtValue(Eigen::VectorXd coeffs, double value);

vector <double> GetLaneSpeeds(double ego_s_pos, double ego_d_pos, double target_speed, json vehicles);
