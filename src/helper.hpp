//
// Created by Marcus Ulmefors on 2017-07-24.
//

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

#endif //PATH_PLANNING_HELPER_H

#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using namespace std;

Eigen::VectorXd jerk_minimize_trajectory(vector<double> start, vector<double> goal, double time);
