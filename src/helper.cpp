#include "helper.hpp"

Eigen::VectorXd JerkMinimizeTrajectory(vector<double> start, vector<double> goal, double time) {

  double alpha_0 = start[0];
  double alpha_1 = start[1];
  double alpha_2 = start[2] / 2;

  // T matrix
  Eigen::MatrixXd T = Eigen::MatrixXd(3, 3);
  T <<  pow(time,3), pow(time,4), pow(time,5),
        3*pow(time,2), 4*pow(time,3), 5*pow(time,4),
        6*time, 12*pow(time,2), 20*pow(time,3);

  // S matrix
  Eigen::VectorXd S = Eigen::VectorXd(3);
  S <<  goal[0] - (start[0] + start[1]*time + 0.5*start[2]*pow(time,2)),
        goal[1] - (start[1] + start[2]*time),
        goal[2] - start[2];

  // Alpha vector
  Eigen::VectorXd alpha_3_4_5 = T.inverse() * S;
  Eigen::VectorXd alpha = Eigen::VectorXd(start.size() + alpha_3_4_5.size());
  alpha << alpha_0, alpha_1, alpha_2, alpha_3_4_5;

  return alpha;
}

double EvaluatePolynomialAtValue(Eigen::VectorXd coeffs, double value) {
  double sum = 0;
  for (size_t i = 0; i < coeffs.size(); i++) {
    sum += coeffs[i] * pow(value, i);
  }
  return sum;
}
