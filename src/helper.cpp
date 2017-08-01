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

vector<double> GetLaneSpeeds(double ego_s_pos, double ego_d_pos, double ego_speed, json vehicles) {

  const double safety_distance_forward = ego_speed * 2;
  const double safety_distance_backward = 5.0;
  const int lane_width = 4;

  const size_t num_lanes = 3;
  const double standard_speed = 100.0; // Large number
  vector<double> lane_speeds;
  lane_speeds.assign(num_lanes, standard_speed);

  for (int i = 0; i < vehicles.size(); i++) {
    json vehicle = vehicles[i];
    int id = vehicle[0];
    double car_x_pos = vehicle[1];
    double car_y_pos = vehicle[2];
    double car_x_vel = vehicle[3];
    double car_y_vel = vehicle[4];
    double car_s_pos = vehicle[5];
    double car_d_pos = vehicle[6];
    double car_speed = sqrt(car_x_vel*car_x_vel + car_y_vel*car_y_vel);


    int car_lane = (int)car_d_pos / lane_width;
    int ego_lane = (int)ego_d_pos / lane_width;

    /*
    if (car_lane == ego_lane) {
      if (car_s_pos < (ego_s_pos + safety_distance_forward)) {
        lane_speeds[car_lane] = min(car_speed, lane_speeds[car_lane]);
      }
    } else {
      if (car_s_pos < (ego_s_pos + safety_distance_forward) && car_s_pos > (ego_s_pos - safety_distance_backward)) {
        lane_speeds[car_lane] = min(car_speed, lane_speeds[car_lane]);
      }
    }
     */

    if (car_s_pos < (ego_s_pos + safety_distance_forward) && car_s_pos > ego_s_pos) {
      lane_speeds[car_lane] = min(car_speed, lane_speeds[car_lane]);
    }

  }

  return lane_speeds;
}
