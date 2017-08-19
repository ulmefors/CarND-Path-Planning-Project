#include "helper.hpp"

vector<double> GetLaneSpeeds(double ego_s_pos, int ego_lane, double target_speed, json vehicles)
{

  const double safety_distance_forward = 60.0;
  const double safety_distance_backward = 10.0;
  const int lane_width = 4;
  const int num_lanes = 3;
  vector<double> lane_speeds (num_lanes, target_speed*2);

  for (auto vehicle : vehicles) {
    int id = vehicle[0];
    double car_x_pos = vehicle[1];
    double car_y_pos = vehicle[2];
    double car_x_vel = vehicle[3];
    double car_y_vel = vehicle[4];
    double car_s_pos = vehicle[5];
    double car_d_pos = vehicle[6];
    double car_speed = sqrt(car_x_vel*car_x_vel + car_y_vel*car_y_vel);

    int car_lane = (int)car_d_pos / lane_width;

    if (car_s_pos < (ego_s_pos + safety_distance_forward))
    {
      double rear_buffer = (car_lane == ego_lane) ? 0.0 : safety_distance_backward;

      if (car_s_pos > (ego_s_pos - rear_buffer))
      {
        lane_speeds[car_lane] = min(car_speed, lane_speeds[car_lane]);
      }
    }
  }

  return lane_speeds;
}


Plan GetPlan(vector<double> lane_speeds, int ego_lane, double max_speed, double ego_speed)
{
  Plan plan;
  double goal_speed = max_speed;
  int goal_lane = ego_lane;

  if (lane_speeds[ego_lane] < max_speed)
  {
    // if current lane speed is limited
    goal_speed = lane_speeds[ego_lane];

    if (ego_lane != 1)
    {
      if (lane_speeds[1] > max_speed)
      {
        goal_lane = 1;
        goal_speed = ego_speed;
      }
    }
    else
    {
      if (lane_speeds[0] > max_speed)
      {
        goal_lane = 0;
        goal_speed = ego_speed;
      }
      else if (lane_speeds[2] > max_speed)
      {
        goal_lane = 2;
        goal_speed = ego_speed;
      }
    }
  }

  plan.lane = goal_lane;
  plan.speed = goal_speed;

  return plan;
}