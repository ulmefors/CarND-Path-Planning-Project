#include "helper.hpp"


vector<double> Helper::GetLaneSpeeds(double ego_s, int ego_lane, double ego_speed, double target_speed, json vehicles)
{

  const double safety_distance_forward = 50.0;
  const double safety_distance_backward = 10.0;
  const int lane_width = 4;
  const int num_lanes = 3;
  double future_time {0.5};
  vector<double> lane_speeds (num_lanes, target_speed*2);

  for (auto vehicle : vehicles) {
    int id = vehicle[0];
    double car_x = vehicle[1];
    double car_y = vehicle[2];
    double car_x_dot = vehicle[3];
    double car_y_dot = vehicle[4];
    double car_s = vehicle[5];
    double car_d = vehicle[6];
    double car_speed = sqrt(car_x_dot*car_x_dot + car_y_dot*car_y_dot);

    int car_lane = (int)car_d / lane_width;
    double car_s_future = car_s + future_time*car_speed;
    double ego_s_future = ego_s + future_time*ego_speed;

    if (car_s < (ego_s + safety_distance_forward))
    {
      double rear_buffer = (car_lane == ego_lane) ? 0.0 : safety_distance_backward;

      bool car_close_to_ego = car_s > (ego_s - rear_buffer);
      bool car_close_to_ego_future = car_s_future > (ego_s_future - rear_buffer);
      if (car_close_to_ego || car_close_to_ego_future)
      {
        lane_speeds[car_lane] = min(car_speed, lane_speeds[car_lane]);
      }
    }
  }

  return lane_speeds;
}


Plan Helper::GetPlan(vector<double> lane_speeds, int ego_lane, double max_speed, double ego_speed)
{
  // Stand by lane change decision to avoid swerving
  int lane_change_inertia {6}; // Minimum number of seconds between lane changes
  long seconds = std::chrono::system_clock::now().time_since_epoch().count() / 1000/1000/1000;
  bool allow_lane_change = (seconds - this->lane_change_timestamp) > lane_change_inertia;
  if (!allow_lane_change)
  {
    return {this->lane, ego_speed, false};
  }

  // Lane numbers
  int left_lane {0};
  int center_lane {1};
  int right_lane {2};

  // Default to staying in same lane with max speed
  Plan plan = Plan(ego_lane, max_speed, false);

  // Current lane speed is limited
  if (lane_speeds[ego_lane] < max_speed)
  {
    // Slow down to follow car ahead
    plan.speed = lane_speeds[ego_lane];

    if (ego_lane == center_lane)
    {
      if (lane_speeds[left_lane] > max_speed) // Free speed in left lane
        plan = Plan(left_lane, ego_speed, true);
      else if (lane_speeds[right_lane] > max_speed) // Free speed in right lane
        plan = Plan(right_lane, ego_speed, true);
    }
    else
    {
      if (lane_speeds[center_lane] > max_speed) // Free speed in center lane
        plan = Plan(center_lane, ego_speed, true);
    }
  }

  // Update last lane change
  if (plan.change)
  {
    this->lane_change_timestamp = seconds;
    this->lane = plan.lane;
  }

  return plan;
}
