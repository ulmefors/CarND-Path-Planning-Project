#include "helper.hpp"

/**
 * Calculate travel speed of each lane based on ego vehicle state and sensor fusion data of surrounding vehicles
 *
 * @param ego_s ego vehicle s position
 * @param ego_lane  ego vehicle lane
 * @param ego_speed ego vehicle speed
 * @param max_speed maximum speed during unconstrained travel
 * @param vehicles sensor fusion data of surrounding vehicles
 * @return speed for each lane (left, center, right)
 */
vector<double> Helper::GetLaneSpeeds(double ego_s, int ego_lane, double ego_speed, double max_speed, json vehicles)
{
  // Safety distance in front of ego vehicle
  const double safety_distance_forward {50};

  // Safety distance behind ego vehicle to avoid collision when changing lanes
  // Not applicable for same lane
  const double safety_distance_backward {10};

  // Contant number of lanes and lane width
  const int lane_width {4};
  const int num_lanes {3};

  // Time to prediction of future vehicle
  double future_time {0.5};

  // Initialize lane speeds as higher than maximum and constrained if vehicles are present
  vector<double> lane_speeds (num_lanes, max_speed*2);

  // Limit lane speed if a vehicle is within safety margin of ego vehicle
  for (auto const &vehicle : vehicles)
  {
    double car_x_dot = vehicle[3];
    double car_y_dot = vehicle[4];
    double car_s = vehicle[5];
    double car_d = vehicle[6];
    double car_speed = sqrt(car_x_dot*car_x_dot + car_y_dot*car_y_dot);

    // Determine vehicle lane
    int car_lane = (int)car_d / lane_width;

    // Determine future predicted position of vehicle and ego
    double car_s_future = car_s + future_time*car_speed;
    double ego_s_future = ego_s + future_time*ego_speed;

    // Vehicle is not far ahead of ego vehicle
    if (car_s < (ego_s + safety_distance_forward))
    {
      // Safety distance behind ego only necessary if vehicle not in same lane
      double rear_buffer = (car_lane == ego_lane) ? 0 : safety_distance_backward;

      // Vehicle is close to ego now or in the future
      bool car_close_to_ego_now = car_s > (ego_s - rear_buffer);
      bool car_close_to_ego_future = car_s_future > (ego_s_future - rear_buffer);

      // Limit speed if vehicle is (will be) close to ego
      if (car_close_to_ego_now || car_close_to_ego_future)
      {
        lane_speeds[car_lane] = min(car_speed, lane_speeds[car_lane]);
      }
    }
  }

  return lane_speeds;
}

/**
 * Determine whether to stay in current lane or change to adjacent lane
 *
 * @param lane_speeds travel speed of each lane
 * @param ego_lane current lane of ego vehicle
 * @param max_speed maximum speed for unconstrained travel
 * @param ego_speed ego vehicle current speed
 * @return optimal lane and speed
 */
LanePlan Helper::GetLanePlan(vector<double> lane_speeds, int ego_lane, double max_speed, double ego_speed)
{
  // Lane change decision should be firm
  // If two adjacent lanes are similar in suitability the ego vehicle will risk swerving between the lanes

  // New lane change decision cannot be taken within defined time window
  int lane_change_inertia {6};

  // Determine whether lane change has occured recently
  long seconds = std::chrono::system_clock::now().time_since_epoch().count()/1000/1000/1000;
  bool allow_lane_change = (seconds - this->lane_change_timestamp) > lane_change_inertia;

  // Stay in current lane at current speed if lane change is not allowed
  if (!allow_lane_change) return {this->lane, ego_speed, false};

  // Lane numbers
  int left_lane {0};
  int center_lane {1};
  int right_lane {2};

  // Default to staying in same lane with max speed
  LanePlan plan = LanePlan(ego_lane, max_speed, false);

  // Current lane speed is limited
  if (lane_speeds[ego_lane] < max_speed)
  {
    // Slow down to follow car ahead
    plan.speed = lane_speeds[ego_lane];

    if (ego_lane == center_lane)
    {
      if (lane_speeds[left_lane] > max_speed) // Free speed in left lane
        plan = LanePlan(left_lane, ego_speed, true);
      else if (lane_speeds[right_lane] > max_speed) // Free speed in right lane
        plan = LanePlan(right_lane, ego_speed, true);
    }
    else
    {
      if (lane_speeds[center_lane] > max_speed) // Free speed in center lane
        plan = LanePlan(center_lane, ego_speed, true);
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

vector<double> extend_waypoints(vector<double>& map_waypoints)
{
  map_waypoints.insert(map_waypoints.begin(), map_waypoints[map_waypoints.size()-1]);
  map_waypoints.push_back(map_waypoints[1]);
  return map_waypoints;
}