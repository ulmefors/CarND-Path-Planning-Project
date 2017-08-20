# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


## Design
Waypoints throughout the entire lap are connected using splines. Values `x, y, dx, dy` can thus be found for any value of `s`.
In order to achieve smooth driving path (avoid erroneous linear extrapolation) at the start/finish line the spline is extended by one point in each direction.

```
// helper.cpp extend_waypoints  
map_waypoints.insert(map_waypoints.begin(), map_waypoints[map_waypoints.size()-1]);
map_waypoints.push_back(map_waypoints[1]);
```
[extend_waypoints() in helper.cpp](src/helper.cpp)

In order for `s`-values to be always increasing, the extrapolation values are adjusted with the length of the track.
```
map_waypoints_s[0] -= max_s;
map_waypoints_s[map_waypoints_s.size()-1] += max_s;
```
[main() in main.cpp](src/main.cpp)

### Behavior
Optimal lane is decided based on ego vehicle state and the sensor fusion data of surrounding vehicles. 
```
int car_lane = (int)car_d / lane_width;
vector<double> lane_speeds = helper.GetLaneSpeeds(car_s, car_lane, car_speed, max_speed, sensor_fusion);
LanePlan plan = helper.GetLanePlan(lane_speeds, car_lane, max_speed, car_speed);
int goal_lane = plan.lane;
double goal_speed = plan.speed;
```
[main() in main.cpp](src/main.cpp)

Lane speeds for each lane are calculated based on the sensor fusion data received from the simulator.
The speed of vehicles that are within a defined safety distance from the ego vehicle in each lane will be recorded.
The slowest speed determines the overall lane speed. 
```
// Vehicle is not far ahead of ego vehicle
if (car_s < (ego_s + safety_distance_forward))
{
  // Safety distance behind ego only necessary if vehicle not in same lane
  double rear_buffer = (car_lane == ego_lane) ? 0 : safety_distance_backward;

  // Vehicle close to ego now or in the future
  bool car_close_to_ego_now = car_s > (ego_s - rear_buffer);
  bool car_close_to_ego_future = car_s_future > (ego_s_future - rear_buffer);

  // Limit speed if vehicle is (will be) close to ego
  if (car_close_to_ego_now || car_close_to_ego_future)
  {
    lane_speeds[car_lane] = min(car_speed, lane_speeds[car_lane]);
  }
}
```
[Helper::GetLaneSpeeds() in helper.cpp](src/helper.cpp)


Knowledge of lane speeds will allow for decision of preferred lane.
Since two adjacent lanes can be similar in suitability, the vehicle can make conflicting decisions in two analyses that are very close in time.
For that reason, inertia is introduced in the lane changing algorithm which enforces a minimum time between lane changes.
This minimum time guarantees that an initiated lane change will be completed and thereby eliminates swerving between two lanes.  

```
// New lane change decision cannot be taken within defined time window
int lane_change_inertia {6};

// Determine whether lane change has occured recently
long seconds = std::chrono::system_clock::now().time_since_epoch().count()/1000/1000/1000;
bool allow_lane_change = (seconds - this->lane_change_timestamp) > lane_change_inertia;

// Stay in current lane at current speed if lane change is not allowed
if (!allow_lane_change) return {this->lane, ego_speed, false};
```
[Helper::GetLanePlan() in helper.cpp](src/helper.cpp)

If the ego vehicle current lane speed is limited by traffic, adjacent lanes will be checked for higher available speeds.

```
// If current lane speed is limited
if (lane_speeds[ego_lane] < max_speed)
{
  // Slow down to follow car ahead
  plan.speed = lane_speeds[ego_lane];

  if (ego_lane == center_lane)
  {
    // Change to left or right lane if free
    if (lane_speeds[left_lane] > max_speed)
      plan = LanePlan(left_lane, ego_speed, true);
    else if (lane_speeds[right_lane] > max_speed)
      plan = LanePlan(right_lane, ego_speed, true);
  }
  else
  {
    // Change to center lane if free
    if (lane_speeds[center_lane] > max_speed)
      plan = LanePlan(center_lane, ego_speed, true);
  }
}
```

If lane change occurs, the lane number and time of decision is recorded so that next lane change cannot happen until enough time has elapsed.
```
// Save lane number and time of change to avoid rapid lane change and swerving
if (plan.change)
{
  this->lane_change_timestamp = seconds;
  this->lane = plan.lane;
}
```
[Helper::GetLanePlan() in helper.cpp](src/helper.cpp)


### Trajectory


### Waypoints
Waypoints are located in the center of the road with approximately 40 m average spacing. Each waypoint in the list ([data/highway_map.txt](data/highway_map.txt)) contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.
                                                                                         
The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

 
### Simulator
The Term3 Simulator can be downloaded from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
