#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helper.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/**
 * @param x vehicle position
 * @param y vehicle position
 * @param maps_x waypoint positions
 * @param maps_y waypoint positions
 * @return index closest waypoint
 */
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  assert(maps_x.size() == maps_y.size());
  int nb_waypoints = maps_x.size();

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

  // Closest waypoint is behind ego vehicle if outside +/- 45°
	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

  // Return first waypoint if trying to access index out of bounds
	return closestWaypoint % nb_waypoints;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// Calculate acceleration based on series of positions
vector<vector<double>> CalculateState(int pos, json previous_path_x, json previous_path_y)
{
  double timestep = 0.02;
  double car_x = previous_path_x[pos];
  double car_y = previous_path_y[pos];

  // Speeds in first two steps, and corresponding acceleration
  // double car_acc;
  double car_acc_x;
  double car_acc_y;

  double speed_before_x = (double(previous_path_x[pos]) - double(previous_path_x[pos-1])) / timestep;
  double speed_after_x = (double(previous_path_x[pos+1]) - double(previous_path_x[pos])) / timestep;
  double speed_before_y = (double(previous_path_y[pos]) - double(previous_path_y[pos-1])) / timestep;
  double speed_after_y = (double(previous_path_y[pos+1]) - double(previous_path_y[pos])) / timestep;
  car_acc_x = (speed_after_x - speed_before_x)/timestep;
  car_acc_y = (speed_after_y - speed_before_y)/timestep;

  /*
  double speed_before = distance(previous_path_x[pos-1], previous_path_y[pos-1],
                                 previous_path_x[pos], previous_path_y[pos]) / timestep;
  double speed_after = distance(previous_path_x[pos], previous_path_y[pos],
                                previous_path_x[pos+1], previous_path_y[pos+1]) / timestep;
  car_acc = (speed_after - speed_before)/timestep;
  */

  return {
      {car_x, (speed_before_x + speed_after_x)/2, car_acc_x},
      {car_y, (speed_before_y + speed_after_y)/2, car_acc_y}
  };
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // Create splines
  tk::spline x, y, dx, dy;
  x.set_points(map_waypoints_s, map_waypoints_x);
  y.set_points(map_waypoints_s, map_waypoints_y);
  dx.set_points(map_waypoints_s, map_waypoints_dx);
  dy.set_points(map_waypoints_s, map_waypoints_dy);

  Helper helper = Helper();
  helper.x = x;
  helper.y = y;
  helper.dx = dx;
  helper.dy = dy;

  h.onMessage([&helper, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Constants
          const double max_speed = 20; // 10 m/s = 22.37 mph
          const double timestep = 0.02; // 0.02 second update
          const int lane_width = 4; // 4 m lane
          const double limit_mean_acc = 1.0;
          const double speed_safety_factor = 0.95;

          // Ego vehicle localization data (global coordinates)
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed_mph = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          json sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          vector<double> next_s_vals;
          vector<double> next_d_vals;

          tk::spline x_spline = helper.x;
          tk::spline y_spline = helper.y;
          tk::spline dx_spline = helper.dx;
          tk::spline dy_spline = helper.dy;

          cout << "s: " << car_s << " d: " << car_d << " x: " << car_x << " y: " << car_y << endl;

          // Ego vehicle derived state
          double car_yaw_rad = deg2rad(car_yaw);
          double car_speed = car_speed_mph * 0.44704; // mph to mps
          double car_speed_x = car_speed * cos(car_yaw_rad);
          double car_speed_y = car_speed * sin(car_yaw_rad);

          int ego_lane = (int)car_d / lane_width;

          // Get lane speeds
          int goal_lane = ego_lane; // Default to current lane
          double goal_speed = max_speed;

          // Choose lane
          vector<double> lane_speeds = GetLaneSpeeds(car_s, car_d, max_speed, sensor_fusion);

          if (lane_speeds[ego_lane] < max_speed) {
            // if current lane speed is limited
            goal_speed = lane_speeds[ego_lane] * speed_safety_factor;

            if (ego_lane != 1) {
              if (lane_speeds[1] > max_speed) {
                goal_lane = 1;
                goal_speed = car_speed;
              }
            }
            else {
              if (lane_speeds[0] > max_speed) {
                goal_lane = 0;
                goal_speed = car_speed;
              }
              else if (lane_speeds[2] > max_speed) {
                goal_lane = 2;
                goal_speed = car_speed;
              }
            }
          } else {
            // if current lane speed is not limited
            goal_speed = max_speed;
          }

          /*
          // Set goal position
          const double goal_d = ((double)goal_lane + 1./2.) * (double)lane_width;
          const double goal_acc = 0;

          // Get accelerations
          int position = 0;
          double car_acc_x = 0;
          double car_acc_y = 0;
          if (previous_path_x.size() > position + 2) {
            vector<vector<double>> state = CalculateState(position, previous_path_x, previous_path_y, timestep);
            vector<double> state_x = state[0];
            vector<double> state_y = state[1];
            car_acc_x = state_x[2];
            car_acc_y = state_y[2];
          }

          // Position ahead
          int pos_ahead = 10;
          vector<double> state_ahead_x = {0, 0, 0};
          vector<double> state_ahead_y = {0, 0, 0};
          if (previous_path_x.size() > position + 2) {
            vector<vector<double>> state = CalculateState(pos_ahead, previous_path_x, previous_path_y, timestep);
            state_ahead_x = state[0];
            state_ahead_y = state[1];
          }

          // XY coordinates
          double goal_x;
          double goal_y;
          double goal_s;

          // Find next waypoint
          int car_waypoint = NextWaypoint(car_x + car_speed_x * 0.2, car_y + car_speed_y * 0.2, car_yaw_rad, map_waypoints_x, map_waypoints_y);
          int waypoint = car_waypoint;
          goal_x = map_waypoints_x[waypoint];
          goal_y = map_waypoints_y[waypoint];
          goal_s = map_waypoints_s[waypoint];
          double dx = map_waypoints_dx[waypoint];
          double dy = map_waypoints_dy[waypoint];
          double goal_yaw_rad = -atan2(-dx, -dy);
          if (goal_yaw_rad > pi()) goal_yaw_rad -= 2*pi();

          // TODO: Limit goal speed based on current speed
          double goal_speed_x = goal_speed * cos(goal_yaw_rad);
          double goal_speed_y = goal_speed * sin(goal_yaw_rad);
          //double goal_speed_x = min(goal_speed, car_speed + time * limit_mean_acc) * cos(goal_yaw_rad);
          //double goal_speed_y = min(goal_speed, car_speed + time * limit_mean_acc) * sin(goal_yaw_rad);

          double travel_distance = goal_s - car_s;
          // Wrap around max_s = 6945.554
          if (travel_distance < 0) travel_distance += 6945.554;
          double travel_time = travel_distance / ((goal_speed + car_speed) * 0.5 );
          double travel_steps = travel_time / timestep;


          // cout << waypoint << " " << car_x << " " << goal_x << " " << car_y << " " << goal_y << " " << car_s << " " << goal_s << " " << travel_distance << endl;
          // cout << car_yaw_rad << " " << goal_yaw_rad << " " << car_speed_x << " " << goal_speed_x << " " << car_speed_y << " " << goal_speed_y << endl;
          // cout << endl;


          // Adjustment in D
          goal_x += dx * goal_d;
          goal_y += dy * goal_d;

          vector<double> x_start = {car_x, car_speed_x, car_acc_x};
          vector<double> x_goal = {goal_x, goal_speed_x, goal_acc};

          vector<double> y_start = {car_y, car_speed_y, car_acc_y};
          vector<double> y_goal = {goal_y, goal_speed_y, goal_acc};

          Eigen::VectorXd x_ahead_alpha = JerkMinimizeTrajectory(state_ahead_x, x_goal, travel_time - 0.2);
          Eigen::VectorXd y_ahead_alpha = JerkMinimizeTrajectory(state_ahead_y, y_goal, travel_time - 0.2);

          Eigen::VectorXd x_alpha = JerkMinimizeTrajectory(x_start, x_goal, travel_time);
          Eigen::VectorXd y_alpha = JerkMinimizeTrajectory(y_start, y_goal, travel_time);
          */

          double target_speed = min(lane_speeds[1], max_speed * 0.9);
          double step = target_speed * timestep;
          double d_val = 6.0;
          int horizon = 100;
          int num_prev_path_points = previous_path_x.size();
          double s_start = -1;
          double last_x = -1;
          double last_y = -1;

          int num_new_steps = horizon - num_prev_path_points;

          if (!previous_path_x.empty()) {
            for (int i = 0; i < num_prev_path_points; ++i) {
              // TODO: Confirm this line
              double s = helper.s_vals[i + num_new_steps];
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
              next_s_vals.push_back(s);
            }
            last_x = previous_path_x[num_prev_path_points-1];
            last_y = previous_path_y[num_prev_path_points-1];

            double s_range = 2.0;
            int half_range = 100;
            double min_distance = 1000; // large number
            double s_closest = 0;
            double s_approx = end_path_s;
            //double s_approx = helper.s_vals[num_prev_path_points -1 + num_new_steps];

            cout << "s_approx: " << s_approx << " x(s_approx): " << x_spline(s_approx) + d_val*dx_spline(s_approx) << " x_prev: " << last_x << endl;

            for(int i = 0; i <= 2*half_range; ++i) {
              double s = s_approx + ((double)(i-half_range))*s_range/(double)half_range;
              double x = x_spline(s) + d_val*dx_spline(s);
              double y = y_spline(s) + d_val*dy_spline(s);
              double dist = distance(x, y, last_x, last_y);
              if (dist < min_distance) {
                min_distance = dist;
                s_closest = s;
              }
            }
            s_start = s_closest;
          }
          else {
            s_start = car_s - step;
            cout << "car_s: " << car_s << endl;
          }

          if (previous_path_x.empty()) {

            Eigen::VectorXd s_alpha = JerkMinimizeTrajectory({s_start, 0, 0},
                                                             {s_start + 1, 1.5, 3},
                                                             num_new_steps * timestep);

            for (int i = 0; i < num_new_steps; ++i) {
              //double s = s_start + (i+1)*step;
              double s = EvaluatePolynomialAtValue(s_alpha, timestep * (i+1));
              double x = x_spline(s) + d_val*dx_spline(s);
              double y = y_spline(s) + d_val*dy_spline(s);
              next_x_vals.push_back(x);
              next_y_vals.push_back(y);
              next_s_vals.push_back(s);
              cout << "Init point: " << s << " "<< x << " " << y << endl;
            }
          }
          else {
            vector<vector<double>> xy_state = CalculateState(num_prev_path_points-2, previous_path_x, previous_path_y);
            vector<double> x_state = xy_state[0];
            vector<double> y_state = xy_state[1];
            double total_speed = sqrt(x_state[1]*x_state[1] + y_state[1]*y_state[1]);
            double total_acc = sqrt(x_state[2]*x_state[2] + y_state[2]*y_state[2]);


            double travel_time = num_new_steps * timestep;
            cout << "state: " << total_speed << " " << total_acc << " " << travel_time << endl;
            Eigen::VectorXd s_alpha = JerkMinimizeTrajectory({s_start, total_speed, total_acc},
                                                             {s_start + num_new_steps*step, total_speed, 0}, travel_time);

            for (int i = 0; i < num_new_steps; ++i) {
              //double s = s_start + (i+1)*step;
              double s = EvaluatePolynomialAtValue(s_alpha, timestep * (i+1));
              double dx = dx_spline(s);
              double dy = dy_spline(s);
              double x = x_spline(s) + d_val*dx;
              double y = y_spline(s) + d_val*dy;
              next_x_vals.push_back(x);
              next_y_vals.push_back(y);
              next_s_vals.push_back(s);
              cout << "add s,x,y: " << s << " "<< x << " " << y << endl;
            }
          }

                   /*
          vector<double> s_start_state = {s_start, target_speed, car_acc_x};
          vector<double> s_goal_state = {goal_x, goal_speed_x, goal_acc};
          */

          /*
          double s = s_start;
          double dx = dx_spline(s);
          double dy = dy_spline(s);
          double x = x_spline(s) + d_val*dx;
          double y = y_spline(s) + d_val*dy;
          cout << s << " " << x << " " << y << " " << last_x << " " << last_y << endl;
          */



          /*

          if (previous_path_x.size() > 10) {
            for (size_t i = 0; i < 10; i ++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            for (int i = 0; i < travel_steps - pos_ahead and i < 50; i++) {
              double x_position = EvaluatePolynomialAtValue(x_ahead_alpha, timestep * i);
              double y_position = EvaluatePolynomialAtValue(y_ahead_alpha, timestep * i);
              next_x_vals.push_back(x_position);
              next_y_vals.push_back(y_position);
            }

          }
          else {
            for (size_t i = 0; i < travel_steps; i++) {
              double x_position = EvaluatePolynomialAtValue(x_alpha, timestep * i);
              double y_position = EvaluatePolynomialAtValue(y_alpha, timestep * i);
              next_x_vals.push_back(x_position);
              next_y_vals.push_back(y_position);
            }
          }
           */

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          // Save for next cycle
          helper.x_vals = next_x_vals;
          helper.y_vals = next_y_vals;
          helper.s_vals = next_s_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
