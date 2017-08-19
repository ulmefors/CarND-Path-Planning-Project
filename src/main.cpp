#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
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
  helper.ref_speed = 0;

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
          const double max_speed_mph {48};
          const double imp_metric_conversion {0.447};
          const double max_speed = max_speed_mph * imp_metric_conversion;
          const double timestep {0.02}; // 0.02 second update
          const double speed_increment {0.3};
          const double lane_correction{0.25};
          const int lane_width {4}; // 4 m lane

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

          // Number of points in path received from Simulator
          int prev_path_size = previous_path_x.size();

          // Ego vehicle derived state
          double car_yaw_rad = deg2rad(car_yaw);
          double car_speed = car_speed_mph * imp_metric_conversion; // mph to mps
          double car_speed_x = car_speed * cos(car_yaw_rad);
          double car_speed_y = car_speed * sin(car_yaw_rad);
          int ego_lane = (int)car_d / lane_width;

          // Get lane plan
          vector<double> lane_speeds = GetLaneSpeeds(car_s, ego_lane, max_speed, sensor_fusion);
          Plan plan = GetPlan(lane_speeds, ego_lane, max_speed, car_speed);
          int goal_lane = plan.lane;
          double goal_speed = plan.speed;

          // Max speed
          double ref_speed = helper.ref_speed;
          if (ref_speed > (goal_speed - speed_increment))
          {
            ref_speed -= speed_increment;
          }
          else if (ref_speed < max_speed)
          {
            ref_speed += speed_increment;
          }
          helper.ref_speed = ref_speed;

          // Points used to create smooth spline trajectory ahead from reference position
          vector <double> ptsx, ptsy;

          // Ego vehicle reference position  current position or last position in previous path), and shortly before
          double ref_x, ref_y;
          double pre_ref_x, pre_ref_y;
          double ref_s;

          // Yaw at reference position
          double ref_yaw;

          // Choose reference position based on previous path length
          if (prev_path_size < 2)
          {
            ref_yaw = car_yaw_rad;

            ref_x = car_x;
            ref_y = car_y;

            pre_ref_x = ref_x - cos(ref_yaw);
            pre_ref_y = ref_y - sin(ref_yaw);

            ref_s = car_s;
          }
          else
          {
            ref_x = previous_path_x[prev_path_size-1];
            ref_y = previous_path_y[prev_path_size-1];

            pre_ref_x = previous_path_x[prev_path_size-2];
            pre_ref_y = previous_path_y[prev_path_size-2];

            ref_yaw = atan2(ref_y-pre_ref_y, ref_x-pre_ref_x);

            ref_s = end_path_s;
          }

          // Push first two points
          ptsx.push_back(pre_ref_x);
          ptsx.push_back(ref_x);
          ptsy.push_back(pre_ref_y);
          ptsy.push_back(ref_y);


          // Add cartesian coordinates for waypoints ahead
          double wp_spacing {30};
          int num_wp = 3;
          for (int i = 0; i < num_wp; ++i)
          {
            double s = ref_s + (i+1)*wp_spacing;
            double d = ((double)goal_lane+0.5)*(double)lane_width; // Center of choosen lane
            d+=(1-goal_lane)*lane_correction; // Correction to avoid warning in simulator
            double wp_x = x_spline(s) + d*dx_spline(s);
            double wp_y = y_spline(s) + d*dy_spline(s);
            ptsx.push_back(wp_x);
            ptsy.push_back(wp_y);
          }

          // Transform into ego vehicle coordinates
          for (int i = 0; i < ptsx.size(); ++i)
          {
            double x = ptsx[i] - ref_x;
            double y = ptsy[i] - ref_y;

            ptsx[i] = cos(0-ref_yaw)*x - sin(0-ref_yaw)*y;
            ptsy[i] = sin(0-ref_yaw)*x + cos(0-ref_yaw)*y;
          }

          // Define spline where y is defined by x
          tk::spline cartesian;
          cartesian.set_points(ptsx, ptsy);

          // Add previous positions
          for (int i = 0; i < prev_path_size; ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Plan 30 meters straight ahead (horizon)
          double horizon_steps {50};
          double horizon_x {30};
          double horizon_y = cartesian(horizon_x);
          double horizon_distance = distance(0, 0, horizon_x, horizon_y);

          // Start at origin in ego vehicle coordinates
          double x_future {0};
          double y_future {0};

          // Add new points (global coordinates
          double num_steps = horizon_distance/timestep/ref_speed;
          double step_length_x = horizon_x/num_steps;
          for (int i = 0; i < horizon_steps - prev_path_size; ++i)
          {
            x_future += step_length_x;
            y_future = cartesian(x_future);

            // Convert to global coordinates
            double x_global, y_global;
            x_global = cos(ref_yaw)*x_future - sin(ref_yaw)*y_future;
            y_global = sin(ref_yaw)*x_future + cos(ref_yaw)*y_future;
            x_global += ref_x;
            y_global += ref_y;

            next_x_vals.push_back(x_global);
            next_y_vals.push_back(y_global);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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
