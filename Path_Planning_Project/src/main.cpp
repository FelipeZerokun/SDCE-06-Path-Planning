#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  // Define necessary variables before connecting to the simulator
  int lane = 1; // I start in the center lane (0 for left lane, 1 for center lane and 2 for right lane)
  double ref_vel = 0; // Reference velocity for my SPEED LIMIT (MPH)


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // I use these to check how the car positions work
          //std::cout<<"car S: "<<car_s<<" car D: "<<car_d<<std::endl;
          //std::cout<<"car X: "<<car_x<<" car Y: "<<car_y<<std::endl;


          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // MY CODE STARTS HERE

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // I will follow the Structure of the "Project Q&A" class to get a smooth car movement



          // Like in my test code, I always need to be checking whether the list is about to empty
          int path_size = previous_path_x.size();
          //std::cout<<"Path size: "<<path_size<<std::endl;
          // After coding the car smooth movement, I will code the car behaviour
          // I want to stay in the center lane unless there is a slower car in front of me.
          // If that is the case, I will try to pass the car by going to the left lane and accelerating to pass the car.

          if(path_size > 0)
          {
              // If there are points in the previous path, my car state will be the end path S.
              car_s = end_path_s;
          }

          // Assign a variable to check if there are cars near my car
          bool KL = keep_lane(lane, sensor_fusion, path_size, car_s);
          bool left_safe = false;
          bool right_safe = false;

          if (KL)
          {
            left_safe = false;
            right_safe = false;
            std::cout<<"Current state: Keep lane "<<lane<<std::endl;
            if(ref_vel < 49.5)
            {
              ref_vel += 0.224;
            }
          }
          else if (!KL)
          {
            std::cout<<"Current state: Preparing for lane change! "<<lane<<std::endl;
            if(ref_vel > 29.5)
            {
              ref_vel -= 0.224;
            }

            if(lane == 1)
            {
              left_safe = look_left(lane, path_size, sensor_fusion, car_x, car_y, car_speed);
              right_safe = look_right(lane, path_size, sensor_fusion, car_x, car_y, car_speed);
              if(left_safe)
              {
                lane = 0;
                //left_safe = false;
              }
              else if(right_safe)
              {
                lane = 2;
                //right_safe = false;
              }
            } 
            if(lane == 0)
            {
              right_safe = look_right(lane, path_size, sensor_fusion, car_x, car_y, car_speed);
              if(right_safe)
              {
                lane = 1;
                //right_safe = false;
              }
            }
            if(lane == 2)
            {
              left_safe = look_left(lane, path_size, sensor_fusion, car_x, car_y, car_speed);
              if(left_safe)
              {
                lane = 1;
                //left_safe = false;
              }
            }
          }

          // First, I will create a list of X and Y points evenly spaced. This is so the Spline library can create the
          // trajectory filling all the space between my X and Y points.

          vector<double> x_list;
          vector<double> y_list;

          // Reference x, y and yaw states
          // To create the points, I need to take into account the actual state of the car
          double reference_x = car_x;
          double reference_y = car_y;
          double reference_yaw = deg2rad(car_yaw);

          if (path_size < 2)
          {
            // I will use the two points to make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            // With these two points into the list, I make sure that the path is tangent to the car
            x_list.push_back(prev_car_x);
            x_list.push_back(car_x);

            y_list.push_back(prev_car_y);
            y_list.push_back(car_y);
          }
          else
          {
            // If I have some previous points, I will use them as Reference to make the next points in the path
            reference_x = previous_path_x[path_size-1];
            reference_y = previous_path_y[path_size-1];

            double prev_reference_x = previous_path_x[path_size-2];
            double prev_reference_y = previous_path_y[path_size-2];
            reference_yaw = atan2(reference_y-prev_reference_y,reference_x-prev_reference_x);
            // same as before, I need to make sure that the next points are tangent to the previous ones.
            x_list.push_back(prev_reference_x);
            x_list.push_back(reference_x);

            y_list.push_back(prev_reference_y);
            y_list.push_back(reference_y);
          }

          // Now, I need to make the points of reference which will be some distance apart, and use them to build a
          // path using the spline library
          int sep_dist = 30; // Separation distance between my reference points
          vector<double> next_wp0 = getXY(car_s + sep_dist, (2 + lane*4), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + sep_dist*2, (2 + lane*4), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + sep_dist*3, (2 + lane*4), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          x_list.push_back(next_wp0[0]);
          x_list.push_back(next_wp1[0]);
          x_list.push_back(next_wp2[0]);

          y_list.push_back(next_wp0[1]);
          y_list.push_back(next_wp1[1]);
          y_list.push_back(next_wp2[1]);

          // Now I need to shift my points to the car's reference frame
          for (int j = 0; j<x_list.size(); j++)
          {
            // shift the car reference angle to 0
            double shift_x = x_list[j]-reference_x;
            double shift_y = y_list[j]-reference_y;

            x_list[j] = (shift_x * cos(0-reference_yaw) - shift_y * sin(0-reference_yaw));
            y_list[j] = (shift_x * sin(0-reference_yaw) + shift_y * cos(0-reference_yaw));
          }

          // Now I define the spline object
          tk::spline s;

          // set X and Y points to the spline
          s.set_points(x_list, y_list);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // First, I will begin with all the previous path points, like in my test code
          for (int k = 0; k < previous_path_x.size();k++)
          {
            next_x_vals.push_back(previous_path_x[k]);
            next_y_vals.push_back(previous_path_y[k]);
          }

          // Using the math explained in the "Project Q&A" class
          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

          double x_add_on = 0;

          // fill up the rest of our path planner after filling it with our previous points
          // Here we will make sure the output is always 50 points
          for (int l = 0; l <= 50-previous_path_x.size(); l++)
          {
            // from the Q&A video explanation
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal the reference frame
            x_point = (x_ref*cos(reference_yaw)-y_ref*sin(reference_yaw));
            y_point = (x_ref*sin(reference_yaw)+y_ref*cos(reference_yaw));

            x_point += reference_x;
            y_point += reference_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

          // My first approach. To understand how to car "reads" the way points.
          // Just trying to get the car to move in the middle lane as straight as possible
          /* This is my first approach to the car movement
          if(path_size > 0)
          {
            for (int i = 0; i < path_size; ++i)
            {
            	next_x_vals.push_back(previous_path_x[i]);
            	next_y_vals.push_back(previous_path_y[i]);
            }
          }
          else
          {
            for (int j = 0; j <50;++j)
            {
              double next_s = car_s+j*0.2;
              double next_d = car_d;
              xy_vals = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              next_x_vals.push_back(xy_vals[0]);
              next_y_vals.push_back(xy_vals[1]);
            }
          }
		  */

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
