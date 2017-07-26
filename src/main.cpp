#include <uWS/uWS.h>
#include <iostream>
#include <limits>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // min_cte is the minimum absolute cross track error seen when changing directions
  double min_cte = std::numeric_limits<double>::max();

  // last_cte is the previously seen cross track error
  double last_cte = 0.0;

  // last_dcte is the previous delta cross track error calculated
  double last_dcte = 0.1;

  // count keeps track of the total number of telemetry packages seen
  // used for calculating an average (squared) error
  int count = 0;

  // PID controller to control the steering angle
  PID steer_pid;
  steer_pid.Init(0.375, 0.003125, 5.625);

  // PID controller to control the throttle
  PID speed_pid;
  speed_pid.Init(0.578125, 0.00125, 6.75);

  h.onMessage([&steer_pid, &speed_pid, &count, &min_cte, &last_cte, &last_dcte](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          count++;
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;

          // Used for calculating debugging values to determine good parameters

          // dcte is the delta cross track error
          double dcte = cte - last_cte;
          
          // if the last delta cross track error is effectively zero or
          // if the current delta cross track error has a different sign than the last one
          if ((std::abs(last_dcte) < 0.001) || ((dcte * last_dcte) < 0)) {

              // check if the cross track error is less than the minimum one seen
              if (std::abs(cte) < min_cte) {
                  min_cte = std::abs(cte);
              }
          }

          // update last_(d)cte for the next telemtry package
          last_cte = cte;
          last_dcte = dcte;

          // Update the steering PID with the current cross track error
          steer_pid.UpdateError(cte);

          // Calculate the new steering angle
          steer_value = steer_pid.CalculateControlValue();

          // Clamp steering angle to [-1, 1]
          if (steer_value < -1.0) {
            steer_value = -1.0;
          } else if (steer_value > 1.0) {
            steer_value = 1.0;
          }

          // Update the throttle PID with the current speed recentered around 30mph
          speed_pid.UpdateError(speed - 30);

          // Calculate the new throttle value
          throttle_value = speed_pid.CalculateControlValue();

          // Clamp the throttle value to [0, 1]
          if (throttle_value < 0.0) {
              throttle_value = 0.0;
          } else if (throttle_value > 1.0) {
              throttle_value = 1.0;
          }

          
          // DEBUG
          std::cout << "MIN CTE: " << min_cte << " AVG ERROR: " << steer_pid.TotalError() / count << " CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed << " Angle: " << angle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
