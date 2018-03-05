#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <string>
#include <fstream>

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void reset_sim(uWS::WebSocket<uWS::SERVER> ws)  {

  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

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

int main(int argc, const char * argv[])
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  double Kp = 0.24;
  double Ki = 0.016;
  double Kd = 0.86;
  double  target_speed = 10.0;

  // Get command string parameters: target speed, Kp, Ki, Kd
  if(argc > 1) {
    try {
      target_speed = stof(argv[1]);
    } catch (...)  {
      cout << "target_speed" << endl;
      return -1;
    }
  }
  cout << "Traget Speed = " << target_speed << endl;

  if(argc > 2) {
    try {
      Kp = stof(argv[2]);
    } catch (...)  {
      cout << "Kp" << endl;
      return -1;
    }
  }
  if(argc > 3) {
    try {
      Ki = stof(argv[3]);
    } catch (...)  {
      cout << "Ki" << endl;
      return -1;
    }
  }
  if(argc > 4) {
    try {
      Kd = stof(argv[4]);
    } catch (...)  {
      cout << "Kd" << endl;
      return -1;
    }
  }

  // init PID controller
  pid.Init(Kp, Ki, Kd);
  cout << "Kp: " << pid.Kp_ << " Ki: " << pid.Ki_ << " Kd: " << pid.Kd_ << endl;

  // speed control
  PID speed_pid;
  speed_pid.Init(0.1, 0.02, 0);

  // Save results in a file
  ofstream fout;
  const string  fname = "../data/pid_output.txt";

  fout.open(fname);

  if (!fout.is_open()) {
    cout << "Unable to open output file " << fname << endl;
    return -1;
  }

  h.onMessage([&pid, &speed_pid, &fout, &target_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = -pid.TotalError();

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Angle: " << angle << std::endl;

          // speed control
          speed_pid.UpdateError(speed - target_speed);
          double throttle = -speed_pid.TotalError();

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle; //0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // save results
          fout << cte << "\t" << angle << "\t" << steer_value << endl;
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
    fout.close();
    return -1;
  }
  h.run();

  fout.close();
  cout <<"Results saved in file " << fname << endl;
}
