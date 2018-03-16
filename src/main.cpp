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
  // For 10 mph
  //double Kp = 0.24;
  //double Ki = 0.016;
  //double Kd = 0.86;
  // For 20 mph
  double Kp = 0.131835; //0.15
  double Ki = 0.0435481;  //0.015
  double Kd = 0.765783; //0.75;
  // For 30 mph
  //double Kp = 0.0869;
  //double Ki = 0.0189;
  //double Kd = 0.46;

  double  target_speed = 0.;
  double  max_speed = 22.;
  bool is_twiddle = false;


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
      int tw = stof(argv[2]);
      if(tw == 1) {
        is_twiddle = true;
      }
    } catch (...)  {
      cout << "twiddle" << endl;
      return -1;
    }
  }
  cout << "twiddle = " << is_twiddle << endl;

  if(argc > 3) {
    try {
      Kp = stof(argv[3]);
    } catch (...)  {
      cout << "Kp" << endl;
      return -1;
    }
  }
  if(argc > 4) {
    try {
      Ki = stof(argv[4]);
    } catch (...)  {
      cout << "Ki" << endl;
      return -1;
    }
  }
  if(argc > 5) {
    try {
      Kd = stof(argv[5]);
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
  //speed_pid.Init(0.22, 0, 0);
  speed_pid.Init(0.132, 0.2, 0.123);

  // Save results in a file
  ofstream fout;
  const string  fname = "../data/pid_output.txt";

  fout.open(fname);

  if (!fout.is_open()) {
    cout << "Unable to open output file " << fname << endl;
    return -1;
  }

  h.onMessage([&pid, &speed_pid, &fout, &target_speed, &is_twiddle, &max_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          if (is_twiddle == true) {

            if(pid.Twiddle(0.1) == true)
              reset_sim(ws);

            cout << "Step: " << pid.n_step << " State: " << pid.twiddle_state << " B.Error: " << pid.best_error;
            cout << " Kp: " << pid.Kp_ << " Ki: " << pid.Ki_ << " Kd: " << pid.Kd_ << endl;
          }

          steer_value = -pid.TotalError();

          // DEBUG
          if (is_twiddle == false)
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Angle: " << angle << std::endl;

          // speed control
          double throttle = 0.3; // default - no speed control
          if(target_speed != 0) {
            speed_pid.UpdateError(speed - target_speed);
            throttle = -speed_pid.TotalError();
          }
          else if (speed > max_speed) {
            target_speed = max_speed;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle; //0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // save results
          fout << cte << "\t" << angle << "\t" << steer_value << "\t" << speed << endl;
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
