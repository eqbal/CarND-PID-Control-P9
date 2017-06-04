#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <ctime>
#include <chrono>

#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include "FileWriter.h"

#define PRINT_STATS 0
#define WRITE_OUTPUT 0

// for convenience
using json = nlohmann::json;
using std::chrono::system_clock;

void reset_simulator(uWS::WebSocket<uWS::SERVER> &ws);
double normalize(double value, double min = -1.0, double max = 1.0);
std::string hasData(std::string s);

int main(int argc, char *argv[]) {
  uWS::Hub h;

  PID pid_cte;
  PID pid_speed;

  // manual determined values
  double kp_cte = 0.25;
  double ki_cte = 0.08;
  double kd_cte = 300;

  double kp_speed = 0.14;
  double ki_speed = 0.0;
  double kd_speed = 2.0;

  // Set values if parameters given via console parameters
  if (argc >= 7) {
    kp_cte = std::stod(argv[1]);
    ki_cte = std::stod(argv[2]);
    kd_cte = std::stod(argv[3]);
    kp_speed = std::stod(argv[4]);
    ki_speed = std::stod(argv[5]);
    kd_speed = std::stod(argv[6]);
  }

  // values under control
  double steer_value = 0.0;
  double throttle = 1.0;

  // Initialize the PID controllers.
  pid_cte.Init(0.0, kp_cte, ki_cte, kd_cte);
  pid_speed.Init(1.0, kp_speed, ki_speed, kd_speed);

#if WRITE_OUTPUT
  // Construct filename with current date time
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream ss;
  ss << "test_" << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S") << ".csv";

  FileWriter fileWriter(ss.str());

  fileWriter.writePidParameters("PID_cte:", kp_cte, ki_cte, kd_cte);
  fileWriter.writePidParameters("PID_speed:", kp_speed, ki_speed, kd_speed);
  fileWriter.writeLine("Opt:none");
  size_t time_start = 0;
#endif

#if PRINT_STATS
  double max_speed = 0;
  double avg_speed = 0;
  size_t counter = 0;
#endif

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          size_t time_ms =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  system_clock::now().time_since_epoch())
                  .count();

          // Check if car is off the road
          if (std::fabs(cte) > 3.0) {
            reset_simulator(ws);
            std::cout << "BANG\n" << std::endl;
            exit(-1);
          }

          // Calculate steering angle
          pid_cte.Update(cte, time_ms * 1e-3);
          steer_value = pid_cte.GetCorrection();
          steer_value = normalize(steer_value);

          // Calculate throttle
          pid_speed.Update(std::fabs(cte) * speed * std::fabs(steer_value),
                           time_ms * 1e-3);
          throttle = pid_speed.GetCorrection();
          throttle = normalize(throttle);

#if PRINT_STATS
          // Calculate average speed
          counter++;
          avg_speed = ((avg_speed * (counter - 1)) + speed) / counter;

          // Remember maximum speed
          if (speed > max_speed) max_speed = speed;

          std::cout << "CTE: " << cte << " Angle: " << angle
                    << " Steering Value: " << steer_value
                    << " Diff: " << (angle / 25.0) - steer_value
                    << " Speed: " << speed << " Throttle: " << throttle
                    << " Avg. speed: " << avg_speed
                    << " Max speed: " << max_speed << std::endl;
#endif
#if WRITE_OUTPUT
          if (time_start <= 0) time_start = time_ms;

          fileWriter.writeLine(time_ms - time_start, cte, speed, angle,
                               steer_value, throttle, pid_cte.GetTotalError(),
                               pid_cte.GetAveragedError());
#endif
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

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

std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void reset_simulator(uWS::WebSocket<uWS::SERVER> &ws) {
  // reset
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

double normalize(double value, double min, double max) {
  if (value > max) value = max;
  if (value < min) value = min;
  return value;
}
