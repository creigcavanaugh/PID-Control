#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <float.h>

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

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;
  PID pid_throttle;
  // TODO: Initialize the pid variable.

  // Counter for twiddle
  int twit = 0;
  int dpi = 0;
  int dps = 0;
  long double cumerr = 0.0;
  long double best_error = DBL_MAX;

  //Steering PID Initial Values
  double init_Kp = atof(argv[1]);      //Kp = How hard do you want to steer back to the center line.  Remember to be negative.
  double init_Ki = atof(argv[2]);      //Ki = Update if there is a consistant drift / alignment error
  double init_Kd = atof(argv[3]);      //Kd = Prevent oscillations around the center line

  //Throttle PID Initial Values
  double init_t_Kp = atof(argv[4]);      
  double init_t_Ki = atof(argv[5]);      
  double init_t_Kd = atof(argv[6]);      

  double p[4] = {init_Kp, init_Kd, init_t_Kp, init_t_Kd};
  double p_best[4] = {init_Kp, init_Kd, init_t_Kp, init_t_Kd};
  double dp[4] = {0.01, 0.01, 0.01, 0.01};

  pid.Init(init_Kp, init_Ki, init_Kd);
  pid_throttle.Init(init_t_Kp, init_t_Ki, init_t_Kd);

  h.onMessage([&pid, &pid_throttle, &twit, &cumerr, &best_error, &p, &p_best, &dp, &dpi](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          //Speed Limiting PID based on cte
          double throttle_error = fabs(cte);

          //If the speed is less than 45, don't limit the throttle
          if (speed < 45) {
            throttle_error = 0;
          }

          pid_throttle.UpdateError(throttle_error);
          throttle_value = (1.2 - pid_throttle.TotalError());
          //throttle_value = (((1.2 - pid_throttle.TotalError()) + (throttle_value * 1.0))/2.0);
          

          //Limit throttle
          if (throttle_value < -1.0) {
            throttle_value = -1.0;
          }

          if (throttle_value > 1.2) {
            throttle_value = 1.2;
          }



          twit++;
          cumerr += fabs(cte);

          if (dp[0] + dp[1] + dp[2] + dp[3] > 0.001){

            if( twit % 1000 == 999) {
 
              //Update best error and reinit pid
              if (cumerr < best_error){
                best_error = cumerr;
                p_best[0] = p[0];
                p_best[1] = p[1];
                p_best[2] = p[2];
                p_best[3] = p[3];
                dp[dpi % 4] *= 1.001;
              }
              else {
                dp[dpi % 4] *= 0.999;
              }

              if ((dpi % 8) > 4){
                p[dpi % 4] += dp[dpi % 4];
              }
              else {
                p[dpi % 4] -=  2 * dp[dpi % 4];
              }

              dpi++;
            
              pid.Init(p[0], 0, p[1]);
              pid_throttle.Init(p[2], 0, p[3]);

              cumerr = 0;

            }
          }


          // DEBUG
          std::cout << "Best Error: " << best_error << " cumerr: " << cumerr << " p[0]: " << p[0] << " p[1]: " << p[1] << " p[2]: " << p[2] << " p[3]: " << p[3] << std::endl;
          std::cout << "PBEST: " << " pb[0]: " << p_best[0] << " pb[1]: " << p_best[1] << " pb[2]: " << p_best[2] << " pb[3]: " << p_best[3] << std::endl;
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Terror: " << throttle_error << " Throttle: " << throttle_value << std::endl;

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
