#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // reference trajectory in world coordinate system
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          // initial state in world coordinate system
          // j[1] is the data JSON object
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];

          // v, delta and a are same in world and vehicle coordinate system
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          // define actuator actuation_delay_ms
          const int actuation_delay_ms = 100;
          const double actuation_delay_sec = actuation_delay_ms/1000.;

          // convert absolute reference points to vehicle coordinates
          assert(ptsx.size() == ptsy.size());
          const size_t n_pts = ptsx.size();
          auto ptsx_vehicle = Eigen::VectorXd(n_pts);
          auto ptsy_vehicle = Eigen::VectorXd(n_pts);
          for (unsigned i=0; i<n_pts; i++) {
            double x_rel = ptsx[i] - px;
            double y_rel = ptsy[i] - py;
            double psi_rel = 0.0-psi;
            ptsx_vehicle(i) = x_rel*cos(psi_rel) - y_rel * sin(psi_rel);
            ptsy_vehicle(i) = x_rel*sin(psi_rel) + y_rel * cos(psi_rel);
          }

          // fit reference trajectory
          Eigen::VectorXd ref_trajectory_coeffs = polyfit(ptsx_vehicle, ptsy_vehicle, 3);

          // The initial undelayed state (in vehicle coordinate system).
          const double x0 = 0;
          const double y0 = 0;
          const double psi0 = 0;
          const double cte0 = ref_trajectory_coeffs[0];
          const double epsi0 = -atan(ref_trajectory_coeffs[1]);

          // The initial state for mpc must be the state where the actuation applies.
          // thus compute the initial state by using the actuation actuation_delay_ms.
          Eigen::VectorXd state(6);
          const double x0_delayed = x0 + (v * cos(psi0) * actuation_delay_sec);
          const double y0_delayed = y0 + (v * sin(psi0) * actuation_delay_sec);
          const double psi0_delayed = psi0 - (v * delta *actuation_delay_sec / mpc.Lf);
          const double v_delayed = v + (a * actuation_delay_sec);
          const double cte0_delayed = cte0 + (v * sin(epsi0) * actuation_delay_sec);
          const double epsi0_delayed = epsi0 - (v * atan(ref_trajectory_coeffs[1]) * actuation_delay_sec / mpc.Lf);

          state << x0_delayed, y0_delayed, psi0_delayed, v_delayed, cte0_delayed, epsi0_delayed;

          auto mpc_result = mpc.Solve(state, ref_trajectory_coeffs);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = mpc_result.delta/deg2rad(25);
          msgJson["throttle"] = mpc_result.a;

          //std::cout << "delta=" << mpc_result.delta << "; a=" << mpc_result.a
          //          << "; cte=" << cte0 << "; epsi=" << epsi0 << std::endl;

          //Display the MPC predicted trajectory
          msgJson["mpc_x"] = mpc_result.x;
          msgJson["mpc_y"] = mpc_result.y;

          // Display the waypoints/reference line in vehicle coord sys
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double next_x_step = 2.;
          unsigned int n_ref_pts = 20;
          for (unsigned int i=0; i<n_ref_pts; i++) {
            double x = i*next_x_step;
            next_x_vals.push_back(x);
            next_y_vals.push_back(polyeval(ref_trajectory_coeffs, x));
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(actuation_delay_ms));
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
