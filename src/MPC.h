#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
public:

  /**
   * This value assumes the model presented in the classroom is used.
   *
   * It was obtained by measuring the radius formed by running the vehicle in the
   * simulator around in a circle with a constant steering angle and velocity on a
   * flat terrain.
   *
   * Lf was tuned until the the radius formed by the simulating the model
   * presented in the classroom matched the previous radius.
   *
   * This is the length from front to CoG that has a similar radius.
   */
  const double Lf = 2.67;

  struct Result {
    double a;
    double delta;
    vector<double> x;
    vector<double> y;
  };

  MPC();

  virtual ~MPC();

  /**
   * Solve the model given an initial state and polynomial coefficients of the reference path.
   * Return the first actuations.
   * @param state initial state - the first actuations returned apply to this state
   * @param coeffs reference path polynomial coefficients
   * @return
   */
  MPC::Result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
