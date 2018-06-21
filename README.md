# Model Predictive Control for Udacity Simulator
Self-Driving Car Engineer Nanodegree Program

## Overview
This project implements a model predictive controler (MPC) to steer 
a vehicle via throttle and left/right steering in Udacitys simulator to drive safely and fast on a race
track without leaving the road.

## Rubric Points

### The Model

The model is implemented in [MPC.cpp](src/MPC.cpp). Class FG_eval defines the cost function and the state transition equations.
The function MPC::Solve defines constraints for the initial state only to fix all state variables
to the specific initial state values and MPC::Solve also defines upper and lower bounds.

#### State transition equations
The state equations are defined in [MPC.cpp](src/MPC.cpp) lines 110 to 118. 
The state equations implement a dynamic vehicle model from the Udacity course material
that fits the vehicle simulator behaviour.
The transition between subsequent states is defined via an equation for each state variable (e.g. x, y, v).
The upper and lower boundary constraints are set to zero for all equations so that the
solver finds an actuator solution for each state that fulfills the equations (see [MPC.cpp](src/MPC.cpp), lines 175-180).
As an exception, the initial state equations are set to the specific values of the given initial state.  

The following dynamic model is implemented:
* x[t] = x[t-1] + (v[t-1] * cos(psi[t-1]) * dt)
* y[t] = y[t-1] + (v[t-1] * sin(psi[t-1]) * dt)
* psi[t] = psi[t-1] - v[t-1] * delta[t-1] / Lf * dt);
* v[t] = v[t-1] + (a[t-1] * dt);
* cte[t] = cte[t-1] + (v[t-1] * sin(epsi[t-1]) * dt);
* epsi[t] = (psi[t-1] - ref_psi[t-1]) + (v[t-1] * delta[t-1] / Lf * dt);
      
The variables are defined like this:
* x, y: x and y position of the ego vehicle in its vehicle coordinate system with the undelayed initial state as origin
* v: velocity in miles per hour
* a: throttle in range [-1, 1]
* Lf: distance from center of rear axis to front of the vehicle (constant 2.67m)
* dt: delta time between time steps, 100 ms is used
* delta: yaw rate
* psi: heading in radians
* cte: cross track error which is the distance to reference waypoint path in meters
* epsi: heading error which is the difference to the reference heading in radians
* ref_psi: reference (desired) heading in radians 

#### Cost function
The cost function ([MPC.cpp](src/MPC.cpp), lines 45 - 66) sums a penalty for
* cross track error
* error on vehicle heading
* deviation from reference velocity 
* using steering actuator
* using steering actuator multiplied by velocity in order to avoid drastic steering in high velocity
* delta of actuator values between timesteps in order to have smooth actuator changes

The cost function makes sure that the vehicle keeps on close to the reference 
path (center of road), heads into the reference direction, drives fast enough,
steers less at high speeds and actuator changes are low. 

Choosing the factors on the individual cost function parts was done via manually tuning
with a lot of experiments. Finally adding the combined error to penalize steering on higher velocity
improved the model performance significantly.
Since I did not normalize the errors to a common value range, the factors also include a normalizing 
component e.g. to normalize between actual to desired velocity error and actual to desired heading error. 

I set the reference velocity to 90. In order to achieve a higher velocity (by increasing reference velocity)
and also in order to go faster in curves, more cost factor parameter tuning would be needed.
An automation of exploring cost factor combinations would help but requires an automatic reset of
the simulator. 

#### Actuators
The actuators are steering and throttle. The model constraints steering to between [-25, 25] degrees
and throttle to [-1, 1]. The actuators are part of the state transition equations.

### Timestep Length and Elapsed Duration (N & dt)

For the MPC, the number of timesteps (N) was set to 10 with an elapsed duration between timesteps (dt) of 0.1 seconds.
This allows a foresight of 1 second for the actuators computed by MPC. This short foresight keeps the 
computational complexity for the solver low and is enough to handle an upcoming curve. It is not necessary
to have a foresight longer then this. Choosing 100ms for dt was necessary to have faster reactions on actuators. 
Other values tried were N=20 with dt=0.1 and N=20 with dt=0.2 but results lead to problems in the s-curve.
Also the 200ms for dt appeared to be too less reactive.

### Polynomial Fitting and MPC Preprocessing

The waypoints received in the telemetry event are transformed from world to vehicle coordinate system ([main.cpp](src/main.cpp):106).
Then a third order polynomial is fitted to the waypoints. This polynomial and delayed initial state is then used to 
calculate the actuators via mpc.

### Model Predictive Control with Latency

The actuators have a delay of 100 milliseconds which is implemented via sending the actuator inputs 100ms after
they have been determined via mpc (see [main.cpp](src/main.cpp):182). 
To take this delay into account, the mpc must calculate actuator values that can be applied
on the state - called delayed state from now on - after 100 ms have passed. 
To achieve this, this delayed state must be computed and passed as initial state into the mpc.
The delayed state is calculated in [main.cpp](src/main.cpp):131 using the vehicle dynamic model.  

### Simulation - The vehicle must successfully drive a lap around the track.
The vehicle drives around the track without leaving the road and without driving on ledges.
In curves the vehicle slows down as intended. 
After curves the vehicle accelerates. The vehicle can drive lap after lap successfully 
(also with a running start from lap 2 on).


## Udacity Instructions

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

### Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
    5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

### Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).



