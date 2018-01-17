# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

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


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## The Model

Information about the vehicle is given to us in the form of a state vector

```
x, y - location of the vehicle
psi - yaw (angle of vehicle)
v - speed
cte - cross track error
epsi - orientation error

Actuators:
a - acceleration
delta - car's steering angle
```

The update equations can be found on lines 100-105 in mpc.cpp
```
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```

## Timestep length and Elapsed Duration (N & dt)
Started off with N = 25 and dt = 0.05.  The lines were all over the place, so I tried out the following values for N: 20, 15, and 10.  Anything over than 10 seemed to not work well and the driving was all over the place.  For dt, I tried the following values: 0.05, 0.10, 0.15, and 0.20.  It seemed to work best at 0.20 and anything greater than that would  cause the car to sway and go off the track.

## Polynomial Fitting and MPC Preprocessing
The waypoints are converted into vehicle coordinates so everything is kept consistent.  This is done on lines 109-116 in main.cpp
```
for (size_t i = 0; i < ptsx.size(); i++) {
  double dx = ptsx[i] - px;
  double dy = ptsy[i] - py;
  double x_coord = dx*cos(-psi) - dy*sin(-psi);
  double y_coord = dx*sin(-psi) + dy*cos(-psi);
  car_coords_x.push_back(x_coord);
  car_coords_y.push_back(y_coord);
}
```
A third degree polynomial is used to fit the points of the road.  In the next following lines, the polynomial is found,  and the cte and epsi are found.  After that the mpc is used to solve for the appropriate steering and throttle values.
```
auto coeffs = polyfit(car_ptsx, car_ptsy, 3);
double epsi = -atan(coeffs[1]);
double cte = polyeval(coeffs, 0);
VectorXd state(6);
state << 0.0, 0.0, 0.0, v, cte, epsi;

auto res = mpc.Solve(state, coeffs);
steer_value = res[6];
throttle_value = res[7];
```

## Latency
Latency is accounted for on lines 102-107 in main.cpp.  The update equations are done in map coordinates before the state is converted into vehicle coordinates.
```
  double const dt = 0.1; // 100 ms
  double const Lf = 2.67;
  px += v * cos(psi) * dt;
  py += v * sin(psi) * dt;
  psi += v * steer_value * dt / Lf;
  v += throttle_value * dt;
```
The vehicle now moves at a faster rate and does not jerk and stop consistently.  The vehicle does cross over the lines at two points, but it never goes over the ledge or curb.
