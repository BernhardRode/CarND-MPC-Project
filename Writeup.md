# CarND-Controls-MPC

## Reflections

### Simulation 

[![CarND-Controls-MPC in Simulator](https://img.youtube.com/vi/XDN3H4yX0uc/0.jpg)](https://youtu.be/XDN3H4yX0uc)

https://youtu.be/XDN3H4yX0uc

### The Model

The controller uses a simple Kintematic model. These models simplfy calculations by ignoring tire forces, gravity, etc. Of course, they do suffer in accuracy and may to be suitable for every situation (snow, etc.).

The model from the lessons was used:

```
//Latency predict at t+1
//Predicted car's state after latency transcurred
// x, y and psi are all zero after transformation above
double pred_px = 0.0 + v * dt; // Since psi is zero, cos(0) = 1
const double pred_py = 0.0;    // Since sin(0) = 0, (y + v * 0 * dt)
double pred_psi = 0.0 + v * -steer_value / Lf * dt;
double pred_v = v + throttle_value * dt;
double pred_cte = cte + v * sin(epsi) * dt;
double pred_epsi = epsi + v * -steer_value / Lf * dt;
```

As Lf I used 2.67.

### Timestep Length and Elapsed Duration (N & dt)

`size_t N = 10;`

N represents the number of steps, the controller will try to predict future states.

`double dt = 0.01;`

dt represents the duration in s, that each of those steps will count for. 

The chosen values for **N** and **dt** of 10 and 0.1 causes the controller to predict 10 steps, each of a 0.1 sec (100 ms) duration.

I've played around with different parameters, but this combination used to work very good.

I tried:
* 20/0.05 - worked quite well for one lap, but then the car struggled to stay on the road.
* 5/0.2 - did not work at all
* 20/0.1 - did not go so well, too
* 12/0.08 - worked, but is out of spec

### Polynomial Fitting and MPC Preprocessing

After we get data from the simulator, it is being transformed to vehicle coordinates, using the current vehicle position and heading (provided by the simulator).
Afterwards, the points are fitted to a 3rd order polynomial.

### Model Predictive Control with Latency

Latency has been added to the system:

`this_thread::sleep_for(chrono::milliseconds(100));`

Costs have been added, too:

* x_start
* y_start
* psi_start
* v_start
* cte_start
* epsi_start

Additionally, previous actuations (t-0.1s) have been fed to the solver, to deal with latency.




