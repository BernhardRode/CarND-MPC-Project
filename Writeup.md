# CarND-Controls-MPC

## Reflections

### Simulation 

<iframe width="854" height="480" src="https://www.youtube.com/embed/XDN3H4yX0uc" frameborder="0" allowfullscreen></iframe>

### The Model

The controller uses a simple Kintematic model. These models simplfy calculations by ignoring tire forces, gravity, etc. Of course, they do suffer in accuracy and may to be suitable for every situation (snow, etc.).

### Timestep Length and Elapsed Duration (N & dt)

`size_t N = 12;`

N represents the number of steps, the controller will try to predict future states.

`double dt = 0.08;`

dt represents the duration in s, that each of those steps will count for. 

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




