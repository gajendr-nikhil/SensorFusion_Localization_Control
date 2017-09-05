# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

[image1]: ./images/state.png "Vehicle state"

## Model

Model predictive control involves simulating different actuator inputs predicting the resulting trajectory and selecting the trajectory with minimum cost. We are using global kinematic model to achieve this. Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass.

#### State of a vehicle
State of a vechicle is represented by vehicle position (x, y), orientation, and velocity. If we know the current state of the vehicle we have to find what's its state in the future.

| Vehicle state |
|--------------------------------|
|![alt text][image1]             | 

The path planning block charts a reference trajectory (as a 3rd degree polynomial) using environmental model, the map, and vehicle location. Finally, the control loop applies the actuators, steering wheel and throttle/brake, to follow the reference trajectory to minimize the error between vehicle's actual path and the reference trajectory. We minimize this error by predicting the vehicle's actual path and adjusting the control inputs to minimize the difference between that prediction and reference trajectory.

##### Cross Track Error
If the vehicle is traveling down a straight road, then we can express the error between the center of the road and the vehicle's position as the cross track error.

##### Orientation Error
Difference between vechicle orientation and trajectory orientation.

Ideally, both of these errors would be 0 - there would be no difference from the actual vehicle position and heading to the desired position and heading.

#### Cost function
Our cost should be a function of how far these errors are from 0. Cost function not only include the state, but also includes the control input. This allows us to penalize the magnitude of input as well as the change rate.

### MPC algorithm
Model predictive control uses an optimizer to find the control inputs and minimize the cost function. We execute only first set of control inputs. This brings vehicle to a new state and then you repeat the process. 

#### MPC preprocessing
Transform waypoints from map coordinates to vehicle coordinate system (car as origin)

	1. Define the duration of trajector, T, by choosing N and dt.
	2. We define vehicle model and constraints such as actuator limitations.
	3. We define the cost function.
	4. State feedback loop
		* Pass the first state to MPC
		* Optimization solver is called. Solver uses the initial state, model constraints and cost function to return a vector of control inputs that minimize the cost function. The solver we used is called IPOPT
		* We apply the first control input to the vehicle.
		* Repeat the loop.

#### Latency
In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. This is a problem called "latency". Model Predictive Controller can adapt quite well because we can model this latency in the system. A latency of 100 milliseconds has been implemented in `main.cpp`

##### Handle latency
Predict the car's position, orientation, and velocity after the latency using car's current speed and position and previous steering angle and throttle input to feed it to MPC algorithm. This makes sure the MPC algorithm prediction is correct even with latency.

### Timestep Length and Elapsed Duration (N & dt)
N (Number of timesteps) & dt (timestep duration) represent horizon into to the future to predict actuator changes. 
I started with predicting 4 seconds in future (N = 20 & dt = 0.2). I was able to get good results at 30mph speed. As the speed increased to 50 mph, prediction went wrong and car went off track because optimizer was looking too far ahead.
I started reducing the N & dt. I tried 10 <= N <= 20 (step = 1) & 0.05 <= dt <= 0.08 (step = 0.01). Got the best and smooth driving result for N = 15 & dt = 0.06 with speed = 50 mph.


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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

