# PID Controller
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Project Description
Build a PID controller and tune the PID hyperparameters to drive a car around a track by controlling steering angle of the car.

## Reflection
Video of car driving around the track can be found [here.](https://github.com/gajendrakumar-nikhil/SensorFusion_Localization_Control/blob/master/PID-Control/reflection.mov)

### Proportional Coefficient
Makes the vehicle steer in proportion to Cross Track Error. CTE is how far is the car from the middle of the road. No matter how small the coefficient is, it will eventually turns the wheels towards the reference trajectory resulting in overshooting.

### Differential coefficient
Is there a way to avoid overshooting ? How about temporal derivative of CTE ? When the car is turned enough to reduce the cross track error, differential term will notice that we are already reducing the CTE because of Proportional part, it counter steers. This will allow the car to reach our target trajectory smoothly.

### Integral Coefficient
Just PD controller is not enough to handle systematic bias (if we drive a car in the normal steering mode leads to a trajectory far away from the goal because of tire mis alignment). All we need to do is to notice the CTE is increasing over time because of systematic bias, we will have to steer more and more to compensate for this bias. This is done by measuring the integral/sum of CTEs over time. 

I chose all the below final P, I, and D coefficients by trial and error.

* `Kp = 0.1` (selected this by trying out all values between 0.07 to 0.12 in steps of 0.01) 
* `Ki = 0.00015` (since there is not much of systematic bias in the simulator, integral coefficient is set a very small value)
* `Kd = 2.2` (set to a slightly high value to avoid car going off track at sharp turns)