# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

4th project of the 2nd term.

## Introduction

The purpose of this project is to create a PID controller to drive a car smoothly and successfully around the Udacity simulator.

The task is to keep a car on a given track controlling the steering angle within the range [-25°, 25°] and a throttle within [-1, 1]. The measured values are the cross-track-error (CTE) for the current position of the car as well as the speed of the car. The task is to maximize the speed while keeping the car on the street (minimize the CTE).

PID stands for Proportional, Integral, and Derivative controller. A PID controller continuously calculates an error value `e(t)` as the difference between a desired setpoint and a measured process variable and applies a correction based on proportional, integral, and derivative terms (sometimes denoted P, I, and D respectively).


![](assets/PID.png)


## Flow with the Car

![](assets/PID-Car.png)


## Notes

- CTE measurement errors. For example at start and at end of the bridge in the test track.

- High steering angles reduces also the speed of the car.

- New telemetry data will only be send after sending a steer command to the simulator. So processing time affects the sampling rate.

## Limitations

It's not possible to set the start position of the car within the test track nor can you determine the current position or the driven distance.

## Tuning

I used the `Twiddle` algorithm to tune the coefficient for the P, I and D components of the controller. 

What the twiddle algorithm does is continually adjust the PID coefficients as the car drives, trying to find the best (or lowset) overall error. I found the following parameters which are set in the PID initialization as ones that provided a nice smooth driving experience around the track.

```
double kp_cte = 0.25;
double ki_cte = 0.08;
double kd_cte = 300;

double kp_speed = 0.14;
double ki_speed = 0.0;
double kd_speed = 2.0;
```

## Reflections
The PID controller is made up fo 3 components:

### P - Proportional:

Here we steer the car proportionally to its offset from the center of the road (or desired path). An issue with just using a proportional controller is overshoot.

[![](http://img.youtube.com/vi/LSonm1fgvZc/0.jpg)](https://youtu.be/LSonm1fgvZc)

### D - Differential:

Here we add some feedback into the controller about how quickly it is moving back to the required path.

Check out the video below to show the effect of P & D.

[![](http://img.youtube.com/vi/Kk6OF_-DFQM/0.jpg)](https://youtu.be/Kk6OF_-DFQM)

### I - Integration:

Here we integrate (sum) over the error and feed this signal back into our controller to compensate for any drift in the vehicle. Drift can happen for various reasons such as misalignments of the wheels for example.


## Results

To reduce load on the test machine and increase sampling time by minimizing the response time, the implementation is tested with the lowest simulator details and resolution of 640x480.

Without video capturing a average speed of approximately 45 mph (72.42 km/h) and a maximum speed of around 75 mph (120.7 km/h) can be achieved. 

Although the car sometimes oscillate a lot around the track center it keeps staying on the track without touching the road surface marking.

Check out the video below in which PID has be applied using the best parameters found with the twiddle algorithm.

[![](http://img.youtube.com/vi/q9ha47pzDTk/0.jpg)](https://youtu.be/q9ha47pzDTk)


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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
