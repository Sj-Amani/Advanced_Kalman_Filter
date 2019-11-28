# Extended Kalman Filter

![Advanced_Kalman_Filter_gif](results/Advanced_Kalman_Filter.gif)

## Overview
In this project I will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. The main codes are in the "src/" directory:
* FusionEKF.h
* FusionEKF.cpp
* kalman_filter.h
* kalman_filter.cpp
* tools.h
* tools.cpp

Goals: `Obtaining RMSE values that are lower than a specific tolerance using c++`.

This project involves the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete (e.g. for ubuntu, run the script: `install-ubuntu.sh`), the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF
6. cd ..
7. ./term2_sim.x86_64 (Launch the simulator)
8. If you see this message, it is working: `Listening to port 4567 Connected!!!`

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build Instructions

1. Clone this repo.
2. Run relevant .sh file. For example, in Ubuntu, run the script: `install-ubuntu.sh`
3. Make a build directory: `mkdir build && cd build`
4. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
5. Run it: `./ExtendedKF `
6. Run the simulator: `cd ..  && ./term2_sim.x86_64`
7. If you see this message, it is working: `Listening to port 4567 Connected!!!`

## Editor Settings

In order to keep it as simple and environment agnostic as possible, the editor configuration files are kept out of this repo . However, it's recommended using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Hints and Tips!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.
* Programmers have reported rapid expansion of log files when using the term 2 simulator.  This appears to be associated with not being connected to uWebSockets.  If this does occur,  please make sure you are conneted to uWebSockets. The following workaround may also be effective at preventing large log files.

    + create an empty log file
    + remove write permissions so that the simulator can't write to log
 * Please note that the ```Eigen``` library does not initialize ```VectorXd``` or ```MatrixXd``` objects with zeros upon creation.


References:
---
https://classroom.udacity.com/nanodegrees

https://github.com/udacity/CarND-Mercedes-SF-Utilities

https://github.com/nlohmann/json

https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

https://github.com/georgesung/extended_kalman_filter

https://github.com/udacity/self-driving-car-sim/releases

How Referencing This Project
---
If you like my code and you want to use it in your project, please refer it like this:

`Amani, Sajjad. "Advanced Kalman Filter for Autonomous Vehicles." GitHub, 28 November 2019, https://github.com/Sj-Amani/Advanced_Kalman_Filter`

