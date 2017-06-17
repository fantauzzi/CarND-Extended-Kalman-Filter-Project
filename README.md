# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project I utilize an Extended Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

A specific version of uWebSocketIO must be installed for the program to interact with the simulator. To install it, run `install-ubuntu.sh` or `install-mac.sh` from the project folder.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. `mkdir build`
2. `cd build`
3. `cmake ..`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles"`
4. `make`
5. `./ExtendedKF [OPTION]`

   where [OPTION] is optional and can be one of:
   * `radar` to process only RADAR measurements coming from the simulator, and ignore LIDAR;
   * `lidar` to process only LIDAR measurements coming from the simulator, and ignore RADAR.
   * if no [OPTION] is provided, all measurements coming from the simulator are processed by the Kalman Filter.

---

## Other Dependencies

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

## Project Content

* `CMakeLists.txt` cmake configuration file;
* `cmakepatch.txt` cmake configuration file;
* `.cproject` Eclipse CDT project file; note that in order to use it, you should first remove the cmake configuration files from the project directory;
* `data/obj_pose-laser-radar-synthetic-input.txt` list of sensor readings produced by the simulator (they are provided for reference, chaning them will not affect the simulator behavior);
* `docs/Input_Output File Format.txt` description of the `obj_pose-laser-radar-synthetic-input.txt` file format;
* `.project` Eclipse CDT project file;
* `README.md` this file;
* `.settings/` directory with Eclipse CDT project settings;
* `src/` directory containing the project C++ source code.
