## Self-Driving Car: Path Planning

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

In this project, I have implemented a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower-moving traffic all by using localization, sensor fusion, and map data.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    sudo apt-get update
    sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    mkdir build
    cd build
    cmake ..
    make 
    sudo make install
    cd ../..
    sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
    sudo rm -r uWebSockets
    ```
    
### Runing
Be consider before cloning and building the project, you should download the related [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) which is made by the [Udacity](http://www.udacity.com). 
1. Clone this repo. ```git clone https://github.com/PooyaAlamirpour/PathPlanning.git```
2. ```cd PathPlanning```
3. Make a build directory: ```mkdir build && cd build```
4. Compile: ```cmake .. && make```
5. Run it: ```./path_planning```

Once the code has been built and run successfully, run the simulator. To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```
sudo chmod u+x term3_sim.x86_64
```
The result is depicted as below:

![Output](https://github.com/PooyaAlamirpour/PathPlanning/blob/master/Images/Output.png)

### Project structure
The directory structure of this repository is as follows:
```bash
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|___data
|   |   
|   |   map_data.txt
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Reference
[]()
