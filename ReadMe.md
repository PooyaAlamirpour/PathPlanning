## Self-Driving Car: Path Planning

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

In this project, I have implemented a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower-moving traffic all by using localization, sensor fusion, and map data. This project is made in following steps:

1. Construct interpolated waypoints
2. Getting data from sensors and generating prediction
3. calculate suitable path

## 1. Construct interpolated waypoints
There is a file that is called `highway_map.csv`. I have uploaded this file into the `data` project. So at first we should read this file and try to interpolate bunch of point around the car. I have created a header file which is named `constants.h
` and I have put some variable in this file. For exampl, `PREVIOUS_PATH_POINTS_TO_KEEP` is defined for determining the number of points which are supposed to use for interpolatore from behind of the car. So we can create a path with maximum size of this parameter. It help us to create smooth path for car movement. So it is generated a set of tight spaces that can help for making more accuracy. 
For working on this project I have used a simulator which was created by Udacity. It can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). The simulator returns some points from prevoiuse generated path. Already, previouse path is important for finding the current state of the car and contructing future plan. So if you want to upldate the state of the car you should be considered that there is a class that is named `Vehicle` which has some usefull methods for changin lane and generating path. If you want to move and keep the car in the own lane, or if you want to change the lane of the car into left or ritgh, you can use `update_available_states` method. This method has 2 arguments, one is used for change lane to the left and another is used for right. 

## 2. Getting data from sensors and generating prediction
If you run the simulator, you can receive sensor data and all trajectories for each car in each iteration. These trajectories are used in combination with a set of cost functions to prepare the best trajectory for the vehicle. The states of each car are updated based on the current position. You can use this data for preventing ti change lane if there would be a car immediately in the destination lane. Each state consists of position, velocity, and acceleration in both S and D dimensions. As I have mentioned, Each trajectory is evaluated according to a set of cost functions, and the best trajectory is selected if its const function would be lowest that other trajectories. In this implementation, there are some parameters for the cost functions:
* Collision: punishes a trajectory that collides with any predicted traffic trajectories.
* Buffer: punishes a trajectory that comes within a certain distance of another traffic vehicle trajectory.
* In-lane buffer: punishes driving in lanes with relatively nearby traffic.
* Efficiency: punishes trajectories with lower target velocity.

## 3. calculate suitable path
The new path consists of a certain number of points from the previous path, which is received from the simulator at each iteration. A spline is generated with the last two points of the previous path. Be consider that in this implementation, each generated point is stored for use in the future. If there is not any path in the current state, it is used from the current position, heading, and velocity. The end of the path consists of two points 45 meters on average ahead. To prevent additional acceleration, the velocity is incremented or decremented by a small amount, and the corresponding next points are calculated along the x and y splines created earlier. The path planner has been managed to accumulate incident-free runs of over four miles multiple times. You can watch the prepared [video](https://www.youtube.com/watch?v=z6p7Zyk8W8E).

## Dependencies
If you want to run this project, you have to install some dependencies ad below:
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


