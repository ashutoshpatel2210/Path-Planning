# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./PathPlanning.PNG "Simulator Output"

## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Implementation

My implementation heavily relies on the usage of Behaviour planning (defining state machine), prediction and trajectory (spline function), which fits a line to given points in a smooth function.

## Compilation
The code compiles correctly. No changes were made in the cmake configuration. A new file is added src/spline.h. It is the Cubic Spline interpolation implementation

## Valid Trajectories

### The car is able to drive at least 4.32 miles without incident.

I ran simulatior for 10 miles and it is running without any incident

![alt text][image1]

### The car drives according to the speed limit.

No red flags are observed for speed limit. Car drives according to speed limit and most of time at ~49.5MPH. During lane changes or when vehicle in front of car , then speed is going down. 

### Max Acceleration and Jerk are not Exceeded. 

No red flags are observed for acceleration and jerk.

### Car does not have collisions.

As per Image, Car does not have collisions till 10 Miles.

## The car stays in its lane, except for the time between changing lanes.

Car always stays in its lane and changed the lanes when vehicle is in front of car and left or right lane is empty.

### The car is able to change lanes

when vehicle is in front of car and left or right lane is empty, Car changes lanes smoothly, no max jerk or acceleration red flag is observed.  

## Reflection

### There is a reflection on how to generate paths.

#### Details

The code consists of 3 parts 

1. Prediction - Check car in the environment too close or near in left or right lane. 
2. Behaviour Planning - Define state machine , how car will move based prediction.
3. Trejectory - Calculation of trejectory based on speed and behaviour output. 

##### Prediction

This part of code (line 108: 147) predicts the cars with respect to subject based on telemetry and sensor fusion data. It deals with following 

1. Is there a car in front of our car?
2. is there a car in right lane and lane changing safe or not?
2. is there a car in left lane and lane changing safe or not?

A car in above 3 considered as not safe situation if our car is within 30m in front of us or behind.

##### Behaviour Planning

This part of code (line 148: 176) deals with state machine of car based on prediction. For the project, I selected 3 states: Keep car in same lane, Change left lane or change right lane. 

The code defines state machine like this. 

1. if there is car in front of our car, then
	1. Switch to right lane if there is no car in right side and it is not going beyond lane boundary.
    2. Switch to left lane if there is no car in left side and it is not going beyond lane boundary.
    3. If no other option is available, then slow down car speed. 
    
It can increase the speed gradually if no car in front of us and try to keep it in center. 

##### Trajectory

This code (line 179:282) does the calculation of the trajectory based on the speed and behaviour output for lane, car coordinates and past path points.
The code uses spline function, which fits a line to given points in a smooth function. The speed change is decided on the behavior part, and it is used in that part(ref_vel) to increase/decrease speed on every trajectory points instead of doing it for the complete trajectory.
