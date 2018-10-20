# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program  
Video of this code in action: https://youtu.be/DicghCRyzKU

[//]: # (Image References)
[image1]: ./PathPlanner.png

![alt text][image1]

## Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
   
### Model Documentation
This project requires a simulated car to navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit, with a variety of constraints on safety and passenger comfort. This documentation discusses the design of a path planner to complete this task.

### References
The classroom walkthrough was very helpful, as is the spline library referenced in the classroom: https://kluge.in-chemnitz.de/opensource/spline/. I used both extensively.

### Behaviours
The path planner has three main behaviours: lane keeping, waiting to change, and changing. I implemented features in the following order, building up to a path planner fully capable of these high level behaviours:

1. basic lane follower
2. trajectory smoothing using splines
3. the ability to change lanes (but not to decide to do so)
4. the ability to regulate speed, instead of just setting a constant speed
5. logic for deciding to change lanes

### Lane Keeping
Lane keeping is the default state of the path planner. It can only transition to "waiting to change".
This part of the code is based on the classroom walkthrough, however several things are different in my implementation.
The steps to generating a lane keeping trajectory are:

1. Get 2 waypoints fromt the end of the previous path (this keeps the path continuous)
2. Add some waypoints ahead of the previous path using Frenet coordinates
3. Convert to a "local" reference frame (end of the previous path)
4. Create a spline from these waypoints
5. Using the spline, generate a trajectory up to some horizon distance
6. Convert back to global coordinates
7. Feed into simulator.

Steps 1 through 4 are completed between lines 420 and 490 in main.cpp.
Splines are useful because they a smooth and continuous, which means that if we use them carefully we avoid large acceleration and jerk which are uncomfortable for passengers.
Local coordinates simplifies a lot of the maths, but also prevents the situation where you try to fit a spline to a relation that is not a function, i.e. x[i] > x[i+1]. This works as long as the turn is not too sharp and the horizon is not too long.

One change compared the walkthough is that here I use a constant horizon distance (and max trajectory points limit) instead of a constant number of trajectory points. I find this is more stable and easier to generate consistent lane changes, since new trajectory points are always added to the end of the old points. This is implemented on line 556 of main.cpp.

The only other capability required to successfuly lane-keep is speed regulation. Without this, the car will stay in it's lane but bulldoze all the other cars out of its way. I achieved this by regulating the distance between trajectory points fed to the simulator. The target speed is normally set to maximum (21m/s), but if a vehicle is close in front, the target speed is set to match that vehicle. The speed of each subsequent waypoint is then increased or decreased by 0.75 of the maximum allowable acceleration until the target speed is reached. Waypoint speed and distance are related by the constant waypoint time gap, 0.02s. Speed regulation is done between lines 527 abd 248 of main.cpp.

### Waiting to Change
The "waiting to change" state is entered from "lane keeping" when there is a slower car blocking the current lane. This state is marked in the code by line 355 of main.cpp. In this state, the path planner evaluates potential lane changes, and if it is safe to do a lane change, this state will decide which lane to change to and then execute that change. Evaluating lane changes requires filtering the sensor data.

Sensor data is filtered by lane, and by close and far vehicles. After filtering, I end up with six numbers: how many vehicles that are close (blocking) in each lane, and how many vehicles are further ahead in each lane.

The criteria for changing lanes in the outer two lanes is simple: if the current lane is blocked and the adjacent is not, change, otherwise stay. The car will match the speed of the vehicle in front until there is room to change and then change to the middle lane.

Changing from the middle lane is slighly complicated by the choice of which lane to change to. For this, I implement a cost function for each lane, found on line 402 of main.cpp. It uses the following logic:

Left Lane Cost = 10 * #(close vehicles in lane) + #(further ahead vehicles in lane)
Middle Lane Cost = 9
Right Lane Cost = 10 * #(close vehicles in lane) + #(further ahead vehicles in lane)

The path planner chooses the lane with the lowest cost. If there are close vehicles blocking a lane change to both of the left and right lanes, they will cost more than staying in the middle lane so it will stay. Otherwise, it will choose the lane to change to based on how many vehicles are lane further ahead - the more empty lane is chosen. This cost function seems to work fairly well in practice.

### Lane Change
Lane changing is actually handled mostly by the spline function. The 'waiting to change' state will output a target lane when it has decided it is safe to change, and this is used in the waypoints that the spline is fitted to in lines 451-459 of main.cpp. 


