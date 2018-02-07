# Path Planning Project

## Overview
The aim of this project is to navigate a car through traffic in a highway scenario by generating appropriate trajectories and planning behaviors according to traffic rules. The simulator provides information about the ego vehicle localization data as well as sensor fusion data, with localization data of other vehicles it senses.

The car's localization data incluse the following measurements:
1. x and y global map co-ordinates
2. s and d frenet coordinates
3. yaw
4. speed

The Sensor fusion data including the localization data of other vehicles is as follows:
[car ID, car's x position in map coordinates, car's y position in map coordinates,
 car's x velocity in m/s, car's y velocity in m/s, 
 car's s position in frenet coordinates, car's d position in frenet coordinates]

 The code should output next `x` and `y` desired values (global co-ordinates) that the car will follow. The trajectory generated should be such that the normal, tangential and total accelerations, and jerk are within the maximumum allowable limits. Further, the maximum allowable speed limit is 50 miles/hour.

## Trajectory generation

When we provide the points for the car to follow the car will go between two points within 20 milliseconds. In order that we follow the 50 MPH speed limit we need to generate points which are 0.5m apart. Since the car moves 50 times a second, a distance of 0.5m per move will create a velocity of 25 m/s. 25 m/s is close to 50 MPH. 

We will generate a vector of 50 co-ordinate pairs with next `x` and `y` locations. The number of points to be generated is given by:

`N = target_dist/(time_delta * ref_vel/2.24);`

Here, `target_dist` is an reference distance, `time_delta` is the time in which the car will move between two points(20ms), and `ref_vel` is the target speed.

We use the cubic spline C++ implementation at http://kluge.in-chemnitz.de/opensource/spline/ to generate smooth transition between points. This spline is generated from three calculated `s` points on the trajectory which are 30m apart and the `d` position is generated according to the desired lane location. We convert these three points to the cartesian co-ordinate system `x` and `y`.
```
/In Frenet add evenly 30m evenly spaced points ahead of the starting reference
vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
``` 

Using the points generated so far we can create a spline as follows:
```
tk::spline s;
 s.set_points(pts_x, pts_y);
```

Next, we convert the points generated to to car's local frame of reference to make it math-easy for further calculations by essestially keeping the car's yaw angle in line with the x-axis. Now that we have the spline, we further generate the `N` equally spaced points to be fitted to the spline. We can generate the 'y' co-ordinate of a point in the spline by evaluating it on a known 'x' coordinate:
`double y_point = s(x_point);`

We try maintaining a trajectory vector of 50 points and use previous points we recieve from the simulator and only generate additional points as required to complete the set of 50 points. This also helps in keeping the trajectory smooth and avoid abrupt changes which would be the case of we wouldn't use the previous path points. Finally we convert the local car co-ordinates to global or map co-ordinates:
```
//convert back from local to global coordinates
x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

x_point += ref_x;
y_point += ref_y;

next_x_vals.push_back(x_point);
next_y_vals.push_back(y_point);
```

## Navigation

A simple state machine is implemented for lane navigation through highway traffic environment. We define three flags `car_ahead`, `car_in_left_lane` and `car_in_right_lane` and set these flags to true once we detect cars in any lane as reported by the sensor fusion localization data. We have defined five states as `KEEP_LANE`, `SLOW_DOWN`, `CHANGE_LEFT`, `CHANGE_RIGHT`, `KEEP_SPEED`. We have declared two more variables acting as range of detection: 
`detection_range` and `detection_range_behind`. When we a vehicle comes within this range we find out in which lane the car is using the `which_lane()` function.

The ego vehicle starts in the center lane and keeps the lane until a slower moving car comes in its way. It then checks if there is any incoming vehicle on the left lane first, since passing should be done ideally from the left lane (fast lane). If there is no vehicle ahead, it executes a lane change on the left. If there is a vehicle on the left lane then it tries to execute a lane change on the right if it is safe to do so. If all the lanes are blocked it will keep the lane and keep speed. If the car is in the right most lane and the road on the left (center lane) is empty it will move into the center lane. This gives us two options for future lane changes in case a vehicle obstructs.

