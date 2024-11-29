# object_tracking_2d
***THIS DOCUMENTATION WAS MADE AUTOMATICALLY***  
ROS 2 package for object tracking using 2D laser scan.
## :gear: Prepare
```
```
## :rocket: Quick Start !
```
ros2 launch object_tracking_2d object_tracking_2d.launch.py
```
## :thinking: How ?
#### Key Word : Kalman filter, Hungarian, Local minimum 

This object tracking system leverages Kalman filtering for state estimation and the Hungarian algorithm for data association.
1. Compute local minimums  
    Local minimums are found from 2D LiDARs. They are candidates of objects to be tracked. Some pre-process are there to use 2D laser scan data. See [src/laser_scan_filter.cpp](./include/object_tracking_2d/laser_scan_filter.hpp).
2. Estimate state of your system  
    This system assumes a constant-velocity model. Define the system, observation equations, and noise. See [kalman_filter](./kalman_filter/) and [src/system.cpp](./src/system.cpp).
3. Associate a track with an observation  
    The Hungarian algorithm handles frame-to-frame associations, critical for consistent tracking.

## /tracking/lidar_object_tracker node
### Required Topic
- *`/scan:`* (sensor_msgs/msg/LaserScan)  
### Published Topic
- *`/tracking/objects:`* (track_msgs/msg/TrackArray)  
- *`/tracking/objects/dynamic/position:`* (visualization_msgs/msg/MarkerArray)  
- *`/tracking/objects/dynamic/position/covariance:`* (visualization_msgs/msg/MarkerArray)  
- *`/tracking/objects/dynamic/trajectory:`* (visualization_msgs/msg/MarkerArray)  
- *`/tracking/objects/dynamic/velocity:`* (visualization_msgs/msg/MarkerArray)  
- *`/tracking/objects/static/position:`* (visualization_msgs/msg/MarkerArray)  
- *`/tracking/objects/static/position/covariance:`* (visualization_msgs/msg/MarkerArray)  
- *`/tracking/objects/static/trajectory:`* (visualization_msgs/msg/MarkerArray)  
- *`/tracking/objects/static/velocity:`* (visualization_msgs/msg/MarkerArray)  

### Required Param
- *`angle_max`* (double, default: )  
  **Description**:  max angle in a horizontal direction to object to be detected  

- *`angle_min`* (double, default: )  
  **Description**:  min angle in a horizontal direction to object to be detected  

- *`cluster_angle_tolerance`* (double, default: )  
  **Description**:  gap between two objects in a horizontal direction  
  **Additional Constraints**: highly depending on situation (e.g. large number under the situation that there are many objects) 

- *`cluster_dist_tolerance`* (double, default: )  
  **Description**:  gap between two objects in a radial direction  
  **Additional Constraints**: highly depending on situation (e.g. large number under the situation that there are many objects) 

- *`cluster_points`* (integer, default: )  
  **Description**:  clustering an object  
  **Additional Constraints**: resolution of laserscan, ma_interval and cluster_points highly depend on each other 

- *`frames_dyn2sta`* (integer, default: )  
  **Description**:  the number of frames to be changed from dynamic to static  

- *`frames_limit`* (integer, default: )  
  **Description**:  frame out if object has not been corrected for over this  

- *`frames_sta2dyn`* (integer, default: )  
  **Description**:  the number of frames to be changed from static to dynamic  

- *`frequency`* (integer, default: )  
  **Description**:  detection callback frequency  
  **Additional Constraints**: has to be less than lidar scan frequency 

- *`lidar_frame`* (string, default: )  
  **Description**:   

- *`ma_interval`* (integer, default: )  
  **Description**:  ave of ma_interval samples for moving average pre-process  
  **Additional Constraints**: resolution of laserscan, ma_interval and cluster_points highly depend on each other 

- *`matching_dist`* (double, default: )  
  **Description**:  matching with observation within this distance  

- *`max_track_num`* (integer, default: )  
  **Description**:   

- *`observation_noise_variance`* (double, default: )  
  **Description**:  variance wrt observation noise  

- *`odom_frame`* (string, default: )  
  **Description**:   

- *`position_error_variance`* (double, default: )  
  **Description**:  variance wrt position error  

- *`position_noise_variance`* (double, default: )  
  **Description**:  variance wrt position noise  

- *`range_max`* (double, default: )  
  **Description**:  max distance from lasescan frame to object to be detected  

- *`range_min`* (double, default: )  
  **Description**:  min distance from sensor frame to object to be detected  

## :books: Reference
- []()