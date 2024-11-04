# object_tracking_2d
ROS 2 package for object tracking using 2D laser scan.

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


## :gear: Parameter Tuning
### Laser scan filter
- ```frequency``` ... This has to be higher than odometry frequency 
- ```cluster_angle_tolerance, cluster_dist_tolerance``` ... Local minimums which are positioned within ```cluster_angle_tolerance``` or ```cluster_dist_tolerance``` from the previous one do not join object candidates.

### Kalman filter
Carefully define system and sensor noise for optimal tracking accuracy.

Others are tracking parameters.
- ```frames_limit``` ... Tracks are not longer tracked if they are not corrected for more than frames_limit. 
- ```velocity_limit``` ... Tracks with higher than vleocity_limit are deleted.
- ```time_horizon``` ... This may be usefull for specific project. Predicted tracks in time_horizon seconds future are published.