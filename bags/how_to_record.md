1. start recording
```
rosbag record -a -O testing_rosbag_recording.bag
```
or for smaller file
```
rosbag record -O testing_rosbag_recording.bag /map /move_base/RRTPlannerROS/global_plan /move_base/DWAPlannerROS/local_plan /amcl_pose /particlecloud /scan /move_base/global_costmap/costmap /move_base/current_goal /move_base/local_costmap/costmap /tf /tf_static /frame_path_est
```
2. start sim or robot control
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
or
```
```
3. start localization 
```
roslaunch rrt_planner rrt_rosbag.launch
```
4. start teleop
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

5. calibrate the local stop teleop and run 2d nav goal
6. stop recording when finished