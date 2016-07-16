# ITOMP-EXEC
re-implemented ITOMP for Gazebo or real robot execution

## Requirements
* Ubuntu 14.04
* ROS indigo
* ROS Eigen
* Moveit!
* openni_launch
 * $ sudo apt-get install ros-indigo-openni-launch
* openni_tracker
 * $ sudo apt-get install ros-indigo-openni-tracker
* dlib (http://dlib.net/)
 * put dlib include directory to itomp_exec/lib/
* pcml

## Components
* msgs
 * FutureObstacleDistribution
 * FutureObstacleDistributions
* Nodes
 * future_obstacle_publisher  
     **Parameters**  
       input_stream_type (string, default: realtime)  
       \* One of "realtime", "cad120"  
       joints_type (string, default: upper_body)  
       \* One of "whole_body", "upper_body"  
       render (bool, default: false)  
       cad120_directory (string, required when input_stream_type = "cad120")  
     **Subscribed Topics**  
       TF topics published by openni_tracker (/head_1, /neck_1, etc.)
     **Published Topics**  
       future_obstacle_publisher/future_obstacle_distributions (pcml/FutureObstacleDistributions)
 * future_obstacle_visualizer  
     **Subscribed topics**  
       future_obstacle_publisher/future_obstacle_distributions (pcml/FutureObstacleDistributions)  
     **Published Topics**  
       /future_obstacle_distributions_marker_array (visualization_msgs/MarkerArray)
* Launches
 * test_fetch.launch use_real_robot:=(true/false) use_gazebo:=(true/false) use_rviz:=(true/false) use_benchmark1:=(true/false) use_benchmark2:=(true/false)  
  \* Set either use_real_robot or use_gazebo to true  
  \* Set either use_benchmark1 or use_benchmark2 to true  
  \* use_rviz = true as default
 * itomp_ur5.launch  
  \* Install UR5 gazebo (http://wiki.ros.org/ur_gazebo)  
  \* need to change controller's name to 'arm_controller'. The file is at ur5_moveit_config/config/controllers.yaml

## Build
* Package is organized using catkin  
  $ roscd  
  $ catkin_make

