# Agile Quadrotor Simulation

## Description
This ROS package is meant to be used as a tool to test out several agile/aggressive maneuvers using a simulated model of a 
quadrotor. The simulation has been implemented in ROS-Gazebo simulation environment using PX4-SITL. The main objective of
this package is to provide a platform for testing out various trajectory generation techniques and control stratgies for
effectvely executing a manuever. More details about the working of each component of the algorithm is discussed in detail
in the wiki section of the project. Special mention is required for the package by Jaeyoung-Lim https://github.com/Jaeyoung-Lim/mavros_controllers. The original code has been forked from his repository and I got many insights
pertaining to the structure of the code from his work. 

## System Specifications
The simulation platform was designed and tested in the following work environment:
* Ubuntu 16.04 running on a system with Intel Core i7 CPU @ 2.20GHz × 8 and 8GB RAM
* ROS Kinetic
* Gazebo 7.0.0
* PX4 SITL 


## Installation
The detailed instructions for setting up the simulation environement have been descirbed in the wiki associated with this project. The method of installation of the package and its dependencies will be descirbed here.

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src  
```
### apriltag_ros 
April Tag detection algorithm and its associated ROS wrapper is obtained from https://github.com/AprilRobotics/apriltag_ros. The installation instructions for the package are provided.
```shell
cd ~/catkin_ws/src  
git clone https://github.com/AprilRobotics/apriltag.git
git clone https://github.com/AprilRobotics/apriltag_ros.git
cd ~/catkin_ws 
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

### robot_localisation 
robot_localisation package is used for implementing Kalman filter. More details of the package can be found in https://github.com/cra-ros-pkg/robot_localization/tree/kinetic-devel. Some more insights relating to the functioning and usage of the package can be found at https://kapernikov.com/the-ros-robot_localization-package/. 

```shell
cd ~/catkin_ws/src  
git clone https://github.com/cra-ros-pkg/robot_localization.git
cd ~/catkin_ws 
catkin build
```

### ackermann_vehicle 

```shell
cd ~/catkin_ws/src  
git clone https://github.com/abhip64/ackermann_vehicle.git
git clone https://github.com/abhip64/ackermann_msgs.git
cd ~/catkin_ws 
catkin build
```

### Agile-Quadrotor 
```shell
cd ~/catkin_ws/src  
git clone https://github.com/abhip64/Agile-Quadrotor.git
cd ~/catkin_ws 
catkin build
```

After all the packages have been succesfully built, it is necessary to setup the configuration files in apriltag_ros and robot_localisation packages to work with my package. The files that need to be changed are given in the "Auxiliary Files" (https://github.com/abhip64/Agile-Quadrotor/tree/master/auxiliary_files) folder in agile_quad package. Execute the following commands for updating the configuration files.

```shell
cp -f ~/catkin_ws/src/agile_quad/auxiliary_files/apriltag/continuous_detection.launch ~/catkin_ws/src/apriltag_ros/apriltag_ros/launch/continuous_detection.launch
cp -f ~/catkin_ws/src/agile_quad/auxiliary_files/apriltag/settings.yaml ~/catkin_ws/src/apriltag_ros/apriltag_ros/config/settings.yaml
cp -f ~/catkin_ws/src/agile_quad/auxiliary_files/apriltag/tags.yaml ~/catkin_ws/src/apriltag_ros/apriltag_ros/config/tags.yaml
cp -f ~/catkin_ws/src/agile_quad/auxiliary_files/robot_localisation/ekf_template.yaml ~/catkin_ws/src/robot_localization/params/ekf_template.yaml
```

## Usage

The current version of the project implements three types of maneuvers making use of external geometric controller. The details on changing the parameters of these maneuvers are discussed in detail in the wiki associated with the project.

### Circular Trajectory
```shell
roslaunch quadrotor_sim quadrotor_circle_trajectory.launch
```

### Slit Traversal Maneuver
```shell
roslaunch quadrotor_sim quadrotor_slit_trajectory.launch
```

### UGV Tracking and Following
```shell
roslaunch quadrotor_sim quadrotor_ugv_follow_maneuver.launch 
```





