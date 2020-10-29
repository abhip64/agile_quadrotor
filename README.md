# Agile Quadrotor Simulation

## Description
This ROS package is meant to be used as a tool to test out several agile/aggressive maneuvers using a simulated model of a 
quadrotor. The simulation has been implemented in ROS-Gazebo simulation environment using PX4-SITL. The main objective of
this package is to provide a platform for testing out various trajectory generation techniques and control strategies for
effectively executing a manuever. More details about the working of each component of the algorithm is discussed in detail
in the wiki section of the project. Special mention is required for the package by Jaeyoung-Lim https://github.com/Jaeyoung-Lim/mavros_controllers. The original code had been forked from his repository and I got many insights
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
git clone -b kinetic-devel https://github.com/cra-ros-pkg/robot_localization.git
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

### agile_quadrotor 
```shell
cd ~/catkin_ws/src  
git clone https://github.com/abhip64/agile_quadrotor.git
cd ~/catkin_ws 
catkin build
```

After all the packages have been succesfully built, it is necessary to setup the configuration files in apriltag_ros and robot_localisation packages to work with my package. The files that need to be changed are given in the "Auxiliary Files" (https://github.com/abhip64/Agile-Quadrotor/tree/master/auxiliary_files) folder in Agile-Quadrotor package. Execute the following commands for updating the configuration files.

```shell
cp -f ~/catkin_ws/src/agile_quadrotor/auxiliary_files/apriltag/continuous_detection.launch ~/catkin_ws/src/apriltag_ros/apriltag_ros/launch/continuous_detection.launch
cp -f ~/catkin_ws/src/agile_quadrotor/auxiliary_files/apriltag/settings.yaml ~/catkin_ws/src/apriltag_ros/apriltag_ros/config/settings.yaml
cp -f ~/catkin_ws/src/agile_quadrotor/auxiliary_files/apriltag/tags.yaml ~/catkin_ws/src/apriltag_ros/apriltag_ros/config/tags.yaml
cp -f ~/catkin_ws/src/agile_quadrotor/auxiliary_files/robot_localisation/ekf_template.yaml ~/catkin_ws/src/robot_localization/params/ekf_template.yaml
```

Also add this line to the bashrc file to ensure gazebo finds the sdf models of the quadrotor

```shell
export GAZEBO_MODEL_PATH=~/catkin_ws/src/agile_quadrotor/quadrotor_sim/models:${GAZEBO_MODEL_PATH}
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

## References

<a id="1">[1]</a> 
D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," 2011 IEEE International Conference on Robotics and Automation, Shanghai, 2011, pp. 2520-2525, doi: 10.1109/ICRA.2011.5980409.

<a id="2">[2]</a>
C. Richter, A. Bry, and N. Roy, “Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments,” in International Journal of Robotics Research, Springer, 2016.

<a id="3">[3]</a>
T. Moore and D. Stouch, "A Generalized Extended Kalman Filter Implementation for the Robot Operating System", Proceedings of the 13th International Conference on Intelligent Autonomous Systems (IAS-13), 2014



