# Agile Quadrotor Simulation

This ROS package is meant to be used as a tool to test out several agile/aggressive maneuvers using a simulated model of a 
quadrotor. The simulation has been implemented in ROS-Gazebo simulation environment using PX4-SITL. The main objective of
this package is to provide a platform for testing out various trajectory generation techniques and control stratgies for
effectvely executing a manuever. More details about the working of each component of the algorithm is discussed in detail
in the wiki section of the project. Special mention is required for the package by Jaeyoung-Lim https://github.com/Jaeyoung-Lim/mavros_controllers. 

## Specifications
The simulation platform was designed and tested in the following work environment:
* Ubuntu 16.04 running on a system with Intel Core i7 CPU @ 2.20GHz × 8 and 8GB RAM
* ROS Kinetic
* Gazebo 7.0.0
* PX4 SITL 


## Installation
Installation steps have been described for a clean Ubuntu system.

### ROS Installation
ROS installation is done by following the steps given in http://wiki.ros.org/kinetic/Installation/Ubuntu. I have provided here the steps that I have followed. It would be best practice to follow the latest steps that are given in the ROS wiki as the security key can get updated. When using the steps given in ROS wiki make sure to install the **Desktop-Full Install** option.

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-kinetic-desktop-full
```

```shell
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

```shell
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo apt install python-rosdep

sudo rosdep init
rosdep update
```

Now for setting up Catkin build environment for building the ROS package, the installation instructions are taken from https://catkin-tools.readthedocs.io/en/latest/installing.html.

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get update


sudo apt-get install python-catkin-tools
```




