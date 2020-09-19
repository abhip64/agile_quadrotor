
#ifndef ETH_TRAJECTORY_H
#define ETH_TRAJECTORY_H

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>


void eth_trajectory_init(mav_trajectory_generation::Vertex::Vector, std::vector<double>, int);


Eigen::Vector3d eth_trajectory_pos(double time);

Eigen::Vector3d eth_trajectory_vel(double time);

Eigen::Vector3d eth_trajectory_acc(double time);

Eigen::Vector3d eth_trajectory_jerk(double time);

#endif