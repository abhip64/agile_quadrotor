
#ifndef ETH_TRAJECTORY_H
#define ETH_TRAJECTORY_H

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>


void eth_set_pos(Eigen::Vector3d, Eigen::Vector3d);

void eth_set_vel(Eigen::Vector3d, Eigen::Vector3d);

void eth_set_acc(Eigen::Vector3d);

double eth_trajectory_init(double);

Eigen::Vector3d calc_inter_pos(Eigen::Vector3d);

Eigen::Vector3d eth_trajectory_pos(double time);

Eigen::Vector3d eth_trajectory_vel(double time);

Eigen::Vector3d eth_trajectory_acc(double time);

Eigen::Vector3d eth_trajectory_jerk(double time);

Eigen::Vector3d eth_trajectory_angvel(double time);


#endif