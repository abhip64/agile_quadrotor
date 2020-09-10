
#ifndef MATH_OPERATIONS_H
#define MATH_OPERATIONS_H

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#define deg_to_rad M_PI/180.0

#define rad_to_deg 180.0/M_PI

Eigen::Vector3d vee_map(const Eigen::Matrix3d );

Eigen::Vector4d quatMultiplication(const Eigen::Vector4d&, const Eigen::Vector4d&);

Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d );

Eigen::Vector4d acc2quaternion(const Eigen::Vector3d, double);

Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d );

Eigen::Vector3d toEigen(const geometry_msgs::Point& );

Eigen::Vector3d toEigen(const geometry_msgs::Vector3& );

Eigen::Vector3d quat2euler(const Eigen::Vector4d&);

#endif
