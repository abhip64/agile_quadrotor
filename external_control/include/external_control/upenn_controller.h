
#ifndef UPENN_CONTROLLER_H
#define UPENN_CONTROLLER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "quadrotor_sim/math_operations.h"
#include "external_control/controller_class.h"


class upenn_controller: public control_class{

private:

    ros::NodeHandle nh_;

    //Eigen::Matrix3d J;

    double Kpos_x_, Kpos_y_, Kpos_z_;
    double Kvel_x_, Kvel_y_, Kvel_z_;
    double Kr, Kw;

    Eigen::Vector3d Kpos_, Kvel_, Kr_, Kw_;

    //Acceleration due to gravity in NED frame
    Eigen::Vector3d g_;

    double norm_thrust_const_;
    double max_fb_acc_;

public:

	upenn_controller(const ros::NodeHandle& nh);
    
    virtual ~upenn_controller();

	Eigen::Vector4d calculate_control_fb(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, double&, Eigen::Vector4d&, Eigen::Vector4d&);
	Eigen::Vector4d attcontroller(const Eigen::Vector4d&, const Eigen::Vector3d&, Eigen::Vector4d&, Eigen::Vector3d&, Eigen::Vector3d&, double);

};



#endif