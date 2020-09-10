
#ifndef SLIT_MANEUVER_H
#define SLIT_MANEUVER_H

#include <ros/ros.h>
#include <Eigen/Dense>

class slit_traverse: public trajectory_class{

private:

	ros::NodeHandle nh_;

public:

	slit_traverse();
    
    ~slit_traverse();

    float gap_roll;
    float gap_pitch;

	Eigen::Vector3d trajectory_pos(double);
	Eigen::Vector3d trajectory_vel(double);
	Eigen::Vector3d trajectory_acc(double);
	Eigen::Vector3d trajectory_jerk(double);
	Eigen::Vector3d trajectory_angvel(double);

};



#endif