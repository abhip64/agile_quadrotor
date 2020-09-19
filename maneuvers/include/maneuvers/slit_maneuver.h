
#ifndef SLIT_MANEUVER_H
#define SLIT_MANEUVER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "maneuvers/trajectory_class.h"
#include "maneuvers/eth_trajectory.h"

class slit_traverse: public trajectory_class{

private:

	ros::NodeHandle nh_;

	Eigen::Vector3d target_position;
	Eigen::Vector3d target_velocity;
	Eigen::Vector3d target_acceleration;
	Eigen::Vector3d target_jerk;
	Eigen::Vector3d target_angvel;
	double 			target_yaw;
	int 			type;

	Eigen::Vector3d g_;


	
public:

	slit_traverse(const ros::NodeHandle& nh);
    
    ~slit_traverse();

    double slit_roll;
    double slit_pitch;

    Eigen::Vector3d slit_pos;

	Eigen::Vector3d init_pos, init_vel;

	Eigen::Vector3d final_pos, final_vel;

    void trajectory_generator(double);

    double maneuver_init(double);

    Eigen::Vector3d calculate_trajectory_angvel();

	Eigen::Vector3d get_target_pos();
	Eigen::Vector3d get_target_vel();
	Eigen::Vector3d get_target_acc();
	Eigen::Vector3d get_target_jerk();
	Eigen::Vector3d get_target_angvel();
	double		    get_target_yaw();
	int 			get_type();

};



#endif