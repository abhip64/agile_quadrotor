
#ifndef CIRCLE_MANEUVER_H
#define CIRCLE_MANEUVER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "maneuvers/trajectory_class.h"

class circle_traverse: public trajectory_class{

private:

	ros::NodeHandle nh_;

	Eigen::Vector3d target_position;
	Eigen::Vector3d target_velocity;
	Eigen::Vector3d target_acceleration;
	Eigen::Vector3d target_jerk;
	Eigen::Vector3d target_angvel;
	double 			target_yaw;
	int 			type;

public:

	circle_traverse(const ros::NodeHandle& nh);
    
    ~circle_traverse();

    //Radius of the circle to be traversed
    double circle_radius;

    //Angular Velocity of the quadrotor while traversing the circle
    double circle_ang_velocity;

    //Height of the trajectory from ground
    double circle_height;

    //Number of Revolutions
    int circle_rev;

    void trajectory_generator(double);

    double maneuver_init();

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