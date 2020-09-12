
#ifndef FLIP_MANEUVER_H
#define FLIP_MANEUVER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "maneuvers/trajectory_class.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "maneuvers/eth_trajectory.h"

class flip_traverse: public trajectory_class{

private:

	ros::NodeHandle nh_;

	Eigen::Vector3d target_position;
	Eigen::Vector3d target_velocity;
	Eigen::Vector3d target_acceleration;
	Eigen::Vector3d target_jerk;
	Eigen::Vector3d target_angvel;
	double 			target_yaw;
	int 			type;

	Eigen::Vector3d mavVel_;
	Eigen::Vector3d mavPos_;

	double g;

public:

	flip_traverse(const ros::NodeHandle& nh);
    
    ~flip_traverse();

    ros::Subscriber mavposeSub_, mavtwistSub_;

    //Constant thrust to weight magnitude to be mainteined during flip
    double Tc;

    //Constant energy maintained during flip
    double energy;

    //Radius of curvature  
    double r;

    double pitch_angle;

    double curr_vel;

    double T1, T2;

    Eigen::Vector3d flip_init_pos;

    Eigen::Vector3d flip_init_vel;

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

	void mavposeCallback(const geometry_msgs::PoseStamped& );
 	void mavtwistCallback(const geometry_msgs::TwistStamped& );  


};



#endif