
#ifndef UGV_FOLLOW_MANEUVER_H
#define UGV_FOLLOW_MANEUVER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "maneuvers/trajectory_class.h"
#include "maneuvers/eth_trajectory.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"

class ugv_follow: public trajectory_class{

private:

	//ROS NodeHandle
	ros::NodeHandle nh_;

	//Trajectory Targets for the controller
	Eigen::Vector3d target_position;
	Eigen::Vector3d target_velocity;
	Eigen::Vector3d target_acceleration;
	Eigen::Vector3d target_jerk;
	Eigen::Vector3d target_angvel;
	double 			target_yaw;
	int 			type;

	//UGV Current State Parameters
	Eigen::Vector3d ugv_position;
	Eigen::Vector3d ugv_velocity;
	Eigen::Vector3d ugv_acceleration;
	Eigen::Vector4d ugv_attitude;
	double 			ugv_yaw;

	//UAV current state parameters
	Eigen::Vector3d mavPos_, mavVel_;

	//UGV future state parameters
	Eigen::Vector3d ugvpos_future, ugvvel_future;

	//Acceleration due to gravity
	Eigen::Vector3d g_;

	//Time to intersection of trajectories of the UGV and quadrotor
	double time_to_intersection;

	//Current distance between the UGV and the quadrotor
	double dist_to_ugv;

	//Mean velocity at which the quadrotor is assumed to follow the UGV
	double ugv_follow_velocity;

	//Subscriber definitions
	ros::Subscriber vehicle_posSub_, accelSub_, mavPoseSub_, mavtwistSub_;

	//Variable representing the phase of operation of the ugv tracking maneuver. Tracking and 
	//following of the UGV is carried out in two phases. The manuever starts with the quadrotor trying
    //to catch up with the UGV. This becomes phase 1. In the next phase the quadrotor tries to closely 
    //follow the UGV at a given height by making use of onboard camera to detect the UGV
	int phase;

	//Dummy variable for storing time
	double elasped_time;

	//Height at which the quadrotor follows the UGV
	double hover_height;

	//To see if user has commanded landing 
	int ugv_land_init;

	//Start time of the landing operation
	ros::Time start_time;
	
public:

	//Constructor
	ugv_follow(const ros::NodeHandle& nh);
    
    //Destructor
    ~ugv_follow();

    //Function to generate and store the trajectory targets for the controller
    void trajectory_generator(double);

    //Function to intialise the trajectory for completing the manuever
    double maneuver_init(double);

    //Function to calculate the desired angular velocity making using of the differential
    //flatness property of the quadrotor
    Eigen::Vector3d calculate_trajectory_angvel();

    //Functions to return the desired trajectory targets 
	Eigen::Vector3d get_target_pos();
	Eigen::Vector3d get_target_vel();
	Eigen::Vector3d get_target_acc();
	Eigen::Vector3d get_target_jerk();
	Eigen::Vector3d get_target_angvel();
	double		    get_target_yaw();
	int 			get_type();

	//Function to predict the state of the UGV after a given time based on the current acceleration 
	//of the UGV
	bool future_state_predict();

	//Function subscribing to the current acceleration of the UGV
	void ugv_accel_sub(const geometry_msgs::AccelWithCovarianceStamped&);

	//Function subscribing to the current position and attitude of the quadrotor
	void mav_pose_sub(const geometry_msgs::PoseStamped&);  

	//Function subscribing to the current velocity of the quadrotor
	void mavtwistCallback(const geometry_msgs::TwistStamped&); 

	//Function subscribing to the current state of the UGV
	void ugv_state_sub(const nav_msgs::Odometry&);

	//Function to calculate the distance along the XY plane for two given vectors
	double planar_distance_calculator(Eigen::Vector3d&, Eigen::Vector3d&);
};


#endif