/*
EXECUTE THE CIRCULAR MANEUVER
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "maneuvers/flip_maneuver.h"
#include <iostream>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
flip_traverse::flip_traverse(const ros::NodeHandle& nh): nh_(nh)
{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SUBSCRIBER DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Subscribe to the current position and attitude of the quadrotor
  mavposeSub_       = nh_.subscribe("/mavros/local_position/pose", 50, &flip_traverse::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
    
  //Subscribe to the current velocity and angular velocity of the quadrotor
  mavtwistSub_      = nh_.subscribe("/mavros/local_position/velocity_local", 50, &flip_traverse::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  flip_init_vel << 0.0, 0.0, 0.0;
  flip_init_pos << 0.0, 0.0, 0.0;

  mavPos_       << 0.0, 0.0, 10.0;
  mavVel_       << 0.0, 0.0, 0.0;

  nh_.param<double>("/trajectory_generator/flip_init_vel_x", flip_init_vel[0], 3.0);

  nh_.param<double>("/trajectory_generator/flip_init_vel_y", flip_init_vel[1], 0.0);

  nh_.param<double>("/trajectory_generator/Tc", Tc, 1.1);

  nh_.param<double>("/trajectory_generator/flip_init_pos_x", flip_init_pos[0], 5.0);

  nh_.param<double>("/trajectory_generator/flip_init_pos_y", flip_init_pos[1], 0.0);

  nh_.param<double>("/trajectory_generator/flip_height", flip_init_pos[2], 10.0);


  curr_vel = flip_init_vel.norm();

  r        = (curr_vel*curr_vel)/(Tc*g - g);

  pitch_angle = 0.0;

  energy   = (curr_vel*curr_vel)/2.0;

  g        = 9.8;

}

flip_traverse::~flip_traverse() {
  //Destructor
}


double flip_traverse::maneuver_init()
{
  //std::cout<<mavPos_<<" "<<mavVel_<<"\n";

  eth_set_pos(mavPos_,flip_init_pos);

  eth_set_vel(mavVel_,flip_init_vel);

  //T1 = eth_trajectory_init();

  T1 = 0.0;
  
  T2 = 1.0;

  return  T1 + T2;
}

void flip_traverse::trajectory_generator(double time)
{
  if(time<T1)
  {
    target_position     = eth_trajectory_pos(time);
    target_velocity     = eth_trajectory_vel(time);
    target_acceleration = eth_trajectory_acc(time);
    type                = 1;
  }
  else
  {
    pitch_angle         += (curr_vel*0.01)/r;

    curr_vel             = pow(2*std::max(0.1,(energy - g*(mavPos_[2] - flip_init_pos[2]))),0.5);

    target_velocity     << curr_vel*cos(pitch_angle), 0, curr_vel*sin(pitch_angle);

    target_acceleration << Tc*g*sin(pitch_angle), 0, (Tc*g*cos(pitch_angle)) - g;

    //target_position     += Eigen::Vector3d(0.01*target_velocity[0],0,0.01*target_velocity[2]);

    r                    = (curr_vel*curr_vel)/(Tc*g - g*cos(pitch_angle));

    type                 = 4;
  }

  target_angvel         << 0.0, 0.0, 0.0;

  target_yaw             = 0.0;

}

Eigen::Vector3d flip_traverse::calculate_trajectory_angvel()
{

}

Eigen::Vector3d flip_traverse::get_target_pos()
{
  return target_position;
}

Eigen::Vector3d flip_traverse::get_target_vel()
{
  return target_velocity;
}

Eigen::Vector3d flip_traverse::get_target_acc()
{
  return target_acceleration;
}

Eigen::Vector3d flip_traverse::get_target_angvel()
{
  return target_angvel;
}

Eigen::Vector3d flip_traverse::get_target_jerk()
{
  return target_jerk;
}

double flip_traverse::get_target_yaw()
{
  return target_yaw;
}

int flip_traverse::get_type()
{
  return type;
}


void flip_traverse::mavtwistCallback(const geometry_msgs::TwistStamped& msg){  
  mavVel_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
}


void flip_traverse::mavposeCallback(const geometry_msgs::PoseStamped& msg){

  mavPos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

}