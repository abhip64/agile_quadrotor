/*
Main cpp file associated with trajectory generation. Different maneuvers are specified as seperate files in
this package. trajectoryPublisher file deals with the task of executing the specific maneuver by making 
use of the trajectory generation package 'mav_trajectory_generation'. 
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "maneuvers/trajectoryPublisher.h"
#include <iostream>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh) :
  nh_(nh),
  motion_selector_(0),publish_data(0){

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PUBLISHER DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Desired values of position, velocity, acceleration, angular velocity and yaw along the trajectory that needs
  //to be acheived are published 
  desired_trajPub_  = nh_.advertise<control_msg::TargetTrajectory>("reference/setpoint", 100);

  //The controller type specifies the mode in which the controller has to run depending on the specific 
  //maneuver that has to be achieved
  controllertype_   = nh_.advertise<std_msgs::Int8>("/trajectory/controller_type",100);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SUBSCRIBER DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Subscribe to the current position and attitude of the quadrotor
  mavposeSub_       = nh_.subscribe("/mavros/local_position/pose", 50, &trajectoryPublisher::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  
  //vehicle_pos_Sub_  = nh_.subscribe("/odometry/filtered", 50, &trajectoryPublisher::update_ugv_state, this,ros::TransportHints().tcpNoDelay());
  
  //Subscribe to the current velocity and angular velocity of the quadrotor
  mavtwistSub_      = nh_.subscribe("/mavros/local_position/velocity_local", 50, &trajectoryPublisher::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
  
  //accelSub_         = nh_.subscribe("/accel/filtered", 30, &trajectoryPublisher::accel_sub, this,ros::TransportHints().tcpNoDelay());

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//TIMER DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Publishing of the desired trajectory setpoints at a specific frequency. The defualt frequency is 100Hz
  refloop_timer_    = nh_.createTimer(ros::Duration(0.01), &trajectoryPublisher::refCallback, this);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SERVICES DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Service to start trajectory publishing. This service can be accessed by other packages to trigger
  //trajectory generation
  trajtriggerServ_  = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//MANEUVER SELECTION FOR EXECUTION

  //Maneuver type obtained from the parameter server. Default maneuver is circle
  nh_.param<int>("/maneuver_select", maneuver_type_select, 0);

  switch (maneuver_type_select)
  {
    case 0:
    {
      maneuver_select = std::make_shared<circle_traverse>(nh_);
      break;
    }

    default:
    { 
      ROS_ERROR("INVALID MANEUVER SELECTED FOR EXECUTION");
      break;
    }
  }

  //Time for which trajectory should be executed
  T = maneuver_select->maneuver_init();

}


void trajectoryPublisher::mavtwistCallback(const geometry_msgs::TwistStamped& msg){  
  mavVel_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
}


void trajectoryPublisher::mavposeCallback(const geometry_msgs::PoseStamped& msg){

  mavPos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;

}

void trajectoryPublisher::updateReference() {

  trigger_time_ = (ros::Time::now() - start_time_).toSec();

  //The trajectory is updated until the total time for trajectory execution is completed
  if(trigger_time_<T)
  {
    maneuver_select->trajectory_generator(trigger_time_);

    p_targ   = maneuver_select->get_target_pos();
    v_targ   = maneuver_select->get_target_vel();
    w_targ   = maneuver_select->get_target_angvel();
    a_targ   = maneuver_select->get_target_acc();
    yaw_targ = maneuver_select->get_target_yaw();
  }
  else
    motion_selector_ = 1;
}

void trajectoryPublisher::pubflatrefState(){
  
  control_msg::TargetTrajectory msg;

  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "map";
  
  msg.position.x      = p_targ(0);
  msg.position.y      = p_targ(1);
  msg.position.z      = p_targ(2);
  
  msg.velocity.x      = v_targ(0);
  msg.velocity.y      = v_targ(1);
  msg.velocity.z      = v_targ(2);
  
  msg.acceleration.x  = a_targ(0);
  msg.acceleration.y  = a_targ(1);
  msg.acceleration.z  = a_targ(2);
  
  msg.ang_vel.x       = w_targ(0);
  msg.ang_vel.y       = w_targ(1);
  msg.ang_vel.z       = w_targ(2);

  msg.yaw.data        = yaw_targ;

  typepublish();

  desired_trajPub_.publish(msg);
}


void trajectoryPublisher::typepublish()
{
  std_msgs::Int8 cont_type;

  cont_type.data = motion_selector_;

  controllertype_.publish(cont_type);
}

void trajectoryPublisher::refCallback(const ros::TimerEvent& event){

  if(publish_data)
  {
    updateReference();

    if(motion_selector_ == 0)
      pubflatrefState();
    else if(motion_selector_ == 2)
      typepublish();
  }
}

bool trajectoryPublisher::triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  
  publish_data = req.data;
  
  //Trajectory publishing start time
  start_time_ = ros::Time::now(); 

  res.success = publish_data;

  if(publish_data)
    res.message = "Trajectory triggered";
  else
    res.message = "Trajectory stopped publishing";
}


