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
  motion_selector_(1),publish_data(0){

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PUBLISHER DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Desired values of position, velocity, acceleration, angular velocity and yaw along the trajectory that needs
  //to be acheived are published 
  desired_trajPub_  = nh_.advertise<control_msg::TargetTrajectory>("reference/setpoint", 100);

  //The controller type specifies the mode in which the controller has to run depending on the specific 
  //maneuver that has to be achieved
  //controllertype_   = nh_.advertise<std_msgs::Int8>("/trajectory/controller_type",100);
 
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
  nh_.param<int>("/quadrotor_sim/maneuver_select", maneuver_type_select, 0);

  //Initialising the corresponding maneuver class based on user selection
  switch (maneuver_type_select)
  {
    case 0:
    {
      //Class for circular trajectory
      maneuver_select = std::make_shared<circle_traverse>(nh_);
      break;
    }
    case 1:
    {
      //Class for slit traversal maneuver
      maneuver_select = std::make_shared<slit_traverse>(nh_);
      break;
    }
    case 2:
    {
      //Class for flip maneuver. Currently not active
      maneuver_select = std::make_shared<flip_traverse>(nh_);
      break;
    }
    case 3:
    {
      //Class for UGV tracking, following and landing maneuver
      maneuver_select = std::make_shared<ugv_follow>(nh_);
      break;
    }

    default:
    { 
      ROS_ERROR("INVALID MANEUVER SELECTED FOR EXECUTION");
      break;
    }
  }

  //Carry out all message subscriptions before initiating the maneuver
  ros::spinOnce();

  //Time for which trajectory should be executed
  T = maneuver_select->maneuver_init(0.0);

}

//Function to obtain the trajectory targets from the specific maneuver class
void trajectoryPublisher::updateReference() {

  //Time for start of trajectory publishing
  trigger_time_ = (ros::Time::now() - start_time_).toSec();

  //The trajectory is updated until the total time for trajectory execution is completed
  if(trigger_time_<T)
  {
    //Determining the trajectory targets for the given trigger time from the corresponding maneuver class
    maneuver_select->trajectory_generator(trigger_time_);

    p_targ           = maneuver_select->get_target_pos();
    v_targ           = maneuver_select->get_target_vel();
    w_targ           = maneuver_select->get_target_angvel();
    a_targ           = maneuver_select->get_target_acc();
    yaw_targ         = maneuver_select->get_target_yaw();
    motion_selector_ = maneuver_select->get_type();
  }
  //If the time for which the maneuver needs to be executed is completed then the controller is asked to 
  //ignore the trajectory targets and initiate landing at the home position
  else
    motion_selector_ = 0;
}

//Function for publishing the trajectory targets
void trajectoryPublisher::pubflatrefState(){
  
  control_msg::TargetTrajectory msg;

  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "map";
  msg.type_mask       = motion_selector_;
  
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

  desired_trajPub_.publish(msg);
}


void trajectoryPublisher::refCallback(const ros::TimerEvent& event){

  if(publish_data)
  {
    updateReference();
    
    pubflatrefState();
  }

  //This is used for the quadrotor landing maneuver on the UGV. When the quadrotor lands, the trajectory
  //pubisher is stopped to prevent further commands to be passed on to the controller. CAN BE IMPROVED
  if(motion_selector_ == 5)
    refloop_timer_.stop();

}

//Function for starting and stopping the trajectory publishing node
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


