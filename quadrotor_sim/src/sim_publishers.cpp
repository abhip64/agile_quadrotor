/*
All publisher definitions for the quadrotor simulation are given here
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "quadrotor_sim/quad_sim.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PUBLISHERS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Publish the control inputs to PX4. This includes the desired angular velocity and desired thrust magnitude
void quadrotor_sim::pubRateCommands(const Eigen::Vector4d &cmd){
  mavros_msgs::AttitudeTarget msg;
  
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x     = cmd(0);
  msg.body_rate.y     = cmd(1);
  msg.body_rate.z     = cmd(2);
  msg.type_mask       = 128; //Ignore orientation messages
  msg.thrust          = cmd(3);
  
  control_inputsPub_.publish(msg);
}


void quadrotor_sim::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component    = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state        = (int)companion_state_;

  systemstatusPub_.publish(msg);
}




