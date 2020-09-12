/*
All subscriber definitions for the quadrotor simulation are given here
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "quadrotor_sim/quad_sim.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SUBSCRIBERS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Subscriber receiving the trajectory information from the trajectory node. The trajectory information
//can include desired position, velocity, acceleration, yaw and angular velocity.
void quadrotor_sim::trajectorytargetCallback(const control_msg::TargetTrajectory& msg) {

  reference_request_last_ = reference_request_now_;
  reference_request_now_  = ros::Time::now();
  reference_request_dt_   = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_      = toEigen(msg.position);
  targetVel_      = toEigen(msg.velocity);
  targetAcc_      = toEigen(msg.acceleration);
  targetW_        = toEigen(msg.ang_vel);
  targetYaw_      = msg.yaw.data;
  controller_type = msg.type_mask;

}

//This parameters selects the type of controller that needs to be selected for executing a specific maneuver.
//Like position controller, velocity controller, angle controller etc. This data is obtained from the 
//trajectory node.
/*
void quadrotor_sim::typeCallback(const std_msgs::Int8& msg)
{
  controller_type = int(msg.data); 
}
*/

//Subscribing to current quadrotor position and attitude published by PX4
void quadrotor_sim::mavposeCallback(const geometry_msgs::PoseStamped& msg){
  if(!received_home_pose){
      received_home_pose = true;
      home_pose_ = msg.pose;
      ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }

  mavPos_ = toEigen(msg.pose.position);
  
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;
  
}

//Subscribing to current quadrotor velocity published by PX4
void quadrotor_sim::mavtwistCallback(const geometry_msgs::TwistStamped& msg){  
  mavVel_  = toEigen(msg.twist.linear);
}

//Subscribing to current quadrotor angular velocity published by PX4
void quadrotor_sim::imuCallback(const sensor_msgs::Imu& msg)
{
  mavRate_ = toEigen(msg.angular_velocity);
}

//Subscribing to determine the current operating mode of the quadrotor. PX4 has to be set to 
//OFFBOARD mode for it to take inputs from an external controller
void quadrotor_sim::mavstateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}



