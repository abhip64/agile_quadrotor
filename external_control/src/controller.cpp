

#include "quad_perch/controller.h"

#define rad_to_deg 180.0/M_PI

using namespace Eigen;
using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
controller::controller(const ros::NodeHandle& nh):
  nh_(nh),
  fail_detec_(false),
  ctrl_enable_(true),
  landing_commanded_(false),
  node_state(WAITING_FOR_HOME_POSE),exec_cont(0){

  //start_time = ros::Time::now();

//Subscribers of current Quadrotor states
  //Current Mode of Operation of the Quadrotor
  mavstateSub_ = nh_.subscribe("/mavros/state", 10, &controller::mavstateCallback, this,ros::TransportHints().tcpNoDelay());
  //Current Position and Orientation of the Quadrotor
  mavposeSub_  = nh_.subscribe("/mavros/local_position/pose", 50, &controller::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  //Current Linear and Angular Velocity  of the Quadorotor
  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity_local", 50, &controller::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
  imuSub_ = nh_.subscribe("/mavros/imu/data", 50, &controller::imuCallback, this,ros::TransportHints().tcpNoDelay());


//Subscribers of reference quadrotor states
  //Reference Position, Velocity, Acceleration, Angular Velocity
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 100, &controller::flattargetCallback, this, ros::TransportHints().tcpNoDelay());
  //Reference Yaw
  yawreferenceSub_  = nh_.subscribe("reference/yaw", 10, &controller::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  //Roll and Pitch Setpoint
  angleSub_         = nh_.subscribe("/trajectory/angle", 100, &controller::rptargetCallback, this, ros::TransportHints().tcpNoDelay());
  //Controller Type Selection
  controllerType_   = nh_.subscribe("/trajectory/controller_type", 100, &controller::typeCallback, this, ros::TransportHints().tcpNoDelay());

//Services
  //To start the trajectory generation from the trajectory generation node
  start_traj_client_ = nh_.serviceClient<std_srvs::SetBool>("start");
  //Start the external controller
  ctrltriggerServ_   = nh_.advertiseService("trigger_controller", &controller::ctrltriggerCallback, this);
  //Serive to Arm and Disarm the Quadrotor
  arming_client_     = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  //To set the mode of Quadrotor
  set_mode_client_   = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  //To call the land service of the Quadrotor
  land_service_      = nh_.advertiseService("land", &controller::landCallback, this);

  start_ilc_pub_     = nh_.serviceClient<std_srvs::SetBool>("start_ilc");

//Timers for constant loop rate
  //Calls the controller loop
  cmdloop_timer_    = nh_.createTimer(ros::Duration(0.01), &controller::cmdloopCallback, this); 
  //Checks the system status loop if it is OFFBOARD
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &controller::statusloopCallback, this); 

//Publishers
  //Publish the angular velocities and thrust value which is used by PX4 to control the Quadrotor
  angularVelPub_    = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 100);
  //Publish the reference trajectory that is to be used by RViz 
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 10);
  //Publish Reference Pose for RViz visualization
  posehistoryPub_   = nh_.advertise<nav_msgs::Path>("/geometric_controller/path", 10);
  //Status of the companion process
  systemstatusPub_  = nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 10);
  //Publishing setpoint position data for PX4 tracking
  target_pose_pub_  = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  euler_ang_pub_    = nh_.advertise<control_msgs::RollPitchTarget>("/current_angle", 50);

//Variables addded to the ROS parameter server
  nh_.param<string>("mavname", mav_name_, "quad");
  nh_.param<bool>("enable_sim", sim_enable_, true);

  nh_.param<double>("yaw_heading", mavYaw_, 0.0);

  nh_.param<int>("posehistory_window", posehistory_window_, 200);

  double take_off_height = 5.0;
  //nh.getParam("/take_off_height", take_off_height);

//Variable Initialisation
  //Target Position to be achieved after takeoff
  targetPos_ << 0.0, 0.0, take_off_height; 
  //Target Velocity to be achieved after takeoff
  targetVel_ << 0.0, 0.0, 0.0;
  //Initialisation for current Quadrotor position and velcoity
  mavPos_    << 0.0, 0.0, 0.0;
  mavVel_    << 0.0, 0.0, 0.0;

  nh_.getParam("/controllers/exec_cont", exec_cont);

  //if(exec_cont == 0)
  //control_tech = std::make_shared<eth_controller>(nh_);
  //else if(exec_cont == 1)
  control_tech = std::make_shared<upenn_controller>(nh_);
  //else if(exec_cont == 2)
  //control_tech = std::make_shared<euler_controller>(nh_);
  //else if(exec_cont == 3)
  //control_tech = std::make_shared<lqr_controller>(nh_);

}

controller::~controller() {
  //Destructor
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void controller::flattargetCallback(const control_msgs::FlatTarget& msg) {

  reference_request_last_ = reference_request_now_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_  = toEigen(msg.position);
  targetVel_  = toEigen(msg.velocity);
  targetAcc_  = toEigen(msg.acceleration);
  targetW_    = toEigen(msg.ang_vel);

}

void controller::rptargetCallback(const control_msgs::RollPitchTarget& msg)
{
  targetRoll_  = double(msg.roll);
  targetPitch_ = double(msg.pitch);
}

void controller::typeCallback(const std_msgs::Int8& msg)
{
  controller_type = int(msg.data);  
}

void controller::yawtargetCallback(const std_msgs::Float32& msg) {
  mavYaw_ = double(msg.data);
}

void controller::mavposeCallback(const geometry_msgs::PoseStamped& msg){
  if(!received_home_pose){
      received_home_pose = true;
      home_pose_ = msg.pose;
      ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  current_time = msg.header.stamp;
  
  //std::cout<<current_time.toSec()<<"\n";

  mavPos_ = toEigen(msg.pose.position);
  
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;

  euler_angle = quat2euler(mavAtt_);

  euler_ang.roll = euler_angle[0]*rad_to_deg;
  euler_ang.pitch = euler_angle[1]*rad_to_deg;

  euler_ang_pub_.publish(euler_ang);
  
}

void controller::mavtwistCallback(const geometry_msgs::TwistStamped& msg){  
  mavVel_  = toEigen(msg.twist.linear);
}

void controller::imuCallback(const sensor_msgs::Imu& msg)
{
  mavRate_ = toEigen(msg.angular_velocity);
}

bool controller::landCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
  node_state = LANDING;
}

void controller::cmdloopCallback(const ros::TimerEvent& event){
  switch (node_state) {
  case WAITING_FOR_HOME_POSE:
      waitForPredicate(&received_home_pose, "Waiting for home pose...");
      ROS_INFO("Got pose! Drone Ready to be armed.");
      node_state = TAKE_OFF;
      break;

  case MISSION_EXECUTION:
{

    Eigen::Vector3d pos_error;
    Eigen::Vector3d vel_error;
    Eigen::Vector3d w_error;
    
    if(controller_type == 0)
    {
      pos_error = mavPos_ - targetPos_;
      vel_error = mavVel_ - targetVel_;
      //w_error   = mavRate_ - targetW_;

      cmdBodyRate_ = control_tech->pos_control(pos_error, mavVel_, targetVel_, mavRate_, targetW_, targetAcc_, mavYaw_, mavAtt_, q_des);
    }
    else if(controller_type == 1)
    { double z_err = mavPos_[2] - targetPos_[2];
      cmdBodyRate_ = control_tech->ang_control(targetRoll_, z_err, mavRate_, mavAtt_, q_des);
    }
    else if(controller_type == 2)
    {
      vel_error    = mavVel_;
      cmdBodyRate_ = control_tech->vel_control(vel_error, mavRate_, mavAtt_, q_des);
    }
    else if(controller_type == 3)
    {
      pos_error    = mavPos_ - targetPos_;
      vel_error    = mavVel_ - targetVel_;
      cmdBodyRate_ = control_tech->flip_control(pos_error, vel_error, targetAcc_, mavAtt_, q_des);
    }
    else if(controller_type == 4)
    {
      //std::cout<<"HOVERING CONTROLLER **"<<"\n"<<"*****"<<"\n";
      pos_error    = mavPos_ - targetPos_;
      //std::cout<<pos_error<<"\n"<<"******"<<"\n";
      //pos_error    = targetPos_;
      vel_error    = mavVel_ - targetVel_;
      cmdBodyRate_ = control_tech->hover_control(pos_error, vel_error, mavYaw_, mavAtt_, q_des);
    }
    pubReferencePose(targetPos_, q_des);
    pubRateCommands(cmdBodyRate_);
    appendPoseHistory();
    pubPoseHistory();
 
    break;
}
  case TAKE_OFF: {

    //current_time = (ros::Time::now() - start_time).toSec();

    geometry_msgs::PoseStamped takeoffmsg;
    takeoffmsg.header.stamp = ros::Time::now();
    
    takeoffmsg.pose.position.x = targetPos_[0];
    takeoffmsg.pose.position.y = targetPos_[1];
    takeoffmsg.pose.position.z = targetPos_[2];

    target_pose_pub_.publish(takeoffmsg);

 if((abs(mavPos_[2]-targetPos_[2])<0.01))
    //if((current_time.toSec()>20.0))
    {
      node_state = MISSION_EXECUTION;
      
      std_srvs::SetBool set_traj;
      set_traj.request.data = true;
      set_traj.response.success = true;

      start_traj_client_.call(set_traj);
      //start_ilc_pub_.call(set_traj);
    }
    ros::spinOnce();
    break;
  }

  case LANDING: {
    geometry_msgs::PoseStamped landingmsg;
    landingmsg.header.stamp = ros::Time::now();
    landingmsg.pose = home_pose_;
    landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;
    target_pose_pub_.publish(landingmsg);
    node_state = LANDED;
    ros::spinOnce();
    break;
  }
  case LANDED:
    ROS_INFO("Landed. Please set to position control and disarm.");
    cmdloop_timer_.stop();
    break;
  }
}

void controller::mavstateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}

void controller::statusloopCallback(const ros::TimerEvent& event){
  if(sim_enable_){
    // Enable OFFBoard mode and arm automatically
    // This is only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
      if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent){
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if( !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
        if( arming_client_.call(arm_cmd_) && arm_cmd_.response.success){
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void controller::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude){
  geometry_msgs::PoseStamped msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}

void controller::pubRateCommands(const Eigen::Vector4d &cmd){
  mavros_msgs::AttitudeTarget msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id= "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128; //Ignore orientation messages
  msg.thrust = cmd(3);
  
  angularVelPub_.publish(msg);
}

void controller::pubPoseHistory(){
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posehistory_vector_;
  
  posehistoryPub_.publish(msg);
}

void controller::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void controller::appendPoseHistory(){
  posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
  if(posehistory_vector_.size() > posehistory_window_){
    posehistory_vector_.pop_back();
  }
}

geometry_msgs::PoseStamped controller::vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation){
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}


bool controller::ctrltriggerCallback(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &res){
  bool mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
}



