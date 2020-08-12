#include "quad_perch/trajectoryPublisher.h"

#include <iostream>

using namespace std;
using namespace Eigen;

bool started = 0;

int phase = 0;

bool traj_complete = 1;

double time_to_intersection = 0;

trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh) :
  nh_(nh),
  motion_selector_(0),publish_data(0){

//Publisher
  trajectoryPub_    = nh_.advertise<nav_msgs::Path>("/trajectory_publisher/trajectory", 10);
  flatreferencePub_ = nh_.advertise<control_msgs::FlatTarget>("reference/flatsetpoint", 100);
  controllertype_   = nh_.advertise<std_msgs::Int8>("/trajectory/controller_type",100);
  anglePub_         = nh_.advertise<control_msgs::RollPitchTarget>("/trajectory/angle",100);

//Subscriber
  mavposeSub_      = nh_.subscribe("/mavros/local_position/pose", 50, &trajectoryPublisher::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  vehicle_pos_Sub_ = nh_.subscribe("/odometry/filtered", 50, &trajectoryPublisher::update_ugv_state, this,ros::TransportHints().tcpNoDelay());
  mavtwistSub_     = nh_.subscribe("/mavros/local_position/velocity_local", 50, &trajectoryPublisher::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
  accelSub_        = nh_.subscribe("/accel/filtered", 30, &trajectoryPublisher::accel_sub, this,ros::TransportHints().tcpNoDelay());

//Timers
  trajloop_timer_ = nh_.createTimer(ros::Duration(1), &trajectoryPublisher::loopCallback, this);
  refloop_timer_  = nh_.createTimer(ros::Duration(0.01), &trajectoryPublisher::refCallback, this);

//Services
  trajtriggerServ_ = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

    //Variable Definitions
  position_.resize(2);

  velocity_.resize(2);

  T.resize(1);

  image_id = 0;

  vehPos_ << 5, 0, 2.0;

  //position_.at(0) <<0, 0, 5.0;

  //position_.at(1) << 5.0, 0.0 ,5.0;

  //std::cout<<position_.at(1)<<"\n"<<" ****** "<<"\n";

  //position_.at(1) << 1.0, 0.0, 10.0;
  //eth_set_pos(position_.at(0),position_.at(1));

  //eth_trajectory_init();

}



void trajectoryPublisher::mavtwistCallback(const geometry_msgs::TwistStamped& msg){  
  mavVel_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
}

void trajectoryPublisher::accel_sub(const geometry_msgs::AccelWithCovarianceStamped& msg){  
  vehAcc_ << msg.accel.accel.linear.x, msg.accel.accel.linear.y, msg.accel.accel.linear.z;
}

void trajectoryPublisher::update_ugv_state(const nav_msgs::Odometry& msg)
{
  vehPos_ << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
  vehVel_<< msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;

  vehAtt_ << msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z;

  yaw = atan2(2 * (vehAtt_(0) * vehAtt_(3) + vehAtt_(1) * vehAtt_(2)), 1 - 2 * (vehAtt_(2) * vehAtt_(2) + vehAtt_(3) * vehAtt_(3)));

  Eigen::Matrix3d R;

  R <<  cos(yaw), -sin(yaw), 0,
        sin(yaw),  cos(yaw), 0,
        0       ,  0       , 1;

  vehVel_ = R*vehVel_;

  circle_theta = atan2(vehPos_(1),vehPos_(0));

  if(circle_theta<0.0)
    circle_theta += 6.28;

  //std::cout<<circle_theta<<"\n";
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

  if((trigger_time_ < T.at(0))&&(phase==2))
  {
  p_targ = eth_trajectory_pos(trigger_time_);
  v_targ = eth_trajectory_vel(trigger_time_);
  w_targ = eth_trajectory_angvel(trigger_time_);
  a_targ = eth_trajectory_acc(trigger_time_);

  vehVel_future = vehVel_ + vehAcc_*(T.at(0) - trigger_time_);

  time_to_intersection = T.at(0) - trigger_time_;

  //Eigen::Vector3d veh_pos_new;
  //veh_pos_new(0) = 3*cos(circle_theta + time_to_intersection*1.4);
  //veh_pos_new(1) = 3*sin(circle_theta + time_to_intersection*1.4);

  Eigen::Vector2d planar_error;

  Eigen::Vector3d veh_pos_new = (vehPos_ + vehVel_future*(T.at(0) - trigger_time_));

  planar_error << veh_pos_new(0) - vehPos_future(0), veh_pos_new(1) - vehPos_future(1);

  double dist = planar_error.norm();

  if(dist>0.05)
  {
    vehPos_future = veh_pos_new;
    traj_complete = 1;
    //std::cout<<"1"<<"\n";
  }
   
  motion_selector_ = 0;
  }
  else if(phase == 2)
  {
    traj_complete = 1;

    //std::cout<<"2"<<"\n";
  }
  //else if(trigger_time_ < (T.at(0) + T.at(1)))
  //{
  //motion_selector_ = 1;
  //}
  else if(phase==3)
  {

    //if(image_id == 7)
    {
      //p_targ << image_pos(1), 0.0, 0.0;
      //p_targ += mavPos_;
      //v_targ << 0.0, 0.0, 0.0;
      
      p_targ = vehPos_;
      p_targ(2) = 0.5;
      //p_targ << -image_pos(1), 0.0, image_pos(2) - 2.0;
      v_targ = vehVel_;
      a_targ << 0.0, 0.0, 0.0;
      w_targ << 0.0, 0.0, 0.0;

      //std::cout<<vehPos_<<"\n"<<"#########"<<"\n";
      //cout<<"Camera"<<"\n"<<"******"<<"\n";
      motion_selector_ = 0;

      //std::cout<<"3"<<"\n";

      if((ros::Time::now() - time_to_land).toSec()>3.0)
      { phase = 4;
        //init_landing();
        start_time_ = ros::Time::now();
      }
    }  
  }
  else if((trigger_time_ < 5.0)&&phase == 4)
  {

  Eigen::Vector2d planar_error;

  planar_error << mavPos_(0) - vehPos_(0), mavPos_(1) - vehPos_(1);

  double dist = planar_error.norm();

    future_state_predict(dist, 0.2);
   // circle_future_state_predict(0.0, 0.1);

     p_targ = vehPos_;
     p_targ(2) = 0.5 - 0.1*trigger_time_;
     v_targ = vehVel_;
     
    motion_selector_ = 4;
  }
  else if(phase == 4)
  {

    p_targ = vehPos_;
    v_targ = vehVel_;
    a_targ << 0.0, 0.0, -10.0;
    motion_selector_ = 0;

    traj_complete = 0;
  }
}

void trajectoryPublisher::pubflatrefState(){
  control_msgs::FlatTarget msg;

  msg.header.stamp = curr_time_;
  msg.header.frame_id = "map";
  msg.position.x = p_targ(0);
  msg.position.y = p_targ(1);
  msg.position.z = p_targ(2);
  msg.velocity.x = v_targ(0);
  msg.velocity.y = v_targ(1);
  msg.velocity.z = v_targ(2);
  msg.acceleration.x = a_targ(0);
  msg.acceleration.y = a_targ(1);
  msg.acceleration.z = a_targ(2);
  msg.ang_vel.x = w_targ(0);
  msg.ang_vel.y = w_targ(1);
  msg.ang_vel.z = w_targ(2);

  typepublish();
  flatreferencePub_.publish(msg);
}



void trajectoryPublisher::loopCallback(const ros::TimerEvent& event){
  //Slow Loop publishing trajectory information
  if(publish_data)
  {

  Eigen::Vector2d planar_error;

  planar_error << mavPos_(0) - vehPos_(0), mavPos_(1) - vehPos_(1);

  double dist = planar_error.norm();

  future_state_predict(dist, 1.0);
  //future_state_predict(0.0, 0.0);

  //circle_future_state_predict(dist, 1.0);

  if(dist > 1.0 && traj_complete == 1)
  {
  
  position_.at(0) = mavPos_;

  vehPos_future(2) = 2.0;

  position_.at(1) = vehPos_future;

  velocity_.at(0) = mavVel_;

  velocity_.at(1) = vehVel_future;


  eth_set_pos(position_.at(0),position_.at(1));

  eth_set_vel(velocity_.at(0),velocity_.at(1));

  //set_mid_pos(vehPos_,vehVel_*5.0);

  T.at(0) = eth_trajectory_init(time_to_intersection);

  start_time_ = ros::Time::now();
  
  started = 1;

  phase = 2;

  traj_complete = 0;
  }
  else if(dist < 1.0 && phase==2)
  {  
    if(phase == 2)
      time_to_land = ros::Time::now();
    phase = 3;

  }
}
}

void trajectoryPublisher::typepublish()
{
  std_msgs::Int8 cont_type;

  cont_type.data = motion_selector_;

  controllertype_.publish(cont_type);
}

void trajectoryPublisher::refCallback(const ros::TimerEvent& event){
  //Fast Loop publishing reference states
  //curr_time_ = ros::Time::now();

  if(publish_data&&started)
  {
    updateReference();

    if(motion_selector_ == 0 ||motion_selector_ == 4)
      pubflatrefState();
    else if(motion_selector_ == 2)
      typepublish();
  }
}

bool trajectoryPublisher::triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  
  publish_data = req.data;
  
  start_time_ = ros::Time::now(); 
  res.success = publish_data;

  if(publish_data)
    res.message = "Trajectory triggered";
  else
    res.message = "Trajectory stopped publishing";
}

void trajectoryPublisher::future_state_predict(double dist, double time)
{

  time_to_intersection = dist/5.0 + time; 

  vehVel_future = vehVel_ + vehAcc_*time_to_intersection;
  vehPos_future = vehPos_ + vehVel_future*time_to_intersection;
 
}

void trajectoryPublisher::circle_future_state_predict(double dist, double time)
{

  float w = 1.4;

  float r = 3.0;

  time_to_intersection = dist/5.0 + time;

  vehPos_future(0) = r*cos(circle_theta + time_to_intersection*w);
  vehPos_future(1) = r*sin(circle_theta + time_to_intersection*w);

  vehVel_future(0) = -r*w*sin(circle_theta + time_to_intersection*w);
  vehVel_future(1) =  r*w*cos(circle_theta + time_to_intersection*w);
 
}

void trajectoryPublisher::init_landing()
{
  future_state_predict(0.0,1.0);

  position_.at(0) = mavPos_;

  vehPos_future(2) = 0.0;

  position_.at(1) = vehPos_future;


  velocity_.at(0) = mavVel_;

  vehVel_future(2) = 0.0;

  velocity_.at(1) = vehVel_future;

  Eigen::Vector3d final_acc(0.0, 0.0, 0.0);

  //std::cout<<velocity_.at(1)<<"\n";
  //velocity_.at(1) *= 2.0;

  //std::cout<<mavPos_(0)<<" "<<vehPos_future(0)<<"\n"<<mavPos_(1)<<" "<<vehPos_future(1)<<"\n"<<mavPos_(2)<<" "<<vehPos_future(2)<<"\n";

  //if(dist > 1.0 && traj_complete == 1)
  
//  std::cout<<velocity_.at(0)<<"\n";

  eth_set_pos(position_.at(0),position_.at(1));

  eth_set_vel(velocity_.at(0),velocity_.at(1));

  eth_set_acc(final_acc);

  T.at(0) = eth_trajectory_init(time_to_intersection);

  start_time_ = ros::Time::now();
  
}