//  May/2018, ETHZ, Jaeyoung Lim, jalim@ethz.ch

#ifndef TRAJECTORYPUBLISHER_H
#define TRAJECTORYPUBLISHER_H

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <Eigen/Dense>

#include "quad_perch/math_operations.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include "control_msgs/FlatTarget.h"
#include "control_msgs/RollPitchTarget.h"
#include "quad_perch/eth_trajectory.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"

using namespace std;
using namespace Eigen;

class trajectoryPublisher
{
private:
  ros::NodeHandle nh_;

  ros::Publisher trajectoryPub_;

  ros::Publisher flatreferencePub_;

  ros::Publisher controllertype_, anglePub_ ;

  ros::Subscriber mavposeSub_, vehicle_pos_Sub_, mavtwistSub_, accelSub_;

  ros::ServiceServer trajtriggerServ_;
  ros::Timer trajloop_timer_;
  ros::Timer refloop_timer_;
  ros::Time start_time_, curr_time_, time_to_land;

  Eigen::Vector3d p_targ, v_targ, a_targ, w_targ, mavPos_, mavVel_;

  Eigen::Vector4d mavAtt_, vehAtt_;

  Eigen::Vector3d vehPos_, vehVel_, vehAcc_, vehPos_future, vehVel_future;

  double trigger_time_;

  int motion_selector_;

  bool publish_data;
  
  std::vector<Eigen::Vector3d> position_;
  std::vector<Eigen::Vector3d> velocity_;

  std::vector<float> T;

  int image_id;

//This is due to the discrepancy in velocity and position direction which is the output of kalman filter package
  float yaw;

  float circle_theta;

public:
  trajectoryPublisher(const ros::NodeHandle& nh);
  void updateReference();

  void pubflatrefState();
  void typepublish();
  void anglerefState();
  void loopCallback(const ros::TimerEvent& event);
  void refCallback(const ros::TimerEvent& event);
  bool triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void mavposeCallback(const geometry_msgs::PoseStamped& );
  void update_ugv_state(const nav_msgs::Odometry&);
  //void image_pos_sub(const apriltag_ros::AprilTagDetectionArray&);
  void mavtwistCallback(const geometry_msgs::TwistStamped& );  
  void accel_sub(const geometry_msgs::AccelWithCovarianceStamped& );
  void future_state_predict(double,double);
  void circle_future_state_predict(double,double);
  void init_landing();

  };


#endif
