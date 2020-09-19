//  May/2018, ETHZ, Jaeyoung Lim, jalim@ethz.ch

#ifndef TRAJECTORYPUBLISHER_H
#define TRAJECTORYPUBLISHER_H

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include "control_msg/TargetTrajectory.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"

#include "maneuvers/trajectory_class.h"
#include "maneuvers/circle_maneuver.h"
#include "maneuvers/flip_maneuver.h"
#include "maneuvers/slit_maneuver.h"
#include "maneuvers/ugv_follow_maneuver.h"

using namespace std;
using namespace Eigen;

class trajectoryPublisher
{
private:

  ros::NodeHandle nh_;

  ros::Publisher trajectoryPub_;

  ros::Publisher desired_trajPub_;

  ros::Publisher controllertype_;

  //ros::Subscriber mavposeSub_, mavtwistSub_;

  ros::ServiceServer trajtriggerServ_;
  ros::Timer trajloop_timer_;
  ros::Timer refloop_timer_;
  ros::Time start_time_;

  Eigen::Vector3d p_targ, v_targ, a_targ, w_targ;

  double yaw_targ;

  Eigen::Vector3d mavPos_, mavVel_;

  Eigen::Vector4d mavAtt_;

  double trigger_time_;
  double T;

  int motion_selector_;

  int maneuver_type_select;

  bool publish_data;

  std::shared_ptr<trajectory_class> maneuver_select;

public:
  trajectoryPublisher(const ros::NodeHandle& nh);
  
  void updateReference();
  void pubflatrefState();
  void typepublish();
  void loopCallback(const ros::TimerEvent& event);
  void refCallback(const ros::TimerEvent& event);
  bool triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void mavposeCallback(const geometry_msgs::PoseStamped& );
  void mavtwistCallback(const geometry_msgs::TwistStamped& );  

  };


#endif
