
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CompanionProcessStatus.h>

#include <nav_msgs/Path.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <control_msgs/FlatTarget.h>
#include <control_msgs/RollPitchTarget.h>

#include <std_srvs/SetBool.h>

#include "quad_perch/upenn_controller.h"
#include "quad_perch/controller_class.h"
#include "quad_perch/math_operations.h"


using namespace std;
using namespace Eigen;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};


class controller
{
  private:
    ros::NodeHandle nh_;

    ros::Subscriber flatreferenceSub_;
    ros::Subscriber mavstateSub_;
    ros::Subscriber mavposeSub_;
    ros::Subscriber mavtwistSub_;
    ros::Subscriber yawreferenceSub_;
    ros::Subscriber angleSub_;
    ros::Subscriber controllerType_;
    ros::Subscriber imuSub_;

    ros::Publisher angularVelPub_, target_pose_pub_, referencePosePub_;
    ros::Publisher posehistoryPub_;
    ros::Publisher euler_ang_pub_;
    ros::Publisher systemstatusPub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient start_traj_client_, start_ilc_pub_;
    ros::ServiceServer ctrltriggerServ_;
    ros::ServiceServer land_service_;
    ros::Timer cmdloop_timer_, statusloop_timer_;
    ros::Time last_request_, reference_request_now_, reference_request_last_;

    string mav_name_;
    bool fail_detec_, ctrl_enable_;
    int ctrl_mode_;
    bool landing_commanded_;
    bool sim_enable_;

    double reference_request_dt_;
    
    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
    MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;

    Eigen::Vector4d mavAtt_, q_des;

    int controller_type;

    control_msgs::RollPitchTarget euler_ang;
    Eigen::Vector3d euler_angle;

    Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetW_;
    double targetRoll_, targetPitch_;

    Eigen::Vector3d mavPos_, mavVel_, mavRate_;
    double mavYaw_;

    Eigen::Vector4d cmdBodyRate_; 

    int posehistory_window_;

    void startTraj();
    void pubRateCommands(const Eigen::Vector4d &cmd);
    void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);
    void pubPoseHistory();
    void pubSystemStatus();
    void appendPoseHistory();
    void flattargetCallback(const control_msgs::FlatTarget& msg);
    void yawtargetCallback(const std_msgs::Float32& msg);
    void cmdloopCallback(const ros::TimerEvent& event);
    void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);
    void mavposeCallback(const geometry_msgs::PoseStamped& msg);
    void mavtwistCallback(const geometry_msgs::TwistStamped& msg);
    void statusloopCallback(const ros::TimerEvent& event);
    void rptargetCallback(const control_msgs::RollPitchTarget&);
    void typeCallback(const std_msgs::Int8&);
    void imuCallback(const sensor_msgs::Imu&);

    bool ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool landCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

   
    geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);
    Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);

    int exec_cont;

    enum FlightState {
      WAITING_FOR_HOME_POSE, TAKE_OFF, MISSION_EXECUTION, LANDING, LANDED
    } node_state;

    template <class T>
    void waitForPredicate(const T* pred, const std::string& msg, double hz = 2.0){
        ros::Rate pause(hz);
        ROS_INFO_STREAM(msg);
        while(ros::ok() && !(*pred)){
            ros::spinOnce();
            pause.sleep();
        }
    };

    geometry_msgs::Pose home_pose_;
    bool received_home_pose;

    std::shared_ptr<control_class> control_tech;

    ros::Time start_time, current_time;

  public:

    controller(const ros::NodeHandle& nh);
    
    virtual ~controller();

    static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d vector_acc, double yaw);
    static Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d R);

};


#endif
