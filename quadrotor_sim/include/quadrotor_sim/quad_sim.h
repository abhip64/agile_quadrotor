
#ifndef QUAD_SIM_H
#define QUAD_SIM_H

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iostream>

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
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <control_msg/TargetTrajectory.h>
#include <std_srvs/SetBool.h>

#include "quadrotor_sim/controllers/upenn_controller.h"
#include "quadrotor_sim/controllers/controller_class.h"
#include "quadrotor_sim/math_operations.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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


class quadrotor_sim
{
  private:
    //NodeHandle    
    ros::NodeHandle nh_;

    //Subscriber Declaration
    ros::Subscriber referenceTrajSub_, controllerTypeSub_;
    ros::Subscriber mavstateSub_;
    ros::Subscriber mavposeSub_;
    ros::Subscriber mavtwistSub_;
    ros::Subscriber controllerType_;
    ros::Subscriber imuSub_;

    //Publisher Declaration
    ros::Publisher control_inputsPub_, target_posePub_;
    ros::Publisher systemstatusPub_;
    
    //Service Client Declaration
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient start_traj_client_;
    
    //Service Server Declaration
    ros::ServiceServer ctrltriggerServ_;
    ros::ServiceServer land_service_;

    //Timer Declaration
    ros::Timer cmdloop_timer_, statusloop_timer_;
    ros::Time last_request_, reference_request_now_, reference_request_last_;

    string mav_name_;

    bool fail_detec_, ctrl_enable_;
    bool landing_commanded_;
    bool sim_enable_;
    bool received_home_pose;

    int ctrl_mode_;
    int control_loop_freq;
    int controller_type;
    int select_controller;

    double reference_request_dt_;
    double take_off_height;
    double targetYaw_;

    geometry_msgs::Pose home_pose_;

    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;

    MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;

    Eigen::Vector4d mavAtt_, q_des;

    //Control inputs passed on to PX4 controller
    Eigen::Vector4d cmdBodyRate_; 

    //Desired trajectory values. This includes the desired position, velocity, acceleration and angular
    //velocity along the trajectory
    Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetW_;
    //The current quadrotor position, linear velocity and angular velocity
    Eigen::Vector3d mavPos_, mavVel_, mavRate_;

    //This kind of declaration makes it possible to change the controller that is used for generating
    //the control inputs. Details of changing the controller used will be given in the wiki.
    std::shared_ptr<control_class> control_tech;

    void pubRateCommands(const Eigen::Vector4d &cmd);
    void pubSystemStatus();
    void trajectorytargetCallback(const control_msg::TargetTrajectory& msg);
    void cmdloopCallback(const ros::TimerEvent& event);
    void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);
    void mavposeCallback(const geometry_msgs::PoseStamped& msg);
    void mavtwistCallback(const geometry_msgs::TwistStamped& msg);
    void statusloopCallback(const ros::TimerEvent& event);
    void typeCallback(const std_msgs::Int8&);
    void imuCallback(const sensor_msgs::Imu&);

    bool ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool landCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

    geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);
    
    Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);

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

  public:

    quadrotor_sim(const ros::NodeHandle& nh);
    
    virtual ~quadrotor_sim();
};


#endif
