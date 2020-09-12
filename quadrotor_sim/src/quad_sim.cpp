/*
This is the main file that is called to run the quadrotor simulation. It acts as an intermediary between the trajectory
generation node and the controller node. Separating the nodes allows for modularity in the design. Various controllers
and trajectory design techniques can be tested and simulated with this approach.
This structure of the code is courtesy to Jaeyoung-Lim and has been adapted from his package. More details on
this can be found on the wiki.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "quadrotor_sim/quad_sim.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCTOR
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
quadrotor_sim::quadrotor_sim(const ros::NodeHandle& nh):
  nh_(nh),
  fail_detec_(false),
  ctrl_enable_(true),
  landing_commanded_(false),
  node_state(WAITING_FOR_HOME_POSE){

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SUBSCRIBER DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Subscribers of current Quadrotor states
  //Current Mode of Operation of the Quadrotor
  mavstateSub_ = nh_.subscribe("/mavros/state", 10, &quadrotor_sim::mavstateCallback, this,ros::TransportHints().tcpNoDelay());
  //Current Position and Orientation of the Quadrotor
  mavposeSub_  = nh_.subscribe("/mavros/local_position/pose", 50, &quadrotor_sim::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  //Current Linear and Angular Velocity of the Quadorotor
  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity_local", 50, &quadrotor_sim::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
  //Readings of the IMU - Angular Velocity. For some reason the mavtwistsub_ does not provide angular velocity values
  imuSub_      = nh_.subscribe("/mavros/imu/data", 50, &quadrotor_sim::imuCallback, this,ros::TransportHints().tcpNoDelay());


//Subscribers of reference quadrotor states published by the trajectory publisher node
  //Reference Position, Velocity, Acceleration, Angular Velocity
  referenceTrajSub_   = nh_.subscribe("reference/setpoint", 100, &quadrotor_sim::trajectorytargetCallback, this, ros::TransportHints().tcpNoDelay());
  //Controller Type Selection
  //controllerTypeSub_   = nh_.subscribe("/trajectory/controller_type", 100, &quadrotor_sim::typeCallback, this, ros::TransportHints().tcpNoDelay());

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SERVICES DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Start the external controller
  ctrltriggerServ_   = nh_.advertiseService("trigger_controller", &quadrotor_sim::ctrltriggerCallback, this);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CLIENT DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //To start the trajectory generation and publishing from the trajectory generation node
  start_traj_client_  = nh_.serviceClient<std_srvs::SetBool>("start");
  //Serive to Arm and Disarm the Quadrotor
  arming_client_      = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  //To set the mode of Quadrotor
  set_mode_client_    = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  //To call the land service of the Quadrotor
  //land_service_client = nh_.advertiseService("land", &quadrotor_sim::landCallback, this);


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PUBLISHER DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Publish the angular velocities and thrust values calculated by the external controller.
  //These are used by PX4 to move the quadrotor to the desired state
  control_inputsPub_  = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 100);
  //Status of the companion process
  systemstatusPub_    = nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 10);
  //Publishing setpoint position data. PX4 can use this data for running in position control mode
  target_posePub_     = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Variable Intialisation from parameter server
  //Checks if take_off_height parameter is loaded in the parameter server. If not default value of 5.0m
  //is taken
  nh.param<double>("/quadrotor_sim/take_off_height", take_off_height, 5.0);
  //Checks if select_controller parameter is loaded in the parameter server. If not default value of 0
  //is taken
  nh_.param<int>("/quadrotor_sim/select_controller", select_controller, 0);
  //Control loop frequency mentioned in the parameter server
  nh_.param<int>("/quadrotor_sim/control_loop_freq", control_loop_freq, 100);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//TIMER DEFINITION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Calls the controller loop at a fixed frequency. By default it is 100Hz and it can be set to any value in
  //the parameter server 
  //cmdloop_timer_    = nh_.createTimer(ros::Rate(control_loop_freq), &quadrotor_sim::cmdloopCallback, this); 
  cmdloop_timer_    = nh_.createTimer(ros::Duration(0.01), &quadrotor_sim::cmdloopCallback, this);


  //Controlling the quadrotor from an external controller requires setting the PX4 mode to OFFBOARD
  //This timer checks the current system status of PX4 at a fixed frequency and changes it to OFFBOARD if required
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &quadrotor_sim::statusloopCallback, this); 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Variable Initialisation
  //Target Position to be achieved after takeoff
  targetPos_ << 0.0, 0.0, take_off_height; 
  //Target Velocity to be achieved after takeoff
  targetVel_ << 0.0, 0.0, 0.0;
  //Initialisation of current Quadrotor position and velcoity
  mavPos_    << 0.0, 0.0, 0.0;
  mavVel_    << 0.0, 0.0, 0.0;

  controller_type = 1;
//Assigning the controller based on the requirement
  //if(exec_cont == 0)
  //control_tech = std::make_shared<eth_controller>(nh_);
  //else if(exec_cont == 1)
  control_tech = std::make_shared<upenn_controller>(nh_);
  //else if(exec_cont == 2)
  //control_tech = std::make_shared<euler_controller>(nh_);
  //else if(exec_cont == 3)
  //control_tech = std::make_shared<lqr_controller>(nh_);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DESTRUCTOR
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
quadrotor_sim::~quadrotor_sim() {

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool quadrotor_sim::landCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
  node_state = LANDING;
}

//The main loop of the simulation. It is called at a fixed rate using a timer. The rate can be 
//decided in the Parameter file in the config folder. This loop controls the data
//that is to be passed on to the controller based on the operating mode of the quadrotor i.e whether it is 
//taking off, completed takeoff, landing or has landed.
void quadrotor_sim::cmdloopCallback(const ros::TimerEvent& event){
  
  switch (node_state) {
  
  //Waits for the simulated quadrotor to be active by checking if simulated quadrotor has published
  //its home pose
  case WAITING_FOR_HOME_POSE:
      waitForPredicate(&received_home_pose, "Waiting for home pose...");
      ROS_INFO("Got pose! Drone Ready to be armed.");
      node_state = TAKE_OFF;
      break;

  //Take off to the specified height
  case TAKE_OFF: 
  {
    geometry_msgs::PoseStamped takeoffmsg;

    takeoffmsg.header.stamp    = ros::Time::now();
    takeoffmsg.pose.position.x = targetPos_[0];
    takeoffmsg.pose.position.y = targetPos_[1];
    takeoffmsg.pose.position.z = targetPos_[2];

    target_posePub_.publish(takeoffmsg);

    //Checks if the takeoff height is acheived
    if((abs(mavPos_[2]-targetPos_[2])<0.01))
    {
      node_state = MISSION_EXECUTION;
      
      std_srvs::SetBool set_traj;
      set_traj.request.data = true;
      set_traj.response.success = true;

      start_traj_client_.call(set_traj);
    }
    ros::spinOnce();
    break;
  }
  //Trajectory tracking happens here. The controller mode of operation is selected here
  case MISSION_EXECUTION:
  {
    if(controller_type)
    {
      Eigen::Vector3d pos_error;
      Eigen::Vector3d vel_error;

      if(controller_type == 1){
        pos_error   = mavPos_ - targetPos_;
        vel_error   = mavVel_ - targetVel_;
      }
      else if(controller_type == 2){
        pos_error  << 0.0, 0.0, 0.0;
        vel_error   = mavVel_ - targetVel_;
      }
      else if(controller_type == 3){
        pos_error   = mavPos_ - targetPos_;
        vel_error  << 0.0, 0.0, 0.0;
      }    
      else if(controller_type == 4){
        pos_error  << 0.0, 0.0, 0.0;
        vel_error  << 0.0, 0.0, 0.0;
      }    
      
      cmdBodyRate_ = control_tech->calculate_control_fb(pos_error, vel_error, mavRate_, targetW_, targetAcc_, targetYaw_, mavAtt_, q_des);
    }
    else
    {
      node_state = LANDING;
    }
    pubRateCommands(cmdBodyRate_);

    break;
  }
  //Initiate landing phase
  case LANDING: 
  {
    geometry_msgs::PoseStamped landingmsg;
    Eigen::Vector3d error;
    landingmsg.header.stamp = ros::Time::now();
    landingmsg.pose = home_pose_;
    landingmsg.pose.position.z += take_off_height; 
    target_posePub_.publish(landingmsg);
    
    error << mavPos_[0] - landingmsg.pose.position.x, mavPos_[1] - landingmsg.pose.position.y, mavPos_[2] - landingmsg.pose.position.z;
    double dist_norm = error.norm();
    
    if(dist_norm<0.05)
    {
      node_state = LANDED;   
      statusloop_timer_.stop();
      offb_set_mode_.request.custom_mode = "AUTO.LAND";
      if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent){
      ROS_INFO("Landing Mode enabled");
      cmdloop_timer_.stop();
      }
    }
    ros::spinOnce();
    break;
  }
  }
}

//Loop to check whether PX4 is in OFFBOARD mode. If it is not effort is made to shift to OFFBOARD mode
//and arm the quadrotor. current_state_ variable has the data related to PX4 mode.
void quadrotor_sim::statusloopCallback(const ros::TimerEvent& event){

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
  
  pubSystemStatus();
}

//To enable and disable the controller using ROS Service
bool quadrotor_sim::ctrltriggerCallback(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &res){
  bool mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
}


