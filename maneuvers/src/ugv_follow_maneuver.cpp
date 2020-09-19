/*
EXECUTE THE SLIT TRAVERSAL MANEUVER
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "maneuvers/ugv_follow_maneuver.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ugv_follow::ugv_follow(const ros::NodeHandle& nh): nh_(nh)
{
  //Current position of the quadrotor
  mavPos_          << 0.0, 0.0, 0.0;
  //Current velocity of the quadrotor
  mavVel_          << 0.0, 0.0, 0.0;

  //Current position of the UGV obtained from the Kalman Filter
  ugv_position     << 0.0, 0.0, 0.0;
  //Current velocity of the UGV obtained from the Kalman Filter
  ugv_velocity     << 0.0, 0.0, 0.0;
  //Current acceleration of the UGV obtained from the Kalman Filter
  ugv_acceleration << 0.0, 0.0, 0.0;
  //Current orientation of the UGV obtained from the Kalman Filter
  ugv_attitude     << 1.0, 0.0, 0.0, 0.0;

  //Variable to store the position of the UGV at some time in future
  ugvpos_future    << 0.0, 0.0, 0.0;
  //Variable to store the velocity of the UGV at some time on future
  ugvvel_future    << 0.0, 0.0, 0.0;
  //Yaw orientation fof the UGV
  ugv_yaw = 0.0;

  elasped_time = 0.0;

  //Tracking and following of the UGV is carried out in two phases. The manuever starts with the quadrotor trying
  //to catch up with the UGV. This becomes phase 1. In the next phase the quadrotor tries to closely follow the
  //UGV at a given height by making use of onboard camera to detect the UGV
  phase = 1;

  //Acceleration due to gravity in NED frame
  g_               << 0.0, 0.0, 9.81;

  //The mean velocity at which the quadrotor is assumed to follow the ugv. This parameter is used for determining
  //the point and time of intersection. Altering the parameter in the parameter file can speed up or slow down
  //the quadrotor approach maneuver.
  nh_.param<double>("/trajectory_generator/ugv_follow_velocity", ugv_follow_velocity, 5.0);

  //Height above the UGV at which the quadrotor tracks the UGV
  nh_.param<double>("/trajectory_generator/hover_height", hover_height, 1.0);

  //Subscriber to obtain the current position, velocity, acceleration and attitude of the UGV from a Kalman Filter
  vehicle_posSub_   = nh_.subscribe("/odometry/filtered", 50, &ugv_follow::ugv_state_sub, this,ros::TransportHints().tcpNoDelay());
  
  //Subscriber to obtain the current acceleration of the UGV from the Kalman Filter
  accelSub_         = nh_.subscribe("/accel/filtered", 30, &ugv_follow::ugv_accel_sub, this,ros::TransportHints().tcpNoDelay());

  //Subscriber to obtain the current position and attitude of the quadrotor
  mavPoseSub_       = nh_.subscribe("/mavros/local_position/pose", 50, &ugv_follow::mav_pose_sub, this,ros::TransportHints().tcpNoDelay());

  //Subscriber to obtain the current velocity of the quadrotor
  mavtwistSub_      = nh_.subscribe("/mavros/local_position/velocity_local", 50, &ugv_follow::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());

}

ugv_follow::~ugv_follow() {
  //Destructor
}


double ugv_follow::maneuver_init(double trigger_time)
{
  //Distance between the UGV and quadrotor along the XY plane
  dist_to_ugv = planar_distance_calculator(mavPos_,ugv_position);

  //If the above calculated distance is less than 0.5m then phase 2 of UGV tracking is initiated
  if(dist_to_ugv < 0.5)
  {
    phase = 2;
    time_to_intersection = 0.5;
  }
  else
  {
    phase = 1;
    time_to_intersection = dist_to_ugv/ugv_follow_velocity + 1.0 - trigger_time;

    //If the quadrotor was unable to reach the UGV in the calculated time to intersection then 
    //time to intersection parameter is re-initialised
    if(time_to_intersection < 0.0)
      time_to_intersection = dist_to_ugv/ugv_follow_velocity + 1.0;
  }
  
  //Intersection trajectory recalculation happens only when there is significant variation in the 
  //predicted intersection point over successive iterations
  if(future_state_predict() && phase == 1)
  {
  elasped_time = trigger_time;

  //Setting the desired z axis position of the quadrotor to be the hover height
  ugvpos_future[2] = hover_height;

  //Trajectory generation is carried out using the mav_trajectory_generation package
  mav_trajectory_generation::Vertex::Vector vertices;
  //Defintion of two waypoints. The first waypoint is the current position of the quadrotor and the next
  //waypoint is the future position of the UGV at the assumed point of intersection between the UGV
  //and the quadrotor
  mav_trajectory_generation::Vertex start(dimension),  end(dimension);

  //Number of dimensions that need to be considered for each waypoint e.g. Dimension of 3 requires the definition
  //of the waypoint derivatives along X, Y and Z.
  const int dimension = 3;

  //Generated trajectory has to be a velocity minimal trajectory for quick approach
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::VELOCITY;

  //The starting point of the trajectory. This is the current position of the quadrotor
  start.makeStartOrEnd(mavPos_, derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, mavVel_);
  vertices.push_back(start);

  //The ending point of the trajectory is the future position and velocity of the UGV at the
  //calculated point of intersection between the UGV and the quadrotor
  end.makeStartOrEnd(ugvpos_future, derivative_to_optimize); 
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, ugvvel_future);
  vertices.push_back(end);

  //Calculation of the velocity continous optimal trajectory that passes through the given vertices.
  //Using the generated trajectory object, the desired position, velocity, accelration and jerk along
  //the trajectory can be convienently calculated. Desired angular velocity can also be determined
  //taking into consideration the differenetial flatnees nature of the quadrotor. Yaw planning is not
  //coupled with trajectory design

  //Time to intersection is assigned as the segment time
  std::vector<double> segment_times;
  segment_times.push_back(time_to_intersection);
  //Total time for traversing the trajectory

  eth_trajectory_init(vertices, segment_times, derivative_to_optimize);
 
  }
  //Return a large value for return time to ensure tracking of the UGV by the quadrotor is not stopped
  //abruptly
  return 1000.0;
}

void ugv_follow::trajectory_generator(double time)
{
  //Recalculate the interception trajectory with the UGV
  maneuver_init(time);
  
  //Resetting the time to 0 after a new trajectory has been calculated from the above function call. 
  //When a new trajectory is calculated by the mav_trajectory_generation package the position, velocity
  //and acceleration along that trajectory are calculated from time 0. So the trigger time obtained 
  //from trajectoryPublisher.cpp has to be re intialised to 0 after every new interception trajectory
  //determination
  time -= elasped_time;

  //Phase 1 corresponds to high speed approach maneuver towards the UGV
  if(phase == 1)
  {
  //Target position obtained from the generated trajectory object using the mav_trajectory_generation package
  target_position     = eth_trajectory_pos(time);
  
  //Target velocity obtained from the generated trajectory object using the mav_trajectory_generation package
  target_velocity     = eth_trajectory_vel(time);

  //Target acceleration obtained from the generated trajectory object using the mav_trajectory_generation package
  target_acceleration = eth_trajectory_acc(time);

  //Target jerk obtained from the generated trajectory object using the mav_trajectory_generation package
  target_jerk         = eth_trajectory_jerk(time);

  //Target jerk obtained from the generated trajectory object using the mav_trajectory_generation package
  target_angvel       = calculate_trajectory_angvel();
  }
  //Phase 2 corresponds to close tracking of the UGV at a given hover height
  else if(phase == 2)
  {
  //Target position obtained from the generated trajectory object using the mav_trajectory_generation package
  target_position     = ugvpos_future;
  target_position[2]  = hover_height;
 
  //Target velocity obtained from the generated trajectory object using the mav_trajectory_generation package
  target_velocity     = ugvvel_future;
  
  //Target acceleration obtained from the generated trajectory object using the mav_trajectory_generation package
  target_acceleration = ugv_acceleration;

  //Target jerk obtained from the generated trajectory object using the mav_trajectory_generation package
  target_jerk        << 0.0, 0.0, 0.0;

  //Target jerk obtained from the generated trajectory object using the mav_trajectory_generation package
  target_angvel      << 0.0, 0.0, 0.0;

  }
  //Target Yaw is taken to be 0.0
  target_yaw          = 0.0;

  //This parameter decides the mode of operation of the controller. The controller can work in 5 modes
  //MODE 0 - IGNORE ALL TRAJECTORY TARGET INPUTS (POSITION, VELOCITY, ACCELERATION, ANGULAR VELOCITY, YAW)
  //MODE 1 - ACCEPT ALL TRAJECTORY TARGET INPUTS FOR CONTROL OUTPUT CALCULATION
  //MODE 2 - IGNORE ONLY POSITION FOR CONTROL OUTPUT CALCULATION (ERROR IS TAKEN AS 0)
  //MODE 3 - IGNORE ONLY VELOCITY FOR CONTROL OUTPUT CALCULATION (ERROR IS TAKEN AS 0)
  //MODE 4 - IGNORE POSITION AND VELOCITY FOR CONTROL OUTPUT CALCULATION (ERROR IS TAKEN AS 0)
  type = 1;

}

Eigen::Vector3d ugv_follow::calculate_trajectory_angvel()
{
  Eigen::Vector3d acc, jerk, h, zb, w, xc, yb, xb;

  float m = 0.95;

  Eigen::Vector3d g;

  g << 0, 0, 9.8;
  
  acc  = target_acceleration + g;
  jerk = target_jerk;
  
  double u = acc.norm();

  zb = acc/u;

  xc << cos(target_yaw),sin(target_yaw),0;

  yb = zb.cross(xc) / (zb.cross(xc)).norm();
  xb = yb.cross(zb) / (yb.cross(zb)).norm();

  h = (m/u)*(jerk - (zb.dot(jerk))*zb);

  w(0) = -h.dot(yb);

  w(1) = h.dot(xb);

  w(2) = 0;

  return w;
}

Eigen::Vector3d ugv_follow::get_target_pos()
{
  //Return the desired position
  return target_position;
}

Eigen::Vector3d ugv_follow::get_target_vel()
{
  //Return the desired velocity
  return target_velocity;
}

Eigen::Vector3d ugv_follow::get_target_acc()
{
  //Return the desired acceleration  
  return target_acceleration;
}

Eigen::Vector3d ugv_follow::get_target_angvel()
{
  //Return the desired angular velocity
  return target_angvel;
}

Eigen::Vector3d ugv_follow::get_target_jerk()
{
  //Return the desired jerk
  return target_jerk;
}

double ugv_follow::get_target_yaw()
{
  //Return the desired yaw
  return target_yaw;
}

int ugv_follow::get_type()
{
  //Return the desired mode of operation of the controller
  return type;
}

//Function to subscribe to the current acceleration of the UGV
void ugv_follow::ugv_accel_sub(const geometry_msgs::AccelWithCovarianceStamped& msg){  
  ugv_acceleration << msg.accel.accel.linear.x, msg.accel.accel.linear.y, msg.accel.accel.linear.z;
}

//Function to subscribe to the current position, velocity and attitude of the UGV
void ugv_follow::ugv_state_sub(const nav_msgs::Odometry& msg)
{
  //Current UGV position
  ugv_position << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
  //Current UGV velocity
  ugv_velocity << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
  //Current UGV attitude 
  ugv_attitude << msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z;
  //Current UGV Yaw calculated from the attitude
  ugv_yaw       = atan2(2 * (ugv_attitude(0) * ugv_attitude(3) + ugv_attitude(1) * ugv_attitude(2)), 1 - 2 * (ugv_attitude(2) * ugv_attitude(2) + ugv_attitude(3) * ugv_attitude(3)));

  //Velocity transformation is required for aligning the direction of velocity published from the 
  //Kalman filter with the axes direction used in the program
  Eigen::Matrix3d R;

  R            <<  cos(ugv_yaw), -sin(ugv_yaw), 0,
                   sin(ugv_yaw),  cos(ugv_yaw), 0,
                   0           ,  0           , 1;

  ugv_velocity  = R*ugv_velocity;
}

//Function to subscribe to the current position of the quadrotor
void ugv_follow::mav_pose_sub(const geometry_msgs::PoseStamped& msg){
  mavPos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

//Function to subscribe to the current velocity of the quadrotor
void ugv_follow::mavtwistCallback(const geometry_msgs::TwistStamped& msg){  
  mavVel_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
}

//Function to determine the future position and velocity of the UGV based on its current position,
//velocity and acceleration. The predicted data is then used for predicting interception trajectories
//with the quadrotor. The future states are predicted using a linear motion model
bool ugv_follow::future_state_predict()
{

  Eigen::Vector3d dummy_pos;

  double dummy_error;

  ugvvel_future = ugv_velocity + ugv_acceleration*time_to_intersection;

  dummy_pos     = ugv_position + ugvvel_future*time_to_intersection;

  //The error between the previously predicted future UGV position and the future UGV position
  //predicted now. The approach trajectory is recalculted only if the variation between the 
  //two is significant
  dummy_error   = planar_distance_calculator(dummy_pos, ugvpos_future);

  ugvpos_future = dummy_pos;

  if(dummy_error > 0.2)
    return 1;
  else
    return 0;
 
}

//Function to calculate the distance along the XY plane for two given vectors
double ugv_follow::planar_distance_calculator(Eigen::Vector3d& v1, Eigen::Vector3d& v2)
{
  return pow(((v1[0]-v2[0])*(v1[0]-v2[0]) + (v1[1]-v2[1])*(v1[1]-v2[1])),0.5);
}