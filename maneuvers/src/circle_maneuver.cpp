/*
EXECUTE THE CIRCULAR MANEUVER
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "maneuvers/circle_maneuver.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
circle_traverse::circle_traverse(const ros::NodeHandle& nh): nh_(nh)
{

  //Obtaining the radius of the circular trajectory to be traversed from the parameter server. The
  //default radius is 1.0m
  nh_.param<double>("/trajectory_generator/circle_radius", circle_radius, 1.0);

  //Obtaining the angular velocity of the circular trajectory to be traversed from the parameter 
  //server. The default angular velocity is 0.1 rad/s
  nh_.param<double>("/trajectory_generator/circle_angvel", circle_ang_velocity, 0.1);

  //Obtaining the height of the circular trajectory to be traversed from the parameter 
  //server. The default height is 5.0m
  nh_.param<double>("/trajectory_generator/circle_height", circle_height, 5.0);

  //Number of revolutions of the given circle to be executed. Default is 5
  nh_.param<int>("/trajectory_generator/circle_rev", circle_rev, 5.0);

}

circle_traverse::~circle_traverse() {
  //Destructor
}


double circle_traverse::maneuver_init(double time)
{
  //The total execution time of the trajectory = Number of revolutions * Time Period of a single revolution
  return circle_rev*(2*M_PI/circle_ang_velocity);
}

void circle_traverse::trajectory_generator(double time)
{
  //Circular trajectory generation is quite straightforward and is done using the parametric equation of the circle.
  target_position     << circle_radius*sin(circle_ang_velocity*time), circle_radius*cos(circle_ang_velocity*time), circle_height;
  
  //The velocity, acceleration and jerk along the trajectory can be appropriately obtained by differentiating the 
  //parametric equation for position of the circle. 
  target_velocity     << cos(circle_ang_velocity*time), -sin(circle_ang_velocity*time), 0;
  target_velocity     *= circle_radius*circle_ang_velocity;

  target_acceleration << sin(circle_ang_velocity*time), cos(circle_ang_velocity*time), 0;
  target_acceleration *= -circle_radius*circle_ang_velocity*circle_ang_velocity;

  target_jerk          = -target_velocity*circle_ang_velocity*circle_ang_velocity;

  //The method of calculation of the angular velocity is described in detail in the github wiki. 
  target_angvel        = calculate_trajectory_angvel();

  //Yaw is calculated to ensure that the quadrotor always aligns itself to the direction of motion. The front of
  //the quadrotor is 90 degree offset from the 0 degree yaw direction of the controller. Thus an additional
  //offset term is required.  
  target_yaw           = atan2(target_position(1),target_position(0)) - M_PI/2.0;

  //This parameter decides the mode of operation of the controller. The controller can work in 5 modes
  //MODE 0 - IGNORE ALL TRAJECTORY TARGET INPUTS (POSITION, VELOCITY, ACCELERATION, ANGULAR VELOCITY, YAW)
  //MODE 1 - ACCEPT ALL TRAJECTORY TARGET INPUTS FOR CONTROL OUTPUT CALCULATION
  //MODE 2 - IGNORE ONLY POSITION FOR CONTROL OUTPUT CALCULATION (ERROR IS TAKEN AS 0)
  //MODE 3 - IGNORE ONLY VELOCITY FOR CONTROL OUTPUT CALCULATION (ERROR IS TAKEN AS 0)
  //MODE 4 - IGNORE POSITION AND VELOCITY FOR CONTROL OUTPUT CALCULATION (ERROR IS TAKEN AS 0)
  type = 1;

}

Eigen::Vector3d circle_traverse::calculate_trajectory_angvel()
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

Eigen::Vector3d circle_traverse::get_target_pos()
{
  //Return the desired position
  return target_position;
}

Eigen::Vector3d circle_traverse::get_target_vel()
{
  //Return the desired velocity
  return target_velocity;
}

Eigen::Vector3d circle_traverse::get_target_acc()
{
  //Return the desired acceleration
  return target_acceleration;
}

Eigen::Vector3d circle_traverse::get_target_angvel()
{
  //Return the desired anguler velocity
  return target_angvel;
}

Eigen::Vector3d circle_traverse::get_target_jerk()
{
  //Return the desired jerk
  return target_jerk;
}

double circle_traverse::get_target_yaw()
{
  //Return the desired yaw
  return target_yaw;
}

int circle_traverse::get_type()
{
  //Return the desired mode of controller operation
  return type;
}